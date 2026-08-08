"""本地完整 SITL 编排与远端机载服务连接工作流。"""

from __future__ import annotations

import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from .config import (
    INTERFACE_VERSION,
    ONBOARD_PARAM_FILE,
    ardupilot_root,
    find_sim_vehicle,
    mavros_apm_config,
    ros_setup_files,
)
from .event_log import EventLog, LogLevel
from .models import VehicleSnapshot
from .process_manager import CleanupReport, ManagedProcess, ProcessSupervisor
from .ros_controller import GroundStationRosController


StatusCallback = Callable[[LogLevel, str], None]
DoneCallback = Callable[[bool, str], None]

# 机载 ControlStatus 默认 10 Hz；诊断要求至少 5 Hz 且最长断流不超过 0.5 秒。
_COMMUNICATION_OBSERVATION_SECONDS = 3.0
_COMMUNICATION_MIN_RATE_HZ = 5.0
_COMMUNICATION_MAX_GAP_SECONDS = 0.5


@dataclass(frozen=True)
class _CommunicationMetrics:
    """一次纯订阅通讯检测的状态样本、速率、最大间隔与最终快照。"""

    snapshot: VehicleSnapshot
    samples: int
    rate_hz: float
    max_gap_seconds: float


class _WorkflowCancelled(RuntimeError):
    """内部异常：用户请求断开/清理时中断当前初始化。"""


class _PreflightFailed(RuntimeError):
    """内部异常：依赖检查失败且尚未改变外部环境。"""


class EnvironmentInitializer:
    """管理本地 SITL、完整实机连接及零命令实机通讯检测。"""

    def __init__(
        self,
        ros_controller: GroundStationRosController,
        supervisor: ProcessSupervisor | None = None,
        event_log: EventLog | None = None,
    ) -> None:
        """绑定地面站客户端及仅用于本地仿真的进程管理器。"""
        self._ros = ros_controller
        self._events = event_log or ros_controller.event_log
        self._supervisor = supervisor or ProcessSupervisor(self._events)
        self._state_lock = threading.RLock()
        self._cleanup_lock = threading.Lock()
        self._workflow_thread: threading.Thread | None = None
        self._workflow_name: str | None = None
        self._cancel_event = threading.Event()

    @property
    def supervisor(self) -> ProcessSupervisor:
        """暴露本地仿真进程管理器以便显示日志与测试。"""
        return self._supervisor

    @property
    def busy(self) -> bool:
        """指示连接/初始化线程是否仍在执行。"""
        with self._state_lock:
            thread = self._workflow_thread
            return thread is not None and thread.is_alive()

    def initialize_simulation(
        self, status: StatusCallback, done: DoneCallback
    ) -> bool:
        """异步启动 SITL、MAVROS、同款机载 C++ 服务与 RViz。

        仿真不调用 set_gp_origin：SITL 使用自身 Home（默认 CMAC）建立 EKF
        原点，本地位姿应在原点附近。强制写入与 SITL Home 不一致的经纬高
        会导致 local ENU 偏移达数百万米，GUI/RViz 位姿发散。
        """
        return self._start_workflow(
            "simulation",
            lambda: self._simulation_workflow(status),
            status,
            done,
        )

    def initialize_hardware(
        self,
        origin: tuple[float, float, float],
        status: StatusCallback,
        done: DoneCallback,
    ) -> bool:
        """异步完整连接局域网机载服务，不远程管理无人机进程。"""
        return self._start_workflow(
            "hardware",
            lambda: self._hardware_workflow(origin, status),
            status,
            done,
        )

    def test_hardware_communication(
        self,
        status: StatusCallback,
        done: DoneCallback,
    ) -> bool:
        """异步检测实机状态/日志链路，不申请租约、发命令或管理进程。"""
        return self._start_workflow(
            "communication",
            lambda: self._communication_workflow(status),
            status,
            done,
            cleanup_on_failure=False,
            operation_label="通讯检测",
        )

    def cancel_hardware_communication_test(self) -> bool:
        """只请求结束纯订阅检测，不释放租约或触碰任何受管进程。"""
        with self._state_lock:
            thread = self._workflow_thread
            if (
                self._workflow_name != "communication"
                or thread is None
                or not thread.is_alive()
            ):
                return False
            self._cancel_event.set()
        self._events.warn("environment", "操作者请求终止实机通讯检测")
        return True

    def _start_workflow(
        self,
        name: str,
        workflow: Callable[[], str],
        status: StatusCallback,
        done: DoneCallback,
        *,
        cleanup_on_failure: bool = True,
        operation_label: str = "初始化/连接",
    ) -> bool:
        """保证工作流互斥，并让纯诊断失败时跳过任何环境清理动作。"""
        with self._state_lock:
            if self._workflow_thread is not None and self._workflow_thread.is_alive():
                self._events.warn("environment", "已有初始化/连接流程正在执行")
                done(False, "已有初始化/连接流程正在执行")
                return False
            self._cancel_event = threading.Event()
            self._workflow_name = name

            def runner() -> None:
                try:
                    message = workflow()
                except _PreflightFailed as exc:
                    message = f"{operation_label}前检查失败: {exc}"
                    self._events.error("environment", message)
                    done(False, message)
                except _WorkflowCancelled:
                    if cleanup_on_failure:
                        report = self._terminate_local_processes()
                        message = (
                            "操作已取消，本地仿真进程与控制租约已清理"
                            if report.success
                            else "操作已取消，但本地清理仍有残留: "
                            f"{report.remaining}"
                        )
                        level = LogLevel.WARN if report.success else LogLevel.ERROR
                    else:
                        self._ros.disable_remote_logs()
                        message = (
                            f"{operation_label}已取消；未申请控制权、未发送命令，"
                            "未管理任何进程"
                        )
                        level = LogLevel.WARN
                    self._events.emit(level, "environment", message)
                    done(False, message)
                except Exception as exc:
                    if cleanup_on_failure:
                        self._publish_status(
                            status,
                            LogLevel.WARN,
                            f"{operation_label}失败，正在清理本地环境: {exc}",
                        )
                        report = self._terminate_local_processes()
                        message = (
                            f"{operation_label}失败: {exc}"
                            f"{self._cleanup_suffix(report)}"
                        )
                    else:
                        self._ros.disable_remote_logs()
                        message = (
                            f"{operation_label}失败: {exc}；未申请控制权、"
                            "未发送命令，未启动/停止任何进程"
                        )
                    self._events.error("environment", message)
                    done(False, message)
                else:
                    self._events.info("environment", message)
                    done(True, message)
                finally:
                    with self._state_lock:
                        if self._workflow_thread is threading.current_thread():
                            self._workflow_name = None

            self._workflow_thread = threading.Thread(
                target=runner,
                name=f"ground-station-{name}-init",
                daemon=True,
            )
            self._workflow_thread.start()
            return True

    def cleanup(self) -> CleanupReport:
        """释放控制租约并只结束本机仿真进程，不触碰远端机载服务。"""
        self._events.info("environment", "请求断开并清理本地仿真环境")
        with self._state_lock:
            self._cancel_event.set()
            workflow_thread = self._workflow_thread
        if (
            workflow_thread is not None
            and workflow_thread.is_alive()
            and workflow_thread is not threading.current_thread()
        ):
            workflow_thread.join(timeout=6.0)
        return self._terminate_local_processes()

    def _simulation_workflow(self, status: StatusCallback) -> str:
        """执行可取消的完整闭环仿真初始化（不写 GPS 原点，沿用 SITL Home）。"""
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 客户端未就绪: {self._ros.error or '未知原因'}"
                )
            sim_vehicle = find_sim_vehicle()
            if sim_vehicle is None:
                raise FileNotFoundError("未找到 sim_vehicle.py，请配置 ArduPilot PATH")
            apm_config = mavros_apm_config()
            if not apm_config.is_file():
                raise FileNotFoundError(f"MAVROS 参数文件不存在: {apm_config}")
            if not ONBOARD_PARAM_FILE.is_file():
                raise FileNotFoundError(f"机载控制参数不存在: {ONBOARD_PARAM_FILE}")
            for package in (
                "mavros",
                "guided_interfaces",
                "onboard_control",
                "guided_sim",
            ):
                self._verify_ros_package(package, ros_setup_files())
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        self._publish_status(
            status, LogLevel.INFO, "正在释放旧租约并清理本地仿真环境..."
        )
        cleanup = self._terminate_local_processes()
        if cleanup.remaining:
            raise RuntimeError(f"旧进程清理不完整: {cleanup.remaining}")
        self._ros.enable_control()
        self._check_cancelled()

        self._publish_status(status, LogLevel.INFO, "1/5 正在启动 ArduPilot SITL...")
        sitl = self._supervisor.start(
            "sitl",
            [
                str(sim_vehicle),
                "-v",
                "ArduCopter",
                "--param",
                "GUID_OPTIONS=8",
            ],
            cwd=ardupilot_root(sim_vehicle),
            keep_stdin_open=True,
        )
        if not self._wait_tcp("127.0.0.1", 5762, 35.0, sitl):
            raise RuntimeError(f"SITL 未开放 TCP 5762；日志: {sitl.log_path}")

        self._publish_status(status, LogLevel.INFO, "2/5 正在启动仿真 MAVROS...")
        mavros = self._supervisor.start(
            "mavros_sim",
            [
                "ros2",
                "run",
                "mavros",
                "mavros_node",
                "--ros-args",
                "-p",
                "fcu_url:=tcp://127.0.0.1:5762",
                "--params-file",
                str(apm_config),
            ],
            setup_files=ros_setup_files(),
        )
        self._wait_process_stable(mavros, 1.0)

        self._publish_status(
            status, LogLevel.INFO, "3/5 正在启动独立机载 C++ 控制服务..."
        )
        onboard = self._supervisor.start(
            "onboard_control",
            [
                "ros2",
                "run",
                "onboard_control",
                "onboard_control_node",
                "--ros-args",
                "--params-file",
                str(ONBOARD_PARAM_FILE),
            ],
            setup_files=ros_setup_files(),
        )
        self._wait_onboard(20.0, onboard)
        self._wait_connected(35.0, onboard)
        self._wait_thrust_mode(35.0, onboard)
        self._wait_control_authority(12.0, onboard)

        self._publish_status(
            status, LogLevel.INFO, "4/5 正在由机载服务配置消息频率并等待 EKF..."
        )
        rate_result = self._wait_ticket(self._ros.request_set_rates(), 25.0)
        if rate_result is None:
            raise RuntimeError("消息频率配置等待超时")
        if not rate_result.success:
            raise RuntimeError(rate_result.message)
        # 故意不调用 set_gp_origin：SITL Home 与 GUI 缓存经纬高通常不一致，
        # 写入错误原点会使 /mavros/local_position 偏移数百万米。
        self._wait_local_position(45.0, onboard)

        self._publish_status(status, LogLevel.INFO, "5/5 正在启动 RViz...")
        rviz = self._supervisor.start(
            "rviz",
            ["ros2", "launch", "guided_sim", "visualize.launch.py"],
            setup_files=ros_setup_files(),
        )
        self._wait_process_stable(rviz, 2.0)
        return (
            "仿真闭环初始化完成：SITL/MAVROS/机载 C++ 控制/RViz；"
            f"日志目录: {self._supervisor.log_directory}"
        )

    def _hardware_workflow(
        self, origin: tuple[float, float, float], status: StatusCallback
    ) -> str:
        """完整连接远端机载服务，校验租约/飞控并执行连接维护命令。"""
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 客户端未就绪: {self._ros.error or '未知原因'}"
                )
            self._verify_ros_package("guided_interfaces", ros_setup_files())
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        self._publish_status(
            status,
            LogLevel.INFO,
            "正在停止本地仿真；远端无人机进程不会被地面站终止...",
        )
        cleanup = self._terminate_local_processes()
        if cleanup.remaining:
            raise RuntimeError(f"本地仿真清理不完整: {cleanup.remaining}")
        self._ros.enable_remote_logs()
        self._ros.enable_control()
        self._check_cancelled()

        self._publish_status(
            status, LogLevel.INFO, "1/4 正在等待局域网机载控制服务..."
        )
        self._wait_onboard(30.0)
        snapshot = self._ros.snapshot()
        if snapshot.interface_version != INTERFACE_VERSION:
            raise RuntimeError(
                f"接口版本不兼容：地面站 {INTERFACE_VERSION} / "
                f"机载端 {snapshot.interface_version or '--'}"
            )

        self._publish_status(
            status, LogLevel.INFO, "2/4 正在申请单一控制权并等待飞控连接..."
        )
        self._wait_control_authority(15.0)
        self._wait_connected(35.0)
        self._wait_thrust_mode(35.0)

        self._publish_status(
            status,
            LogLevel.INFO,
            "3/4 正在由机载 MAVROS 配置消息频率并写入飞控原点...",
        )
        rate_result = self._wait_ticket(self._ros.request_set_rates(), 25.0)
        if rate_result is None or not rate_result.success:
            raise RuntimeError(
                rate_result.message if rate_result is not None else "消息频率配置超时"
            )
        origin_result = self._wait_ticket(
            self._ros.request_set_gp_origin(*origin), 10.0
        )
        if origin_result is None or not origin_result.success:
            raise RuntimeError(
                origin_result.message
                if origin_result is not None
                else "飞控原点设置超时"
            )

        self._publish_status(
            status, LogLevel.INFO, "4/4 正在等待远端本地位置就绪..."
        )
        self._wait_local_position(40.0)
        return (
            "实机机载服务连接完成；地面站已持有高层命令租约，"
            "MAVROS/驱动/控制循环均在无人机上运行"
        )

    def _communication_workflow(self, status: StatusCallback) -> str:
        """仅订阅并测量实机状态/日志链路，禁止任何控制或进程生命周期调用。"""
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 客户端未就绪: {self._ros.error or '未知原因'}"
                )
            snapshot = self._ros.snapshot()
            if self._ros.control_enabled or snapshot.control_authority:
                raise RuntimeError("当前客户端已有控制会话，请先正常断开后再检测")
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        existing_events = self._events.snapshot()
        starting_log_sequence = existing_events[-1].sequence if existing_events else 0
        self._ros.enable_remote_logs()
        try:
            self._publish_status(
                status,
                LogLevel.INFO,
                "1/2 正在被动等待机载 ControlStatus；不申请租约或发送命令...",
            )
            self._wait_onboard(15.0)
            snapshot = self._ros.snapshot()
            if snapshot.interface_version != INTERFACE_VERSION:
                raise RuntimeError(
                    f"接口版本不兼容：地面站 {INTERFACE_VERSION} / "
                    f"机载端 {snapshot.interface_version or '--'}"
                )

            self._publish_status(
                status,
                LogLevel.INFO,
                "2/2 正在测量 3 秒状态流并接收远端日志；全程零控制命令...",
            )
            metrics = self._observe_communication_link(
                _COMMUNICATION_OBSERVATION_SECONDS
            )
            remote_log_count = sum(
                event.source.startswith("remote-rosout:")
                for event in self._events.events_after(starting_log_sequence)
            )
            armed_state = "已武装（仅观测）" if metrics.snapshot.armed else "未武装"
            fcu_state = "已连接" if metrics.snapshot.connected else "未连接"
            lease_state = metrics.snapshot.lease_owner or "无"
            return (
                "实机通讯链路检测通过："
                f"ControlStatus {metrics.samples} 条 / {metrics.rate_hz:.2f} Hz，"
                f"最大接收间隔 {metrics.max_gap_seconds * 1000.0:.1f} ms；"
                f"飞控{fcu_state}、{armed_state}，租约持有者 {lease_state}，"
                f"检测窗收到远端日志 {remote_log_count} 条；"
                "未申请控制权、未发送心跳/维护/飞行指令，未管理任何进程"
            )
        finally:
            self._ros.disable_remote_logs()

    def _observe_communication_link(self, duration: float) -> _CommunicationMetrics:
        """在有界窗口内只读统计新状态样本，持续验证未进入本客户端控制态。"""
        start_count, _ = self._ros.status_observation()
        started_at = time.monotonic()
        deadline = started_at + duration
        latest = self._ros.snapshot()
        while time.monotonic() < deadline:
            self._check_cancelled()
            latest = self._ros.snapshot()
            if not latest.onboard_available:
                raise RuntimeError("检测期间机载 ControlStatus 中断")
            if self._ros.control_enabled or latest.control_authority:
                raise RuntimeError("通讯检测期间客户端意外进入控制态，已立即中止")
            self._cancel_event.wait(0.05)

        ended_at = time.monotonic()
        end_count, receive_times = self._ros.status_observation()
        samples = max(0, end_count - start_count)
        elapsed = max(ended_at - started_at, 1e-6)
        window_times = [
            stamp for stamp in receive_times if started_at <= stamp <= ended_at
        ]
        gap_points = [started_at, *window_times, ended_at]
        max_gap = max(
            later - earlier for earlier, later in zip(gap_points, gap_points[1:])
        )
        rate_hz = samples / elapsed
        minimum_samples = max(3, int(elapsed * _COMMUNICATION_MIN_RATE_HZ + 0.999))
        if samples < minimum_samples:
            raise RuntimeError(
                f"状态接收频率过低：{samples} 条/{elapsed:.2f}s "
                f"({rate_hz:.2f} Hz)，要求至少 {_COMMUNICATION_MIN_RATE_HZ:.1f} Hz"
            )
        if max_gap > _COMMUNICATION_MAX_GAP_SECONDS:
            raise RuntimeError(
                f"状态接收出现 {max_gap * 1000.0:.1f} ms 断流，"
                f"上限 {_COMMUNICATION_MAX_GAP_SECONDS * 1000.0:.0f} ms"
            )
        return _CommunicationMetrics(latest, samples, rate_hz, max_gap)

    def _verify_ros_package(self, package: str, setup_files: tuple[Path, ...]) -> None:
        """在指定 overlay 中验证所需 ROS 包。"""
        result = self._supervisor.run_checked(
            ["ros2", "pkg", "prefix", package], setup_files=setup_files
        )
        if result.returncode != 0:
            detail = result.stdout.strip() or "package not found"
            raise RuntimeError(f"ROS 包 {package} 不可用: {detail}")

    def _wait_onboard(
        self, timeout: float, process: ManagedProcess | None = None
    ) -> None:
        """等待新鲜且版本明确的机载聚合状态。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            if self._ros.snapshot().onboard_available:
                return
            self._cancel_event.wait(0.1)
        suffix = f"；日志: {process.log_path}" if process is not None else ""
        raise RuntimeError(f"等待机载控制服务超时{suffix}")

    def _wait_connected(
        self, timeout: float, process: ManagedProcess | None = None
    ) -> None:
        """等待机载状态确认飞控连接。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            if self._ros.snapshot().connected:
                return
            self._cancel_event.wait(0.1)
        raise RuntimeError("等待机载飞控连接超时")

    def _wait_control_authority(
        self, timeout: float, process: ManagedProcess | None = None
    ) -> None:
        """等待本客户端获得机载端唯一控制租约。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            if self._ros.snapshot().control_authority:
                return
            self._cancel_event.wait(0.1)
        snapshot = self._ros.snapshot()
        detail = self._ros.lease_error or f"当前持有者: {snapshot.lease_owner or '无'}"
        raise RuntimeError(f"无法获得机载控制权: {detail}")

    def _wait_local_position(
        self, timeout: float, process: ManagedProcess | None = None
    ) -> None:
        """等待机载状态确认本地位姿新鲜。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            if self._ros.snapshot().local_position_valid:
                return
            self._cancel_event.wait(0.1)
        raise RuntimeError("等待机载本地位置超时")

    def _wait_thrust_mode(
        self, timeout: float, process: ManagedProcess | None = None
    ) -> None:
        """确认 ArduPilot 将 SET_ATTITUDE_TARGET.thrust 解释为真实推力。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            if self._ros.snapshot().thrust_mode_verified:
                return
            self._cancel_event.wait(0.1)
        raise RuntimeError(
            "未确认 ArduPilot GUID_OPTIONS bit 3；请在飞控设置 GUID_OPTIONS=8"
        )

    def _wait_ticket(self, ticket: int, timeout: float):
        """可取消地等待机载命令结果。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            result = self._ros.wait_for_result(ticket, timeout=0.2)
            if result is not None:
                return result
        return None

    def _wait_tcp(
        self,
        host: str,
        port: int,
        timeout: float,
        process: ManagedProcess,
    ) -> bool:
        """等待 SITL TCP 端口并监视其进程存活。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            try:
                with socket.create_connection((host, port), timeout=0.25):
                    return True
            except OSError:
                self._cancel_event.wait(0.15)
        return False

    def _wait_process_stable(self, process: ManagedProcess, duration: float) -> None:
        """要求新进程在观察窗内持续存活。"""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_process(process)
            self._cancel_event.wait(0.1)

    @staticmethod
    def _check_process(process: ManagedProcess | None) -> None:
        """若受管进程提前退出则给出日志位置。"""
        if process is not None and not process.running:
            raise RuntimeError(
                f"{process.name} 提前退出 (code={process.process.returncode})；"
                f"日志: {process.log_path}"
            )

    def _check_cancelled(self) -> None:
        """在每个可取消等待点抛出内部取消异常。"""
        if self._cancel_event.is_set():
            raise _WorkflowCancelled()

    def _terminate_local_processes(self) -> CleanupReport:
        """串行释放租约并清理本项目本地仿真进程。"""
        with self._cleanup_lock:
            self._ros.disable_remote_logs()
            self._ros.release_control(timeout=1.0)
            report = self._supervisor.terminate_all()
            self._ros.mark_environment_stopped()
            return report

    def _publish_status(
        self, callback: StatusCallback, level: LogLevel, message: str
    ) -> None:
        """由环境源端同步生成等级，再把同一事件转交界面状态栏。"""
        self._events.emit(level, "environment", message)
        callback(level, message)

    @staticmethod
    def _cleanup_suffix(report: CleanupReport) -> str:
        """将本地清理结果压缩为错误消息后缀。"""
        if report.success:
            return "；本地仿真进程已清理，远端机载端未受影响"
        return f"；本地清理仍有残留: {report.remaining or report.errors}"
