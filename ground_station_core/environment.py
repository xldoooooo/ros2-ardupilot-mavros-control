"""本地完整 SITL 编排与远端机载服务连接工作流。"""

from __future__ import annotations

import socket
import threading
import time
from collections.abc import Callable
from pathlib import Path

from .config import (
    INTERFACE_VERSION,
    ONBOARD_PARAM_FILE,
    ardupilot_root,
    find_sim_vehicle,
    mavros_apm_config,
    ros_setup_files,
)
from .process_manager import CleanupReport, ManagedProcess, ProcessSupervisor
from .ros_controller import GroundStationRosController


StatusCallback = Callable[[str], None]
DoneCallback = Callable[[bool, str], None]


class _WorkflowCancelled(RuntimeError):
    """内部异常：用户请求断开/清理时中断当前初始化。"""


class _PreflightFailed(RuntimeError):
    """内部异常：依赖检查失败且尚未改变外部环境。"""


class EnvironmentInitializer:
    """仿真时管理本地进程，实机时只连接远端机载服务。"""

    def __init__(
        self,
        ros_controller: GroundStationRosController,
        supervisor: ProcessSupervisor | None = None,
    ) -> None:
        """绑定地面站客户端及仅用于本地仿真的进程管理器。"""
        self._ros = ros_controller
        self._supervisor = supervisor or ProcessSupervisor()
        self._state_lock = threading.RLock()
        self._cleanup_lock = threading.Lock()
        self._workflow_thread: threading.Thread | None = None
        self._cancel_event = threading.Event()

    @property
    def supervisor(self) -> ProcessSupervisor:
        """暴露本地仿真进程管理器以便显示日志与测试。"""
        return self._supervisor

    @property
    def busy(self) -> bool:
        """指示连接/初始化线程是否仍在执行。"""
        with self._state_lock:
            return self._workflow_thread is not None and self._workflow_thread.is_alive()

    def initialize_simulation(
        self, status: StatusCallback, done: DoneCallback
    ) -> bool:
        """异步启动 SITL、MAVROS、同款机载 C++ 服务与 RViz。"""
        return self._start_workflow(
            "simulation", lambda: self._simulation_workflow(status), status, done
        )

    def initialize_hardware(
        self,
        origin: tuple[float, float, float],
        status: StatusCallback,
        done: DoneCallback,
    ) -> bool:
        """异步连接局域网中的机载服务，不远程管理无人机进程。"""
        return self._start_workflow(
            "hardware", lambda: self._hardware_workflow(origin, status), status, done
        )

    def _start_workflow(
        self,
        name: str,
        workflow: Callable[[], str],
        status: StatusCallback,
        done: DoneCallback,
    ) -> bool:
        """保证同一时刻只有一个环境工作流。"""
        with self._state_lock:
            if self._workflow_thread is not None and self._workflow_thread.is_alive():
                done(False, "已有初始化/连接流程正在执行")
                return False
            self._cancel_event = threading.Event()

            def runner() -> None:
                try:
                    message = workflow()
                except _PreflightFailed as exc:
                    done(False, f"初始化前检查失败: {exc}")
                except _WorkflowCancelled:
                    report = self._terminate_local_processes()
                    done(
                        False,
                        "操作已取消，本地仿真进程与控制租约已清理"
                        if report.success
                        else f"操作已取消，但本地清理仍有残留: {report.remaining}",
                    )
                except Exception as exc:
                    status(f"初始化/连接失败，正在清理本地环境: {exc}")
                    report = self._terminate_local_processes()
                    done(False, f"初始化/连接失败: {exc}{self._cleanup_suffix(report)}")
                else:
                    done(True, message)

            self._workflow_thread = threading.Thread(
                target=runner,
                name=f"ground-station-{name}-init",
                daemon=True,
            )
            self._workflow_thread.start()
            return True

    def cleanup(self) -> CleanupReport:
        """释放控制租约并只结束本机仿真进程，不触碰远端机载服务。"""
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
        """执行可取消的完整闭环仿真初始化。"""
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

        status("正在释放旧租约并清理本地仿真环境...")
        cleanup = self._terminate_local_processes()
        if cleanup.remaining:
            raise RuntimeError(f"旧进程清理不完整: {cleanup.remaining}")
        self._ros.enable_control()
        self._check_cancelled()

        status("1/5 正在启动 ArduPilot SITL...")
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

        status("2/5 正在启动仿真 MAVROS...")
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

        status("3/5 正在启动独立机载 C++ 控制服务...")
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

        status("4/5 正在由机载服务配置消息频率并等待 EKF...")
        rate_result = self._wait_ticket(self._ros.request_set_rates(), 25.0)
        if rate_result is None:
            raise RuntimeError("消息频率配置等待超时")
        if not rate_result.success:
            raise RuntimeError(rate_result.message)
        self._wait_local_position(45.0, onboard)

        status("5/5 正在启动 RViz...")
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
        """连接远端机载服务，校验接口/租约/飞控并执行维护命令。"""
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 客户端未就绪: {self._ros.error or '未知原因'}"
                )
            self._verify_ros_package("guided_interfaces", ros_setup_files())
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        status("正在停止本地仿真；远端无人机进程不会被地面站终止...")
        cleanup = self._terminate_local_processes()
        if cleanup.remaining:
            raise RuntimeError(f"本地仿真清理不完整: {cleanup.remaining}")
        self._ros.enable_control()
        self._check_cancelled()

        status("1/4 正在等待局域网机载控制服务...")
        self._wait_onboard(30.0)
        snapshot = self._ros.snapshot()
        if snapshot.interface_version != INTERFACE_VERSION:
            raise RuntimeError(
                f"接口版本不兼容：地面站 {INTERFACE_VERSION} / "
                f"机载端 {snapshot.interface_version or '--'}"
            )

        status("2/4 正在申请单一控制权并等待飞控连接...")
        self._wait_control_authority(15.0)
        self._wait_connected(35.0)
        self._wait_thrust_mode(35.0)

        status("3/4 正在由机载 MAVROS 配置消息频率和 GPS 原点...")
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
                origin_result.message if origin_result is not None else "GPS 原点设置超时"
            )

        status("4/4 正在等待远端本地位置就绪...")
        self._wait_local_position(40.0)
        return (
            "实机机载服务连接完成；地面站仅持有高层命令租约，"
            "MAVROS/驱动/控制循环均在无人机上运行"
        )

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
            self._ros.release_control(timeout=1.0)
            report = self._supervisor.terminate_all()
            self._ros.mark_environment_stopped()
            return report

    @staticmethod
    def _cleanup_suffix(report: CleanupReport) -> str:
        """将本地清理结果压缩为错误消息后缀。"""
        if report.success:
            return "；本地仿真进程已清理，远端机载端未受影响"
        return f"；本地清理仍有残留: {report.remaining or report.errors}"
