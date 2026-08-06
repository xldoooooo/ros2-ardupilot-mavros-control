"""仿真与实机的一键初始化工作流。"""

from __future__ import annotations

import socket
import threading
import time
from collections.abc import Callable
from pathlib import Path

from .config import (
    EXTNAV_SETUP,
    ODIN_SETUP,
    REAL_FCU_URL,
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
    """内部异常：用户请求清理时中断当前初始化流程。"""


class _PreflightFailed(RuntimeError):
    """内部异常：依赖检查失败且尚未改变当前外部环境。"""


class EnvironmentInitializer:
    """异步编排 SITL/MAVROS/RViz 或 Odin/MAVROS/extnav/GPS 原点。"""

    def __init__(
        self,
        ros_controller: GroundStationRosController,
        supervisor: ProcessSupervisor | None = None,
    ) -> None:
        """绑定常驻 ROS 控制器和可替换的外部进程管理器。"""
        self._ros = ros_controller
        self._supervisor = supervisor or ProcessSupervisor()
        self._state_lock = threading.RLock()
        self._cleanup_lock = threading.Lock()
        self._workflow_thread: threading.Thread | None = None
        self._cancel_event = threading.Event()

    @property
    def supervisor(self) -> ProcessSupervisor:
        """暴露进程管理器以便 GUI 显示日志位置和测试。"""
        return self._supervisor

    @property
    def busy(self) -> bool:
        """指示初始化线程是否仍在运行。"""
        with self._state_lock:
            return self._workflow_thread is not None and self._workflow_thread.is_alive()

    def initialize_simulation(
        self, status: StatusCallback, done: DoneCallback
    ) -> bool:
        """异步执行 SITL → MAVROS → 频率设置 → RViz。"""
        return self._start_workflow(
            "simulation", lambda: self._simulation_workflow(status), status, done
        )

    def initialize_hardware(
        self,
        origin: tuple[float, float, float],
        status: StatusCallback,
        done: DoneCallback,
    ) -> bool:
        """异步执行 Odin → 实机 MAVROS → extnav → 频率/GPS 原点。"""
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
        """保证同一时刻只有一个初始化线程。"""
        with self._state_lock:
            if self._workflow_thread is not None and self._workflow_thread.is_alive():
                done(False, "已有初始化流程正在执行")
                return False
            self._cancel_event = threading.Event()

            def runner() -> None:
                try:
                    message = workflow()
                except _PreflightFailed as exc:
                    done(False, f"初始化前检查失败: {exc}")
                except _WorkflowCancelled:
                    report = self._terminate_external_processes()
                    done(
                        False,
                        "初始化已取消，相关进程已清理"
                        if report.success
                        else f"初始化已取消，但清理仍有残留: {report.remaining}",
                    )
                except Exception as exc:
                    status(f"初始化失败，正在清理: {exc}")
                    report = self._terminate_external_processes()
                    suffix = self._cleanup_suffix(report)
                    done(False, f"初始化失败: {exc}{suffix}")
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
        """取消初始化、等待其停步，再彻底终止并验证所有外部 ROS 进程。"""
        with self._state_lock:
            self._cancel_event.set()
            workflow_thread = self._workflow_thread
        if (
            workflow_thread is not None
            and workflow_thread.is_alive()
            and workflow_thread is not threading.current_thread()
        ):
            workflow_thread.join(timeout=6.0)

        return self._terminate_external_processes()

    def _simulation_workflow(self, status: StatusCallback) -> str:
        """执行可取消的仿真环境初始化步骤。"""
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 节点未就绪: {self._ros.error or '未知原因'}"
                )
            sim_vehicle = find_sim_vehicle()
            if sim_vehicle is None:
                raise FileNotFoundError("未找到 sim_vehicle.py，请配置 ArduPilot PATH")
            apm_config = mavros_apm_config()
            if not apm_config.is_file():
                raise FileNotFoundError(f"MAVROS 参数文件不存在: {apm_config}")
            self._verify_ros_package("mavros", ros_setup_files())
            self._verify_ros_package("guided_sim", ros_setup_files())
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        status("正在清理旧环境...")
        cleanup = self._terminate_external_processes()
        if cleanup.remaining:
            raise RuntimeError(f"旧进程清理不完整: {cleanup.remaining}")
        self._check_cancelled()

        status("1/4 正在启动 ArduPilot SITL...")
        sitl = self._supervisor.start(
            "sitl",
            [str(sim_vehicle), "-v", "ArduCopter"],
            cwd=ardupilot_root(sim_vehicle),
            keep_stdin_open=True,
        )
        if not self._wait_tcp("127.0.0.1", 5762, 35.0, sitl):
            raise RuntimeError(f"SITL 未开放 TCP 5762；日志: {sitl.log_path}")

        status("2/4 正在启动 MAVROS 并等待飞控连接...")
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
        self._wait_connected(35.0, mavros)

        status("3/4 正在设置 MAVLink 消息频率...")
        rate_ticket = self._ros.request_set_rates()
        rate_result = self._wait_ticket(rate_ticket, 25.0)
        if rate_result is None:
            raise RuntimeError("消息频率设置等待超时")
        if not rate_result.success:
            raise RuntimeError(rate_result.message)

        status("3/4 正在等待 EKF 本地位置就绪...")
        self._wait_local_position(45.0, mavros)

        status("4/4 正在启动 RViz...")
        rviz = self._supervisor.start(
            "rviz",
            ["ros2", "launch", "guided_sim", "visualize.launch.py"],
            setup_files=ros_setup_files(),
        )
        self._wait_process_stable(rviz, 2.0)
        return f"仿真环境初始化完成；日志目录: {self._supervisor.log_directory}"

    def _hardware_workflow(
        self, origin: tuple[float, float, float], status: StatusCallback
    ) -> str:
        """执行可取消的实机环境初始化步骤，完整替代 odin1.sh。"""
        # 先验证部署依赖，缺失时不破坏当前正在运行的仿真环境。
        try:
            if not self._ros.ready:
                raise RuntimeError(
                    f"地面站 ROS 节点未就绪: {self._ros.error or '未知原因'}"
                )
            hardware_setup = ros_setup_files((ODIN_SETUP,))
            extnav_setup = ros_setup_files((EXTNAV_SETUP,))
            self._verify_ros_package("odin_ros_driver", hardware_setup)
            self._verify_ros_package("mavros", hardware_setup)
            self._verify_ros_package("extnav_bridge", extnav_setup)
        except Exception as exc:
            raise _PreflightFailed(str(exc)) from exc

        status("正在清理旧环境...")
        cleanup = self._terminate_external_processes()
        if cleanup.remaining:
            raise RuntimeError(f"旧进程清理不完整: {cleanup.remaining}")
        self._check_cancelled()

        status("1/5 正在启动 Odin 驱动...")
        odin = self._supervisor.start(
            "odin_driver",
            ["ros2", "launch", "odin_ros_driver", "odin1_ros2.launch.py"],
            setup_files=hardware_setup,
        )
        self._wait_process_stable(odin, 5.0)

        status("2/5 正在启动实机 MAVROS...")
        mavros = self._supervisor.start(
            "mavros_hardware",
            [
                "ros2",
                "launch",
                "mavros",
                "apm.launch",
                f"fcu_url:={REAL_FCU_URL}",
            ],
            setup_files=hardware_setup,
        )
        self._wait_connected(35.0, mavros)

        status("3/5 正在启动 Odin 外部导航桥接...")
        extnav = self._supervisor.start(
            "extnav_bridge",
            [
                "ros2",
                "run",
                "extnav_bridge",
                "extnav_to_vision_pose",
                "--ros-args",
                "-p",
                "vision_rate_hz:=40.0",
                "-p",
                "ctrl_rate_hz:=100.0",
                "-p",
                "odom_topic:=/odin1/odometry_highfreq",
                "-p",
                "roll_cam:=0.0",
                "-p",
                "pitch_cam:=0.0",
                "-p",
                "yaw_cam:=0.0",
                "-p",
                "odin_x:=0.06",
                "-p",
                "odin_y:=-0.03",
                "-p",
                "odin_z:=0.05",
            ],
            setup_files=extnav_setup,
        )
        self._wait_process_stable(extnav, 1.0)

        status("4/5 正在设置 MAVLink 消息频率...")
        rate_result = self._wait_ticket(self._ros.request_set_rates(), 25.0)
        if rate_result is None or not rate_result.success:
            raise RuntimeError(
                rate_result.message if rate_result is not None else "消息频率设置等待超时"
            )

        status("5/5 正在设置 GPS 原点...")
        origin_result = self._wait_ticket(
            self._ros.request_set_gp_origin(*origin), 10.0
        )
        if origin_result is None or not origin_result.success:
            raise RuntimeError(
                origin_result.message if origin_result is not None else "GPS 原点设置等待超时"
            )
        return f"实机环境初始化完成；{origin_result.message}"

    def _verify_ros_package(self, package: str, setup_files: tuple[Path, ...]) -> None:
        """在目标 overlay 环境中验证初始化所需 ROS 包。"""
        result = self._supervisor.run_checked(
            ["ros2", "pkg", "prefix", package], setup_files=setup_files
        )
        if result.returncode != 0:
            detail = result.stdout.strip() or "package not found"
            raise RuntimeError(f"ROS 包 {package} 不可用: {detail}")

    def _wait_connected(self, timeout: float, process: ManagedProcess) -> None:
        """等待新鲜的 /mavros/state 连接状态，并监视 MAVROS 提前退出。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            if not process.running:
                raise RuntimeError(
                    f"{process.name} 提前退出 (code={process.process.returncode})；"
                    f"日志: {process.log_path}"
                )
            if self._ros.snapshot().connected:
                return
            self._cancel_event.wait(0.1)
        raise RuntimeError(f"等待飞控连接超时；日志: {process.log_path}")

    def _wait_local_position(self, timeout: float, process: ManagedProcess) -> None:
        """等待新鲜本地位姿，避免 GUI 刚显示完成便遭遇 PreArm EKF 拒绝。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            if not process.running:
                raise RuntimeError(
                    f"{process.name} 提前退出 (code={process.process.returncode})；"
                    f"日志: {process.log_path}"
                )
            if self._ros.snapshot().local_position_valid:
                return
            self._cancel_event.wait(0.1)
        raise RuntimeError(f"等待 EKF 本地位置超时；日志: {process.log_path}")

    def _wait_ticket(self, ticket: int, timeout: float):
        """可取消地等待 ROS 命令票据，避免清理按钮被长超时阻塞。"""
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
        """等待 SITL TCP 端口，期间检查进程存活和取消请求。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            if not process.running:
                raise RuntimeError(
                    f"{process.name} 提前退出 (code={process.process.returncode})；"
                    f"日志: {process.log_path}"
                )
            try:
                with socket.create_connection((host, port), timeout=0.25):
                    return True
            except OSError:
                self._cancel_event.wait(0.15)
        return False

    def _wait_process_stable(self, process: ManagedProcess, duration: float) -> None:
        """要求新进程在短观察窗内持续存活，捕获缺包等即时错误。"""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_cancelled()
            if not process.running:
                raise RuntimeError(
                    f"{process.name} 提前退出 (code={process.process.returncode})；"
                    f"日志: {process.log_path}"
                )
            self._cancel_event.wait(0.1)

    def _check_cancelled(self) -> None:
        """在每个可取消等待点抛出内部取消异常。"""
        if self._cancel_event.is_set():
            raise _WorkflowCancelled()

    def _terminate_external_processes(self) -> CleanupReport:
        """串行化实际清理，避免失败回滚、关闭按钮和退出操作相互竞争。"""
        with self._cleanup_lock:
            report = self._supervisor.terminate_all()
            self._ros.mark_environment_stopped()
            return report

    @staticmethod
    def _cleanup_suffix(report: CleanupReport) -> str:
        """将失败清理结果压缩为错误消息后缀。"""
        if report.success:
            return "；已清理所有已启动进程"
        return f"；清理仍有残留: {report.remaining or report.errors}"
