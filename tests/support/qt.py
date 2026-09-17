"""Qt 闭环共用的后端替身、窗口创建与清理；不连接实机。"""

from __future__ import annotations

import os

# 必须在首次导入 Qt 前选择无显示服务平台，保证测试可在 CI/headless 运行。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import (  # noqa: E402
    QEvent,
)
from PySide6.QtWidgets import (  # noqa: E402
    QApplication,
)

from ground_station_core.config import (  # noqa: E402
    INTERFACE_VERSION,
)
from ground_station_core.event_log import EventLog, LogLevel  # noqa: E402
from ground_station_core.models import (  # noqa: E402
    CommandResult,
    FlightMode,
    VehicleSnapshot,
)
from ground_station_core.process_manager import CleanupReport  # noqa: E402
from ground_station_core.qt_ui.main_window import GroundStationWindow  # noqa: E402
from ground_station_core.qt_ui.theme import apply_theme  # noqa: E402
from ground_station_core.upstream.journal import RawFrameJournal  # noqa: E402
from ground_station_core.upstream.mapping import (  # noqa: E402
    COMMAND_MAPPINGS,
)
from ground_station_core.upstream.models import (  # noqa: E402
    UpstreamConnectionSnapshot,
    UpstreamStandbyPolicy,
)


class _FakeRosController:
    """只记录高层调用的 Qt 测试替身，不包含任何飞控算法。"""

    def __init__(self, events: EventLog, snapshot: VehicleSnapshot) -> None:
        self.event_log = events
        self.source_id = "qt-test"
        self.ready = True
        self.error = None
        self.start_calls = 0
        self.stop_calls = 0
        self.current_snapshot = snapshot
        self.calls: list[tuple[str, object]] = []
        self.results: list[CommandResult] = []
        self._ticket = 0

    def start(self) -> None:
        """模拟已就绪客户端。"""
        self.start_calls += 1
        self.ready = True

    def stop(self) -> None:
        """模拟客户端停止。"""
        self.stop_calls += 1
        self.ready = False

    def snapshot(self) -> VehicleSnapshot:
        """返回测试设置的权威快照。"""
        return self.current_snapshot

    def results_after(self, sequence: int) -> list[CommandResult]:
        """返回指定序号后的测试结果。"""
        return [result for result in self.results if result.sequence > sequence]

    def _record(self, name: str, argument: object = None) -> int:
        """记录一次 GUI 到后端的高层调用。"""
        self._ticket += 1
        self.calls.append((name, argument))
        return self._ticket

    def request_takeoff(self, altitude: float) -> int:
        return self._record("takeoff", altitude)

    def request_land(self) -> int:
        return self._record("land")

    def request_clear_abnormal(self) -> int:
        """记录入库后的异常清除请求。"""
        return self._record("clear_abnormal")

    def request_hover(self) -> int:
        return self._record("hover")

    def adjust_velocity(self, *values: float) -> int:
        return self._record("motion", values)

    def request_waypoints(
        self,
        waypoints: object,
        strategy: object = 0,
        reference_generator: object = 0,
        tracking_controller: object = 0,
        photo_nos: object = (),
    ) -> int:
        return self._record(
            "waypoints",
            (
                tuple(waypoints),
                strategy,
                reference_generator,
                tracking_controller,
                tuple(photo_nos),
            ),
        )

    def publish_waypoint_preview(self, waypoints: object) -> bool:
        """记录只读预览快照，不占用飞行命令序号。"""
        self._record("waypoint_preview", tuple(waypoints))
        return self.ready

    def request_set_gp_origin(self, *origin: float) -> int:
        return self._record("set_gp_origin", origin)


class _FakeEnvironment:
    """同步完成环境流程的测试替身。"""

    def __init__(self) -> None:
        self.busy = False
        self.mode = "none"
        self.last_origin = None
        self.communication_tests = 0
        self.preview_modes: list[str] = []
        self.cleanup_calls = 0

    def initialize_simulation(self, status, done) -> bool:
        # 仿真不接收/不写入 GPS 原点；与 EnvironmentInitializer 一致。
        self.mode = "simulation"
        self.last_origin = None
        status(LogLevel.INFO, "仿真测试环境已启动")
        done(True, "仿真测试环境完成")
        return True

    def initialize_hardware(self, origin, status, done) -> bool:
        self.mode = "hardware"
        self.last_origin = tuple(origin)
        status(LogLevel.INFO, "实机完整测试连接")
        done(True, "实机测试连接完成")
        return True

    def test_hardware_communication(self, status, done) -> bool:
        """同步模拟独立诊断；不改变已建立环境的 mode。"""
        self.communication_tests += 1
        status(LogLevel.INFO, "实机通讯零命令检测")
        done(True, "实机通讯链路检测通过；未发送命令")
        return True

    def ensure_waypoint_preview(self, connection_mode: str) -> str:
        """模拟仿真复用或实机独立 RViz 窗口。"""
        self.preview_modes.append(connection_mode)
        return (
            "已复用仿真会话中的 RViz 窗口"
            if connection_mode == "simulation"
            else "已启动地面端独立实机 RViz 预览窗口"
        )

    @staticmethod
    def cancel_hardware_communication_test() -> bool:
        """同步替身在调用返回时已经结束，因此没有可取消任务。"""
        return False

    def cleanup(self) -> CleanupReport:
        self.cleanup_calls += 1
        self.mode = "none"
        return CleanupReport()


class _PendingCommunicationEnvironment(_FakeEnvironment):
    """保持通讯检测待完成，用于验证红色终止图标和第二次点击。"""

    def __init__(self) -> None:
        super().__init__()
        self._done = None
        self.cancel_requests = 0

    def test_hardware_communication(self, status, done) -> bool:
        self.communication_tests += 1
        self.busy = True
        self._done = done
        status(LogLevel.INFO, "实机通讯检测运行中")
        return True

    def cancel_hardware_communication_test(self) -> bool:
        if not self.busy:
            return False
        self.cancel_requests += 1
        return True

    def finish_cancel(self) -> None:
        """模拟环境线程观察到取消事件并投递终态。"""
        self.busy = False
        assert self._done is not None
        self._done(False, "通讯检测已取消；未申请控制权、未发送命令")


class _FakeUpstreamService:
    """隔离 Qt 测试与真实 WebSocket 线程，并记录主窗口薄适配调用。"""

    def __init__(self) -> None:
        self.journal = RawFrameJournal()
        self.command_mappings = COMMAND_MAPPINGS
        self.standby_policy = UpstreamStandbyPolicy()
        self.calls: list[tuple[str, object]] = []
        self.low_power_return_required = False
        self.connected = False

    def start(self) -> None:
        self.calls.append(("start", None))

    def stop(self, timeout: float = 3.0) -> bool:
        self.calls.append(("stop", timeout))
        return True

    def snapshot(self) -> UpstreamConnectionSnapshot:
        return UpstreamConnectionSnapshot(
            "ws://127.0.0.1:8581/ws",
            "UAV01001",
            self.connected,
            self.connected,
            "已连接" if self.connected else "已断开",
            "测试替身",
        )

    def observe_vehicle(
        self, snapshot: object, mode: str, *, can_takeoff: bool = False
    ) -> bool:
        self.calls.append(("vehicle", (snapshot, mode, can_takeoff)))
        return self.low_power_return_required

    def observe_result(self, result: object) -> None:
        self.calls.append(("result", result))

    def reset_runtime(self) -> None:
        self.calls.append(("reset", None))

    def report_waypoints_staged(self) -> bool:
        self.calls.append(("staged", None))
        return True

    def begin_mission(
        self, ticket: int, kind: str, point_indexes: tuple[int, ...]
    ) -> None:
        self.calls.append(("mission", (ticket, kind, point_indexes)))

    def begin_landing(self, ticket: int) -> None:
        self.calls.append(("landing", ticket))

    def block_standby(self) -> None:
        self.calls.append(("standby_block", None))

    def release_standby(self) -> None:
        self.calls.append(("standby_release", None))

    def is_in_hangar(self, snapshot: VehicleSnapshot) -> bool:
        policy = self.standby_policy
        return snapshot.local_position_valid and (
            abs(snapshot.x) < policy.x_tolerance_meters
            and abs(snapshot.y) < policy.y_tolerance_meters
            and abs(snapshot.z) < policy.z_tolerance_meters
        )

    def connect(self, url: str, client_no: str) -> None:
        self.calls.append(("connect", (url, client_no)))

    def disconnect(self) -> None:
        self.calls.append(("disconnect", None))

    def restart(self, url: str, client_no: str) -> None:
        self.calls.append(("restart", (url, client_no)))

    def save_configuration(self, url: str, client_no: str) -> None:
        self.calls.append(("save", (url, client_no)))


def _application() -> QApplication:
    """复用单一 QApplication，避免 Qt 全局实例冲突。"""
    application = QApplication.instance() or QApplication([])
    apply_theme(application)
    return application


def _operational_snapshot(*, armed: bool) -> VehicleSnapshot:
    """返回通过全部链路门控的可控快照。"""
    return VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        connected=True,
        armed=armed,
        autopilot_mode="GUIDED",
        local_position_valid=True,
        active_mode=FlightMode.HOVER if armed else FlightMode.IDLE,
        controller_active=armed,
        lease_owner="qt-test",
        lease_active=True,
        control_authority=True,
        thrust_mode_verified=True,
        control_rate_hz=100.0,
    )


def _window(
    snapshot: VehicleSnapshot,
    environment: _FakeEnvironment | None = None,
) -> tuple[GroundStationWindow, _FakeRosController]:
    """创建已显示但不启动真实 ROS 的测试窗口。"""
    application = _application()
    events = EventLog()
    ros = _FakeRosController(events, snapshot)
    window = GroundStationWindow(
        event_log=events,
        ros_controller=ros,
        environment=environment or _FakeEnvironment(),
        upstream_service=_FakeUpstreamService(),
        auto_start=False,
    )
    window.show()
    application.processEvents()
    return window, ros


def _close_window(window: GroundStationWindow) -> None:
    """测试结束时绕过生产退出流程并销毁窗口。"""
    window._timer.stop()
    window._allow_close = True
    window.close()
    window.deleteLater()
    application = _application()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()
