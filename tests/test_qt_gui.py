"""PySide6 地面站状态门控、危险确认、日志筛选和响应布局测试。"""

from __future__ import annotations

import os
from dataclasses import replace
import math
from pathlib import Path
from unittest.mock import patch

# 必须在首次导入 Qt 前选择无显示服务平台，保证测试可在 CI/headless 运行。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QWheelEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QLabel, QMessageBox  # noqa: E402

from ground_station_core.config import (  # noqa: E402
    INTERFACE_VERSION,
    LAND_SPEED,
    TAKEOFF_SPEED,
    VELOCITY_SCALE,
)
from ground_station_core.event_log import EventLog, LogLevel  # noqa: E402
from ground_station_core.models import (  # noqa: E402
    CommandResult,
    FlightMode,
    VehicleSnapshot,
)
from ground_station_core.process_manager import CleanupReport  # noqa: E402
from ground_station_core.qt_ui.log_panel import LogPanel  # noqa: E402
from ground_station_core.qt_ui.main_window import GroundStationWindow  # noqa: E402
from ground_station_core.qt_ui.state import derive_availability  # noqa: E402
from ground_station_core.qt_ui.theme import apply_theme  # noqa: E402


class _FakeRosController:
    """只记录高层调用的 Qt 测试替身，不包含任何飞控算法。"""

    def __init__(self, events: EventLog, snapshot: VehicleSnapshot) -> None:
        self.event_log = events
        self.source_id = "qt-test"
        self.ready = True
        self.error = None
        self.current_snapshot = snapshot
        self.calls: list[tuple[str, object]] = []
        self.results: list[CommandResult] = []
        self._ticket = 0

    def start(self) -> None:
        """模拟已就绪客户端。"""
        self.ready = True

    def stop(self) -> None:
        """模拟客户端停止。"""
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

    def request_hover(self) -> int:
        return self._record("hover")

    def adjust_velocity(self, *values: float) -> int:
        return self._record("motion", values)

    def request_waypoints(self, waypoints: object, strategy: object = 0) -> int:
        return self._record("waypoints", (tuple(waypoints), strategy))

    def request_set_gp_origin(self, *origin: float) -> int:
        return self._record("set_gp_origin", origin)


class _FakeEnvironment:
    """同步完成环境流程的测试替身。"""

    def __init__(self) -> None:
        self.busy = False
        self.mode = "none"
        self.last_origin = None
        self.communication_tests = 0

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

    @staticmethod
    def cancel_hardware_communication_test() -> bool:
        """同步替身在调用返回时已经结束，因此没有可取消任务。"""
        return False

    def cleanup(self) -> CleanupReport:
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
        auto_start=False,
    )
    window.show()
    application.processEvents()
    return window, ros


def _close_window(window: GroundStationWindow) -> None:
    """测试结束时绕过生产退出流程并销毁窗口。"""
    window._allow_close = True
    window.close()
    _application().processEvents()


def test_availability_requires_explicit_environment_and_preserves_land() -> None:
    """未初始化环境时飞控全部禁用，冲突时仍保留 LAND 安全动作。"""
    snapshot = _operational_snapshot(armed=True)
    offline = derive_availability(
        snapshot,
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=False,
        connection_mode="none",
        waypoint_count=2,
        waypoint_running=False,
    )
    assert offline.start_environment
    assert offline.communication_test
    assert offline.origin_settings
    assert not offline.stop_simulation
    assert not offline.disconnect_hardware
    assert not offline.motion
    assert not offline.land
    assert not offline.waypoint_edit
    assert not offline.waypoint_send

    ready = derive_availability(
        snapshot,
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="simulation",
        waypoint_count=2,
        waypoint_running=False,
    )
    # 会话已建立后禁止再次启动仿真/连接实机，并锁定原点齿轮。
    assert not ready.start_environment
    assert not ready.communication_test
    assert not ready.origin_settings
    assert ready.stop_simulation
    assert not ready.disconnect_hardware
    assert ready.waypoint_edit
    assert ready.motion and ready.hover and ready.land and ready.waypoint_send

    hardware = derive_availability(
        snapshot,
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="hardware",
        waypoint_count=2,
        waypoint_running=False,
    )
    assert not hardware.start_environment
    assert not hardware.origin_settings
    assert not hardware.stop_simulation
    assert hardware.disconnect_hardware
    # 完整实机会话与仿真使用相同控制门控，保持原连接按钮的完整功能。
    assert not hardware.takeoff
    assert hardware.land
    assert hardware.motion
    assert hardware.hover
    assert hardware.waypoint_send
    assert hardware.flight_reason == "飞行控制链路已就绪"

    conflict = derive_availability(
        replace(snapshot, setpoint_conflict=True),
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="simulation",
        waypoint_count=2,
        waypoint_running=False,
    )
    assert not conflict.start_environment
    assert not conflict.motion
    assert conflict.land


def test_environment_session_gates_start_buttons_and_waypoint_widgets() -> None:
    """无会话时航点组件全禁用；会话建立后禁用双启动入口、原点齿轮并互斥启用关闭按钮。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        assert window.operations.simulation_button.isEnabled()
        assert window.operations.hardware_button.isEnabled()
        assert window.operations.communication_test_button.isEnabled()
        assert window.operations.origin_settings_button.isEnabled()
        assert not window.operations.stop_simulation_button.isEnabled()
        assert not window.operations.disconnect_hardware_button.isEnabled()
        assert not window.waypoints.x_input.isEnabled()
        assert not window.waypoints.add_button.isEnabled()
        assert not window.waypoints.send_button.isEnabled()
        assert not window.waypoints.table.isEnabled()

        window._initialize_simulation()
        window._refresh()
        assert window._environment_active
        assert window._connection_mode == "simulation"
        assert not window.operations.simulation_button.isEnabled()
        assert not window.operations.hardware_button.isEnabled()
        assert not window.operations.communication_test_button.isEnabled()
        assert not window.operations.origin_settings_button.isEnabled()
        assert window.operations.stop_simulation_button.isEnabled()
        assert not window.operations.disconnect_hardware_button.isEnabled()
        assert window.waypoints.x_input.isEnabled()
        assert window.waypoints.add_button.isEnabled()
        assert not window.waypoints.send_button.isEnabled()
    finally:
        _close_window(window)


def test_success_buttons_declare_hover_style() -> None:
    """绿色 success 按钮样式表须包含 hover 变深规则，与 primary/danger 一致。"""
    from ground_station_core.qt_ui.theme import COLORS, STYLE_SHEET

    assert "success_hover" in COLORS
    assert COLORS["success_hover"] != COLORS["success"]
    assert 'QPushButton[role="success"]:hover' in STYLE_SHEET
    assert COLORS["success_hover"] in STYLE_SHEET


def test_log_panel_combines_independent_source_generated_levels() -> None:
    """DEBUG 默认关闭；各等级仍可独立组合且不猜测消息严重度。"""
    application = _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.show()
    events.debug("source", "debug line")
    events.info("source", "warning word but source says info")
    events.warn("source", "warn line")
    events.error("source", "error line")
    events.info("source", "long " + "payload " * 100)
    panel.poll()
    assert panel.viewer.horizontalScrollBar().value() == 0
    assert not panel.level_checks[LogLevel.DEBUG].isChecked()
    assert "debug line" not in panel.displayed_text
    panel.level_checks[LogLevel.DEBUG].setChecked(True)
    panel.level_checks[LogLevel.INFO].setChecked(False)
    panel.level_checks[LogLevel.WARN].setChecked(False)
    application.processEvents()

    assert panel.selected_levels() == frozenset((LogLevel.DEBUG, LogLevel.ERROR))
    assert "debug line" in panel.displayed_text
    assert "error line" in panel.displayed_text
    assert "warn line" not in panel.displayed_text
    assert "warning word but source says info" not in panel.displayed_text
    assert panel.search_input.height() == panel.clear_button.height()
    assert not any(
        label.text() == "等级" for label in panel.findChildren(QLabel)
    )
    panel.close()


def test_all_coordinate_inputs_ignore_mouse_wheel() -> None:
    """航点数值框与原点配置对话框内的数值框都不得因滚轮改变数值。"""
    from ground_station_core.qt_ui.operations_panel import OriginConfigDialog

    window, _ros = _window(_operational_snapshot(armed=False))
    dialog = OriginConfigDialog(window.operations.origin(), parent=window)
    try:
        controls = (
            dialog.latitude_input,
            dialog.longitude_input,
            dialog.altitude_input,
            window.operations.takeoff_altitude_input,
            window.operations.takeoff_speed_input,
            window.operations.land_speed_input,
            window.waypoints.x_input,
            window.waypoints.y_input,
            window.waypoints.z_input,
            window.waypoints.yaw_input,
        )
        assert len(controls) == 10
        for control in controls:
            # 滚轮语义与生产状态门控正交；逐个临时启用后直接投递事件。
            control.setEnabled(True)
            original = control.value()
            local_position = QPointF(control.rect().center())
            global_position = QPointF(control.mapToGlobal(QPoint(5, 5)))
            event = QWheelEvent(
                local_position,
                global_position,
                QPoint(0, 0),
                QPoint(0, 120),
                Qt.MouseButton.NoButton,
                Qt.KeyboardModifier.NoModifier,
                Qt.ScrollPhase.NoScrollPhase,
                False,
            )
            QApplication.sendEvent(control, event)
            assert control.value() == original
            assert not event.isAccepted()
    finally:
        dialog.close()
        _close_window(window)


def test_origin_settings_feed_full_hardware_connection_but_not_simulation() -> None:
    """齿轮只缓存本地值；完整实机连接接收原点，仿真仍使用自身 Home。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        assert window.operations.origin_settings_button.isVisible()
        assert not hasattr(window.operations, "origin_button")

        custom = (31.0, 121.0, 10.0)
        window.operations._origin = custom
        window.operations._refresh_origin_summary()
        assert window.operations.origin() == custom
        assert "实机连接原点" in window.operations.origin_summary.text()
        assert "使用 SITL 自身 Home" in window.operations.simulation_button.toolTip()

        help_icons = {
            icon.accessibleName(): icon
            for icon in window.findChildren(QLabel, "cardHelpIcon")
        }
        assert set(help_icons) == {
            "环境与连接帮助",
            "飞行动作帮助",
            "手动操纵帮助",
            "航点任务帮助",
        }
        assert "本地 SITL 使用自身 Home" in help_icons[
            "环境与连接帮助"
        ].toolTip()
        assert "Wi-Fi 按钮仅检测通讯" in help_icons[
            "环境与连接帮助"
        ].toolTip()
        assert window.findChildren(QLabel, "cardSubtitle") == []

        # 仿真：不得把 GUI 缓存原点塞进工作流（避免与 SITL Home 冲突）。
        window._initialize_simulation()
        env = window._environment
        assert env.mode == "simulation"
        assert env.last_origin is None

        # 实机：恢复完整连接，确认文案必须明示租约/维护以及独立起飞确认。
        window._environment_active = False
        window._connection_mode = "none"
        confirmations: list[tuple[str, str]] = []

        def confirm(title: str, message: str, **_kwargs) -> bool:
            confirmations.append((title, message))
            return True

        window._confirm_action = confirm
        window._initialize_hardware()
        assert env.mode == "hardware"
        assert env.last_origin == custom
        assert confirmations
        assert "申请控制租约" in confirmations[-1][1]
        assert "控制心跳" in confirmations[-1][1]
        assert "写入飞控原点" in confirmations[-1][1]
        assert "连接动作本身不会解锁或起飞" in confirmations[-1][1]
    finally:
        _close_window(window)


def test_wifi_button_is_right_of_gear_and_does_not_create_control_session() -> None:
    """独立 Wi-Fi 图标位于齿轮右侧，点击只跑诊断且不改变环境状态。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        gear = window.operations.origin_settings_button
        wifi = window.operations.communication_test_button
        assert gear.isVisible() and wifi.isVisible()
        assert not wifi.icon().isNull()
        assert wifi.accessibleName() == "检测实机通讯链路"
        assert wifi.x() > gear.x()
        assert wifi.size() == gear.size()
        assert "不申请租约" in wifi.toolTip()
        assert "不发送命令" in wifi.toolTip()

        env = window._environment
        QTest.mouseClick(wifi, Qt.MouseButton.LeftButton)
        _application().processEvents()

        assert env.communication_tests == 1
        assert env.mode == "none"
        assert env.last_origin is None
        assert not window._environment_active
        assert window._connection_mode == "none"
        assert not ros.calls
        assert "未发送命令" in window.activity_banner.message_label.text()
    finally:
        _close_window(window)


def test_wifi_button_turns_into_cancellable_red_stop_control() -> None:
    """检测期间 Wi-Fi 图标切成红色终止方块，第二次点击只取消诊断。"""
    environment = _PendingCommunicationEnvironment()
    window, ros = _window(
        _operational_snapshot(armed=False), environment=environment
    )
    try:
        wifi = window.operations.communication_test_button
        idle_icon_key = wifi.icon().cacheKey()
        QTest.mouseClick(wifi, Qt.MouseButton.LeftButton)
        _application().processEvents()

        assert window._communication_busy
        assert wifi.isEnabled()
        assert wifi.icon().cacheKey() != idle_icon_key
        assert wifi.accessibleName() == "终止实机通讯链路检测"
        assert "红色方块" in wifi.toolTip()

        QTest.mouseClick(wifi, Qt.MouseButton.LeftButton)
        _application().processEvents()
        assert environment.cancel_requests == 1
        assert window._communication_cancel_pending
        assert not wifi.isEnabled()
        assert not ros.calls

        environment.finish_cancel()
        _application().processEvents()
        assert not window._communication_busy
        assert not window._communication_cancel_pending
        assert wifi.isEnabled()
        assert wifi.icon().cacheKey() == idle_icon_key
        assert "已取消" in window.activity_banner.message_label.text()
        assert window.activity_banner.property("tone") == "warn"
    finally:
        _close_window(window)


def test_simulation_skips_takeoff_confirmation_but_hardware_keeps_it() -> None:
    """仿真起飞直接发送；相同操作在实机会话仍受默认取消确认保护。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        assert not window.operations.takeoff_button.isEnabled()
        window._initialize_simulation()
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()

        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("仿真起飞不得弹出二次确认")
        )
        window.operations.takeoff_altitude_input.setValue(0.5)
        window._takeoff()
        assert ros.calls == [("takeoff", 0.5)]

        window._pending_commands.clear()
        window._connection_mode = "hardware"
        window._refresh()
        window._confirm_action = lambda *_args, **_kwargs: False
        window._takeoff()
        assert ros.calls == [("takeoff", 0.5)]
        window._confirm_action = lambda *_args, **_kwargs: True
        window._takeoff()
        assert ros.calls == [("takeoff", 0.5), ("takeoff", 0.5)]
        assert not window.operations.takeoff_button.isEnabled()
    finally:
        _close_window(window)


def test_simulation_skips_land_confirmation_but_hardware_keeps_it() -> None:
    """仿真 LAND 直接发送，实机 LAND 的二次确认与取消语义保持不变。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("仿真降落不得弹出二次确认")
        )
        window._land()
        assert ros.calls == [("land", None)]

        window._pending_commands.clear()
        window._connection_mode = "hardware"
        window._refresh()
        window._confirm_action = lambda *_args, **_kwargs: False
        window._land()
        assert ros.calls == [("land", None)]
        window._confirm_action = lambda *_args, **_kwargs: True
        window._land()
        assert ros.calls == [("land", None), ("land", None)]
    finally:
        _close_window(window)


def test_speed_presets_and_manual_selectors_match_ui_scope() -> None:
    """起降参数与手动选择器均跟随对应动作的状态门控。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        operations = window.operations
        assert operations.takeoff_speed() == TAKEOFF_SPEED
        assert operations.land_speed() == LAND_SPEED
        assert "暂不下传飞控" in operations.takeoff_speed_input.toolTip()
        assert "暂不下传飞控" in operations.land_speed_input.toolTip()
        assert operations.takeoff_altitude_input.suffix() == "m"
        assert operations.takeoff_speed_input.suffix() == "m/s"
        assert operations.land_speed_input.suffix() == "m/s"
        assert all(
            control.buttonSymbols() == control.ButtonSymbols.UpDownArrows
            for control in (
                operations.takeoff_altitude_input,
                operations.takeoff_speed_input,
                operations.land_speed_input,
            )
        )
        assert not operations.takeoff_button.isEnabled()
        assert not operations.takeoff_altitude_input.isEnabled()
        assert not operations.takeoff_speed_input.isEnabled()
        assert not operations.land_button.isEnabled()
        assert not operations.land_speed_input.isEnabled()
        assert all(
            not control.isEnabled()
            for control in (
                operations.coordinate_mode_combo,
                operations.left_sensitivity_combo,
                operations.right_sensitivity_combo,
            )
        )
        assert all(
            control.toolTip() == window._availability.flight_reason
            for control in (
                operations.coordinate_mode_combo,
                operations.left_sensitivity_combo,
                operations.right_sensitivity_combo,
            )
        )

        window._initialize_simulation()
        window._refresh()
        assert operations.takeoff_button.isEnabled()
        assert operations.takeoff_altitude_input.isEnabled()
        assert operations.takeoff_speed_input.isEnabled()
        assert not operations.land_button.isEnabled()
        assert not operations.land_speed_input.isEnabled()
        assert not operations.coordinate_mode_combo.isEnabled()
        assert not operations.left_sensitivity_combo.isEnabled()
        assert not operations.right_sensitivity_combo.isEnabled()

        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        assert not operations.takeoff_button.isEnabled()
        assert not operations.takeoff_altitude_input.isEnabled()
        assert not operations.takeoff_speed_input.isEnabled()
        assert operations.land_button.isEnabled()
        assert operations.land_speed_input.isEnabled()
        assert operations.coordinate_mode_combo.isEnabled()
        assert operations.left_sensitivity_combo.isEnabled()
        assert operations.right_sensitivity_combo.isEnabled()
        assert "机体坐标" in operations.coordinate_mode_combo.toolTip()
        assert "低 0.5×" in operations.left_sensitivity_combo.toolTip()
        assert "低 0.5×" in operations.right_sensitivity_combo.toolTip()
        assert operations.stop_simulation_button.text() == "终止本地仿真"
        assert operations.stop_simulation_button.property("role") == "primary"
    finally:
        _close_window(window)


def test_input_focus_blocks_keyboard_flight_shortcut() -> None:
    """数值输入聚焦时 W 等字符不能穿透成飞行命令。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.operations.motion_buttons["up"].isEnabled()

        window.waypoints.x_input.setFocus()
        _application().processEvents()
        QTest.keyClick(window, Qt.Key.Key_W)
        _application().processEvents()
        assert not ros.calls

        window.waypoints.x_input.clearFocus()
        window.setFocus()
        _application().processEvents()
        QTest.keyClick(window, Qt.Key.Key_W)
        _application().processEvents()
        assert ros.calls == [("motion", (0.0, 0.0, 0.2, 0.0))]
    finally:
        _close_window(window)


def test_waypoint_confirmation_and_responsive_two_column_splitters() -> None:
    """仿真航点免确认、实机保留确认，且缩放后两栏仍同时可见。"""
    from ground_station_core.models import WaypointFlightStrategy

    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.waypoints.add_button.isEnabled()
        window.waypoints.add_button.click()
        window.waypoints.x_input.setValue(2.0)
        window.waypoints.add_button.click()
        window._refresh()
        assert window.waypoints.send_button.isEnabled()
        assert window.waypoints.strategy_combo.isEnabled()
        assert window.waypoints.strategy_combo.count() == 3
        assert window.waypoints.selected_strategy() is WaypointFlightStrategy.STRAIGHT

        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("仿真航点发送不得弹出二次确认")
        )
        window._send_waypoints(window.waypoints.waypoints)
        assert len(ros.calls) == 1
        assert ros.calls[-1][0] == "waypoints"

        window._waypoint_running = False
        window._pending_commands.discard("waypoints")
        window._connection_mode = "hardware"
        window._refresh()
        window.waypoints.strategy_combo.setCurrentIndex(1)  # 自动避障（预留）
        window._confirm_action = lambda *_args, **_kwargs: False
        window._send_waypoints(
            window.waypoints.waypoints, window.waypoints.selected_strategy()
        )
        assert len(ros.calls) == 1
        window._confirm_action = lambda *_args, **_kwargs: True
        window._send_waypoints(
            window.waypoints.waypoints, window.waypoints.selected_strategy()
        )
        assert len(ros.calls) == 2
        assert ros.calls[-1][0] == "waypoints"
        waypoints_arg, strategy_arg = ros.calls[-1][1]
        assert len(waypoints_arg) == 2
        # 界面可选避障，但当前实现路径仍传递所选策略枚举；执行侧按直线处理。
        assert strategy_arg is WaypointFlightStrategy.AVOID

        for width, height in ((1180, 700), (1800, 1000)):
            window.resize(width, height)
            _application().processEvents()
            assert window.size().width() == width
            assert window.size().height() == height
            assert window.operations.isVisible()
            assert window.operations.manual_panel.isVisible()
            assert window.waypoints.isVisible()
            assert window.log_panel.isVisible()
            assert window.workspace_splitter.count() == 2
            assert window.operations.width() >= 650
            assert window.operations.manual_panel.width() >= 640
            assert window.waypoints.width() > 380
            assert window.log_panel.height() >= window.log_panel.minimumHeight()
    finally:
        _close_window(window)


def test_operations_layout_and_us_mode_manual_controls() -> None:
    """双卡标题对齐，手动区使用带反馈和摘要的美国手双摇杆。"""
    window, _ros = _window(_operational_snapshot(armed=True))
    try:
        window.resize(1600, 920)
        _application().processEvents()
        panel = window.operations

        # 上排两卡片同高起点，手动操纵卡片位于它们下方。
        assert (
            panel.environment_card.geometry().x()
            < panel.flight_card.geometry().x()
        )
        assert (
            panel.environment_card.geometry().y()
            == panel.flight_card.geometry().y()
        )
        top_bottom = max(
            panel.environment_card.geometry().bottom(),
            panel.flight_card.geometry().bottom(),
        )
        assert panel.manual_panel.geometry().y() > top_bottom
        assert panel.manual_card.title_label.text() == "手动操纵"
        assert not panel.findChildren(QLabel, "shortcutHint")
        assert (
            panel.environment_card.title_label.geometry().topLeft()
            == panel.flight_card.title_label.geometry().topLeft()
        )
        assert panel.left_stick_group.property("joystickDeck") is True
        assert panel.right_stick_group.property("joystickDeck") is True

        def positions(group):
            layout = group.layout()
            return {
                layout.itemAt(index).widget(): layout.getItemPosition(index)[:2]
                for index in range(layout.count())
            }

        left = positions(panel.left_stick_group)
        assert left[panel.motion_buttons["up"]] == (0, 1)
        assert left[panel.motion_buttons["yaw_left"]] == (1, 0)
        assert left[panel.motion_buttons["yaw_right"]] == (1, 2)
        assert left[panel.motion_buttons["down"]] == (2, 1)

        right = positions(panel.right_stick_group)
        assert right[panel.motion_buttons["forward"]] == (0, 1)
        assert right[panel.motion_buttons["left"]] == (1, 0)
        assert right[panel.motion_buttons["right"]] == (1, 2)
        assert right[panel.motion_buttons["back"]] == (2, 1)
        assert left[panel.left_sensitivity_combo.parentWidget()] == (1, 1)
        assert right[panel.right_sensitivity_combo.parentWidget()] == (1, 1)

        # 新增下拉框复用航点策略的完整菜单，不使用会裁切的原生弹层。
        for combo in (
            panel.coordinate_mode_combo,
            panel.left_sensitivity_combo,
            panel.right_sensitivity_combo,
        ):
            combo.showPopup()
            _application().processEvents()
            popup = combo.popup_menu
            actions = popup.actions()
            assert len(actions) == combo.count()
            assert [action.text() for action in actions] == [
                combo.itemText(index) for index in range(combo.count())
            ]
            assert all(
                popup.actionGeometry(action).height() > 0
                for action in actions
            )
            assert popup.actionGeometry(actions[-1]).bottom() < popup.height()
            assert popup.frameGeometry().top() >= combo.mapToGlobal(
                QPoint(0, combo.height())
            ).y()
            combo.hidePopup()

        # 仅重排按钮位置，八个按钮对应的速度/偏航增量保持原值。
        emitted: list[tuple[float, float, float, float]] = []
        panel.motion_requested.connect(
            lambda vx, vy, vz, yaw: emitted.append((vx, vy, vz, yaw))
        )
        expected = {
            "up": (0.0, 0.0, VELOCITY_SCALE, 0.0),
            "down": (0.0, 0.0, -VELOCITY_SCALE, 0.0),
            "yaw_left": (0.0, 0.0, 0.0, VELOCITY_SCALE),
            "yaw_right": (0.0, 0.0, 0.0, -VELOCITY_SCALE),
            "forward": (VELOCITY_SCALE, 0.0, 0.0, 0.0),
            "back": (-VELOCITY_SCALE, 0.0, 0.0, 0.0),
            "left": (0.0, VELOCITY_SCALE, 0.0, 0.0),
            "right": (0.0, -VELOCITY_SCALE, 0.0, 0.0),
        }
        for name, command in expected.items():
            panel.motion_buttons[name].setEnabled(True)
            panel.motion_buttons[name].click()
            assert emitted[-1] == command

        hover_origin = panel.hover_button.mapTo(panel.manual_card, QPoint(0, 0))

        def stick_spacing() -> tuple[int, int, int]:
            """返回相对手动卡片的左留白、右留白和两个底盘中缝。"""
            left_origin = panel.left_stick_group.mapTo(
                panel.manual_card, QPoint(0, 0)
            )
            right_origin = panel.right_stick_group.mapTo(
                panel.manual_card, QPoint(0, 0)
            )
            return (
                left_origin.x(),
                panel.manual_card.width()
                - right_origin.x()
                - panel.right_stick_group.width(),
                right_origin.x()
                - left_origin.x()
                - panel.left_stick_group.width(),
            )

        left_origin = panel.left_stick_group.mapTo(
            panel.manual_card, QPoint(0, 0)
        )
        assert hover_origin.y() > (
            left_origin.y() + panel.left_stick_group.height()
        )
        assert abs(
            hover_origin.x()
            + panel.hover_button.width() / 2
            - panel.manual_card.width() / 2
        ) < 20
        default_spacing = stick_spacing()
        assert min(default_spacing[:2]) > 24
        assert abs(default_spacing[0] - default_spacing[1]) < 12
        assert 20 <= default_spacing[2] <= 32

        # 外侧留白随可用宽度变化，中缝不会被无限拉大。
        window.resize(1180, 700)
        _application().processEvents()
        narrow_spacing = stick_spacing()
        window.resize(1800, 1000)
        _application().processEvents()
        wide_spacing = stick_spacing()
        assert min(narrow_spacing[:2]) > 12
        assert wide_spacing[0] > narrow_spacing[0]
        assert wide_spacing[1] > narrow_spacing[1]
        assert abs(narrow_spacing[0] - narrow_spacing[1]) < 12
        assert abs(wide_spacing[0] - wide_spacing[1]) < 12
        assert 20 <= narrow_spacing[2] <= 32
        assert 20 <= wide_spacing[2] <= 32
        assert panel.hover_button.text() == "制动并悬停  SPACE"

        # 按下时按钮和底盘偏移同步，松开后都回中。
        forward = panel.motion_buttons["forward"]
        indicator = panel._motion_indicators["forward"]
        forward.pressed.emit()
        assert forward.property("manualActive") is True
        assert indicator.offset == (0.0, -1.0)
        forward.released.emit()
        assert forward.property("manualActive") is False
        assert indicator.offset == (0.0, 0.0)

        # 大数字摘要常驻，XYZ/jitter/miss 等工程信息默认折叠。
        assert panel.manual_summary_panel.isVisible()
        assert panel.altitude_summary_value.text() == "+0.00"
        assert panel.control_authority_chip.text() == "控制权 · 已取得"
        assert "尚未发送" in panel.last_manual_command_chip.text()
        assert not panel.engineering_panel.isVisible()
        panel.engineering_toggle.click()
        _application().processEvents()
        assert panel.engineering_panel.isVisible()
        assert "jitter" in panel.controller_value.text()
    finally:
        _close_window(window)


def test_manual_coordinate_modes_and_independent_stick_sensitivity() -> None:
    """机体/ENU 坐标转换和左右摇杆灵敏度对鼠标、键盘统一生效。"""
    snapshot = replace(
        _operational_snapshot(armed=True),
        yaw=math.pi / 2.0,
        z=3.25,
        target_vx=0.3,
        target_vy=0.4,
        target_vz=-0.1,
        target_yaw_rate=math.radians(15.0),
    )
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.operations

        assert panel.coordinate_mode() == "body"
        assert panel.coordinate_mode_combo.currentText() == "机体坐标"
        assert panel.altitude_summary_value.text() == "+3.25"
        assert panel.horizontal_summary_value.text() == "0.50"
        assert panel.vertical_summary_value.text() == "-0.10"
        assert panel.yaw_rate_summary_value.text() == "+15.0"

        # 右摇杆低灵敏度：机头朝 +Y 时，I 的机体前进转换为 ENU +Y。
        panel.right_sensitivity_combo.setCurrentIndex(0)
        panel.motion_buttons["forward"].click()
        command = ros.calls[-1][1]
        assert ros.calls[-1][0] == "motion"
        assert abs(command[0]) < 1e-9
        assert abs(command[1] - VELOCITY_SCALE * 0.5) < 1e-9

        # 同一灵敏度在 ENU 模式下不旋转，I 直接增加本地 +X。
        panel.coordinate_mode_combo.setCurrentIndex(1)
        coordinate_event = window.event_log.snapshot()[-1]
        assert coordinate_event.level is LogLevel.INFO
        assert coordinate_event.source == "operator"
        assert "坐标系切换为「本地 ENU」" in coordinate_event.message
        assert "固定 X/Y 轴" in coordinate_event.message
        panel.motion_buttons["forward"].click()
        command = ros.calls[-1][1]
        assert abs(command[0] - VELOCITY_SCALE * 0.5) < 1e-9
        assert abs(command[1]) < 1e-9

        # 左摇杆高灵敏度独立控制升降/偏航，不受右摇杆倍率影响。
        panel.left_sensitivity_combo.setCurrentIndex(2)
        panel.motion_buttons["up"].click()
        assert ros.calls[-1][1] == (0.0, 0.0, VELOCITY_SCALE * 2.0, 0.0)
        panel.motion_buttons["yaw_left"].click()
        assert ros.calls[-1][1] == (0.0, 0.0, 0.0, VELOCITY_SCALE * 2.0)

        # 键盘快捷键也使用当前 ENU 坐标和右摇杆低灵敏度。
        panel.coordinate_mode_combo.clearFocus()
        window.setFocus()
        QTest.keyClick(window, Qt.Key.Key_I)
        _application().processEvents()
        assert ros.calls[-1][1] == (VELOCITY_SCALE * 0.5, 0.0, 0.0, 0.0)
        assert "尚未发送" not in panel.last_manual_command_chip.text()
    finally:
        _close_window(window)


def test_waypoint_editor_compacts_rows_icons_and_downward_strategy_popup() -> None:
    """航点输入含单位/箭头，“+”添加与三项完整向下菜单均可用。"""
    from ground_station_core.qt_ui.theme import COLORS, STYLE_SHEET

    window, _ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.waypoints
        assert panel.add_button.text() == "+"
        assert panel.add_button.property("role") == "neutral"
        input_heights = {
            control.height()
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        }
        assert input_heights == {panel._ROW_HEIGHT}
        assert all(
            control.buttonSymbols() == control.ButtonSymbols.UpDownArrows
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        )
        assert [
            panel.x_input.suffix(),
            panel.y_input.suffix(),
            panel.z_input.suffix(),
            panel.yaw_input.suffix(),
        ] == ["m", "m", "m", "°"]
        assert "单位" in panel.x_input.toolTip()
        assert panel.table.horizontalHeader().height() == panel._ROW_HEIGHT
        assert panel.table.verticalHeader().defaultSectionSize() == panel._ROW_HEIGHT

        panel.add_button.click()
        panel.x_input.setValue(2.0)
        panel.add_button.click()
        assert all(
            panel.table.rowHeight(row) == panel._ROW_HEIGHT
            for row in range(panel.table.rowCount())
        )
        assert panel.up_button.text() == panel.down_button.text() == ""
        assert panel.remove_button.text() == ""
        assert not panel.up_button.icon().isNull()
        assert not panel.down_button.icon().isNull()
        assert not panel.remove_button.icon().isNull()
        assert "删除" in panel.remove_button.accessibleName()
        assert panel.up_button.x() < panel.down_button.x() < panel.remove_button.x()
        assert panel.clear_button.text() == "清空"
        assert panel.send_button.property("role") == "primary"
        assert "QProgressBar#waypointProgress::chunk" in STYLE_SHEET
        assert COLORS["success"] in STYLE_SHEET

        combo = panel.strategy_combo
        combo.showPopup()
        _application().processEvents()
        popup = combo.popup_menu
        popup_top = popup.frameGeometry().top()
        combo_bottom = combo.mapToGlobal(QPoint(0, combo.height())).y()
        assert popup_top >= combo_bottom
        actions = popup.actions()
        assert len(actions) == combo.count() == 3
        assert [action.text() for action in actions] == [
            combo.itemText(index) for index in range(combo.count())
        ]
        assert all(popup.actionGeometry(action).height() > 0 for action in actions)
        assert popup.actionGeometry(actions[-1]).bottom() < popup.height()
        assert popup.width() >= max(
            popup.fontMetrics().horizontalAdvance(action.text()) + 24
            for action in actions
        )
        actions[1].trigger()
        _application().processEvents()
        assert combo.currentIndex() == 1
        assert not popup.isVisible()

        panel._progress_tracking = True
        window._waypoint_running = True
        window._refresh()
        assert not panel.add_button.isEnabled()
        assert not panel.strategy_combo.isEnabled()
        assert all(
            not control.isEnabled()
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        )
    finally:
        _close_window(window)


def test_clearing_completed_waypoints_resets_stale_onboard_progress() -> None:
    """清空本地列表后，机载旧快照不能把已结束任务进度重新显示出来。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window.waypoints.add_button.click()
        window.waypoints.set_result("任务执行中", running=True)
        stale_snapshot = replace(
            ros.current_snapshot, waypoint_count=1, waypoint_index=1
        )
        window.waypoints.update_progress(stale_snapshot)
        assert window.waypoints.progress.value() == 1
        assert "1/1" in window.waypoints.progress.format()

        window.waypoints.clear_waypoints()
        window.waypoints.update_progress(stale_snapshot)
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.maximum() == 1
        assert window.waypoints.progress.format() == "尚未执行"
    finally:
        _close_window(window)


def test_running_waypoint_result_does_not_reset_existing_progress() -> None:
    """后续 RUNNING 结果不得把已完成航点进度闪回“等待机载任务进度”。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.add_button.click()
        window.waypoints.x_input.setValue(2.0)
        window.waypoints.add_button.click()
        window._refresh()
        window.waypoints.set_result("任务执行中", running=True)
        progressed = replace(
            ros.current_snapshot, waypoint_count=2, waypoint_index=1
        )
        window.waypoints.update_progress(progressed)
        before = (
            window.waypoints.progress.value(),
            window.waypoints.progress.maximum(),
            window.waypoints.progress.format(),
        )

        window.waypoints.set_result("前往航点 2/2", running=True)
        assert (
            window.waypoints.progress.value(),
            window.waypoints.progress.maximum(),
            window.waypoints.progress.format(),
        ) == before
        assert "等待" not in window.waypoints.progress.format()

        # 只有真正发送下一项新任务时，才允许显式回到新的等待状态。
        window.waypoints.set_result("上一任务完成", running=False)
        window._send_waypoints(window.waypoints.waypoints)
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.format() == "等待机载任务进度…"
    finally:
        _close_window(window)


def test_compact_status_menu_shadow_and_terminal_entry_are_present() -> None:
    """顶部保留菜单、完整终端文案、红色退出入口和四周阴影。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        assert window.findChild(type(window.connection_label), "windowTitle") is None
        assert [action.text() for action in window.menuBar().actions()] == [
            "设置",
            "帮助",
        ]
        assert [action.text() for action in window.settings_menu.actions()] == [
            "显示实时日志",
            "恢复默认布局",
        ]
        assert window.terminal_button.isVisible()
        assert window.terminal_button.text() == "在此处打开终端"
        assert window.exit_button.isVisible()
        assert window.exit_button.text() == "退出地面站"
        assert window.exit_button.property("role") == "danger"
        assert window.exit_button.x() > window.terminal_button.x()
        assert window.windowFlags() & Qt.WindowType.FramelessWindowHint
        assert window.testAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        assert window.outer_window_frame.graphicsEffect() is not None
        assert window.outer_window_frame.geometry() == window.rect().adjusted(
            14, 14, -14, -14
        )
        assert window.window_surface.graphicsEffect() is None
        assert window.minimize_button.isVisible()
        assert window.maximize_button.isVisible()
        assert window.close_button.isVisible()
        assert window._resize_edges_at(0, 0) == (
            Qt.Edge.LeftEdge | Qt.Edge.TopEdge
        )
        assert window._resize_edges_at(window.width(), window.height()) == (
            Qt.Edge.RightEdge | Qt.Edge.BottomEdge
        )
        assert window.activity_banner.maximumHeight() <= 42
        for badge in (
            window.link_badge,
            window.aircraft_badge,
            window.mode_badge,
            window.control_badge,
        ):
            assert badge.maximumHeight() <= 42
    finally:
        _close_window(window)


def test_exit_button_always_requires_default_cancel_confirmation() -> None:
    """未飞行时点击右上退出也必须先显示危险操作式二次确认。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    confirmations: list[tuple[str, str, bool]] = []
    try:
        def reject_exit(title: str, message: str, critical: bool = False) -> bool:
            confirmations.append((title, message, critical))
            return False

        window._confirm_action = reject_exit
        QTest.mouseClick(window.exit_button, Qt.MouseButton.LeftButton)
        _application().processEvents()

        assert confirmations
        assert confirmations[-1][0] == "退出地面站"
        assert "终止本项目启动的本地仿真进程" in confirmations[-1][1]
        assert confirmations[-1][2] is False
        assert not window._shutting_down
        assert window.isVisible()
    finally:
        _close_window(window)


def test_all_message_boxes_use_frameless_shadow_surface() -> None:
    """确认、警告、帮助和关于共用带标题、边框与阴影的子窗口实现。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    dialog = window._message_box(
        "确认起飞",
        "请确认飞行区域安全。",
        QMessageBox.Icon.Warning,
        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
        QMessageBox.StandardButton.Cancel,
    )
    try:
        dialog.show()
        QTest.qWait(50)
        QApplication.instance().processEvents()
        assert dialog.windowFlags() & Qt.WindowType.FramelessWindowHint
        assert dialog.testAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        assert dialog.surface.graphicsEffect() is not None
        assert dialog.surface.geometry() == dialog.rect().adjusted(14, 14, -14, -14)
        title = dialog.findChild(type(window.connection_label), "dialogTitle")
        close_button = dialog.findChild(
            type(window.terminal_button), "dialogCloseButton"
        )
        assert title is not None
        assert close_button is not None
        assert dialog.defaultButton() is dialog.button(
            QMessageBox.StandardButton.Cancel
        )
        assert dialog.width() >= 430
        assert dialog.height() >= 180
    finally:
        dialog.close()
        _close_window(window)


def test_maximized_window_removes_shadow_margin_and_restores_it() -> None:
    """最大化时不保留透明缝隙，还原后恢复完整外缘阴影。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        window._toggle_maximized()
        QTest.qWait(40)
        assert window.isMaximized()
        assert window.contentsMargins().left() == 1
        assert not window._window_shadow.isEnabled()
        assert window.outer_window_frame.property("windowMaximized") is True
        assert window.maximize_button.text() == "❐"

        window._toggle_maximized()
        QTest.qWait(40)
        assert not window.isMaximized()
        assert window.contentsMargins().left() == 15
        assert window._window_shadow.isEnabled()
        assert window.outer_window_frame.property("windowMaximized") is False
        assert window.maximize_button.text() == "□"
    finally:
        _close_window(window)


def test_terminal_launcher_uses_current_directory_without_shell() -> None:
    """终端入口选择已安装程序，并把地面站当前目录作为子进程工作目录。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        with (
            patch(
                "ground_station_core.qt_ui.main_window.shutil.which",
                side_effect=lambda name: "/usr/bin/x-terminal-emulator"
                if name == "x-terminal-emulator"
                else None,
            ),
            patch(
                "ground_station_core.qt_ui.main_window.QProcess.startDetached",
                return_value=(True, 1234),
            ) as start_detached,
        ):
            window._open_terminal()
        start_detached.assert_called_once_with(
            "/usr/bin/x-terminal-emulator", [], str(Path.cwd())
        )
        assert "已启动当前目录终端" in window.activity_banner.message_label.text()
    finally:
        _close_window(window)


def test_log_auto_scroll_can_be_disabled() -> None:
    """关闭自动滚动后，追加日志不得把视口强制拉到底部。"""
    application = _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.resize(640, 240)
    panel.show()
    application.processEvents()

    for index in range(80):
        events.info("source", f"seed line {index:03d} " + ("x" * 40))
    panel.poll()
    application.processEvents()

    vertical = panel.viewer.verticalScrollBar()
    assert vertical.maximum() > 0
    vertical.setValue(0)
    panel.auto_scroll.setChecked(False)
    application.processEvents()
    assert vertical.value() == 0

    events.info("source", "new line while auto-scroll disabled " + ("y" * 40))
    panel.poll()
    application.processEvents()
    # 允许 1px 量级布局误差，但绝不能跳到末尾。
    assert vertical.value() <= 2
    assert vertical.value() < vertical.maximum()
    panel.close()
