"""PySide6 地面站状态门控、危险确认、日志筛选和响应布局测试。"""

from __future__ import annotations

import os
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

# 必须在首次导入 Qt 前选择无显示服务平台，保证测试可在 CI/headless 运行。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QWheelEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QLabel, QMessageBox  # noqa: E402

from ground_station_core.config import INTERFACE_VERSION  # noqa: E402
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
        status(LogLevel.INFO, f"实机测试连接：{origin[0]:.3f}")
        done(True, "实机测试连接完成")
        return True

    def cleanup(self) -> CleanupReport:
        self.mode = "none"
        return CleanupReport()


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
) -> tuple[GroundStationWindow, _FakeRosController]:
    """创建已显示但不启动真实 ROS 的测试窗口。"""
    application = _application()
    events = EventLog()
    ros = _FakeRosController(events, snapshot)
    window = GroundStationWindow(
        event_log=events,
        ros_controller=ros,
        environment=_FakeEnvironment(),
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
    """各等级可独立组合筛选，且不从消息文本重新判断严重度。"""
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
    panel.level_checks[LogLevel.INFO].setChecked(False)
    panel.level_checks[LogLevel.WARN].setChecked(False)
    application.processEvents()

    assert panel.selected_levels() == frozenset((LogLevel.DEBUG, LogLevel.ERROR))
    assert "debug line" in panel.displayed_text
    assert "error line" in panel.displayed_text
    assert "warn line" not in panel.displayed_text
    assert "warning word but source says info" not in panel.displayed_text
    panel.close()


def test_all_coordinate_inputs_ignore_mouse_wheel() -> None:
    """航点数值框与原点配置对话框内的数值框都不得因滚轮改变数值。"""
    from ground_station_core.qt_ui.operations_panel import OriginConfigDialog

    window, _ros = _window(_operational_snapshot(armed=False))
    dialog = OriginConfigDialog(window.operations.origin(), parent=window)
    try:
        # 航点数值框仅在环境会话建立后启用，否则滚轮事件会被 Qt 禁用控件吞掉。
        window._environment_active = True
        window._refresh()
        controls = (
            dialog.latitude_input,
            dialog.longitude_input,
            dialog.altitude_input,
            window.operations.takeoff_altitude_input,
            window.waypoints.x_input,
            window.waypoints.y_input,
            window.waypoints.z_input,
            window.waypoints.yaw_input,
        )
        assert len(controls) == 8
        for control in controls:
            assert control.isEnabled()
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


def test_origin_settings_are_local_and_applied_on_hardware_only() -> None:
    """齿轮仅缓存原点；仿真启动不写原点，实机连接才传入缓存原点。"""
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

        visible_copy = " ".join(
            label.text() for label in window.operations.findChildren(QLabel)
        )
        assert "本地 SITL 使用自身 Home" in visible_copy
        assert "启动本地仿真」或「连接实机服务」时一并写入" not in visible_copy

        # 仿真：不得把 GUI 缓存原点塞进工作流（避免与 SITL Home 冲突）。
        window._initialize_simulation()
        env = window._environment
        assert env.mode == "simulation"
        assert env.last_origin is None

        # 实机：确认后才使用缓存原点。
        window._environment_active = False
        window._connection_mode = "none"
        window._confirm_action = lambda *_args, **_kwargs: True
        window._initialize_hardware()
        assert env.mode == "hardware"
        assert env.last_origin == custom
    finally:
        _close_window(window)


def test_environment_gate_and_takeoff_confirmation() -> None:
    """环境完成前禁止起飞，危险确认取消时不得触发后端调用。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        assert not window.operations.takeoff_button.isEnabled()
        window._initialize_simulation()
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()

        window._confirm_action = lambda *_args, **_kwargs: False
        window._takeoff()
        assert not ros.calls

        window.operations.takeoff_altitude_input.setValue(0.5)
        window._confirm_action = lambda *_args, **_kwargs: True
        window._takeoff()
        assert ros.calls == [("takeoff", 0.5)]
        assert not window.operations.takeoff_button.isEnabled()
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


def test_waypoint_confirmation_and_responsive_three_column_splitters() -> None:
    """航点上传需确认，最小与放大尺寸下三栏和日志均同时可见。"""
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

        window._confirm_action = lambda *_args, **_kwargs: False
        window._send_waypoints(window.waypoints.waypoints)
        assert not ros.calls
        window._confirm_action = lambda *_args, **_kwargs: True
        window.waypoints.strategy_combo.setCurrentIndex(1)  # 自动避障（预留）
        window._send_waypoints(
            window.waypoints.waypoints, window.waypoints.selected_strategy()
        )
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
            assert window.operations.width() > 300
            assert window.operations.manual_panel.width() > 320
            assert window.waypoints.width() > 380
            assert window.log_panel.height() >= window.log_panel.minimumHeight()
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


def test_compact_status_menu_shadow_and_terminal_entry_are_present() -> None:
    """顶部不再显示大标题，同时保留紧凑状态、菜单、终端入口和四周阴影。"""
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
