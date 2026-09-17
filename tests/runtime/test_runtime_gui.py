"""runtime 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _FakeEnvironment,
    _FakeRosController,
    _FakeUpstreamService,
    _PendingCommunicationEnvironment,
    _application,
    _close_window,
    _operational_snapshot,
    _window,
)
import threading
from dataclasses import replace
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QLabel
from ground_station_core.event_log import EventLog
from ground_station_core.models import VehicleSnapshot
from ground_station_core.qt_ui.main_window import GroundStationWindow
from ground_station_core.qt_ui.state import derive_availability


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
    assert not offline.waypoint_configuration
    assert not offline.waypoint_send
    assert not offline.waypoint_preview

    lazy_ros = derive_availability(
        VehicleSnapshot(),
        ros_ready=False,
        busy=False,
        closing=False,
        environment_active=False,
        connection_mode="none",
        waypoint_count=0,
        waypoint_running=False,
    )
    assert lazy_ros.start_environment
    assert lazy_ros.communication_test
    assert not lazy_ros.takeoff

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
    assert not ready.waypoint_configuration
    assert ready.waypoint_preview
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
    assert hardware.waypoint_preview
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

    abnormal_ground = derive_availability(
        replace(_operational_snapshot(armed=False), vehicle_abnormal=True),
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="simulation",
        waypoint_count=2,
        waypoint_running=False,
    )
    abnormal_air = derive_availability(
        replace(snapshot, vehicle_abnormal=True),
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="simulation",
        waypoint_count=2,
        waypoint_running=False,
    )
    assert not abnormal_ground.takeoff
    assert not abnormal_air.waypoint_send
    assert abnormal_air.land and abnormal_air.hover
    assert abnormal_air.flight_reason == "无人机状态异常"

    endpoint_conflict = derive_availability(
        replace(snapshot, endpoint_conflict=True),
        ros_ready=True,
        busy=False,
        closing=False,
        environment_active=True,
        connection_mode="hardware",
        waypoint_count=2,
        waypoint_running=False,
    )
    assert not endpoint_conflict.takeoff
    assert not endpoint_conflict.land
    assert not endpoint_conflict.motion
    assert not endpoint_conflict.hover
    assert not endpoint_conflict.waypoint_send
    assert "多个机载状态发布者" in endpoint_conflict.flight_reason


def test_opening_window_does_not_start_ros_until_a_workflow_is_requested() -> None:
    """只显示 GUI 不得创建 DDS participant；入口仍可按需启动后台。"""
    application = _application()
    events = EventLog()
    ros = _FakeRosController(events, VehicleSnapshot())
    ros.ready = False
    window = GroundStationWindow(
        event_log=events,
        ros_controller=ros,
        environment=_FakeEnvironment(),
        upstream_service=_FakeUpstreamService(),
    )
    window.show()
    application.processEvents()
    try:
        assert ros.start_calls == 0
        assert window.operations.simulation_button.isEnabled()
        assert window.operations.hardware_button.isEnabled()
        assert window.operations.communication_test_button.isEnabled()
        assert window.link_badge.value_label.text() == "ROS IDLE"
    finally:
        _close_window(window)


def test_cleanup_in_progress_keeps_all_environment_entry_points_disabled() -> None:
    """旧清理未完成时不得并发启动仿真、实机连接或通讯检测。"""
    application = _application()
    window, _ros = _window(VehicleSnapshot())
    release = threading.Event()
    cleanup_thread = threading.Thread(target=release.wait, daemon=True)
    cleanup_thread.start()
    window._cleanup_thread = cleanup_thread
    try:
        window._refresh()
        application.processEvents()
        assert not window.operations.simulation_button.isEnabled()
        assert not window.operations.hardware_button.isEnabled()
        assert not window.operations.communication_test_button.isEnabled()
    finally:
        release.set()
        cleanup_thread.join(timeout=1.0)
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
        assert "本地 SITL 使用自身 Home" in help_icons["环境与连接帮助"].toolTip()
        assert "Wi-Fi 按钮仅检测通讯" in help_icons["环境与连接帮助"].toolTip()
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
    window, ros = _window(_operational_snapshot(armed=False), environment=environment)
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


def test_external_termination_skips_dialog_and_cleans_backend_once() -> None:
    """SIGHUP 等外部退出必须完成环境/ROS 清理，且同步兜底不得重复执行。"""
    environment = _FakeEnvironment()
    window, ros = _window(_operational_snapshot(armed=False), environment=environment)
    try:
        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("外部终止信号不应等待无人可操作的确认框")
        )
        window.request_external_shutdown("SIGHUP")

        for _attempt in range(100):
            _application().processEvents()
            if window._allow_close:
                break
            QTest.qWait(10)

        assert window._allow_close
        assert environment.cleanup_calls == 1
        assert ros.stop_calls == 1
        assert not ros.ready
        assert window.finalize_process_exit().success
        assert environment.cleanup_calls == 1
        assert ros.stop_calls == 1
    finally:
        _close_window(window)


def test_event_loop_exit_fallback_cleans_without_close_event() -> None:
    """事件循环若绕过窗口关闭事件，进程退出兜底仍必须清理且保持幂等。"""
    environment = _FakeEnvironment()
    window, ros = _window(_operational_snapshot(armed=False), environment=environment)
    try:
        first = window.finalize_process_exit()
        second = window.finalize_process_exit()

        assert first.success and second.success
        assert environment.cleanup_calls == 1
        assert ros.stop_calls == 1
        assert not ros.ready
    finally:
        _close_window(window)
