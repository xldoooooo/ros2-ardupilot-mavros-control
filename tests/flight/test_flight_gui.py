"""flight 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _application,
    _close_window,
    _operational_snapshot,
    _window,
)
import math
from dataclasses import replace
from unittest.mock import patch
from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel
from ground_station_core.config import (
    HARDWARE_BATTERY_GOOD_VOLTAGE,
    HARDWARE_BATTERY_WARNING_VOLTAGE,
    LAND_SPEED,
    SIMULATION_BATTERY_GOOD_PERCENTAGE,
    SIMULATION_BATTERY_WARNING_PERCENTAGE,
    STATUS_RATE_TARGET_HZ,
    STATUS_RATE_TOLERANCE_HZ,
    TAKEOFF_SPEED,
    VELOCITY_SCALE,
)
from ground_station_core.event_log import LogLevel
from ground_station_core.models import FlightMode
from ground_station_core.qt_ui.state import derive_availability


def test_land_remains_available_in_every_armed_mode_and_busy_workflow() -> None:
    """可靠持权链上的任何已武装状态都允许幂等重发 LAND。"""
    snapshot = _operational_snapshot(armed=True)
    for mode in FlightMode:
        availability = derive_availability(
            replace(snapshot, active_mode=mode),
            ros_ready=True,
            busy=True,
            closing=False,
            environment_active=True,
            connection_mode="simulation",
            waypoint_count=1,
            waypoint_running=True,
            flight_sequence_active=True,
        )
        assert availability.land, mode
        assert not availability.takeoff

    # 重复发送仍必须服从真实链路边界，不会绕过关闭或端点冲突。
    closing = derive_availability(
        snapshot,
        ros_ready=True,
        busy=False,
        closing=True,
        environment_active=True,
        connection_mode="simulation",
        waypoint_count=1,
        waypoint_running=False,
    )
    assert not closing.land


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


def test_manual_tooltips_and_detailed_increment_state() -> None:
    """手动动作提示语义准确，详细状态按当前左右灵敏度即时更新。"""
    snapshot = replace(
        _operational_snapshot(armed=True),
        roll=math.radians(7.6),
        pitch=math.radians(-4.2),
        yaw=math.radians(92.3),
        vx=0.3,
        vy=0.4,
        battery_valid=True,
        battery_voltage=15.76,
        battery_current=3.21,
        battery_percentage=0.74,
        status_rate_hz=9.98,
        status_age_seconds=0.04,
    )
    window, _ros = _window(snapshot)
    try:
        panel = window.operations
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()

        expected_tooltips = {
            "up": "增加向上的期望速度",
            "down": "增加向下的期望速度",
            "yaw_left": "增加向左偏航的期望角速度",
            "yaw_right": "增加向右偏航的期望角速度",
            "forward": "增加水平向前的期望速度",
            "back": "增加水平向后的期望速度",
            "left": "增加水平向左的期望速度",
            "right": "增加水平向右的期望速度",
        }
        for name, phrase in expected_tooltips.items():
            button = panel.motion_buttons[name]
            assert button.isEnabled()
            assert phrase in button.toolTip()
            assert button.accessibleDescription() == button.toolTip()
        assert "制动并悬停指令" in panel.hover_button.toolTip()
        assert panel.engineering_toggle.text() == "详细状态"

        position_row = panel.engineering_left_layout.getItemPosition(
            panel.engineering_left_layout.indexOf(panel.position_value)
        )
        increment_row = panel.engineering_increment_layout.getItemPosition(
            panel.engineering_increment_layout.indexOf(panel.vertical_increment_value)
        )
        assert position_row[1] == 1
        assert increment_row[1] == 1
        assert (
            panel.engineering_left_panel.geometry().x()
            < panel.engineering_increment_panel.geometry().x()
        )
        assert panel.vertical_increment_value.text() == "±0.20 m/s"
        assert panel.yaw_increment_value.text() == "±11.5 °/s"
        assert panel.longitudinal_increment_value.text() == "±0.20 m/s"
        assert panel.lateral_increment_value.text() == "±0.20 m/s"
        assert panel.attitude_value.text() == "俯仰 -4.2° · 滚转 +7.6°"

        # 五个状态块在坐标系右侧按指定顺序展示。
        status_chips = (
            panel.control_authority_chip,
            panel.autopilot_mode_chip,
            panel.battery_status_chip,
            panel.communication_rate_chip,
            panel.last_manual_command_chip,
        )
        chip_positions = [
            chip.mapTo(panel.manual_card, QPoint(0, 0)).x() for chip in status_chips
        ]
        assert chip_positions == sorted(chip_positions)
        assert panel.autopilot_mode_chip.text() == "飞控模式 · GUIDED"
        assert panel.battery_status_chip.text() == "电池 · 74%"
        assert panel.battery_status_chip.property("tone") == "good"
        assert panel.communication_rate_chip.text() == "通讯频率 · 9.98 Hz"
        assert panel.communication_rate_chip.property("tone") == "good"
        assert "age" not in panel.communication_rate_chip.text()

        # 六项摘要和左侧详细状态严格使用任务指定的顺序。
        summary_titles = [
            label.text()
            for label in panel.manual_summary_panel.findChildren(QLabel)
            if label.objectName() == "manualSummaryTitle"
        ]
        assert summary_titles == [
            "实际高度",
            "实际速度",
            "实际航向",
            "指令水平速度",
            "指令垂直速度",
            "指令转向速度",
        ]
        assert panel.actual_speed_summary_value.text() == "0.50"
        detail_labels = [
            panel.engineering_left_layout.itemAtPosition(row, 0).widget().text()
            for row in range(6)
        ]
        assert detail_labels == [
            "实际位姿",
            "目标位姿",
            "实际速度",
            "目标速度",
            "控制周期",
            "安全门控",
        ]
        assert panel.actual_velocity_value.text() == ("Vx +0.30  Vy +0.40  Vz +0.00")
        right_labels = {
            label.text()
            for label in panel.engineering_increment_panel.findChildren(QLabel)
            if label.objectName() == "mutedLabel"
        }
        assert "飞行姿态" in right_labels
        assert "俯仰/滚转" in right_labels
        assert not {"地面↔机载", "电池", "飞控模式"} & right_labels

        panel.left_sensitivity_combo.setCurrentIndex(0)
        panel.right_sensitivity_combo.setCurrentIndex(2)
        assert panel.vertical_increment_value.text() == "±0.10 m/s"
        assert panel.yaw_increment_value.text() == "±5.7 °/s"
        assert panel.longitudinal_increment_value.text() == "±0.40 m/s"
        assert panel.lateral_increment_value.text() == "±0.40 m/s"

        # 禁用态优先解释安全门控；重新就绪后恢复动作本身的语义提示。
        window._environment_active = False
        window._connection_mode = "none"
        window._refresh()
        assert panel.motion_buttons["left"].toolTip() == (
            window._availability.flight_reason
        )
        assert panel.hover_button.toolTip() == window._availability.flight_reason
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert "增加水平向左的期望速度" in panel.motion_buttons["left"].toolTip()
    finally:
        _close_window(window)


def test_manual_status_chip_thresholds_and_battery_mode_format() -> None:
    """通讯与电池状态块使用可调阈值，仿真/实机不混显单位。"""
    snapshot = replace(
        _operational_snapshot(armed=True),
        battery_valid=True,
        battery_voltage=HARDWARE_BATTERY_GOOD_VOLTAGE,
        battery_percentage=SIMULATION_BATTERY_GOOD_PERCENTAGE,
        status_rate_hz=STATUS_RATE_TARGET_HZ,
    )
    window, ros = _window(snapshot)
    try:
        panel = window.operations
        window._environment_active = True
        window._connection_mode = "hardware"

        hardware_cases = (
            (HARDWARE_BATTERY_GOOD_VOLTAGE, "good"),
            (HARDWARE_BATTERY_GOOD_VOLTAGE - 0.01, "warning"),
            (HARDWARE_BATTERY_WARNING_VOLTAGE, "warning"),
            (HARDWARE_BATTERY_WARNING_VOLTAGE - 0.01, "bad"),
        )
        for voltage, tone in hardware_cases:
            ros.current_snapshot = replace(snapshot, battery_voltage=voltage)
            window._refresh()
            assert panel.battery_status_chip.text() == f"电池 · {voltage:.2f} V"
            assert panel.battery_status_chip.property("tone") == tone
            assert "%" not in panel.battery_status_chip.text()

        window._connection_mode = "simulation"
        simulation_cases = (
            (SIMULATION_BATTERY_GOOD_PERCENTAGE, "good"),
            (SIMULATION_BATTERY_GOOD_PERCENTAGE - 0.01, "warning"),
            (SIMULATION_BATTERY_WARNING_PERCENTAGE, "warning"),
            (SIMULATION_BATTERY_WARNING_PERCENTAGE - 0.01, "bad"),
        )
        for percentage, tone in simulation_cases:
            ros.current_snapshot = replace(
                snapshot,
                battery_percentage=percentage,
            )
            window._refresh()
            assert panel.battery_status_chip.text() == (
                f"电池 · {percentage * 100.0:.0f}%"
            )
            assert panel.battery_status_chip.property("tone") == tone
            assert " V" not in panel.battery_status_chip.text()

        rate_cases = (
            (STATUS_RATE_TARGET_HZ - STATUS_RATE_TOLERANCE_HZ, "good"),
            (STATUS_RATE_TARGET_HZ + STATUS_RATE_TOLERANCE_HZ, "good"),
            (STATUS_RATE_TARGET_HZ - STATUS_RATE_TOLERANCE_HZ - 0.01, "warning"),
            (STATUS_RATE_TARGET_HZ + STATUS_RATE_TOLERANCE_HZ + 0.01, "warning"),
        )
        for rate, tone in rate_cases:
            ros.current_snapshot = replace(snapshot, status_rate_hz=rate)
            window._refresh()
            assert panel.communication_rate_chip.text() == (f"通讯频率 · {rate:.2f} Hz")
            assert panel.communication_rate_chip.property("tone") == tone
            assert "age" not in panel.communication_rate_chip.text()

        # 常见的 9.xx/10.xx 位数变化不得改变组件或后续状态块的位置。
        window.resize(1600, 920)
        stable_widths = []
        following_chip_positions = []
        for rate in (10.00, 9.99, 10.00, 9.98):
            ros.current_snapshot = replace(snapshot, status_rate_hz=rate)
            window._refresh()
            _application().processEvents()
            stable_widths.append(panel.communication_rate_chip.width())
            following_chip_positions.append(
                panel.last_manual_command_chip.mapTo(
                    panel.manual_card, QPoint(0, 0)
                ).x()
            )
        assert len(set(stable_widths)) == 1
        assert len(set(following_chip_positions)) == 1

        # 最小宽度不是固定/最大宽度，异常的更长内容仍能响应式扩展。
        ros.current_snapshot = replace(snapshot, status_rate_hz=100.0)
        window._refresh()
        _application().processEvents()
        assert panel.communication_rate_chip.width() > stable_widths[0]
        assert (
            panel.communication_rate_chip.width()
            >= panel.communication_rate_chip.sizeHint().width()
        )

        # 最小窗口使用较长实机文字时也不裁字。
        window._connection_mode = "hardware"
        ros.current_snapshot = replace(
            snapshot,
            autopilot_mode="STABILIZE",
            battery_voltage=HARDWARE_BATTERY_GOOD_VOLTAGE - 0.05,
        )
        window._refresh()
        panel.last_manual_command_chip.setText("最近指令 · 355.7 s")
        window.resize(1180, 700)
        _application().processEvents()
        chips = (
            panel.control_authority_chip,
            panel.autopilot_mode_chip,
            panel.battery_status_chip,
            panel.communication_rate_chip,
            panel.last_manual_command_chip,
        )
        assert panel._manual_status_wrapped is True
        chip_bounds = []
        for chip in chips:
            origin = chip.mapTo(panel.manual_card, QPoint(0, 0))
            chip_bounds.append((origin.x(), origin.x() + chip.width()))
            assert chip.width() >= chip.sizeHint().width()
        assert all(
            left[1] <= right[0] for left, right in zip(chip_bounds, chip_bounds[1:])
        )
        assert chip_bounds[-1][1] <= panel.manual_card.width()
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


def test_operations_layout_and_us_mode_manual_controls() -> None:
    """双卡标题对齐，手动区使用带反馈和摘要的美国手双摇杆。"""
    window, _ros = _window(_operational_snapshot(armed=True))
    try:
        window.resize(1600, 920)
        _application().processEvents()
        panel = window.operations

        # 上排两卡片同高起点，手动操纵卡片位于它们下方。
        assert panel.environment_card.geometry().x() < panel.flight_card.geometry().x()
        assert panel.environment_card.geometry().y() == panel.flight_card.geometry().y()
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
            assert all(popup.actionGeometry(action).height() > 0 for action in actions)
            assert popup.actionGeometry(actions[-1]).bottom() < popup.height()
            assert (
                popup.frameGeometry().top()
                >= combo.mapToGlobal(QPoint(0, combo.height())).y()
            )
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
            left_origin = panel.left_stick_group.mapTo(panel.manual_card, QPoint(0, 0))
            right_origin = panel.right_stick_group.mapTo(
                panel.manual_card, QPoint(0, 0)
            )
            return (
                left_origin.x(),
                panel.manual_card.width()
                - right_origin.x()
                - panel.right_stick_group.width(),
                right_origin.x() - left_origin.x() - panel.left_stick_group.width(),
            )

        left_origin = panel.left_stick_group.mapTo(panel.manual_card, QPoint(0, 0))
        assert hover_origin.y() > (left_origin.y() + panel.left_stick_group.height())
        assert (
            abs(
                hover_origin.x()
                + panel.hover_button.width() / 2
                - panel.manual_card.width() / 2
            )
            < 20
        )
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
        vx=0.1,
        vy=0.2,
        vz=0.2,
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

        assert panel.coordinate_mode() == "enu"
        assert panel.coordinate_mode_combo.currentText() == "本地 ENU"
        assert panel.altitude_summary_value.text() == "+3.25"
        assert panel.actual_speed_summary_value.text() == "0.30"
        assert panel.horizontal_summary_value.text() == "0.50"
        assert panel.vertical_summary_value.text() == "-0.10"
        assert panel.yaw_rate_summary_value.text() == "+15.0"

        # 默认 ENU 不旋转，右摇杆低灵敏度的 I 直接增加本地 +X。
        panel.right_sensitivity_combo.setCurrentIndex(0)
        panel.motion_buttons["forward"].click()
        command = ros.calls[-1][1]
        assert ros.calls[-1][0] == "motion"
        assert abs(command[0] - VELOCITY_SCALE * 0.5) < 1e-9
        assert abs(command[1]) < 1e-9

        # 切换机体坐标后，机头朝 +Y 时 I 转换为 ENU +Y。
        panel.coordinate_mode_combo.setCurrentIndex(0)
        coordinate_event = window.event_log.snapshot()[-1]
        assert coordinate_event.level is LogLevel.INFO
        assert coordinate_event.source == "operator"
        assert "坐标系切换为「机体坐标」" in coordinate_event.message
        assert "按最新机头航向旋转" in coordinate_event.message
        panel.motion_buttons["forward"].click()
        command = ros.calls[-1][1]
        assert abs(command[0]) < 1e-9
        assert abs(command[1] - VELOCITY_SCALE * 0.5) < 1e-9

        # 切回 ENU 后恢复固定轴语义。
        panel.coordinate_mode_combo.setCurrentIndex(1)
        coordinate_event = window.event_log.snapshot()[-1]
        assert "坐标系切换为「本地 ENU」" in coordinate_event.message
        assert "固定 X/Y 轴" in coordinate_event.message

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


def test_reboot_fcu_confirmation_and_ground_gates() -> None:
    """Only fresh, unarmed hardware idle permits reboot; cancel/state race sends nothing."""
    snapshot = replace(_operational_snapshot(armed=False), on_ground=True)
    window, ros = _window(snapshot)
    ros.request_reboot_fcu = lambda: ros.calls.append(("reboot_fcu", None)) or 99
    try:
        window._environment_active = True
        window._connection_mode = "hardware"
        window._refresh()
        assert window.reboot_fcu_button.isEnabled()
        with patch.object(window, "_confirm_action", return_value=False):
            window._reboot_fcu()
        assert not ros.calls
        for changes in (
            {"armed": True},
            {"on_ground": False},
            {"reboot_in_progress": True},
            {"connected": False},
            {"controller_active": True},
            {"active_mode": FlightMode.WAYPOINT},
        ):
            ros.current_snapshot = replace(snapshot, **changes)
            window._refresh()
            assert not window.reboot_fcu_button.isEnabled()
        ros.current_snapshot = snapshot
        window._connection_mode = "simulation"
        window._refresh()
        assert not window.reboot_fcu_button.isEnabled()
        window._connection_mode = "hardware"

        def change_state(*args, **kwargs):
            ros.current_snapshot = replace(snapshot, armed=True)
            return True

        with patch.object(window, "_confirm_action", side_effect=change_state):
            window._reboot_fcu()
        assert not ros.calls
        ros.current_snapshot = snapshot
        with patch.object(window, "_confirm_action", return_value=True):
            window._reboot_fcu()
        assert ros.calls == [("reboot_fcu", None)]
        assert not window.reboot_fcu_button.isEnabled()
    finally:
        _close_window(window)
