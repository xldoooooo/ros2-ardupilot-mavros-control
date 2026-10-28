"""落地低电压起飞门控；仅使用 Qt/ROS 替身，不连接实机。"""

from __future__ import annotations

from dataclasses import replace

import pytest

from ground_station_core.config import HARDWARE_LOW_VOLTAGE
from ground_station_core.qt_ui.state import takeoff_voltage_reason
from ground_station_core.upstream.mapping import parse_command
from tests.support.qt import _close_window, _operational_snapshot, _window


def test_low_voltage_blocks_button_manual_and_upstream_takeoff() -> None:
    """无 WebSocket 也禁用按钮，直接调用及 01/03 均拒绝，电压恢复则解禁。"""
    snapshot = replace(
        _operational_snapshot(armed=False), on_ground=True,
        battery_valid=True, battery_voltage=HARDWARE_LOW_VOLTAGE - 0.1,
    )
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "hardware"
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "低电压测试")
        window._refresh()
        assert not window._upstream.snapshot().connected
        assert not window.operations.takeoff_button.isEnabled()
        assert "低于阈值" in window.operations.takeoff_button.toolTip()
        notices = len(window.event_log.snapshot())
        window._refresh()
        assert len(window.event_log.snapshot()) == notices

        assert window._takeoff() is None
        for number in ("01", "03"):
            window._handle_upstream_command(parse_command(
                {"clientNo": "UAV01001", "commandNo": number}, "UAV01001",
            ))
            assert any(
                f"拒绝上位机命令 {number}" in event.message
                and "低于阈值" in event.message
                for event in window.event_log.snapshot()
            )
        assert not ros.calls
        assert not window._pending_commands
        assert window._upstream_sequence is None

        ros.current_snapshot = replace(snapshot, battery_voltage=HARDWARE_LOW_VOLTAGE)
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()
        window._confirm_action = lambda *_args, **_kwargs: True
        assert window._takeoff() is not None
        assert [name for name, _args in ros.calls] == ["takeoff"]
    finally:
        _close_window(window)


@pytest.mark.parametrize("source", ["manual", "01", "03"])
def test_takeoff_rechecks_voltage_after_confirmation(source) -> None:
    """确认或倒计时结束时电压跌破阈值，不入队且不残留巡检组合。"""
    snapshot = replace(
        _operational_snapshot(armed=False), on_ground=True,
        battery_valid=True, battery_voltage=HARDWARE_LOW_VOLTAGE,
    )
    window, ros = _window(snapshot)

    def confirm_with_voltage_drop(*_args, **_kwargs):
        """模拟模态确认期间收到新的低电压遥测。"""
        ros.current_snapshot = replace(snapshot, battery_voltage=HARDWARE_LOW_VOLTAGE - 0.1)
        return True

    try:
        window._environment_active = True
        window._connection_mode = "hardware"
        window._confirm_action = confirm_with_voltage_drop
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "确认期间低电压")
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()
        if source == "manual":
            assert window._takeoff() is None
        else:
            window._handle_upstream_command(parse_command(
                {"clientNo": "UAV01001", "commandNo": source}, "UAV01001",
            ))
        assert not ros.calls
        assert not window._pending_commands
        assert window._upstream_sequence is None
        assert not window.operations.takeoff_button.isEnabled()
        assert any(
            "拒绝起飞" in event.message and "低于阈值" in event.message
            for event in window.event_log.snapshot()
        )
    finally:
        _close_window(window)


def test_airborne_low_voltage_keeps_existing_flight_controls() -> None:
    """已离地低电压不触发飞行指令、不锁控制，落地后才禁起飞。"""
    snapshot = replace(
        _operational_snapshot(armed=True), on_ground=False,
        battery_valid=True, battery_voltage=HARDWARE_LOW_VOLTAGE - 0.1,
    )
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "hardware"
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "空中低电压")
        window._refresh()
        assert window._availability.motion
        assert window._availability.hover
        assert window._availability.waypoint_send
        assert window._availability.land
        assert not ros.calls
        assert not takeoff_voltage_reason(snapshot, "hardware")
        ros.current_snapshot = replace(
            snapshot, battery_voltage=HARDWARE_LOW_VOLTAGE + 0.1,
        )
        before = window._availability
        window._refresh()
        assert window._availability == before

        ros.current_snapshot = replace(
            _operational_snapshot(armed=False), on_ground=True,
            battery_valid=True, battery_voltage=snapshot.battery_voltage,
        )
        window._refresh()
        assert not window.operations.takeoff_button.isEnabled()
    finally:
        _close_window(window)


@pytest.mark.parametrize("voltage", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_voltage_does_not_add_a_takeoff_restriction(voltage) -> None:
    """无有效电压不新增拒绝条件；仿真仍使用原百分比逻辑。"""
    snapshot = replace(
        _operational_snapshot(armed=False), on_ground=True,
        battery_valid=True, battery_voltage=voltage,
    )
    assert not takeoff_voltage_reason(snapshot, "hardware")
    low = replace(snapshot, battery_voltage=HARDWARE_LOW_VOLTAGE - 0.1)
    assert not takeoff_voltage_reason(replace(low, battery_valid=False), "hardware")
    assert not takeoff_voltage_reason(low, "simulation")
