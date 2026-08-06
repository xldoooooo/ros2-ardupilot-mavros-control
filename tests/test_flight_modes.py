"""飞行模式仲裁、覆盖清理和航点完成行为的单元测试。"""

from ground_station_core.config import HOVER_PARAM_DEFAULTS
from ground_station_core.dob_controller import DobGains
from ground_station_core.flight_modes.keyboard_control import KeyboardControlMode
from ground_station_core.flight_modes.waypoint_flight import WaypointFlightMode
from ground_station_core.mode_manager import FlightModeManager
from ground_station_core.models import FlightMode, VehicleSnapshot


def _gains() -> DobGains:
    """返回项目默认 DOB 增益。"""
    return DobGains.from_mapping(HOVER_PARAM_DEFAULTS)


def test_latest_button_mode_overrides_previous_mode() -> None:
    """航点与键盘按键按最后一次输入互斥接管，并清理旧状态。"""
    events: list[tuple[int, bool, str, bool]] = []
    manager = FlightModeManager()
    keyboard = KeyboardControlMode(manager, _gains())
    waypoint = WaypointFlightMode(manager, _gains(), lambda *args: events.append(args))

    keyboard.adjust(0.2, 0.0, 0.0, 0.0)
    assert manager.current is FlightMode.KEYBOARD
    assert keyboard.velocity == (0.2, 0.0, 0.0, 0.0)

    waypoint.start(7, [(1.0, 2.0, 3.0, 0.0)])
    assert manager.current is FlightMode.WAYPOINT
    assert keyboard.velocity == (0.0, 0.0, 0.0, 0.0)

    keyboard.adjust(0.0, -0.2, 0.0, 0.0)
    assert manager.current is FlightMode.KEYBOARD
    assert events == [(7, False, "航点任务已被其他飞行模式覆盖", True)]


def test_hover_captures_position_and_zeroes_velocity() -> None:
    """悬停仍属于键盘模式，但会清零累加速度。"""
    manager = FlightModeManager()
    keyboard = KeyboardControlMode(manager, _gains())
    keyboard.adjust(0.2, 0.2, -0.2, 0.2)
    keyboard.hover(VehicleSnapshot(x=1.0, y=2.0, z=3.0, yaw=0.4))

    assert manager.current is FlightMode.KEYBOARD
    assert keyboard.hovering
    assert keyboard.velocity == (0.0, 0.0, 0.0, 0.0)


def test_waypoint_reports_completion_once() -> None:
    """到达最后航点后只报告一次完成，并继续保留终点悬停状态。"""
    events: list[tuple[int, bool, str, bool]] = []
    manager = FlightModeManager()
    mode = WaypointFlightMode(
        manager,
        _gains(),
        lambda *args: events.append(args),
        hold_time=0.0,
    )
    mode._dob.publish = lambda *args, **kwargs: None
    mode.start(9, [(0.0, 0.0, 1.0, 0.0)])
    state = VehicleSnapshot(armed=True, autopilot_mode="GUIDED", z=1.0)

    mode.publish(None, None, None, state)
    mode.publish(None, None, None, state)
    mode.publish(None, None, None, state)

    assert len(events) == 1
    assert events[0][0] == 9
    assert events[0][1] is True
    assert events[0][3] is True
