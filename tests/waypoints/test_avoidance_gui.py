"""避障 UI 的能力门控、起飞配置与上位机路由回归；不连接实机。"""

from dataclasses import replace
from types import SimpleNamespace
import time

import pytest

from ground_station_core.config import INTERFACE_VERSION
from ground_station_core.models import FlightMode, VehicleSnapshot
from ground_station_core.ros_controller import GroundStationRosController
from ground_station_core.upstream.mapping import parse_command
from tests.support.qt import _close_window, _operational_snapshot, _window


def test_avoidance_readiness_gates_gui_and_upstream_without_straight_fallback() -> None:
    """相同不可用规划能力下，直线正常起飞；避障 GUI/上位机均明确拒绝。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()
        window.waypoints.strategy_combo.setCurrentIndex(1)
        assert not window.operations.takeoff_button.isEnabled()
        assert "机载避障尚未就绪" in window._availability.flight_reason
        assert not window.waypoints.reference_combo.isEnabled()
        assert window.waypoints.reference_combo.currentText() == "规划器带时间轨迹"
        assert window.waypoints.selected_reference_generator() == 2
        assert window.waypoints.selected_tracking_controller() == 1
        assert not window.waypoints.tracking_combo.isEnabled()
        assert window._takeoff() is None
        window._handle_upstream_command(parse_command(
            {"clientNo": "UAV01001", "commandNo": "01"}, "UAV01001"
        ))
        assert not ros.calls
        window.waypoints.add_button.click()
        window._handle_upstream_command(parse_command(
            {"clientNo": "UAV01001", "commandNo": "03"}, "UAV01001"
        ))
        assert window._upstream_sequence is None
        assert not ros.calls
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        assert not window.waypoints.send_button.isEnabled()
        assert window._send_waypoints(window.waypoints.waypoints) is None
        assert not ros.calls
        # 飞行配置仍锁定；避障暂失能力不影响人工悬停与降落。
        assert not window.waypoints.strategy_combo.isEnabled()
        assert window.operations.hover_button.isEnabled()
        assert window.operations.land_button.isEnabled()
    finally:
        _close_window(window)


def test_ready_avoidance_takeoff_locks_configuration_and_displays_wait_state() -> None:
    """自主避障实际传输 1/2/1，机载 WAITING 倒计时在航点卡可见。"""
    snapshot = replace(_operational_snapshot(armed=False), avoidance_ready=True)
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.strategy_combo.setCurrentIndex(1)
        assert window.operations.takeoff_button.isEnabled()
        window._takeoff()
        assert ros.calls[-1] == ("takeoff", (0.3, 1, 2, 1))
        ros.current_snapshot = replace(
            snapshot, armed=True, active_mode=FlightMode.WAYPOINT,
            flight_strategy=1, avoidance_state=1,
            avoidance_wait_remaining_seconds=8.2, avoidance_detail="规划无路",
        )
        window._refresh()
        assert not window.waypoints.strategy_combo.isEnabled()
        assert window.waypoints.status_label.isVisible()
        assert "8.2 s" in window.waypoints.status_label.text()
        assert "规划无路" in window.waypoints.status_label.text()
    finally:
        _close_window(window)


def test_hover_on_obstacle_keeps_smooth_generator_and_forces_trajectory_tracking() -> None:
    """遇障悬停允许原有连续生成器选择，禁用位置阶跃和位置跟踪。"""
    window, ros = _window(replace(_operational_snapshot(armed=False), avoidance_ready=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.reference_combo.setCurrentIndex(0)
        window.waypoints.tracking_combo.setCurrentIndex(0)
        window.waypoints.strategy_combo.setCurrentIndex(2)
        assert window.waypoints.selected_reference_generator() == 2
        assert not window.waypoints.reference_combo.model().item(0).isEnabled()
        assert window.waypoints.reference_combo.isEnabled()
        window.waypoints.reference_combo.setCurrentIndex(3)
        window._takeoff()
        assert ros.calls[-1] == ("takeoff", (0.3, 2, 3, 1))
        window.waypoints.strategy_combo.setCurrentIndex(0)
        assert window.waypoints.reference_combo.model().item(0).isEnabled()
        assert window.waypoints.tracking_combo.isEnabled()
    finally:
        _close_window(window)


def test_takeoff_transport_preserves_all_lock_fields() -> None:
    """锁定字段必须真实写入同一次 FlightCommand 服务请求。"""
    class Request:
        COMMAND_TAKEOFF = 1
        COMMAND_LAND = 2
        COMMAND_HOVER = 3
        COMMAND_CANCEL = 4
        COMMAND_CONFIGURE_RATES = 5
        COMMAND_CLEAR_ABNORMAL = 6
        COMMAND_REBOOT_FCU = 7

    requests = []
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or SimpleNamespace(done=lambda: False),
    )
    controller = GroundStationRosController(source_id="avoidance-transport")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True, interface_version=INTERFACE_VERSION,
        control_authority=True, lease_owner="avoidance-transport",
    )
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    ticket = controller.request_takeoff(0.7, 2, 3, 1)
    pending = {}
    controller._process_one_command({
        "node": SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: object())
        )),
        "clients": {"flight": client},
        "FlightCommand": SimpleNamespace(Request=Request),
    }, pending)
    assert ticket in pending
    request = requests[0]
    assert (request.command, request.value) == (1, 0.7)
    assert (request.flight_strategy, request.reference_generator, request.tracking_controller) == (2, 3, 1)


def test_planner_reference_is_readback_only_and_unknown_strategy_does_not_fallback() -> None:
    """非法输入不得被规范为可执行的直线基线。"""
    controller = GroundStationRosController(source_id="avoidance-invalid")
    with pytest.raises(ValueError):
        controller.request_waypoints(((1, 0, 1, 0),), reference_generator=4)
    with pytest.raises(ValueError):
        controller.request_waypoints(((1, 0, 1, 0),), strategy=99)
    with pytest.raises(ValueError):
        controller.request_takeoff(0.3, reference_generator=4)
    assert controller._command_queue.empty()


@pytest.mark.parametrize('strategy,generator', [(0, 3), (1, 2), (2, 1)])
def test_reconnected_gui_restores_locked_configuration_from_hover_status(strategy, generator):
    """重连的实际悬停参考为 0，也必须恢复起飞时锁定的三项配置。"""
    snapshot = replace(
        _operational_snapshot(armed=True), avoidance_ready=True,
        waypoint_configuration_locked=True, flight_strategy=strategy,
        locked_reference_generator=generator, locked_tracking_controller=1,
        active_reference_generator=0,
    )
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.waypoints.selected_strategy() == strategy
        assert window.waypoints.selected_reference_generator() == generator
        assert window.waypoints.selected_tracking_controller() == 1
        assert not window.waypoints.strategy_combo.isEnabled()
        assert not window.waypoints.reference_combo.isEnabled()
        assert not window.waypoints.tracking_combo.isEnabled()
        window.waypoints.add_button.click()
        window._refresh()
        window._send_waypoints(window.waypoints.waypoints)
        assert ros.calls[-1][0] == "waypoints"
        assert ros.calls[-1][1][1:4] == (strategy, generator, 1)
    finally:
        _close_window(window)
