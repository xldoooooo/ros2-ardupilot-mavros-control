"""地面站高层协议客户端的排队、状态映射与无服务失败测试。"""

import time
from types import SimpleNamespace

from ground_station_core.config import INTERFACE_VERSION
from ground_station_core.models import FlightMode, VehicleSnapshot
from ground_station_core.ros_controller import (
    GroundStationRosController,
    _VehicleStateStore,
)


def test_controller_rejects_takeoff_without_onboard_and_stops() -> None:
    """机载服务不存在时必须明确失败，不能退回地面站本地控制。"""
    controller = GroundStationRosController(source_id="pytest-no-onboard")
    controller.start()
    try:
        assert controller.ready
        ticket = controller.request_takeoff(0.3)
        result = controller.wait_for_result(ticket, timeout=3.0)
        assert result is not None
        assert not result.success
        assert "机载控制服务不可用" in result.message
    finally:
        controller.stop()

    assert not controller.ready
    assert controller.active_mode is FlightMode.IDLE


def test_all_flight_inputs_share_monotonic_sequence() -> None:
    """服务命令与方向意图必须共享同一序号，供机载端拒绝旧输入。"""
    controller = GroundStationRosController(source_id="pytest-order")
    takeoff = controller.request_takeoff(0.3)
    motion = controller.adjust_velocity(0.2, 0.0, 0.0, 0.0)
    waypoint = controller.request_waypoints(((1.0, 0.0, 1.0, 0.0),), strategy=1)

    queued = [controller._command_queue.get_nowait() for _ in range(3)]
    assert [item.ticket for item in queued] == [takeoff, motion, waypoint]
    assert [item.name for item in queued] == ["takeoff", "motion", "waypoints"]
    assert queued[2].argument == {
        "waypoints": ((1.0, 0.0, 1.0, 0.0),),
        "strategy": 1,
    }
    assert takeoff < motion < waypoint


def test_status_store_maps_remote_mode_and_lease_owner() -> None:
    """GUI 展示必须来自机载聚合状态，而不是本地猜测。"""
    store = _VehicleStateStore("gcs-test")
    vector = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    message = SimpleNamespace(
        interface_version=INTERFACE_VERSION,
        fcu_connected=True,
        armed=True,
        autopilot_mode="GUIDED",
        local_position_valid=True,
        position=vector,
        velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        yaw=0.4,
        control_mode=4,
        controller_active=True,
        target_position=vector,
        target_velocity=SimpleNamespace(x=0.5, y=0.0, z=0.0),
        target_yaw=0.6,
        target_yaw_rate=0.1,
        lease_owner="gcs-test",
        lease_active=True,
        waypoint_index=2,
        waypoint_count=3,
        message_rates_configured=True,
        thrust_mode_verified=True,
        hover_throttle=0.39,
        setpoint_conflict=False,
        failsafe_reason="",
        status_message="执行中",
        control_rate_hz=99.8,
        max_jitter_ms=1.2,
        deadline_miss_count=2,
    )
    store.update(message)
    snapshot = store.snapshot()

    assert snapshot.onboard_available
    assert snapshot.active_mode is FlightMode.WAYPOINT
    assert snapshot.control_authority
    assert snapshot.waypoint_index == 2
    assert snapshot.control_rate_hz == 99.8
    assert snapshot.hover_throttle == 0.39


def test_previous_interface_version_is_rejected_before_command_transport() -> None:
    """1.0 机载端不得在 ExecuteWaypoints 结构升级后被误判为兼容。"""
    controller = GroundStationRosController(source_id="gcs-version-gate")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version="1.0",
        control_authority=True,
        lease_owner="gcs-version-gate",
    )
    controller._state._last_status_time = time.monotonic()
    ticket = controller.request_takeoff(0.3)

    # 版本检查发生在读取任何 ROS 实体前，空字典可证明请求没有进入传输层。
    controller._process_one_command({}, {})
    result = controller.wait_for_result(ticket, timeout=0.1)

    assert INTERFACE_VERSION == "2.0"
    assert result is not None
    assert not result.success
    assert "接口版本不兼容" in result.message
