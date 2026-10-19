"""Verify map task coordinates and ground-only switching without a vehicle."""

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from ground_station_core.event_log import EventLog
from ground_station_core.models import VehicleSnapshot
from ground_station_core.ros_controller import GroundStationRosController, task_waypoint_to_local
from ground_station_core.upstream.mapping import UpstreamProtocolError, parse_command
from ground_station_core.upstream.status_projector import UpstreamStatusProjector
from tests.support.qt import _close_window, _operational_snapshot, _window


def test_map_preview_applies_translation_rotation_and_heading() -> None:
    """Odin map numbers must be transformed before using the MAVROS map label."""
    snapshot = VehicleSnapshot(
        navigation_frame="map", task_to_local=(4, -2, 3, 0, 0, math.sqrt(0.5), math.sqrt(0.5)),
    )
    assert task_waypoint_to_local((2, 0, -1, 0), snapshot) == pytest.approx((4, 0, 2, math.pi / 2))
    assert task_waypoint_to_local((2, 0, -1, 0), VehicleSnapshot()) == (2, 0, -1, 0)
    with pytest.raises(ValueError, match="定位"):
        task_waypoint_to_local((2, 0, -1, 0), replace(snapshot, navigation_ready=False))


def test_map_websocket_accepts_negative_z_and_reports_task_position() -> None:
    """Map Z may be negative, while hangar membership remains FCU-local."""
    payload = {
        "clientNo": "test", "commandNo": "02", "taskPoints": [{
            "index": 1, "x": 3, "y": 4, "z": -2,
            "forwardAngle": 90, "cameraAngle": 0, "photoNo": "one",
        }],
    }
    assert parse_command(payload, "test", "map").waypoints[0] == pytest.approx((3, 4, -2, math.pi/2))
    with pytest.raises(UpstreamProtocolError, match="z"):
        parse_command(payload, "test", "odom")
    sent = []
    projector = UpstreamStatusProjector(lambda: "test", lambda message: sent.append(message) or True, EventLog())
    snapshot = VehicleSnapshot(
        local_position_valid=True, navigation_frame="map", task_position=(3, 4, -2),
    )
    assert projector.is_in_hangar(snapshot)
    projector.observe_vehicle(snapshot, "hardware", now=5)
    assert any(item.get("data") == {"X": 3, "Y": 4, "Z": -2} for item in sent)


def test_navigation_switch_preserves_waypoints_and_follows_real_status() -> None:
    """Two successive mode changes wait for actual frame and lease recovery."""
    window, ros = _window(replace(_operational_snapshot(armed=False), on_ground=True))
    ros.request_navigation_mode = lambda mode: ros._record("navigation_mode", mode)
    window._confirm_action = lambda *args, **kwargs: True
    try:
        window._environment_active = True
        window._connection_mode = "hardware"
        points = ((1.0, 0.0, 1.0, 0.0),)
        window.waypoints.replace_waypoints(points, "test")
        window._refresh()
        assert window.navigation_switch.isEnabled()
        for expected_frame in ("map", "odom"):
            window.navigation_switch.click()
            assert ros.calls[-1] == ("navigation_mode", expected_frame)
            assert not window.navigation_switch.isEnabled()
            ros.current_snapshot = replace(ros.current_snapshot, navigation_frame=expected_frame)
            window._navigation_switch_started -= 2
            window._refresh()
            window._refresh()
            assert window._navigation_switch_target is None
            assert window.navigation_switch.isChecked() == (expected_frame == "map")
            assert window.correction_panel_button.isEnabled() == (expected_frame == "odom")
            assert window.waypoints.waypoints == points
        ros.current_snapshot = replace(ros.current_snapshot, navigation_frame="map", navigation_ready=False,
                                       navigation_detail="等待重定位")
        window._refresh()
        assert not window.operations.takeoff_button.isEnabled()
        assert window.operations.altitude_summary_value.text() == "--"
        assert "等待重定位" in window.activity_banner.message_label.text()
    finally:
        _close_window(window)


def test_map_pose_capture_and_return_keep_their_respective_frames() -> None:
    """Captured points use map pose; an explicit home command still uses FCU local."""
    snapshot = replace(_operational_snapshot(armed=True), navigation_frame="map",
                       task_position=(10.0, -4.0, -2.0), task_yaw=1.2)
    window, ros = _window(snapshot)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints._add_current_waypoint()
        assert window.waypoints.waypoints == ((10.0, -4.0, -2.0, 1.2),)
        calls = []
        ros.request_waypoints = lambda *args, **kwargs: calls.append((args, kwargs)) or 77
        assert window._start_return_sequence("return", snapshot)
        assert calls[-1][0][0][0][:3] == (0.0, 0.0, window.operations.takeoff_altitude())
        assert calls[-1][1] == {"local_coordinates": True}
        assert window.waypoints.waypoints == ((10.0, -4.0, -2.0, 1.2),)
    finally:
        _close_window(window)


def test_navigation_restart_acceptance_has_a_service_result() -> None:
    """Restart acceptance completes locally even if the service then exits."""
    controller = GroundStationRosController(source_id="test")
    ticket = controller.request_navigation_mode("map")
    command = controller._command_queue.get_nowait()
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(accepted=True, message="重启中"))
    pending = {ticket: (future, command, 100.0)}
    controller._poll_pending_services(pending)
    result = controller.results_after(0)[-1]
    assert result.success and result.final and result.command == "navigation_mode"
    assert not pending
