"""Exercise real map execution using localhost DDS and fictitious MAVROS services only."""

import json
import math
import subprocess

import pytest

from test_avoidance_executor import avoidance_node  # Reuse the isolated FCU/planner double.


@pytest.fixture
def navigation_process_settings(tmp_path, monkeypatch):
    """Shorten only navigation waits and install a harmless switch marker script."""
    original = subprocess.Popen
    marker = tmp_path / "switch-called"
    failure_marker = tmp_path / "switch-failure.result"
    script = tmp_path / "fake-navigation-switch.sh"
    script.write_text(f"#!/bin/bash\ntouch '{marker}'\n")

    def launch(args, **kwargs):
        """Append test parameters after the common fixture's normal arguments."""
        assert kwargs["env"]["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"
        assert int(kwargs["env"]["ROS_DOMAIN_ID"]) != 0
        kwargs["env"] = dict(kwargs["env"], NAVIGATION_SWITCH_RESULT_FILE=str(failure_marker))
        values = {
            "navigation_status_timeout_seconds": .8,
            "navigation_tf_timeout_seconds": .6,
            "navigation_hold_timeout_seconds": .6,
            "navigation_retarget_interval_seconds": .1,
            "navigation_retarget_blend_seconds": .2,
            "navigation_switch_script": str(script),
        }
        args = list(args)
        for name, value in values.items():
            args += ["-p", f"{name}:={value}"]
        return original(args, **kwargs)

    monkeypatch.setattr(subprocess, "Popen", launch)
    return marker, failure_marker


@pytest.fixture
def navigation_node(navigation_process_settings, avoidance_node):
    """Add real driver status/TF/extnav messages after safely disarming the double."""
    from correction_interfaces.msg import ExtnavCorrectionStatus
    from geometry_msgs.msg import TransformStamped
    from guided_interfaces.srv import ExecuteWaypoints, SetNavigationMode
    from mavros_msgs.srv import CommandBool, SetMode
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    h = avoidance_node
    h.state.armed = False
    h.until(lambda: not h.status[-1].armed)
    h.marker, h.failure_marker = navigation_process_settings
    h.nav_enabled, h.tf_enabled, h.localized = False, True, True
    h.driver_session = "navigation-test-session"
    h.translation = [-1996.06, -1001.97, -6.85]
    h.mode_calls, h.arm_calls = [], []
    status_pub = h.node.create_publisher(
        String, "/odin1/navigation_status",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tf_pub = h.node.create_publisher(TFMessage, "/tf", 10)
    h.display_transforms = []
    h.node.create_subscription(
        TFMessage, "/tf_static", h.display_transforms.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    extnav_pub = h.node.create_publisher(ExtnavCorrectionStatus, "/extnav/correction_status", 10)
    h.navigation = h.node.create_client(
        SetNavigationMode, h.mavros.replace("_mavros", "_onboard") + "/set_navigation_mode")

    def set_mode(request, response):
        """Record requested modes without changing simulated aircraft state."""
        h.mode_calls.append(request.custom_mode)
        response.mode_sent = True
        return response

    def arm(request, response):
        """Record attempted arming; tests never need simulated arming service success."""
        h.arm_calls.append(request.value)
        response.success = False
        return response

    h.node.create_service(SetMode, h.mavros + "/set_mode", set_mode)
    h.node.create_service(CommandBool, h.mavros + "/cmd/arming", arm)
    original_emit_result = h.emit_result

    def emit_result(request):
        """Extend the shared planner double to reach nonzero Y/Z map-derived goals."""
        if request.mode != request.PLAN or h.policy != "clear":
            original_emit_result(request)
            return
        from guided_interfaces.msg import PolynomialSegment
        start = (h.position_x, 0., h.height)
        goal = (request.goal.x, request.goal.y, request.goal.z)
        delta = tuple(end - begin for begin, end in zip(start, goal))
        duration = 2. * math.sqrt(sum(value * value for value in delta)) / .05
        coefficients = [[begin, 2. * offset / duration, -offset / duration ** 2, 0.]
                        for begin, offset in zip(start, delta)]
        result = h.result(request)
        result.segments = [PolynomialSegment(duration=duration, x=coefficients[0],
                                             y=coefficients[1], z=coefficients[2])]
        h.reply.publish(result)

    h.emit_result = emit_result

    def tick():
        """Publish native odom<-map direction and authoritative extnav origin offset."""
        if not h.nav_enabled:
            return
        status_pub.publish(String(data=json.dumps({
            "mode": "map", "map_path": "/synthetic/navigation-map",
            "session_id": h.driver_session, "connected": True,
            "localized": h.localized, "tf_age_seconds": 0.0,
        })))
        extnav_pub.publish(ExtnavCorrectionStatus(
            interface_version="2.0", service_available=True, odin_available=True,
            odin_session_id="extnav-test-session", final_sample_available=True,
            final_sample_odin_session_id="extnav-test-session",
            final_sample_reference_mode="local_identity_origin",
            lever_arm_x_m=.06, lever_arm_y_m=-.03, lever_arm_z_m=.05))
        if h.tf_enabled:
            transform = TransformStamped(child_frame_id="map")
            transform.header.frame_id = "odom"
            transform.header.stamp = h.node.get_clock().now().to_msg()
            for axis, value in zip("xyz", h.translation):
                setattr(transform.transform.translation, axis, value)
            transform.transform.rotation.z = math.sqrt(.5)
            transform.transform.rotation.w = math.sqrt(.5)
            tf_pub.publish(TFMessage(transforms=[transform]))

    timer = h.node.create_timer(.04, tick)

    def map_ready():
        """Select map while disarmed, then wait for the actual executor's ready state."""
        h.nav_enabled = True
        h.until(lambda: h.status[-1].navigation_frame == "map"
                and h.status[-1].navigation_ready)
        h.until(lambda: any(transform.header.frame_id == "odom"
                            and transform.child_frame_id == "fcu_local"
                            for message in h.display_transforms for transform in message.transforms))
        transform = next(transform for message in reversed(h.display_transforms)
                         for transform in message.transforms if transform.child_frame_id == "fcu_local")
        assert (transform.transform.translation.x, transform.transform.translation.y,
                transform.transform.translation.z, transform.transform.rotation.x,
                transform.transform.rotation.y, transform.transform.rotation.z,
                transform.transform.rotation.w) == pytest.approx((-.06, .03, -.05, 0., 0., 0., 1.))

    def execute(strategy=0, local=False):
        """Submit a far-origin map waypoint whose local endpoint is (4, 0, 1.2)."""
        from guided_interfaces.msg import Waypoint
        request = h.request(ExecuteWaypoints)
        request.ttl_ms = 3000
        request.flight_strategy = strategy
        request.reference_generator = 0 if strategy == 0 else 2
        request.tracking_controller = 1
        request.coordinate_frame = "map" if h.nav_enabled else "odom"
        request.local_coordinates = local
        waypoint = Waypoint(yaw=-.5)
        values = (4., 0., 1.2) if local or not h.nav_enabled else (1002., -2000., 8.)
        waypoint.position.x, waypoint.position.y, waypoint.position.z = values
        request.waypoints = [waypoint]
        return h.call(h.waypoints, request)

    h.map_ready, h.execute_navigation = map_ready, execute
    h.policy = "clear"
    try:
        yield h
    finally:
        h.node.destroy_timer(timer)


@pytest.mark.parametrize("strategy", [0, 1, 2])
def test_map_endpoint_and_ordinary_relocalization(navigation_node, strategy):
    """All strategies use rotated map XYZ; small corrections preserve readiness and goals."""
    from guided_interfaces.msg import ControlStatus

    h = navigation_node
    h.map_ready()
    h.state.armed = True
    h.until(lambda: h.status[-1].armed)
    assert h.execute_navigation(strategy).accepted
    if strategy == 0:
        h.until(lambda: h.status[-1].target_position.x > 3.99)
        before = h.status[-1]
        assert (before.target_position.x, before.target_position.y,
                before.target_position.z) == pytest.approx((4., 0., 1.2))
        h.until(lambda: abs(h.status[-1].target_yaw - (math.pi / 2 - .5)) < 1.e-6)
    else:
        h.until(lambda: any(request.mode != request.CANCEL for _, request in h.requests))
        old = next(request for _, request in h.requests if request.mode != request.CANCEL)
        assert (old.goal.x, old.goal.y, old.goal.z) == pytest.approx((4., 0., 1.2))
        h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
                and h.status[-1].control_mode == ControlStatus.MODE_WAYPOINT)
        if strategy == 1:
            h.until(lambda: any(path.poses for path in h.paths))
            assert next(path for path in reversed(h.paths) if path.poses).header.frame_id == "fcu_local"
    checkpoint = len(h.status)
    initial_reference_x = h.status[-1].target_position.x
    h.translation[0] += .12
    if strategy == 0:
        h.until(lambda: h.status[-1].target_position.x > 4.115)
        assert h.status[-1].target_position.x == pytest.approx(4.12, abs=.005)
    else:
        h.until(lambda: any(request.request_id != old.request_id and request.mode != request.CANCEL
                            and abs(request.goal.x - 4.12) < .005 for _, request in h.requests))
        relevant = [request for _, request in h.requests]
        replacement = next(index for index, request in enumerate(relevant)
                           if request.mode != request.CANCEL and request.request_id != old.request_id
                           and abs(request.goal.x - 4.12) < .005)
        assert relevant[replacement].task_revision != old.task_revision
        if strategy == 1:
            cancel = next(index for index, request in enumerate(relevant)
                          if request.mode == request.CANCEL and request.request_id == old.request_id)
            assert cancel < replacement
        else:
            # Several normal map corrections must not repeatedly restart a cleared line segment.
            h.spin(.25)
            for correction in (2, 3):
                h.translation[0] += .12
                expected_x = 4. + correction * .12
                h.until(lambda: any(abs(request.goal.x - expected_x) < .005
                                    for _, request in h.requests))
                h.spin(.25)
            assert all(status.avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
                       for status in h.status[checkpoint:])
            assert h.status[-1].target_position.x > initial_reference_x + .05
    assert all(status.navigation_ready for status in h.status[checkpoint:])
    assert not h.arm_calls


def test_unlocalized_map_rejects_takeoff_and_local_return(navigation_node):
    """Neither initial map waypoints nor explicitly local return bypass localization."""
    from guided_interfaces.srv import FlightCommand
    h = navigation_node
    h.localized = False
    h.nav_enabled = True
    h.until(lambda: h.status[-1].navigation_frame == "map")
    assert not h.status[-1].navigation_ready
    assert not h.execute_navigation().accepted
    assert not h.execute_navigation(local=True).accepted
    request = h.request(FlightCommand)
    request.ttl_ms, request.value = 3000, 1.
    request.command = request.COMMAND_TAKEOFF
    assert not h.call(h.flight, request).accepted
    h.spin(.2)
    assert not h.arm_calls
    h.localized = True
    h.map_ready()
    h.state.armed = True
    h.until(lambda: h.status[-1].armed)
    assert h.execute_navigation(local=True).accepted
    h.until(lambda: h.status[-1].target_position.x > 3.99)
    assert (h.status[-1].target_position.x, h.status[-1].target_position.y,
            h.status[-1].target_position.z) == pytest.approx((4., 0., 1.2))
    h.until(lambda: abs(h.status[-1].target_yaw + .5) < 1.e-6)


@pytest.mark.parametrize("fault", ["jump", "timeout", "session"])
def test_navigation_fault_holds_then_lands_and_rejects_airborne_switch(navigation_node, fault):
    """Observe the actual HOLD mode and fake LAND request after invalidating navigation."""
    from guided_interfaces.msg import ControlStatus
    from guided_interfaces.srv import SetNavigationMode
    h = navigation_node
    h.map_ready()
    h.state.armed = True
    h.until(lambda: h.status[-1].armed)
    assert h.execute_navigation().accepted
    request = h.request(SetNavigationMode)
    request.ttl_ms, request.mode = 3000, "odom"
    assert not h.call(h.navigation, request).accepted
    h.spin(.1)
    assert not h.marker.exists()
    checkpoint = len(h.status)
    if fault == "jump":
        h.translation[0] += 3.
    elif fault == "timeout":
        h.tf_enabled = False
    else:
        h.driver_session = "replacement-session"
    h.until(lambda: any(status.control_mode == ControlStatus.MODE_FAILSAFE_HOLD
                        for status in h.status[checkpoint:]))
    assert not h.status[-1].navigation_ready
    h.until(lambda: "LAND" in h.mode_calls)
    assert any(status.control_mode == ControlStatus.MODE_FAILSAFE_HOLD
               for status in h.status[checkpoint:])
    assert not h.arm_calls


def test_odom_without_driver_preserves_local_waypoints(navigation_node):
    """Legacy odom needs no Odin navigation status and keeps its existing local endpoint."""
    h = navigation_node
    assert h.status[-1].navigation_frame == "odom"
    assert h.status[-1].navigation_ready
    h.state.armed = True
    h.until(lambda: h.status[-1].armed)
    assert h.execute_navigation().accepted
    h.until(lambda: h.status[-1].target_position.x > 3.99)
    assert (h.status[-1].target_position.x, h.status[-1].target_position.y,
            h.status[-1].target_position.z) == pytest.approx((4., 0., 1.2))
    h.until(lambda: abs(h.status[-1].target_yaw + .5) < 1.e-6)


def test_ground_navigation_switch_failure_restores_readiness(navigation_node):
    """A worker's atomic failure marker clears pending state and permits a later retry."""
    from guided_interfaces.srv import SetNavigationMode

    h = navigation_node
    h.until(lambda: h.status[-1].on_ground and h.navigation.service_is_ready())
    request = h.request(SetNavigationMode)
    request.ttl_ms, request.mode = 3000, "map"
    assert h.call(h.navigation, request).accepted
    h.until(lambda: h.marker.exists() and not h.status[-1].navigation_ready)
    pending = h.failure_marker.with_suffix(".pending")
    pending.write_text("synthetic worker restart failure\n")
    pending.replace(h.failure_marker)
    h.until(lambda: h.status[-1].navigation_ready
            and "定位切换失败" in h.status[-1].status_message)
    assert "synthetic worker restart failure" in h.status[-1].status_message
    assert h.status[-1].navigation_frame == "odom"
    request = h.request(SetNavigationMode)
    request.ttl_ms, request.mode = 3000, "map"
    assert h.call(h.navigation, request).accepted
    assert not h.failure_marker.exists()
    assert not h.arm_calls
