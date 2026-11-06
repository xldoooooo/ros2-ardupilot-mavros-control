"""Map/TF/navigation service doubles for the isolated production executor fixture."""
import json
import math
import subprocess

import pytest


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
    h.rotation = (0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4))
    h.arm_accept = False
    h.on_arm = lambda: None
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
        """Record arming; takeoff tests can accept and model a Home-height update."""
        h.arm_calls.append(request.value)
        response.success = h.arm_accept
        if h.arm_accept:
            h.state.armed = request.value
            h.on_arm()
            h.pubs["state"].publish(h.state)
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
            for axis, value in zip("xyzw", h.rotation):
                setattr(transform.transform.rotation, axis, value)
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
