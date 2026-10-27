"""Verify absolute task Z and minimum climb against the real executor on isolated DDS."""

import math

import pytest

from test_navigation_executor import navigation_node, navigation_process_settings
from test_avoidance_executor import avoidance_node


@pytest.fixture
def takeoff_node(navigation_node):
    """Add measured Home altitude and a recording takeoff service to the existing double."""
    from mavros_msgs.srv import CommandTOL
    from std_msgs.msg import Float64

    h = navigation_node
    h.home_altitude, h.post_arm_home_altitude = 8., .35
    h.altitude_enabled = True
    h.takeoffs = []
    h.arm_accept = True
    publisher = h.node.create_publisher(Float64, h.mavros + "/global_position/rel_alt", 10)

    def publish_altitude():
        """Keep the pre/post-arm datum change independent of the executor's calculations."""
        if h.altitude_enabled:
            publisher.publish(Float64(data=h.home_altitude))

    def update_home():
        """Model the real FCU's Home update during arming."""
        h.home_altitude = h.post_arm_home_altitude

    def takeoff(request, response):
        """Accept the command without manufacturing any measured climb."""
        h.takeoffs.append(request.altitude)
        response.success = True
        return response

    h.on_arm = update_home
    timer = h.node.create_timer(.025, publish_altitude)
    service = h.node.create_service(CommandTOL, h.mavros + "/cmd/takeoff", takeoff)
    h.spin(.2)
    try:
        yield h
    finally:
        h.node.destroy_timer(timer)
        h.node.destroy_service(service)


def request_takeoff(h, target):
    """Submit a straight takeoff through the same high-level interface as the GUI."""
    from guided_interfaces.srv import FlightCommand
    request = h.request(FlightCommand)
    request.command, request.ttl_ms, request.value = request.COMMAND_TAKEOFF, 3000, target
    return request, h.call(h.flight, request)


@pytest.mark.parametrize("frame,start_z,target,expected_local_z", [
    ("odom", 1.2, 1.7, 1.7),
    ("odom", -.7, -.2, -.2),
    ("map", .1, 100.5, .55),
    ("tilted_map", 0., .5, .45 + (.5 - .14 * math.sin(.1)) / math.cos(.1)),
])
def test_absolute_target_home_update_and_vertical_handover(
    takeoff_node, frame, start_z, target, expected_local_z
):
    """Resolve nonzero/signed task heights, use new Home data, and keep takeoff vertical."""
    from guided_interfaces.msg import CommandResult, ControlStatus
    h = takeoff_node
    h.height, h.position_x = start_z, .2
    if frame != "odom":
        h.translation = [0., 0., -100.]
        h.rotation = (0., 0., 0., 1.)
        if frame == "tilted_map":
            h.translation[2] = .4
            h.rotation = (0., math.sin(.05), 0., math.cos(.05))
        h.map_ready()
    h.spin(.2)
    request, response = request_takeoff(h, target)
    assert response.accepted, response.message
    h.until(lambda: bool(h.takeoffs))
    assert h.takeoffs == pytest.approx([.35 + expected_local_z - start_z])
    h.height = expected_local_z - .11
    h.spin(.15)
    assert not any(r.sequence == request.sequence and r.final for r in h.results)
    h.height = expected_local_z
    h.until(lambda: any(r.sequence == request.sequence and r.final and
                        r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))
    h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER)
    target_position = h.status[-1].target_position
    assert (target_position.x, target_position.y, target_position.z) == pytest.approx(
        (.2, 0., expected_local_z))


@pytest.mark.parametrize("avoidance_node,minimum", [({}, .2), ({"minimum_takeoff_rise_m": .4}, .4)],
                         indirect=["avoidance_node"])
def test_minimum_actual_rise_rejects_before_arming_and_controls_handover(takeoff_node, minimum):
    """Enforce the configured minimum on climb distance, including its exact boundary."""
    from guided_interfaces.msg import CommandResult, ControlStatus
    h = takeoff_node
    h.height = 1.
    h.spin(.2)
    for target in (1. - .1, 1. + minimum - .01):
        _, response = request_takeoff(h, target)
        assert not response.accepted
        assert "实际上升量" in response.message
        assert not h.arm_calls and not h.takeoffs
    request, response = request_takeoff(h, 1. + minimum)
    assert response.accepted, response.message
    h.until(lambda: bool(h.takeoffs))
    h.height = 1. + minimum - .01
    h.spin(.15)
    assert h.status[-1].control_mode == ControlStatus.MODE_TAKEOFF
    assert not any(r.sequence == request.sequence and r.final for r in h.results)
    h.height = 1. + minimum
    h.until(lambda: any(r.sequence == request.sequence and r.final and
                        r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))


def test_each_takeoff_uses_its_own_height_and_home_datum(takeoff_node):
    """A second flight must not reuse the preceding takeoff's target or Home offset."""
    from guided_interfaces.msg import CommandResult
    h = takeoff_node
    for start_z, target, home in ((-.7, -.2, .35), (1.2, 1.7, 2.)):
        h.state.armed, h.height = False, start_z
        h.post_arm_home_altitude = home
        h.spin(.2)
        count = len(h.takeoffs)
        request, response = request_takeoff(h, target)
        assert response.accepted, response.message
        h.until(lambda: len(h.takeoffs) > count)
        assert h.takeoffs[-1] == pytest.approx(home + .5)
        h.height = target
        h.until(lambda: any(r.sequence == request.sequence and r.final and
                            r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))
