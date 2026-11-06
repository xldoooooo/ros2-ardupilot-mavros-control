"""Exercise real map execution using localhost DDS and fictitious MAVROS services only."""

import math

import pytest

pytestmark = pytest.mark.integration

from tests.support.avoidance_node import avoidance_node
from tests.support.navigation_node import navigation_node, navigation_process_settings


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
