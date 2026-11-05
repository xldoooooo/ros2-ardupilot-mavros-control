"""Exercise real onboard Tag arrival transitions with isolated localhost ROS services."""
from __future__ import annotations

import pytest

from test_avoidance_executor import avoidance_node  # Reuse measured-state/process isolation.


@pytest.fixture
def calibration_node(avoidance_node):
    """Add calibration evidence and services, never emulate the onboard state machine."""
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.srv import SetMode
    from correction_interfaces.msg import CorrectionStatus, ExtnavCorrectionStatus
    from correction_interfaces.srv import ClearWindow, SetCalibrationOwner, StartCorrection, StopCorrection
    from guided_interfaces.msg import VideoCapture

    h = avoidance_node
    h.policy = "clear"
    h.starts, h.clears, h.stops, h.land_calls, h.captures = [], [], [], [], []
    h.calibration = CorrectionStatus(
        interface_version="2.0", service_available=True, service_instance_id="calibration-instance",
        window_size=3, configured_tag_ids=[0, 1, 2, 3], window_state="empty")
    h.extnav = ExtnavCorrectionStatus(
        interface_version="2.0", service_available=True, odin_available=True,
        odin_session_id="calibration-session", final_sample_available=True,
        final_sample_odin_session_id="calibration-session")
    h.external_x, h.publish_calibration, h.publish_velocity = 0., True, True
    pubs = {
        "status": h.node.create_publisher(CorrectionStatus, "/correction_service/status", 10),
        "extnav": h.node.create_publisher(ExtnavCorrectionStatus, "/extnav/correction_status", 10),
        "pose": h.node.create_publisher(PoseStamped, "/extnav/pose_fcu", 10),
        "velocity": h.node.create_publisher(TwistStamped, "/extnav/velocity_fcu", 10),
    }

    def owner(req, res):
        """Model an exclusive lease with release stopping any active service job."""
        res.accepted = True
        h.calibration.automatic_owner = req.source_id if req.acquire else ""
        if not req.acquire:
            h.calibration.active = False
        return res

    def clear(req, res):
        """Clear only service history; the extnav correction remains unchanged."""
        h.clears.append(req)
        h.calibration.window_revision += 1
        h.calibration.window_count = 0
        res.accepted = True
        res.service_instance_id = h.calibration.service_instance_id
        res.window_revision = h.calibration.window_revision
        return res

    def start(req, res):
        """Record FIRST/NEXT and hold at sampling until the test supplies a result."""
        h.starts.append(req)
        res.accepted, res.job_id = True, f"calibration-job-{len(h.starts)}"
        res.service_instance_id = h.calibration.service_instance_id
        h.calibration.job_id = res.job_id
        h.calibration.state, h.calibration.active = h.calibration.STATE_SAMPLING, True
        return res

    def stop(req, res):
        """Record asynchronous cancellation without manufacturing task results."""
        h.stops.append(req)
        h.calibration.active = False
        res.accepted = True
        return res

    def mode(req, res):
        """Record the existing LAND service path while keeping the measured vehicle fixed."""
        h.land_calls.append(req.custom_mode)
        res.mode_sent = True
        return res

    def tick():
        """Publish independent bridge and service observations at runtime cadence."""
        if not h.publish_calibration:
            return
        stamp = h.node.get_clock().now().to_msg()
        h.calibration.header.stamp = stamp
        h.extnav.header.stamp = stamp
        pubs["status"].publish(h.calibration)
        pubs["extnav"].publish(h.extnav)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "odom"
        pose.pose.position.x, pose.pose.position.z = h.external_x, h.height
        pose.pose.orientation.w = 1.
        pubs["pose"].publish(pose)
        velocity = TwistStamped()
        velocity.header = pose.header
        if h.publish_velocity:
            pubs["velocity"].publish(velocity)

    def finish():
        """Supply matching job/application/frozen-sample evidence, not a flight transition."""
        c, e = h.calibration, h.extnav
        c.state, c.active = c.STATE_SUCCEEDED, False
        c.resources_released = c.window_saved = c.extnav_applied = True
        c.application_state = c.APPLICATION_APPLIED_ACK
        c.window_revision += 1
        c.window_count += 1
        c.window_state = "valid"
        e.revision += 1
        c.extnav_revision = e.revision
        e.correction_valid = e.final_sample_correction_valid = True
        e.applied_job_id = c.job_id
        e.final_sample_revision = e.revision

    services = [
        h.node.create_service(SetCalibrationOwner, "/correction_service/set_calibration_owner", owner),
        h.node.create_service(ClearWindow, "/correction_service/clear_window", clear),
        h.node.create_service(StartCorrection, "/correction_service/start", start),
        h.node.create_service(StopCorrection, "/correction_service/stop", stop),
        h.node.create_service(SetMode, h.mavros + "/set_mode", mode),
    ]
    subscription = h.node.create_subscription(VideoCapture, "/video_service/capture", h.captures.append, 10)
    timer = h.node.create_timer(.025, tick)
    h.finish_calibration = finish
    h.spin(.2)
    try:
        yield h
    finally:
        h.node.destroy_timer(timer)
        h.node.destroy_subscription(subscription)
        for service in services:
            h.node.destroy_service(service)


def execute(h, points, strategy=0, local=False):
    """Submit absolute odom points through the production command service."""
    from guided_interfaces.srv import ExecuteWaypoints
    req = h.request(ExecuteWaypoints)
    req.ttl_ms = 3000
    req.coordinate_frame = "odom"
    req.local_coordinates = local
    req.flight_strategy = strategy
    req.reference_generator, req.tracking_controller = (2 if strategy == 2 else 0), 1
    req.waypoints = points
    return h.call(h.waypoints, req)


def point(x=0., tag=0, calibration=True, inspection=False, continue_window=True):
    """Create an arrival action; all unmarked points remain ordinary by default."""
    from guided_interfaces.msg import Waypoint
    p = Waypoint(is_calibration=calibration, is_inspection=inspection,
                 calibration_tag_id=tag, continue_window=continue_window)
    p.position.x, p.position.z = x, 1.
    return p


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .1}], indirect=True)
def test_calibration_validates_sequence_and_modes(calibration_node):
    """Unknown/same-window Tags and unsupported frames/modes reject before any action."""
    h = calibration_node
    for points, strategy, local in [
        ([point(tag=9)], 0, False),
        ([point(), point()], 0, False),
        ([point()], 1, False),
        ([point()], 0, True),
    ]:
        assert not execute(h, points, strategy, local).accepted
    assert execute(h, [point(), point(1., continue_window=False)]).accepted
    assert not h.starts


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .1}], indirect=True)
def test_application_requires_fusion_and_preserves_absolute_goals(calibration_node):
    """Capture immediately, freeze the index, then join measured corrected pose to the unchanged goal."""
    h = calibration_node
    assert execute(h, [point(inspection=True), point(1.4, calibration=False)]).accepted
    h.until(lambda: len(h.starts) == 1)
    h.until(lambda: h.captures and h.status[-1].calibration_active)
    assert len(h.captures) == 1 and len(h.clears) == 1
    assert h.starts[0].operation == h.calibration.OPERATION_FIRST
    assert h.status[-1].waypoint_index == 1
    assert not any(r.final and r.command == "waypoints" for r in h.results)
    h.external_x = .7
    h.finish_calibration()
    h.spin(.5)
    assert h.status[-1].calibration_active and h.status[-1].waypoint_index == 1
    # Apparent velocity during EKF correction must not command physical braking or advance the point.
    h.velocity_x = .8
    h.spin(.2)
    assert h.status[-1].waypoint_index == 1
    assert abs(h.attitudes[-1].orientation.x) < .02
    assert abs(h.attitudes[-1].orientation.y) < .02
    h.velocity_x = 0.
    h.position_x = .7  # A measured EKF change, independent of the service ACK.
    h.until(lambda: h.status[-1].waypoint_index == 2)
    h.until(lambda: abs(h.status[-1].target_position.x - 1.4) < .01)
    assert len(h.starts) == 1 and not h.land_calls
    assert len(h.captures) == 1


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .1}], indirect=True)
@pytest.mark.parametrize("strategy", [0, 2])
def test_last_point_and_next_segment(calibration_node, strategy):
    """The final point waits for applied/fused evidence; FIRST/NEXT follow the task window."""
    h = calibration_node
    assert execute(h, [point(), point(tag=1), point(continue_window=False)], strategy=strategy).accepted
    for index in range(3):
        h.until(lambda: len(h.starts) == index + 1)
        h.spin(.15)
        assert h.status[-1].calibration_active
        assert h.status[-1].waypoint_index == index + 1
        assert not any(r.final and r.command == "waypoints" for r in h.results)
        h.finish_calibration()
    h.until(lambda: any(r.final and r.command == "waypoints" for r in h.results))
    from guided_interfaces.msg import CommandResult
    assert any(r.final and r.command == "waypoints" and r.status == CommandResult.STATUS_SUCCEEDED
               for r in h.results)
    assert [r.operation for r in h.starts] == [h.calibration.OPERATION_FIRST,
                                             h.calibration.OPERATION_NEXT,
                                             h.calibration.OPERATION_FIRST]
    assert len(h.clears) == 2 and not h.captures and not h.land_calls


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .1,
                                              "calibration_timeout_seconds": 2.5}], indirect=True)
@pytest.mark.parametrize("failure", ["failed", "session", "offline", "velocity", "timeout", "cancel", "mode"])
def test_failure_lands_and_cancel_ignores_late_result(calibration_node, failure):
    """Exceptions enter existing LAND, whereas explicit hover cancels without an automatic landing."""
    from guided_interfaces.srv import FlightCommand
    from guided_interfaces.msg import CommandResult
    h = calibration_node
    assert execute(h, [point(), point(2., calibration=False)]).accepted
    h.until(lambda: bool(h.starts))
    if failure == "failed":
        h.calibration.state = h.calibration.STATE_FAILED
        h.calibration.last_error = "camera open failed"
    elif failure == "session":
        h.extnav.odin_session_id = "restarted-session"
    elif failure == "offline":
        h.publish_calibration = False
    elif failure == "velocity":
        h.finish_calibration()
        h.publish_velocity = False
    elif failure in ("cancel", "mode"):
        if failure == "cancel":
            assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
        else:
            h.state.mode = "LOITER"
            h.until(lambda: not h.status[-1].calibration_active)
            h.until(lambda: bool(h.stops))
        h.finish_calibration()
        h.spin(.4)
        assert not h.status[-1].calibration_active and not h.land_calls
        assert not any(r.final and r.command == "waypoints" and
                       r.status == CommandResult.STATUS_SUCCEEDED for r in h.results)
        if failure == "cancel":
            assert h.status[-1].target_position.x == pytest.approx(h.position_x)
        return
    h.until(lambda: "LAND" in h.land_calls, 6.)
    h.until(lambda: not h.status[-1].calibration_active)
    assert any(r.final and r.command == "waypoints" and r.status == CommandResult.STATUS_FAILED
               for r in h.results)
