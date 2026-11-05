"""Exercise real onboard Tag arrival transitions with isolated localhost ROS services."""
from __future__ import annotations

import math

import pytest

from test_avoidance_executor import avoidance_node  # Reuse measured-state/process isolation.


@pytest.fixture
def calibration_node(avoidance_node, monkeypatch):
    """Add calibration evidence and services, never emulate the onboard state machine."""
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.srv import SetMode
    from nav_msgs.msg import Odometry
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
    h.raw_xy, h.raw_yaw, h.raw_body_velocity = (0., 0.), 0., (0., 0., 0.)
    h.position_y, h.measured_yaw, h.publish_raw = 0., 0., True
    original_pose_publish = h.pubs["local_position/pose"].publish

    def measured_pose(message):
        """Allow independent FCU Y/yaw without changing the shared flight fixture."""
        message.pose.position.y = h.position_y
        message.pose.orientation.z = math.sin(h.measured_yaw / 2.)
        message.pose.orientation.w = math.cos(h.measured_yaw / 2.)
        original_pose_publish(message)

    monkeypatch.setattr(h.pubs["local_position/pose"], "publish", measured_pose)
    pubs = {
        "status": h.node.create_publisher(CorrectionStatus, "/correction_service/status", 10),
        "extnav": h.node.create_publisher(ExtnavCorrectionStatus, "/extnav/correction_status", 10),
        "pose": h.node.create_publisher(PoseStamped, "/extnav/pose_fcu", 10),
        "velocity": h.node.create_publisher(TwistStamped, "/extnav/velocity_fcu", 10),
        "raw": h.node.create_publisher(Odometry, "/odin1/odometry_highfreq", 10),
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
        raw = Odometry()
        raw.header.stamp, raw.header.frame_id, raw.child_frame_id = stamp, "odom", "fcu"
        raw.pose.pose.position.x, raw.pose.pose.position.y = h.raw_xy
        raw.pose.pose.position.z = h.height
        raw.pose.pose.orientation.z, raw.pose.pose.orientation.w = math.sin(h.raw_yaw/2), math.cos(h.raw_yaw/2)
        raw.twist.twist.linear.x, raw.twist.twist.linear.y, raw.twist.twist.linear.z = h.raw_body_velocity
        if h.publish_raw:
            pubs["raw"].publish(raw)
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
def test_normal_capture_cleanup_waits_for_application(calibration_node):
    """Normal camera cleanup must not cancel a valid candidate or trigger LAND."""
    from guided_interfaces.msg import CommandResult
    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts) and h.status[-1].calibration_active)
    h.calibration.state = h.calibration.STATE_STOPPING
    h.calibration.application_state = h.calibration.APPLICATION_PENDING
    h.calibration.message = "候选已冻结；正在释放采集资源"
    h.calibration.resources_released = False
    h.spin(.6)
    assert not h.land_calls and not h.stops
    assert h.status[-1].calibration_active and h.status[-1].waypoint_index == 1
    assert not any(r.final and r.command == "waypoints" for r in h.results)
    h.calibration.state = h.calibration.STATE_APPLYING
    h.calibration.resources_released = True
    h.spin(.2)
    assert not h.land_calls and not h.stops
    h.finish_calibration()
    h.until(lambda: any(r.final and r.command == "waypoints" and
                       r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))
    assert not h.land_calls


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
@pytest.mark.parametrize("failure", ["failed", "session", "offline", "velocity", "raw", "timeout", "cancel", "mode"])
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
    elif failure == "timeout":
        # A cleanup phase that never completes remains subject to the existing total timeout.
        h.calibration.state = h.calibration.STATE_STOPPING
    elif failure == "session":
        h.extnav.odin_session_id = "restarted-session"
    elif failure == "offline":
        h.publish_calibration = False
    elif failure in ("velocity", "raw"):
        h.finish_calibration()
        if failure == "raw":
            h.external_x = 3.  # Keep fusion pending until the last raw sample actually expires.
            h.publish_raw = False
        else:
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


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .1}], indirect=True)
def test_calibration_application_and_fusion_preserve_height(calibration_node):
    """Measured XY may follow fusion; Z must retain its target and restore altitude in both directions."""
    from guided_interfaces.msg import CommandResult

    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts) and h.status[-1].calibration_active)
    target_z = h.status[-1].target_position.z
    assert target_z == pytest.approx(1.)
    h.external_x = 3.  # Keep fusion pending independently of the measured height.
    for stage in ("applying", "fusion"):
        if stage == "applying":
            h.calibration.state = h.calibration.STATE_APPLYING
        else:
            h.finish_calibration()
        for height, x in ((.65, .4), (1.25, -.2)):
            h.height, h.position_x = height, x
            h.spin(.3)
            status = h.status[-1]
            assert status.calibration_active and not h.land_calls
            assert status.position.z == pytest.approx(height)
            assert status.target_position.x == pytest.approx(x)
            assert status.target_position.z == pytest.approx(target_z)
            assert status.target_velocity.z == pytest.approx(0.)
            assert status.target_acceleration.z == pytest.approx(0.)
            # The fixture supplies MOT_THST_HOVER=.22; measured velocities stay zero.
            if height < target_z:
                assert h.attitudes[-1].thrust > .22
            else:
                assert h.attitudes[-1].thrust < .22
    h.height, h.position_x = target_z, h.external_x
    h.until(lambda: any(r.final and r.command == "waypoints" and
                       r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))
    assert not h.land_calls


@pytest.mark.parametrize("avoidance_node", [{
    "waypoint_hold_seconds": .1, "trajectory_dob_L_xy": 0.,
    "jerk_limit_modes.straight": False,
}], indirect=True)
def test_fusion_preserves_physical_anchor_across_coordinate_changes(calibration_node):
    """Keep physical XY/yaw error and body velocity despite EKF/Tag frame jumps and yaw wrap."""
    from test_avoidance_executor import _issued_acceleration

    h = calibration_node
    h.position_x, h.raw_xy = .08, (.5, -.2)
    h.spin(.1)
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    # A pre-existing .08 m FCU hover error must not be discarded when anchoring in raw Odin.
    anchor = (.42, -.2)
    h.external_x = 5.  # Keep the independently corrected pose outside the fusion gate.
    for stage in ("applying", "fusion"):
        if stage == "applying":
            h.calibration.state = h.calibration.STATE_APPLYING
        else:
            h.finish_calibration()
        for offset, raw_yaw, body_velocity in [((0., 0.), 0., (0., 0., 0.)),
                                              ((.28, .61), .35, (0., 0., 0.)),
                                              ((0., 0.), .35, (.1, -.05, 0.))]:
            previous_raw_acceleration = None
            for frame_degrees in (0., 90., 179., -179.):
                frame = math.radians(frame_degrees)
                c, s = math.cos(frame), math.sin(frame)
                h.raw_xy = (anchor[0] + offset[0], anchor[1] + offset[1])
                h.raw_yaw, h.raw_body_velocity = raw_yaw, body_velocity
                h.position_x = 1. + c * offset[0] - s * offset[1]
                h.position_y = -.5 + s * offset[0] + c * offset[1]
                h.measured_yaw = math.remainder(frame + raw_yaw, 2 * math.pi)
                h.spin(.25)
                status = h.status[-1]
                assert status.calibration_active and not h.land_calls
                assert (status.target_position.x, status.target_position.y) == pytest.approx((1., -.5))
                assert math.remainder(status.target_yaw - frame, 2*math.pi) == pytest.approx(0., abs=1.e-6)
                assert status.target_position.z == pytest.approx(1.)
                acceleration = _issued_acceleration(h.attitudes[-1])
                raw_acceleration = (c*acceleration[0] + s*acceleration[1],
                                    -s*acceleration[0] + c*acceleration[1])
                if previous_raw_acceleration is not None:
                    assert raw_acceleration == pytest.approx(previous_raw_acceleration, abs=1.e-5)
                previous_raw_acceleration = raw_acceleration
                if offset != (0., 0.):
                    assert sum(a*e for a, e in zip(raw_acceleration, offset)) < -.1
                elif body_velocity != (0., 0., 0.):
                    velocity = (math.cos(raw_yaw)*body_velocity[0] - math.sin(raw_yaw)*body_velocity[1],
                                math.sin(raw_yaw)*body_velocity[0] + math.cos(raw_yaw)*body_velocity[1])
                    assert sum(a*v for a, v in zip(raw_acceleration, velocity)) < -.01
                else:
                    assert raw_acceleration == pytest.approx((0., 0.), abs=1.e-6)


@pytest.mark.parametrize("avoidance_node", [{
    "control_frequency": 20., "status_frequency": 100., "waypoint_hold_seconds": .1,
    "max_jerk_xy": .2, "dob_L_xy": 0., "trajectory_dob_L_xy": 0.,
    "jerk_limit_modes.hover": True, "jerk_limit_modes.straight": True,
}], indirect=True)
def test_cancel_reframes_history_after_pose_callback_before_control_tick(calibration_node):
    """A pose update just before cancel must rotate retained slew history, not the new hover target."""
    from guided_interfaces.srv import FlightCommand
    from test_avoidance_executor import _issued_acceleration

    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    h.external_x = 3.
    h.finish_calibration()
    h.raw_xy, h.position_x = (.3, 0.), .3
    h.spin(2.)
    count = len(h.attitudes)
    h.until(lambda: len(h.attitudes) > count)
    before = _issued_acceleration(h.attitudes[-1])
    assert before[0] < -.02 and abs(before[1]) < 1.e-6
    # The intentionally slow control timer leaves room for pose + command callbacks between ticks.
    count = len(h.attitudes)
    h.position_x, h.position_y, h.measured_yaw = 0., .3, math.pi/2
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.pose.position.z = h.height
    h.pubs["local_position/pose"].publish(pose)
    h.until(lambda: abs(h.status[-1].yaw - math.pi/2) < 1.e-6)
    assert len(h.attitudes) == count  # Verify the intended interleaving, not a timing assumption.
    assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    h.until(lambda: len(h.attitudes) > count)
    after = _issued_acceleration(h.attitudes[count])
    assert abs(after[0]) < 1.e-6 and after[1] < -.005
    h.until(lambda: not h.status[-1].calibration_active)
    assert (h.status[-1].target_position.x, h.status[-1].target_position.y) == pytest.approx((0., .3))
    assert not h.land_calls
