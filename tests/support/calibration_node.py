"""Calibration service doubles layered on the isolated production executor fixture."""
from __future__ import annotations

import math

import pytest


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
