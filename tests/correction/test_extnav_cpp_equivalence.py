"""Compare Python and production C++ extnav behavior through isolated ROS APIs."""

from __future__ import annotations

import copy
import math
import time

import numpy as np
import pytest

from tests.support.extnav import (
    DEFAULT_PARAMETERS,
    ProcessPair,
    _collector,
    _stamp_ns,
    native_executable,
)


# These contracts require ROS discovery and the dedicated native build.
pytestmark = pytest.mark.integration


def _check_parameters_and_qos(pair):
    """Verify actual parameter defaults and DDS endpoint contracts on both nodes."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

    for endpoint in pair.endpoints:
        request = pair.GetParameters.Request()
        request.names = list(DEFAULT_PARAMETERS)
        future = endpoint["parameter_client"].call_async(request)
        pair.wait(future.done, "query all application parameters")
        for key, value in zip(request.names, future.result().values):
            expected = endpoint["parameters"][key]
            if key.endswith("_topic") or key.endswith("_service"):
                expected = DEFAULT_PARAMETERS[key]
            actual = value.string_value if isinstance(expected, str) else value.double_value
            assert actual == expected, f"{endpoint['label']} parameter {key}"
        for key, is_subscription, reliability, durability, depth in (
            ("odom_topic", True, ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, 5),
            ("corrected_odom_topic", False, ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE, 5),
            ("pose_fcu_topic", False, ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE, 10),
            ("vel_fcu_topic", False, ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE, 10),
            ("vision_pose_topic", False, ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE, 10),
            ("correction_status_topic", False, ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1),
        ):
            query = (pair.node.get_subscriptions_info_by_topic if is_subscription
                     else pair.node.get_publishers_info_by_topic)
            pair.wait(lambda: bool(query(endpoint["parameters"][key])), f"discover {key} QoS")
            info = next(item for item in query(endpoint["parameters"][key])
                        if item.node_name == endpoint["name"])
            qos = info.qos_profile
            assert (qos.reliability, qos.durability) == (
                reliability, durability,
            ), f"{endpoint['label']} {key}: {qos}"
            # Fast DDS reports UNKNOWN/0 for remote history/depth; check them
            # whenever the selected middleware actually exposes those values.
            if qos.history != HistoryPolicy.UNKNOWN:
                assert qos.history == HistoryPolicy.KEEP_LAST
            if qos.depth:
                assert qos.depth == depth


def _check_auxiliary_reader_churn(pair, raw):
    """Keep timer accounting and resume readers only with a new source sample."""
    from geometry_msgs.msg import PoseStamped, TwistStamped

    channels = (("pose", "pose_fcu_topic", PoseStamped),
                ("velocity", "vel_fcu_topic", TwistStamped))
    for item in pair.endpoints:
        for key, _, _ in channels:
            pair.node.destroy_subscription(item[f"{key}_subscription"])
    pair.wait(lambda: all(not pair.node.get_subscriptions_info_by_topic(item["parameters"][parameter])
                         for item in pair.endpoints for _, parameter, _ in channels),
              "auxiliary readers left DDS graph")
    removed_at = time.time_ns()
    pair.wait(lambda: all(_stamp_ns(item["status"][-1]) > removed_at
                         for item in pair.endpoints), "status after auxiliary reader departure")
    baseline = [item["status"][-1] for item in pair.endpoints]
    pair.wait(lambda: all(_stamp_ns(item["status"][-1]) > _stamp_ns(before)
                         for item, before in zip(pair.endpoints, baseline)),
              "next status with no auxiliary readers")
    after = [item["status"][-1] for item in pair.endpoints]
    for item, before, status in zip(pair.endpoints, baseline, after):
        # The shared counter includes each control tick even with both readers gone.
        assert status.final_pose_messages > before.final_pose_messages, (
            f"{item['label']} control timer accounting stopped without readers"
        )
        assert status.final_sample_available and status.raw_messages == pair.sent
    for item in pair.endpoints:
        for key, parameter, message_type in channels:
            item[f"{key}_subscription"] = pair.node.create_subscription(
                message_type, item["parameters"][parameter], _collector(item[key]), 10
            )
    pair.wait(lambda: all(item[f"{key}_subscription"].get_publisher_count() == 1
                         for item in pair.endpoints for key, _, _ in channels),
              "auxiliary readers rejoined")
    pair.assert_stopped()
    raw.header.stamp.nanosec += 1
    raw.pose.pose.position.x += 0.123
    pair.publish(raw)


@pytest.mark.parametrize("installation_rpy", [(0.0, 0.0, 0.0), (0.21, -0.17, 0.37)])
def test_extnav_cpp_matches_python_process_contract(installation_rpy, monkeypatch):
    """Compare public numerical, session, CAS, snapshot and reader-churn behavior."""
    executable = native_executable()
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    pair = ProcessPair(executable, installation_rpy, monkeypatch)
    try:
        pair.ready()
        _check_parameters_and_qos(pair)
        pair.compare_state(raw_messages=0, correction_valid=False, revision=0,
                           odin_available=False, final_sample_available=False)
        raw = Odometry()
        raw.header.stamp.sec = 100
        raw.header.frame_id = "odom"
        raw.child_frame_id = "fcu"
        raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z = (1.2, -2.3, 0.81)
        raw.pose.pose.orientation.x, raw.pose.pose.orientation.y = (0.2, -0.3)
        raw.pose.pose.orientation.z, raw.pose.pose.orientation.w = (0.4, 0.8)
        raw.twist.twist.linear.x, raw.twist.twist.linear.y, raw.twist.twist.linear.z = (0.7, -0.2, 0.13)
        raw.twist.twist.angular.x, raw.twist.twist.angular.y, raw.twist.twist.angular.z = (0.21, -0.18, 0.53)
        matrix = np.arange(36, dtype=float).reshape(6, 6) / 20.0
        raw.pose.covariance = (matrix + matrix.T + np.eye(6)).reshape(-1).tolist()
        raw.twist.covariance = (matrix + matrix.T + 2.0 * np.eye(6)).reshape(-1).tolist()
        pair.publish(raw)
        initial_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
        assert all(initial_sessions)

        for rejection in (
            {"expected_revision": 5}, {"odin_session_id": "stale-session"},
            {"sample_count": 0}, {"position_std_m": -1.0},
            {"yaw_std_rad": math.nan}, {"correction_yaw_rad": math.inf},
            {"correction_x_m": 101.0},
        ):
            assert not pair.call(**rejection).accepted
        assert pair.call().accepted
        pair.compare_state(correction_valid=True, revision=1, final_sample_available=False)
        pair.assert_stopped()
        # Idempotency precedes CAS and preserves the frozen-sample invalidation.
        assert pair.call(expected_revision=0).accepted
        assert not pair.call(job_id="conflict", expected_revision=0).accepted
        pair.compare_state(revision=1, final_sample_available=False)
        raw.header.stamp.nanosec += 1
        pair.publish(raw)
        for sentinel in (-1.0, math.nan):
            raw.header.stamp.nanosec += 1
            raw.pose.covariance[0] = sentinel
            raw.twist.covariance[0] = sentinel
            pair.publish(raw)
            for item in pair.endpoints:
                np.testing.assert_allclose(
                    item["corrected"][-1].pose.covariance, raw.pose.covariance,
                    equal_nan=True, err_msg=f"{item['label']} sentinel covariance",
                )
        assert not pair.call(valid=False, odin_session_id="stale-session").accepted
        clear = pair.call(valid=False)
        assert clear.accepted and not clear.applied and clear.revision == 2
        pair.compare_state(correction_valid=False, revision=2, final_sample_available=False)
        pair.assert_stopped()
        assert pair.call(valid=False).revision == 2
        raw.header.stamp.nanosec += 1
        pair.publish(raw)

        # Reordering on either side of the former 250 ms boundary must leave the
        # live correction and output unchanged, then allow the next fresh sample.
        assert pair.call(job_id="before-reordering").accepted
        pair.compare_state(correction_valid=True, revision=3)
        previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
        raw.header.stamp.sec, raw.header.stamp.nanosec = (101, 0)
        pair.publish(raw)
        rejected_at = time.time_ns()
        before_rejection = [dict((key, item[key][-1])
                                 for key in ("corrected", "pose", "vision", "velocity"))
                            for item in pair.endpoints]
        rejected = copy.deepcopy(raw)
        rejected.pose.pose.position.x = -123.0
        for stamp in ((101, 0), (100, 850_000_000), (100, 750_000_000),
                      (100, 749_999_999), (99, 0)):
            rejected.header.stamp.sec, rejected.header.stamp.nanosec = stamp
            for endpoint in pair.endpoints:
                endpoint["publisher"].publish(rejected)
        pair.wait(lambda: all(_stamp_ns(item["status"][-1]) > rejected_at
                             for item in pair.endpoints), "status after rejected samples")
        pair.compare_state(raw_messages=pair.sent, final_sample_available=True,
                           correction_valid=True, revision=3)
        pair.assert_stopped()
        assert all(item[key][-1] is message
                   for item, previous in zip(pair.endpoints, before_rejection)
                   for key, message in previous.items())
        assert previous_sessions == [item["status"][-1].odin_session_id for item in pair.endpoints]
        raw.header.stamp.sec, raw.header.stamp.nanosec = (102, 0)
        pair.publish(raw)
        pair.compare_state(correction_valid=True, revision=3)
        assert previous_sessions == [item["status"][-1].odin_session_id for item in pair.endpoints]
        for frame_field in ("header", "child"):
            previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
            broken = copy.deepcopy(raw)
            if frame_field == "header":
                broken.header.frame_id = "legacy_odom"
            else:
                broken.child_frame_id = "imu"
            for endpoint in pair.endpoints:
                endpoint["publisher"].publish(broken)
            pair.compare_state(odin_available=False, final_sample_available=False,
                               raw_messages=pair.sent)
            pair.assert_stopped()
            raw.header.stamp.nanosec += 1
            pair.publish(raw)
            assert all(item["status"][-1].odin_session_id != previous
                       for item, previous in zip(pair.endpoints, previous_sessions))
        previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
        raw.header.stamp.sec, raw.header.stamp.nanosec = (0, 0)
        pair.publish(raw)
        assert previous_sessions == [item["status"][-1].odin_session_id for item in pair.endpoints]

        # Only malformed orientation/position invalidate the final snapshot; raw
        # corrected passthrough remains observable, including the malformed fields.
        for invalid in ("zero_quaternion", "nan_quaternion", "inf_quaternion", "nan_position"):
            broken = copy.deepcopy(raw)
            raw.header.stamp.nanosec += 1
            broken.header.stamp.nanosec = raw.header.stamp.nanosec
            if invalid == "zero_quaternion":
                broken.pose.pose.orientation.x = broken.pose.pose.orientation.y = 0.0
                broken.pose.pose.orientation.z = broken.pose.pose.orientation.w = 0.0
            elif invalid == "nan_quaternion":
                broken.pose.pose.orientation.w = math.nan
            elif invalid == "inf_quaternion":
                broken.pose.pose.orientation.w = math.inf
            else:
                broken.pose.pose.position.x = math.nan
            pair.publish(broken, final=False)
            assert all(item["status"][-1].last_error for item in pair.endpoints)
            pair.assert_stopped()
            raw.header.stamp.nanosec += 1
            pair.publish(raw)

        # Reader churn changes corrected publication only, never raw processing or
        # the FCU timer path. Wait for DDS discovery rather than assuming its latency.
        for endpoint in pair.endpoints:
            pair.node.destroy_subscription(endpoint["corrected_subscription"])
        pair.wait(lambda: all(not pair.node.get_subscriptions_info_by_topic(
            item["parameters"]["corrected_odom_topic"]
        ) for item in pair.endpoints), "corrected readers left DDS graph")
        settled = time.monotonic() + 0.2
        pair.wait(lambda: time.monotonic() >= settled, "reader departure poll")
        before = [item["status"][-1].corrected_messages for item in pair.endpoints]
        # Refresh counters before the measured reader-free sample.
        pair.compare_state(raw_messages=pair.sent)
        raw.header.stamp.nanosec += 1
        pair.publish(raw, corrected=False)
        assert before == [item["status"][-1].corrected_messages for item in pair.endpoints]
        for endpoint in pair.endpoints:
            endpoint["corrected_subscription"] = pair.node.create_subscription(
                Odometry, endpoint["parameters"]["corrected_odom_topic"],
                _collector(endpoint["corrected"]), qos_profile_sensor_data,
            )
        pair.wait(lambda: all(item["corrected_subscription"].get_publisher_count() == 1
                              for item in pair.endpoints), "corrected readers matched")
        settled = time.monotonic() + 0.25
        pair.wait(lambda: time.monotonic() >= settled, "reader arrival poll")
        raw.header.stamp.nanosec += 1
        pair.publish(raw)
        assert all(item["status"][-1].corrected_messages > count
                   for item, count in zip(pair.endpoints, before))

        _check_auxiliary_reader_churn(pair, raw)
        assert pair.call(job_id="before-gap").accepted
        pair.compare_state(correction_valid=True, final_sample_available=False)
        raw.header.stamp.sec, raw.header.stamp.nanosec = (200, 0)
        pair.publish(raw)
        revision = pair.endpoints[0]["status"][-1].revision
        previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
        # Repeated traffic must still time out and stay unavailable beyond the gap.
        repeated_until = time.monotonic() + 3.0
        while time.monotonic() < repeated_until:
            for endpoint in pair.endpoints:
                endpoint["publisher"].publish(raw)
            pair.executor.spin_once(timeout_sec=0.02)
        pair.compare_state(odin_available=False, correction_valid=False,
                           final_sample_available=False, raw_messages=pair.sent)
        assert all(not item["status"][-1].odin_session_id for item in pair.endpoints)
        assert pair.endpoints[0]["status"][-1].revision == revision + 1
        pair.assert_stopped()
        assert not pair.call().accepted
        # A genuine clock restart can recover after the same existing timeout.
        raw.header.stamp.sec = 10
        raw.header.stamp.nanosec = 0
        pair.publish(raw)
        assert all(item["status"][-1].odin_session_id not in previous_sessions
                   for item in pair.endpoints)
    finally:
        pair.close()
