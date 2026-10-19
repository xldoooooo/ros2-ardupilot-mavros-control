"""Compare isolated Python and C++ extnav processes through their public ROS API.

Set EXTNAV_CPP_EXECUTABLE to the candidate binary; no aircraft or production
topics are used. Run with the project venv after sourcing the ROS interfaces.
"""

from __future__ import annotations

import copy
import math
import os
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np
import pytest

# The Python source remains the independent reference until candidate acceptance.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
# These are all public application parameters; endpoint values are isolated below.
DEFAULT_PARAMETERS = {
    "vision_pose_topic": "/mavros/vision_pose/pose",
    "pose_fcu_topic": "/extnav/pose_fcu",
    "vel_fcu_topic": "/extnav/velocity_fcu",
    "vision_rate_hz": 30.0,
    "ctrl_rate_hz": 100.0,
    "odom_topic": "/odin1/odometry_highfreq",
    "corrected_odom_topic": "/odin1/odometry_highfreq_corrected",
    "correction_status_topic": "/extnav/correction_status",
    "set_correction_service": "/extnav/set_correction",
    "odin_session_gap_seconds": 2.0,
    "odin_stamp_regression_seconds": 0.25,
    "max_correction_translation_m": 100.0,
    "roll_cam": 0.0,
    "pitch_cam": 0.0,
    "yaw_cam": 0.0,
    "odin_x": 0.0,
    "odin_y": 0.0,
    "odin_z": 0.0,
}
# Compare deterministic state; UUIDs, wall-clock ages and timer counts vary naturally.
STATE_FIELDS = (
    "interface_version", "service_available", "odin_available", "correction_valid",
    "revision", "reset_counter", "correction_x_m", "correction_y_m",
    "correction_yaw_deg", "applied_job_id", "horizontal_reference_mode",
    "lever_arm_x_m", "lever_arm_y_m", "lever_arm_z_m",
    "installation_roll_rad", "installation_pitch_rad", "installation_yaw_rad",
    "final_sample_available", "final_sample_correction_valid",
    "final_sample_revision", "final_sample_reference_mode", "raw_messages",
)


def _stamp_ns(message):
    """Read a ROS header stamp without converting through floating point."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _collector(messages):
    """Wrap deque.append with a Python signature accepted by rclpy."""
    def append(message):
        """Retain the latest observable ROS messages."""
        messages.append(message)

    return append


def _assert_ros_equal(reference, candidate, path="message"):
    """Compare every ROS field, reporting a precise path and preserving NaNs."""
    from rosidl_runtime_py.convert import message_to_ordereddict

    def compare(first, second, field):
        """Walk nested ROS mappings and compare floating point fields numerically."""
        if isinstance(first, dict):
            assert first.keys() == second.keys(), field
            for name in first:
                compare(first[name], second[name], f"{field}.{name}")
        elif isinstance(first, (list, tuple)):
            np.testing.assert_allclose(
                first, second, rtol=1e-12, atol=1e-12, equal_nan=True,
                err_msg=field,
            )
        elif isinstance(first, float):
            np.testing.assert_allclose(
                first, second, rtol=1e-12, atol=1e-12, equal_nan=True,
                err_msg=field,
            )
        else:
            assert first == second, f"{field}: Python={first!r}, C++={second!r}"

    compare(message_to_ordereddict(reference), message_to_ordereddict(candidate), path)


class _ProcessPair:
    """Own two child nodes and exercise only their public topics and services."""

    def __init__(self, executable, rpy, monkeypatch):
        """Create isolated ROS clients before starting the reference and candidate."""
        import rclpy
        from correction_interfaces.msg import ExtnavCorrectionStatus
        from correction_interfaces.srv import SetCorrection
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.qos import qos_profile_sensor_data

        self.rclpy = rclpy
        self.SetCorrection = SetCorrection
        self.GetParameters = GetParameters
        self.children = []
        self.logs = []
        self.endpoints = []
        self.sent = 0
        # Avoid the production domain and the project's fixed 230/231 test domains.
        self.domain = 170 + os.getpid() % 40
        monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=self.domain)
        self.node = Node(f"extnav_contract_probe_{os.getpid()}", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.temporary = tempfile.TemporaryDirectory(
            prefix="extnav-cpp-contract-", dir=PROJECT_ROOT / "agent" / "codex"
        )
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        commands = (
            [str(PROJECT_ROOT / ".venv/bin/python"),
             str(PROJECT_ROOT / "correction_service/extnav_patch/extnav_to_vision_pose.py")],
            [str(executable)],
        )
        for label, command in zip(("python", "cpp"), commands):
            name = f"extnav_contract_{label}_{os.getpid()}"
            prefix = f"/extnav_contract_{os.getpid()}/{label}"
            parameters = dict(DEFAULT_PARAMETERS)
            for key in parameters:
                if key.endswith("_topic") or key.endswith("_service"):
                    parameters[key] = f"{prefix}/{key}"
            parameters.update(zip(("roll_cam", "pitch_cam", "yaw_cam"), rpy))
            parameters.update(odin_x=0.173, odin_y=-0.067, odin_z=0.219)
            args = ["--ros-args", "-r", f"__node:={name}"]
            # Endpoint defaults stay observable through parameters: ROS remaps alone
            # isolate their graph names without overriding the original values.
            for key, default in DEFAULT_PARAMETERS.items():
                if key.endswith("_topic") or key.endswith("_service"):
                    args += ["-r", f"{default}:={parameters[key]}"]
                elif parameters[key] != default:
                    args += ["-p", f"{key}:={parameters[key]}"]
            endpoint = {
                "label": label, "name": name, "parameters": parameters,
                "status": deque(maxlen=32), "corrected": deque(maxlen=64),
                "pose": deque(maxlen=64), "vision": deque(maxlen=64),
                "velocity": deque(maxlen=64),
            }
            endpoint["publisher"] = self.node.create_publisher(
                Odometry, parameters["odom_topic"], qos_profile_sensor_data
            )
            endpoint["corrected_subscription"] = self.node.create_subscription(
                Odometry, parameters["corrected_odom_topic"],
                _collector(endpoint["corrected"]), qos_profile_sensor_data,
            )
            for key, message_type, parameter, qos in (
                ("status", ExtnavCorrectionStatus, "correction_status_topic", status_qos),
                ("pose", PoseStamped, "pose_fcu_topic", 10),
                ("vision", PoseStamped, "vision_pose_topic", 10),
                ("velocity", TwistStamped, "vel_fcu_topic", 10),
            ):
                endpoint[f"{key}_subscription"] = self.node.create_subscription(
                    message_type, parameters[parameter], _collector(endpoint[key]), qos
                )
            endpoint["client"] = self.node.create_client(
                SetCorrection, parameters["set_correction_service"]
            )
            endpoint["parameter_client"] = self.node.create_client(
                GetParameters, f"/{name}/get_parameters"
            )
            self.endpoints.append(endpoint)
            log = open(Path(self.temporary.name) / f"{label}.log", "w+")
            self.logs.append(log)
            environment = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
            self.children.append(subprocess.Popen(
                command + args, env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            ))

    def wait(self, predicate, description, timeout=5.0):
        """Spin until observable progress or include child diagnostics in a failure."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            for child in self.children:
                assert child.poll() is None, self.diagnostics(description)
            self.executor.spin_once(timeout_sec=0.02)
        raise AssertionError(self.diagnostics(f"timeout: {description}"))

    def diagnostics(self, description):
        """Report recent statuses and bounded process logs on contract failures."""
        output = [description]
        for endpoint, log in zip(self.endpoints, self.logs):
            log.flush()
            log.seek(0)
            output.append(f"{endpoint['label']} log:\n{log.read()[-5000:]}")
            if endpoint["status"]:
                output.append(f"{endpoint['label']} status: {endpoint['status'][-1]}")
        return "\n".join(output)

    def ready(self):
        """Wait for services, raw readers and initial authoritative status."""
        self.wait(
            lambda: all(
                item["client"].service_is_ready()
                and item["parameter_client"].service_is_ready()
                and item["publisher"].get_subscription_count() == 1
                and item["corrected_subscription"].get_publisher_count() == 1
                and item["status"] for item in self.endpoints
            ), "both process interfaces discovered", timeout=15.0,
        )
        # A matched reader precedes the bridge's existing 0.1 s discovery poll.
        discovery_settled = time.monotonic() + 0.2
        self.wait(lambda: time.monotonic() >= discovery_settled,
                  "initial corrected reader discovery poll")

    def compare_state(self, **expected):
        """Check both stable snapshots and verify their process-local sessions."""
        self.wait(
            lambda: all(
                all(getattr(item["status"][-1], key) == value
                    for key, value in expected.items()) for item in self.endpoints
            ), f"status {expected}",
        )
        first, second = (item["status"][-1] for item in self.endpoints)
        for field in STATE_FIELDS:
            assert getattr(first, field) == getattr(second, field), (
                f"status.{field}: Python={getattr(first, field)!r}, "
                f"C++={getattr(second, field)!r}"
            )
        for status in (first, second):
            if status.final_sample_available:
                assert status.final_sample_odin_session_id == status.odin_session_id
                assert status.final_sample_revision == status.revision
            else:
                assert status.final_sample_odin_session_id == ""
        return first

    def publish(self, raw, *, final=True, corrected=True):
        """Send one identical input and compare messages after both callbacks finish."""
        for endpoint in self.endpoints:
            endpoint["publisher"].publish(raw)
        self.sent += 1
        self.compare_state(raw_messages=self.sent, final_sample_available=final)
        if corrected:
            self.wait(
                lambda: all(
                    any(_stamp_ns(message) == _stamp_ns(raw)
                        for message in item["corrected"])
                    for item in self.endpoints
                ), "corrected input sample received",
            )
            messages = [next(message for message in reversed(item["corrected"])
                             if _stamp_ns(message) == _stamp_ns(raw))
                        for item in self.endpoints]
            _assert_ros_equal(*messages, path="corrected")
            assert messages[0].header == raw.header
            assert messages[0].child_frame_id == raw.child_frame_id
        if final:
            # Wait for output generated after the callback was acknowledged. This
            # excludes queued timer messages belonging to the previous snapshot.
            earliest = time.time_ns()
            self.wait(
                lambda: all(
                    item[key] and _stamp_ns(item[key][-1]) >= earliest
                    for item in self.endpoints for key in ("pose", "vision", "velocity")
                ), "fresh timer output for acknowledged sample",
            )
            for key in ("pose", "vision", "velocity"):
                messages = [copy.deepcopy(item[key][-1]) for item in self.endpoints]
                for message in messages:
                    assert message.header.frame_id == "odom"
                    assert _stamp_ns(message) > _stamp_ns(raw)
                    message.header.stamp.sec = 0
                    message.header.stamp.nanosec = 0
                _assert_ros_equal(*messages, path=key)

    def call(self, **values):
        """Submit equivalent requests using each process's own opaque session ID."""
        responses = []
        for endpoint in self.endpoints:
            request = self.SetCorrection.Request()
            request.job_id = "contract-job"
            request.odin_session_id = endpoint["status"][-1].odin_session_id
            request.expected_revision = endpoint["status"][-1].revision
            request.valid = True
            request.correction_x_m = 1.125
            request.correction_y_m = -2.75
            request.correction_yaw_rad = 0.623
            request.sample_count = 17
            request.position_std_m = 0.02
            request.yaw_std_rad = 0.03
            request.reprojection_error_px = 0.4
            request.odom_match_error_ms = 2.0
            for key, value in values.items():
                setattr(request, key, value)
            future = endpoint["client"].call_async(request)
            self.wait(future.done, f"{endpoint['label']} correction response")
            response = future.result()
            assert response is not None
            assert response.odin_session_id == endpoint["status"][-1].odin_session_id
            responses.append(response)
        for field in ("accepted", "applied", "revision", "message"):
            assert getattr(responses[0], field) == getattr(responses[1], field), (
                f"response.{field}: Python={getattr(responses[0], field)!r}, "
                f"C++={getattr(responses[1], field)!r}"
            )
        return responses[0]

    def assert_stopped(self):
        """Require timer outputs to stop after a snapshot-invalidating transition."""
        # Let already-queued DDS messages drain before measuring quiescence.
        drain_until = time.monotonic() + 0.08
        self.wait(lambda: time.monotonic() >= drain_until, "drain timer messages")
        stamps = [tuple(_stamp_ns(item[key][-1]) if item[key] else None
                        for key in ("pose", "vision", "velocity"))
                  for item in self.endpoints]
        stop_until = time.monotonic() + 0.12
        self.wait(lambda: time.monotonic() >= stop_until, "observe stopped outputs")
        assert stamps == [tuple(_stamp_ns(item[key][-1]) if item[key] else None
                                for key in ("pose", "vision", "velocity"))
                          for item in self.endpoints], self.diagnostics("stale output")

    def close(self):
        """Stop only children created by this harness and release the isolated context."""
        for child in self.children:
            if child.poll() is None:
                child.terminate()
        for child in self.children:
            try:
                child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3.0)
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.context.shutdown()
        for log in self.logs:
            log.close()
        self.temporary.cleanup()


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
    """Keep control timer accounting and restore its frozen sample after reader churn."""
    from geometry_msgs.msg import PoseStamped, TwistStamped

    channels = (("pose", "pose_fcu_topic", PoseStamped),
                ("velocity", "vel_fcu_topic", TwistStamped))
    previous = {}
    for item in pair.endpoints:
        for key, _, _ in channels:
            previous[item["label"], key] = copy.deepcopy(item[key][-1])
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
    pair.wait(lambda: all(_stamp_ns(item["vision"][-1]) > _stamp_ns(status)
                         for item, status in zip(pair.endpoints, after)),
              "vision samples spanning timer accounting interval")
    for item, before, status in zip(pair.endpoints, baseline, after):
        vision_count = sum(_stamp_ns(before) < _stamp_ns(message) <= _stamp_ns(status)
                           for message in item["vision"])
        # The shared counter includes each control tick even with both readers gone.
        assert status.final_pose_messages - before.final_pose_messages > vision_count, (
            f"{item['label']} control timer accounting stopped without readers"
        )
        assert status.final_sample_available and status.raw_messages == pair.sent
    resumed_at = time.time_ns()
    for item in pair.endpoints:
        for key, parameter, message_type in channels:
            item[f"{key}_subscription"] = pair.node.create_subscription(
                message_type, item["parameters"][parameter], _collector(item[key]), 10
            )
    pair.wait(lambda: all(_stamp_ns(item[key][-1]) >= resumed_at
                         for item in pair.endpoints for key, _, _ in channels),
              "auxiliary outputs resume without new raw")
    for item in pair.endpoints:
        for key, _, _ in channels:
            expected = previous[item["label"], key]
            actual = copy.deepcopy(item[key][-1])
            actual.header.stamp = expected.header.stamp
            _assert_ros_equal(expected, actual, path=f"{item['label']}.{key}.reader_resume")
    raw.header.stamp.nanosec += 1
    raw.pose.pose.position.x += 0.123
    pair.publish(raw)


@pytest.mark.parametrize("installation_rpy", [(0.0, 0.0, 0.0), (0.21, -0.17, 0.37)])
def test_extnav_cpp_matches_python_process_contract(installation_rpy, monkeypatch):
    """Compare public numerical, session, CAS, snapshot and reader-churn behavior."""
    binary = os.environ.get("EXTNAV_CPP_EXECUTABLE")
    if not binary:
        pytest.skip("set EXTNAV_CPP_EXECUTABLE to the isolated C++ candidate binary")
    executable = Path(binary).expanduser().resolve()
    assert executable.is_file() and os.access(executable, os.X_OK), executable
    pytest.importorskip("rclpy")
    pytest.importorskip("correction_interfaces.msg")
    pytest.importorskip("correction_interfaces.srv")
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    pair = _ProcessPair(executable, installation_rpy, monkeypatch)
    try:
        pair.ready()
        _check_parameters_and_qos(pair)
        pair.compare_state(raw_messages=0, correction_valid=False, revision=0,
                           odin_available=False, final_sample_available=False)
        raw = Odometry()
        raw.header.stamp.sec = 100
        raw.header.frame_id = "contract_odom"
        raw.child_frame_id = "contract_imu"
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

        if any(installation_rpy):
            # The existing public contract deliberately rejects Tag correction when
            # installation rotation is active; the rewrite must preserve it.
            assert not pair.call().accepted
            pair.compare_state(correction_valid=False, revision=0)
            return
        else:
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

        # A regression exactly at the threshold is tolerated; one nanosecond more
        # starts a new session. Zero stamps do not trigger the regression rule.
        previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
        raw.header.stamp.sec, raw.header.stamp.nanosec = (101, 0)
        pair.publish(raw)
        raw.header.stamp.sec, raw.header.stamp.nanosec = (100, 750_000_000)
        pair.publish(raw)
        assert previous_sessions == [item["status"][-1].odin_session_id for item in pair.endpoints]
        raw.header.stamp.nanosec = 499_999_999
        pair.publish(raw)
        assert all(item["status"][-1].odin_session_id != previous
                   for item, previous in zip(pair.endpoints, previous_sessions))
        for frame_field in ("header", "child"):
            previous_sessions = [item["status"][-1].odin_session_id for item in pair.endpoints]
            if frame_field == "header":
                raw.header.frame_id = "changed_parent"
            else:
                raw.child_frame_id = "changed_child"
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
        raw.header.stamp.nanosec += 1
        pair.publish(raw)
        revision = pair.endpoints[0]["status"][-1].revision
        pair.compare_state(odin_available=False, correction_valid=False,
                           final_sample_available=False)
        assert all(not item["status"][-1].odin_session_id for item in pair.endpoints)
        assert pair.endpoints[0]["status"][-1].revision == revision + 1
        pair.assert_stopped()
        assert not pair.call().accepted
        raw.header.stamp.sec = 200
        raw.header.stamp.nanosec = 0
        pair.publish(raw)
        assert all(item["status"][-1].odin_session_id not in previous_sessions
                   for item in pair.endpoints)
    finally:
        pair.close()
