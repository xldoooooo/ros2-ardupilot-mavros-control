"""Shared isolated extnav process harness and native test-build discovery."""

from __future__ import annotations

import copy
import os
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np
import pytest

# Repository root used for the source reference and isolated native build.
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


class ProcessPair:
    """Own two child nodes and exercise only their public topics and services."""

    def __init__(self, executable, rpy, monkeypatch, extra_parameters=None):
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
            parameters.update(extra_parameters or {})
            args = ["--ros-args", "-r", f"__node:={name}"]
            # Endpoint defaults stay observable through parameters: ROS remaps alone
            # isolate their graph names without overriding the original values.
            for key, value in parameters.items():
                default = DEFAULT_PARAMETERS.get(key)
                if key.endswith("_topic") or key.endswith("_service"):
                    args += ["-r", f"{default}:={parameters[key]}"]
                elif parameters[key] != default:
                    encoded = str(value).lower() if isinstance(value, bool) else str(value)
                    args += ["-p", f"{key}:={encoded}"]
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
        earliest = time.time_ns()
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
            # Each source is emitted once: it can arrive before the status timer ACK.
            self.wait(
                lambda: all(
                    item[key] and _stamp_ns(item[key][-1]) >= earliest
                    for item in self.endpoints for key in ("pose", "velocity")
                ) and all(
                    item["vision"] and (
                        _stamp_ns(item["vision"][-1]) == _stamp_ns(raw)
                        if _stamp_ns(raw) > 0 else _stamp_ns(item["vision"][-1]) >= earliest
                    ) for item in self.endpoints
                ), "source-stamped vision and host-stamped auxiliary outputs",
            )
            for key in ("pose", "vision", "velocity"):
                messages = [copy.deepcopy(item[key][-1]) for item in self.endpoints]
                for message in messages:
                    assert message.header.frame_id == "odom"
                    if key == "vision" and _stamp_ns(raw) > 0:
                        assert message.header.stamp == raw.header.stamp
                    else:
                        assert _stamp_ns(message) >= earliest
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
        messages = [tuple(item[key][-1] if item[key] else None
                        for key in ("pose", "vision", "velocity"))
                  for item in self.endpoints]
        stop_until = time.monotonic() + 0.12
        self.wait(lambda: time.monotonic() >= stop_until, "observe stopped outputs")
        assert all(previous is (item[key][-1] if item[key] else None)
                   for item, before in zip(self.endpoints, messages)
                   for key, previous in zip(("pose", "vision", "velocity"), before)), (
            self.diagnostics("stale output")
        )

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


def native_executable() -> Path:
    """Require the dedicated native test build, or an explicitly supplied binary."""
    configured = os.environ.get("EXTNAV_CPP_EXECUTABLE")
    executable = (Path(configured).expanduser() if configured else
                  PROJECT_ROOT / "build/test-extnav/install/extnav_bridge/lib/extnav_bridge/extnav_to_vision_pose_native").resolve()
    if not executable.is_file() or not os.access(executable, os.X_OK):
        pytest.fail(
            f"Native extnav test executable unavailable: {executable}. "
            "Run bash tests/build_native.sh --packages extnav_bridge to build the candidate with "
            "correction_interfaces enabled, or set EXTNAV_CPP_EXECUTABLE. "
            "Use --level unit to run without native integration tests.",
            pytrace=False,
        )
    return executable
