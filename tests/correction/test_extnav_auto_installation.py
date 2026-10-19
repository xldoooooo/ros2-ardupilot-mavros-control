"""Exercise automatic installation pitch through isolated DDS, without aircraft access.

Set EXTNAV_CPP_EXECUTABLE to the candidate and use the project venv after
sourcing ROS and correction_interfaces. All application endpoints are remapped.
"""

from __future__ import annotations

import math
import os
import secrets
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

# Process logs belong to the project agent directory, rather than production logs.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
# A short real-time window keeps DDS tests quick while preserving the 10 Hz sampler.
CALIBRATION_SECONDS = 0.3
# Exercise the rotating lever as well as orientation, with a nonzero 3D offset.
LEVER = np.array((0.173, -0.067, 0.219))
# These absolute production defaults must never escape the isolated namespace.
ENDPOINTS = {
    "raw": "/odin1/odometry_highfreq",
    "corrected": "/odin1/odometry_highfreq_corrected",
    "pose": "/extnav/pose_fcu",
    "velocity": "/extnav/velocity_fcu",
    "vision": "/mavros/vision_pose/pose",
    "status": "/extnav/correction_status",
    "correction": "/extnav/set_correction",
    "imu": "/mavros/imu/data",
    "state": "/mavros/state",
    "events": "/parameter_events",
    "rosout": "/rosout",
}


def _write_quaternion(message, rotation):
    """Write independent SciPy xyzw values to a ROS quaternion."""
    message.x, message.y, message.z, message.w = map(float, rotation.as_quat())


def _installation(status):
    """Read the actual applied mounting angles from the public status message."""
    return np.array((status.installation_roll_rad, status.installation_pitch_rad,
                     status.installation_yaw_rad))


class _Bridge:
    """Own one candidate subprocess and synthetic 50 Hz ROS input stream."""

    def __init__(self, executable, manual_degrees, auto):
        """Start the candidate with unique graph names and an isolated ROS context."""
        import rclpy
        from correction_interfaces.msg import ExtnavCorrectionStatus
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu

        self.manual = np.radians(manual_degrees)
        self.child = None
        self.statuses = deque(maxlen=32)
        self.poses = deque(maxlen=32)
        self.visions = deque(maxlen=32)
        self.raw = Odometry()
        self.raw.header.frame_id = "synthetic_odom"
        self.raw.child_frame_id = "synthetic_odin_imu"
        self.raw.pose.pose.orientation.w = 1.0
        self.raw.pose.pose.position.x = 1.2
        self.raw.pose.pose.position.y = -2.3
        self.raw.pose.pose.position.z = 0.81
        self.imu = Imu()
        self.imu.header.frame_id = "synthetic_fcu"
        self.imu.orientation.w = 1.0
        self.imu.linear_acceleration.z = 9.80665
        self.state = None
        self.State = State
        self.send_imu = True
        self.send_raw = True
        self.vary = None
        suffix = secrets.token_hex(5)
        prefix = f"/auto_installation_{suffix}"
        self.topics = {key: f"{prefix}/{key}" for key in ENDPOINTS}
        self.context = Context()
        domain = 170 + secrets.randbelow(40)
        rclpy.init(args=[], context=self.context, domain_id=domain)
        self.node = Node(f"auto_installation_probe_{suffix}", context=self.context,
                         namespace=prefix)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.raw_pub = self.node.create_publisher(
            Odometry, self.topics["raw"], qos_profile_sensor_data)
        self.imu_pub = self.node.create_publisher(
            Imu, self.topics["imu"], qos_profile_sensor_data)
        self.state_pub = self.node.create_publisher(State, self.topics["state"], 10)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(ExtnavCorrectionStatus, self.topics["status"],
                                      self.on_status, status_qos)
        self.node.create_subscription(PoseStamped, self.topics["pose"], self.on_pose, 10)
        self.node.create_subscription(PoseStamped, self.topics["vision"], self.on_vision, 10)
        self.timer = self.node.create_timer(0.02, self.publish)
        args = [str(executable), "--ros-args", "-r", f"__node:=bridge_{suffix}",
                "-r", f"__ns:={prefix}"]
        for key, default in ENDPOINTS.items():
            args += ["-r", f"{default}:={self.topics[key]}"]
        parameters = dict(zip(("roll_cam", "pitch_cam", "yaw_cam"), self.manual))
        parameters.update(zip(("odin_x", "odin_y", "odin_z"), LEVER))
        if auto is not None:
            parameters.update(auto_installation=auto,
                              auto_installation_seconds=CALIBRATION_SECONDS)
        for name, value in parameters.items():
            encoded = str(value).lower() if isinstance(value, bool) else str(float(value))
            args += ["-p", f"{name}:={encoded}"]
        log_directory = PROJECT_ROOT / "agent/codex"
        log_directory.mkdir(parents=True, exist_ok=True)
        self.log = tempfile.TemporaryFile(mode="w+", dir=log_directory)
        self.child = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT,
                                      env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
                                      start_new_session=True)

    def on_status(self, message):
        """Retain authoritative status updates."""
        self.statuses.append(message)

    def on_pose(self, message):
        """Retain FCU control output."""
        self.poses.append(message)

    def on_vision(self, message):
        """Retain independent vision timer output."""
        self.visions.append(message)

    def publish(self):
        """Publish fresh synthetic samples on the executor's real wall-clock cadence."""
        if self.vary is not None:
            self.vary(time.monotonic())
        stamp = self.node.get_clock().now().to_msg()
        self.raw.header.stamp = stamp
        self.imu.header.stamp = stamp
        if self.send_raw:
            self.raw_pub.publish(self.raw)
        if self.send_imu:
            self.imu_pub.publish(self.imu)
        if self.state is not None:
            self.state.header.stamp = stamp
            self.state_pub.publish(self.state)

    def diagnostics(self, reason):
        """Include recent status and child logs when a public contract fails."""
        self.log.flush()
        self.log.seek(0)
        return f"{reason}\nstatus={list(self.statuses)[-1:]}\n{self.log.read()[-5000:]}"

    def wait(self, predicate, reason, timeout=5.0):
        """Spin while streaming until a bounded observable condition becomes true."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.child.poll() is None, self.diagnostics("candidate exited")
            if predicate():
                return
            self.executor.spin_once(timeout_sec=0.01)
        raise AssertionError(self.diagnostics(reason))

    def run(self, seconds):
        """Stream for an actual elapsed interval, without fabricated ROS time."""
        deadline = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= deadline, "stream interval", seconds + 2.0)

    def set_attitudes(self, odin_fcu, mount, fcu_world_yaw=0.0):
        """Generate Odin and FCU attitudes with arbitrary independent world headings."""
        _write_quaternion(self.raw.pose.pose.orientation, odin_fcu * mount)
        fcu = Rotation.from_euler("z", fcu_world_yaw) * odin_fcu
        _write_quaternion(self.imu.orientation, fcu)
        gravity = fcu.inv().apply((0.0, 0.0, 9.80665))
        self.imu.linear_acceleration.x = float(gravity[0])
        self.imu.linear_acceleration.y = float(gravity[1])
        self.imu.linear_acceleration.z = float(gravity[2])

    def allow_calibration(self, connected=True, armed=False):
        """Publish an explicit FCU connection and arming state."""
        self.state = self.State(connected=connected, armed=armed)

    def assert_output(self, installation):
        """Check q_OF=q_OI*q_IF and raw+T-R_OF*T in both public pose outputs."""
        q = self.raw.pose.pose.orientation
        raw_rotation = Rotation.from_quat((q.x, q.y, q.z, q.w))
        expected = raw_rotation * Rotation.from_euler("xyz", installation).inv()
        p = self.raw.pose.pose.position
        expected_position = np.array((p.x, p.y, p.z)) + LEVER - expected.apply(LEVER)
        assert self.poses and self.visions, self.diagnostics("missing pose outputs")
        for message in (self.poses[-1], self.visions[-1]):
            q = message.pose.orientation
            actual = Rotation.from_quat((q.x, q.y, q.z, q.w))
            assert (expected.inv() * actual).magnitude() < math.radians(0.03)
            p = message.pose.position
            np.testing.assert_allclose((p.x, p.y, p.z), expected_position, atol=0.0002)

    def assert_manual(self):
        """Ensure rejected calibration still publishes using configured installation."""
        np.testing.assert_allclose(_installation(self.statuses[-1]), self.manual, atol=1e-10)
        assert not any("自动安装角已锁定" in message.last_event for message in self.statuses)
        self.assert_output(self.manual)

    def lock(self, expected_degrees):
        """Require one successful calibration and removal of its temporary readers."""
        self.wait(lambda: any("自动安装角已锁定" in item.last_event for item in self.statuses),
                  "automatic installation did not lock")
        expected = np.radians(expected_degrees)
        actual = _installation(self.statuses[-1])
        np.testing.assert_array_equal(actual[[0, 2]], self.manual[[0, 2]])
        assert actual[1] == pytest.approx(expected[1], abs=math.radians(0.03))
        self.wait(lambda: self.imu_pub.get_subscription_count() == 0
                  and self.state_pub.get_subscription_count() == 0,
                  "temporary calibration subscribers remain")
        self.run(0.2)
        self.assert_output(expected)

    def close(self):
        """Stop only the subprocess and ROS objects created by this fixture."""
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait(timeout=3.0)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
        self.log.close()


@pytest.fixture
def bridge_factory(monkeypatch):
    """Provide independent candidates without importing fixtures from older tests."""
    binary = os.environ.get("EXTNAV_CPP_EXECUTABLE")
    if not binary:
        pytest.skip("set EXTNAV_CPP_EXECUTABLE to the isolated C++ candidate binary")
    executable = Path(binary).expanduser().resolve()
    assert executable.is_file() and os.access(executable, os.X_OK), executable
    pytest.importorskip("rclpy")
    pytest.importorskip("correction_interfaces.msg")
    pytest.importorskip("mavros_msgs.msg")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    children = []

    def create(manual=(0.0, -22.0, 0.0), auto=True):
        """Wait for the new candidate while keeping state unknown during discovery."""
        bridge = _Bridge(executable, manual, auto)
        children.append(bridge)
        bridge.wait(lambda: bridge.statuses and bridge.poses and bridge.visions
                    and bridge.raw_pub.get_subscription_count() == 1,
                    "candidate discovery", timeout=15.0)
        if auto:
            bridge.wait(lambda: bridge.imu_pub.get_subscription_count() == 1
                        and bridge.state_pub.get_subscription_count() == 1,
                        "calibration input discovery")
        return bridge

    try:
        yield create
    finally:
        for child in children:
            child.close()


def test_reported_pitch_correction_and_lock_survive_arming(bridge_factory):
    """Recover -25.3 degrees, then retain the lock despite changed mounting inputs."""
    bridge = bridge_factory()
    mount = Rotation.from_euler("y", -25.3, degrees=True)
    body = Rotation.from_euler("y", 0.6, degrees=True)
    bridge.set_attitudes(body, mount)
    bridge.run(0.8)
    bridge.assert_manual()
    fallback = bridge.poses[-1].pose.orientation
    assert Rotation.from_quat((fallback.x, fallback.y, fallback.z, fallback.w)).as_euler(
        "xyz", degrees=True)[1] == pytest.approx(-2.7, abs=0.001)
    bridge.allow_calibration()
    bridge.lock((0.0, -25.3, 0.0))
    locked = _installation(bridge.statuses[-1]).copy()
    bridge.allow_calibration(armed=True)
    bridge.set_attitudes(Rotation.from_euler("xyz", (8.0, -6.0, 71.0), degrees=True),
                         Rotation.from_euler("y", -20.0, degrees=True), 1.7)
    bridge.run(1.0)
    np.testing.assert_array_equal(_installation(bridge.statuses[-1]), locked)
    bridge.assert_output(locked)
    bridge.allow_calibration(armed=False)
    bridge.run(0.8)
    np.testing.assert_array_equal(_installation(bridge.statuses[-1]), locked)
    assert bridge.imu_pub.get_subscription_count() == bridge.state_pub.get_subscription_count() == 0


@pytest.mark.parametrize("pitch", (-20.0, -25.0, -30.0))
def test_tilted_body_and_independent_world_yaw_recover_mount(bridge_factory, pitch):
    """Fit pitch from gravity while preserving configured roll and yaw exactly."""
    bridge = bridge_factory(manual=(3.0, -22.0, 12.0))
    expected = (3.0, pitch, 12.0)
    bridge.set_attitudes(
        Rotation.from_euler("xyz", (9.0, -7.0, 63.0), degrees=True),
        Rotation.from_euler("xyz", expected, degrees=True), fcu_world_yaw=-1.9)
    bridge.allow_calibration()
    bridge.lock(expected)


def test_unknown_armed_motion_and_stale_imu_preserve_manual_fallback(bridge_factory):
    """Require known disarmed state, fresh IMU, low rates and a continuous quiet window."""
    bridge = bridge_factory()
    body = Rotation.from_euler("xyz", (4.0, -3.0, 17.0), degrees=True)
    mount = Rotation.from_euler("y", -25.0, degrees=True)
    bridge.set_attitudes(body, mount, fcu_world_yaw=1.1)
    bridge.run(0.8)
    bridge.assert_manual()
    for connected, armed in ((False, False), (True, True)):
        bridge.allow_calibration(connected=connected, armed=armed)
        bridge.run(0.8)
        bridge.assert_manual()
    # Make one gate invalid before publishing the newly allowed state.
    bridge.raw.twist.twist.linear.x = 0.3
    bridge.allow_calibration()
    bridge.run(0.8)
    bridge.assert_manual()
    bridge.raw.twist.twist.angular.z = 0.1
    bridge.raw.twist.twist.linear.x = 0.0
    bridge.run(0.8)
    bridge.assert_manual()
    bridge.imu.angular_velocity.y = 0.1
    bridge.raw.twist.twist.angular.z = 0.0
    bridge.run(0.8)
    bridge.assert_manual()
    # Leave a quiet but stale IMU sample; arming prevents calibration while aging it.
    bridge.allow_calibration(armed=True)
    bridge.imu.angular_velocity.y = 0.0
    bridge.run(0.2)
    bridge.send_imu = False
    bridge.run(0.8)
    bridge.allow_calibration()
    bridge.run(0.8)
    bridge.assert_manual()

    def vary_attitude(now):
        """Change gravity direction even when both reported gyroscopes read zero."""
        moving_body = Rotation.from_euler("x", 0.07 * math.sin(13.0 * now)) * body
        bridge.set_attitudes(moving_body, mount, fcu_world_yaw=1.1)

    bridge.vary = vary_attitude
    bridge.send_imu = True
    bridge.run(1.0)
    bridge.vary = None
    bridge.run(0.05)
    bridge.assert_manual()
    bridge.set_attitudes(body, mount, fcu_world_yaw=1.1)
    bridge.lock((0.0, -25.0, 0.0))


def test_default_disabled_keeps_manual_mount_without_calibration_readers(bridge_factory):
    """Keep automatic calibration opt-in, including its subscription overhead."""
    bridge = bridge_factory(auto=None)
    bridge.set_attitudes(Rotation.from_euler("y", 0.6, degrees=True),
                         Rotation.from_euler("y", -25.3, degrees=True))
    bridge.allow_calibration()
    bridge.run(1.0)
    bridge.assert_manual()
    assert bridge.imu_pub.get_subscription_count() == bridge.state_pub.get_subscription_count() == 0


def test_active_tag_pauses_automatic_installation_until_cleared(bridge_factory):
    """Retain the existing Tag correction contract while automatic fitting is paused."""
    from correction_interfaces.srv import SetCorrection

    bridge = bridge_factory(manual=(0.0, 0.0, 0.0))
    bridge.set_attitudes(Rotation.from_euler("y", 0.6, degrees=True),
                         Rotation.from_euler("y", -25.3, degrees=True))
    client = bridge.node.create_client(SetCorrection, bridge.topics["correction"])
    bridge.wait(client.service_is_ready, "Tag service discovery")
    request = SetCorrection.Request(
        job_id="pause-auto-fit", odin_session_id=bridge.statuses[-1].odin_session_id,
        expected_revision=bridge.statuses[-1].revision, valid=True,
        correction_x_m=0.2, correction_y_m=-0.1, correction_yaw_rad=0.1,
        sample_count=10,
    )
    future = client.call_async(request)
    bridge.wait(future.done, "apply Tag correction")
    assert future.result().accepted and future.result().applied
    bridge.allow_calibration()
    bridge.run(1.0)
    assert bridge.statuses[-1].correction_valid
    np.testing.assert_array_equal(_installation(bridge.statuses[-1]), np.zeros(3))
    assert not any("自动安装角已锁定" in item.last_event for item in bridge.statuses)
    assert bridge.imu_pub.get_subscription_count() == bridge.state_pub.get_subscription_count() == 1

    request.valid = False
    request.job_id = "clear-tag-for-auto-fit"
    request.expected_revision = bridge.statuses[-1].revision
    future = client.call_async(request)
    bridge.wait(future.done, "clear Tag correction")
    assert future.result().accepted
    bridge.lock((0.0, -25.3, 0.0))
