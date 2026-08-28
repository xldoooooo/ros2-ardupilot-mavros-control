"""在隔离 DDS 域验证 extnav identity、CAS、持续生效和 Odin session 失效。"""

from __future__ import annotations

import importlib.util
import math
import threading
import time
from pathlib import Path
from typing import Any

import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _load_extnav_module() -> Any:
    """加载待部署的 canonical extnav 源码。"""
    path = (
        PROJECT_ROOT
        / "correction_service"
        / "extnav_patch"
        / "extnav_to_vision_pose.py"
    )
    spec = importlib.util.spec_from_file_location("task27_extnav_ros", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wait_for(predicate, timeout: float = 4.0) -> None:
    """有限等待后台 executor 的可观测结果。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for ROS integration condition")


def test_extnav_identity_atomic_apply_and_session_invalidation(monkeypatch) -> None:
    """修正服务不存在时 extnav 仍应独立维持数据链和 active T/R。"""
    rclpy = pytest.importorskip("rclpy")
    correction_messages = pytest.importorskip("correction_interfaces.msg")
    correction_services = pytest.importorskip("correction_interfaces.srv")
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=230)
    extnav = _load_extnav_module()
    bridge = extnav.OdometryBridge(context=context)
    tester = Node("task27_extnav_tester", context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(bridge)
    executor.add_node(tester)
    corrected: list[Odometry] = []
    statuses: list[Any] = []
    tester.create_subscription(
        Odometry,
        "/odin1/odometry_highfreq_corrected",
        corrected.append,
        qos_profile_sensor_data,
    )
    transient = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    tester.create_subscription(
        correction_messages.ExtnavCorrectionStatus,
        "/extnav/correction_status",
        statuses.append,
        transient,
    )
    publisher = tester.create_publisher(
        Odometry, "/odin1/odometry_highfreq", qos_profile_sensor_data
    )
    client = tester.create_client(
        correction_services.SetCorrection, "/extnav/set_correction"
    )
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        raw = Odometry()
        raw.header.stamp.sec = 100
        raw.header.frame_id = "odin"
        raw.child_frame_id = "imu"
        raw.pose.pose.position.x = 1.0
        raw.pose.pose.position.y = 2.0
        raw.pose.pose.position.z = 0.7
        raw.pose.pose.orientation.w = 1.0
        for _ in range(5):
            publisher.publish(raw)
            time.sleep(0.04)
        _wait_for(lambda: corrected and statuses and statuses[-1].odin_session_id)
        assert corrected[-1].pose.pose.position.x == raw.pose.pose.position.x
        assert corrected[-1].pose.pose.position.y == raw.pose.pose.position.y
        initial = statuses[-1]
        assert not initial.correction_valid
        assert initial.revision == 0

        assert client.wait_for_service(timeout_sec=2.0)
        request = correction_services.SetCorrection.Request()
        request.job_id = "integration-job"
        request.odin_session_id = initial.odin_session_id
        request.expected_revision = initial.revision
        request.valid = True
        request.correction_x_m = 10.0
        request.correction_y_m = -3.0
        request.correction_yaw_rad = math.pi / 2.0
        request.sample_count = 24
        request.position_std_m = 0.01
        request.yaw_std_rad = math.radians(0.1)
        request.reprojection_error_px = 0.5
        request.odom_match_error_ms = 3.0
        response = client.call_async(request)
        _wait_for(response.done)
        ack = response.result()
        assert ack.accepted and ack.applied and ack.revision == 1

        raw.header.stamp.nanosec = 100_000_000
        publisher.publish(raw)
        _wait_for(
            lambda: (
                corrected
                and math.isclose(corrected[-1].pose.pose.position.x, 8.0, abs_tol=1e-9)
            )
        )
        assert math.isclose(corrected[-1].pose.pose.position.y, -2.0, abs_tol=1e-9)
        assert corrected[-1].pose.pose.position.z == 0.7
        _wait_for(lambda: statuses[-1].correction_valid)
        assert statuses[-1].applied_job_id == "integration-job"

        # 没有 correction_service 节点参与，继续发布仍使用 extnav 自己保存的修正。
        raw.header.stamp.nanosec = 200_000_000
        publisher.publish(raw)
        _wait_for(lambda: statuses[-1].corrected_messages >= 7)
        assert math.isclose(corrected[-1].pose.pose.position.x, 8.0, abs_tol=1e-9)

        # 超过 session gap 后旧修正必须先失效，随后新样本 identity 透传。
        _wait_for(
            lambda: (
                statuses
                and not statuses[-1].odin_available
                and not statuses[-1].correction_valid
            ),
            timeout=4.0,
        )
        invalidated_revision = statuses[-1].revision
        assert invalidated_revision == 2
        raw.header.stamp.sec = 200
        raw.header.stamp.nanosec = 0
        publisher.publish(raw)
        _wait_for(
            lambda: (
                statuses[-1].odin_available
                and statuses[-1].odin_session_id != initial.odin_session_id
            )
        )
        _wait_for(
            lambda: math.isclose(
                corrected[-1].pose.pose.position.x,
                raw.pose.pose.position.x,
                abs_tol=1e-9,
            )
        )
        assert not statuses[-1].correction_valid
        assert statuses[-1].revision == invalidated_revision
        assert statuses[-1].raw_messages == statuses[-1].corrected_messages
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(tester)
        executor.remove_node(bridge)
        tester.destroy_node()
        bridge.destroy_node()
        context.shutdown()
