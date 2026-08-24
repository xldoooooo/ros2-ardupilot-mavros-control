"""独立机载视频节点的隔离 ROS 域与逐条抓拍回归测试。"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _FakeCameraController:
    """只模拟媒体结果，不打开设备、端口或任何真实飞行链路。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.running = False
        self.start_calls = 0
        self.stop_calls = 0
        self.capture_calls: list[tuple[str, str]] = []
        self.closed = False
        self.mediamtx_binary = None
        self.lens_config_path = None

    def start(self, _config) -> dict[str, object]:
        with self._lock:
            self.start_calls += 1
            self.running = True
        return self.status()

    def stop(self) -> dict[str, object]:
        with self._lock:
            self.stop_calls += 1
            self.running = False
        return self.status()

    def request_snapshot(self, *, kind: str, photo_no: str) -> dict[str, str]:
        with self._lock:
            self.capture_calls.append((kind, photo_no))
            position = len(self.capture_calls)
        if photo_no == "fail":
            from camera_app.controller import CameraServiceError

            raise CameraServiceError("模拟抓拍失败")
        return {"path": f"/home/share/jpg/{kind}-{position}.jpg"}

    def status(self) -> dict[str, object]:
        with self._lock:
            running = self.running
        return {
            "state": "running" if running else "stopped",
            "running": running,
            "rtsp_url": "rtsp://aircraft.test:8554/camera",
            "recording_file": "/home/share/current.mp4" if running else "",
            "last_recording_file": "/home/share/last.mp4" if not running else "",
            "last_snapshot_file": "",
            "last_error": "",
            "config": {
                "video_directory": "/home/share",
                "image_directory": "/home/share/jpg",
            },
        }

    def close(self) -> None:
        self.closed = True
        self.running = False


def test_video_node_process_exits_cleanly_on_sigint() -> None:
    """Jazzy signal handler 已关闭 context 时，主函数不得重复 shutdown 后报错。"""
    pytest.importorskip("rclpy")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "226"
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    environment["PYTHONUNBUFFERED"] = "1"
    process = subprocess.Popen(
        [sys.executable, str(PROJECT_ROOT / "video_service" / "onboard_video_node.py")],
        cwd=PROJECT_ROOT / "video_service",
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    try:
        time.sleep(1.0)
        assert process.poll() is None
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=8.0)
        assert process.returncode == 0, output
        assert "独立机载视频服务已启动" in output
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3.0)


def test_video_node_processes_every_capture_and_survives_media_failure() -> None:
    """volatile 抓拍逐条回报，单条失败不结束节点或污染后续命令。"""
    rclpy = pytest.importorskip("rclpy")
    from onboard_video_node import OnboardVideoNode
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from guided_interfaces.msg import (
        VideoCapture,
        VideoCaptureResult,
        VideoControl,
        VideoStatus,
    )
    from guided_interfaces.srv import SetVideoState

    context = Context()
    rclpy.init(args=[], context=context, domain_id=229)
    fake = _FakeCameraController()
    video_node = OnboardVideoNode(controller=fake, context=context)
    probe = rclpy.create_node("video_service_test_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(video_node)
    executor.add_node(probe)

    transient = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    events = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
    controls = probe.create_publisher(VideoControl, "/video_service/control", transient)
    states = probe.create_client(SetVideoState, "/video_service/set_video_state")
    captures = probe.create_publisher(VideoCapture, "/video_service/capture", events)
    results: list[VideoCaptureResult] = []
    statuses: list[VideoStatus] = []
    probe.create_subscription(
        VideoCaptureResult,
        "/video_service/capture_result",
        results.append,
        events,
    )
    probe.create_subscription(
        VideoStatus,
        "/video_service/status",
        statuses.append,
        transient,
    )

    def spin_until(predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    try:
        assert spin_until(lambda: bool(statuses) and states.service_is_ready())
        start = SetVideoState.Request()
        start.stamp = probe.get_clock().now().to_msg()
        # 模拟飞机无外网且绝对墙钟比地面端慢 30 天。
        start.stamp.sec += 30 * 24 * 60 * 60
        start.source_id = "pytest-direct-video"
        start.sequence = 1
        start.ttl_ms = 5000
        start.enabled = True
        start_future = states.call_async(start)
        assert spin_until(start_future.done)
        assert start_future.result().accepted
        assert "已排队" in start_future.result().message
        assert spin_until(lambda: fake.start_calls == 1)
        assert spin_until(lambda: statuses and statuses[-1].running)

        duplicate_future = states.call_async(start)
        assert spin_until(duplicate_future.done)
        assert not duplicate_future.result().accepted
        assert "重复或乱序" in duplicate_future.result().message
        assert fake.start_calls == 1

        stale = SetVideoState.Request()
        stale.stamp = probe.get_clock().now().to_msg()
        stale.stamp.sec += 30 * 24 * 60 * 60 - 30
        stale.source_id = start.source_id
        stale.sequence = 2
        stale.ttl_ms = 100
        stale.enabled = False
        stale_future = states.call_async(stale)
        assert spin_until(stale_future.done)
        assert not stale_future.result().accepted
        assert stale_future.result().message == "视频命令已过期"
        assert fake.stop_calls == 0

        for sequence, photo_no in enumerate(("A", "fail", "C"), start=10):
            message = VideoCapture()
            message.source_id = "pytest"
            message.sequence = sequence
            message.kind = VideoCapture.KIND_UPSTREAM_WAYPOINT
            message.mission_sequence = 77
            message.waypoint_index = sequence - 9
            message.photo_no = photo_no
            captures.publish(message)
        assert spin_until(lambda: len(results) == 3)
        assert [item.sequence for item in results] == [10, 11, 12]
        assert [item.success for item in results] == [True, False, True]
        assert [item.waypoint_index for item in results] == [1, 2, 3]
        assert fake.capture_calls == [
            ("upstream", "A"),
            ("upstream", "fail"),
            ("upstream", "C"),
        ]
        assert "模拟抓拍失败" in results[1].message

        stop = VideoControl()
        stop.source_id = "pytest"
        stop.sequence = 2
        stop.enabled = False
        controls.publish(stop)
        assert spin_until(lambda: fake.stop_calls == 1)
        assert spin_until(lambda: statuses and not statuses[-1].running)
        assert statuses[-1].interface_version == "3.2"
        assert statuses[-1].rtsp_url == "rtsp://aircraft.test:8554/camera"
    finally:
        video_node.close()
        executor.remove_node(video_node)
        executor.remove_node(probe)
        video_node.destroy_node()
        probe.destroy_node()
        context.shutdown()
    assert fake.closed


def test_onboard_extended_state_is_the_only_onboard_video_state_path() -> None:
    """飞行节点只按 ExtendedState 发布视频状态，不暴露纯视频手动入口。"""
    rclpy = pytest.importorskip("rclpy")
    from mavros_msgs.msg import ExtendedState
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from guided_interfaces.msg import VideoControl

    executable = (
        PROJECT_ROOT
        / "install"
        / "onboard_control"
        / "lib"
        / "onboard_control"
        / "onboard_control_node"
    )
    if not executable.is_file():
        pytest.skip("onboard_control 尚未构建")

    domain_id = 228
    previous_discovery = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain_id)
    probe = rclpy.create_node("video_edge_test_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    extended_states = probe.create_publisher(
        ExtendedState, "/video_truth_mavros/extended_state", 10
    )
    transient = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    observed: list[VideoControl] = []
    probe.create_subscription(
        VideoControl, "/video_truth/control", observed.append, transient
    )
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "mavros_prefix:=/video_truth_mavros",
            "-p",
            "interface_prefix:=/video_truth_onboard",
            "-p",
            "video_prefix:=/video_truth",
            "-p",
            "fcu_parameter_check_initial_delay_seconds:=60.0",
        ],
        cwd=PROJECT_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    def spin_until(predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def publish_landed_state(value: int) -> None:
        message = ExtendedState()
        message.landed_state = value
        extended_states.publish(message)

    try:
        assert spin_until(
            lambda: probe.count_subscribers("/video_truth_mavros/extended_state") == 1
        )
        service_names = {
            name for name, _types in probe.get_service_names_and_types()
        }
        assert "/video_truth_onboard/set_video_state" not in service_names
        publish_landed_state(ExtendedState.LANDED_STATE_ON_GROUND)
        assert spin_until(lambda: len(observed) >= 1)
        publish_landed_state(ExtendedState.LANDED_STATE_IN_AIR)
        assert spin_until(lambda: len(observed) >= 2)
        publish_landed_state(ExtendedState.LANDED_STATE_ON_GROUND)
        assert spin_until(lambda: len(observed) >= 3)
        assert [message.enabled for message in observed[:3]] == [False, True, False]
        assert process.poll() is None
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3.0)
        executor.remove_node(probe)
        probe.destroy_node()
        context.shutdown()
        if previous_discovery is None:
            os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
        else:
            os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = previous_discovery


def test_panel_ros_client_discovers_status_and_closes_without_stop_command() -> None:
    """面板独立 context 可自动发现地址；close 只销毁客户端，不发布关闭。"""
    rclpy = pytest.importorskip("rclpy")
    from camera_app.ros_client import OnboardVideoClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from guided_interfaces.msg import VideoCapture, VideoStatus
    from guided_interfaces.srv import SetVideoState

    context = Context()
    rclpy.init(args=[], context=context, domain_id=227)
    server = rclpy.create_node("panel_video_client_test_server", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    transient = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    events = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
    statuses = server.create_publisher(VideoStatus, "/video_service/status", transient)
    captures: list[VideoCapture] = []
    server.create_subscription(
        VideoCapture, "/video_service/capture", captures.append, events
    )
    state_requests: list[bool] = []

    def set_state(request, response):
        state_requests.append(bool(request.enabled))
        response.accepted = True
        response.message = "视频期望状态已发布"
        return response

    server.create_service(
        SetVideoState, "/video_service/set_video_state", set_state
    )
    client = OnboardVideoClient(domain_id=227)
    client.start()

    def spin_until(predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    try:
        message = VideoStatus()
        message.interface_version = "3.2"
        message.service_available = True
        message.running = True
        message.state = "running"
        message.rtsp_url = "rtsp://aircraft.test:8554/camera"
        message.video_directory = "/home/share"
        message.image_directory = "/home/share/jpg"
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not client.status().get(
            "service_available"
        ):
            statuses.publish(message)
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.02)
        assert client.status()["rtsp_url"] == message.rtsp_url
        assert spin_until(
            lambda: server.count_clients("/video_service/set_video_state") == 1
        )

        state_done = threading.Event()
        state_errors: list[str] = []
        client.request_state(
            True,
            lambda _result, error: (
                state_errors.append(error),
                state_done.set(),
            ),
        )
        assert spin_until(state_done.is_set)
        assert state_errors == [""]
        assert state_requests == [True]

        capture_done = threading.Event()
        client.request_snapshot(
            lambda _result, _error: capture_done.set()
        )
        assert spin_until(lambda: capture_done.is_set() and len(captures) == 1)
        assert captures[0].kind == VideoCapture.KIND_MANUAL
        assert captures[0].source_id == client.source_id
        assert captures[0].source_id.startswith("ground-camera-panel-")

        client.STATUS_STALE_SECONDS = 0.05
        time.sleep(0.08)
        stale = client.status()
        assert not stale["service_available"]
        assert stale["state"] == "stale"
    finally:
        client.close()
        executor.remove_node(server)
        server.destroy_node()
        context.shutdown()
    assert state_requests == [True]
