"""独立面板使用的机载视频 ROS 客户端，不接入飞行控制会话。"""

from __future__ import annotations

import os
import queue
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


ResultCallback = Callable[[dict[str, Any], str], None]


@dataclass(frozen=True)
class _QueuedCommand:
    """跨线程交给 ROS executor 的一条视频操作。"""

    kind: str
    enabled: bool = False
    callback: ResultCallback | None = None


class OnboardVideoClient:
    """在显式 ROS 域中订阅视频状态并异步发送纯视频命令。"""

    STATUS_STALE_SECONDS = 3.0

    def __init__(self, *, domain_id: int | None = None) -> None:
        configured_domain = os.environ.get("VIDEO_SERVICE_ROS_DOMAIN_ID", "0")
        self.domain_id = int(configured_domain) if domain_id is None else int(domain_id)
        if not 0 <= self.domain_id <= 232:
            raise ValueError("VIDEO_SERVICE_ROS_DOMAIN_ID 必须位于 0～232")
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {}
        self._status_received_monotonic = 0.0
        self._startup_error = ""
        self._commands: queue.Queue[_QueuedCommand] = queue.Queue(maxsize=64)
        self._stop_requested = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._sequence = 0
        # 每次面板进程使用独立来源标识；否则面板重启后序号重新从 1 开始，
        # 会被机载节点的重放保护误判为旧命令。
        self.source_id = f"ground-camera-panel-{uuid.uuid4().hex[:12]}"

    def start(self) -> None:
        """幂等启动独立 rclpy context；默认使用真机域 0 和子网发现。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_requested.clear()
        self._ready.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="camera-panel-onboard-ros",
            daemon=True,
        )
        self._thread.start()

    def status(self) -> dict[str, Any]:
        """返回带接收时钟保鲜判断的视频服务快照。"""
        with self._lock:
            snapshot = dict(self._status)
            received = self._status_received_monotonic
            startup_error = self._startup_error
        age = time.monotonic() - received if received else float("inf")
        if age > self.STATUS_STALE_SECONDS:
            snapshot.update(
                service_available=False,
                running=False,
                state="stale" if received else "unavailable",
            )
            if startup_error:
                snapshot["last_error"] = startup_error
        snapshot["age_seconds"] = age
        return snapshot

    def request_state(
        self, enabled: bool, callback: ResultCallback | None = None
    ) -> None:
        """异步请求独立视频节点切换最新视频期望状态。"""
        self._enqueue(_QueuedCommand("state", bool(enabled), callback))

    def request_snapshot(self, callback: ResultCallback | None = None) -> None:
        """可靠发布一条不持久化的人工抓拍事件。"""
        self._enqueue(_QueuedCommand("snapshot", callback=callback))

    def close(self) -> None:
        """停止客户端 context；绝不发送视频关闭或飞行控制命令。"""
        self._stop_requested.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._thread = None

    def _enqueue(self, command: _QueuedCommand) -> None:
        """只做有界内存排队，Qt 主线程不会等待 DDS。"""
        if self._thread is None or not self._thread.is_alive():
            self.start()
        try:
            self._commands.put_nowait(command)
        except queue.Full:
            if command.callback is not None:
                command.callback({}, "机载视频命令队列已满")

    def _next_sequence(self) -> int:
        """生成面板进程内单调的纯视频请求序号。"""
        self._sequence += 1
        return self._sequence

    def _run(self) -> None:
        """拥有 ROS context、节点与 executor 的唯一线程。"""
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from guided_interfaces.msg import VideoCapture, VideoStatus
            from guided_interfaces.srv import SetVideoState
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            # Detached 面板可能继承 SITL 的 LOCALHOST/domain 环境；只在创建本
            # context 时覆盖，确保真机模式默认发现域 0 子网节点。
            environment_keys = (
                "ROS_LOCALHOST_ONLY",
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
            )
            previous = {key: os.environ.get(key) for key in environment_keys}
            # Jazzy 已用 discovery range 取代 ROS_LOCALHOST_ONLY；删除继承的
            # LOCALHOST_ONLY=1 即可解除 SITL 隔离，同时避免每次打开面板都告警。
            os.environ.pop("ROS_LOCALHOST_ONLY", None)
            os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
            try:
                context = Context()
                rclpy.init(args=[], context=context, domain_id=self.domain_id)
            finally:
                for key, value in previous.items():
                    if value is None:
                        os.environ.pop(key, None)
                    else:
                        os.environ[key] = value

            node = rclpy.create_node(
                "ground_camera_panel_video_client",
                context=context,
                namespace="/",
            )
            transient_status = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            reliable_events = QoSProfile(
                depth=256,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            node.create_subscription(
                VideoStatus,
                "/video_service/status",
                self._ingest_status,
                transient_status,
            )
            capture_publisher = node.create_publisher(
                VideoCapture,
                "/video_service/capture",
                reliable_events,
            )
            state_client = node.create_client(
                SetVideoState,
                "/video_service/set_video_state",
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._ready.set()

            while not self._stop_requested.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
                while True:
                    try:
                        command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    if command.kind == "snapshot":
                        message = VideoCapture()
                        message.header.stamp = node.get_clock().now().to_msg()
                        message.source_id = self.source_id
                        message.sequence = self._next_sequence()
                        message.kind = VideoCapture.KIND_MANUAL
                        message.mission_sequence = 0
                        message.waypoint_index = 0
                        message.photo_no = ""
                        capture_publisher.publish(message)
                        self._callback(
                            command.callback,
                            {"published": True, "sequence": int(message.sequence)},
                            "",
                        )
                        continue
                    if not state_client.service_is_ready():
                        self._callback(
                            command.callback,
                            {},
                            "独立机载视频状态接口尚未发现",
                        )
                        continue
                    request = SetVideoState.Request()
                    request.stamp = node.get_clock().now().to_msg()
                    request.source_id = self.source_id
                    request.sequence = self._next_sequence()
                    request.ttl_ms = 5000
                    request.enabled = command.enabled
                    future = state_client.call_async(request)

                    def completed(done: Any, callback: ResultCallback | None = command.callback) -> None:
                        try:
                            response = done.result()
                        except Exception as exc:  # rclpy future surfaces transport errors here.
                            self._callback(callback, {}, str(exc))
                            return
                        error = "" if response.accepted else str(response.message)
                        self._callback(
                            callback,
                            {
                                "accepted": bool(response.accepted),
                                "message": str(response.message),
                            },
                            error,
                        )

                    future.add_done_callback(completed)
        except Exception as exc:
            with self._lock:
                self._startup_error = f"机载视频 ROS 客户端启动失败：{exc}"
            self._ready.set()
        finally:
            if executor is not None and node is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    context.shutdown()
                except Exception:
                    pass

    def _ingest_status(self, message: Any) -> None:
        """只缓存视频状态；不修改面板地址输入或任何飞行状态。"""
        snapshot = {
            "interface_version": str(message.interface_version),
            "service_available": bool(message.service_available),
            "running": bool(message.running),
            "state": str(message.state),
            "rtsp_url": str(message.rtsp_url),
            "video_directory": str(message.video_directory),
            "image_directory": str(message.image_directory),
            "current_video_path": str(message.current_video_path),
            "last_video_path": str(message.last_video_path),
            "last_image_path": str(message.last_image_path),
            "last_error": str(message.last_error),
        }
        with self._lock:
            self._status = snapshot
            self._status_received_monotonic = time.monotonic()
            self._startup_error = ""

    @staticmethod
    def _callback(
        callback: ResultCallback | None,
        result: dict[str, Any],
        error: str,
    ) -> None:
        """隔离 UI 回调异常，避免终止 ROS executor。"""
        if callback is None:
            return
        try:
            callback(result, error)
        except Exception:
            pass
