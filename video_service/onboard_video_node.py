#!/usr/bin/env python3
"""独立机载 ROS 视频节点；摄像头故障不进入 onboard_control 生命周期。"""

from __future__ import annotations

import queue
import threading
from collections import deque
from typing import Any

from camera_app.config import DEFAULT_MEDIAMTX_BINARY, CameraConfig
from camera_app.controller import CameraController, CameraServiceError
from camera_app.onboard_config import OnboardVideoSettings, load_onboard_settings

from guided_interfaces.msg import (
    VideoCapture,
    VideoCaptureResult,
    VideoControl,
    VideoStatus,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


VIDEO_INTERFACE_VERSION = "3.2"
VIDEO_PREFIX = "/video_service"


class OnboardVideoNode(Node):
    """把可靠 ROS 期望状态和逐条抓拍命令映射到现有视频控制器。"""

    def __init__(
        self,
        *,
        controller: CameraController | None = None,
        context: Any | None = None,
    ) -> None:
        super().__init__("video_service_node", context=context)
        self._settings: OnboardVideoSettings | None = None
        self._configuration_error = ""
        try:
            self._settings = load_onboard_settings()
        except (OSError, ValueError) as exc:
            self._configuration_error = str(exc)

        initial = self._settings.camera if self._settings else CameraConfig.defaults()
        self._controller = controller or CameraController(
            initial_config=initial,
            mediamtx_binary=(
                self._settings.mediamtx_binary
                if self._settings
                else DEFAULT_MEDIAMTX_BINARY
            ),
            lens_config_path=(self._settings.lens_config if self._settings else None),
            persist_config=False,
            strict_mode_probe=False,
        )
        self._shutdown = threading.Event()
        self._control_queue: queue.Queue[VideoControl] = queue.Queue(maxsize=64)
        self._capture_queue: queue.Queue[VideoCapture] = queue.Queue(maxsize=256)
        self._seen_capture_keys: set[tuple[str, int]] = set()
        self._seen_capture_order: deque[tuple[str, int]] = deque(maxlen=1024)

        transient_status = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_events = QoSProfile(
            depth=256,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            VideoStatus, f"{VIDEO_PREFIX}/status", transient_status
        )
        self._capture_result_publisher = self.create_publisher(
            VideoCaptureResult, f"{VIDEO_PREFIX}/capture_result", reliable_events
        )
        self._control_subscription = self.create_subscription(
            VideoControl,
            f"{VIDEO_PREFIX}/control",
            self._on_control,
            transient_status,
        )
        self._capture_subscription = self.create_subscription(
            VideoCapture,
            f"{VIDEO_PREFIX}/capture",
            self._on_capture,
            reliable_events,
        )

        status_period = self._settings.status_period_seconds if self._settings else 1.0
        self._status_timer = self.create_timer(status_period, self._publish_status)
        self._control_worker = threading.Thread(
            target=self._control_loop,
            name="onboard-video-control",
            daemon=True,
        )
        self._capture_worker = threading.Thread(
            target=self._capture_loop,
            name="onboard-video-capture",
            daemon=True,
        )
        self._control_worker.start()
        self._capture_worker.start()
        self._publish_status()
        self.get_logger().info("独立机载视频服务已启动；飞控不会等待本节点")

    def _on_control(self, message: VideoControl) -> None:
        """回调只排队最新期望状态，不执行设备或外部进程 I/O。"""
        try:
            self._control_queue.put_nowait(message)
        except queue.Full:
            try:
                self._control_queue.get_nowait()
            except queue.Empty:
                pass
            self._control_queue.put_nowait(message)

    def _on_capture(self, message: VideoCapture) -> None:
        """每条截图命令独立排队；队列满也逐条发布明确失败结果。"""
        key = (str(message.source_id), int(message.sequence))
        if key in self._seen_capture_keys:
            return
        if len(self._seen_capture_order) == self._seen_capture_order.maxlen:
            oldest = self._seen_capture_order.popleft()
            self._seen_capture_keys.discard(oldest)
        self._seen_capture_order.append(key)
        self._seen_capture_keys.add(key)
        try:
            self._capture_queue.put_nowait(message)
        except queue.Full:
            self._publish_capture_result(message, False, "", "抓拍队列已满")

    def _control_loop(self) -> None:
        """串行执行启停；失败仅进入 VideoStatus，不结束 ROS 节点。"""
        while not self._shutdown.is_set():
            try:
                command = self._control_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            # VideoControl 表示最新期望状态，可以合并积压；与必须逐条完成的
            # VideoCapture 不同，避免“刚点关闭却先完成一次昂贵启动”。
            while True:
                try:
                    command = self._control_queue.get_nowait()
                except queue.Empty:
                    break
            try:
                if command.enabled:
                    settings = load_onboard_settings()
                    self._settings = settings
                    self._configuration_error = ""
                    self._controller.mediamtx_binary = settings.mediamtx_binary
                    self._controller.lens_config_path = settings.lens_config
                    self._controller.start(settings.camera.to_dict())
                else:
                    self._controller.stop()
            except (CameraServiceError, OSError, ValueError) as exc:
                self._configuration_error = str(exc)
                self.get_logger().error(f"视频状态切换失败：{exc}")
            self._publish_status()

    def _capture_loop(self) -> None:
        """逐条完成抓拍并逐条回报，禁止使用会合并事件的 SIGUSR1 路径。"""
        while not self._shutdown.is_set():
            try:
                command = self._capture_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            kind = {
                VideoCapture.KIND_MANUAL: "manual",
                VideoCapture.KIND_GCS_WAYPOINT: "gcs",
                VideoCapture.KIND_UPSTREAM_WAYPOINT: "upstream",
            }.get(int(command.kind), "")
            if not kind:
                self._publish_capture_result(command, False, "", "未知抓拍来源")
                continue
            try:
                result = self._controller.request_snapshot(
                    kind=kind, photo_no=str(command.photo_no)
                )
            except CameraServiceError as exc:
                self._publish_capture_result(command, False, "", str(exc))
            else:
                self._publish_capture_result(
                    command, True, str(result["path"]), "JPG 抓拍完成"
                )
            self._publish_status()

    def _publish_capture_result(
        self,
        command: VideoCapture,
        success: bool,
        path: str,
        message: str,
    ) -> None:
        """完整回显关联字段，使地面端可处理乱序完成事件。"""
        result = VideoCaptureResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.source_id = command.source_id
        result.sequence = command.sequence
        result.success = bool(success)
        result.kind = command.kind
        result.mission_sequence = command.mission_sequence
        result.waypoint_index = command.waypoint_index
        result.photo_no = command.photo_no
        result.path = path
        result.message = message
        self._capture_result_publisher.publish(result)

    def _publish_status(self) -> None:
        """周期发布 RTSP 发现与媒体路径；错误信息不进入飞控 ControlStatus。"""
        status: dict[str, Any] = self._controller.status()
        config = status.get("config", {})
        message = VideoStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.interface_version = VIDEO_INTERFACE_VERSION
        message.service_available = True
        message.running = bool(status.get("running", False))
        message.state = str(status.get("state", "error"))
        message.rtsp_url = str(status.get("rtsp_url", ""))
        message.video_directory = str(config.get("video_directory", ""))
        message.image_directory = str(config.get("image_directory", ""))
        message.current_video_path = str(status.get("recording_file", ""))
        message.last_video_path = str(status.get("last_recording_file", ""))
        message.last_image_path = str(status.get("last_snapshot_file", ""))
        message.last_error = self._configuration_error or str(
            status.get("last_error", "")
        )
        self._status_publisher.publish(message)

    def close(self) -> None:
        """只清理本视频节点拥有的线程和媒体进程。"""
        self._shutdown.set()
        # 控制器的外部命令自身均有超时；先让工作线程退出，避免与 close()
        # 同时操作 FFmpeg/MediaMTX。超时后控制器仍会执行幂等兜底清理。
        self._control_worker.join(timeout=12.0)
        self._capture_worker.join(timeout=8.0)
        self._controller.close()


def main() -> int:
    """运行独立 ROS executor；退出不调用或停止 onboard_control。"""
    rclpy.init()
    node = OnboardVideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        # Jazzy 的默认 SIGINT handler 会先关闭全局 context；重复 shutdown 会
        # 抛 RCLError 并让一次正常的 systemd stop 被错误标记为失败。
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
