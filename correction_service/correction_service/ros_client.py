"""独立修正面板的 ROS 2 客户端；只读状态并发送 start/stop 请求。"""

from __future__ import annotations

import math
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
class _Command:
    """跨线程交给 ROS executor 的单条校准操作。"""

    kind: str
    expected_tag_id: int = 0
    apply: bool = False
    job_id: str = ""
    callback: ResultCallback | None = None


class CorrectionPanelClient:
    """在真机 DDS 域内提供无飞行指令的修正任务控制和对照状态。"""

    STATUS_STALE_SECONDS = 3.0
    SERVICE_DISCOVERY_TIMEOUT_SECONDS = 2.0

    def __init__(self, *, domain_id: int | None = None) -> None:
        configured = os.environ.get("CORRECTION_ROS_DOMAIN_ID", "0")
        self.domain_id = int(configured) if domain_id is None else int(domain_id)
        if not 0 <= self.domain_id <= 232:
            raise ValueError("CORRECTION_ROS_DOMAIN_ID 必须位于 0～232")
        self.source_id = f"ground-correction-panel-{uuid.uuid4().hex[:12]}"
        self._lock = threading.Lock()
        self._snapshots: dict[str, dict[str, Any]] = {}
        self._received: dict[str, float] = {}
        self._startup_error = ""
        self._commands: queue.Queue[_Command] = queue.Queue(maxsize=32)
        self._stop_requested = threading.Event()
        self._thread: threading.Thread | None = None
        self._sequence = 0

    def start(self) -> None:
        """幂等启动独立 rclpy context，不复用地面站飞行控制客户端。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_requested.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="correction-panel-ros",
            daemon=True,
        )
        self._thread.start()

    def status(self) -> dict[str, Any]:
        """返回各话题快照及独立保鲜标记，供 Qt 定时读取。"""
        now = time.monotonic()
        with self._lock:
            snapshots = {key: dict(value) for key, value in self._snapshots.items()}
            received = dict(self._received)
            startup_error = self._startup_error
        for key, snapshot in snapshots.items():
            age = now - received.get(key, 0.0)
            snapshot["age_seconds"] = age
            snapshot["fresh"] = age <= self.STATUS_STALE_SECONDS
        return {
            "domain_id": self.domain_id,
            "startup_error": startup_error,
            "correction": snapshots.get("correction", {}),
            "extnav": snapshots.get("extnav", {}),
            "result": snapshots.get("result", {}),
            "raw": snapshots.get("raw", {}),
            "corrected": snapshots.get("corrected", {}),
            "final": snapshots.get("final", {}),
        }

    def request_start(
        self,
        expected_tag_id: int,
        apply: bool,
        callback: ResultCallback | None = None,
    ) -> None:
        """异步开始 dry-run 或需要 extnav ACK 的应用任务。"""
        self._enqueue(
            _Command(
                "start",
                expected_tag_id=int(expected_tag_id),
                apply=bool(apply),
                callback=callback,
            )
        )

    def request_stop(
        self, job_id: str = "", callback: ResultCallback | None = None
    ) -> None:
        """异步停止当前唯一任务；不会清除已由 extnav ACK 的修正。"""
        self._enqueue(_Command("stop", job_id=str(job_id), callback=callback))

    def close(self) -> None:
        """只关闭面板 ROS context；不隐式停止任务或改变 active correction。"""
        self._stop_requested.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._thread = None

    def _enqueue(self, command: _Command) -> None:
        """有界排队；面板主线程永不等待 DDS。"""
        if self._thread is None or not self._thread.is_alive():
            self.start()
        try:
            self._commands.put_nowait(command)
        except queue.Full:
            self._callback(command.callback, {}, "修正命令队列已满")

    def _next_sequence(self) -> int:
        """返回面板进程内单调请求序号。"""
        self._sequence += 1
        return self._sequence

    def _run(self) -> None:
        """创建订阅/服务客户端并串行提交面板命令。"""
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )

            from correction_interfaces.msg import (
                CorrectionResult,
                CorrectionStatus,
                ExtnavCorrectionStatus,
            )
            from correction_interfaces.srv import StartCorrection, StopCorrection

            environment_keys = (
                "ROS_LOCALHOST_ONLY",
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
            )
            previous = {key: os.environ.get(key) for key in environment_keys}
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
                "ground_correction_panel_client",
                context=context,
                namespace="/",
            )
            transient = QoSProfile(
                depth=16,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(
                CorrectionStatus,
                "/correction_service/status",
                lambda message: self._store(
                    "correction", self._correction_status(message)
                ),
                transient,
            )
            node.create_subscription(
                CorrectionResult,
                "/correction_service/result",
                lambda message: self._store("result", self._result(message)),
                transient,
            )
            node.create_subscription(
                ExtnavCorrectionStatus,
                "/extnav/correction_status",
                lambda message: self._store("extnav", self._extnav_status(message)),
                transient,
            )
            node.create_subscription(
                Odometry,
                "/odin1/odometry_highfreq",
                lambda message: self._store("raw", self._odometry(message)),
                qos_profile_sensor_data,
            )
            node.create_subscription(
                Odometry,
                "/odin1/odometry_highfreq_corrected",
                lambda message: self._store("corrected", self._odometry(message)),
                qos_profile_sensor_data,
            )
            node.create_subscription(
                PoseStamped,
                "/mavros/vision_pose/pose",
                lambda message: self._store("final", self._pose(message)),
                qos_profile_sensor_data,
            )
            start_client = node.create_client(
                StartCorrection, "/correction_service/start"
            )
            stop_client = node.create_client(StopCorrection, "/correction_service/stop")
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            while not self._stop_requested.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
                while True:
                    try:
                        command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    client = start_client if command.kind == "start" else stop_client
                    if not client.wait_for_service(
                        timeout_sec=self.SERVICE_DISCOVERY_TIMEOUT_SECONDS
                    ):
                        self._callback(
                            command.callback,
                            {},
                            f"未发现 correction_service {command.kind} 接口",
                        )
                        continue
                    if command.kind == "start":
                        request = StartCorrection.Request()
                        request.expected_tag_id = command.expected_tag_id
                        request.apply = command.apply
                    else:
                        request = StopCorrection.Request()
                        request.job_id = command.job_id
                    request.stamp = node.get_clock().now().to_msg()
                    request.source_id = self.source_id
                    request.sequence = self._next_sequence()
                    future = client.call_async(request)

                    def completed(
                        done: Any,
                        callback: ResultCallback | None = command.callback,
                        kind: str = command.kind,
                    ) -> None:
                        try:
                            response = done.result()
                        except Exception as exc:
                            self._callback(callback, {}, str(exc))
                            return
                        payload = {
                            "accepted": bool(response.accepted),
                            "message": str(response.message),
                        }
                        if kind == "start":
                            payload["job_id"] = str(response.job_id)
                        error = "" if response.accepted else str(response.message)
                        self._callback(callback, payload, error)

                    future.add_done_callback(completed)
        except Exception as exc:
            with self._lock:
                self._startup_error = f"修正面板 ROS 客户端启动失败：{exc}"
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

    def _store(self, key: str, snapshot: dict[str, Any]) -> None:
        """原子替换某类话题的最新纯数据快照。"""
        with self._lock:
            self._snapshots[key] = snapshot
            self._received[key] = time.monotonic()
            self._startup_error = ""

    @staticmethod
    def _correction_status(message: Any) -> dict[str, Any]:
        """投影修正服务状态为 Qt 无关字典。"""
        return {
            "service_available": bool(message.service_available),
            "active": bool(message.active),
            "state": str(message.state_text),
            "job_id": str(message.job_id),
            "expected_tag_id": int(message.expected_tag_id),
            "detected_tag_id": int(message.detected_tag_id),
            "apply_requested": bool(message.apply_requested),
            "session": str(message.odin_session_id),
            "frames_received": int(message.frames_received),
            "frames_processed": int(message.frames_processed),
            "detections": int(message.detections_total),
            "samples": int(message.samples_accepted),
            "rejected": int(message.samples_rejected),
            "elapsed_s": float(message.elapsed_s),
            "x_m": float(message.candidate_x_m),
            "y_m": float(message.candidate_y_m),
            "yaw_deg": float(message.candidate_yaw_deg),
            "tilt_deg": float(message.correction_tilt_deg),
            "position_std_m": float(message.position_std_m),
            "yaw_std_deg": float(message.yaw_std_deg),
            "reprojection_px": float(message.reprojection_error_px),
            "odom_match_ms": float(message.odom_match_error_ms),
            "odom_time_source": str(message.odom_time_source),
            "processing_rate_hz": float(message.processing_rate_hz),
            "processing_time_ms": float(message.processing_time_ms),
            "converged": bool(message.converged),
            "extnav_applied": bool(message.extnav_applied),
            "revision": int(message.extnav_revision),
            "message": str(message.message),
            "last_error": str(message.last_error),
        }

    @staticmethod
    def _extnav_status(message: Any) -> dict[str, Any]:
        """投影 extnav 权威 correction/session 状态。"""
        return {
            "service_available": bool(message.service_available),
            "odin_available": bool(message.odin_available),
            "session": str(message.odin_session_id),
            "valid": bool(message.correction_valid),
            "revision": int(message.revision),
            "reset_counter": int(message.reset_counter),
            "x_m": float(message.correction_x_m),
            "y_m": float(message.correction_y_m),
            "yaw_deg": float(message.correction_yaw_deg),
            "job_id": str(message.applied_job_id),
            "raw_age_s": float(message.raw_age_s),
            "raw_messages": int(message.raw_messages),
            "corrected_messages": int(message.corrected_messages),
            "last_event": str(message.last_event),
            "last_error": str(message.last_error),
        }

    @staticmethod
    def _result(message: Any) -> dict[str, Any]:
        """投影最近一次可靠任务终态。"""
        return {
            "job_id": str(message.job_id),
            "success": bool(message.success),
            "applied": bool(message.applied),
            "outcome": str(message.outcome),
            "x_m": float(message.correction_x_m),
            "y_m": float(message.correction_y_m),
            "yaw_deg": float(message.correction_yaw_deg),
            "samples": int(message.samples_accepted),
            "duration_s": float(message.duration_s),
            "message": str(message.message),
            "log_path": str(message.log_path),
        }

    @classmethod
    def _odometry(cls, message: Any) -> dict[str, Any]:
        """提取 Odin 原始/修正里程计位置和 yaw。"""
        return cls._pose_values(message.pose.pose)

    @classmethod
    def _pose(cls, message: Any) -> dict[str, Any]:
        """提取实际送入 MAVROS 话题的位置和 yaw。"""
        return cls._pose_values(message.pose)

    @staticmethod
    def _pose_values(pose: Any) -> dict[str, Any]:
        """把 ROS pose 转成便于对照的有限标量。"""
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            yaw_deg = math.nan
        else:
            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            yaw_deg = math.degrees(
                math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            )
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "z_m": float(pose.position.z),
            "yaw_deg": yaw_deg,
        }

    @staticmethod
    def _callback(
        callback: ResultCallback | None,
        payload: dict[str, Any],
        error: str,
    ) -> None:
        """隔离 UI 回调异常，避免杀死 ROS executor。"""
        if callback is None:
            return
        try:
            callback(payload, error)
        except Exception:
            pass
