"""独立修正面板 ROS 2 客户端：权威滑窗状态与四类异步写操作。"""

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
    operation: int = 0
    expected_tag_id: int = 0
    window_size: int = 5
    apply: bool = False
    job_id: str = ""
    service_instance_id: str = ""
    window_revision: int = 0
    extnav_revision: int = 0
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
            "pose_fcu": snapshots.get("pose_fcu", {}),
            "final": snapshots.get("final", {}),
        }

    def request_start(
        self,
        operation: int,
        expected_tag_id: int,
        window_size: int,
        apply: bool,
        service_instance_id: str,
        window_revision: int,
        extnav_revision: int,
        callback: ResultCallback | None = None,
    ) -> None:
        """携带服务实例与双 revision 异步开始 first/next。"""
        self._enqueue(
            _Command(
                "start",
                operation=int(operation),
                expected_tag_id=int(expected_tag_id),
                window_size=int(window_size),
                apply=bool(apply),
                service_instance_id=str(service_instance_id),
                window_revision=int(window_revision),
                extnav_revision=int(extnav_revision),
                callback=callback,
            )
        )

    def request_stop(
        self,
        job_id: str = "",
        service_instance_id: str = "",
        callback: ResultCallback | None = None,
    ) -> None:
        """异步停止当前唯一任务；不会清除已由 extnav ACK 的修正。"""
        self._enqueue(
            _Command(
                "stop",
                job_id=str(job_id),
                service_instance_id=str(service_instance_id),
                callback=callback,
            )
        )

    def request_clear(
        self,
        service_instance_id: str,
        window_revision: int,
        callback: ResultCallback | None = None,
    ) -> None:
        """异步清空服务端窗口；该接口与 extnav clear 完全独立。"""
        self._enqueue(
            _Command(
                "clear",
                service_instance_id=str(service_instance_id),
                window_revision=int(window_revision),
                callback=callback,
            )
        )

    def request_apply_saved(
        self,
        service_instance_id: str,
        window_revision: int,
        extnav_revision: int,
        callback: ResultCallback | None = None,
    ) -> None:
        """异步应用已保存候选或幂等重试 unknown，不重新开相机。"""
        self._enqueue(
            _Command(
                "apply_saved",
                service_instance_id=str(service_instance_id),
                window_revision=int(window_revision),
                extnav_revision=int(extnav_revision),
                callback=callback,
            )
        )

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
            from correction_interfaces.srv import (
                ApplySavedCorrection,
                ClearWindow,
                StartCorrection,
                StopCorrection,
            )

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
                "/extnav/pose_fcu",
                lambda message: self._store("pose_fcu", self._pose(message)),
                qos_profile_sensor_data,
            )
            node.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                lambda message: self._store("final", self._pose(message)),
                qos_profile_sensor_data,
            )
            start_client = node.create_client(
                StartCorrection, "/correction_service/start"
            )
            stop_client = node.create_client(StopCorrection, "/correction_service/stop")
            clear_client = node.create_client(
                ClearWindow, "/correction_service/clear_window"
            )
            apply_saved_client = node.create_client(
                ApplySavedCorrection, "/correction_service/apply_saved"
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            while not self._stop_requested.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
                while True:
                    try:
                        command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    clients = {
                        "start": start_client,
                        "stop": stop_client,
                        "clear": clear_client,
                        "apply_saved": apply_saved_client,
                    }
                    client = clients[command.kind]
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
                        request.operation = command.operation
                        request.expected_tag_id = command.expected_tag_id
                        request.window_size = command.window_size
                        request.expected_service_instance_id = (
                            command.service_instance_id
                        )
                        request.expected_window_revision = command.window_revision
                        request.expected_extnav_revision = command.extnav_revision
                        request.apply = command.apply
                    elif command.kind == "stop":
                        request = StopCorrection.Request()
                        request.job_id = command.job_id
                        request.expected_service_instance_id = (
                            command.service_instance_id
                        )
                    elif command.kind == "clear":
                        request = ClearWindow.Request()
                        request.expected_service_instance_id = (
                            command.service_instance_id
                        )
                        request.expected_window_revision = command.window_revision
                    else:
                        request = ApplySavedCorrection.Request()
                        request.expected_service_instance_id = (
                            command.service_instance_id
                        )
                        request.expected_window_revision = command.window_revision
                        request.expected_extnav_revision = command.extnav_revision
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
                        except Exception as exc:  # noqa: BLE001 - future 可透传服务异常
                            self._callback(callback, {}, str(exc))
                            return
                        payload = {
                            "accepted": bool(response.accepted),
                            "message": str(response.message),
                        }
                        if kind in ("start", "apply_saved"):
                            payload["job_id"] = str(response.job_id)
                        if hasattr(response, "service_instance_id"):
                            payload["service_instance_id"] = str(
                                response.service_instance_id
                            )
                        if hasattr(response, "window_revision"):
                            payload["window_revision"] = int(response.window_revision)
                        error = "" if response.accepted else str(response.message)
                        self._callback(callback, payload, error)

                    future.add_done_callback(completed)
        except Exception as exc:  # noqa: BLE001 - 后台 ROS 线程需向 UI 暴露启动失败
            with self._lock:
                self._startup_error = f"修正面板 ROS 客户端启动失败：{exc}"
        finally:
            if executor is not None and node is not None:
                try:
                    executor.remove_node(node)
                except Exception:  # noqa: BLE001, S110 - 尽力清理，保留原启动错误
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:  # noqa: BLE001, S110 - 尽力清理，保留原启动错误
                    pass
            if context is not None:
                try:
                    context.shutdown()
                except Exception:  # noqa: BLE001, S110 - 尽力清理，保留原启动错误
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
            "interface_version": str(message.interface_version),
            "service_available": bool(message.service_available),
            "service_instance_id": str(message.service_instance_id),
            "window_revision": int(message.window_revision),
            "config_fingerprint": str(message.config_fingerprint),
            "active": bool(message.active),
            "state": str(message.state_text),
            "operation": str(message.operation_text),
            "job_id": str(message.job_id),
            "expected_tag_id": int(message.expected_tag_id),
            "detected_tag_id": int(message.detected_tag_id),
            "apply_requested": bool(message.apply_requested),
            "session": str(message.odin_session_id),
            "base_revision": int(message.base_revision),
            "window_size": int(message.window_size),
            "window_count": int(message.window_count),
            "success_count": int(message.successful_calibrations),
            "next_calibration": int(message.next_calibration_number),
            "window_state": str(message.window_state),
            "window_message": str(message.window_message),
            "keyframes": [
                {
                    "tag_id": int(item.tag_id),
                    "p_x_m": float(item.world_x_m),
                    "p_y_m": float(item.world_y_m),
                    "q_x_m": float(item.odin_x_m),
                    "q_y_m": float(item.odin_y_m),
                    "samples": int(item.samples_accepted),
                    "rejected": int(item.samples_rejected),
                    "blocks": int(item.independent_blocks),
                    "span_s": float(item.span_s),
                    "position_std_m": float(item.position_std_m),
                    "sigma_m": float(item.effective_sigma_m),
                    "weight": float(item.weight),
                    "reprojection_px": float(item.reprojection_error_px),
                    "match_ms": float(item.odom_match_error_ms),
                    "sync_motion_m": float(item.max_sync_motion_error_m),
                    "time_source": str(item.odom_time_source),
                }
                for item in message.keyframes
            ],
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
            "candidate_saved": bool(message.candidate_saved),
            "candidate_valid": bool(message.candidate_valid),
            "candidate_stale": bool(message.candidate_stale),
            "candidate_stage": str(message.candidate_stage),
            "candidate_window_revision": int(message.candidate_window_revision),
            "candidate_extnav_revision": int(message.candidate_extnav_revision),
            "baseline_tag_a": int(message.baseline_tag_a),
            "baseline_tag_b": int(message.baseline_tag_b),
            "odin_baseline_m": float(message.odin_baseline_m),
            "world_baseline_m": float(message.world_baseline_m),
            "odin_direction_deg": float(message.odin_baseline_direction_deg),
            "world_direction_deg": float(message.world_baseline_direction_deg),
            "registration_rms_m": float(message.registration_rms_m),
            "registration_max_m": float(message.registration_max_residual_m),
            "model_yaw_std_deg": float(message.model_yaw_std_deg),
            "two_point_limited": bool(message.two_point_limited),
            "delta_x_m": float(message.delta_x_m),
            "delta_y_m": float(message.delta_y_m),
            "delta_yaw_deg": float(message.delta_yaw_deg),
            "position_jump_m": float(message.expected_position_jump_m),
            "yaw_jump_deg": float(message.expected_yaw_jump_deg),
            "can_apply": bool(message.can_apply),
            "apply_reason": str(message.apply_reason),
            "application_state": str(message.application_state_text),
            "application_confirmed_by_status": bool(
                message.application_confirmed_by_status
            ),
            "window_saved": bool(message.window_saved),
            "resources_released": bool(message.resources_released),
            "extnav_applied": bool(message.extnav_applied),
            "revision": int(message.extnav_revision),
            "message": str(message.message),
            "last_error": str(message.last_error),
            "log_path": str(message.log_path),
        }

    @staticmethod
    def _extnav_status(message: Any) -> dict[str, Any]:
        """投影 extnav 权威 correction/session 状态。"""
        return {
            "interface_version": str(message.interface_version),
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
            "reference_mode": str(message.horizontal_reference_mode),
            "lever_arm_m": (
                float(message.lever_arm_x_m),
                float(message.lever_arm_y_m),
                float(message.lever_arm_z_m),
            ),
            "installation_rpy": (
                float(message.installation_roll_rad),
                float(message.installation_pitch_rad),
                float(message.installation_yaw_rad),
            ),
            "final_pose_messages": int(message.final_pose_messages),
            "final_sample_available": bool(message.final_sample_available),
            "final_sample_valid": bool(message.final_sample_correction_valid),
            "final_sample_revision": int(message.final_sample_revision),
            "final_sample_session": str(message.final_sample_odin_session_id),
            "final_sample_reference_mode": str(message.final_sample_reference_mode),
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
            "interface_version": str(message.interface_version),
            "service_instance_id": str(message.service_instance_id),
            "job_id": str(message.job_id),
            "operation": int(message.operation),
            "success": bool(message.success),
            "window_saved": bool(message.window_saved),
            "resources_released": bool(message.resources_released),
            "application_state": str(message.application_state_text),
            "confirmed_by_status": bool(message.application_confirmed_by_status),
            "applied": bool(message.applied),
            "outcome": str(message.outcome),
            "x_m": float(message.correction_x_m),
            "y_m": float(message.correction_y_m),
            "yaw_deg": float(message.correction_yaw_deg),
            "samples": int(message.samples_accepted),
            "window_revision": int(message.window_revision),
            "window_count": int(message.window_count),
            "success_count": int(message.successful_calibrations),
            "registration_rms_m": float(message.registration_rms_m),
            "model_yaw_std_deg": float(message.model_yaw_std_deg),
            "position_jump_m": float(message.expected_position_jump_m),
            "yaw_jump_deg": float(message.expected_yaw_jump_deg),
            "candidate_stage": str(message.candidate_stage),
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
        except Exception:  # noqa: BLE001, S110 - UI 回调不得终止 ROS executor
            pass
