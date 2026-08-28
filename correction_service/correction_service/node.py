"""独立 ROS 2 修正节点：按命令开相机、稳健估计并由 extnav ACK 后提交终态。"""

from __future__ import annotations

import math
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image

from correction_interfaces.msg import (
    CorrectionResult,
    CorrectionStatus,
    ExtnavCorrectionStatus,
)
from correction_interfaces.srv import SetCorrection, StartCorrection, StopCorrection

from . import INTERFACE_VERSION
from .camera_process import CameraProcess
from .config import ServiceConfig, load_config
from .detector import AprilTagDetector
from .estimator import CorrectionEstimator, CorrectionSample, QualitySnapshot
from .geometry import compute_planar_correction, transform_from_pose
from .journal import JobJournal, configure_service_logger
from .synchronizer import (
    OdometrySynchronizer,
    is_apply_time_source_safe,
    stamp_to_nanoseconds,
)

STATE_TEXT = {
    CorrectionStatus.STATE_IDLE: "idle",
    CorrectionStatus.STATE_STARTING: "starting",
    CorrectionStatus.STATE_SAMPLING: "sampling",
    CorrectionStatus.STATE_CONVERGED: "converged",
    CorrectionStatus.STATE_APPLYING: "applying",
    CorrectionStatus.STATE_SUCCEEDED: "succeeded",
    CorrectionStatus.STATE_FAILED: "failed",
    CorrectionStatus.STATE_STOPPING: "stopping",
}


@dataclass
class _ExtnavSnapshot:
    """带本机接收时间的 extnav 权威 session/revision 快照。"""

    received_monotonic: float = 0.0
    service_available: bool = False
    odin_available: bool = False
    odin_session_id: str = ""
    correction_valid: bool = False
    revision: int = 0
    applied_job_id: str = ""


@dataclass
class _Job:
    """唯一活动或最近完成任务的线程安全共享状态。"""

    job_id: str
    source_id: str
    expected_tag_id: int
    apply_requested: bool
    odin_session_id: str
    base_revision: int
    started_monotonic: float
    started_ros_ns: int
    estimator: CorrectionEstimator
    journal: JobJournal
    state: int = CorrectionStatus.STATE_STARTING
    message: str = "正在启动下视相机"
    last_error: str = ""
    active: bool = True
    detected_tag_id: int = -1
    frames_received: int = 0
    frames_processed: int = 0
    detections_total: int = 0
    samples_accepted: int = 0
    samples_rejected: int = 0
    snapshot: QualitySnapshot = field(default_factory=QualitySnapshot)
    extnav_applied: bool = False
    extnav_revision: int = 0
    outcome: str = "running"
    stop_event: threading.Event = field(default_factory=threading.Event)
    user_stop: bool = False
    failure_reason: str = ""
    first_image_event: threading.Event = field(default_factory=threading.Event)
    first_expected_tag_monotonic: float = 0.0
    last_summary_monotonic: float = 0.0
    thread: threading.Thread | None = None
    camera: CameraProcess | None = None


class CorrectionServiceNode(Node):
    """与飞控/视频生命周期解耦的单任务修正服务。"""

    def __init__(self) -> None:
        super().__init__("correction_service")
        default_config = str(
            Path(get_package_share_directory("correction_service")) / "config"
        )
        self.declare_parameter("config_dir", default_config)
        self.config: ServiceConfig = load_config(
            str(self.get_parameter("config_dir").value)
        )
        if self.config.interface_version != INTERFACE_VERSION:
            raise ValueError(
                "general_settings.yaml interface_version 与源码不一致："
                f"{self.config.interface_version} != {INTERFACE_VERSION}"
            )

        self._service_logger = configure_service_logger(self.config.logging)
        self._detector = AprilTagDetector(self.config.intrinsics, self.config.detection)
        self._synchronizer = OdometrySynchronizer(self.config.synchronization)
        self._lock = threading.RLock()
        self._job: _Job | None = None
        self._image_queue: queue.Queue[Image] = queue.Queue(maxsize=1)
        self._last_sequences: dict[str, int] = {}
        self._extnav = _ExtnavSnapshot()
        self._raw_received_monotonic = 0.0
        self._raw_stamp_ns = 0
        self._local_session_id = ""
        self._odom_subscription = None
        self._odom_capture_active = False
        self._closing = threading.Event()

        transient_status = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        transient_results = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topics = self.config.topics
        self._status_pub = self.create_publisher(
            CorrectionStatus, topics["correction_status"], transient_status
        )
        self._result_pub = self.create_publisher(
            CorrectionResult, topics["correction_result"], transient_results
        )
        self._raw_odometry_topic = topics["raw_odometry"]
        self.create_subscription(
            Image,
            topics["camera_image"],
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ExtnavCorrectionStatus,
            topics["extnav_status"],
            self._on_extnav_status,
            transient_status,
        )
        self._set_client = self.create_client(
            SetCorrection, topics["set_correction_service"]
        )
        self.create_service(StartCorrection, topics["start_service"], self._on_start)
        self.create_service(StopCorrection, topics["stop_service"], self._on_stop)
        self.create_timer(0.25, self._publish_status)

        self._service_logger.info(
            "节点启动 config=%s raw_on_demand=%s image=%s",
            self.config.config_dir,
            topics["raw_odometry"],
            topics["camera_image"],
        )
        self.get_logger().info(
            "AprilTag-Odin 修正服务 1.0 已启动；默认 idle，相机保持关闭"
        )
        self._publish_status()

    def _on_odometry(self, message: Odometry) -> None:
        """缓存带 header/接收双时间轴的 Odin 历史并识别本地 session 变化。"""
        with self._lock:
            if not self._odom_capture_active:
                return
        now_monotonic = time.monotonic()
        arrival_ros_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        had_local_session = bool(self._local_session_id)
        gap = (
            now_monotonic - self._raw_received_monotonic
            if self._raw_received_monotonic > 0.0
            else 0.0
        )
        session_changed = (
            not self._local_session_id
            or gap > 2.0
            or (
                self._raw_stamp_ns > 0
                and stamp_ns > 0
                and stamp_ns < self._raw_stamp_ns - 250_000_000
            )
        )
        if session_changed:
            self._synchronizer.clear()
            self._local_session_id = f"local-{stamp_ns}-{uuid.uuid4().hex[:8]}"
            with self._lock:
                job = self._job
                if (
                    job is not None
                    and job.active
                    and had_local_session
                    and job.odin_session_id.startswith("local-")
                ):
                    self._fail_job_locked(job, "Odin session 在任务期间改变")
        self._raw_received_monotonic = now_monotonic
        self._raw_stamp_ns = stamp_ns
        self._synchronizer.add(message, arrival_ros_ns)

    def _start_odometry_capture(self) -> None:
        """为唯一任务创建高频订阅；idle 不持续消费 400 Hz Odin CPU/RSS。"""
        if self._odom_subscription is not None:
            raise RuntimeError("Odin 任务订阅尚未释放")
        self._synchronizer.clear()
        self._raw_received_monotonic = 0.0
        self._raw_stamp_ns = 0
        self._local_session_id = ""
        self._odom_capture_active = True
        try:
            self._odom_subscription = self.create_subscription(
                Odometry,
                self._raw_odometry_topic,
                self._on_odometry,
                qos_profile_sensor_data,
            )
        except Exception:
            self._odom_capture_active = False
            raise

    def _stop_odometry_capture(self) -> None:
        """先关闭回调门再销毁订阅和历史，避免任务间复用旧坐标系样本。"""
        with self._lock:
            self._odom_capture_active = False
            subscription = self._odom_subscription
            self._odom_subscription = None
        if subscription is not None and not self.destroy_subscription(subscription):
            self._service_logger.error("无法销毁任务 Odin odometry 订阅")
        self._synchronizer.clear()
        self._raw_received_monotonic = 0.0
        self._raw_stamp_ns = 0
        self._local_session_id = ""

    def _await_fresh_odometry(self, job: _Job) -> None:
        """开相机前有限等待任务专属订阅收到 Odin，禁止复用 idle 旧样本。"""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._raise_if_stopping(job)
            with self._lock:
                received = self._raw_received_monotonic
                local_session = self._local_session_id
            if received > 0.0 and time.monotonic() - received <= 0.5:
                if not job.odin_session_id:
                    job.odin_session_id = local_session
                job.journal.write(
                    "odometry_ready",
                    odin_session_id=job.odin_session_id,
                    local_session_id=local_session,
                )
                return
            time.sleep(0.02)
        raise RuntimeError("任务订阅后 2.0s 内没有收到新鲜 Odin odometry")

    def _on_image(self, message: Image) -> None:
        """回调只替换单槽最新帧，检测始终在任务线程中运行。"""
        with self._lock:
            job = self._job
            if job is None or not job.active:
                return
            job.frames_received += 1
            job.first_image_event.set()
        try:
            self._image_queue.put_nowait(message)
        except queue.Full:
            try:
                self._image_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._image_queue.put_nowait(message)
            except queue.Full:
                pass

    def _on_extnav_status(self, message: ExtnavCorrectionStatus) -> None:
        """缓存 extnav 权威 session；运行中变化会使候选失败而不是跨 session 应用。"""
        snapshot = _ExtnavSnapshot(
            received_monotonic=time.monotonic(),
            service_available=bool(message.service_available),
            odin_available=bool(message.odin_available),
            odin_session_id=str(message.odin_session_id),
            correction_valid=bool(message.correction_valid),
            revision=int(message.revision),
            applied_job_id=str(message.applied_job_id),
        )
        with self._lock:
            self._extnav = snapshot
            job = self._job
            if (
                job is not None
                and job.active
                and job.odin_session_id.startswith("odin-")
                and (
                    not snapshot.odin_available
                    or snapshot.odin_session_id != job.odin_session_id
                )
            ):
                self._fail_job_locked(job, "extnav 报告 Odin session 已改变或失效")

    def _accept_sequence(self, source_id: str, sequence: int) -> tuple[bool, str]:
        """对每个地面面板来源执行单调序号防重放。"""
        source = source_id.strip()
        if not source or sequence <= 0:
            return False, "source_id 不能为空且 sequence 必须大于零"
        previous = self._last_sequences.get(source, 0)
        if sequence <= previous:
            return False, f"重复或乱序请求：{sequence} <= {previous}"
        self._last_sequences[source] = sequence
        return True, ""

    def _on_start(self, request, response):
        """快速接受任务并异步开相机；同一时刻严格只有一个 job。"""
        response.accepted = False
        response.job_id = ""
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._accept_sequence(
                str(request.source_id), int(request.sequence)
            )
            if not accepted:
                response.message = reason
                return response
            current = self._job
            if current is not None and current.active:
                response.message = f"已有校准任务运行：{current.job_id}"
                return response
            expected_tag_id = int(request.expected_tag_id)
            if expected_tag_id not in self.config.tags:
                response.message = f"tag_pose.csv 未配置 Tag {expected_tag_id}"
                return response
            extnav = self._extnav
            extnav_fresh = (
                extnav.received_monotonic > 0.0
                and time.monotonic() - extnav.received_monotonic <= 2.0
                and extnav.service_available
                and extnav.odin_available
                and bool(extnav.odin_session_id)
            )
            if bool(request.apply) and not extnav_fresh:
                response.message = "apply=true 需要新鲜且可用的 extnav session/status"
                return response
            session_id = extnav.odin_session_id if extnav_fresh else ""
            base_revision = extnav.revision if extnav_fresh else 0
            job_id = uuid.uuid4().hex[:12]
            journal = JobJournal(self.config.logging.directory, job_id)
            job = _Job(
                job_id=job_id,
                source_id=str(request.source_id),
                expected_tag_id=expected_tag_id,
                apply_requested=bool(request.apply),
                odin_session_id=session_id,
                base_revision=base_revision,
                started_monotonic=time.monotonic(),
                started_ros_ns=self.get_clock().now().nanoseconds,
                estimator=CorrectionEstimator(self.config.quality),
                journal=journal,
            )
            self._job = job
            try:
                self._start_odometry_capture()
            except Exception as exc:
                self._job = current
                response.message = f"无法创建任务 Odin 订阅：{exc}"
                return response
            self._drain_image_queue()
            job.journal.write(
                "job_started",
                expected_tag_id=expected_tag_id,
                apply=job.apply_requested,
                odin_session_id=session_id,
                base_revision=base_revision,
                tag_pose=self.config.tags[expected_tag_id].__dict__,
            )
            job.thread = threading.Thread(
                target=self._run_job,
                args=(job,),
                name=f"correction-job-{job_id}",
                daemon=True,
            )
            job.thread.start()
            response.accepted = True
            response.job_id = job_id
            response.message = "校准任务已异步启动"
            self._service_logger.info(
                "任务启动 job=%s tag=%d apply=%s session=%s",
                job_id,
                expected_tag_id,
                job.apply_requested,
                session_id,
            )
        self._publish_status()
        return response

    def _on_stop(self, request, response):
        """显式停止唯一活动任务；线程负责有序释放相机并发布最终 result。"""
        response.accepted = False
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._accept_sequence(
                str(request.source_id), int(request.sequence)
            )
            if not accepted:
                response.message = reason
                return response
            job = self._job
            if job is None or not job.active:
                response.message = "当前没有活动校准任务"
                return response
            if request.job_id and str(request.job_id) != job.job_id:
                response.message = f"job_id 不匹配，当前为 {job.job_id}"
                return response
            job.user_stop = True
            job.state = CorrectionStatus.STATE_STOPPING
            job.message = "收到显式 stop，正在关闭相机并汇报结果"
            job.stop_event.set()
            response.accepted = True
            response.message = "停止请求已接受"
        self._publish_status()
        return response

    def _run_job(self, job: _Job) -> None:
        """拥有相机进程和检测循环；发布终态前必须先确认相机已释放。"""
        camera_log = job.journal.path.with_suffix(".camera.log")
        camera = CameraProcess(
            self.config.camera,
            self.config.lens_controls,
            camera_log,
            self._service_logger,
        )
        job.camera = camera
        final_success = False
        final_message = "任务线程异常结束"
        try:
            self._await_fresh_odometry(job)
            camera.start()
            camera_deadline = (
                time.monotonic() + self.config.timeouts.camera_start_seconds
            )
            while not job.first_image_event.wait(timeout=0.1):
                camera.ensure_running()
                self._raise_if_stopping(job)
                if time.monotonic() >= camera_deadline:
                    raise RuntimeError("相机节点启动后未在时限内收到实际图像")
            # 相机开流会重置 UVC 参数，必须在第一帧后再写入并读回。
            time.sleep(1.0)
            camera.ensure_running()
            camera.apply_lens_controls()
            with self._lock:
                if job.failure_reason:
                    raise RuntimeError(job.failure_reason)
                job.state = CorrectionStatus.STATE_SAMPLING
                job.message = "正在检测预期 Tag 并计算候选修正"
            job.journal.write(
                "camera_ready", pid=camera.pid, camera_log=str(camera_log)
            )

            next_detection_stamp_ns = 0
            processing_period_ns = int(1e9 / self.config.detection.processing_rate_hz)
            while not self._closing.is_set():
                self._raise_if_stopping(job)
                camera.ensure_running()
                if (
                    self.config.timeouts.ground_max_runtime_seconds > 0.0
                    and time.monotonic() - job.started_monotonic
                    > self.config.timeouts.ground_max_runtime_seconds
                ):
                    raise RuntimeError("地面校准任务超过配置生命周期")
                try:
                    image = self._image_queue.get(timeout=0.2)
                except queue.Empty:
                    self._check_first_tag_timeout(job)
                    continue
                image_stamp_ns = stamp_to_nanoseconds(image.header.stamp)
                if image_stamp_ns < next_detection_stamp_ns:
                    continue
                next_detection_stamp_ns = image_stamp_ns + processing_period_ns
                self._process_image(job, image, image_stamp_ns)
                self._check_first_tag_timeout(job)
                snapshot = job.snapshot
                if snapshot.diverged:
                    raise RuntimeError(snapshot.reason)
                if snapshot.converged:
                    with self._lock:
                        if job.state not in (
                            CorrectionStatus.STATE_APPLYING,
                            CorrectionStatus.STATE_SUCCEEDED,
                        ):
                            job.state = CorrectionStatus.STATE_CONVERGED
                            job.message = "候选已收敛；apply=false 持续更新直到 stop"
                    if job.apply_requested:
                        if not is_apply_time_source_safe(snapshot.odom_time_source):
                            raise RuntimeError(
                                "apply=true 的候选混用了时间轴或不是历史时间匹配"
                            )
                        with self._lock:
                            job.state = CorrectionStatus.STATE_APPLYING
                            job.message = "候选合格，等待 extnav 原子应用 ACK"
                        revision = self._apply_to_extnav(job, snapshot)
                        with self._lock:
                            job.extnav_applied = True
                            job.extnav_revision = revision
                            job.outcome = "applied"
                        final_success = True
                        final_message = "extnav 已确认应用修正"
                        return
        except _UserStopped:
            snapshot = job.snapshot
            final_success = snapshot.converged and not job.apply_requested
            outcome = "stopped_converged" if final_success else "stopped_without_result"
            with self._lock:
                job.outcome = outcome
            final_message = (
                "显式停止；已保留收敛候选但未修改 extnav"
                if final_success
                else "显式停止时尚无合格收敛结果"
            )
        except Exception as exc:
            final_success = False
            final_message = str(exc)
        finally:
            # active 在清理完成前保持为真，避免新任务与旧相机进程争用设备。
            with self._lock:
                job.state = CorrectionStatus.STATE_STOPPING
                job.message = f"{final_message}；正在关闭下视相机"
            try:
                camera.stop()
            except Exception as exc:
                self._service_logger.error("任务 %s 相机清理失败：%s", job.job_id, exc)
                final_success = False
                final_message = f"{final_message}；相机清理失败：{exc}"
            job.camera = None
            self._stop_odometry_capture()
            self._finish_job(
                job,
                success=final_success,
                message=final_message,
            )

    def _raise_if_stopping(self, job: _Job) -> None:
        """区分用户 stop、session/回调失败和节点关闭。"""
        with self._lock:
            if job.failure_reason:
                raise RuntimeError(job.failure_reason)
            if job.user_stop:
                raise _UserStopped
        if job.stop_event.is_set() or self._closing.is_set():
            raise RuntimeError("节点关闭，任务取消")

    def _check_first_tag_timeout(self, job: _Job) -> None:
        """长期未见预期 Tag 时明确失败，避免高负载相机无限空转。"""
        if job.first_expected_tag_monotonic > 0.0:
            return
        if (
            time.monotonic() - job.started_monotonic
            > self.config.timeouts.first_tag_seconds
        ):
            raise RuntimeError(
                f"{self.config.timeouts.first_tag_seconds:.1f}s 内未识别到预期 "
                f"Tag {job.expected_tag_id}"
            )

    def _image_to_gray(self, image: Image) -> np.ndarray:
        """校验硬件相机节点的 mono8 紧凑帧，拒绝隐式颜色/步长猜测。"""
        if image.encoding != "mono8":
            raise ValueError(f"相机图像编码必须为 mono8，实际 {image.encoding}")
        if (
            int(image.width) != self.config.camera.width
            or int(image.height) != self.config.camera.height
        ):
            raise ValueError(
                f"相机图像尺寸 {image.width}x{image.height} 与标定 "
                f"{self.config.camera.width}x{self.config.camera.height} 不一致"
            )
        if int(image.step) != int(image.width):
            raise ValueError(f"mono8 图像 step={image.step} 非紧凑宽度 {image.width}")
        pixels = np.frombuffer(image.data, dtype=np.uint8)
        expected = int(image.width) * int(image.height)
        if pixels.size != expected:
            raise ValueError(f"图像字节数 {pixels.size} != {expected}")
        return pixels.reshape(int(image.height), int(image.width))

    def _process_image(self, job: _Job, image: Image, image_stamp_ns: int) -> None:
        """对一帧执行检测、同时间匹配、完整 SE(3) 链和质量更新。"""
        batch = self._detector.detect(self._image_to_gray(image), self.config.tags)
        job.estimator.record_processing(image_stamp_ns, batch.processing_time_ms)
        with self._lock:
            job.frames_processed += 1
            job.detections_total += len(batch.marker_ids)
            job.snapshot = job.estimator.snapshot()

        if not batch.marker_ids:
            with self._lock:
                job.samples_rejected += 1
            return
        if self.config.detection.reject_multiple_tags and len(batch.marker_ids) > 1:
            self._reject_sample(job, f"同帧检测到多个 Tag：{batch.marker_ids}")
            return
        if job.expected_tag_id not in batch.marker_ids:
            self._reject_sample(
                job,
                f"检测到 {batch.marker_ids}，但预期 Tag {job.expected_tag_id}",
            )
            return
        job.first_expected_tag_monotonic = time.monotonic()
        detection = next(
            (item for item in batch.detections if item.tag_id == job.expected_tag_id),
            None,
        )
        if detection is None:
            self._reject_sample(job, "预期 Tag 解码成功但 PnP 求解失败")
            return
        with self._lock:
            job.detected_tag_id = detection.tag_id
        if (
            detection.reprojection_error_px
            > self.config.detection.max_reprojection_error_px
        ):
            self._reject_sample(
                job,
                f"重投影 RMS {detection.reprojection_error_px:.3f}px 超限",
            )
            return
        odometry_match = self._synchronizer.match(image_stamp_ns)
        if odometry_match is None:
            self._reject_sample(job, "图像时间戳附近没有满足阈值的 Odin 样本")
            return
        pose = odometry_match.message.pose.pose
        odin_from_imu = transform_from_pose(
            np.array((pose.position.x, pose.position.y, pose.position.z)),
            np.array(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ),
        )
        correction = compute_planar_correction(
            detection.camera_from_tag_standard,
            self.config.tags[job.expected_tag_id],
            self.config.t_imu_camera,
            odin_from_imu,
        )
        if (
            math.degrees(correction.tilt_rad)
            > self.config.detection.max_correction_tilt_deg
        ):
            self._reject_sample(
                job,
                f"完整空间修正 tilt={math.degrees(correction.tilt_rad):.3f}deg 超限",
            )
            return
        sample = CorrectionSample(
            stamp_ns=image_stamp_ns,
            x_m=correction.x_m,
            y_m=correction.y_m,
            yaw_rad=correction.yaw_rad,
            tilt_rad=correction.tilt_rad,
            reprojection_error_px=detection.reprojection_error_px,
            odom_match_error_ms=odometry_match.delta_ms,
            odom_time_source=odometry_match.time_source,
            processing_time_ms=batch.processing_time_ms,
        )
        snapshot = job.estimator.add(sample)
        with self._lock:
            job.samples_accepted += 1
            job.snapshot = snapshot
            job.message = snapshot.reason
        now = time.monotonic()
        if (
            now - job.last_summary_monotonic
            >= self.config.logging.sample_summary_period_seconds
        ):
            job.last_summary_monotonic = now
            job.journal.write(
                "candidate",
                tag_id=detection.tag_id,
                x_m=snapshot.x_m,
                y_m=snapshot.y_m,
                yaw_deg=math.degrees(snapshot.yaw_rad),
                tilt_deg=math.degrees(snapshot.tilt_rad),
                inliers=snapshot.inlier_samples,
                window=snapshot.window_samples,
                position_std_m=snapshot.position_std_m,
                yaw_std_deg=math.degrees(snapshot.yaw_std_rad),
                reprojection_error_px=snapshot.reprojection_error_px,
                odom_match_error_ms=snapshot.odom_match_error_ms,
                odom_time_source=snapshot.odom_time_source,
                processing_rate_hz=snapshot.processing_rate_hz,
                processing_time_ms=snapshot.processing_time_ms,
                converged=snapshot.converged,
                reason=snapshot.reason,
            )

    def _reject_sample(self, job: _Job, reason: str) -> None:
        """计数可解释的帧级拒绝，但不把单帧问题升级为服务故障。"""
        with self._lock:
            job.samples_rejected += 1
            job.message = reason

    def _apply_to_extnav(self, job: _Job, snapshot: QualitySnapshot) -> int:
        """提交冻结候选并等待明确 accepted+applied+revision ACK。"""
        with self._lock:
            extnav = self._extnav
            if (
                not extnav.service_available
                or not extnav.odin_available
                or extnav.odin_session_id != job.odin_session_id
                or extnav.revision != job.base_revision
            ):
                raise RuntimeError("提交前 extnav session/revision 已改变")
        if not self._set_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("未发现 extnav SetCorrection 服务")
        request = SetCorrection.Request()
        request.job_id = job.job_id
        request.odin_session_id = job.odin_session_id
        request.expected_revision = job.base_revision
        request.valid = True
        request.correction_x_m = snapshot.x_m
        request.correction_y_m = snapshot.y_m
        request.correction_yaw_rad = snapshot.yaw_rad
        request.sample_count = snapshot.inlier_samples
        request.position_std_m = snapshot.position_std_m
        request.yaw_std_rad = snapshot.yaw_std_rad
        request.reprojection_error_px = snapshot.reprojection_error_px
        request.odom_match_error_ms = snapshot.odom_match_error_ms
        future = self._set_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=self.config.timeouts.extnav_apply_seconds):
            raise RuntimeError("等待 extnav 应用 ACK 超时")
        try:
            response = future.result()
        except Exception as exc:
            raise RuntimeError(f"extnav SetCorrection 调用失败：{exc}") from exc
        if (
            response is None
            or not response.accepted
            or not response.applied
            or response.odin_session_id != job.odin_session_id
            or int(response.revision) <= job.base_revision
        ):
            detail = response.message if response is not None else "空响应"
            raise RuntimeError(f"extnav 未确认应用候选：{detail}")
        job.journal.write(
            "extnav_applied",
            revision=int(response.revision),
            response=str(response.message),
        )
        return int(response.revision)

    def _fail_job_locked(self, job: _Job, reason: str) -> None:
        """从订阅回调异步令任务失败；不在 executor 回调中阻塞清理。"""
        if not job.failure_reason:
            job.failure_reason = reason
            job.last_error = reason
            job.stop_event.set()

    def _finish_job(self, job: _Job, *, success: bool, message: str) -> None:
        """冻结最终状态并发布可靠 result；extnav 只有 ACK 成功才记 applied。"""
        with self._lock:
            job.active = False
            job.message = message
            job.last_error = "" if success else message
            job.state = (
                CorrectionStatus.STATE_SUCCEEDED
                if success
                else CorrectionStatus.STATE_FAILED
            )
            if not success and job.outcome == "running":
                job.outcome = "failed"
            snapshot = job.snapshot
        job.journal.write(
            "job_finished",
            success=success,
            applied=job.extnav_applied,
            revision=job.extnav_revision,
            outcome=job.outcome,
            message=message,
            samples_accepted=job.samples_accepted,
            samples_rejected=job.samples_rejected,
            x_m=snapshot.x_m,
            y_m=snapshot.y_m,
            yaw_deg=(
                math.degrees(snapshot.yaw_rad)
                if math.isfinite(snapshot.yaw_rad)
                else None
            ),
        )
        result = self._make_result(job, success)
        self._result_pub.publish(result)
        self._service_logger.info(
            "任务结束 job=%s success=%s applied=%s outcome=%s message=%s",
            job.job_id,
            success,
            job.extnav_applied,
            job.outcome,
            message,
        )
        self._publish_status()

    def _make_result(self, job: _Job, success: bool) -> CorrectionResult:
        """把冻结任务状态转换为可录包的可靠最终消息。"""
        snapshot = job.snapshot
        message = CorrectionResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.interface_version = INTERFACE_VERSION
        message.job_id = job.job_id
        message.expected_tag_id = job.expected_tag_id
        message.detected_tag_id = job.detected_tag_id
        message.apply_requested = job.apply_requested
        message.success = success
        message.applied = job.extnav_applied
        message.odin_session_id = job.odin_session_id
        message.extnav_revision = job.extnav_revision
        message.correction_x_m = _finite_or_zero(snapshot.x_m)
        message.correction_y_m = _finite_or_zero(snapshot.y_m)
        message.correction_yaw_deg = _degrees_or_zero(snapshot.yaw_rad)
        message.correction_tilt_deg = _degrees_or_zero(snapshot.tilt_rad)
        message.samples_accepted = job.samples_accepted
        message.samples_rejected = job.samples_rejected
        message.position_std_m = _finite_or_zero(snapshot.position_std_m)
        message.yaw_std_deg = _degrees_or_zero(snapshot.yaw_std_rad)
        message.reprojection_error_px = _finite_or_zero(snapshot.reprojection_error_px)
        message.odom_match_error_ms = _finite_or_zero(snapshot.odom_match_error_ms)
        message.duration_s = max(0.0, time.monotonic() - job.started_monotonic)
        message.processing_rate_hz = snapshot.processing_rate_hz
        message.processing_time_ms = snapshot.processing_time_ms
        message.outcome = job.outcome
        message.message = job.message
        message.log_path = str(job.journal.path)
        return message

    def _publish_status(self) -> None:
        """发布 idle/运行/终态和所有调试质量指标。"""
        with self._lock:
            job = self._job
            message = CorrectionStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.interface_version = INTERFACE_VERSION
            message.service_available = True
            if job is None:
                message.active = False
                message.state = CorrectionStatus.STATE_IDLE
                message.state_text = STATE_TEXT[CorrectionStatus.STATE_IDLE]
                message.expected_tag_id = -1
                message.detected_tag_id = -1
                message.message = "服务空闲；下视相机关闭"
            else:
                snapshot = job.snapshot
                message.active = job.active
                message.state = job.state
                message.state_text = STATE_TEXT.get(job.state, "unknown")
                message.job_id = job.job_id
                message.expected_tag_id = job.expected_tag_id
                message.detected_tag_id = job.detected_tag_id
                message.apply_requested = job.apply_requested
                message.odin_session_id = job.odin_session_id
                message.base_revision = job.base_revision
                message.frames_received = job.frames_received
                message.frames_processed = job.frames_processed
                message.detections_total = job.detections_total
                message.samples_accepted = job.samples_accepted
                message.samples_rejected = job.samples_rejected
                message.elapsed_s = max(0.0, time.monotonic() - job.started_monotonic)
                message.candidate_x_m = _finite_or_zero(snapshot.x_m)
                message.candidate_y_m = _finite_or_zero(snapshot.y_m)
                message.candidate_yaw_deg = _degrees_or_zero(snapshot.yaw_rad)
                message.correction_tilt_deg = _degrees_or_zero(snapshot.tilt_rad)
                message.position_std_m = _finite_or_zero(snapshot.position_std_m)
                message.yaw_std_deg = _degrees_or_zero(snapshot.yaw_std_rad)
                message.reprojection_error_px = _finite_or_zero(
                    snapshot.reprojection_error_px
                )
                message.odom_match_error_ms = _finite_or_zero(
                    snapshot.odom_match_error_ms
                )
                message.odom_time_source = snapshot.odom_time_source
                message.processing_rate_hz = snapshot.processing_rate_hz
                message.processing_time_ms = snapshot.processing_time_ms
                message.converged = snapshot.converged
                message.extnav_applied = job.extnav_applied
                message.extnav_revision = job.extnav_revision
                message.message = job.message
                message.last_error = job.last_error
        self._status_pub.publish(message)

    def _drain_image_queue(self) -> None:
        """新任务不得处理上一任务残留帧。"""
        while True:
            try:
                self._image_queue.get_nowait()
            except queue.Empty:
                return

    def close(self) -> None:
        """停止任务线程；不会清除 extnav 已经 ACK 的 active correction。"""
        self._closing.set()
        with self._lock:
            job = self._job
            if job is not None and job.active:
                job.stop_event.set()
            thread = job.thread if job is not None else None
        if thread is not None and thread.is_alive():
            thread.join(timeout=8.0)
        self._stop_odometry_capture()


class _UserStopped(Exception):
    """内部控制流：用户 stop 与故障/节点关闭采用不同最终语义。"""


def _finite_or_zero(value: float) -> float:
    """ROS 状态不用 NaN/inf，未形成指标时明确投影为 0。"""
    return float(value) if math.isfinite(value) else 0.0


def _degrees_or_zero(value: float) -> float:
    """有限弧度转度；未形成指标时返回 0。"""
    return math.degrees(value) if math.isfinite(value) else 0.0


def main(args=None) -> None:
    """使用多线程 executor，让相机/里程计回调在任务线程等待 ACK 时继续推进。"""
    rclpy.init(args=args)
    node = CorrectionServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.remove_node(node)
        node.destroy_node()
        # SIGINT 会由 rclpy 的默认信号处理器先关闭 context；try_shutdown 避免
        # systemd 正常 stop 因二次 shutdown 被误记为失败。
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
