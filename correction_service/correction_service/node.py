"""独立多 Tag 滑窗修正节点：单次采样、事务提交、对账与资源释放。"""

from __future__ import annotations

import math
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field, replace
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
    CalibrationKeyframe as CalibrationKeyframeMessage,
)
from correction_interfaces.msg import (
    CorrectionResult,
    CorrectionStatus,
    ExtnavCorrectionStatus,
)
from correction_interfaces.srv import (
    ApplySavedCorrection,
    ClearWindow,
    SetCorrection,
    StartCorrection,
    StopCorrection,
)

from . import INTERFACE_VERSION
from .camera_process import CameraProcess
from .config import ServiceConfig, load_config
from .detector import AprilTagDetector
from .estimator import CorrectionEstimator, CorrectionSample, QualitySnapshot
from .geometry import (
    compute_planar_correction,
    expected_fcu_jump,
    transform_from_pose,
    wrap_angle,
)
from .journal import JobJournal, configure_service_logger
from .synchronizer import (
    OdometrySynchronizer,
    is_apply_time_source_safe,
    stamp_to_nanoseconds,
)
from .window import (
    CalibrationKeyframe,
    RegistrationResult,
    WindowCandidate,
    build_temporary_window,
    residual_under_correction,
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
OPERATION_TEXT = {
    CorrectionStatus.OPERATION_NONE: "none",
    CorrectionStatus.OPERATION_FIRST: "first",
    CorrectionStatus.OPERATION_NEXT: "next",
    CorrectionStatus.OPERATION_APPLY_SAVED: "apply_saved",
}
APPLICATION_TO_MESSAGE = {
    "none": CorrectionStatus.APPLICATION_NONE,
    "not_requested": CorrectionStatus.APPLICATION_NOT_REQUESTED,
    "pending": CorrectionStatus.APPLICATION_PENDING,
    "applied_ack": CorrectionStatus.APPLICATION_APPLIED_ACK,
    "applied_status": CorrectionStatus.APPLICATION_APPLIED_STATUS,
    "rejected": CorrectionStatus.APPLICATION_REJECTED,
    "unknown": CorrectionStatus.APPLICATION_UNKNOWN,
}


@dataclass
class _ExtnavSnapshot:
    """带本机接收时间的 extnav 权威 active、session 和物理参数快照。"""

    received_monotonic: float = 0.0
    interface_version: str = ""
    service_available: bool = False
    odin_available: bool = False
    odin_session_id: str = ""
    correction_valid: bool = False
    revision: int = 0
    applied_job_id: str = ""
    correction_x_m: float = 0.0
    correction_y_m: float = 0.0
    correction_yaw_rad: float = 0.0
    lever_arm_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    installation_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass(frozen=True)
class _ApplicationOutcome:
    """SetCorrection 的 ACK、权威状态对账或不确定终态。"""

    state: str
    revision: int
    message: str
    confirmed_by_status: bool = False


@dataclass
class _Job:
    """唯一活动或最近完成操作的线程安全共享状态。"""

    job_id: str
    source_id: str
    operation: int
    expected_tag_id: int
    window_size: int
    apply_requested: bool
    odin_session_id: str
    base_revision: int
    base_window_revision: int
    started_monotonic: float
    started_ros_ns: int
    estimator: CorrectionEstimator | None
    journal: JobJournal
    state: int = CorrectionStatus.STATE_STARTING
    message: str = "正在准备操作"
    last_error: str = ""
    active: bool = True
    detected_tag_id: int = -1
    frames_received: int = 0
    frames_processed: int = 0
    detections_total: int = 0
    samples_accepted: int = 0
    samples_rejected: int = 0
    snapshot: QualitySnapshot = field(default_factory=QualitySnapshot)
    candidate: WindowCandidate | None = None
    temporary_window: tuple[CalibrationKeyframe, ...] = ()
    pre_eviction_registration: RegistrationResult | None = None
    window_saved: bool = False
    resources_released: bool = False
    application_state: str = "none"
    application_confirmed_by_status: bool = False
    extnav_revision: int = 0
    outcome: str = "running"
    stop_event: threading.Event = field(default_factory=threading.Event)
    user_stop: bool = False
    apply_request_sent: bool = False
    failure_reason: str = ""
    first_image_event: threading.Event = field(default_factory=threading.Event)
    first_expected_tag_monotonic: float = 0.0
    last_summary_monotonic: float = 0.0
    last_image_stamp_ns: int = 0
    thread: threading.Thread | None = None
    camera: CameraProcess | None = None
    increment_window_on_apply: bool = True
    retrying_unknown: bool = False


class CorrectionServiceNode(Node):
    """与飞控/视频生命周期解耦的服务端权威滑窗状态机。"""

    EXTNAV_STATUS_MAX_AGE_S = 2.0
    INSTALLATION_ANGLE_TOLERANCE_RAD = 1e-9

    def __init__(self, **node_kwargs) -> None:
        """严格加载 2.0 配置并仅创建低频状态接口，默认不订高频 Odin。"""
        super().__init__("correction_service", **node_kwargs)
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
        self._raw_arrival_ros_ns = 0
        self._raw_frame_pair: tuple[str, str] | None = None
        self._latest_raw_transform: np.ndarray | None = None
        self._local_session_id = ""
        self._odom_subscription = None
        self._image_subscription = None
        self._odom_capture_active = False
        self._closing = threading.Event()
        self._resource_fault = ""

        self._service_instance_id = uuid.uuid4().hex
        self._window_revision = 0
        self._window_size = self.config.window.default_size
        self._successful_calibrations = 0
        self._window: tuple[CalibrationKeyframe, ...] = ()
        self._saved_candidate: WindowCandidate | None = None
        self._pending_candidate: WindowCandidate | None = None
        self._window_message = "服务实例启动；窗口为空"
        self._audit = JobJournal(
            self.config.logging.directory,
            f"service-{self._service_instance_id[:8]}",
        )

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
        self._camera_image_topic = topics["camera_image"]
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
        self.create_service(
            ClearWindow, topics["clear_window_service"], self._on_clear_window
        )
        self.create_service(
            ApplySavedCorrection,
            topics["apply_saved_service"],
            self._on_apply_saved,
        )
        self.create_timer(0.25, self._publish_status)

        self._audit.write(
            "service_started",
            interface_version=INTERFACE_VERSION,
            service_instance_id=self._service_instance_id,
            config_fingerprint=self.config.config_fingerprint,
            window_size=self._window_size,
            tag_map={key: value.__dict__ for key, value in self.config.tags.items()},
            t_imu_camera=self.config.t_imu_camera.tolist(),
        )
        self._service_logger.info(
            "节点启动 instance=%s config=%s fingerprint=%s raw_on_demand=%s",
            self._service_instance_id,
            self.config.config_dir,
            self.config.config_fingerprint,
            self._raw_odometry_topic,
        )
        self.get_logger().info(
            "AprilTag-Odin 多 Tag 滑窗修正服务 2.0 已启动；默认 idle，相机保持关闭"
        )
        self._publish_status()

    def _on_odometry(self, message: Odometry) -> None:
        """缓存任务专属原始 Odin，并在时钟/session 异常时取消且清空窗口。"""
        with self._lock:
            if not self._odom_capture_active:
                return
        now_monotonic = time.monotonic()
        arrival_ros_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        frame_pair = (str(message.header.frame_id), str(message.child_frame_id))
        gap = (
            now_monotonic - self._raw_received_monotonic
            if self._raw_received_monotonic > 0.0
            else 0.0
        )
        discontinuity = ""
        if self._raw_received_monotonic > 0.0 and gap > 2.0:
            discontinuity = f"任务 Odin 断流 {gap:.3f}s"
        elif self._raw_stamp_ns > 0 and stamp_ns < self._raw_stamp_ns - 250_000_000:
            discontinuity = "任务 Odin header 时间戳回退"
        elif (
            self._raw_arrival_ros_ns > 0
            and arrival_ros_ns < self._raw_arrival_ros_ns - 250_000_000
        ):
            discontinuity = "主机 ROS 时钟回退"
        elif self._raw_frame_pair is not None and frame_pair != self._raw_frame_pair:
            discontinuity = "任务 Odin frame_id 改变"
        if discontinuity:
            with self._lock:
                self._invalidate_window_locked(discontinuity)
                job = self._job
                if job is not None and job.active:
                    self._fail_job_locked(job, discontinuity)
            self._synchronizer.clear()
            self._local_session_id = f"local-{stamp_ns}-{uuid.uuid4().hex[:8]}"
        elif not self._local_session_id:
            self._local_session_id = f"local-{stamp_ns}-{uuid.uuid4().hex[:8]}"

        pose = message.pose.pose
        try:
            raw_transform = transform_from_pose(
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
        except ValueError as exc:
            with self._lock:
                job = self._job
                if job is not None and job.active:
                    self._fail_job_locked(job, f"原始 Odin 位姿无效：{exc}")
            return
        self._raw_received_monotonic = now_monotonic
        self._raw_stamp_ns = stamp_ns
        self._raw_arrival_ros_ns = arrival_ros_ns
        self._raw_frame_pair = frame_pair
        self._latest_raw_transform = raw_transform
        self._synchronizer.add(message, arrival_ros_ns)

    def _start_odometry_capture(self) -> None:
        """为一个采样或 apply_saved 操作创建有界高频订阅。"""
        if self._resource_fault:
            raise RuntimeError(f"上次资源清理失败，需重启服务：{self._resource_fault}")
        if self._odom_subscription is not None:
            raise RuntimeError("Odin 任务订阅尚未释放")
        self._synchronizer.clear()
        self._raw_received_monotonic = 0.0
        self._raw_stamp_ns = 0
        self._raw_arrival_ros_ns = 0
        self._raw_frame_pair = None
        self._latest_raw_transform = None
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

    def _stop_odometry_capture(self) -> bool:
        """关闭回调门并销毁订阅；失败会锁存资源故障而不是继续新任务。"""
        with self._lock:
            self._odom_capture_active = False
            subscription = self._odom_subscription
            self._odom_subscription = None
        released = True
        if subscription is not None and not self.destroy_subscription(subscription):
            released = False
            self._resource_fault = "无法销毁任务 Odin odometry 订阅"
            self._service_logger.error(self._resource_fault)
        self._synchronizer.clear()
        self._raw_received_monotonic = 0.0
        self._raw_stamp_ns = 0
        self._raw_arrival_ros_ns = 0
        self._raw_frame_pair = None
        self._latest_raw_transform = None
        self._local_session_id = ""
        return released

    def _start_image_capture(self) -> None:
        """只为 first/next 创建相机话题订阅；apply_saved 和 idle 不订图像。"""
        if self._resource_fault:
            raise RuntimeError(f"上次资源清理失败，需重启服务：{self._resource_fault}")
        if self._image_subscription is not None:
            raise RuntimeError("相机图像订阅尚未释放")
        self._drain_image_queue()
        self._image_subscription = self.create_subscription(
            Image,
            self._camera_image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

    def _stop_image_capture(self) -> bool:
        """销毁任务专属图像订阅；失败时锁存资源故障。"""
        with self._lock:
            subscription = self._image_subscription
            self._image_subscription = None
        released = True
        if subscription is not None and not self.destroy_subscription(subscription):
            released = False
            self._resource_fault = "无法销毁任务相机图像订阅"
            self._service_logger.error(self._resource_fault)
        self._drain_image_queue()
        return released

    def _await_fresh_odometry(self, job: _Job) -> None:
        """有限等待 raw，并持续核对 extnav 权威 session 与任务基线。"""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._raise_if_stopping(job)
            with self._lock:
                extnav_ready, reason = self._extnav_ready_locked()
                if not extnav_ready:
                    raise RuntimeError(reason)
                if self._extnav.odin_session_id != job.odin_session_id:
                    raise RuntimeError("等待 raw 时 extnav Odin session 已改变")
                received = self._raw_received_monotonic
                local_session = self._local_session_id
            if received > 0.0 and time.monotonic() - received <= 0.5:
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
            if (
                job is None
                or not job.active
                or job.operation == CorrectionStatus.OPERATION_APPLY_SAVED
            ):
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
        """缓存权威状态；明确 session 失效会取消采样并清空原始 Q 窗口。"""
        snapshot = _ExtnavSnapshot(
            received_monotonic=time.monotonic(),
            interface_version=str(message.interface_version),
            service_available=bool(message.service_available),
            odin_available=bool(message.odin_available),
            odin_session_id=str(message.odin_session_id),
            correction_valid=bool(message.correction_valid),
            revision=int(message.revision),
            applied_job_id=str(message.applied_job_id),
            correction_x_m=float(message.correction_x_m),
            correction_y_m=float(message.correction_y_m),
            correction_yaw_rad=math.radians(float(message.correction_yaw_deg)),
            lever_arm_m=np.array(
                (
                    message.lever_arm_x_m,
                    message.lever_arm_y_m,
                    message.lever_arm_z_m,
                ),
                dtype=np.float64,
            ),
            installation_rpy=np.array(
                (
                    message.installation_roll_rad,
                    message.installation_pitch_rad,
                    message.installation_yaw_rad,
                ),
                dtype=np.float64,
            ),
        )
        with self._lock:
            self._extnav = snapshot
            job = self._job
            explicit_unavailable = snapshot.interface_version == INTERFACE_VERSION and (
                not snapshot.service_available
                or not snapshot.odin_available
                or not snapshot.odin_session_id
            )
            session_changed = bool(
                self._window
                and snapshot.odin_session_id
                and snapshot.odin_session_id != self._window[0].odin_session_id
            )
            if self._window and (explicit_unavailable or session_changed):
                self._invalidate_window_locked(
                    "extnav 明确报告 Odin session 不可用或已改变"
                )
            if (
                job is not None
                and job.active
                and (
                    snapshot.interface_version != INTERFACE_VERSION
                    or not snapshot.odin_available
                    or snapshot.odin_session_id != job.odin_session_id
                )
            ):
                self._fail_job_locked(job, "extnav 报告接口/session 已改变或失效")

    def _extnav_ready_locked(self) -> tuple[bool, str]:
        """判定滑窗保存也必须依赖的新鲜 2.0 extnav session。"""
        extnav = self._extnav
        if extnav.interface_version and extnav.interface_version != INTERFACE_VERSION:
            return False, (
                f"extnav 接口版本 {extnav.interface_version} != {INTERFACE_VERSION}，"
                "禁用写操作"
            )
        if (
            extnav.received_monotonic <= 0.0
            or time.monotonic() - extnav.received_monotonic
            > self.EXTNAV_STATUS_MAX_AGE_S
        ):
            return False, "extnav 状态不可用或过期"
        if not extnav.service_available or not extnav.odin_available:
            return False, "extnav/Odin 当前明确不可用"
        if not extnav.odin_session_id:
            return False, "extnav 没有可信 Odin session"
        return True, ""

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

    def _validate_common_request_locked(
        self,
        source_id: str,
        sequence: int,
        expected_instance: str,
        expected_window_revision: int,
    ) -> tuple[bool, str]:
        """统一校验 sequence、实例、窗口 revision、并发和 unknown 锁。"""
        accepted, reason = self._accept_sequence(source_id, sequence)
        if not accepted:
            return False, reason
        if expected_instance != self._service_instance_id:
            return False, "service instance 已改变，请刷新权威状态"
        if int(expected_window_revision) != self._window_revision:
            return False, (
                f"window revision 冲突：期望 {expected_window_revision}，"
                f"当前 {self._window_revision}"
            )
        if self._job is not None and self._job.active:
            return False, f"已有操作运行：{self._job.job_id}"
        if self._pending_candidate is not None:
            return False, "存在 application_unknown；只能用应用当前结果幂等重试/对账"
        if self._resource_fault:
            return False, f"资源故障需重启服务：{self._resource_fault}"
        return True, ""

    def _on_start(self, request, response):
        """验证 first/next 与三个 revision 后异步启动一次性采样。"""
        response.accepted = False
        response.job_id = ""
        response.service_instance_id = self._service_instance_id
        response.window_revision = self._window_revision
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._validate_common_request_locked(
                str(request.source_id),
                int(request.sequence),
                str(request.expected_service_instance_id),
                int(request.expected_window_revision),
            )
            if not accepted:
                response.message = reason
                return response
            ready, reason = self._extnav_ready_locked()
            if not ready:
                response.message = reason
                return response
            if int(request.expected_extnav_revision) != self._extnav.revision:
                response.message = "extnav revision 已改变，请刷新后重新显式提交"
                return response
            operation = int(request.operation)
            if operation not in (
                CorrectionStatus.OPERATION_FIRST,
                CorrectionStatus.OPERATION_NEXT,
            ):
                response.message = "operation 必须为 first 或 next"
                return response
            if operation == CorrectionStatus.OPERATION_FIRST and self._window:
                response.message = "首次校准仅允许在空窗口执行"
                return response
            if operation == CorrectionStatus.OPERATION_NEXT and not self._window:
                response.message = "后续校准需要同 session 的已有 keyframe"
                return response
            window_size = int(request.window_size)
            if not 2 <= window_size <= self.config.window.maximum_size:
                response.message = "滑窗长度超出配置范围"
                return response
            if self._window and window_size != self._window_size:
                response.message = "窗口非空时滑窗长度已锁定"
                return response
            expected_tag_id = int(request.expected_tag_id)
            if expected_tag_id not in self.config.tags:
                response.message = f"tag_pose.csv 未配置 Tag {expected_tag_id}"
                return response
            if any(item.tag_id == expected_tag_id for item in self._window):
                response.message = "当前窗口已含相同 Tag ID，不能增加独立基线"
                return response
            if self._window and self._window_is_stale_locked():
                response.message = "滑窗已过期，必须先清空再重新首次校准"
                return response

            self._window_size = window_size
            job_id = uuid.uuid4().hex[:12]
            journal = JobJournal(self.config.logging.directory, job_id)
            estimator = CorrectionEstimator(
                self.config.quality,
                self.config.keyframe,
                first_calibration=(operation == CorrectionStatus.OPERATION_FIRST),
            )
            job = _Job(
                job_id=job_id,
                source_id=str(request.source_id),
                operation=operation,
                expected_tag_id=expected_tag_id,
                window_size=window_size,
                apply_requested=bool(request.apply),
                odin_session_id=self._extnav.odin_session_id,
                base_revision=self._extnav.revision,
                base_window_revision=self._window_revision,
                started_monotonic=time.monotonic(),
                started_ros_ns=self.get_clock().now().nanoseconds,
                estimator=estimator,
                journal=journal,
                application_state="pending" if request.apply else "not_requested",
                message="正在启动下视相机",
            )
            self._job = job
            try:
                self._start_odometry_capture()
                self._start_image_capture()
            except Exception as exc:  # noqa: BLE001 - ROS 创建实体会透传多类运行时异常
                job.active = False
                self._stop_image_capture()
                self._stop_odometry_capture()
                response.message = f"无法创建任务输入订阅：{exc}"
                return response
            job.journal.write(
                "job_started",
                operation=OPERATION_TEXT[operation],
                expected_tag_id=expected_tag_id,
                apply=job.apply_requested,
                service_instance_id=self._service_instance_id,
                window_revision=self._window_revision,
                window_size=window_size,
                odin_session_id=job.odin_session_id,
                extnav_revision=job.base_revision,
                config_fingerprint=self.config.config_fingerprint,
                tag_map={
                    key: value.__dict__ for key, value in self.config.tags.items()
                },
                t_imu_camera=self.config.t_imu_camera.tolist(),
                window_before=[
                    item.audit_dict(self.config.window) for item in self._window
                ],
            )
            job.thread = threading.Thread(
                target=self._run_sampling_job,
                args=(job,),
                name=f"correction-job-{job_id}",
                daemon=True,
            )
            job.thread.start()
            response.accepted = True
            response.job_id = job_id
            response.window_revision = self._window_revision
            response.message = "单次收敛校准已异步启动"
            self._service_logger.info(
                "任务启动 job=%s op=%s tag=%d apply=%s session=%s",
                job_id,
                OPERATION_TEXT[operation],
                expected_tag_id,
                job.apply_requested,
                job.odin_session_id,
            )
        self._publish_status()
        return response

    def _on_stop(self, request, response):
        """停止采样；应用请求已发出后只继续对账，绝不伪装回滚。"""
        response.accepted = False
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._accept_sequence(
                str(request.source_id), int(request.sequence)
            )
            if not accepted:
                response.message = reason
                return response
            if str(request.expected_service_instance_id) != self._service_instance_id:
                response.message = "service instance 已改变"
                return response
            job = self._job
            if job is None or not job.active:
                response.message = "当前没有活动校准操作"
                return response
            if request.job_id and str(request.job_id) != job.job_id:
                response.message = f"job_id 不匹配，当前为 {job.job_id}"
                return response
            job.user_stop = True
            if job.apply_request_sent:
                job.message = "stop 到达时应用已发出；继续等待 ACK/状态对账"
            else:
                job.state = CorrectionStatus.STATE_STOPPING
                job.message = "收到 stop，正在丢弃临时 keyframe 并释放资源"
                job.stop_event.set()
            job.journal.write(
                "stop_requested", apply_request_sent=job.apply_request_sent
            )
            response.accepted = True
            response.message = job.message
        self._publish_status()
        return response

    def _on_clear_window(self, request, response):
        """清空窗口/N/候选并递增 revision，但不调用 extnav maintenance clear。"""
        response.accepted = False
        response.service_instance_id = self._service_instance_id
        response.window_revision = self._window_revision
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._validate_common_request_locked(
                str(request.source_id),
                int(request.sequence),
                str(request.expected_service_instance_id),
                int(request.expected_window_revision),
            )
            if not accepted:
                response.message = reason
                return response
            before = [item.audit_dict(self.config.window) for item in self._window]
            self._invalidate_window_locked("用户显式清空滑窗")
            self._audit.write(
                "window_cleared",
                source_id=str(request.source_id),
                window_before=before,
                window_revision=self._window_revision,
                extnav_untouched=True,
            )
            response.accepted = True
            response.window_revision = self._window_revision
            response.message = "滑窗、候选和累计次数已清空；extnav active 未改变"
        self._publish_status()
        return response

    def _on_apply_saved(self, request, response):
        """应用已保存候选，或以原 job/candidate 幂等重试 unknown。"""
        response.accepted = False
        response.job_id = ""
        response.service_instance_id = self._service_instance_id
        response.window_revision = self._window_revision
        response.message = "请求被拒绝"
        with self._lock:
            accepted, reason = self._accept_sequence(
                str(request.source_id), int(request.sequence)
            )
            if not accepted:
                response.message = reason
                return response
            if str(request.expected_service_instance_id) != self._service_instance_id:
                response.message = "service instance 已改变"
                return response
            if int(request.expected_window_revision) != self._window_revision:
                response.message = "window revision 已改变"
                return response
            if self._job is not None and self._job.active:
                response.message = f"已有操作运行：{self._job.job_id}"
                return response
            ready, reason = self._extnav_ready_locked()
            if not ready:
                response.message = reason
                return response
            if int(request.expected_extnav_revision) != self._extnav.revision:
                response.message = "extnav revision 已改变，请刷新并重新显式确认"
                return response
            retrying_unknown = self._pending_candidate is not None
            candidate = self._pending_candidate or self._saved_candidate
            if candidate is None or not candidate.valid:
                response.message = "当前没有已保存的合格候选"
                return response
            if candidate.aged(time.monotonic(), self.config.window):
                response.message = "保存候选已过期，不能应用"
                return response
            if candidate.odin_session_id != self._extnav.odin_session_id:
                response.message = "保存候选 Odin session 已失效"
                return response
            if candidate.config_fingerprint != self.config.config_fingerprint:
                response.message = "保存候选配置指纹已失效"
                return response
            if (
                candidate.saved
                and candidate.saved_window_revision != self._window_revision
            ):
                response.message = "保存候选不属于当前窗口 revision"
                return response
            if (
                not candidate.saved
                and candidate.base_window_revision != self._window_revision
            ):
                response.message = "unknown pending 的窗口基线已改变"
                return response
            if (
                retrying_unknown
                and self._extnav.revision != candidate.expected_extnav_revision
            ):
                already_matches = (
                    self._extnav.correction_valid
                    and self._extnav.applied_job_id == candidate.application_job_id
                    and np.allclose(
                        (
                            self._extnav.correction_x_m,
                            self._extnav.correction_y_m,
                            self._extnav.correction_yaw_rad,
                        ),
                        (candidate.x_m, candidate.y_m, candidate.yaw_rad),
                        atol=1e-9,
                        rtol=0.0,
                    )
                )
                if not already_matches:
                    response.message = (
                        "unknown 后 extnav 已被其他 revision 覆盖；"
                        "不能刷新 CAS 强行重放，未知诊断保持锁存"
                    )
                    return response

            application_job_id = candidate.application_job_id
            job = _Job(
                job_id=application_job_id,
                source_id=str(request.source_id),
                operation=CorrectionStatus.OPERATION_APPLY_SAVED,
                expected_tag_id=-1,
                window_size=self._window_size,
                apply_requested=True,
                odin_session_id=candidate.odin_session_id,
                base_revision=self._extnav.revision,
                base_window_revision=self._window_revision,
                started_monotonic=time.monotonic(),
                started_ros_ns=self.get_clock().now().nanoseconds,
                estimator=None,
                journal=JobJournal(self.config.logging.directory, application_job_id),
                candidate=replace(
                    candidate,
                    expected_extnav_revision=self._extnav.revision,
                    application_state=(
                        "unknown" if self._pending_candidate is not None else "pending"
                    ),
                ),
                temporary_window=candidate.keyframes,
                application_state="pending",
                increment_window_on_apply=not candidate.saved,
                retrying_unknown=retrying_unknown,
                message="正在获取新鲜 raw 复算当前飞控中心跳变",
            )
            self._job = job
            try:
                self._start_odometry_capture()
            except Exception as exc:  # noqa: BLE001 - ROS 创建实体会透传多类运行时异常
                job.active = False
                response.message = f"无法创建短时 Odin 订阅：{exc}"
                return response
            job.journal.write(
                "apply_saved_started",
                retry_unknown=retrying_unknown,
                candidate=self._candidate_audit(job.candidate),
                window_revision=self._window_revision,
                extnav_revision=self._extnav.revision,
            )
            job.thread = threading.Thread(
                target=self._run_apply_saved_job,
                args=(job,),
                name=f"correction-apply-{application_job_id}",
                daemon=True,
            )
            job.thread.start()
            response.accepted = True
            response.job_id = application_job_id
            response.message = "保存候选应用/对账已异步启动"
        self._publish_status()
        return response

    def _run_sampling_job(self, job: _Job) -> None:
        """拥有相机采样，冻结候选后先释放资源，再保存或调用 extnav。"""
        camera_log = job.journal.path.with_suffix(".camera.log")
        camera = CameraProcess(
            self.config.camera,
            self.config.lens_controls,
            camera_log,
            self._service_logger,
        )
        job.camera = camera
        failure = ""
        try:
            self._await_fresh_odometry(job)
            camera.start()
            deadline = time.monotonic() + self.config.timeouts.camera_start_seconds
            while not job.first_image_event.wait(timeout=0.1):
                camera.ensure_running()
                self._raise_if_stopping(job)
                if time.monotonic() >= deadline:
                    raise RuntimeError("相机节点启动后未在时限内收到实际图像")
            # 相机开流会重置 UVC 参数，必须在第一帧后再写入并读回。
            time.sleep(1.0)
            camera.ensure_running()
            camera.apply_lens_controls()
            with self._lock:
                if job.failure_reason:
                    raise RuntimeError(job.failure_reason)
                job.state = CorrectionStatus.STATE_SAMPLING
                job.message = "正在采集当前 Tag 停留段并生成单一 keyframe"
            job.journal.write(
                "camera_ready", pid=camera.pid, camera_log=str(camera_log)
            )
            self._sample_until_candidate(job, camera)
        except _UserStopped:
            failure = "显式停止；临时 keyframe 已丢弃，旧窗口和 active 保持不变"
            job.outcome = "stopped_before_commit"
        except Exception as exc:  # noqa: BLE001 - 工作线程必须把所有采集异常转为失败结果
            failure = str(exc)
        finally:
            with self._lock:
                job.state = CorrectionStatus.STATE_STOPPING
                job.message = f"{failure or '候选已冻结'}；正在释放采集资源"
            resources_released = True
            try:
                camera.stop()
            except Exception as exc:  # noqa: BLE001 - 清理失败必须锁存并阻止后续任务
                resources_released = False
                self._resource_fault = f"相机清理失败：{exc}"
                failure = f"{failure or '候选已冻结'}；{self._resource_fault}"
                self._service_logger.error(
                    "任务 %s %s", job.job_id, self._resource_fault
                )
            job.camera = None
            resources_released = self._stop_image_capture() and resources_released
            resources_released = self._stop_odometry_capture() and resources_released
            job.resources_released = resources_released

        if failure or not job.resources_released or job.candidate is None:
            if job.apply_requested and not job.apply_request_sent:
                job.application_state = "rejected"
            self._finish_job(job, success=False, message=failure or "资源未完整释放")
            return
        if job.apply_requested:
            self._apply_and_finalize(job)
        else:
            self._save_dry_run_and_finalize(job)

    def _sample_until_candidate(self, job: _Job, camera: CameraProcess) -> None:
        """检测直到一次收敛，随后立即冻结并自动结束采集。"""
        next_detection_stamp_ns = 0
        period_ns = int(1e9 / self.config.detection.processing_rate_hz)
        while not self._closing.is_set():
            self._raise_if_stopping(job)
            camera.ensure_running()
            if (
                self.config.timeouts.ground_max_runtime_seconds > 0.0
                and time.monotonic() - job.started_monotonic
                > self.config.timeouts.ground_max_runtime_seconds
            ):
                raise RuntimeError("单次采样超过配置最大时长")
            try:
                image = self._image_queue.get(timeout=0.2)
            except queue.Empty:
                self._check_first_tag_timeout(job)
                continue
            image_stamp_ns = stamp_to_nanoseconds(image.header.stamp)
            if (
                job.last_image_stamp_ns > 0
                and image_stamp_ns < job.last_image_stamp_ns - 250_000_000
            ):
                raise RuntimeError("相机采集时钟回退，丢弃本次停留段")
            job.last_image_stamp_ns = image_stamp_ns
            if image_stamp_ns < next_detection_stamp_ns:
                continue
            next_detection_stamp_ns = image_stamp_ns + period_ns
            self._process_image(job, image, image_stamp_ns)
            self._check_first_tag_timeout(job)
            snapshot = job.snapshot
            if snapshot.diverged:
                raise RuntimeError(snapshot.reason)
            if snapshot.converged:
                with self._lock:
                    job.state = CorrectionStatus.STATE_CONVERGED
                    job.message = "停留段收敛，正在冻结滑窗候选"
                    self._freeze_candidate_locked(job)
                return

    def _run_apply_saved_job(self, job: _Job) -> None:
        """不开相机，只短时获取 raw、释放订阅并应用同一个冻结候选。"""
        failure = ""
        try:
            self._await_fresh_odometry(job)
            with self._lock:
                if job.candidate is None:
                    raise RuntimeError("保存候选不存在")
                job.candidate = self._evaluate_candidate_apply_locked(job.candidate)
                if not job.candidate.can_apply:
                    raise RuntimeError(job.candidate.apply_reason)
        except _UserStopped:
            failure = "显式停止；应用请求尚未发出"
            job.outcome = "stopped_before_apply"
        except Exception as exc:  # noqa: BLE001 - 工作线程必须把所有应用异常转为失败结果
            failure = str(exc)
        finally:
            job.resources_released = self._stop_odometry_capture()
        if failure or not job.resources_released:
            job.application_state = "rejected"
            self._finish_job(job, success=False, message=failure or "Odin 订阅清理失败")
            return
        self._apply_and_finalize(job)

    def _check_first_tag_timeout(self, job: _Job) -> None:
        """长期未见预期 Tag 时关闭高负载资源。"""
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
        """按图像时间匹配 raw，完整 SE(3) 后同时形成粗修正和原始 Q。"""
        assert job.estimator is not None
        batch = self._detector.detect(self._image_to_gray(image), self.config.tags)
        job.estimator.record_processing(image_stamp_ns, batch.processing_time_ms)
        with self._lock:
            job.frames_processed += 1
            job.detections_total += len(batch.marker_ids)
            job.snapshot = job.estimator.snapshot()

        if not batch.marker_ids:
            self._reject_sample(job, "未检测到 Tag", journal=False)
            return
        if self.config.detection.reject_multiple_tags and len(batch.marker_ids) > 1:
            self._reject_sample(job, f"同帧检测到多个 Tag：{batch.marker_ids}")
            return
        if job.expected_tag_id not in batch.marker_ids:
            self._reject_sample(
                job, f"检测到 {batch.marker_ids}，但预期 Tag {job.expected_tag_id}"
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
                job, f"重投影 RMS {detection.reprojection_error_px:.3f}px 超限"
            )
            return
        match = self._synchronizer.match(image_stamp_ns)
        if match is None:
            self._reject_sample(job, "图像时间戳附近没有满足阈值的 Odin 历史样本")
            return
        pose = match.message.pose.pose
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
        linear = match.message.twist.twist.linear
        speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        sample = CorrectionSample(
            stamp_ns=image_stamp_ns,
            x_m=correction.x_m,
            y_m=correction.y_m,
            yaw_rad=correction.yaw_rad,
            tilt_rad=correction.tilt_rad,
            reprojection_error_px=detection.reprojection_error_px,
            odom_match_error_ms=match.delta_ms,
            odom_time_source=match.time_source,
            processing_time_ms=batch.processing_time_ms,
            odin_tag_x_m=correction.odin_tag_x_m,
            odin_tag_y_m=correction.odin_tag_y_m,
            odin_speed_mps=speed,
        )
        snapshot = job.estimator.add(sample)
        with self._lock:
            job.samples_accepted += 1
            job.snapshot = snapshot
            job.message = snapshot.reason
        tag = self.config.tags[job.expected_tag_id]
        job.journal.write(
            "sample_accepted",
            image_stamp_ns=image_stamp_ns,
            tag_id=detection.tag_id,
            P_world_m=[tag.x, tag.y],
            Q_odin_m=[correction.odin_tag_x_m, correction.odin_tag_y_m],
            single_tag_correction=[
                correction.x_m,
                correction.y_m,
                correction.yaw_rad,
            ],
            tilt_rad=correction.tilt_rad,
            reprojection_error_px=detection.reprojection_error_px,
            odom_match_error_ms=match.delta_ms,
            odom_time_source=match.time_source,
            odin_speed_mps=speed,
        )
        now = time.monotonic()
        if (
            now - job.last_summary_monotonic
            >= self.config.logging.sample_summary_period_seconds
        ):
            job.last_summary_monotonic = now
            job.journal.write(
                "keyframe_progress",
                Q_odin_m=[snapshot.odin_tag_x_m, snapshot.odin_tag_y_m],
                inliers=snapshot.inlier_samples,
                window_samples=snapshot.window_samples,
                independent_blocks=snapshot.independent_blocks,
                effective_sigma_m=snapshot.effective_position_sigma_m,
                max_sync_motion_error_m=snapshot.max_sync_motion_error_m,
                position_std_m=snapshot.position_std_m,
                yaw_std_deg=math.degrees(snapshot.yaw_std_rad),
                odom_time_source=snapshot.odom_time_source,
                converged=snapshot.converged,
                reason=snapshot.reason,
            )

    def _reject_sample(self, job: _Job, reason: str, *, journal: bool = True) -> None:
        """记录帧级拒绝；无 Tag 高频事件只计数以避免日志刷屏。"""
        with self._lock:
            job.samples_rejected += 1
            job.message = reason
        if journal:
            job.journal.write("sample_rejected", reason=reason)

    def _freeze_candidate_locked(self, job: _Job) -> None:
        """从收敛停留段创建 keyframe，并在旧窗口副本上原子求候选。"""
        snapshot = job.snapshot
        tag = self.config.tags[job.expected_tag_id]
        keyframe = CalibrationKeyframe(
            tag_id=tag.tag_id,
            world_x_m=tag.x,
            world_y_m=tag.y,
            odin_x_m=snapshot.odin_tag_x_m,
            odin_y_m=snapshot.odin_tag_y_m,
            capture_start_ns=snapshot.capture_start_ns,
            capture_end_ns=snapshot.capture_end_ns,
            created_monotonic=time.monotonic(),
            created_ros_ns=self.get_clock().now().nanoseconds,
            samples_accepted=job.samples_accepted,
            samples_rejected=job.samples_rejected,
            independent_blocks=snapshot.independent_blocks,
            position_std_m=snapshot.odin_tag_position_std_m,
            effective_sigma_m=snapshot.effective_position_sigma_m,
            reprojection_error_px=snapshot.reprojection_error_px,
            odom_match_error_ms=snapshot.odom_match_error_ms,
            max_sync_motion_error_m=snapshot.max_sync_motion_error_m,
            correction_tilt_rad=snapshot.tilt_rad,
            single_tag_yaw_rad=snapshot.yaw_rad,
            odom_time_source=snapshot.odom_time_source,
            odin_session_id=job.odin_session_id,
            config_fingerprint=self.config.config_fingerprint,
            source_job_id=job.job_id,
        )
        if job.operation == CorrectionStatus.OPERATION_FIRST:
            registration = RegistrationResult(
                valid=True,
                reason="首次单 Tag 完整 SE(3) 粗修正；尚无空间基线",
                x_m=snapshot.x_m,
                y_m=snapshot.y_m,
                yaw_rad=snapshot.yaw_rad,
                rms_m=0.0,
                max_residual_m=0.0,
                model_yaw_std_rad=snapshot.yaw_std_rad,
                residuals_m=(0.0,),
            )
            temporary_window = (keyframe,)
            stage = "rough_single_tag"
            candidate_x, candidate_y, candidate_yaw = (
                snapshot.x_m,
                snapshot.y_m,
                snapshot.yaw_rad,
            )
            quality_reason = registration.reason
        else:
            prior = self._saved_candidate
            if prior is not None:
                prior_residual = residual_under_correction(
                    keyframe, (prior.x_m, prior.y_m, prior.yaw_rad)
                )
                keyframe = replace(keyframe, prior_model_residual_m=prior_residual)
                if prior_residual > self.config.window.max_prior_model_residual_m:
                    job.journal.write(
                        "window_candidate_rejected",
                        reason="新 keyframe 在旧模型下残差明显超限",
                        new_keyframe=keyframe.audit_dict(self.config.window),
                        prior_model_residual_m=prior_residual,
                        threshold_m=self.config.window.max_prior_model_residual_m,
                        window_before=[
                            item.audit_dict(self.config.window) for item in self._window
                        ],
                    )
                    raise RuntimeError(
                        "新 keyframe 在旧模型下残差明显超限；保留旧窗口，"
                        "不自动删除旧锚点"
                    )
            temporary_window, registration, pre_registration = build_temporary_window(
                self._window, keyframe, job.window_size, self.config.window
            )
            job.pre_eviction_registration = pre_registration
            if not registration.valid:
                job.journal.write(
                    "window_candidate_rejected",
                    reason=registration.reason,
                    new_keyframe=keyframe.audit_dict(self.config.window),
                    window_before=[
                        item.audit_dict(self.config.window) for item in self._window
                    ],
                    pre_eviction_registration=self._registration_audit(
                        pre_registration
                    ),
                )
                raise RuntimeError(registration.reason)
            temporary_window = tuple(
                replace(item, prior_model_residual_m=residual)
                for item, residual in zip(
                    temporary_window, registration.residuals_m, strict=True
                )
            )
            stage = "refined_multi_tag"
            candidate_x, candidate_y, candidate_yaw = (
                registration.x_m,
                registration.y_m,
                registration.yaw_rad,
            )
            quality_reason = registration.reason

        candidate = WindowCandidate(
            keyframes=temporary_window,
            x_m=candidate_x,
            y_m=candidate_y,
            yaw_rad=candidate_yaw,
            stage=stage,
            odin_session_id=job.odin_session_id,
            config_fingerprint=self.config.config_fingerprint,
            source_job_id=job.job_id,
            application_job_id=(
                job.job_id if job.apply_requested else f"{job.job_id}-apply"
            ),
            base_window_revision=job.base_window_revision,
            expected_extnav_revision=job.base_revision,
            created_monotonic=time.monotonic(),
            created_ros_ns=self.get_clock().now().nanoseconds,
            valid=True,
            quality_reason=quality_reason,
            position_std_m=max(
                snapshot.position_std_m, snapshot.effective_position_sigma_m
            ),
            yaw_std_rad=(
                snapshot.yaw_std_rad
                if stage == "rough_single_tag"
                else registration.model_yaw_std_rad
            ),
            correction_tilt_rad=snapshot.tilt_rad,
            reprojection_error_px=snapshot.reprojection_error_px,
            odom_match_error_ms=snapshot.odom_match_error_ms,
            odom_time_source=snapshot.odom_time_source,
            registration=registration,
            application_state="pending" if job.apply_requested else "not_requested",
        )
        candidate = self._evaluate_candidate_apply_locked(candidate)
        if job.apply_requested and not candidate.can_apply:
            raise RuntimeError(candidate.apply_reason)
        job.candidate = candidate
        job.temporary_window = temporary_window
        job.journal.write(
            "candidate_frozen",
            candidate=self._candidate_audit(candidate),
            temporary_window=[
                item.audit_dict(self.config.window) for item in temporary_window
            ],
            evicted_tag_ids=[
                item.tag_id
                for item in self._window
                if all(item.tag_id != kept.tag_id for kept in temporary_window)
            ],
            pre_eviction_registration=(
                self._registration_audit(job.pre_eviction_registration)
                if job.pre_eviction_registration is not None
                else None
            ),
            thresholds=self.config.window.__dict__,
            fcu_reference={
                "lever_arm_m": self.config.fcu_reference.lever_arm_m.tolist(),
                "active_mode_before": (
                    "tag_world_xy_local_z"
                    if self._extnav.correction_valid
                    else "local_identity_origin"
                ),
                "candidate_mode_after": "tag_world_xy_local_z",
                "require_zero_installation_angles": (
                    self.config.fcu_reference.require_zero_installation_angles
                ),
            },
        )

    def _evaluate_candidate_apply_locked(
        self, candidate: WindowCandidate
    ) -> WindowCandidate:
        """核对物理参数并在同一新鲜 raw 上计算实际 FCU 中心跳变。"""
        ready, reason = self._extnav_ready_locked()
        if not ready:
            return replace(candidate, can_apply=False, apply_reason=reason)
        extnav = self._extnav
        if extnav.odin_session_id != candidate.odin_session_id:
            return replace(
                candidate, can_apply=False, apply_reason="候选 Odin session 已失效"
            )
        if extnav.revision != candidate.expected_extnav_revision:
            return replace(
                candidate,
                can_apply=False,
                apply_reason="extnav revision 已改变，必须刷新并重新显式提交",
            )
        if candidate.aged(time.monotonic(), self.config.window):
            return replace(
                candidate, can_apply=False, apply_reason="候选已超过最大年龄"
            )
        expected_lever = self.config.fcu_reference.lever_arm_m
        if not np.allclose(
            extnav.lever_arm_m,
            expected_lever,
            atol=self.config.fcu_reference.parameter_tolerance,
            rtol=0.0,
        ):
            return replace(
                candidate, can_apply=False, apply_reason="extnav 杆臂与已推导配置不一致"
            )
        if (
            self.config.fcu_reference.require_zero_installation_angles
            and not np.allclose(
                extnav.installation_rpy,
                np.zeros(3),
                atol=self.INSTALLATION_ANGLE_TOLERANCE_RAD,
                rtol=0.0,
            )
        ):
            return replace(
                candidate,
                can_apply=False,
                apply_reason="extnav 安装角非零，中心公式需重推",
            )
        if (
            self._latest_raw_transform is None
            or self._raw_received_monotonic <= 0.0
            or time.monotonic() - self._raw_received_monotonic > 0.5
        ):
            return replace(
                candidate, can_apply=False, apply_reason="没有新鲜 raw 复算实际跳变"
            )
        old = (
            (
                extnav.correction_x_m,
                extnav.correction_y_m,
                extnav.correction_yaw_rad,
            )
            if extnav.correction_valid
            else None
        )
        new = (candidate.x_m, candidate.y_m, candidate.yaw_rad)
        position_jump, yaw_jump = expected_fcu_jump(
            self._latest_raw_transform, old, new, expected_lever
        )
        delta_x = candidate.x_m - (
            extnav.correction_x_m if extnav.correction_valid else 0.0
        )
        delta_y = candidate.y_m - (
            extnav.correction_y_m if extnav.correction_valid else 0.0
        )
        delta_yaw = wrap_angle(
            candidate.yaw_rad
            - (extnav.correction_yaw_rad if extnav.correction_valid else 0.0)
        )
        if not is_apply_time_source_safe(candidate.odom_time_source):
            reason = "候选混用了时间轴或未使用历史时间匹配"
            can_apply = False
        elif position_jump > self.config.window.max_apply_position_jump_m:
            reason = "当前飞控参考中心预计水平跳变超限"
            can_apply = False
        elif yaw_jump > self.config.window.max_apply_yaw_jump_rad:
            reason = "当前姿态预计 yaw 跳变超限"
            can_apply = False
        else:
            limitation = (
                "；两点解离群识别能力有限"
                if candidate.registration.two_point_limited
                else ""
            )
            reason = (
                "候选可显式应用；PoseStamped 无 estimator reset counter，"
                "ACK 不代表 EKF 稳定" + limitation
            )
            can_apply = True
        return replace(
            candidate,
            delta_x_m=delta_x,
            delta_y_m=delta_y,
            delta_yaw_rad=delta_yaw,
            expected_position_jump_m=position_jump,
            expected_yaw_jump_rad=yaw_jump,
            can_apply=can_apply,
            apply_reason=reason,
        )

    def _save_dry_run_and_finalize(self, job: _Job) -> None:
        """资源释放后原子保存 dry-run 窗口；不接触 extnav。"""
        assert job.candidate is not None
        try:
            with self._lock:
                candidate = self._commit_new_window_locked(
                    job, job.candidate, application_state="not_requested"
                )
                job.candidate = candidate
                job.application_state = "not_requested"
                job.outcome = "saved_unapplied"
        except RuntimeError as exc:
            self._finish_job(job, success=False, message=str(exc))
            return
        self._finish_job(
            job,
            success=True,
            message="候选和 keyframe 已保存；dry-run 未调用 extnav",
        )

    def _apply_and_finalize(self, job: _Job) -> None:
        """资源释放后提交冻结候选，并按 ACK/status/unknown 分别落账。"""
        assert job.candidate is not None
        with self._lock:
            job.state = CorrectionStatus.STATE_APPLYING
            job.message = "资源已释放；正在提交冻结候选并等待 ACK/权威状态对账"
            job.apply_request_sent = True
            job.application_state = "pending"
            job.candidate = replace(job.candidate, application_state="pending")
            self._pending_candidate = job.candidate
        outcome = self._apply_to_extnav(job, job.candidate)
        success = False
        try:
            with self._lock:
                job.application_state = outcome.state
                job.application_confirmed_by_status = outcome.confirmed_by_status
                job.extnav_revision = outcome.revision
                candidate = replace(
                    job.candidate,
                    application_state=outcome.state,
                    application_confirmed_by_status=outcome.confirmed_by_status,
                )
                job.candidate = candidate
                if outcome.state in ("applied_ack", "applied_status"):
                    if job.increment_window_on_apply:
                        candidate = self._commit_new_window_locked(
                            job,
                            candidate,
                            application_state=outcome.state,
                            extnav_revision=outcome.revision,
                        )
                        job.candidate = candidate
                    else:
                        self._saved_candidate = replace(
                            candidate,
                            saved=True,
                            saved_window_revision=self._window_revision,
                            expected_extnav_revision=outcome.revision,
                        )
                        job.candidate = self._saved_candidate
                    self._pending_candidate = None
                    job.outcome = outcome.state
                    success = True
                elif outcome.state == "rejected":
                    if (
                        self._pending_candidate is not None
                        and self._pending_candidate.saved
                    ):
                        self._saved_candidate = replace(
                            self._pending_candidate, application_state="rejected"
                        )
                    self._pending_candidate = None
                    job.outcome = "application_rejected"
                else:
                    self._pending_candidate = replace(
                        candidate, application_state="unknown"
                    )
                    if candidate.saved:
                        self._saved_candidate = self._pending_candidate
                    job.outcome = "application_unknown"
        except RuntimeError as exc:
            # extnav 已确认后本地保存失败不能伪装成未应用或自动回滚。
            with self._lock:
                job.application_state = outcome.state
                job.outcome = "applied_but_window_commit_failed"
                job.candidate = replace(
                    job.candidate,
                    application_state=outcome.state,
                    application_confirmed_by_status=outcome.confirmed_by_status,
                )
                # pending 只表示应用事实未知；这里应用已由 ACK/状态确认，不能误报 unknown。
                self._pending_candidate = None
                self._window_message = (
                    "extnav 已确认应用，但临时窗口未保存；正式窗口保持原状"
                )
            self._finish_job(
                job,
                success=False,
                message=f"{outcome.message}；但本地窗口保存失败：{exc}",
            )
            return
        self._finish_job(job, success=success, message=outcome.message)

    def _commit_new_window_locked(
        self,
        job: _Job,
        candidate: WindowCandidate,
        *,
        application_state: str,
        extnav_revision: int | None = None,
    ) -> WindowCandidate:
        """CAS 保存临时窗口并且仅在此处增加 N/window revision。"""
        if self._window_revision != job.base_window_revision:
            raise RuntimeError("保存前 window revision 已改变，旧窗口保持不变")
        ready, reason = self._extnav_ready_locked()
        if not ready or self._extnav.odin_session_id != job.odin_session_id:
            raise RuntimeError(reason or "保存前 Odin session 已改变")
        if candidate.config_fingerprint != self.config.config_fingerprint:
            raise RuntimeError("保存前配置指纹已改变")
        self._window = candidate.keyframes
        self._successful_calibrations += 1
        self._window_revision += 1
        job.window_saved = True
        applied_revision = (
            int(extnav_revision)
            if extnav_revision is not None
            else self._extnav.revision
        )
        saved = replace(
            candidate,
            saved=True,
            saved_window_revision=self._window_revision,
            expected_extnav_revision=applied_revision,
            application_state=application_state,
        )
        self._saved_candidate = saved
        self._window_message = (
            f"已保存第 {self._successful_calibrations} 次校准；"
            f"窗口 {len(self._window)}/{self._window_size}"
        )
        job.journal.write(
            "window_committed",
            window_revision=self._window_revision,
            successful_calibrations=self._successful_calibrations,
            application_state=application_state,
            extnav_revision=applied_revision,
            window=[item.audit_dict(self.config.window) for item in self._window],
        )
        return saved

    def _apply_to_extnav(
        self, job: _Job, candidate: WindowCandidate
    ) -> _ApplicationOutcome:
        """提交一次幂等 job；丢 ACK 后只凭新鲜权威状态有限对账。"""
        with self._lock:
            ready, reason = self._extnav_ready_locked()
            if not ready:
                return _ApplicationOutcome("rejected", self._extnav.revision, reason)
            if (
                self._extnav.odin_session_id != candidate.odin_session_id
                or self._extnav.revision != candidate.expected_extnav_revision
            ):
                state = "unknown" if job.retrying_unknown else "rejected"
                return _ApplicationOutcome(
                    state,
                    self._extnav.revision,
                    "提交前 extnav session/revision 已改变；不能强行刷新 CAS",
                )
        if not self._set_client.wait_for_service(timeout_sec=2.0):
            return _ApplicationOutcome(
                "rejected",
                candidate.expected_extnav_revision,
                "未发现 extnav SetCorrection 服务",
            )
        request = SetCorrection.Request()
        request.job_id = candidate.application_job_id
        request.odin_session_id = candidate.odin_session_id
        request.expected_revision = candidate.expected_extnav_revision
        request.valid = True
        request.correction_x_m = candidate.x_m
        request.correction_y_m = candidate.y_m
        request.correction_yaw_rad = candidate.yaw_rad
        request.sample_count = sum(
            item.samples_accepted for item in candidate.keyframes
        )
        request.position_std_m = candidate.position_std_m
        request.yaw_std_rad = candidate.yaw_std_rad
        request.reprojection_error_px = candidate.reprojection_error_px
        request.odom_match_error_ms = candidate.odom_match_error_ms
        job.journal.write(
            "extnav_request_sent",
            job_id=request.job_id,
            session=request.odin_session_id,
            expected_revision=request.expected_revision,
            candidate=[candidate.x_m, candidate.y_m, candidate.yaw_rad],
            expected_fcu_jump=[
                candidate.expected_position_jump_m,
                candidate.expected_yaw_jump_rad,
            ],
            fcu_reference_mode="tag_world_xy_local_z",
            lever_arm_m=self.config.fcu_reference.lever_arm_m.tolist(),
        )
        future = self._set_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if completed.wait(timeout=self.config.timeouts.extnav_apply_seconds):
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - rclpy future 可透传任意服务端异常
                return self._reconcile_application(
                    candidate, f"SetCorrection 返回异常：{exc}"
                )
            if (
                response is not None
                and response.accepted
                and response.applied
                and response.odin_session_id == candidate.odin_session_id
                and int(response.revision) >= candidate.expected_extnav_revision
            ):
                job.journal.write(
                    "extnav_applied_ack",
                    revision=int(response.revision),
                    response=str(response.message),
                )
                return _ApplicationOutcome(
                    "applied_ack",
                    int(response.revision),
                    "extnav ACK 已确认应用；这不代表 MAVROS EKF 已稳定接受坐标重置",
                )
            detail = response.message if response is not None else "空响应"
            if job.retrying_unknown:
                return _ApplicationOutcome(
                    "unknown",
                    candidate.expected_extnav_revision,
                    f"unknown 幂等重试被拒绝（{detail}）；无法断言历史请求从未应用",
                )
            job.journal.write("extnav_rejected", response=str(detail))
            return _ApplicationOutcome(
                "rejected",
                candidate.expected_extnav_revision,
                f"extnav 明确拒绝候选：{detail}",
            )
        return self._reconcile_application(candidate, "等待 extnav ACK 超时")

    def _reconcile_application(
        self, candidate: WindowCandidate, trigger: str
    ) -> _ApplicationOutcome:
        """ACK 不确定时检查 applied_job_id/session/revision/数值的权威状态证据。"""
        deadline = time.monotonic() + self.config.timeouts.extnav_reconcile_seconds
        while time.monotonic() < deadline:
            with self._lock:
                extnav = self._extnav
                fresh = (
                    extnav.received_monotonic > 0.0
                    and time.monotonic() - extnav.received_monotonic
                    <= self.EXTNAV_STATUS_MAX_AGE_S
                )
                matches = (
                    fresh
                    and extnav.interface_version == INTERFACE_VERSION
                    and extnav.odin_session_id == candidate.odin_session_id
                    and extnav.correction_valid
                    and extnav.applied_job_id == candidate.application_job_id
                    and extnav.revision > candidate.expected_extnav_revision
                    and np.allclose(
                        (
                            extnav.correction_x_m,
                            extnav.correction_y_m,
                            extnav.correction_yaw_rad,
                        ),
                        (candidate.x_m, candidate.y_m, candidate.yaw_rad),
                        atol=1e-9,
                        rtol=0.0,
                    )
                )
                revision = extnav.revision
            if matches:
                return _ApplicationOutcome(
                    "applied_status",
                    revision,
                    f"{trigger}；经新鲜 extnav 权威状态确认已应用（非服务 ACK）",
                    confirmed_by_status=True,
                )
            time.sleep(0.05)
        return _ApplicationOutcome(
            "unknown",
            candidate.expected_extnav_revision,
            f"{trigger}且状态对账仍无定论；锁存 application_unknown，禁止覆盖",
        )

    def _raise_if_stopping(self, job: _Job) -> None:
        """提交前响应 stop；提交后 stop 不撤销已发出的服务请求。"""
        with self._lock:
            if job.failure_reason:
                raise RuntimeError(job.failure_reason)
            if job.user_stop and not job.apply_request_sent:
                raise _UserStopped
        if (
            job.stop_event.is_set() or self._closing.is_set()
        ) and not job.apply_request_sent:
            raise RuntimeError("节点关闭，操作取消")

    def _fail_job_locked(self, job: _Job, reason: str) -> None:
        """从订阅回调异步令任务失败，不在 executor 回调中阻塞清理。"""
        if not job.failure_reason:
            job.failure_reason = reason
            job.last_error = reason
            job.stop_event.set()

    def _invalidate_window_locked(self, reason: str) -> None:
        """清空所有可继续原始 Q 状态并递增 revision；不改 extnav active。"""
        had_state = bool(
            self._window
            or self._saved_candidate is not None
            or self._pending_candidate is not None
            or self._successful_calibrations
        )
        self._window = ()
        self._saved_candidate = None
        self._pending_candidate = None
        self._successful_calibrations = 0
        self._window_revision += 1
        self._window_message = reason
        self._audit.write(
            "window_invalidated",
            reason=reason,
            had_state=had_state,
            window_revision=self._window_revision,
            extnav_untouched=True,
        )

    def _window_is_stale_locked(self) -> bool:
        """窗口整体过期时拒绝偷偷淘汰至单点再当作首次。"""
        if not self._window:
            return False
        ordered = tuple(sorted(self._window, key=lambda item: item.created_monotonic))
        if (
            time.monotonic() - ordered[-1].created_monotonic
            > self.config.window.maximum_keyframe_gap_seconds
        ):
            return True
        return (
            ordered[-1].created_monotonic - ordered[0].created_monotonic
            > self.config.window.maximum_window_span_seconds
        )

    def _finish_job(self, job: _Job, *, success: bool, message: str) -> None:
        """冻结互相独立的保存、应用、资源事实并发布可靠终态。"""
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
        job.journal.write(
            "job_finished",
            success=success,
            window_saved=job.window_saved,
            resources_released=job.resources_released,
            application_state=job.application_state,
            application_confirmed_by_status=job.application_confirmed_by_status,
            extnav_revision=job.extnav_revision,
            outcome=job.outcome,
            message=message,
            candidate=self._candidate_audit(job.candidate),
        )
        self._result_pub.publish(self._make_result(job, success))
        self._service_logger.info(
            "操作结束 job=%s success=%s saved=%s app=%s resources=%s outcome=%s",
            job.job_id,
            success,
            job.window_saved,
            job.application_state,
            job.resources_released,
            job.outcome,
        )
        self._publish_status()

    def _make_result(self, job: _Job, success: bool) -> CorrectionResult:
        """把冻结操作状态转换为接口 2.0 可靠结果。"""
        snapshot = job.snapshot
        candidate = job.candidate
        message = CorrectionResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.interface_version = INTERFACE_VERSION
        message.service_instance_id = self._service_instance_id
        message.job_id = job.job_id
        message.operation = job.operation
        message.expected_tag_id = job.expected_tag_id
        message.detected_tag_id = job.detected_tag_id
        message.apply_requested = job.apply_requested
        message.success = success
        message.window_saved = job.window_saved
        message.resources_released = job.resources_released
        message.application_state = APPLICATION_TO_MESSAGE.get(
            job.application_state, CorrectionStatus.APPLICATION_NONE
        )
        message.application_state_text = job.application_state
        message.application_confirmed_by_status = job.application_confirmed_by_status
        message.applied = job.application_state in ("applied_ack", "applied_status")
        message.odin_session_id = job.odin_session_id
        message.window_revision = self._window_revision
        message.window_count = len(self._window)
        message.window_size = self._window_size
        message.successful_calibrations = self._successful_calibrations
        message.extnav_revision = job.extnav_revision
        message.correction_x_m = _candidate_value(candidate, "x_m")
        message.correction_y_m = _candidate_value(candidate, "y_m")
        message.correction_yaw_deg = _candidate_degrees(candidate, "yaw_rad")
        message.correction_tilt_deg = _candidate_degrees(
            candidate, "correction_tilt_rad"
        )
        message.samples_accepted = job.samples_accepted
        message.samples_rejected = job.samples_rejected
        message.position_std_m = _candidate_value(candidate, "position_std_m")
        message.yaw_std_deg = _candidate_degrees(candidate, "yaw_std_rad")
        message.reprojection_error_px = _candidate_value(
            candidate, "reprojection_error_px"
        )
        message.odom_match_error_ms = _candidate_value(candidate, "odom_match_error_ms")
        registration = candidate.registration if candidate is not None else None
        message.registration_rms_m = _registration_value(registration, "rms_m")
        message.registration_max_residual_m = _registration_value(
            registration, "max_residual_m"
        )
        message.model_yaw_std_deg = _registration_degrees(
            registration, "model_yaw_std_rad"
        )
        message.expected_position_jump_m = _candidate_value(
            candidate, "expected_position_jump_m"
        )
        message.expected_yaw_jump_deg = _candidate_degrees(
            candidate, "expected_yaw_jump_rad"
        )
        message.duration_s = max(0.0, time.monotonic() - job.started_monotonic)
        message.processing_rate_hz = snapshot.processing_rate_hz
        message.processing_time_ms = snapshot.processing_time_ms
        message.candidate_stage = candidate.stage if candidate is not None else ""
        message.outcome = job.outcome
        message.message = job.message
        message.log_path = str(job.journal.path)
        return message

    def _publish_status(self) -> None:
        """发布任务之外仍完整保留的服务端窗口、候选和 pending 状态。"""
        with self._lock:
            job = self._job
            candidate = (
                job.candidate
                if job is not None and job.active and job.candidate is not None
                else self._pending_candidate or self._saved_candidate
            )
            message = CorrectionStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.interface_version = INTERFACE_VERSION
            message.service_available = not bool(self._resource_fault)
            message.service_instance_id = self._service_instance_id
            message.window_revision = self._window_revision
            message.config_fingerprint = self.config.config_fingerprint
            message.window_size = self._window_size
            message.window_count = len(self._window)
            message.successful_calibrations = self._successful_calibrations
            message.next_calibration_number = self._successful_calibrations + 1
            message.keyframes = [self._keyframe_message(item) for item in self._window]
            message.window_state = self._window_state_locked()
            message.window_message = self._window_message
            if job is None:
                message.active = False
                message.state = CorrectionStatus.STATE_IDLE
                message.state_text = STATE_TEXT[CorrectionStatus.STATE_IDLE]
                message.operation = CorrectionStatus.OPERATION_NONE
                message.operation_text = OPERATION_TEXT[CorrectionStatus.OPERATION_NONE]
                message.expected_tag_id = -1
                message.detected_tag_id = -1
                message.resources_released = self._odom_subscription is None
                message.message = "服务空闲；下视相机和任务 Odin 订阅已关闭"
            else:
                snapshot = job.snapshot
                message.active = job.active
                message.state = job.state
                message.state_text = STATE_TEXT.get(job.state, "unknown")
                message.operation = job.operation
                message.operation_text = OPERATION_TEXT.get(job.operation, "unknown")
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
                message.window_saved = job.window_saved
                message.resources_released = job.resources_released
                message.extnav_revision = job.extnav_revision
                message.log_path = str(job.journal.path)
                message.message = job.message
                message.last_error = job.last_error
            self._fill_candidate_status(message, candidate)
            if self._resource_fault:
                message.last_error = self._resource_fault
                message.message = f"资源故障：{self._resource_fault}"
        self._status_pub.publish(message)

    def _fill_candidate_status(
        self, message: CorrectionStatus, candidate: WindowCandidate | None
    ) -> None:
        """把粗/精候选及配准、delta、跳变和应用事实投影到状态。"""
        if candidate is None:
            message.baseline_tag_a = -1
            message.baseline_tag_b = -1
            message.application_state = CorrectionStatus.APPLICATION_NONE
            message.application_state_text = "none"
            return
        registration = candidate.registration
        message.candidate_saved = candidate.saved
        message.candidate_valid = candidate.valid
        message.candidate_stale = candidate.aged(time.monotonic(), self.config.window)
        message.candidate_stage = candidate.stage
        message.candidate_window_revision = candidate.saved_window_revision
        message.candidate_extnav_revision = candidate.expected_extnav_revision
        message.candidate_created_ns = max(0, candidate.created_ros_ns)
        message.candidate_x_m = candidate.x_m
        message.candidate_y_m = candidate.y_m
        message.candidate_yaw_deg = math.degrees(candidate.yaw_rad)
        message.correction_tilt_deg = math.degrees(candidate.correction_tilt_rad)
        message.position_std_m = candidate.position_std_m
        message.yaw_std_deg = math.degrees(candidate.yaw_std_rad)
        message.reprojection_error_px = candidate.reprojection_error_px
        message.odom_match_error_ms = candidate.odom_match_error_ms
        message.odom_time_source = candidate.odom_time_source
        message.converged = candidate.valid
        message.baseline_tag_a = registration.baseline_tag_a
        message.baseline_tag_b = registration.baseline_tag_b
        message.odin_baseline_m = registration.odin_baseline_m
        message.world_baseline_m = registration.world_baseline_m
        message.odin_baseline_direction_deg = math.degrees(
            registration.odin_baseline_direction_rad
        )
        message.world_baseline_direction_deg = math.degrees(
            registration.world_baseline_direction_rad
        )
        message.registration_rms_m = _finite_or_zero(registration.rms_m)
        message.registration_max_residual_m = _finite_or_zero(
            registration.max_residual_m
        )
        message.model_yaw_std_deg = _degrees_or_zero(registration.model_yaw_std_rad)
        message.two_point_limited = registration.two_point_limited
        delta_x = candidate.delta_x_m
        delta_y = candidate.delta_y_m
        delta_yaw = candidate.delta_yaw_rad
        can_apply = candidate.can_apply
        apply_reason = candidate.apply_reason
        extnav = self._extnav
        extnav_fresh = (
            extnav.received_monotonic > 0.0
            and time.monotonic() - extnav.received_monotonic
            <= self.EXTNAV_STATUS_MAX_AGE_S
        )
        if extnav_fresh and extnav.odin_session_id == candidate.odin_session_id:
            active_x = extnav.correction_x_m if extnav.correction_valid else 0.0
            active_y = extnav.correction_y_m if extnav.correction_valid else 0.0
            active_yaw = extnav.correction_yaw_rad if extnav.correction_valid else 0.0
            delta_x = candidate.x_m - active_x
            delta_y = candidate.y_m - active_y
            delta_yaw = wrap_angle(candidate.yaw_rad - active_yaw)
            if (
                self._pending_candidate is None
                and extnav.revision != candidate.expected_extnav_revision
            ):
                can_apply = False
                apply_reason = (
                    "extnav revision 已改变；delta 已按当前 active 刷新，"
                    "应用当前结果时将重新获取 raw 并复算实际跳变"
                )
        message.delta_x_m = _finite_or_zero(delta_x)
        message.delta_y_m = _finite_or_zero(delta_y)
        message.delta_yaw_deg = _degrees_or_zero(delta_yaw)
        message.expected_position_jump_m = _finite_or_zero(
            candidate.expected_position_jump_m
        )
        message.expected_yaw_jump_deg = _degrees_or_zero(
            candidate.expected_yaw_jump_rad
        )
        message.can_apply = (
            can_apply
            and not message.candidate_stale
            and self._pending_candidate is None
        )
        message.apply_reason = apply_reason
        state = (
            self._pending_candidate.application_state
            if self._pending_candidate is not None
            else candidate.application_state
        )
        message.application_state = APPLICATION_TO_MESSAGE.get(
            state, CorrectionStatus.APPLICATION_NONE
        )
        message.application_state_text = state
        message.application_confirmed_by_status = (
            candidate.application_confirmed_by_status
        )
        message.extnav_applied = state in ("applied_ack", "applied_status")

    def _keyframe_message(
        self, keyframe: CalibrationKeyframe
    ) -> CalibrationKeyframeMessage:
        """把服务端 keyframe 完整投影为 ROS 快照。"""
        message = CalibrationKeyframeMessage()
        message.tag_id = keyframe.tag_id
        message.world_x_m = keyframe.world_x_m
        message.world_y_m = keyframe.world_y_m
        message.odin_x_m = keyframe.odin_x_m
        message.odin_y_m = keyframe.odin_y_m
        message.capture_start_ns = max(0, keyframe.capture_start_ns)
        message.capture_end_ns = max(0, keyframe.capture_end_ns)
        message.span_s = keyframe.span_s
        message.samples_accepted = keyframe.samples_accepted
        message.samples_rejected = keyframe.samples_rejected
        message.independent_blocks = keyframe.independent_blocks
        message.position_std_m = keyframe.position_std_m
        message.effective_sigma_m = keyframe.effective_sigma_m
        message.weight = keyframe.weight(self.config.window)
        message.reprojection_error_px = keyframe.reprojection_error_px
        message.odom_match_error_ms = keyframe.odom_match_error_ms
        message.max_sync_motion_error_m = keyframe.max_sync_motion_error_m
        message.correction_tilt_deg = math.degrees(keyframe.correction_tilt_rad)
        message.single_tag_yaw_deg = math.degrees(keyframe.single_tag_yaw_rad)
        message.prior_model_residual_m = _finite_or_zero(
            keyframe.prior_model_residual_m
        )
        message.odom_time_source = keyframe.odom_time_source
        message.odin_session_id = keyframe.odin_session_id
        message.config_fingerprint = keyframe.config_fingerprint
        message.source_job_id = keyframe.source_job_id
        return message

    def _window_state_locked(self) -> str:
        """生成 GUI 门控所需的权威窗口状态。"""
        if self._pending_candidate is not None:
            return "application_unknown"
        if not self._window:
            return "empty"
        if self._window_is_stale_locked():
            return "stale"
        if len(self._window) == 1:
            return "rough"
        if len(self._window) == 2:
            return "two_point_limited"
        return "refined"

    def _candidate_audit(
        self, candidate: WindowCandidate | None
    ) -> dict[str, object] | None:
        """生成候选、质量、delta 与应用状态的严格 JSON 快照。"""
        if candidate is None:
            return None
        return {
            "stage": candidate.stage,
            "correction": [candidate.x_m, candidate.y_m, candidate.yaw_rad],
            "session": candidate.odin_session_id,
            "config_fingerprint": candidate.config_fingerprint,
            "source_job_id": candidate.source_job_id,
            "application_job_id": candidate.application_job_id,
            "base_window_revision": candidate.base_window_revision,
            "expected_extnav_revision": candidate.expected_extnav_revision,
            "valid": candidate.valid,
            "quality_reason": candidate.quality_reason,
            "position_std_m": candidate.position_std_m,
            "yaw_std_rad": candidate.yaw_std_rad,
            "odom_time_source": candidate.odom_time_source,
            "registration": self._registration_audit(candidate.registration),
            "delta": [
                candidate.delta_x_m,
                candidate.delta_y_m,
                candidate.delta_yaw_rad,
            ],
            "expected_jump": [
                candidate.expected_position_jump_m,
                candidate.expected_yaw_jump_rad,
            ],
            "can_apply": candidate.can_apply,
            "apply_reason": candidate.apply_reason,
            "saved": candidate.saved,
            "saved_window_revision": candidate.saved_window_revision,
            "application_state": candidate.application_state,
            "application_confirmed_by_status": (
                candidate.application_confirmed_by_status
            ),
            "fcu_reference": {
                "candidate_mode": "tag_world_xy_local_z",
                "lever_arm_m": self.config.fcu_reference.lever_arm_m.tolist(),
                "require_zero_installation_angles": (
                    self.config.fcu_reference.require_zero_installation_angles
                ),
            },
        }

    @staticmethod
    def _registration_audit(
        registration: RegistrationResult | None,
    ) -> dict[str, object] | None:
        """序列化全部配准诊断，保留两点能力限制。"""
        if registration is None:
            return None
        return {
            "valid": registration.valid,
            "reason": registration.reason,
            "x_m": registration.x_m,
            "y_m": registration.y_m,
            "yaw_rad": registration.yaw_rad,
            "rms_m": registration.rms_m,
            "max_residual_m": registration.max_residual_m,
            "model_yaw_std_rad": registration.model_yaw_std_rad,
            "residuals_m": registration.residuals_m,
            "baseline_tags": [
                registration.baseline_tag_a,
                registration.baseline_tag_b,
            ],
            "odin_baseline_m": registration.odin_baseline_m,
            "world_baseline_m": registration.world_baseline_m,
            "two_point_limited": registration.two_point_limited,
        }

    def _drain_image_queue(self) -> None:
        """新任务不得处理上一任务残留帧。"""
        while True:
            try:
                self._image_queue.get_nowait()
            except queue.Empty:
                return

    def close(self) -> None:
        """停止任务并释放本节点资源；不会清除 extnav active correction。"""
        self._closing.set()
        with self._lock:
            job = self._job
            if job is not None and job.active:
                job.stop_event.set()
            thread = job.thread if job is not None else None
        if thread is not None and thread.is_alive():
            thread.join(timeout=12.0)
        self._stop_image_capture()
        self._stop_odometry_capture()


class _UserStopped(Exception):
    """内部控制流：用户 stop 与故障/节点关闭采用不同终态。"""


def _finite_or_zero(value: float) -> float:
    """ROS 标量配合显式 valid/state 字段投影缺失值；日志仍保存 null。"""
    return float(value) if math.isfinite(value) else 0.0


def _degrees_or_zero(value: float) -> float:
    """有限弧度转度；缺失由相邻状态字段解释。"""
    return math.degrees(value) if math.isfinite(value) else 0.0


def _candidate_value(candidate: WindowCandidate | None, field_name: str) -> float:
    """安全读取候选有限标量。"""
    return _finite_or_zero(float(getattr(candidate, field_name, math.nan)))


def _candidate_degrees(candidate: WindowCandidate | None, field_name: str) -> float:
    """安全读取候选弧度并转度。"""
    return _degrees_or_zero(float(getattr(candidate, field_name, math.nan)))


def _registration_value(
    registration: RegistrationResult | None, field_name: str
) -> float:
    """安全读取配准有限标量。"""
    return _finite_or_zero(float(getattr(registration, field_name, math.nan)))


def _registration_degrees(
    registration: RegistrationResult | None, field_name: str
) -> float:
    """安全读取配准弧度并转度。"""
    return _degrees_or_zero(float(getattr(registration, field_name, math.nan)))


def main(args=None) -> None:
    """多线程 executor 允许图像/Odin/status 在操作线程等待 ACK 时继续推进。"""
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
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
