#!/usr/bin/env python3
"""Odin 到 MAVROS 的外部导航桥，并在同一节点内原子维护可选 SE(2) 修正。

原始 /odin1/odometry_highfreq 始终是唯一输入。修正接口包缺失、尚未校准、
Odin 断流/重启或 correction_service 崩溃时，本节点继续 identity 透传原始数据。
"""

from __future__ import annotations

import copy
import math
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

try:
    from correction_interfaces.msg import ExtnavCorrectionStatus
    from correction_interfaces.srv import SetCorrection

    CORRECTION_API_AVAILABLE = True
except ImportError:
    # 缺少新增接口绝不能让既有 Odin→MAVROS 链路停止。
    ExtnavCorrectionStatus = None  # type: ignore[assignment,misc]
    SetCorrection = None  # type: ignore[assignment,misc]
    CORRECTION_API_AVAILABLE = False


CORRECTION_INTERFACE_VERSION = "1.0"


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """ZYX 欧拉角转 ROS xyzw 四元数。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_quaternion(quaternion: tuple[float, ...]) -> tuple[float, ...]:
    """规范化 xyzw 四元数并拒绝非有限或零范数输入。"""
    values = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(float(value) for value in values / norm)


def multiply_quaternions(
    first_xyzw: tuple[float, ...], second_xyzw: tuple[float, ...]
) -> tuple[float, ...]:
    """Hamilton 四元数乘法 first * second。"""
    x1, y1, z1, w1 = first_xyzw
    x2, y2, z2, w2 = second_xyzw
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_to_rotation_matrix(q_xyzw: tuple[float, ...]) -> np.ndarray:
    """规范化四元数并转换为 3x3 旋转矩阵。"""
    x, y, z, w = normalize_quaternion(q_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _rotate_covariance(covariance: list[float], transform: np.ndarray) -> list[float]:
    """按 6D 块变换旋转有效 covariance；ROS 的 -1 unknown 哨兵原样保留。"""
    values = np.asarray(covariance, dtype=np.float64)
    if values.size != 36 or values[0] < 0.0 or not np.isfinite(values).all():
        return list(covariance)
    matrix = values.reshape(6, 6)
    rotated = transform @ matrix @ transform.T
    return [float(value) for value in rotated.reshape(-1)]


def apply_planar_correction(
    message: Odometry,
    correction_x_m: float,
    correction_y_m: float,
    correction_yaw_rad: float,
) -> Odometry:
    """左乘 C_world_odin；z 不平移，姿态和水平线速度按同一 yaw 旋转。"""
    if not all(
        math.isfinite(value)
        for value in (correction_x_m, correction_y_m, correction_yaw_rad)
    ):
        raise ValueError("non-finite correction")
    corrected = copy.deepcopy(message)
    cosine, sine = math.cos(correction_yaw_rad), math.sin(correction_yaw_rad)
    rotation = np.array(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))
    position = message.pose.pose.position
    corrected.pose.pose.position.x = (
        cosine * position.x - sine * position.y + correction_x_m
    )
    corrected.pose.pose.position.y = (
        sine * position.x + cosine * position.y + correction_y_m
    )
    corrected.pose.pose.position.z = position.z

    orientation = message.pose.pose.orientation
    raw_quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    yaw_quaternion = rpy_to_quaternion(0.0, 0.0, correction_yaw_rad)
    q_corrected = normalize_quaternion(
        multiply_quaternions(yaw_quaternion, normalize_quaternion(raw_quaternion))
    )
    (
        corrected.pose.pose.orientation.x,
        corrected.pose.pose.orientation.y,
        corrected.pose.pose.orientation.z,
        corrected.pose.pose.orientation.w,
    ) = q_corrected

    # 当前 Odin/extnav 生产链把 linear velocity 作为 odom/world 向量使用；
    # angular velocity 仍是机体系量，不随世界坐标重建旋转。
    linear = message.twist.twist.linear
    corrected.twist.twist.linear.x = cosine * linear.x - sine * linear.y
    corrected.twist.twist.linear.y = sine * linear.x + cosine * linear.y
    corrected.twist.twist.linear.z = linear.z
    pose_transform = np.zeros((6, 6), dtype=np.float64)
    pose_transform[:3, :3] = rotation
    pose_transform[3:, 3:] = rotation
    twist_transform = np.eye(6, dtype=np.float64)
    twist_transform[:3, :3] = rotation
    corrected.pose.covariance = _rotate_covariance(
        message.pose.covariance, pose_transform
    )
    corrected.twist.covariance = _rotate_covariance(
        message.twist.covariance, twist_transform
    )
    return corrected


def _stamp_nanoseconds(message: Odometry) -> int:
    """取 Odin header 时间；零时间仍由 gap 判据识别 session。"""
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class OdometryBridge(Node):
    """保持原双频 extnav 输出，并原子维护 correction revision/session。"""

    def __init__(self, **node_kwargs) -> None:
        """创建生产节点；可注入独立 rclpy context 供隔离集成测试使用。"""
        super().__init__("extnav_to_vision_pose", **node_kwargs)

        self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("pose_fcu_topic", "/extnav/pose_fcu")
        self.declare_parameter("vel_fcu_topic", "/extnav/velocity_fcu")
        self.declare_parameter("vision_rate_hz", 30.0)
        self.declare_parameter("ctrl_rate_hz", 100.0)
        self.declare_parameter("odom_topic", "/odin1/odometry_highfreq")
        self.declare_parameter(
            "corrected_odom_topic", "/odin1/odometry_highfreq_corrected"
        )
        self.declare_parameter("correction_status_topic", "/extnav/correction_status")
        self.declare_parameter("set_correction_service", "/extnav/set_correction")
        self.declare_parameter("odin_session_gap_seconds", 2.0)
        self.declare_parameter("odin_stamp_regression_seconds", 0.25)
        self.declare_parameter("max_correction_translation_m", 100.0)

        self.declare_parameter("roll_cam", 0.0)
        self.declare_parameter("pitch_cam", 0.0)
        self.declare_parameter("yaw_cam", 0.0)
        self.declare_parameter("odin_x", 0.0)
        self.declare_parameter("odin_y", 0.0)
        self.declare_parameter("odin_z", 0.0)

        self.roll_cam = float(self.get_parameter("roll_cam").value)
        self.pitch_cam = float(self.get_parameter("pitch_cam").value)
        self.yaw_cam = float(self.get_parameter("yaw_cam").value)
        self.session_gap_seconds = float(
            self.get_parameter("odin_session_gap_seconds").value
        )
        self.stamp_regression_ns = int(
            float(self.get_parameter("odin_stamp_regression_seconds").value) * 1e9
        )
        self.max_correction_translation_m = float(
            self.get_parameter("max_correction_translation_m").value
        )
        if self.session_gap_seconds <= 0.0 or self.max_correction_translation_m <= 0.0:
            raise ValueError("invalid correction/session limit")

        self.T = np.array(
            [
                self.get_parameter("odin_x").value,
                self.get_parameter("odin_y").value,
                self.get_parameter("odin_z").value,
            ],
            dtype=np.float64,
        )
        q_x = rpy_to_quaternion(self.roll_cam, 0.0, 0.0)
        q_y = rpy_to_quaternion(0.0, self.pitch_cam, 0.0)
        q_z = rpy_to_quaternion(0.0, 0.0, self.yaw_cam)
        self.q_IO = normalize_quaternion(
            multiply_quaternions(multiply_quaternions(q_x, q_y), q_z)
        )
        qx, qy, qz, qw = self.q_IO
        self.q_IO_conj = (-qx, -qy, -qz, qw)
        self.R_IO = quaternion_to_rotation_matrix(self.q_IO)

        vision_topic = str(self.get_parameter("vision_pose_topic").value)
        pose_fcu_topic = str(self.get_parameter("pose_fcu_topic").value)
        velocity_fcu_topic = str(self.get_parameter("vel_fcu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        corrected_topic = str(self.get_parameter("corrected_odom_topic").value)
        vision_rate_hz = float(self.get_parameter("vision_rate_hz").value)
        ctrl_rate_hz = float(self.get_parameter("ctrl_rate_hz").value)
        if vision_rate_hz <= 0.0 or ctrl_rate_hz <= 0.0:
            raise ValueError("invalid publication rate")

        self.vis_pub = self.create_publisher(PoseStamped, vision_topic, 10)
        self.pose_fcu_pub = self.create_publisher(PoseStamped, pose_fcu_topic, 10)
        self.vel_fcu_pub = self.create_publisher(TwistStamped, velocity_fcu_topic, 10)
        self.corrected_pub = self.create_publisher(
            Odometry, corrected_topic, qos_profile_sensor_data
        )
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data
        )

        self.latest_msg: Odometry | None = None
        self.last_raw_monotonic = 0.0
        self.last_raw_stamp_ns = 0
        self.last_frame_pair: tuple[str, str] | None = None
        self.odin_available = False
        self.odin_session_id = ""
        self.raw_messages = 0
        self.corrected_messages = 0
        self.correction_valid = False
        self.correction_x_m = 0.0
        self.correction_y_m = 0.0
        self.correction_yaw_rad = 0.0
        self.revision = 0
        self.reset_counter = 0
        self.applied_job_id = ""
        self.last_event = "extnav 启动，identity passthrough"
        self.last_error = ""

        self.status_pub = None
        self.set_service = None
        if CORRECTION_API_AVAILABLE:
            transient_status = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status_pub = self.create_publisher(
                ExtnavCorrectionStatus,
                str(self.get_parameter("correction_status_topic").value),
                transient_status,
            )
            self.set_service = self.create_service(
                SetCorrection,
                str(self.get_parameter("set_correction_service").value),
                self._set_correction,
            )
        else:
            self.last_error = "correction_interfaces 不可用；保持 identity passthrough"
            self.get_logger().error(self.last_error)

        self.timer_vision = self.create_timer(
            1.0 / vision_rate_hz, self.vision_timer_callback
        )
        self.timer_ctrl = self.create_timer(
            1.0 / ctrl_rate_hz, self.ctrl_timer_callback
        )
        self.status_timer = self.create_timer(0.5, self._status_timer_callback)

        self.get_logger().info(
            f"Bridge started: raw={odom_topic}, corrected={corrected_topic}, "
            f"vision={vision_topic}@{vision_rate_hz}Hz, control@{ctrl_rate_hz}Hz"
        )
        self.get_logger().info(
            f"cam_rpy=({self.roll_cam},{self.pitch_cam},{self.yaw_cam}), "
            f"T=({self.T[0]:.2f},{self.T[1]:.2f},{self.T[2]:.2f})"
        )
        self.get_logger().warning(
            "TODO(task27-reset-counter): 当前 /mavros/vision_pose/pose 是 PoseStamped，"
            "无法携带 MAVLink estimator reset_counter；本节点仅在修正状态中递增计数。"
        )

    def _odom_cb(self, message: Odometry) -> None:
        """识别 Odin session、应用当前快照，并无条件发布 corrected 话题。"""
        now_monotonic = time.monotonic()
        stamp_ns = _stamp_nanoseconds(message)
        frame_pair = (message.header.frame_id, message.child_frame_id)
        gap = (
            now_monotonic - self.last_raw_monotonic
            if self.last_raw_monotonic > 0.0
            else 0.0
        )
        new_session_reason = ""
        if not self.odin_available or not self.odin_session_id:
            new_session_reason = "首个 Odin 样本"
        elif gap > self.session_gap_seconds:
            new_session_reason = f"Odin 断流 {gap:.3f}s 后恢复"
        elif (
            self.last_raw_stamp_ns > 0
            and stamp_ns > 0
            and stamp_ns < self.last_raw_stamp_ns - self.stamp_regression_ns
        ):
            new_session_reason = "Odin 时间戳回退"
        elif self.last_frame_pair is not None and frame_pair != self.last_frame_pair:
            new_session_reason = "Odin frame_id 改变"
        if new_session_reason:
            self._begin_odin_session(message, new_session_reason)

        self.last_raw_monotonic = now_monotonic
        self.last_raw_stamp_ns = stamp_ns
        self.last_frame_pair = frame_pair
        self.odin_available = True
        self.raw_messages += 1
        try:
            corrected = (
                apply_planar_correction(
                    message,
                    self.correction_x_m,
                    self.correction_y_m,
                    self.correction_yaw_rad,
                )
                if self.correction_valid
                else copy.deepcopy(message)
            )
        except Exception as exc:
            self._invalidate(f"修正计算异常，退回 identity：{exc}")
            corrected = copy.deepcopy(message)
        self.latest_msg = corrected
        self.corrected_pub.publish(corrected)
        self.corrected_messages += 1

    def _begin_odin_session(self, message: Odometry, reason: str) -> None:
        """新建不可跨重启复用的 session，并先使旧修正失效。"""
        if self.correction_valid:
            self._invalidate(f"{reason}，旧修正立即失效")
        stamp_ns = _stamp_nanoseconds(message)
        self.odin_session_id = f"odin-{stamp_ns}-{uuid.uuid4().hex[:8]}"
        self.odin_available = True
        self.last_event = f"{reason}：session={self.odin_session_id}"
        self.last_error = ""
        self.get_logger().warning(self.last_event)
        self._publish_status()

    def _advance_revision(self) -> None:
        """每次 active 坐标变换改变时递增 revision 和内部 reset counter。"""
        self.revision += 1
        self.reset_counter = (self.reset_counter + 1) % 256

    def _invalidate(self, reason: str) -> None:
        """原子退回 identity；不停止 raw/corrected/MAVROS 数据流。"""
        if self.correction_valid:
            self._advance_revision()
        self.correction_valid = False
        self.correction_x_m = 0.0
        self.correction_y_m = 0.0
        self.correction_yaw_rad = 0.0
        self.applied_job_id = ""
        self.last_event = reason
        self.last_error = reason
        self.get_logger().warning(reason)
        self._publish_status()

    def _set_correction(self, request, response):
        """以 session+revision CAS 原子应用候选；失败不改变当前 active 值。"""
        response.accepted = False
        response.applied = False
        response.revision = self.revision
        response.odin_session_id = self.odin_session_id
        response.message = "请求被拒绝"

        requested_values = (
            float(request.correction_x_m),
            float(request.correction_y_m),
            float(request.correction_yaw_rad),
        )
        if request.valid and (
            request.job_id == self.applied_job_id
            and request.odin_session_id == self.odin_session_id
            and self.correction_valid
            and np.allclose(
                requested_values,
                (self.correction_x_m, self.correction_y_m, self.correction_yaw_rad),
                atol=1e-12,
            )
        ):
            response.accepted = True
            response.applied = True
            response.message = "同一 job 已幂等应用"
            return response
        if int(request.expected_revision) != self.revision:
            response.message = (
                f"revision 冲突：期望 {request.expected_revision}，当前 {self.revision}"
            )
            return response
        if not request.valid:
            if (
                request.odin_session_id
                and request.odin_session_id != self.odin_session_id
            ):
                response.message = "清除请求的 Odin session 不匹配"
                return response
            changed = self.correction_valid
            if changed:
                self._advance_revision()
            self.correction_valid = False
            self.correction_x_m = 0.0
            self.correction_y_m = 0.0
            self.correction_yaw_rad = 0.0
            self.applied_job_id = ""
            self.last_event = "收到显式清除请求，identity passthrough"
            self.last_error = ""
            response.accepted = True
            response.applied = False
            response.revision = self.revision
            response.message = self.last_event
            self._publish_status()
            return response

        if not self.odin_available or not self.odin_session_id:
            response.message = "Odin 当前不可用"
            return response
        if request.odin_session_id != self.odin_session_id:
            response.message = "候选 Odin session 已失效"
            return response
        quality_values = (
            request.position_std_m,
            request.yaw_std_rad,
            request.reprojection_error_px,
            request.odom_match_error_ms,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in quality_values):
            response.message = "候选质量指标无效"
            return response
        if int(request.sample_count) == 0:
            response.message = "候选样本数为零"
            return response
        if not all(math.isfinite(value) for value in requested_values):
            response.message = "候选修正含非有限值"
            return response
        if (
            math.hypot(requested_values[0], requested_values[1])
            > self.max_correction_translation_m
        ):
            response.message = "候选水平修正超过 extnav 绝对上限"
            return response

        self.correction_x_m = requested_values[0]
        self.correction_y_m = requested_values[1]
        self.correction_yaw_rad = math.atan2(
            math.sin(requested_values[2]), math.cos(requested_values[2])
        )
        self.correction_valid = True
        self.applied_job_id = str(request.job_id)
        self._advance_revision()
        self.last_event = (
            f"已应用 job={self.applied_job_id} revision={self.revision} "
            f"x={self.correction_x_m:+.4f} y={self.correction_y_m:+.4f} "
            f"yaw={math.degrees(self.correction_yaw_rad):+.4f}deg"
        )
        self.last_error = ""
        self.get_logger().warning(self.last_event)
        response.accepted = True
        response.applied = True
        response.revision = self.revision
        response.odin_session_id = self.odin_session_id
        response.message = self.last_event
        self._publish_status()
        return response

    def _status_timer_callback(self) -> None:
        """Odin 超时即发布 invalid，并周期发布最新透传/修正计数。"""
        if self.last_raw_monotonic > 0.0:
            age = time.monotonic() - self.last_raw_monotonic
            if age > self.session_gap_seconds and self.odin_available:
                self.odin_available = False
                self.odin_session_id = ""
                if self.correction_valid:
                    self._invalidate(f"Odin 数据超时 {age:.3f}s，旧修正立即失效")
                else:
                    self.last_event = f"Odin 数据超时 {age:.3f}s，identity 等待恢复"
        self._publish_status()

    def _publish_status(self) -> None:
        """发布 extnav 权威 active correction；接口缺失时静默保持原链路。"""
        if self.status_pub is None:
            return
        message = ExtnavCorrectionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.interface_version = CORRECTION_INTERFACE_VERSION
        message.service_available = True
        message.odin_available = self.odin_available
        message.odin_session_id = self.odin_session_id
        message.correction_valid = self.correction_valid
        message.revision = self.revision
        message.reset_counter = self.reset_counter
        message.correction_x_m = self.correction_x_m
        message.correction_y_m = self.correction_y_m
        message.correction_yaw_deg = math.degrees(self.correction_yaw_rad)
        message.applied_job_id = self.applied_job_id
        message.raw_age_s = (
            max(0.0, time.monotonic() - self.last_raw_monotonic)
            if self.last_raw_monotonic > 0.0
            else math.inf
        )
        message.raw_messages = self.raw_messages
        message.corrected_messages = self.corrected_messages
        message.last_event = self.last_event
        message.last_error = self.last_error
        self.status_pub.publish(message)

    def _get_data(self):
        """从实际将发送给 MAVROS 的最新 Odometry 提取数据。"""
        if self.latest_msg is None:
            return None
        stamp = self.get_clock().now().to_msg()
        pose = self.latest_msg.pose.pose
        twist = self.latest_msg.twist.twist
        return pose.position, pose.orientation, twist.linear, twist.angular, stamp

    def _apply_transform(self, position, quaternion, linear, angular):
        """保留既有 Odin 安装与 FCU lever-arm 变换。"""
        q_odom = normalize_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )
        p_odom = np.array((position.x, position.y, position.z))
        v_odom = np.array((linear.x, linear.y, linear.z))
        w_odom = np.array((angular.x, angular.y, angular.z))
        q_imu = multiply_quaternions(self.q_IO, q_odom)
        q_imu = normalize_quaternion(multiply_quaternions(q_imu, self.q_IO_conj))
        rotation_imu = quaternion_to_rotation_matrix(q_imu)
        p_imu = p_odom + self.T - rotation_imu @ self.T
        v_imu = v_odom - rotation_imu @ self.R_IO @ np.cross(
            w_odom, self.R_IO.T @ self.T
        )
        return p_imu, v_imu, q_imu

    @staticmethod
    def _make_pose(stamp, position, quaternion):
        """构造既有 MAVROS/FCU PoseStamped 输出。"""
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    @staticmethod
    def _make_twist(stamp, velocity):
        """构造既有控制器 TwistStamped 输出。"""
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.twist.linear.x = float(velocity[0])
        message.twist.linear.y = float(velocity[1])
        message.twist.linear.z = float(velocity[2])
        return message

    def vision_timer_callback(self) -> None:
        """按配置频率向 MAVROS 发布最终修正后的视觉位姿。"""
        data = self._get_data()
        if data is None:
            return
        position, quaternion, linear, angular, stamp = data
        p_imu, _v_imu, q_imu = self._apply_transform(
            position, quaternion, linear, angular
        )
        self.vis_pub.publish(self._make_pose(stamp, p_imu, q_imu))

    def ctrl_timer_callback(self) -> None:
        """按 100 Hz 发布与 MAVROS 使用同一修正快照的控制位姿/速度。"""
        data = self._get_data()
        if data is None:
            return
        position, quaternion, linear, angular, stamp = data
        p_imu, v_imu, q_imu = self._apply_transform(
            position, quaternion, linear, angular
        )
        self.pose_fcu_pub.publish(self._make_pose(stamp, p_imu, q_imu))
        self.vel_fcu_pub.publish(self._make_twist(stamp, v_imu))


def main(args=None) -> None:
    """ROS 2 可执行入口。"""
    rclpy.init(args=args)
    node = OdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
