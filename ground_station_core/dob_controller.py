"""键盘悬停与航点跟踪共用的 PD+DOB 位置控制器。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .config import GRAVITY_ACC
from .models import VehicleSnapshot


@dataclass(frozen=True)
class DobGains:
    """PD+DOB 控制器增益及推力映射参数。"""

    wn_xy: float
    zeta_xy: float
    wn_z: float
    zeta_z: float
    observer_xy: float
    observer_z: float
    hover_throttle: float
    thrust_ratio: float
    uav_weight: float

    @classmethod
    def from_mapping(cls, params: dict[str, float]) -> "DobGains":
        """从项目 YAML 使用的参数名创建增益对象。"""
        return cls(
            wn_xy=params["hover_wn_xy"],
            zeta_xy=params["hover_zeta_xy"],
            wn_z=params["hover_wn_z"],
            zeta_z=params["hover_zeta_z"],
            observer_xy=params["dob_L_xy"],
            observer_z=params["dob_L_z"],
            hover_throttle=params["hover_throttle"],
            thrust_ratio=params["thrust_ratio"],
            uav_weight=params["uav_weight"],
        )


class DobPositionController:
    """复刻原 keyboard_vel_controller.cpp 的位置 PD 与一阶 DOB。"""

    def __init__(self, gains: DobGains) -> None:
        """保存增益并创建零状态观测器。"""
        self._gains = gains
        self.reset()

    def reset(self) -> None:
        """清空观测器状态；下一次发布先发送位置设定点。"""
        self._first_frame = True
        self._z_x = self._z_y = self._z_z = 0.0
        self._d_x = self._d_y = self._d_z = 0.0
        self._u_x = self._u_y = self._u_z = 0.0
        self._last_time = time.monotonic()

    @staticmethod
    def _quat_from_rotmat(matrix: tuple[tuple[float, ...], ...]) -> tuple[float, ...]:
        """用 Shepperd 方法将旋转矩阵转换为 (x, y, z, w) 四元数。"""
        (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = matrix
        trace = r00 + r11 + r22
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w, x, y, z = (
                0.25 * scale,
                (r21 - r12) / scale,
                (r02 - r20) / scale,
                (r10 - r01) / scale,
            )
        elif r00 > r11 and r00 > r22:
            scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
            w, x, y, z = (
                (r21 - r12) / scale,
                0.25 * scale,
                (r01 + r10) / scale,
                (r02 + r20) / scale,
            )
        elif r11 > r22:
            scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
            w, x, y, z = (
                (r02 - r20) / scale,
                (r01 + r10) / scale,
                0.25 * scale,
                (r12 + r21) / scale,
            )
        else:
            scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
            w, x, y, z = (
                (r10 - r01) / scale,
                (r02 + r20) / scale,
                (r12 + r21) / scale,
                0.25 * scale,
            )
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        return x / norm, y / norm, z / norm, w / norm

    def publish(self, node, attitude_publisher, position_publisher, snapshot: VehicleSnapshot,
                target: tuple[float, float, float, float]) -> None:
        """根据状态与目标计算一次控制量并发布 MAVROS setpoint。"""
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import AttitudeTarget

        target_x, target_y, target_z, target_yaw = target
        if self._first_frame:
            self._first_frame = False
            self._last_time = time.monotonic()
            pose = PoseStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = target_z
            pose.pose.orientation.z = math.sin(target_yaw / 2.0)
            pose.pose.orientation.w = math.cos(target_yaw / 2.0)
            position_publisher.publish(pose)
            return

        gains = self._gains
        kp_xy = gains.wn_xy * gains.wn_xy
        kd_xy = 2.0 * gains.zeta_xy * gains.wn_xy
        kp_z = gains.wn_z * gains.wn_z
        kd_z = 2.0 * gains.zeta_z * gains.wn_z
        acc_x = kp_xy * (target_x - snapshot.x) - kd_xy * snapshot.vx
        acc_y = kp_xy * (target_y - snapshot.y) - kd_xy * snapshot.vy
        acc_z = kp_z * (target_z - snapshot.z) - kd_z * snapshot.vz

        # 半隐式离散化的一阶扰动观测器，与原实现的 5ms 更新阈值一致。
        now = time.monotonic()
        dt = now - self._last_time
        if dt > 0.005:
            l_xy, l_z = gains.observer_xy, gains.observer_z
            denominator_xy = l_xy * dt + 1.0
            denominator_z = l_z * dt + 1.0
            self._z_x = (
                self._z_x - l_xy * l_xy * dt * snapshot.vx - l_xy * self._u_x * dt
            ) / denominator_xy
            self._z_y = (
                self._z_y - l_xy * l_xy * dt * snapshot.vy - l_xy * self._u_y * dt
            ) / denominator_xy
            self._z_z = (
                self._z_z - l_z * l_z * dt * snapshot.vz - l_z * self._u_z * dt
            ) / denominator_z
            self._d_x = self._z_x + l_xy * snapshot.vx
            self._d_y = self._z_y + l_xy * snapshot.vy
            self._d_z = self._z_z + l_z * snapshot.vz
            self._last_time = now

        self._u_x = acc_x - self._d_x
        self._u_y = acc_y - self._d_y
        self._u_z = acc_z - self._d_z
        total_x, total_y = self._u_x, self._u_y
        total_z = self._u_z + GRAVITY_ACC
        acceleration_norm = math.sqrt(
            total_x * total_x + total_y * total_y + total_z * total_z
        )

        epsilon = 1e-6
        if acceleration_norm < epsilon:
            quat = (0.0, 0.0, math.sin(target_yaw / 2.0), math.cos(target_yaw / 2.0))
        else:
            body_x = total_x / acceleration_norm
            body_y = total_y / acceleration_norm
            body_z = total_z / acceleration_norm
            yaw_x, yaw_y = math.cos(target_yaw), math.sin(target_yaw)
            projection = yaw_x * body_x + yaw_y * body_y
            control_x = yaw_x - projection * body_x
            control_y = yaw_y - projection * body_y
            control_norm = math.hypot(control_x, control_y)
            if control_norm < epsilon:
                control_x, control_y, control_norm = 1.0, 0.0, 1.0
            control_x, control_y = control_x / control_norm, control_y / control_norm
            lateral_x = -body_z * control_y
            lateral_y = body_z * control_x
            lateral_z = body_x * control_y - body_y * control_x
            lateral_norm = math.sqrt(
                lateral_x * lateral_x + lateral_y * lateral_y + lateral_z * lateral_z
            )
            if lateral_norm < epsilon:
                lateral_x, lateral_y, lateral_z, lateral_norm = 0.0, 1.0, 0.0, 1.0
            lateral_x /= lateral_norm
            lateral_y /= lateral_norm
            lateral_z /= lateral_norm
            quat = self._quat_from_rotmat(
                (
                    (control_x, lateral_x, body_x),
                    (control_y, lateral_y, body_y),
                    (0.0, lateral_z, body_z),
                )
            )

        weight = gains.uav_weight * GRAVITY_ACC
        thrust = gains.uav_weight * acceleration_norm
        maximum = weight * gains.thrust_ratio
        if thrust <= weight:
            throttle = gains.hover_throttle * thrust / weight
        else:
            throttle = gains.hover_throttle + (1.0 - gains.hover_throttle) * (
                thrust - weight
            ) / (maximum - weight)
        throttle = max(0.0, min(1.0, throttle))

        message = AttitudeTarget()
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = quat
        message.thrust = throttle
        message.type_mask = 1 + 2 + 4  # 忽略三个机体系角速度，只使用姿态与推力。
        message.header.stamp = node.get_clock().now().to_msg()
        attitude_publisher.publish(message)
