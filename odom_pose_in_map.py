#!/usr/bin/env python3
"""由 odom->map TF 和 odom->imu 里程计解算 map->imu 位姿并打印。"""

import math
import time
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """归一化 ROS xyzw 四元数，拒绝无效姿态。"""
    norm = math.sqrt(sum(component * component for component in q))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("quaternion is zero or non-finite")
    return tuple(component / norm for component in q)  # type: ignore[return-value]


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    """返回单位四元数的逆；输入顺序为 ROS 的 x, y, z, w。"""
    return -q[0], -q[1], -q[2], q[3]


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    """计算 Hamilton 积 lhs * rhs，表示先应用 rhs 再应用 lhs。"""
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    """用单位四元数旋转三维向量。"""
    vx, vy, vz = vector
    qx, qy, qz, qw = q
    # 展开 q * [v, 0] * q^-1，避免为每帧创建两个临时四元数。
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_map_pose(
    odom_to_map_translation: Vector3,
    odom_to_map_rotation: Quaternion,
    odom_to_imu_position: Vector3,
    odom_to_imu_rotation: Quaternion,
) -> Tuple[Vector3, Quaternion]:
    """计算 T_map_imu = inverse(T_odom_map) * T_odom_imu。"""
    q_odom_map = normalize_quaternion(odom_to_map_rotation)
    q_odom_imu = normalize_quaternion(odom_to_imu_rotation)
    q_map_odom = quaternion_conjugate(q_odom_map)

    displacement_in_odom = tuple(
        odom_to_imu_position[index] - odom_to_map_translation[index]
        for index in range(3)
    )
    position_in_map = rotate_vector(q_map_odom, displacement_in_odom)
    rotation_in_map = normalize_quaternion(
        quaternion_multiply(q_map_odom, q_odom_imu)
    )
    return position_in_map, rotation_in_map


def quaternion_to_rpy(q: Quaternion) -> Vector3:
    """把单位四元数转换为 XYZ 固定轴 roll/pitch/yaw，单位为弧度。"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class OdomPoseInMap(Node):
    """缓存 odom->map 动态 TF，并对每帧 Odin odometry 做坐标变换。"""

    def __init__(self) -> None:
        """创建两个只读订阅；节点不会发布消息或调用任何飞控服务。"""
        super().__init__("odom_pose_in_map")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("odometry_topic", "/odin1/odometry_highfreq")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_rate_hz", 10.0)

        self.tf_topic = str(self.get_parameter("tf_topic").value)
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.output_period = self._output_period(
            float(self.get_parameter("output_rate_hz").value)
        )

        self.odom_to_map: Tuple[Vector3, Quaternion] | None = None
        self.last_output_monotonic = 0.0
        self.last_missing_tf_warning = 0.0

        # /tf 与高频传感器话题均只需最新数据，BEST_EFFORT 避免积压旧样本。
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_subscription = self.create_subscription(
            TFMessage, self.tf_topic, self._on_tf, sensor_qos
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.odometry_topic, self._on_odometry, sensor_qos
        )

        self.get_logger().info(
            f"listening to {self.tf_topic} ({self.odom_frame}->{self.map_frame}) "
            f"and {self.odometry_topic}"
        )
        print(
            "stamp[s] map_frame child_frame "
            "x[m] y[m] z[m] qx qy qz qw roll[deg] pitch[deg] yaw[deg]",
            flush=True,
        )

    @staticmethod
    def _output_period(rate_hz: float) -> float:
        """将打印频率转为周期；非正值表示打印每一帧。"""
        if not math.isfinite(rate_hz):
            raise ValueError("output_rate_hz must be finite")
        return 0.0 if rate_hz <= 0.0 else 1.0 / rate_hz

    def _on_tf(self, message: TFMessage) -> None:
        """从可能包含多个变换的 /tf 消息中提取 odom->map。"""
        for transform in message.transforms:
            if (
                transform.header.frame_id != self.odom_frame
                or transform.child_frame_id != self.map_frame
            ):
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            candidate = (
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            if not all(math.isfinite(value) for group in candidate for value in group):
                self.get_logger().warning("ignored non-finite odom->map TF")
                continue
            try:
                self.odom_to_map = (
                    candidate[0],
                    normalize_quaternion(candidate[1]),
                )
            except ValueError as error:
                self.get_logger().warning(f"ignored invalid odom->map TF: {error}")

    def _on_odometry(self, message: Odometry) -> None:
        """使用最近收到的 odom->map TF 解算当前 map->imu 位姿并限频打印。"""
        if message.header.frame_id != self.odom_frame:
            self.get_logger().warning(
                f"ignored odometry frame '{message.header.frame_id}', "
                f"expected '{self.odom_frame}'",
                throttle_duration_sec=2.0,
            )
            return
        if self.odom_to_map is None:
            now = time.monotonic()
            if now - self.last_missing_tf_warning >= 2.0:
                self.get_logger().warning(
                    f"waiting for {self.odom_frame}->{self.map_frame} on {self.tf_topic}"
                )
                self.last_missing_tf_warning = now
            return

        pose = message.pose.pose
        odom_position = (pose.position.x, pose.position.y, pose.position.z)
        odom_rotation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        try:
            map_position, map_rotation = compose_map_pose(
                self.odom_to_map[0],
                self.odom_to_map[1],
                odom_position,
                odom_rotation,
            )
        except ValueError as error:
            self.get_logger().warning(f"ignored invalid odometry pose: {error}")
            return

        now = time.monotonic()
        if self.output_period > 0.0 and (
            now - self.last_output_monotonic < self.output_period
        ):
            return
        self.last_output_monotonic = now

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        roll, pitch, yaw = quaternion_to_rpy(map_rotation)
        child_frame = message.child_frame_id or "imu"
        print(
            f"{stamp:.9f} {self.map_frame} {child_frame} "
            f"{map_position[0]:+.6f} {map_position[1]:+.6f} "
            f"{map_position[2]:+.6f} {map_rotation[0]:+.7f} "
            f"{map_rotation[1]:+.7f} {map_rotation[2]:+.7f} "
            f"{map_rotation[3]:+.7f} {math.degrees(roll):+.3f} "
            f"{math.degrees(pitch):+.3f} {math.degrees(yaw):+.3f}",
            flush=True,
        )


def main(args=None) -> None:
    """运行只读解算节点，并在 Ctrl+C 后干净退出。"""
    rclpy.init(args=args)
    node = OdomPoseInMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
