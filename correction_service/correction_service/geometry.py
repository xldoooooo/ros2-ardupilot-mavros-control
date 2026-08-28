"""AprilTag、相机、Odin IMU 的完整 SE(3) 组合与最终 SE(2) 提取。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import TagPose


@dataclass(frozen=True)
class PlanarCorrection:
    """由完整空间变换得到、可交给 extnav 的水平修正候选。"""

    x_m: float
    y_m: float
    yaw_rad: float
    tilt_rad: float
    world_imu: np.ndarray
    world_from_odin: np.ndarray


def wrap_angle(angle: float) -> float:
    """把弧度角规范到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """把 ROS xyzw 四元数转换为正交旋转矩阵。"""
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("四元数无效")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """把正交旋转矩阵稳定转换为 ROS xyzw 四元数。"""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def rotation_z(yaw_rad: float) -> np.ndarray:
    """构造绕世界 Z 轴的旋转矩阵。"""
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """组合 3x3 旋转和三维平移为 4x4 刚体变换。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def transform_from_pose(
    position_xyz: np.ndarray, quaternion_xyzw: np.ndarray
) -> np.ndarray:
    """从 ROS 位姿建立坐标系变换。"""
    position = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    if not np.isfinite(position).all():
        raise ValueError("位姿平移含非有限值")
    return homogeneous(quaternion_to_matrix(quaternion_xyzw), position)


def world_tag_transform(tag: TagPose) -> np.ndarray:
    """Tag 配置坐标系：+X 指向图案上方，+Y 指向图案左方，+Z 朝上。"""
    return homogeneous(rotation_z(tag.yaw_rad), np.array((tag.x, tag.y, tag.z)))


def configured_tag_from_standard() -> np.ndarray:
    """把 OpenCV 的 +X右/+Y上 Tag 坐标转换为配置的 +X上/+Y左。"""
    # p_standard = R_standard_configured * p_configured
    rotation = np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    return homogeneous(rotation, np.zeros(3))


def compute_planar_correction(
    camera_from_tag_standard: np.ndarray,
    tag: TagPose,
    imu_from_camera: np.ndarray,
    odin_from_imu: np.ndarray,
) -> PlanarCorrection:
    """按任务规定先完成 SE(3) 链，再仅提取世界<-Odin 的 x/y/yaw。"""
    camera_from_tag_standard = np.asarray(
        camera_from_tag_standard, dtype=np.float64
    ).reshape(4, 4)
    imu_from_camera = np.asarray(imu_from_camera, dtype=np.float64).reshape(4, 4)
    odin_from_imu = np.asarray(odin_from_imu, dtype=np.float64).reshape(4, 4)
    if not all(
        np.isfinite(item).all()
        for item in (camera_from_tag_standard, imu_from_camera, odin_from_imu)
    ):
        raise ValueError("空间变换含非有限值")

    camera_from_tag_configured = (
        camera_from_tag_standard @ configured_tag_from_standard()
    )
    world_from_camera = world_tag_transform(tag) @ np.linalg.inv(
        camera_from_tag_configured
    )
    world_from_imu = world_from_camera @ np.linalg.inv(imu_from_camera)
    world_from_odin = world_from_imu @ np.linalg.inv(odin_from_imu)

    rotation = world_from_odin[:3, :3]
    yaw = wrap_angle(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))
    # 非平面分量只用于质量门控；extnav 明确不应用 z/roll/pitch。
    tilt = math.acos(float(np.clip(rotation[2, 2], -1.0, 1.0)))
    return PlanarCorrection(
        x_m=float(world_from_odin[0, 3]),
        y_m=float(world_from_odin[1, 3]),
        yaw_rad=yaw,
        tilt_rad=tilt,
        world_imu=world_from_imu,
        world_from_odin=world_from_odin,
    )


def planar_transform(x_m: float, y_m: float, yaw_rad: float) -> np.ndarray:
    """构造 extnav 实际应用的 SE(2) 嵌入 SE(3) 变换。"""
    return homogeneous(rotation_z(yaw_rad), np.array((x_m, y_m, 0.0)))
