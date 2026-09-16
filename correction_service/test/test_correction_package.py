"""ament/colcon 包内快速回归，避免部署构建只验证“能够安装”。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from correction_service.camera_process import parse_v4l2_control_value
from correction_service.geometry import (
    compute_planar_correction,
    configured_tag_from_standard,
    homogeneous,
    planar_transform,
    rotation_z,
    world_tag_transform,
)
from correction_service.synchronizer import is_apply_time_source_safe

from correction_service.config import load_config

PACKAGE_ROOT = Path(__file__).resolve().parent.parent


def test_aircraft_configuration_loads_for_expected_tag() -> None:
    """真机临时配置须可解析，并保留上一颗相机的安装方向假设。"""
    config = load_config(PACKAGE_ROOT / "config")

    assert config.detection.tag_family == "tag36h11"
    assert config.camera.width == 1920
    assert config.camera.height == 1080
    assert config.camera.fps == 120
    assert config.tags[0].size_m == 0.170
    assert config.detection.max_correction_tilt_deg == 10.0
    assert math.degrees(config.window.max_apply_yaw_jump_rad) == 45.0
    assert list(config.lens_controls)[:3] == [
        "auto_exposure",
        "white_balance_automatic",
        "focus_automatic_continuous",
    ]
    assert config.lens_controls["auto_exposure"] == 3
    assert config.lens_controls["brightness"] == 0
    assert math.isclose(config.intrinsics.calibration_rms_px, 0.8863085437393902)
    assert np.allclose(config.intrinsics.distortion,
                       [0.03626268524324754, -0.056048601794863075,
                        0.0010844756398792788, -0.0010052529149732062, 0.00958122664915688])
    rotation = config.t_imu_camera[:3, :3]
    # Task32撤销把Tag约定错误归因于相机的180°补偿；原始画面上方接近
    # Odin +X，光轴接近 Odin -Z。Tag官方图案与OpenCV角点差异由geometry处理。
    image_up_in_imu = rotation @ np.array((0.0, -1.0, 0.0))
    optical_axis_in_imu = rotation @ np.array((0.0, 0.0, 1.0))
    assert image_up_in_imu[0] > math.cos(math.radians(10.0))
    assert optical_axis_in_imu[2] < -math.cos(math.radians(10.0))


def test_installed_camera_profile_archives_are_complete() -> None:
    """ament 安装包必须递归携带 Wasintek/UQ212 档案，不能只保留活动根配置。"""
    required = {"camera.conf", "intrinsics.yaml", "extrinsics.yaml", "lens.conf"}
    for profile_name in ("Wasintek", "UQ212"):
        profile = PACKAGE_ROOT / "config" / profile_name
        assert profile.is_dir()
        assert {path.name for path in profile.iterdir()} == required


def test_planar_correction_rotates_translation_and_preserves_z() -> None:
    """extnav 所需 SE(2) 应左乘位姿，而不是分别给 x/y/yaw 加常数。"""
    pose = np.eye(4, dtype=float)
    pose[:3, 3] = (1.0, 2.0, 0.7)
    corrected = planar_transform(10.0, -3.0, math.pi / 2.0) @ pose

    assert np.allclose(corrected[:3, 3], (8.0, -2.0, 0.7), atol=1e-12)


def test_single_tag_keeps_task27_full_se3_xy_translation() -> None:
    """包级回归：首次非零 tilt 仍须直接投影 C_full，而非单点 P/Q 重锚。"""
    config = load_config(PACKAGE_ROOT / "config")
    tag = config.tags[0]
    pitch = math.radians(7.0)
    pitch_rotation = np.array(
        (
            (math.cos(pitch), 0.0, math.sin(pitch)),
            (0.0, 1.0, 0.0),
            (-math.sin(pitch), 0.0, math.cos(pitch)),
        )
    )
    expected = homogeneous(
        rotation_z(math.radians(-15.0)) @ pitch_rotation,
        np.array((0.08, -0.04, 0.0)),
    )
    odin_from_imu = homogeneous(
        rotation_z(math.radians(9.0)), np.array((0.4, -0.2, 0.7))
    )
    world_from_imu = expected @ odin_from_imu
    world_from_camera = world_from_imu @ config.t_imu_camera
    camera_from_tag_configured = np.linalg.inv(world_from_camera) @ world_tag_transform(
        tag
    )
    camera_from_tag_standard = camera_from_tag_configured @ np.linalg.inv(
        configured_tag_from_standard()
    )

    correction = compute_planar_correction(
        camera_from_tag_standard, tag, config.t_imu_camera, odin_from_imu
    )

    assert np.allclose(correction.world_from_odin, expected, atol=1e-10)
    assert np.allclose((correction.x_m, correction.y_m), expected[:2, 3], atol=1e-10)


def test_runtime_compatibility_helpers_accept_real_aircraft_values() -> None:
    """飞机 V4L2 枚举读回与跨 epoch 历史匹配均应允许应用。"""
    assert parse_v4l2_control_value("auto_exposure: 1 (Manual Mode)") == 1
    assert is_apply_time_source_safe("arrival_history")
