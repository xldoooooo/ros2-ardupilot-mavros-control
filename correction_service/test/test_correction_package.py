"""ament/colcon 包内快速回归，避免部署构建只验证“能够安装”。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from correction_service.camera_process import parse_v4l2_control_value
from correction_service.geometry import planar_transform
from correction_service.synchronizer import is_apply_time_source_safe

from correction_service.config import load_config

PACKAGE_ROOT = Path(__file__).resolve().parent.parent


def test_calibrated_configuration_loads_for_expected_tag() -> None:
    """随包部署的真机配置必须保持可解析且明确使用 tag36h11/tag 0。"""
    config = load_config(PACKAGE_ROOT / "config")

    assert config.detection.tag_family == "tag36h11"
    assert config.camera.width == 1920
    assert config.camera.height == 1080
    assert config.tags[0].size_m == 0.170


def test_planar_correction_rotates_translation_and_preserves_z() -> None:
    """extnav 所需 SE(2) 应左乘位姿，而不是分别给 x/y/yaw 加常数。"""
    pose = np.eye(4, dtype=float)
    pose[:3, 3] = (1.0, 2.0, 0.7)
    corrected = planar_transform(10.0, -3.0, math.pi / 2.0) @ pose

    assert np.allclose(corrected[:3, 3], (8.0, -2.0, 0.7), atol=1e-12)


def test_runtime_compatibility_helpers_accept_real_aircraft_values() -> None:
    """飞机 V4L2 枚举读回与跨 epoch 历史匹配均应允许应用。"""
    assert parse_v4l2_control_value("auto_exposure: 1 (Manual Mode)") == 1
    assert is_apply_time_source_safe("arrival_history")
