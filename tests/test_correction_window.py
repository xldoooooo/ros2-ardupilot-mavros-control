"""task29 多 Tag 配准、keyframe 不确定度、FIFO 和飞控中心数学回归。"""

from __future__ import annotations

import importlib.util
import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
from correction_service.estimator import CorrectionEstimator, CorrectionSample
from correction_service.geometry import (
    expected_fcu_jump,
    homogeneous,
    project_fcu_reference_pose,
    rotation_z,
)
from correction_service.window import (
    CalibrationKeyframe,
    build_temporary_window,
    solve_weighted_se2,
)

from correction_service.config import load_config

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG = load_config(PROJECT_ROOT / "correction_service" / "config")


def _keyframe(
    tag_id: int,
    q_xy: tuple[float, float],
    p_xy: tuple[float, float],
    *,
    sigma: float = 0.012,
    created: float | None = None,
    time_source: str = "header",
) -> CalibrationKeyframe:
    """构造具备全部生产元数据的合成 keyframe。"""
    timestamp = float(tag_id * 10 if created is None else created)
    return CalibrationKeyframe(
        tag_id=tag_id,
        world_x_m=p_xy[0],
        world_y_m=p_xy[1],
        odin_x_m=q_xy[0],
        odin_y_m=q_xy[1],
        capture_start_ns=tag_id * 10_000_000_000,
        capture_end_ns=tag_id * 10_000_000_000 + 2_500_000_000,
        created_monotonic=timestamp,
        created_ros_ns=tag_id * 10_000_000_000,
        samples_accepted=30,
        samples_rejected=2,
        independent_blocks=10,
        position_std_m=0.004,
        effective_sigma_m=sigma,
        reprojection_error_px=0.4,
        odom_match_error_ms=3.0,
        max_sync_motion_error_m=0.001,
        correction_tilt_rad=math.radians(1.0),
        single_tag_yaw_rad=0.0,
        odom_time_source=time_source,
        odin_session_id="odin-test",
        config_fingerprint="geometry-v1",
        source_job_id=f"job-{tag_id}",
    )


def _world_point(
    q_xy: tuple[float, float], yaw: float, translation: tuple[float, float]
) -> tuple[float, float]:
    """用已知 orientation-preserving SE(2) 生成世界对应点。"""
    vector = rotation_z(yaw)[:2, :2] @ np.asarray(q_xy)
    return float(vector[0] + translation[0]), float(vector[1] + translation[1])


def test_two_point_equal_weight_recovers_known_transform_across_yaw_wrap() -> None:
    """两点闭式解必须跨 ±pi 恢复完整 C，并明确能力限制。"""
    yaw = math.radians(179.7)
    translation = (2.4, -1.7)
    q_points = ((-2.0, 0.3), (2.0, -0.4))
    frames = tuple(
        _keyframe(index, q, _world_point(q, yaw, translation))
        for index, q in enumerate(q_points)
    )

    result = solve_weighted_se2(frames, CONFIG.window)

    assert result.valid
    assert result.two_point_limited
    assert math.isclose(result.x_m, translation[0], abs_tol=1e-12)
    assert math.isclose(result.y_m, translation[1], abs_tol=1e-12)
    assert math.isclose(result.yaw_rad, yaw, abs_tol=1e-12)
    assert result.rms_m < 1e-12
    # 两点拟合内残差必为零，但绝对位置 sigma 仍必须给出非零 yaw 不确定度。
    assert result.model_yaw_std_rad > 0.0


def test_unequal_weight_translation_uses_weighted_centroids() -> None:
    """不等权不能退化成两个端点的普通平均平移。"""
    yaw = math.radians(24.0)
    q_points = ((-2.0, 0.0), (2.0, 0.0), (0.0, 3.0))
    p_points = [list(_world_point(q, yaw, (1.0, -0.5))) for q in q_points]
    p_points[2][0] += 0.03
    frames = (
        _keyframe(0, q_points[0], tuple(p_points[0]), sigma=0.010),
        _keyframe(1, q_points[1], tuple(p_points[1]), sigma=0.020),
        _keyframe(2, q_points[2], tuple(p_points[2]), sigma=0.040),
    )

    result = solve_weighted_se2(frames, CONFIG.window)

    assert result.valid
    weights = np.asarray([item.weight(CONFIG.window) for item in frames])
    p = np.asarray(p_points)
    q = np.asarray(q_points)
    expected_t = (weights[:, None] * p).sum(axis=0) / weights.sum() - rotation_z(
        result.yaw_rad
    )[:2, :2] @ ((weights[:, None] * q).sum(axis=0) / weights.sum())
    assert np.allclose((result.x_m, result.y_m), expected_t, atol=1e-12)


def test_collinear_but_separated_points_are_valid_for_planar_yaw() -> None:
    """二维共线点只要有展开量即可观 yaw，不能套用三维满秩条件。"""
    yaw = math.radians(-31.0)
    q_points = ((-3.0, 0.0), (0.0, 0.0), (4.0, 0.0))
    frames = tuple(
        _keyframe(index, q, _world_point(q, yaw, (0.4, 2.0)))
        for index, q in enumerate(q_points)
    )

    result = solve_weighted_se2(frames, CONFIG.window)

    assert result.valid
    assert not result.two_point_limited
    assert math.isclose(result.yaw_rad, yaw, abs_tol=1e-12)


def test_registration_rejects_near_duplicate_scale_error_and_mirror() -> None:
    """近邻、尺度变化和镜像错误对应均不能伪装成刚体修正。"""
    near = (
        _keyframe(0, (0.0, 0.0), (0.0, 0.0)),
        _keyframe(1, (0.3, 0.0), (0.3, 0.0)),
    )
    assert not solve_weighted_se2(near, CONFIG.window).valid

    scale = (
        _keyframe(0, (0.0, 0.0), (0.0, 0.0)),
        _keyframe(1, (3.0, 0.0), (4.0, 0.0)),
    )
    assert not solve_weighted_se2(scale, CONFIG.window).valid

    q_points = ((0.0, 0.0), (3.0, 0.0), (0.0, 3.0))
    mirrored = tuple(
        _keyframe(index, q, (-q[0], q[1])) for index, q in enumerate(q_points)
    )
    mirror_result = solve_weighted_se2(mirrored, CONFIG.window)
    assert not mirror_result.valid
    assert mirror_result.max_residual_m > CONFIG.window.max_registration_residual_m


def test_fifo_checks_pre_eviction_consistency_and_never_deletes_bad_evidence() -> None:
    """新点与将淘汰锚点矛盾时，不能靠 FIFO 后的两点零残差通过。"""
    existing = (
        _keyframe(0, (0.0, 0.0), (0.0, 0.0)),
        _keyframe(1, (3.0, 0.0), (3.0, 0.0)),
    )
    inconsistent = _keyframe(2, (6.0, 0.0), (6.0, 0.5))

    target, target_result, pre_result = build_temporary_window(
        existing, inconsistent, 2, CONFIG.window
    )

    assert target == existing
    assert not pre_result.valid
    assert not target_result.valid


def test_fifo_successfully_evicts_oldest_only_after_full_consistency_check() -> None:
    """一致的新点应先通过三点审查，再按顺序只淘汰最旧 keyframe。"""
    yaw = math.radians(12.0)
    translation = (0.7, -1.1)
    existing = (
        _keyframe(0, (-3.0, 0.0), _world_point((-3.0, 0.0), yaw, translation)),
        _keyframe(1, (0.0, 0.0), _world_point((0.0, 0.0), yaw, translation)),
    )
    newest = _keyframe(
        2,
        (3.0, 1.0),
        _world_point((3.0, 1.0), yaw, translation),
    )

    target, target_result, pre_result = build_temporary_window(
        existing, newest, 2, CONFIG.window
    )

    assert pre_result.valid and target_result.valid
    assert [item.tag_id for item in target] == [1, 2]
    assert math.isclose(target_result.yaw_rad, yaw, abs_tol=1e-12)


def test_window_rejects_excessive_time_span_and_adjacent_gap() -> None:
    """同 session 也不得跨越超限的窗口跨度或停留段间隔。"""
    yaw = 0.0
    translation = (0.0, 0.0)
    too_far = CONFIG.window.maximum_window_span_seconds + 1.0
    frames = (
        _keyframe(
            0, (0.0, 0.0), _world_point((0.0, 0.0), yaw, translation), created=0.0
        ),
        _keyframe(
            1, (2.0, 0.0), _world_point((2.0, 0.0), yaw, translation), created=too_far
        ),
    )

    span_result = solve_weighted_se2(frames, CONFIG.window)
    gap_time = CONFIG.window.maximum_keyframe_gap_seconds + 1.0
    gap_frames = (
        frames[0],
        replace(frames[1], created_monotonic=gap_time),
    )
    gap_result = solve_weighted_se2(gap_frames, CONFIG.window)

    assert not span_result.valid
    assert "时间跨度" in span_result.reason
    assert not gap_result.valid
    assert "间隔" in gap_result.reason


def test_duplicate_tag_and_nonfinite_input_are_rejected() -> None:
    """窗口 Tag ID 唯一且所有坐标/绝对 sigma 必须有限。"""
    duplicate = (
        _keyframe(0, (0.0, 0.0), (0.0, 0.0)),
        _keyframe(0, (3.0, 0.0), (3.0, 0.0)),
    )
    assert not solve_weighted_se2(duplicate, CONFIG.window).valid
    invalid = (
        _keyframe(0, (0.0, 0.0), (0.0, 0.0)),
        replace(_keyframe(1, (3.0, 0.0), (3.0, 0.0)), odin_x_m=math.nan),
    )
    assert not solve_weighted_se2(invalid, CONFIG.window).valid


def _sample(
    index: int, *, source: str = "header", speed: float = 0.1
) -> CorrectionSample:
    """构造高度相关但跨有效时间块的停留段样本。"""
    jitter = (index % 5 - 2) * 0.0002
    return CorrectionSample(
        stamp_ns=index * 100_000_000,
        x_m=0.2 + jitter,
        y_m=-0.1 - jitter,
        # 后续 keyframe 不应把这个故意很差的单 Tag yaw 当成最终精标门。
        yaw_rad=math.radians((index % 7 - 3) * 2.0),
        tilt_rad=math.radians(1.0),
        reprojection_error_px=0.4,
        odom_match_error_ms=3.0,
        odom_time_source=source,
        processing_time_ms=8.0,
        odin_tag_x_m=4.0 + jitter,
        odin_tag_y_m=2.0 - jitter,
        odin_speed_mps=speed,
    )


def test_next_keyframe_uses_time_blocks_and_nonzero_uncertainty_floor() -> None:
    """连续帧不能把 sigma 压到零，后续也不能平均单 Tag yaw 得最终 yaw。"""
    estimator = CorrectionEstimator(
        CONFIG.quality, CONFIG.keyframe, first_calibration=False
    )
    snapshot = None
    for index in range(30):
        snapshot = estimator.add(_sample(index))

    assert snapshot is not None and snapshot.converged
    assert snapshot.independent_blocks >= CONFIG.keyframe.minimum_independent_blocks
    assert (
        snapshot.effective_position_sigma_m >= CONFIG.keyframe.minimum_effective_sigma_m
    )
    assert snapshot.yaw_std_rad > CONFIG.quality.max_yaw_std_rad


def test_keyframe_rejects_mixed_time_sources_and_large_sync_motion_error() -> None:
    """同段混用时间轴或高速下匹配误差过大时不得保存生产 keyframe。"""
    mixed = CorrectionEstimator(
        CONFIG.quality, CONFIG.keyframe, first_calibration=False
    )
    for index in range(30):
        source = "arrival_history" if index == 15 else "header"
        snapshot = mixed.add(_sample(index, source=source))
    assert not snapshot.converged
    assert snapshot.odom_time_source == "mixed"

    moving = CorrectionEstimator(
        CONFIG.quality, CONFIG.keyframe, first_calibration=False
    )
    for index in range(30):
        snapshot = moving.add(_sample(index, speed=20.0))
    assert not snapshot.converged
    assert snapshot.max_sync_motion_error_m > CONFIG.keyframe.max_sync_motion_error_m


def _rpy_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """生成测试用 ZYX 姿态矩阵。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
    )


def test_valid_fcu_center_removes_fixed_xy_origin_for_arbitrary_pose_and_yaw() -> None:
    """有效分支必须是 Qp+t-QR*T；valid identity 也不能回到旧 +T_xy。"""
    lever = np.array((0.06, -0.03, 0.05))
    raw_rotation = _rpy_rotation(0.21, -0.17, 0.73)
    raw_position = np.array((1.2, -0.8, 0.6))
    raw = homogeneous(raw_rotation, raw_position)
    for yaw in (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi):
        correction = (0.4, -0.2, yaw)
        projected = project_fcu_reference_pose(raw, correction, lever)
        world_rotation = rotation_z(yaw)
        expected_xy = (
            world_rotation @ raw_position
            + np.array((correction[0], correction[1], 0.0))
            - world_rotation @ raw_rotation @ lever
        )[:2]
        old_z = raw_position[2] + lever[2] - (world_rotation @ raw_rotation @ lever)[2]
        assert np.allclose(projected.position[:2], expected_xy, atol=1e-12)
        assert math.isclose(projected.position[2], old_z, abs_tol=1e-12)

    invalid = project_fcu_reference_pose(raw, None, lever)
    valid_identity = project_fcu_reference_pose(raw, (0.0, 0.0, 0.0), lever)
    assert np.allclose(invalid.position[:2] - valid_identity.position[:2], lever[:2])
    assert math.isclose(invalid.position[2], valid_identity.position[2], abs_tol=1e-12)


def test_expected_jump_includes_valid_invalid_origin_branch() -> None:
    """首次 identity-valid 的实际跳变必须包含去除旧 +T_xy 的中心语义变化。"""
    lever = np.array((0.06, -0.03, 0.05))
    raw = homogeneous(np.eye(3), np.array((0.0, 0.0, 0.7)))
    position_jump, yaw_jump = expected_fcu_jump(raw, None, (0.0, 0.0, 0.0), lever)
    assert math.isclose(position_jump, math.hypot(0.06, -0.03), abs_tol=1e-12)
    assert yaw_jump == 0.0


def _load_extnav_module() -> Any:
    """加载受控 extnav 源以交叉验证同一中心公式。"""
    path = PROJECT_ROOT / "correction_service/extnav_patch/extnav_to_vision_pose.py"
    spec = importlib.util.spec_from_file_location("task29_extnav_math", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_extnav_and_candidate_jump_use_identical_center_formula() -> None:
    """extnav 两路最终 pose 与后端跳变门必须共享同一已推导数值关系。"""
    extnav = _load_extnav_module()
    lever = np.array((0.06, -0.03, 0.05))
    rotation = _rpy_rotation(-0.12, 0.09, 1.1)
    raw = np.array((2.0, -1.0, 0.8))
    corrected = np.array((-0.4, 3.2, 0.8))

    actual, mode = extnav.compute_fcu_reference_position(
        raw, corrected, rotation, lever, True
    )

    expected = corrected - rotation @ lever
    expected[2] = raw[2] + lever[2] - (rotation @ lever)[2]
    assert mode == "tag_world_xy_local_z"
    assert np.allclose(actual, expected, atol=1e-12)
