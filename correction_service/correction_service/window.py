"""多 Tag keyframe FIFO、绝对逆方差加权 SE(2) 配准与质量门控。"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from itertools import pairwise

import numpy as np

from .config import WindowSettings
from .geometry import wrap_angle


@dataclass(frozen=True)
class CalibrationKeyframe:
    """一次 Tag 停留段的稳健中心和可审计公制不确定度。"""

    tag_id: int
    world_x_m: float
    world_y_m: float
    odin_x_m: float
    odin_y_m: float
    capture_start_ns: int
    capture_end_ns: int
    created_monotonic: float
    created_ros_ns: int
    samples_accepted: int
    samples_rejected: int
    independent_blocks: int
    position_std_m: float
    effective_sigma_m: float
    reprojection_error_px: float
    odom_match_error_ms: float
    max_sync_motion_error_m: float
    correction_tilt_rad: float
    single_tag_yaw_rad: float
    odom_time_source: str
    odin_session_id: str
    config_fingerprint: str
    source_job_id: str
    prior_model_residual_m: float = math.nan

    @property
    def span_s(self) -> float:
        """返回停留段采集时间跨度。"""
        return max(0.0, (self.capture_end_ns - self.capture_start_ns) / 1e9)

    def weight(self, settings: WindowSettings) -> float:
        """从绝对公制各向同性 sigma 产生有界逆方差权重。"""
        sigma = max(1e-9, float(self.effective_sigma_m))
        # 只为数值稳定限制权重；yaw sigma 仍使用有物理单位的绝对权重。
        return float(np.clip(1.0 / (sigma * sigma), 16.0, 1.0e6))

    def audit_dict(self, settings: WindowSettings) -> dict[str, object]:
        """生成足以离线复算配准的严格 JSON 友好快照。"""
        return {
            "tag_id": self.tag_id,
            "P_world_m": [self.world_x_m, self.world_y_m],
            "Q_odin_m": [self.odin_x_m, self.odin_y_m],
            "capture_start_ns": self.capture_start_ns,
            "capture_end_ns": self.capture_end_ns,
            "span_s": self.span_s,
            "samples_accepted": self.samples_accepted,
            "samples_rejected": self.samples_rejected,
            "independent_blocks": self.independent_blocks,
            "position_std_m": self.position_std_m,
            "effective_sigma_m": self.effective_sigma_m,
            "weight": self.weight(settings),
            "reprojection_error_px": self.reprojection_error_px,
            "odom_match_error_ms": self.odom_match_error_ms,
            "max_sync_motion_error_m": self.max_sync_motion_error_m,
            "correction_tilt_deg": math.degrees(self.correction_tilt_rad),
            "single_tag_yaw_deg": math.degrees(self.single_tag_yaw_rad),
            "prior_model_residual_m": self.prior_model_residual_m,
            "odom_time_source": self.odom_time_source,
            "odin_session_id": self.odin_session_id,
            "config_fingerprint": self.config_fingerprint,
            "source_job_id": self.source_job_id,
        }


@dataclass(frozen=True)
class RegistrationResult:
    """完整窗口的 orientation-preserving SE(2) 解及全部诊断。"""

    valid: bool
    reason: str
    x_m: float = math.nan
    y_m: float = math.nan
    yaw_rad: float = math.nan
    rms_m: float = math.inf
    max_residual_m: float = math.inf
    model_yaw_std_rad: float = math.inf
    residuals_m: tuple[float, ...] = ()
    baseline_tag_a: int = -1
    baseline_tag_b: int = -1
    odin_baseline_m: float = 0.0
    world_baseline_m: float = 0.0
    odin_baseline_direction_rad: float = 0.0
    world_baseline_direction_rad: float = 0.0
    two_point_limited: bool = False


@dataclass(frozen=True)
class WindowCandidate:
    """冻结的粗/精候选及其窗口、revision、跳变与应用事实。"""

    keyframes: tuple[CalibrationKeyframe, ...]
    x_m: float
    y_m: float
    yaw_rad: float
    stage: str
    odin_session_id: str
    config_fingerprint: str
    source_job_id: str
    application_job_id: str
    base_window_revision: int
    expected_extnav_revision: int
    created_monotonic: float
    created_ros_ns: int
    valid: bool
    quality_reason: str
    position_std_m: float
    yaw_std_rad: float
    correction_tilt_rad: float
    reprojection_error_px: float
    odom_match_error_ms: float
    odom_time_source: str
    registration: RegistrationResult
    delta_x_m: float = math.nan
    delta_y_m: float = math.nan
    delta_yaw_rad: float = math.nan
    expected_position_jump_m: float = math.nan
    expected_yaw_jump_rad: float = math.nan
    can_apply: bool = False
    apply_reason: str = "尚未复算新鲜 raw 样本处跳变"
    saved: bool = False
    saved_window_revision: int = 0
    application_state: str = "none"
    application_confirmed_by_status: bool = False

    def aged(self, now_monotonic: float, settings: WindowSettings) -> bool:
        """候选冻结时间超过配置上限后禁止应用。"""
        return (
            now_monotonic - self.created_monotonic > settings.candidate_max_age_seconds
        )

    def with_application(self, **changes: object) -> WindowCandidate:
        """以不可变替换记录一次应用或保存状态变化。"""
        return replace(self, **changes)


def _invalid(reason: str, **metrics: object) -> RegistrationResult:
    """构造带可用诊断但明确不可提交的配准结果。"""
    return RegistrationResult(valid=False, reason=reason, **metrics)


def solve_weighted_se2(
    keyframes: tuple[CalibrationKeyframe, ...], settings: WindowSettings
) -> RegistrationResult:
    """从原始 P/Q 每次重算加权 SE(2)，不拟合镜像、尺度或历史增量。"""
    if len(keyframes) < 2:
        return _invalid("精标至少需要两个空间 keyframe")
    scalar_values = [
        value
        for item in keyframes
        for value in (
            item.world_x_m,
            item.world_y_m,
            item.odin_x_m,
            item.odin_y_m,
            item.effective_sigma_m,
        )
    ]
    if not all(math.isfinite(value) for value in scalar_values):
        return _invalid("keyframe 含非有限坐标或不确定度")
    if len({item.tag_id for item in keyframes}) != len(keyframes):
        return _invalid("窗口内 Tag ID 重复，不能形成新的独立空间基线")
    if len({item.odin_session_id for item in keyframes}) != 1:
        return _invalid("窗口混合了不同 Odin session")
    if len({item.config_fingerprint for item in keyframes}) != 1:
        return _invalid("窗口混合了不同几何/采集配置")
    if len({item.odom_time_source for item in keyframes}) != 1:
        return _invalid("窗口混合了 header 与 arrival_history 时间源")

    ordered = tuple(sorted(keyframes, key=lambda item: item.created_monotonic))
    window_span = ordered[-1].created_monotonic - ordered[0].created_monotonic
    if window_span > settings.maximum_window_span_seconds:
        return _invalid("滑窗时间跨度已超限，必须清空后重采")
    if any(
        right.created_monotonic - left.created_monotonic
        > settings.maximum_keyframe_gap_seconds
        for left, right in pairwise(ordered)
    ):
        return _invalid("相邻 keyframe 间隔已超限，必须清空后重采")

    p = np.asarray(
        [(item.world_x_m, item.world_y_m) for item in keyframes], dtype=np.float64
    )
    q = np.asarray(
        [(item.odin_x_m, item.odin_y_m) for item in keyframes], dtype=np.float64
    )
    longest: tuple[float, int, int, float] | None = None
    for first in range(len(keyframes) - 1):
        for second in range(first + 1, len(keyframes)):
            odin_length = float(np.linalg.norm(q[second] - q[first]))
            world_length = float(np.linalg.norm(p[second] - p[first]))
            metrics = {
                "baseline_tag_a": keyframes[first].tag_id,
                "baseline_tag_b": keyframes[second].tag_id,
                "odin_baseline_m": odin_length,
                "world_baseline_m": world_length,
            }
            if min(odin_length, world_length) < settings.minimum_baseline_m:
                return _invalid("不同 Tag 的 O/W 空间基线过短", **metrics)
            ratio_error = abs(world_length - odin_length) / max(
                world_length, odin_length
            )
            if ratio_error > settings.maximum_baseline_ratio_error:
                return _invalid(
                    "O/W 基线长度不一致，刚体模型或 Tag 对应有误", **metrics
                )
            if longest is None or odin_length > longest[0]:
                longest = (odin_length, first, second, world_length)
    assert longest is not None

    weights = np.asarray([item.weight(settings) for item in keyframes])
    weight_sum = float(weights.sum())
    p_center = (weights[:, None] * p).sum(axis=0) / weight_sum
    q_center = (weights[:, None] * q).sum(axis=0) / weight_sum
    p_centered = p - p_center
    q_centered = q - q_center
    a = float(np.sum(weights * np.sum(q_centered * p_centered, axis=1)))
    b = float(
        np.sum(
            weights
            * (
                q_centered[:, 0] * p_centered[:, 1]
                - q_centered[:, 1] * p_centered[:, 0]
            )
        )
    )
    spread = float(np.sum(weights * np.sum(q_centered * q_centered, axis=1)))
    if spread <= 1e-12 or math.hypot(a, b) <= 1e-12:
        return _invalid("加权空间展开量退化，无法观测二维 yaw")
    yaw = wrap_angle(math.atan2(b, a))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array(((cosine, -sine), (sine, cosine)), dtype=np.float64)
    translation = p_center - rotation @ q_center
    predicted = (rotation @ q.T).T + translation
    residuals = np.linalg.norm(p - predicted, axis=1)
    rms = math.sqrt(float(np.sum(weights * residuals * residuals) / weight_sum))
    maximum = float(np.max(residuals))
    yaw_sigma = 1.0 / math.sqrt(spread)
    _, first, second, world_length = longest
    odin_vector = q[second] - q[first]
    world_vector = p[second] - p[first]
    common = {
        "x_m": float(translation[0]),
        "y_m": float(translation[1]),
        "yaw_rad": yaw,
        "rms_m": rms,
        "max_residual_m": maximum,
        "model_yaw_std_rad": yaw_sigma,
        "residuals_m": tuple(float(value) for value in residuals),
        "baseline_tag_a": keyframes[first].tag_id,
        "baseline_tag_b": keyframes[second].tag_id,
        "odin_baseline_m": float(longest[0]),
        "world_baseline_m": float(world_length),
        "odin_baseline_direction_rad": math.atan2(odin_vector[1], odin_vector[0]),
        "world_baseline_direction_rad": math.atan2(world_vector[1], world_vector[0]),
        "two_point_limited": len(keyframes) == 2,
    }
    if rms > settings.max_registration_rms_m:
        return _invalid("加权配准 RMS 超限，窗口不符合单一 SE(2) 模型", **common)
    if maximum > settings.max_registration_residual_m:
        return _invalid("最大公制残差超限，拒绝自动删除不利 keyframe", **common)
    if yaw_sigma > settings.max_model_yaw_std_rad:
        return _invalid("模型 yaw 标准差估算超限", **common)
    reason = (
        "两点解合格，但离群识别能力有限"
        if len(keyframes) == 2
        else "多点加权 SE(2) 配准通过全部质量门"
    )
    return RegistrationResult(valid=True, reason=reason, **common)


def build_temporary_window(
    existing: tuple[CalibrationKeyframe, ...],
    new_keyframe: CalibrationKeyframe,
    window_size: int,
    settings: WindowSettings,
) -> tuple[tuple[CalibrationKeyframe, ...], RegistrationResult, RegistrationResult]:
    """先审查淘汰前全体一致性，再模拟 FIFO 并返回正式候选窗口。"""
    if any(item.tag_id == new_keyframe.tag_id for item in existing):
        invalid = _invalid("当前窗口已含相同 Tag ID，成功次数不增加")
        return existing, invalid, invalid
    pre_eviction = existing + (new_keyframe,)
    pre_result = solve_weighted_se2(pre_eviction, settings)
    if not pre_result.valid:
        return existing, pre_result, pre_result
    target = pre_eviction[-int(window_size) :]
    target_result = solve_weighted_se2(target, settings)
    return target, target_result, pre_result


def residual_under_correction(
    keyframe: CalibrationKeyframe, correction: tuple[float, float, float]
) -> float:
    """计算新点在旧模型下的诊断残差，不用于累加候选。"""
    x_m, y_m, yaw_rad = correction
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    predicted_x = cosine * keyframe.odin_x_m - sine * keyframe.odin_y_m + x_m
    predicted_y = sine * keyframe.odin_x_m + cosine * keyframe.odin_y_m + y_m
    return math.hypot(
        keyframe.world_x_m - predicted_x, keyframe.world_y_m - predicted_y
    )
