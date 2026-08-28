"""滚动稳健汇总逐帧 SE(2) 候选，并在满足全部质量门时一次性收敛。"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .config import QualitySettings
from .geometry import wrap_angle


@dataclass(frozen=True)
class CorrectionSample:
    """一帧合格 Tag 与同时间 Odin 样本得到的修正候选。"""

    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    tilt_rad: float
    reprojection_error_px: float
    odom_match_error_ms: float
    odom_time_source: str
    processing_time_ms: float


@dataclass(frozen=True)
class QualitySnapshot:
    """当前稳健窗口的候选值、离散度、性能和收敛/发散结论。"""

    window_samples: int = 0
    inlier_samples: int = 0
    span_seconds: float = 0.0
    x_m: float = math.nan
    y_m: float = math.nan
    yaw_rad: float = math.nan
    tilt_rad: float = math.nan
    position_std_m: float = math.inf
    yaw_std_rad: float = math.inf
    position_range_m: float = math.inf
    yaw_range_rad: float = math.inf
    reprojection_error_px: float = math.inf
    odom_match_error_ms: float = math.inf
    odom_time_source: str = ""
    processing_rate_hz: float = 0.0
    processing_time_ms: float = 0.0
    converged: bool = False
    diverged: bool = False
    reason: str = "等待样本"


def _circular_mean(values: np.ndarray) -> float:
    """计算 [-pi,pi] 角数组的圆均值。"""
    return wrap_angle(
        math.atan2(float(np.sin(values).mean()), float(np.cos(values).mean()))
    )


def _circular_deltas(values: np.ndarray, center: float) -> np.ndarray:
    """返回每个角相对中心的最短有符号差。"""
    return np.arctan2(np.sin(values - center), np.cos(values - center))


def _robust_mask(values: np.ndarray, scale: float) -> np.ndarray:
    """按逐维 MAD 去除明显离群帧，并给近零噪声保留数值下限。"""
    center = np.median(values, axis=0)
    absolute = np.abs(values - center)
    mad = np.median(absolute, axis=0)
    floors = np.array((0.0005, 0.0005, math.radians(0.01)))
    limits = scale * np.maximum(1.4826 * mad, floors)
    return np.all(absolute <= limits, axis=1)


class CorrectionEstimator:
    """只维护候选窗口；它从不直接修改 extnav active correction。"""

    def __init__(self, settings: QualitySettings) -> None:
        self._settings = settings
        self._samples: deque[CorrectionSample] = deque(
            maxlen=settings.rolling_window_samples
        )
        self._processing: deque[tuple[int, float]] = deque(maxlen=256)

    def reset(self) -> None:
        """新任务或 Odin session 改变时彻底丢弃旧候选。"""
        self._samples.clear()
        self._processing.clear()

    def record_processing(self, stamp_ns: int, duration_ms: float) -> None:
        """记录每次实际 AprilTag 检测耗时，包含无 Tag 帧。"""
        if math.isfinite(duration_ms) and duration_ms >= 0.0:
            self._processing.append((int(stamp_ns), float(duration_ms)))

    def add(self, sample: CorrectionSample) -> QualitySnapshot:
        """加入单帧合格候选并返回新的完整质量快照。"""
        values = (
            sample.x_m,
            sample.y_m,
            sample.yaw_rad,
            sample.tilt_rad,
            sample.reprojection_error_px,
            sample.odom_match_error_ms,
            sample.processing_time_ms,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("修正样本含非有限值")
        self._samples.append(sample)
        return self.snapshot()

    def snapshot(self) -> QualitySnapshot:
        """使用当前窗口计算稳健中心、离散度和全部门控结果。"""
        if not self._samples:
            rate, processing = self._processing_metrics()
            return QualitySnapshot(
                processing_rate_hz=rate,
                processing_time_ms=processing,
            )

        samples = tuple(self._samples)
        x = np.array([item.x_m for item in samples], dtype=np.float64)
        y = np.array([item.y_m for item in samples], dtype=np.float64)
        yaw = np.array([item.yaw_rad for item in samples], dtype=np.float64)
        yaw_seed = _circular_mean(yaw)
        yaw_delta = _circular_deltas(yaw, yaw_seed)
        features = np.column_stack((x, y, yaw_delta))
        mask = (
            _robust_mask(features, self._settings.mad_outlier_scale)
            if len(samples) >= 8
            else np.ones(len(samples), dtype=bool)
        )
        if int(mask.sum()) < min(3, len(samples)):
            mask[:] = True

        selected = tuple(item for item, keep in zip(samples, mask) if keep)
        sx = x[mask]
        sy = y[mask]
        syaw = yaw[mask]
        candidate_x = float(np.median(sx))
        candidate_y = float(np.median(sy))
        candidate_yaw = _circular_mean(syaw)
        yaw_errors = _circular_deltas(syaw, candidate_yaw)
        position_errors = np.hypot(sx - candidate_x, sy - candidate_y)
        position_std = float(np.sqrt(np.mean(position_errors * position_errors)))
        yaw_std = float(np.sqrt(np.mean(yaw_errors * yaw_errors)))
        position_range = float(max(np.ptp(sx), np.ptp(sy)))
        yaw_range = float(np.ptp(yaw_errors))
        span = max(0.0, (selected[-1].stamp_ns - selected[0].stamp_ns) / 1e9)
        reprojection = float(np.mean([item.reprojection_error_px for item in selected]))
        match_error = float(np.mean([item.odom_match_error_ms for item in selected]))
        tilt = float(np.median([item.tilt_rad for item in selected]))
        time_sources = {item.odom_time_source for item in selected}
        time_source = next(iter(time_sources)) if len(time_sources) == 1 else "mixed"
        rate, processing = self._processing_metrics()

        raw_position_range = float(max(np.ptp(x), np.ptp(y)))
        raw_yaw_range = float(np.ptp(_circular_deltas(yaw, yaw_seed)))
        diverged = len(samples) >= max(8, self._settings.minimum_samples // 2) and (
            raw_position_range > self._settings.divergence_position_range_m
            or raw_yaw_range > self._settings.divergence_yaw_range_rad
        )
        gates = (
            (len(selected) >= self._settings.minimum_samples, "样本数不足"),
            (span >= self._settings.minimum_span_seconds, "稳定采样时长不足"),
            (position_std <= self._settings.max_position_std_m, "水平离散度过大"),
            (yaw_std <= self._settings.max_yaw_std_rad, "偏航离散度过大"),
            (position_range <= self._settings.max_position_range_m, "水平范围过大"),
            (yaw_range <= self._settings.max_yaw_range_rad, "偏航范围过大"),
        )
        converged = not diverged and all(passed for passed, _ in gates)
        if diverged:
            reason = "候选修正明显发散"
        elif converged:
            reason = "候选修正已收敛"
        else:
            reason = next(message for passed, message in gates if not passed)

        return QualitySnapshot(
            window_samples=len(samples),
            inlier_samples=len(selected),
            span_seconds=span,
            x_m=candidate_x,
            y_m=candidate_y,
            yaw_rad=candidate_yaw,
            tilt_rad=tilt,
            position_std_m=position_std,
            yaw_std_rad=yaw_std,
            position_range_m=position_range,
            yaw_range_rad=yaw_range,
            reprojection_error_px=reprojection,
            odom_match_error_ms=match_error,
            odom_time_source=time_source,
            processing_rate_hz=rate,
            processing_time_ms=processing,
            converged=converged,
            diverged=diverged,
            reason=reason,
        )

    def _processing_metrics(self) -> tuple[float, float]:
        """返回最近检测的实际速率和平均耗时。"""
        if not self._processing:
            return 0.0, 0.0
        durations = [item[1] for item in self._processing]
        if len(self._processing) < 2:
            rate = 0.0
        else:
            span = (self._processing[-1][0] - self._processing[0][0]) / 1e9
            rate = (len(self._processing) - 1) / span if span > 0.0 else 0.0
        return float(rate), float(np.mean(durations))
