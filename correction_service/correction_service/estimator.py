"""滚动稳健汇总逐帧 SE(2) 候选，并在满足全部质量门时一次性收敛。"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .config import KeyframeSettings, QualitySettings
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
    odin_tag_x_m: float = math.nan
    odin_tag_y_m: float = math.nan
    odin_speed_mps: float = 0.0


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
    odin_tag_x_m: float = math.nan
    odin_tag_y_m: float = math.nan
    odin_tag_position_std_m: float = math.inf
    odin_tag_position_range_m: float = math.inf
    effective_position_sigma_m: float = math.inf
    independent_blocks: int = 0
    max_sync_motion_error_m: float = math.inf
    capture_start_ns: int = 0
    capture_end_ns: int = 0
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
    floors = np.full(values.shape[1], 0.0005, dtype=np.float64)
    if values.shape[1] >= 3:
        floors[2] = math.radians(0.01)
    limits = scale * np.maximum(1.4826 * mad, floors)
    return np.all(absolute <= limits, axis=1)


class CorrectionEstimator:
    """只维护候选窗口；它从不直接修改 extnav active correction。"""

    def __init__(
        self,
        settings: QualitySettings,
        keyframe_settings: KeyframeSettings | None = None,
        *,
        first_calibration: bool = True,
    ) -> None:
        """建立一个停留段估计器；后续标定不以单 Tag yaw 作为精标门。"""
        self._settings = settings
        self._keyframe_settings = keyframe_settings
        self._first_calibration = bool(first_calibration)
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
            sample.odin_speed_mps,
        )
        if not all(
            math.isfinite(value) and value >= 0.0 for value in values[-1:]
        ) or not all(math.isfinite(value) for value in values[:-1]):
            raise ValueError("修正样本含非有限值")
        if self._keyframe_settings is not None and not all(
            math.isfinite(value) for value in (sample.odin_tag_x_m, sample.odin_tag_y_m)
        ):
            raise ValueError("生产 keyframe 样本缺少有限 Odin Tag 中心")
        self._samples.append(sample)
        return self.snapshot()

    def samples(self) -> tuple[CorrectionSample, ...]:
        """返回审计日志使用的不可变样本副本。"""
        return tuple(self._samples)

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
        qx = np.array(
            [
                item.odin_tag_x_m if math.isfinite(item.odin_tag_x_m) else item.x_m
                for item in samples
            ],
            dtype=np.float64,
        )
        qy = np.array(
            [
                item.odin_tag_y_m if math.isfinite(item.odin_tag_y_m) else item.y_m
                for item in samples
            ],
            dtype=np.float64,
        )
        yaw_seed = _circular_mean(yaw)
        yaw_delta = _circular_deltas(yaw, yaw_seed)
        # 首次修正必须保持 Task27 的 x/y/yaw 离群筛选；新增 Q 只作为首个
        # keyframe 的质量证据，不能反向改变粗修正的入选帧或候选数值。
        features = (
            np.column_stack((x, y, yaw_delta))
            if self._first_calibration
            else np.column_stack((qx, qy))
        )
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
        sqx = qx[mask]
        sqy = qy[mask]
        candidate_x = float(np.median(sx))
        candidate_y = float(np.median(sy))
        candidate_yaw = _circular_mean(syaw)
        yaw_errors = _circular_deltas(syaw, candidate_yaw)
        position_errors = np.hypot(sx - candidate_x, sy - candidate_y)
        position_std = float(np.sqrt(np.mean(position_errors * position_errors)))
        yaw_std = float(np.sqrt(np.mean(yaw_errors * yaw_errors)))
        position_range = float(max(np.ptp(sx), np.ptp(sy)))
        yaw_range = float(np.ptp(yaw_errors))
        candidate_qx = float(np.median(sqx))
        candidate_qy = float(np.median(sqy))
        q_errors = np.hypot(sqx - candidate_qx, sqy - candidate_qy)
        q_position_std = float(np.sqrt(np.mean(q_errors * q_errors)))
        q_position_range = float(max(np.ptp(sqx), np.ptp(sqy)))
        span = max(0.0, (selected[-1].stamp_ns - selected[0].stamp_ns) / 1e9)
        reprojection = float(np.mean([item.reprojection_error_px for item in selected]))
        match_error = float(np.mean([item.odom_match_error_ms for item in selected]))
        tilt = float(np.median([item.tilt_rad for item in selected]))
        time_sources = {item.odom_time_source for item in selected}
        time_source = next(iter(time_sources)) if len(time_sources) == 1 else "mixed"
        rate, processing = self._processing_metrics()
        independent_blocks, effective_sigma = self._effective_sigma(selected)
        max_sync_motion_error = max(
            item.odin_speed_mps * item.odom_match_error_ms / 1000.0 for item in selected
        )

        raw_position_range = float(max(np.ptp(x), np.ptp(y)))
        raw_yaw_range = float(np.ptp(_circular_deltas(yaw, yaw_seed)))
        raw_q_range = float(max(np.ptp(qx), np.ptp(qy)))
        if self._first_calibration:
            diverged = len(samples) >= max(8, self._settings.minimum_samples // 2) and (
                raw_position_range > self._settings.divergence_position_range_m
                or raw_yaw_range > self._settings.divergence_yaw_range_rad
                or raw_q_range > self._settings.divergence_position_range_m
            )
        else:
            diverged = len(samples) >= max(8, self._settings.minimum_samples // 2) and (
                raw_q_range > self._settings.divergence_position_range_m
            )
        gates = [
            (len(selected) >= self._settings.minimum_samples, "样本数不足"),
            (span >= self._settings.minimum_span_seconds, "稳定采样时长不足"),
            (
                q_position_std <= self._settings.max_position_std_m,
                "Odin Tag 中心离散度过大",
            ),
            (
                q_position_range <= self._settings.max_position_range_m,
                "Odin Tag 中心范围过大",
            ),
            (time_source != "mixed", "同一 keyframe 混用了时间源"),
        ]
        if self._keyframe_settings is not None:
            gates.extend(
                (
                    (
                        independent_blocks
                        >= self._keyframe_settings.minimum_independent_blocks,
                        "有效独立时间块不足",
                    ),
                    (
                        effective_sigma
                        <= self._keyframe_settings.maximum_effective_sigma_m,
                        "keyframe 有效位置不确定度过大",
                    ),
                    (
                        max_sync_motion_error
                        <= self._keyframe_settings.max_sync_motion_error_m,
                        "同步运动误差上界过大",
                    ),
                )
            )
        if self._first_calibration:
            gates.extend(
                (
                    (
                        position_std <= self._settings.max_position_std_m,
                        "粗修正水平离散度过大",
                    ),
                    (yaw_std <= self._settings.max_yaw_std_rad, "粗修正偏航离散度过大"),
                    (
                        position_range <= self._settings.max_position_range_m,
                        "粗修正水平范围过大",
                    ),
                    (
                        yaw_range <= self._settings.max_yaw_range_rad,
                        "粗修正偏航范围过大",
                    ),
                )
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
            odin_tag_x_m=candidate_qx,
            odin_tag_y_m=candidate_qy,
            odin_tag_position_std_m=q_position_std,
            odin_tag_position_range_m=q_position_range,
            effective_position_sigma_m=effective_sigma,
            independent_blocks=independent_blocks,
            max_sync_motion_error_m=max_sync_motion_error,
            capture_start_ns=selected[0].stamp_ns,
            capture_end_ns=selected[-1].stamp_ns,
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

    def _effective_sigma(
        self, selected: tuple[CorrectionSample, ...]
    ) -> tuple[int, float]:
        """按时间块估计随机项，并与世界测量/外参/同步误差下限合成。"""
        settings = self._keyframe_settings
        if settings is None:
            return len(selected), 0.0
        block_ns = max(1, int(settings.correlation_block_seconds * 1e9))
        origin = selected[0].stamp_ns
        buckets: dict[int, list[tuple[float, float]]] = {}
        for sample in selected:
            index = max(0, (sample.stamp_ns - origin) // block_ns)
            buckets.setdefault(index, []).append(
                (sample.odin_tag_x_m, sample.odin_tag_y_m)
            )
        centers = np.asarray(
            [
                np.median(np.asarray(values, dtype=np.float64), axis=0)
                for values in buckets.values()
            ],
            dtype=np.float64,
        )
        center = np.median(centers, axis=0)
        deviations = np.linalg.norm(centers - center, axis=1)
        block_std = float(np.sqrt(np.mean(deviations * deviations)))
        independent = len(centers)
        random_sigma = max(
            settings.odin_position_noise_floor_m,
            block_std / math.sqrt(max(1, independent)),
        )
        sync_sigma = max(
            sample.odin_speed_mps * sample.odom_match_error_ms / 1000.0
            for sample in selected
        )
        combined = math.sqrt(
            random_sigma * random_sigma
            + settings.tag_world_sigma_m * settings.tag_world_sigma_m
            + settings.extrinsic_sigma_m * settings.extrinsic_sigma_m
            + sync_sigma * sync_sigma
        )
        return independent, max(settings.minimum_effective_sigma_m, combined)
