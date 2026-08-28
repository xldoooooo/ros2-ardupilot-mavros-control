"""保留 Odin 历史并按图像采集时间选择同一时刻的里程计，而非处理时最新值。"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from typing import Any

from .config import SynchronizationSettings

APPLY_SAFE_TIME_SOURCES = frozenset(("header", "arrival_history"))


def stamp_to_nanoseconds(stamp: Any) -> int:
    """把 ROS builtin_interfaces/Time 转为有符号纳秒整数。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class _OdomRecord:
    """同一条 Odin 消息的传感器头时间和本机接收时间。"""

    message: Any
    header_ns: int
    arrival_ns: int


@dataclass(frozen=True)
class OdomMatch:
    """图像对应的 Odin 消息、绝对时间差和实际采用的时间轴。"""

    message: Any
    delta_ms: float
    time_source: str


class OdometrySynchronizer:
    """线程安全的短历史缓冲；同 epoch 时不允许悄悄回退到接收时间。"""

    def __init__(self, settings: SynchronizationSettings) -> None:
        self._settings = settings
        self._records: deque[_OdomRecord] = deque()
        self._lock = threading.Lock()

    def add(self, message: Any, arrival_ns: int) -> None:
        """加入 Odin 样本并按接收时钟裁剪旧历史。"""
        record = _OdomRecord(
            message=message,
            header_ns=stamp_to_nanoseconds(message.header.stamp),
            arrival_ns=int(arrival_ns),
        )
        cutoff = record.arrival_ns - int(
            self._settings.odometry_history_seconds * 1_000_000_000
        )
        with self._lock:
            self._records.append(record)
            while self._records and self._records[0].arrival_ns < cutoff:
                self._records.popleft()

    def clear(self) -> None:
        """Odin session 改变时清除不能跨建系复用的历史。"""
        with self._lock:
            self._records.clear()

    def latest(self) -> Any | None:
        """只供启动前可用性检查，不参与图像几何计算。"""
        with self._lock:
            return self._records[-1].message if self._records else None

    def match(self, image_stamp_ns: int) -> OdomMatch | None:
        """优先严格匹配 header；仅在两个时间 epoch 不兼容时使用接收时间。"""
        with self._lock:
            records = tuple(self._records)
        if not records:
            return None

        image_stamp_ns = int(image_stamp_ns)
        closest_header = min(
            records, key=lambda item: abs(item.header_ns - image_stamp_ns)
        )
        header_delta_ns = abs(closest_header.header_ns - image_stamp_ns)
        epoch_tolerance_ns = int(
            self._settings.header_epoch_tolerance_seconds * 1_000_000_000
        )
        if header_delta_ns <= epoch_tolerance_ns:
            if header_delta_ns > int(self._settings.max_header_delta_ms * 1_000_000):
                return None
            return OdomMatch(
                message=closest_header.message,
                delta_ms=header_delta_ns / 1_000_000.0,
                time_source="header",
            )

        closest_arrival = min(
            records, key=lambda item: abs(item.arrival_ns - image_stamp_ns)
        )
        arrival_delta_ns = abs(closest_arrival.arrival_ns - image_stamp_ns)
        if arrival_delta_ns > int(self._settings.max_arrival_delta_ms * 1_000_000):
            return None
        return OdomMatch(
            message=closest_arrival.message,
            delta_ms=arrival_delta_ns / 1_000_000.0,
            time_source="arrival_history",
        )


def is_apply_time_source_safe(time_source: str) -> bool:
    """只允许传感器 header 或有界历史接收时间匹配，拒绝 mixed/最新值。"""
    return str(time_source) in APPLY_SAFE_TIME_SOURCES
