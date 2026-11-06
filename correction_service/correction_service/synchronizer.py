"""原生 Odin 匹配结果、ROS 时间转换与应用时间轴约束。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# 仅允许同源 header 或有界历史接收时间匹配用于校准应用。
APPLY_SAFE_TIME_SOURCES = frozenset(("header", "arrival_history"))


def stamp_to_nanoseconds(stamp: Any) -> int:
    """把 ROS builtin_interfaces/Time 转为有符号纳秒整数。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class OdomMatch:
    """图像对应的 Odin 消息、绝对时间差和实际采用的时间轴。"""

    message: Any
    delta_ms: float
    time_source: str


def is_apply_time_source_safe(time_source: str) -> bool:
    """只允许传感器 header 或有界历史接收时间匹配，拒绝 mixed/最新值。"""
    return str(time_source) in APPLY_SAFE_TIME_SOURCES
