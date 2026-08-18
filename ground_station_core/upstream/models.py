"""上位机通讯线程、协议映射与 Qt 适配层共享的不可变模型。"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Any


class UpstreamAction(str, Enum):
    """上位机命令转换后的地面站高层动作。"""

    TAKEOFF = "takeoff"
    STAGE_WAYPOINTS = "stage_waypoints"
    EXECUTE_WAYPOINTS = "execute_waypoints"
    RETURN_HOME = "return_home"
    LAND = "land"
    EMERGENCY_LAND = "emergency_land"


@dataclass(frozen=True)
class UpstreamCommand:
    """已经完成字段校验和单位转换、可投递给 Qt 主线程的命令。"""

    client_no: str
    command_no: str
    label: str
    action: UpstreamAction
    raw_payload: Mapping[str, Any]
    waypoints: tuple[tuple[float, float, float, float], ...] = ()
    point_indexes: tuple[int, ...] = ()
    ignored_camera_fields: bool = False


@dataclass(frozen=True)
class RawFrame:
    """仅供上位机通讯面板展示的一条原始 WebSocket 文本帧。"""

    sequence: int
    timestamp: datetime
    direction: str
    payload: str


@dataclass(frozen=True)
class UpstreamConnectionSnapshot:
    """面板轮询使用的线程安全连接状态快照。"""

    url: str
    client_no: str
    desired_connected: bool
    connected: bool
    state: str
    detail: str


@dataclass(frozen=True)
class UpstreamStandbyPolicy:
    """机库位置阈值与巡检后待机延时的可配策略。"""

    x_tolerance_meters: float = 1.0
    y_tolerance_meters: float = 1.0
    z_tolerance_meters: float = 0.5
    inspection_delay_seconds: float = 60.0

    def __post_init__(self) -> None:
        """拒绝会让机库判定失去边界的非有限或非正参数。"""
        tolerances = (
            self.x_tolerance_meters,
            self.y_tolerance_meters,
            self.z_tolerance_meters,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in tolerances):
            raise ValueError("机库 XYZ 阈值必须是正的有限数")
        if (
            not math.isfinite(self.inspection_delay_seconds)
            or self.inspection_delay_seconds < 0.0
        ):
            raise ValueError("巡检后待机延时必须是非负有限数")
