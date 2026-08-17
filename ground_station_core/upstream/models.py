"""上位机通讯线程、协议映射与 Qt 适配层共享的不可变模型。"""

from __future__ import annotations

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
