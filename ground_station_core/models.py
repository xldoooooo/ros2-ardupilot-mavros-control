"""地面站跨模块共享的不可变数据模型。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FlightMode(str, Enum):
    """GUI 层的互斥飞行控制模式。"""

    IDLE = "待机"
    TAKEOFF_LAND = "起飞/降落"
    KEYBOARD = "键盘控制"
    WAYPOINT = "航点飞行"


@dataclass(frozen=True)
class VehicleSnapshot:
    """来自 MAVROS 回调的飞行器状态快照。"""

    connected: bool = False
    armed: bool = False
    autopilot_mode: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    local_position_valid: bool = False


@dataclass(frozen=True)
class CommandRequest:
    """从 GUI 投递到 ROS 后台线程的命令。"""

    ticket: int
    name: str
    argument: object = None
    flight_action: int = 0


@dataclass(frozen=True)
class CommandResult:
    """ROS 后台线程返回给 GUI 或命令行测试的结果。"""

    sequence: int
    ticket: int
    command: str
    success: bool
    message: str
    final: bool = True
