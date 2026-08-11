"""地面站薄客户端跨 GUI、ROS 与环境编排共享的不可变模型。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FlightMode(str, Enum):
    """由机载状态消息报告、GUI 只读展示的控制模式。"""

    IDLE = "待机"
    TAKEOFF = "起飞"
    KEYBOARD = "键盘 PD+DOB"
    HOVER = "悬停 PD+DOB"
    WAYPOINT = "航点飞行"
    LAND = "降落"
    FAILSAFE = "失联保护"


class WaypointFlightStrategy(int, Enum):
    """航点任务飞行策略；数值与 ExecuteWaypoints.srv 常量对齐。

    当前机载端仅实现 STRAIGHT；AVOID / HOVER_ON_OBSTACLE 为预留，
    地面站与机载均按直线飞行执行，直至避障能力落地。
    """

    STRAIGHT = 0
    AVOID = 1
    HOVER_ON_OBSTACLE = 2

    @property
    def label(self) -> str:
        """返回设置菜单与下拉框使用的中文名称。"""
        return {
            WaypointFlightStrategy.STRAIGHT: "直线飞行",
            WaypointFlightStrategy.AVOID: "自动避障",
            WaypointFlightStrategy.HOVER_ON_OBSTACLE: "遇到障碍悬停",
        }[self]

    @classmethod
    def from_value(cls, value: object) -> "WaypointFlightStrategy":
        """将任意整型/枚举值规范为已知策略，未知值回退为直线飞行。"""
        try:
            return cls(int(value))
        except (TypeError, ValueError):
            return cls.STRAIGHT


@dataclass(frozen=True)
class VehicleSnapshot:
    """来自机载聚合状态接口的飞行器、租约和控制诊断快照。"""

    onboard_available: bool = False
    interface_version: str = ""
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
    roll: float = 0.0
    pitch: float = 0.0
    battery_valid: bool = False
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_percentage: float = 0.0
    status_rate_hz: float = 0.0
    status_age_seconds: float = 0.0
    local_position_valid: bool = False
    active_mode: FlightMode = FlightMode.IDLE
    controller_active: bool = False
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_yaw: float = 0.0
    target_vx: float = 0.0
    target_vy: float = 0.0
    target_vz: float = 0.0
    target_yaw_rate: float = 0.0
    lease_owner: str = ""
    lease_active: bool = False
    control_authority: bool = False
    waypoint_index: int = 0
    waypoint_count: int = 0
    message_rates_configured: bool = False
    thrust_mode_verified: bool = False
    hover_throttle: float = 0.0
    endpoint_conflict: bool = False
    setpoint_conflict: bool = False
    failsafe_reason: str = ""
    status_message: str = ""
    control_rate_hz: float = 0.0
    max_jitter_ms: float = 0.0
    deadline_miss_count: int = 0


@dataclass(frozen=True)
class CommandRequest:
    """从 GUI 投递给地面站 ROS 客户端线程的高层请求。"""

    ticket: int
    name: str
    argument: object = None


@dataclass(frozen=True)
class CommandResult:
    """ROS 后台线程返回给 GUI 或命令行测试的结果。"""

    sequence: int
    ticket: int
    command: str
    success: bool
    message: str
    final: bool = True
