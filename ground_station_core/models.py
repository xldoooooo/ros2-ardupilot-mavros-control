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


class WaypointReferenceGenerator(int, Enum):
    """航点命令生成方式；数值与 ExecuteWaypoints.srv 常量对齐。"""

    STEP_POSITION = 0
    SECOND_ORDER_FILTER = 1
    TRAPEZOIDAL_PROFILE = 2
    JERK_LIMITED_S_CURVE = 3

    @property
    def label(self) -> str:
        """返回航点实验下拉框使用的短标签。"""
        return {
            WaypointReferenceGenerator.STEP_POSITION: "位置阶跃（基线）",
            WaypointReferenceGenerator.SECOND_ORDER_FILTER: "二阶命令滤波",
            WaypointReferenceGenerator.TRAPEZOIDAL_PROFILE: "普通梯形速度",
            WaypointReferenceGenerator.JERK_LIMITED_S_CURVE: "限 jerk S 曲线",
        }[self]

    @classmethod
    def from_value(cls, value: object) -> "WaypointReferenceGenerator":
        """把界面/调用值规范为已知方法，未知值安全回退到既有基线。"""
        try:
            return cls(int(value))
        except (TypeError, ValueError):
            return cls.STEP_POSITION


class WaypointTrackingController(int, Enum):
    """航点跟踪控制方式；两项仍复用唯一机载 DobController。"""

    POSITION_PD_DOB = 0
    TRAJECTORY_PD_DOB = 1

    @property
    def label(self) -> str:
        """返回航点实验下拉框使用的短标签。"""
        return {
            WaypointTrackingController.POSITION_PD_DOB: "位置 PD+DOB（基线）",
            WaypointTrackingController.TRAJECTORY_PD_DOB: "轨迹 PD+DOB",
        }[self]

    @classmethod
    def from_value(cls, value: object) -> "WaypointTrackingController":
        """把界面/调用值规范为已知控制器，未知值回退到既有基线。"""
        try:
            return cls(int(value))
        except (TypeError, ValueError):
            return cls.POSITION_PD_DOB


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
    target_ax: float = 0.0
    target_ay: float = 0.0
    target_az: float = 0.0
    target_yaw_rate: float = 0.0
    active_reference_generator: WaypointReferenceGenerator = (
        WaypointReferenceGenerator.STEP_POSITION
    )
    active_tracking_controller: WaypointTrackingController = (
        WaypointTrackingController.POSITION_PD_DOB
    )
    reference_phase: int = 0
    lease_owner: str = ""
    lease_active: bool = False
    control_authority: bool = False
    active_command_sequence: int = 0
    waypoint_index: int = 0
    waypoint_count: int = 0
    waypoint_arrival_failure_count: int = 0
    vehicle_abnormal: bool = False
    vehicle_abnormal_reason: str = ""
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


@dataclass(frozen=True)
class VideoServiceSnapshot:
    """独立视频服务的发现地址、媒体路径和带新鲜度的运行快照。"""

    service_available: bool = False
    interface_version: str = ""
    running: bool = False
    state: str = "unavailable"
    rtsp_url: str = ""
    video_directory: str = ""
    image_directory: str = ""
    current_video_path: str = ""
    last_video_path: str = ""
    last_image_path: str = ""
    last_error: str = ""
    age_seconds: float = float("inf")


@dataclass(frozen=True)
class VideoCaptureEvent:
    """地面站按本地顺序消费的一条视频截图完成事件。"""

    event_sequence: int
    source_id: str
    command_sequence: int
    success: bool
    kind: int
    mission_sequence: int
    waypoint_index: int
    photo_no: str
    path: str
    message: str
