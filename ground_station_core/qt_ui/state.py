"""把机载权威快照转换为 Qt 控件启用策略。"""

from __future__ import annotations

from dataclasses import dataclass

from ..models import FlightMode, VehicleSnapshot


@dataclass(frozen=True)
class UiAvailability:
    """一次界面刷新内使用的完整按钮状态与禁用原因。"""

    start_environment: bool
    stop_simulation: bool
    disconnect_hardware: bool
    communication_test: bool
    origin_settings: bool
    takeoff: bool
    land: bool
    motion: bool
    hover: bool
    waypoint_send: bool
    waypoint_preview: bool
    waypoint_edit: bool
    waypoint_configuration: bool
    flight_reason: str


def derive_availability(
    snapshot: VehicleSnapshot,
    *,
    ros_ready: bool,
    busy: bool,
    closing: bool,
    environment_active: bool,
    connection_mode: str = "none",
    pending_mode: str = "none",
    waypoint_count: int,
    waypoint_running: bool,
    flight_sequence_active: bool = False,
) -> UiAvailability:
    """按连接、租约、飞行安全状态和任务状态计算互斥操作。"""
    mode = str(connection_mode or "none")
    pending = str(pending_mode or "none")
    # 当前会话或正在启动中的会话类型，用于互斥启用对应“关闭/断开”按钮。
    simulation_session = mode == "simulation" or (busy and pending == "simulation")
    hardware_session = mode == "hardware" or (busy and pending == "hardware")

    # 仿真或实机会话已建立时禁止再次点启动，须先断开/清理。
    # ROS is started lazily by the selected workflow, so an idle client must not
    # disable the three entry points and force DDS discovery at GUI startup.
    start_environment = not busy and not closing and not environment_active
    # Wi-Fi 检测与环境初始化互斥，避免在既有租约会话中伪装成零命令诊断。
    communication_test = start_environment
    # 仅本会话类型可关：仿真中禁用“断开实机”，实机中禁用“关闭仿真”。
    stop_simulation = not closing and simulation_session
    disconnect_hardware = not closing and hardware_session
    # 原点齿轮仅本地缓存，但会话中/工作流进行中禁止改，避免与已启动环境语义混淆。
    origin_settings = not closing and not busy and not environment_active

    if closing:
        reason = "地面站正在安全退出"
    elif busy:
        reason = "环境或通讯工作流正在执行"
    elif not ros_ready:
        reason = "ROS 2 客户端尚未就绪"
    elif not environment_active:
        reason = "尚未启动仿真或连接机载服务"
    elif not snapshot.onboard_available:
        reason = "尚未连接机载控制服务"
    elif snapshot.endpoint_conflict:
        reason = "检测到多个机载状态发布者，已禁止控制"
    elif not snapshot.connected:
        reason = "机载服务尚未连接飞控"
    elif not snapshot.control_authority:
        reason = "当前地面站没有控制权"
    elif not snapshot.local_position_valid:
        reason = "本地位置尚未就绪"
    elif not snapshot.thrust_mode_verified:
        reason = "尚未验证 GUID_OPTIONS 真实推力语义"
    elif snapshot.setpoint_conflict:
        reason = "检测到姿态 setpoint 发布者冲突"
    elif snapshot.vehicle_abnormal:
        reason = snapshot.vehicle_abnormal_reason or "无人机状态异常"
    elif flight_sequence_active:
        reason = "上位机组合操作仍在执行"
    else:
        reason = "飞行控制链路已就绪"

    # LAND 不应被 UI 工作流互斥锁死；只要可靠的持权命令链仍在，
    # 即使正处于 LAND 或上一个 LAND 请求尚未终结，也允许幂等重发。
    reliable_command_link = (
        ros_ready
        and not closing
        and environment_active
        and snapshot.onboard_available
        and not snapshot.endpoint_conflict
        and snapshot.connected
        and snapshot.control_authority
    )
    command_link = reliable_command_link and not busy
    control_ready = (
        command_link
        and snapshot.local_position_valid
        and snapshot.thrust_mode_verified
        and not snapshot.setpoint_conflict
    )
    airborne_control_mode = snapshot.active_mode in {
        FlightMode.KEYBOARD,
        FlightMode.HOVER,
        FlightMode.WAYPOINT,
    }

    return UiAvailability(
        start_environment=start_environment,
        stop_simulation=stop_simulation,
        disconnect_hardware=disconnect_hardware,
        communication_test=communication_test,
        origin_settings=origin_settings,
        takeoff=(
            control_ready
            and not snapshot.vehicle_abnormal
            and not flight_sequence_active
            and not snapshot.armed
            and snapshot.active_mode is FlightMode.IDLE
        ),
        # LAND 是安全动作：不受工作流、模式、位置或推力诊断门控。
        land=reliable_command_link and snapshot.armed,
        motion=control_ready and snapshot.armed and airborne_control_mode,
        hover=control_ready and snapshot.armed and airborne_control_mode,
        waypoint_send=(
            control_ready
            and snapshot.armed
            and airborne_control_mode
            and waypoint_count > 0
            and not waypoint_running
            and not snapshot.vehicle_abnormal
        ),
        # 预览只发布本地只读 Marker/Path，不要求武装或控制租约；仍须绑定明确会话。
        waypoint_preview=(
            ros_ready
            and not busy
            and not closing
            and environment_active
            and mode in {"simulation", "hardware"}
            and waypoint_count > 0
        ),
        # 无环境会话时禁止编辑/操作任何航点组件；任务运行中同样锁定编辑。
        waypoint_edit=(
            environment_active and not closing and not waypoint_running
        ),
        # 三种算法选择只允许在解除武装后修改；起飞后保留已选值供任务上传。
        waypoint_configuration=(
            environment_active
            and not closing
            and not busy
            and not waypoint_running
            and snapshot.onboard_available
            and snapshot.connected
            and not snapshot.armed
            and snapshot.active_mode is FlightMode.IDLE
        ),
        flight_reason=reason,
    )
