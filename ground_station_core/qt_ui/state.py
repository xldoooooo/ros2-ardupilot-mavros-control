"""把机载权威快照转换为 Qt 控件启用策略。"""

from __future__ import annotations

from dataclasses import dataclass

from ..models import FlightMode, VehicleSnapshot


@dataclass(frozen=True)
class UiAvailability:
    """一次界面刷新内使用的完整按钮状态与禁用原因。"""

    start_environment: bool
    cleanup: bool
    set_origin: bool
    takeoff: bool
    land: bool
    motion: bool
    hover: bool
    waypoint_send: bool
    waypoint_edit: bool
    flight_reason: str


def derive_availability(
    snapshot: VehicleSnapshot,
    *,
    ros_ready: bool,
    busy: bool,
    closing: bool,
    environment_active: bool,
    waypoint_count: int,
    waypoint_running: bool,
) -> UiAvailability:
    """按连接、租约、飞行安全状态和任务状态计算互斥操作。"""
    start_environment = ros_ready and not busy and not closing
    # 初始化期间保留“断开”作为取消入口；实际清理线程会在主窗口再防重复。
    cleanup = not closing and (
        busy or environment_active or snapshot.onboard_available
    )

    if closing:
        reason = "地面站正在安全退出"
    elif busy:
        reason = "环境工作流正在执行"
    elif not ros_ready:
        reason = "ROS 2 客户端尚未就绪"
    elif not snapshot.onboard_available:
        reason = "尚未连接机载控制服务"
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
    else:
        reason = "飞行控制链路已就绪"

    command_link = (
        ros_ready
        and not busy
        and not closing
        and environment_active
        and snapshot.onboard_available
        and snapshot.connected
        and snapshot.control_authority
    )
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
        cleanup=cleanup,
        set_origin=control_ready and not snapshot.armed,
        takeoff=(
            control_ready
            and not snapshot.armed
            and snapshot.active_mode is FlightMode.IDLE
        ),
        # LAND 是安全动作：只要求可靠命令链路，不受位置/推力诊断门控。
        land=(
            command_link
            and snapshot.armed
            and snapshot.active_mode is not FlightMode.LAND
        ),
        motion=control_ready and snapshot.armed and airborne_control_mode,
        hover=control_ready and snapshot.armed and airborne_control_mode,
        waypoint_send=(
            control_ready
            and snapshot.armed
            and airborne_control_mode
            and waypoint_count > 0
            and not waypoint_running
        ),
        waypoint_edit=not closing and not waypoint_running,
        flight_reason=reason,
    )
