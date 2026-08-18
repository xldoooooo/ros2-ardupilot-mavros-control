"""把地面站权威快照与命令结果投影为上位机协议状态。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from ..event_log import EventLog
from ..models import CommandResult, FlightMode, VehicleSnapshot
from .models import UpstreamStandbyPolicy
from .protocol import make_status


@dataclass
class _MissionState:
    """一项由上位机触发且尚未得到终态的航点任务。"""

    ticket: int
    kind: str
    point_indexes: tuple[int, ...]
    started: bool = False
    reported_points: int = 0


class UpstreamStatusProjector:
    """在 Qt 线程内维护事件边沿，发送端仅接收不可变 JSON 副本。"""

    TELEMETRY_PERIOD_SECONDS = 1.0
    SIMULATION_LOW_PERCENTAGE = 20.0
    HARDWARE_LOW_VOLTAGE = 22.2

    def __init__(
        self,
        client_no: Callable[[], str],
        emit: Callable[[Mapping[str, Any]], bool],
        event_log: EventLog,
        standby_policy: UpstreamStandbyPolicy | None = None,
    ) -> None:
        self._client_no = client_no
        self._emit = emit
        self._events = event_log
        self._standby_policy = standby_policy or UpstreamStandbyPolicy()
        self._lock = threading.RLock()
        self._last_telemetry_at = 0.0
        self._low_power_reported = False
        self._mission: _MissionState | None = None
        self._landing_ticket: int | None = None
        self._landing_reported = False
        self._standby_blocked = False
        self._standby_reported = False
        self._generic_state_notice_logged = False

    @property
    def standby_policy(self) -> UpstreamStandbyPolicy:
        """返回面板和组合动作共用的不可变待机策略。"""
        return self._standby_policy

    def block_standby(self) -> None:
        """任务下发或执行期间禁止意外发送 01。"""
        with self._lock:
            self._standby_blocked = True

    def release_standby(self) -> None:
        """组合动作完成后重新允许入库待机边沿。"""
        with self._lock:
            self._standby_blocked = False

    def is_in_hangar(self, snapshot: VehicleSnapshot) -> bool:
        """用当前可配 XYZ 阈值判定飞行器是否位于机库。"""
        policy = self._standby_policy
        return snapshot.local_position_valid and all(
            math.isfinite(value)
            for value in (snapshot.x, snapshot.y, snapshot.z)
        ) and (
            abs(snapshot.x) < policy.x_tolerance_meters
            and abs(snapshot.y) < policy.y_tolerance_meters
            and abs(snapshot.z) < policy.z_tolerance_meters
        )

    def report_waypoints_staged(self) -> bool:
        """GUI 已成功替换航点后发送一次 02 状态。"""
        return self._send("02")

    def begin_mission(
        self, ticket: int, kind: str, point_indexes: tuple[int, ...]
    ) -> None:
        """绑定本地 ROS ticket，避免旧任务终态误关闭新返航任务。"""
        with self._lock:
            self._mission = _MissionState(
                ticket=int(ticket), kind=str(kind), point_indexes=tuple(point_indexes)
            )

    def begin_landing(self, ticket: int) -> None:
        """记录降落请求；真正进入 LAND 或收到运行回报后才发送 07。"""
        with self._lock:
            self._landing_ticket = int(ticket)
            self._landing_reported = False

    def observe_vehicle(
        self,
        snapshot: VehicleSnapshot,
        connection_mode: str,
        *,
        now: float | None = None,
        can_takeoff: bool = False,
    ) -> bool:
        """投影任务/待机边沿和 1 Hz 遥测，返回低电量返航需求。"""
        if connection_mode not in {"simulation", "hardware"}:
            return False
        current_time = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._generic_state_notice_logged:
                # TODO(上位机通用状态): 协议未给飞控/控制权变化独立编号；
                # 现阶段只发送表 2 中有确定映射的任务状态与遥测。
                self._events.info(
                    "upstream",
                    "飞控与控制权变化当前没有独立状态编号；仅上报已定义的任务状态和遥测",
                )
                self._generic_state_notice_logged = True
            self._observe_mission(snapshot)
            if (
                self._landing_ticket is not None
                and not self._landing_reported
                and snapshot.active_mode is FlightMode.LAND
            ):
                self._send("07")
                self._landing_reported = True
            low_power_return_required = False
            if (
                current_time - self._last_telemetry_at
                >= self.TELEMETRY_PERIOD_SECONDS
            ):
                self._last_telemetry_at = current_time
                low_power_return_required = self._send_telemetry(
                    snapshot, connection_mode
                )
            if snapshot.armed:
                self._standby_reported = False
            if (
                can_takeoff
                and not self._standby_blocked
                and not self._standby_reported
                and self.is_in_hangar(snapshot)
                and self._send("01")
            ):
                self._standby_reported = True
            return low_power_return_required

    def observe_result(self, result: CommandResult) -> None:
        """用同一可靠终态驱动 GUI 完成态与上位机 08，杜绝推测完成。"""
        with self._lock:
            if result.command == "land" and result.ticket == self._landing_ticket:
                if result.success and not self._landing_reported:
                    self._send("07")
                    self._landing_reported = True
                if result.final:
                    self._landing_ticket = None
                return

            mission = self._mission
            if (
                result.command != "waypoints"
                or mission is None
                or result.ticket != mission.ticket
            ):
                return
            if result.success and not result.final and not mission.started:
                # 第一条可靠 RUNNING 表示机载已接收；模式快照通常会在随后一帧确认。
                return
            if not result.final:
                return
            if result.success:
                if mission.kind == "inspection":
                    self._report_points(mission, len(mission.point_indexes))
                    # TODO(上位机媒体): 相机、照片和媒体尚未接入，路径按空字符串上报。
                    self._send("08", {"videoPath": "", "JPGPath": ""})
            else:
                self._events.warn(
                    "upstream",
                    f"上位机触发的航点任务未完成，不发送 08：{result.message}",
                )
            self._mission = None

    def reset_runtime(self) -> None:
        """环境断开时丢弃飞行事件关联，不影响 WebSocket 独立连接。"""
        with self._lock:
            self._mission = None
            self._landing_ticket = None
            self._landing_reported = False
            self._last_telemetry_at = 0.0
            self._low_power_reported = False
            self._standby_blocked = False
            self._standby_reported = False

    def _observe_mission(self, snapshot: VehicleSnapshot) -> None:
        """按机载 current-target 索引计算已完成格数，与 GUI 采用同一规则。"""
        mission = self._mission
        if mission is None:
            return
        if (
            snapshot.active_mode is FlightMode.WAYPOINT
            and snapshot.active_command_sequence == mission.ticket
            and not mission.started
        ):
            mission.started = self._send(
                "05" if mission.kind == "return" else "03"
            )
        if not mission.started or snapshot.active_mode is not FlightMode.WAYPOINT:
            return
        if mission.kind != "inspection":
            return
        completed = max(0, int(snapshot.waypoint_index) - 1)
        completed = min(completed, len(mission.point_indexes))
        self._report_points(mission, completed)

    def _report_points(self, mission: _MissionState, completed: int) -> None:
        """逐格补发 09，短暂跳帧也不会漏掉已完成点位。"""
        while mission.reported_points < completed:
            position = mission.reported_points
            point_index = mission.point_indexes[position]
            # TODO(上位机媒体): pointPic 待相机链路完成后接入真实照片路径。
            self._send(
                "09",
                {
                    "pointNo": str(point_index),
                    "pointName": f"巡检点位 {point_index}",
                    "pointPic": "",
                },
            )
            mission.reported_points += 1

    def _send_telemetry(
        self, snapshot: VehicleSnapshot, connection_mode: str
    ) -> bool:
        """仿真发送百分比、实机发送电压，并返回空中低电量需求。"""
        low_power: bool | None = None
        if snapshot.battery_valid:
            if connection_mode == "simulation":
                power = snapshot.battery_percentage * 100.0
                if math.isfinite(power):
                    power = max(0.0, min(100.0, power))
                    self._send("0A", {"uavPower": round(power, 2)})
                    low_power = power < self.SIMULATION_LOW_PERCENTAGE
            else:
                voltage = snapshot.battery_voltage
                if math.isfinite(voltage) and voltage > 0.0:
                    self._send("0A", {"uavPower": round(voltage, 2)})
                    low_power = voltage < self.HARDWARE_LOW_VOLTAGE
        if snapshot.local_position_valid and all(
            math.isfinite(value) for value in (snapshot.x, snapshot.y, snapshot.z)
        ):
            self._send(
                "0B",
                {
                    "X": round(snapshot.x, 3),
                    "Y": round(snapshot.y, 3),
                    "Z": round(snapshot.z, 3),
                },
            )
        if low_power is True and not self._low_power_reported and self._send("0C"):
            self._low_power_reported = True
            self._events.warn("upstream", "已上报低电量 0C")
        elif low_power is False:
            self._low_power_reported = False
        # 上报连接不影响本地安全动作；低电量持续时每个
        # 遥测周期重申，直到组合动作被地面站接受或飞行器降落。
        return low_power is True and snapshot.armed

    def _send(self, status: str, data: Mapping[str, Any] | None = None) -> bool:
        """使用当前面板配置的无人机编号生成并排队状态。"""
        try:
            return self._emit(make_status(self._client_no(), status, data))
        except Exception as exc:  # noqa: BLE001 - 状态插件故障不得影响 GUI 刷新。
            self._events.warn("upstream", f"上位机状态 {status} 排队失败：{exc}")
            return False
