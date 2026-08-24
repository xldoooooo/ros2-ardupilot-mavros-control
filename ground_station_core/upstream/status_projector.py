"""把地面站权威快照与命令结果投影为上位机协议状态。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from .models import UpstreamStandbyPolicy
from .protocol import make_status
from ..config import INTERFACE_VERSION
from ..event_log import EventLog
from ..models import (
    CommandResult,
    FlightMode,
    VehicleSnapshot,
    VideoCaptureEvent,
    VideoServiceSnapshot,
)


@dataclass
class _MissionState:
    """一项由上位机触发且尚未得到终态的航点任务。"""

    ticket: int
    kind: str
    point_indexes: tuple[int, ...]
    started: bool = False
    reported_points: int = 0
    completed_points: int = 0
    picture_paths: dict[int, str] = field(default_factory=dict)
    flight_completed: bool = False
    landing_completed: bool = False
    media_deadline: float = 0.0


class UpstreamStatusProjector:
    """在 Qt 线程内维护事件边沿，发送端仅接收不可变 JSON 副本。"""

    TELEMETRY_PERIOD_SECONDS = 1.0
    SIMULATION_LOW_PERCENTAGE = 20.0
    HARDWARE_LOW_VOLTAGE = 22.2
    MEDIA_RESULT_WAIT_SECONDS = 15.0

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
        self._video = VideoServiceSnapshot()

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
            self._finish_completed_mission(current_time)
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
                mission = self._mission
                if (
                    result.final
                    and result.success
                    and mission is not None
                    and mission.kind == "inspection"
                    and mission.flight_completed
                ):
                    # 降落可靠终态来自解除武装；视频关闭也由同一真实边沿触发。
                    # 从此刻起等待最后图片结果和封装后的 VideoStatus 路径。
                    mission.landing_completed = True
                    mission.media_deadline = (
                        time.monotonic() + self.MEDIA_RESULT_WAIT_SECONDS
                    )
                if result.final:
                    self._landing_ticket = None
                self._finish_completed_mission(time.monotonic())
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
                    mission.completed_points = len(mission.point_indexes)
                    mission.flight_completed = True
                    self._report_points(mission)
                else:
                    self._mission = None
            else:
                self._events.warn(
                    "upstream",
                    f"上位机触发的航点任务未完成，不发送 08：{result.message}",
                )
                self._mission = None

    def observe_video_status(self, snapshot: VideoServiceSnapshot) -> None:
        """保存独立视频服务新鲜快照，供巡检 08 填写实际媒体路径。"""
        with self._lock:
            self._video = snapshot
            self._finish_completed_mission(time.monotonic())

    def observe_video_capture(self, event: VideoCaptureEvent) -> None:
        """按任务与 1-based 航点索引关联真实截图结果，乱序结果可先缓存。"""
        with self._lock:
            mission = self._mission
            if (
                mission is None
                or mission.kind != "inspection"
                or event.mission_sequence != mission.ticket
                or not 1 <= event.waypoint_index <= len(mission.point_indexes)
            ):
                return
            mission.picture_paths[event.waypoint_index] = (
                event.path if event.success else ""
            )
            if not event.success:
                self._events.warn(
                    "upstream",
                    f"巡检点 {event.waypoint_index} 抓拍失败：{event.message}",
                )
            self._report_points(mission)
            self._finish_completed_mission(time.monotonic())

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
            self._video = VideoServiceSnapshot()

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
        mission.completed_points = max(mission.completed_points, completed)
        self._report_points(mission)

    def _report_points(self, mission: _MissionState) -> None:
        """仅在真实抓拍结果到达后按协议点号顺序发送 09。"""
        while mission.reported_points < mission.completed_points:
            position = mission.reported_points
            waypoint_index = position + 1
            if waypoint_index not in mission.picture_paths:
                return
            point_index = mission.point_indexes[position]
            self._send(
                "09",
                {
                    "pointNo": str(point_index),
                    "pointName": f"巡检点位 {point_index}",
                    "pointPic": mission.picture_paths[waypoint_index],
                },
            )
            mission.reported_points += 1

    def _finish_completed_mission(self, now: float) -> None:
        """全部图片已回或超时后发送 08；媒体失败不反向改变飞行结果。"""
        mission = self._mission
        if (
            mission is None
            or mission.kind != "inspection"
            or not mission.flight_completed
            or not mission.landing_completed
        ):
            return
        expected = len(mission.point_indexes)
        all_results_arrived = len(mission.picture_paths) >= expected
        compatible_video = (
            self._video.service_available
            and self._video.interface_version == INTERFACE_VERSION
        )
        video_finalized = (
            compatible_video
            and not self._video.running
            and bool(self._video.last_video_path)
        )
        if (not all_results_arrived or not video_finalized) and now < mission.media_deadline:
            return
        if not all_results_arrived:
            missing = [
                index
                for index in range(1, expected + 1)
                if index not in mission.picture_paths
            ]
            for index in missing:
                mission.picture_paths[index] = ""
            self._events.warn(
                "upstream",
                f"等待巡检图片结果超时，点位 {missing} 的 pointPic 按空路径如实上报",
            )
        self._report_points(mission)
        video_path = ""
        jpg_path = ""
        if compatible_video:
            jpg_path = self._video.image_directory
            if video_finalized:
                video_path = self._video.last_video_path
            else:
                self._events.warn(
                    "upstream",
                    "等待录像封装超时，08 的 videoPath 按空值如实上报",
                )
        else:
            self._events.warn(
                "upstream",
                "视频服务状态不可用或接口版本不兼容，08 的媒体路径按空值如实上报",
            )
        self._send("08", {"videoPath": video_path, "JPGPath": jpg_path})
        self._mission = None

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
        # 投影器只计算低电量候选；服务层仅在 WebSocket 在线时开放动作触发。
        # 低电量持续时每个遥测周期重申，直到组合被接受或飞行器降落。
        return low_power is True and snapshot.armed

    def _send(self, status: str, data: Mapping[str, Any] | None = None) -> bool:
        """使用当前面板配置的无人机编号生成并排队状态。"""
        try:
            return self._emit(make_status(self._client_no(), status, data))
        except Exception as exc:  # noqa: BLE001 - 状态插件故障不得影响 GUI 刷新。
            self._events.warn("upstream", f"上位机状态 {status} 排队失败：{exc}")
            return False
