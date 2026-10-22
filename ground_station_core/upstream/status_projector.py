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
from ..event_log import EventLog
from ..ground_media_reader import processed_download
from ..models import (
    CommandResult,
    FlightMode,
    VehicleSnapshot,
    VideoCaptureEvent,
    VideoServiceSnapshot,
)

# 这些状态已明确不能交付处理文件；传输 retry 仍可恢复，继续等待。
MEDIA_FAILURE_STATES = frozenset({"error", "processing_error", "deleted", "missing"})


@dataclass
class _MissionState:
    """一项由上位机触发且尚未得到终态的航点任务。"""

    ticket: int
    kind: str
    point_indexes: tuple[int, ...]
    source_id: str = ""
    aircraft_id: str = ""
    client_no: str = ""
    started: bool = False
    reported_points: int = 0
    completed_points: int = 0
    captures: dict[int, VideoCaptureEvent] = field(default_factory=dict)
    flight_completed: bool = False
    landing_completed: bool = False
    completion_data: dict[str, Any] | None = None


class UpstreamStatusProjector:
    """用锁保护 GUI 飞行事件与后台地面媒体快照，发送独立 JSON 副本。"""

    # 遥测上报周期，以及仿真百分比/实机电压的低电量阈值。
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
        """维护飞行边沿与独立地面媒体记录，不执行网络请求。"""
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
        self._media_records: tuple[Mapping[str, Any], ...] = ()
        self._media_error = ""
        self._completed_missions: list[_MissionState] = []

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
        self,
        ticket: int,
        kind: str,
        point_indexes: tuple[int, ...],
        *,
        source_id: str = "",
        aircraft_id: str = "",
    ) -> None:
        """绑定本地 ROS ticket，避免旧任务终态误关闭新返航任务。"""
        with self._lock:
            self._mission = _MissionState(
                ticket=int(ticket),
                kind=str(kind),
                point_indexes=tuple(point_indexes),
                source_id=source_id,
                aircraft_id=aircraft_id,
                client_no=self._client_no(),
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
            self._finish_completed_mission()
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
                    # 飞行完成不等待上传、去畸变或录像封装。
                    mission.landing_completed = True
                if result.final:
                    self._landing_ticket = None
                self._finish_completed_mission()
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
        """机载 VideoStatus 仅表示机载状态，不能作为地面下载路径。"""

    def observe_ground_media(
        self, records: tuple[Mapping[str, Any], ...], error: str = ""
    ) -> None:
        """接收后台地面目录快照，并补发本次任务实际可下载的媒体。"""
        with self._lock:
            self._media_records = records
            self._media_error = error
            missions = list(self._completed_missions)
            if self._mission is not None:
                missions.append(self._mission)
            for mission in missions:
                self._report_points(mission)
                if mission.landing_completed and mission.flight_completed:
                    self._report_completed_media(mission)

    def observe_video_capture(self, event: VideoCaptureEvent) -> None:
        """按任务与 1-based 航点索引关联真实截图结果，乱序结果可先缓存。"""
        with self._lock:
            missions = [self._mission, *reversed(self._completed_missions)]
            mission = next((item for item in missions if (
                item is not None and item.kind == "inspection"
                and event.mission_sequence == item.ticket
                and (not item.source_id or event.source_id == item.source_id)
                and 1 <= event.waypoint_index <= len(item.point_indexes)
            )), None)
            if mission is None:
                return
            if not mission.source_id:
                mission.source_id = event.source_id
            mission.captures[event.waypoint_index] = event
            if not event.success:
                self._events.warn(
                    "upstream",
                    f"巡检点 {event.waypoint_index} 抓拍失败：{event.message}",
                )
            self._report_points(mission)
            if mission.completion_data is not None:
                self._report_completed_media(mission)
            self._finish_completed_mission()

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
            # 已完成任务仍可在飞机离线后收到地面媒体入库更新。

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

    def _mission_records(self, mission: _MissionState) -> list[Mapping[str, Any]]:
        """用客户端身份和任务编号隔离多飞机、重启前后的同号任务。"""
        if not mission.source_id:
            return []
        records = []
        for record in self._media_records:
            metadata = record.get("metadata") or {}
            if not isinstance(metadata, Mapping):
                continue
            sequence = record.get("mission_sequence", metadata.get("mission_sequence"))
            if (
                str(metadata.get("source_id", "")) == mission.source_id
                and str(sequence) == str(mission.ticket)
                and (not mission.aircraft_id or record.get("aircraft_id") == mission.aircraft_id)
            ):
                records.append(record)
        return records

    def _select_media(
        self, records: list[Mapping[str, Any]]
    ) -> Mapping[str, Any]:
        """优先使用可下载重建版，其次实验版；原片永不成为对外路径。"""
        processed = [r for r in records if r.get("variant") in {"experiment", "rebuilt"}]
        # 原片仅作上传/处理失败的状态来源，下载路径仍由 processed_download 限制。
        return max(processed or records, key=lambda r: (
            bool(processed_download(r)), r.get("variant") == "rebuilt",
            r.get("state") == "ready",
        ), default={})

    def _point_media(self, mission: _MissionState, index: int) -> dict[str, Any]:
        """关联真实抓拍编号与地面处理结果，不信任机载本地路径。"""
        event = mission.captures.get(index)
        records = []
        for record in self._mission_records(mission):
            metadata = record.get("metadata") or {}
            if record.get("kind") != "image":
                continue
            waypoint = record.get("waypoint_index", metadata.get("waypoint_index"))
            photo = record.get("photo_no", metadata.get("photo_no", ""))
            if str(waypoint) != str(index):
                continue
            if event is not None and (
                str(photo) != event.photo_no
                or (metadata.get("sequence") is not None
                    and str(metadata["sequence"]) != str(event.command_sequence))
            ):
                continue
            records.append(record)
        record = self._select_media(records)
        path = processed_download(record)
        state = str(record.get("state") or "pending")
        error = str(record.get("error") or "")
        if event is not None and not event.success and not path:
            state, error = "error", event.message
        return {
            "pointNo": str(mission.point_indexes[index - 1]),
            "photoNo": event.photo_no if event is not None else str(record.get("photo_no") or ""),
            "path": path, "state": state, "error": error,
            "directory": str(record.get("download_directory") or "") if path else "",
        }

    def _report_points(self, mission: _MissionState) -> None:
        """处理版可下载或抓拍明确失败后，按点号顺序发送 09。"""
        while mission.reported_points < mission.completed_points:
            index = mission.reported_points + 1
            media = self._point_media(mission, index)
            if not media["path"] and media["state"] not in MEDIA_FAILURE_STATES:
                return
            point = mission.point_indexes[index - 1]
            payload = {
                "pointNo": str(point), "pointName": f"巡检点位 {point}",
                "pointPic": media["path"], "mediaStatus": media["state"],
                "mediaError": media["error"],
            }
            if not self._send_mission(mission, "09", payload):
                return
            mission.reported_points += 1

    def _finish_completed_mission(self) -> None:
        """飞行可靠结束立即发送 08，媒体随后由地面目录独立更新。"""
        mission = self._mission
        if (
            mission is None or mission.kind != "inspection"
            or not mission.flight_completed or not mission.landing_completed
        ):
            return
        self._report_points(mission)
        self._report_completed_media(mission)
        self._completed_missions.append(mission)
        self._mission = None

    def _report_completed_media(self, mission: _MissionState) -> None:
        """媒体状态变化时补发 08；空路径与 incomplete 均如实上报。"""
        records = self._mission_records(mission)
        video = self._select_media([r for r in records if r.get("kind") == "video"])
        video_path = processed_download(video)
        images = [
            self._point_media(mission, index)
            for index in range(1, len(mission.point_indexes) + 1)
        ]
        states = [str(video.get("state") or "pending"), *[item["state"] for item in images]]
        if MEDIA_FAILURE_STATES.intersection(states) or (self._media_error and not self._media_records):
            state = "error"
        elif not video_path or any(not item["path"] for item in images):
            state = "retry" if "retry" in states else "processing"
        else:
            state = "incomplete" if "incomplete" in states else "ready"
        data = {
            "videoPath": video_path,
            "JPGPath": next((item["directory"] for item in images if item["path"]), ""),
            "mediaStatus": state,
            "videoStatus": str(video.get("state") or "pending"),
            "mediaError": (
                self._media_error or str(video.get("error") or "")
                or next((item["error"] for item in images if item["error"]), "")
            ),
            "images": images,
        }
        if data == mission.completion_data:
            return
        payload = {**data, "mediaUpdate": mission.completion_data is not None}
        if self._send_mission(mission, "08", payload):
            mission.completion_data = data

    def _send_mission(
        self, mission: _MissionState, status: str, data: Mapping[str, Any]
    ) -> bool:
        """延迟媒体使用任务开始时的甲方编号，避免切机后归属改变。"""
        try:
            return self._emit(make_status(mission.client_no, status, data))
        except Exception as exc:  # noqa: BLE001 - 媒体插件不得影响飞控。
            self._events.warn("upstream", f"上位机状态 {status} 排队失败：{exc}")
            return False

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
        x, y, z, _ = snapshot.task_pose
        if snapshot.local_position_valid and snapshot.navigation_ready and all(
            math.isfinite(value) for value in (x, y, z)
        ):
            self._send(
                "0B",
                {
                    "X": round(x, 3),
                    "Y": round(y, 3),
                    "Z": round(z, 3),
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
