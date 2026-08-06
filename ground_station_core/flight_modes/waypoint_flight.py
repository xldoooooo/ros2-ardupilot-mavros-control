"""航点飞行模式：按序执行本地 ENU 航点并在终点稳定悬停。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Sequence

from ..dob_controller import DobGains, DobPositionController
from ..mode_manager import FlightModeManager
from ..models import FlightMode, VehicleSnapshot


Waypoint = tuple[float, float, float, float]


class WaypointFlightMode:
    """维护航点任务状态，并用共用 PD+DOB 控制器跟踪当前航点。"""

    def __init__(
        self,
        manager: FlightModeManager,
        gains: DobGains,
        result_callback: Callable[[int, bool, str, bool], None],
        tolerance: float = 0.3,
        hold_time: float = 1.0,
    ) -> None:
        """注册航点模式及任务进度回调。"""
        self._manager = manager
        self._lock = threading.RLock()
        self._dob = DobPositionController(gains)
        self._result_callback = result_callback
        self._tolerance = tolerance
        self._hold_time = hold_time
        self._waypoints: list[Waypoint] = []
        self._index = 0
        self._arrival_started: float | None = None
        self._ticket = 0
        self._active = False
        self._completed = False
        manager.register(FlightMode.WAYPOINT, self.deactivate)

    def start(self, ticket: int, waypoints: Sequence[Waypoint]) -> None:
        """激活航点模式并复制任务数据，避免 GUI 后续修改影响执行。"""
        if not waypoints:
            raise ValueError("航点列表不能为空")
        normalized = [tuple(float(value) for value in waypoint) for waypoint in waypoints]
        if any(len(waypoint) != 4 for waypoint in normalized):
            raise ValueError("每个航点必须包含 x、y、z、yaw")

        self._manager.activate(FlightMode.WAYPOINT)
        with self._lock:
            self._ticket = ticket
            self._waypoints = normalized
            self._index = 0
            self._arrival_started = None
            self._active = True
            self._completed = False
            self._dob.reset()

    def deactivate(self) -> None:
        """被其他按键覆盖时立即取消当前航点任务。"""
        with self._lock:
            cancelled = self._active and not self._completed
            ticket = self._ticket
            self._active = False
            self._completed = False
            self._waypoints.clear()
            self._index = 0
            self._arrival_started = None
            self._dob.reset()
        if cancelled:
            self._result_callback(
                ticket, False, "航点任务已被其他飞行模式覆盖", True
            )

    def reset(self) -> None:
        """显式清空所有航点状态。"""
        self.deactivate()

    def publish(self, node, attitude_publisher, position_publisher,
                snapshot: VehicleSnapshot) -> None:
        """发布当前航点控制量，并在到达保持时间满足后推进任务。"""
        with self._lock:
            if not self._active or not self._waypoints:
                return
            target = self._waypoints[self._index]

        if not snapshot.armed or snapshot.autopilot_mode != "GUIDED":
            return
        self._dob.publish(node, attitude_publisher, position_publisher, snapshot, target)

        distance = math.sqrt(
            (snapshot.x - target[0]) ** 2
            + (snapshot.y - target[1]) ** 2
            + (snapshot.z - target[2]) ** 2
        )
        now = time.monotonic()
        with self._lock:
            if not self._active:
                return
            if distance >= self._tolerance:
                self._arrival_started = None
                return
            if self._arrival_started is None:
                self._arrival_started = now
                return
            if now - self._arrival_started < self._hold_time:
                return

            self._index += 1
            self._arrival_started = None
            if self._index >= len(self._waypoints):
                # 保持最后一个航点为输出目标，但任务只报告一次完成。
                self._index = len(self._waypoints) - 1
                if not self._completed:
                    self._completed = True
                    self._result_callback(
                        self._ticket,
                        True,
                        f"航点任务完成 — 已到达全部 {len(self._waypoints)} 个航点，悬停中",
                        True,
                    )
                return

            self._dob.reset()
            next_target = self._waypoints[self._index]
            self._result_callback(
                self._ticket,
                True,
                f"航点 {self._index + 1}/{len(self._waypoints)}: "
                f"前往 ({next_target[0]:.1f}, {next_target[1]:.1f}, {next_target[2]:.1f})",
                False,
            )
