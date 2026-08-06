"""键盘控制模式：累加速度方向命令与 PD+DOB 定点悬停。"""

from __future__ import annotations

import threading

from ..dob_controller import DobGains, DobPositionController
from ..mode_manager import FlightModeManager
from ..models import FlightMode, VehicleSnapshot


class KeyboardControlMode:
    """维护键盘速度状态；任一方向键或悬停键都会覆盖当前模式。"""

    def __init__(self, manager: FlightModeManager, gains: DobGains) -> None:
        """注册键盘模式并初始化速度与悬停控制器。"""
        self._manager = manager
        self._lock = threading.RLock()
        self._dob = DobPositionController(gains)
        self._velocity = [0.0, 0.0, 0.0, 0.0]
        self._hover_target: tuple[float, float, float, float] | None = None
        self._enabled = False
        manager.register(FlightMode.KEYBOARD, self.deactivate)

    @property
    def velocity(self) -> tuple[float, float, float, float]:
        """返回当前累加速度 (vx, vy, vz, yaw_rate)。"""
        with self._lock:
            return tuple(self._velocity)

    @property
    def hovering(self) -> bool:
        """指示键盘模式当前是否使用定点悬停分支。"""
        with self._lock:
            return self._hover_target is not None

    def adjust(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """激活键盘模式并按原控制器语义累加一次速度。"""
        self._manager.activate(FlightMode.KEYBOARD)
        with self._lock:
            self._hover_target = None
            self._enabled = True
            increments = (vx, vy, vz, yaw_rate)
            for index, increment in enumerate(increments):
                self._velocity[index] += increment

    def hover(self, snapshot: VehicleSnapshot) -> None:
        """激活键盘模式，以当前位姿作为 PD+DOB 悬停目标。"""
        self._manager.activate(FlightMode.KEYBOARD)
        with self._lock:
            self._velocity[:] = (0.0, 0.0, 0.0, 0.0)
            self._hover_target = (snapshot.x, snapshot.y, snapshot.z, snapshot.yaw)
            self._enabled = True
            self._dob.reset()

    def deactivate(self) -> None:
        """被其他模式覆盖时清除速度，防止恢复后发送历史命令。"""
        with self._lock:
            self._velocity[:] = (0.0, 0.0, 0.0, 0.0)
            self._hover_target = None
            self._enabled = False
            self._dob.reset()

    def reset(self) -> None:
        """显式复位键盘控制状态。"""
        self.deactivate()

    def publish(self, node, velocity_publisher, attitude_publisher,
                position_publisher, snapshot: VehicleSnapshot) -> None:
        """按当前键盘子模式发布一次速度或悬停 setpoint。"""
        from geometry_msgs.msg import Twist

        with self._lock:
            if not self._enabled:
                return
            velocity = tuple(self._velocity)
            hover_target = self._hover_target

        if hover_target is not None:
            if snapshot.armed and snapshot.autopilot_mode == "GUIDED":
                self._dob.publish(
                    node,
                    attitude_publisher,
                    position_publisher,
                    snapshot,
                    hover_target,
                )
            return

        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = velocity[:3]
        message.angular.z = velocity[3]
        velocity_publisher.publish(message)
