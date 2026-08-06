"""起飞/降落模式：完成 GUIDED、武装、起飞与 LAND 服务序列。"""

from __future__ import annotations

import threading

from ..mode_manager import FlightModeManager
from ..models import FlightMode, VehicleSnapshot
from ..ros_services import RosServiceHelper


class TakeoffLandMode:
    """封装起降流程；起飞时不会再错误地先解除已武装飞行器。"""

    def __init__(self, manager: FlightModeManager) -> None:
        """注册起降模式并创建可跨线程设置的取消标志。"""
        self._manager = manager
        self._cancelled = threading.Event()
        manager.register(FlightMode.TAKEOFF_LAND, self.deactivate)

    def activate(self) -> None:
        """让起降模式覆盖当前模式，并清除本次操作的取消标志。"""
        self._cancelled.clear()
        self._manager.activate(FlightMode.TAKEOFF_LAND)

    def deactivate(self) -> None:
        """其他模式接管时通知尚未发出的起降步骤停止。"""
        self._cancelled.set()

    def execute_takeoff(
        self,
        altitude: float,
        helper: RosServiceHelper,
        snapshot_getter,
        takeoff_client,
        set_mode_client,
        arming_client,
    ) -> tuple[bool, str]:
        """执行完整起飞服务序列并等待达到合理离地高度。"""
        from mavros_msgs.srv import CommandTOL

        if altitude <= 0.0:
            return False, "起飞高度必须大于 0"
        if not snapshot_getter().connected:
            return False, "飞控未连接，请先初始化仿真或实机环境"

        ok, message = helper.ensure_guided(set_mode_client)
        if not ok or self._cancelled.is_set() or not helper.active:
            return (
                False,
                "起飞已被后续飞行模式按键覆盖"
                if self._cancelled.is_set() or not helper.active
                else message,
            )
        ok, message = helper.ensure_armed(arming_client)
        if not ok or self._cancelled.is_set() or not helper.active:
            return (
                False,
                "起飞已被后续飞行模式按键覆盖"
                if self._cancelled.is_set() or not helper.active
                else message,
            )

        if not takeoff_client.wait_for_service(timeout_sec=3.0):
            return False, "MAVROS 起飞服务不可用"
        request = CommandTOL.Request()
        request.altitude = float(altitude)
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0

        accepted = False
        for attempt in range(1, 4):
            if self._cancelled.is_set() or not helper.active:
                return False, "起飞已被后续飞行模式按键覆盖"
            future = takeoff_client.call_async(request)
            if helper.wait_future(future, 5.0, f"起飞尝试 {attempt}"):
                response = future.result()
                if response is not None and response.success:
                    accepted = True
                    break
        if not accepted:
            return False, "飞控连续 3 次拒绝起飞指令"

        # 低高度任务至少确认 0.15m；更高任务确认达到目标的 80%（预留制动误差）。
        threshold = max(0.15, altitude * 0.8)
        reached = helper.wait_state(
            lambda state: state.z >= threshold and state.armed,
            timeout=45.0,
        )
        if not reached:
            if self._cancelled.is_set() or not helper.active:
                return False, "起飞指令已接受，但控制已被后续飞行模式按键覆盖"
            state: VehicleSnapshot = snapshot_getter()
            return False, f"起飞指令已接受，但高度确认超时 (当前 {state.z:.2f}m)"
        return True, f"起飞成功 — 当前高度 {snapshot_getter().z:.2f}m"

    def execute_land(self, helper: RosServiceHelper, set_mode_client) -> tuple[bool, str]:
        """切换 LAND 模式；由飞控继续完成下降与上锁。"""
        from mavros_msgs.srv import SetMode

        if not set_mode_client.wait_for_service(timeout_sec=3.0):
            return False, "MAVROS 模式服务不可用"
        request = SetMode.Request()
        request.custom_mode = "LAND"
        future = set_mode_client.call_async(request)
        if not helper.wait_future(future, 5.0, "降落"):
            return False, "降落指令超时"
        response = future.result()
        if response is None or not response.mode_sent:
            return False, "飞控拒绝 LAND 模式"
        return True, "降落指令已发送 — LAND 模式"
