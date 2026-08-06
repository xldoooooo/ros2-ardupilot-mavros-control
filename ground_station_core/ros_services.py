"""飞行模式共用的 MAVROS 服务等待与状态确认工具。"""

from __future__ import annotations

import time
from collections.abc import Callable

from .models import VehicleSnapshot


class RosServiceHelper:
    """在单个 rclpy 执行线程中可靠等待异步服务和状态回调。"""

    def __init__(
        self,
        node,
        snapshot_getter: Callable[[], VehicleSnapshot],
        running: Callable[[], bool],
    ) -> None:
        """绑定当前节点、状态读取器和生命周期判定函数。"""
        self._node = node
        self._snapshot_getter = snapshot_getter
        self._running = running

    @property
    def active(self) -> bool:
        """返回所属命令是否仍是有效的当前操作。"""
        return self._running()

    def wait_future(self, future, timeout: float, label: str) -> bool:
        """持续 spin 当前节点直至 future 完成、停止或超时。"""
        import rclpy

        deadline = time.monotonic() + timeout
        while self._running() and not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.02)
            if time.monotonic() >= deadline:
                print(f"[GS] {label} 超时 ({timeout:.1f}s)", flush=True)
                return False
        return future.done()

    def wait_state(
        self,
        predicate: Callable[[VehicleSnapshot], bool],
        timeout: float,
    ) -> bool:
        """等待 MAVROS 状态满足 predicate。"""
        import rclpy

        deadline = time.monotonic() + timeout
        while self._running() and time.monotonic() < deadline:
            if predicate(self._snapshot_getter()):
                return True
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return predicate(self._snapshot_getter())

    def ensure_guided(self, set_mode_client, timeout: float = 8.0) -> tuple[bool, str]:
        """切换到 GUIDED 并等待状态话题确认。"""
        from mavros_msgs.srv import SetMode

        if self._snapshot_getter().autopilot_mode == "GUIDED":
            return True, "已处于 GUIDED"
        if not set_mode_client.wait_for_service(timeout_sec=3.0):
            return False, "MAVROS 模式服务不可用"
        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        future = set_mode_client.call_async(request)
        if not self.wait_future(future, 5.0, "切换 GUIDED"):
            return False, "切换 GUIDED 超时"
        response = future.result()
        if response is None or not response.mode_sent:
            return False, "飞控拒绝 GUIDED 模式"
        if not self.wait_state(lambda state: state.autopilot_mode == "GUIDED", timeout):
            return False, "未收到 GUIDED 状态确认"
        return True, "GUIDED 已确认"

    def ensure_armed(self, arming_client, timeout: float = 40.0) -> tuple[bool, str]:
        """确保飞行器武装，并容忍 SITL 启动阶段短暂的 EKF/Home 检查。"""
        from mavros_msgs.srv import CommandBool

        if self._snapshot_getter().armed:
            return True, "飞行器已武装"
        if not arming_client.wait_for_service(timeout_sec=3.0):
            return False, "MAVROS 武装服务不可用"

        # ArduPilot 刚连接时常会短暂报告 Need Alt Estimate / waiting for home。
        # 用户已明确按下起飞键，因此在有限超时内重试，而不是降低 ARMING_CHECK。
        deadline = time.monotonic() + timeout
        attempt = 0
        while self._running() and time.monotonic() < deadline:
            attempt += 1
            request = CommandBool.Request()
            request.value = True
            future = arming_client.call_async(request)
            if not self.wait_future(future, 4.0, f"武装尝试 {attempt}"):
                return False, "武装服务超时"
            response = future.result()
            if response is not None and response.success:
                remaining = max(0.1, deadline - time.monotonic())
                if self.wait_state(lambda state: state.armed, min(5.0, remaining)):
                    return True, "武装已确认"
            if self.wait_state(lambda state: state.armed, 1.2):
                return True, "武装已确认"
        return False, f"飞控在 {attempt} 次尝试后仍拒绝武装（请检查 PreArm 信息）"
