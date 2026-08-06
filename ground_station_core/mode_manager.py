"""线程安全的飞行模式仲裁器，保证任一时刻只有一个模式接管输出。"""

from __future__ import annotations

import threading
from collections.abc import Callable

from .models import FlightMode


class FlightModeManager:
    """集中处理模式覆盖，并通知被覆盖模式释放内部状态。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current = FlightMode.IDLE
        self._deactivate_callbacks: dict[FlightMode, Callable[[], None]] = {}
        self._listeners: list[Callable[[FlightMode, FlightMode], None]] = []

    @property
    def current(self) -> FlightMode:
        """返回当前接管控制输出的模式。"""
        with self._lock:
            return self._current

    def register(
        self, mode: FlightMode, deactivate_callback: Callable[[], None]
    ) -> None:
        """注册模式退出时的清理回调。"""
        if mode is FlightMode.IDLE:
            raise ValueError("IDLE 模式不接受退出回调")
        with self._lock:
            self._deactivate_callbacks[mode] = deactivate_callback

    def add_listener(
        self, listener: Callable[[FlightMode, FlightMode], None]
    ) -> None:
        """注册模式变更监听器。"""
        with self._lock:
            self._listeners.append(listener)

    def activate(self, mode: FlightMode) -> FlightMode:
        """让指定模式覆盖当前模式，并返回先前模式。"""
        if not isinstance(mode, FlightMode):
            raise TypeError("mode 必须为 FlightMode")

        with self._lock:
            previous = self._current
            if previous is mode:
                return previous
            self._current = mode
            deactivate = self._deactivate_callbacks.get(previous)
            listeners = tuple(self._listeners)

        # 回调在锁外执行，避免模式内部清理再次访问仲裁器时死锁。
        if deactivate is not None:
            deactivate()
        for listener in listeners:
            listener(previous, mode)
        return previous

    def clear(self) -> FlightMode:
        """释放当前模式并切换为待机。"""
        return self.activate(FlightMode.IDLE)
