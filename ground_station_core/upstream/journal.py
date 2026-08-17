"""上位机通讯面板专用的有界原始报文日志。"""

from __future__ import annotations

import threading
from collections import deque
from datetime import datetime

from .models import RawFrame


class RawFrameJournal:
    """线程安全保存原始收发帧，不写入地面站人类维护日志。"""

    def __init__(self, max_frames: int = 2000) -> None:
        if max_frames <= 0:
            raise ValueError("max_frames 必须大于零")
        self._lock = threading.RLock()
        self._sequence = 0
        self._frames: deque[RawFrame] = deque(maxlen=max_frames)

    def append(self, direction: str, payload: str) -> RawFrame:
        """原子保存一条未经格式化的 WebSocket 文本。"""
        with self._lock:
            self._sequence += 1
            frame = RawFrame(
                self._sequence,
                datetime.now().astimezone(),
                str(direction),
                str(payload),
            )
            self._frames.append(frame)
            return frame

    def frames_after(self, sequence: int) -> tuple[RawFrame, ...]:
        """按单调序号返回面板尚未读取的帧。"""
        with self._lock:
            return tuple(frame for frame in self._frames if frame.sequence > sequence)

    def snapshot(self) -> tuple[RawFrame, ...]:
        """返回当前有界历史的不可变副本。"""
        with self._lock:
            return tuple(self._frames)

    def clear(self) -> None:
        """只清空面板日志；序号继续单调递增。"""
        with self._lock:
            self._frames.clear()
