"""线程安全的结构化日志总线，供后端在事件产生时同步标注等级。"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum


class LogLevel(IntEnum):
    """地面站统一使用的四级日志严重度。"""

    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40

    @property
    def label(self) -> str:
        """返回固定宽度的英文等级标签，便于日志面板扫描。"""
        return self.name


@dataclass(frozen=True)
class LogEvent:
    """一条不可变日志事件；等级由产生该事件的模块明确给出。"""

    sequence: int
    timestamp: datetime
    level: LogLevel
    source: str
    message: str


class EventLog:
    """保存有界日志历史，并支持 GUI 按单调序号增量读取。"""

    def __init__(self, max_events: int = 8000) -> None:
        """创建日志缓冲；上限防止长时间仿真无限占用内存。"""
        if max_events <= 0:
            raise ValueError("max_events 必须大于零")
        self._lock = threading.RLock()
        self._sequence = 0
        self._events: deque[LogEvent] = deque(maxlen=max_events)

    def emit(self, level: LogLevel, source: str, message: str) -> LogEvent:
        """原子写入一条已分级事件，并返回最终事件对象。"""
        if not isinstance(level, LogLevel):
            level = LogLevel(level)
        clean_source = str(source).strip() or "ground-station"
        clean_message = str(message).rstrip() or "--"
        with self._lock:
            self._sequence += 1
            event = LogEvent(
                sequence=self._sequence,
                timestamp=datetime.now().astimezone(),
                level=level,
                source=clean_source,
                message=clean_message,
            )
            self._events.append(event)
            return event

    def debug(self, source: str, message: str) -> LogEvent:
        """记录调试事件。"""
        return self.emit(LogLevel.DEBUG, source, message)

    def info(self, source: str, message: str) -> LogEvent:
        """记录正常运行事件。"""
        return self.emit(LogLevel.INFO, source, message)

    def warn(self, source: str, message: str) -> LogEvent:
        """记录需要操作者关注但尚可继续的事件。"""
        return self.emit(LogLevel.WARN, source, message)

    def error(self, source: str, message: str) -> LogEvent:
        """记录失败或安全相关异常事件。"""
        return self.emit(LogLevel.ERROR, source, message)

    def events_after(self, sequence: int) -> tuple[LogEvent, ...]:
        """返回指定序号之后仍保留在有界缓冲中的全部事件。"""
        with self._lock:
            return tuple(event for event in self._events if event.sequence > sequence)

    def snapshot(self) -> tuple[LogEvent, ...]:
        """返回当前完整日志历史的不可变副本。"""
        with self._lock:
            return tuple(self._events)
