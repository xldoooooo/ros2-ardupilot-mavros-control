"""线程安全的结构化日志总线，支持可选文件落盘与源端等级标注。"""

from __future__ import annotations

import sys
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from pathlib import Path

# 长期无人值守时保留最多约 120 MiB 的地面事件历史。
_LOG_MAX_BYTES = 20 * 1024 * 1024
_LOG_BACKUP_COUNT = 5


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

    def __init__(
        self, max_events: int = 8000, *, log_path: str | Path | None = None,
    ) -> None:
        """创建有界内存缓冲；指定路径时额外追加落盘，不清除历史会话。"""
        if max_events <= 0:
            raise ValueError("max_events 必须大于零")
        self._lock = threading.RLock()
        self._sequence = 0
        self._events: deque[LogEvent] = deque(maxlen=max_events)
        self._log_path = Path(log_path).expanduser() if log_path is not None else None

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
            if self._log_path is not None:
                try:
                    self._log_path.parent.mkdir(parents=True, exist_ok=True)
                    line = (
                        f"{event.timestamp.isoformat(timespec='milliseconds')} "
                        f"[{event.level.label}] [{event.source}] {event.message}\n"
                    )
                    if self._log_path.exists() and (
                        self._log_path.stat().st_size + len(line.encode("utf-8"))
                        > _LOG_MAX_BYTES
                    ):
                        self._rotate_file()
                    with self._log_path.open("a", encoding="utf-8") as stream:
                        stream.write(line)
                except OSError as exc:
                    # 日志磁盘故障不阻断控制；避免后续事件反复报同一写入错误。
                    print(f"地面站日志落盘失败 {self._log_path}：{exc}", file=sys.stderr)
                    self._log_path = None
            return event

    def _rotate_file(self) -> None:
        """在现有写锁内轮转历史文件，保留最近五份完整日志。"""
        for index in range(_LOG_BACKUP_COUNT, 0, -1):
            source = (
                self._log_path if index == 1
                else Path(f"{self._log_path}.{index - 1}")
            )
            if source.exists():
                source.replace(Path(f"{self._log_path}.{index}"))

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
