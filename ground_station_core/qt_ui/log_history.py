"""后台保存当前 GUI 会话日志，提供有界内存之外的完整筛选历史。"""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import queue
import sqlite3
import sys
import tempfile
import threading

from ..event_log import LogEvent, LogLevel


class SessionLogHistory:
    """串行归档与查询；Qt 线程只投递不可变事件，不等待文件或 SQL。"""

    def __init__(self) -> None:
        """创建会话工作线程，临时归档在组件销毁后自动回收。"""
        self._requests: queue.SimpleQueue = queue.SimpleQueue()
        self._closed = False
        self._latest_generation = 0
        self._thread = threading.Thread(
            target=self._run, name="gui-log-history", daemon=True,
        )
        self._thread.start()

    def append(self, events: tuple[LogEvent, ...]) -> None:
        """按 GUI 消费顺序归档，查询始终排在之前的写入之后。"""
        if not self._closed:
            self._requests.put(("append", events))

    def query(
        self, generation: int, sequence: int, hidden_before: int,
        levels: frozenset[LogLevel], needle: str, limit: int,
        completed: Callable,
    ) -> None:
        """异步读取最后一页匹配历史，保留 Unicode casefold 搜索语义。"""
        if not self._closed:
            self._latest_generation = generation
            self._requests.put((
                "query", generation, sequence, hidden_before,
                levels, needle, limit, completed,
            ))

    def close(self, _unused: object = None) -> None:
        """请求串行结束，绝不在 GUI 线程 join 或等待磁盘。"""
        if not self._closed:
            self._closed = True
            self._requests.put(None)

    def _run(self) -> None:
        """独占 SQLite 连接；归档故障时回退内存以保留会话检索。"""
        temporary = None
        connection = None
        fallback: list[LogEvent] = []
        error = ""
        try:
            try:
                temporary = tempfile.TemporaryDirectory(prefix="ground-station-log-")
                connection = sqlite3.connect(f"{temporary.name}/history.sqlite")
                # 会话临时副本无需崩溃恢复，正式轮转日志仍由 EventLog 保存。
                connection.execute("PRAGMA journal_mode=MEMORY")
                connection.execute("PRAGMA synchronous=OFF")
                connection.execute(
                    "CREATE TABLE events (sequence INTEGER PRIMARY KEY, "
                    "timestamp TEXT, level INTEGER, source TEXT, message TEXT, search TEXT)"
                )
            except (OSError, sqlite3.Error) as exc:
                error = f"GUI 日志归档不可用，回退内存历史：{exc}"
                print(error, file=sys.stderr)
                if connection is not None:
                    connection.close()
                connection = None

            while True:
                request = self._requests.get()
                if request is None:
                    break
                if request[0] == "append":
                    events = request[1]
                    if connection is None:
                        fallback.extend(events)
                        continue
                    try:
                        connection.executemany(
                            "INSERT INTO events VALUES (?, ?, ?, ?, ?, ?)",
                            ((event.sequence, event.timestamp.isoformat(), int(event.level),
                              event.source, event.message,
                              f"{event.source} {event.message}".casefold()) for event in events),
                        )
                        connection.commit()
                    except sqlite3.Error as exc:
                        # 已落盘记录仍先读回，避免归档失败导致旧历史静默消失。
                        try:
                            connection.rollback()
                            rows = connection.execute(
                                "SELECT sequence,timestamp,level,source,message FROM events "
                                "ORDER BY sequence"
                            ).fetchall()
                        except sqlite3.Error:
                            rows = []
                            print("GUI 旧日志归档无法读取；正式日志文件仍由 EventLog 保留", file=sys.stderr)
                        fallback = [self._event(row) for row in rows]
                        last = fallback[-1].sequence if fallback else 0
                        fallback.extend(event for event in events if event.sequence > last)
                        connection.close()
                        connection = None
                        error = f"GUI 日志归档失败，回退内存历史：{exc}"
                        print(error, file=sys.stderr)
                    continue

                _, generation, sequence, hidden, levels, needle, limit, completed = request
                if generation != self._latest_generation:
                    continue
                try:
                    if connection is None:
                        events = tuple(event for event in fallback
                                       if hidden < event.sequence <= sequence
                                       and event.level in levels
                                       and (not needle or needle in f"{event.source} {event.message}".casefold()))[-limit:]
                    elif not levels:
                        events = ()
                    else:
                        placeholders = ",".join("?" for _ in levels)
                        rows = connection.execute(
                            "SELECT sequence,timestamp,level,source,message FROM events "
                            f"WHERE sequence>? AND sequence<=? AND level IN ({placeholders}) "
                            "AND instr(search,?)>0 ORDER BY sequence DESC LIMIT ?",
                            (hidden, sequence, *(int(level) for level in levels), needle, limit),
                        ).fetchall()
                        events = tuple(self._event(row) for row in reversed(rows))
                    completed(generation, sequence, events, error)
                except (RuntimeError, sqlite3.Error) as exc:
                    # Qt 接收组件可能已经销毁；工作线程仍须能收到关闭哨兵。
                    if isinstance(exc, sqlite3.Error):
                        try:
                            completed(generation, sequence, None, f"GUI 日志查询失败：{exc}")
                        except RuntimeError:
                            pass
        finally:
            if connection is not None:
                connection.close()
            if temporary is not None:
                temporary.cleanup()

    @staticmethod
    def _event(row: tuple) -> LogEvent:
        """重建查询页内的不可变事件，避免长期保留全部 Python 对象。"""
        sequence, timestamp, level, source, message = row
        return LogEvent(sequence, datetime.fromisoformat(timestamp), LogLevel(level), source, message)
