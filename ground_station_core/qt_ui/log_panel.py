"""实时结构化日志查看器，支持按源端等级和文本筛选。"""

from __future__ import annotations

from collections import deque
import time

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..event_log import EventLog, LogEvent, LogLevel
from .theme import COLORS
from .log_history import SessionLogHistory


# 与原文本框容量一致；完整会话由后台归档保存供筛选。
_HISTORY_CACHE_SIZE = 8000
# 每次排版只占用少量事件循环时间，突发日志继续排队。
_RENDER_BUDGET_SECONDS = 0.004
_RENDER_BATCH_SIZE = 64
# 合并连续键入的搜索，不延迟日志接收或业务状态刷新。
_SEARCH_DELAY_MS = 100


class _HistoryBridge(QObject):
    """只把后台历史查询的纯数据送回 Qt 线程。"""

    completed = Signal(int, int, object, str)


class LogPanel(QFrame):
    """增量读取共享日志缓冲；筛选只影响显示，不更改原始事件。"""

    _LEVEL_COLORS = {
        LogLevel.DEBUG: COLORS["muted"],
        LogLevel.INFO: COLORS["text"],
        LogLevel.WARN: COLORS["warning"],
        LogLevel.ERROR: COLORS["danger"],
    }

    def __init__(self, event_log: EventLog, parent: QWidget | None = None) -> None:
        """创建工具栏、计数器和只读富文本日志框。"""
        super().__init__(parent)
        self.setObjectName("card")
        self.setMinimumHeight(150)
        self._event_log = event_log
        self._events: deque[LogEvent] = deque(maxlen=_HISTORY_CACHE_SIZE)
        self._counts = {level: 0 for level in LogLevel}
        self._history = SessionLogHistory()
        self.destroyed.connect(self._history.close)
        self._history_bridge = _HistoryBridge(self)
        self._history_bridge.completed.connect(self._history_completed)
        self._filter_generation = 0
        self._loading_history = False
        self._history_error = ""
        self._display_dirty = False
        self._pending: deque[LogEvent] = deque(maxlen=_HISTORY_CACHE_SIZE)
        self._query_tail: deque[LogEvent] = deque(maxlen=_HISTORY_CACHE_SIZE)
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._drain_pending)
        self._search_timer = QTimer(self)
        self._search_timer.setSingleShot(True)
        self._search_timer.setInterval(_SEARCH_DELAY_MS)
        self._search_timer.timeout.connect(self._rebuild)
        self._timestamp_format = self._format("#708090")
        self._source_format = self._format("#355269")
        self._message_formats = {
            level: self._format(color) for level, color in self._LEVEL_COLORS.items()
        }
        self._level_formats = {
            level: self._format(color, bold=True) for level, color in self._LEVEL_COLORS.items()
        }
        self._last_sequence = 0
        self._hidden_before_sequence = 0
        # 重建时强制跟随末尾，不受自动滚动开关影响。
        self._force_scroll_on_append = False

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(7)
        toolbar = QHBoxLayout()
        title = QLabel("实时日志")
        title.setObjectName("cardTitle")
        toolbar.addWidget(title)

        self.level_checks: dict[LogLevel, QCheckBox] = {}
        for level in LogLevel:
            checkbox = QCheckBox(level.label)
            checkbox.setObjectName(f"logLevel{level.label.title()}")
            checkbox.setChecked(level is not LogLevel.DEBUG)
            checkbox.setToolTip(f"独立显示或隐藏 {level.label} 日志")
            checkbox.toggled.connect(self._rebuild)
            self.level_checks[level] = checkbox
            toolbar.addWidget(checkbox)

        self.search_input = QLineEdit()
        self.search_input.setObjectName("logSearchInput")
        self.search_input.setProperty("compact", True)
        self.search_input.setPlaceholderText("搜索来源或消息")
        self.search_input.setClearButtonEnabled(True)
        self.search_input.textChanged.connect(self._schedule_search)
        toolbar.addWidget(self.search_input, 1)

        self.auto_scroll = QCheckBox("自动滚动")
        self.auto_scroll.setObjectName("logAutoScroll")
        self.auto_scroll.setChecked(True)
        self.auto_scroll.setToolTip("关闭后保留当前阅读位置，新日志不再强制滚到底部")
        toolbar.addWidget(self.auto_scroll)
        self.counter_label = QLabel("D 0 · I 0 · W 0 · E 0")
        self.counter_label.setObjectName("mutedLabel")
        toolbar.addWidget(self.counter_label)
        self.clear_button = QPushButton("清空显示")
        self.clear_button.setObjectName("clearLogDisplayButton")
        self.clear_button.setProperty("compact", True)
        self.clear_button.clicked.connect(self.clear_display)
        toolbar.addWidget(self.clear_button)
        root.addLayout(toolbar)

        self.viewer = QTextEdit()
        self.viewer.setObjectName("logViewer")
        self.viewer.setReadOnly(True)
        self.viewer.setAcceptRichText(True)
        self.viewer.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.viewer.document().setMaximumBlockCount(_HISTORY_CACHE_SIZE)
        root.addWidget(self.viewer, 1)

    @property
    def displayed_text(self) -> str:
        """返回当前筛选后的纯文本，供复制与自动化回归使用。"""
        return self.viewer.toPlainText()

    def poll(self) -> int:
        """增量收集与归档日志；隐藏时只更新数据，不构造富文本。"""
        incoming = self._event_log.events_after(self._last_sequence)
        if not incoming:
            return 0
        self._events.extend(incoming)
        self._history.append(incoming)
        self._last_sequence = incoming[-1].sequence
        for event in incoming:
            self._counts[event.level] += 1
        self._update_counts()
        if not self.isVisible() or self.window().isMinimized():
            self._display_dirty = True
            return len(incoming)
        key = self._filter_key()
        matching = (event for event in incoming if self._matches(event, key))
        if self._loading_history:
            self._query_tail.extend(matching)
        elif not self._search_timer.isActive():
            self._pending.extend(matching)
            self._drain_pending()
        return len(incoming)

    def clear_display(self) -> None:
        """隐藏已有记录，保留累计计数与可用归档，不影响源日志。"""
        self.poll()
        self._hidden_before_sequence = self._last_sequence
        self._filter_generation += 1
        self._loading_history = False
        self._search_timer.stop()
        self._render_timer.stop()
        self._pending.clear()
        self._query_tail.clear()
        self.viewer.clear()
        self._display_dirty = False
        self._force_scroll_on_append = False

    def selected_levels(self) -> frozenset[LogLevel]:
        """返回当前独立勾选的日志等级集合。"""
        return frozenset(
            level for level, checkbox in self.level_checks.items() if checkbox.isChecked()
        )

    def _filter_key(self) -> tuple:
        """每个批次只读取一次筛选控件，循环中只处理纯数据。"""
        return (self._hidden_before_sequence, self.selected_levels(),
                self.search_input.text().strip().casefold())

    @staticmethod
    def _matches(event: LogEvent, key: tuple) -> bool:
        """保留来源加消息的 Unicode 子串搜索和清空边界语义。"""
        hidden, levels, needle = key
        return (event.sequence > hidden and event.level in levels
                and (not needle or needle in f"{event.source} {event.message}".casefold()))

    @staticmethod
    def _format(color: str, *, bold: bool = False) -> QTextCharFormat:
        """缓存文字格式，避免逐条 HTML 解析或重复构造颜色。"""
        format_ = QTextCharFormat()
        format_.setForeground(QColor(color))
        format_.setFontWeight(QFont.Weight.Bold if bold else QFont.Weight.Normal)
        return format_

    def _drain_pending(self) -> None:
        """分批插入带格式的文本，每批仅排版和跟随滚动一次。"""
        self._render_timer.stop()
        if not self._pending:
            self._force_scroll_on_append = False
            return
        if not self.isVisible() or self.window().isMinimized():
            self._pending.clear()
            self._display_dirty = True
            return
        horizontal = self.viewer.horizontalScrollBar()
        vertical = self.viewer.verticalScrollBar()
        horizontal_position, vertical_position = horizontal.value(), vertical.value()
        follow = self._force_scroll_on_append or self.auto_scroll.isChecked()
        document = self.viewer.document()
        cursor = QTextCursor(document)
        cursor.movePosition(QTextCursor.MoveOperation.End)
        started = time.perf_counter()
        # 保留增量布局；反复关闭/开启 layout 会使整份历史重新排版。
        cursor.beginEditBlock()
        try:
            for _ in range(_RENDER_BATCH_SIZE):
                if not self._pending:
                    break
                event = self._pending.popleft()
                if not document.isEmpty():
                    cursor.insertBlock()
                timestamp = event.timestamp.strftime("%H:%M:%S.%f")[:-3]
                cursor.insertText(timestamp + " ", self._timestamp_format)
                cursor.insertText(f"[{event.level.label:<5}] ", self._level_formats[event.level])
                cursor.insertText(f"[{event.source}] ", self._source_format)
                # HTML 的 <br> 对应同段落换行；保留一条事件一个 block 的容量语义。
                cursor.insertText(event.message.replace("\n", "\u2028"), self._message_formats[event.level])
                if time.perf_counter() - started >= _RENDER_BUDGET_SECONDS:
                    break
        finally:
            cursor.endEditBlock()
        if follow:
            self.viewer.setTextCursor(cursor)
            self._scroll_to_latest()
        else:
            vertical.setValue(vertical_position)
        horizontal.setValue(horizontal_position)
        if self._pending:
            self._render_timer.start(0)
        else:
            self._force_scroll_on_append = False

    def _schedule_search(self, _text: str) -> None:
        """合并连续输入并立即废弃旧查询及尚未绘制的旧结果。"""
        self._filter_generation += 1
        self._loading_history = False
        self._pending.clear()
        self._query_tail.clear()
        self._render_timer.stop()
        self._search_timer.start()

    def _rebuild(self, _unused: object = None) -> None:
        """筛选当前会话；热缓存内直接匹配，更早历史交给后台归档。"""
        self._search_timer.stop()
        self._filter_generation += 1
        self._pending.clear()
        self._query_tail.clear()
        self._render_timer.stop()
        self._loading_history = False
        if not self.isVisible() or self.window().isMinimized():
            self._display_dirty = True
            return
        self._display_dirty = False
        key = self._filter_key()
        if sum(self._counts.values()) <= len(self._events):
            self._replace_display(tuple(event for event in self._events if self._matches(event, key)))
            return
        self._loading_history = True
        self._history.query(
            self._filter_generation, self._last_sequence, *key, _HISTORY_CACHE_SIZE,
            self._history_bridge.completed.emit,
        )

    def _history_completed(
        self, generation: int, sequence: int, events: object, error: str,
    ) -> None:
        """只接纳当前筛选结果，并补齐查询期间新到达的匹配日志。"""
        if generation != self._filter_generation:
            return
        self._loading_history = False
        if error and error != self._history_error:
            self._history_error = error
            self._event_log.warn("ui", error)
        if events is None:
            return
        tail = tuple(event for event in self._query_tail if event.sequence > sequence)
        self._query_tail.clear()
        self._replace_display((*events, *tail))

    def _replace_display(self, events: tuple[LogEvent, ...]) -> None:
        """替换筛选结果，随后分批排版且保持原有重建后贴底行为。"""
        self.viewer.clear()
        self._pending.extend(events)
        self._force_scroll_on_append = True
        self._drain_pending()

    def showEvent(self, event) -> None:  # noqa: N802
        """恢复显示时补齐隐藏期间的日志，不重建已保持同步的文本。"""
        super().showEvent(event)
        if self._display_dirty:
            self._rebuild()

    def _scroll_to_latest(self) -> None:
        """只跟随最新垂直行，长消息不强制横向滚动。"""
        vertical = self.viewer.verticalScrollBar()
        vertical.setValue(vertical.maximum())

    def _update_counts(self) -> None:
        """增量计数保留全部已收集事件，不再扫描整个会话历史。"""
        self.counter_label.setText(
            f"D {self._counts[LogLevel.DEBUG]} · I {self._counts[LogLevel.INFO]} · "
            f"W {self._counts[LogLevel.WARN]} · E {self._counts[LogLevel.ERROR]}"
        )
