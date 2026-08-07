"""实时结构化日志查看器，支持按源端等级和文本筛选。"""

from __future__ import annotations

import html

from PySide6.QtGui import QTextCursor
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
        self._events: list[LogEvent] = []
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

        level_label = QLabel("等级")
        level_label.setObjectName("mutedLabel")
        toolbar.addWidget(level_label)
        self.level_checks: dict[LogLevel, QCheckBox] = {}
        for level in LogLevel:
            checkbox = QCheckBox(level.label)
            checkbox.setObjectName(f"logLevel{level.label.title()}")
            checkbox.setChecked(True)
            checkbox.setToolTip(f"独立显示或隐藏 {level.label} 日志")
            checkbox.toggled.connect(self._rebuild)
            self.level_checks[level] = checkbox
            toolbar.addWidget(checkbox)

        self.search_input = QLineEdit()
        self.search_input.setObjectName("logSearchInput")
        self.search_input.setPlaceholderText("搜索来源或消息")
        self.search_input.setClearButtonEnabled(True)
        self.search_input.textChanged.connect(self._rebuild)
        toolbar.addWidget(self.search_input, 1)

        self.auto_scroll = QCheckBox("自动滚动")
        self.auto_scroll.setObjectName("logAutoScroll")
        self.auto_scroll.setChecked(True)
        self.auto_scroll.setToolTip("关闭后保留当前阅读位置，新日志不再强制滚到底部")
        toolbar.addWidget(self.auto_scroll)
        self.counter_label = QLabel("D 0 · I 0 · W 0 · E 0")
        self.counter_label.setObjectName("mutedLabel")
        toolbar.addWidget(self.counter_label)
        clear_button = QPushButton("清空显示")
        clear_button.setProperty("compact", True)
        clear_button.clicked.connect(self.clear_display)
        toolbar.addWidget(clear_button)
        root.addLayout(toolbar)

        self.viewer = QTextEdit()
        self.viewer.setObjectName("logViewer")
        self.viewer.setReadOnly(True)
        self.viewer.setAcceptRichText(True)
        self.viewer.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.viewer.document().setMaximumBlockCount(8000)
        root.addWidget(self.viewer, 1)

    @property
    def displayed_text(self) -> str:
        """返回当前筛选后的纯文本，供复制与自动化回归使用。"""
        return self.viewer.toPlainText()

    def poll(self) -> int:
        """读取新事件并仅追加符合当前筛选条件的日志行。"""
        incoming = self._event_log.events_after(self._last_sequence)
        if not incoming:
            return 0
        self._events.extend(incoming)
        self._last_sequence = max(event.sequence for event in incoming)
        for event in incoming:
            if self._matches(event):
                self._append_event(event)
        self._update_counts()
        return len(incoming)

    def clear_display(self) -> None:
        """隐藏当前历史但保留源日志，后续实时事件仍会继续显示。"""
        self.poll()
        self._hidden_before_sequence = self._last_sequence
        self.viewer.clear()

    def selected_levels(self) -> frozenset[LogLevel]:
        """返回当前独立勾选的日志等级集合。"""
        return frozenset(
            level
            for level, checkbox in self.level_checks.items()
            if checkbox.isChecked()
        )

    def _matches(self, event: LogEvent) -> bool:
        """按等级、搜索文本和显示清空边界判断事件可见性。"""
        if event.sequence <= self._hidden_before_sequence:
            return False
        if event.level not in self.selected_levels():
            return False
        needle = self.search_input.text().strip().casefold()
        if needle and needle not in f"{event.source} {event.message}".casefold():
            return False
        return True

    def _append_event(self, event: LogEvent) -> None:
        """以固定时间/等级/来源列追加一条已分级事件。"""
        timestamp = event.timestamp.strftime("%H:%M:%S.%f")[:-3]
        color = self._LEVEL_COLORS[event.level]
        source = html.escape(event.source)
        message = html.escape(event.message).replace("\n", "<br>")
        line = (
            f'<span style="color:#708090">{timestamp}</span> '
            f'<span style="color:{color};font-weight:700">'
            f'[{event.level.label:<5}]</span> '
            f'<span style="color:#355269">[{source}]</span> '
            f'<span style="color:{color}">{message}</span>'
        )
        # 关闭自动滚动时必须保留原滚动位置；setTextCursor(End) 会强制跳底。
        horizontal = self.viewer.horizontalScrollBar()
        vertical = self.viewer.verticalScrollBar()
        horizontal_position = horizontal.value()
        vertical_position = vertical.value()
        follow = self._force_scroll_on_append or self.auto_scroll.isChecked()

        cursor = self.viewer.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        if not self.viewer.document().isEmpty():
            cursor.insertBlock()
        cursor.insertHtml(line)
        if follow:
            self.viewer.setTextCursor(cursor)
            self._scroll_to_latest()
        else:
            vertical.setValue(vertical_position)
        horizontal.setValue(horizontal_position)

    def _rebuild(self, _unused: object = None) -> None:
        """筛选条件变化时从本地事件副本重建可见文本。"""
        self.viewer.clear()
        previous = self._force_scroll_on_append
        # 重建时始终贴底一次，避免空白视口；之后仍尊重自动滚动开关。
        self._force_scroll_on_append = True
        try:
            for event in self._events:
                if self._matches(event):
                    self._append_event(event)
        finally:
            self._force_scroll_on_append = previous
        if self.auto_scroll.isChecked():
            self._scroll_to_latest()

    def _scroll_to_latest(self) -> None:
        """只跟随最新垂直行，避免长消息遮住时间戳和等级列。"""
        vertical = self.viewer.verticalScrollBar()
        vertical.setValue(vertical.maximum())

    def _update_counts(self) -> None:
        """展示当前进程内各源端等级的累计事件数。"""
        counts = {level: 0 for level in LogLevel}
        for event in self._events:
            counts[event.level] += 1
        self.counter_label.setText(
            f"D {counts[LogLevel.DEBUG]} · I {counts[LogLevel.INFO]} · "
            f"W {counts[LogLevel.WARN]} · E {counts[LogLevel.ERROR]}"
        )
