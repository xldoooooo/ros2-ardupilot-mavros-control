"""日志分批绘制、异步会话检索及隐藏恢复的交互边界回归。"""

from __future__ import annotations

import time

from PySide6.QtCore import QEvent, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ground_station_core.event_log import EventLog, LogLevel
from ground_station_core.qt_ui.log_panel import LogPanel
from tests.support.qt import _application


def _wait_for(predicate) -> None:
    """等待后台查询和 Qt 排版完成，不依赖固定的磁盘或桌面速度。"""
    deadline = time.monotonic() + 5.0
    while not predicate() and time.monotonic() < deadline:
        QTest.qWait(5)
    assert predicate()


def _dispose(panel: LogPanel) -> None:
    """销毁组件并确认会话归档线程自然收尾。"""
    history = panel._history
    panel.close()
    panel.deleteLater()
    QApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    history._thread.join(timeout=3.0)
    assert not history._thread.is_alive()


def test_evicted_hidden_logs_remain_searchable_with_unicode_and_counts(monkeypatch) -> None:
    """热缓存淘汰、隐藏与清空均不丢会话检索或累计计数。"""
    monkeypatch.setattr("ground_station_core.qt_ui.log_panel._HISTORY_CACHE_SIZE", 20)
    _application()
    events = EventLog(max_events=8)
    panel = LogPanel(events)
    panel.show()
    first = events.info("Straße", "old <value> & line\ncontinued")
    panel.poll()
    panel.hide()
    for index in range(40):
        events.debug("worker", f"detail {index}")
        panel.poll()
    assert len(panel._events) == 20
    assert "I 1" in panel.counter_label.text()
    assert "D 40" in panel.counter_label.text()
    panel.search_input.setText("STRASSE")
    panel.show()
    try:
        _wait_for(lambda: "continued" in panel.displayed_text)
        assert "old <value> & line\ncontinued" in panel.displayed_text
        assert panel.viewer.document().blockCount() == 1
        panel.clear_display()
        panel._history_completed(panel._filter_generation - 1, first.sequence, (first,), "")
        assert panel.displayed_text == ""
        events.info("Straße", "new visible record")
        panel.poll()
        assert "new visible record" in panel.displayed_text
        assert "old <value>" not in panel.displayed_text
    finally:
        _dispose(panel)


def test_old_query_cannot_replace_current_filter_or_drop_live_tail(monkeypatch) -> None:
    """新筛选废弃旧结果，查询期间到达的记录按序且只显示一次。"""
    monkeypatch.setattr("ground_station_core.qt_ui.log_panel._HISTORY_CACHE_SIZE", 4)
    _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.show()
    for index in range(6):
        events.info("source", f"seed {index}")
    panel.poll()
    pending = []
    monkeypatch.setattr(panel._history, "query", lambda *args: pending.append(args))
    try:
        panel.search_input.setText("seed")
        panel._rebuild()
        old_generation = panel._filter_generation
        panel.search_input.setText("new")
        panel._rebuild()
        generation = panel._filter_generation
        boundary = panel._last_sequence
        live = events.info("source", "new tail")
        panel.poll()
        panel._history_completed(old_generation, boundary, (events.snapshot()[0],), "")
        panel._history_completed(generation, boundary, (), "")
        assert panel.displayed_text.count("new tail") == 1
        assert "seed" not in panel.displayed_text
        assert panel._last_sequence == live.sequence
    finally:
        _dispose(panel)


def test_large_render_yields_and_preserves_manual_scroll_and_text_selection() -> None:
    """大批日志可分片，关闭跟随后仍保留阅读位置与文本选择。"""
    app = _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.resize(900, 240)
    panel.show()
    assert QTest.qWaitForWindowExposed(panel)
    app.processEvents()
    try:
        for index in range(1600):
            events.info("worker", f"message {index:04d} " + "x" * 100)
        panel.poll()
        assert panel._pending
        heartbeat = []
        QTimer.singleShot(0, lambda: heartbeat.append(True))
        _wait_for(lambda: bool(heartbeat) and not panel._pending
                  and panel.viewer.document().blockCount() == 1600)
        assert panel.viewer.document().blockCount() == 1600
        panel.auto_scroll.setChecked(False)
        panel.viewer.verticalScrollBar().setValue(0)
        cursor = panel.viewer.textCursor()
        cursor.setPosition(0)
        cursor.setPosition(5, cursor.MoveMode.KeepAnchor)
        panel.viewer.setTextCursor(cursor)
        selected = panel.viewer.textCursor().selectedText()
        events.warn("worker", "new record while reading")
        panel.poll()
        app.processEvents()
        assert panel.viewer.verticalScrollBar().value() <= 2
        assert panel.viewer.textCursor().selectedText() == selected
        assert "new record while reading" in panel.displayed_text
    finally:
        _dispose(panel)


def test_archive_storage_error_preserves_search_with_explicit_warning(monkeypatch, capsys) -> None:
    """临时磁盘不可用时保留原有内存检索能力并明确告警。"""
    monkeypatch.setattr("ground_station_core.qt_ui.log_panel._HISTORY_CACHE_SIZE", 2)

    def unavailable(**_kwargs):
        """模拟归档目录不可写，不影响正常日志总线。"""
        raise OSError("test storage unavailable")

    monkeypatch.setattr("ground_station_core.qt_ui.log_history.tempfile.TemporaryDirectory", unavailable)
    _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.show()
    for index in range(4):
        events.info("source", f"record {index}")
        panel.poll()
    try:
        panel.search_input.setText("record 0")
        panel._rebuild()
        _wait_for(lambda: "record 0" in panel.displayed_text)
        assert any("回退内存历史" in event.message for event in events.snapshot())
        assert "test storage unavailable" in capsys.readouterr().err
    finally:
        _dispose(panel)
