"""结构化日志总线、持久化与内存保留回归测试。"""

from __future__ import annotations

from ground_station_core.event_log import EventLog, LogLevel


def test_event_log_preserves_source_levels_and_monotonic_sequence() -> None:
    """日志显示层之前必须已经具有来源、等级和单调序号。"""
    events = EventLog(max_events=8)
    first = events.debug("test", "detail")
    second = events.error("controller", "failure")

    assert first.level is LogLevel.DEBUG
    assert second.level is LogLevel.ERROR
    assert second.sequence == first.sequence + 1
    assert events.events_after(first.sequence) == (second,)


def test_event_log_file_survives_memory_eviction_and_new_session(tmp_path) -> None:
    """落盘历史跨会话追加，内存淘汰不能删除失败追溯记录。"""
    log_path = tmp_path / "ground" / "events.log"
    events = EventLog(max_events=1, log_path=log_path)
    first = events.info("upstream", "收到命令 03")
    events.error("upstream", "takeoff 阶段失败：GUIDED 被拒绝")
    assert len(events.snapshot()) == 1
    EventLog(log_path=log_path).warn("upstream", "拒绝命令 03：尚未连接")

    saved = log_path.read_text(encoding="utf-8")
    assert first.timestamp.isoformat(timespec="milliseconds") in saved
    assert "[INFO] [upstream] 收到命令 03" in saved
    assert "[ERROR] [upstream] takeoff 阶段失败：GUIDED 被拒绝" in saved
    assert "[WARN] [upstream] 拒绝命令 03：尚未连接" in saved


def test_event_log_file_error_keeps_in_memory_logging_available(tmp_path, capsys) -> None:
    """磁盘不可写时保留内存日志并明确提示，不阻断业务调用。"""
    blocked = tmp_path / "not-a-directory"
    blocked.write_text("occupied", encoding="utf-8")
    events = EventLog(log_path=blocked / "events.log")
    event = events.error("upstream", "巡检失败")
    assert events.snapshot() == (event,)
    assert "地面站日志落盘失败" in capsys.readouterr().err


def test_event_log_rotates_bounded_history(tmp_path, monkeypatch) -> None:
    """长期落盘超限后保留最新内容，历史文件数量受控。"""
    monkeypatch.setattr("ground_station_core.event_log._LOG_MAX_BYTES", 150)
    path = tmp_path / "events.log"
    events = EventLog(log_path=path)
    for index in range(20):
        events.info("test", f"巡检状态 {index:02d}")
    assert "巡检状态 19" in path.read_text()
    assert len(list(tmp_path.glob("events.log*"))) == 6
    assert "巡检状态 17" in (tmp_path / "events.log.1").read_text()
