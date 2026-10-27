"""结构化日志总线与受管进程实时分级输出回归测试。"""

from __future__ import annotations

from ground_station_core.event_log import EventLog, LogLevel
from ground_station_core.process_manager import ProcessSupervisor


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


def test_process_output_is_teed_to_file_and_structured_log() -> None:
    """外部进程输出应实时进入统一日志，并保留其显式 WARN 标记。"""
    events = EventLog()
    supervisor = ProcessSupervisor(events)
    process = supervisor.start(
        "pytest_log_probe",
        [
            "python3",
            "-u",
            "-c",
            "print('[WARN] source warning'); print('plain information')",
        ],
    )
    process.process.wait(timeout=3.0)
    assert process.output_thread is not None
    process.output_thread.join(timeout=2.0)
    report = supervisor.terminate_all()

    output_events = [
        event for event in events.snapshot() if event.source == process.name
    ]
    assert any(
        event.level is LogLevel.WARN and "source warning" in event.message
        for event in output_events
    )
    assert any(
        event.level is LogLevel.INFO and "plain information" in event.message
        for event in output_events
    )
    assert "plain information" in process.log_path.read_text(encoding="utf-8")
    assert report.success


def test_sitl_mavros_startup_noise_is_demoted_to_debug() -> None:
    """生产源名的 SITL/MAVROS 刷屏降级，显式告警仍保持高可见度。"""
    classify = ProcessSupervisor._explicit_output_level
    embedding = (
        "Embedding file default_params/quadplane.parm:"
        "Tools/autotest/default_params/quadplane.parm"
    )
    assert classify(embedding, "sitl") is LogLevel.DEBUG
    assert classify(embedding, "mavros") is LogLevel.DEBUG
    # EnvironmentInitializer 的真实受管进程名是 mavros_sim，不能只测别名。
    assert (
        classify(
            "[INFO] [1723000000.1] [mavros_node]: Starting mavros_node container",
            "mavros_sim",
        )
        is LogLevel.DEBUG
    )
    assert classify("[WARN] link down", "mavros_sim") is LogLevel.WARN
    assert classify("[ERROR] fcu crashed", "sitl") is LogLevel.ERROR
    # 非 chatty 源的普通输出仍为 INFO。
    assert classify("mission accepted", "operator") is LogLevel.INFO
