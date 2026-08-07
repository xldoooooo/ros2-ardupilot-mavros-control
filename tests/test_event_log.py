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

    output_events = [event for event in events.snapshot() if event.source == process.name]
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
