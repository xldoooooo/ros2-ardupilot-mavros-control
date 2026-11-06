"""受管进程输出分类、日志保存及退出清理回归。"""

from __future__ import annotations

import sys

import pytest

from ground_station_core.event_log import EventLog, LogLevel
from ground_station_core.process_manager import ProcessSupervisor


@pytest.mark.integration
def test_process_output_is_teed_to_file_and_structured_log() -> None:
    """外部进程输出应实时进入统一日志，并保留其显式 WARN 标记。"""
    events = EventLog()
    supervisor = ProcessSupervisor(events)
    process = supervisor.start(
        "pytest_log_probe",
        [
            sys.executable,
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
