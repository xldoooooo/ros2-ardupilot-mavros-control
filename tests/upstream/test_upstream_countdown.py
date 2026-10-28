"""上位机无人值守倒计时闭环；实机模式只使用 Qt 后端替身。"""

from __future__ import annotations

import time
from dataclasses import replace

import pytest
from PySide6.QtCore import QPoint, QTimer, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QFrame, QLabel, QMessageBox

from ground_station_core.models import (
    CommandResult,
    WaypointReferenceGenerator,
    WaypointTrackingController,
)
from ground_station_core.upstream.mapping import parse_command
from tests.support.qt import _close_window, _operational_snapshot, _window


def _hardware_window(*, armed: bool):
    """仅切换替身窗口的门控模式，绝不创建真实 ROS 或硬件连接。"""
    window, ros = _window(_operational_snapshot(armed=armed))
    window._environment_active = True
    window._connection_mode = "hardware"
    window._refresh()
    return window, ros


def _command(number: str):
    """构造经过实际协议解析的动作命令。"""
    return parse_command({"clientNo": "UAV01001", "commandNo": number}, "UAV01001")


def test_hardware_inspection_completes_all_countdowns_without_confirmation(monkeypatch) -> None:
    """03 的三个阶段均等待真实三秒并自动提交，重复命令不重入。"""
    window, ros = _hardware_window(armed=False)
    observations = []
    elapsed = []
    create_dialog = window._message_box

    def record_dialog(*args, **kwargs):
        """记录实际按钮、3/2/1 文本及下发前的后端调用数。"""
        dialog = create_dialog(*args, **kwargs)
        started = time.monotonic()
        call_count = len(ros.calls)
        observed_text = []

        def observe() -> None:
            """在真正的模态事件循环中读取倒计时视图。"""
            text = dialog.informativeText()
            if observed_text and observed_text[-1] == text:
                return
            observed_text.append(text)
            title = dialog.findChild(QFrame, "dialogTitleBar")
            body = dialog.findChild(QLabel, "qt_msgbox_label")
            observations.append((
                dialog.standardButtons(), text,
                len(ros.calls), call_count,
                title.mapTo(dialog, QPoint()).y() + title.height()
                <= body.mapTo(dialog, QPoint()).y(),
            ))

        # 观察实际文字变化，避免其他回归负载推迟 Qt 调度时误判秒数。
        observer = QTimer(dialog)
        observer.timeout.connect(observe)
        dialog.finished.connect(observer.stop)
        observer.start(50)
        dialog.finished.connect(lambda _result: elapsed.append(time.monotonic() - started))
        return dialog

    monkeypatch.setattr(window, "_message_box", record_dialog)
    try:
        staged = parse_command({
            "clientNo": "UAV01001", "commandNo": "02",
            "taskPoints": [{
                "index": 8, "x": 1, "y": 2, "z": 1.5,
                "forwardAngle": 90, "cameraAngle": 1495, "photoNo": "photo-8",
            }],
        }, "UAV01001")
        window._handle_upstream_command(staged)
        QTimer.singleShot(150, lambda: window._handle_upstream_command(_command("03")))
        window._handle_upstream_command(_command("03"))
        assert [name for name, _args in ros.calls] == ["takeoff"]
        assert window._upstream_sequence.ticket == 1

        ros.results.append(CommandResult(1, 1, "takeoff", True, "起飞完成"))
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        assert [name for name, _args in ros.calls] == ["takeoff", "waypoints"]
        assert ros.calls[-1][1][4] == ("photo-8",)
        assert ("mission", (2, "inspection", (8,))) in window._upstream.calls

        ros.results.append(CommandResult(2, 2, "waypoints", True, "巡检完成"))
        window._refresh()
        assert [name for name, _args in ros.calls] == ["takeoff", "waypoints", "land"]
        assert ("landing", 3) in window._upstream.calls

        ros.results.append(CommandResult(3, 3, "land", True, "降落完成"))
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence.phase == "standby"
        assert window._upstream_sequence.standby_not_before > time.monotonic()
        window._upstream_sequence.standby_not_before = 0.0
        window._refresh()
        assert window._upstream_sequence is None
        assert window._upstream_countdown is None
        assert len(elapsed) == 3 and all(duration >= 3.0 for duration in elapsed)
        assert len(observations) == 9
        for index, (buttons, text, count, before, layout_ok) in enumerate(observations):
            assert buttons == QMessageBox.StandardButton.Cancel
            assert f"{3 - index % 3} 秒后自动执行" in text
            assert count == before
            assert layout_ok
    finally:
        _close_window(window)


def test_hardware_upstream_takeoff_return_and_land_commands_run_unattended() -> None:
    """连续执行 01/05/06/07，05 与 07 共用紧急降落入口。"""
    window, ros = _hardware_window(armed=False)
    try:
        window._handle_upstream_command(_command("01"))
        assert ros.calls[-1][0] == "takeoff"
        ros.results.append(CommandResult(1, 1, "takeoff", True, "起飞完成"))
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()

        window._handle_upstream_command(_command("05"))
        assert ros.calls[-1][0] == "land"
        assert window._upstream_sequence.kind == "emergency"
        ros.results.append(CommandResult(2, 2, "land", True, "降落完成"))
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence is None

        for number in ("06", "07"):
            ros.current_snapshot = _operational_snapshot(armed=True)
            window._handle_upstream_command(_command(number))
            assert ros.calls[-1][0] == "land"
        assert [name for name, _args in ros.calls] == [
            "takeoff", "land", "land", "land",
        ]
        assert window._upstream_sequence.kind == "emergency"
    finally:
        _close_window(window)


def test_upstream_experimental_waypoint_warning_also_counts_down() -> None:
    """实验组合警告与实机航点确认均自动结束，保留原有配置。"""
    window, ros = _hardware_window(armed=False)
    try:
        window.waypoints.reference_combo.setCurrentIndex(
            WaypointReferenceGenerator.JERK_LIMITED_S_CURVE.value
        )
        window.waypoints.tracking_combo.setCurrentIndex(
            WaypointTrackingController.POSITION_PD_DOB.value
        )
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        started = time.monotonic()
        window._start_return_sequence("return", ros.current_snapshot)
        assert time.monotonic() - started >= 6.0
        assert [name for name, _args in ros.calls] == ["waypoints"]
        assert ros.calls[0][1][2:4] == (
            WaypointReferenceGenerator.JERK_LIMITED_S_CURVE,
            WaypointTrackingController.POSITION_PD_DOB,
        )
    finally:
        _close_window(window)


@pytest.mark.parametrize("cancel_kind", ["button", "escape", "close", "lost_authority"])
def test_cancelled_inspection_countdown_never_sends_or_retries(cancel_kind) -> None:
    """取消、Esc、关闭及控制权丢失均终止组合，原定时器不会补发。"""
    window, ros = _hardware_window(armed=False)

    def cancel() -> None:
        """在倒计时尚未结束时操作真实 Qt 消息框或权威快照。"""
        dialog = window._upstream_countdown
        if cancel_kind == "button":
            dialog.button(QMessageBox.StandardButton.Cancel).click()
        elif cancel_kind == "escape":
            QTest.keyClick(dialog, Qt.Key.Key_Escape)
        elif cancel_kind == "close":
            dialog.close()
        else:
            ros.current_snapshot = replace(ros.current_snapshot, control_authority=False)

    try:
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "倒计时取消测试")
        QTimer.singleShot(150, cancel)
        window._handle_upstream_command(_command("03"))
        assert window._upstream_countdown is None
        assert window._upstream_sequence is None
        QTest.qWait(3100)
        assert not ros.calls
    finally:
        _close_window(window)


@pytest.mark.parametrize("number", ["05", "07"])
def test_emergency_land_preempts_countdown_without_resuming_old_land(number) -> None:
    """05/07 可打断普通降落倒计时，旧调用不得覆盖新紧急降落。"""
    window, ros = _hardware_window(armed=True)
    try:
        QTimer.singleShot(150, lambda: window._handle_upstream_command(_command(number)))
        window._handle_upstream_command(_command("06"))
        assert ros.calls == [("land", None)]
        assert window._upstream_sequence.kind == "emergency"
        assert window._upstream_sequence.phase == "land"
        assert window._upstream_sequence.ticket == 1
        assert window._upstream_countdown is None
        assert ("landing", 1) in window._upstream.calls
    finally:
        _close_window(window)


def test_land_countdown_does_not_advance_an_old_inspection_result() -> None:
    """降落倒计时内收到旧起飞成功时，不得嵌套提交巡检航点。"""
    window, ros = _hardware_window(armed=False)

    def finish_takeoff() -> None:
        """模拟已发出的旧起飞在降落倒计时期间返回可靠终态。"""
        ros.results.append(CommandResult(1, 1, "takeoff", True, "起飞完成"))
        window._refresh()

    try:
        window._connection_mode = "simulation"
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "降落抢占测试")
        window._handle_upstream_command(_command("03"))
        window._connection_mode = "hardware"
        ros.current_snapshot = _operational_snapshot(armed=True)
        QTimer.singleShot(150, finish_takeoff)
        window._handle_upstream_command(_command("06"))
        assert [name for name, _args in ros.calls] == ["takeoff", "land"]
        assert window._upstream_sequence is None
        assert window._upstream_countdown is None
        assert ("landing", 2) in window._upstream.calls
    finally:
        _close_window(window)
