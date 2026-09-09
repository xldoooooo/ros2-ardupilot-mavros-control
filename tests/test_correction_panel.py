"""独立 Tag-Odin 2.0 面板的滑窗门控、展示、确认和生命周期测试。"""

from __future__ import annotations

import os
from typing import Any

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QMessageBox

from correction_service.correction_panel import CorrectionPanelWindow


class _FakeCorrectionClient:
    """不创建 DDS participant 的 2.0 面板客户端替身。"""

    def __init__(self) -> None:
        self.started = 0
        self.closed = 0
        self.start_requests: list[tuple[int, int, int, bool, str, int, int]] = []
        self.stop_requests: list[tuple[str, str]] = []
        self.clear_requests: list[tuple[str, int]] = []
        self.apply_saved_requests: list[tuple[str, int, int]] = []
        self.current_status: dict[str, Any] = {
            "correction": {
                "fresh": True,
                "interface_version": "2.0",
                "service_available": True,
                "service_instance_id": "instance-test",
                "window_revision": 0,
                "window_size": 5,
                "window_count": 0,
                "success_count": 0,
                "next_calibration": 1,
                "window_state": "empty",
                "window_message": "窗口为空",
                "keyframes": [],
                "active": False,
                "state": "idle",
                "operation": "none",
                "message": "服务空闲；下视相机关闭",
                "application_state": "none",
                "candidate_saved": False,
                "candidate_valid": False,
                "candidate_stale": False,
            },
            "extnav": {
                "fresh": True,
                "interface_version": "2.0",
                "service_available": True,
                "odin_available": True,
                "valid": False,
                "session": "odin-test",
                "revision": 0,
                "last_event": "identity passthrough",
                "reference_mode": "local_identity_origin",
                "lever_arm_m": (0.06, -0.03, 0.05),
                "final_sample_available": True,
                "final_sample_valid": False,
                "final_sample_revision": 0,
                "final_sample_session": "odin-test",
                "final_sample_reference_mode": "local_identity_origin",
            },
            "raw": {
                "fresh": True,
                "age_seconds": 0.1,
                "x_m": 1.0,
                "y_m": 2.0,
                "z_m": 0.1,
                "yaw_deg": 3.0,
            },
            "corrected": {
                "fresh": True,
                "age_seconds": 0.1,
                "x_m": 1.0,
                "y_m": 2.0,
                "z_m": 0.1,
                "yaw_deg": 3.0,
            },
            "pose_fcu": {
                "fresh": True,
                "age_seconds": 0.1,
                "x_m": 1.1,
                "y_m": 2.1,
                "z_m": 0.1,
                "yaw_deg": 3.2,
            },
            "final": {
                "fresh": True,
                "age_seconds": 0.2,
                "x_m": 1.2,
                "y_m": 2.2,
                "z_m": 0.1,
                "yaw_deg": 3.3,
            },
            "result": {},
            "startup_error": "",
        }

    def start(self) -> None:
        self.started += 1

    def close(self) -> None:
        self.closed += 1

    def status(self) -> dict[str, Any]:
        return self.current_status

    def request_start(
        self,
        operation: int,
        tag_id: int,
        window_size: int,
        apply: bool,
        instance: str,
        window_revision: int,
        extnav_revision: int,
        callback=None,
    ) -> None:
        self.start_requests.append(
            (
                operation,
                tag_id,
                window_size,
                apply,
                instance,
                window_revision,
                extnav_revision,
            )
        )
        if callback is not None:
            callback({"accepted": True, "job_id": "job-test", "message": "ok"}, "")

    def request_stop(self, job_id: str, instance: str, callback=None) -> None:
        self.stop_requests.append((job_id, instance))
        if callback is not None:
            callback({"accepted": True, "message": "stopping"}, "")

    def request_clear(self, instance: str, revision: int, callback=None) -> None:
        self.clear_requests.append((instance, revision))
        if callback is not None:
            callback({"accepted": True, "message": "cleared"}, "")

    def request_apply_saved(
        self, instance: str, window_revision: int, extnav_revision: int, callback=None
    ) -> None:
        self.apply_saved_requests.append((instance, window_revision, extnav_revision))
        if callback is not None:
            callback({"accepted": True, "job_id": "apply-job", "message": "ok"}, "")


def _application() -> QApplication:
    """复用 Qt 全局应用。"""
    return QApplication.instance() or QApplication([])


def test_panel_starts_first_dry_run_and_displays_four_chain_positions() -> None:
    """默认 first 必须 dry-run，并分别展示 raw/corrected/FCU/EKF final。"""
    application = _application()
    client = _FakeCorrectionClient()
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        window.tag_id_input.setValue(7)
        window.first_button.click()
        application.processEvents()

        assert client.started == 1
        assert client.start_requests == [(1, 7, 5, False, "instance-test", 0, 0)]
        assert "x=+1.0000" in window.raw_pose.text()
        assert "x=+1.0000" in window.corrected_pose.text()
        assert "x=+1.1000" in window.pose_fcu.text()
        assert "x=+1.2000" in window.final_pose.text()
        assert (
            "sample=local_identity_origin (identity) r0"
            in window.reference_value.text()
        )
        assert "session=odin-test" in window.reference_value.text()
        assert "job-test" in window.command_message.text()
    finally:
        window.close()

    assert client.closed == 1
    assert client.stop_requests == []


def test_panel_defers_apply_until_frozen_candidate_confirmation(monkeypatch) -> None:
    """勾选应用也先 dry-run，候选数值可见后才弹一次确认并调用 apply_saved。"""
    application = _application()
    client = _FakeCorrectionClient()
    confirmations: list[str] = []

    def confirm(_parent, _title, text, *_args, **_kwargs):
        confirmations.append(text)
        return QMessageBox.StandardButton.Yes

    monkeypatch.setattr(QMessageBox, "warning", confirm)
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        window.apply_checkbox.setChecked(True)
        window.first_button.click()
        application.processEvents()
        assert client.start_requests == [(1, 0, 5, False, "instance-test", 0, 0)]
        assert client.apply_saved_requests == []

        client.current_status["correction"].update(
            window_revision=1,
            window_count=1,
            success_count=1,
            next_calibration=2,
            window_state="rough",
            candidate_saved=True,
            candidate_valid=True,
            candidate_stage="rough_single_tag",
            candidate_window_revision=1,
            application_state="not_requested",
            x_m=0.4,
            y_m=-0.2,
            yaw_deg=0.15,
            position_jump_m=0.03,
            yaw_jump_deg=0.15,
        )
        client.current_status["result"] = {
            "fresh": True,
            "job_id": "job-test",
            "success": True,
            "window_saved": True,
            "window_revision": 1,
            "application_state": "not_requested",
            "outcome": "saved_unapplied",
            "message": "saved",
        }
        window._refresh()
        application.processEvents()

        assert client.apply_saved_requests == [("instance-test", 1, 0)]
        assert len(confirmations) == 1
        assert "x +0.4000 m" in confirmations[0]
        assert "rough_single_tag" in confirmations[0]
        assert "reset counter" in confirmations[0]
    finally:
        window.close()


def test_panel_derives_dynamic_next_and_window_lock_from_server() -> None:
    """N/按钮/窗口长度必须来自服务端窗口，GUI 不自行计数。"""
    application = _application()
    client = _FakeCorrectionClient()
    client.current_status["correction"].update(
        window_revision=4,
        window_count=1,
        success_count=1,
        next_calibration=2,
        window_state="rough",
        keyframes=[
            {
                "tag_id": 0,
                "p_x_m": 0.0,
                "p_y_m": 0.0,
                "q_x_m": 1.0,
                "q_y_m": 2.0,
                "sigma_m": 0.012,
                "samples": 30,
                "blocks": 10,
                "time_source": "header",
            }
        ],
    )
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        assert not window.first_button.isEnabled()
        assert window.next_button.isEnabled()
        assert window.next_button.text() == "开始第 2 次校准"
        assert not window.window_size_input.isEnabled()
        window.tag_id_input.setValue(1)
        window.next_button.click()
        application.processEvents()
        assert client.start_requests == [(2, 1, 5, False, "instance-test", 4, 0)]
        assert "Tag 0" in window.keyframe_values.text()
    finally:
        window.close()


def test_panel_only_enables_stop_for_active_job() -> None:
    """唯一活动状态由服务端控制，采样中只能停止而不能再 start。"""
    application = _application()
    client = _FakeCorrectionClient()
    client.current_status["correction"].update(
        active=True,
        state="sampling",
        operation="next",
        job_id="active-job",
        message="采样中",
    )
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        assert not window.first_button.isEnabled()
        assert not window.next_button.isEnabled()
        assert window.stop_button.isEnabled()

        window.stop_button.click()
        application.processEvents()
        assert client.stop_requests == [("active-job", "instance-test")]
    finally:
        window.close()


def test_panel_apply_saved_confirmation_and_unknown_gate(monkeypatch) -> None:
    """保存候选和 unknown 都只能经详细确认调用 apply_saved，冲突写操作禁用。"""
    application = _application()
    client = _FakeCorrectionClient()
    client.current_status["correction"].update(
        window_revision=3,
        window_count=2,
        success_count=2,
        next_calibration=3,
        window_state="two_point_limited",
        candidate_saved=True,
        candidate_valid=True,
        candidate_stage="refined_multi_tag",
        application_state="not_requested",
        x_m=0.4,
        y_m=-0.2,
        yaw_deg=0.15,
        position_jump_m=0.03,
        yaw_jump_deg=0.15,
    )
    monkeypatch.setattr(
        QMessageBox,
        "warning",
        lambda *args, **kwargs: QMessageBox.StandardButton.Yes,
    )
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        assert window.apply_saved_button.isEnabled()
        window.apply_saved_button.click()
        application.processEvents()
        assert client.apply_saved_requests == [("instance-test", 3, 0)]

        client.current_status["correction"].update(
            application_state="unknown", window_state="application_unknown"
        )
        window._refresh()
        assert not window.next_button.isEnabled()
        assert not window.clear_button.isEnabled()
        assert window.apply_saved_button.isEnabled()
        assert "未知" in window.apply_saved_button.text()
    finally:
        window.close()


def test_spin_boxes_ignore_wheel_events() -> None:
    """Tag ID 与滑窗长度不得因页面滚动被意外修改。"""
    application = _application()
    client = _FakeCorrectionClient()
    window = CorrectionPanelWindow(client=client)

    class _Event:
        ignored = False

        def ignore(self) -> None:
            self.ignored = True

    try:
        window.show()
        application.processEvents()
        tag_event = _Event()
        window_event = _Event()
        window.tag_id_input.wheelEvent(tag_event)  # type: ignore[arg-type]
        window.window_size_input.wheelEvent(window_event)  # type: ignore[arg-type]
        assert tag_event.ignored and window_event.ignored
    finally:
        window.close()
