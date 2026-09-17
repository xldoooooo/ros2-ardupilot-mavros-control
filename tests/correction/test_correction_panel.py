"""独立 Tag-Odin 2.0 面板的响应式布局、诊断展示、门控和生命周期测试。"""

from __future__ import annotations

import os
from typing import Any

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from correction_service.ros_client import CorrectionPanelClient
from PySide6.QtWidgets import QApplication, QMessageBox, QScrollArea

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
            "legacy_t_pose_fcu": {
                "fresh": False,
                "available": False,
                "reason": "修正未生效；identity 分支没有 Task29 前后差异",
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
    """默认 first 必须 dry-run，并展示四段生产链及旧 T 公式诊断行。"""
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
        assert "identity 分支没有 Task29 前后差异" in window.legacy_t_pose_fcu.text()
        assert "x=+1.2000" in window.final_pose.text()
        assert "Tag解码次数=0" in window.sample_value.text()
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


def test_legacy_t_comparison_adds_only_historical_fixed_xy_offset() -> None:
    """旧 T 对照必须由当前 FCU 同源位姿只加 T_xy，不改 z/yaw 或生产快照。"""
    pose_fcu = {
        "fresh": True,
        "age_seconds": 0.02,
        "x_m": 0.24,
        "y_m": 0.15,
        "z_m": 0.72,
        "yaw_deg": -1.5,
    }
    extnav = {
        "fresh": True,
        "age_seconds": 0.10,
        "valid": True,
        "revision": 7,
        "session": "odin-current",
        "lever_arm_m": (0.06, -0.03, 0.05),
        "final_sample_available": True,
        "final_sample_valid": True,
        "final_sample_revision": 7,
        "final_sample_session": "odin-current",
    }

    comparison = CorrectionPanelClient._legacy_t_pose_fcu(pose_fcu, extnav)

    assert comparison["available"] is True
    assert comparison["x_m"] == 0.30
    assert comparison["y_m"] == 0.12
    assert comparison["z_m"] == pose_fcu["z_m"]
    assert comparison["yaw_deg"] == pose_fcu["yaw_deg"]
    assert comparison["delta_x_m"] == 0.06
    assert comparison["delta_y_m"] == -0.03
    assert comparison["age_seconds"] == 0.02
    assert comparison["extnav_age_seconds"] == 0.10
    assert pose_fcu["x_m"] == 0.24
    assert extnav["lever_arm_m"] == (0.06, -0.03, 0.05)


def test_legacy_t_comparison_rejects_identity_or_unaligned_sample() -> None:
    """未激活修正或最终样本 revision 不一致时，不得伪造旧公式对照。"""
    pose_fcu = {
        "fresh": True,
        "age_seconds": 0.02,
        "x_m": 0.24,
        "y_m": 0.15,
        "z_m": 0.72,
        "yaw_deg": -1.5,
    }
    extnav = {
        "fresh": True,
        "age_seconds": 0.10,
        "valid": False,
        "revision": 7,
        "session": "odin-current",
        "lever_arm_m": (0.06, -0.03, 0.05),
        "final_sample_available": True,
        "final_sample_valid": False,
        "final_sample_revision": 7,
        "final_sample_session": "odin-current",
    }

    identity = CorrectionPanelClient._legacy_t_pose_fcu(pose_fcu, extnav)
    assert identity["available"] is False
    assert "identity 分支" in identity["reason"]

    extnav.update(valid=True, final_sample_valid=True, final_sample_revision=6)
    mismatched = CorrectionPanelClient._legacy_t_pose_fcu(pose_fcu, extnav)
    assert mismatched["available"] is False
    assert "revision/session" in mismatched["reason"]

    extnav["final_sample_revision"] = 7
    pose_fcu["age_seconds"] = 0.2
    older_pose = CorrectionPanelClient._legacy_t_pose_fcu(pose_fcu, extnav)
    assert older_pose["available"] is False
    assert "extnav 状态之后" in older_pose["reason"]


def test_panel_displays_live_legacy_t_pose_as_diagnostic_only() -> None:
    """修正有效时，GUI 应显示旧公式位姿、固定差值和对应 revision。"""
    application = _application()
    client = _FakeCorrectionClient()
    client.current_status["extnav"].update(
        valid=True,
        revision=7,
        reference_mode="tag_world_xy_local_z",
        final_sample_valid=True,
        final_sample_revision=7,
        final_sample_reference_mode="tag_world_xy_local_z",
    )
    client.current_status["legacy_t_pose_fcu"] = {
        "fresh": True,
        "available": True,
        "age_seconds": 0.1,
        "x_m": 0.30,
        "y_m": 0.12,
        "z_m": 0.72,
        "yaw_deg": -1.5,
        "delta_x_m": 0.06,
        "delta_y_m": -0.03,
        "revision": 7,
        "session": "odin-test",
    }

    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        text = window.legacy_t_pose_fcu.text()
        assert "x=+0.3000" in text
        assert "y=+0.1200" in text
        assert "Δxy=(+0.0600, -0.0300)" in text
        assert "r7" in text
    finally:
        window.close()


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


def test_panel_reflows_commands_without_horizontal_scroll_at_compact_width() -> None:
    """窄窗口应重排全部顶部操作，并保持内容无需横向滚动即可使用。"""
    application = _application()
    window = CorrectionPanelWindow(client=_FakeCorrectionClient())
    try:
        window.show()
        window.resize(560, 520)
        application.processEvents()

        scroll = window.findChild(QScrollArea)
        assert scroll is not None
        assert window._compact_layout is True
        assert scroll.horizontalScrollBar().maximum() == 0
        viewport = scroll.viewport().rect()
        for button in (
            window.first_button,
            window.next_button,
            window.stop_button,
            window.clear_button,
            window.apply_saved_button,
        ):
            assert viewport.contains(
                button.mapTo(scroll.viewport(), button.rect().topLeft())
            )
            assert viewport.contains(
                button.mapTo(scroll.viewport(), button.rect().bottomRight())
            )

        command_positions = {
            widget: window._command_layout.getItemPosition(
                window._command_layout.indexOf(widget)
            )
            for widget in (
                window.tag_id_input,
                window.window_size_input,
                window.first_button,
                window.next_button,
                window.stop_button,
                window.clear_button,
                window.apply_saved_button,
            )
        }
        assert command_positions[window.tag_id_input][:2] == (0, 1)
        assert command_positions[window.window_size_input][:2] == (1, 1)
        assert command_positions[window.first_button][:2] == (3, 0)
        assert command_positions[window.next_button][:2] == (3, 1)
        assert command_positions[window.stop_button][:2] == (4, 0)
        assert command_positions[window.clear_button][:2] == (4, 1)
        assert command_positions[window.apply_saved_button] == (5, 0, 1, 2)

        correction_position = window._status_layout.getItemPosition(
            window._status_layout.indexOf(window._correction_group)
        )
        extnav_position = window._status_layout.getItemPosition(
            window._status_layout.indexOf(window._extnav_group)
        )
        assert correction_position[:2] == (0, 0)
        assert extnav_position[:2] == (1, 0)

        window.resize(1180, 860)
        application.processEvents()
        assert window._compact_layout is False
        assert scroll.horizontalScrollBar().maximum() == 0
        assert window._command_layout.getItemPosition(
            window._command_layout.indexOf(window.stop_button)
        ) == (2, 2, 1, 2)
        assert window._status_layout.getItemPosition(
            window._status_layout.indexOf(window._extnav_group)
        )[:2] == (0, 1)
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
