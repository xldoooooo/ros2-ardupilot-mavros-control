"""独立 Tag-Odin 面板的控制门、展示和生命周期回归测试。"""

from __future__ import annotations

import os
from typing import Any

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from correction_service.correction_panel import CorrectionPanelWindow


class _FakeCorrectionClient:
    """不创建 DDS participant 的面板客户端替身。"""

    def __init__(self) -> None:
        self.started = 0
        self.closed = 0
        self.start_requests: list[tuple[int, bool]] = []
        self.stop_requests: list[str] = []
        self.current_status: dict[str, Any] = {
            "correction": {
                "fresh": True,
                "service_available": True,
                "active": False,
                "state": "idle",
                "message": "服务空闲；下视相机关闭",
            },
            "extnav": {
                "fresh": True,
                "service_available": True,
                "odin_available": True,
                "valid": False,
                "session": "odin-test",
                "revision": 0,
                "last_event": "identity passthrough",
            },
            "raw": {"fresh": True, "x_m": 1.0, "y_m": 2.0, "z_m": 0.1, "yaw_deg": 3.0},
            "corrected": {
                "fresh": True,
                "x_m": 1.0,
                "y_m": 2.0,
                "z_m": 0.1,
                "yaw_deg": 3.0,
            },
            "final": {
                "fresh": True,
                "x_m": 1.1,
                "y_m": 2.1,
                "z_m": 0.1,
                "yaw_deg": 3.2,
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

    def request_start(self, tag_id: int, apply: bool, callback=None) -> None:
        self.start_requests.append((tag_id, apply))
        if callback is not None:
            callback({"accepted": True, "job_id": "job-test", "message": "ok"}, "")

    def request_stop(self, job_id: str, callback=None) -> None:
        self.stop_requests.append(job_id)
        if callback is not None:
            callback({"accepted": True, "message": "stopping"}, "")


def _application() -> QApplication:
    """复用 Qt 全局应用。"""
    return QApplication.instance() or QApplication([])


def test_panel_starts_dry_run_and_displays_three_chain_positions() -> None:
    """默认开始必须是 apply=false，并同时展示 raw/corrected/MAVROS final。"""
    application = _application()
    client = _FakeCorrectionClient()
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        window.tag_id_input.setValue(7)
        window.start_button.click()
        application.processEvents()

        assert client.started == 1
        assert client.start_requests == [(7, False)]
        assert "x=+1.0000" in window.raw_pose.text()
        assert "x=+1.0000" in window.corrected_pose.text()
        assert "x=+1.1000" in window.final_pose.text()
        assert "job-test" in window.command_message.text()
    finally:
        window.close()

    assert client.closed == 1
    assert client.stop_requests == []


def test_panel_only_enables_stop_for_active_job() -> None:
    """one-job 状态由服务权威控制，面板不能在 active 时再发 start。"""
    application = _application()
    client = _FakeCorrectionClient()
    client.current_status["correction"].update(
        active=True,
        state="sampling",
        job_id="active-job",
        message="采样中",
    )
    window = CorrectionPanelWindow(client=client)
    try:
        window.show()
        application.processEvents()
        assert not window.start_button.isEnabled()
        assert window.stop_button.isEnabled()

        window.stop_button.click()
        application.processEvents()
        assert client.stop_requests == ["active-job"]
    finally:
        window.close()
