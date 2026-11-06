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

    def request_tags(self, *, save=False, csv="", service_instance_id="",
                     window_revision=0, callback=None) -> None:
        """模拟当前飞机的配置读取，避免 UI 测试创建真实 DDS。"""
        if callback:
            callback({"accepted": True, "csv": csv if save else
                      "tag_id,x,y,z,yaw_deg,size_m\n0,0,0,0,0,0.1\n",
                      "service_instance_id": "instance-test",
                      "config_path": "/aircraft/correction/tag_pose.csv", "message": "ok"}, "")

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
    """默认 first 必须 dry-run，并展示四段 FCU 生产链。"""
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
        assert window.pose_table.verticalHeaderItem(0).text() == "FCU 原始"
        assert window.pose_table.verticalHeaderItem(1).text() == "FCU 修正"
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


def test_panel_derives_dynamic_next_but_allows_manual_window_debugging() -> None:
    """N 来自服务端；手动空闲可尝试 FIRST/改窗口，准入由服务检查。"""
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
        assert window.first_button.isEnabled()
        assert window.next_button.isEnabled()
        assert window.next_button.text() == "开始第 2 次校准"
        assert window.window_size_input.isEnabled()
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
    """保存候选和 unknown 经详细确认调用 apply_saved；空闲时允许调试服务拒绝。"""
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
        assert window.next_button.isEnabled()
        assert window.clear_button.isEnabled()
        assert window.apply_saved_button.isEnabled()
        assert "未知" in window.apply_saved_button.text()
    finally:
        window.close()


def test_panel_reflows_commands_without_horizontal_scroll_at_compact_width() -> None:
    """窄窗口应重排全部顶部操作，并保持内容无需横向滚动即可使用。"""
    from tests.support.qt import _wait_for
    from correction_service.correction_panel import PANEL_STYLE_SHEET

    application = _application()
    window = CorrectionPanelWindow(client=_FakeCorrectionClient())
    # 与独立入口相同的主题；不依赖其他测试是否先应用全局样式。
    window.setStyleSheet(PANEL_STYLE_SHEET)
    try:
        window.show()
        window.resize(560, 520)
        _wait_for(lambda: window.width() == 560 and window.height() == 520)
        application.processEvents()

        scroll = window.findChild(QScrollArea)
        assert scroll is not None
        assert window._compact_layout is True
        assert window._side_by_side is False
        from PySide6.QtWidgets import QTabWidget
        assert window.findChild(QTabWidget) is None
        assert window.tag_editor.isVisible()
        assert window._main_layout.getItemPosition(
            window._main_layout.indexOf(window._tag_group)
        )[:2] == (1, 0)
        _wait_for(lambda: scroll.horizontalScrollBar().maximum() == 0)
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
                window._inputs_row,
                window.first_button,
                window.next_button,
                window.stop_button,
                window.clear_button,
                window.apply_saved_button,
            )
        }
        assert command_positions[window._inputs_row] == (0, 0, 1, 2)
        assert window.tag_id_input.maximumWidth() == 130
        assert window.window_size_input.maximumWidth() == 130
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
        _wait_for(lambda: window.width() == 1180 and window.height() == 860)
        application.processEvents()
        assert window._compact_layout is True
        assert window._side_by_side is True
        assert window.tag_editor.isVisible()
        assert window._main_layout.getItemPosition(
            window._main_layout.indexOf(window._tag_group)
        )[:2] == (0, 1)
        _wait_for(lambda: scroll.horizontalScrollBar().maximum() == 0)
        assert scroll.horizontalScrollBar().maximum() == 0
        assert window._command_layout.getItemPosition(
            window._command_layout.indexOf(window.stop_button)
        ) == (4, 0, 1, 1)
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


def test_tag_editor_roundtrip_sort_validation_and_draft_preservation() -> None:
    """配置表格按数字排序、阻止重复 ID，并保持刷新期间的草稿。"""
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    application = _application()
    client = _FakeCorrectionClient()
    window = CorrectionPanelWindow(client=client)
    try:
        editor = window.tag_editor
        assert editor.table.rowCount() == 1
        editor._populate("tag_id,x,y,z,yaw_deg,size_m\n10,1,2,3,45,0.2\n2,4,5,6,90,0.3\n")
        assert [editor.table.item(row, 0).text() for row in range(2)] == ["2", "10"]
        from PySide6.QtWidgets import QLineEdit
        window.show()
        # Wayland 的窗口 expose/焦点确认均异步，编辑测试须先等待真实焦点。
        assert QTest.qWaitForWindowExposed(window)
        window.activateWindow()
        assert QTest.qWaitForWindowActive(window)
        application.processEvents()
        editor.table.editItem(editor.table.item(0, 1))
        application.processEvents()
        cell_editor = editor.table.findChild(QLineEdit)
        assert cell_editor is not None
        cell_editor.setText("8.125")
        editor.save_button.click()
        application.processEvents()
        window._refresh()
        assert editor.table.item(0, 1).text() == "8.125"
        editor.table.item(1, 0).setData(Qt.ItemDataRole.EditRole, 2)
        editor.save_button.click()
        assert "唯一" in editor.message.text()
        editor.table.item(1, 0).setData(Qt.ItemDataRole.EditRole, 7)
        editor.save_button.click()
        application.processEvents()
        assert editor.message.text() == "ok"
        assert editor.table.item(0, 1).text() == "8.125"
        editor.add_button.click()
        assert editor.table.rowCount() == 3
        assert editor.table.item(2, 0).text() == "8"
        editor.table.selectRow(0)
        editor.delete_button.click()
        assert editor.table.rowCount() == 2
        assert [editor.table.item(row, 0).text() for row in range(2)] == ["2", "7"]
        client.current_status["correction"]["active"] = True
        window._refresh()
        assert not editor.save_button.isEnabled()
        assert not window.details_group.isChecked()
    finally:
        window.close()


def test_tag_toolbar_moves_parameters_preserves_ids_and_clears_draft() -> None:
    """移动交换五项参数但不动 ID；减号删末行，清空后仍可尝试调试保存。"""
    application = _application()
    window = CorrectionPanelWindow(client=_FakeCorrectionClient())
    try:
        editor = window.tag_editor
        editor._populate("tag_id,x,y,z,yaw_deg,size_m\n2,1,2,3,45,0.2\n7,4,5,6,90,0.3\n")
        editor.table.setCurrentCell(0, 1)
        editor.table.selectRow(0)
        assert not editor.up_button.isEnabled() and editor.down_button.isEnabled()
        editor.down_button.click()
        assert [editor.table.item(row, 0).text() for row in range(2)] == ["2", "7"]
        assert [editor.table.item(0, col).text() for col in range(1, 6)] == ["4", "5", "6", "90", "0.3"]
        assert [editor.table.item(1, col).text() for col in range(1, 6)] == ["1", "2", "3", "45", "0.2"]
        assert editor.table.currentRow() == 1
        assert editor.up_button.isEnabled() and not editor.down_button.isEnabled()
        editor.up_button.click()
        assert editor.table.item(0, 1).text() == "1"
        editor.table.selectRow(0)
        editor.delete_button.click()
        assert editor.table.rowCount() == 1 and editor.table.item(0, 0).text() == "2"
        editor.clear_button.click()
        assert editor.table.rowCount() == 0
        assert editor.save_button.isEnabled()
        assert not editor.clear_button.isEnabled() and not editor.delete_button.isEnabled()
        editor.x_input.lineEdit().setText("1.2345")
        editor.y_input.setValue(-2.5)
        editor.z_input.setValue(0.75)
        editor.yaw_input.setValue(45.125)
        editor.size_input.setValue(0.0465)
        editor.add_button.click()
        assert editor.table.item(0, 0).text() == "0"
        assert [float(editor.table.item(0, col).text()) for col in range(1, 6)] == [
            1.2345, -2.5, 0.75, 45.125, 0.0465,
        ]
        assert editor.add_button.text() == "+"
        assert editor.add_button.width() == editor.add_button.height()
        editor.add_button.click()
        assert editor.table.item(1, 0).text() == "1"
        assert editor.x_input.value() == 1.2345
        assert editor.save_button.isEnabled()
    finally:
        window.close()


def test_automatic_owner_makes_idle_and_active_panel_readonly_then_restores_manual():
    """独占覆盖 job 前后间隙；查看/重读可用，释放后恢复原有编辑和校准门控。"""
    from PySide6.QtWidgets import QAbstractItemView

    _application()
    client = _FakeCorrectionClient()
    window = CorrectionPanelWindow(client=client)
    try:
        window._refresh()
        editor = window.tag_editor
        triggers = editor.table.editTriggers()
        for active in (False, True):
            client.current_status["correction"].update(
                automatic_owner="onboard-calibration-test", active=active,
            )
            window._refresh()
            assert "只读" in window.command_message.text()
            for button in (window.first_button, window.next_button, window.stop_button,
                           window.clear_button, window.apply_saved_button,
                           editor.save_button, editor.add_button, editor.up_button,
                           editor.down_button, editor.delete_button, editor.clear_button):
                assert not button.isEnabled()
            assert editor.reload_button.isEnabled() and editor.table.isEnabled()
            assert editor.table.editTriggers() == QAbstractItemView.EditTrigger.NoEditTriggers
            editor._load()  # 异步读取完成也不能重新开放配置写入。
            assert not editor.save_button.isEnabled()
            editor.table.selectRow(0)
            assert not editor.delete_button.isEnabled()
            window._start()
            window._stop()
            window._clear()
            window._apply_saved()
            assert not client.start_requests and not client.stop_requests
            assert not client.clear_requests and not client.apply_saved_requests
        client.current_status["correction"].update(automatic_owner="", active=False)
        window._refresh()
        for button in (window.first_button, window.next_button, window.stop_button,
                       window.clear_button, window.apply_saved_button):
            assert button.isEnabled()
        assert editor.save_button.isEnabled() and editor.add_button.isEnabled()
        assert editor.delete_button.isEnabled()
        assert editor.table.editTriggers() == triggers
        client.current_status["correction"]["active"] = True
        window._refresh()
        assert window.stop_button.isEnabled()  # 普通手动任务仍允许用户停止。
    finally:
        window.close()


def test_client_rejects_manual_writes_under_owner_but_keeps_tag_reads():
    """客户端提交与队列出队共用 owner 门控，不新增手动权限限制。"""
    from correction_service.ros_client import _Command

    client = CorrectionPanelClient(domain_id=225)
    errors = []
    client._snapshots["correction"] = {"automatic_owner": "onboard-calibration-test"}
    for command in ("start", "stop", "clear", "apply_saved", "tags"):
        request = _Command(command, save=command == "tags", callback=lambda payload, error: errors.append(error))
        assert not client._manual_command_allowed(request)
    assert len(errors) == 5 and all("只读" in message for message in errors)
    assert client._manual_command_allowed(_Command("tags", save=False))
    client._snapshots["correction"]["automatic_owner"] = ""
    assert client._manual_command_allowed(_Command("start"))
