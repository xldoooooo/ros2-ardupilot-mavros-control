"""地面文件面板的HTTP边界与异步离线浏览验证，不连接飞机。"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from video_service.file_manager_app import FileManagerWindow, MediaClient, record_basis


@pytest.fixture
def media_api():
    """启动仅用于本地验证的媒体HTTP服务。"""
    records = [
        {"id": "raw", "aircraft_id": "uav001", "session_id": "task-a", "kind": "video", "variant": "original", "state": "ready", "cache_state": "cleaned", "created_at": "2026-10-22T10:00:00", "path": "/media/raw.mp4"},
        {"id": "processed", "aircraft_id": "uav002", "session_id": "task-b", "kind": "image", "variant": "experiment", "state": "ready", "created_at": "2026-10-21T10:00:00", "path": "/media/photo.jpg"},
    ]
    actions = []

    class Handler(BaseHTTPRequestHandler):
        """返回固定记录、下载内容和动作确认。"""

        def do_GET(self):  # noqa: N802
            """延迟列表响应以验证Qt事件循环不阻塞。"""
            if self.path == "/api/media":
                time.sleep(0.1)
                body = json.dumps({"records": records, "storage": {"bytes": 1024, "max_bytes": 2048}}).encode()
            else:
                body = b"processed-content"
            self.send_response(200)
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):  # noqa: N802
            """记录动作与请求ID。"""
            actions.append((self.path, json.loads(self.rfile.read(int(self.headers["Content-Length"])))))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"{}")

        def log_message(self, *_args):
            """隐藏测试HTTP访问日志。"""
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    yield f"http://127.0.0.1:{server.server_port}", records, actions
    server.shutdown()
    server.server_close()


def wait_for(app, predicate):
    """处理Qt事件直到异步任务完成或超时。"""
    deadline = time.monotonic() + 3
    while not predicate() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert predicate()


def test_file_api_download_and_original_boundary(media_api, tmp_path: Path):
    """原片不可下载；处理版通过API保存，重建保留原片ID。"""
    url, records, actions = media_api
    client = MediaClient(url)
    with pytest.raises(ValueError, match="原始缓存"):
        client.download(records[0], tmp_path / "raw.mp4")
    target = tmp_path / "photo.jpg"
    client.download(records[1], target)
    assert target.read_bytes() == b"processed-content"
    assert not target.with_name("photo.jpg.part").exists()
    client.request("/api/rebuild", {"id": records[0]["id"]})
    assert actions == [("/api/rebuild", {"id": "raw"})]
    assert "按策略清理" in record_basis(records[0])
    assert "缺失" not in record_basis(records[0])


def test_file_window_async_filter_and_close(media_api):
    """飞机离线无关；筛选保持真实状态，关窗不发送停止请求。"""
    url, _records, actions = media_api
    app = QApplication.instance() or QApplication([])
    window = FileManagerWindow(api_url=url)
    window.show()
    heartbeat = []
    QTimer.singleShot(15, lambda: heartbeat.append(True))
    wait_for(app, lambda: bool(window._records))
    assert heartbeat
    assert window.table.rowCount() == 2
    assert "1.0 KiB" in window.storage.text()
    window.table.selectRow(0)
    assert not window.save_button.isEnabled()
    assert window.rebuild_button.isEnabled()
    window.aircraft.setCurrentIndex(window.aircraft.findData("uav002"))
    assert window.table.rowCount() == 1
    window.table.selectRow(0)
    assert window.save_button.isEnabled()
    assert not window.rebuild_button.isEnabled()
    assert window.table.item(0, 5).text() == "实时处理版"
    window.date.setText("2026-10-22")
    assert window.table.rowCount() == 0
    window.close()
    assert not window._timer.isActive()
    assert actions == []


def test_file_manager_launcher_is_independent():
    """主窗口以项目Python启动独立文件面板，不触发环境清理。"""
    import sys
    from unittest.mock import patch

    from ground_station_core.config import PROJECT_ROOT
    from tests.support.qt import _FakeEnvironment, _close_window, _operational_snapshot, _window

    environment = _FakeEnvironment()
    window, _ros = _window(_operational_snapshot(armed=False), environment=environment)
    try:
        with patch(
            "ground_station_core.qt_ui.main_window.QProcess.startDetached",
            return_value=(True, 4322),
        ) as start_detached:
            window.file_manager_button.click()
        panel_script = PROJECT_ROOT / "video_service/file_manager.py"
        start_detached.assert_called_once_with(
            sys.executable, [str(panel_script)], str(panel_script.parent)
        )
        assert environment.cleanup_calls == 0
    finally:
        _close_window(window)


def test_camera_media_status_async_context_and_parent_close(media_api):
    """状态组件复用inventory并传递任务筛选，关父面板仅停轮询。"""
    import sys
    from unittest.mock import patch
    from PySide6.QtWidgets import QWidget
    from video_service.camera_app.media_status import GroundMediaStatus

    url, _records, actions = media_api
    app = QApplication.instance() or QApplication([])
    parent = QWidget()
    status = GroundMediaStatus(parent, api_url=url)
    parent.show()
    wait_for(app, lambda: bool(status._inventory))
    assert "原片缓存" in status.video_status.text()
    assert "按策略清理" in status.video_status.text()
    status.set_context("uav001", "task-a")
    assert status.files_button.text() == "查看本次任务文件"
    with patch(
        "video_service.camera_app.media_status.QProcess.startDetached",
        return_value=(True, 4323),
    ) as launch:
        status.files_button.click()
    expected = Path(__file__).resolve().parents[2] / "video_service/file_manager.py"
    launch.assert_called_once_with(
        sys.executable,
        [str(expected), "--aircraft-id", "uav001", "--session-id", "task-a"],
        str(expected.parent),
    )
    status._completed({"streams": [{"aircraft_id": "uav001", "session_id": "task-a", "state": "recording", "url": "rtsp://ground/uav001/rectified", "error": "已知中断"}], "records": [
        {"aircraft_id": "uav001", "session_id": "task-a", "kind": "video",
         "variant": "experiment", "state": "needs_attention", "error": "录像交付终止"},
    ], "error": "磁盘异常"}, "")
    assert "录制中" in status.stream_status.text()
    assert "rtsp://ground" in status.stream_status.text()
    assert "已知中断" in status.error_status.text()
    assert "磁盘异常" in status.error_status.text()
    assert "实时处理版" in status.video_status.text()
    assert "交付终止，需人工处理" in status.video_status.text()
    assert "1 条录像异常" in status.summary.text()
    status._completed({}, "服务离线")
    assert "服务离线" in status.error_status.text()
    assert "录制中" in status.stream_status.text()
    parent.close()
    assert not status._timer.isActive()
    assert actions == []


def test_file_window_initial_mission_filter(media_api):
    """摄像头快捷入口在第一次inventory读取时应用任务筛选。"""
    url, _records, _actions = media_api
    app = QApplication.instance() or QApplication([])
    window = FileManagerWindow(api_url=url, aircraft_id="uav002", session_id="task-b")
    wait_for(app, lambda: bool(window._records))
    assert window.aircraft.currentData() == "uav002"
    assert window.mission.currentData() == "task-b"
    assert window.table.rowCount() == 1
    window.close()


def test_file_window_refresh_reuses_cells_and_preserves_selection(monkeypatch):
    """轮询不重建整表，状态变化只更新单元格并保留选择与列宽。"""
    from PySide6.QtWidgets import QHeaderView

    monkeypatch.setattr(FileManagerWindow, "refresh", lambda _self: None)
    app = QApplication.instance() or QApplication([])
    window = FileManagerWindow(api_url="http://127.0.0.1:1")
    records = [
        {"id": str(index), "aircraft_id": "uav002", "session_id": "task-a",
         "kind": "video", "variant": "experiment", "state": "recording",
         "created_at": 1793150000, "path": f"/media/processed/{index}.mp4"}
        for index in range(300)
    ]
    try:
        window._completed("refresh", {"records": records, "deliveries": [
            {"aircraft_id": "uav002", "session_id": "task-a", "state": "failed", "error": "已超时"},
        ]}, "")
        assert window.table.rowCount() == len(records)
        window.table.selectRow(150)
        window.table.setColumnWidth(1, 312)
        item = window.table.item(150, 6)
        window._completed("refresh", {"records": [dict(record) for record in records]}, "")
        assert window.table.item(150, 6) is item
        updated = [dict(record) for record in records]
        updated[150]["state"] = "ready"
        window._completed("refresh", {"records": updated}, "")
        assert window.table.item(150, 6) is item
        assert item.text() == "保存完成"
        assert window.selected_record()["id"] == "150"
        assert window.table.columnWidth(1) == 312
        assert window.table.horizontalHeader().sectionResizeMode(1) == QHeaderView.ResizeMode.Interactive
        assert window.table.updatesEnabled()
    finally:
        window.close()
        app.processEvents()


def test_file_window_directory_probe_runs_only_on_click_in_worker(monkeypatch, tmp_path):
    """选中行不探测文件系统，慢挂载点目录检查不阻塞Qt事件循环。"""
    from PySide6.QtGui import QDesktopServices

    monkeypatch.setattr(FileManagerWindow, "refresh", lambda _self: None)
    app = QApplication.instance() or QApplication([])
    window = FileManagerWindow(api_url="http://127.0.0.1:1")
    entered, release = threading.Event(), threading.Event()
    opened, probes, heartbeat = [], [], []
    original_is_dir = Path.is_dir

    def slow_is_dir(path):
        """让选定目录模拟等待挂载点，记录调用线程。"""
        if path == tmp_path:
            probes.append(threading.get_ident())
            entered.set()
            release.wait(2)
        return original_is_dir(path)

    monkeypatch.setattr(Path, "is_dir", slow_is_dir)
    monkeypatch.setattr(QDesktopServices, "openUrl", lambda url: opened.append(url.toLocalFile()))
    try:
        window._completed("refresh", {"records": [{
            "id": "video", "variant": "experiment", "kind": "video", "state": "ready",
            "path": str(tmp_path / "video.mp4"),
        }]}, "")
        window.table.selectRow(0)
        assert not probes
        window.directory_button.click()
        QTimer.singleShot(0, lambda: heartbeat.append(True))
        wait_for(app, lambda: entered.is_set() and bool(heartbeat))
        assert len(probes) == 1
        assert probes[0] != threading.get_ident()
        assert not opened
        release.set()
        wait_for(app, lambda: bool(opened))
        assert opened == [str(tmp_path)]
    finally:
        release.set()
        window.close()
        app.processEvents()


def test_inventory_unix_date_processed_size_and_cache_cleanup(media_api, monkeypatch):
    """真实合同使用UTC Unix时间及处理版大小，机载缓存删除按策略清理。"""
    from datetime import datetime, timezone
    from video_service.file_manager_app import format_created_at, state_label

    url, records, _actions = media_api
    with monkeypatch.context() as timezone_environment:
        timezone_environment.setenv("TZ", "Asia/Shanghai")
        time.tzset()
        try:
            timestamp = datetime(2026, 10, 21, 16, 30, tzinfo=timezone.utc).timestamp()
            records[0].update(created_at=timestamp, size=1024, processed_size=2048)
            records[1].update(created_at=timestamp, size=0, processed_size=4096, cache_state="deleted", state="incomplete")
            assert format_created_at(timestamp) == "2026-10-22 00:30:00"
            assert format_created_at("2026-10-21T16:30:00Z") == "2026-10-22 00:30:00"
            assert format_created_at("2026-10-22T00:30:00") == "2026-10-22 00:30:00"
            app = QApplication.instance() or QApplication([])
            window = FileManagerWindow(api_url=url)
            wait_for(app, lambda: bool(window._records))
            window.date.setText("2026-10-22")
            assert window.table.rowCount() == 2
            assert window.table.item(0, 9).text() == "1.0 KiB"
            assert window.table.item(1, 9).text() == "4.0 KiB"
            assert window.table.item(1, 8).text() == "2026-10-22 00:30:00"
            assert "机载缓存：按策略清理" in record_basis(records[1])
            assert "缺失" not in record_basis(records[1])
            assert state_label("incomplete") == "录像已知不完整"
            assert state_label("held") == "原片暂存，按需上传"
            assert state_label("retained") == "无需上传，保留原片"
            assert state_label("needs_attention") == "交付终止，需人工处理"
            assert "机载缓存：机载保留" in record_basis({"cache_state": "retained"})
            stopped = record_basis({"state": "needs_attention", "error": "磁盘已满"})
            assert "交付终止，需人工处理" in stopped
            assert "磁盘已满" in stopped
            window.date.setText("2026-10-21")
            assert window.table.rowCount() == 0
            window.close()
        finally:
            # monkeypatch退出后恢复系统本地时区转换。
            timezone_environment.undo()
            time.tzset()
