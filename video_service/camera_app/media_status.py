"""摄像头面板可插入的地面媒体后台状态；只读轮询不管理服务生命周期。"""

from __future__ import annotations

import sys
import threading
from pathlib import Path
from typing import Any

from PySide6.QtCore import QEvent, QObject, QProcess, QTimer, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from video_service.file_manager_app import (
    MediaClient, configured_api_url, format_created_at, record_basis, state_label,
)

# 状态刷新频率与文件窗口保持一致，网络工作全部在后台线程。
_POLL_INTERVAL_MS = 4000
# 文件管理与摄像头窗口为独立进程；关闭任一窗口不停止媒体后台。
_FILE_MANAGER_SCRIPT = Path(__file__).resolve().parent.parent / "file_manager.py"


class _StatusBridge(QObject):
    """把HTTP工作线程的纯数据转回Qt线程。"""

    done = Signal(object, str)


class GroundMediaStatus(QWidget):
    """显示地面处理流、任务录像和异常，并提供独立文件窗口入口。"""

    def __init__(self, parent: QWidget | None = None, *, api_url: str | None = None) -> None:
        """读取地面配置并启动后台只读轮询。"""
        super().__init__(parent)
        self._busy = False
        self._stopped = False
        self._aircraft_id = ""
        self._session_id = ""
        self._latest_context: tuple[str, str] = ("", "")
        self._inventory: dict[str, Any] = {}
        self._config_error = ""
        try:
            self._client = MediaClient(api_url or configured_api_url())
        except Exception as exc:
            self._client = MediaClient("http://127.0.0.1:8090")
            self._config_error = f"配置读取失败，暂用本机默认地址：{exc}"
        self._bridge = _StatusBridge(self)
        self._bridge.done.connect(self._completed)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        header = QHBoxLayout()
        header.addWidget(QLabel("地面媒体后台"), 1)
        self.files_button = QPushButton("文件管理")
        self.files_button.clicked.connect(self._open_files)
        header.addWidget(self.files_button)
        layout.addLayout(header)
        self.stream_status = QLabel("处理流：正在读取地面后台…")
        self.video_status = QLabel("任务录像：尚无后台记录")
        self.error_status = QLabel(self._config_error)
        for label in (self.stream_status, self.video_status, self.error_status):
            label.setWordWrap(True)
            layout.addWidget(label)
        self.setToolTip(f"只读媒体状态：{self._client.api_url}；关窗不停止服务")
        self._timer = QTimer(self)
        self._timer.setInterval(_POLL_INTERVAL_MS)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        if parent is not None:
            # 父窗口关闭即可停止轮询，无需其closeEvent额外接入。
            parent.installEventFilter(self)
        QTimer.singleShot(0, self.refresh)

    def set_context(self, aircraft_id: str = "", session_id: str = "") -> None:
        """可选限制到指定飞机/任务；不修改后台状态或预览地址。"""
        self._aircraft_id = aircraft_id
        self._session_id = session_id
        self._render()

    def refresh(self) -> None:
        """在后台线程读取inventory，慢请求不阻塞预览窗口。"""
        if self._busy or self._stopped:
            return
        self._busy = True
        bridge = self._bridge
        client = self._client

        def worker() -> None:
            """HTTP响应经信号回到Qt，窗口销毁后静默结束。"""
            result, error = {}, ""
            try:
                result = client.request("/api/media")
            except Exception as exc:
                error = str(exc)
            try:
                bridge.done.emit(result, error)
            except RuntimeError:
                pass

        threading.Thread(target=worker, name="ground-media-status", daemon=True).start()

    def _completed(self, inventory: dict[str, Any], error: str) -> None:
        """保留上次可用状态，显示本次连接失败或后台异常。"""
        self._busy = False
        if self._stopped:
            return
        if error:
            self.error_status.setText(f"地面媒体状态读取失败：{error}；以上状态保留上次结果")
            return
        self._inventory = inventory
        self._render()

    def _matches_context(self, item: dict[str, Any]) -> bool:
        """按显式上下文筛选，未指定时显示全部飞机。"""
        return (
            (not self._aircraft_id or str(item.get("aircraft_id", "")) == self._aircraft_id)
            and (not self._session_id or str(item.get("session_id", "")) == self._session_id)
        )

    def _render(self) -> None:
        """仅展示后台报告的流与录像阶段，不据文件存在宣称完整。"""
        streams = [item for item in self._inventory.get("streams", []) if self._matches_context(item)]
        videos = [item for item in self._inventory.get("records", [])
                  if item.get("kind") == "video" and self._matches_context(item)]
        videos.sort(key=lambda item: format_created_at(item.get("created_at")), reverse=True)
        stream_lines = []
        errors = [self._config_error, str(self._inventory.get("error") or "")]
        for stream in streams:
            aircraft = str(stream.get("aircraft_id") or "未知飞机")
            phase = state_label(stream.get("state"))
            stream_lines.append(f"{aircraft}：{phase} {stream.get('url', '')}")
            if stream.get("error"):
                errors.append(f"{aircraft}：{stream['error']}")
        self.stream_status.setText("地面处理流：\n" + ("\n".join(stream_lines) or "当前无活动流；已存文件仍可浏览"))
        video_lines = []
        for video in videos[:3]:
            variant = {"original": "原片缓存", "experiment": "实验处理版", "rebuilt": "原片重建版"}.get(video.get("variant"), "未知版本")
            video_lines.append(
                f"{video.get('aircraft_id', '未知飞机')} / {video.get('session_id', '未知任务')} "
                f"{variant}：{state_label(video.get('state'))}；{record_basis(video)}"
            )
        self.video_status.setText("最近任务录像：\n" + ("\n".join(video_lines) or "尚无录像记录"))
        self.error_status.setText("；".join(value for value in errors if value))
        # 优先定位当前活动任务，其次最近录像；多流且无上下文时保留全库入口。
        candidates = streams if len(streams) == 1 else (videos[:1] if not streams else [])
        if self._session_id:
            self._latest_context = (self._aircraft_id, self._session_id)
        elif candidates:
            self._latest_context = (
                str(candidates[0].get("aircraft_id") or ""),
                str(candidates[0].get("session_id") or ""),
            )
        else:
            self._latest_context = (self._aircraft_id, "")
        self.files_button.setText("查看本次任务文件" if self._latest_context[1] else "文件管理")

    def _open_files(self) -> None:
        """独立启动文件管理并传递可用的飞机/任务筛选。"""
        aircraft, session = self._latest_context
        arguments = [str(_FILE_MANAGER_SCRIPT)]
        if aircraft:
            arguments += ["--aircraft-id", aircraft]
        if session:
            arguments += ["--session-id", session]
        try:
            started, _pid = QProcess.startDetached(
                sys.executable, arguments, str(_FILE_MANAGER_SCRIPT.parent)
            )
        except Exception as exc:
            self.error_status.setText(f"文件管理窗口启动失败：{exc}")
            return
        if not started:
            self.error_status.setText("文件管理窗口启动失败，请检查项目Python与PySide6环境")

    def stop(self) -> None:
        """只停止本组件轮询，不发送任何服务停止请求。"""
        self._stopped = True
        self._timer.stop()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:  # noqa: N802
        """父面板关闭时自动释放轮询，保持三行插入方式。"""
        if event.type() == QEvent.Type.Close:
            self.stop()
        return super().eventFilter(watched, event)

    def closeEvent(self, event: Any) -> None:  # noqa: N802
        """独立关闭组件同样只停止读取，不影响后台。"""
        self.stop()
        super().closeEvent(event)
