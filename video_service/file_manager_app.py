"""地面媒体库薄客户端：HTTP后台访问、飞机/任务筛选和处理版下载。"""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlencode, urljoin
from urllib.request import Request, urlopen

from PySide6.QtCore import QObject, Qt, QTimer, QUrl, Signal
from PySide6.QtGui import QCloseEvent, QDesktopServices, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QFileDialog, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QSplitter,
    QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

from ground_station_core.qt_ui.window_chrome import ShadowWindowChromeMixin
from video_service.relay_config import project_path

# 后台默认端口；GROUND_MEDIA_CONFIG 可替换部署配置。
_DEFAULT_API_URL = "http://127.0.0.1:8090"
# 轮询与网络超时仅影响媒体功能，不阻塞 Qt 或飞控。
_REFRESH_INTERVAL_MS = 4000
_HTTP_TIMEOUT_SECONDS = 10
# 初始列宽允许手动调整，避免每个单元格变化都触发全表内容测量。
_TABLE_COLUMN_WIDTHS = (90, 240, 65, 75, 65, 110, 110, 260, 165, 100, 320)
# 显示实际记录状态，不把缓存清理等同于地面缺失。
_STATE_LABELS = {
    "pending": "待上传", "queued": "待上传", "uploading": "上传中",
    "uploaded": "已上传", "awaiting_ack": "等待入库确认", "confirmed": "已确认入库",
    "held": "原片暂存，按需上传", "needs_attention": "交付终止，需人工处理",
    "processing": "处理中", "rebuilding": "原片重建中", "recording": "录制中",
    "starting": "启动中", "saving": "封装保存中",
    "ready": "保存完成", "completed": "保存完成", "failed": "失败",
    "error": "异常", "interrupted": "录像已知中断", "missing": "文件缺失",
    "deleted": "已删除", "cleaned": "按策略清理", "expired": "按策略清理",
    "retained": "无需上传，保留原片", "present": "存在", "unknown": "未知",
    "incomplete": "录像已知不完整", "original_ready": "原片已入库",
    "processing_error": "处理失败", "retry": "上传失败待重试",
    "acknowledged": "已确认入库", "cache_deleted": "按策略清理",
}
# 复用窗口外框，保持文件窗口自身依赖轻量。
PANEL_STYLE_SHEET = """
QWidget { color: #182433; font-family: "Noto Sans CJK SC", sans-serif; font-size: 10pt; }
QMainWindow { background: transparent; }
QFrame#subpanelWindowFrame { background: #f7f8fa; border: 1px solid #8595a5; border-radius: 8px; }
QFrame#subpanelTitleBar { background: white; border-bottom: 1px solid #cfd6de; }
QPushButton, QComboBox, QLineEdit { min-height: 29px; padding: 3px 8px; }
QTableWidget, QTextEdit { background: white; border: 1px solid #cfd6de; }
QLabel#imagePreview { background: #182433; color: #d9e1e8; }
"""


def configured_api_url() -> str:
    """读取地面媒体配置中的 API 地址，未配置时访问本机后台。"""
    config = project_path(os.environ.get(
        "GROUND_MEDIA_CONFIG",
        str(Path(__file__).resolve().parent.parent / "config/ground/media.json"),
    ))
    if not config.exists():
        return _DEFAULT_API_URL
    payload = json.loads(config.read_text(encoding="utf-8"))
    return str(payload.get("api_url", _DEFAULT_API_URL)).rstrip("/")


def human_size(value: Any) -> str:
    """把记录字节数转换为便于比较的容量文字。"""
    try:
        size = float(value)
    except (TypeError, ValueError):
        return "未知"
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if abs(size) < 1024 or unit == "TiB":
            return f"{size:.1f} {unit}"
        size /= 1024
    return "未知"


def state_label(value: Any) -> str:
    """保留未知后台状态，避免把新状态误报成成功。"""
    text = str(value or "unknown")
    return _STATE_LABELS.get(text, text)


def format_created_at(value: Any) -> str:
    """将Unix时间或ISO时间转换成系统本地日期时间。"""
    if value is None or value == "":
        return "未知"
    try:
        timestamp = float(value)
        moment = datetime.fromtimestamp(timestamp, tz=timezone.utc)
    except (TypeError, ValueError, OverflowError, OSError):
        try:
            moment = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        except ValueError:
            return str(value)
    return moment.astimezone().strftime("%Y-%m-%d %H:%M:%S")


def record_basis(record: dict[str, Any]) -> str:
    """仅依据后台提供的信息显示缺失、错误或缓存关系。"""
    details = [str(record[key]) for key in ("error", "missing_reason", "interruption") if record.get(key)]
    state = str(record.get("state", "unknown"))
    if state in (
        "pending", "queued", "uploading", "failed", "error", "interrupted", "missing",
        "deleted", "incomplete", "processing_error", "retry", "needs_attention",
    ):
        details.append(state_label(state))
    cache = str(record.get("cache_state", "unknown"))
    # 缓存是否保留与上传是否需要是两种状态，兼容旧库存记录。
    cache_label = {"deleted": "按策略清理", "retained": "机载保留"}.get(cache, state_label(cache))
    details.append(f"机载缓存：{cache_label}")
    return "；".join(dict.fromkeys(details))


class MediaClient:
    """访问地面后台；不连接飞机或推断本地媒体目录。"""

    def __init__(self, api_url: str) -> None:
        """保存服务根地址。"""
        self.api_url = api_url.rstrip("/")

    def request(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        """读取列表或提交单文件管理动作。"""
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(self.api_url + path, data=body, headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=_HTTP_TIMEOUT_SECONDS) as response:
            result = json.load(response)
        if result.get("error") and path != "/api/media":
            raise RuntimeError(str(result["error"]))
        return result

    def file_url(self, record: dict[str, Any]) -> str:
        """仅给处理版生成后台下载地址，禁止对外下载原始缓存。"""
        if record.get("variant") == "original":
            raise ValueError("原始缓存仅供后台处理，不提供公开下载")
        path = str(record.get("download_url") or "/api/file?" + urlencode({"id": record["id"]}))
        return urljoin(self.api_url + "/", path)

    def download(self, record: dict[str, Any], destination: Path | None = None) -> Any:
        """后台读取图片或以暂存文件下载处理版，完成后原子发布。"""
        with urlopen(self.file_url(record), timeout=_HTTP_TIMEOUT_SECONDS) as response:
            if destination is None:
                return response.read()
            partial = destination.with_name(destination.name + ".part")
            try:
                with partial.open("wb") as output:
                    while chunk := response.read(1024 * 1024):
                        output.write(chunk)
                partial.replace(destination)
            except Exception:
                partial.unlink(missing_ok=True)
                raise
        return str(destination)


class _WorkerBridge(QObject):
    """把后台 HTTP 结果送回 Qt 主线程。"""

    done = Signal(str, object, str)


class FileManagerWindow(ShadowWindowChromeMixin, QMainWindow):
    """独立文件窗口；后台服务与窗口具有不同生命周期。"""

    def __init__(
        self, *, api_url: str | None = None, aircraft_id: str = "", session_id: str = ""
    ) -> None:
        """构建界面并在后台轮询地面媒体服务。"""
        super().__init__()
        self._records: list[dict[str, Any]] = []
        self._visible: list[dict[str, Any]] = []
        self._busy: set[str] = set()
        self._closed = False
        self._preview_id = ""
        self._initial_filters = {"aircraft_id": aircraft_id, "session_id": session_id}
        self._bridge = _WorkerBridge(self)
        self._bridge.done.connect(self._completed)
        config_error = ""
        try:
            self._client = MediaClient(api_url or configured_api_url())
        except Exception as exc:
            self._client = MediaClient(_DEFAULT_API_URL)
            config_error = f"配置读取失败，暂用本机默认地址：{exc}"
        self.setWindowTitle("地面媒体文件管理")
        self.resize(1350, 820)
        self._configure_window_chrome()
        self._build_ui()
        if config_error:
            self.message.setText(config_error)
        self._timer = QTimer(self)
        self._timer.setInterval(_REFRESH_INTERVAL_MS)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        QTimer.singleShot(0, self.refresh)

    def _build_ui(self) -> None:
        """集中展示筛选、媒体关系、预览和操作按钮。"""
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.addWidget(self._build_window_title_bar("文件管理"))
        filters = QHBoxLayout()
        self.aircraft = QComboBox()
        self.mission = QComboBox()
        self.date = QLineEdit()
        self.date.setPlaceholderText("日期筛选，例如 2026-10-22")
        self.refresh_button = QPushButton("刷新")
        for label, widget in (("飞机", self.aircraft), ("任务", self.mission), ("日期", self.date)):
            filters.addWidget(QLabel(label))
            filters.addWidget(widget, 1)
        filters.addWidget(self.refresh_button)
        layout.addLayout(filters)
        self.storage = QLabel("地面存储：未知")
        layout.addWidget(self.storage)
        split = QSplitter()
        self.table = QTableWidget(0, 11)
        self.table.setHorizontalHeaderLabels([
            "飞机", "任务 / 会话", "航点", "photoNo", "类型", "版本", "阶段",
            "状态 / 缺失依据", "时间", "大小", "文件",
        ])
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setWordWrap(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for column, width in enumerate(_TABLE_COLUMN_WIDTHS):
            self.table.setColumnWidth(column, width)
        self.table.horizontalHeader().setStretchLastSection(True)
        split.addWidget(self.table)
        side = QWidget()
        side_layout = QVBoxLayout(side)
        self.preview = QLabel("选择处理版图片后点击预览")
        self.preview.setObjectName("imagePreview")
        self.preview.setMinimumSize(260, 200)
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.details = QTextEdit()
        self.details.setReadOnly(True)
        side_layout.addWidget(self.preview, 1)
        side_layout.addWidget(self.details, 1)
        split.addWidget(side)
        split.setSizes([950, 350])
        layout.addWidget(split, 1)
        actions = QHBoxLayout()
        self.preview_button = QPushButton("图片预览")
        self.open_button = QPushButton("打开录像")
        self.directory_button = QPushButton("打开目录")
        self.save_button = QPushButton("另存下载")
        self.delete_button = QPushButton("删除")
        self.rebuild_button = QPushButton("从原片整段重建")
        for button in (self.preview_button, self.open_button, self.directory_button, self.save_button, self.delete_button, self.rebuild_button):
            actions.addWidget(button)
        layout.addLayout(actions)
        self.message = QLabel(f"后台：{self._client.api_url}；原片仅用于内部缓存与重建")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        self.setCentralWidget(root)
        self.refresh_button.clicked.connect(self.refresh)
        self.aircraft.currentIndexChanged.connect(self._filter_records)
        self.mission.currentIndexChanged.connect(self._filter_records)
        self.date.textChanged.connect(self._filter_records)
        self.table.itemSelectionChanged.connect(self._selection_changed)
        self.preview_button.clicked.connect(self._preview_image)
        self.open_button.clicked.connect(self._open_video)
        self.directory_button.clicked.connect(self._open_directory)
        self.save_button.clicked.connect(self._save_file)
        self.delete_button.clicked.connect(self._delete_file)
        self.rebuild_button.clicked.connect(self._rebuild_file)
        self._selection_changed()

    def _run(self, name: str, action: Callable[[], Any]) -> None:
        """后台执行所有网络操作，关闭窗口也不停止服务。"""
        if name in self._busy or self._closed:
            return
        self._busy.add(name)
        self._selection_changed()
        bridge = self._bridge

        def worker() -> None:
            """把线程结果或错误交回 UI，容忍窗口已经销毁。"""
            result, error = None, ""
            try:
                result = action()
            except Exception as exc:
                error = str(exc)
            try:
                bridge.done.emit(name, result, error)
            except RuntimeError:
                pass

        threading.Thread(target=worker, name=f"media-files-{name}", daemon=True).start()

    def refresh(self) -> None:
        """异步刷新列表，失败时保留最近一次可见数据。"""
        self._run("refresh", lambda: self._client.request("/api/media"))

    def _completed(self, name: str, result: Any, error: str) -> None:
        """在 Qt 线程更新列表、图片或操作结果。"""
        self._busy.discard(name)
        if self._closed:
            return
        if error:
            self.message.setText(f"{name} 失败：{error}；列表保留上次读取结果")
        elif name == "refresh":
            self._records = result.get("records", [])
            storage = result.get("storage", {})
            maximum = storage.get("max_bytes", 0)
            limit = human_size(maximum) if maximum else "未设上限"
            scope = storage.get("scope", "后台未提供统计范围")
            self.storage.setText(f"地面存储：{human_size(storage.get('bytes'))} / {limit}；统计范围：{scope}")
            self._update_filters()
            warning = f"；后台异常：{result['error']}" if result.get("error") else ""
            self.message.setText(f"已读取 {len(self._records)} 条记录；后台 {self._client.api_url}{warning}")
        elif name == "preview":
            if str((self.selected_record() or {}).get("id", "")) == self._preview_id:
                pixmap = QPixmap()
                if pixmap.loadFromData(result):
                    self.preview.setPixmap(pixmap.scaled(self.preview.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
                else:
                    self.message.setText("图片内容无法解码")
        elif name == "open":
            QDesktopServices.openUrl(QUrl.fromLocalFile(result))
            self.message.setText(f"已打开处理版录像：{result}")
        elif name == "directory":
            QDesktopServices.openUrl(QUrl.fromLocalFile(result))
            self.message.setText(f"已打开本机目录：{result}")
        elif name == "save":
            self.message.setText(f"已下载：{result}")
        else:
            self.message.setText("操作已提交，后续阶段由后台更新")
            self.refresh()
        self._selection_changed()

    def _update_filters(self) -> None:
        """更新飞机及任务选项并保留用户选择。"""
        for combo, key in ((self.aircraft, "aircraft_id"), (self.mission, "session_id")):
            previous = combo.currentData() or self._initial_filters.pop(key, "") or None
            values = sorted({str(record.get(key) or "未知") for record in self._records})
            if previous and previous not in values:
                values.append(previous)
            if [combo.itemData(index) for index in range(combo.count())] == [None, *values]:
                continue
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("全部", None)
            for value in values:
                combo.addItem(value, value)
            index = combo.findData(previous)
            combo.setCurrentIndex(max(index, 0))
            combo.blockSignals(False)
        self._filter_records()

    def _filter_records(self) -> None:
        """按飞机、任务和日期过滤，刷新时保留选择。"""
        selected_id = (self.selected_record() or {}).get("id")
        visible = [record for record in self._records if
            (self.aircraft.currentData() is None or str(record.get("aircraft_id") or "未知") == self.aircraft.currentData())
            and (self.mission.currentData() is None or str(record.get("session_id") or "未知") == self.mission.currentData())
            and self.date.text().strip() in format_created_at(record.get("created_at"))]
        if visible == self._visible:
            # 后台轮询无变化时保留表格、滚动位置及用户调整的列宽。
            self._visible = visible
            return
        self._visible = visible
        self.table.setUpdatesEnabled(False)
        self.table.blockSignals(True)
        self.table.setCurrentCell(-1, -1)
        self.table.setRowCount(len(self._visible))
        selected_row = -1
        for row, record in enumerate(self._visible):
            variant = {"original": "原始缓存", "experiment": "实时处理版", "rebuilt": "原片重建版"}.get(record.get("variant"), "未知")
            values = [record.get("aircraft_id", "未知"), f"{record.get('mission_sequence', '未知')} / {record.get('session_id', '未知')}",
                record.get("waypoint_index", "未知"), record.get("photo_no", "未知"), record.get("kind", "未知"), variant,
                state_label(record.get("state")), record_basis(record), format_created_at(record.get("created_at")),
                human_size(record.get("size") if record.get("variant") == "original"
                           else record.get("processed_size", record.get("size"))), record.get("path", "")]
            for column, value in enumerate(values):
                text = str(value)
                item = self.table.item(row, column)
                if item is None:
                    item = QTableWidgetItem(text)
                    item.setToolTip(text)
                    self.table.setItem(row, column, item)
                elif item.text() != text:
                    item.setText(text)
                    item.setToolTip(text)
            if record.get("id") == selected_id:
                selected_row = row
        if selected_row >= 0:
            self.table.selectRow(selected_row)
        self.table.blockSignals(False)
        self.table.setUpdatesEnabled(True)
        self._selection_changed()

    def selected_record(self) -> dict[str, Any] | None:
        """返回当前单选行对应的完整后台记录。"""
        row = self.table.currentRow()
        return self._visible[row] if 0 <= row < len(self._visible) else None

    def _selection_changed(self) -> None:
        """仅给已有处理版提供下载入口，原片可请求后台重建。"""
        record = self.selected_record() or {}
        processed = bool(record.get("id")) and record.get("variant") in ("experiment", "rebuilt")
        available = processed and (bool(record.get("download_url")) or record.get("state") in ("ready", "completed"))
        action_busy = bool(self._busy - {"refresh", "preview"})
        self.preview_button.setEnabled(available and record.get("kind") == "image" and "preview" not in self._busy)
        self.open_button.setEnabled(available and record.get("kind") == "video" and not action_busy)
        self.save_button.setEnabled(available and not action_busy)
        self.directory_button.setEnabled(bool(record.get("path")) and not action_busy)
        self.delete_button.setEnabled(bool(record.get("id")) and not action_busy)
        self.rebuild_button.setEnabled(bool(record.get("id")) and record.get("kind") == "video" and record.get("variant") == "original" and not action_busy)
        if record:
            self.details.setPlainText(
                f"飞机：{record.get('aircraft_id', '未知')}\n"
                f"任务：{record.get('mission_sequence', '未知')}\n"
                f"会话：{record.get('session_id', '未知')}\n"
                f"航点：{record.get('waypoint_index', '未知')}\n"
                f"photoNo：{record.get('photo_no', '未知')}\n"
                f"阶段：{state_label(record.get('state'))}\n\n"
                f"关系 / 缺失依据：\n{record_basis(record)}\n\n"
                f"地面路径：{record.get('path', '尚未发布')}\n\n"
                "状态与缺失依据来自后台记录；不代表已核验逐帧完整。"
            )
        else:
            self.details.setPlainText("请选择文件。状态未知表示后台尚未提供依据；不代表已核验完整。")
        if str(record.get("id", "")) != self._preview_id:
            self.preview.clear()
            self.preview.setText("选择处理版图片后点击预览")

    def _preview_image(self) -> None:
        """后台读取当前处理版图片，不通过 GUI 访问飞机。"""
        record = self.selected_record()
        if record:
            self._preview_id = str(record["id"])
            self._run("preview", lambda: self._client.download(record))

    def _open_video(self) -> None:
        """下载处理版到用户缓存后交给系统播放器。"""
        record = self.selected_record()
        if not record:
            return
        cache = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "ardupilot-media-preview"
        cache.mkdir(parents=True, exist_ok=True)
        name = Path(str(record.get("path") or "video.mp4")).name
        # ID只作为本地缓存前缀，防止后台路径逃逸缓存目录。
        prefix = str(record["id"]).replace("/", "_").replace("\\", "_")
        self.message.setText("正在下载处理版录像，完成后打开…")
        self._run("open", lambda: self._client.download(record, cache / f"{prefix}-{name}"))

    def _open_directory(self) -> None:
        """后台核验本机目录后打开，选中行不访问可能阻塞的挂载点。"""
        record = self.selected_record()
        if record and record.get("path"):
            directory = Path(record["path"]).parent

            def check_directory() -> str:
                """只检查当前 GUI 机器的路径，远端后台路径不作本地猜测。"""
                if not directory.is_dir():
                    raise RuntimeError("本机不存在该目录；后台在其他机器时请使用另存下载")
                return str(directory)

            self._run("directory", check_directory)

    def _save_file(self) -> None:
        """选择保存路径后在后台下载处理版。"""
        record = self.selected_record()
        if not record:
            return
        name = Path(str(record.get("path") or "media")).name
        destination, _ = QFileDialog.getSaveFileName(self, "另存处理版", name)
        if destination:
            self._run("save", lambda: self._client.download(record, Path(destination)))

    def _delete_file(self) -> None:
        """确认后删除后台媒体记录及文件。"""
        record = self.selected_record()
        if record and QMessageBox.question(self, "删除媒体", "删除选中文件？此操作由地面后台执行，可能影响原片重建。") == QMessageBox.StandardButton.Yes:
            self._run("delete", lambda: self._client.request("/api/delete", {"id": record["id"]}))

    def _rebuild_file(self) -> None:
        """只从原始录像请求整段重建，避免二次去畸变。"""
        record = self.selected_record()
        if record and record.get("variant") == "original":
            self._run("rebuild", lambda: self._client.request("/api/rebuild", {"id": record["id"]}))

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        """只停止本窗口轮询，不向媒体后台发送停止命令。"""
        self._closed = True
        self._timer.stop()
        super().closeEvent(event)
