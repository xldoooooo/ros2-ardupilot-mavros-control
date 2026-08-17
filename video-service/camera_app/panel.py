"""与地面站配色协调、但运行于独立进程的摄像头配置面板。"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import sys
import threading
from typing import Any

from PySide6.QtCore import QObject, QProcess, QTimer, Qt, QUrl, Signal
from PySide6.QtMultimedia import QMediaPlayer
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .config import (
    CODEC_LABELS,
    CONTAINER_LABELS,
    CameraConfig,
    VIDEO_SERVICE_ROOT,
    format_fps,
)
from .controller import CameraServiceError
from .ipc import CameraServiceClient


SERVICE_SCRIPT = VIDEO_SERVICE_ROOT / "camera_service.py"
DESKTOP_APPLICATION_NAME = "ROS 2 ArduPilot Camera Panel"


PANEL_STYLE_SHEET = """
QWidget {
    color: #182433;
    font-family: "Noto Sans CJK SC", "Noto Sans", "DejaVu Sans", sans-serif;
    font-size: 10pt;
}
QMainWindow, QWidget#cameraPanelRoot { background: #eef1f4; }
QFrame#panelHeader, QFrame#previewCard, QGroupBox {
    background: #ffffff;
    border: 1px solid #cfd6de;
    border-radius: 4px;
}
QFrame#panelHeader { border-left: 4px solid #245f87; }
QLabel#panelTitle { font-size: 16pt; font-weight: 700; }
QLabel#mutedLabel { color: #5f6f80; }
QGroupBox {
    margin-top: 11px;
    padding: 12px 9px 9px 9px;
    font-weight: 700;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 9px;
    padding: 0 4px;
}
QLineEdit, QComboBox, QSpinBox {
    min-height: 30px;
    padding: 2px 7px;
    background: #ffffff;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
    selection-background-color: #245f87;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus {
    border: 2px solid #245f87;
}
QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled {
    color: #98a4b1; background: #edf0f2; border-color: #dbe0e5;
}
QPushButton {
    min-height: 32px;
    padding: 3px 11px;
    background: #f7f8fa;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
    font-weight: 600;
}
QPushButton:hover { background: #e8edf1; border-color: #82909e; }
QPushButton:disabled {
    color: #98a4b1; background: #edf0f2; border-color: #dbe0e5;
}
QPushButton[role="primary"] {
    color: white; background: #245f87; border-color: #245f87;
}
QPushButton[role="primary"]:hover {
    background: #194d70; border-color: #194d70;
}
QPushButton[role="success"] {
    color: white; background: #247457; border-color: #247457;
}
QPushButton[role="success"]:hover {
    background: #1b5c44; border-color: #1b5c44;
}
QPushButton[role="danger"] {
    color: white; background: #a7352a; border-color: #a7352a;
}
QPushButton[role="danger"]:hover {
    background: #87271f; border-color: #87271f;
}
QPushButton[role="primary"]:disabled,
QPushButton[role="success"]:disabled,
QPushButton[role="danger"]:disabled {
    color: #98a4b1; background: #edf0f2; border-color: #dbe0e5;
}
QWidget#videoSurface, QLabel#previewPlaceholder { background: #101820; }
QLabel#previewPlaceholder {
    color: #b8c5cf;
    font-size: 12pt;
    qproperty-alignment: AlignCenter;
}
QLineEdit#rtspUrlDisplay {
    font-family: "DejaVu Sans Mono", monospace;
    color: #245f87;
    background: #f5f8fa;
}
QScrollArea { border: none; background: transparent; }
QStatusBar { background: #ffffff; border-top: 1px solid #cfd6de; }
"""


class _PanelBridge(QObject):
    """把后台Socket请求结果安全投递到Qt主线程。"""

    completed = Signal(str, object, str)


class CameraPanelWindow(QMainWindow):
    """摄像头配置、独立启停、RTSP预览和截图操作窗口。"""

    def __init__(
        self,
        *,
        client: CameraServiceClient | None = None,
        auto_bootstrap: bool = True,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.client = client or CameraServiceClient()
        self._bridge = _PanelBridge(self)
        self._bridge.completed.connect(self._on_request_completed)
        self._request_in_flight: set[str] = set()
        self._service_ready = False
        self._config_loaded = False
        self._last_status: dict[str, Any] = {}
        self._all_modes: list[dict[str, Any]] = []
        self._active_preview_url = ""
        self._action_busy = False
        self._stopping_camera = False

        # 桌面环境可能直接使用窗口标题作为 Dock 名称，保持 ASCII 避免乱码。
        self.setWindowTitle(DESKTOP_APPLICATION_NAME)
        self.setMinimumSize(980, 680)
        self.resize(1180, 780)
        self._build_ui()
        self._connect_signals()
        self._configure_player()
        self._refresh_controls()

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(1000)
        self._poll_timer.timeout.connect(self._poll_status)
        self._poll_timer.start()
        if auto_bootstrap:
            QTimer.singleShot(0, self._bootstrap_service)

    def _build_ui(self) -> None:
        """构建顶部操作区、左侧预览与右侧可滚动配置区。"""
        root_widget = QWidget()
        root_widget.setObjectName("cameraPanelRoot")
        self.setCentralWidget(root_widget)
        root = QVBoxLayout(root_widget)
        root.setContentsMargins(14, 14, 14, 10)
        root.setSpacing(10)

        header = QFrame()
        header.setObjectName("panelHeader")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(14, 10, 12, 10)
        title = QLabel("摄像头配置面板")
        title.setObjectName("panelTitle")
        header_layout.addWidget(title, 1)
        self.start_button = QPushButton("开启摄像头")
        self.start_button.setProperty("role", "success")
        self.stop_button = QPushButton("关闭摄像头")
        self.stop_button.setProperty("role", "danger")
        self.snapshot_button = QPushButton("保存当前图片")
        self.snapshot_button.setProperty("role", "primary")
        header_layout.addWidget(self.start_button)
        header_layout.addWidget(self.stop_button)
        header_layout.addWidget(self.snapshot_button)
        root.addWidget(header)

        content = QHBoxLayout()
        content.setSpacing(10)
        content.addWidget(self._build_preview_card(), 7)
        content.addWidget(self._build_settings_scroll(), 4)
        root.addLayout(content, 1)

    def _build_preview_card(self) -> QFrame:
        """创建直接拉最终RTSP地址的实时画面与运行摘要。"""
        card = QFrame()
        card.setObjectName("previewCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        title_row = QHBoxLayout()
        title = QLabel("RTSP 实时画面")
        title.setStyleSheet("font-size: 11pt; font-weight: 700;")
        self.reconnect_button = QPushButton("重新连接预览")
        title_row.addWidget(title, 1)
        title_row.addWidget(self.reconnect_button)
        layout.addLayout(title_row)

        self.preview_stack = QStackedWidget()
        self.preview_stack.setObjectName("videoSurface")
        self.preview_stack.setMinimumSize(640, 360)
        self.preview_placeholder = QLabel("RTSP画面将在摄像头启动后显示")
        self.preview_placeholder.setObjectName("previewPlaceholder")
        self.preview_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_widget = QVideoWidget()
        self.video_widget.setObjectName("videoSurface")
        self.preview_stack.addWidget(self.preview_placeholder)
        self.preview_stack.addWidget(self.video_widget)
        layout.addWidget(self.preview_stack, 1)

        url_row = QHBoxLayout()
        url_label = QLabel("拉流地址")
        self.rtsp_url_display = QLineEdit()
        self.rtsp_url_display.setObjectName("rtspUrlDisplay")
        self.rtsp_url_display.setReadOnly(True)
        self.copy_url_button = QPushButton("复制")
        url_row.addWidget(url_label)
        url_row.addWidget(self.rtsp_url_display, 1)
        url_row.addWidget(self.copy_url_button)
        layout.addLayout(url_row)

        metrics = QGridLayout()
        metrics.setHorizontalSpacing(8)
        metrics.setVerticalSpacing(5)
        self.recording_value = QLabel("—")
        self.elapsed_value = QLabel("00:00:00")
        self.frame_value = QLabel("0")
        self.fps_value = QLabel("0.0 fps")
        for column, (label, widget) in enumerate(
            (
                ("当前录像", self.recording_value),
                ("录制时长", self.elapsed_value),
                ("已采集帧", self.frame_value),
                ("实测平均", self.fps_value),
            )
        ):
            heading = QLabel(label)
            heading.setObjectName("mutedLabel")
            widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            metrics.addWidget(heading, 0, column)
            metrics.addWidget(widget, 1, column)
        metrics.setColumnStretch(0, 4)
        for column in range(1, 4):
            metrics.setColumnStretch(column, 1)
        layout.addLayout(metrics)
        return card

    def _build_settings_scroll(self) -> QScrollArea:
        """创建设备、地址、画质和存储四组配置。"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form_widget = QWidget()
        layout = QVBoxLayout(form_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        device_group = QGroupBox("摄像头设备")
        device_layout = QGridLayout(device_group)
        device_layout.setHorizontalSpacing(8)
        device_layout.setColumnStretch(0, 0)
        device_layout.setColumnStretch(1, 1)
        device_layout.setColumnStretch(2, 0)
        self.device_combo = QComboBox()
        self.device_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.device_combo.setMinimumContentsLength(10)
        self.device_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        self.refresh_devices_button = QPushButton("检测设备")
        self.refresh_devices_button.setFixedWidth(88)
        self.device_label = QLabel("设备")
        device_layout.addWidget(self.device_label, 0, 0)
        device_layout.addWidget(self.device_combo, 0, 1)
        device_layout.addWidget(self.refresh_devices_button, 0, 2)
        self.device_path_label = QLabel("—")
        self.device_path_label.setObjectName("mutedLabel")
        self.device_path_label.setWordWrap(True)
        device_layout.addWidget(self.device_path_label, 1, 1, 1, 2)
        layout.addWidget(device_group)

        address_group = QGroupBox("RTSP 推流与拉流地址")
        address_layout = QGridLayout(address_group)
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("例如 192.168.1.10")
        self.port_input = QSpinBox()
        self.port_input.setRange(1024, 65535)
        self.path_input = QLineEdit()
        self.path_input.setPlaceholderText("camera")
        address_layout.addWidget(QLabel("局域网 IP"), 0, 0)
        address_layout.addWidget(self.ip_input, 0, 1)
        address_layout.addWidget(QLabel("端口"), 1, 0)
        address_layout.addWidget(self.port_input, 1, 1)
        address_layout.addWidget(QLabel("地址路径"), 2, 0)
        address_layout.addWidget(self.path_input, 2, 1)
        layout.addWidget(address_group)

        quality_group = QGroupBox("推流画质与编码")
        quality_layout = QGridLayout(quality_group)
        self.codec_combo = QComboBox()
        for codec, label in CODEC_LABELS.items():
            self.codec_combo.addItem(label, codec)
        self.mode_combo = QComboBox()
        self.mode_combo.setMinimumContentsLength(20)
        quality_layout.addWidget(QLabel("摄像头编码"), 0, 0)
        quality_layout.addWidget(self.codec_combo, 0, 1)
        quality_layout.addWidget(QLabel("分辨率 / 帧率"), 1, 0)
        quality_layout.addWidget(self.mode_combo, 1, 1)
        layout.addWidget(quality_group)

        storage_group = QGroupBox("录像与图片保存")
        storage_layout = QGridLayout(storage_group)
        self.container_combo = QComboBox()
        for container_name, label in CONTAINER_LABELS.items():
            self.container_combo.addItem(label, container_name)
        self.video_path_input = QLineEdit()
        self.image_path_input = QLineEdit()
        self.video_browse_button = QPushButton("选择")
        self.image_browse_button = QPushButton("选择")
        storage_layout.addWidget(QLabel("录像格式"), 0, 0)
        storage_layout.addWidget(self.container_combo, 0, 1, 1, 2)
        storage_layout.addWidget(QLabel("视频路径"), 1, 0)
        storage_layout.addWidget(self.video_path_input, 1, 1)
        storage_layout.addWidget(self.video_browse_button, 1, 2)
        storage_layout.addWidget(QLabel("图片路径"), 2, 0)
        storage_layout.addWidget(self.image_path_input, 2, 1)
        storage_layout.addWidget(self.image_browse_button, 2, 2)
        layout.addWidget(storage_group)

        self.apply_button = QPushButton("保存配置")
        self.apply_button.setProperty("role", "primary")
        layout.addWidget(self.apply_button)
        self.operation_message = QLabel("正在连接独立摄像头后台服务…")
        self.operation_message.setObjectName("mutedLabel")
        self.operation_message.setWordWrap(True)
        layout.addWidget(self.operation_message)
        layout.addStretch(1)
        scroll.setWidget(form_widget)
        return scroll

    def _connect_signals(self) -> None:
        """连接配置变化、后台动作和预览控制。"""
        self.start_button.clicked.connect(self._start_camera)
        self.stop_button.clicked.connect(self._stop_camera)
        self.snapshot_button.clicked.connect(self._save_snapshot)
        self.apply_button.clicked.connect(self._save_configuration)
        self.refresh_devices_button.clicked.connect(self._request_probe)
        self.device_combo.currentIndexChanged.connect(self._device_changed)
        self.codec_combo.currentIndexChanged.connect(self._filter_modes)
        self.reconnect_button.clicked.connect(self._reconnect_preview)
        self.copy_url_button.clicked.connect(self._copy_rtsp_url)
        self.video_browse_button.clicked.connect(
            lambda: self._choose_directory(self.video_path_input, "选择视频保存路径")
        )
        self.image_browse_button.clicked.connect(
            lambda: self._choose_directory(self.image_path_input, "选择图片保存路径")
        )
        self.ip_input.textChanged.connect(self._update_rtsp_url_from_fields)
        self.port_input.valueChanged.connect(self._update_rtsp_url_from_fields)
        self.path_input.textChanged.connect(self._update_rtsp_url_from_fields)

    def _configure_player(self) -> None:
        """使用PySide6自带FFmpeg后端拉取最终RTSP流。"""
        self.player = QMediaPlayer(self)
        self.player.setVideoOutput(self.video_widget)
        self.player.errorOccurred.connect(self._player_error)
        self.player.mediaStatusChanged.connect(self._player_media_status_changed)

    def _dispose_player(self) -> None:
        """彻底卸载当前媒体源和输出，避免同地址重连复用旧RTSP会话。"""
        player = self.player
        for signal, slot in (
            (player.errorOccurred, self._player_error),
            (player.mediaStatusChanged, self._player_media_status_changed),
        ):
            try:
                signal.disconnect(slot)
            except (RuntimeError, TypeError):
                pass
        player.stop()
        player.setSource(QUrl())
        player.setVideoOutput(None)
        player.deleteLater()

    def _recreate_player(self) -> None:
        """为下一次RTSP连接创建全新的Qt/FFmpeg播放后端。"""
        self._dispose_player()
        self._configure_player()

    def _bootstrap_service(self) -> None:
        """优先连接已有后台；不存在时独立拉起且不绑定面板生命周期。"""
        self._submit(
            "bootstrap-status",
            lambda: self.client.request("status", timeout=0.5),
        )

    def _launch_service(self) -> None:
        """通过当前项目Python分离启动后台，面板关闭后服务继续运行。"""
        started, _pid = QProcess.startDetached(
            sys.executable,
            [str(SERVICE_SCRIPT), "serve"],
            str(VIDEO_SERVICE_ROOT),
        )
        if not started:
            self._set_service_failure("无法启动独立摄像头后台服务")
            return
        self.operation_message.setText("后台已拉起，正在等待控制Socket…")
        self._submit("bootstrap-wait", lambda: self.client.wait_until_ready(5.0))

    def _submit(self, name: str, operation: Callable[[], dict[str, Any]]) -> None:
        """在短生命周期Python线程中执行可能阻塞的本机Socket请求。"""
        if name in self._request_in_flight:
            return
        self._request_in_flight.add(name)

        def worker() -> None:
            try:
                result = operation()
                error = ""
            except Exception as exc:
                result = {}
                error = str(exc)
            self._bridge.completed.emit(name, result, error)

        threading.Thread(
            target=worker,
            name=f"camera-panel-{name}",
            daemon=True,
        ).start()

    def _on_request_completed(
        self, name: str, result: object, error: str
    ) -> None:
        """统一消费后台结果并刷新控件，不从工作线程操作Qt。"""
        self._request_in_flight.discard(name)
        data = result if isinstance(result, dict) else {}
        if name == "bootstrap-status" and error:
            self._launch_service()
            return
        if name.startswith("bootstrap"):
            if error:
                self._set_service_failure(error)
                return
            self._service_ready = True
            self.operation_message.setText("独立摄像头后台服务已连接。")
            self._apply_status(data)
            self._request_probe()
            return
        if name == "status":
            if error:
                self._set_service_failure(error)
            else:
                self._service_ready = True
                self._apply_status(data)
            return
        if name == "probe":
            if error:
                self.operation_message.setText(f"设备检测失败：{error}")
            else:
                self._apply_probe(data)
            self._refresh_controls()
            return

        self._action_busy = False
        if name == "stop":
            self._stopping_camera = False
        if error:
            self.operation_message.setText(error)
            self._show_error("摄像头操作失败", error)
        else:
            if name in {"start", "stop"}:
                self._apply_status(data)
            if name == "configure":
                self.operation_message.setText("摄像头配置已保存。")
            elif name == "start":
                self.operation_message.clear()
            elif name == "stop":
                self.operation_message.setText("摄像头已关闭，录像已安全封装。")
            elif name == "snapshot":
                path = str(data.get("path", ""))
                self.operation_message.setText(f"JPG已保存：{path}")
        if self._service_ready:
            QTimer.singleShot(0, self._poll_status)
        self._refresh_controls()

    def _poll_status(self) -> None:
        """每秒异步读取一次服务状态，后台离线不阻塞界面。"""
        if not self._service_ready or "status" in self._request_in_flight:
            return
        self._submit("status", lambda: self.client.request("status", timeout=0.7))

    def _request_probe(self) -> None:
        """按当前设备刷新真实V4L2模式，而不是使用硬编码列表。"""
        if not self._service_ready:
            return
        device = self.device_combo.currentData() or self._configured_device()
        self.operation_message.setText("正在读取摄像头支持的格式、分辨率和帧率…")
        self._submit(
            "probe",
            lambda: self.client.request(
                "probe", {"device": str(device)}, timeout=6.0
            ),
        )
        # 探测结束前不能提交旧设备的模式，避免双摄像头切换时形成混合配置。
        self._refresh_controls()

    def _apply_probe(self, data: dict[str, Any]) -> None:
        """更新设备和模式，并保留本次实际探测的设备选择。"""
        configured = self._last_status.get("config", {})
        configured_device = str(configured.get("device", self._configured_device()))
        current_device = str(self.device_combo.currentData() or "")
        selected_device = str(
            data.get("selected_device") or current_device or configured_device
        )
        devices = data.get("devices", [])
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        for item in devices if isinstance(devices, list) else []:
            if isinstance(item, dict):
                label = str(item.get("label", "摄像头"))
                self.device_combo.addItem(label, item.get("path"))
        if self.device_combo.count() == 0:
            self.device_combo.addItem(selected_device, selected_device)
        selected_index = self.device_combo.findData(selected_device)
        if selected_index < 0:
            # 设备可能在枚举和能力读取之间拔出；保留路径以展示真实探测错误。
            self.device_combo.addItem(selected_device, selected_device)
            selected_index = self.device_combo.count() - 1
        self.device_combo.setCurrentIndex(selected_index)
        self.device_combo.blockSignals(False)
        self.device_path_label.setText(str(self.device_combo.currentData() or "—"))

        raw_modes = data.get("modes", [])
        self._all_modes = [item for item in raw_modes if isinstance(item, dict)]
        probe_error = str(data.get("error", ""))
        if probe_error:
            self.operation_message.setText(f"设备能力读取失败：{probe_error}")
        else:
            self.operation_message.setText(
                f"检测到 {len(self._all_modes)} 个离散视频模式。"
            )
        self._filter_modes()

    def _filter_modes(self) -> None:
        """编码选择变化时只显示该摄像头真正支持的画质组合。"""
        codec = str(self.codec_combo.currentData() or "h264")
        configured = self._last_status.get("config", {})
        target = (
            int(configured.get("width", 1920)),
            int(configured.get("height", 1080)),
            float(configured.get("fps", 30.0)),
        )
        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        selected_index = -1
        for item in self._all_modes:
            if item.get("codec") != codec:
                continue
            width = int(item.get("width", 0))
            height = int(item.get("height", 0))
            fps = float(item.get("fps", 0.0))
            label = f"{width}×{height}  ·  {format_fps(fps)} fps"
            self.mode_combo.addItem(label, item)
            if (width, height, fps) == target:
                selected_index = self.mode_combo.count() - 1
        self.mode_combo.setCurrentIndex(selected_index if selected_index >= 0 else 0)
        self.mode_combo.blockSignals(False)
        self._refresh_controls()

    def _apply_status(self, status: dict[str, Any]) -> None:
        """刷新运行状态、进度、预览和初次配置，不打断用户正在编辑。"""
        self._last_status = status
        config = status.get("config", {})
        if isinstance(config, dict) and not self._config_loaded:
            self._populate_configuration(config)
            self._config_loaded = True

        state = str(status.get("state", "stopped"))
        running = bool(status.get("running", False))

        wall_elapsed = float(status.get("elapsed_seconds", 0.0) or 0.0)
        media_elapsed = float(status.get("media_elapsed_seconds", 0.0) or 0.0)
        elapsed = media_elapsed if media_elapsed > 0.0 else wall_elapsed
        self.elapsed_value.setText(self._format_elapsed(elapsed))
        self.frame_value.setText(str(int(status.get("frame", 0) or 0)))
        measured_fps = float(status.get("measured_fps", 0.0) or 0.0)
        self.fps_value.setText(f"{measured_fps:.1f} fps")
        recording = str(status.get("recording_file", ""))
        self.recording_value.setText(Path(recording).name if recording else "—")
        self.recording_value.setToolTip(recording)
        url = str(status.get("rtsp_url", ""))
        if url:
            self.rtsp_url_display.setText(url)
        if running and not self._stopping_camera:
            self._ensure_preview(url)
        else:
            self._stop_preview("等待摄像头启动")
        error = str(status.get("last_error", ""))
        if error and state == "error":
            self.operation_message.setText(error)
        self._refresh_controls()

    def _populate_configuration(self, config: dict[str, Any]) -> None:
        """把服务端权威配置写入表单一次，后续轮询不覆盖用户编辑。"""
        self.ip_input.setText(str(config.get("rtsp_ip", "127.0.0.1")))
        self.port_input.setValue(int(config.get("rtsp_port", 8554)))
        self.path_input.setText(str(config.get("rtsp_path", "camera")))
        self.video_path_input.setText(str(config.get("video_directory", "")))
        self.image_path_input.setText(str(config.get("image_directory", "")))
        codec_index = self.codec_combo.findData(str(config.get("codec", "h264")))
        self.codec_combo.setCurrentIndex(max(0, codec_index))
        container_index = self.container_combo.findData(
            str(config.get("container", "mp4"))
        )
        self.container_combo.setCurrentIndex(max(0, container_index))
        device = str(config.get("device", ""))
        self.device_combo.addItem(device, device)
        self.device_combo.setCurrentIndex(0)
        self.device_path_label.setText(device)
        self._update_rtsp_url_from_fields()

    def _configuration_from_fields(self) -> dict[str, Any]:
        """收集完整配置；具体范围和硬件支持仍由后台最终校验。"""
        mode = self.mode_combo.currentData()
        if not isinstance(mode, dict):
            raise CameraServiceError("请先检测并选择摄像头画质")
        return {
            "device": str(self.device_combo.currentData() or ""),
            "codec": str(self.codec_combo.currentData()),
            "width": int(mode.get("width", 0)),
            "height": int(mode.get("height", 0)),
            "fps": float(mode.get("fps", 0.0)),
            "rtsp_ip": self.ip_input.text().strip(),
            "rtsp_port": self.port_input.value(),
            "rtsp_path": self.path_input.text().strip().strip("/"),
            "container": str(self.container_combo.currentData()),
            "video_directory": self.video_path_input.text().strip(),
            "image_directory": self.image_path_input.text().strip(),
        }

    def _save_configuration(self) -> None:
        """异步保存配置，不自动重启或中断摄像头。"""
        try:
            config = self._configuration_from_fields()
        except CameraServiceError as exc:
            self._show_error("无法保存配置", str(exc))
            return
        self._begin_action("正在保存配置…")
        self._submit(
            "configure",
            lambda: self.client.request(
                "configure", {"config": config}, timeout=5.0
            ),
        )

    def _start_camera(self) -> None:
        """把当前表单作为完整配置交给独立后台启动。"""
        try:
            config = self._configuration_from_fields()
        except CameraServiceError as exc:
            self._show_error("无法启动摄像头", str(exc))
            return
        self._begin_action("正在启动MediaMTX、摄像头与录像…")
        self._submit(
            "start",
            lambda: self.client.request(
                "start", {"config": config}, timeout=25.0
            ),
        )

    def _stop_camera(self) -> None:
        """请求后台安全封装录像；不关闭地面站或本面板。"""
        self._stopping_camera = True
        self._stop_preview("等待摄像头启动")
        self._begin_action("正在安全关闭摄像头并封装录像…")
        self._submit(
            "stop", lambda: self.client.request("stop", timeout=20.0)
        )

    def _save_snapshot(self) -> None:
        """通过与SIGUSR1相同的后台截图路径保存当前JPG。"""
        self._begin_action("正在从RTSP保存当前图片…")
        self._submit(
            "snapshot", lambda: self.client.request("snapshot", timeout=10.0)
        )

    def _begin_action(self, message: str) -> None:
        """锁定会改变服务状态的按钮，并保留预览和地面站响应。"""
        self._action_busy = True
        self.operation_message.setText(message)
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        """按后台状态统一门控表单与启停按钮。"""
        state = str(self._last_status.get("state", "stopped"))
        running = bool(self._last_status.get("running", False))
        editable = self._service_ready and state == "stopped"
        editable = editable and not self._action_busy
        editable = editable and "probe" not in self._request_in_flight
        mode_ready = self.mode_combo.count() > 0

        for widget in (
            self.device_combo,
            self.refresh_devices_button,
            self.ip_input,
            self.port_input,
            self.path_input,
            self.codec_combo,
            self.mode_combo,
            self.container_combo,
            self.video_path_input,
            self.image_path_input,
            self.video_browse_button,
            self.image_browse_button,
        ):
            widget.setEnabled(editable)
        self.apply_button.setEnabled(editable and mode_ready)
        self.start_button.setEnabled(editable and mode_ready and not running)
        self.stop_button.setEnabled(
            self._service_ready
            and state in {"starting", "running", "error"}
            and not self._action_busy
        )
        self.snapshot_button.setEnabled(running and not self._action_busy)
        self.reconnect_button.setEnabled(running)

    def _ensure_preview(self, url: str) -> None:
        """仅在服务运行且地址变化时创建RTSP读取会话。"""
        if not url:
            return
        if (
            self._active_preview_url == url
            and self.player.source().toString() == url
        ):
            return
        if self.player.source().isValid():
            self._recreate_player()
        self._active_preview_url = url
        self.preview_placeholder.setText("正在连接RTSP…")
        self.preview_stack.setCurrentWidget(self.preview_placeholder)
        self.player.setSource(QUrl(url))
        self.player.play()

    def _reconnect_preview(self) -> None:
        """只重建本面板RTSP客户端，不触碰摄像头或录像进程。"""
        url = str(self._last_status.get("rtsp_url", ""))
        url = url or self._active_preview_url
        self._recreate_player()
        self._active_preview_url = ""
        self._ensure_preview(url)

    def _stop_preview(self, message: str) -> None:
        """卸载本次解码会话并预建新播放器，摄像头后台不受影响。"""
        had_session = bool(self._active_preview_url) or self.player.source().isValid()
        if had_session:
            self._recreate_player()
        else:
            self.player.stop()
            self.player.setSource(QUrl())
        self._active_preview_url = ""
        self.preview_placeholder.setText(message)
        self.preview_stack.setCurrentWidget(self.preview_placeholder)

    def _player_error(
        self, _error: QMediaPlayer.Error, message: str
    ) -> None:
        """预览错误仅显示在面板，不改变推流和录像状态。"""
        self.preview_placeholder.setText(
            f"RTSP预览暂不可用\n{message}\n推流和录像后台不受影响"
        )
        self.preview_stack.setCurrentWidget(self.preview_placeholder)

    def _player_media_status_changed(self, status: QMediaPlayer.MediaStatus) -> None:
        """媒体缓冲完成后切换到画面，否则保留可读占位提示。"""
        if status in {
            QMediaPlayer.MediaStatus.LoadedMedia,
            QMediaPlayer.MediaStatus.BufferedMedia,
        }:
            self.preview_stack.setCurrentWidget(self.video_widget)
        elif status == QMediaPlayer.MediaStatus.LoadingMedia:
            self.preview_placeholder.setText("正在连接RTSP…")
            self.preview_stack.setCurrentWidget(self.preview_placeholder)

    def _device_changed(self) -> None:
        """设备变化后更新完整路径并重新读取该节点能力。"""
        self.device_path_label.setText(str(self.device_combo.currentData() or "—"))
        if self._config_loaded:
            self._request_probe()

    def _update_rtsp_url_from_fields(self) -> None:
        """编辑IP、端口或路径时即时显示最终拉流地址。"""
        ip = self.ip_input.text().strip() or "127.0.0.1"
        path = self.path_input.text().strip().strip("/") or "camera"
        self.rtsp_url_display.setText(f"rtsp://{ip}:{self.port_input.value()}/{path}")

    def _copy_rtsp_url(self) -> None:
        """复制完整RTSP地址供局域网客户端使用。"""
        clipboard = QApplication.clipboard()
        clipboard.setText(self.rtsp_url_display.text())
        self.statusBar().showMessage("RTSP拉流地址已复制", 2500)

    def _choose_directory(self, target: QLineEdit, title: str) -> None:
        """用原生目录选择器更新保存路径，不创建或清空已有目录。"""
        initial = target.text().strip() or str(Path.home())
        selected = QFileDialog.getExistingDirectory(self, title, initial)
        if selected:
            target.setText(selected)

    def _configured_device(self) -> str:
        """在首轮设备枚举前返回服务端或默认设备路径。"""
        config = self._last_status.get("config", {})
        if isinstance(config, dict) and config.get("device"):
            return str(config["device"])
        return CameraConfig.defaults().device

    def _set_service_failure(self, message: str) -> None:
        """标记后台离线；地面站和当前窗口本身继续可用。"""
        self._service_ready = False
        self.operation_message.setText(message)
        self._stop_preview("摄像头后台服务不可用")
        self._refresh_controls()

    def _show_error(self, title: str, message: str) -> None:
        """显示继承面板主题的本地错误框，不调用地面站模态流程。"""
        dialog = QMessageBox(self)
        dialog.setWindowTitle(title)
        dialog.setIcon(QMessageBox.Icon.Warning)
        dialog.setText(message)
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.exec()

    @staticmethod
    def _format_elapsed(seconds: float) -> str:
        """把录像时长格式化为固定宽度时分秒。"""
        total = max(0, int(seconds))
        hours, remainder = divmod(total, 3600)
        minutes, secs = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        """关闭面板只停止预览和轮询，独立推流/录像保持运行。"""
        self._poll_timer.stop()
        self._dispose_player()
        event.accept()
