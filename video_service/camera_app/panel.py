"""独立摄像头面板：三种视频源共用手填 RTSP 预览器。"""

from __future__ import annotations

import sys
import threading
import time
from collections.abc import Callable
from enum import Enum
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from PySide6.QtCore import QObject, QProcess, QTimer, QUrl, Qt, Signal
from PySide6.QtGui import (
    QColor,
    QIcon,
    QKeySequence,
    QPainter,
    QPalette,
    QPen,
    QPixmap,
    QShortcut,
)
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
    QStyle,
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
# 播放中超过该时长没有有效帧就遮住原生视频 surface，避免显示陈旧或透底内容。
PREVIEW_FRAME_TIMEOUT_SECONDS = 1.5


class SourceMode(str, Enum):
    """面板命令目标；预览器始终只读取地址框中的 RTSP。"""

    LOCAL = "local"
    ONBOARD = "onboard"
    EXTERNAL = "external"


class NoWheelComboBox(QComboBox):
    """忽略滚轮，防止滚动设置页时误改枚举值。"""

    def wheelEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        event.ignore()


class NoWheelSpinBox(QSpinBox):
    """忽略滚轮，端口只能通过键盘或步进按钮修改。"""

    def wheelEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        event.ignore()


class OpaqueBlackPreview(QLabel):
    """无视频时逐像素绘制纯黑，避免样式或原生子窗口留下透明孔洞。"""

    def __init__(self) -> None:
        super().__init__("")
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)

    def paintEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        """始终覆盖整个脏区；预览状态文字只允许显示在黑区之外。"""
        painter = QPainter(self)
        painter.fillRect(event.rect(), Qt.GlobalColor.black)


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
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 4px; }
QLineEdit, QComboBox, QSpinBox {
    min-height: 30px;
    padding: 2px 7px;
    background: #ffffff;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
    selection-background-color: #245f87;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus { border: 2px solid #245f87; }
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
QPushButton[role="primary"]:hover { background: #194d70; border-color: #194d70; }
QPushButton[role="success"] {
    color: white; background: #247457; border-color: #247457;
}
QPushButton[role="success"]:hover { background: #1b5c44; border-color: #1b5c44; }
QPushButton[role="danger"] {
    color: white; background: #a7352a; border-color: #a7352a;
}
QPushButton[role="danger"]:hover { background: #87271f; border-color: #87271f; }
QPushButton[role="primary"]:disabled,
QPushButton[role="success"]:disabled,
QPushButton[role="danger"]:disabled {
    color: #98a4b1; background: #edf0f2; border-color: #dbe0e5;
}
QPushButton#iconButton { min-width: 34px; max-width: 34px; padding: 2px; }
QWidget#videoSurface, QLabel#previewPlaceholder { background: #000000; }
QLabel#previewPlaceholder {
    color: #000000;
    qproperty-alignment: AlignCenter;
}
QLineEdit#rtspUrlInput {
    font-family: "DejaVu Sans Mono", monospace;
    color: #245f87;
    background: #f5f8fa;
}
QScrollArea { border: none; background: transparent; }
QStatusBar { background: #ffffff; border-top: 1px solid #cfd6de; }
"""


class _PanelBridge(QObject):
    """把后台 Socket/ROS 请求结果安全投递到 Qt 主线程。"""

    completed = Signal(str, object, str)


class CameraPanelWindow(QMainWindow):
    """本机、真机或指定 RTSP 的配置、命令和独立预览窗口。"""

    def __init__(
        self,
        *,
        client: CameraServiceClient | None = None,
        onboard_client: Any | None = None,
        auto_bootstrap: bool = True,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.client = client or CameraServiceClient()
        self._onboard_client = onboard_client
        self._bridge = _PanelBridge(self)
        self._bridge.completed.connect(self._on_request_completed)
        self._request_in_flight: set[str] = set()
        self._service_ready = False
        self._config_loaded = False
        self._last_status: dict[str, Any] = {}
        self._onboard_status: dict[str, Any] = {}
        self._all_modes: list[dict[str, Any]] = []
        self._active_preview_url = ""
        self._url_user_edited = False
        self._action_busy = False
        self._stopping_camera = False
        self._clock: Callable[[], float] = time.monotonic
        self._played_seconds = 0.0
        self._playing_since: float | None = None
        self._preview_frame_count = 0
        self._last_preview_frame_at: float | None = None

        self.setWindowTitle(DESKTOP_APPLICATION_NAME)
        self.setMinimumSize(980, 680)
        self.resize(1180, 780)
        self._build_ui()
        self._connect_signals()
        self._configure_player()
        self._refresh_controls()

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(1000)
        self._poll_timer.timeout.connect(self._poll_all_status)
        self._poll_timer.start()
        self._metrics_timer = QTimer(self)
        self._metrics_timer.setInterval(250)
        self._metrics_timer.timeout.connect(self._refresh_playback_metrics)
        self._metrics_timer.start()
        if auto_bootstrap:
            QTimer.singleShot(0, self._bootstrap_service)

    @property
    def source_mode(self) -> SourceMode:
        """返回当前命令目标，不把它与播放器状态混为一谈。"""
        return SourceMode(str(self.source_combo.currentData()))

    def _build_ui(self) -> None:
        """构建顶部命令、左侧预览和右侧本机发布配置。"""
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
        self.start_button = QPushButton("开启本机摄像头")
        self.start_button.setProperty("role", "success")
        self.stop_button = QPushButton("关闭本机摄像头")
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
        """创建只由地址框和播放键驱动的 RTSP 实时画面。"""
        card = QFrame()
        card.setObjectName("previewCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        title = QLabel("RTSP 实时画面")
        title.setStyleSheet("font-size: 11pt; font-weight: 700;")
        layout.addWidget(title)

        self.preview_stack = QStackedWidget()
        self.preview_stack.setObjectName("videoSurface")
        self.preview_stack.setMinimumSize(640, 360)
        # QVideoWidget 内部包含原生 QWindow；没有有效帧时始终用普通、
        # 不透明的黑色 QWidget 完整遮住它，提示信息放到预览区外。
        black_palette = self.preview_stack.palette()
        black_palette.setColor(QPalette.ColorRole.Window, QColor("#000000"))
        self.preview_stack.setPalette(black_palette)
        self.preview_stack.setAutoFillBackground(True)
        self.preview_placeholder = OpaqueBlackPreview()
        self.preview_placeholder.setObjectName("previewPlaceholder")
        self.preview_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_placeholder.setPalette(black_palette)
        self.video_widget = QVideoWidget()
        self.video_widget.setObjectName("videoSurface")
        self.preview_stack.addWidget(self.preview_placeholder)
        self.preview_stack.addWidget(self.video_widget)
        layout.addWidget(self.preview_stack, 1)

        url_row = QHBoxLayout()
        url_row.addWidget(QLabel("拉流地址"))
        self.rtsp_url_input = QLineEdit()
        self.rtsp_url_input.setObjectName("rtspUrlInput")
        self.rtsp_url_input.setPlaceholderText("rtsp://主机:端口/路径")
        self.play_pause_button = QPushButton()
        self.play_pause_button.setObjectName("iconButton")
        self.play_pause_button.setToolTip("播放 RTSP（空格键）")
        self.play_pause_button.setAccessibleName("播放或暂停 RTSP")
        self.copy_url_button = QPushButton()
        self.copy_url_button.setObjectName("iconButton")
        self.copy_url_button.setIcon(self._copy_icon())
        self.copy_url_button.setToolTip("复制 RTSP 拉流地址")
        self.copy_url_button.setAccessibleName("复制 RTSP 拉流地址")
        url_row.addWidget(self.rtsp_url_input, 1)
        url_row.addWidget(self.play_pause_button)
        url_row.addWidget(self.copy_url_button)
        layout.addLayout(url_row)

        metrics = QGridLayout()
        metrics.setHorizontalSpacing(18)
        self.elapsed_value = QLabel("00:00:00")
        self.fps_value = QLabel("0.0 fps")
        for column, (label, widget) in enumerate(
            (("播放时长", self.elapsed_value), ("平均帧数", self.fps_value))
        ):
            heading = QLabel(label)
            heading.setObjectName("mutedLabel")
            widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            metrics.addWidget(heading, 0, column)
            metrics.addWidget(widget, 1, column)
            metrics.setColumnStretch(column, 1)
        layout.addLayout(metrics)
        return card

    def _build_settings_scroll(self) -> QScrollArea:
        """创建视频源选择及仅本机模式可编辑的发布配置。"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form_widget = QWidget()
        layout = QVBoxLayout(form_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        source_group = QGroupBox("视频源模式")
        source_layout = QGridLayout(source_group)
        self.source_combo = NoWheelComboBox()
        self.source_combo.addItem("本机摄像头", SourceMode.LOCAL.value)
        self.source_combo.addItem("真机摄像头", SourceMode.ONBOARD.value)
        self.source_combo.addItem("指定 RTSP", SourceMode.EXTERNAL.value)
        self.fetch_onboard_url_button = QPushButton("读取真机地址")
        self.source_status_label = QLabel("本机后台状态待获取")
        self.source_status_label.setObjectName("mutedLabel")
        self.source_status_label.setWordWrap(True)
        source_layout.addWidget(self.source_combo, 0, 0)
        source_layout.addWidget(self.fetch_onboard_url_button, 0, 1)
        source_layout.addWidget(self.source_status_label, 1, 0, 1, 2)
        layout.addWidget(source_group)

        self.device_group = QGroupBox("摄像头设备")
        device_layout = QGridLayout(self.device_group)
        device_layout.setHorizontalSpacing(8)
        device_layout.setColumnStretch(1, 1)
        self.device_combo = NoWheelComboBox()
        self.device_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.device_combo.setMinimumContentsLength(10)
        self.device_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
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
        layout.addWidget(self.device_group)

        self.address_group = QGroupBox("RTSP 推流地址")
        address_layout = QGridLayout(self.address_group)
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("例如 192.168.1.10")
        self.port_input = NoWheelSpinBox()
        self.port_input.setRange(1024, 65535)
        self.path_input = QLineEdit()
        self.path_input.setPlaceholderText("camera")
        address_layout.addWidget(QLabel("对外展示 IP"), 0, 0)
        address_layout.addWidget(self.ip_input, 0, 1)
        address_layout.addWidget(QLabel("发布端口"), 1, 0)
        address_layout.addWidget(self.port_input, 1, 1)
        address_layout.addWidget(QLabel("发布路径"), 2, 0)
        address_layout.addWidget(self.path_input, 2, 1)
        layout.addWidget(self.address_group)

        self.quality_group = QGroupBox("推流画质与编码")
        quality_layout = QGridLayout(self.quality_group)
        self.codec_combo = NoWheelComboBox()
        for codec, label in CODEC_LABELS.items():
            self.codec_combo.addItem(label, codec)
        self.mode_combo = NoWheelComboBox()
        self.mode_combo.setMinimumContentsLength(20)
        quality_layout.addWidget(QLabel("摄像头编码"), 0, 0)
        quality_layout.addWidget(self.codec_combo, 0, 1)
        quality_layout.addWidget(QLabel("分辨率 / 帧率"), 1, 0)
        quality_layout.addWidget(self.mode_combo, 1, 1)
        layout.addWidget(self.quality_group)

        self.storage_group = QGroupBox("录像与图片保存")
        storage_layout = QGridLayout(self.storage_group)
        self.container_combo = NoWheelComboBox()
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
        layout.addWidget(self.storage_group)

        self.apply_button = QPushButton("保存本机配置")
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
        """连接三模式命令、后台动作和独立播放器控制。"""
        self.start_button.clicked.connect(self._start_camera)
        self.stop_button.clicked.connect(self._stop_camera)
        self.snapshot_button.clicked.connect(self._save_snapshot)
        self.apply_button.clicked.connect(self._save_configuration)
        self.refresh_devices_button.clicked.connect(self._request_probe)
        self.device_combo.currentIndexChanged.connect(self._device_changed)
        self.codec_combo.currentIndexChanged.connect(self._filter_modes)
        self.source_combo.currentIndexChanged.connect(self._source_changed)
        self.fetch_onboard_url_button.clicked.connect(self._fill_onboard_url)
        self.rtsp_url_input.textEdited.connect(self._mark_url_edited)
        self.play_pause_button.clicked.connect(self._toggle_preview)
        self.copy_url_button.clicked.connect(self._copy_rtsp_url)
        self.video_browse_button.clicked.connect(
            lambda: self._choose_directory(self.video_path_input, "选择视频保存路径")
        )
        self.image_browse_button.clicked.connect(
            lambda: self._choose_directory(self.image_path_input, "选择图片保存路径")
        )
        self.preview_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.preview_shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
        self.preview_shortcut.activated.connect(self._toggle_preview)

    def _configure_player(self) -> None:
        """使用 PySide6 FFmpeg 后端拉流，并只统计实际播放帧。"""
        self.player = QMediaPlayer(self)
        self.player.setVideoOutput(self.video_widget)
        self.player.errorOccurred.connect(self._player_error)
        self.player.mediaStatusChanged.connect(self._player_media_status_changed)
        self.player.playbackStateChanged.connect(self._player_state_changed)
        self._sync_play_button()
        if not hasattr(self, "video_sink"):
            self.video_sink = self.video_widget.videoSink()
            self.video_sink.videoFrameChanged.connect(self._count_video_frame)

    def _dispose_player(self) -> None:
        """卸载媒体源和输出，保证换地址后不复用旧 RTSP 会话。"""
        player = self.player
        for signal, slot in (
            (player.errorOccurred, self._player_error),
            (player.mediaStatusChanged, self._player_media_status_changed),
            (player.playbackStateChanged, self._player_state_changed),
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
        """为下一条地址创建全新的 Qt/FFmpeg 播放后端。"""
        self._dispose_player()
        self._configure_player()

    def _bootstrap_service(self) -> None:
        """连接或分离启动本机后台；不影响真机/外部 RTSP 预览。"""
        self._submit("bootstrap-status", lambda: self.client.request("status", timeout=0.5))

    def _launch_service(self) -> None:
        """用项目 Python 分离启动本机后台，面板关闭后后台继续运行。"""
        started, _pid = QProcess.startDetached(
            sys.executable,
            [str(SERVICE_SCRIPT), "serve"],
            str(VIDEO_SERVICE_ROOT),
        )
        if not started:
            self._set_service_failure("无法启动独立摄像头后台服务")
            return
        self.operation_message.setText("本机后台已拉起，正在等待控制 Socket…")
        self._submit("bootstrap-wait", lambda: self.client.wait_until_ready(5.0))

    def _submit(self, name: str, operation: Callable[[], dict[str, Any]]) -> None:
        """在短生命周期线程中执行可能阻塞的本机 Socket 请求。"""
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

        threading.Thread(target=worker, name=f"camera-panel-{name}", daemon=True).start()

    def _on_request_completed(self, name: str, result: object, error: str) -> None:
        """统一消费 Socket/ROS 结果并在 Qt 主线程刷新控件。"""
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
            self.operation_message.setText("本机摄像头后台服务已连接。")
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
            title = (
                "真机摄像头操作失败"
                if name in {"onboard-start", "onboard-stop", "onboard-snapshot"}
                else "摄像头操作失败"
            )
            self._show_error(title, error)
        else:
            if name in {"start", "stop"}:
                self._apply_status(data)
            messages = {
                "configure": "本机摄像头配置已保存。",
                "start": "本机摄像头已启动。",
                "stop": "本机摄像头已关闭，录像已安全封装。",
                "onboard-start": "真机视频开启期望已提交，等待状态回报。",
                "onboard-stop": "真机视频关闭期望已提交，等待状态回报。",
                "onboard-snapshot": "真机人工抓拍请求已发布。",
            }
            if name == "snapshot":
                self.operation_message.setText(f"JPG 已保存：{data.get('path', '')}")
            elif name in messages:
                self.operation_message.setText(messages[name])
        if self._service_ready and name in {"configure", "start", "stop", "snapshot"}:
            QTimer.singleShot(0, self._poll_status)
        self._refresh_controls()

    def _poll_all_status(self) -> None:
        """轮询两个独立控制面；任一失败都不能停止播放器。"""
        self._poll_status()
        self._poll_onboard_status()

    def _poll_status(self) -> None:
        """异步读取本机后台状态，不阻塞 Qt。"""
        if not self._service_ready or "status" in self._request_in_flight:
            return
        self._submit("status", lambda: self.client.request("status", timeout=0.7))

    def _poll_onboard_status(self) -> None:
        """读取缓存的 ROS 状态；后台消息不会覆盖用户手填地址。"""
        if self.source_mode is not SourceMode.ONBOARD:
            return
        client = self._ensure_onboard_client()
        if client is None:
            return
        try:
            self._onboard_status = dict(client.status())
        except Exception as exc:
            self._onboard_status = {"state": "error", "last_error": str(exc)}
        if (
            self._onboard_status.get("service_available")
            and self._onboard_status.get("interface_version") != "3.2"
        ):
            self._onboard_status.update(
                service_available=False,
                running=False,
                state="incompatible",
                last_error=(
                    "视频接口版本不兼容：期望 3.2，收到 "
                    f"{self._onboard_status.get('interface_version') or '--'}"
                ),
            )
        if (
            not self._url_user_edited
            and not self.rtsp_url_input.text().strip()
            and self._onboard_status.get("service_available")
            and self._onboard_status.get("rtsp_url")
        ):
            self._set_rtsp_url(str(self._onboard_status["rtsp_url"]))
        self._refresh_source_status()
        self._refresh_controls()

    def _request_probe(self) -> None:
        """按当前本机设备读取真实 V4L2 模式。"""
        if not self._service_ready or self.source_mode is not SourceMode.LOCAL:
            return
        device = self.device_combo.currentData() or self._configured_device()
        self.operation_message.setText("正在读取摄像头支持的格式、分辨率和帧率…")
        self._submit(
            "probe",
            lambda: self.client.request("probe", {"device": str(device)}, timeout=6.0),
        )
        self._refresh_controls()

    def _apply_probe(self, data: dict[str, Any]) -> None:
        """更新设备与模式，并保留实际探测的设备选择。"""
        configured = self._last_status.get("config", {})
        configured_device = str(configured.get("device", self._configured_device()))
        current_device = str(self.device_combo.currentData() or "")
        selected_device = str(data.get("selected_device") or current_device or configured_device)
        devices = data.get("devices", [])
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        for item in devices if isinstance(devices, list) else []:
            if isinstance(item, dict):
                self.device_combo.addItem(str(item.get("label", "摄像头")), item.get("path"))
        if self.device_combo.count() == 0:
            self.device_combo.addItem(selected_device, selected_device)
        selected_index = self.device_combo.findData(selected_device)
        if selected_index < 0:
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
            excluded = data.get("excluded_modes", [])
            excluded_count = len(excluded) if isinstance(excluded, list) else 0
            message = f"检测到 {len(self._all_modes)} 个可用于 RTSP 的视频模式。"
            if excluded_count:
                message += f" 已隐藏 {excluded_count} 个超过 RTSP/JPEG 2040 像素限制的模式。"
            self.operation_message.setText(message)
        self._filter_modes()

    def _filter_modes(self) -> None:
        """编码变化时只显示摄像头真实支持的组合。"""
        codec = str(self.codec_combo.currentData() or "h264")
        configured = self._last_status.get("config", {})
        target = (
            int(configured.get("width", 1280)),
            int(configured.get("height", 720)),
            float(configured.get("fps", 120.0)),
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
            self.mode_combo.addItem(f"{width}×{height}  ·  {format_fps(fps)} fps", item)
            if (width, height, fps) == target:
                selected_index = self.mode_combo.count() - 1
        self.mode_combo.setCurrentIndex(selected_index if selected_index >= 0 else 0)
        self.mode_combo.blockSignals(False)
        self._refresh_controls()

    def _apply_status(self, status: dict[str, Any]) -> None:
        """刷新本机服务状态；绝不自动播放、暂停或覆盖手填地址。"""
        self._last_status = status
        config = status.get("config", {})
        if isinstance(config, dict) and not self._config_loaded:
            self._populate_configuration(config)
            self._config_loaded = True
        suggested_url = str(status.get("rtsp_url", ""))
        if suggested_url and not self._url_user_edited and not self.rtsp_url_input.text().strip():
            self._set_rtsp_url(suggested_url)
        error = str(status.get("last_error", ""))
        if error and str(status.get("state", "")) == "error":
            self.operation_message.setText(error)
        self._refresh_source_status()
        self._refresh_controls()

    def _populate_configuration(self, config: dict[str, Any]) -> None:
        """把本机权威配置写入表单一次，轮询不覆盖后续编辑。"""
        self.ip_input.setText(str(config.get("rtsp_ip", "127.0.0.1")))
        self.port_input.setValue(int(config.get("rtsp_port", 8554)))
        self.path_input.setText(str(config.get("rtsp_path", "camera")))
        self.video_path_input.setText(str(config.get("video_directory", "")))
        self.image_path_input.setText(str(config.get("image_directory", "")))
        self.codec_combo.setCurrentIndex(max(0, self.codec_combo.findData(str(config.get("codec", "h264")))))
        self.container_combo.setCurrentIndex(
            max(0, self.container_combo.findData(str(config.get("container", "mp4"))))
        )
        device = str(config.get("device", ""))
        self.device_combo.addItem(device, device)
        self.device_combo.setCurrentIndex(0)
        self.device_path_label.setText(device)

    def _configuration_from_fields(self) -> dict[str, Any]:
        """收集本机发布配置；硬件支持仍由后台最终校验。"""
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
        """只在本机模式保存配置，不自动重启或中断摄像头。"""
        if self.source_mode is not SourceMode.LOCAL:
            return
        try:
            config = self._configuration_from_fields()
        except CameraServiceError as exc:
            self._show_error("无法保存配置", str(exc))
            return
        self._begin_action("正在保存本机摄像头配置…")
        self._submit("configure", lambda: self.client.request("configure", {"config": config}, timeout=5.0))

    def _start_camera(self) -> None:
        """按模式启动本机后台或发布真机视频期望；外部模式无命令。"""
        if self.source_mode is SourceMode.ONBOARD:
            self._request_onboard_state(True)
            return
        if self.source_mode is not SourceMode.LOCAL:
            return
        try:
            config = self._configuration_from_fields()
        except CameraServiceError as exc:
            self._show_error("无法启动摄像头", str(exc))
            return
        self._begin_action("正在启动本机 MediaMTX、摄像头与录像…")
        self._submit("start", lambda: self.client.request("start", {"config": config}, timeout=25.0))

    def _stop_camera(self) -> None:
        """按模式关闭目标摄像头；不改变当前 RTSP 播放器。"""
        if self.source_mode is SourceMode.ONBOARD:
            self._request_onboard_state(False)
            return
        if self.source_mode is not SourceMode.LOCAL:
            return
        self._stopping_camera = True
        self._begin_action("正在关闭本机摄像头并封装录像…")
        self._submit("stop", lambda: self.client.request("stop", timeout=20.0))

    def _save_snapshot(self) -> None:
        """本机走 IPC，真机走逐条可靠 VideoCapture；外部源不支持。"""
        if self.source_mode is SourceMode.ONBOARD:
            client = self._ensure_onboard_client()
            if client is None:
                return
            self._begin_action("正在发布真机人工抓拍请求…")
            client.request_snapshot(
                lambda result, error: self._bridge.completed.emit(
                    "onboard-snapshot", result, error
                )
            )
            return
        if self.source_mode is not SourceMode.LOCAL:
            return
        self._begin_action("正在从本机 RTSP 保存图片…")
        self._submit(
            "snapshot",
            lambda: self.client.request("snapshot", {"kind": "manual"}, timeout=10.0),
        )

    def _request_onboard_state(self, enabled: bool) -> None:
        """异步调用独立视频服务，响应只代表期望状态已排队。"""
        client = self._ensure_onboard_client()
        if client is None:
            return
        name = "onboard-start" if enabled else "onboard-stop"
        self._begin_action("正在提交真机视频状态期望…")
        client.request_state(
            enabled,
            lambda result, error: self._bridge.completed.emit(name, result, error),
        )

    def _begin_action(self, message: str) -> None:
        """锁定会改变服务状态的按钮，播放器保持响应。"""
        self._action_busy = True
        self.operation_message.setText(message)
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        """用来源模式和服务状态门控，预览控件始终独立可用。"""
        mode = self.source_mode
        local_state = str(self._last_status.get("state", "stopped"))
        local_running = bool(self._last_status.get("running", False))
        editable = (
            mode is SourceMode.LOCAL
            and self._service_ready
            and local_state == "stopped"
            and not self._action_busy
            and "probe" not in self._request_in_flight
        )
        for group in (self.device_group, self.address_group, self.quality_group, self.storage_group):
            group.setEnabled(editable)
        mode_ready = self.mode_combo.count() > 0
        self.apply_button.setEnabled(editable and mode_ready)
        self.fetch_onboard_url_button.setEnabled(mode is SourceMode.ONBOARD)

        if mode is SourceMode.LOCAL:
            self.start_button.setText("开启本机摄像头")
            self.stop_button.setText("关闭本机摄像头")
            self.start_button.setEnabled(editable and mode_ready and not local_running)
            self.stop_button.setEnabled(
                self._service_ready
                and local_state in {"starting", "running", "error"}
                and not self._action_busy
            )
            self.snapshot_button.setEnabled(local_running and not self._action_busy)
        elif mode is SourceMode.ONBOARD:
            self.start_button.setText("开启真机摄像头")
            self.stop_button.setText("关闭真机摄像头")
            client_ready = self._ensure_onboard_client() is not None
            self.start_button.setEnabled(client_ready and not self._action_busy)
            self.stop_button.setEnabled(client_ready and not self._action_busy)
            self.snapshot_button.setEnabled(
                client_ready
                and bool(self._onboard_status.get("running", False))
                and not self._action_busy
            )
        else:
            self.start_button.setText("开启摄像头")
            self.stop_button.setText("关闭摄像头")
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            self.snapshot_button.setEnabled(False)
        self.rtsp_url_input.setEnabled(True)
        self.play_pause_button.setEnabled(True)
        self.copy_url_button.setEnabled(True)

    def _source_changed(self) -> None:
        """切换来源只卸载预览并重置统计，绝不发送停止命令。"""
        self._stop_preview("输入 RTSP 地址后点击播放")
        if self.source_mode is SourceMode.ONBOARD:
            self._ensure_onboard_client()
            self._poll_onboard_status()
        self._refresh_source_status()
        self._refresh_controls()

    def _ensure_onboard_client(self) -> Any | None:
        """延迟创建纯视频 ROS 客户端，使本机/外部模式不依赖 ROS。"""
        if self._onboard_client is None:
            try:
                from .ros_client import OnboardVideoClient

                self._onboard_client = OnboardVideoClient()
            except Exception as exc:
                self.operation_message.setText(str(exc))
                return None
        try:
            self._onboard_client.start()
        except Exception as exc:
            self.operation_message.setText(f"机载视频客户端启动失败：{exc}")
            return None
        return self._onboard_client

    def _fill_onboard_url(self) -> None:
        """显式用最新且未过期的 VideoStatus 地址填入查看器。"""
        self._poll_onboard_status()
        status = self._onboard_status
        url = str(status.get("rtsp_url", ""))
        if not status.get("service_available") or not url:
            self.operation_message.setText("尚未收到可用的真机视频地址，请稍后重试。")
            return
        self._set_rtsp_url(url)
        self._url_user_edited = False
        self.operation_message.setText("已读取真机 RTSP 地址；点击播放开始预览。")

    def _refresh_source_status(self) -> None:
        """展示当前命令目标状态，不把它当作播放器状态。"""
        if self.source_mode is SourceMode.LOCAL:
            state = str(self._last_status.get("state", "unavailable"))
            self.source_status_label.setText(f"本机摄像头服务：{state}")
        elif self.source_mode is SourceMode.ONBOARD:
            state = str(self._onboard_status.get("state", "unavailable"))
            error = str(self._onboard_status.get("last_error", ""))
            self.source_status_label.setText(
                f"真机视频服务：{state}" + (f" · {error}" if error else "")
            )
        else:
            self.source_status_label.setText("指定 RTSP 仅提供预览，不发送摄像头命令。")

    def _toggle_preview(self) -> None:
        """播放中暂停；暂停同地址恢复；新地址彻底重建后播放。"""
        if self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()
            return
        url = self.rtsp_url_input.text().strip()
        error = self._validate_rtsp_url(url)
        if error:
            self._show_black_preview(error)
            return
        if (
            self.player.playbackState() == QMediaPlayer.PlaybackState.PausedState
            and url == self._active_preview_url
        ):
            self.player.play()
            return
        if self.player.source().isValid() or self._active_preview_url:
            self._recreate_player()
        self._active_preview_url = url
        self._reset_playback_metrics()
        self._show_black_preview("正在连接 RTSP；收到首个有效视频帧后显示画面。")
        self.player.setSource(QUrl(url))
        self.player.play()

    def _ensure_preview(self, url: str) -> None:
        """供测试/显式调用使用；后台状态轮询不会调用此方法。"""
        self._set_rtsp_url(url)
        self._toggle_preview()

    def _reconnect_preview(self) -> None:
        """显式重建当前地址的播放器，不触碰摄像头进程。"""
        url = self.rtsp_url_input.text().strip() or self._active_preview_url
        self._recreate_player()
        self._active_preview_url = ""
        self._set_rtsp_url(url)
        self._toggle_preview()

    def _stop_preview(self, message: str) -> None:
        """只停止并卸载本窗口解码会话。"""
        if self.player.source().isValid() or self._active_preview_url:
            self._recreate_player()
        else:
            self.player.stop()
            self.player.setSource(QUrl())
        self._active_preview_url = ""
        self._reset_playback_metrics()
        self._show_black_preview(message)
        self._sync_play_button()

    def _player_error(self, _error: QMediaPlayer.Error, message: str) -> None:
        """预览错误只显示在面板，不改变推流、录像或飞行状态。"""
        self._last_preview_frame_at = None
        self._show_black_preview(
            f"RTSP 预览暂不可用：{message}；摄像头后台不受影响。"
        )

    def _player_media_status_changed(self, status: QMediaPlayer.MediaStatus) -> None:
        """媒体状态不代表已有画面；只有有效 sink 帧可以暴露原生 surface。"""
        if status == QMediaPlayer.MediaStatus.LoadingMedia:
            self._last_preview_frame_at = None
            self._show_black_preview("正在连接 RTSP；等待有效视频帧。")
        elif (
            status == QMediaPlayer.MediaStatus.StalledMedia
            and self.player.playbackState()
            != QMediaPlayer.PlaybackState.PausedState
        ):
            self._last_preview_frame_at = None
            self._show_black_preview("RTSP 数据已中断，预览区保持纯黑并等待恢复。")
        elif status in {
            QMediaPlayer.MediaStatus.NoMedia,
            QMediaPlayer.MediaStatus.InvalidMedia,
            QMediaPlayer.MediaStatus.EndOfMedia,
        }:
            self._last_preview_frame_at = None
            self._show_black_preview("RTSP 当前没有可显示的视频画面。")
        elif (
            status
            in {
                QMediaPlayer.MediaStatus.LoadedMedia,
                QMediaPlayer.MediaStatus.BufferedMedia,
            }
            and self._last_preview_frame_at is None
        ):
            self._show_black_preview("RTSP 已连接；等待首个有效视频帧。")

    def _player_state_changed(self, state: QMediaPlayer.PlaybackState) -> None:
        """用单调时钟累计真正处于 Playing 的时间区间。"""
        now = self._clock()
        if state == QMediaPlayer.PlaybackState.PlayingState:
            if self._playing_since is None:
                self._playing_since = now
            # 从暂停恢复时给解码器一个完整 watchdog 周期；暂停期间最后
            # 一帧继续保留，不会因墙钟经过而在恢复瞬间被清空。
            if self._last_preview_frame_at is not None:
                self._last_preview_frame_at = now
        elif self._playing_since is not None:
            self._played_seconds += max(0.0, now - self._playing_since)
            self._playing_since = None
        if (
            state == QMediaPlayer.PlaybackState.StoppedState
            and self._active_preview_url
        ):
            self._last_preview_frame_at = None
            self._show_black_preview("RTSP 播放已停止。")
        self._sync_play_button()
        self._refresh_playback_metrics()

    def _count_video_frame(self, frame: Any) -> None:
        """首个有效帧才显示原生 surface；无效帧立即恢复纯黑。"""
        valid = bool(frame.isValid()) if hasattr(frame, "isValid") else bool(frame)
        state = self.player.playbackState()
        if not valid:
            if state != QMediaPlayer.PlaybackState.PausedState:
                self._last_preview_frame_at = None
                self._show_black_preview("RTSP 当前没有有效视频帧。")
            return

        self._last_preview_frame_at = self._clock()
        if (
            self._active_preview_url
            and state != QMediaPlayer.PlaybackState.StoppedState
        ):
            self.preview_stack.setCurrentWidget(self.video_widget)
        if state == QMediaPlayer.PlaybackState.PlayingState:
            self._preview_frame_count += 1

    def _played_time(self) -> float:
        """返回已完成区间与当前 Playing 区间的总秒数。"""
        current = 0.0
        if self._playing_since is not None:
            current = max(0.0, self._clock() - self._playing_since)
        return self._played_seconds + current

    def _refresh_playback_metrics(self) -> None:
        """刷新播放指标，并在播放中断帧时切回不透明黑色占位。"""
        elapsed = self._played_time()
        self.elapsed_value.setText(self._format_elapsed(elapsed))
        average = self._preview_frame_count / elapsed if elapsed > 0.0 else 0.0
        self.fps_value.setText(f"{average:.1f} fps")
        if (
            self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState
            and self._active_preview_url
            and self._last_preview_frame_at is not None
            and self._clock() - self._last_preview_frame_at
            > PREVIEW_FRAME_TIMEOUT_SECONDS
        ):
            self._last_preview_frame_at = None
            self._show_black_preview("RTSP 超时未收到新画面，预览区保持纯黑。")

    def _reset_playback_metrics(self) -> None:
        """新地址或模式切换时重置本窗口统计。"""
        self._played_seconds = 0.0
        self._playing_since = None
        self._preview_frame_count = 0
        self._last_preview_frame_at = None
        self._refresh_playback_metrics()

    def _show_black_preview(self, message: str = "") -> None:
        """隐藏原生视频子窗口，并在预览区外展示状态说明。"""
        self.preview_placeholder.clear()
        self.video_widget.hide()
        self.preview_stack.setCurrentWidget(self.preview_placeholder)
        self.preview_placeholder.show()
        self.preview_stack.update()
        if message:
            self.operation_message.setText(message)

    def _sync_play_button(self) -> None:
        """同步无文本媒体图标、提示和辅助功能名称。"""
        playing = self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState
        standard = QStyle.StandardPixmap.SP_MediaPause if playing else QStyle.StandardPixmap.SP_MediaPlay
        self.play_pause_button.setIcon(self.style().standardIcon(standard))
        self.play_pause_button.setToolTip("暂停 RTSP（空格键）" if playing else "播放 RTSP（空格键）")

    def _device_changed(self) -> None:
        """本机设备变化后显示完整路径并重新读取能力。"""
        self.device_path_label.setText(str(self.device_combo.currentData() or "—"))
        if self._config_loaded and self.source_mode is SourceMode.LOCAL:
            self._request_probe()

    def _mark_url_edited(self, _text: str) -> None:
        """记录人工编辑，后续本机或真机状态不得覆盖。"""
        self._url_user_edited = True

    def _set_rtsp_url(self, url: str) -> None:
        """程序化填值不伪装成人工编辑。"""
        self.rtsp_url_input.setText(url)

    def _copy_rtsp_url(self) -> None:
        """复制地址框原值，不重新拼接发布配置。"""
        QApplication.clipboard().setText(self.rtsp_url_input.text())
        self.statusBar().showMessage("RTSP 拉流地址已复制", 2500)

    def _choose_directory(self, target: QLineEdit, title: str) -> None:
        """用原生目录选择器更新路径，不清空已有目录。"""
        initial = target.text().strip() or str(Path.home())
        selected = QFileDialog.getExistingDirectory(self, title, initial)
        if selected:
            target.setText(selected)

    def _configured_device(self) -> str:
        """首轮枚举前返回本机服务端或默认设备路径。"""
        config = self._last_status.get("config", {})
        if isinstance(config, dict) and config.get("device"):
            return str(config["device"])
        return CameraConfig.defaults().device

    def _set_service_failure(self, message: str) -> None:
        """仅标记本机后台离线；外部/真机 RTSP 播放保持不变。"""
        self._service_ready = False
        self.operation_message.setText(message)
        self._refresh_source_status()
        self._refresh_controls()

    def _show_error(self, title: str, message: str) -> None:
        """显示本地错误框，不调用地面站模态流程。"""
        dialog = QMessageBox(self)
        dialog.setWindowTitle(title)
        dialog.setIcon(QMessageBox.Icon.Warning)
        dialog.setText(message)
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.exec()

    @staticmethod
    def _validate_rtsp_url(url: str) -> str:
        """只接受带主机名的 RTSP URL。"""
        if not url:
            return "请输入 RTSP 拉流地址"
        try:
            parsed = urlsplit(url)
            _ = parsed.port
        except ValueError as exc:
            return f"RTSP 地址无效：{exc}"
        if parsed.scheme.lower() != "rtsp" or not parsed.hostname:
            return "拉流地址必须是包含主机名的 rtsp:// URL"
        return ""

    @staticmethod
    def _copy_icon() -> QIcon:
        """优先使用桌面主题复制图标，无主题时绘制双矩形回退。"""
        icon = QIcon.fromTheme("edit-copy")
        if not icon.isNull():
            return icon
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setPen(QPen(QColor("#245f87"), 1.6))
        painter.drawRect(6, 3, 10, 11)
        painter.drawRect(3, 6, 10, 11)
        painter.end()
        return QIcon(pixmap)

    @staticmethod
    def _format_elapsed(seconds: float) -> str:
        """把播放时长格式化为固定宽度时分秒。"""
        total = max(0, int(seconds))
        hours, remainder = divmod(total, 3600)
        minutes, secs = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        """关窗只停预览、轮询和客户端 context，绝不发送摄像头停止。"""
        self._poll_timer.stop()
        self._metrics_timer.stop()
        self._dispose_player()
        if self._onboard_client is not None:
            try:
                self._onboard_client.close()
            except Exception:
                pass
        event.accept()
