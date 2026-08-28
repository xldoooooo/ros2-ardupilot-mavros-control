"""PySide6 地面站主窗口：组合面板、桥接线程事件并执行安全交互。"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QEvent, QObject, QProcess, Qt, QTimer, Signal
from PySide6.QtGui import (
    QAction,
    QCloseEvent,
    QColor,
    QKeySequence,
    QMouseEvent,
    QResizeEvent,
    QShortcut,
)
from PySide6.QtWidgets import (
    QAbstractButton,
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QFileDialog,
    QFrame,
    QGraphicsDropShadowEffect,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..config import INTERFACE_VERSION, MAX_WAYPOINT_COUNT, PROJECT_ROOT
from ..environment import EnvironmentInitializer
from ..event_log import EventLog, LogLevel
from ..models import (
    FlightMode,
    VehicleSnapshot,
    WaypointFlightStrategy,
    WaypointReferenceGenerator,
    WaypointTrackingController,
)
from ..process_manager import CleanupReport
from ..ros_controller import GroundStationRosController
from ..upstream import (
    UpstreamAction,
    UpstreamCommand,
    UpstreamCommunicationService,
)
from ..waypoint_io import (
    WaypointImportError,
    load_waypoint_file,
    waypoint_file_dialog_filter,
)
from .log_panel import LogPanel
from .operations_panel import OperationsPanel
from .state import derive_availability
from .upstream_panel import UpstreamCommunicationPanel
from .waypoint_panel import WaypointPanel
from .widgets import (
    ActivityBanner,
    ShadowMessageBox,
    StatusBadge,
    repolish,
    set_text_if_changed,
    set_tooltip_if_changed,
)


class _ThreadBridge(QObject):
    """把 Python/ROS 工作线程的纯数据安全投递回 Qt 主线程。"""

    environment_status = Signal(object, str)
    environment_done = Signal(bool, str)
    communication_done = Signal(bool, str)
    cleanup_done = Signal(object)
    shutdown_done = Signal(object)
    upstream_command = Signal(object)


@dataclass
class _UpstreamFlightSequence:
    """由可靠 ROS ticket 推进的上位机组合飞行动作。"""

    kind: str
    phase: str
    waypoints: tuple[tuple[float, float, float, float], ...] = ()
    point_indexes: tuple[int, ...] = ()
    photo_nos: tuple[str, ...] = ()
    strategy: WaypointFlightStrategy = WaypointFlightStrategy.STRAIGHT
    reference_generator: WaypointReferenceGenerator = (
        WaypointReferenceGenerator.STEP_POSITION
    )
    tracking_controller: WaypointTrackingController = (
        WaypointTrackingController.POSITION_PD_DOB
    )
    ticket: int | None = None
    standby_not_before: float = 0.0


class GroundStationWindow(QMainWindow):
    """地面站单窗口入口；持续飞控仍完全位于机载 C++ 服务。"""

    _REFRESH_INTERVAL_MS = 100
    _SHADOW_MARGIN = 14

    def __init__(
        self,
        *,
        event_log: EventLog | None = None,
        ros_controller: GroundStationRosController | None = None,
        environment: EnvironmentInitializer | None = None,
        upstream_service: UpstreamCommunicationService | None = None,
        auto_start: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        """创建后端依赖、模块化面板、线程桥和周期状态刷新。"""
        super().__init__(parent)
        inferred_log = getattr(ros_controller, "event_log", None)
        self._events = event_log or inferred_log or EventLog()
        self._ros = ros_controller or GroundStationRosController(event_log=self._events)
        self._environment = environment or EnvironmentInitializer(
            self._ros, event_log=self._events
        )
        self._bridge = _ThreadBridge(self)
        self._upstream = upstream_service or UpstreamCommunicationService(
            event_log=self._events,
            on_command=self._bridge.upstream_command.emit,
        )
        self._upstream_panel: UpstreamCommunicationPanel | None = None

        self._environment_active = False
        self._connection_mode = "none"
        self._pending_environment_mode = "none"
        self._workflow_busy = False
        self._communication_busy = False
        self._communication_cancel_pending = False
        self._waypoint_running = False
        self._waypoint_preview_active = False
        self._upstream_point_indexes: tuple[int, ...] = ()
        self._upstream_photo_nos: tuple[str, ...] = ()
        self._upstream_sequence: _UpstreamFlightSequence | None = None
        self._abnormal_return_latched = False
        self._hardware_low_power_notice_latched = False
        self._hardware_abnormal_notice_latched = False
        self._pending_commands: set[str] = set()
        self._active_waypoint_ticket: int | None = None
        self._last_result_sequence = 0
        self._last_video_result_sequence = 0
        self._last_ros_error = ""
        self._cleanup_thread: threading.Thread | None = None
        self._shutdown_thread: threading.Thread | None = None
        self._shutdown_cleanup_lock = threading.Lock()
        self._shutdown_report: CleanupReport | None = None
        self._shutting_down = False
        self._allow_close = False
        # 仅缓存显示层应用签名；ROS 快照和安全状态仍保持 10 Hz 读取。
        self._last_availability_render_key: object | None = None
        self._availability = derive_availability(
            VehicleSnapshot(),
            ros_ready=False,
            busy=False,
            closing=False,
            environment_active=False,
            connection_mode="none",
            pending_mode="none",
            waypoint_count=0,
            waypoint_running=False,
        )

        self.setWindowTitle("ArduPilot ROS 2 工程地面站")
        self.setObjectName("groundStationWindow")
        self._configure_window_chrome()
        self.resize(1600, 920)
        self.setMinimumSize(1180, 700)
        self._build_ui()
        self._build_menu_bar()
        self._connect_signals()
        self._setup_shortcuts()

        # 插件启动失败不得影响 ROS、仿真或原有本地操作入口。
        try:
            self._upstream.start()
        except Exception as exc:
            self._events.warn("upstream", f"上位机通讯模块启动失败：{exc}")

        self._timer = QTimer(self)
        self._timer.setInterval(self._REFRESH_INTERVAL_MS)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._events.info("ui", "Qt 地面站界面已创建")
        self._refresh()
        # Production stays outside the DDS graph until the operator requests a workflow.
        if auto_start:
            QTimer.singleShot(0, self._start_ros)

    @property
    def event_log(self) -> EventLog:
        """返回当前窗口的共享结构化日志总线。"""
        return self._events

    def _configure_window_chrome(self) -> None:
        """创建覆盖菜单、内容和状态栏的完整自绘外框与阴影。"""
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setMouseTracking(True)
        content_margin = self._SHADOW_MARGIN + 1
        self.setContentsMargins(*([content_margin] * 4))

        self.outer_window_frame = QFrame(self)
        self.outer_window_frame.setObjectName("outerWindowFrame")
        self.outer_window_frame.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents, True
        )
        self.outer_window_frame.setProperty("windowMaximized", False)
        self._window_shadow = QGraphicsDropShadowEffect(self.outer_window_frame)
        self._window_shadow.setBlurRadius(30.0)
        self._window_shadow.setOffset(0.0, 3.0)
        self._window_shadow.setColor(QColor(16, 30, 44, 118))
        self.outer_window_frame.setGraphicsEffect(self._window_shadow)
        self.outer_window_frame.lower()

    def _sync_window_chrome(self) -> None:
        """按普通或最大化状态同步留边、圆角、阴影和控制按钮。"""
        maximized = self.isMaximized() or self.isFullScreen()
        frame_margin = 0 if maximized else self._SHADOW_MARGIN
        content_margin = frame_margin + 1
        expected_margins = (content_margin,) * 4
        current = self.contentsMargins()
        if (current.left(), current.top(), current.right(), current.bottom()) != (
            expected_margins
        ):
            self.setContentsMargins(*expected_margins)
        self.outer_window_frame.setGeometry(
            self.rect().adjusted(
                frame_margin, frame_margin, -frame_margin, -frame_margin
            )
        )
        self.outer_window_frame.lower()
        if self._window_shadow.isEnabled() == maximized:
            self._window_shadow.setEnabled(not maximized)
        if self.outer_window_frame.property("windowMaximized") != maximized:
            self.outer_window_frame.setProperty("windowMaximized", maximized)
            repolish(self.outer_window_frame)
        if hasattr(self, "maximize_button"):
            set_text_if_changed(
                self.maximize_button, "❐" if maximized else "□"
            )
            set_tooltip_if_changed(
                self.maximize_button, "还原" if maximized else "最大化"
            )

    def _resize_edges_at(self, x: float, y: float) -> Qt.Edge:
        """把透明留边中的坐标映射为原生四边或四角缩放方向。"""
        if self.isMaximized() or self.isFullScreen():
            return Qt.Edge(0)
        band = self._SHADOW_MARGIN
        edges = Qt.Edge(0)
        if x <= band:
            edges |= Qt.Edge.LeftEdge
        elif x >= self.width() - band:
            edges |= Qt.Edge.RightEdge
        if y <= band:
            edges |= Qt.Edge.TopEdge
        elif y >= self.height() - band:
            edges |= Qt.Edge.BottomEdge
        return edges

    @staticmethod
    def _resize_cursor(edges: Qt.Edge) -> Qt.CursorShape | None:
        """返回与缩放边/角一致的鼠标形状。"""
        if edges in (
            Qt.Edge.LeftEdge | Qt.Edge.TopEdge,
            Qt.Edge.RightEdge | Qt.Edge.BottomEdge,
        ):
            return Qt.CursorShape.SizeFDiagCursor
        if edges in (
            Qt.Edge.RightEdge | Qt.Edge.TopEdge,
            Qt.Edge.LeftEdge | Qt.Edge.BottomEdge,
        ):
            return Qt.CursorShape.SizeBDiagCursor
        if edges & (Qt.Edge.LeftEdge | Qt.Edge.RightEdge):
            return Qt.CursorShape.SizeHorCursor
        if edges & (Qt.Edge.TopEdge | Qt.Edge.BottomEdge):
            return Qt.CursorShape.SizeVerCursor
        return None

    def _toggle_maximized(self) -> None:
        """在最大化和普通窗口之间切换。"""
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()

    def resizeEvent(self, event: QResizeEvent) -> None:  # noqa: N802 - Qt API
        """窗口尺寸变化时保持外框覆盖完整客户区。"""
        super().resizeEvent(event)
        self._sync_window_chrome()

    def changeEvent(self, event: QEvent) -> None:  # noqa: N802 - Qt API
        """最大化状态变化后在下一事件周期刷新阴影留边。"""
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange:
            QTimer.singleShot(0, self._sync_window_chrome)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """在透明外沿显示正确的缩放光标。"""
        cursor = self._resize_cursor(
            self._resize_edges_at(event.position().x(), event.position().y())
        )
        if cursor is None:
            self.unsetCursor()
        else:
            self.setCursor(cursor)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt API
        """左键按下外沿时交给窗口系统执行原生缩放。"""
        if event.button() == Qt.MouseButton.LeftButton:
            edges = self._resize_edges_at(
                event.position().x(), event.position().y()
            )
            handle = self.windowHandle()
            if edges and handle is not None and handle.startSystemResize(edges):
                event.accept()
                return
        super().mousePressEvent(event)

    def leaveEvent(self, event: QEvent) -> None:  # noqa: N802 - Qt API
        """鼠标离开窗口后恢复默认指针。"""
        self.unsetCursor()
        super().leaveEvent(event)

    def eventFilter(self, watched: object, event: QEvent) -> bool:  # noqa: N802
        """允许从菜单空白处拖动、双击最大化自绘窗口。"""
        if watched is getattr(self, "_drag_menu_bar", None) and isinstance(
            event, QMouseEvent
        ):
            point = event.position().toPoint()
            empty_area = self._drag_menu_bar.actionAt(point) is None
            if empty_area and event.button() == Qt.MouseButton.LeftButton:
                if event.type() == QEvent.Type.MouseButtonDblClick:
                    self._toggle_maximized()
                    event.accept()
                    return True
                if event.type() == QEvent.Type.MouseButtonPress:
                    handle = self.windowHandle()
                    if handle is not None and handle.startSystemMove():
                        event.accept()
                        return True
        return super().eventFilter(watched, event)

    def _build_ui(self) -> None:
        """在完整窗口外框内构建状态带、双栏工作区和底部日志。"""
        central = QWidget()
        central.setObjectName("centralRoot")
        # centralRoot 的 QSS 背景覆盖完整客户区；声明不透明可阻止 Qt
        # 在 2x 高 DPI resize 时递归重绘其后的透明顶层表面。
        central.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        surface = QFrame()
        surface.setObjectName("windowSurface")
        outer.addWidget(surface)
        self.window_surface = surface

        root = QVBoxLayout(surface)
        root.setContentsMargins(10, 9, 10, 8)
        root.setSpacing(9)

        status_row = QHBoxLayout()
        status_row.setSpacing(6)
        self.connection_label = QLabel("环境 · 未初始化")
        self.connection_label.setObjectName("environmentChip")
        status_row.addWidget(self.connection_label)
        self.link_badge = StatusBadge("机载链路")
        self.aircraft_badge = StatusBadge("飞行器")
        self.mode_badge = StatusBadge("控制模式")
        self.control_badge = StatusBadge("控制健康")
        for badge in (
            self.link_badge,
            self.aircraft_badge,
            self.mode_badge,
            self.control_badge,
        ):
            status_row.addWidget(badge, 1)
        self.activity_banner = ActivityBanner()
        status_row.addWidget(self.activity_banner, 2)
        root.addLayout(status_row)

        self.main_splitter = QSplitter(Qt.Orientation.Vertical)
        self.main_splitter.setObjectName("mainVerticalSplitter")
        self.main_splitter.setChildrenCollapsible(False)
        self.workspace_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.workspace_splitter.setObjectName("workspaceSplitter")
        self.workspace_splitter.setChildrenCollapsible(False)

        self.operations = OperationsPanel()
        operations_scroll = QScrollArea()
        operations_scroll.setObjectName("operationsScroll")
        operations_scroll.setWidgetResizable(True)
        operations_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        operations_scroll.setFrameShape(QFrame.Shape.NoFrame)
        operations_scroll.setWidget(self.operations)
        self.waypoints = WaypointPanel()
        waypoints_scroll = QScrollArea()
        waypoints_scroll.setObjectName("waypointsScroll")
        waypoints_scroll.setWidgetResizable(True)
        waypoints_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        waypoints_scroll.setFrameShape(QFrame.Shape.NoFrame)
        waypoints_scroll.setWidget(self.waypoints)
        self.workspace_splitter.addWidget(operations_scroll)
        self.workspace_splitter.addWidget(waypoints_scroll)
        self.workspace_splitter.setStretchFactor(0, 7)
        self.workspace_splitter.setStretchFactor(1, 5)
        self.workspace_splitter.setSizes((720, 500))

        self.log_panel = LogPanel(self._events)
        self.main_splitter.addWidget(self.workspace_splitter)
        self.main_splitter.addWidget(self.log_panel)
        self.main_splitter.setStretchFactor(0, 8)
        self.main_splitter.setStretchFactor(1, 2)
        self.main_splitter.setSizes((670, 170))
        root.addWidget(self.main_splitter, 1)

        self.statusBar().showMessage(
            f"接口 {INTERFACE_VERSION} · ROS 客户端尚未启动"
        )

    def _build_menu_bar(self) -> None:
        """构建设置/帮助菜单与右上角窗口控制条。"""
        menu_bar = self.menuBar()
        menu_bar.setNativeMenuBar(False)
        menu_bar.setMouseTracking(True)
        menu_bar.installEventFilter(self)
        self._drag_menu_bar = menu_bar

        self.settings_menu = menu_bar.addMenu("设置")
        self.show_log_action = QAction("显示实时日志", self)
        self.show_log_action.setCheckable(True)
        self.show_log_action.setChecked(True)
        self.show_log_action.toggled.connect(self.log_panel.setVisible)
        self.settings_menu.addAction(self.show_log_action)
        reset_layout_action = QAction("恢复默认布局", self)
        reset_layout_action.triggered.connect(self._reset_layout)
        self.settings_menu.addAction(reset_layout_action)

        help_menu = menu_bar.addMenu("帮助")
        shortcut_action = QAction("键盘控制说明", self)
        shortcut_action.triggered.connect(self._show_shortcut_help)
        help_menu.addAction(shortcut_action)
        about_action = QAction("关于地面站", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

        controls = QWidget()
        controls.setObjectName("windowControlStrip")
        self.window_controls = controls
        controls_layout = QHBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 4, 0)
        controls_layout.setSpacing(3)
        self.upstream_panel_button = QPushButton("上位机通讯面板")
        self.upstream_panel_button.setObjectName("upstreamPanelButton")
        self.upstream_panel_button.setProperty("compact", True)
        self.upstream_panel_button.setToolTip(
            "打开 WebSocket 连接、映射、原始报文与 JSON 配置面板"
        )
        self.upstream_panel_button.clicked.connect(self._open_upstream_panel)
        controls_layout.addWidget(self.upstream_panel_button)
        self.camera_panel_button = QPushButton("摄像头配置面板")
        self.camera_panel_button.setObjectName("cameraPanelButton")
        self.camera_panel_button.setProperty("compact", True)
        self.camera_panel_button.setToolTip(
            "打开独立摄像头推流、录像与截图配置面板"
        )
        self.camera_panel_button.clicked.connect(self._open_camera_panel)
        controls_layout.addWidget(self.camera_panel_button)
        self.correction_panel_button = QPushButton("Tag-Odin 修正面板")
        self.correction_panel_button.setObjectName("correctionPanelButton")
        self.correction_panel_button.setProperty("compact", True)
        self.correction_panel_button.setToolTip(
            "打开独立 AprilTag-Odin 校准、质量与数据链对照面板"
        )
        self.correction_panel_button.clicked.connect(self._open_correction_panel)
        controls_layout.addWidget(self.correction_panel_button)
        self.exit_button = QPushButton("退出地面站")
        self.exit_button.setObjectName("exitButton")
        self.exit_button.setProperty("role", "danger")
        self.exit_button.setProperty("compact", True)
        self.exit_button.setToolTip("安全退出地面站")
        self.exit_button.clicked.connect(self.close)
        controls_layout.addWidget(self.exit_button)
        self.minimize_button = self._window_control_button("—", "最小化")
        self.maximize_button = self._window_control_button("□", "最大化")
        self.close_button = self._window_control_button("×", "关闭", close=True)
        self.minimize_button.clicked.connect(self.showMinimized)
        self.maximize_button.clicked.connect(self._toggle_maximized)
        self.close_button.clicked.connect(self.close)
        controls_layout.addWidget(self.minimize_button)
        controls_layout.addWidget(self.maximize_button)
        controls_layout.addWidget(self.close_button)
        menu_bar.setCornerWidget(controls, Qt.Corner.TopRightCorner)

    @staticmethod
    def _window_control_button(
        text: str, tooltip: str, *, close: bool = False
    ) -> QPushButton:
        """创建固定宽度的自绘窗口控制按钮。"""
        button = QPushButton(text)
        button.setProperty("windowControl", True)
        button.setProperty("closeControl", close)
        button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        button.setFixedWidth(31)
        button.setToolTip(tooltip)
        return button

    def _reset_layout(self) -> None:
        """恢复双栏与日志区域的默认比例并确保日志可见。"""
        self.workspace_splitter.setSizes((720, 500))
        self.main_splitter.setSizes((670, 170))
        self.log_panel.show()
        self.show_log_action.setChecked(True)

    def _show_shortcut_help(self) -> None:
        """显示飞行键位及输入焦点安全规则。"""
        self._show_notice(
            "键盘控制说明",
            "W/S：升降    I/K：前后    J/L：左右\n"
            "A/D：偏航    Space：悬停\n\n"
            "当数值框、搜索框、日志或按钮获得焦点时，飞行快捷键自动停用。",
            QMessageBox.Icon.Information,
        )

    def _show_about(self) -> None:
        """展示地面站职责边界和接口版本。"""
        self._show_notice(
            "关于地面站",
            f"ArduPilot ROS 2 工程地面站\n接口 {INTERFACE_VERSION}\n\n"
            "本界面负责高层任务、状态和日志；100 Hz 飞行控制闭环位于机载服务。",
            QMessageBox.Icon.Information,
        )

    def _message_box(
        self,
        title: str,
        message: str,
        icon: QMessageBox.Icon,
        buttons: QMessageBox.StandardButton = QMessageBox.StandardButton.Ok,
        default: QMessageBox.StandardButton = QMessageBox.StandardButton.Ok,
    ) -> ShadowMessageBox:
        """构造统一带边框和阴影的子窗口，但不立即进入模态循环。"""
        dialog = ShadowMessageBox(self)
        dialog.setWindowTitle(title)
        dialog.setText(message)
        dialog.setIcon(icon)
        dialog.setStandardButtons(buttons)
        dialog.setDefaultButton(default)
        return dialog

    def _show_notice(
        self, title: str, message: str, icon: QMessageBox.Icon
    ) -> None:
        """显示只有确认按钮的统一提示子窗口。"""
        self._message_box(title, message, icon).exec()

    def _open_camera_panel(self) -> None:
        """分离启动独立摄像头面板，不纳入ROS或仿真进程清理链。"""
        panel_script = PROJECT_ROOT / "video_service" / "camera_panel.py"
        if not panel_script.is_file():
            message = f"未找到摄像头配置面板：{panel_script}"
            self._events.error("camera", message)
            self._show_notice("无法打开摄像头面板", message, QMessageBox.Icon.Warning)
            return
        try:
            started, process_id = QProcess.startDetached(
                sys.executable,
                [str(panel_script)],
                str(panel_script.parent),
            )
        except Exception as exc:
            started, process_id = False, 0
            self._events.error("camera", f"摄像头面板启动异常：{exc}")
        if started:
            self._events.info(
                "camera", f"已启动独立摄像头配置面板（PID={process_id}）"
            )
            self.activity_banner.set_message(
                "已打开独立摄像头配置面板。", LogLevel.INFO
            )
            return
        message = "摄像头配置面板启动失败，请检查项目Python和PySide6环境。"
        self._events.error("camera", message)
        self._show_notice("无法打开摄像头面板", message, QMessageBox.Icon.Warning)

    def _open_correction_panel(self) -> None:
        """分离启动修正面板，不把相机任务或 extnav 修正纳入 GUI 清理链。"""
        panel_script = PROJECT_ROOT / "correction_service" / "correction_panel.py"
        if not panel_script.is_file():
            message = f"未找到 Tag-Odin 修正面板：{panel_script}"
            self._events.error("correction", message)
            self._show_notice("无法打开修正面板", message, QMessageBox.Icon.Warning)
            return
        try:
            started, process_id = QProcess.startDetached(
                sys.executable,
                [str(panel_script)],
                str(panel_script.parent),
            )
        except Exception as exc:
            started, process_id = False, 0
            self._events.error("correction", f"修正面板启动异常：{exc}")
        if started:
            self._events.info(
                "correction", f"已启动独立 Tag-Odin 修正面板（PID={process_id}）"
            )
            self.activity_banner.set_message(
                "已打开独立 Tag-Odin 修正面板。", LogLevel.INFO
            )
            return
        message = "Tag-Odin 修正面板启动失败，请检查接口构建和 PySide6 环境。"
        self._events.error("correction", message)
        self._show_notice("无法打开修正面板", message, QMessageBox.Icon.Warning)

    def _open_upstream_panel(self) -> None:
        """打开或复用独立上位机通讯面板，关闭面板不改变连接。"""
        if self._upstream_panel is None:
            # 面板必须是无 transient parent 的独立顶层窗口，否则窗口管理器会
            # 强制其压在主窗口上方，并可能忽略标题栏最小化操作。
            self._upstream_panel = UpstreamCommunicationPanel(self._upstream)
        if self._upstream_panel.isMinimized():
            self._upstream_panel.showNormal()
        else:
            self._upstream_panel.show()
        self._upstream_panel.raise_()
        self._upstream_panel.activateWindow()

    def _dispose_upstream_panel(self) -> None:
        """随主窗口退出销毁独立面板，避免其继续维持 Qt 事件循环。"""
        if self._upstream_panel is None:
            return
        self._upstream_panel.close()
        self._upstream_panel.deleteLater()
        self._upstream_panel = None

    def _connect_signals(self) -> None:
        """连接面板意图、线程桥和主窗口业务槽。"""
        self.operations.simulation_requested.connect(self._initialize_simulation)
        self.operations.hardware_requested.connect(self._initialize_hardware)
        self.operations.communication_test_requested.connect(
            self._test_hardware_communication
        )
        self.operations.stop_simulation_requested.connect(self._stop_simulation)
        self.operations.disconnect_hardware_requested.connect(
            self._disconnect_hardware
        )
        self.operations.takeoff_requested.connect(self._takeoff)
        self.operations.land_requested.connect(self._land)
        self.operations.hover_requested.connect(self._hover)
        self.operations.motion_requested.connect(self._send_motion)
        self.operations.coordinate_mode_changed.connect(
            self._log_coordinate_mode_change
        )
        self.waypoints.send_requested.connect(self._send_waypoints)
        self.waypoints.clear_requested.connect(self._confirm_clear_waypoints)
        self.waypoints.preview_requested.connect(self._preview_waypoints)
        self.waypoints.import_file_requested.connect(self._choose_waypoint_file)
        self.waypoints.files_dropped.connect(self._import_dropped_waypoint_files)
        self.waypoints.waypoints_changed.connect(self._on_waypoints_changed)
        self._bridge.environment_status.connect(self._on_environment_status)
        self._bridge.environment_done.connect(self._on_environment_done)
        self._bridge.communication_done.connect(self._on_communication_done)
        self._bridge.cleanup_done.connect(self._on_cleanup_done)
        self._bridge.shutdown_done.connect(self._on_shutdown_done)
        self._bridge.upstream_command.connect(self._handle_upstream_command)

    def _setup_shortcuts(self) -> None:
        """注册原功能键位；输入控件聚焦时由槽函数主动忽略。"""
        definitions = {
            "W": "up",
            "S": "down",
            "I": "forward",
            "K": "back",
            "J": "left",
            "L": "right",
            "A": "yaw_left",
            "D": "yaw_right",
        }
        self._shortcuts: list[QShortcut] = []
        for key, name in definitions.items():
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
            shortcut.activated.connect(
                lambda button_name=name: self._shortcut_motion(button_name)
            )
            self._shortcuts.append(shortcut)
        hover_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        hover_shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
        hover_shortcut.activated.connect(self._shortcut_hover)
        self._shortcuts.append(hover_shortcut)

    def _start_ros(self) -> None:
        """显式请求时创建 ROS 客户端；单纯打开窗口不会加入远端 DDS 图。"""
        if self._shutting_down:
            return
        try:
            self._ros.start()
        except Exception as exc:
            self._events.error("ui", f"启动 ROS 2 客户端失败：{exc}")
            self.activity_banner.set_message(str(exc), LogLevel.ERROR)
        self._refresh()

    # ---- 环境工作流 ----

    def _initialize_simulation(self) -> None:
        """选择本地仿真工作流；SITL 使用自身 Home，不写 GUI 缓存原点。"""
        if self._environment_active and not self._confirm_action(
            "切换为本地仿真",
            "当前环境将断开并释放控制权，本项目启动的本地进程会被清理。"
            "确认切换到本地 SITL 仿真吗？",
        ):
            return
        self._events.info("operator", "操作者请求启动本地完整仿真")
        self._begin_environment_workflow("simulation", "正在启动本地仿真环境…")
        started = self._environment.initialize_simulation(
            self._queue_environment_status, self._queue_environment_done
        )
        if not started:
            self._workflow_busy = False
            self._refresh()

    def _initialize_hardware(self) -> None:
        """经过明确实机风险确认后建立可正常控制的远端连接。"""
        origin = self.operations.origin()
        message = (
            "即将连接可能对应真实飞行器的机载服务并申请控制租约。\n\n"
            "连接流程会发送控制心跳、配置飞控消息频率，并写入飞控原点 "
            f"{origin[0]:.7f}, {origin[1]:.7f}, {origin[2]:.1f} m；"
            "完成后将开放地面站控制功能。连接动作本身不会解锁或起飞，"
            "解锁/起飞仍须操作者另行确认。"
        )
        if not self._confirm_action("连接实机服务", message, critical=True):
            return
        self._events.warn("operator", "操作者确认完整连接实机机载服务")
        self._begin_environment_workflow("hardware", "正在连接实机机载服务…")
        started = self._environment.initialize_hardware(
            origin, self._queue_environment_status, self._queue_environment_done
        )
        if not started:
            self._workflow_busy = False
            self._refresh()

    def _test_hardware_communication(self) -> None:
        """启动纯订阅检测；检测期间同一按钮切换为显式终止入口。"""
        if self._communication_busy:
            self._cancel_hardware_communication_test()
            return
        if not self._availability.communication_test:
            return
        self._events.info("operator", "操作者请求检测实机通讯链路（零控制命令）")
        self._communication_busy = True
        self._communication_cancel_pending = False
        self.activity_banner.set_message(
            "正在被动检测实机状态与日志链路…", LogLevel.INFO
        )
        self._refresh()
        started = self._environment.test_hardware_communication(
            self._queue_environment_status,
            self._queue_communication_done,
        )
        if not started:
            self._communication_busy = False
            self._communication_cancel_pending = False
            self._refresh()

    def _cancel_hardware_communication_test(self) -> None:
        """请求环境线程停止检测，且不复用会清理进程/租约的通用断开路径。"""
        if self._communication_cancel_pending:
            return
        cancel = getattr(
            self._environment, "cancel_hardware_communication_test", None
        )
        if cancel is None or not cancel():
            self.activity_banner.set_message(
                "通讯检测已结束或暂时无法终止。", LogLevel.WARN
            )
            return
        self._communication_cancel_pending = True
        self._events.warn("operator", "操作者请求终止实机通讯检测")
        self.activity_banner.set_message("正在终止实机通讯检测…", LogLevel.WARN)
        self._refresh()

    def _begin_environment_workflow(self, mode: str, message: str) -> None:
        """原子锁定互斥入口并记录待完成环境类型。"""
        self._pending_environment_mode = mode
        self._workflow_busy = True
        self.activity_banner.set_message(message, LogLevel.INFO)
        self._refresh()

    def _queue_environment_status(self, level: LogLevel, message: str) -> None:
        """供环境工作线程调用的 Qt 信号桥。"""
        self._bridge.environment_status.emit(level, message)

    def _queue_environment_done(self, success: bool, message: str) -> None:
        """供环境工作线程投递最终状态。"""
        self._bridge.environment_done.emit(success, message)

    def _queue_communication_done(self, success: bool, message: str) -> None:
        """供通讯检测线程投递结果，不复用会建立会话的完成信号。"""
        self._bridge.communication_done.emit(success, message)

    def _on_environment_status(self, level: LogLevel, message: str) -> None:
        """在主线程显示源端已经标级的环境进度。"""
        self.activity_banner.set_message(message, level)

    def _on_environment_done(self, success: bool, message: str) -> None:
        """完成环境切换并解除互斥锁。"""
        self._workflow_busy = False
        self._waypoint_preview_active = False
        self._hardware_low_power_notice_latched = False
        self._hardware_abnormal_notice_latched = False
        self._environment_active = success
        self._connection_mode = self._pending_environment_mode if success else "none"
        self._pending_environment_mode = "none"
        if not success:
            self._upstream_sequence = None
            self._abnormal_return_latched = False
            try:
                self._upstream.reset_runtime()
            except Exception as exc:
                self._events.warn("upstream", f"会话状态重置失败：{exc}")
        self.activity_banner.set_message(
            message, LogLevel.INFO if success else LogLevel.ERROR
        )
        self._refresh()

    def _on_communication_done(self, success: bool, message: str) -> None:
        """显示 Wi-Fi 检测结果，保持环境与连接模式不变。"""
        was_cancelled = self._communication_cancel_pending or "已取消" in message
        self._communication_busy = False
        self._communication_cancel_pending = False
        if success:
            level = LogLevel.INFO
        elif was_cancelled:
            level = LogLevel.WARN
        else:
            level = LogLevel.ERROR
        self.activity_banner.set_message(message, level)
        self._refresh()

    def _stop_simulation(self) -> None:
        """确认后终止本地 SITL 会话并释放控制权。"""
        self._begin_session_teardown(
            kind="simulation",
            title="终止本地仿真",
            progress="正在释放控制权并终止本地仿真进程…",
            confirm_body=(
                "确认终止本项目启动的本地 SITL、MAVROS、机载节点与 RViz 吗？"
                "控制租约将被释放。"
            ),
        )

    def _disconnect_hardware(self) -> None:
        """确认后释放实机控制租约；不远程终止机载进程。"""
        self._begin_session_teardown(
            kind="hardware",
            title="断开实机连接",
            progress="正在释放控制权并断开实机机载服务…",
            confirm_body=(
                "确认断开与远端机载服务的连接并释放控制租约吗？"
                "远端机载进程不会被终止；本机 RViz 预览及残留仿真进程"
                "会一并清理。"
            ),
        )

    def _begin_session_teardown(
        self,
        *,
        kind: str,
        title: str,
        progress: str,
        confirm_body: str,
    ) -> None:
        """后台统一释放租约，并仅终止本机受管仿真/预览进程。"""
        if self._cleanup_thread is not None and self._cleanup_thread.is_alive():
            return
        active = self._connection_mode == kind or (
            self._workflow_busy and self._pending_environment_mode == kind
        )
        if not active:
            return
        snapshot = self._ros.snapshot()
        risk = (
            "飞行器当前已武装。断开会释放控制租约，机载端将按失联策略"
            "先悬停、宽限期后自动降落；该操作不是立即降落命令。\n\n"
            if snapshot.armed
            else ""
        )
        if not self._confirm_action(
            title,
            risk + confirm_body,
            critical=snapshot.armed,
        ):
            return
        self._events.warn("operator", f"操作者确认：{title}")
        self._workflow_busy = True
        self.activity_banner.set_message(progress, LogLevel.WARN)

        def worker() -> None:
            try:
                report = self._environment.cleanup()
            except Exception as exc:
                report = CleanupReport(errors=(str(exc),))
                self._events.error("environment", f"清理线程异常：{exc}")
            self._bridge.cleanup_done.emit(report)

        self._cleanup_thread = threading.Thread(
            target=worker, name="ground-station-qt-cleanup", daemon=True
        )
        self._cleanup_thread.start()
        self._refresh()

    def _on_cleanup_done(self, report: CleanupReport) -> None:
        """显示可审计清理结果并恢复环境入口。"""
        self._workflow_busy = False
        self._waypoint_preview_active = False
        self._environment_active = False
        self._connection_mode = "none"
        self._pending_environment_mode = "none"
        self._upstream_sequence = None
        self._abnormal_return_latched = False
        self._hardware_low_power_notice_latched = False
        self._hardware_abnormal_notice_latched = False
        try:
            self._upstream.reset_runtime()
        except Exception as exc:
            self._events.warn("upstream", f"会话状态重置失败：{exc}")
        if report.success:
            message = (
                f"断开完成：终止 {report.managed_stopped} 个受管进程、"
                f"{len(report.stale_stopped)} 个历史残留；远端机载服务未终止。"
            )
            level = LogLevel.INFO
        else:
            message = f"清理不完整：残留={report.remaining}，错误={report.errors}"
            level = LogLevel.ERROR
        self.activity_banner.set_message(message, level)
        self._refresh()

    # ---- 飞行动作 ----

    def _handle_upstream_command(self, command: UpstreamCommand) -> None:
        """在 Qt 主线程按当前活动会话把已映射动作交给原有操作入口。"""
        if not isinstance(command, UpstreamCommand):
            self._events.warn("upstream", "拒绝无法识别的上位机命令对象")
            return
        # 使用与按钮相同的最新门控；不活动时绝不猜测仿真或实机目标。
        self._refresh()
        snapshot = self._ros.snapshot()
        if (
            not self._environment_active
            or self._connection_mode not in {"simulation", "hardware"}
        ):
            self._reject_upstream_command(command, "尚未启动仿真或连接实机")
            return
        safety_land = command.action in {
            UpstreamAction.LAND,
            UpstreamAction.EMERGENCY_LAND,
        }
        if self._shutting_down or (self._workflow_busy and not safety_land):
            self._reject_upstream_command(command, "地面站工作流正忙或正在退出")
            return

        if command.action is UpstreamAction.STAGE_WAYPOINTS:
            if not self._availability.waypoint_edit:
                self._reject_upstream_command(
                    command, "当前任务状态不允许替换航点列表"
                )
                return
            self.waypoints.replace_waypoints(command.waypoints, "上位机命令 02")
            self._upstream_point_indexes = command.point_indexes
            self._upstream_photo_nos = command.photo_nos
            try:
                self._upstream.block_standby()
            except Exception as exc:
                self._events.warn("upstream", f"待机状态锁定失败：{exc}")
            self._handle_ignored_upstream_point_fields(command)
            self._events.info(
                "upstream",
                f"上位机命令 02 已替换 GUI 航点列表，共 {len(command.waypoints)} 点",
            )
            self.activity_banner.set_message(
                f"已接收上位机航点：{len(command.waypoints)} 个。",
                LogLevel.INFO,
            )
            try:
                self._upstream.report_waypoints_staged()
            except Exception as exc:
                self._events.warn("upstream", f"状态 02 上报失败：{exc}")
            self._refresh()
            return

        if command.action is UpstreamAction.EXECUTE_WAYPOINTS:
            self._start_inspection_sequence(command, snapshot)
            return

        if command.action is UpstreamAction.RETURN_HOME:
            self._start_return_sequence("return", snapshot, command=command)
            return

        if command.action is UpstreamAction.TAKEOFF:
            if not self._availability.takeoff:
                self._reject_upstream_command(command, self._availability.flight_reason)
                return
            if self._takeoff() is None:
                self._reject_upstream_command(command, "起飞操作未获得本地确认")
            return

        if command.action in {UpstreamAction.LAND, UpstreamAction.EMERGENCY_LAND}:
            if not self._availability.land:
                self._reject_upstream_command(command, self._availability.flight_reason)
                return
            ticket = self._land(refresh_after_queue=False)
            if ticket is None:
                self._reject_upstream_command(command, "降落操作未获得本地确认")
                return
            try:
                self._upstream.begin_landing(ticket)
            except Exception as exc:
                self._events.warn("upstream", f"降落状态关联失败：{exc}")
            if command.action is UpstreamAction.EMERGENCY_LAND:
                try:
                    self._upstream.block_standby()
                except Exception as exc:
                    self._events.warn("upstream", f"待机状态锁定失败：{exc}")
                self._upstream_sequence = _UpstreamFlightSequence(
                    kind="emergency", phase="land", ticket=ticket
                )
            else:
                self._upstream_sequence = None
                try:
                    self._upstream.release_standby()
                except Exception as exc:
                    self._events.warn("upstream", f"待机状态解锁失败：{exc}")
            self._refresh()
            return

        self._reject_upstream_command(command, "当前地面站没有该动作实现")

    def _reject_upstream_command(
        self, command: UpstreamCommand, reason: str
    ) -> None:
        """统一给维护者显示业务拒绝；协议接收确认仍已由通讯线程发送。"""
        message = f"拒绝上位机命令 {command.command_no}（{command.label}）：{reason}"
        self._events.warn("upstream", message)
        self.activity_banner.set_message(message, LogLevel.WARN)

    def _start_inspection_sequence(
        self, command: UpstreamCommand, snapshot: VehicleSnapshot
    ) -> None:
        """把 03 组合为起飞、巡检航点和末点降落。"""
        values = self.waypoints.waypoints
        if not values:
            self._reject_upstream_command(command, "GUI 航点列表为空，请先下发命令 02")
            return
        if self._upstream_sequence is not None:
            self._reject_upstream_command(command, "已有上位机组合动作正在执行")
            return
        if snapshot.vehicle_abnormal:
            self._reject_upstream_command(
                command,
                snapshot.vehicle_abnormal_reason or "无人机状态异常",
            )
            return
        # 03 的协议语义是完整组合，已在飞行时不能跳过起飞阶段。
        if snapshot.armed:
            self._reject_upstream_command(command, "飞行器已在飞行，命令 03 必须从起飞开始")
            return
        if not self._availability.takeoff:
            self._reject_upstream_command(command, self._availability.flight_reason)
            return

        sequence = _UpstreamFlightSequence(
            kind="inspection",
            phase="takeoff",
            waypoints=values,
            point_indexes=self._point_indexes_for(values),
            photo_nos=self._photo_nos_for(values),
            strategy=self.waypoints.selected_strategy(),
            reference_generator=self.waypoints.selected_reference_generator(),
            tracking_controller=self.waypoints.selected_tracking_controller(),
        )
        self._upstream_sequence = sequence
        try:
            self._upstream.block_standby()
        except Exception as exc:
            self._events.warn("upstream", f"待机状态锁定失败：{exc}")

        ticket = self._takeoff(refresh_after_queue=False)
        if ticket is None:
            self._abort_upstream_sequence("起飞操作未获得本地确认")
            return
        sequence.ticket = ticket
        self._events.info(
            "upstream",
            "上位机命令 03 已启动组合动作：起飞→巡检航点→末点降落",
        )
        self._refresh()

    def _start_return_sequence(
        self,
        kind: str,
        snapshot: VehicleSnapshot,
        *,
        command: UpstreamCommand | None = None,
    ) -> bool:
        """以当前三项航点组合执行原点航点，稳定后降落。"""
        active = self._upstream_sequence
        if active is not None and active.kind in {
            "return",
            "low_power",
            "abnormal",
            "emergency",
        }:
            if command is not None:
                self._reject_upstream_command(command, "返航或降落组合已在执行")
            return False
        if not snapshot.armed:
            if command is not None:
                self._reject_upstream_command(command, "飞行器尚未起飞")
            return False
        if not self._availability.hover:
            if command is not None:
                self._reject_upstream_command(command, self._availability.flight_reason)
            else:
                self._events.warn(
                    "upstream",
                    f"无法启动 {kind} 返航：{self._availability.flight_reason}",
                )
            return False

        # TODO(返航机库): 当前以本地 ENU 原点与起飞高度作为机库上方点；
        # 待机库设备接入后，应改为实际引导/对接位。
        return_point = (
            0.0,
            0.0,
            float(self.operations.takeoff_altitude()),
            float(snapshot.yaw),
        )
        sequence = _UpstreamFlightSequence(
            kind=kind,
            phase="waypoints_pending",
            waypoints=(return_point,),
            point_indexes=(1,),
            photo_nos=(),
            strategy=self.waypoints.selected_strategy(),
            reference_generator=self.waypoints.selected_reference_generator(),
            tracking_controller=self.waypoints.selected_tracking_controller(),
        )
        self._upstream_sequence = sequence
        try:
            self._upstream.block_standby()
        except Exception as exc:
            self._events.warn("upstream", f"待机状态锁定失败：{exc}")
        if kind == "low_power":
            # TODO(低电量充电): 当前仅执行返航与降落，待充电设备接入后
            # 再增加对接、充电和续检策略。
            self._events.warn("upstream", "低电量已触发自动返航与降落组合")
        elif kind == "abnormal":
            # TODO(异常策略): 当前由平滑航点启动或入点连续失败标记异常；
            # 后续应把其他权威健康项纳入同一分类与恢复策略。
            self._events.error(
                "upstream",
                "无人机状态异常已触发返航与降落组合："
                f"{snapshot.vehicle_abnormal_reason or '--'}",
            )
        self._submit_sequence_waypoints(sequence)
        if kind == "abnormal" and sequence.ticket is not None:
            self._abnormal_return_latched = True
        return sequence.ticket is not None

    def _handle_vehicle_abnormal(
        self, snapshot: VehicleSnapshot, *, upstream_connected: bool
    ) -> None:
        """按 WebSocket 和环境类型处理机载异常边沿。"""
        if not snapshot.vehicle_abnormal:
            self._abnormal_return_latched = False
            self._hardware_abnormal_notice_latched = False
            return
        if not upstream_connected:
            # 上位机离线时禁用通讯协议附带的异常自动返航策略。
            self._hardware_abnormal_notice_latched = False
            return
        if self._connection_mode == "hardware":
            if not self._hardware_abnormal_notice_latched:
                reason = snapshot.vehicle_abnormal_reason or "原因未提供"
                message = (
                    "检测到实机无人机状态异常；当前仅提示地面站，"
                    f"未执行自动返航或降落：{reason}"
                )
                self._events.error("upstream", message)
                self.activity_banner.set_message(message, LogLevel.ERROR)
                self._hardware_abnormal_notice_latched = True
            # TODO(实机异常返航): 完善实机风险确认、接管和返航降落策略后实现。
            pass  # noqa: PIE790 - 用户要求显式保留空实现。
            return
        if self._connection_mode != "simulation":
            return
        sequence = self._upstream_sequence
        if sequence is not None and sequence.kind in {
            "return",
            "low_power",
            "emergency",
        }:
            sequence.kind = "abnormal"
            self._abnormal_return_latched = True
            return
        if self._abnormal_return_latched:
            return
        if not snapshot.armed:
            try:
                in_hangar = self._upstream.is_in_hangar(snapshot)
            except Exception as exc:
                self._events.warn("upstream", f"机库位置判定失败：{exc}")
                return
            if not in_hangar:
                return
            try:
                self._upstream.block_standby()
            except Exception as exc:
                self._events.warn("upstream", f"待机状态锁定失败：{exc}")
            self._upstream_sequence = _UpstreamFlightSequence(
                kind="abnormal", phase="standby"
            )
            self._abnormal_return_latched = True
            return
        self._start_return_sequence("abnormal", snapshot)

    def _submit_sequence_waypoints(
        self, sequence: _UpstreamFlightSequence
    ) -> None:
        """只在当前序列仍有效时下发其原子航点任务。"""
        if self._upstream_sequence is not sequence:
            return
        # 先切换到不可重入阶段；_send_waypoints 可能会触发 Qt 刷新。
        sequence.phase = "waypoints_submitting"
        ticket = self._send_waypoints(
            sequence.waypoints,
            sequence.strategy,
            sequence.reference_generator,
            sequence.tracking_controller,
            sequence.photo_nos,
            allow_running_override=sequence.kind != "inspection",
            refresh_after_queue=False,
        )
        if ticket is None:
            self._abort_upstream_sequence("航点阶段未进入本地发送队列")
            return
        sequence.phase = "waypoints"
        sequence.ticket = ticket
        mission_kind = "inspection" if sequence.kind == "inspection" else "return"
        try:
            self._upstream.begin_mission(
                ticket, mission_kind, sequence.point_indexes
            )
        except Exception as exc:
            self._events.warn("upstream", f"任务状态关联失败：{exc}")
        if sequence.kind != "inspection":
            # 新航点请求由机载端原子覆盖旧任务；GUI 同步显示真实返航点。
            self.waypoints.replace_waypoints(
                sequence.waypoints, "上位机返航组合"
            )
            self._upstream_point_indexes = (1,)
            self._upstream_photo_nos = ()
        self._events.info(
            "upstream",
            f"{sequence.kind} 组合已排队 {len(sequence.waypoints)} 个航点",
        )
        self._refresh()

    def _observe_upstream_sequence_result(self, result: object) -> None:
        """仅用当前阶段 ticket 的可靠终态推进下一阶段。"""
        from ..models import CommandResult

        if not isinstance(result, CommandResult) or not result.final:
            return
        sequence = self._upstream_sequence
        if sequence is None or result.ticket != sequence.ticket:
            return
        if not result.success:
            self._abort_upstream_sequence(
                f"{sequence.phase} 阶段失败：{result.message}"
            )
            return
        if sequence.phase == "takeoff":
            sequence.phase = "waypoints_pending"
            sequence.ticket = None
        elif sequence.phase == "waypoints":
            sequence.phase = "land_pending"
            sequence.ticket = None
        elif sequence.phase == "land":
            sequence.phase = "standby"
            sequence.ticket = None
            delay = (
                self._upstream.standby_policy.inspection_delay_seconds
                if sequence.kind == "inspection"
                else 0.0
            )
            sequence.standby_not_before = time.monotonic() + delay
            self._events.info(
                "upstream",
                f"{sequence.kind} 已降落，待机判定将在 {delay:.1f} s 后开放",
            )
        elif sequence.phase == "clear_abnormal":
            self._complete_upstream_sequence("异常已清除，允许发送待机 01")

    def _drive_upstream_sequence(self, snapshot: VehicleSnapshot) -> None:
        """在每次权威快照刷新后驱动已就绪但尚未下发的阶段。"""
        sequence = self._upstream_sequence
        if sequence is None:
            return
        if sequence.phase == "waypoints_pending":
            if not snapshot.armed:
                return
            allowed = (
                self._availability.waypoint_send
                if sequence.kind == "inspection"
                else self._availability.hover
            )
            if allowed:
                self._submit_sequence_waypoints(sequence)
            return
        if sequence.phase == "land_pending":
            if not snapshot.armed:
                sequence.phase = "standby"
                sequence.standby_not_before = time.monotonic()
                return
            if not self._availability.land:
                return
            # 先离开 pending，防止排队后的刷新重复下发 LAND。
            sequence.phase = "land_submitting"
            ticket = self._land(refresh_after_queue=False)
            if ticket is None:
                self._abort_upstream_sequence("降落阶段未获得本地确认")
                return
            sequence.phase = "land"
            sequence.ticket = ticket
            try:
                self._upstream.begin_landing(ticket)
            except Exception as exc:
                self._events.warn("upstream", f"降落状态关联失败：{exc}")
            self._refresh()
            return
        if sequence.phase != "standby":
            return
        if snapshot.armed or time.monotonic() < sequence.standby_not_before:
            return
        try:
            in_hangar = self._upstream.is_in_hangar(snapshot)
        except Exception as exc:
            self._events.warn("upstream", f"机库位置判定失败：{exc}")
            return
        if not in_hangar:
            return
        if sequence.kind == "abnormal" and snapshot.vehicle_abnormal:
            # TODO(异常清除): 现阶段的近原点判定只是机库证据占位；
            # 未来必须在此叠加机库硬件确认后才允许清除。
            ticket = self._ros.request_clear_abnormal()
            self._pending_commands.add("clear_abnormal")
            sequence.phase = "clear_abnormal"
            sequence.ticket = ticket
            self._events.info("upstream", "已入库并请求清除无人机异常状态")
            return
        self._complete_upstream_sequence("已降落且位于机库阈值内")

    def _abort_upstream_sequence(self, reason: str) -> None:
        """如实终止组合，不伪造后续阶段成功。"""
        sequence = self._upstream_sequence
        self._upstream_sequence = None
        try:
            self._upstream.release_standby()
        except Exception as exc:
            self._events.warn("upstream", f"待机状态解锁失败：{exc}")
        label = sequence.kind if sequence is not None else "upstream"
        message = f"{label} 组合已终止：{reason}"
        self._events.error("upstream", message)
        self.activity_banner.set_message(message, LogLevel.ERROR)

    def _complete_upstream_sequence(self, reason: str) -> None:
        """完成组合并解锁待机投影；01 仍由权威快照判定。"""
        sequence = self._upstream_sequence
        self._upstream_sequence = None
        try:
            self._upstream.release_standby()
        except Exception as exc:
            self._events.warn("upstream", f"待机状态解锁失败：{exc}")
        label = sequence.kind if sequence is not None else "upstream"
        self._events.info("upstream", f"{label} 组合完成：{reason}")

    def _point_indexes_for(
        self, waypoints: tuple[tuple[float, float, float, float], ...]
    ) -> tuple[int, ...]:
        """保留命令 02 的点号；本地编辑后安全回退为当前 GUI 顺序号。"""
        if len(self._upstream_point_indexes) == len(waypoints):
            return self._upstream_point_indexes
        return tuple(range(1, len(waypoints) + 1))

    def _photo_nos_for(
        self, waypoints: tuple[tuple[float, float, float, float], ...]
    ) -> tuple[str, ...]:
        """只在列表仍对应最近命令 02 时带出甲方 photoNo。"""
        if len(self._upstream_photo_nos) == len(waypoints):
            return self._upstream_photo_nos
        return ()

    def _handle_ignored_upstream_point_fields(
        self, command: UpstreamCommand
    ) -> None:
        """集中记录仍未消费的云台字段；photoNo 已随航点进入机载端。"""
        if not command.ignored_camera_fields:
            return
        self._events.info(
            "upstream",
            "命令 02 的 photoNo 已随航点传给视频服务；cameraAngle 仍按要求忽略",
        )

    def _log_coordinate_mode_change(self, mode: str, label: str) -> None:
        """将操作者切换的手动坐标系及其输入语义写入结构化日志。"""
        detail = (
            "右摇杆增量按最新机头航向旋转到本地 ENU"
            if mode == "body"
            else "右摇杆增量沿本地 ENU 固定 X/Y 轴发送"
        )
        self._events.info(
            "operator", f"手动操纵坐标系切换为「{label}」：{detail}"
        )

    def _takeoff(self, *, refresh_after_queue: bool = True) -> int | None:
        """仿真直接请求起飞；实机仍须高风险确认。"""
        altitude = self.operations.takeoff_altitude()
        simulation = self._connection_mode == "simulation"
        if not simulation:
            if not self._confirm_action(
                "确认起飞",
                f"飞行器将尝试切换 GUIDED、武装并起飞至 {altitude:.1f} m。\n\n"
                "请确认螺旋桨区域无人、飞行空间安全且可随时人工接管。",
                critical=True,
            ):
                return None
            self._events.warn("operator", f"操作者确认起飞至 {altitude:.1f} m")
        else:
            self._events.info("operator", f"仿真模式请求起飞至 {altitude:.1f} m")
        self._pending_commands.add("takeoff")
        ticket = self._ros.request_takeoff(altitude)
        self.activity_banner.set_message("起飞请求已发送，等待机载确认…", LogLevel.WARN)
        if refresh_after_queue:
            self._refresh()
        return ticket

    def _land(self, *, refresh_after_queue: bool = True) -> int | None:
        """仿真直接请求 LAND；实机仍须危险操作确认。"""
        simulation = self._connection_mode == "simulation"
        if not simulation:
            if not self._confirm_action(
                "确认降落",
                "飞行器将切换 LAND 并开始下降。请确认降落区安全。",
                critical=True,
            ):
                return None
            self._events.warn("operator", "操作者确认发送 LAND")
        else:
            self._events.info("operator", "仿真模式请求发送 LAND")
        self._pending_commands.add("land")
        ticket = self._ros.request_land()
        sequence = self._upstream_sequence
        if refresh_after_queue and sequence is not None and sequence.phase == "land":
            # 人工重发会让机载端的新 LAND ticket 覆盖旧 ticket；组合锁
            # 必须同步重绑，否则解除武装后仍会永久等待已失效的终态。
            sequence.ticket = ticket
            try:
                self._upstream.begin_landing(ticket)
            except Exception as exc:
                self._events.warn("upstream", f"降落状态重绑失败：{exc}")
        self.activity_banner.set_message("降落请求已发送，等待机载确认…", LogLevel.WARN)
        if refresh_after_queue:
            self._refresh()
        return ticket

    def _send_motion(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """发送一次速度/偏航增量，持续状态仍由机载端持有。"""
        if not self._availability.motion:
            return
        self._ros.adjust_velocity(vx, vy, vz, yaw_rate)
        self.operations.mark_manual_command()
        self._events.debug(
            "operator",
            f"运动增量 V=({vx:+.2f},{vy:+.2f},{vz:+.2f}) yaw={yaw_rate:+.2f}",
        )
        self.activity_banner.set_message("运动意图已发送。", LogLevel.INFO)

    def _hover(self) -> None:
        """请求机载端抓取当前位置并进入悬停。"""
        if not self._availability.hover:
            return
        self._ros.request_hover()
        self.operations.mark_manual_command()
        self._events.info("operator", "操作者请求悬停")
        self.activity_banner.set_message("悬停请求已发送。", LogLevel.INFO)

    def _send_waypoints(
        self,
        waypoints: object,
        strategy: object | None = None,
        reference_generator: object | None = None,
        tracking_controller: object | None = None,
        photo_nos: object | None = None,
        *,
        allow_running_override: bool = False,
        refresh_after_queue: bool = True,
    ) -> int | None:
        """确认组合语义后原子上传航点与三项飞行前锁定配置。"""
        from ..models import WaypointFlightStrategy

        values = tuple(waypoints)
        permitted = (
            self._availability.hover
            if allow_running_override
            else self._availability.waypoint_send
        )
        if not values or not permitted:
            return None
        selected = (
            strategy
            if strategy is not None
            else self.waypoints.selected_strategy()
        )
        flight_strategy = WaypointFlightStrategy.from_value(selected)
        selected_generator = (
            reference_generator
            if reference_generator is not None
            else self.waypoints.selected_reference_generator()
        )
        generator = WaypointReferenceGenerator.from_value(selected_generator)
        selected_controller = (
            tracking_controller
            if tracking_controller is not None
            else self.waypoints.selected_tracking_controller()
        )
        controller = WaypointTrackingController.from_value(selected_controller)
        first = values[0]
        last = values[-1]
        combination_warnings: list[str] = []
        if flight_strategy is not WaypointFlightStrategy.STRAIGHT:
            combination_warnings.append(
                f"避障策略「{flight_strategy.label}」尚未实现，机载端将按直线飞行执行。"
            )
        recommended_pair = (
            generator is WaypointReferenceGenerator.STEP_POSITION
            and controller is WaypointTrackingController.POSITION_PD_DOB
        ) or (
            generator is not WaypointReferenceGenerator.STEP_POSITION
            and controller is WaypointTrackingController.TRAJECTORY_PD_DOB
        )
        if not recommended_pair:
            combination_warnings.append(
                f"「{generator.label} + {controller.label}」不是已验证搭配；"
                "本次只保证基线以及三种连续参考 + 轨迹 PD+DOB 组合。"
            )
        if combination_warnings and not self._confirm_action(
            "实验组合警告",
            "\n\n".join(combination_warnings)
            + "\n\n如仍要执行，请确认已理解该组合未经调试。",
            critical=True,
        ):
            return None

        simulation = self._connection_mode == "simulation"
        if not simulation:
            if not self._confirm_action(
                "确认执行航点任务",
                f"即将上传并执行 {len(values)} 个本地 ENU 航点。\n"
                f"飞行策略：{flight_strategy.label}\n"
                f"命令生成：{generator.label}\n"
                f"跟踪控制：{controller.label}\n"
                f"首点 ({first[0]:+.1f}, {first[1]:+.1f}, {first[2]:+.1f})，"
                f"末点 ({last[0]:+.1f}, {last[1]:+.1f}, {last[2]:+.1f})。\n\n"
                "任务执行将覆盖当前手动/悬停模式，确认继续吗？",
                critical=True,
            ):
                return None
            self._events.warn(
                "operator",
                f"操作者确认执行 {len(values)} 个航点（避障={flight_strategy.label}，"
                f"命令生成={generator.label}，跟踪={controller.label}）",
            )
        else:
            self._events.info(
                "operator",
                f"仿真模式请求执行 {len(values)} 个航点"
                f"（避障={flight_strategy.label}，命令生成={generator.label}，"
                f"跟踪={controller.label}）",
            )
        self._waypoint_running = True
        self._pending_commands.add("waypoints")
        # 新任务先清掉上一任务的完成进度；同一任务后续 RUNNING 回报不会重置。
        self.waypoints.reset_progress()
        self.waypoints.set_result("航点已排队，等待机载服务接收…", running=True)
        ticket = self._ros.request_waypoints(
            values,
            flight_strategy,
            generator,
            controller,
            tuple(photo_nos) if photo_nos is not None else (),
        )
        self._active_waypoint_ticket = ticket
        if refresh_after_queue:
            self._refresh()
        return ticket

    def _confirm_clear_waypoints(self) -> None:
        """避免误触清空尚未上传的任务编辑结果。"""
        count = len(self.waypoints.waypoints)
        if not count:
            return
        if self._confirm_action(
            "清空航点",
            f"确认删除本地列表中的 {count} 个航点吗？该操作不会取消机载任务。",
        ):
            self.waypoints.clear_waypoints()
            self._refresh()

    def _preview_waypoints(self) -> None:
        """打开/复用当前会话 RViz，并发布只读航点与直线路径快照。"""
        values = self.waypoints.waypoints
        if not values or not self._availability.waypoint_preview:
            return
        try:
            window_message = self._environment.ensure_waypoint_preview(
                self._connection_mode
            )
            if not self._ros.publish_waypoint_preview(values):
                raise RuntimeError("ROS 预览发布者未就绪")
        except Exception as exc:
            self._waypoint_preview_active = False
            message = f"航点预览失败：{exc}"
            self._events.error("visualization", message)
            self.activity_banner.set_message(message, LogLevel.ERROR)
            return

        self._waypoint_preview_active = True
        message = (
            f"{window_message}；已预览 {len(values)} 个航点和 "
            f"{max(0, len(values) - 1)} 段直线。"
        )
        self._events.info("visualization", message)
        self.activity_banner.set_message(message, LogLevel.INFO)

    def _on_waypoints_changed(self, message: str) -> None:
        """记录编辑；预览开启后持续替换快照，空列表也会清除旧 Marker。"""
        # 普通 GUI 编辑采用当前顺序号；命令 02 的原始点号会在替换后重新写回。
        self._upstream_point_indexes = tuple(
            range(1, len(self.waypoints.waypoints) + 1)
        )
        self._upstream_photo_nos = ()
        self._events.debug("waypoint-editor", message)
        if not self._waypoint_preview_active:
            return
        if not self._ros.publish_waypoint_preview(self.waypoints.waypoints):
            self._waypoint_preview_active = False
            self.activity_banner.set_message(
                "航点列表已更新，但 RViz 预览发布者不可用，请重新点击预览。",
                LogLevel.WARN,
            )

    def _choose_waypoint_file(self) -> None:
        """打开单文件选择器，并把选中的受支持文件交给统一导入流程。"""
        if not self._availability.waypoint_edit:
            return
        example_directory = PROJECT_ROOT / "examples"
        initial_directory = (
            example_directory if example_directory.is_dir() else PROJECT_ROOT
        )
        selected_path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "选择航点文件",
            str(initial_directory),
            waypoint_file_dialog_filter(),
        )
        if selected_path:
            self._import_waypoint_file(Path(selected_path))

    def _import_dropped_waypoint_files(self, dropped: object) -> None:
        """接收列表拖放路径；一次只允许一个文件进入导入流程。"""
        if isinstance(dropped, (str, Path)):
            paths = (Path(dropped),)
        else:
            try:
                paths = tuple(Path(item) for item in dropped)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                paths = ()
        if len(paths) != 1:
            self._show_waypoint_import_error("一次只能导入一个航点文件")
            return
        self._import_waypoint_file(paths[0])

    def _import_waypoint_file(self, path: Path) -> None:
        """完整解析后默认取消地确认，确认成功才原子替换当前列表。"""
        if not self._availability.waypoint_edit:
            self._show_waypoint_import_error(
                "当前航点列表不可编辑，请先启动仿真或连接机载服务"
            )
            return
        try:
            imported = load_waypoint_file(path)
        except WaypointImportError as exc:
            self._show_waypoint_import_error(str(exc))
            return

        existing_count = len(self.waypoints.waypoints)
        source_label = path.name or str(path)
        if not self._confirm_action(
            "从文件导入航点",
            f"已从 {source_label} 读取 {len(imported)} 个航点"
            f"（单次上限 {MAX_WAYPOINT_COUNT} 个）。\n"
            "X/Y/Z 将按绝对本地 ENU 坐标（米）导入，Yaw 按角度读取。\n\n"
            f"继续将清空并替换当前列表中的 {existing_count} 个航点。"
            "该操作不会取消已发送到机载端的任务。",
        ):
            self._events.debug(
                "waypoint-import", f"操作者取消从 {source_label} 导入航点"
            )
            return
        # 模态确认期间安全状态仍可能变化，替换前再次执行编辑门检查。
        if not self._availability.waypoint_edit:
            self._show_waypoint_import_error("航点编辑已被锁定，本次未替换列表")
            return

        self.waypoints.replace_waypoints(imported, source_label)
        self._events.info(
            "waypoint-import",
            f"已从 {source_label} 导入 {len(imported)} 个绝对 ENU 航点，"
            f"替换原列表 {existing_count} 个航点",
        )
        self.activity_banner.set_message(
            f"已导入 {len(imported)} 个航点。", LogLevel.INFO
        )
        self._refresh()

    def _show_waypoint_import_error(self, message: str) -> None:
        """统一记录并显示不会改变现有列表的导入失败。"""
        self._events.warn("waypoint-import", f"航点导入失败：{message}")
        self.activity_banner.set_message(f"航点导入失败：{message}", LogLevel.WARN)
        self._show_notice("航点导入失败", message, QMessageBox.Icon.Warning)

    # ---- 周期刷新与结果 ----

    def _refresh(self) -> None:
        """统一刷新快照、按钮状态、命令结果和日志，不跨线程触碰 Qt。"""
        snapshot = self._ros.snapshot()
        try:
            upstream_connected = bool(self._upstream.snapshot().connected)
        except Exception as exc:
            # 无法确认连接即按断线处理，飞行动作保持故障关闭。
            upstream_connected = False
            self._events.warn("upstream", f"读取上位机连接状态失败：{exc}")
        self._update_status_badges(snapshot)
        self.operations.update_snapshot(snapshot, self._connection_mode)
        self.waypoints.update_progress(snapshot)
        self._consume_video_results()
        self._consume_results()

        cleanup_active = (
            self._cleanup_thread is not None and self._cleanup_thread.is_alive()
        )
        busy = (
            self._workflow_busy
            or self._communication_busy
            or cleanup_active
            or bool(getattr(self._environment, "busy", False))
        )
        self._availability = derive_availability(
            snapshot,
            ros_ready=bool(self._ros.ready),
            busy=busy,
            closing=self._shutting_down,
            environment_active=self._environment_active,
            connection_mode=self._connection_mode,
            pending_mode=self._pending_environment_mode,
            waypoint_count=len(self.waypoints.waypoints),
            waypoint_running=self._waypoint_running,
            flight_sequence_active=self._upstream_sequence is not None,
        )
        render_key = (
            self._availability,
            self._shutting_down,
            self._communication_busy,
            self._communication_cancel_pending,
            cleanup_active,
            frozenset(self._pending_commands),
        )
        if render_key != self._last_availability_render_key:
            self.operations.apply_availability(
                self._availability,
                closing=self._shutting_down,
                communication_running=self._communication_busy,
                communication_cancel_pending=self._communication_cancel_pending,
            )
            self.exit_button.setEnabled(not self._shutting_down)
            self.waypoints.apply_availability(self._availability)
            if cleanup_active:
                self.operations.stop_simulation_button.setEnabled(False)
                self.operations.disconnect_hardware_button.setEnabled(False)
            # 起飞仍严格防重入；LAND 必须能在前一请求未终结时幂等重发。
            if "takeoff" in self._pending_commands:
                self.operations.takeoff_button.setEnabled(False)
            self._last_availability_render_key = render_key

        self._drive_upstream_sequence(snapshot)
        self._handle_vehicle_abnormal(
            snapshot, upstream_connected=upstream_connected
        )

        try:
            low_power_return_required = self._upstream.observe_vehicle(
                snapshot,
                self._connection_mode,
                can_takeoff=self._availability.takeoff,
            )
            if (
                not upstream_connected
                or self._connection_mode != "hardware"
                or not snapshot.armed
            ):
                self._hardware_low_power_notice_latched = False
            if (
                low_power_return_required
                and upstream_connected
                and self._connection_mode == "simulation"
            ):
                self._start_return_sequence("low_power", snapshot)
            elif (
                low_power_return_required
                and upstream_connected
                and self._connection_mode == "hardware"
            ):
                if not self._hardware_low_power_notice_latched:
                    message = (
                        "检测到实机低电量；当前仅提示地面站，"
                        "未执行自动返航或降落"
                    )
                    self._events.warn("upstream", message)
                    self.activity_banner.set_message(message, LogLevel.WARN)
                    self._hardware_low_power_notice_latched = True
                # TODO(实机低电量返航): 完善实机阈值、接管和返航降落策略后实现。
                pass  # noqa: PIE790 - 用户要求显式保留空实现。
        except Exception as exc:
            # 通讯插件任何故障都不得中断 10 Hz 原地面站刷新。
            self._events.warn("upstream", f"状态投影异常：{exc}")

        if self._ros.error and self._ros.error != self._last_ros_error:
            self._last_ros_error = self._ros.error
            self.activity_banner.set_message(
                f"ROS 2 客户端错误：{self._ros.error}", LogLevel.ERROR
            )
        self.log_panel.poll()
        source_id = getattr(self._ros, "source_id", "--")
        status_message = f"接口 {INTERFACE_VERSION} · source_id={source_id}"
        if self.statusBar().currentMessage() != status_message:
            self.statusBar().showMessage(status_message)

    def _consume_results(self) -> None:
        """增量显示可靠命令结果；日志等级已由 ROS 源端同步生成。"""
        for result in self._ros.results_after(self._last_result_sequence):
            self._last_result_sequence = max(
                self._last_result_sequence, result.sequence
            )
            stale_waypoint_result = (
                result.command == "waypoints"
                and self._active_waypoint_ticket is not None
                and result.ticket != self._active_waypoint_ticket
            )
            if result.final and not stale_waypoint_result:
                self._pending_commands.discard(result.command)
            level = LogLevel.INFO if result.success else LogLevel.ERROR
            if not stale_waypoint_result:
                self.activity_banner.set_message(result.message, level)
            try:
                self._upstream.observe_result(result)
            except Exception as exc:
                self._events.warn("upstream", f"命令结果投影异常：{exc}")
            self._observe_upstream_sequence_result(result)
            if result.command == "waypoints" and not stale_waypoint_result:
                running = result.success and not result.final
                if result.final:
                    completed_current_task = (
                        self._active_waypoint_ticket == result.ticket
                    )
                    self._waypoint_running = False
                    self._active_waypoint_ticket = None
                    if result.success and completed_current_task:
                        self.waypoints.complete_progress(
                            result.waypoint_index, result.waypoint_count
                        )
                self.waypoints.set_result(result.message, running=running)

    def _consume_video_results(self) -> None:
        """把独立视频状态与截图事件投影到上位机，不参与飞行按钮门控。"""
        video_snapshot = getattr(self._ros, "video_snapshot", None)
        observe_status = getattr(self._upstream, "observe_video_status", None)
        if callable(video_snapshot) and callable(observe_status):
            try:
                observe_status(video_snapshot())
            except Exception as exc:
                self._events.warn("upstream", f"视频状态投影异常：{exc}")

        results_after = getattr(self._ros, "video_capture_results_after", None)
        observe_capture = getattr(self._upstream, "observe_video_capture", None)
        if not callable(results_after) or not callable(observe_capture):
            return
        try:
            events = results_after(self._last_video_result_sequence)
        except Exception as exc:
            self._events.warn("upstream", f"读取视频截图结果失败：{exc}")
            return
        for event in events:
            self._last_video_result_sequence = max(
                self._last_video_result_sequence,
                int(getattr(event, "event_sequence", 0)),
            )
            try:
                observe_capture(event)
            except Exception as exc:
                self._events.warn("upstream", f"截图结果投影异常：{exc}")

    def _update_status_badges(self, snapshot: VehicleSnapshot) -> None:
        """把权威快照压缩成四个可快速扫描的状态卡。"""
        if self._ros.error:
            self.link_badge.set_status("ROS ERROR", "bad", self._ros.error)
        elif snapshot.onboard_available and snapshot.control_authority:
            self.link_badge.set_status(
                "ONLINE · CONTROL",
                "good",
                f"接口 {snapshot.interface_version}；租约持有者 {snapshot.lease_owner}",
            )
        elif snapshot.onboard_available:
            self.link_badge.set_status(
                "ONLINE · READ ONLY",
                "warn",
                f"租约持有者：{snapshot.lease_owner or '无'}",
            )
        elif self._ros.ready:
            self.link_badge.set_status("WAITING ONBOARD", "warn")
        else:
            self.link_badge.set_status("ROS IDLE", "neutral", "首次连接操作时启动")

        if not snapshot.connected:
            self.aircraft_badge.set_status("FCU OFFLINE", "neutral")
        elif snapshot.armed:
            self.aircraft_badge.set_status(
                "ARMED", "bad", f"ArduPilot 模式：{snapshot.autopilot_mode or '--'}"
            )
        else:
            self.aircraft_badge.set_status(
                "CONNECTED · DISARMED",
                "good",
                f"ArduPilot 模式：{snapshot.autopilot_mode or '--'}",
            )

        if snapshot.vehicle_abnormal:
            self.mode_badge.set_status(
                "无人机状态异常",
                "bad",
                snapshot.vehicle_abnormal_reason
                or f"航点稳定判定失败 {snapshot.waypoint_arrival_failure_count} 次",
            )
        else:
            mode_tone = (
                "bad"
                if snapshot.active_mode is FlightMode.FAILSAFE
                else "accent"
                if snapshot.active_mode is not FlightMode.IDLE
                else "neutral"
            )
            self.mode_badge.set_status(
                snapshot.active_mode.value,
                mode_tone,
                snapshot.status_message or snapshot.failsafe_reason,
            )

        if snapshot.setpoint_conflict:
            self.control_badge.set_status("SETPOINT CONFLICT", "bad")
        elif snapshot.controller_active:
            self.control_badge.set_status(
                f"{snapshot.control_rate_hz:.1f} Hz",
                "good",
                f"最大抖动 {snapshot.max_jitter_ms:.3f} ms；"
                f"超期 {snapshot.deadline_miss_count} 次；"
                f"hover throttle {snapshot.hover_throttle:.3f}",
            )
        elif snapshot.onboard_available and (
            not snapshot.local_position_valid or not snapshot.thrust_mode_verified
        ):
            self.control_badge.set_status("PREFLIGHT WAIT", "warn")
        else:
            self.control_badge.set_status("IDLE", "neutral")

        names = {
            "none": "未初始化",
            "simulation": "本地 SITL 仿真",
            "hardware": "实机机载服务",
        }
        set_text_if_changed(
            self.connection_label, f"环境 · {names[self._connection_mode]}"
        )

    # ---- 快捷键与对话框 ----

    @staticmethod
    def _focus_is_input() -> bool:
        """输入/日志/按钮聚焦时不把按键解释为飞行命令。"""
        focus = QApplication.focusWidget()
        return isinstance(
            focus,
            (
                QLineEdit,
                QAbstractSpinBox,
                QComboBox,
                QTextEdit,
                QPlainTextEdit,
                QAbstractButton,
            ),
        )

    def _shortcut_motion(self, name: str) -> None:
        """仅在非输入焦点且对应按钮可用时触发统一摇杆入口。"""
        if (
            self._focus_is_input()
            or not self.operations.motion_buttons[name].isEnabled()
        ):
            return
        self.operations.trigger_motion(name)

    def _shortcut_hover(self) -> None:
        """Space 只在非输入焦点且悬停可用时生效。"""
        if self._focus_is_input() or not self.operations.hover_button.isEnabled():
            return
        self._hover()

    def _confirm_action(
        self, title: str, message: str, critical: bool = False
    ) -> bool:
        """用默认取消的模态框确认危险或不可逆操作。"""
        dialog = self._message_box(
            title,
            message,
            QMessageBox.Icon.Critical if critical else QMessageBox.Icon.Warning,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        yes_button = dialog.button(QMessageBox.StandardButton.Yes)
        if yes_button is not None:
            yes_button.setText("确认执行")
        cancel_button = dialog.button(QMessageBox.StandardButton.Cancel)
        if cancel_button is not None:
            cancel_button.setText("取消")
        return dialog.exec() == QMessageBox.StandardButton.Yes

    # ---- 安全退出 ----

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API
        """任何主动退出都先二次确认，再后台清理环境并停止 ROS。"""
        if self._allow_close:
            self._dispose_upstream_panel()
            event.accept()
            return
        if self._shutting_down:
            event.ignore()
            return
        snapshot = self._ros.snapshot()
        flight_active = snapshot.armed or snapshot.controller_active
        title = "飞行中退出地面站" if flight_active else "退出地面站"
        if flight_active:
            message = (
                "飞行器仍处于武装或机载控制活动状态。退出会释放控制租约，"
                "机载端随后按失联策略悬停并自动降落。\n\n确认退出吗？"
            )
        else:
            message = (
                "退出将释放控制租约、终止本项目启动的本地仿真进程和 RViz 预览进程，"
                "并停止地面站 ROS 客户端。\n\n确认退出吗？"
            )
        if not self._confirm_action(title, message, critical=flight_active):
            event.ignore()
            return
        event.ignore()
        self._begin_shutdown()

    def request_external_shutdown(self, reason: str) -> None:
        """终端或进程管理器请求退出时跳过交互确认并执行同一清理链。"""
        self._begin_shutdown(
            source="shutdown",
            message=f"收到 {reason} 终止信号，正在安全退出地面站",
        )

    def _begin_shutdown(
        self,
        *,
        source: str = "operator",
        message: str = "操作者请求退出地面站",
    ) -> None:
        """锁定界面并启动不会阻塞 Qt 事件循环的安全退出线程。"""
        if self._shutting_down:
            return
        self._shutting_down = True
        self._events.warn(source, message)
        self.activity_banner.set_message(
            "正在安全退出：释放租约、清理本地仿真并停止 ROS…",
            LogLevel.WARN,
        )
        self._refresh()

        def worker() -> None:
            report = self._cleanup_backend_once()
            self._bridge.shutdown_done.emit(report)

        self._shutdown_thread = threading.Thread(
            target=worker, name="ground-station-qt-shutdown", daemon=True
        )
        self._shutdown_thread.start()

    def _cleanup_backend_once(self) -> CleanupReport:
        """幂等释放租约、清理本地进程并停止 ROS，供所有退出路径复用。"""
        with self._shutdown_cleanup_lock:
            if self._shutdown_report is not None:
                return self._shutdown_report
            try:
                self._upstream.stop(timeout=3.0)
            except Exception as exc:
                # 插件退出失败只记录，主体仍继续释放飞行会话和 ROS。
                self._events.warn("upstream", f"停止上位机通讯模块失败：{exc}")
            try:
                # EnvironmentInitializer 会合并并发清理请求并设置硬超时；这里
                # 不再先 join 后二次进入清理，以免两条线程互等同一把锁。
                report = self._environment.cleanup()
                self._ros.stop()
            except Exception as exc:
                report = CleanupReport(errors=(str(exc),))
                self._events.error("shutdown", f"安全退出异常：{exc}")
            self._shutdown_report = report
            return report

    def finalize_process_exit(self) -> CleanupReport:
        """Qt 事件循环返回前的同步兜底；正常退出时直接复用已有结果。"""
        if not self._shutting_down:
            self._shutting_down = True
            self._events.warn(
                "shutdown", "Qt 事件循环已结束，执行地面站后端兜底清理"
            )
        return self._cleanup_backend_once()

    def _on_shutdown_done(self, report: CleanupReport) -> None:
        """完成最后一次日志刷新后允许 Qt 真正销毁窗口。"""
        if report.success:
            self._events.info("shutdown", "地面站安全退出完成")
        else:
            self._events.error(
                "shutdown",
                f"退出清理仍有残留：{report.remaining}，错误={report.errors}",
            )
        self.log_panel.poll()
        self._timer.stop()
        self._allow_close = True
        self.close()
