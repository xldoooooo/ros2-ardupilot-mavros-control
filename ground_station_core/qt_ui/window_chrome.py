"""独立 Qt 工具窗口复用的透明留边、自绘阴影和窗口控制。"""

from __future__ import annotations

from PySide6.QtCore import QEvent, Qt, QTimer
from PySide6.QtGui import QColor, QMouseEvent, QResizeEvent
from PySide6.QtWidgets import (
    QFrame,
    QGraphicsDropShadowEffect,
    QHBoxLayout,
    QLabel,
    QPushButton,
)


class _DraggableTitleBar(QFrame):
    """把自绘标题栏的拖动和双击交给窗口系统处理。"""

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        """左键按下空白标题区域时开始原生窗口拖动。"""
        if event.button() == Qt.MouseButton.LeftButton:
            handle = self.window().windowHandle()
            if handle is not None and handle.startSystemMove():
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        """双击标题栏切换最大化，并保持普通顶层窗口语义。"""
        if event.button() == Qt.MouseButton.LeftButton:
            window = self.window()
            window.showNormal() if window.isMaximized() else window.showMaximized()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)


class ShadowWindowChromeMixin:
    """为 QWidget/QMainWindow 顶层窗口增加可缩放的应用内阴影。"""

    _SHADOW_MARGIN = 14

    def _configure_window_chrome(self) -> None:
        """创建不依赖 Wayland 合成器的透明留边与自绘阴影。"""
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setMouseTracking(True)
        content_margin = self._SHADOW_MARGIN + 1
        self.setContentsMargins(*([content_margin] * 4))

        self.outer_window_frame = QFrame(self)
        self.outer_window_frame.setObjectName("subpanelWindowFrame")
        self.outer_window_frame.setProperty("windowMaximized", False)
        self.outer_window_frame.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents, True
        )
        self._window_shadow = QGraphicsDropShadowEffect(self.outer_window_frame)
        self._window_shadow.setBlurRadius(30.0)
        self._window_shadow.setOffset(0.0, 3.0)
        self._window_shadow.setColor(QColor(16, 30, 44, 118))
        self.outer_window_frame.setGraphicsEffect(self._window_shadow)
        self.outer_window_frame.lower()

    def _build_window_title_bar(self, title_text: str) -> QFrame:
        """提供可拖动标题与最小化、最大化和关闭控制。"""
        title_bar = _DraggableTitleBar()
        title_bar.setObjectName("subpanelTitleBar")
        layout = QHBoxLayout(title_bar)
        layout.setContentsMargins(12, 6, 7, 6)
        layout.setSpacing(3)
        title = QLabel(title_text)
        title.setObjectName("subpanelWindowTitle")
        title.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        layout.addWidget(title, 1)
        self.minimize_button = self._window_control_button("—", "最小化")
        self.maximize_button = self._window_control_button("□", "最大化")
        self.close_button = self._window_control_button("×", "关闭", close=True)
        self.minimize_button.clicked.connect(self.showMinimized)
        self.maximize_button.clicked.connect(self._toggle_maximized)
        self.close_button.clicked.connect(self.close)
        layout.addWidget(self.minimize_button)
        layout.addWidget(self.maximize_button)
        layout.addWidget(self.close_button)
        self.window_title_bar = title_bar
        return title_bar

    @staticmethod
    def _window_control_button(
        text: str, tooltip: str, *, close: bool = False
    ) -> QPushButton:
        """创建与主窗口一致的固定宽度窗口控制按钮。"""
        button = QPushButton(text)
        button.setProperty("windowControl", True)
        button.setProperty("closeControl", close)
        button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        button.setFixedWidth(31)
        button.setToolTip(tooltip)
        return button

    def _toggle_maximized(self) -> None:
        """切换最大化状态，阴影同步由窗口状态事件完成。"""
        self.showNormal() if self.isMaximized() else self.showMaximized()

    def _sync_window_chrome(self) -> None:
        """同步普通/最大化状态下的留边、投影和控制按钮。"""
        maximized = self.isMaximized() or self.isFullScreen()
        frame_margin = 0 if maximized else self._SHADOW_MARGIN
        content_margin = frame_margin + 1
        margins = (content_margin,) * 4
        current = self.contentsMargins()
        if (
            current.left(),
            current.top(),
            current.right(),
            current.bottom(),
        ) != margins:
            self.setContentsMargins(*margins)
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
            style = self.outer_window_frame.style()
            style.unpolish(self.outer_window_frame)
            style.polish(self.outer_window_frame)
            self.outer_window_frame.update()
        if hasattr(self, "maximize_button"):
            self.maximize_button.setText("❐" if maximized else "□")
            self.maximize_button.setToolTip("还原" if maximized else "最大化")

    def _resize_edges_at(self, x: float, y: float) -> Qt.Edge:
        """把透明留边命中位置转换为原生窗口缩放边。"""
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
        """返回缩放边或缩放角对应的鼠标指针。"""
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

    def resizeEvent(self, event: QResizeEvent) -> None:  # noqa: N802
        """窗口变化时保持背景框和阴影覆盖完整客户区。"""
        super().resizeEvent(event)
        self._sync_window_chrome()

    def changeEvent(self, event: QEvent) -> None:  # noqa: N802
        """窗口状态改变后刷新最大化边距和按钮。"""
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange:
            QTimer.singleShot(0, self._sync_window_chrome)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        """在透明阴影留边显示对应的缩放指针。"""
        cursor = self._resize_cursor(
            self._resize_edges_at(event.position().x(), event.position().y())
        )
        self.unsetCursor() if cursor is None else self.setCursor(cursor)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        """从透明留边发起窗口系统原生缩放。"""
        if event.button() == Qt.MouseButton.LeftButton:
            edges = self._resize_edges_at(event.position().x(), event.position().y())
            handle = self.windowHandle()
            if edges and handle is not None and handle.startSystemResize(edges):
                event.accept()
                return
        super().mousePressEvent(event)

    def leaveEvent(self, event: QEvent) -> None:  # noqa: N802
        """鼠标离开窗口时恢复默认指针。"""
        self.unsetCursor()
        super().leaveEvent(event)
