"""Qt 地面站复用卡片、状态徽章、数值输入和活动提示组件。"""

from __future__ import annotations

from PySide6.QtCore import QEvent, QPoint, Qt, QTimer
from PySide6.QtGui import QColor, QMouseEvent, QResizeEvent, QShowEvent, QWheelEvent
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGraphicsDropShadowEffect,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMenu,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..event_log import LogLevel


def repolish(widget: QWidget) -> None:
    """属性选择器变化后立即刷新目标控件样式。"""
    style = widget.style()
    style.unpolish(widget)
    style.polish(widget)
    widget.update()


def set_text_if_changed(widget: QLabel | QPushButton, text: str) -> bool:
    """仅在可见文本真正变化时写入，避免无效布局和重绘事件。"""
    if widget.text() == text:
        return False
    widget.setText(text)
    return True


def set_tooltip_if_changed(widget: QWidget, text: str) -> bool:
    """仅在提示文字变化时写入，避免周期刷新产生 ToolTipChange 风暴。"""
    if widget.toolTip() == text:
        return False
    widget.setToolTip(text)
    return True


class ShadowMessageBox(QMessageBox):
    """带自绘标题、完整边框和透明留边阴影的统一消息框。"""

    _SHADOW_MARGIN = 14

    def __init__(self, parent: QWidget | None = None) -> None:
        """创建无原生装饰但保留模态按钮语义的消息框。"""
        super().__init__(parent)
        self.setObjectName("shadowMessageBox")
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setContentsMargins(*([self._SHADOW_MARGIN] * 4))
        self.setMinimumWidth(430)

        self._surface = QFrame(self)
        self._surface.setObjectName("dialogSurface")
        self._surface.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents, True
        )
        shadow = QGraphicsDropShadowEffect(self._surface)
        shadow.setBlurRadius(28.0)
        shadow.setOffset(0.0, 3.0)
        shadow.setColor(QColor(17, 31, 46, 112))
        self._surface.setGraphicsEffect(shadow)
        self._surface.lower()
        self._title_bar: QFrame | None = None
        self._title_label: QLabel | None = None

    @property
    def surface(self) -> QFrame:
        """暴露阴影表面供视觉回归检查。"""
        return self._surface

    def _ensure_title_bar(self) -> None:
        """把自绘标题栏插到 QMessageBox 原有网格布局上方。"""
        if self._title_bar is not None:
            return
        layout = self.layout()
        if not isinstance(layout, QGridLayout):
            return

        # QMessageBox 没有插入行 API，先保存并整体下移原生图标、正文和按钮。
        positions = [
            layout.getItemPosition(index) for index in range(layout.count())
        ]
        existing = [(layout.takeAt(0), position) for position in positions]
        for item, (row, column, row_span, column_span) in existing:
            if item is not None:
                layout.addItem(item, row + 1, column, row_span, column_span)

        title_bar = QFrame()
        title_bar.setObjectName("dialogTitleBar")
        title_layout = QHBoxLayout(title_bar)
        title_layout.setContentsMargins(2, 0, 0, 5)
        title_layout.setSpacing(8)
        title_label = QLabel(self.windowTitle() or "提示")
        title_label.setObjectName("dialogTitle")
        close_button = QPushButton("×")
        close_button.setObjectName("dialogCloseButton")
        close_button.setProperty("windowControl", True)
        close_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        close_button.setToolTip("关闭")
        close_button.clicked.connect(self.reject)
        title_layout.addWidget(title_label, 1)
        title_layout.addWidget(close_button)
        title_bar.installEventFilter(self)
        title_label.installEventFilter(self)
        layout.addWidget(title_bar, 0, 0, 1, max(1, layout.columnCount()))
        self._title_bar = title_bar
        self._title_label = title_label

    def eventFilter(self, watched: object, event: QEvent) -> bool:  # noqa: N802
        """允许从自绘标题栏拖动消息框。"""
        if (
            watched in (self._title_bar, self._title_label)
            and event.type() == QEvent.Type.MouseButtonPress
            and isinstance(event, QMouseEvent)
            and event.button() == Qt.MouseButton.LeftButton
        ):
            handle = self.windowHandle()
            if handle is not None and handle.startSystemMove():
                event.accept()
                return True
        return super().eventFilter(watched, event)

    def resizeEvent(self, event: QResizeEvent) -> None:  # noqa: N802 - Qt API
        """始终把表面放在透明阴影留边内部。"""
        super().resizeEvent(event)
        self._update_surface_geometry()

    def _update_surface_geometry(self) -> None:
        """更新弹窗背景表面并保持它位于正文下方。"""
        margin = self._SHADOW_MARGIN
        geometry = self.rect().adjusted(margin, margin, -margin, -margin)
        self._surface.setGeometry(geometry)
        self._surface.lower()

    def showEvent(self, event: QShowEvent) -> None:  # noqa: N802 - Qt API
        """显示前补齐标题栏并刷新阴影几何。"""
        self._ensure_title_bar()
        super().showEvent(event)
        hint = self.sizeHint()
        self._target_dialog_size = (
            max(430, hint.width()),
            max(180, hint.height()),
        )
        self._lock_dialog_size()
        # QMessageBox 会在 showEvent 返回后再次收缩，稍后固定一次最终尺寸。
        QTimer.singleShot(25, self._lock_dialog_size)

    def _lock_dialog_size(self) -> None:
        """在 QMessageBox 延迟调整尺寸后恢复阴影和最小可读面积。"""
        target_width, target_height = getattr(
            self, "_target_dialog_size", (430, 180)
        )
        # Qt 会在 showEvent 返回后才完成正文换行；此时 sizeHint 可能比
        # 首次测量更高。重新取最终提示尺寸，避免长确认文字被旧高度裁掉。
        hint = self.sizeHint()
        target_width = max(target_width, 430, hint.width())
        target_height = max(target_height, 180, hint.height())
        self._target_dialog_size = (target_width, target_height)
        self.setFixedSize(target_width, target_height)
        self._update_surface_geometry()

    def exec(self) -> int:
        """进入模态循环前确保自绘标题已经安装。"""
        self._ensure_title_bar()
        return super().exec()


class NoWheelDoubleSpinBox(QDoubleSpinBox):
    """只接受键盘和步进按钮输入，滚轮用于滚动所属页面。"""

    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802 - Qt API
        """忽略滚轮增减，继续把事件交给父级滚动区域处理。"""
        event.ignore()


class DownwardComboBox(QComboBox):
    """用完整菜单替代会被桌面样式裁切的原生组合框弹层。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """创建复用弹出菜单，并将选择结果同步回组合框索引。"""
        super().__init__(parent)
        self._popup_menu = QMenu(self)
        self._popup_menu.setObjectName("downwardComboPopup")
        self._popup_menu.triggered.connect(self._select_popup_action)

    @property
    def popup_menu(self) -> QMenu:
        """暴露实际弹层，供几何回归和无障碍检查使用。"""
        return self._popup_menu

    def showPopup(self) -> None:  # noqa: N802 - Qt API
        """在控件正下方一次显示全部条目，不创建滚动视口。"""
        if self.count() <= 0:
            return
        self._popup_menu.clear()
        for index in range(self.count()):
            action = self._popup_menu.addAction(self.itemText(index))
            action.setData(index)
            action.setCheckable(True)
            action.setChecked(index == self.currentIndex())
            model_index = self.model().index(index, self.modelColumn())
            action.setEnabled(
                bool(self.model().flags(model_index) & Qt.ItemFlag.ItemIsEnabled)
            )
        self._popup_menu.ensurePolished()
        self._popup_menu.setMinimumWidth(
            max(self.width(), self._popup_menu.sizeHint().width())
        )
        self._popup_menu.popup(self.mapToGlobal(QPoint(0, self.height())))
        self._move_popup_below()
        # 窗口系统可能在 popup() 后再次约束位置，下一事件周期再校正。
        QTimer.singleShot(0, self._move_popup_below)

    def hidePopup(self) -> None:  # noqa: N802 - Qt API
        """关闭自定义完整菜单。"""
        self._popup_menu.hide()

    def _select_popup_action(self, action: object) -> None:
        """把菜单条目索引写回组合框，并保留用户激活信号语义。"""
        data = action.data() if hasattr(action, "data") else None
        if not isinstance(data, int) or not 0 <= data < self.count():
            return
        self.setCurrentIndex(data)
        self.activated.emit(data)
        self.textActivated.emit(self.itemText(data))
        self.hidePopup()

    def _move_popup_below(self) -> None:
        """把完整菜单左上角固定到组合框左下角。"""
        if not self._popup_menu.isVisible():
            return
        self._popup_menu.move(self.mapToGlobal(QPoint(0, self.height())))


class Card(QFrame):
    """带可选标题、悬停帮助和统一内容边距的工程卡片。"""

    def __init__(
        self, title: str, subtitle: str = "", parent: QWidget | None = None
    ) -> None:
        """创建卡片；空标题时完全省略标题行，供紧凑操作区复用。"""
        super().__init__(parent)
        self.setObjectName("card")
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 16)
        root.setSpacing(10)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("cardTitle")
        self.help_icon: QLabel | None = None
        if title or subtitle:
            title_row = QHBoxLayout()
            title_row.setSpacing(6)
            title_row.addWidget(self.title_label)
            if subtitle:
                help_icon = QLabel("?")
                help_icon.setObjectName("cardHelpIcon")
                help_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
                help_icon.setFixedSize(18, 18)
                help_icon.setToolTip(subtitle)
                help_icon.setAccessibleName(f"{title}帮助")
                help_icon.setAccessibleDescription(subtitle)
                title_row.addWidget(help_icon)
                self.help_icon = help_icon
            title_row.addStretch(1)
            root.addLayout(title_row)
        else:
            self.title_label.hide()

        self.content_layout = QVBoxLayout()
        self.content_layout.setSpacing(9)
        root.addLayout(self.content_layout)


class StatusBadge(QFrame):
    """顶部单行状态徽章，使用左侧色条表达状态。"""

    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        """创建标题和值标签。"""
        super().__init__(parent)
        self.setObjectName("statusBadge")
        self.setProperty("tone", "neutral")
        self.setMinimumHeight(38)
        self.setMaximumHeight(42)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(7)
        title_label = QLabel(title.upper())
        title_label.setObjectName("mutedLabel")
        self.value_label = QLabel("--")
        self.value_label.setObjectName("statusValue")
        self.value_label.setWordWrap(False)
        self.value_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        layout.addWidget(title_label)
        layout.addWidget(self.value_label, 1)

    def set_status(self, value: str, tone: str = "neutral", detail: str = "") -> None:
        """更新状态文本、语气色和完整详情提示。"""
        if self.property("tone") != tone:
            self.setProperty("tone", tone)
            repolish(self)
        set_text_if_changed(self.value_label, value)
        set_tooltip_if_changed(self, detail or value)


class ActivityBanner(QFrame):
    """在状态行内集中显示最近一次工作流或命令状态。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """创建固定前缀与可换行消息标签。"""
        super().__init__(parent)
        self.setObjectName("activityBanner")
        self.setProperty("tone", "debug")
        self.setMinimumHeight(38)
        self.setMaximumHeight(42)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(7)
        prefix = QLabel("动态")
        prefix.setObjectName("mutedLabel")
        self.message_label = QLabel("等待初始化仿真或连接实机机载服务")
        self.message_label.setWordWrap(False)
        layout.addWidget(prefix)
        layout.addWidget(self.message_label, 1)

    def set_message(self, message: str, level: LogLevel = LogLevel.DEBUG) -> None:
        """按源端日志等级更新提示语气，不在显示层重新分类。"""
        tones = {
            LogLevel.DEBUG: "debug",
            LogLevel.INFO: "info",
            LogLevel.WARN: "warn",
            LogLevel.ERROR: "error",
        }
        tone = tones[level]
        if self.property("tone") != tone:
            self.setProperty("tone", tone)
            repolish(self)
        set_text_if_changed(self.message_label, message)
        set_tooltip_if_changed(self, message)
