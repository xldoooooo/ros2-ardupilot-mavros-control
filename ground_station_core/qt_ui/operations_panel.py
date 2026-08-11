"""环境、起降、手动意图和遥测操作面板。"""

from __future__ import annotations

import math
from pathlib import Path
import time

from PySide6.QtCore import QPointF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QIcon, QPaintEvent, QPainter, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ..config import (
    DEFAULT_GPS_ORIGIN,
    LAND_SPEED,
    TAKEOFF_ALTITUDE,
    TAKEOFF_SPEED,
    VELOCITY_SCALE,
)
from ..models import VehicleSnapshot
from .state import UiAvailability
from .theme import COLORS
from .widgets import (
    Card,
    DownwardComboBox,
    NoWheelDoubleSpinBox,
    repolish,
    set_text_if_changed,
)


# 手动按钮描述的是“期望量增量”，避免把机载接受命令误写成已实现的物理运动。
_MANUAL_MOTION_TOOLTIPS = {
    "up": "向飞行器发送指令，增加向上的期望速度",
    "down": "向飞行器发送指令，增加向下的期望速度",
    "yaw_left": "向飞行器发送指令，增加向左偏航的期望角速度",
    "yaw_right": "向飞行器发送指令，增加向右偏航的期望角速度",
    "forward": "向飞行器发送指令，增加水平向前的期望速度；方向按当前坐标系解释",
    "back": "向飞行器发送指令，增加水平向后的期望速度；方向按当前坐标系解释",
    "left": "向飞行器发送指令，增加水平向左的期望速度；方向按当前坐标系解释",
    "right": "向飞行器发送指令，增加水平向右的期望速度；方向按当前坐标系解释",
}
_HOVER_TOOLTIP = "向机载服务发送制动并悬停指令，清零目标速度并保持当前位置"


class OriginConfigDialog(QDialog):
    """配置完整实机连接使用的 EKF/飞控原点，仅在本地保存。"""

    def __init__(
        self,
        origin: tuple[float, float, float],
        parent: QWidget | None = None,
    ) -> None:
        """用当前缓存原点填充表单。"""
        super().__init__(parent)
        self.setObjectName("originConfigDialog")
        self.setWindowTitle("配置飞控原点")
        self.setModal(True)
        self.setMinimumWidth(360)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        hint = QLabel(
            "此处仅缓存实机飞控/EKF 原点，完整连接实机服务时才写入。"
            "本地 SITL 使用自身 Home，Wi-Fi 通讯检测也不会写入该值。"
        )
        hint.setObjectName("mutedLabel")
        hint.setWordWrap(True)
        root.addWidget(hint)

        form = QFormLayout()
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(8)
        self.latitude_input = self._spin_box(-90.0, 90.0, 7, " °")
        self.longitude_input = self._spin_box(-180.0, 180.0, 7, " °")
        self.altitude_input = self._spin_box(-1000.0, 10000.0, 1, " m")
        self.latitude_input.setValue(origin[0])
        self.longitude_input.setValue(origin[1])
        self.altitude_input.setValue(origin[2])
        form.addRow("纬度", self.latitude_input)
        form.addRow("经度", self.longitude_input)
        form.addRow("海拔", self.altitude_input)
        root.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.button(QDialogButtonBox.StandardButton.Save).setText("保存")
        buttons.button(QDialogButtonBox.StandardButton.Cancel).setText("取消")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

    @staticmethod
    def _spin_box(
        minimum: float, maximum: float, decimals: int, suffix: str
    ) -> NoWheelDoubleSpinBox:
        """创建与主面板一致的无滚轮数值框。"""
        control = NoWheelDoubleSpinBox()
        control.setRange(minimum, maximum)
        control.setDecimals(decimals)
        control.setSingleStep(10 ** (-min(decimals, 3)))
        control.setSuffix(suffix)
        control.setProperty("compactValueInput", True)
        control.setKeyboardTracking(False)
        return control

    def origin(self) -> tuple[float, float, float]:
        """返回对话框中当前编辑的原点。"""
        return (
            self.latitude_input.value(),
            self.longitude_input.value(),
            self.altitude_input.value(),
        )


class _JoystickOffsetIndicator(QWidget):
    """在摇杆中央显示当前按键对应的二维偏移。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        """创建固定尺寸的回中圆盘指示器。"""
        super().__init__(parent)
        self._offset = (0.0, 0.0)
        self.setObjectName("joystickOffsetIndicator")
        self.setFixedSize(44, 26)
        self.setAccessibleName("摇杆实时偏移")

    @property
    def offset(self) -> tuple[float, float]:
        """返回当前归一化偏移，供界面回归检查。"""
        return self._offset

    def set_offset(self, x: float, y: float) -> None:
        """更新归一化偏移并立即重绘。"""
        self._offset = (
            max(-1.0, min(1.0, float(x))),
            max(-1.0, min(1.0, float(y))),
        )
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:  # noqa: N802 - Qt API
        """绘制底盘、回中十字与随按键移动的实心圆点。"""
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        bounds = self.rect().adjusted(4, 2, -4, -2)
        center = QPointF(bounds.center())
        painter.setPen(QPen(QColor(COLORS["border_strong"]), 1.2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(bounds)
        painter.setPen(QPen(QColor(COLORS["border"]), 1.0))
        painter.drawLine(
            QPointF(bounds.left() + 5, center.y()),
            QPointF(bounds.right() - 5, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), bounds.top() + 4),
            QPointF(center.x(), bounds.bottom() - 4),
        )
        dot = QPointF(
            center.x() + self._offset[0] * 11.0,
            center.y() + self._offset[1] * 5.0,
        )
        painter.setPen(QPen(QColor(COLORS["accent"]), 1.0))
        painter.setBrush(QBrush(QColor(COLORS["accent"])))
        painter.drawEllipse(dot, 4.5, 4.5)


class OperationsPanel(QWidget):
    """按上下两排组织连接、飞行动作与手动操纵，并发出纯 UI 意图。"""

    # 齿轮与 Wi-Fi 均使用紧凑正方形，给最小窗口下的两个文字按钮保留宽度。
    _AUXILIARY_BUTTON_SIZE = 36

    simulation_requested = Signal()
    hardware_requested = Signal()
    communication_test_requested = Signal()
    stop_simulation_requested = Signal()
    disconnect_hardware_requested = Signal()
    takeoff_requested = Signal()
    land_requested = Signal()
    hover_requested = Signal()
    motion_requested = Signal(float, float, float, float)
    coordinate_mode_changed = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        """构建上排双卡片与下排手动操纵卡片。"""
        super().__init__(parent)
        self.setMinimumWidth(650)
        self._origin = tuple(DEFAULT_GPS_ORIGIN)
        self._latest_yaw = 0.0
        self._last_manual_command_at: float | None = None
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 4, 0)
        root.setSpacing(10)

        # 第一排把连接与飞行动作并列；第二排手动操纵占满左侧工作区。
        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.environment_card = self._build_environment_card()
        self.flight_card = self._build_flight_card()
        for card in (self.environment_card, self.flight_card):
            card.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.Preferred,
            )
            card_layout = card.layout()
            if card_layout is not None:
                card_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        top_row.addWidget(self.environment_card, 1)
        top_row.addWidget(self.flight_card, 1)
        root.addLayout(top_row)

        self.manual_panel = QWidget()
        self.manual_panel.setObjectName("manualOperationsPanel")
        manual_layout = QVBoxLayout(self.manual_panel)
        manual_layout.setContentsMargins(0, 0, 0, 0)
        self.manual_card = self._build_manual_card()
        manual_layout.addWidget(self.manual_card)
        root.addWidget(self.manual_panel)
        root.addStretch(1)

    def _build_environment_card(self) -> Card:
        """创建仿真/实机入口、原点齿轮、零命令通讯检测与断开操作。"""
        card = Card(
            "环境与连接",
            "仿真仅管理本机进程；连接实机会申请控制租约并执行连接维护，\n"
            "但不会远程启动或终止机载服务。右侧 Wi-Fi 按钮仅检测通讯；\n"
            "本地 SITL 使用自身 Home。",
        )
        action_row = QHBoxLayout()
        self.simulation_button = self._button(
            "启动仿真", "primary", "simulationButton"
        )
        self.hardware_button = self._button(
            "连接实机", "primary", "hardwareButton"
        )
        # 正方形齿轮与通讯图标保持同一紧凑尺寸。
        self.origin_settings_button = self._button(
            "⚙", "neutral", "originSettingsButton"
        )
        self.origin_settings_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.origin_settings_button.setToolTip(
            "配置实机飞控/EKF 原点（仅本地保存；完整连接实机时写入）"
        )
        # 独立 Wi-Fi 按钮只触发纯订阅检测，位置固定在原点齿轮右侧。
        self.communication_test_button = self._button(
            "", "neutral", "communicationTestButton"
        )
        asset_directory = Path(__file__).resolve().parent / "assets"
        self._wifi_icon = QIcon(str(asset_directory / "wifi.svg"))
        self._communication_stop_icon = QIcon(
            str(asset_directory / "stop-square.svg")
        )
        self.communication_test_button.setIcon(self._wifi_icon)
        self.communication_test_button.setIconSize(QSize(20, 20))
        self.communication_test_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.communication_test_button.setAccessibleName("检测实机通讯链路")
        self.communication_test_button.setAccessibleDescription(
            "只接收状态与日志，不申请控制租约或发送命令"
        )
        self.simulation_button.clicked.connect(self.simulation_requested)
        self.hardware_button.clicked.connect(self.hardware_requested)
        self.origin_settings_button.clicked.connect(self._open_origin_settings)
        self.communication_test_button.clicked.connect(
            self.communication_test_requested
        )
        action_row.addWidget(self.simulation_button, 1)
        action_row.addWidget(self.hardware_button, 1)
        action_row.addWidget(
            self.origin_settings_button, 0, Qt.AlignmentFlag.AlignVCenter
        )
        action_row.addWidget(
            self.communication_test_button, 0, Qt.AlignmentFlag.AlignVCenter
        )
        card.content_layout.addLayout(action_row)
        self._size_auxiliary_buttons()

        self.origin_summary = QLabel()
        self.origin_summary.setObjectName("mutedLabel")
        self.origin_summary.setWordWrap(True)
        self._refresh_origin_summary()
        card.content_layout.addWidget(self.origin_summary)

        process_row = QHBoxLayout()
        self.stop_simulation_button = self._button(
            "终止本地仿真", "primary", "stopSimulationButton"
        )
        self.disconnect_hardware_button = self._button(
            "断开实机连接", "danger", "disconnectHardwareButton"
        )
        self.stop_simulation_button.clicked.connect(self.stop_simulation_requested)
        self.disconnect_hardware_button.clicked.connect(
            self.disconnect_hardware_requested
        )
        process_row.addWidget(self.stop_simulation_button, 1)
        process_row.addWidget(self.disconnect_hardware_button, 1)
        card.content_layout.addLayout(process_row)
        return card

    def _build_flight_card(self) -> Card:
        """创建起飞（含高度设定）与降落的高风险动作区。"""
        card = Card(
            "飞行动作",
            "按钮只发送高层请求；GUIDED、武装、起降与安全状态\n"
            "均由机载服务裁决。",
        )
        takeoff_row = QHBoxLayout()
        takeoff_row.setSpacing(5)
        self.takeoff_button = self._button("起飞", "success", "takeoffButton")
        self.takeoff_button.clicked.connect(self.takeoff_requested)
        altitude_label = QLabel("高度")
        altitude_label.setObjectName("mutedLabel")
        self.takeoff_altitude_input = self._spin_box(0.1, 50.0, 1, "m")
        self.takeoff_altitude_input.setObjectName("takeoffAltitudeInput")
        self.takeoff_altitude_input.setValue(TAKEOFF_ALTITUDE)
        self.takeoff_altitude_input.setSingleStep(0.1)
        self.takeoff_altitude_input.setToolTip("起飞目标高度（相对起飞点，单位：m）")
        self.takeoff_altitude_input.setMinimumWidth(68)
        self.takeoff_altitude_input.setMaximumWidth(76)
        speed_label = QLabel("速度")
        speed_label.setObjectName("mutedLabel")
        self.takeoff_speed_input = self._spin_box(0.1, 10.0, 1, "m/s")
        self.takeoff_speed_input.setObjectName("takeoffSpeedInput")
        self.takeoff_speed_input.setValue(TAKEOFF_SPEED)
        self.takeoff_speed_input.setSingleStep(0.1)
        self.takeoff_speed_input.setToolTip(
            "起飞速度预设，单位：m/s（本次仅完善界面，暂不下传飞控）"
        )
        self.takeoff_speed_input.setMinimumWidth(80)
        self.takeoff_speed_input.setMaximumWidth(84)
        takeoff_row.addWidget(self.takeoff_button, 1)
        takeoff_row.addWidget(altitude_label)
        takeoff_row.addWidget(self.takeoff_altitude_input)
        takeoff_row.addWidget(speed_label)
        takeoff_row.addWidget(self.takeoff_speed_input)
        card.content_layout.addLayout(takeoff_row)

        land_row = QHBoxLayout()
        land_row.setSpacing(6)
        self.land_button = self._button("降落", "danger", "landButton")
        self.land_button.clicked.connect(self.land_requested)
        land_speed_label = QLabel("速度")
        land_speed_label.setObjectName("mutedLabel")
        self.land_speed_input = self._spin_box(0.3, 2.0, 1, "m/s")
        self.land_speed_input.setObjectName("landSpeedInput")
        self.land_speed_input.setValue(LAND_SPEED)
        self.land_speed_input.setSingleStep(0.1)
        self.land_speed_input.setToolTip(
            "降落速度预设，单位：m/s（本次仅完善界面，暂不下传飞控）"
        )
        self.land_speed_input.setMinimumWidth(80)
        self.land_speed_input.setMaximumWidth(84)
        land_row.addWidget(self.land_button, 1)
        land_row.addWidget(land_speed_label)
        land_row.addWidget(self.land_speed_input)
        card.content_layout.addLayout(land_row)
        return card

    def _build_manual_card(self) -> Card:
        """创建带双摇杆反馈、状态摘要和折叠工程信息的手动区。"""
        card = Card(
            "手动操纵",
            "美国手双摇杆布局；可选择机体坐标或本地 ENU，左右摇杆\n"
            "分别调整单次输入灵敏度，机载 100 Hz C++ PD+DOB 执行。",
        )
        card.content_layout.setSpacing(7)
        self.motion_buttons: dict[str, QPushButton] = {}
        self._motion_vectors: dict[
            str, tuple[float, float, float, float]
        ] = {}
        self._motion_indicators: dict[str, _JoystickOffsetIndicator] = {}
        self._motion_offsets: dict[str, tuple[float, float]] = {}
        self._active_indicator_action: dict[
            _JoystickOffsetIndicator, str
        ] = {}

        # 坐标系、控制权与最近指令年龄始终显示在操纵区顶部。
        status_row = QHBoxLayout()
        status_row.setSpacing(7)
        coordinate_label = QLabel("坐标系")
        coordinate_label.setObjectName("mutedLabel")
        self.coordinate_mode_combo = DownwardComboBox()
        self.coordinate_mode_combo.setObjectName("manualCoordinateMode")
        self.coordinate_mode_combo.addItem("机体坐标", "body")
        self.coordinate_mode_combo.addItem("本地 ENU", "enu")
        self.coordinate_mode_combo.setToolTip(
            "机体坐标：I/J 始终沿机头前方/左侧；本地 ENU：按固定 X/Y 轴输入"
        )
        self.coordinate_mode_combo.setProperty(
            "baseToolTip", self.coordinate_mode_combo.toolTip()
        )
        self.coordinate_mode_combo.setAccessibleName("手动操纵坐标系")
        self.coordinate_mode_combo.currentIndexChanged.connect(
            self._emit_coordinate_mode_changed
        )
        self.control_authority_chip = QLabel("控制权 · 未取得")
        self.control_authority_chip.setObjectName("manualStatusChip")
        self.control_authority_chip.setProperty("tone", "bad")
        self.last_manual_command_chip = QLabel("最近指令 · 尚未发送")
        self.last_manual_command_chip.setObjectName("manualStatusChip")
        self.last_manual_command_chip.setProperty("tone", "neutral")
        status_row.addWidget(coordinate_label)
        status_row.addWidget(self.coordinate_mode_combo)
        status_row.addStretch(1)
        status_row.addWidget(self.control_authority_chip)
        status_row.addWidget(self.last_manual_command_chip)
        card.content_layout.addLayout(status_row)

        # 左摇杆：W/S 控制升降，A/D 控制偏航。
        left_definitions = (
            ("up", "上升  W", 0, 1, (0.0, 0.0, VELOCITY_SCALE, 0.0)),
            (
                "yaw_left",
                "左转  A",
                1,
                0,
                (0.0, 0.0, 0.0, VELOCITY_SCALE),
            ),
            (
                "yaw_right",
                "右转  D",
                1,
                2,
                (0.0, 0.0, 0.0, -VELOCITY_SCALE),
            ),
            ("down", "下降  S", 2, 1, (0.0, 0.0, -VELOCITY_SCALE, 0.0)),
        )
        self.left_sensitivity_combo = self._sensitivity_combo("左摇杆灵敏度")
        self.left_stick_group = self._build_motion_group(
            "left", left_definitions, self.left_sensitivity_combo
        )

        # 右摇杆：I/K 控制前后，J/L 控制左右平移。
        right_definitions = (
            ("forward", "前进  I", 0, 1, (VELOCITY_SCALE, 0.0, 0.0, 0.0)),
            ("left", "左移  J", 1, 0, (0.0, VELOCITY_SCALE, 0.0, 0.0)),
            (
                "right",
                "右移  L",
                1,
                2,
                (0.0, -VELOCITY_SCALE, 0.0, 0.0),
            ),
            ("back", "后退  K", 2, 1, (-VELOCITY_SCALE, 0.0, 0.0, 0.0)),
        )
        self.right_sensitivity_combo = self._sensitivity_combo("右摇杆灵敏度")
        self.right_stick_group = self._build_motion_group(
            "right", right_definitions, self.right_sensitivity_combo
        )

        # 外侧弹性留白与底盘按 1:4 分配剩余宽度，中缝保持紧凑；窄屏不贴边，
        # 宽屏优先放大底盘，达到最大宽度后再自然增加两侧留白。
        stick_row = QHBoxLayout()
        stick_row.setSpacing(0)
        stick_row.addStretch(1)
        stick_row.addWidget(
            self.left_stick_group,
            4,
            Qt.AlignmentFlag.AlignTop,
        )
        stick_row.addSpacing(24)
        stick_row.addWidget(
            self.right_stick_group,
            4,
            Qt.AlignmentFlag.AlignTop,
        )
        stick_row.addStretch(1)
        card.content_layout.addLayout(stick_row)

        self.hover_button = self._button(
            "制动并悬停  SPACE", "success", "hoverButton"
        )
        self.hover_button.setToolTip(_HOVER_TOOLTIP)
        self.hover_button.setProperty("baseToolTip", _HOVER_TOOLTIP)
        self.hover_button.setAccessibleDescription(_HOVER_TOOLTIP)
        self.hover_button.clicked.connect(self.hover_requested)
        self.hover_button.setMinimumWidth(260)
        hover_row = QHBoxLayout()
        hover_row.addStretch(1)
        hover_row.addWidget(self.hover_button)
        hover_row.addStretch(1)
        card.content_layout.addLayout(hover_row)

        # 主视图只保留适合快速扫视的大数字；原始工程量移入折叠区。
        self.manual_summary_panel = QWidget()
        self.manual_summary_panel.setObjectName("manualSummaryPanel")
        summary = QHBoxLayout(self.manual_summary_panel)
        summary.setContentsMargins(0, 0, 0, 0)
        summary.setSpacing(7)
        summary_definitions = (
            ("实际高度", "m", "altitude_summary_value"),
            ("实际航向", "°", "heading_summary_value"),
            ("目标水平", "m/s", "horizontal_summary_value"),
            ("目标升降", "m/s", "vertical_summary_value"),
            ("目标偏航", "°/s", "yaw_rate_summary_value"),
        )
        for title, unit, attribute in summary_definitions:
            metric, value = self._summary_metric(title, unit)
            setattr(self, attribute, value)
            summary.addWidget(metric, 1)
        card.content_layout.addWidget(self.manual_summary_panel)

        self.engineering_toggle = QToolButton()
        self.engineering_toggle.setObjectName("engineeringTelemetryToggle")
        self.engineering_toggle.setText("详细状态")
        self.engineering_toggle.setCheckable(True)
        self.engineering_toggle.setChecked(False)
        self.engineering_toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.engineering_toggle.setToolButtonStyle(
            Qt.ToolButtonStyle.ToolButtonTextBesideIcon
        )
        self.engineering_toggle.setToolTip(
            "展开 XYZ、目标位姿、控制周期和当前单次指令增量"
        )
        self.engineering_toggle.toggled.connect(self._set_engineering_visible)
        card.content_layout.addWidget(
            self.engineering_toggle,
            0,
            Qt.AlignmentFlag.AlignLeft,
        )

        self.engineering_panel = QWidget()
        self.engineering_panel.setObjectName("manualEngineeringPanel")
        self.engineering_panel.setMinimumWidth(580)
        self.engineering_panel.setMaximumWidth(900)
        self.engineering_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Preferred,
        )
        self.telemetry_panel = self.engineering_panel
        details = QHBoxLayout(self.engineering_panel)
        details.setContentsMargins(0, 0, 0, 0)
        details.setSpacing(12)
        self.engineering_left_panel = QWidget()
        telemetry = QGridLayout(self.engineering_left_panel)
        self.engineering_left_layout = telemetry
        telemetry.setContentsMargins(0, 0, 0, 0)
        telemetry.setHorizontalSpacing(10)
        telemetry.setVerticalSpacing(5)
        self.position_value = self._metric("X --  Y --  Z --  Yaw --")
        self.velocity_value = self._metric("Vx --  Vy --  Vz --  YawRate --")
        self.target_value = self._metric("X --  Y --  Z --  Yaw --")
        self.position_value.setToolTip("ψ 表示实际偏航角")
        self.velocity_value.setToolTip("ωz 表示目标偏航角速度，单位 rad/s")
        self.target_value.setToolTip("ψ 表示目标偏航角")
        self.controller_value = self._metric("-- Hz · jitter -- ms · miss --")
        self.safety_value = self._metric("位置 WAIT · 推力 WAIT · 发布源 WAIT")
        rows = (
            ("实际位姿", self.position_value),
            ("目标速度", self.velocity_value),
            ("目标位姿", self.target_value),
            ("控制周期", self.controller_value),
            ("安全门控", self.safety_value),
        )
        for row, (label, value) in enumerate(rows):
            label_widget = QLabel(label)
            label_widget.setObjectName("mutedLabel")
            telemetry.addWidget(label_widget, row, 0)
            telemetry.addWidget(value, row, 1)

        telemetry.setColumnStretch(1, 1)
        details.addWidget(self.engineering_left_panel, 2)

        # 右栏只陈述当前按一下按键所发送的增量；机载端仍会累加并按安全上限裁剪。
        divider = QFrame()
        divider.setObjectName("manualDetailsDivider")
        divider.setFrameShape(QFrame.Shape.VLine)
        divider.setFrameShadow(QFrame.Shadow.Sunken)
        details.addWidget(divider)
        self.engineering_increment_panel = QWidget()
        increment_layout = QGridLayout(self.engineering_increment_panel)
        self.engineering_increment_layout = increment_layout
        increment_layout.setContentsMargins(0, 0, 0, 0)
        increment_layout.setHorizontalSpacing(8)
        increment_layout.setVerticalSpacing(5)
        self.manual_increment_heading = QLabel("单次指令增量")
        self.manual_increment_heading.setObjectName("mutedLabel")
        self.manual_increment_heading.setToolTip(
            "显示当前灵敏度下每按一次所发送的期望量增量；不是实测运动量"
        )
        increment_layout.addWidget(self.manual_increment_heading, 0, 0, 1, 2)
        self.vertical_increment_value = self._metric("±-- m/s")
        self.yaw_increment_value = self._metric("±-- °/s")
        self.longitudinal_increment_value = self._metric("±-- m/s")
        self.lateral_increment_value = self._metric("±-- m/s")
        increment_rows = (
            ("升降 W/S", self.vertical_increment_value),
            ("偏航 A/D", self.yaw_increment_value),
            ("前后 I/K", self.longitudinal_increment_value),
            ("横移 J/L", self.lateral_increment_value),
        )
        for row, (label, value) in enumerate(increment_rows, start=1):
            label_widget = QLabel(label)
            label_widget.setObjectName("mutedLabel")
            increment_layout.addWidget(label_widget, row, 0)
            increment_layout.addWidget(value, row, 1)
        increment_layout.setColumnStretch(1, 1)
        details.addWidget(self.engineering_increment_panel, 1)
        self.left_sensitivity_combo.currentIndexChanged.connect(
            self._refresh_manual_increment_details
        )
        self.right_sensitivity_combo.currentIndexChanged.connect(
            self._refresh_manual_increment_details
        )
        self._refresh_manual_increment_details()
        card.content_layout.addWidget(self.engineering_panel)
        self.engineering_panel.setVisible(False)
        return card

    def _build_motion_group(
        self,
        side: str,
        definitions: tuple[
            tuple[str, str, int, int, tuple[float, float, float, float]], ...
        ],
        sensitivity: QComboBox,
    ) -> QFrame:
        """创建带轮廓、回中指示和中央灵敏度控件的十字摇杆。"""
        group = QFrame()
        group.setObjectName(f"{side}JoystickDeck")
        group.setProperty("joystickDeck", True)
        group.setMinimumWidth(278)
        group.setMaximumWidth(380)
        group.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        grid = QGridLayout(group)
        grid.setContentsMargins(12, 11, 12, 11)
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(7)
        indicator = _JoystickOffsetIndicator()
        for name, text, row, column, values in definitions:
            grid.addWidget(self._motion_button(name, text, values), row, column)
            self._motion_indicators[name] = indicator
            self._motion_offsets[name] = (float(column - 1), float(row - 1))

        center = QWidget()
        center.setObjectName("joystickCenterControls")
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(3)
        center_layout.addWidget(
            indicator, 0, Qt.AlignmentFlag.AlignHCenter
        )
        center_layout.addWidget(
            sensitivity, 0, Qt.AlignmentFlag.AlignHCenter
        )
        grid.addWidget(center, 1, 1, Qt.AlignmentFlag.AlignCenter)
        for column in range(3):
            grid.setColumnStretch(column, 1)
        return group

    @staticmethod
    def _sensitivity_combo(accessible_name: str) -> DownwardComboBox:
        """创建低、中、高三档的单次摇杆增量倍率选择器。"""
        combo = DownwardComboBox()
        combo.setProperty("sensitivityControl", True)
        combo.setAccessibleName(accessible_name)
        combo.setToolTip(f"{accessible_name}：低 0.5× / 中 1.0× / 高 2.0×")
        combo.setProperty("baseToolTip", combo.toolTip())
        combo.addItem("低 0.5×", 0.5)
        combo.addItem("中 1.0×", 1.0)
        combo.addItem("高 2.0×", 2.0)
        combo.setCurrentIndex(1)
        combo.setFixedWidth(86)
        return combo

    @staticmethod
    def _summary_metric(title: str, unit: str) -> tuple[QFrame, QLabel]:
        """创建一个标题、醒目数值和单位组成的摘要卡片。"""
        metric = QFrame()
        metric.setObjectName("manualSummaryMetric")
        metric.setMinimumWidth(96)
        layout = QVBoxLayout(metric)
        layout.setContentsMargins(9, 6, 9, 7)
        layout.setSpacing(1)
        title_label = QLabel(title)
        title_label.setObjectName("manualSummaryTitle")
        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        value = QLabel("--")
        value.setObjectName("manualSummaryValue")
        unit_label = QLabel(unit)
        unit_label.setObjectName("manualSummaryUnit")
        value_row.addWidget(value)
        value_row.addWidget(unit_label)
        value_row.addStretch(1)
        layout.addWidget(title_label)
        layout.addLayout(value_row)
        return metric, value

    def _set_engineering_visible(self, visible: bool) -> None:
        """展开或收起原始位姿、目标、控制周期与指令增量。"""
        self.engineering_panel.setVisible(visible)
        self.engineering_toggle.setArrowType(
            Qt.ArrowType.DownArrow if visible else Qt.ArrowType.RightArrow
        )

    def _refresh_manual_increment_details(self, _index: int = -1) -> None:
        """按两个摇杆的当前倍率刷新用户可读的单次期望量增量。"""
        left_increment = VELOCITY_SCALE * float(
            self.left_sensitivity_combo.currentData()
        )
        right_increment = VELOCITY_SCALE * float(
            self.right_sensitivity_combo.currentData()
        )
        yaw_degrees = math.degrees(left_increment)
        set_text_if_changed(
            self.vertical_increment_value,
            f"±{left_increment:.2f} m/s",
        )
        set_text_if_changed(
            self.yaw_increment_value,
            f"±{yaw_degrees:.1f} °/s",
        )
        set_text_if_changed(
            self.longitudinal_increment_value,
            f"±{right_increment:.2f} m/s",
        )
        set_text_if_changed(
            self.lateral_increment_value,
            f"±{right_increment:.2f} m/s",
        )

    @staticmethod
    def _button(text: str, role: str, object_name: str) -> QPushButton:
        """创建带语义角色和自动宽度策略的按钮。"""
        button = QPushButton(text)
        button.setObjectName(object_name)
        button.setProperty("role", role)
        button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        return button

    @staticmethod
    def _spin_box(
        minimum: float, maximum: float, decimals: int, suffix: str
    ) -> NoWheelDoubleSpinBox:
        """创建带明确范围的坐标输入，避免自由文本解析歧义。"""
        control = NoWheelDoubleSpinBox()
        control.setRange(minimum, maximum)
        control.setDecimals(decimals)
        control.setSingleStep(10 ** (-min(decimals, 3)))
        control.setSuffix(suffix)
        control.setProperty("compactValueInput", True)
        control.setKeyboardTracking(False)
        return control

    @staticmethod
    def _metric(text: str) -> QLabel:
        """创建可复制的等宽遥测值标签。"""
        label = QLabel(text)
        label.setObjectName("metricValue")
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        label.setWordWrap(True)
        return label

    def _motion_button(
        self, name: str, text: str, values: tuple[float, float, float, float]
    ) -> QPushButton:
        """创建运动按钮，统一绑定鼠标反馈和命令生成入口。"""
        button = self._button(text, "neutral", f"motion_{name}")
        button.setProperty("compact", True)
        button.setProperty("manualActive", False)
        tooltip = _MANUAL_MOTION_TOOLTIPS[name]
        button.setToolTip(tooltip)
        button.setProperty("baseToolTip", tooltip)
        button.setAccessibleDescription(tooltip)
        button.pressed.connect(
            lambda action=name: self._set_motion_feedback(action, True)
        )
        button.released.connect(
            lambda action=name: self._set_motion_feedback(action, False)
        )
        button.clicked.connect(
            lambda _checked=False, action=name: self.trigger_motion(
                action, pulse=False
            )
        )
        self.motion_buttons[name] = button
        self._motion_vectors[name] = values
        return button

    def coordinate_mode(self) -> str:
        """返回当前手动 XY 输入坐标系标识。"""
        return str(self.coordinate_mode_combo.currentData())

    def _emit_coordinate_mode_changed(self, _index: int) -> None:
        """把坐标系实际切换作为语义信号交给主窗口记录。"""
        self.coordinate_mode_changed.emit(
            self.coordinate_mode(), self.coordinate_mode_combo.currentText()
        )

    def _sensitivity_for(self, name: str) -> float:
        """返回目标动作所属摇杆的当前倍率。"""
        combo = (
            self.left_sensitivity_combo
            if name in {"up", "down", "yaw_left", "yaw_right"}
            else self.right_sensitivity_combo
        )
        return float(combo.currentData())

    def _command_for(self, name: str) -> tuple[float, float, float, float]:
        """应用独立灵敏度，并按需把机体 XY 增量旋转到本地 ENU。"""
        factor = self._sensitivity_for(name)
        vx, vy, vz, yaw_rate = (
            value * factor for value in self._motion_vectors[name]
        )
        if self.coordinate_mode() == "body" and (vx != 0.0 or vy != 0.0):
            cosine = math.cos(self._latest_yaw)
            sine = math.sin(self._latest_yaw)
            vx, vy = cosine * vx - sine * vy, sine * vx + cosine * vy
        values = (vx, vy, vz, yaw_rate)
        return tuple(0.0 if abs(value) < 1e-12 else value for value in values)

    def trigger_motion(self, name: str, *, pulse: bool = True) -> None:
        """让鼠标和键盘通过同一路径生成摇杆增量并更新反馈。"""
        button = self.motion_buttons.get(name)
        if button is None or not button.isEnabled():
            return
        if pulse:
            self._set_motion_feedback(name, True)
            QTimer.singleShot(
                160,
                lambda action=name: self._set_motion_feedback(action, False),
            )
        self.motion_requested.emit(*self._command_for(name))

    def _set_motion_feedback(self, name: str, active: bool) -> None:
        """同步按钮高亮与所属底盘的二维实时偏移。"""
        button = self.motion_buttons.get(name)
        indicator = self._motion_indicators.get(name)
        if button is None or indicator is None:
            return
        if button.property("manualActive") != active:
            button.setProperty("manualActive", active)
            repolish(button)
        if active:
            self._active_indicator_action[indicator] = name
            indicator.set_offset(*self._motion_offsets[name])
        elif self._active_indicator_action.get(indicator) == name:
            self._active_indicator_action.pop(indicator, None)
            indicator.set_offset(0.0, 0.0)

    def mark_manual_command(self) -> None:
        """记录最近一次实际发送的运动或制动悬停指令。"""
        self._last_manual_command_at = time.monotonic()
        self._refresh_manual_command_age()

    def _refresh_manual_command_age(self) -> None:
        """刷新最近手动指令距当前时刻的可扫读年龄。"""
        if self._last_manual_command_at is None:
            text = "最近指令 · 尚未发送"
        else:
            elapsed = max(0.0, time.monotonic() - self._last_manual_command_at)
            text = f"最近指令 · {elapsed:.1f} s"
        set_text_if_changed(self.last_manual_command_chip, text)

    def _size_auxiliary_buttons(self) -> None:
        """把齿轮与 Wi-Fi 按钮固定为一致的紧凑正方形。"""
        for button in (
            self.origin_settings_button,
            self.communication_test_button,
        ):
            button.setFixedSize(
                self._AUXILIARY_BUTTON_SIZE,
                self._AUXILIARY_BUTTON_SIZE,
            )

    def origin(self) -> tuple[float, float, float]:
        """返回仅供完整实机连接工作流写入飞控的本地缓存原点。"""
        return self._origin

    def takeoff_altitude(self) -> float:
        """返回当前起飞设定高度（米）。"""
        return float(self.takeoff_altitude_input.value())

    def takeoff_speed(self) -> float:
        """返回当前起飞最大爬升速度（米/秒）。"""
        return float(self.takeoff_speed_input.value())

    def land_speed(self) -> float:
        """返回当前 LAND 目标下降速度（米/秒）。"""
        return float(self.land_speed_input.value())

    def _refresh_origin_summary(self) -> None:
        """在环境卡片展示当前缓存原点摘要。"""
        lat, lon, alt = self._origin
        set_text_if_changed(
            self.origin_summary,
            f"实机连接原点 · Lat {lat:.7f}  Lon {lon:.7f}  Alt {alt:.1f} m"
        )

    def _open_origin_settings(self) -> None:
        """打开原点配置对话框；确认后只更新本地缓存。"""
        dialog = OriginConfigDialog(self._origin, parent=self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._origin = dialog.origin()
            self._refresh_origin_summary()

    def update_snapshot(self, snapshot: VehicleSnapshot) -> None:
        """刷新摘要、常驻状态与默认折叠的机载工程诊断。"""
        self._latest_yaw = snapshot.yaw
        set_text_if_changed(self.altitude_summary_value, f"{snapshot.z:+.2f}")
        set_text_if_changed(
            self.heading_summary_value,
            f"{math.degrees(snapshot.yaw):+.1f}",
        )
        set_text_if_changed(
            self.horizontal_summary_value,
            f"{math.hypot(snapshot.target_vx, snapshot.target_vy):.2f}",
        )
        set_text_if_changed(
            self.vertical_summary_value, f"{snapshot.target_vz:+.2f}"
        )
        set_text_if_changed(
            self.yaw_rate_summary_value,
            f"{math.degrees(snapshot.target_yaw_rate):+.1f}",
        )

        authority = bool(snapshot.control_authority)
        authority_text = "控制权 · 已取得" if authority else "控制权 · 未取得"
        authority_tone = "good" if authority else "bad"
        set_text_if_changed(self.control_authority_chip, authority_text)
        if self.control_authority_chip.property("tone") != authority_tone:
            self.control_authority_chip.setProperty("tone", authority_tone)
            repolish(self.control_authority_chip)
        self._refresh_manual_command_age()

        set_text_if_changed(
            self.position_value,
            f"X {snapshot.x:+.2f}  Y {snapshot.y:+.2f}  Z {snapshot.z:+.2f}  "
            f"ψ {math.degrees(snapshot.yaw):+.1f}°",
        )
        set_text_if_changed(
            self.velocity_value,
            f"Vx {snapshot.target_vx:+.2f}  Vy {snapshot.target_vy:+.2f}  "
            f"Vz {snapshot.target_vz:+.2f}  ωz {snapshot.target_yaw_rate:+.2f}",
        )
        set_text_if_changed(
            self.target_value,
            f"X {snapshot.target_x:+.2f}  Y {snapshot.target_y:+.2f}  "
            f"Z {snapshot.target_z:+.2f}  "
            f"ψ {math.degrees(snapshot.target_yaw):+.1f}°",
        )
        set_text_if_changed(
            self.controller_value,
            f"{snapshot.control_rate_hz:.2f} Hz · "
            f"jitter {snapshot.max_jitter_ms:.2f} ms "
            f"· miss {snapshot.deadline_miss_count}",
        )
        set_text_if_changed(
            self.safety_value,
            f"位置 {'OK' if snapshot.local_position_valid else 'WAIT'} · "
            f"推力 {'OK' if snapshot.thrust_mode_verified else 'WAIT'} · "
            f"发布源 {'CONFLICT' if snapshot.setpoint_conflict else 'OK'}",
        )

    def apply_availability(
        self,
        state: UiAvailability,
        closing: bool = False,
        communication_running: bool = False,
        communication_cancel_pending: bool = False,
    ) -> None:
        """统一应用状态机，并让检测中的 Wi-Fi 按钮保留显式终止入口。"""
        self.simulation_button.setEnabled(state.start_environment)
        self.hardware_button.setEnabled(state.start_environment)
        self.communication_test_button.setEnabled(
            (state.communication_test or communication_running)
            and not communication_cancel_pending
            and not closing
        )
        self.origin_settings_button.setEnabled(state.origin_settings)
        self.stop_simulation_button.setEnabled(state.stop_simulation)
        self.disconnect_hardware_button.setEnabled(state.disconnect_hardware)
        self.takeoff_button.setEnabled(state.takeoff)
        # 参数输入与对应动作使用同一门控，禁用时同步显示灰色锁定态。
        for control in (
            self.takeoff_altitude_input,
            self.takeoff_speed_input,
        ):
            control.setEnabled(state.takeoff and not closing)
        self.land_speed_input.setEnabled(state.land and not closing)
        self.land_button.setEnabled(state.land)
        self.hover_button.setEnabled(state.hover)
        for button in self.motion_buttons.values():
            button.setEnabled(state.motion)
        # 坐标系和灵敏度会改变下一条手动指令，与运动按钮共用同一门控。
        for control in (
            self.coordinate_mode_combo,
            self.left_sensitivity_combo,
            self.right_sensitivity_combo,
        ):
            control.setEnabled(state.motion)
            control.setToolTip(
                str(control.property("baseToolTip") or "")
                if state.motion
                else state.flight_reason
            )
        if not state.motion:
            for name in self.motion_buttons:
                self._set_motion_feedback(name, False)

        if state.start_environment:
            self.simulation_button.setToolTip(
                "启动本机 SITL、MAVROS、机载节点与 RViz；使用 SITL 自身 Home"
            )
            self.hardware_button.setToolTip(
                "连接远端机载服务：申请控制租约、配置消息频率并写入飞控原点"
            )
        else:
            env_tip = (
                "当前已有仿真或机载会话，请先关闭仿真/断开实机后再切换"
                if not closing
                else "地面站正在安全退出"
            )
            self.simulation_button.setToolTip(env_tip)
            self.hardware_button.setToolTip(env_tip)

        if communication_running:
            self.communication_test_button.setIcon(self._communication_stop_icon)
            self.communication_test_button.setAccessibleName("终止实机通讯链路检测")
            self.communication_test_button.setAccessibleDescription(
                "立即终止当前纯订阅通讯检测"
            )
            self.communication_test_button.setToolTip(
                "正在检测实机通讯；点击红色方块终止检测"
                if not communication_cancel_pending
                else "正在终止实机通讯检测…"
            )
        elif state.communication_test:
            self.communication_test_button.setIcon(self._wifi_icon)
            self.communication_test_button.setAccessibleName("检测实机通讯链路")
            self.communication_test_button.setAccessibleDescription(
                "只接收状态与日志，不申请控制租约或发送命令"
            )
            self.communication_test_button.setToolTip(
                "检测实机通讯链路：只接收状态与日志，不申请租约、不发送命令、"
                "不启动或停止任何机载服务"
            )
        elif closing:
            self.communication_test_button.setIcon(self._wifi_icon)
            self.communication_test_button.setToolTip("地面站正在安全退出")
        else:
            self.communication_test_button.setIcon(self._wifi_icon)
            self.communication_test_button.setToolTip(
                "已有环境会话或工作流正在执行，请先完成或断开后再检测通讯"
            )

        if state.origin_settings:
            self.origin_settings_button.setToolTip(
                "配置飞控/EKF 原点（仅本地保存；完整连接实机时写入）"
            )
        elif closing:
            self.origin_settings_button.setToolTip("地面站正在安全退出")
        else:
            self.origin_settings_button.setToolTip(
                "仿真运行中或已连接实机时不可修改原点；请先关闭仿真/断开实机"
            )

        if state.stop_simulation:
            self.stop_simulation_button.setToolTip(
                "释放控制权并结束本项目启动的本地 SITL/MAVROS/机载/RViz 进程"
            )
        else:
            self.stop_simulation_button.setToolTip(
                "仅在本地仿真会话（或正在启动仿真）时可用"
            )
        if state.disconnect_hardware:
            self.disconnect_hardware_button.setToolTip(
                "释放实机控制租约；不会远程终止机载服务进程"
            )
        else:
            self.disconnect_hardware_button.setToolTip(
                "仅在实机连接会话（或正在连接实机）时可用"
            )

        for button in (self.takeoff_button, self.land_button):
            button.setToolTip(state.flight_reason)
        for button in (self.hover_button, *self.motion_buttons.values()):
            button.setToolTip(
                str(button.property("baseToolTip") or "")
                if button.isEnabled()
                else state.flight_reason
            )
