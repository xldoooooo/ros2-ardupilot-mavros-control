"""环境、GPS、起降、手动意图和遥测操作面板。"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..config import DEFAULT_GPS_ORIGIN, TAKEOFF_ALTITUDE, VELOCITY_SCALE
from ..models import VehicleSnapshot
from .state import UiAvailability
from .widgets import Card, NoWheelDoubleSpinBox


class OperationsPanel(QWidget):
    """提供独立的连接栏和手动栏，并发出不含后端依赖的 UI 意图。"""

    simulation_requested = Signal()
    hardware_requested = Signal()
    cleanup_requested = Signal()
    exit_requested = Signal()
    origin_requested = Signal()
    takeoff_requested = Signal()
    land_requested = Signal()
    hover_requested = Signal()
    motion_requested = Signal(float, float, float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        """构建可被主窗口分别放入左栏和中栏的两个操作面板。"""
        super().__init__(parent)
        self.setMinimumWidth(330)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 4, 0)
        root.setSpacing(10)
        root.addWidget(self._build_environment_card())
        root.addWidget(self._build_flight_card())
        root.addStretch(1)

        self.manual_panel = QWidget()
        self.manual_panel.setObjectName("manualOperationsPanel")
        self.manual_panel.setMinimumWidth(350)
        manual_layout = QVBoxLayout(self.manual_panel)
        manual_layout.setContentsMargins(4, 0, 4, 0)
        manual_layout.setSpacing(10)
        manual_layout.addWidget(self._build_manual_card())
        manual_layout.addStretch(1)

    def _build_environment_card(self) -> Card:
        """创建仿真/实机入口、GPS 原点和断开操作。"""
        card = Card(
            "环境与连接",
            "仿真仅管理本机进程；实机连接不会远程启动或终止机载服务。",
        )
        action_row = QHBoxLayout()
        self.simulation_button = self._button(
            "启动本地仿真", "primary", "simulationButton"
        )
        self.hardware_button = self._button(
            "连接实机服务", "primary", "hardwareButton"
        )
        self.simulation_button.clicked.connect(self.simulation_requested)
        self.hardware_button.clicked.connect(self.hardware_requested)
        action_row.addWidget(self.simulation_button)
        action_row.addWidget(self.hardware_button)
        card.content_layout.addLayout(action_row)

        origin_title = QLabel("GPS 原点（WGS84）")
        origin_title.setObjectName("mutedLabel")
        card.content_layout.addWidget(origin_title)
        origin_grid = QGridLayout()
        origin_grid.setHorizontalSpacing(8)
        origin_grid.setVerticalSpacing(6)
        self.latitude_input = self._spin_box(-90.0, 90.0, 7, " °")
        self.longitude_input = self._spin_box(-180.0, 180.0, 7, " °")
        self.altitude_input = self._spin_box(-1000.0, 10000.0, 1, " m")
        self.latitude_input.setValue(DEFAULT_GPS_ORIGIN[0])
        self.longitude_input.setValue(DEFAULT_GPS_ORIGIN[1])
        self.altitude_input.setValue(DEFAULT_GPS_ORIGIN[2])
        for column, (label, control) in enumerate(
            (
                ("纬度", self.latitude_input),
                ("经度", self.longitude_input),
                ("海拔", self.altitude_input),
            )
        ):
            origin_grid.addWidget(QLabel(label), 0, column)
            origin_grid.addWidget(control, 1, column)
            origin_grid.setColumnStretch(column, 1)
        self.origin_button = self._button("写入飞控原点", "primary", "originButton")
        self.origin_button.clicked.connect(self.origin_requested)
        origin_grid.addWidget(self.origin_button, 2, 0, 1, 3)
        card.content_layout.addLayout(origin_grid)

        process_row = QHBoxLayout()
        self.cleanup_button = self._button(
            "断开 / 关闭本地仿真", "danger", "cleanupButton"
        )
        self.exit_button = self._button("退出地面站", "neutral", "exitButton")
        self.cleanup_button.clicked.connect(self.cleanup_requested)
        self.exit_button.clicked.connect(self.exit_requested)
        process_row.addWidget(self.cleanup_button, 1)
        process_row.addWidget(self.exit_button)
        card.content_layout.addLayout(process_row)
        return card

    def _build_flight_card(self) -> Card:
        """创建起飞和降落的高风险动作区。"""
        card = Card(
            "飞行动作",
            "按钮只发送高层请求，GUIDED、武装、起降与安全状态由机载服务裁决。",
        )
        row = QHBoxLayout()
        self.takeoff_button = self._button(
            f"起飞至 {TAKEOFF_ALTITUDE:.1f} m", "success", "takeoffButton"
        )
        self.land_button = self._button("降落", "danger", "landButton")
        self.takeoff_button.clicked.connect(self.takeoff_requested)
        self.land_button.clicked.connect(self.land_requested)
        row.addWidget(self.takeoff_button, 2)
        row.addWidget(self.land_button, 1)
        card.content_layout.addLayout(row)
        return card

    def _build_manual_card(self) -> Card:
        """创建八向增量意图、悬停和关键遥测展示。"""
        card = Card(
            "手动运动意图",
            f"每次输入增量 {VELOCITY_SCALE:.1f} m/s（偏航为 rad/s）；"
            "机载 100 Hz C++ PD+DOB 执行。",
        )
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        self.motion_buttons: dict[str, QPushButton] = {}
        definitions = (
            ("up", "上升  W", 0, 1, (0.0, 0.0, VELOCITY_SCALE, 0.0)),
            ("left", "左移  J", 1, 0, (0.0, VELOCITY_SCALE, 0.0, 0.0)),
            ("right", "右移  L", 1, 2, (0.0, -VELOCITY_SCALE, 0.0, 0.0)),
            ("down", "下降  S", 2, 1, (0.0, 0.0, -VELOCITY_SCALE, 0.0)),
        )
        for name, text, row, column, values in definitions:
            button = self._motion_button(name, text, values)
            grid.addWidget(button, row, column)
        self.hover_button = self._button("悬停  SPACE", "success", "hoverButton")
        self.hover_button.clicked.connect(self.hover_requested)
        grid.addWidget(self.hover_button, 1, 1)
        for column in range(3):
            grid.setColumnStretch(column, 1)
        card.content_layout.addLayout(grid)

        linear_row = QHBoxLayout()
        for name, text, values in (
            ("forward", "前进  I", (VELOCITY_SCALE, 0.0, 0.0, 0.0)),
            ("back", "后退  K", (-VELOCITY_SCALE, 0.0, 0.0, 0.0)),
            ("yaw_left", "左转  A", (0.0, 0.0, 0.0, VELOCITY_SCALE)),
            ("yaw_right", "右转  D", (0.0, 0.0, 0.0, -VELOCITY_SCALE)),
        ):
            linear_row.addWidget(self._motion_button(name, text, values))
        card.content_layout.addLayout(linear_row)

        hint = QLabel("W/S 上下 · I/K 前后 · J/L 左右 · A/D 偏航 · Space 悬停")
        hint.setObjectName("shortcutHint")
        hint.setWordWrap(True)
        card.content_layout.addWidget(hint)

        telemetry = QGridLayout()
        telemetry.setHorizontalSpacing(10)
        telemetry.setVerticalSpacing(5)
        self.position_value = self._metric("X --  Y --  Z --  Yaw --")
        self.velocity_value = self._metric("Vx --  Vy --  Vz --  YawRate --")
        self.target_value = self._metric("X --  Y --  Z --  Yaw --")
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
        card.content_layout.addLayout(telemetry)
        return card

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
        """创建一次性速度/偏航增量按钮并绑定其不可变参数。"""
        button = self._button(text, "neutral", f"motion_{name}")
        button.clicked.connect(
            lambda _checked=False, command=values: self.motion_requested.emit(*command)
        )
        self.motion_buttons[name] = button
        return button

    def origin(self) -> tuple[float, float, float]:
        """返回已由 QDoubleSpinBox 范围约束的 GPS 原点。"""
        return (
            self.latitude_input.value(),
            self.longitude_input.value(),
            self.altitude_input.value(),
        )

    def update_snapshot(self, snapshot: VehicleSnapshot) -> None:
        """只读展示机载聚合状态，不在 GUI 推导控制状态。"""
        self.position_value.setText(
            f"X {snapshot.x:+.2f}  Y {snapshot.y:+.2f}  Z {snapshot.z:+.2f}  "
            f"Yaw {math.degrees(snapshot.yaw):+.1f}°"
        )
        self.velocity_value.setText(
            f"Vx {snapshot.target_vx:+.2f}  Vy {snapshot.target_vy:+.2f}  "
            f"Vz {snapshot.target_vz:+.2f}  YawRate {snapshot.target_yaw_rate:+.2f}"
        )
        self.target_value.setText(
            f"X {snapshot.target_x:+.2f}  Y {snapshot.target_y:+.2f}  "
            f"Z {snapshot.target_z:+.2f}  Yaw {math.degrees(snapshot.target_yaw):+.1f}°"
        )
        self.controller_value.setText(
            f"{snapshot.control_rate_hz:.2f} Hz · "
            f"jitter {snapshot.max_jitter_ms:.2f} ms "
            f"· miss {snapshot.deadline_miss_count}"
        )
        self.safety_value.setText(
            f"位置 {'OK' if snapshot.local_position_valid else 'WAIT'} · "
            f"推力 {'OK' if snapshot.thrust_mode_verified else 'WAIT'} · "
            f"发布源 {'CONFLICT' if snapshot.setpoint_conflict else 'OK'}"
        )

    def apply_availability(self, state: UiAvailability, closing: bool = False) -> None:
        """统一应用状态机计算结果，避免按钮各自维护零散条件。"""
        self.simulation_button.setEnabled(state.start_environment)
        self.hardware_button.setEnabled(state.start_environment)
        self.cleanup_button.setEnabled(state.cleanup)
        self.exit_button.setEnabled(not closing)
        self.origin_button.setEnabled(state.set_origin)
        self.takeoff_button.setEnabled(state.takeoff)
        self.land_button.setEnabled(state.land)
        self.hover_button.setEnabled(state.hover)
        for button in self.motion_buttons.values():
            button.setEnabled(state.motion)

        for button in (
            self.origin_button,
            self.takeoff_button,
            self.land_button,
            self.hover_button,
            *self.motion_buttons.values(),
        ):
            button.setToolTip(state.flight_reason)
