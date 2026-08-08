"""环境、起降、手动意图和遥测操作面板。"""

from __future__ import annotations

import math

from PySide6.QtCore import QEvent, Qt, Signal
from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
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


class OriginConfigDialog(QDialog):
    """配置 EKF/飞控原点；仅保存本地值，不在此刻写入飞控。"""

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
            "此处仅缓存实机飞控/EKF 原点，并在连接实机服务时写入。"
            "本地 SITL 使用自身 Home，不使用该缓存原点。"
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
        control.setKeyboardTracking(False)
        return control

    def origin(self) -> tuple[float, float, float]:
        """返回对话框中当前编辑的原点。"""
        return (
            self.latitude_input.value(),
            self.longitude_input.value(),
            self.altitude_input.value(),
        )


class OperationsPanel(QWidget):
    """提供独立的连接栏和手动栏，并发出不含后端依赖的 UI 意图。"""

    simulation_requested = Signal()
    hardware_requested = Signal()
    stop_simulation_requested = Signal()
    disconnect_hardware_requested = Signal()
    exit_requested = Signal()
    takeoff_requested = Signal()
    land_requested = Signal()
    hover_requested = Signal()
    motion_requested = Signal(float, float, float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        """构建可被主窗口分别放入左栏和中栏的两个操作面板。"""
        super().__init__(parent)
        self.setMinimumWidth(330)
        self._origin = tuple(DEFAULT_GPS_ORIGIN)
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
        """创建仿真/实机入口、原点配置齿轮与断开操作。"""
        card = Card(
            "环境与连接",
            "仿真仅管理本机进程；实机连接不会远程启动或终止机载服务。"
            "缓存原点仅在连接实机时写入；本地 SITL 使用自身 Home。",
        )
        action_row = QHBoxLayout()
        self.simulation_button = self._button(
            "启动本地仿真", "primary", "simulationButton"
        )
        self.hardware_button = self._button(
            "连接实机服务", "primary", "hardwareButton"
        )
        # 正方形齿轮：边长跟随左侧主按钮实际高度。
        self.origin_settings_button = self._button(
            "⚙", "neutral", "originSettingsButton"
        )
        self.origin_settings_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.origin_settings_button.setToolTip(
            "配置实机飞控/EKF 原点（仅本地保存；连接实机时写入）"
        )
        self.simulation_button.clicked.connect(self.simulation_requested)
        self.hardware_button.clicked.connect(self.hardware_requested)
        self.origin_settings_button.clicked.connect(self._open_origin_settings)
        self.simulation_button.installEventFilter(self)
        action_row.addWidget(self.simulation_button, 1)
        action_row.addWidget(self.hardware_button, 1)
        action_row.addWidget(
            self.origin_settings_button, 0, Qt.AlignmentFlag.AlignVCenter
        )
        card.content_layout.addLayout(action_row)
        self._sync_origin_settings_square()

        self.origin_summary = QLabel()
        self.origin_summary.setObjectName("mutedLabel")
        self.origin_summary.setWordWrap(True)
        self._refresh_origin_summary()
        card.content_layout.addWidget(self.origin_summary)

        process_row = QHBoxLayout()
        self.stop_simulation_button = self._button(
            "关闭本地仿真", "danger", "stopSimulationButton"
        )
        self.disconnect_hardware_button = self._button(
            "断开实机连接", "danger", "disconnectHardwareButton"
        )
        self.exit_button = self._button("退出地面站", "neutral", "exitButton")
        self.stop_simulation_button.clicked.connect(self.stop_simulation_requested)
        self.disconnect_hardware_button.clicked.connect(
            self.disconnect_hardware_requested
        )
        self.exit_button.clicked.connect(self.exit_requested)
        process_row.addWidget(self.stop_simulation_button, 1)
        process_row.addWidget(self.disconnect_hardware_button, 1)
        process_row.addWidget(self.exit_button)
        card.content_layout.addLayout(process_row)
        return card

    def _build_flight_card(self) -> Card:
        """创建起飞（含高度设定）与降落的高风险动作区。"""
        card = Card(
            "飞行动作",
            "按钮只发送高层请求，GUIDED、武装、起降与安全状态由机载服务裁决。",
        )
        takeoff_row = QHBoxLayout()
        takeoff_row.setSpacing(8)
        self.takeoff_button = self._button("起飞", "success", "takeoffButton")
        self.takeoff_button.clicked.connect(self.takeoff_requested)
        altitude_label = QLabel("高度")
        altitude_label.setObjectName("mutedLabel")
        self.takeoff_altitude_input = self._spin_box(0.1, 50.0, 1, " m")
        self.takeoff_altitude_input.setObjectName("takeoffAltitudeInput")
        self.takeoff_altitude_input.setValue(TAKEOFF_ALTITUDE)
        self.takeoff_altitude_input.setSingleStep(0.1)
        self.takeoff_altitude_input.setToolTip("起飞目标高度（相对起飞点）")
        self.takeoff_altitude_input.setMaximumWidth(120)
        takeoff_row.addWidget(self.takeoff_button, 1)
        takeoff_row.addWidget(altitude_label)
        takeoff_row.addWidget(self.takeoff_altitude_input)
        card.content_layout.addLayout(takeoff_row)

        self.land_button = self._button("降落", "danger", "landButton")
        self.land_button.clicked.connect(self.land_requested)
        card.content_layout.addWidget(self.land_button)
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

    def eventFilter(self, watched: object, event: QEvent) -> bool:  # noqa: N802
        """左侧主按钮尺寸变化时保持齿轮为正方形。"""
        if watched is self.simulation_button and event.type() in {
            QEvent.Type.Show,
            QEvent.Type.Resize,
            QEvent.Type.LayoutRequest,
        }:
            self._sync_origin_settings_square()
        return super().eventFilter(watched, event)

    def _sync_origin_settings_square(self) -> None:
        """把齿轮边长设为与「启动本地仿真」按钮相同的高度。"""
        height = self.simulation_button.height()
        if height <= 1:
            height = max(
                self.simulation_button.sizeHint().height(),
                self.simulation_button.minimumSizeHint().height(),
                34,
            )
        if self.origin_settings_button.width() != height or (
            self.origin_settings_button.height() != height
        ):
            self.origin_settings_button.setFixedSize(height, height)

    def origin(self) -> tuple[float, float, float]:
        """返回仅供实机连接工作流写入飞控的本地缓存原点。"""
        return self._origin

    def takeoff_altitude(self) -> float:
        """返回当前起飞设定高度（米）。"""
        return float(self.takeoff_altitude_input.value())

    def _refresh_origin_summary(self) -> None:
        """在环境卡片展示当前缓存原点摘要。"""
        lat, lon, alt = self._origin
        self.origin_summary.setText(
            f"实机连接原点 · Lat {lat:.7f}  Lon {lon:.7f}  Alt {alt:.1f} m"
        )

    def _open_origin_settings(self) -> None:
        """打开原点配置对话框；确认后只更新本地缓存。"""
        dialog = OriginConfigDialog(self._origin, parent=self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._origin = dialog.origin()
            self._refresh_origin_summary()

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
        self.origin_settings_button.setEnabled(state.origin_settings)
        self.stop_simulation_button.setEnabled(state.stop_simulation)
        self.disconnect_hardware_button.setEnabled(state.disconnect_hardware)
        self.exit_button.setEnabled(not closing)
        self.takeoff_button.setEnabled(state.takeoff)
        # 高度可随时预置；起飞按钮仍受飞行门控。
        self.takeoff_altitude_input.setEnabled(not closing)
        self.land_button.setEnabled(state.land)
        self.hover_button.setEnabled(state.hover)
        for button in self.motion_buttons.values():
            button.setEnabled(state.motion)

        if state.start_environment:
            self.simulation_button.setToolTip(
                "启动本机 SITL、MAVROS、机载节点与 RViz；使用 SITL 自身 Home"
            )
            self.hardware_button.setToolTip(
                "连接局域网远端机载服务，并写入已配置飞控原点"
            )
        else:
            env_tip = (
                "当前已有仿真或机载会话，请先关闭仿真/断开实机后再切换"
                if not closing
                else "地面站正在安全退出"
            )
            self.simulation_button.setToolTip(env_tip)
            self.hardware_button.setToolTip(env_tip)

        if state.origin_settings:
            self.origin_settings_button.setToolTip(
                "配置飞控/EKF 原点（仅本地保存；连接实机时写入）"
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

        for button in (
            self.takeoff_button,
            self.land_button,
            self.hover_button,
            *self.motion_buttons.values(),
        ):
            button.setToolTip(state.flight_reason)
