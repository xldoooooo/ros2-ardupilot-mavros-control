"""本地 ENU 航点编辑、排序、进度与上传面板。"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QDragEnterEvent, QDragMoveEvent, QDropEvent, QIcon
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..config import (
    WAYPOINT_HORIZONTAL_LIMIT_METERS,
    WAYPOINT_YAW_LIMIT_DEGREES,
    WAYPOINT_Z_MAX_METERS,
    WAYPOINT_Z_MIN_METERS,
)
from ..models import (
    FlightMode,
    VehicleSnapshot,
    WaypointFlightStrategy,
    WaypointReferenceGenerator,
    WaypointTrackingController,
)
from .state import UiAvailability
from .widgets import Card, DownwardComboBox, NoWheelDoubleSpinBox


class WaypointDropTable(QTableWidget):
    """接收一个或多个本地文件路径，由主窗口统一校验并导入。"""

    files_dropped = Signal(object)

    def __init__(self, rows: int, columns: int) -> None:
        """启用只接收外部文件的拖放模式，不允许表格内部移动。"""
        super().__init__(rows, columns)
        self.setAcceptDrops(True)
        self.setDragDropMode(QAbstractItemView.DragDropMode.DropOnly)

    @staticmethod
    def _has_local_files(event: QDragEnterEvent | QDragMoveEvent) -> bool:
        """仅接受带至少一个本地文件 URL 的拖放载荷。"""
        urls = event.mimeData().urls()
        return bool(urls) and all(url.isLocalFile() for url in urls)

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:  # noqa: N802
        """允许资源管理器中的本地文件进入航点列表。"""
        if self._has_local_files(event):
            event.acceptProposedAction()
            return
        event.ignore()

    def dragMoveEvent(self, event: QDragMoveEvent) -> None:  # noqa: N802
        """拖动期间持续显示当前载荷可放置。"""
        if self._has_local_files(event):
            event.acceptProposedAction()
            return
        event.ignore()

    def dropEvent(self, event: QDropEvent) -> None:  # noqa: N802
        """把全部本地路径交给主窗口，以便明确拒绝多文件导入。"""
        paths = tuple(
            url.toLocalFile()
            for url in event.mimeData().urls()
            if url.isLocalFile()
        )
        if not paths:
            event.ignore()
            return
        self.files_dropped.emit(paths)
        event.acceptProposedAction()


class WaypointPanel(QWidget):
    """维护尚未上传的本地航点副本，并把最终列表作为单一信号发出。"""

    _ROW_HEIGHT = 28

    # 参数依次为航点、避障策略空壳、命令生成方式和跟踪控制方式。
    send_requested = Signal(object, object, object, object)
    clear_requested = Signal()
    preview_requested = Signal()
    import_file_requested = Signal()
    files_dropped = Signal(object)
    waypoints_changed = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        """构建数值输入、航点表、排序按钮和执行进度。"""
        super().__init__(parent)
        self.setMinimumWidth(420)
        self.setMinimumHeight(500)
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._editing_enabled = True
        self._preview_enabled = False
        self._progress_tracking = False
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 0, 0, 0)
        root.setSpacing(10)
        # 上方编辑卡吃掉多余高度；下方执行卡贴底，高度固定。
        root.addWidget(self._build_editor_card(), 1)
        root.addWidget(self._build_execution_card(), 0)

    def _build_editor_card(self) -> Card:
        """创建航点输入与可伸缩表格：仅中间列表随面板高度变长。"""
        card = Card(
            "航点任务",
            "坐标系为本地 ENU；上传的是列表副本，\n"
            "到达判定与推进由机载服务完成。",
        )
        card.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        # 标签与输入同排：X [框] Y [框] Z [框] Yaw [框]，压缩顶部高度留给列表。
        coords = QHBoxLayout()
        coords.setSpacing(3)
        self.x_input = self._coordinate_input(
            -WAYPOINT_HORIZONTAL_LIMIT_METERS,
            WAYPOINT_HORIZONTAL_LIMIT_METERS,
            2,
            " m",
        )
        self.y_input = self._coordinate_input(
            -WAYPOINT_HORIZONTAL_LIMIT_METERS,
            WAYPOINT_HORIZONTAL_LIMIT_METERS,
            2,
            " m",
        )
        self.z_input = self._coordinate_input(
            WAYPOINT_Z_MIN_METERS, WAYPOINT_Z_MAX_METERS, 2, " m"
        )
        self.yaw_input = self._coordinate_input(
            -WAYPOINT_YAW_LIMIT_DEGREES,
            WAYPOINT_YAW_LIMIT_DEGREES,
            1,
            " °",
        )
        self.z_input.setValue(1.0)
        for text, control in (
            ("X", self.x_input),
            ("Y", self.y_input),
            ("Z", self.z_input),
            ("Yaw", self.yaw_input),
        ):
            label = QLabel(text)
            label.setObjectName("mutedLabel")
            label.setSizePolicy(
                QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
            )
            coords.addWidget(label, 0)
            coords.addWidget(control, 1)
        self.add_button = self._button("+", "neutral", "addWaypointButton")
        self.add_button.setProperty("compact", True)
        self.add_button.setFixedSize(self._ROW_HEIGHT, self._ROW_HEIGHT)
        self.add_button.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed
        )
        self.add_button.setToolTip("将当前 X、Y、Z、Yaw 追加到航点列表")
        self.add_button.setProperty("baseToolTip", self.add_button.toolTip())
        self.add_button.clicked.connect(self._add_waypoint)
        coords.addWidget(self.add_button, 0)
        # stretch=0：坐标与添加操作固定为单行，把纵向空间留给表格。
        card.content_layout.addLayout(coords, 0)

        self.table = WaypointDropTable(0, 5)
        self.table.setObjectName("waypointTable")
        self.table.setHorizontalHeaderLabels(
            ("#", "X / m", "Y / m", "Z / m", "Yaw / °")
        )
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setMinimumHeight(95)
        self.table.setToolTip("可将一个 CSV 航点文件直接拖入此列表")
        self.table.setProperty("baseToolTip", self.table.toolTip())
        self.table.setAccessibleDescription(
            "本地 ENU 航点列表；支持拖入一个 CSV 文件并在确认后替换列表"
        )
        # 取消固定最大高度，使中间列表成为唯一纵向伸缩区。
        self.table.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        vertical_header = self.table.verticalHeader()
        vertical_header.setVisible(False)
        vertical_header.setMinimumSectionSize(self._ROW_HEIGHT)
        vertical_header.setDefaultSectionSize(self._ROW_HEIGHT)
        header = self.table.horizontalHeader()
        header.setFixedHeight(self._ROW_HEIGHT)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        for column in range(1, 5):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._update_local_controls)
        self.table.files_dropped.connect(self.files_dropped.emit)
        card.content_layout.addWidget(self.table, 1)

        controls = QHBoxLayout()
        assets = Path(__file__).resolve().parent / "assets"
        self.up_button = self._icon_button(
            QIcon(str(assets / "chevron-up.svg")),
            "上移选中航点",
            "moveWaypointUpButton",
        )
        self.down_button = self._icon_button(
            QIcon(str(assets / "chevron-down.svg")),
            "下移选中航点",
            "moveWaypointDownButton",
        )
        self.remove_button = self._icon_button(
            QIcon(str(assets / "minus-red.svg")),
            "删除选中航点",
            "removeWaypointButton",
        )
        self.clear_button = self._button("清空", "danger", "clearWaypointButton")
        self.clear_button.setToolTip("清空全部本地航点")
        self.clear_button.setProperty("baseToolTip", self.clear_button.toolTip())
        self.remove_button.clicked.connect(self._remove_selected)
        self.up_button.clicked.connect(lambda: self._move_selected(-1))
        self.down_button.clicked.connect(lambda: self._move_selected(1))
        self.clear_button.clicked.connect(self.clear_requested)
        for button in (
            self.up_button,
            self.down_button,
            self.remove_button,
            self.clear_button,
        ):
            button.setProperty("compact", True)
            controls.addWidget(button)
        controls.addStretch(1)
        self.preview_button = self._button(
            "预览", "neutral", "previewWaypointButton"
        )
        self.preview_button.setProperty("compact", True)
        self.preview_button.setToolTip(
            "在 RViz 中显示当前航点、航点间直线和无人机实时位姿"
        )
        self.preview_button.setProperty(
            "baseToolTip", self.preview_button.toolTip()
        )
        self.preview_button.setAccessibleName("在 RViz 中预览航点")
        self.preview_button.clicked.connect(self.preview_requested)
        controls.addWidget(self.preview_button)
        self.import_button = self._button(
            "从文件导入", "neutral", "importWaypointButton"
        )
        self.import_button.setProperty("compact", True)
        self.import_button.setToolTip(
            "从单个 CSV 文件导入绝对本地 ENU 航点，并替换当前列表"
        )
        self.import_button.setProperty("baseToolTip", self.import_button.toolTip())
        self.import_button.setAccessibleName("从文件导入航点")
        self.import_button.clicked.connect(self.import_file_requested)
        controls.addWidget(self.import_button)
        # stretch=0：排序/清空条贴在表格下方、随卡片但不抢高度。
        card.content_layout.addLayout(controls, 0)

        # Card 外层默认不给 content 伸缩；仅本卡让 content 吃掉多余高度。
        card_root = card.layout()
        if isinstance(card_root, QVBoxLayout):
            for index in range(card_root.count()):
                item = card_root.itemAt(index)
                if item is not None and item.layout() is card.content_layout:
                    card_root.setStretch(index, 1)
                    break

        self._update_local_controls()
        return card

    def _build_execution_card(self) -> Card:
        """创建无标题的紧凑进度、发送与三项实验配置区。"""
        card = Card("")
        card.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum
        )
        action_row = QHBoxLayout()
        action_row.setSpacing(8)
        self.progress = QProgressBar()
        self.progress.setObjectName("waypointProgress")
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("尚未执行")
        self.send_button = self._button(
            "发送并执行航点", "primary", "sendWaypointButton"
        )
        self.send_button.clicked.connect(self._emit_send_requested)
        action_row.addWidget(self.progress, 2)
        action_row.addWidget(self.send_button, 1)
        card.content_layout.addLayout(action_row)
        # 保留非可视状态接收器，使现有结果更新链与内部接口不变；
        # 不加入布局，避免灰色说明行占用执行卡高度。
        self.status_label = QLabel("请先编辑航点，再在飞行器就绪后发送。", card)
        self.status_label.setObjectName("mutedLabel")
        self.status_label.hide()

        # 三列只负责选择；飞行中由状态门控统一锁定，任务内部不做热切换。
        selection_row = QHBoxLayout()
        selection_row.setSpacing(8)
        self.strategy_combo = DownwardComboBox()
        self.strategy_combo.setObjectName("waypointStrategyCombo")
        self.strategy_combo.setToolTip(
            "航点飞行策略。当前仅实现「直线飞行」；"
            "「自动避障」与「遇到障碍悬停」为预留选项，发送后仍按直线飞行执行。"
        )
        self.strategy_combo.setProperty("baseToolTip", self.strategy_combo.toolTip())
        for strategy in WaypointFlightStrategy:
            self.strategy_combo.addItem(strategy.label, strategy)
        self.strategy_combo.setCurrentIndex(0)

        self.reference_combo = DownwardComboBox()
        self.reference_combo.setObjectName("waypointReferenceGeneratorCombo")
        self.reference_combo.setToolTip(
            "选择航点到连续位置/速度/加速度参考的生成方式；参数位于 control.yaml。"
        )
        self.reference_combo.setProperty("baseToolTip", self.reference_combo.toolTip())
        for generator in WaypointReferenceGenerator:
            self.reference_combo.addItem(generator.label, generator)
        # GUI 默认使用当前推荐的连续梯形速度参考；协议的未知值回退仍保留基线。
        self.reference_combo.setCurrentIndex(
            WaypointReferenceGenerator.TRAPEZOIDAL_PROFILE.value
        )

        self.tracking_combo = DownwardComboBox()
        self.tracking_combo.setObjectName("waypointTrackingControllerCombo")
        self.tracking_combo.setToolTip(
            "位置 PD+DOB 保留基线；轨迹 PD+DOB 使用独立低带宽增益和加速度前馈。"
        )
        self.tracking_combo.setProperty("baseToolTip", self.tracking_combo.toolTip())
        for controller in WaypointTrackingController:
            self.tracking_combo.addItem(controller.label, controller)
        self.tracking_combo.setCurrentIndex(
            WaypointTrackingController.TRAJECTORY_PD_DOB.value
        )

        for label_text, combo in (
            ("避障策略", self.strategy_combo),
            ("命令生成", self.reference_combo),
            ("跟踪控制", self.tracking_combo),
        ):
            field = QVBoxLayout()
            field.setSpacing(4)
            label = QLabel(label_text)
            label.setObjectName("waypointMethodLabel")
            label.setBuddy(combo)
            combo.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
            )
            combo.setMinimumWidth(0)
            field.addWidget(label)
            field.addWidget(combo)
            selection_row.addLayout(field, 1)
        card.content_layout.addLayout(selection_row)
        return card

    def selected_strategy(self) -> WaypointFlightStrategy:
        """返回当前下拉框选中的航点飞行策略。"""
        data = self.strategy_combo.currentData()
        if isinstance(data, WaypointFlightStrategy):
            return data
        return WaypointFlightStrategy.from_value(data)

    def selected_reference_generator(self) -> WaypointReferenceGenerator:
        """返回解除武装阶段选定的航点命令生成方式。"""
        data = self.reference_combo.currentData()
        if isinstance(data, WaypointReferenceGenerator):
            return data
        return WaypointReferenceGenerator.from_value(data)

    def selected_tracking_controller(self) -> WaypointTrackingController:
        """返回解除武装阶段选定的航点跟踪控制方式。"""
        data = self.tracking_combo.currentData()
        if isinstance(data, WaypointTrackingController):
            return data
        return WaypointTrackingController.from_value(data)

    def _emit_send_requested(self) -> None:
        """原子发出航点和飞行前锁定的三项选择。"""
        self.send_requested.emit(
            tuple(self._waypoints),
            self.selected_strategy(),
            self.selected_reference_generator(),
            self.selected_tracking_controller(),
        )

    @staticmethod
    def _coordinate_input(
        minimum: float, maximum: float, decimals: int, suffix: str
    ) -> NoWheelDoubleSpinBox:
        """创建带窄型步进箭头和可见单位的紧凑航点输入。"""
        control = NoWheelDoubleSpinBox()
        control.setRange(minimum, maximum)
        control.setDecimals(decimals)
        control.setSingleStep(0.1)
        control.setProperty("compactValueInput", True)
        control.setSuffix(suffix.strip())
        control.setToolTip(f"输入值单位：{suffix.strip()}")
        control.setProperty("baseToolTip", control.toolTip())
        control.setKeyboardTracking(False)
        control.setProperty("waypointCoordinate", True)
        control.setFixedHeight(WaypointPanel._ROW_HEIGHT)
        control.setMinimumWidth(58)
        control.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        return control

    @staticmethod
    def _button(text: str, role: str, object_name: str) -> QPushButton:
        """创建带稳定对象名的航点操作按钮。"""
        button = QPushButton(text)
        button.setObjectName(object_name)
        button.setProperty("role", role)
        button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        button.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        return button

    @classmethod
    def _icon_button(
        cls, icon: QIcon, tooltip: str, object_name: str
    ) -> QPushButton:
        """创建只显示图标、但保留无障碍名称和明确提示的表格操作按钮。"""
        button = cls._button("", "neutral", object_name)
        button.setIcon(icon)
        button.setIconSize(QSize(16, 16))
        button.setFixedSize(32, cls._ROW_HEIGHT)
        button.setToolTip(tooltip)
        button.setProperty("baseToolTip", tooltip)
        button.setAccessibleName(tooltip)
        return button

    @property
    def waypoints(self) -> tuple[tuple[float, float, float, float], ...]:
        """返回待上传航点的不可变副本。"""
        return tuple(self._waypoints)

    def _add_waypoint(self) -> None:
        """读取已约束数值并追加一个弧度偏航航点。"""
        waypoint = (
            self.x_input.value(),
            self.y_input.value(),
            self.z_input.value(),
            math.radians(self.yaw_input.value()),
        )
        self._waypoints.append(waypoint)
        self._refresh_table(len(self._waypoints) - 1)
        self.status_label.setText(f"本地列表包含 {len(self._waypoints)} 个航点，尚未上传。")
        self.waypoints_changed.emit(f"已添加航点 #{len(self._waypoints)}")

    def _remove_selected(self) -> None:
        """删除当前选中行并保持邻近选择。"""
        row = self.table.currentRow()
        if row < 0 or row >= len(self._waypoints):
            return
        self._waypoints.pop(row)
        next_row = min(row, len(self._waypoints) - 1) if self._waypoints else None
        self._refresh_table(next_row)
        self.waypoints_changed.emit(f"已删除航点 #{row + 1}")

    def _move_selected(self, offset: int) -> None:
        """将选中航点上移或下移一行。"""
        row = self.table.currentRow()
        target = row + offset
        if row < 0 or target < 0 or target >= len(self._waypoints):
            return
        self._waypoints[row], self._waypoints[target] = (
            self._waypoints[target],
            self._waypoints[row],
        )
        self._refresh_table(target)
        self.waypoints_changed.emit(f"航点 #{row + 1} 已移动至 #{target + 1}")

    def clear_waypoints(self) -> None:
        """在主窗口完成二次确认后清空本地航点。"""
        count = len(self._waypoints)
        self._waypoints.clear()
        self._refresh_table()
        self.reset_progress()
        self.status_label.setText("航点列表已清空。")
        self.waypoints_changed.emit(f"已清空 {count} 个航点")

    def replace_waypoints(
        self,
        waypoints: tuple[tuple[float, float, float, float], ...],
        source_label: str,
    ) -> None:
        """在主窗口完成文件校验与确认后原子替换本地航点列表。"""
        self._waypoints[:] = waypoints
        self._refresh_table(0)
        self.reset_progress()
        count = len(self._waypoints)
        self.status_label.setText(f"已从 {source_label} 导入 {count} 个航点，尚未上传。")
        self.waypoints_changed.emit(f"已从 {source_label} 导入 {count} 个航点")

    def reset_progress(self) -> None:
        """清除已结束任务的进度，并忽略机载端短期残留的旧快照。"""
        self._progress_tracking = False
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("尚未执行")

    def _refresh_table(self, selected_row: int | None = None) -> None:
        """由内存列表重建只读表格，保持编号与弧度/角度转换一致。"""
        self.table.setRowCount(len(self._waypoints))
        for row, waypoint in enumerate(self._waypoints):
            values = (
                str(row + 1),
                f"{waypoint[0]:+.2f}",
                f"{waypoint[1]:+.2f}",
                f"{waypoint[2]:+.2f}",
                f"{math.degrees(waypoint[3]):+.1f}",
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(row, column, item)
        if selected_row is not None and 0 <= selected_row < self.table.rowCount():
            self.table.selectRow(selected_row)
        self._update_local_controls()

    def _update_local_controls(self) -> None:
        """按选择行和编辑锁更新删除、排序及清空按钮。"""
        row = self.table.currentRow()
        valid = 0 <= row < len(self._waypoints)
        self.remove_button.setEnabled(self._editing_enabled and valid)
        self.up_button.setEnabled(self._editing_enabled and valid and row > 0)
        self.down_button.setEnabled(
            self._editing_enabled and valid and row < len(self._waypoints) - 1
        )
        self.clear_button.setEnabled(self._editing_enabled and bool(self._waypoints))
        self.preview_button.setEnabled(
            self._preview_enabled and bool(self._waypoints)
        )
        self.import_button.setEnabled(self._editing_enabled)
        if not self._waypoints:
            self.preview_button.setToolTip("请先添加或导入至少一个航点")
        elif self._preview_enabled or self._editing_enabled:
            self.preview_button.setToolTip(
                str(self.preview_button.property("baseToolTip") or "")
            )

    def apply_availability(self, state: UiAvailability) -> None:
        """仅在已启动仿真/实机会话时可编辑；上传仍受完整飞行门控。"""
        self._editing_enabled = state.waypoint_edit
        self._preview_enabled = state.waypoint_preview
        for control in (
            self.x_input,
            self.y_input,
            self.z_input,
            self.yaw_input,
            self.add_button,
            self.import_button,
        ):
            control.setEnabled(state.waypoint_edit)
        method_combos = (
            self.strategy_combo,
            self.reference_combo,
            self.tracking_combo,
        )
        for combo in method_combos:
            combo.setEnabled(state.waypoint_configuration)
        self.table.setEnabled(state.waypoint_edit)
        self.send_button.setEnabled(state.waypoint_send)
        if not state.waypoint_edit:
            edit_tip = (
                state.flight_reason
                if state.flight_reason
                else "需先启动仿真或连接机载服务"
            )
            for control in (
                self.x_input,
                self.y_input,
                self.z_input,
                self.yaw_input,
                self.add_button,
                self.remove_button,
                self.up_button,
                self.down_button,
                self.clear_button,
                self.import_button,
                self.table,
                self.send_button,
            ):
                control.setToolTip(edit_tip)
        else:
            for control in (
                self.x_input,
                self.y_input,
                self.z_input,
                self.yaw_input,
                self.add_button,
                self.remove_button,
                self.up_button,
                self.down_button,
                self.clear_button,
                self.import_button,
                self.table,
            ):
                control.setToolTip(str(control.property("baseToolTip") or ""))
            self.send_button.setToolTip(state.flight_reason)
        if state.waypoint_configuration:
            for combo in method_combos:
                combo.setToolTip(str(combo.property("baseToolTip") or ""))
        else:
            configuration_tip = (
                "飞行过程中禁止修改航点策略、命令生成和跟踪控制；"
                "请在降落并解除武装后选择下一组实验配置。"
                if state.waypoint_edit
                else state.flight_reason
            )
            for combo in method_combos:
                combo.setToolTip(configuration_tip)
        if not self._waypoints:
            preview_tip = "请先添加或导入至少一个航点"
        elif state.waypoint_preview:
            preview_tip = str(self.preview_button.property("baseToolTip") or "")
        else:
            preview_tip = state.flight_reason or "需先启动仿真或连接机载服务"
        self.preview_button.setToolTip(preview_tip)
        self._update_local_controls()

    def update_progress(self, snapshot: VehicleSnapshot) -> None:
        """把机载 1-based 当前目标索引换算为实际已完成航点格数。"""
        if self._progress_tracking and snapshot.waypoint_count > 0:
            self.progress.setRange(0, snapshot.waypoint_count)
            # WAYPOINT 模式中的索引指向“正在飞往”的目标，因此完成数少一；
            # 可靠终态切到 HOVER 后索引等于总数，最后一格才真正填满。
            progress_value = snapshot.waypoint_index
            if snapshot.active_mode is FlightMode.WAYPOINT:
                progress_value = max(0, progress_value - 1)
            progress_value = min(progress_value, snapshot.waypoint_count)
            self.progress.setValue(progress_value)
            self.progress.setFormat(
                f"已完成 {progress_value}/{snapshot.waypoint_count}"
            )

    def complete_progress(self, waypoint_index: int, waypoint_count: int) -> None:
        """用可靠成功终态补齐可能被 LAND 状态覆盖的最后一格。"""
        if waypoint_count <= 0 or waypoint_index != waypoint_count:
            return
        self._progress_tracking = True
        self.progress.setRange(0, waypoint_count)
        self.progress.setValue(waypoint_count)
        self.progress.setFormat(f"已完成 {waypoint_count}/{waypoint_count}")

    def set_result(self, message: str, running: bool) -> None:
        """展示可靠命令结果，并在运行期间明确锁定上传按钮文本。"""
        # 仅在任务首次进入运行态时初始化；后续每个机载 RUNNING 结果不得
        # 把已经推进的进度短暂重置成“等待”，否则界面会每到一个点闪烁。
        if running and not self._progress_tracking:
            self._progress_tracking = True
            self.progress.setRange(0, max(1, len(self._waypoints)))
            self.progress.setValue(0)
            self.progress.setFormat("等待机载任务进度…")
        self.status_label.setText(message)
        self.send_button.setText("任务执行中…" if running else "发送并执行航点")
