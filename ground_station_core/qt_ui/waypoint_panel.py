"""本地 ENU 航点编辑、排序、进度与上传面板。"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
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

from ..models import VehicleSnapshot, WaypointFlightStrategy
from .state import UiAvailability
from .widgets import Card, NoWheelDoubleSpinBox


class WaypointPanel(QWidget):
    """维护尚未上传的本地航点副本，并把最终列表作为单一信号发出。"""

    # 参数为 (waypoints_tuple, WaypointFlightStrategy)。
    send_requested = Signal(object, object)
    clear_requested = Signal()
    waypoints_changed = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        """构建数值输入、航点表、排序按钮和执行进度。"""
        super().__init__(parent)
        self.setMinimumWidth(420)
        self.setMinimumHeight(500)
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._editing_enabled = True
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
            "坐标系为本地 ENU；上传的是列表副本，到达判定与推进由机载服务完成。",
        )
        card.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        # 标签与输入同排：X [框] Y [框] Z [框] Yaw [框]，压缩顶部高度留给列表。
        coords = QHBoxLayout()
        coords.setSpacing(6)
        self.x_input = self._coordinate_input(-10000.0, 10000.0, 2, " m")
        self.y_input = self._coordinate_input(-10000.0, 10000.0, 2, " m")
        self.z_input = self._coordinate_input(-1000.0, 10000.0, 2, " m")
        self.yaw_input = self._coordinate_input(-180.0, 180.0, 1, " °")
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

        inputs = QVBoxLayout()
        inputs.setSpacing(6)
        inputs.addLayout(coords)
        self.add_button = self._button("添加航点", "primary", "addWaypointButton")
        self.add_button.clicked.connect(self._add_waypoint)
        inputs.addWidget(self.add_button)
        # stretch=0：坐标行与添加按钮固定在卡片顶部。
        card.content_layout.addLayout(inputs, 0)

        self.table = QTableWidget(0, 5)
        self.table.setObjectName("waypointTable")
        self.table.setHorizontalHeaderLabels(
            ("#", "X / m", "Y / m", "Z / m", "Yaw / °")
        )
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setMinimumHeight(95)
        # 取消固定最大高度，使中间列表成为唯一纵向伸缩区。
        self.table.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        for column in range(1, 5):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._update_local_controls)
        card.content_layout.addWidget(self.table, 1)

        controls = QHBoxLayout()
        self.remove_button = self._button("删除选中", "neutral", "removeWaypointButton")
        self.up_button = self._button("上移", "neutral", "moveWaypointUpButton")
        self.down_button = self._button("下移", "neutral", "moveWaypointDownButton")
        self.clear_button = self._button("清空", "danger", "clearWaypointButton")
        self.remove_button.clicked.connect(self._remove_selected)
        self.up_button.clicked.connect(lambda: self._move_selected(-1))
        self.down_button.clicked.connect(lambda: self._move_selected(1))
        self.clear_button.clicked.connect(self.clear_requested)
        for button in (
            self.remove_button,
            self.up_button,
            self.down_button,
            self.clear_button,
        ):
            button.setProperty("compact", True)
            controls.addWidget(button)
        controls.addStretch(1)
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
        """创建任务发送、权威进度和结果状态区（高度固定贴底）。"""
        card = Card("执行与进度")
        card.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum
        )
        self.progress = QProgressBar()
        self.progress.setObjectName("waypointProgress")
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("尚未执行")
        card.content_layout.addWidget(self.progress)
        self.status_label = QLabel("请先编辑航点，再在飞行器就绪后发送。")
        self.status_label.setObjectName("mutedLabel")
        self.status_label.setWordWrap(True)
        card.content_layout.addWidget(self.status_label)

        # 左：发送执行；右：飞行策略（避障类仅预留，当前均按直线飞行）。
        send_row = QHBoxLayout()
        send_row.setSpacing(8)
        self.send_button = self._button("发送并执行航点", "success", "sendWaypointButton")
        self.send_button.clicked.connect(self._emit_send_requested)
        self.strategy_combo = QComboBox()
        self.strategy_combo.setObjectName("waypointStrategyCombo")
        self.strategy_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self.strategy_combo.setToolTip(
            "航点飞行策略。当前仅实现「直线飞行」；"
            "「自动避障」与「遇到障碍悬停」为预留选项，发送后仍按直线飞行执行。"
        )
        for strategy in WaypointFlightStrategy:
            self.strategy_combo.addItem(strategy.label, strategy)
        self.strategy_combo.setCurrentIndex(0)
        send_row.addWidget(self.send_button, 1)
        send_row.addWidget(self.strategy_combo, 1)
        card.content_layout.addLayout(send_row)
        return card

    def selected_strategy(self) -> WaypointFlightStrategy:
        """返回当前下拉框选中的航点飞行策略。"""
        data = self.strategy_combo.currentData()
        if isinstance(data, WaypointFlightStrategy):
            return data
        return WaypointFlightStrategy.from_value(data)

    def _emit_send_requested(self) -> None:
        """发出航点列表与所选策略（策略尚未实现时仍按直线飞行）。"""
        self.send_requested.emit(tuple(self._waypoints), self.selected_strategy())

    @staticmethod
    def _coordinate_input(
        minimum: float, maximum: float, decimals: int, suffix: str
    ) -> NoWheelDoubleSpinBox:
        """创建有界航点数值输入（纵向固定，不参与栏高伸缩）。"""
        control = NoWheelDoubleSpinBox()
        control.setRange(minimum, maximum)
        control.setDecimals(decimals)
        control.setSingleStep(0.1)
        control.setSuffix(suffix)
        control.setKeyboardTracking(False)
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

    def apply_availability(self, state: UiAvailability) -> None:
        """仅在已启动仿真/实机会话时可编辑；上传仍受完整飞行门控。"""
        self._editing_enabled = state.waypoint_edit
        for control in (
            self.x_input,
            self.y_input,
            self.z_input,
            self.yaw_input,
            self.add_button,
            self.strategy_combo,
        ):
            control.setEnabled(state.waypoint_edit)
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
                self.table,
                self.strategy_combo,
                self.send_button,
            ):
                control.setToolTip(edit_tip)
        else:
            self.send_button.setToolTip(state.flight_reason)
            self.strategy_combo.setToolTip(
                "航点飞行策略。当前仅实现「直线飞行」；"
                "「自动避障」与「遇到障碍悬停」为预留选项，发送后仍按直线飞行执行。"
            )
        self._update_local_controls()

    def update_progress(self, snapshot: VehicleSnapshot) -> None:
        """直接显示机载报告的 1-based 航点索引和总数。"""
        if self._progress_tracking and snapshot.waypoint_count > 0:
            self.progress.setRange(0, snapshot.waypoint_count)
            progress_value = min(
                snapshot.waypoint_index, snapshot.waypoint_count
            )
            self.progress.setValue(progress_value)
            self.progress.setFormat(
                f"机载进度 {snapshot.waypoint_index}/{snapshot.waypoint_count}"
            )

    def set_result(self, message: str, running: bool) -> None:
        """展示可靠命令结果，并在运行期间明确锁定上传按钮文本。"""
        if running:
            self._progress_tracking = True
            self.progress.setRange(0, max(1, len(self._waypoints)))
            self.progress.setValue(0)
            self.progress.setFormat("等待机载任务进度…")
        self.status_label.setText(message)
        self.send_button.setText("任务执行中…" if running else "发送并执行航点")
