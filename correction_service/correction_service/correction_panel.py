"""独立标记校准面板：Tag 配置、滑窗操作、位姿对照与折叠诊断。"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QResizeEvent, QWheelEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ground_station_core.qt_ui.theme import STYLE_SHEET
from ground_station_core.qt_ui.window_chrome import ShadowWindowChromeMixin

from .ros_client import CorrectionPanelClient
from .tag_editor import TagConfigEditor

# 独立面板桌面身份；保持与已有启动入口兼容。
DESKTOP_APPLICATION_NAME = "ROS 2 AprilTag Odin Correction"
# 既有校准协议；Tag 编辑为新增的独立可选服务。
INTERFACE_VERSION = "2.0"
# 对应 StartCorrection 首次和后续操作枚举。
OPERATION_FIRST = 1
OPERATION_NEXT = 2
# 窄屏使用两列命令和纵向状态组；左右并排时命令区也保持紧凑。
COMPACT_LAYOUT_WIDTH_PX = 820
# 两个编辑区域均保留完整输入宽度后，再切为左右并排。
SIDE_BY_SIDE_WIDTH_PX = 1100
# 复用主GUI主题，仅补充独立校准面板的布局与状态层级。
PANEL_STYLE_SHEET = STYLE_SHEET + """
QWidget {
    color: #182433;
    font-family: "Noto Sans CJK SC", "Noto Sans", sans-serif;
    font-size: 10pt;
}
QMainWindow { background: transparent; }
QWidget#correctionPanelRoot { background: #eef1f4; }
QFrame#subpanelWindowFrame {
    background: #f7f8fa;
    border: 1px solid #8595a5;
    border-radius: 8px;
}
QFrame#subpanelWindowFrame[windowMaximized="true"] { border-radius: 0; }
QFrame#subpanelTitleBar {
    background: #ffffff;
    border: none;
    border-bottom: 1px solid #cfd6de;
}
QLabel#subpanelWindowTitle { font-size: 11pt; font-weight: 700; }
QGroupBox {
    background: #ffffff;
    border: 1px solid #cfd6de;
    border-radius: 4px;
    margin-top: 12px;
    padding: 12px 9px 9px 9px;
    font-weight: 700;
}
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 4px; }
QLabel#valueLabel { font-family: "DejaVu Sans Mono", monospace; font-weight: 600; }
QLabel#warningLabel { color: #8b4a18; }
QLabel#summaryValue { color: #245f87; font-size: 12pt; font-weight: 700; }
QSpinBox {
    min-height: 30px;
    padding: 2px 7px;
    background: white;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
}
QPushButton[windowControl="true"] {
    min-height: 25px;
    max-height: 25px;
    min-width: 28px;
    padding: 0;
    background: transparent;
    border: 1px solid transparent;
}
QPushButton[closeControl="true"]:hover { color: white; background: #a7352a; }
"""


class _NoWheelSpinBox(QSpinBox):
    """禁用滚轮改值，避免滚动页面时误改 Tag ID 或滑窗长度。"""

    def wheelEvent(self, event: QWheelEvent) -> None:
        """将滚轮交还页面滚动，不改输入值。"""
        event.ignore()


class _AutomaticReadOnlyTagEditor(TagConfigEditor):
    """复用现有配置编辑器；自动校准期间只保留查看和重新读取。"""

    def set_service_status(self, status: dict[str, Any]) -> None:
        """将 owner 作为保存门控，不改变真实校准状态或覆盖编辑草稿。"""
        super().set_service_status({
            **status, "active": bool(status.get("active") or status.get("automatic_owner"))
        })
        # 手动空闲可尝试保存；空表/unknown 的拒绝仍由原保存逻辑和服务负责。
        if (
            not status.get("automatic_owner") and not status.get("active")
            and status.get("fresh") and status.get("service_available")
            and status.get("interface_version") == INTERFACE_VERSION
        ):
            self.save_button.setEnabled(not self._busy)
        if status.get("automatic_owner"):
            self.connection_notice.setText(f"自动校准独占中：{status['automatic_owner']}；Tag 配置只读")
        self._update_row_controls()

    def _update_row_controls(self, *_args: Any) -> None:
        """读取完成和选中行变化也不能重新开放自动期间的编辑按钮。"""
        super()._update_row_controls(*_args)
        if not hasattr(self, "_manual_edit_triggers"):
            self._manual_edit_triggers = self.table.editTriggers()
        automatic = bool(self._status.get("automatic_owner"))
        self.table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
            if automatic else self._manual_edit_triggers
        )
        self.add_button.setEnabled(not automatic and not self._busy)
        for widget in self._parameter_inputs:
            widget.setEnabled(not automatic and not self._busy)
        if automatic:
            for button in (self.save_button, self.up_button, self.down_button,
                           self.delete_button, self.clear_button):
                button.setEnabled(False)


class _PanelBridge(QObject):
    """把 ROS 客户端线程的请求结果投递到 Qt 主线程。"""

    completed = Signal(str, object, str)


class CorrectionPanelWindow(ShadowWindowChromeMixin, QMainWindow):
    """不加入飞行会话、也不发送飞行命令的滑窗标定子面板。"""

    def __init__(
        self,
        *,
        client: CorrectionPanelClient | None = None,
        parent: QWidget | None = None,
    ) -> None:
        """创建独立面板与异步客户端，不接管飞行会话。"""
        super().__init__(parent)
        self.client = client or CorrectionPanelClient()
        self._bridge = _PanelBridge(self)
        self._bridge.completed.connect(self._on_completed)
        self._request_busy = False
        self._latest_status: dict[str, Any] = {}
        self._window_value_initialized = False
        self._deferred_apply_requested = False
        self._deferred_apply_job_id = ""
        self._compact_layout: bool | None = None
        self._side_by_side: bool | None = None

        self.setWindowTitle(DESKTOP_APPLICATION_NAME)
        self._configure_window_chrome()
        self.setMinimumSize(560, 520)
        self.resize(1180, 860)
        self._build_ui()
        self.client.start()
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def _build_ui(self) -> None:
        """在同一页面构建校准、Tag编辑、主要状态和位姿对照区。"""
        root = QWidget()
        root.setObjectName("correctionPanelRoot")
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_window_title_bar("标记校准面板"))

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        body = QWidget()
        scroll.setWidget(body)
        layout = QVBoxLayout(body)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        warning = QLabel(
            "仅修正 x / y / yaw；本面板不会解锁、起飞或发送飞行模式命令。"
            "应用 ACK 也不代表 EKF 已稳定，PoseStamped 仍没有 reset counter。"
        )
        warning.setObjectName("warningLabel")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        self._main_layout = QGridLayout()
        self._main_layout.setSpacing(12)
        layout.addLayout(self._main_layout, 1)
        self._calibration_column = QWidget()
        calibration_layout = QVBoxLayout(self._calibration_column)
        calibration_layout.setContentsMargins(0, 0, 0, 0)
        calibration_layout.setSpacing(10)
        self._tag_group = QGroupBox("Tag 配置")
        self.tag_editor = _AutomaticReadOnlyTagEditor(self.client)
        tag_layout = QVBoxLayout(self._tag_group)
        tag_layout.setContentsMargins(6, 8, 6, 6)
        tag_layout.addWidget(self.tag_editor)

        command_group = QGroupBox("多 Tag 滑窗标定")
        self._command_layout = QGridLayout(command_group)
        self._tag_id_label = QLabel("预期 Tag ID")
        self.tag_id_input = _NoWheelSpinBox()
        self.tag_id_input.setRange(0, 2_147_483_647)
        self._window_size_label = QLabel("滑窗长度")
        self.window_size_input = _NoWheelSpinBox()
        self.window_size_input.setRange(2, 20)
        self.window_size_input.setValue(5)
        self._inputs_row = QWidget()
        inputs = QHBoxLayout(self._inputs_row)
        inputs.setContentsMargins(0, 0, 0, 0)
        for widget in (self._tag_id_label, self.tag_id_input, self._window_size_label, self.window_size_input):
            inputs.addWidget(widget)
        inputs.addStretch(1)
        self.apply_checkbox = QCheckBox("本次收敛后弹窗确认应用（默认仅 dry-run）")
        self.apply_checkbox.setChecked(False)
        self._command_layout.addWidget(self.apply_checkbox, 1, 0, 1, 4)

        self.first_button = QPushButton("开始首次校准")
        self.first_button.setProperty("role", "primary")
        self.first_button.clicked.connect(lambda: self._start(OPERATION_FIRST))
        self.start_button = self.first_button  # 保留既有测试/调用入口。
        self.next_button = QPushButton("开始第 2 次校准")
        self.next_button.setProperty("role", "primary")
        self.next_button.clicked.connect(lambda: self._start(OPERATION_NEXT))
        self.stop_button = QPushButton("停止本次采样")
        self.stop_button.setProperty("role", "danger")
        self.stop_button.clicked.connect(self._stop)
        self._command_layout.addWidget(self.first_button, 2, 0)
        self._command_layout.addWidget(self.next_button, 2, 1)
        self._command_layout.addWidget(self.stop_button, 2, 2, 1, 2)

        self.clear_button = QPushButton("清空滑窗")
        self.clear_button.clicked.connect(self._clear)
        self.apply_saved_button = QPushButton("应用当前滑窗结果")
        self.apply_saved_button.clicked.connect(self._apply_saved)
        self._command_layout.addWidget(self.clear_button, 3, 0, 1, 2)
        self._command_layout.addWidget(self.apply_saved_button, 3, 2, 1, 2)
        self.command_message = QLabel("等待 correction_service 2.0 状态")
        self.command_message.setWordWrap(True)
        self._command_layout.addWidget(self.command_message, 4, 0, 1, 4)
        calibration_layout.addWidget(command_group)
        for control in (self.tag_id_input, self.window_size_input):
            control.setMaximumWidth(130)
        for button in (self.first_button, self.next_button, self.stop_button,
                       self.clear_button, self.apply_saved_button):
            button.setMaximumWidth(210)
        self.apply_checkbox.setText("收敛后确认应用（默认仅采样）")

        window_group = QGroupBox("服务端权威滑窗")
        window_form = QFormLayout(window_group)
        self.window_summary = self._value_label()
        self.window_revisions = self._value_label()
        self.window_tags = self._value_label()
        self.keyframe_values = QLabel("—")
        self.keyframe_values.setWordWrap(True)
        self.keyframe_values.setTextInteractionFlags(
            self.keyframe_values.textInteractionFlags()
        )
        window_form.addRow("状态 / N", self.window_summary)
        window_form.addRow("Tag 顺序", self.window_tags)
        calibration_layout.addWidget(window_group)
        calibration_layout.addStretch(1)

        self._status_layout = QGridLayout()
        correction_group = QGroupBox("候选与质量")
        correction_form = QFormLayout(correction_group)
        self.correction_state = self._value_label()
        self.candidate_value = self._value_label()
        self.quality_value = self._value_label()
        self.registration_value = self._value_label()
        self.delta_value = self._value_label()
        self.sample_value = self._value_label()
        self.performance_value = self._value_label()
        self.apply_state_value = QLabel("—")
        self.apply_state_value.setWordWrap(True)
        correction_form.addRow("任务", self.correction_state)
        correction_form.addRow("候选", self.candidate_value)
        correction_form.addRow("delta / 预计跳变", self.delta_value)
        correction_form.addRow("可应用性", self.apply_state_value)
        self._correction_group = correction_group
        self._status_layout.addWidget(correction_group, 0, 0)

        extnav_group = QGroupBox("extnav 权威状态")
        extnav_form = QFormLayout(extnav_group)
        self.extnav_state = self._value_label()
        self.extnav_value = self._value_label()
        self.session_value = self._value_label()
        self.reference_value = self._value_label()
        self.extnav_event = QLabel("—")
        self.extnav_event.setWordWrap(True)
        extnav_form.addRow("Odin / 修正", self.extnav_state)
        extnav_form.addRow("active", self.extnav_value)
        extnav_form.addRow("最近事件", self.extnav_event)
        self._extnav_group = extnav_group
        self._status_layout.addWidget(extnav_group, 0, 1)
        layout.addLayout(self._status_layout)

        self.window_summary.setObjectName("summaryValue")
        self.extnav_value.setObjectName("summaryValue")
        self.candidate_value.setObjectName("summaryValue")

        compare_group = QGroupBox("FCU 位姿对照 · 不同参考系，不可直接相减")
        compare_layout = QVBoxLayout(compare_group)
        self.pose_table = QTableWidget(4, 5)
        self.pose_table.setHorizontalHeaderLabels(("x (m)", "y (m)", "z (m)", "yaw (°)", "年龄 (s)"))
        self.pose_table.setVerticalHeaderLabels(("FCU 原始", "FCU 修正", "FCU 输入", "EKF 融合"))
        self.pose_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.pose_table.horizontalHeader().setMinimumSectionSize(45)
        self.pose_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.pose_table.setAlternatingRowColors(True)
        self.pose_table.setFixedHeight(165)
        compare_layout.addWidget(self.pose_table)
        layout.addWidget(compare_group)

        self.raw_pose = self._value_label()
        self.corrected_pose = self._value_label()
        self.pose_fcu = self._value_label()
        self.final_pose = self._value_label()
        self.result_value = QLabel("尚无终态结果")
        self.result_value.setWordWrap(True)
        details = QWidget()
        detail_form = QFormLayout(details)
        for title, label in (
            ("instance / revisions", self.window_revisions),
            ("keyframe P/Q/质量", self.keyframe_values),
            ("停留段质量", self.quality_value),
            ("基线 / 配准", self.registration_value),
            ("帧/样本", self.sample_value),
            ("性能/时间源", self.performance_value),
            ("session / revision", self.session_value),
            ("中心模式 / 局部原点 b", self.reference_value),
            ("FCU 原始 · odom / 未经 Tag 修正", self.raw_pose),
            ("FCU 修正 · 世界 XY / 原始 Z", self.corrected_pose),
            ("FCU 输入 · extnav", self.pose_fcu),
            ("MAVROS EKF 融合输出", self.final_pose),
            ("最近任务终态", self.result_value),
        ):
            detail_form.addRow(title, label)
        self.details_group = QGroupBox("展开诊断与任务详情")
        self.details_group.setCheckable(True)
        self.details_group.setChecked(False)
        details.setVisible(False)
        self.details_group.toggled.connect(details.setVisible)
        QVBoxLayout(self.details_group).addWidget(details)
        layout.addWidget(self.details_group)
        root_layout.addWidget(scroll, 1)
        self._update_responsive_layout(self.width())
        self._sync_window_chrome()

    @staticmethod
    def _value_label() -> QLabel:
        """创建可换行收缩且不会因状态刷新改变字体的等宽数值标签。"""
        label = QLabel("—")
        label.setObjectName("valueLabel")
        label.setWordWrap(True)
        label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        return label

    def _update_responsive_layout(self, width: int) -> None:
        """按窗口宽度排列两个编辑区、命令与状态，保持内容可访问。"""
        side_by_side = width >= SIDE_BY_SIDE_WIDTH_PX
        compact = side_by_side or width < COMPACT_LAYOUT_WIDTH_PX
        if self._compact_layout == compact and self._side_by_side == side_by_side:
            return
        self._compact_layout = compact
        self._side_by_side = side_by_side
        self._main_layout.removeWidget(self._calibration_column)
        self._main_layout.removeWidget(self._tag_group)
        self._main_layout.setColumnStretch(0, 4 if side_by_side else 1)
        self._main_layout.setColumnStretch(1, 5 if side_by_side else 0)
        self._main_layout.addWidget(self._calibration_column, 0, 0)
        self._main_layout.addWidget(self._tag_group, 0 if side_by_side else 1, 1 if side_by_side else 0)

        command_widgets = (
            self._inputs_row,
            self.apply_checkbox,
            self.first_button,
            self.next_button,
            self.stop_button,
            self.clear_button,
            self.apply_saved_button,
            self.command_message,
        )
        for widget in command_widgets:
            self._command_layout.removeWidget(widget)
        for column in range(4):
            self._command_layout.setColumnStretch(column, 0)

        if compact:
            self._command_layout.addWidget(self._inputs_row, 0, 0, 1, 2)
            self._command_layout.addWidget(self.apply_checkbox, 2, 0, 1, 2)
            self._command_layout.addWidget(self.first_button, 3, 0)
            self._command_layout.addWidget(self.next_button, 3, 1)
            self._command_layout.addWidget(self.stop_button, 4, 0)
            self._command_layout.addWidget(self.clear_button, 4, 1)
            self._command_layout.addWidget(self.apply_saved_button, 5, 0, 1, 2)
            self._command_layout.addWidget(self.command_message, 6, 0, 1, 2)
            self._command_layout.setColumnStretch(0, 1)
            self._command_layout.setColumnStretch(1, 1)
        else:
            self._command_layout.addWidget(self._inputs_row, 0, 0, 1, 4)
            self._command_layout.addWidget(self.apply_checkbox, 1, 0, 1, 4)
            self._command_layout.addWidget(self.first_button, 2, 0)
            self._command_layout.addWidget(self.next_button, 2, 1)
            self._command_layout.addWidget(self.stop_button, 2, 2, 1, 2)
            self._command_layout.addWidget(self.clear_button, 3, 0, 1, 2)
            self._command_layout.addWidget(self.apply_saved_button, 3, 2, 1, 2)
            self._command_layout.addWidget(self.command_message, 4, 0, 1, 4)
            for column in range(4):
                self._command_layout.setColumnStretch(column, 1)

        status_compact = width < COMPACT_LAYOUT_WIDTH_PX
        self._status_layout.removeWidget(self._correction_group)
        self._status_layout.removeWidget(self._extnav_group)
        self._status_layout.setColumnStretch(0, 3 if not status_compact else 1)
        self._status_layout.setColumnStretch(1, 2 if not status_compact else 0)
        self._status_layout.addWidget(self._correction_group, 0, 0)
        self._status_layout.addWidget(
            self._extnav_group,
            1 if status_compact else 0,
            0 if status_compact else 1,
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        """窗口缩放时同步自绘边框，并在断点两侧重排面板。"""
        super().resizeEvent(event)
        if hasattr(self, "_command_layout"):
            self._update_responsive_layout(event.size().width())

    def _start(self, operation: int = OPERATION_FIRST) -> None:
        """先异步 dry-run；选中应用时待候选冻结后再逐值二次确认。"""
        correction = self._latest_status.get("correction", {})
        if correction.get("automatic_owner"):
            self.command_message.setText("自动校准期间面板只读，请通过飞行任务取消/接管")
            return
        extnav = self._latest_status.get("extnav", {})
        self._deferred_apply_requested = self.apply_checkbox.isChecked()
        self._deferred_apply_job_id = ""
        self._request_busy = True
        self.command_message.setText(
            "正在提交 dry-run start 请求；候选冻结前不会授权 extnav 写入…"
            if self._deferred_apply_requested
            else "正在提交 dry-run start 请求…"
        )
        self.client.request_start(
            operation,
            self.tag_id_input.value(),
            self.window_size_input.value(),
            False,
            str(correction.get("service_instance_id", "")),
            int(correction.get("window_revision", 0)),
            int(extnav.get("revision", 0)),
            lambda payload, error: self._bridge.completed.emit("start", payload, error),
        )
        self._refresh_controls()

    def _stop(self) -> None:
        """停止状态中唯一 job；提交已发出时后端仍会如实完成对账。"""
        correction = self._latest_status.get("correction", {})
        if correction.get("automatic_owner"):
            self.command_message.setText("自动校准期间面板只读，请通过飞行任务取消/接管")
            return
        self._request_busy = True
        self.command_message.setText("正在提交 stop 请求…")
        self.client.request_stop(
            str(correction.get("job_id", "")),
            str(correction.get("service_instance_id", "")),
            lambda payload, error: self._bridge.completed.emit("stop", payload, error),
        )
        self._refresh_controls()

    def _clear(self) -> None:
        """默认取消确认后只清空服务窗口，明确说明 extnav active 不受影响。"""
        correction = self._latest_status.get("correction", {})
        if correction.get("automatic_owner"):
            self.command_message.setText("自动校准期间面板只读，请通过飞行任务取消/接管")
            return
        answer = QMessageBox.warning(
            self,
            "确认清空滑窗",
            f"将清空 {correction.get('window_count', 0)} 个 keyframe、候选和累计 N。\n"
            "extnav 当前 active correction 不会被清除。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._request_busy = True
        self.client.request_clear(
            str(correction.get("service_instance_id", "")),
            int(correction.get("window_revision", 0)),
            lambda payload, error: self._bridge.completed.emit("clear", payload, error),
        )
        self._refresh_controls()

    def _apply_saved(self) -> None:
        """逐值展示保存候选、阶段、revisions 和预计跳变后再显式应用。"""
        correction = self._latest_status.get("correction", {})
        if correction.get("automatic_owner"):
            self.command_message.setText("自动校准期间面板只读，请通过飞行任务取消/接管")
            return
        extnav = self._latest_status.get("extnav", {})
        unknown = correction.get("application_state") == "unknown"
        text = (
            f"候选阶段：{correction.get('candidate_stage', '—')}\n"
            f"C = x {correction.get('x_m', 0):+.4f} m，"
            f"y {correction.get('y_m', 0):+.4f} m，"
            f"yaw {correction.get('yaw_deg', 0):+.3f}°\n"
            f"窗口：{correction.get('window_count', 0)}/"
            f"{correction.get('window_size', 0)}，"
            f"window r{correction.get('window_revision', 0)}，"
            f"session={extnav.get('session', '—')}，"
            f"extnav r{extnav.get('revision', 0)}\n"
            f"最近复算预计跳变：{correction.get('position_jump_m', 0):.4f} m / "
            f"{correction.get('yaw_jump_deg', 0):.3f}°。"
            "服务会重新短时订阅 raw 并再次门控。\n\n"
            + (
                "当前为 application_unknown；将使用同一 job/candidate 幂等重试，"
                "不会刷新 CAS 强行覆盖。\n"
                if unknown
                else ""
            )
            + "PoseStamped 无 estimator reset counter；应用事实不等于 EKF 稳定或可实飞。"
        )
        answer = QMessageBox.warning(
            self,
            "确认应用保存候选" if not unknown else "确认重试未知应用",
            text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._request_busy = True
        self.client.request_apply_saved(
            str(correction.get("service_instance_id", "")),
            int(correction.get("window_revision", 0)),
            int(extnav.get("revision", 0)),
            lambda payload, error: self._bridge.completed.emit(
                "apply_saved", payload, error
            ),
        )
        self._refresh_controls()

    def _on_completed(self, kind: str, payload: dict[str, Any], error: str) -> None:
        """显示服务级 ACK；采样/应用可靠终态仍以 result 话题为准。"""
        self._request_busy = False
        if error:
            if kind == "start":
                self._deferred_apply_requested = False
                self._deferred_apply_job_id = ""
            self.command_message.setText(f"{kind} 被拒绝/失败：{error}")
        else:
            job = f" job={payload.get('job_id')}" if payload.get("job_id") else ""
            if kind == "start" and self._deferred_apply_requested:
                self._deferred_apply_job_id = str(payload.get("job_id", ""))
                self._deferred_apply_requested = False
                self.apply_checkbox.setChecked(False)
                self.command_message.setText(
                    f"dry-run 已接受{job}；候选保存后将逐值弹窗确认应用"
                )
            else:
                self.command_message.setText(
                    f"{kind} 已接受{job}：{payload.get('message', '')}"
                )
        self._refresh_controls()

    def _refresh(self) -> None:
        """读取纯数据快照并刷新窗口、候选、应用事实和四段位姿。"""
        status = self.client.status()
        self._latest_status = status
        self.tag_editor.set_service_status({
            **status.get("correction", {}),
            "client_error": str(status.get("startup_error", "")),
            "aircraft_available": any(
                status.get(key, {}).get("fresh")
                for key in ("extnav", "raw", "corrected", "pose_fcu", "final")
            ),
        })
        correction = status.get("correction", {})
        extnav = status.get("extnav", {})
        startup_error = str(status.get("startup_error", ""))
        correction_fresh = bool(correction.get("fresh"))
        extnav_fresh = bool(extnav.get("fresh"))

        if startup_error:
            self.command_message.setText(startup_error)
        if correction_fresh:
            if not self._window_value_initialized:
                self.window_size_input.setValue(
                    max(2, int(correction.get("window_size", 5)))
                )
                self._window_value_initialized = True
            self.next_button.setText(
                f"开始第 {correction.get('next_calibration', 2)} 次校准"
            )
            self.correction_state.setText(
                f"{correction.get('operation', '—')}/"
                f"{correction.get('state', '—')} · {correction.get('message', '')}"
            )
            self.window_summary.setText(
                f"{correction.get('window_state', '—')} · "
                f"{correction.get('window_count', 0)}/"
                f"{correction.get('window_size', 0)} · "
                f"成功 N={correction.get('success_count', 0)}，"
                f"下一次={correction.get('next_calibration', 1)}"
            )
            self.window_revisions.setText(
                f"{correction.get('service_instance_id', '—')[:12]} / "
                f"window r{correction.get('window_revision', 0)} / "
                f"extnav r{extnav.get('revision', 0)}\n"
                f"service session={correction.get('service_session', '—')} / "
                f"window session={correction.get('window_session', '—')}"
            )
            keyframes = correction.get("keyframes", [])
            self.window_tags.setText(
                " → ".join(str(item.get("tag_id")) for item in keyframes) or "空"
            )
            self.keyframe_values.setText(self._format_keyframes(keyframes))
            self.candidate_value.setText(
                f"{correction.get('candidate_stage', '—')} · "
                f"x={correction.get('x_m', 0):+.4f} m  "
                f"y={correction.get('y_m', 0):+.4f} m  "
                f"yaw={correction.get('yaw_deg', 0):+.3f}°  "
                f"tilt={correction.get('tilt_deg', 0):.3f}°"
            )
            self.quality_value.setText(
                f"pos σ={correction.get('position_std_m', 0):.4f} m  "
                f"yaw σ={correction.get('yaw_std_deg', 0):.3f}°  "
                f"reproj={correction.get('reprojection_px', 0):.3f} px  "
                f"match={correction.get('odom_match_ms', 0):.2f} ms"
            )
            self.registration_value.setText(
                f"Tag {correction.get('baseline_tag_a', -1)}↔"
                f"{correction.get('baseline_tag_b', -1)} · "
                f"O/W={correction.get('odin_baseline_m', 0):.3f}/"
                f"{correction.get('world_baseline_m', 0):.3f} m · "
                f"RMS/max={correction.get('registration_rms_m', 0):.4f}/"
                f"{correction.get('registration_max_m', 0):.4f} m · "
                f"model yaw σ={correction.get('model_yaw_std_deg', 0):.3f}°"
            )
            self.delta_value.setText(
                f"Δ=({correction.get('delta_x_m', 0):+.4f}, "
                f"{correction.get('delta_y_m', 0):+.4f}) m / "
                f"{correction.get('delta_yaw_deg', 0):+.3f}° · "
                f"FCU jump={correction.get('position_jump_m', 0):.4f} m / "
                f"{correction.get('yaw_jump_deg', 0):.3f}°"
            )
            self.sample_value.setText(
                f"recv/proc={correction.get('frames_received', 0)}/"
                f"{correction.get('frames_processed', 0)}  "
                f"Tag解码次数={correction.get('detections', 0)}  "
                f"accept/reject={correction.get('samples', 0)}/"
                f"{correction.get('rejected', 0)}"
            )
            self.performance_value.setText(
                f"{correction.get('processing_rate_hz', 0):.2f} Hz / "
                f"{correction.get('processing_time_ms', 0):.1f} ms · "
                f"{correction.get('odom_time_source', '—')}"
            )
            self.apply_state_value.setText(
                f"{correction.get('application_state', 'none')} · "
                f"{'可应用' if correction.get('can_apply') else '不可应用/需复查'} · "
                f"{correction.get('apply_reason', '')}"
            )
        else:
            self.correction_state.setText("correction_service 状态不可用/过期")
            for label in (self.window_summary, self.window_tags, self.candidate_value,
                          self.delta_value, self.apply_state_value):
                label.setText("不可用/过期")

        if extnav_fresh:
            self.extnav_state.setText(
                f"Odin={'可用' if extnav.get('odin_available') else '不可用'} · "
                f"correction={'有效' if extnav.get('valid') else 'identity'} · "
                f"API {extnav.get('interface_version', '—')}"
            )
            self.extnav_value.setText(
                f"x={extnav.get('x_m', 0):+.4f} m  "
                f"y={extnav.get('y_m', 0):+.4f} m  "
                f"yaw={extnav.get('yaw_deg', 0):+.3f}°"
            )
            self.session_value.setText(
                f"{extnav.get('session', '—')} / r{extnav.get('revision', 0)} "
                f"reset={extnav.get('reset_counter', 0)}"
            )
            lever = extnav.get("lever_arm_m", (0.0, 0.0, 0.0))
            if extnav.get("final_sample_available"):
                final_sample = (
                    f"sample={extnav.get('final_sample_reference_mode', '—')} "
                    f"({'valid' if extnav.get('final_sample_valid') else 'identity'}) "
                    f"r{extnav.get('final_sample_revision', 0)} "
                    f"session={extnav.get('final_sample_session', '—')}"
                )
            else:
                final_sample = "sample=waiting fresh raw"
            self.reference_value.setText(
                f"active={extnav.get('reference_mode', '—')} / "
                f"b=({lever[0]:+.3f},{lever[1]:+.3f},{lever[2]:+.3f}) m / "
                f"{final_sample}"
            )
            self.extnav_event.setText(str(extnav.get("last_event", "—")))
        else:
            self.extnav_state.setText("extnav correction 状态不可用/过期")
            self.extnav_value.setText("不可用/过期")

        for row, key in enumerate(("raw", "corrected", "pose_fcu", "final")):
            snapshot = status.get(key, {})
            for column, field in enumerate(("x_m", "y_m", "z_m", "yaw_deg", "age_seconds")):
                text = f"{snapshot.get(field, 0):+.4f}" if snapshot.get("fresh") else "—"
                self.pose_table.setItem(row, column, QTableWidgetItem(text))
        self.raw_pose.setText(self._format_pose(status.get("raw", {})))
        self.corrected_pose.setText(self._format_pose(status.get("corrected", {})))
        self.pose_fcu.setText(self._format_pose(status.get("pose_fcu", {})))
        self.final_pose.setText(self._format_pose(status.get("final", {})))
        result = status.get("result", {})
        if result.get("fresh"):
            self.result_value.setText(
                f"job={result.get('job_id', '—')} · success={result.get('success')} · "
                f"window_saved={result.get('window_saved')} · "
                f"resources_released={result.get('resources_released')} · "
                f"application={result.get('application_state', 'none')} "
                f"(status_confirm={result.get('confirmed_by_status')})\n"
                f"{result.get('outcome', '')} · {result.get('message', '')}\n"
                f"C=({result.get('x_m', 0):+.4f}, {result.get('y_m', 0):+.4f}) m, "
                f"yaw={result.get('yaw_deg', 0):+.3f}° · "
                f"window r{result.get('window_revision', 0)} · "
                f"N={result.get('success_count', 0)}\n"
                f"log={result.get('log_path', '')}"
            )
            self._maybe_offer_deferred_apply(result, correction)
        self._refresh_controls()

    def _maybe_offer_deferred_apply(
        self, result: dict[str, Any], correction: dict[str, Any]
    ) -> None:
        """只为匹配 job 的已保存 dry-run 候选弹出一次逐值应用确认。"""
        if correction.get("automatic_owner"):
            self._deferred_apply_job_id = ""
            return
        if not self._deferred_apply_job_id:
            return
        if str(result.get("job_id", "")) != self._deferred_apply_job_id:
            return
        if not result.get("success") or not result.get("window_saved"):
            self.command_message.setText("dry-run 未保存合格候选，已取消后续应用确认")
            self._deferred_apply_job_id = ""
            return
        candidate_ready = bool(
            correction.get("candidate_saved")
            and correction.get("candidate_valid")
            and int(correction.get("candidate_window_revision", -1))
            == int(result.get("window_revision", -2))
        )
        if not candidate_ready:
            return
        self._deferred_apply_job_id = ""
        QTimer.singleShot(0, self._apply_saved)

    @staticmethod
    def _format_keyframes(keyframes: list[dict[str, Any]]) -> str:
        """逐行显示 P/Q、样本、时间块和绝对公制 sigma。"""
        if not keyframes:
            return "空窗口"
        return "\n".join(
            f"Tag {item.get('tag_id')}: "
            f"P=({item.get('p_x_m', 0):+.3f},{item.get('p_y_m', 0):+.3f}) m, "
            f"Q=({item.get('q_x_m', 0):+.3f},{item.get('q_y_m', 0):+.3f}) m, "
            f"σeff={item.get('sigma_m', 0):.4f} m, "
            f"samples/blocks={item.get('samples', 0)}/{item.get('blocks', 0)}, "
            f"{item.get('time_source', '—')}"
            for item in keyframes
        )

    @staticmethod
    def _format_pose(snapshot: dict[str, Any]) -> str:
        """格式化一类位姿并显式显示本机接收年龄。"""
        if not snapshot.get("fresh"):
            return "不可用/过期"
        return (
            f"x={snapshot.get('x_m', 0):+.4f} m  "
            f"y={snapshot.get('y_m', 0):+.4f} m  "
            f"z={snapshot.get('z_m', 0):+.4f} m  "
            f"yaw={snapshot.get('yaw_deg', 0):+.3f}°  "
            f"age={snapshot.get('age_seconds', 0):.2f}s"
        )

    def _refresh_controls(self) -> None:
        """自动期间只读；手动空闲时开放调试，由服务保留原有准入规则。"""
        correction = self._latest_status.get("correction", {})
        correction_ready = bool(
            correction.get("fresh")
            and correction.get("service_available")
            and correction.get("interface_version") == INTERFACE_VERSION
        )
        manual = not bool(correction.get("automatic_owner"))
        active = bool(correction.get("active"))
        unknown = correction.get("application_state") == "unknown"
        # 手动空闲时允许调试全部命令；窗口、Tag 和应用准入由服务解释拒绝原因。
        writable = correction_ready and manual and not active and not self._request_busy
        self.first_button.setEnabled(writable)
        self.next_button.setEnabled(writable)
        self.stop_button.setEnabled(correction_ready and manual and not self._request_busy)
        self.clear_button.setEnabled(writable)
        self.apply_saved_button.setText(
            "重试/对账未知应用" if unknown else "应用当前滑窗结果"
        )
        self.apply_saved_button.setEnabled(writable)
        self.tag_id_input.setEnabled(writable)
        self.window_size_input.setEnabled(writable)
        self.apply_checkbox.setEnabled(writable)
        if not manual:
            self.command_message.setText(f"自动校准独占中：{correction['automatic_owner']}；面板只读")

    def closeEvent(self, event: Any) -> None:
        """关闭本地面板连接，但不把 UI 生命周期当成 stop/clear。"""
        self._timer.stop()
        self.client.close()
        super().closeEvent(event)


def main(args: list[str] | None = None) -> int:
    """作为 ROS console script 启动独立 Qt 事件循环。"""
    import sys

    from PySide6.QtWidgets import QApplication

    application = QApplication(args if args is not None else sys.argv)
    application.setApplicationName(DESKTOP_APPLICATION_NAME)
    application.setApplicationDisplayName(DESKTOP_APPLICATION_NAME)
    application.setStyle("Fusion")
    application.setStyleSheet(PANEL_STYLE_SHEET)
    window = CorrectionPanelWindow()
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
