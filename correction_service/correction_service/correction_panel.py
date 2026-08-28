"""独立 AprilTag-Odin 修正面板：任务控制、质量、ACK 与三路位姿对照。"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ground_station_core.qt_ui.window_chrome import ShadowWindowChromeMixin

from .ros_client import CorrectionPanelClient

DESKTOP_APPLICATION_NAME = "ROS 2 AprilTag Odin Correction"
PANEL_STYLE_SHEET = """
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
QSpinBox {
    min-height: 30px;
    padding: 2px 7px;
    background: white;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
}
QPushButton {
    min-height: 32px;
    padding: 3px 12px;
    background: #f7f8fa;
    border: 1px solid #aeb8c4;
    border-radius: 3px;
    font-weight: 600;
}
QPushButton:hover { background: #e8edf1; }
QPushButton:disabled { color: #98a4b1; background: #edf0f2; }
QPushButton[role="primary"] {
    color: white;
    background: #245f87;
    border-color: #245f87;
}
QPushButton[role="danger"] { color: white; background: #a7352a; border-color: #a7352a; }
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


class _PanelBridge(QObject):
    """把 ROS 客户端线程的请求结果投递到 Qt 主线程。"""

    completed = Signal(str, object, str)


class CorrectionPanelWindow(ShadowWindowChromeMixin, QMainWindow):
    """不加入地面站飞行会话的校准调试子面板。"""

    def __init__(
        self,
        *,
        client: CorrectionPanelClient | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.client = client or CorrectionPanelClient()
        self._bridge = _PanelBridge(self)
        self._bridge.completed.connect(self._on_completed)
        self._request_busy = False
        self._latest_status: dict[str, Any] = {}

        self.setWindowTitle(DESKTOP_APPLICATION_NAME)
        self._configure_window_chrome()
        self.setMinimumSize(860, 620)
        self.resize(1050, 720)
        self._build_ui()
        self.client.start()
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def _build_ui(self) -> None:
        """构建任务输入、质量、extnav 和 raw/corrected/final 对照区。"""
        root = QWidget()
        root.setObjectName("correctionPanelRoot")
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_window_title_bar("AprilTag-Odin 修正面板"))
        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        warning = QLabel(
            "仅修正 Odin 世界系的 x / y / yaw；本面板不会解锁、起飞或发送飞行模式命令。"
        )
        warning.setObjectName("warningLabel")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        command_group = QGroupBox("校准任务")
        command_layout = QGridLayout(command_group)
        command_layout.addWidget(QLabel("预期 Tag ID"), 0, 0)
        self.tag_id_input = QSpinBox()
        self.tag_id_input.setRange(0, 2_147_483_647)
        self.tag_id_input.setValue(0)
        command_layout.addWidget(self.tag_id_input, 0, 1)
        self.apply_checkbox = QCheckBox("收敛后提交 extnav（需要 ACK）")
        self.apply_checkbox.setChecked(False)
        command_layout.addWidget(self.apply_checkbox, 0, 2, 1, 2)
        self.start_button = QPushButton("开始")
        self.start_button.setProperty("role", "primary")
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton("停止")
        self.stop_button.setProperty("role", "danger")
        self.stop_button.clicked.connect(self._stop)
        command_layout.addWidget(self.start_button, 1, 0, 1, 2)
        command_layout.addWidget(self.stop_button, 1, 2, 1, 2)
        self.command_message = QLabel("等待 correction_service 状态")
        self.command_message.setWordWrap(True)
        command_layout.addWidget(self.command_message, 2, 0, 1, 4)
        layout.addWidget(command_group)

        status_row = QHBoxLayout()
        correction_group = QGroupBox("候选与质量")
        correction_form = QFormLayout(correction_group)
        self.correction_state = self._value_label()
        self.candidate_value = self._value_label()
        self.quality_value = self._value_label()
        self.sample_value = self._value_label()
        self.performance_value = self._value_label()
        correction_form.addRow("状态", self.correction_state)
        correction_form.addRow("候选", self.candidate_value)
        correction_form.addRow("离散/残差", self.quality_value)
        correction_form.addRow("帧/样本", self.sample_value)
        correction_form.addRow("性能/时间源", self.performance_value)
        status_row.addWidget(correction_group, 3)

        extnav_group = QGroupBox("extnav 权威状态")
        extnav_form = QFormLayout(extnav_group)
        self.extnav_state = self._value_label()
        self.extnav_value = self._value_label()
        self.session_value = self._value_label()
        self.extnav_event = QLabel("—")
        self.extnav_event.setWordWrap(True)
        extnav_form.addRow("Odin / 修正", self.extnav_state)
        extnav_form.addRow("active", self.extnav_value)
        extnav_form.addRow("session / revision", self.session_value)
        extnav_form.addRow("最近事件", self.extnav_event)
        status_row.addWidget(extnav_group, 2)
        layout.addLayout(status_row)

        compare_group = QGroupBox("实际数据链对照")
        compare_form = QFormLayout(compare_group)
        self.raw_pose = self._value_label()
        self.corrected_pose = self._value_label()
        self.final_pose = self._value_label()
        compare_form.addRow("Odin raw", self.raw_pose)
        compare_form.addRow("extnav corrected", self.corrected_pose)
        compare_form.addRow("MAVROS final", self.final_pose)
        layout.addWidget(compare_group)

        result_group = QGroupBox("最近任务终态")
        result_layout = QVBoxLayout(result_group)
        self.result_value = QLabel("尚无终态结果")
        self.result_value.setWordWrap(True)
        result_layout.addWidget(self.result_value)
        layout.addWidget(result_group)
        layout.addStretch(1)
        root_layout.addWidget(body, 1)
        self._sync_window_chrome()

    @staticmethod
    def _value_label() -> QLabel:
        """创建不会因状态刷新改变字体的等宽数值标签。"""
        label = QLabel("—")
        label.setObjectName("valueLabel")
        label.setTextInteractionFlags(label.textInteractionFlags())
        return label

    def _start(self) -> None:
        """在 apply=true 时二次确认，再异步发送 start。"""
        apply = self.apply_checkbox.isChecked()
        if apply:
            answer = QMessageBox.warning(
                self,
                "确认应用修正",
                "收敛后将原子修改 extnav 的 x/y/yaw 坐标变换。\n"
                "Tag 世界位姿必须准确；该操作不会解锁或起飞。",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        self._request_busy = True
        self.command_message.setText("正在提交 start 请求…")
        self.client.request_start(
            self.tag_id_input.value(),
            apply,
            lambda payload, error: self._bridge.completed.emit("start", payload, error),
        )
        self._refresh_controls()

    def _stop(self) -> None:
        """停止状态中报告的唯一 job，不清除 extnav active correction。"""
        job_id = str(self._latest_status.get("correction", {}).get("job_id", ""))
        self._request_busy = True
        self.command_message.setText("正在提交 stop 请求…")
        self.client.request_stop(
            job_id,
            lambda payload, error: self._bridge.completed.emit("stop", payload, error),
        )
        self._refresh_controls()

    def _on_completed(self, kind: str, payload: dict[str, Any], error: str) -> None:
        """显示 start/stop 的服务级 ACK；终态仍以 result 话题为准。"""
        self._request_busy = False
        if error:
            self.command_message.setText(f"{kind} 被拒绝/失败：{error}")
        else:
            job = f" job={payload.get('job_id')}" if payload.get("job_id") else ""
            self.command_message.setText(
                f"{kind} 已接受{job}：{payload.get('message', '')}"
            )
        self._refresh_controls()

    def _refresh(self) -> None:
        """读取纯数据快照并刷新所有门控与质量字段。"""
        status = self.client.status()
        self._latest_status = status
        correction = status.get("correction", {})
        extnav = status.get("extnav", {})
        startup_error = str(status.get("startup_error", ""))
        correction_fresh = bool(correction.get("fresh"))
        extnav_fresh = bool(extnav.get("fresh"))

        if startup_error:
            self.command_message.setText(startup_error)
        if correction_fresh:
            self.correction_state.setText(
                f"{correction.get('state', '—')} · {correction.get('message', '')}"
            )
            self.candidate_value.setText(
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
            self.sample_value.setText(
                f"recv={correction.get('frames_received', 0)}  "
                f"proc={correction.get('frames_processed', 0)}  "
                f"tag={correction.get('detections', 0)}  "
                f"accept/reject={correction.get('samples', 0)}/"
                f"{correction.get('rejected', 0)}"
            )
            self.performance_value.setText(
                f"{correction.get('processing_rate_hz', 0):.2f} Hz / "
                f"{correction.get('processing_time_ms', 0):.1f} ms · "
                f"{correction.get('odom_time_source', '—')}"
            )
        else:
            self.correction_state.setText("correction_service 状态不可用/过期")

        if extnav_fresh:
            self.extnav_state.setText(
                f"Odin={'可用' if extnav.get('odin_available') else '不可用'} · "
                f"correction={'有效' if extnav.get('valid') else 'identity'}"
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
            self.extnav_event.setText(str(extnav.get("last_event", "—")))
        else:
            self.extnav_state.setText("extnav correction 状态不可用/过期")

        self.raw_pose.setText(self._format_pose(status.get("raw", {})))
        self.corrected_pose.setText(self._format_pose(status.get("corrected", {})))
        self.final_pose.setText(self._format_pose(status.get("final", {})))
        result = status.get("result", {})
        if result.get("fresh"):
            self.result_value.setText(
                f"job={result.get('job_id', '—')} · "
                f"success={result.get('success')} applied={result.get('applied')} · "
                f"{result.get('outcome', '')} · {result.get('message', '')}\n"
                f"x={result.get('x_m', 0):+.4f} m, "
                f"y={result.get('y_m', 0):+.4f} m, "
                f"yaw={result.get('yaw_deg', 0):+.3f}°, "
                f"samples={result.get('samples', 0)}, "
                f"duration={result.get('duration_s', 0):.1f}s\n"
                f"log={result.get('log_path', '')}"
            )
        self._refresh_controls()

    @staticmethod
    def _format_pose(snapshot: dict[str, Any]) -> str:
        """格式化 raw/corrected/final；过期数据不伪装成实时状态。"""
        if not snapshot.get("fresh"):
            return "不可用/过期"
        return (
            f"x={snapshot.get('x_m', 0):+.4f} m  "
            f"y={snapshot.get('y_m', 0):+.4f} m  "
            f"z={snapshot.get('z_m', 0):+.4f} m  "
            f"yaw={snapshot.get('yaw_deg', 0):+.3f}°"
        )

    def _refresh_controls(self) -> None:
        """只在服务新鲜且无任务时允许 start，活动时才允许 stop。"""
        correction = self._latest_status.get("correction", {})
        extnav = self._latest_status.get("extnav", {})
        available = bool(
            correction.get("fresh") and correction.get("service_available")
        )
        active = bool(correction.get("active"))
        apply_ready = bool(
            extnav.get("fresh")
            and extnav.get("service_available")
            and extnav.get("odin_available")
        )
        self.start_button.setEnabled(
            available
            and not active
            and not self._request_busy
            and (not self.apply_checkbox.isChecked() or apply_ready)
        )
        self.stop_button.setEnabled(available and active and not self._request_busy)
        self.tag_id_input.setEnabled(not active and not self._request_busy)
        self.apply_checkbox.setEnabled(not active and not self._request_busy)

    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        """关闭本地面板连接，但不把 UI 生命周期当成任务 stop 命令。"""
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
