"""上位机 WebSocket 配置、映射、原始报文与 JSON 说明面板。"""

from __future__ import annotations

import json

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QTextCursor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..upstream.mapping import UpstreamProtocolError
from ..upstream.protocol import command_topic, json_examples, status_topic
from ..upstream.service import UpstreamCommunicationService
from .window_chrome import ShadowWindowChromeMixin


class UpstreamCommunicationPanel(ShadowWindowChromeMixin, QWidget):
    """独立管理上位机通讯服务；关闭窗口不会关闭连接或地面站。"""

    def __init__(
        self,
        service: UpstreamCommunicationService,
    ) -> None:
        """从服务快照构建可最小化、无主窗口置顶关系的顶层面板。"""
        # QDialog(parent) 会成为主窗口的 transient child：窗口管理器通常强制
        # 它位于主窗口上方，而且默认不提供有效的最小化窗口提示。此处使用
        # 无父级的普通顶层窗口，让系统原生管理层级与最小化行为。
        super().__init__(None, Qt.WindowType.Window)
        self._service = service
        self._last_raw_sequence = 0
        self.setWindowTitle("上位机通讯面板")
        self.setObjectName("upstreamCommunicationPanel")
        self._configure_window_chrome()
        self.resize(980, 720)
        self.setMinimumSize(760, 560)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_window_title_bar(self.windowTitle()))

        body = QWidget()
        body.setObjectName("subpanelWindowBody")
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(14, 12, 14, 14)
        body_layout.setSpacing(10)
        self.tabs = QTabWidget()
        self.tabs.setObjectName("upstreamCommunicationTabs")
        self.tabs.addTab(self._build_connection_tab(), "连接配置")
        self.tabs.addTab(self._build_mapping_tab(), "指令映射")
        self.tabs.addTab(self._build_raw_log_tab(), "原始报文")
        self.tabs.addTab(self._build_json_tab(), "JSON 格式")
        body_layout.addWidget(self.tabs, 1)
        root.addWidget(body, 1)

        self._render_json_examples()
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._poll)
        self._timer.start()
        self._poll()
        self._sync_window_chrome()

    def _build_connection_tab(self) -> QWidget:
        """创建 URL、无人机编号、主题和独立生命周期操作。"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        form = QFormLayout()
        snapshot = self._service.snapshot()
        self.url_input = QLineEdit(snapshot.url)
        self.url_input.setObjectName("upstreamUrlInput")
        self.url_input.setPlaceholderText("ws://127.0.0.1:8581/ws")
        self.client_no_input = QLineEdit(snapshot.client_no)
        self.client_no_input.setObjectName("upstreamClientNoInput")
        self.client_no_input.setPlaceholderText("UAV01001")
        self.connection_state = QLabel("--")
        self.connection_state.setObjectName("upstreamConnectionState")
        self.connection_detail = QLabel("--")
        self.connection_detail.setWordWrap(True)
        self.command_topic_label = QLabel("--")
        self.command_topic_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.status_topic_label = QLabel("--")
        self.status_topic_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        form.addRow("WebSocket URL", self.url_input)
        form.addRow("无人机编号", self.client_no_input)
        form.addRow("连接状态", self.connection_state)
        form.addRow("状态说明", self.connection_detail)
        form.addRow("控制主题", self.command_topic_label)
        form.addRow("状态主题", self.status_topic_label)
        layout.addLayout(form)

        buttons = QHBoxLayout()
        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("upstreamConnectButton")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setObjectName("upstreamDisconnectButton")
        self.restart_button = QPushButton("重启连接")
        self.restart_button.setObjectName("upstreamRestartButton")
        self.connect_button.clicked.connect(self._connect)
        self.disconnect_button.clicked.connect(self._service.disconnect)
        self.restart_button.clicked.connect(self._restart)
        buttons.addWidget(self.connect_button)
        buttons.addWidget(self.disconnect_button)
        buttons.addWidget(self.restart_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)

        note = QLabel(
            "此连接与仿真/实机会话相互独立。没有活动飞行环境时，命令会确认接收，"
            "但地面站会明确拒绝执行。关闭本面板不会断开连接。"
        )
        note.setObjectName("mutedLabel")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch(1)
        self.url_input.textChanged.connect(self._render_json_examples)
        self.client_no_input.textChanged.connect(self._render_json_examples)
        return tab

    def _build_mapping_tab(self) -> QWidget:
        """直接展示独立映射文件，避免 UI 复制另一份业务关系。"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.mapping_table = QTableWidget(0, 4)
        self.mapping_table.setObjectName("upstreamCommandMappingTable")
        self.mapping_table.setHorizontalHeaderLabels(
            ("命令编号", "上位机命令", "本地动作", "实际效果")
        )
        self.mapping_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.mapping_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        mappings = self._service.command_mappings
        self.mapping_table.setRowCount(len(mappings))
        for row, mapping in enumerate(mappings):
            for column, text in enumerate(
                (
                    mapping.command_no,
                    mapping.label,
                    mapping.action.value,
                    mapping.local_effect,
                )
            ):
                self.mapping_table.setItem(row, column, QTableWidgetItem(text))
        header = self.mapping_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.mapping_table)
        note = QLabel(
            "forwardAngle 按角度制偏航角转换；cameraAngle 与 photoNo 当前仅校验并忽略。"
        )
        note.setObjectName("mutedLabel")
        note.setWordWrap(True)
        layout.addWidget(note)
        return tab

    def _build_raw_log_tab(self) -> QWidget:
        """创建与地面站维护日志完全分离的原始帧查看器。"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        controls = QHBoxLayout()
        label = QLabel("RX/TX WebSocket 原始文本帧")
        label.setObjectName("mutedLabel")
        self.raw_search_input = QLineEdit()
        self.raw_search_input.setObjectName("upstreamRawFrameSearch")
        self.raw_search_input.setPlaceholderText("搜索原始报文")
        self.raw_search_button = QPushButton("查找下一个")
        self.raw_search_button.setObjectName("searchUpstreamRawFramesButton")
        self.raw_search_button.clicked.connect(self._find_next_raw_frame)
        self.raw_search_input.returnPressed.connect(self._find_next_raw_frame)
        self.clear_raw_button = QPushButton("清空原始报文")
        self.clear_raw_button.setObjectName("clearUpstreamRawFramesButton")
        self.clear_raw_button.clicked.connect(self._clear_raw_frames)
        controls.addWidget(label)
        controls.addStretch(1)
        controls.addWidget(self.raw_search_input)
        controls.addWidget(self.raw_search_button)
        controls.addWidget(self.clear_raw_button)
        layout.addLayout(controls)
        self.raw_log = QPlainTextEdit()
        self.raw_log.setObjectName("upstreamRawFrameLog")
        self.raw_log.setReadOnly(True)
        self.raw_log.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.raw_log.document().setMaximumBlockCount(4000)
        layout.addWidget(self.raw_log, 1)
        return tab

    def _build_json_tab(self) -> QWidget:
        """展示主题、单位、阈值、未实现项和可复制 JSON。"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.json_text = QPlainTextEdit()
        self.json_text.setObjectName("upstreamJsonExamples")
        self.json_text.setReadOnly(True)
        self.json_text.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self.json_text)
        return tab

    def _connect(self) -> None:
        """校验当前输入并请求服务连接，不阻塞 Qt 事件循环。"""
        try:
            self._service.connect(
                self.url_input.text().strip(), self.client_no_input.text().strip()
            )
        except (TypeError, ValueError, UpstreamProtocolError) as exc:
            self.connection_state.setText("配置无效")
            self.connection_detail.setText(str(exc))

    def _restart(self) -> None:
        """校验当前输入并重建 WebSocket 会话。"""
        try:
            self._service.restart(
                self.url_input.text().strip(), self.client_no_input.text().strip()
            )
        except (TypeError, ValueError, UpstreamProtocolError) as exc:
            self.connection_state.setText("配置无效")
            self.connection_detail.setText(str(exc))

    def _poll(self) -> None:
        """增量刷新连接状态和原始帧，不跨线程共享 Qt 控件。"""
        snapshot = self._service.snapshot()
        self.connection_state.setText(snapshot.state)
        self.connection_detail.setText(snapshot.detail)
        self.disconnect_button.setEnabled(
            snapshot.desired_connected or snapshot.connected
        )
        for frame in self._service.journal.frames_after(self._last_raw_sequence):
            self._last_raw_sequence = max(self._last_raw_sequence, frame.sequence)
            timestamp = frame.timestamp.strftime("%H:%M:%S.%f")[:-3]
            self.raw_log.appendPlainText(
                f"[{timestamp}] {frame.direction} {frame.payload}"
            )

    def _clear_raw_frames(self) -> None:
        """清空专用缓冲和当前文本，不影响连接。"""
        self._service.journal.clear()
        self.raw_log.clear()

    def _find_next_raw_frame(self) -> None:
        """在当前原始报文文本内循环查找下一个简单关键词。"""
        query = self.raw_search_input.text()
        if not query:
            return
        if self.raw_log.find(query):
            return
        cursor = self.raw_log.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        self.raw_log.setTextCursor(cursor)
        self.raw_log.find(query)

    def _render_json_examples(self) -> None:
        """随无人机编号更新主题和完整 JSON 示例。"""
        client_no = self.client_no_input.text().strip()
        try:
            control_topic = command_topic(client_no)
            report_topic = status_topic(client_no)
            examples = json_examples(client_no)
        except UpstreamProtocolError:
            self.command_topic_label.setText("--")
            self.status_topic_label.setText("--")
            self.json_text.setPlainText("请输入合法的非空无人机编号。")
            return
        self.command_topic_label.setText(control_topic)
        self.status_topic_label.setText(report_topic)
        sections = [
            "配置说明",
            f"控制主题: {control_topic}",
            f"状态主题: {report_topic}",
            "0A: 仿真发送 0~100 百分比；实机发送电压 V",
            "03: 起飞至设定高度 → 巡检航点 → 末点降落",
            "05: 当前控制组合返回原点起飞高度 → 稳定降落",
            "08: 仅巡检航点全部完成时发送；返航航点不发送",
            (
                "0C: WebSocket 在线时上报低电量；在线仿真触发返航降落，"
                "在线实机当前只提示地面站（TODO），断线不触发动作"
            ),
            (
                "01: 可起飞或组合降落后，仅在 "
                f"|X|<{self._service.standby_policy.x_tolerance_meters:.2f} m、"
                f"|Y|<{self._service.standby_policy.y_tolerance_meters:.2f} m、"
                f"|Z|<{self._service.standby_policy.z_tolerance_meters:.2f} m 时发送"
            ),
            (
                "巡检降落后 01 延时: "
                f"{self._service.standby_policy.inspection_delay_seconds:.1f} s"
            ),
            (
                "无人机异常: WebSocket 在线仿真执行返航降落；在线实机当前只提示"
                "地面站（TODO），断线不触发动作"
            ),
            "相机/云台/照片/FTP/RTSP/媒体路径: 当前未实现",
        ]
        for title, payload in examples:
            sections.extend(
                ("", title, json.dumps(payload, ensure_ascii=False, indent=2))
            )
        self.json_text.setPlainText("\n".join(sections))

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        """关闭面板时保存最后有效输入，不断开现有会话。"""
        try:
            self._service.save_configuration(
                self.url_input.text().strip(),
                self.client_no_input.text().strip(),
            )
        except (TypeError, ValueError, UpstreamProtocolError):
            pass
        super().closeEvent(event)
