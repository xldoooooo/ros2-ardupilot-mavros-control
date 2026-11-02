"""标记校准面板的远端 Tag 配置表格，显式保存后才写入飞机 CSV。"""

from __future__ import annotations

import csv
import io
from pathlib import Path
from typing import Any

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QStyledItemDelegate, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ground_station_core.qt_ui import theme

from .config import TAG_COLUMNS, format_tag_csv, parse_tag_csv

# 与主GUI航点表格使用相同的行高和紧凑图标尺寸。
ROW_HEIGHT = 28


class _TagIdItem(QTableWidgetItem):
    """以整数顺序比较 ID，避免 Qt 将编辑后的数字当字符串排序。"""

    def __lt__(self, other: QTableWidgetItem) -> bool:
        """非法草稿排在末尾，交由保存校验提示。"""
        try:
            left = (0, int(self.text()))
        except ValueError:
            left = (1, self.text())
        try:
            right = (0, int(other.text()))
        except ValueError:
            right = (1, other.text())
        return left < right


class _TextDelegate(QStyledItemDelegate):
    """保留自由数值输入，由保存操作统一给出带行号的校验错误。"""

    def createEditor(self, parent, option, index):
        """避免默认整数编辑器限制可用 Tag ID。"""
        editor = QLineEdit(parent)
        editor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return editor


class TagConfigEditor(QWidget):
    """维护未保存草稿，并异步读写当前 correction_service 的配置。"""

    completed = Signal(object, str)

    def __init__(self, client: Any, parent: QWidget | None = None) -> None:
        """构建六列表格及新增、移动、末行删除、清空和读写操作。"""
        super().__init__(parent)
        self.client = client
        self._status: dict[str, Any] = {}
        self._loaded_instance = ""
        self._requested_instance = ""
        self._busy = False
        self._dirty = False
        self._saving = False
        self._error = ""
        self.completed.connect(self._on_completed)
        layout = QVBoxLayout(self)
        hint = QLabel("编辑当前飞机的 Tag 世界坐标；位置/边长单位 m，航向单位 °。")
        hint.setWordWrap(True)
        hint_row = QHBoxLayout()
        hint_row.addWidget(hint, 1)
        layout.addLayout(hint_row)
        self.connection_notice = QLabel()
        self.connection_notice.setWordWrap(True)
        self.connection_notice.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.connection_notice)
        self.path_label = QLabel("等待修正服务，连接后自动读取 tag_pose.csv")
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.path_label)
        self.table = QTableWidget(0, len(TAG_COLUMNS))
        self.table.setObjectName("tagConfigTable")
        self.table.setHorizontalHeaderLabels(("tag_id", "X / m", "Y / m", "Z / m", "Yaw / °", "Size / m"))
        self.table.setItemDelegate(_TextDelegate(self.table))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setMinimumSectionSize(48)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setFixedHeight(ROW_HEIGHT)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setMinimumSectionSize(ROW_HEIGHT)
        self.table.verticalHeader().setDefaultSectionSize(ROW_HEIGHT)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked | QAbstractItemView.EditTrigger.EditKeyPressed
        )
        self.table.setToolTip("单击选中整行，双击编辑；上下移动仅交换参数，Tag ID 保持升序。")
        self.table.setFixedHeight(230)
        self.table.setAlternatingRowColors(True)
        self.table.itemChanged.connect(self._on_item_changed)
        self.table.setSortingEnabled(True)
        self.table.sortItems(0, Qt.SortOrder.AscendingOrder)
        self.table.horizontalHeader().setSectionsClickable(False)
        layout.addWidget(self.table)
        actions = QHBoxLayout()
        actions.setSpacing(2)
        assets = Path(theme.__file__).with_name("assets")
        self.up_button = self._icon_button(assets / "chevron-up.svg", "上移选中 Tag 的配置参数（ID 保持不变）")
        self.up_button.clicked.connect(lambda: self._move_selected(-1))
        self.down_button = self._icon_button(assets / "chevron-down.svg", "下移选中 Tag 的配置参数（ID 保持不变）")
        self.down_button.clicked.connect(lambda: self._move_selected(1))
        self.delete_button = self._icon_button(assets / "minus-red.svg", "删除最下方一行 Tag")
        self.delete_button.clicked.connect(self._delete)
        self.clear_button = QPushButton("清空")
        self.clear_button.setProperty("role", "danger")
        self.clear_button.setToolTip("清空全部 Tag 草稿，保存前需至少新增一个 Tag")
        self.clear_button.clicked.connect(self._clear)
        for button in (self.up_button, self.down_button, self.delete_button, self.clear_button):
            actions.addWidget(button)
        actions.addStretch(1)
        self.add_button = QPushButton("新增 Tag")
        self.add_button.setToolTip("在列表末尾追加一个 Tag")
        self.add_button.clicked.connect(self._add)
        hint_row.addWidget(self.add_button)
        self.reload_button = QPushButton("重新读取")
        self.reload_button.clicked.connect(self._load)
        self.save_button = QPushButton("保存并应用")
        self.save_button.setProperty("role", "primary")
        self.save_button.clicked.connect(self._save)
        for button in (self.reload_button, self.save_button):
            actions.addWidget(button)
        for button in (self.add_button, self.up_button, self.down_button,
                       self.delete_button, self.clear_button, self.reload_button, self.save_button):
            button.setProperty("compact", True)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.table.itemSelectionChanged.connect(self._update_row_controls)
        self.table.currentCellChanged.connect(self._update_row_controls)
        layout.addLayout(actions)
        self.message = QLabel("未保存的编辑不会影响校准。保存变更将清空旧滑窗和候选，保留当前生效修正。")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        layout.addStretch(1)
        self.set_service_status({})

    def set_service_status(self, status: dict[str, Any]) -> None:
        """每个服务实例首次可用时读取一次，刷新状态不覆盖编辑草稿。"""
        self._status = status
        instance = str(status.get("service_instance_id", ""))
        ready = bool(status.get("fresh") and status.get("service_available") and instance)
        if status.get("client_error"):
            notice = f"修正客户端启动失败：{status['client_error']}"
        elif not ready:
            if status.get("aircraft_available"):
                notice = "已收到飞机数据，但 correction 服务不可用或通信中断。"
            else:
                notice = "未收到飞机数据或 correction 服务状态，可能尚未连接飞机或修正服务未启动。"
            notice += "请检查飞机网络及 odin-correction.service；当前无法读取或保存配置，空表不表示 CSV 文件为空。"
        elif self._error:
            notice = self._error
        elif status.get("active"):
            notice = "飞机 correction 服务已连接；校准任务运行中，暂不能保存配置。"
        elif status.get("application_state") == "unknown":
            notice = "飞机 correction 服务已连接；修正应用结果尚未确认，暂不能保存配置。"
        elif self._loaded_instance == instance and self.table.rowCount() == 0:
            notice = "Tag 列表为空；请新增 Tag 后保存。"
        else:
            notice = (
                "飞机 correction 服务已连接。" if self._loaded_instance == instance
                else "飞机 correction 服务已连接，等待读取 Tag 配置。"
            )
        self.connection_notice.setText(notice)
        warning = (
            not ready or bool(status.get("client_error")) or bool(self._error)
            or bool(status.get("active")) or status.get("application_state") == "unknown"
            or (self._loaded_instance == instance and self.table.rowCount() == 0)
        )
        self.connection_notice.setStyleSheet(
            f"color: {'#8b4a18' if warning else '#245f87'}; font-weight: 600;"
        )
        can_save = (ready and instance == self._loaded_instance
                    and self.table.rowCount() > 0
                    and not status.get("active") and status.get("application_state") != "unknown")
        self.save_button.setEnabled(can_save and not self._busy)
        self.reload_button.setEnabled(ready and not self._busy)
        for widget in (self.table, self.add_button):
            widget.setEnabled(not self._busy)
        self._update_row_controls()
        if ready and instance != self._requested_instance and not self._busy:
            self._requested_instance = instance
            if self._dirty:
                self.message.setText("服务实例已改变；草稿已保留，请重新读取当前飞机配置后再保存。")
            else:
                self._load()

    def _load(self) -> None:
        """显式重新读取会替换草稿；状态轮询本身不改表格。"""
        self._request(save=False)

    def _request(self, *, save: bool, text: str = "") -> None:
        """将文件操作交给 ROS 线程，Qt 主线程只维护编辑状态。"""
        self._busy = True
        self._saving = save
        self._error = ""
        self.message.setText("正在保存并应用…" if save else "正在读取飞机 tag_pose.csv…")
        self.set_service_status(self._status)
        self.client.request_tags(
            save=save, csv=text,
            service_instance_id=self._loaded_instance,
            window_revision=int(self._status.get("window_revision", 0)),
            callback=lambda payload, error: self.completed.emit(payload, error),
        )

    def _on_completed(self, payload: dict[str, Any], error: str) -> None:
        """只在 ACK 成功后用服务端回读替换草稿并显示真实文件路径。"""
        self._busy = False
        if not error:
            try:
                tags = parse_tag_csv(str(payload.get("csv", "")))
            except ValueError as exc:
                error = str(exc)
        if error:
            self._error = f"Tag 配置{'保存' if self._saving else '读取'}失败：{error}"
            self.message.setText(self._error)
        else:
            self._error = ""
            self._populate(format_tag_csv(tags))
            self._loaded_instance = str(payload.get("service_instance_id", ""))
            self.path_label.setText(str(payload.get("config_path", "")))
            self.message.setText(str(payload.get("message", "")))
            self._dirty = False
        self.set_service_status(self._status)

    def _populate(self, text: str) -> None:
        """将已验证的 CSV 按数值 ID 排序显示，保留浮点精度。"""
        rows = list(csv.reader(io.StringIO(text)))[1:]
        self.table.blockSignals(True)
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                item = _TagIdItem(value) if column == 0 else QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(row, column, item)
        self.table.setSortingEnabled(True)
        self.table.horizontalHeader().setSectionsClickable(False)
        self.table.sortItems(0, Qt.SortOrder.AscendingOrder)
        self.table.blockSignals(False)
        self._update_row_controls()

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        """编辑结束后按数值 ID 重排，非法输入保留到保存时提示。"""
        if item.column() == 0:
            self.table.sortItems(0, Qt.SortOrder.AscendingOrder)
        self._dirty = True
        self.message.setText("有未保存的修改；点击“保存并应用”后生效。")

    def _csv(self) -> str:
        """收集所有行，交由与服务端相同的解析器检查。"""
        stream = io.StringIO(newline="")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(TAG_COLUMNS)
        for row in range(self.table.rowCount()):
            writer.writerow([self.table.item(row, column).text()
                             if self.table.item(row, column) else ""
                             for column in range(len(TAG_COLUMNS))])
        return stream.getvalue()

    def _save(self) -> None:
        """拒绝重复 ID、非有限坐标及非正尺寸，错误不丢失草稿。"""
        # 先提交仍处于编辑状态的单元格，再读取其模型值。
        self.save_button.setFocus()
        try:
            text = format_tag_csv(parse_tag_csv(self._csv()))
        except ValueError as exc:
            self._error = f"未保存：{exc}"
            self.message.setText(self._error)
            self.set_service_status(self._status)
            return
        self._request(save=True, text=text)

    @staticmethod
    def _icon_button(path: Path, tooltip: str) -> QPushButton:
        """使用主GUI相同的SVG、尺寸及反馈样式创建紧凑表格按钮。"""
        button = QPushButton()
        button.setProperty("tableIcon", True)
        button.setIcon(QIcon(str(path)))
        button.setIconSize(QSize(16, 16))
        button.setFixedSize(32, ROW_HEIGHT)
        button.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        button.setToolTip(tooltip)
        button.setAccessibleName(tooltip)
        return button

    def _update_row_controls(self, *_args: Any) -> None:
        """按选中行、列表数量及请求状态启用移动、末行删除和清空。"""
        row = self.table.currentRow()
        count = self.table.rowCount()
        selected = bool(self.table.selectionModel().selectedRows())
        self.up_button.setEnabled(not self._busy and selected and row > 0)
        self.down_button.setEnabled(not self._busy and selected and 0 <= row < count - 1)
        self.delete_button.setEnabled(not self._busy and count > 0)
        self.clear_button.setEnabled(not self._busy and count > 0)

    def _move_selected(self, offset: int) -> None:
        """仅交换两行的五项配置参数，ID作为行编号留在原位。"""
        self.table.setFocus()
        row = self.table.currentRow()
        target = row + offset
        if row < 0 or not 0 <= target < self.table.rowCount():
            return
        self.table.blockSignals(True)
        for column in range(1, len(TAG_COLUMNS)):
            current_item = self.table.takeItem(row, column)
            target_item = self.table.takeItem(target, column)
            self.table.setItem(row, column, target_item)
            self.table.setItem(target, column, current_item)
        self.table.blockSignals(False)
        self.table.setCurrentCell(target, 1)
        self.table.selectRow(target)
        self._dirty = True
        self.message.setText("Tag 参数已移动，ID 保持升序；尚未保存。")
        self._update_row_controls()

    def _add(self) -> None:
        """在最大ID后追加一行，默认边长沿用首行或使用0.1m。"""
        self.table.setFocus()
        used = []
        for row in range(self.table.rowCount()):
            try:
                used.append(int(self.table.item(row, 0).text()))
            except ValueError:
                pass
        tag_id = max(used, default=-1) + 1
        size = self.table.item(0, 5).text() if self.table.rowCount() else "0.1"
        text = self._csv() + f"{tag_id},0,0,0,0,{size}\n"
        # 允许其他行暂存非法输入，新增操作不强制校验整个草稿。
        self._populate(text)
        self.table.setCurrentCell(self.table.rowCount() - 1, 1)
        self.table.selectRow(self.table.rowCount() - 1)
        self._dirty = True
        self.message.setText("已新增 Tag，尚未保存。")
        self.set_service_status(self._status)

    def _delete(self) -> None:
        """减号始终删除最下方一行，与当前选择无关。"""
        self.table.setFocus()
        if self.table.rowCount():
            self.table.removeRow(self.table.rowCount() - 1)
            self._dirty = True
            self.message.setText(
                "已删除末行 Tag，尚未保存。" if self.table.rowCount()
                else "已删除末行 Tag；请新增 Tag 后保存。"
            )
        self.set_service_status(self._status)

    def _clear(self) -> None:
        """清空全部本地草稿，不立即写入飞机配置。"""
        self.table.setFocus()
        self.table.setRowCount(0)
        self._dirty = True
        self.message.setText("Tag 草稿已清空；请新增 Tag 后保存。")
        self.set_service_status(self._status)
