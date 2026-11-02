"""标记校准面板的远端 Tag 配置表格，显式保存后才写入飞机 CSV。"""

from __future__ import annotations

import csv
import io
from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QPushButton, QStyledItemDelegate, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from .config import TAG_COLUMNS, format_tag_csv, parse_tag_csv


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
        return QLineEdit(parent)


class TagConfigEditor(QWidget):
    """维护未保存草稿，并异步读写当前 correction_service 的配置。"""

    completed = Signal(object, str)

    def __init__(self, client: Any, parent: QWidget | None = None) -> None:
        """构建六列表格与新增、删除、重新读取、保存操作。"""
        super().__init__(parent)
        self.client = client
        self._status: dict[str, Any] = {}
        self._loaded_instance = ""
        self._requested_instance = ""
        self._busy = False
        self._dirty = False
        self._saving = False
        self.completed.connect(self._on_completed)
        layout = QVBoxLayout(self)
        hint = QLabel("编辑当前飞机的 Tag 世界坐标；位置/边长单位 m，航向单位 °。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.path_label = QLabel("等待修正服务，连接后自动读取 tag_pose.csv")
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.path_label)
        self.table = QTableWidget(0, len(TAG_COLUMNS))
        self.table.setHorizontalHeaderLabels(TAG_COLUMNS)
        self.table.setItemDelegate(_TextDelegate(self.table))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setMinimumSectionSize(48)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setFixedHeight(230)
        self.table.setAlternatingRowColors(True)
        self.table.itemChanged.connect(self._on_item_changed)
        self.table.setSortingEnabled(True)
        self.table.sortItems(0, Qt.SortOrder.AscendingOrder)
        self.table.horizontalHeader().setSectionsClickable(False)
        layout.addWidget(self.table)
        actions = QHBoxLayout()
        self.add_button = QPushButton("新增 Tag")
        self.add_button.clicked.connect(self._add)
        self.delete_button = QPushButton("删除选中")
        self.delete_button.clicked.connect(self._delete)
        self.reload_button = QPushButton("重新读取")
        self.reload_button.clicked.connect(self._load)
        self.save_button = QPushButton("保存并应用")
        self.save_button.setProperty("role", "primary")
        self.save_button.clicked.connect(self._save)
        for button in (self.add_button, self.delete_button, self.reload_button, self.save_button):
            actions.addWidget(button)
        actions.addStretch()
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
        can_save = (ready and instance == self._loaded_instance
                    and not status.get("active") and status.get("application_state") != "unknown")
        self.save_button.setEnabled(can_save and not self._busy)
        self.reload_button.setEnabled(ready and not self._busy)
        for widget in (self.table, self.add_button, self.delete_button):
            widget.setEnabled(not self._busy)
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
            self.message.setText(f"{'保存' if self._saving else '读取'}失败：{error}")
        else:
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
                self.table.setItem(row, column, item)
        self.table.setSortingEnabled(True)
        self.table.horizontalHeader().setSectionsClickable(False)
        self.table.sortItems(0, Qt.SortOrder.AscendingOrder)
        self.table.blockSignals(False)

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
            self.message.setText(f"未保存：{exc}")
            return
        self._request(save=True, text=text)

    def _add(self) -> None:
        """新增最小未占用 ID，默认边长沿用首行或使用 0.1 m。"""
        used = set()
        for row in range(self.table.rowCount()):
            try:
                used.add(int(self.table.item(row, 0).text()))
            except ValueError:
                pass
        tag_id = 0
        while tag_id in used:
            tag_id += 1
        size = self.table.item(0, 5).text() if self.table.rowCount() else "0.1"
        text = self._csv() + f"{tag_id},0,0,0,0,{size}\n"
        # 允许其他行暂存非法输入，新增操作不强制校验整个草稿。
        self._populate(text)
        self._dirty = True
        self.message.setText("已新增 Tag，尚未保存。")

    def _delete(self) -> None:
        """仅删除明确选中的行，空表会在保存时提示至少保留一个 Tag。"""
        rows = sorted({index.row() for index in self.table.selectionModel().selectedRows()}, reverse=True)
        for row in rows:
            self.table.removeRow(row)
        if rows:
            self._dirty = True
            self.message.setText("已删除选中 Tag，尚未保存。")
