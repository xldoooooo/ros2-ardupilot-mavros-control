# 任务 12 追加修正简报：坐标系日志与完整下拉菜单

日期：2026-08-09

## 修正结果

按用户追加要求完成两项小修，没有改变坐标转换、灵敏度数值、控制协议或机载逻辑。

1. 手动坐标系从“机体坐标”切换到“本地 ENU”或反向切换时，主窗口会写入 `INFO / operator` 结构化日志。
2. 日志同时说明所选模式的实际输入语义：
   - 机体坐标：右摇杆增量按最新机头航向旋转到本地 ENU；
   - 本地 ENU：右摇杆增量沿固定 X/Y 轴发送。
3. 坐标系选择器复用航点策略已经验证过的 `DownwardComboBox`，以完整 `QMenu` 固定显示在控件下方，不再使用可能被桌面样式裁切的原生 combo popup。
4. 同批新增的左右摇杆灵敏度选择器也统一复用相同组件，三项低/中/高一次完整显示，避免保留同类裁切隐患。

## 修改文件

- `ground_station_core/qt_ui/operations_panel.py`
  - 新增坐标系变化语义信号；
  - 坐标系和两个灵敏度选择器改用 `DownwardComboBox`。
- `ground_station_core/qt_ui/main_window.py`
  - 接收坐标系变化信号并写入结构化日志。
- `tests/test_qt_gui.py`
  - 验证三个新增下拉菜单的条目数量、文字、可见高度、最后一项边界和向下位置；
  - 验证 ENU 切换日志的等级、来源和语义文字。
- `MEMORY.md`
  - 记录追加修正基线。

## 验证结果

```text
追加修正定向回归：
2 passed, 22 deselected in 2.62s

加载 Jazzy 与本仓库 overlay 后 Python 全量：
46 passed in 105.64s

compileall：通过
flake8 E9/F63/F7/F82：通过
git diff --check（本次代码/测试文件）：通过
```

视觉证据 `agent/codex/task12-coordinate-menu.png` 显示“机体坐标”和“本地 ENU”两项完整出现；菜单尺寸为 130×74 px，最后一项底部为 70 px，位于菜单可见区域内。

## 安全与范围说明

- 未启动 SITL、MAVROS、机载节点或 RViz。
- 未连接实机，未申请控制租约或发送飞行命令。
- 未执行解锁、起飞、降落或航点。
- 未加入任务外的锁定、ARM/DISARM、RTL、自动回中或其他功能。
