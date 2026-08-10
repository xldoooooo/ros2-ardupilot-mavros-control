# 航点执行与进度区压缩执行报告

## 1. 任务范围

按要求删除右下角“发送并执行航点”上方的灰色说明行，将释放的高度
全部让给上方航点表格。本次不修改 ROS 协议、机载 C++、航点数据、发送门控、
飞行策略、进度跟踪、结果处理或任何飞行逻辑。

本次没有启动 SITL，没有连接实机，没有申请控制租约，也没有解锁或起飞。

## 2. 实施方式

- 从 `Card.content_layout` 中移除灰色 `status_label`，不再分配布局高度或参与绘制。
- 保留该对象为隐藏的状态接收器；`_add_waypoint()`、`clear_waypoints()` 和 `set_result()`
  仍按原路径更新它，从而避免改变现有内部接口和命令结果链。
- 进度条、“发送并执行航点”按钮和策略选择器的尺寸、顺序、样式与相对位置不变。

## 3. 布局与视觉验证

| 窗口尺寸 | 执行卡调整前 | 执行卡调整后 | 表格调整前 | 表格调整后 |
| --- | ---: | ---: | ---: | ---: |
| 1228×924 | 148 px | 118 px | 331 px | 361 px |
| 1600×920 | 148 px | 118 px | 327 px | 357 px |
| 1180×700 | 148 px | 118 px | 210 px | 240 px |

三种尺寸下执行卡均减少 30 px，表格均增加 30 px，没有改变其他组件尺寸。
离屏视觉截图为 `agent/codex/waypoint-execution-compact.png`，人工检查确认说明行已消失，
进度条、按钮、策略下拉框、卡片边界和上方表格均正常。

## 4. 回归结果

```text
航点定向回归：3 passed
Python 全量回归：48 passed in 14.20s
compileall：通过
flake8 E9/F63/F7/F82/E501（max-line-length=88）：通过
ground_station.py --check-environment：通过
colcon build（guided_interfaces/onboard_control/guided_sim）：成功
colcon test-result：5 tests, 0 errors, 0 failures, 0 skipped
```

自动回归另外验证：隐藏状态对象不在执行卡布局内、不可见，但仍能接收
`set_result()` 文本；旧任务进度隔离与运行中进度不重置回归继续通过。

## 5. 过程说明

首次误用 `colcon build --symlink-install`，与现有普通构建缓存冲突。仅将
`build/guided_interfaces` 移动到可恢复的临时备份
`/tmp/codex-guided-build.ORZuXF/guided_interfaces`，然后按普通模式重建，三包全部成功。
没有删除源码或用户文件。

## 6. 本次追加文件

- `ground_station_core/qt_ui/waypoint_panel.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`
- `agent/report/report-2026-08-09-waypoint-execution-panel-compression.md`
- `agent/codex/waypoint-execution-compact.png`
