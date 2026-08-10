# GUI 窗口性能全面优化执行报告

## 1. 目标与约束

本次只优化 PySide6/Qt 地面站窗口的绘制和稳定刷新开销，严格保持：

- 所有 ROS、环境、航点、手动操纵、安全门控与退出逻辑不变；
- 所有控件尺寸、相对位置、splitter 比例、最小/默认窗口尺寸不变；
- frameless、自绘标题栏、14 px 透明留边、圆角、完整阴影和现有 QSS 不变；
- 不通过冻结内容、缩放截图、降低分辨率或减少状态读取频率伪造流畅度。

没有修改 ROS 协议、C++ 机载控制、飞控参数或 `TODO.md`，也没有启动 SITL、连接实机、
申请控制租约、解锁或起飞。

## 2. 根因复核

本机为 X11、5120×2880 物理分辨率、Qt DPR 2.0。1228×924 逻辑窗口实际对应
2456×1848、约 454 万物理像素。

原实现每 100 ms 无条件重写状态标签、按钮 enabled、tooltip、图标和状态栏。稳定快照的
10 次刷新可产生 360 次 `ToolTipChange`、182 次 Paint，且 `outerWindowFrame` 每次都可能
参与重绘。连续 resize 的受控基线为：

| 指标 | 修改前 |
| --- | ---: |
| X11、DPR 2.0 resize wall time | 83.80 ms/次 |
| 同一 resize CPU time | 94.70 ms/次 |
| 稳定 10 次刷新 ToolTipChange | 360 |
| 稳定 10 次刷新 Paint | 182 |

关闭全部绘制后 resize 仅约 0.66 ms/次，确认瓶颈是无效栅格绘制和高 DPI 合成，
不是 splitter、scroll area 布局或窗口系统 resize 调用。

## 3. 实施内容

### 3.1 稳定状态增量应用

`GroundStationWindow` 新增纯显示层签名缓存。ROS 快照、命令结果和日志仍按原 10 Hz 路径
读取，但以下输入均未变化时，不再重复调用两个面板的 `apply_availability()`：

- 完整 `UiAvailability`；
- closing、communication running/cancel pending；
- cleanup 是否仍运行；
- pending command 集合。

签名包含原有临时禁用条件，因此 takeoff/land pending、cleanup 完成、通讯取消、环境状态
变化仍会立即重新应用原门控逻辑。

### 3.2 幂等文本与 tooltip 写入

新增两个小型显示辅助函数，只在新旧字符串不同时调用 Qt setter。已覆盖：

- 四个状态徽章和活动提示；
- 高度、航向、目标速度、控制周期和安全状态；
- 控制权、最近手动命令年龄和原点摘要；
- 环境标签、最大化按钮提示和状态栏。

没有改变格式、精度、单位或刷新数据来源。

### 3.3 安全的不透明内容提示

仅给 `centralRoot` 添加 `WA_OpaquePaintEvent`。该区域始终被现有 `windowSurface` 完整覆盖，
因此 Qt 可以跳过透明顶层之后的重复背景绘制，不改变任何可见像素。

曾试验把同一提示加到操作面板、航点面板、日志和表格 viewport；像素回归立即发现这些
区域会因 Qt 跳过原背景清除而变黑。该方案已完全撤销，自动测试反向断言这些对象不得被
标成 opaque。也试验过 resize 期间冻结内容，结果会出现空白区域，同样没有进入生产代码。

### 3.4 测试窗口资源回收

测试辅助关闭函数现在停止窗口定时器并处理 `DeferredDelete`，避免全量 GUI 测试中的旧窗口、
阴影和计时器累积。该调整只作用于测试生命周期，不进入生产退出流程。

## 4. 性能结果

使用项目 Python 环境和真实 XCB 后端，在屏幕外创建同款测试窗口并保留完整阴影：

| 指标 | 修改前 | 修改后 | 变化 |
| --- | ---: | ---: | ---: |
| resize wall time | 83.80 ms/次 | 63.96 ms/次 | -23.7% |
| resize CPU time | 94.70 ms/次 | 70.46 ms/次 | -25.6% |
| 稳定 10 次刷新 Paint | 182 | 0 | 消除稳定态重绘 |
| 稳定 10 次刷新 ToolTipChange | 360 | 0 | 消除事件风暴 |
| 修改后稳定 10 次刷新总耗时 | -- | 20.26 ms | 约 2.03 ms/次 |
| 修改后纯 GUI 2 秒空闲 CPU | -- | 0.22% | 无 ROS 测试替身 |

resize 数值是同步处理每一个程序化尺寸的保守压力测试；真实鼠标拖动时窗口系统可合并部分
中间尺寸。进一步大幅降低单次 resize 成本需要取消透明阴影、改原生窗口、GPU 重写或显示
缩放快照，这些都会改变本次明确要求保留的视觉或引入新的渲染风险，因此没有采用。

## 5. 视觉与布局验证

在 DPR 2.0 下分别生成修改前后 2456×1848 截图：

- `agent/codex/gui-performance-before.png`
- `agent/codex/gui-performance-after.png`

逐像素比较只有实时日志行的运行时间戳区域不同；排除该非确定文本后：

```text
CHANGED_PIXELS_OUTSIDE_TIMESTAMP 0
```

人工检查确认菜单、按钮、状态带、双摇杆、航点区、日志、边框、圆角和阴影均正常。
现有布局测试继续覆盖 1180×700 与 1800×1000 两种窗口尺寸、两栏同时可见和 splitter 结构。

## 6. 回归结果

```text
Qt 定向回归：26 passed in 10.80s
Python 全量回归：48 passed in 13.29s
compileall：通过
flake8 E9/F63/F7/F82/E501：通过
修改范围 git diff --check：通过
ground_station.py --check-environment：通过
colcon build（guided_interfaces/onboard_control/guided_sim）：成功
colcon test-result：5 tests, 0 errors, 0 failures, 0 skipped
```

仓库全范围 `git diff --check` 仍会报告用户原有 `TODO.md:66` 的 EOF 空行；本任务未经允许
没有修改 `TODO.md`。用户原有 `TODO.md`、`src/guided_sim/rviz/quadcopter.rviz` 和 `assets/`
工作树状态均保持。

## 7. 修改文件

- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/operations_panel.py`
- `ground_station_core/qt_ui/widgets.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`
- `agent/report/report-2026-08-09-gui-window-performance-optimization.md`
- `agent/codex/gui-performance-before.png`
- `agent/codex/gui-performance-after.png`
