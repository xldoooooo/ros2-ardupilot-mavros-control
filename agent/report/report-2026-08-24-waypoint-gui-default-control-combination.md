# 航点 GUI 默认控制组合修改简报

日期：2026-08-24

## 任务目标

将地面站 GUI 右下角“命令生成”和“跟踪控制”两个下拉框的启动默认选项，由“位置阶跃 +
位置 PD+DOB”修改为“普通梯形速度 + 轨迹 PD+DOB”。

本任务只修改地面端 GUI 默认选择与自动化测试，没有连接实机、申请控制权、解锁或起飞。

## 修改内容

- `WaypointPanel` 创建“命令生成”下拉框后，默认选中
  `WaypointReferenceGenerator.TRAPEZOIDAL_PROFILE`。
- 创建“跟踪控制”下拉框后，默认选中
  `WaypointTrackingController.TRAJECTORY_PD_DOB`。
- 航点手动发送、上位机巡检组合和自动返航组合原本都会读取这两个控件的当前枚举，因此默认
  选择会随既有链路实际下发，不需要复制或修改另一套业务默认值。
- 保留协议未知值解析、机载待机状态和异常回退所使用的“位置阶跃 + 位置 PD+DOB”安全基线；
  本次不改变接口常量、控制参数或机载控制器逻辑。
- GUI 回归新增两个默认枚举断言，防止后续只改变显示文字或意外退回索引 0。

## 变更文件

- `ground_station_core/qt_ui/waypoint_panel.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`

## 验证结果

1. `git diff --check`：通过。
2. 默认组合定向 GUI 回归：`1 passed, 44 deselected`。
3. 完整 GUI 测试：`45 passed in 23.84s`。
4. 加载 ROS 2 Jazzy 与项目 `install/` overlay 后执行完整 `tests/`：
   `167 passed in 39.52s`。

## 结果与边界

GUI 新开时会直接显示并返回“普通梯形速度 + 轨迹 PD+DOB”。用户仍可在解除武装待机阶段手动
选择其他组合；飞行中锁定选择、未验证组合确认和机载同一武装周期锁定规则均保持不变。
