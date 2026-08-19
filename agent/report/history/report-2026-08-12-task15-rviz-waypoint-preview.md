# 任务 15：RViz 航点预览实现简报

## 1. 任务与结论

- 日期：2026-08-12
- 目标：在航点操作行的“从文件导入”左侧新增“预览”，点击后用 RViz 显示航点、
  航点间直线和无人机实时位姿。
- 结论：三项首期功能均已实现；仿真复用已有受管 RViz，实机会话使用地面端独立 RViz。
- 安全边界：预览链路不申请控制租约、不进入飞行命令队列、不发布 MAVROS setpoint；本次没有
  连接、解锁或起飞实机，也没有执行 SITL 飞行。

## 2. 实现结果

### 2.1 Qt 操作入口

- 航点操作行新增普通白色“预览”按钮，位置严格在“从文件导入”左侧。
- 只有仿真或实机会话已建立、ROS 客户端就绪且至少存在一个航点时可点击。
- 预览不要求武装、控制租约或正在执行航点任务；航点任务运行时编辑仍锁定，但预览保持可用。
- 首次点击打开或复用 RViz 并发布完整快照；预览激活后，新增、移动、删除、导入和清空航点会
  自动替换 RViz 中的 retained 快照。清空列表会发布 `DELETEALL` 和空 `Path`，不残留旧图形。

### 2.2 RViz 显示数据

| 内容 | ROS 消息与话题 | QoS | 说明 |
|---|---|---|---|
| 航点与编号 | `visualization_msgs/MarkerArray` `/ground_station/waypoint_markers` | Reliable + Transient Local，depth 1 | 蓝色球体和 1 起始的文字编号 |
| 航点直线 | `nav_msgs/Path` `/ground_station/waypoint_path` | Reliable + Transient Local，depth 1 | 按航点顺序直连；姿态保留每点 Yaw |
| 实时机体位姿 | `geometry_msgs/PoseStamped` `/ground_station/vehicle_pose` | Best Effort + Volatile，depth 1 | 由权威 `ControlStatus` 的位置和 Roll/Pitch/Yaw 生成 |

固定坐标系为 `map`。`pose_to_tf.py` 只把地面站聚合后的位姿转换为
`map -> ground_station_preview/base_link`；RobotModel 的 description 话题和所有 URDF frame 也使用
`ground_station_preview` 前缀，因此不会覆盖远端 MAVROS、Odin 或其他节点的 TF。

RViz 配置只保留 `Interact` 工具，不提供 2D/3D Goal 等可能绕过 Qt 安全门控的命令工具。原先把
`PoseStamped` 错配给 Path 显示器的配置已改为类型正确的 `/ground_station/waypoint_path`。

### 2.3 仿真与实机生命周期隔离

| 会话 | DDS 传输 | RViz 策略 | 进程记录 |
|---|---|---|---|
| 仿真 | domain 231 + `LOCALHOST` | 复用仿真初始化时已启动的 RViz；不存在时恢复同一受管位置 | `rviz` |
| 实机 | domain 0 + `SUBNET` | 地面计算机启动一个独立窗口；重复点击复用，不开第二个 | `rviz_hardware_preview` |

启动前会同时校验 GUI 声明的会话模式、ROS 控制器实际 domain 和发现范围。任何不匹配都在访问
进程前拒绝；预览进程显式继承已核验的 domain/discovery，且不会建立仿真与实机桥接。仿真模式会
沿用控制器清除显式真机 peer/server 后的环境；实机模式沿用启动时保存的硬件发现配置。断开、切换
环境或退出地面站时，预览与其他本地受管进程一同结束，远端机载进程不会被终止。

预览发布使用独立的 latest-only 队列和开关；它不消费命令 ticket，不改变控制租约序号，也不进入
安全关键 `_command_queue`。可视化 publisher 只有首次点击后才懒创建，因此纯 Wi-Fi 通讯检测仍是
零可视化发送。

## 3. 主要修改

- `ground_station_core/qt_ui/waypoint_panel.py`、`state.py`、`main_window.py`、`theme.py`：新增按钮、
  可用性门控、点击/自动刷新生命周期和样式。
- `ground_station_core/ros_controller.py`：新增只读 MarkerArray、Path、PoseStamped 发布链路。
- `ground_station_core/environment.py`、`process_manager.py`：实现仿真复用、实机独立窗口、传输校验
  和统一本地清理。
- `src/guided_sim/launch/visualize.launch.py`、`scripts/pose_to_tf.py`、`rviz/quadcopter.rviz`：配置
  隔离模型/TF、正确显示话题和只读工具。
- `src/guided_sim/package.xml`：补充 `nav_msgs`、`visualization_msgs` 运行依赖。
- `tests/`：增加消息内容、QoS、UI、进程复用/隔离和静态 RViz 配置回归。

## 4. 验证结果

### 4.1 自动化与构建

- `compileall`：通过。
- flake8 致命错误选择器以及 88 字符 `E501`：通过。
- 任务范围 `git diff --check`：通过。全工作树检查仍会命中用户已有
  `integration/AGENTS.md` 空白问题，本任务未修改或整理该文件。
- 加载 `/opt/ros/jazzy/setup.bash` 与工作空间 `install/setup.bash` 后，全量 Python：
  **95 passed in 26.42s**。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：
  **3 packages finished**。
- `colcon test` + `colcon test-result --verbose`：
  **5 tests, 0 errors, 0 failures, 0 skipped**。
- 隔离环境检查：domain 232 + `LOCALHOST` 下通过，未接触 domain 0 或 domain 231 的既有会话。

一次未 source 工作空间的中间 pytest 调用因动态库搜索路径缺失出现 3 个
`libguided_interfaces__rosidl_generator_py.so` 加载失败；按项目 ROS 环境重新运行后 95 项全部
通过。这是测试调用环境问题，不是功能断言失败。

### 4.2 实际 RViz 运行验收

在临时 domain 232、`LOCALHOST` 且显式移除 `ROS_STATIC_PEERS`/`ROS_DISCOVERY_SERVER` 的隔离环境中，
启动真实 `visualize.launch.py`，使用未武装、仅本地模拟的 `ControlStatus` 发布 3 个航点与位姿：

```text
runtime_preview_ok markers=7 path_poses=3 pose_x=0.75
```

`markers=7` 对应 1 个 `DELETEALL`、3 个球体和 3 个编号；Path 含 3 个 pose，画面显示两段绿色
直线和当前机体模型。实际视觉检查初次发现 RViz 的 `TF Prefix` 尾部斜杠会形成
`ground_station_preview//a1`，随后去掉尾斜杠并新增静态回归，最终 RobotModel 与所有图层均为正常
状态。最终截图：`agent/codex/task15-rviz-final.png`。

首个运行探针曾误用 rclpy 全局 `spin_once` 驱动自定义 Context；改为显式
`SingleThreadedExecutor` 后通过。验收结束后已停止全部测试 RViz、位姿桥和探针进程；进程扫描无
本任务残留。

## 5. 当前边界与后续项

- 本期路径明确是航点间名义直线，不做碰撞检查，不能标称为避障规划结果。
- 按用户要求，障碍物、Odin 点云/地图、规划曲线和已飞轨迹暂未实现。
- 没有连接真实飞机，因此“domain 0 + SUBNET 的独立地面 RViz”由环境/进程回归覆盖，尚未做真机
  链路现场验收。
- 本次未验证用户手动关闭仿真 RViz 后 launch 中其余节点仍存活时的自动恢复行为；正常地面站
  管理的启动、复用、切换和清理路径已覆盖。
