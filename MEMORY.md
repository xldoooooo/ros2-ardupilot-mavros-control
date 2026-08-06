# 项目重要记忆

## 环境与入口

- 开发机为 Ubuntu 24.04、ROS 2 Jazzy；MAVROS 和 ArduPilot SITL 已安装。
- GUI 入口为仓库根目录 `ground_station.py`，实际实现位于 `ground_station_core/`。
- `test_takeoff5.py <高度>` 是共享起飞模式的 CLI 回归入口，不再维护独立起飞逻辑；运行前需已有 MAVROS/飞控连接。
- `guided_sim` 修改后使用 `source /opt/ros/jazzy/setup.bash && colcon build --packages-select guided_sim` 构建。

## 地面站架构

- `ground_station_core/gui.py`：仅负责 Tk 控件、输入校验和主线程事件消费。
- `ground_station_core/ros_controller.py`：常驻 rclpy 后台线程、状态快照、命令票据和模式输出调度。
- `ground_station_core/flight_modes/`：三个互斥模式分别是 `takeoff_land.py`、`keyboard_control.py`、`waypoint_flight.py`。
- `ground_station_core/mode_manager.py`：最后一次飞行按键/命令接管；输入序列可阻止旧队列命令反向覆盖新模式。
- `ground_station_core/environment.py`：一键仿真与实机初始化编排。
- `ground_station_core/process_manager.py`：独立进程组、日志、SIGINT→SIGTERM→SIGKILL、历史残留/后代扫描和 ROS daemon 关闭。
- 外部进程日志固定写入 `/tmp/ros2_ardupilot_ground_station/`。

## 初始化与部署约定

- 仿真按钮依次启动 `sim_vehicle.py -v ArduCopter`、TCP 5762 MAVROS、设置消息 32/31/105 为 100 Hz、等待 EKF 本地位置、启动 RViz。
- SITL 的 MAVProxy 必须保留打开的 stdin 管道，否则读到 EOF 会退出并带停 SITL。
- 仿真初始化通常约 40–50 秒，主要等待 GPS/EKF 产生新鲜 `/mavros/local_position/pose`；不要删除该就绪门槛。
- 实机默认使用 `/dev/ttyTHS1:460800`，依赖 `~/ws/install/setup.bash` 中的 `odin_ros_driver` 和 `~/vrpn_mavros/install/setup.bash` 中的 `extnav_bridge`。
- 实机路径/串口可用 `GROUND_STATION_ODIN_SETUP`、`GROUND_STATION_EXTNAV_SETUP`、`GROUND_STATION_REAL_FCU_URL` 覆盖。
- 原 `odin1.sh` 及 `shfiles/*.sh` 已有意删除；其功能已迁入 Python，不应恢复脚本形成双重启动路径。

## 控制与生命周期约束

- 起降、键盘、航点模式互斥；后按下的模式会清理前一模式的速度/航点状态。
- 起飞不会再对已武装飞行器先 disarm；会等待 GUIDED、有限重试武装并确认高度。
- GUI 输入框获得焦点时，字母输入不得触发飞行快捷键。
- “关闭所有进程”保留 GUI 内嵌 ROS 节点以便再次初始化，但必须清空飞行状态并彻底结束所有外部 ROS/SITL 进程及 ROS daemon。
- RViz 通过 `visualize.launch.py` 自动加载 `quadcopter.rviz`；`pose_to_tf.py` 对 launch 的 SIGINT 安静退出。

## 已验证基线（2026-08-06）

- `pytest -q tests`：8 passed。
- `colcon build --packages-select guided_sim`：成功。
- 最终代码在同一地面站 ROS 实例内连续两次完成仿真初始化与清理；每次回收 SITL/MAVROS/RViz 3 个受管进程，残留扫描为空，ROS daemon 未运行。
- SITL 实测通过起飞、DOB 悬停、0.8 m 水平航点、LAND/自动解锁。
- 本机缺少 Odin/extnav 工作空间和实机硬件，因此实机端到端尚未验证；按钮会在预检阶段明确失败且不留下进程。
- 详细证据见 `agent/report/report-2026-08-06.md`。

## 版本库卫生

- `.gitignore` 统一排除 Python 缓存、ROS 2/colcon 产物、rosbag、ArduPilot/MAVProxy 日志与运行状态、飞行记录、Deep Copilot 状态目录和通用临时文件。
- `.deep-copilot/` 历史日志已取消 Git 跟踪；运行产生的数据应保留在本地，不应重新提交。
- 仓库不再维护 `README.md`。
