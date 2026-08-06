# 项目重要记忆

## 环境、入口与构建

- 开发机为 Ubuntu 24.04、ROS 2 Jazzy；已安装 MAVROS 与 ArduPilot SITL。
- 地面站入口仍为仓库根目录 `ground_station.py`，Tk GUI 与薄客户端实现位于 `ground_station_core/`；没有引入 PyQt 或浏览器。入口会在未手动 source 时自动加载 `/opt/ros/<distro>/setup.bash` 和本仓库 `install/setup.bash` 后原位重启，因此构建完成后可直接运行 `python ground_station.py`。
- 工作空间需构建 `guided_interfaces`、`onboard_control`、`guided_sim`：`source /opt/ros/jazzy/setup.bash && colcon build --packages-select guided_interfaces onboard_control guided_sim`。
- `test_takeoff5.py <高度>` 走与 GUI 相同的高层机载协议，不含独立控制算法。
- `python ground_station.py --check-environment` 可在不创建窗口的情况下验证自动 overlay、`guided_interfaces` 和地面站 ROS 客户端线程。
- 外部仿真日志写入 `/tmp/ros2_ardupilot_ground_station/`。

## 重构后的部署边界

- `src/guided_interfaces/` 是上位机与机载计算机唯一共享的 ROS 2 高层协议，接口版本为 `1.0`。
- `src/onboard_control/` 是无 GUI 的机载 C++ 服务：控制权仲裁、起降编排、航点推进、失联保护、100 Hz PD+DOB、姿态/推力输出和 MAVROS 网关全部在此。
- `ground_station_core/ros_controller.py` 是薄客户端，只发布心跳/运动意图并调用高层服务；地面站不创建任何 MAVROS setpoint 发布器，也不保存安全关键的持续控制状态。
- `ground_station_core/environment.py` 的仿真路径会在本机启动 SITL、MAVROS、同款机载 C++ 节点和 RViz；实机路径只连接局域网中的远端机载服务，不启动或终止远端 MAVROS、Odin、extnav 或控制节点。
- `src/guided_sim/` 只保留 URDF、RViz 和 TF 可视化桥；旧的未启用 C++ 控制器与 Python PD+DOB/飞行模式副本均已删除，不能恢复成双实现。

## 控制与 ArduPilot 约束

- 上/下/左/右/前/后、偏航、悬停和航点最终全部进入同一个 C++ `DobController`，输出 `/mavros/setpoint_raw/attitude`；该话题只允许机载节点一个发布者。
- 运动按键是带来源、序号和 TTL 的速度增量意图。机载端累加/限速，并积分为有界虚拟位置参考；模式切换时明确抓取当前位置并重置 DOB。
- ArduPilot 必须设置 `GUID_OPTIONS` bit 3（值至少包含 `8`），使 `SET_ATTITUDE_TARGET.thrust` 表示真实归一化推力；未确认时机载端拒绝进入原始姿态/推力控制。
- 机载端同时读取飞控 `MOT_THST_HOVER` 并同步控制器悬停推力。SITL 实测约为 `0.3744`；旧硬编码 `0.2` 会在真实推力模式中持续掉高，禁止恢复。
- 控制器使用实测 `dt`，并对 DOB、加速度、倾角、推力、虚拟参考距离及非有限值做限幅/保护。100 Hz timer 位于独立回调组，多线程执行器避免状态图查询阻塞控制调度。
- 起飞由 ArduPilot GUIDED/arm/takeoff 完成安全离地，接近目标高度后切入同一 PD+DOB 并保持请求高度；LAND 交给 ArduPilot。

## 高层协议与安全状态机

- 同一时刻只有一个 `source_id` 可持有控制租约；地面站以 5 Hz 心跳续租。所有飞行输入共享单调序号并校验时间戳、TTL、重复及乱序。
- 租约丢失时，机载端立即抓取当前位置独立悬停；默认等待 10 秒仍未恢复则自动切换 LAND。地面站恢复租约后需显式发送悬停或新任务退出失联状态。
- 飞控状态或本地位姿/速度超时、非有限控制输出、姿态 setpoint 多发布者、飞行中推力语义校验失效会触发机载 LAND；外部切走 GUIDED 时停止发送 setpoint，避免争夺飞控模式。
- `ControlStatus` 聚合飞控状态、实际/目标位姿速度、模式、航点进度、租约、推力校验、发布者冲突和控制频率/抖动诊断；GUI 只展示该权威状态。
- ROS 2 DDS 当前按“可信同一局域网”设计。两端设置相同 `ROS_DOMAIN_ID` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`；多播不可靠时配置 `ROS_STATIC_PEERS`。本阶段没有加入跨网段路由、VPN、SROS2 身份或加密。

## 生命周期与部署文件

- 仿真依次启动 `sim_vehicle.py -v ArduCopter --param GUID_OPTIONS=8`、TCP 5762 MAVROS、`onboard_control_node` 和 RViz；MAVProxy stdin 必须保持打开。
- 机载启动入口为 `ros2 launch onboard_control control.launch.py`；`src/onboard_control/deploy/` 提供 systemd 与局域网环境示例，安装时必须替换 `WORKSPACE`。
- 上位机局域网变量示例为 `ground_station.env.example`，机载变量示例为 `src/onboard_control/deploy/onboard.env.example`。
- 清理只释放控制租约并结束本地仿真受管进程；历史残留匹配要求本项目独有节点或本地 SITL 端点参数，不能停止 ROS daemon 或任意其他 ROS 工作负载。

## 已验证基线（任务 03，2026-08-06）

- 修改前备份：`/home/nvidia/scq/projects/backups/ros2-ardupilot-sitl-hardware-pre-task03-20260806-175032.tar.gz`；SHA-256 `1ae274af2a8ec5e8a41e92527f2863583b86c39a686a0b79cb285648bedf7add`，gzip 校验通过。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim` 成功；`colcon test-result --verbose` 为 5 tests、0 errors/failures；增加未 source 直接启动回归后 Python 为 7 passed。
- 在线协议注入通过：唯一租约、第二客户端拒绝、重复/乱序拒绝、过期命令拒绝、主动释放。
- 完整 SITL 通过起飞、前后左右上下、左右偏航、悬停、航点、失联悬停/恢复、LAND/解锁；六向与偏航方向检查均无失败。
- 5 秒悬停漂移 `0.039 m`，单航点三维误差 `0.089 m`，最终平均控制频率 `100.03 Hz`，setpoint 冲突为 false，清理 4 个受管进程且无残留。
- 独立失联试验通过：释放租约后立即机载悬停，10 秒宽限期后自主 LAND 并解锁。
- 非实时 Ubuntu 上完整飞行期间记录过 1 次 deadline miss、最大调度间隔抖动 `203.404 ms`；平均频率与飞行未受影响，但实机部署应继续做 CPU/调度隔离和长时间统计，不能宣称硬实时。
- 无真机可用，因此远端伴随计算机、物理飞控、Odin/extnav 和真实 Wi-Fi 端到端尚未验证；详细使用、调试、证据和限制见 `agent/report/report-2026-08-06.md` 的任务 03 章节。

## 版本库卫生

- `.gitignore` 排除 Python/colcon 产物、rosbag、ArduPilot/MAVProxy 日志、EEPROM、飞行记录和通用临时文件。
- `.deep-copilot/` 运行数据不应重新提交；本机 `AGENTS.md` 保持本地。
- 仓库不维护 `README.md`；正式执行与部署说明以当日报告、配置注释和 launch/deploy 示例为准。
