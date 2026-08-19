# 地面站仿真/实机隔离与退出死锁修复报告

日期：2026-08-10

## 任务目标

仅修改地面站侧，使以下行为同时成立：

1. 单纯打开地面站 GUI 不加入真机 DDS 图，也不向真机发送任何 ROS 数据。
2. 本地 SITL 仿真与 domain 0 的真机机载服务同时运行时互不发现、互不通信。
3. 结束本地仿真不会卡住 GUI、终止按钮或窗口关闭流程。
4. 仿真结束后不重启 GUI，即可由操作者点击“连接实机服务”，切回真机 domain 0。
5. 不修改、部署或重启真机机载服务；不解锁、不起飞真机。

## 原因确认

### 1. 仿真与真机原先共享 domain 0

旧地面站在窗口创建后立即启动 ROS participant，本地 SITL、仿真 MAVROS、仿真
`onboard_control_node` 也继承地面站的 domain 0。真机四个服务同样使用 domain 0，
因此两套系统会发现同名的 `/onboard_control/*` 和 `/mavros/*` 端点。此时即使操作者
只想运行本地仿真，地面站与本地节点也会进入真机所在 DDS 图。

这会放大 Jazzy/Humble Fast DDS 跨发行版发现问题，并可能让同名状态、服务和日志端点
混在同一图中。`sequence size exceeds remaining buffer` 并不是 UDP 消息量过大，而是已
确认的跨发行版 Fast DDS discovery/type 解码兼容问题；共享 domain 使它在不需要真机时
也被触发。

### 2. 仿真退出卡死是进程树和日志管道清理问题

`sim_vehicle.py` 是受管进程组长，但它会派生 xterm/ArduCopter 后代。收到 SIGINT 后，
组长可能先退出，后代仍存活并继续持有 stdout 管道写端。旧清理逻辑只继续处理
`record.running == true` 的组长，因此跳过已经退出组长对应的 TERM/KILL 升级。

随后日志读取线程仍阻塞在管道读取，清理线程却跨线程调用 `TextIOWrapper.close()`。
`close()` 会等待读取线程持有的缓冲区锁，导致清理线程永久卡住。退出地面站时又先等待
已有清理线程，再次进入同一清理路径，最终表现为“终止仿真”“退出地面站”和窗口 X
全部无响应。

## 实现结果

### DDS 传输隔离

- 真机保持现状：`ROS_DOMAIN_ID=0`、`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`。
- 本地仿真固定使用：`ROS_DOMAIN_ID=231`、
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。
- SITL、仿真 MAVROS、仿真 onboard 和 RViz 四个子进程均显式继承仿真传输环境。
- 仿真期间暂时移除 `ROS_STATIC_PEERS` 和 `ROS_DISCOVERY_SERVER`，防止用户配置的
  真机静态地址绕过 localhost 自动发现边界；切回实机时恢复 GUI 启动时的原值。
- GUI 默认保持 `ROS IDLE`。单纯打开窗口不会创建 DDS participant；只有操作者点击
  仿真、实机连接或只读通讯检测后才按需启动 ROS。

### 同一 GUI 动态切换

`GroundStationRosController` 现在为每个传输环境创建独立 `rclpy.Context` 和独立 executor。
切换时按以下顺序执行：

1. 释放当前环境租约（若存在）。
2. 停止并销毁旧 executor、node 和 context。
3. 清空旧环境尚未发送的命令队列，并为其生成明确失败结果。
4. 应用目标 domain/发现范围。
5. 创建新 context，等待就绪后才继续对应环境工作流。

因此，仿真结束后 ROS 客户端可以暂留在隔离 domain 231；用户点击“连接实机服务”时，
同一 GUI 会销毁该 context 并创建 domain 0 context，不要求重启 GUI。

机载进程重启后，如果旧租约服务响应提示已经过期但新状态不再确认本客户端持权，地面站
会清除本地 grant hint 并重新申请租约，避免永久只发送无效心跳。

### 冲突故障关闭

- 连续发现多个 `/onboard_control/status` 发布者时设置 `endpoint_conflict`。
- 冲突期间停止租约申请/心跳，拒绝所有高层命令，包含 LAND 在内的 GUI 飞行动作全部
  禁用；这是因为多个同名服务存在时无法保证命令只送达预期飞行器。
- 环境初始化的每个等待门都会检查该冲突并立即失败清理。

### 有界进程清理与 GUI 退出

- 启动时保存真实 PGID；终止前一次性抓取全部后代 PID。
- SIGINT、SIGTERM、SIGKILL 三阶段始终作用于保存的进程组和逃逸到新 session 的后代，
  不再依赖组长是否仍运行。
- 日志线程在 1 秒内未退出时，禁止从另一线程关闭其缓冲流；返回可审计错误而不是卡死。
- 同时发生的“终止仿真”和“退出地面站”清理请求合并为同一次操作，等待设置 15 秒硬
  上限；关闭流程不再先 join 再二次清理。
- 清理线程存活期间，仿真、实机连接和通讯检测入口保持禁用，避免新工作流与旧清理竞争。

## 修改范围

地面站生产代码：

- `ground_station_core/config.py`
- `ground_station_core/models.py`
- `ground_station_core/ros_controller.py`
- `ground_station_core/environment.py`
- `ground_station_core/process_manager.py`
- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/state.py`
- `ground_station.env.example`
- `start_ground_all.sh`

测试代码：

- `tests/test_ros_controller.py`
- `tests/test_environment_communication.py`
- `tests/test_process_manager.py`
- `tests/test_qt_gui.py`
- `tests/test_onboard_deploy.py`

本任务没有修改或同步真机目录，没有修改机载运行逻辑。工作树中已有的
`src/onboard_control/*` 改动属于本任务开始前的其他工作，本任务未覆盖或回退。

## 验证结果

### 自动回归

- 全量 Python：`60 passed in 18.58s`。
- 定向进程/ROS/环境/Qt 回归：`50 passed in 16.96s`。
- `compileall`：通过。
- flake8 致命错误集合 `E9,F63,F7,F82`：通过。
- `bash -n start_ground_all.sh`：通过。
- `shellcheck start_ground_all.sh`：通过。
- 本任务文件的 `git diff --check`：通过。

新增回归覆盖：

- 同一控制器实际销毁旧 context 并从一个隔离 domain 切换到另一个 domain。
- 生产工作流选择仿真 domain 231 后再选择硬件 domain 0。
- 仿真隐藏、实机恢复静态 DDS peer/discovery server。
- 多状态发布者时租约和命令均故障关闭。
- onboard 重启后过期 grant hint 自动重新申请。
- 父进程先退出、后代创建新 session、忽略 INT/TERM 且持有 stdout 时，清理仍有界完成。
- 两个并发清理调用只执行一次进程清理并得到同一结果。
- 清理期间三个环境入口全部禁用。
- 单纯打开 GUI 不启动 ROS。

### 真实本地 SITL 集成

使用生产 `EnvironmentInitializer` 和 `GroundStationRosController` 完成一次真实本地闭环启动
和停止，未解锁、未起飞：

- 传输：`domain=231`、`discovery=LOCALHOST`。
- `onboard_available=true`。
- `fcu_connected=true`。
- `control_authority=true`（仅本地仿真租约）。
- `local_position_valid=true`。
- `armed=false`。
- 清理：`managed_stopped=4`、`stale_stopped=()`、`remaining=()`、`errors=()`。
- 总耗时：53.71 秒。
- 清理后项目相关进程扫描为空，TCP 5762 无监听。
- 用户原有地面站进程 PID 68732 保持运行，未被测试清理误杀。

## 边界与后续人工验证

当前已经运行的 PID 68732 仍是修改前加载到内存的 Python 代码，无法热更新。需要只做一次
地面站重启来加载本次修改；加载后，仿真与实机之间的每次切换都不再要求重启 GUI。

本轮没有点击真实“连接实机服务”，因为该动作会申请真实控制租约、配置消息频率并写入
GPS 原点，属于有状态真机操作。domain 231 本地集成与动态 context 自动回归已通过，但
最终真机连接仍应由操作者在确认飞机安全状态和原点后手动点击验证。

本次隔离保证跨发行版 Fast DDS 报错不会在“仅打开 GUI”或“运行本地仿真”时被真机触发；
当操作者明确连接 domain 0 真机时，Jazzy/Humble 本身的 Fast DDS 类型兼容告警仍可能出现。
彻底消除该底层兼容问题仍需统一 ROS 发行版/RMW 或使用受支持的桥接，不能由地面站业务
代码伪装为已经修复。
