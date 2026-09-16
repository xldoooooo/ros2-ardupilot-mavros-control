# 将 new 的启动优化同步至 refresh

本报告记录 refresh 在无飞控、有 Odin 条件下的部署与验证；new 已下线，本轮未连接 new。

## 同步范围与方式

refresh 从 9e59afb 快进到 66a3a09，包含离线校时兼容的 READY 相对超时、去除串行等待、
提前非强制拉表、必要参数事件校验、重复 ROS CLI 包查询清理及正式按名称优先读取。
使用本地 Git bundle 同步已提交版本，不复制 new 的机器专用配置，也不改动 refresh 的
未跟踪 start_drone/image/ 目录。MAVROS 插件和 Odin 辅助节点均未裁剪。

部署前备份 refresh 原 src/onboard_control、install/onboard_control、启动器和现场环境文件：
`/home/nvidia/ros2-ardupilot-mavros-control/agent/codex/refresh-startup-sync-20260917/predeploy.tar.gz`
SHA-256：6808e79634eadcc2271eac6ef95eee2740d9bc6b63e651ad392edfd83ea3e46a。

现场仍使用 /dev/ttyTHS2、/home/nvidia/catkin_ws/install/setup.bash 与
/home/nvidia/vrpn_mavros/install/setup.bash。/etc/ros2-ardupilot/onboard.env 的 SHA-256 为
f33a02b7574f409b733e3fa3d11216b7bc4bcffa5a4bf91502ef9df69bfd2ab5，保持不变。
机载 systemd 服务原为 disabled/inactive；保持其开机启用策略。

## 验证

refresh ARM64 构建成功，19 项 C++ 测试通过；启动器 --check 通过，四份关键源码/启动文件
SHA-256 与开发机一致。正式四组件启动约 18 秒后主动退出，观测到 3205 条 Odin 里程计、
156 条机载状态，fcu_connected/armed/controller_active/lease_active/thrust_mode_verified/
local_position_valid 始终 false，独立源 255/190 的优先参数请求为 0，未打印 READY。

无飞控时 MAVROS 的版本请求超时属于该硬件条件下的预期现象。Odin 的 RViz 在无显示环境下
退出，以及 SDK 实时调度权限警告仍存在；本次未裁剪辅助节点、未修改调度权限，不能称为零警告。
Odin 主驱动和 extnav 已实际输出/收到里程计。

无飞控验证不能代替完整参数读取、真实 READY、起飞或航点验收；这些需接回飞控后复验。
本轮未发送解锁、起飞或控制租约。最终服务 MainPID=0、inactive、disabled，无机载进程残留。
已保留用户不用时关闭 Odin 的方式；new 未连接，独立 USB Jetson 仍待同步。
过程证据保存在 agent/codex/refresh-startup-sync-20260917/。
