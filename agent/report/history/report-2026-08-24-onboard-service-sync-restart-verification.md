# 机载服务同步、systemd 重装与未解锁验证报告

本文记录 2026-08-24 对当前 Jetson 机载计算机 `nvidia@192.168.112.169` 的选择性同步、原生构建、
飞控与视频 systemd unit 重装、未解锁状态验证，以及 MAVROS 时间同步和 Odin 实时调度告警排查。

## 安全边界

- 全程没有发送模式切换、解锁、起飞、降落、运动、姿态、推力或航点命令。
- 停止服务前两次读取 `/mavros/state`，均确认 `armed=false`、`STABILIZE`。
- 构建 smoke 使用隔离 ROS domain、localhost-only 发现和专用前缀，不连接真实 MAVROS。
- 真机最终仍为 `connected=true`、`armed=false`、`guided=false`、`STABILIZE`。

## 同步与备份

- 本地基线：`main = origin/main = 465ce8a605326920d1478347b0933217d5aab3ad`。
- 飞机 Git HEAD 仍为历史 `6a407133eb77de6fab0ddc63d5506170c64ef444`；现场工作树包含已验证
  的逐文件部署、Odin 自动生成内容和新目录，因此没有执行 pull、reset、clean 或整仓覆盖。
- 同步前备份：
  `/home/nvidia/backups/onboard-sync-pre-20260824-224307.tar.gz`。
- 备份 SHA-256：
  `e714dc92bf49153658e3bfb329e4cedcc19dca6591b3325ea33f85547528abd8`。
- 通过 rsync 只同步当前 Git 跟踪的 61 个机载范围文件：两个 ROS 包、飞控启动/停止/构建入口、
  `start_drone/`、独立视频服务与其部署入口；没有删除或覆盖 Odin 现场文件。
- 同步后和最终重启后均完成本地/飞机全清单 SHA-256 比对，61 个文件全部一致。

## 构建与 systemd 重装

飞控安装器在服务停止后执行 ARM64/Jazzy 原生 Release 构建与验证：

- `guided_interfaces`、`onboard_control`：2 packages finished；
- colcon：19 tests、0 errors、0 failures、0 skipped；
- 隔离 smoke：`interface=3.2`、`fcu_connected=false`、`armed=false`、
  `setpoint_messages=0`；
- `start_onboard_control.sh --check` 正确发现 Jazzy、`/dev/ttyTHS1:460800`、Odin 与 extnav overlay，
  且没有启动组件。

随后分别运行两个正式安装器：

- `src/onboard_control/deploy/install_onboard_service.sh`；
- `video_service/deploy/install_onboard_video_service.sh`。

安装器保留了 `/etc/ros2-ardupilot/onboard.env`、`camera.conf` 和 `lens.conf`。重装后的
`/etc/systemd/system/ros2-ardupilot-onboard.service` 与
`/etc/systemd/system/video-service.service` 均与本地模板替换现场占位符后的内容完全一致。

维护过程有一个非功能性插曲：旧飞控 unit 被 `systemctl stop` 时，启动器收到 SIGINT、完成四组件
清理后返回 130，旧 unit 因此短暂显示 `failed`，使第一条串联维护命令提前退出。检查确认所有组件
均已停止、没有残留后才继续安装；最新 unit 随后正常启动，最终 `Result=success`。

## 重启后飞控结果

- `ros2-ardupilot-onboard.service`：active、running、enabled、`NRestarts=0`；
- MAVROS：connected、`armed=false`、STABILIZE；
- `ControlStatus.interface_version=3.2`；
- 本地位置有效，控制模式为待机；
- 控制器未激活，租约为空，无 setpoint 冲突或 failsafe；
- 消息频率与推力语义已确认；
- 控制频率约 100.04 Hz，本次读取 `max_jitter_ms=4.42`、`deadline_miss_count=0`。

本次只验证未解锁地面状态，不能据此声称已经完成真实飞行验收。

## 重启后视频结果

- `video-service.service`：active、running、enabled、`NRestarts=0`；
- `VideoStatus.interface_version=3.2`，服务可用；
- 独立调用 `/video_service/set_video_state` 开启摄像头，不经过飞控租约；
- RTSP/TCP 实测：H.264、1920×1080、60 fps；
- 生成 `/home/share/recording-20260824-224835-923125.mp4`，大小 14,922,546 bytes，时长
  6.166666 秒；
- 关闭请求成功，最终状态 stopped，FFmpeg、MediaMTX、8554 监听端口均释放。

## `Time jump detected` 原因与影响

飞机安装的 MAVROS 为 2.14.0，`/mavros/time` 当前使用：

- `timesync_mode=MAVLINK`；
- `timesync_rate=10 Hz`；
- `convergence_window=500`；
- `max_deviation_sample=10 ms`；
- `max_consecutive_high_deviation=10`；
- `max_rtt_sample=10 ms`。

MAVROS 的时间插件用 MAVLink `TIMESYNC` 估计 Jetson ROS 时钟与飞控时钟的偏移。滤波器收敛后，
如果低 RTT 样本相对当前偏移估计连续偏差超过 10 ms，超过允许次数后就记录
`Time jump detected` 并重置滤波器。该日志不是解锁、飞行控制或飞控重启事件，也不等同于当前
Linux NTP 失效。

现场确认 Jetson 为 `NTP=yes`、`NTPSynchronized=yes`，`systemd-time-wait-sync.service` 已成功；
重启后的一个样本 RTT 为 7.78 ms、观测偏移与估计偏移相差约 0.9 ms，重启后的观察窗口没有再次
出现 Time jump。重启前的重复告警更像飞控端时间样本、串口排队/负载或瞬时往返时延导致偏移样本
连续异常，但当前证据不足以唯一归因。

单次或偶发重置通常只会让 MAVROS 暂时重新收敛时间偏移；连接、未解锁状态和本项目 100 Hz 控制
循环仍可继续。若持续发生，MAVROS 消息时间戳可能短时不稳定，外部定位、IMU/视觉融合和跨传感器
对时会受到实际影响，因此真实飞行前应长时间记录 `/mavros/timesync_status`、串口吞吐和飞控端
时间源，不能简单隐藏日志或放宽阈值。

MAVROS 当前实现与阈值语义见：
<https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/sys_time.cpp>。

## Odin 无法启用实时调度的原因与影响

Odin `host_sdk_sample` 会尝试把 IMU 专用线程设为 SCHED_FIFO，失败后尝试 SCHED_RR，最后回退普通
调度。现场实际权限为：

- 进程调度策略 SCHED_OTHER、优先级 0；
- `LimitRTPRIO=0`；
- 有效 capability 为空，没有 `CAP_SYS_NICE`；
- 因此 `sched_setscheduler()` 返回 `EPERM`，与日志中的 errno=1 完全一致。

这不会让 Odin 立即不可用：设备与数据流能够启动，四组件服务能达到 READY，重启后控制状态也正常。
实际影响是高 CPU、I/O 或中断压力下 IMU 线程不能抢占普通任务，可能出现更大的调度抖动、延迟或偶发
样本积压；对依赖 Odin 外部定位时间一致性的飞行有潜在影响。

本次没有直接为整个四组件 unit 增加 root 权限或 `CAP_SYS_NICE`。飞控、MAVROS、Odin、extnav 和
onboard_control 共用一个 unit，盲目提权会扩大权限范围，也可能由错误优先级造成系统饥饿。后续应先做
长时间负载与话题时间戳统计，再单独评审 Odin 所需的 `LimitRTPRIO`、capability、线程优先级和 CPU
隔离，并在无桨未解锁台架复验。

## 最终结论

机载运行源码、原生 install 产物、两个 systemd unit 与本地最新机载范围已经同步。飞控四组件和
独立视频服务重启后均正常工作，视频真实 RTSP/录像链路通过，最终飞控始终未解锁、未起飞。仍需
保留的运维风险是 MAVROS 时间同步告警的长期复现性与 Odin 普通调度下的高负载抖动，两者均不能用
本次短时未解锁验证替代正式耐久测试。
