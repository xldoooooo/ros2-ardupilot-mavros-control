# 无外网地面站本地时间校验修复报告

本文记录无外网环境下控制租约/起飞命令因两机绝对时间不同而被拒绝的问题，以及 2026-08-25
在开发机和当前 Jetson 真机完成的修复、部署与未解锁验证。

## 结论

问题已修复并部署。飞行命令不再要求地面站与飞机拥有相同的国际日期，也不再等待外网 NTP 才启动
机载服务；地面站首次成功申请租约时建立发送端本地时间基准，5 Hz 有序心跳持续刷新基准，后续
TTL 只比较两端从该基准起的相对流逝时间。重复/乱序保护、TTL、控制租约和失联保护均保留。

当前线协议与消息结构未改变，接口仍为 3.2；旧地面站继续发送原有 ROS 时间戳即可，无需更改。

## 根因证据

原 `validate_envelope()` 直接计算：

```text
age = 机载 Node::now() - 地面站 stamp
```

只要地面站时间比飞机快 2 秒以上，连首次 `AcquireControl` 都会返回“命令时间戳来自未来，请检查
两端时间同步”。租约无法建立后，GUI 的起飞等所有高层命令自然无法下发。

当前 Jetson 本次启动提供了直接现场证据：

- boot ID：`9d4c2eb9-bc0b-4d2a-b3fa-fd5bbcdb1d50`；
- uptime 约 5 分钟时，journal 的本次启动最早绝对日期仍为 `2026-07-28`；
- NTP 完成后系统日期跳到 `2026-08-24`，偏差约 27 天；
- 旧 unit 直到 `systemd-time-wait-sync.service` 成功后才在 23:46:18 启动四组件。

因此现场现象不是推测：无外网时飞机保存的绝对时间确实可能落后数周，而旧协议把这个偏差错误地
当成网络中过期/未来命令；旧 systemd unit 同时会在冷启动时无限等待远端准确时间源。

## 修改

### 飞行命令时间基准

- 首次 `AcquireControl` 只校验非空来源、TTL 范围和非零时间戳；成功授予租约时保存：
  - 地面站发送时间戳；
  - 机载 `std::chrono::steady_clock` 接收时刻。
- 当前租约持有者的每条有序心跳重新建立同一相对基准，能容忍地面站墙钟校准或跳变。
- 普通命令年龄改为：

  ```text
  age = 机载单调时钟相对流逝时间 - 地面站时间戳相对流逝时间
  ```

- 超过 TTL 的延迟命令仍拒绝；异常超前、错误来源、无租约、重复或乱序命令仍拒绝。
- 主动释放租约会同时清除时间基准。

### 离线启动与视频命令

- `ros2-ardupilot-onboard.service` 只等待 `network-online.target`，移除
  `systemd-time-wait-sync.service` 和 `time-sync.target`；局域网可用但没有互联网时不会被 NTP
  阻塞。
- 独立视频面板的 `SetVideoState` 同样改为按每个随机 `source_id` 建立相对时间基准，避免留下第二处
  同类离线故障。
- README、机载部署文档、接口注释和参数注释已同步说明新语义。

## 自动化验证

开发机 Ubuntu 24.04 / ROS 2 Jazzy：

- `./build_onboard_control.sh --verify`：通过；
- colcon：19 tests、0 errors、0 failures、0 skipped；
- 隔离 smoke：接口 3.2、`fcu_connected=false`、`armed=false`、零姿态 setpoint；
- 完整项目 Python：180 passed；
- Ruff（本次 Python 范围，忽略文件既有的非可执行 shebang 规则）：通过；
- `git diff --check`：通过。

专项 ROS 隔离测试覆盖：

1. 首次租约的地面时间比机载时间快 7 天，成功授予；
2. 有序心跳把地面时间从快 7 天跳到慢 11 天；
3. 跳变后的新鲜维护命令继续接受；
4. 同一基准下故意回退 30 秒、TTL 100 ms 的命令明确返回“命令已过期”；
5. 视频首条命令与服务端绝对时间相差 30 天时仍接受，后续过期命令仍拒绝。

## 真机部署与验证

维护前连续两次读取真实状态，均为：`connected=true`、`armed=false`、STABILIZE、无租约、控制器未
启用；独立视频为 stopped。确认后才停止四组件 unit。

部署前备份：

- 路径：`/home/nvidia/backups/offline-clock-predeploy-20260825-0009.tar.gz`；
- SHA-256：`7f78963ad63dbf80adb73ec7b9fb0f359ba3236c6bdaab302a90800c29c74368`。

13 个相关生产文件通过选择性 rsync 部署并逐文件 SHA-256 比对一致，没有 reset、clean、pull 或覆盖
飞机已有 Odin/现场配置。Jetson ARM64 原生构建、19 项测试和无 MAVROS smoke 全部通过；安装器更新
unit 后重新启动四组件。独立视频仅在确认 stopped 后重启，最终仍为 stopped。

ARM64 上另启 domain 232、localhost-only、假 MAVROS 前缀的隔离节点，验证地面绝对时间从快 30 天
跳到慢 30 天：新鲜 `CLEAR_ABNORMAL` 被接受，回退 30 秒的命令被 TTL 拒绝，随后主动释放测试租约；
探针没有连接真实 MAVROS，也没有残留进程。

最终真实生产状态连续两次为：

- `fcu_connected=true`、`armed=false`、STABILIZE；
- 本地位置有效、消息频率和 `GUID_OPTIONS`/推力语义已确认；
- 无租约、无控制器、无 setpoint 冲突、无 failsafe；
- 控制频率约 100.03 Hz、最大抖动 4.67 ms、deadline miss 0；
- 飞控与视频 unit 均 active/running、`NRestarts=0`；
- 运行中机载二进制与磁盘 install 二进制 SHA-256 同为
  `80c91ca25d14fc93f9b6293ceae2ad802abb9dbabc7b477eaac964a251bd8cce`；
- 已部署 unit 不包含 `systemd-time-wait-sync` 或 `time-sync.target`。

## 安全边界与剩余验收

全过程没有向真机发送模式切换、解锁、起飞、降落、运动、航点、姿态、推力或飞控参数命令；没有
进行真实起飞。专项命令只发往 domain 232 的隔离节点。

本次没有物理断开 WAN 后冷重启飞机，因为这会扩大到网络/整机状态变更；“无外网冷启动”已由 unit
依赖静态验证、数十天注入偏差测试和 ARM64 隔离运行覆盖，但下一次真实离线现场开机仍应记录为最终
运维验收。若飞机离线开机后再接入互联网，应在用户人工解锁前等待 Linux 校时和 MAVROS TIMESYNC
重新稳定并复核 READY；外网时间不再是控制租约或起飞的前置条件。
