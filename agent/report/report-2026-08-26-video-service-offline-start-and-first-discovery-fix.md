# 视频服务离线启动与面板首次发现竞态修复简报

## 任务结论

本次按最小范围完成两项修复：

1. `video-service.service` 不再等待 `systemd-time-wait-sync.service` 或
   `time-sync.target`，与在终端直接执行根目录 `start_onboard_video.sh` 一致；局域网可用但没有
   外网 NTP 时，视频 ROS 节点不再被无限校时等待阻塞。
2. 地面摄像头面板首次提交启停命令时，对 `/video_service/set_video_state` 做最多 2 秒的有限
   服务发现等待，避免新建 DDS participant 后第一次 50 ms 探测窗口造成误报。

用户明确要求暂不处理 Wi-Fi 启动竞态，因此本次没有增加固定 sleep、网卡/IP 等待、NetworkManager
依赖或 systemd 自动重启策略。

## 根因与边界

飞机当前 `systemd-time-wait-sync.service` 为 `Type=oneshot` 且
`TimeoutStartSec=infinity`。旧视频 unit 同时声明外网校时依赖，所以无外网、无局域网 NTP 时不会
执行启动脚本；手工运行脚本绕过 unit 排序后能够立即启动，符合现场现象。

从 `scq@192.168.112.101` 对开机自启动实例检查时，当前有外网校时的启动未复现长期不可达：服务
匹配成功，10 秒收到 11 条接口 3.2 的 `VideoStatus`。但独立测量 10 个新建客户端时，首次 50 ms
检查 10/10 均未就绪，实际发现耗时为 0.13～0.41 秒，证明面板原有一次性检查存在确定竞态。

另确认当前飞机 `network-online.target` 在开机约 6.39 秒完成，而 Wi-Fi 到约 10.17 秒才获得
`192.168.112.169`，且 `NetworkManager-wait-online.service` 被 masked。删除校时依赖后，DDS 仍有
可能在 Wi-Fi 地址出现前初始化；这是已知但按本次要求明确保留的风险，不能把本次修复描述为已经
覆盖真实断 WAN 冷启动。

## 代码修改

- `video_service/deploy/video-service.service.example`
  - 只保留 `Wants=network-online.target` 和 `After=network-online.target`；
  - 新增注释说明视频局域网服务不等待外网校时。
- `video_service/camera_app/ros_client.py`
  - 新增 2 秒服务发现上限；
  - 首次命令改用 `wait_for_service()`，超时后继续返回原有显式错误。
- `tests/test_onboard_deploy.py`
  - 锁定视频 unit 不得再次引入两个校时依赖。
- `tests/test_onboard_video_service.py`
  - 覆盖客户端先提交命令、服务端 150 ms 后出现的首次发现时序。

## 验证结果

- 专项测试：3 passed。
- 完整 `tests/`：182 passed，耗时 42.68 秒。
- 首次发现回归测试连续独立执行 10 次：全部通过。
- `bash -n`：视频启动器与安装器通过。
- `systemd-analyze verify`：新 unit 通过；仅输出飞机既有 NVIDIA unit 的 obsolete syslog 警告。
- `git diff --check`：本次修改无格式问题；命令仍报告用户既有 `TODO.md:100` 尾随空格。
- Ruff 对修改范围报告 12 条既有问题，主要是原文件导入排序、宽泛异常捕获和清理阶段静默异常；
  本次为保持最小范围未顺带重构，不影响上述 pytest 结果。

## 真机部署

部署前再次确认飞机 `armed=false`、FCU connected、STABILIZE，视频为 stopped。只更新并重启了
`video-service.service`：

- 备份：`/home/nvidia/backups/video-service-pre-offline-start-fix-20260826-2115.service`；
- 飞控 unit MainPID 在部署前后均为 `2455`，未重启；
- 视频 MainPID 从 `5038` 更新为 `17896`；
- 视频 unit 最终 enabled、active、`NRestarts=0`、`Result=success`；
- 实际安装 unit 已确认不含 `systemd-time-wait-sync` 或 `time-sync.target`。

全过程没有调用模式切换、解锁、起飞、降落、运动、航点、姿态、推力或飞控参数接口。
