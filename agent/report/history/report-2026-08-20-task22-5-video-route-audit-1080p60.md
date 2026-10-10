# Task 22.5 视频链路边界审计与 1080p60 真机复验简报

## 结论

已按任务 22.5 和用户的最新明确边界完成修正：

| 业务 | 地面端路由 | 机载路由 | 结果 |
| --- | --- | --- | --- |
| 手动开启/关闭 | 摄像头面板直接调用 `/video_service/set_video_state` | video_service | 符合 |
| 手动抓拍 | 摄像头面板直接发布 `/video_service/capture` | video_service | 符合 |
| 起飞/空中自动开启 | 飞行业务进入 onboard_control | onboard_control 根据 MAVROS ExtendedState 发布 `/video_service/control` | 符合 |
| 真实到达航点后抓拍 | 航点任务进入 onboard_control | onboard_control 在到达终态、递增航点索引前发布 `/video_service/capture` | 符合 |
| 落地/解除武装后关闭 | 飞行业务进入 onboard_control | onboard_control 发布 `/video_service/control` | 符合 |

已删除 onboard_control 中冗余的 `/onboard_control/set_video_state` 代理。该代理虽已不被当前面板使用，
但仍允许纯视频命令绕经飞行节点，与本次明确的两条业务链边界不一致。修正后 onboard_control 只保留
飞行边沿事件和航点到达事件的视频消息发布能力。

## 实现变更

- 从 onboard_control 移除 `SetVideoState` service、回调、重放序号状态和相关声明。
- 保留 reliable + transient-local 的 `VideoControl` 自动状态发布和 reliable + volatile 的航点抓拍发布。
- 更新 ROS 接口注释、单测和部署文档，明确面板直连 video_service，onboard_control 不提供手动视频代理。
- 代码默认值、机载配置和文档默认值统一改为 H.264 1920×1080@60。
- 其余视频参数保持不变：MP4、RTSP 8554/camera、`/home/share`、`/home/share/jpg`、手动曝光 25、增益 200。

## 本地回归

- `colcon build --packages-select guided_interfaces onboard_control --cmake-args -DCMAKE_BUILD_TYPE=Release`：通过。
- `pytest -q tests`：166 passed。
- `colcon test --packages-select guided_interfaces onboard_control`：19 tests，0 errors，0 failures，0 skipped。
- 新测试会启动隔离的 onboard_control，明确断言手动代理不存在，并验证 ExtendedState
  `ON_GROUND -> IN_AIR -> ON_GROUND` 仍产生 `false -> true -> false` 的飞行相关视频期望。

## 真机部署与复验

目标是 `nvidia@192.168.112.169`（Ubuntu 24.04/Jazzy/aarch64）。未解锁、未起飞，且没有启动正常飞控栈。

- 已同步改动源码、配置与部署文档，7 个关键文件的 SHA-256 与本地完全一致。
- 真机 ARM64 Release 重建 `guided_interfaces` 和 `onboard_control` 通过。
- 在 `ROS_DOMAIN_ID=231` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 中隔离启动 onboard_control：
  - `/_route_audit_onboard/set_video_state` 数量为 0；
  - 伪造 `LANDED_STATE_IN_AIR` 后收到 `enabled: true`、`source_id: onboard-flight`；
  - 隔离进程已完整清理，未与真实 FCU 交互。
- 重启且只重启了独立 `video-service.service`，手工启动脚本与 systemd 均使用
  `/etc/ros2-ardupilot/camera.conf` 和 `lens.conf`。
- 真实摄像头手动直连测试：
  - `/video_service/set_video_state` 存在，`/onboard_control/set_video_state` 不存在；
  - RTSP 和驱动均为 H.264 1920×1080@60；
  - 开启后镜头读回 `auto_exposure=1`、`exposure_time_absolute=25`、`gain=200`；
  - MP4：`/home/share/recording-20260820-211256-125552.mp4`，32.166666 秒，15,471,611 bytes，H.264 1920×1080@60；
  - JPG：`/home/share/jpg/snapshot-20260820-211327-959172-manual-1.jpg`，36,115 bytes，1920×1080；
  - 关闭后状态为 stopped，FFmpeg、MediaMTX、8554 端口和摄像头均无占用；
  - `video-service.service` 保持 active/running，`NRestarts=0`。

上述两个媒体文件保留在真机上作为本次验收证据。

## 未验证边界

仍未执行真实起飞后自动开启、真实飞抵航点后自动抓拍、真实落地后自动关闭。
这三项只完成了代码审计、自动测试和隔离 ROS 消息仿真，不冒充真实飞行结论；后续必须由用户人工解锁、起飞后补验。

真机的 `ros2-ardupilot-onboard.service` 在本次开始前已为 failed，本次未启动、未重启、未修改该 unit；
不将这个既存状态误报为本次视频修改的故障或验收结果。
