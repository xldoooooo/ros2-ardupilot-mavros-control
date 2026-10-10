# 任务 22.5：新机载计算机未武装真机补测报告

- 日期：2026-08-19
- 飞机伴随计算机：`nvidia@192.168.112.169`
- 硬件：NVIDIA Jetson Orin NX / aarch64
- 系统：Ubuntu 24.04.4 / ROS 2 Jazzy
- 摄像头：Wasintek `2aad:6373`，连接伴随计算机 Type-C
- 任务范围：补齐不需要真实起飞的任务 22.5 真机部署与台架测试
- 安全结论：全程没有调用解锁、起飞、航点或飞行控制服务；开始、过程和结束均权威确认
  `armed=false`。

## 一、最终结论

任务 22.5 的 ARM64 部署、真实摄像头、跨机 RTSP、录像、三类 JPG、镜头参数、地面面板、
ROS 视频代理、状态陈旧判定、进程故障隔离和媒体错误恢复均已在新机载计算机通过。

本轮真机发现并修复了两个只有目标运行时才暴露的问题：

1. ArduPilot 默认不发送 `EXTENDED_SYS_STATE(245)`，导致 `/mavros/extended_state`
   只有发布端而没有样本，非地面站起飞边沿无法检测。机载端现自动请求 245 号消息 2 Hz；
2. Jazzy 收到 SIGINT 后会先关闭 rclpy context，视频节点再次调用 `shutdown()` 会让正常
   systemd stop 错误退出为 1。现已改为幂等关闭，目标机实测 stop 为 `0/SUCCESS`。

最终飞机状态：

```text
FCU connected: true
armed: false
mode: STABILIZE
interface: 3.2
control mode: idle
controller active: false
lease active: false
active command sequence: 0
message rates configured: true
setpoint conflict: false
failsafe reason: empty
control rate: 100.0256 Hz
ExtendedState: ON_GROUND
video service: available, stopped, no error
```

`ros2-ardupilot-onboard.service` 和 `video-service.service` 最终均为 enabled + active；
摄像头、FFmpeg、MediaMTX 和 8554 端口均未占用，生产 `/home/share`、`/home/share/jpg`
最终为空。

## 二、部署前安全核验

首次连接只做只读检查：

- 目标为 Jetson Orin NX、Linux `6.8.12-1021-tegra`、aarch64；
- 原工作区 `/home/nvidia/ros2-ardupilot-mavros-control` 的 source/install 均为接口 3.1；
- 原 `ros2-ardupilot-onboard.service` 没有进程，状态显示 failed，但日志表明它此前已达到
  `READY: FCU connected, unarmed`，随后收到人工/systemd SIGINT；
- 短暂启动原 3.1 服务，只订阅状态，取得：FCU connected、`armed=false`、STABILIZE、
  无租约、控制器未激活；随后立即停止；
- 摄像头稳定路径为
  `/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0`，无人占用；
- 目标机此前没有 FFmpeg、v4l-utils、MediaMTX、`/home/share` 或视频服务。

部署前建立了可回滚备份：

```text
/home/nvidia/backups/task22_5-predeploy-20260819-2317.tar.gz
SHA-256: a070336413b6308db55a2155526be21c87f11fb249ccaca031ca95847697b27c
```

备份包含原 3.1 两个 ROS 包、飞控 systemd unit 和机载环境文件。原工作树中的
`image/cam_in_ex.txt` 与 `src/odin_ros_driver/` 是 Odin 自动生成内容，本次没有 reset、clean、
覆盖或删除。

## 三、ARM64 部署

### 3.1 软件与配置

通过目标机 apt 安装：

- NVIDIA Jetson FFmpeg `n8.0.1-9-g90b8004959-1ubuntu0.1`；
- v4l-utils `1.26.1-4build3`。

飞机无法直连 GitHub，故在开发机从 MediaMTX 官方 v1.20.0 发布页下载 ARM64 归档，按官方
摘要校验后传入目标机：

```text
archive: mediamtx_v1.20.0_linux_arm64.tar.gz
official archive SHA-256: 6aa3c03da7b6477f1e110c8e18e819cf9ef121e8981b52b8f8219982dae35f2f
installed: /usr/local/bin/mediamtx
installed binary SHA-256: 2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8
file: ELF 64-bit ARM aarch64, statically linked
version: v1.20.0
```

安装并启用：

- `/etc/ros2-ardupilot/camera.conf`；
- `/etc/ros2-ardupilot/lens.conf`；
- `/etc/systemd/system/video-service.service`；
- `/home/share`、`/home/share/jpg`，均为 `nvidia:nvidia`、0755；
- 根目录 `video_service/` 及接口 3.2 两个 ROS 包。

视频 unit 与飞控 unit 没有 `Requires`、`PartOf` 或共同监督关系。视频仍只由自己的 systemd
unit 管理，没有加入 `start_drone_all.sh`。

### 3.2 构建与版本

目标机执行两轮：

```bash
./build_onboard_control.sh --verify
```

修复前后结果均为：

- ARM64 Release 构建成功；
- `19 tests, 0 errors, 0 failures, 0 skipped`；
- domain 231、localhost-only、无 MAVROS smoke 通过；
- smoke 为接口 3.2、`fcu_connected=false`、`armed=false`、姿态 setpoint 0 条。

最终 source、install 与运行进程均报告 3.2/3.2.0。

## 四、目标摄像头能力与镜头参数

目标摄像头声明的主要模式与开发机同型号一致：

- H.264：3840×2160@30、1920×1080@60/30、1280×720@120/60/30 等；
- MJPEG：1920×1080@60/30、1280×720@120/60/30；
- MJPEG 3840×2160 与 4048×3040 仍受 RTP/JPEG 2040 像素限制，不用于 RTSP。

真机 H.264 启动后逐项读回：

```text
auto_exposure=1 (Manual Mode)
exposure_time_absolute=25
gain=200
brightness=6
contrast=6
saturation=6
hue=0
sharpness=6
power_line_frequency=1
zoom_absolute=10
```

十项值与任务配置完全一致，说明每次启动前的镜头配置重放在目标驱动上生效。

## 五、真实 H.264 测试

正式配置为 H.264 1920×1080@30、MP4、RTSP/TCP 8554：

- `VideoStatus` 自动发布 `rtsp://192.168.112.169:8554/camera`；
- 飞机本机和开发机跨 Wi-Fi 的 FFprobe 都识别 H.264、1920×1080、30/1；
- 两端各实际解码 90 帧成功；
- 主 H.264 台架 MP4 为 83.333333 秒、218,124,940 bytes；
- 停止后没有 `.partial`，摄像头与 8554 端口释放。

同一运行周期快速发布三条抓拍，全部逐条返回成功：

| 来源 | 实际文件 | 大小 | 尺寸 |
|---|---|---:|---:|
| 面板人工 | `...-manual-1.jpg` | 366,563 bytes | 1920×1080 |
| 我方航点 | `...-gcs-1.jpg` | 366,630 bytes | 1920×1080 |
| 甲方航点 | `...-photoNo-%E5%AE%A2%E6%88%B7%2FA%2001.jpg` | 366,789 bytes | 1920×1080 |

第三条请求中的 `photoNo` 为原始字符串 `客户/A 01`，结果消息仍逐字回显该值，文件名为可逆
URL 编码，`waypoint_index` 仍是独立的 1-based 航点编号。

## 六、真实 MJPEG 测试

临时切换为 MJPEG 1280×720@30、MKV：

- 飞机与开发机识别 MJPEG、1280×720、30/1；
- 跨机解码成功；
- `manual-2.jpg` 为 182,986 bytes、1280×720；
- MKV 为 56.066 秒、277,670,141 bytes、30 fps；
- FFprobe 仅报告厂商 JPEG APP 字段的已知非致命告警，码流和文件可读；
- 停止后没有 partial、端口或设备占用。

测试后 `/etc/ros2-ardupilot/camera.conf` 已恢复正式 H.264 1920×1080@30 + MP4 配置。

## 七、地面面板真实跨机测试

开发机实际使用 `OnboardVideoClient(domain_id=0)`：

- 自动读取接口 3.2、真机 RTSP 地址和媒体目录；
- 通过 `/onboard_control/set_video_state` 开启视频，响应仅为“期望状态已发布”；
- 全程没有申请飞行租约，`lease_owner=''`、`lease_active=false`、飞行命令序号 0；
- 发布人工抓拍后，实际得到 `manual-3.jpg`，371,233 bytes、1920×1080；
- 关闭客户端不发送 stop，视频继续运行；
- 新建第二个客户端具有不同 UUID `source_id`，序号从 1 开始仍能成功关闭视频，没有被旧客户端
  的 replay 状态误判。

真实 Qt `CameraPanelWindow` 使用指定 RTSP 模式：

- 收到 60 个有效 1920×1080 帧，状态为 Playing；
- 显示约 27 fps 和 2 秒播放时长；
- 点击暂停后 1.5 秒内帧计数保持 30、播放时长保持 1.35 秒；
- 按空格恢复后帧计数从 30 增至 50，状态回到 Playing；
- 关闭窗口没有停止真机摄像头。

Qt/FFmpeg 首次尝试非 TCP transport 时打印一次 `461 Unsupported Transport`，随后自动回退并正常
播放；无播放器错误回调，实际帧持续到达。

## 八、ROS、系统服务与故障隔离

### 8.1 `EXTENDED_SYS_STATE` 真机修复

初次运行 3.2 时 `/mavros/extended_state` 有 MAVROS publisher 和 onboard subscriber，但 10 秒
内没有任何样本。手工调用一次只读遥测频率服务：

```text
message_id=245, message_rate=2.0 -> success=true
```

随后立即收到 `landed_state=ON_GROUND`，并观察到机载端发布
`enabled=false / 检测到飞行器位于地面`。因此将 245 加入现有自动消息频率数组；修复版重启后：

- `message_rates_configured=true`；
- 4 秒窗口收到 12 条 ON_GROUND 样本；
- 自动视频状态为关闭；
- 最终真实状态始终 `armed=false`。

目标机另在 domain 225 + localhost-only 运行编译后的节点和假 ExtendedState，观测到严格顺序：

```text
ON_GROUND -> enabled=false
IN_AIR   -> enabled=true
ON_GROUND -> enabled=false
```

该隔离测试没有接触真实 MAVROS、FCU 或飞行器运动。

### 8.2 独立视频停止

客户端先收到新鲜 `VideoStatus`，随后停止 `video-service.service`：

- 4.37 秒后客户端状态变为 `service_available=false, state=stale`；
- 飞控四组件仍 active；
- 真实状态仍 `armed=false`、无租约、无控制器、约 100.01 Hz；
- 摄像头、端口和媒体进程均释放。

修复 rclpy 双重关闭后，再次执行 systemd start→stop：

```text
Main PID: code=exited, status=0/SUCCESS
Active: inactive (dead)
video-service.service: Deactivated successfully
```

随后服务已重新启动并保持 enabled + active。

### 8.3 媒体失败与恢复

临时把设备设为 `/dev/task22_5_missing_camera`，通过真实无租约代理请求开启：

- 代理请求 accepted，只表示消息已发布；
- `VideoStatus` 进入 error，明确报告“摄像头设备不存在”；
- 视频 ROS 节点仍 active；
- 飞控仍 active、`armed=false`、约 100.02 Hz、无 failsafe、无飞行命令；
- 没有 FFmpeg、MediaMTX、设备或端口残留。

恢复正式配置后，不重启视频节点即可再次开启、录像、停止并回到 `state=stopped`、空错误。

在摄像头停止时从开发机发布人工抓拍，真机逐条返回：

```text
success=false
path=''
message='摄像头未运行，无法保存图片'
```

这验证失败也有明确结果，且不会污染飞行状态。

## 九、自动化回归

最终开发机：

```text
pytest tests/: 161 passed in 40.28s
colcon: 19 tests, 0 errors, 0 failures, 0 skipped
```

新增/更新回归包括：

- 自动消息频率集合包含 `EXTENDED_SYS_STATE(245)` 且请求 2 Hz；
- 视频主进程收到 SIGINT 后退出码必须为 0；
- 面板真机客户端取消继承的 `ROS_LOCALHOST_ONLY`，使用 Jazzy
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 且恢复原环境。

目标机两轮 `./build_onboard_control.sh --verify` 也均为 19 tests 零失败和安全 smoke 通过。

## 十、测试证据与清理

真实媒体没有删除，已从生产目录移动到：

```text
/home/nvidia/task22_5-bench-artifacts-20260819/video/
/home/nvidia/task22_5-bench-artifacts-20260819/jpg/
```

其中保留 4 个录像和 5 个 JPG。下载归档、解压目录及任务临时文件已清理；生产媒体目录为空，
视频节点重启后 `current/last video/image path` 均为空。

飞机工作区仍保留用户/Odin 原有改动和本次逐文件同步内容。当前 Git HEAD 是 `6a40713`，接口
3.2 与 `video_service/` 尚未形成远端可拉取提交；本次没有擅自 reset、clean、commit、pull 或
push。后续代码同步必须先处理这项工作树状态，不能覆盖已验证部署。

最终把当前 Jetson/Jazzy/ARM64 基线同步到机载运行文档，开发机与飞机摘要一致：

```text
ONBOARD_DEPLOYMENT.md SHA-256: 3622b1ba472f4284b383c8ad4cb8f0b179e04e095a13a9f836fd6318d79b4c1d
video_service/README.md SHA-256: e82dce4b13b0de460f14eaab6bc28a3980dd697d1c924e7edd7387adeb915e81
```

同步后最后一次只读复核仍为两项 unit enabled + active、`armed=false`、STABILIZE、无租约、
无控制器、无 failsafe、`landed_state=ON_GROUND`、视频 stopped，摄像头和 8554 端口空闲；
修改范围 flake8/compileall/bash 语法/`git diff --check` 及 5 项针对性 ROS 回归均通过。

## 十一、明确未执行的项目

以下项目需要真实飞行，因此按安全约束没有执行：

1. 实际起飞边沿自动开启视频；
2. 实际落地/解除武装边沿自动关闭并封装录像；
3. 实际飞抵巡检航点后由 `onboard_control` 自动触发 JPG；
4. 真实巡检完成后甲方 08/09 的飞行端到端媒体路径。

这些逻辑已通过单元、SITL/隔离 ROS 和本轮真机非飞行组件验证，但不能写成真实飞行通过。未来
若补测，解锁与起飞仍只能由用户本人手动执行。

本轮也没有重启整台 Jetson；systemd enable、人工 start/stop/restart 和进程状态均已验证，但未
用一次非必要整机重启中断当前飞控与网络会话。

## 十二、任务外观察项

最终 `systemctl --failed` 仍有两项本机既有失败：

- `load-iwlwifi.service`；
- `nvpmodel.service`。

当前 Wi-Fi、视频和飞控测试均正常，但本次没有扩大范围修复。Odin launch 在无显示环境仍尝试
启动 RViz 并报 Qt platform plugin 错误，四组件主服务仍能达到 READY；同样留作独立运维任务。
