# 真机视频独立启停与手工启动配置一致性修复简报

## 故障结论

本次真机无法从地面摄像头面板开启由两个问题叠加造成：

1. 面板调用 `/onboard_control/set_video_state`，而飞机的
   `ros2-ardupilot-onboard.service` 已停止并处于 failed；独立视频节点仍可发布
   `VideoStatus`，所以面板持续显示真实的 `stopped`，但启停请求没有接收者。
2. 用户在终端直接运行旧 `start_onboard_video.sh` 后，进程没有 systemd unit 提供的
   `VIDEO_SERVICE_ONBOARD_CONFIG` 环境变量，回退到飞机仓库内旧配置，并再次寻找已删除的
   `video_service/bin/mediamtx/mediamtx`。飞机的 `/usr/local/bin/mediamtx` 和
   `/etc/ros2-ardupilot/camera.conf` 实际均正确。

以上问题与曝光参数改为摄像头打开后设置无关。

## 实现内容

- 独立视频节点新增 `/video_service/set_video_state`，复用 `SetVideoState` 协议，在节点内进行
  TTL、来源和序号重放检查后把最新期望状态交给已有串行视频队列；响应只表示已排队，不伪称
  摄像头已经启动。
- 地面摄像头面板的 `OnboardVideoClient` 改为直接调用上述端点，不再要求 onboard_control、
  飞行租约或 FCU 在线。飞控节点原有 `/video_service/control` 自动起降发布和
  `/onboard_control/set_video_state` 兼容代理均保留。
- `start_onboard_video.sh` 统一配置优先级：显式环境变量 > `/etc/ros2-ardupilot/` 系统配置 >
  仓库默认配置；显式文件不可读时明确失败，不静默换用另一套配置。
- 视频 README、接口注释、部署测试、ROS 集成测试与 MEMORY 已同步更新。

## 真机部署与恢复

目标为 `nvidia@192.168.112.169`，飞控 unit 全程保持原有 failed 状态，没有启动、重启、解锁或
起飞。清理了终端手工启动的孤立视频节点 PID 182500，并恢复独立
`video-service.service`。

首次选择性同步只替换了新视频节点和控制器，飞机旧 `camera_app/config.py` 缺少当前常量，
systemd 因 `ImportError` 发生自动重启。发现后立即停止视频 unit，补齐同一版本的 `config.py`
与 `onboard_config.py`，再次启动成功。最终状态：

```text
enabled
active
MainPID=189084
NRestarts=0
ActiveState=active
SubState=running
VIDEO_SERVICE_ONBOARD_CONFIG=/etc/ros2-ardupilot/camera.conf
VIDEO_SERVICE_LENS_CONFIG=/etc/ros2-ardupilot/lens.conf
```

旧文件临时备份位于飞机 `/tmp/codex-video-independent-fix-20260820`，未删除。

## 真机闭环结果

在 onboard_control 不在线的情况下，直接使用地面面板同一个 `OnboardVideoClient`：

1. `/video_service/set_video_state` 返回“视频开启期望状态已排队”；
2. `VideoStatus` 转为 `running`，FFprobe 实测 H.264 1280×720@120；
3. V4L2 读回 `auto_exposure=1`、`exposure_time_absolute=25`、`gain=200`；
4. 人工抓拍生成 1280×720 JPEG；
5. 关闭请求返回成功，状态回到 `stopped`，录像正常封装为 24.1 秒 H.264 MP4；
6. FFmpeg、MediaMTX、8554 监听和 `/dev/video0`、`/dev/video1` 占用均释放，视频 ROS 节点
   继续由 systemd 常驻等待下一条命令。

测试媒体：

- `/home/share/recording-20260820-204742-418622.mp4`
- `/home/share/jpg/snapshot-20260820-204806-224343-manual-1.jpg`

手工启动验证先停止 systemd unit，再在没有两个视频配置环境变量时运行根脚本；bash trace 明确
选择 `/etc/ros2-ardupilot/camera.conf` 和 `lens.conf`，节点正常启动。5 秒 timeout 发送 SIGINT
后无残留进程，随后 systemd 已恢复 active。这证明手工与系统启动的配置和运行逻辑一致，区别
只剩前台进程由终端管理、systemd 进程由系统管理。

## 自动测试

- `bash -n start_onboard_video.sh`：通过；
- 相关 ROS、面板、摄像头与部署测试：51 passed；
- source ROS 2 Jazzy 与工作区 overlay 后，`pytest -q tests`：166 passed；
- `git diff --check`：通过。

本次没有启动或修改飞控服务，没有执行真机解锁或起飞。
