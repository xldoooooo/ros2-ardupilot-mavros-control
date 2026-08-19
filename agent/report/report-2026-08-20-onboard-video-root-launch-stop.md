# 机载视频根目录启停与彻底残留清理报告

- 日期：2026-08-20
- 真机：`nvidia@192.168.112.169`，Ubuntu 24.04 / Jazzy / aarch64
- 安全边界：未解锁、未起飞，未停止或重启飞控四组件

## 需求结论

直接执行启动脚本会创建当前终端拥有的普通进程，适合临时调试；systemd unit 额外固定运行用户、
工作目录、配置环境、开机启动、异常重启、journal 日志、信号和进程组回收。第 4 步安装的是
机器级配置和媒体目录，第 5 步才把视频节点注册为系统服务，两者职责不同。

部署文档中的 `'/video_service/'` 确实会让未来 Git sparse checkout 拉取整个目录。当前开发树
约 90 MB，其中约 53 MB 是 x86 MediaMTX、37 MB 是历史 demo；当前飞机只有约 440 KB，是因为
任务 22.5 实际采用选择性 rsync，并没有按该 Git 命令重新拉全目录。按用户要求，本轮只分析该
取舍，未擅自重构 sparse 目录；后续更合理的方向是按 common/onboard/ground 目录拆分后做目录级
sparse，而不是长期维护零散文件清单。

## 修改内容

1. 将 `video_service/start_onboard_video.sh` 移至项目根目录 `start_onboard_video.sh`。
2. systemd 模板改为执行 `${ONBOARD_WORKSPACE}/start_onboard_video.sh`。
3. sparse checkout 清单、布局校验、根 README、视频 README、机载部署指导和自动化测试全部同步
   新路径。
4. 新增根目录 `stop_onboard_video.sh`：
   - 默认先停止 `video-service.service`；
   - 查找手工启动或残留的 `start_onboard_video.sh`/`onboard_video_node.py` 及其后代；
   - 查找并清理配置 RTSP 端口拥有者；
   - 查找并清理明确配置的摄像头及 `/dev/v4l/by-id/*-video-index*` 占用者；
   - 依次使用 SIGINT、SIGTERM、SIGKILL，并最终验证无残留；
   - `--restart` 在同一彻底清理后只重启 `video-service.service`；
   - 代码和测试均禁止调用 `stop_onboard_service.sh` 或操作
     `ros2-ardupilot-onboard.service`。

## 真机部署与故障注入

目标 unit 已改为：

```text
ExecStart=/bin/bash -lc 'exec "${ONBOARD_WORKSPACE}/start_onboard_video.sh"'
```

旧子目录启动脚本已从飞机精确移除，根目录两个脚本均可执行。首先执行普通停止，unit 进入
inactive，残留视频节点清理完成。随后人为启动一个不受 systemd 管理、直接读取 Wasintek
H.264 1280×720@120 的 FFmpeg：

```text
orphan_pid=48658
/dev/video0 owner=48658
```

执行 `stop_onboard_video.sh --restart` 后，脚本显示该 PID 和命令行，以 SIGINT 清理成功，随后只
启动独立视频 unit。最终：

```text
ros2-ardupilot-onboard.service: active
video-service.service: enabled + active
FCU: connected, armed=false, guided=false, STABILIZE
VideoStatus: service_available=true, running=false, state=stopped
FFmpeg/MediaMTX: none
RTSP 8554: free
Wasintek camera: free
```

## 回归

```text
root launcher isolated domain 230 start/SIGINT: passed
tests/test_onboard_deploy.py: 14 passed
tests/: 161 passed in 40.05 s
bash -n: passed
git diff --check: passed
```

飞机原 unit 备份为：

```text
/etc/systemd/system/video-service.service.pre-root-launcher-20260820
```

未执行 reset、clean、commit、pull 或 push；既有 dirty worktree 保持不变。
