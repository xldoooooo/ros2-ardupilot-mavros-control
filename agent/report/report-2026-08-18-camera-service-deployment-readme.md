# 摄像头服务 Ubuntu 24.04 部署文档补充简报

日期：2026-08-18

## 完成内容

- 在 `video-service/README.md` 增加 Ubuntu 24.04 部署章节。
- 明确摄像头后端无需 ROS 2、CMake、colcon 或代码编译，只依赖 Python 标准库以及系统
  `ffmpeg`、`ffprobe`、`v4l2-ctl`。
- 分别给出无界面服务、独立摄像头面板和完整地面站三种安装/启动方式。
- 说明仓库内 MediaMTX 为 Linux x86-64 静态程序；ARM64 机器需要替换相同版本的对应二进制。
- 增加 V4L2 设备检查、`video` 组权限、首次配置位置、MJPEG 模式建议、UFW TCP 8554 和
  RTSP-over-TCP 客户端说明。

## 验证结果

- `git diff --check -- video-service/README.md`：通过。
- `python3 video-service/camera_service.py --help`：通过，文档列出的服务入口和子命令存在。
- `video-service/bin/mediamtx/mediamtx --version`：返回 `v1.20.0`。
- 本任务仅修改文档，没有启动摄像头、FFmpeg 推流、MediaMTX 服务或地面站，也没有进行任何
  飞控连接、解锁或起飞操作。

## 未验证范围

- 没有在第二台 Ubuntu 24.04 机器上实际执行全新安装。
- 没有下载或运行 ARM64 MediaMTX；ARM64 部署说明依据当前二进制架构边界给出。
