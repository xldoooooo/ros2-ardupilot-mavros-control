# 双机载安装器与系统 MediaMTX 简报（2026-08-20）

## 完成内容

- 视频安装器移动到 `video_service/deploy/install_onboard_video_service.sh`，机载 sparse checkout
  仍会随整个 `video_service/` 自动取得该入口。
- 新增 `src/onboard_control/deploy/install_onboard_service.sh`：在 Jazzy 目标上运行构建、单测、
  隔离 smoke，首次生成现场环境文件，安装四组件飞控 unit，完成只读 `start_drone_all.sh --check`
  后 `enable --now`。
- 飞控生产 unit 模板改为执行 `start_drone_all.sh`，统一监督 MAVROS、Odin、extnav 与
  onboard_control；不再把旧的单节点示例误当生产模板。
- 飞控安装器发现 `ros2-ardupilot-onboard.service` 已 active 时会直接拒绝，避免在未知飞行状态
  下自动停止或重启；所有检查均不包含解锁、起飞或飞行控制命令。
- 删除 `video_service/bin/` 下约 53 MiB 的 x86-64 MediaMTX 及其捆绑目录。代码、机载默认配置和
  控制器默认值统一依赖系统 `mediamtx`，标准路径为 `/usr/local/bin/mediamtx`。
- `.gitignore` 同时禁止新旧目录名下的 `bin/` 再次加入架构相关二进制。
- 视频安装器不再联网下载或 apt 修改系统；README 明确列出一次性 `apt install ffmpeg v4l-utils
  ca-certificates curl` 和官方 ARM64 MediaMTX v1.20.0 手工下载、SHA-256 校验、系统安装步骤。
- 根 README、机载部署指南、视频 README、sparse checkout 校验和 MEMORY 已同步更新。

## 验证

- 两个安装器、两个启动器和机载工作区脚本通过 `bash -n`；安装器 `--help` 可独立运行。
- `shellcheck` 与 `git diff --check` 通过。
- `tests/test_onboard_deploy.py`、`tests/test_camera_service.py`、
  `tests/test_onboard_video_service.py` 共 49 项通过，覆盖安装器隔离、四组件模板、系统 MediaMTX
  默认路径、摄像头后端和 ROS 视频消息链。
- 项目正式 Python 全量 `tests/`：164 passed。

## 未执行项

- 本次未连接或修改真机，未运行两个真实安装器，也未重启飞控或视频服务。
- 大文件已从当前工作树和下一次提交移除，但历史提交 `a37f6a9` 仍保存旧 blob；彻底缩小普通完整
  clone 需要另行批准历史重写和远端强制更新，本次未在脏工作树上执行该破坏性操作。机载文档的
  `--filter=blob:none` sparse clone 在新提交合入后不会为当前检出下载旧 blob。
- 飞控一键安装以已安装 ROS 2 Jazzy、MAVROS、Odin、extnav 和已接好且可唯一识别的飞控设备为
  前提；缺失或歧义会在 `--check` 阶段如实失败，不会自动猜测硬件路径。
