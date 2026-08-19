# 机载视频一键安装简报（2026-08-20）

## 结果

- 新增根目录 `install_onboard_video_service.sh`，将原先需人工逐条执行的系统依赖、ARM64
  MediaMTX、配置、媒体目录、systemd unit 生成和 `enable --now` 收敛为一条命令。
- 安装器重复执行时保留已有 `/etc/ros2-ardupilot/camera.conf` 与 `lens.conf`，避免覆盖现场已核对
  的设备路径、网络地址和镜头参数。
- unit 增加 `SupplementaryGroups=video`，摄像头权限不再依赖部署用户重新登录刷新附加组。
- 一键安装器已加入机载 sparse checkout 清单；根 README、视频 README 和机载部署文档均已更新。
- 视频 unit 仍与 `ros2-ardupilot-onboard.service` 完全独立；安装器不操作飞控服务，也不发送任何
  飞行命令。开机只启动等待命令的视频节点，不会直接开启摄像头。

## 验证

- `bash -n` 检查一键安装、视频启动、视频停止及机载工作区脚本。
- 运行机载部署与视频 ROS 相关 pytest 共 19 项，全部通过；覆盖可执行权限、帮助文本、配置保留、
  systemd 安装动作、sparse checkout 完整性、飞控生命周期隔离和视频节点退出/消息链。

## 未执行项

- 本次只调整仓库部署入口，没有在当前真机重复运行安装器或重启服务，避免无必要改变已验证的
  运行现场。
- 安装器面向当前明确的 Ubuntu 24.04/Jazzy/ARM64 机载基线；其他架构会明确拒绝，不能把仓库中
  的 amd64 MediaMTX 误装到飞机。
