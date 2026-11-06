<!-- Fresh Jetson setup order, project prerequisites and delivery checks. -->
# 全新 Jetson Ubuntu 24.04 安装清单

本清单用于新的独立机载计算机。命令在目标普通用户下执行，先确认目标身份，不复用其他飞机
的硬件路径和标定。实机解锁、起飞只能由用户手动执行。

## 系统、网络与 Wi-Fi

1. 独立供电，确认数据 USB 或局域网连接。设备模式常用 `192.168.55.1`，多台 Jetson 可复用
   该地址，SSH 前按设备序列号确认身份，必要时使用独立 `HostKeyAlias`。
2. 检查 `hostname`、`/etc/os-release`、`uname -r`、`dpkg --print-architecture`、
   `/proc/device-tree/model`、路由及 DNS。不能假定开发机已配置 USB 网络转发。
3. apt 源改动前备份现有文件并保留 NVIDIA/L4T 源。ARM64 Ubuntu 源使用 `ubuntu-ports`，
   不复制 amd64 的源路径。先 `apt-get update` 和 `apt-get -s upgrade`，审查内核、NVIDIA
   包及删除清单后再升级；保持 SSH 会话和供电。最后执行 `dpkg --audit`，列明保留包和重启需求。
4. Wi-Fi 用 `lspci -nnk`、`dkms status`、`rfkill list` 定位；Intel 8265 + Tegra
   6.8.12-1021 的已知修复见 [Wi-Fi 指南](JETSON_WIFI_8265.md)。其他内核不能照搬强制安装步骤。
   重启后复核驱动、扫描和实际网络连接。

## 项目环境

按[机载部署](ONBOARD_DEPLOYMENT.md)检出 main，不复制开发机或旧飞机的构建产物。
在项目根目录执行：

```bash
./src/onboard_control/deploy/setup_onboard_dependencies.sh
python3 -m venv --system-site-packages .venv
```

安装内容、下载校验、离线 GeographicLib 数据和可选相机工具见
[依赖说明](ONBOARD_DEPENDENCIES.md)。当前下视相机使用项目自带 UVC 节点，常规部署不需要
`--with-camera-deps`；只有使用旧独立 GStreamer 标定工具时才追加该选项。
GitHub 下载受限时可使用临时 SSH 反向代理，端口按实际开发机调整，不写系统全局代理。

1. 在 `config/aircraft.local.env` 选择真实飞机编号，检查其 `onboard.env`、串口及波特率。
2. Jetson 厂商 OpenCV 4.8 须先准备匹配的 cv_bridge，保存 `ODIN_CV_BRIDGE_SETUP`，
   见 [OpenCV overlay](JETSON_OPENCV_OVERLAY.md)。已有兼容 overlay 可只读引用，不改旧工作区。
3. 项目 Odin 随 `drivers/odin/` 原生构建；extnav 从已授权的独立工作区准备，保存
   `EXTNAV_OVERLAY_SETUP`。下视相机不依赖外部相机驱动工作区。
4. 按[避障指南](../../avoidance_bridge/README.md)准备独立规划器，再构建主工程。
5. 核对用户在 `dialout,video,plugdev` 组，变更后重新登录。Odin USB 规则由飞控安装器安装，
   已插入的设备需重新插拔验证权限。

```bash
./src/onboard_control/deploy/build_onboard_control.sh --verify
./correction_service/deploy/install_extnav_correction.sh
```

构建、测试和 localhost 隔离 smoke 必须真实通过；不把无硬件的 smoke 当作串口或定位验收。
配置中的稳定设备路径、地图和标定须由本机实际接线及测量确认，不能沿用参考机假设。

## 安装服务

硬件未齐时，只安装不启用：

```bash
./src/onboard_control/deploy/install_onboard_service.sh --install-only
./video_service/deploy/install_onboard_video_service.sh --install-only
./correction_service/deploy/install_correction_service.sh --install-only
```

该选项仍会构建和核验，但不启用或启动 unit。硬件及 extnav 齐备后，按各自 README 验证
飞控、视频和修正，再使用默认安装入口。三个服务保持独立；相机 `device=auto` 不代表验收通过。
当前正式媒体采用机载私有缓存和地面集中媒体，配置见[视频指南](../../../video_service/README.md)。

## 可选桌面与远程访问

常用工具可按需安装 `openssh-server vim vlc qv4l2 libreoffice`。
中文输入使用 `fcitx5 fcitx5-chinese-addons fcitx5-config-qt` 和对应桌面前端，
通过 `im-config -n fcitx5` 选择并重新登录，在配置界面设置快捷键并实测。
VS Code、NoMachine 等必须选择 ARM64 安装包，安装前用 `dpkg-deb -f` 核对版本和架构；
如需与参考机一致，先确认参考机实际版本。

GNOME“桌面共享”依赖已登录的用户图形会话，与系统级“远程登录”不同；
在目标用户会话设置凭据、TLS 和共享权限，检查 `grdctl status`，再从地面站实际认证。
需要无人值守桌面时单独配置自动登录和密钥环解锁，保留原用户密钥环，不直接覆盖其配置。
仅 unit enabled 或设置开关为 true 不代表 RDP 可连接。

## 交付检查

- `dpkg --audit`、软件架构及保留更新/重启事项已记录。
- Wi-Fi 在重启后仍可扫描和连接；桌面工具、中文输入和远程访问按实际需求完成实测。
- 主工程原生构建、测试、隔离 smoke 通过，缺失的硬件、extnav、相机逐项列明。
- 无外设的新机保持服务 disabled/inactive；接线后的台架检查保持拆桨、未解锁。
- 报告放入 `agent/report/`，`MEMORY.md` 只维护当前基线及未解决风险，不复制安装过程日志。

```bash
systemctl is-enabled ros2-ardupilot-onboard.service video-service.service odin-correction.service
systemctl is-active ros2-ardupilot-onboard.service video-service.service odin-correction.service
```

未启用阶段返回非零并显示 `disabled/inactive` 属于预期状态。
