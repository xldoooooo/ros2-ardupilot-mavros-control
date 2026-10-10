<!-- 本报告记录独立 USB Jetson 的实际配置、验证证据和未完成项，不代表飞机硬件验收。 -->
# 独立 USB Jetson Ubuntu 24.04 环境配置

日期：2026-09-16。目标是全新独立计算机，未连接 Odin、相机、飞控或其它外设。
USB 序列号 `1424324322770`，VID/PID `0955:7020`，地址 `192.168.55.1`，
SSH 别名校验标识 `jetson-usb-1424324322770`；账号 nvidia。
硬件为 Orin NX，Ubuntu 24.04.4、aarch64、Jetson Linux 39.2.1、内核 6.8.12-1021-tegra。
参考机 `192.168.112.169` 仅按用户授权只读查询、导出源码，未修改其服务或文件。
全程未发送解锁、起飞或 setpoint 指令。

## 六项任务结果

| 项目 | 实际结果 |
| --- | --- |
| apt 镜像和升级 | 完成。相同 ARM64 Packages.xz 三轮：清华 3.350/3.636/4.051 秒，中科大 0.687/1.299/1.489 秒。选 USTC ubuntu-ports，保留 Jetson 源，备份原 sources.list。普通 upgrade 更新 250 包，with-new-pkgs 再更新 2 包并安装 19 个拆分固件依赖。 |
| WiFi | 完成识别修复及重启验证。Intel 8265 驱动缺失及压缩固件不兼容分别修复；iwlwifi 自动加载，wlP1p1s0 可扫描 2.4/5 GHz，rfkill 无阻止。未配置热点凭据，未声称无线互联网已验证。 |
| 机载环境 | 完成无外设条件下的软件安装和原生构建验证。先尝试原一键脚本，真实失败于 ROS Jazzy 缺失；补齐脚本并实际执行。三项服务已安装且 disabled/inactive。 |
| 中文输入法 | 完成。与开发机相同 Fcitx5 5.1.7、拼音和 keyboard-us，复制配置，Ctrl+Shift 切换。真实 RDP 桌面输入保存“你好 hello”，重启后 Fcitx5 自动启动。 |
| 指定桌面软件 | LibreOffice、OpenSSH server、VS Code、Vim、VLC、qv4l2 已安装匹配架构。**NoMachine 未完成**：开发机 9.8.2-1 为 amd64，尚未取得同版本 ARM64 包，未安装最新版。 |
| Remmina 远程桌面 | 完成 README 指定的用户桌面共享、关闭系统远程登录、空密码默认 Login 密钥环及 GDM 自动登录。RDP 192.168.55.1:3389，用户/密码按要求设为 nvidia。真实桌面交互及重启后认证均成功。 |

重启后 `dpkg --audit` 无输出，`systemctl --failed` 为 0，`/var/run/reboot-required` 不存在。
APT 剩余 4 项为分阶段发布延后：libnetplan1、netplan-generator、netplan.io、python3-netplan；没有强制绕过 phasing。
原先 nvpmodel/serial-getty 失败状态在此次重启后消失。

## 软件版本与构建

- libreoffice `4:24.2.7-0ubuntu0.24.04.6`，openssh-server `1:9.6p1-3ubuntu13.19`。
- code `1.138.0-1789458676` arm64，vim `2:9.1.0016-1ubuntu7.20`。
- vlc `3.0.20-3build6`，qv4l2 `1.26.1-4build3`。
- fcitx5 `5.1.7-1build3`，fcitx5-chinese-addons `5.1.3-1build3`。
- remmina `1.4.43+dfsg-0ubuntu0.24.04.2`，gnome-remote-desktop `46.3-0ubuntu1.2`。
- ROS 2 Jazzy / MAVROS 2.15.1；MediaMTX v1.20.0 官方 ARM64 包，校验官方 SHA256。
- 用户加入 dialout/video/plugdev，重启后组权限已生效；Odin USB 规则使用 plugdev/0660。
- 项目 `/home/nvidia/ros2-ardupilot-mavros-control`，Python 为 `.venv`（system-site-packages）。
- Odin `/home/nvidia/catkin_ws`；extnav `/home/nvidia/vrpn_mavros`；相机工作区 `/home/nvidia/vins_odin_calib/camera_ws`。
- 在 Odin overlay 原生编译官方 vision_opencv tag 4.1.0（提交 f5b738d9694f0cee5904440d03912fc249943f8a）的 cv_bridge；完整 ROS/main/Odin/extnav source 链验证仍选中此 overlay，Odin ldd 仅 OpenCV .408，无 .406 或缺失库。
- Wasintek 原生构建成功，无 ldd 缺失库，相关 GStreamer 插件存在；无相机，未进行真实采集或解码验收。

## 修补的一键部署疏漏

1. 新增 `setup_onboard_dependencies.sh`：支持 Ubuntu 24.04 amd64/arm64，验证 ROS key 指纹，处理重复 ROS 源，安装编译/运行依赖；可选 Odin/相机依赖；MediaMTX 官方校验；GeographicLib 下载超时及文件校验；用户设备组。
2. 三项服务安装器新增 `--install-only`，保留构建/测试/单元安装，跳过硬件检查、启用和启动，适配本次无外设新机。
3. 修复主项目构建与 correction 安装器混用 symlink 导致已有目录不能被覆盖：主项目显式 `AMENT_CMAKE_SYMLINK_INSTALL=OFF`，独立 extnav 保持原有模式。
4. Odin 厂商脚本发行版匹配遗漏 Jazzy，复现步骤明确 `-DBUILD_SYSTEM=ROS2`。
5. 修复 Odin 与 apt cv_bridge 分别链接 OpenCV 4.8/4.6 的实际冲突，采用独立 overlay，不替换 Jetson 系统 OpenCV。

网络受限时仅使用临时反向代理，重启后已关闭。SourceForge 未及时下载成功，GeographicLib 数据改为从开发机复制架构无关数据集；未把复制结果写成下载成功。
`egm96-5.pgm` SHA256 为 `c4b25a03ec5845cec4778a54b580aeda676363f2205a89137e8677b2337af3ec`。
参考机仅转移必要源码/厂商 ARM64 库，不转移 build/install、标定或记录数据。

## 验证证据

- 开发机项目环境：`pytest -q tests/test_onboard_deploy.py tests/test_correction_deploy.py`：23 passed。
- 目标机最终原生构建：4 packages finished；23 tests，0 errors，0 failures，0 skipped。
- 隔离 smoke：interface=3.2，fcu_connected=false，armed=false，setpoint_messages=0。
- 三项服务 `ros2-ardupilot-onboard`、`video-service`、`odin-correction` 重启后仍 disabled/inactive。
- Ctrl+Shift 真实切换拼音/英文，保存文件 `/home/nvidia/input-method-validation.txt` 内容“你好 hello”；过程截图在本地 `agent/codex/jetson-bootstrap/rdp-input.png`。
- 重启后默认 Secret Service keyring Locked=false，seat0 图形会话 active，用户 RDP active，3389 监听，FreeRDP auth-only 进程退出码 0。
- WiFi 重启后 driver=iwlwifi，无线扫描正常；不是仅手工 modprobe 后的临时状态。

## 未完成项与后续边界

NoMachine 9.8.2-1 ARM64 包尚缺。旧官方 URL `https://download.nomachine.com/download/9.8/Arm/nomachine_9.8.2_1_arm64.deb` 及变体实际重定向到网页，未返回 deb；参考机也未安装且未找到缓存。需要提供可信的 `nomachine_9.8.2_1_arm64.deb`，校验包名、版本和架构后再安装。不以 amd64 包、最新版或重新打包冒充完成。

没有外设，因此飞控串口、Odin 数据、相机画面、标定和完整服务联调均不在本次已验证范围。配置中 overlay 路径已设置，实际接线后仍需核实设备路径再启用服务。未干预现役飞机运行状态。

## 可迁移操作入口

- [完整安装清单](../../src/onboard_control/deploy/JETSON_NEW_MACHINE_CHECKLIST.md)
- [依赖安装说明](../../src/onboard_control/deploy/ONBOARD_DEPENDENCIES.md)
- [Intel 8265 精确修复](../../src/onboard_control/deploy/JETSON_WIFI_8265.md)
- [OpenCV/cv_bridge 隔离构建](../../src/onboard_control/deploy/JETSON_OPENCV_OVERLAY.md)

保留开发机原有 README.md、TODO.md 和未跟踪任务/历史报告，不纳入本次代码提交。
