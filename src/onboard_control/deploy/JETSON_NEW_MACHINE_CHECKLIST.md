<!-- 本清单记录全新 Jetson Ubuntu 24.04 的可迁移安装、桌面和无外设交付顺序。 -->

# 全新 Jetson Ubuntu 24.04 安装操作清单

本文记录独立新机的可迁移安装顺序；不要把 USB 新机与既有 `drone-new` / `drone-old` 混淆。
命令默认在目标普通用户下执行。严禁自动解锁或起飞；环境验收不等于实飞验收。

## 1. 确认目标和网络

1. Jetson 独立供电，用支持数据传输的数据线连接设备模式 USB 接口。
2. 开发机运行 `lsusb`、`ip -brief address`、`journalctl -k -n 40`。
   本次设备为 `0955:7020`，序列号 `1424324322770`，开发机 USB 地址
   `192.168.55.100/24`，Jetson 地址 `192.168.55.1/24`。
3. 多台 Jetson 会复用 USB IP。为已确认的新设备使用独立 SSH 标识，保留其它设备记录：

   ```bash
   ssh -o HostKeyAlias=jetson-usb-1424324322770 nvidia@192.168.55.1
   hostname
   cat /etc/os-release
   uname -r
   dpkg --print-architecture
   cat /proc/device-tree/model
   ip route
   ```

4. 必须确认外网、DNS可用后才开始 apt。本次目标已能经开发机 USB 网络访问外网；
   不要假定每台开发机都已开启转发/NAT。

## 2. 镜像测速、更新和升级

在目标机分别对两镜像下载相同 ARM64 索引，交替测试至少三轮：

```bash
for round in 1 2 3; do
  for mirror in mirrors.tuna.tsinghua.edu.cn mirrors.ustc.edu.cn; do
    curl -L -o /dev/null -sS --connect-timeout 5 --max-time 20 \
      -w "$round $mirror %{http_code} %{time_total} %{speed_download}\n" \
      "https://$mirror/ubuntu-ports/dists/noble/main/binary-arm64/Packages.xz"
  done
done
```

本次中科大更快，使用 `https://mirrors.ustc.edu.cn/ubuntu-ports/`。
修改前备份实际启用的 `/etc/apt/sources.list` 或 `.sources` 文件；保留 NVIDIA/Jetson 源。
ARM64 必须用 `ubuntu-ports`，不能照搬开发机 amd64 的 `ubuntu` 路径。
启用 `noble`、`noble-updates`、`noble-security`、`noble-backports`，组件为
`main restricted universe multiverse`。

```bash
sudo apt-get update
apt-get -s upgrade                 # 先检查是否涉及替换Jetson内核或删除软件
sudo apt-get upgrade
apt-get -s --with-new-pkgs upgrade # 检查 kept-back 包所需的新依赖，仍不得删除软件
sudo apt-get --with-new-pkgs upgrade
sudo dpkg --audit
```

保存完整日志。SSH 包升级期间，新SSH连接可能暂时被拒绝；保持现有升级会话，勿断电。
本次普通 `upgrade` 后仍有包因新增依赖而 kept back，随后用 `--with-new-pkgs upgrade` 实际补齐；
该选项允许安装新依赖但仍不删除已安装包。执行前必须先模拟并检查 Jetson 内核、NVIDIA/L4T 包及
删除清单。保留 phased/kept-back 的最终实际清单，不把任一轮普通 upgrade 称为所有包均已更新。

## 3. Wi-Fi

先 `lspci -nnk`、`uname -r`、`dkms status`、`rfkill list` 定位实际硬件和驱动。
本次 Intel 8265 + Tegra 6.8.12-1021 的双重问题及精确修复见
[JETSON_WIFI_8265.md](JETSON_WIFI_8265.md)。不能直接把该内核上的强制 DKMS
安装经验应用到其它内核。修复后检查 NetworkManager 扫描，并在重启后再次确认自动加载。

## 4. 机载项目环境

1. 使用项目远端的 `main` 源码；参考 [ONBOARD_DEPLOYMENT.md](ONBOARD_DEPLOYMENT.md)
   的 sparse checkout。不要复制开发机 `build/`、`install/` 或 amd64 二进制。
2. **先实际尝试已有入口，并保存失败信息**：

   ```bash
   ./src/onboard_control/deploy/install_onboard_service.sh
   ```

   本次初次失败为 `/opt/ros/jazzy/setup.bash` 缺失。
3. 使用新增前置安装器补齐依赖：

   ```bash
   ROS_APT_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu \
     ./src/onboard_control/deploy/setup_onboard_dependencies.sh --with-odin-deps --with-camera-deps
   python3 -m venv --system-site-packages .venv
   ./src/onboard_control/deploy/build_onboard_control.sh --verify
   ```

   具体组件、下载校验及故障处理见 [ONBOARD_DEPENDENCIES.md](ONBOARD_DEPENDENCIES.md)。
   原生构建、单测和 localhost/domain 231 隔离 smoke 必须真实通过。
4. Odin 驱动、extnav 是仓库外依赖，需要获取对应源代码并在本机 ARM64/Jazzy 原生编译。
   不复制旧机器 install。核对 `/etc/ros2-ardupilot/onboard.env` 中串口、波特率及
   `ODIN_OVERLAY_SETUP`、`EXTNAV_OVERLAY_SETUP`，绝不能猜测硬件路径。
5. 仅装环境、硬件未齐时使用安装器 `--install-only`，保持单元不启动：

   ```bash
   ./src/onboard_control/deploy/install_onboard_service.sh --install-only
   ./video_service/deploy/install_onboard_video_service.sh --install-only
   ./correction_service/deploy/install_correction_service.sh --install-only
   ```

   硬件和外部驱动齐备后，才执行默认安装入口及 `scripts/onboard/start_onboard_control.sh --check`。
6. 视频为独立入口 `./video_service/deploy/install_onboard_video_service.sh`；
   correction 为 `./correction_service/deploy/install_correction_service.sh`。
   按各自 README 配置真实相机稳定路径，不把默认 `device=auto` 当成相机验收通过。

### 网络受限与参考机源码迁移

GitHub 直连失败时，可从开发机建立只绑定新机回环地址的临时反向代理。
以下 `10808` 是本次开发机已有代理端口，迁移到其它开发机时必须按实际端口调整：

```bash
# 开发机终端，保持运行；结束配置后退出即关闭隧道。
ssh -N -o ExitOnForwardFailure=yes -R 10809:127.0.0.1:10808 \
  -o HostKeyAlias=jetson-usb-1424324322770 nvidia@192.168.55.1
# 新机终端，仅对本次命令设置代理，不写入全局 apt 或桌面配置。
HTTPS_PROXY=http://127.0.0.1:10809 \
ROS_APT_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu \
  ./src/onboard_control/deploy/setup_onboard_dependencies.sh --with-odin-deps --with-camera-deps
```

本次 SourceForge 下载未及时完成，改为复制开发机已安装的架构无关 GeographicLib 数据集
（geoids/egm96-5、gravity/egm96、magnetic/emm2015），不是复制 amd64 程序。
在可信参考机执行 `tar -czf geographiclib-datasets.tar.gz -C /usr/share GeographicLib`，
对比传输前后 SHA-256 后，在新机用 `sudo tar -xzf geographiclib-datasets.tar.gz -C /usr/share`
导入。安装脚本仍验证 `egm96-5.pgm` 非空；本次 geoid 文件 SHA-256 为
`c4b25a03ec5845cec4778a54b580aeda676363f2205a89137e8677b2337af3ec`。
首次联网成功后也可直接运行 MAVROS 数据安装脚本；不要在失败时伪造数据文件。

本次经用户授权从 `drone-new` **只读**迁移以下源码，未启停参考机服务：

| 包 | 参考机路径 / 新机路径 | 迁移内容 |
| --- | --- | --- |
| Odin | `/home/nvidia/catkin_ws/src/odin_ros_driver` | 源码、配置、launch、厂家 ARM64 静态库；排除 `.git`、map/log/recorddata、build/install |
| extnav | `/home/nvidia/vrpn_mavros/src/extnav_bridge` | 仅包源码；由当前项目 extnav 安装器核对/部署受控版本 |
| 修正相机 | 主项目 `correction_service/correction_service/uvc_camera_node.py` | 随修正服务构建，直接使用系统 UVC 设备，无外部相机工作区 |

可用 `ssh drone-new 'tar ... -czf - ...' > archive.tar.gz` 直接把源码流写到开发机，
避免在参考机生成临时文件；先检查归档内容、再解到新机对应 `src/`。
Odin 不能直接对系统 `cv_bridge` 构建。先按
[JETSON_OPENCV_OVERLAY.md](JETSON_OPENCV_OVERLAY.md) 在 Odin 独立工作区用厂商 OpenCV 4.8
隔离构建 `cv_bridge`，source 该 overlay 后，再以 symlink 模式构建 Odin。Odin 源码的发行版
自动判断遗漏 Jazzy，必须显式选择 ROS2：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nvidia/catkin_ws
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 colcon build --symlink-install --packages-select odin_ros_driver \
  --cmake-args -DBUILD_SYSTEM=ROS2 -DCMAKE_BUILD_TYPE=Release
cd /home/nvidia/ros2-ardupilot-mavros-control
./correction_service/deploy/install_extnav_correction.sh
```

新机用户需加入 `dialout,video,plugdev`，重新登录后生效；新增依赖安装器已处理该组权限。
本次 Odin USB 权限文件 `/etc/udev/rules.d/99-odin-usb.rules` 为：

```udev
# Odin USB access for the onboard user.
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0660", GROUP="plugdev"
```

安装后 `sudo udevadm control --reload-rules`，设备连接时再确认权限。
不要复制参考机的 `/dev/ttyTHS1` 假定它必然连接新机飞控；只把已构建的 Odin/extnav overlay
路径写入新机 `onboard.env`，硬件路径待实际接线确认。

## 5. 桌面软件与输入法

```bash
sudo apt-get install libreoffice openssh-server vim vlc qv4l2 \
  fcitx5 fcitx5-chinese-addons fcitx5-config-qt fcitx5-frontend-gtk3 \
  fcitx5-frontend-gtk4 fcitx5-frontend-qt5 im-config \
  remmina remmina-plugin-rdp seahorse
sudo systemctl enable --now ssh
im-config -n fcitx5
```

从开发机复制 `~/.config/fcitx5/config`、`profile`，不要复制词库和其它私人数据。
本机 profile 是 `keyboard-us` + `pinyin`，切换键精确为：

```ini
[Hotkey/TriggerKeys]
0=Control+Shift+Shift_L
```

注销重登后确认 Fcitx5 进程、环境变量及实际中文输入。GNOME 的 Super+Space 设置不是
Fcitx5 的 Ctrl+Shift 设置，不要误用桌面 source shortcut 代替。

VS Code 使用[官方 ARM64 deb](https://code.visualstudio.com/docs/setup/linux)：

```bash
curl -fL https://update.code.visualstudio.com/latest/linux-deb-arm64/stable -o vscode-arm64.deb
dpkg-deb -f vscode-arm64.deb Package Version Architecture
sudo apt-get install ./vscode-arm64.deb
```

NoMachine 必须先读取开发机 `dpkg-query -W nomachine`，再获取**同版本 arm64**包，
用 `dpkg-deb -f` 验证后安装。本次基准为 `9.8.2-1`，不能换成最新版，也不能安装 amd64 包。
截至本次检查，多个官方旧下载入口重定向到首页，尚需用户提供可验证的历史 ARM64 安装包。

## 6. Remmina / GNOME 桌面共享

以下命令必须在已登录的目标普通用户 `nvidia` 下执行，不使用 `sudo`。先为该用户配置 GDM 自动
登录；用户模式桌面共享依赖实际图形会话，不能替代登录屏的系统级“远程登录”。本次 RDP 用户名和
密码均为 `nvidia`：

```bash
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"

mkdir -p ~/.local/share/keyrings ~/.local/share/gnome-remote-desktop
chmod 700 ~/.local/share/keyrings ~/.local/share/gnome-remote-desktop
if [[ ! -e ~/.local/share/keyrings/jetson-login.keyring ]]; then
  cat > ~/.local/share/keyrings/jetson-login.keyring <<'EOF'
[keyring]
display-name=Login
ctime=0
mtime=0
lock-on-idle=false
lock-after=false
EOF
  chmod 600 ~/.local/share/keyrings/jetson-login.keyring
fi
cp -an ~/.local/share/keyrings/default \
  ~/.local/share/keyrings/default.before-jetson-setup 2>/dev/null || true
printf 'jetson-login\n' > ~/.local/share/keyrings/default

# 只应在首次配置且没有其它桌面应用使用 Secret Service 时重启密钥环。
systemctl --user stop gnome-keyring-daemon.service gnome-keyring-daemon.socket || true
systemctl --user start gnome-keyring-daemon.socket

tls_dir="$HOME/.local/share/gnome-remote-desktop"
if [[ ! -s "$tls_dir/rdp-tls.key" || ! -s "$tls_dir/rdp-tls.crt" ]]; then
  openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
    -subj "/CN=$(hostname)" \
    -keyout "$tls_dir/rdp-tls.key" -out "$tls_dir/rdp-tls.crt"
  chmod 600 "$tls_dir/rdp-tls.key"
fi
grdctl rdp set-tls-key "$tls_dir/rdp-tls.key"
grdctl rdp set-tls-cert "$tls_dir/rdp-tls.crt"
grdctl rdp set-credentials nvidia nvidia
grdctl rdp disable-view-only
grdctl rdp enable
systemctl --user enable gnome-remote-desktop.service
grdctl status
```

GNOME 设置中保持 **桌面共享**开启、**远程登录**关闭；这里使用用户服务
`gnome-remote-desktop.service`，不使用 `grdctl --headless` 或 `--system`。重登或重启后检查图形
会话、默认密钥环 unlocked、RDP 监听及 Remmina 实际认证。只看到 gsettings 或 unit enabled 不等于
桌面已可连接。

## 7. 交付核验

- `dpkg --audit` 无异常；记录保留更新及是否仍需重启。
- `dpkg-query -W` 核对软件版本和架构，NoMachine未取得同版包则明确未完成。
- Wi-Fi重启后仍可扫描；连接/互联网验证需使用用户指定的网络凭据。
- Fcitx5实际切换、拼音输入和桌面共享真实认证。
- 项目原生构建、测试、隔离smoke结果；外部驱动、飞控和相机缺失必须逐项列出。
- 当前新机没有飞控、Odin 或相机等任何外设，飞控、视频、修正三个机载服务保持 disabled/stopped
  是预期交付状态；不得为了“全绿”而启动服务或沿用旧机设备路径与标定。

  ```bash
  systemctl is-enabled ros2-ardupilot-onboard.service video-service.service odin-correction.service
  systemctl is-active ros2-ardupilot-onboard.service video-service.service odin-correction.service
  ```

  `is-enabled` 和 `is-active` 在此阶段返回非零并显示 `disabled`/`inactive` 属于预期结果。
- 在 `agent/report/` 新增报告，并维护 `MEMORY.md` 中真实基线，避免把独立新机写成原飞机。
