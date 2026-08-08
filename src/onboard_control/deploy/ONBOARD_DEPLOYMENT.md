# 机载服务最小部署与验证

本文档用于把 `guided_interfaces` 与 `onboard_control` 两个 ROS 2 包部署到伴随计算机，当前实机目标为 Ubuntu 22.04、ROS 2 Humble、aarch64。

## 安全边界

- 严禁在本流程中解锁或起飞实机。
- 初次部署只允许编译、单元测试和隔离烟雾测试；不启动真实 MAVROS、Odin、外部定位桥或地面站控制客户端。
- 不修改 `/home/xld` 下的旧仓库、旧脚本和旧工作空间。旧启动流程继续作为回退方案。
- 不安装或启用 systemd 自启动服务。完成独立台架通信验证前，不得把新机载节点加入开机启动。
- `smoke` 使用非零独立 ROS domain、localhost-only 发现和专用 MAVROS 前缀，不会发现真实 MAVROS；它只检查待机状态，并断言没有姿态 setpoint 消息。

## 1. 首次最小拉取

目标目录是 `/home/onboard/ros2-ardupilot-mavros-control`。父目录可保持 `root:root`，只把新建的仓库目录交给部署用户管理：

```bash
sudo install -d -o xld -g xld \
  /home/onboard/ros2-ardupilot-mavros-control

git clone \
  --filter=blob:none \
  --no-checkout \
  https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git \
  /home/onboard/ros2-ardupilot-mavros-control

cd /home/onboard/ros2-ardupilot-mavros-control
git sparse-checkout init --no-cone
git sparse-checkout set \
  '/src/guided_interfaces/' \
  '/src/onboard_control/'
git checkout main
```

检出后，除 Git 元数据外只应出现：

```text
src/guided_interfaces/
src/onboard_control/
```

不要复制开发机的 `build/` 或 `install/`。目标机必须针对 Humble/aarch64 原生编译。

### GitHub 直连超时时

当前无人机能解析 `github.com`，但实测 HTTPS 443 直连可能超时。不要写入永久系统代理，也不要复制包含地面站历史的完整 Git bundle。可从能够访问 GitHub 的地面机建立仅绑定无人机 localhost、只在当前 SSH 会话存活的反向动态 SOCKS 隧道：

```bash
ssh -R 127.0.0.1:19080 xld@192.168.112.186
```

然后在这个 SSH 会话内临时设置：

```bash
export HTTPS_PROXY=socks5h://127.0.0.1:19080
export HTTP_PROXY=socks5h://127.0.0.1:19080
```

再执行本节的 `git clone`，或执行后文的 `onboard_workspace.sh update`。退出 SSH 后隧道自动消失；不要把代理写入 `/home/xld/.gitconfig`、shell 启动文件或 systemd 环境。

## 2. 无系统修改的依赖检查

```bash
cd /home/onboard/ros2-ardupilot-mavros-control
./src/onboard_control/deploy/onboard_workspace.sh show-config
./src/onboard_control/deploy/onboard_workspace.sh deps-check
```

`deps-check` 只读取工具链、Eigen 头文件和 ROS 包索引，不依赖已初始化的 rosdep 数据库，也不会调用 `apt` 或 `sudo`。如果报告缺包，应先人工审查清单；不要直接批量安装或升级已有 ROS 环境。

## 3. 编译、单元测试与安全烟雾测试

```bash
cd /home/onboard/ros2-ardupilot-mavros-control
./src/onboard_control/deploy/onboard_workspace.sh build
./src/onboard_control/deploy/onboard_workspace.sh test
./src/onboard_control/deploy/onboard_workspace.sh smoke
```

也可一次执行：

```bash
./src/onboard_control/deploy/onboard_workspace.sh verify
```

成功烟雾测试必须同时报告：

```text
interface=2.0
fcu_connected=false
armed=false
setpoint_messages=0
```

这只证明 Humble 上的二进制能够启动并发布安全待机状态，不证明跨发行版 DDS、Odin/MAVROS、外部定位、飞控参数或真实控制链已经可用。

## 4. 后续更新

```bash
cd /home/onboard/ros2-ardupilot-mavros-control
./src/onboard_control/deploy/onboard_workspace.sh update
./src/onboard_control/deploy/onboard_workspace.sh verify
```

`update` 只允许 sparse checkout，并要求 Git 工作树干净；更新使用 `git pull --ff-only`，不会 reset、覆盖本地修改或触碰 `/home/xld`。

## 5. Humble/Jazzy 网络变量

机载 Humble 使用：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

地面站 Jazzy 使用：

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_AUTOMATIC_DISCOVERY_RANGE` 和 `ROS_STATIC_PEERS` 不是 Humble 的通用配置接口。Humble 默认通过同网段多播发现；若多播不可靠，需要为双方实际使用的 Fast DDS 版本分别验证 XML discovery 配置。

ROS 官方不保证 Humble 与 Jazzy 跨发行版通信。即使状态话题和服务在台架测试中可见，也只能视为当前版本组合的实测结果，不能据此宣称适合实飞。

## 6. 下一步通信测试顺序

所有测试均保持螺旋桨拆除、飞控不解锁，并由低风险到高风险逐级进行：

1. **DDS 协议测试**：不启动 MAVROS，只启动新机载节点；地面站只读取 `/onboard_control/status`，确认接口 `2.0`、`armed=false`。不要点击 GUI 的“连接实机服务”，因为该工作流会申请租约、配置消息频率并写 GPS 原点。
2. **MAVROS 只读测试**：沿用旧流程单独启动 Odin、MAVROS 和 extnav；只读取 `/mavros/state`、本地位姿和速度，确认 `armed=false`，不调用任何模式、解锁、起飞、原点或消息频率服务。
3. **同机链路测试**：启动新机载节点，只观察其聚合状态是否正确反映飞控连接、位姿、`GUID_OPTIONS` 与 setpoint 发布者冲突；不申请控制租约、不发送 `FlightCommand`/`MotionIntent`/航点。
4. **维护命令测试**：只有前三步均通过且另行确认后，才评估设置消息频率和 GPS 原点。它们会改变飞控运行状态，不属于只读测试。
5. **飞行控制测试**：本任务明确禁止，不能进行解锁、起飞或实机姿态/推力控制。

## 7. systemd 示例

`onboard-control.service.example` 仅供未来部署参考，任务 08 不安装、不启用。使用前必须替换 `ONBOARD_USER`，确认 `/etc/ros2-ardupilot/onboard.env`，并完成独立的台架安全评审。服务只启动 `onboard_control`，不会替代 Odin、MAVROS 或 extnav 的生命周期管理。
