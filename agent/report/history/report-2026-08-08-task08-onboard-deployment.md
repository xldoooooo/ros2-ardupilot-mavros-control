# 任务 08：机载服务安全部署报告

日期：2026-08-08

目标机：`xld@192.168.112.186`

部署目录：`/home/onboard/ros2-ardupilot-mavros-control`

## 1. 结论

任务 08 的“最小拉取、Humble/aarch64 原生编译、包测试、隔离运行验证、可复用脚本与说明文档”已完成。

最终结果：

- 无人机只检出 `src/guided_interfaces/` 和 `src/onboard_control/`；
- 未拉取地面站、Qt、`guided_sim`、SITL 或其他无关工作树文件；
- Humble/aarch64 Release 构建成功；
- `colcon test-result --verbose`：5 tests、0 errors、0 failures、0 skipped；
- 隔离烟雾状态：接口 `2.0`、`fcu_connected=false`、`armed=false`；
- 对隔离姿态话题观察 2 秒，没有收到 setpoint 消息；
- 测试后无 `onboard_control_node`、MAVROS、Odin 或 extnav 残留进程；
- 没有安装或启用 systemd 服务；
- 没有启动真实 MAVROS/Odin/extnav，没有连接飞控，没有调用任何飞行或维护命令；
- 全过程没有解锁或起飞实机。

这证明新机载源码能在当前 Humble/aarch64 计算机上编译、链接、测试并安全待机启动；不证明 Humble/Jazzy 跨发行版 DDS、真实 MAVROS/Odin/extnav、飞控参数或飞行控制已经可用。

## 2. 部署前安全审计

目标机环境：

```text
Ubuntu 22.04.5 LTS
ROS 2 Humble
aarch64 / Jetson
8 CPU / 15 GiB RAM
根分区剩余约 173 GiB
MAVROS 2.14.0
```

部署前 `/home/onboard` 已存在、为空、权限为 `root:root 0755`。本次只在其下新建仓库目录，并保留父目录所有权：

```text
/home/onboard                                      root:root 0755
/home/onboard/ros2-ardupilot-mavros-control        xld:xld   0755
```

部署前没有发现 MAVROS、Odin、extnav、ArduPilot 或新机载控制进程，也没有相关 systemd 服务。

旧回退环境记录：

```text
/home/xld/ros2-ardupilot-mavros-control
HEAD = dad90678e02169b80b44f903753e157c4bfda7c5
工作树 = clean
```

三个关键旧脚本的部署前 SHA-256：

```text
533cc0b8578d4dfde46dc26d5fd69dd16390e95b3fbd579a5ba316fde87b80b0  /home/xld/odin.sh
ddbfb5000a471981038ca78bb8520c9e7fc34405d51b13a301c93a1705a366da  /home/xld/ros2-ardupilot-mavros-control/odin1.sh
8bb06fe3f30e46ee50799c9d175ec227b0f0627bdc633888ef2c3b7280530d48  /home/xld/ros2-ardupilot-mavros-control/shfiles/start_mavros_real.sh
```

部署后重新校验，旧提交、工作树状态和三个哈希完全一致。

## 3. 仓库改动

### 3.1 最小部署助手

新增 `src/onboard_control/deploy/onboard_workspace.sh`，命令包括：

- `show-config`：显示工作区、ROS 发行版、架构、Git 提交和烟雾隔离信息；
- `update`：只允许干净的 sparse checkout，并使用 `git pull --ff-only`；
- `deps-check`：只读检查工具链、Eigen 和 16 个 ROS 包，不调用 apt、sudo 或 rosdep 初始化；
- `build`：Release 构建 `guided_interfaces` 与 `onboard_control`；
- `test`：执行两包测试并汇总 `colcon test-result`；
- `smoke`：在隔离环境启动节点，验证安全待机并有界清理；
- `verify`：依次执行依赖检查、构建、测试和烟雾测试。

脚本不提供清理 build、reset、强制覆盖、系统安装、飞行命令或 MAVROS 启动入口。

### 3.2 烟雾测试硬件隔离

烟雾测试使用：

```text
ROS_DOMAIN_ID=231
Humble: ROS_LOCALHOST_ONLY=1
Jazzy: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
MAVROS prefix=/_task08_smoke_mavros
interface prefix=/_task08_smoke_onboard
```

它不启动 MAVROS，不调用 acquire-control、FlightCommand、MotionIntent、航点、原点或消息频率服务。测试只读取聚合状态并订阅隔离姿态话题；必须看到 FCU 未连接、未武装，并在 2 秒窗口内收不到姿态 setpoint。

节点清理由直接二进制 PID 管理：先 INT，有界等待，再按需 TERM/KILL，避免 `ros2 run` 包装进程不转发信号造成无限等待。

### 3.3 部署说明与服务模板

新增 `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`，覆盖：

- `/home/onboard` 首次 sparse clone；
- GitHub 直连失败时的临时 SSH SOCKS 隧道；
- 只读依赖检查、构建、测试、烟雾与更新命令；
- Humble/Jazzy 网络变量差异；
- systemd 暂不安装的原因；
- 后续通信测试的分级顺序。

更新示例：

- systemd 不再写死 `User=nvidia`、Jazzy 或不存在的 `mavros.service`；
- 环境示例明确 `ROS_DISTRO=humble`、机载工作区和 Humble 的 `ROS_LOCALHOST_ONLY=0`；
- `package.xml` 补充 launch 实际使用的 `ament_index_python`、`launch`、`launch_ros` 运行依赖。

### 3.4 Humble/Jazzy C++ API 兼容

Humble 首次编译在命令时间戳校验处失败：

```text
const rclcpp::Clock cannot call Humble Clock::now()
```

Jazzy 的 `Clock::now()` 是 const，Humble 的不是；两版 `rclcpp::Node::now()` 都是 const。因此只把：

```cpp
get_clock()->now()
```

改为：

```cpp
now()
```

函数 const 性、时间语义、TTL 判断和所有控制逻辑均未改变。修复后 Jazzy 与 Humble 均重新构建和测试通过。

## 4. 实际部署过程

### 4.1 GitHub 网络与最小检出

无人机 DNS 能解析 GitHub，但 HTTPS 443 直连 10 秒超时。首次 clone 只留下临时空目录，没有源码或提交；Git 自行清理后目标仍为空。

没有采用包含地面站历史内容的完整 bundle，也没有写永久代理。通过地面机建立只绑定无人机 `127.0.0.1:19080`、只在 SSH 会话存活的反向动态 SOCKS 隧道后，`git ls-remote` 和 partial clone 成功。

最终 Git 配置：

```text
origin = https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git
sparse mode = non-cone
sparse paths:
  /src/guided_interfaces/
  /src/onboard_control/
```

无人机完成编译后的整个新工作区约 29 MiB；工作树中没有地面站或仿真包。

较旧 Git 还暴露一个细节：`sparse-checkout set --no-cone` 会把 `--no-cone` 当作路径。该项不曾检出额外文件，现已改为只在 `init` 阶段指定 `--no-cone`，并加入回归测试。

### 4.2 构建与运行

第一次 Humble 构建：

```text
guided_interfaces：成功
onboard_control：因 Clock::now const API 差异失败
节点未生成/未运行
```

修复并更新后：

```text
guided_interfaces：成功
onboard_control：成功（aarch64 Release）
colcon tests：5 passed
isolated smoke：passed
```

生成二进制：

```text
ELF 64-bit LSB pie executable, ARM aarch64
```

在同一 shell 中 source `/opt/ros/humble/setup.bash` 和新工作区 `install/setup.bash` 后，`ldd` 所有动态库均解析成功。一次裸 `ldd` 因未 source ROS/overlay 曾显示 ROS 库 not found；这是检查环境错误，不是二进制缺库，随后使用正确运行环境复核通过。

## 5. 验证结果

### 5.1 开发机 Jazzy/x86_64

最终验证：

```text
colcon build guided_interfaces/onboard_control/guided_sim：成功
Python/Qt：30 passed
ROS/C++：5 tests, 0 errors, 0 failures, 0 skipped
部署助手 verify：通过
环境诊断：通过
bash -n / compileall / 修改范围 flake8 / git diff --check：通过
```

过程中有两类首轮问题，均如实保留：

1. 未先 source 新 overlay 就运行 Python 测试，导致自定义 ROS 类型支持库加载失败；正确按“构建→source overlay→测试”重跑后为 30 passed。
2. 初版烟雾清理通过 `ros2 run` 包装 PID 并使用无限 `wait`，状态断言已通过但退出悬挂。只终止了已确认的隔离测试进程组，随后改为直接二进制 PID 和有界清理；复验无残留。

### 5.2 无人机 Humble/aarch64

最终验证：

```text
dependency check：16 ROS packages，全部存在
Release build：2 packages finished
GTest：4/4 passed
colcon aggregate：5 tests, 0 errors/failures/skipped
smoke status：interface=2.0, fcu_connected=false, armed=false
attitude setpoint：2 秒观察窗口内 0 条
dynamic libraries：source 正确环境后全部解析
runtime residue：无
```

## 6. 未执行及限制

本任务故意没有执行：

- `/home/xld` 旧启动流程；
- 真实 Odin、MAVROS、extnav；
- `/dev/ttyTHS1` 飞控连接；
- `ROS_DOMAIN_ID=42` 跨机 DDS；
- GUI“连接实机服务”；
- 控制租约申请；
- 消息频率配置或 GPS 原点写入；
- 飞控参数读取/写入；
- systemd 安装、enable 或 start；
- 任何模式切换、解锁、起飞、姿态/推力控制。

ROS 官方不保证 Humble 与 Jazzy 跨发行版节点通信。当前结果只能证明目标机本地构建与隔离待机，不得扩展为“真机控制已验证”或“可以飞行”。

## 7. 下一步通信测试

严格按以下顺序进行，并持续保持螺旋桨拆除、实机不解锁：

1. 只启动新机载节点，不启动 MAVROS；地面站使用 domain 42，只读取 `/onboard_control/status`，确认接口 2.0 与 `armed=false`。不要点击 GUI 的实机连接按钮。
2. 沿用旧流程单独启动 Odin/MAVROS/extnav，只读 `/mavros/state`、位姿和速度；不调用任何服务。
3. 启动新机载节点，只观察聚合状态能否正确反映飞控、位姿、推力语义和发布者冲突；不申请租约、不发送命令。
4. 前三步通过后，再单独评审“消息频率/GPS 原点”维护写操作。它们不属于只读测试，不能自动执行。
5. 本任务及当前规范禁止实机解锁和起飞；不进行飞行控制测试。

完整手动命令见 `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`。
