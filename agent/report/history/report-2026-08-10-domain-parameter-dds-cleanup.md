# 2026-08-10 目录、ROS Domain、参数同步与 DDS 异常诊断简报

## 1. 任务边界与安全状态

本次对开发机仓库、当前运行的开发机地面站，以及 `192.168.112.186` 上的伴随计算机进行了检查。
实机侧只执行了进程、环境、参数、话题频率、资源和网络队列的读取操作。

- 没有请求或执行解锁、起飞、模式切换、降落、航点、姿态/推力设定点、GPS 原点写入。
- 没有在实机调用 `/mavros/set_message_interval`，没有重启或终止实机上的 MAVROS、Odin、extnav、
  `onboard_control_node`。
- 实机检查期间状态始终为 `armed=false`、`STABILIZE`、无控制租约、控制器未激活。
- 本报告中的代码修正在开发机完成并通过隔离测试，尚未部署或热替换当前实机进程。

## 2. 结论摘要

1. `shfiles/` 并不是源码目录，而是一次旧 SITL/MAVProxy 运行留下的 EEPROM、参数、遥测、日志和
   terrain 缓存；目录内确实没有 shell 文件。它和同批根目录运行产物已经移入系统回收站。
2. 当前地面站与实机四个 ROS 进程实际上都运行在默认 domain 0。只在某个新终端设置
   `ROS_DOMAIN_ID=42` 后执行 `ros2 topic list`，相当于进入一个隔离的新 ROS 图，所以只看到临时 CLI
   节点，不能看到已经在 domain 0 中运行的节点。42 不是地址、频率或链路优化参数。
3. `GUID_OPTIONS` 和 `MOT_THST_HOVER` 在实机中均存在且有效，当前值分别为 `8` 和
   `0.20921991765499115`。原警告发生在 MAVROS 连接飞控后约 35 秒的完整参数表同步窗口，不是参数
   丢失；同步完成后机载状态已经变为 `thrust_mode_verified=true`。
4. `sequence size exceeds remaining buffer` 不是地面站业务消息太多，也没有证据表明当前 UDP 队列
   阻塞。高置信度原因是开发机 Jazzy/Fast DDS 与实机 Humble/Fast DDS 的跨发行版、跨版本 DDS
   discovery/type 元数据反序列化不兼容。ROS 官方不保证跨发行版节点互通。
5. `/mavros/local_position/pose` 的“卡住”包含另一个独立条件：当前拆分后的启动方式没有执行旧
   `odin.sh` 里的三次 MAVLink 消息频率配置，本次运行的多个窗口实测该话题为 **0 Hz**。这能解释
   当前窗口为何一直等不到样本，但不能推导为每次启动都必然 0 Hz，也不能解释用户观察到的间歇成功；
   DDS discovery 错误与飞控是否正在发送 message 32 必须分别判断。
6. 已在本地实现两项直接修正：单纯打开 GUI 不再创建 DDS participant；机载控制节点在 FCU 连接和
   重连后自动异步恢复三类必需 MAVLink 消息为 100 Hz。跨发行版 DDS 的根治仍需要统一网络端 ROS
   发行版/RMW，或加入受支持的桥接边界，不能靠 domain 号解决。

## 3. 文件与目录审计

### 3.1 已清理

以下内容经确认均为被 `.gitignore` 排除、没有活动代码引用的生成物；使用 `gio trash` 移入系统回收站，
可在清空回收站前恢复：

| 项目 | 约占用 | 判断 |
| --- | ---: | --- |
| `shfiles/` | 14 MiB | 旧 SITL EEPROM、参数、tlog/raw、日志、terrain；无 `.sh` |
| `logs/` | 100 MiB | 旧 ArduPilot/SITL 运行日志，与标准 colcon `log/` 不同 |
| `mav.tlog`、`mav.tlog.raw` | 55 MiB | 旧 MAVProxy 遥测记录 |
| `terrain/`、`eeprom.bin`、`mav.parm` | 小于 0.1 MiB | 可再生成的 SITL 状态 |
| 两个 `dob_hover_log_20260806_*.csv` | 0.4 MiB | 旧测试输出 |
| `fishros` | 约 1 KiB | 已下载过的临时安装脚本，不是工程启动入口 |
| Python/pytest 缓存 | 约 1.2 MiB | `__pycache__`、`.pytest_cache`，可再生成 |

合计约回收 **169 MiB**。另外使用 `rmdir` 删除了确认完全为空的目录：

- `run/`
- `src/guided_sim/include/`
- `src/guided_sim/msg/`
- `src/guided_sim/params/`
- `src/guided_sim/src/`

`ground_station_core/flight_modes/` 只剩三个已不存在模块的旧 `.pyc`，也随缓存清理移入回收站。

### 3.2 有意保留

- `build/`、`install/`、`log/` 是标准 colcon 工作空间目录；`install/` 正被地面站 overlay 使用，三者不应
  与无来源的根目录垃圾混在一起判断。本轮未删除。
- `agent/task/`、`agent/report/`、`agent/report/history/` 分别承担任务、当前报告和历史报告职责；已有
  用户迁移/删除状态未被恢复或改写。
- `agent/codex/` 和 `agent/grok/` 是工程规则指定的代理过程文件目录，不建议再额外建立含义重叠的
  通用 `dev/` 桶。
- `assets/`、`image/` 有明确的界面和相机标定内容，保留。

推荐长期保持下面的边界，而不是把不同性质内容统一塞入 `dev/`：

```text
项目根目录
├── ground_station.py / ground_station_core/   # 产品地面站
├── src/                                        # ROS 2 包
├── tests/                                      # 自动回归
├── agent/task|report|codex|grok/               # 任务、报告、代理临时证据
├── assets/ / image/                            # 明确归属的静态/标定资源
└── build/ install/ log/                        # colcon 生成物，可整体重建
```

审计开始前工作树已经包含用户修改的 `TODO.md`、用户删除的旧截图/报告，以及新建的
`agent/report/history/` 等内容。本次全部按原状保留，未进行 git 恢复、暂存、提交或推送。

## 4. `ROS_DOMAIN_ID=42` 的作用

DDS domain 是同一物理网络上的逻辑隔离边界：只有 domain ID 相同的 participant 才互相发现并通信。
ROS 2 未设置 `ROS_DOMAIN_ID` 时使用默认值 0。数字 42 只是一个可选值，没有特殊飞控含义。

本次从实际进程环境读取到：

| 位置/进程 | 实际 domain | 发现范围 |
| --- | ---: | --- |
| 开发机当前地面站 | 0（变量未设置） | `SUBNET` |
| 实机 MAVROS | 0（变量未设置） | Humble 默认局域网发现 |
| 实机 Odin | 0（变量未设置） | Humble 默认局域网发现 |
| 实机 extnav | 0（变量未设置） | Humble 默认局域网发现 |
| 实机 onboard control | 0（变量未设置） | Humble 默认局域网发现 |

在 domain 42 中查询时只出现 `/parameter_events`、`/rosout` 和临时 CLI 节点；切回 domain 0 后完整
MAVROS/Odin/extnav/onboard 图立即可见。因此“写入 42 后 topic list 为空”是预期隔离结果，不是网络坏了。

注意两点：

1. 必须使用准确变量名 `ROS_DOMAIN_ID`；如果字面上只设置 `DOMAIN=42`，ROS 2 不会读取它。
2. 环境变量只在进程启动时读取。在节点启动后修改终端变量，不能把已经运行的节点迁移到另一个
   domain。

当前手工四终端启动方式下，保持全部默认 0 是最少出错的选择。如果要用 42 隔离其他 ROS 设备，必须
先停止相关进程，在 **MAVROS、Odin、extnav、onboard、地面站和所有诊断 CLI 启动前**统一加载同一
环境。42 没有修复 Humble/Jazzy 兼容性的能力。

## 5. `GUID_OPTIONS` / `MOT_THST_HOVER` 警告

### 5.1 两个参数为何重要

- 项目要求 `GUID_OPTIONS` 包含 bit 3（数值 `8`），让 MAVLink `SET_ATTITUDE_TARGET.thrust` 按真实
  归一化推力解释。
- `MOT_THST_HOVER` 是飞控学习/标定得到的悬停推力。机载 PD+DOB 控制器同步该值，才能把垂向加速度
  正确映射为推力。
- 在两项都确认前禁用原始姿态/推力输出是刻意设置的安全门，避免错误推力语义导致掉高或突然爬升。

### 5.2 实机证据与根因

实机只读参数查询得到：

```text
GUID_OPTIONS = 8
MOT_THST_HOVER = 0.20921991765499115
```

启动日志时间线为：

| 相对时间 | 现象 |
| ---: | --- |
| 0 s | MAVROS 报告 FCU connected；机载节点立即尝试同时读取两个动态参数 |
| 0–35 s | MAVROS 完整飞控参数表尚未同步，记录 `Failed to get parameter type`；机载端每 5 秒重复旧提示 |
| 约 35.6 s | MAVROS 报告 parameter list received |
| 随后 | 机载状态变为“已确认 GUID_OPTIONS bit 3，并同步 MOT_THST_HOVER=0.20922” |

所以该日志在本次实机上是**启动暂态提示**，不是当前故障。同步前的后果仅是姿态/推力控制安全门保持
关闭；状态发布、MAVROS 连接和 STABILIZE 待机不受影响。若超过 60 秒仍不能验证，或已经进入原始控制
后校验失效，则应按真实故障处理，禁止起飞/继续控制并检查 MAVROS 参数插件、串口链路和飞控参数。

### 5.3 已实现修正

- FCU 新连接后先显示一次“等待 MAVROS 完成飞控参数同步”。
- 首次查询延后 40 秒，避免在完整参数表到达前反复请求和刷同一 INFO。
- 60 秒内读取不完整按“同步中”处理；超过后明确报告“参数同步超时”。
- 已验证状态不会因一次瞬时读取失败被伪造为失效；FCU 真正断开仍会撤销验证。
- 地面站等待推力语义的超时由 35 秒扩为 60 秒。
- 相同状态文本不再周期性重复写日志。

## 6. 当前通信具体如何工作

### 6.1 地面站到飞机

单纯创建旧版地面站窗口时，ROS 客户端会立刻加入 DDS 图并创建 2 个 publisher、4 个 service client 和
2 个 subscription。控制未开启时它不会发心跳或飞行命令，但仍会产生 DDS participant/topic/type
discovery 流量。修正后，**只打开窗口为 ROS IDLE，不创建 DDS participant，也不发送 discovery 或业务
数据**；点击仿真、连接实机或 Wi-Fi 检测时才按需启动 ROS。

连接后的应用层行为如下：

| 条件 | 地面站发送内容 | 频率/次数 |
| --- | --- | ---: |
| 独立 Wi-Fi 检测 | 仅创建状态和临时 `/rosout` 订阅 | 零租约、零心跳、零维护、零飞行命令 |
| 完整连接，尚无租约 | `AcquireControl` 请求 | 立即一次；未获准时最多 1 Hz 重试 |
| 已持有租约 | `ControlHeartbeat`，租期 1500 ms | 5 Hz |
| 完整连接维护 | 确认消息频率、写 GPS 原点 | 各一次；不是周期流量 |
| 操作员点击/按键 | 起降/模式/航点 service，或一个 `MotionIntent` | 事件驱动，每次操作一条 |

地面站没有 MAVROS 姿态 setpoint publisher；连续 100 Hz 控制只存在于机载节点，并且必须同时满足已武装、
GUIDED、租约/状态/位姿/推力语义有效等门控。本次实机为 STABILIZE、未武装，实测姿态 setpoint 为 0。

### 6.2 飞机到地面站

地面站产品代码直接消费的高层数据只有：

| 内容 | 条件 | 频率 |
| --- | --- | ---: |
| `/onboard_control/status` | onboard 节点运行 | 配置值 10 Hz，实测约 10 Hz |
| `/onboard_control/command_result` | 接到离散请求或任务进度变化 | 事件驱动 |
| `/rosout` | 仅实机连接/通信检测临时开启，且远端有日志 | 事件驱动；可能补发 transient-local 历史 |

当前实机同机链路的 8 秒只读采样为：

| 话题 | 样本数 | 实测频率 |
| --- | ---: | ---: |
| `/odin1/odometry_highfreq` | 1996 | 264.52 Hz |
| `/extnav/pose_fcu` | 757 | 99.92 Hz |
| `/extnav/velocity_fcu` | 756 | 99.90 Hz |
| `/mavros/vision_pose/pose` | 303 | 40.02 Hz |
| `/mavros/state` | 7 | 约 1 Hz |
| `/onboard_control/status` | 80 | 约 10 Hz |
| `/mavros/local_position/pose` | 0 | **0 Hz** |
| `/mavros/local_position/velocity_local` | 0 | **0 Hz** |

这说明当前链路不是“一切都不通”：Odin、extnav、MAVROS state 和机载聚合状态都在持续工作；缺失的是
飞控回传的 LOCAL_POSITION_NED 流。

## 7. `sequence size exceeds remaining buffer` 与 local pose 等待

### 7.1 为什么不是“地面站发太多消息”

地面站开启期间抓取 10 秒 UDP 流量：

- 总计 215 包、68,444 字节，约 6.8 KiB/s。
- 地面站到飞机 40 包、12,086 字节。
- 飞机到地面站 175 包、56,358 字节。

这个量级不可能耗尽 Wi-Fi 或千兆/百兆网带宽。实机 UDP socket 当前收发队列为 0；`UdpInErrors` 虽然
自开机累计为 126,280，但连续 5 秒增量为 0，不能用累计数证明当前地面站造成阻塞。8 核 CPU 整体仍有
约 60% idle；Odin、MAVROS、extnav 单进程负载较高，值得做长期调度监控，但不是这条 CDR 错误的直接
证据。

### 7.2 高置信度根因

两端实际使用：

| 端 | ROS 2 | Fast DDS | `rmw_fastrtps_cpp` |
| --- | --- | --- | --- |
| 开发机 | Jazzy | 2.14.6 | 8.4.3 |
| 实机 | Humble | 2.6.11 | 6.2.10 |

只要 Jazzy 地面站 participant 出现在 Humble 图中，多个 Humble 终端便开始打印 Fast CDR 的
`sequence size exceeds remaining buffer`。Jazzy 侧把部分 Humble endpoint 显示为节点名未知、type hash
`INVALID`。结合低流量、空 UDP 队列和稳定的部分话题送达，最符合证据的解释是：新版/旧版 Fast DDS
在反序列化彼此的 discovery/type 元数据时出现不兼容，而不是 ROS topic 样本积压。精确到哪一种 RTPS
submessage 仍需专门的 RTPS/Wireshark 取证，但不影响“跨 ROS 发行版链路不受支持、不能作为实飞通信
基线”的结论。

ROS 官方发行版说明明确指出，不同 ROS 发行版的节点通信不受保证，可能工作也可能不工作，且属于不受
支持配置：<https://docs.ros.org/en/humble/Releases.html>。

### 7.3 为什么 local pose 看起来卡住

旧 `/home/xld/odin.sh` 会调用三次 `/mavros/set_message_interval`，请求 MAVLink message ID 32
(`LOCAL_POSITION_NED`)、31 (`ATTITUDE_QUATERNION`) 和 105 (`HIGHRES_IMU`) 各 100 Hz。现在分别运行的
`start_odin.sh` 和 `start_extnav.sh` 没有这些调用，且机载状态显示
`message_rates_configured=false`。该字段只是当前 `onboard_control_node` 是否完成过自己的设置序列，不是
从飞控读取 message interval 的结果，因此不能单凭它断言飞控未在发送。本次运行中 MAVROS 没有发布
local pose/velocity 样本是由多个接收窗口直接测得；如果此前地面站、旧 `odin.sh` 或其他路径成功配置
了 message interval，或者飞控按 `SRx_*` stream 配置发送了 message 32，同一个 `topic echo` 即使先
打印一次 buffer error，也仍可能随后收到 pose。

代码历史还表明，“以前能实飞”并不等于以前从未配置频率：仿真初始化和完整实机连接工作流从早期版本
起就在进入 local-position/飞行就绪门之前调用 `request_set_rates()`；旧 `odin.sh` 也执行三次相同的
MAVROS 服务请求。新的自动逻辑是把已有前置条件从地面站/脚本迁移到真正依赖它的机载节点，避免启动
路径和先后顺序决定是否配置，并非首次给系统加入这一要求。

用户指出“有时打印错误后仍能正常启动”后进行了追加只读复核：

- 不存在 Jazzy participant 时，Humble 长连接订阅 20 秒收到 pose 0 条。
- 加入一个零发布的 Jazzy 只读 participant 后，Humble 长连接订阅 45 秒仍收到 pose 0 条；同期
  `/mavros/state` 收到 43 条（0.954 Hz），中途还打印过一次 buffer error。
- Jazzy participant 存在时连续新建 8 次 `ros2 topic echo --once`：6 次打印 buffer error、2 次没有打印；
  当前 8 次都因上游无 pose 在 2 秒内超时。

这组证据修正了最初“打印错误后等待第一条不存在的样本”的过度概括：buffer error 本身是间歇性的，
并不等于 ROS executor、整个链路或该订阅必然死亡；当前 pose 缺失在打开 Jazzy participant 之前已经
存在。用户历史上成功的轮次很可能同时具有有效的 message 32 流，但本次没有捕获到“上游 pose 已知
持续发布”的正样本窗口，因此尚不能量化 DDS 错误是否会让某些新订阅者偶发匹配失败。

已把三类频率维护移到真正依赖它们的 `onboard_control_node`：FCU 每次连接/重连后自动异步设置，失败时
每 5 秒重试；地面站显式 `set_rates` 仍保留，但变为幂等确认。这一修正用于消除上游 message 32 是否
已配置的不确定性，不用于修复跨发行版 DDS discovery。后台维护不会发布伪造的地面站命令结果，也不
包含模式、解锁、起飞或参数写入。

## 8. 跨发行版问题的修复优先级

1. **立即规避（本次已完成代码）**：GUI 打开时保持 ROS IDLE，避免“只开窗口”触发远端异常；local pose
   必需的 MAVLink 频率改由机载节点自行维护。
2. **生产根治**：让网络边界两侧使用相同、受支持的 ROS 发行版和 RMW 组合。可选方案是把地面站 ROS
   薄客户端放入与实机一致的 Humble helper/container，Qt UI 留在开发机；或设计一个有明确消息白名单的
   受支持桥/gateway。
3. **不可作为根治**：换 domain 号、增大 UDP buffer、降低当前应用消息频率。这些可能改变症状，但不会
   让 Humble/Jazzy 跨发行版 DDS 变成受支持配置。
4. **试验性 RMW 切换**：两端统一 Cyclone DDS 可能改变 Fast CDR 症状，但当前两端没有安装对应 RMW，且
   跨发行版仍不受支持。未经独立台架和长时间验证，不应直接在已连接实机上安装/切换。

## 9. 本次代码与配置改动

- `src/onboard_control/src/onboard_control_node.cpp` / 头文件：
  - FCU 连接/重连后自动恢复三类 MAVLink 消息频率；
  - 参数同步宽限、延迟查询、超时提示和日志去重；
  - 显式频率请求可复用自动配置链。
- `ground_station_core/environment.py`：首次工作流按需启动 ROS；参数安全门等待扩为 60 秒。
- `ground_station_core/qt_ui/main_window.py`、`state.py`：默认 GUI 不加入 DDS，显示 `ROS IDLE`，但三个入口
  仍可点击并触发按需启动。
- `ground_station.env.example`、`onboard.env.example`、部署文档：说明 domain 0/42 的一致性要求和跨发行版
  限制。
- `start_ground.sh`、`start_drone.sh`：不再偷偷硬编码 42；保留调用者统一设置的 domain。
- Qt 与通信测试补充了“打开窗口不启动 ROS”和“首次工作流才启动 ROS”的回归。

## 10. 验证结果

- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：3 包成功。
- Python 全量：`50 passed`。
- `colcon test`：`5 tests, 0 errors, 0 failures, 0 skipped`。
- 隔离 fake-MAVROS 集成：首次连接与重连均收到
  `[(32, 100.0), (31, 100.0), (105, 100.0)]`；姿态设定点 `0`。
- `ground_station.py --check-environment`：通过（domain 231、localhost 隔离）。
- `compileall`、shell `bash -n`、Python 致命级 flake8、修改范围 `git diff --check`：通过。
- 全工作树 `git diff --check` 仍会命中用户原有 `TODO.md` 第 33 行尾随空格；本次没有擅自修改该用户内容。

## 11. 尚未完成/需要单独授权的事项

- 新机载二进制尚未部署到或重启于当前实机；当前实机仍在运行检查开始前的版本。
- 未在实机验证自动消息频率重连和 40–60 秒参数同步状态机。部署后应先拆桨、保持未武装，按只读状态、
  local pose 恢复、零 setpoint 的顺序验证。
- Humble/Jazzy 的 DDS 架构根治尚未实施；需要在“统一为同一发行版/RMW”与“增加受支持桥接”之间做部署
  选择，并单独进行台架测试。
