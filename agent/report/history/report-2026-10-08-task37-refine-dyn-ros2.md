<!-- Task37 execution record: independent ROS2 component fixes, reproducible tests and deployment evidence. -->
# Task37：独立避障 ROS2 组件修正

日期：2026-10-08。任务依据：`agent/task/37-refine-dyn-ros2.md` 及同目录审查报告。

## 基线与范围

- 独立仓库：`/home/nvidia/scq/projects/dyn_small_obs_avoidance-ros2`，修改前
  `main=d2e3a694a13de14d3af8969dfa0d468d23650d1c`，工作树干净。
- 修改前已全量归档整个目录，包含 `.git`、源码、`.venv`、构建及安装产物：
  `agent/codex/task37/dyn_small_obs_avoidance-ros2-full-20261008-161219.tar.gz`。
  SHA-256：`60b222e659b6c5814a853d6a114c05b8c37a083bb923f0c1d428fbd10cd5b66b`。
  已检查归档目录列表；原始压缩包只保留本地，不加入 Git。
- 按任务明确要求，并行委派三名 `gpt-6.1-sol high` 子代理负责 wrapper、核心、基准与文档。
  主代理负责协调、统一构建、验收和记录。没有遇到需要 `gpt-6-astra high` 排查的持续分歧。
- 直接实施审查确认的 B01–B05。保留两个独立 ament 包，不接入主工程飞行控制。
  审查 A01–A28 的策略差异和执行接轨、动态预测、ROS1 派生参考与闭环方案需分别记录，
  不把这些建议系统建设项声称为已实现或已验收。
- 主工程任务文件的既有移动/删除和既有未跟踪报告属于用户现场改动，本任务不覆盖或提交它们。

## 实施与验证

源码固定提交：独立仓库 `afc868b`，保留原单参数 `sampleTrajectory(double)` ABI，
新增带采样数量上限的重载。具体修改如下：

| 审查项 | 实施结果 |
| --- | --- |
| B01 | XYZ 名称严格唯一，类型/count/offset/点步长正确且三字段不重叠；PCL 转换前拒绝，兼容其他字段、乱序、点和行 padding |
| B02 | 所有输出构造结束后，唯一成功发布门控再次检查点云/里程计时效；失败清本轮系数、轨迹和 Path，保留最后成功 RViz 预览 |
| B03 | 用整数枚举固定 125 个常规控制组合、20 个初始时长，检查加速度和派生数值可表示性；分配前核查整条轨迹采样量 |
| B04 | 统一文档：PlanResult 为有效性权威，kino_path 是旧时标不刷新的最后成功预览 |
| B05 | benchmark 默认加载正式安装 YAML，支持显式预算；读取运行节点完整参数并导出源码/配置/二进制哈希、平台依赖和全部结果摘要 |

wrapper 默认输出上限为 10,000 点、60 s 曲线时长、0.1 s 构造预算，均可启动时配置。
点数/时长超限返回 INVALID_INPUT，构造超时返回 TIMEOUT；不截断或伪造成功。
预算检查为合作式，不包含 DDS 排队，单次分配/采样可能超出预算后才返回。

### 本地测试

- 两包 Release 原生构建完成。唯一构建警告是系统 PCL/FindFLANN 的 CMake CMP0144 开发者警告。
- 正式 `colcon test`：36 tests，0 errors/failures/skipped；实际逻辑用例为
  **18 核心 + 6 输出门控 + 5 ROS 进程 + 3 基准工具 = 32**，另有 4 个 CTest 包装项。
- ASan/UBSan 并启用泄漏检测：18 核心、6 输出门控、5 实际 ROS 进程测试均通过。
  系统 PCL 和 ROS 依赖本体仍为发行版二进制，没有重编译插桩。
- DDS 用例实际注入 11 类畸形 schema（包括坏字段在前/在后的重复 XYZ、合法重复、越界、
  重叠、错误类型/count、空数据与短行），以及 NaN/±Inf；恢复和合法扩展 padding 均验证。
- B02 用同一个生产发布 helper 和受控时钟，分别确定性验证点云/里程计在构造阶段过期、
  快速成功、构造超时、过期优先与 horizon 门控。真实 DDS 另验证输出限制失败无轨迹；
  没有向运行节点添加测试延迟参数，受控时钟测试不宣称是完整 DDS 中的时钟注入。
- 极小 max_acc、init_max_tau、denormal/最大数值边界、动态搜索实际多个时间层、
  第 49/50/51/99/100/101 次换树和节点池最后槽均有真实核心回归。

日志位于 `agent/codex/task37/`：`colcon-release-test.log`、`colcon-release-results.log`、
`core-sanitized-test.log`、`output-contract-sanitized-test.log`、`pytest-sanitized.log`。

### 保留的失败

首次工具 smoke 和首次 DDS 测试在两包安装尚未完成时运行，旧 wrapper 缺少更新后的采样符号，
首轮 DDS 为 5 failed/3 passed（均为节点符号加载失败）。保留 `benchmark-tool-smoke.*`、
`pytest-release.*` 与首轮进程日志。随后补保留原单参数 ABI、完成两包安装并重跑，
最终 DDS/基准工具 8 passed，正式 colcon 与 sanitizer 全部通过。
这次失败是构建/测试编排问题，不是成功轨迹或避障性能结果。

### 性能

固定源码 `afc868b`（各次 benchmark 的 Git 状态为 clean）、正式 YAML 与实际参数读回绑定。
各次运行 30 s，统计排除供流前 1 s；该秒与启动等待状态仍保留在原始 JSON 中。

| 平台/预算 | 统计期结果 | 搜索 P50/P95/P99/max（ms） | 结果接收间隔 P50/P95/P99/max（ms） |
| --- | --- | --- | --- |
| amd64 / 80 ms | 290 REACH_END，0 失败 | 41.95/43.97/44.82/51.16 | 99.98/102.51/109.22/120.39 |
| amd64 / 300 ms（直接加载默认 YAML） | 290 REACH_END，0 失败 | 42.47/44.07/44.42/50.63 | 100.00/102.29/109.29/120.69 |
| ARM64 / 80 ms | 289 REACH_END，1 TIMEOUT | 68.98/70.99/72.42/80.12 | 100.04/109.53/133.86/136.34 |
| ARM64 / 300 ms（直接加载默认 YAML） | 290 REACH_END，0 失败 | 68.71/70.56/71.14/87.91 | 100.02/108.04/133.39/134.68 |

上述合成绕杆地图只有 402 点，不能代表数万真实 Odin 点下的成功搜索性能。
amd64 两组 CPU 约 0.393/0.397 核，ARM64 两组约 0.667/0.665 核；80 ms 组连续失败约 0.102 s。
各次完整失败、输入/结果年龄、CPU/RSS、序号与参数证据保留在 `benchmark-*-*ms.json`。
结果接收间隔 P99 未达到 100 ms；这些指标也不是传感器到飞行执行器的控制端到端指标。

本地环境：Ubuntu 24.04 amd64，Jazzy，GCC 13.3，CMake 3.28.3，PCL 1.14.0，Eigen 3.4.0；
Python 使用独立仓库 `.venv/bin/python`。环境记录位于 `agent/codex/task37/local-baseline.txt`；
其中工作树状态采集在子代理开始修改之后，修改前干净状态以原始检查和全量归档为准。

## 实机部署与现场规划验证

首次连接 `ssh drone-new`（解析为 `192.168.112.169`）失败：
`ssh: connect to host 192.168.112.169 port 22: No route to host`。
已向用户询问上线状态/正确地址，停止依赖实机连接的工作，继续本地独立任务；未改连 refresh。

用户通知飞机上线后重新连接成功。直接读取 MAVROS 状态为 connected=true、armed=false、
mode=STABILIZE；DDS 发现需要较长等待，使用直接话题发现和显式消息类型后读取成功。
未重启 MAVROS、Odin 或任何生产服务。

new 修改前独立库仍为 `1c87d08` 加现场未提交改动；已完成远端整个目录的全量备份：
`/home/nvidia/ros2-ardupilot-mavros-control/agent/codex/task37/dyn-ros2-predeploy-full-20261008-162534.tar.gz`，
SHA-256：`6e0abd4c6770a364f9a6044d8a0ce227be35af4e81b33d53ebe4ff35d2a8e5fe`。

发现现场 YAML safe_distance 已改为 0.35 m，核心公式另把体素补偿项乘成 0，
实际有效半径为 0.375 m；这与独立仓库标准实现不同。没有默默覆盖或把这种现场策略当成本次
修复成功。已按 AGENTS.md 第 12 条暂停覆盖并询问；用户明确回复：
“更新原目录，现场改动是临时修改的，不需要保留或保护。”
随后在原目录撤销这四个已识别临时修改，将未跟踪旧 Odin launch 移至证据目录，
通过已验证 Git bundle 快进 `main` 到 `afc868b`（GitHub main 已推送同一提交），没有改写历史。
bundle SHA-256：`af86d0356bf68b999d642e7b39a8162fe40696610c2bed6612065b0456cd24ed`。
标准实现使用 0.45 m 默认距离，含体素/采样补偿后约 0.6482 m；源码及 YAML 已现场核对。

new 已完成两个包的 ARM64 原生 Release 构建，以及正式 colcon 36 tests、0 errors/failures/skipped，
同样对应 32 个逻辑用例。Python 使用飞机独立库自身 `.venv`，测试在 domain 232/localhost 隔离。
原有 MAVROS/Odin/onboard_control PID 保持 3625/3600/3597，未停止或重启这些生产服务。

### 两轮实际 Odin 输入

单独启动本任务规划进程，消费 `/odin1/cloud_slam`、`/odin1/odometry`，
仅将目标发布到 `/task37/goal`。参数读回：frame=odom、stamp_clock=receive、
blind_radius=0.5 m、search_budget=0.3 s、z 下界 -2 m；正式 YAML 的碰撞参数未放宽。
盲区为既有显式台架配置，会忽略球内真实障碍，不能当作精确机体分类。

| 指标 | 首轮：同高前向4 m | 第二轮：前向4 m、目标高度 +0.4 m |
| --- | --- | --- |
| 目标位置（odom，m） | (4.001847,0.019015,-0.002167) | (4.003012,0.023579,0.397814) |
| 目标观测 | 60 s | 60.018 s |
| 属于该目标版本的状态 | 599 NO_PATH + 2 NO_MAP | 599 REACH_END + 1 STALE_INPUT |
| 原始/过滤点云接收帧数 | 591 / 576 | 558 / 524 |
| 单帧原始点数范围 | 35781～35918 | 35769～35911 |
| 累计地图峰值点数 | 37091 | 36838 |
| 观测FCU状态 | 63条均armed=false | 61条均armed=false |
| 搜索P50/P95/P99/max（ms） | 0.08/0.11/0.13/0.16 | 71.24/86.71/93.37/97.29 |
| 接收间隔P50/P95/P99/max（ms） | 100.02/106.56/115.29/121.86 | 100.06/114.80/144.38/152.20 |
| 尝试开始到接收年龄P50/P95/P99/max（ms） | 1.23/3.63/4.16/5.59 | 83.43/104.92/114.90/143.93 |

首轮的 NO_MAP 是启动前尚未收到点云，后续 NO_PATH 保留全部记录。与最后结果同一源时标
`1271239747658` 的35855点原始/过滤云显示：起点最近距离1.548396 m，目标最近距离
0.472943 m，小于0.648205 m有效半径；属于目标占用拒绝，快速拒绝不代表成功搜索性能。

为验证现场可用目标，先对未改变的原始点云检查不同目标高度，再单独请求高度 +0.4 m。
没有改变安全距离、补偿或过滤策略，也没有把这个追加目标的成功当成原同高目标的成功。
第二轮换目标前的11次旧目标 NO_PATH 同样保留。599条成功结果均有轨迹，客户端另观测
598条非空Path（独立DDS话题接收数），没有空Path；所有失败结果均为空。
1次 STALE_INPUT 的完整原因为 `blind filter timed out waiting for matched odometry`，
segments与trajectory点数均为0。不能声称输入抖动已完全消除。

原始事件、参数、输入快照与摘要在 `agent/codex/task37/live-20261008-standard/`
和 `agent/codex/task37/live-20261008-raised-goal/`。`events.jsonl` 保留全部状态，
`stats.json`记录统计口径；NPZ保存有界捕获帧窗口，没有声称逐帧保存整轮全部点云。
receive模式的设备源时标不与主机ROS时间混算，真实源输入年龄未知。

### 首条成功曲线独立复核

冻结第一条 REACH_END 对应的完整多项式、实际里程计、目标与原始/过滤输入，
独立检查代码不调用核心 `isSafe()`。曲线总时长3.721652 s：

- 逐轴速度最大绝对值 `[1.800229,0.019791,0.600234] m/s`，加速度
  `[2,0.061100,1] m/s²`，均在正式逐轴2 m/s、2 m/s²约束内；位置边界检查通过。
- 最大段间位置差4.44e-16 m、速度差0；初始位置误差0、速度误差1.15e-19 m/s；
  终点位置误差3.47e-18 m、终端速度范数4.48e-16 m/s，C1与端点约束通过。
- 10009个曲线时间采样对573597个捕获原始点最小距离0.660981 m，
  对358476个过滤点最小距离0.683488 m，均大于0.648205 m有效半径。
- 对同一573597个完整原始点逐点计算到起终点闭直线段的精确距离，最小0.625551 m，
  小于有效半径。多项式曲线相对直线段最大偏移0.142119 m（解析驻点与端点检查），
  可观察到曲线上抬避开与直线更近的点区域。直线距离仍大于原始0.45 m安全距离，
  不能声称该直线撞上原始障碍，或核心实际曾拒绝这条直线。

复核数据：`live-20261008-raised-goal/offline_validation.json`、`first_success.json`、
`first_success_inputs.npz`；图和几何数据：`agent/codex/task37/live-geometry.png`、
`live-geometry.json`。图仅为显示裁剪，不影响全量原始点距离计算。

![现场首条成功曲线与原始点云（odom，目标高度+0.4 m）](/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware/agent/codex/task37/live-geometry.png)

以上捕获帧联合不是精确PCL双bank快照：DDS可能漏帧，累计历史未导出，
审计明确 `map_history_exact=false`。有限曲线采样不是连续时间碰撞证明；点距离也不证明
未观测区域自由。本次可确认真实点数下有目标轨迹输出及捕获云上的独立复核通过，
不等于飞机已沿该曲线完成避障。

### 最终状态与未完成指标

- 本任务临时planner进程组10089/节点10103已SIGINT停止并核对退出；Odin launch 3508、
  Odin 3600、onboard_control 3597、MAVROS 3625保持原PID，未停止或重启生产服务。
- 最终直接回读FCU为 connected=true、armed=false、STABILIZE。
  全程没有执行实机解锁、起飞、模式切换、setpoint或其他飞行指令。
- 独立库代码 `afc868b` 与实测文档 `bde0647` 已推送 main；new 原目录同步同一文档提交，
  文档更新不改变已测源码/安装二进制。refresh 本次未连接、未部署，后续需整体补同步。
- 主工程只更新本报告与 MEMORY；独立性保留，不实现 TODO.md 或飞行执行器集成。
- 仍未实现/验收：审查建议的ROS1修复参考对照harness、观察覆盖、动态障碍预测、执行器接轨、
  全链P99≤100 ms、30分钟热稳态、SITL路径跟踪和实飞闭环。同高原目标仍失败；
  第二轮一次STALE_INPUT及ARM64 80 ms一次TIMEOUT均如实保留。
