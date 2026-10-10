<!-- 本报告记录路径延迟适配、Odin 参数修复、完成航点后的源码行为及验证边界。 -->
# new-1-path-delay：规划延迟的控制层适配

日期：2026-10-10（Asia/Shanghai）。任务：`agent/task/new-1-path-delay.md`。

## 结果与范围

已实现控制层最近点接入：收到新鲜完整规划后，从全部连续三次段中选取距最新 FCU 三维位置最近的点，裁掉其前面的轨迹，并从此处按正时间跟踪。不会为了接入规划而先返回旧零秒起点。原有接入误差、停稳恢复、速度/加速度包线、身份和时效检查保留。

按任务要求启动两名 `gpt-6.1-sol / high` 子代理：一名修复 `blind_radius`，另一名只读解释最后航点后的行为。涉及实机的 SSH、部署、启停和测试全部由主代理完成。未解锁或起飞实机；解锁/起飞仅出现在 domain 231 / LOCALHOST 的真实 ArduPilot SITL。

用户进一步明确授权：仅允许修改独立库的 `path_planning/launch/odin.launch.py`。其算法源码、搜索器和碰撞算法均未改动。new SSH 连接 `192.168.112.169` 超时后，用户明确改为 refresh；new 未修改。

## 控制层实现

- `src/onboard_control/include/onboard_control/avoidance_trajectory.hpp`：加入全曲线最近时刻查询及前缀裁剪。对每段使用归一化时间 `u∈[0,1]`，检查段端点和距离平方导数 `(p(u)-actual)·p'(u)` 的全部实根候选；最高五次，采用导数分区与二分，包括降阶、常值段、多局部极值及重根候选。不是仅按离散路径采样点寻找起点。
- 等距候选选择较晚时刻。ARM 的 FMA 曾使数学重合点与 x86 出现几个浮点 ulp 差异；增加仅用于等距判断的数值容差，未改接入位置/速度的物理阈值。
- 裁掉完整旧段，解析平移首段：新 `c0=p(t*)`、`c1=v(t*)`、`c2=旧c2+3t*旧c3`、`c3=旧c3`，首段时长减去局部 `t*`。后续曲线、速度、加速度和时间尺度保持原样。已在终点时使用 1 微秒常值段保持，以满足现有服务的正时长段契约。
- `src/onboard_control/src/avoidance_execution.cpp:258`：在身份、时效及完整曲线校验后，用接收处理时最新 `vehicle_.position` 裁剪，再检查最近点的 p/v 接入容差。接受后的控制参考、VALIDATE_TRAJECTORY 请求、到达判定与 RViz Path 都消费同一裁剪结果和新零秒。
- `control.yaml` 仅更新两项接入阈值的注释，参数数值仍为 `0.08 m / 0.08 m/s`。补充 README 和增量测试选择，避免 `tests/run.py` 漏掉本次控制层及 C++ 避障回归。

该修复消除“追踪旧轨迹起点”造成的反向接入需求；不保证 PD 实际运动在所有偏差、超调或任意曲线几何下完全没有反向分量。超出最近点 p/v 容差的规划仍拒绝，已进入制动等待时仍要求停稳及持续成功窗口；此次没有改变等待/恢复策略。

## Odin 参数加载

原独立 `odin.launch.py` 把 launch 默认 `blind_radius=0.0` 无条件作为后置参数，从而覆盖 YAML。现在默认 launch 值为空，仅在用户显式输入时添加 override：

1. 未指定 launch 参数：使用选定 `config` YAML 的 `cloud.blind_radius`。
2. YAML 未提供该参数：使用 C++ 节点默认 `0.0`。
3. 显式 `blind_radius:=数值`：覆盖 YAML；显式 `0.0` 也有效。

正式 `avoidance_bridge/avoidance.launch.py` 继续关闭二次盲区滤波：它消费桥维护的累计观测地图，不能无条件套用原始扫描的近身滤波。独立 Odin 的启用半径依赖同坐标系、同源时基和时间匹配的里程计；配置有效不等于内部 SLAM 输入已过滤。未增加依赖或新部署方式，修改 launch 后仍需原生构建/安装并重启。

## 验证

| 验证 | 实际结果 |
| --- | --- |
| 开发机 Release 构建及 CTest | 4/4 套件通过；避障轨迹含 8 项用例，覆盖最近点、跨段裁剪、p/v/a 一致、终点保持、全局多极值和常值段 |
| ROS DDS 执行器与相关 GUI/运行回归 | 33/33 通过；新增延迟回包用例中旧路径起点 x=0，最新飞机位置 x=0.24，执行 Path/VALIDATE 段均从 x=0.24 开始，参考随后向前推进 |
| 增量测试选择 | 23/23 通过 |
| 独立 Odin 真实 launch 参数服务 | 6/6 场景通过：默认 YAML、YAML 0.63、缺省回退、显式 0.25、显式 0.0、嵌套 cloud.blind_radius；只测参数优先级，无合成扫描冒充实机 |
| ArduPilot SITL 绕障 | 成功完成航点并确认降落解除武装；到点拍照 1 次；最小实际圆柱净空 0.446594 m，最大侧向偏移 1.094095 m；最大参考/实际水平速度 0.577044 / 0.616905 m/s |
| 最终版本 SITL 遇障恢复 | 阻塞期间漂移 0.055403 m、拍照 0 次；移走障碍后 0.803532 s 恢复；任务完成拍照 1 次；最大参考/实际水平速度 0.827908 / 0.842932 m/s；最终降落解除武装 |
| refresh ARM64 原生构建与 CTest | 最终 Release 构建成功，4/4 套件通过 |
| 256 段最近点计算探针，100 次 | 开发机均值/最大值 0.536 / 0.544 ms；refresh 0.752 / 2.117 ms。仅为该固定曲线组的计算采样，不是全控制链最坏时延或硬实时证明 |

执行中如实保留的失败与修正：

- 初始数学用例在末点加速度由原曲线切换到静止的浮点边界比较，调整为内部时刻和明确越过终点分别验证；没有改物理断言或包线。
- 首轮 DDS 回包夹具的 0.12 s 延迟叠加 0.2 s 请求间隔，超过该夹具的 0.3 s 时效，正确触发等待超时；将夹具回包延迟改为 0.05 s，使测试专门验证接入而非时效失败。
- 一次与 SITL 并行的 DDS 测试被 SITL 环境清理终止，之后顺序执行全部 33 项通过。
- 首轮 SITL 飞行命令均成功，但脚本在另一话题的拍照事件尚未到达时立即断言失败；修复为最多等待 3 s 收到事件，仍要求恰好 1 次，后续绕障和恢复场景通过。
- refresh 首次 CTest 未 source ROS，Python 测试包装器缺 `ament_cmake_test`；加载环境后发现真实 ARM 等距用例失败（选 t=1 而非 t=7），修复数值等距判断后原生重建，全部通过。

## refresh 未解锁台架

开始时原有 Odin 与独立规划器运行，三个 systemd 服务均 inactive，MAVROS/extnav/onboard/正式桥未运行。先保存现场主工程和 planner launch/YAML：`/home/nvidia/ros2-ardupilot-maintenance/path-delay-20261010/`。选择性同步控制层四个源码/配置文件与独立 launch，保留相机、Tag CSV、飞控环境及 planner.yaml；control.yaml 核对只有两行注释差异。

使用真实 Odin、真实 FCU/MAVROS 和原生构建控制节点。读取 FCU 状态，并只向规划器发布几何目标及向桥发送规划查询；没有申请控制租约，没有航点飞行服务请求。采样时始终 `armed=false`、FCU `STABILIZE`、控制层 IDLE、航点数 0。

旧运行规划器读到半径 **0.35 m**，现场 YAML 实为 **0.8 m**。修复 launch 并原生安装重启后，参数服务读到 **0.8 m**，源码/安装 launch SHA256 一致。一次约 10 s 的短时采样记录 69 个 REACH_END、1 个 STALE_INPUT、30 个未发目标前 WAITING；末五次搜索约 37–54 ms。真实 raw 云最近点约 0.148 m，过滤云最近点约 1.285 m（以最新静止里程计计算，非源时间精配验收）。该结果说明 YAML 实际加载并存在成功规划，不代表每次规划成功、稳定 10 Hz 或航线可实飞。

**正式地图桥实际台架查询仍返回 NO_PATH/search failed。** 桥 ready 只说明输入/接口就绪，不说明有可执行路线；本次不放松 unknown、净空、外参或输入限制来使其通过。20° Odin 安装角、近身观测与障碍真实性风险仍见 MEMORY。移动飞机的延迟补偿由 DDS/连续数学测试和隔离 SITL 验证，桌面静止台架不具备实飞移动真值。

后续最终探针遇到原有 Odin 进程已退出、缺少 `/odin1/odometry`，失败记录保留；使用项目原入口重新启动真实 Odin 后继续检查，没有用历史消息或合成输入补足实机证据。最终检查和清理状态见下方补充。

最终重试读取 0.8 m，记录 91 个 REACH_END、1 个 STALE_INPUT；FCU 9 个状态均未解锁，控制层为 IDLE、航点数 0、桥 ready=true，实际桥查询仍 NO_PATH。该重试使用最终原生构建控制二进制。测试后清理本次新增的 MAVROS/extnav/onboard/正式桥进程，三个 systemd 服务保持 inactive；保留此前使用的独立 Odin 和修复后的独立规划器运行。

## 只读检查：最后一个航点完成后的行为

**正式机载入口：结束新规划/验证请求，继续末点悬停，组件本身保持运行。**

1. 控制层成功到点要求参考轨迹结束、距离 `<0.30 m`、实际速度 `≤0.15 m/s`，并保持 `0.3 s`。`onboard_control_node.cpp:2054` 发到点拍照事件，随后设置 `active_task_=kNone`，`reset_avoidance()` 清轨迹/请求并递增 task revision，进入 MODE_HOVER，参考速度/加速度/yaw rate 归零，发布成功终态。没有自动 LAND 或关闭节点。
2. `avoidance_execution.cpp:120` 在不处于航点任务/模式时返回，停止 PLAN、VALIDATE_TRAJECTORY、CHECK_LINE；晚到结果因任务/身份不匹配被拒绝，不能重新启动任务。控制层继续悬停输出；`avoidance_path` 仍定时发布空 Path 撤销旧显示。
3. 正式 launch 设置 `service_only=true`，独立规划器在 `path_planning.cpp:101` 不创建自动规划定时器；服务保持可用。桥仍维护观测地图/发布 readiness，规划器仍接收地图/发布 filtered_cloud。这些消息不是继续规划。

**单独运行独立 Odin launch：继续对最后收到的 goal 尝试规划和发布。** 默认 `service_only=false`、`planning_rate=10 Hz`；`path_planning.cpp:247` 把目标持续保存直到被替换，`:256` 每轮调用规划，`:337` 每轮发布 plan_result，仅非空成功路径发布 kino_path。没有“飞机已到点便停止”逻辑。REACH_END 表示计算出的曲线达到请求目标，不是飞机业务任务完成；输入失效时继续发失败状态，旧成功 kino_path 可能仍显示。

评价：正式模式把任务生命周期交给唯一控制权威，完成后停止计算请求但维持悬停和后续地图服务，合理。独立模式作为持续重规划台架工具也合理，但不能把持续路径发布理解为航点任务没有结束。末点悬停停止了轨迹验证，因此当前不具备任务完成后持续应对接近动态障碍的执行行为；此次仅检查，不新增功能。

## 证据与同步

开发机原始证据位于 `agent/codex/new-1-path-delay/`：`sitl-avoid-2.json`、`sitl-avoid-recover.json` 及 samples/log，`blind-radius/results.json`，`before-planner.json`、`planner-yaml.json`，投影探针 JSON。首轮失败证据没有覆盖。refresh 构建/运行日志、现场归档和实机探针位于上述 maintenance 目录。

主工程与独立 launch 分别提交并推送 main，工作区既有 TODO 和旧任务归档移动不纳入本次提交。refresh 选择性部署而非清空/重置现场 checkout；new 与 hp-desktop 尚未同步本次修改，下一次须与此前 Task38 欠同步一起处理。未引入依赖、未修改飞控参数/现场标定、未实现 TODO。
