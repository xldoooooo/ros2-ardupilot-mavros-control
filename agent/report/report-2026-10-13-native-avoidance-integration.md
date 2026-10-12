<!-- Native avoidance integration implementation, parameter differences and verification limits. -->
# 原生避障规划接入执行报告

日期：2026-10-13。对应 [dyn inte refine 任务](../task/4-dyn-inte-refine.md)及用户后续实施要求。

## 改动结果

正式桥与独立手动入口共用原生 dual_tree、目标/时间/位移/旧路径碰撞触发、搜索参数及 init=true/NO_PATH 再试规则。桥已删除持续观测地图、射线/unknown 检查、完整地图快照及地图版本重试；未观测区域通过检查按用户要求接受。

桥只做实际 FCU 状态/目标坐标转换、持久目标与原生结果身份适配。默认使用 `/mavros/local_position/odom`，将位置减权威杆臂 T 后送 `/avoidance/odom`，结果多项式常数项加 T。手动目标与地面站目标进入同一个规划节点；地面站通过 onboard 设置当前航点目标，GUI 不成为执行权威。独立纯 Odin 入口默认仍以 Odin 参考点规划，比较结果必须统一输入状态/坐标。

onboard 已删除自主避障的周期 PLAN/VALIDATE、重复规划包线和统一 retiming，直接用原始多项式的 p/v/a 与段时间做既有 PD+DOB 跟踪；最近点裁前缀保留。新 SetGoal 支持目标 ID、幂等与取消，单航点投递一次，普通搜索失败不由执行层重发目标。REACH_HORIZON 只作为中间段，尾前预留制动，不触发拍照/到点。

投递失败序号为0，不冒充已接受原生目标；ACK核对原生session/goal_revision，再转发搜索时间及有序结果，旧目标/坐标结果拒绝。原生idle/近目标空目标身份心跳支持单独重启恢复，不执行搜索；桥重启后仍可根据完整CANCEL身份撤销目标。近目标输入故障恢复允许一次实际状态规划，完整末段正常执行不要求持续重规划。

只修改避障库、桥、避障执行、相关接口/构建/仿真入口及其现有回归和说明。PD+DOB本体、直线参考算法、Tag/Odin安装角、视频、外部导航及用户现有 README/TODO/任务目录改动未改。

## 默认参数逐项差异

按用户最终要求分两类：直线速度XY/Z=1/0.2、加速度XY/Z=0.35/0.15；自主避障及遇障悬停速度XY/Z=1/0.2、加速度XY/Z=2/2。XY为水平合向量范数，Z为绝对值。避障速度保留原直线值，加速度保留原避障算法scalar数值2；旧逐轴2允许水平合2√2，新水平合限额2的语义已明确。用户确认PD+DOB控制输出4/4保持用于纠偏，键盘1.5/0.8保持。已按要求派子代理在独立工作区检查及实施。

| 参数或配置项 | 修改前 | 修改后 | 意义 |
| --- | --- | --- | --- |
| search.max_vel | 2.0 | 1.0 | m/s，旧逐轴scalar兼容值 |
| search.max_acc | 2.0 | 2.0（保留） | m/s²，旧逐轴scalar兼容值 |
| search.max_vel_xy | 无；x/y各2，合速度可达2√2 | 1.0 | m/s，水平合速度范数 |
| search.max_vel_z | 无；逐轴2.0 | 0.2 | m/s，垂直绝对值 |
| search.max_acc_xy | 无；x/y各2，合加速度可达2√2 | 2.0 | m/s²，水平合加速度范数 |
| search.max_acc_z | 无；逐轴2.0 | 2.0 | m/s²，垂直绝对值 |
| search.safe_distance核心默认 | 0.45 | 0.25 | m，对齐原公共YAML，实际YAML值未改 |
| search.search_budget核心默认 | 0.08 | 0.3 | s，对齐原公共YAML，实际YAML值未改 |
| search.lower.z核心/节点回退 | 0.2 | -2.0 | m，对齐原公共YAML |
| search.upper.z节点回退 | 5.0 | 10.0 | m，对齐原公共YAML/核心 |
| planning_frame节点回退 | camera_init | odom | 对齐原公共YAML |
| stamp_clock节点回退 | ros | receive | 对齐原公共YAML |
| service_only | 独立false、正式true | 删除 | 统一原生自主规划 |
| cloud.map_mode | 独立dual_tree、正式latest_observation | 仅dual_tree | 删除快照替换实现 |
| /avoidance/path_planning专属profile | true/latest_observation | 删除 | 正式入口继承公共参数 |
| avoidance_bridge.odometry_topic | /odin1/odometry_highfreq | /mavros/local_position/odom | 实际FCU参考点/速度 |
| avoidance_bridge.cloud_topic | /odin1/cloud_slam_down | 删除 | 云源统一为odin_launch.cloud |
| avoidance_bridge.voxel_size | 0.1 | 删除 | 桥不建图 |
| avoidance_bridge.map_radius | 20.0 | 删除 | 桥不建图 |
| avoidance_bridge.max_cells | 300000 | 删除 | 桥不建图 |
| avoidance_bridge.max_cloud_points | 500000 | 删除 | 原生search.max_cloud_points仍500000 |
| avoidance_max_velocity_xy | 1.0 | 删除 | 统一规划限额，不在执行层二次配置 |
| avoidance_max_velocity_z | 0.2 | 删除 | 同上 |
| avoidance_max_acceleration_xy | 0.35 | 删除 | 由原生实际加速度XY=2直接共享 |
| avoidance_max_acceleration_z | 0.15 | 删除 | 由原生实际加速度Z=2直接共享 |
| straight_max_velocity_xy | 无公共项；三个方法YAML均1 | 1.0 | 直线默认，m/s |
| straight_max_velocity_z | 无公共项；三个方法YAML均0.2 | 0.2 | 直线默认，m/s |
| straight_max_acceleration_xy | 无公共项；三个方法YAML均0.35 | 0.35 | 直线默认，m/s² |
| straight_max_acceleration_z | 无公共项；三个方法YAML均0.15 | 0.15 | 直线默认，m/s² |
| second_order_filter_max_velocity_xy/z、max_acceleration_xy/z | YAML显式1/.2/.35/.15；无YAML回退.30/.15/.18/.10 | YAML冗余项删除，默认继承straight组 | 旧方法参数可显式覆盖直线对照实验 |
| trapezoidal_max_velocity_xy/z、max_acceleration_xy/z | 同上 | 同上 | 同上 |
| s_curve_max_velocity_xy/z、max_acceleration_xy/z | 同上 | 同上 | 同上 |
| 遇障悬停参考最大速度XY/Z | 使用所选直线方法参数 | 原生实际1/.2 | native metadata同步，CLI覆盖也生效 |
| 遇障悬停参考加/减速度XY/Z | 加.35/.15、梯形/S减.4/.18 | 原生实际2/2 | 滤波/S jerk形状参数保留 |
| HORIZON段尾制动估计 | 直线梯形减速度.4/.18 | 原生实际2/2 | 加既有结果间隔预留 |
| PlanResult/AvoidanceResult实际限额字段 | 无 | max_velocity_xy/z、max_acceleration_xy/z四float64 | idle/失败/成功/几何查询均发布实际core值 |

避障四限额只在原生公共YAML配置，控制节点不另保存可调配置；桥转发实际core解析值，控制端据此构造遇障悬停参考和计算自主避障段尾制动。直线四限额独立配置，默认继承的新公共组保持原YAML运行值，直线分支不读取避障metadata。原直线梯形/S减速度.4/.18和S jerk .4/.16保持，因此表中.35/.15是正向加速限额，直线最大减速度仍可高于它；避障与遇障悬停加/减速度均取原生2/2。

scalar兼容规则为显式CLI→YAML→默认，同次显式方向CLI优先；例如 max_vel=2/max_vel_z=0.3 得到水平2、垂直0.3。普通原语端点及shot解析极值在原时间内验限，执行器不再偷偷改变曲线时长。输入状态超过配置限额时原生仍可返回INVALID_INPUT，实际飞机可能有跟踪超调，不能将参考上限等同实际硬上限。

保留公共净空0.25m、体素0.1m、horizon100m、碰撞步长0.05m、w_time10、lambda_heu5、搜索预算0.3s、输入超时0.5s、20帧双树周期、10Hz检查及0.2s/1m重规划。初始时长0.8s、20候选、零初始加速度、NO_PATH重试保持。max_tau仍为原生0.6s；此前低加速度试验的1.2s已撤回。高加速度/低垂直速度导致原±2/±1候选全超限，现只在half-acc*tau超过轴速度cap时按当前速度推导可行正负幅值，保留每轴5值/125组合；默认仅Z需要适配，不降低加速度配置、不改hash/代价/净空。首init同cell短候选保留临时节点，直接修复闭合root误裁剪，有源码解释注释；常规位置hash仍保留原局限。

## 验证结果

仅开发机构建与隔离本地验证；硬件均未连接、解锁、起飞。真实ArduPilot仅domain231/LOCALHOST仿真。

- 两工作区Release构建/安装成功，新接口及消费端同步重建；build_avoidance.sh包含native、接口、桥、onboard、guided_sim。
- 原生最终core22/22、DDS/ROS/统计18/18、CTest2/2；包括远混合/纯Z目标、所有结果实际限额及原段时长。主工程CTest4/4通过。
- 主工程执行器4/4，25.68s；现有悬停恢复例用metadata覆盖为速度.6/.2、加速度1.7/1.4，实际参考达到XY加速度1.7及速度.6，证明继承native实际值。保留单目标流、最近点原时间裁剪、运动中替换、乱序/取消、末段无需刷新、LAND重试等原验收。
- 本轮执行器首跑一例fixture启动等待超时，另3通过；完整复验4/4通过。此前亦出现过孤立时序失败，不能声称长时稳定性已证明。
- 真实native桥identity/extnav各1例通过，12.686/12.982s；包括各进程独立重启、无缓存CANCEL、旧坐标拒绝、手动目标替换撤销业务参考、原曲线/实际metadata和无setpoint发布者。
- 三入口参数18组通过（单个脚本1test、16.859s），CLI/YAML优先级及坏配置一致。此前相关GUI/预览/仿真入口28例通过；本轮GUI未改。
- 最终原曲线滚动4场全部REACH_END；三维目标(3,.5,2.4)的.1/.2s执行前缀分别482/116次成功，48.1/23.0s到目标附近。完整曲线5ms独立验限满足1/.2/2/2；质心最小.250009m、raw最小.232862m。raw表面判据与体素质心判据不混称。
- 新参数真实SITL绕障：航点成功、拍照1次、LAND成功解除武装；实际最大侧移.75752m、最小圆柱表面净空.27473m，**旧脚本>0.35m仍失败**。最大执行参考vxy/vz=.44168/.19501、axy/az=1.41422/.49714，实际vxy=.45016；过程中有2次INVALID_INPUT并恢复，实际速度超调仍可能触发原生输入拒绝。
- 最终真实SITL直线通过：参考水平速度1，正向加速.35、减速.4（原独立减速参数保持），拍照1及LAND；无需规划器。实际水平速度最大1.06174，参考上限不是实际硬上限。
- 最终遇障悬停恢复通过：等待漂移.05451m、等待拍照0，移除后3.21626s恢复、拍照1及LAND。执行参考水平速度1、加/减速度2，实际水平速度最大1.40721，较宽松参考下纠偏超调仍存在。
- 最终自主避障恢复通过：等待漂移.05494m、等待拍照0，移除后4.22039s恢复、拍照1及LAND。参考/实际水平速度最大.65799/.67354。

旧低加速度临时试验保留作为历史：首轮和首init修复后曾NO_PATH/LAND；两轮航点及拍照完成但表面净空.3015/.2426m未过旧>.35m；另自主恢复等待漂移.05546m、移除后3.014s恢复、拍照1及LAND通过。旧.35/.15加速度与临时max_tau=1.2不再是当前基线，不能引用为新版验收。

主过程数据位于agent/codex/native-avoidance-refine；最终原曲线滚动位于独立库agent/codex/avoidance-independent-profile。独立逐项报告见同级库agent/report/report-2026-10-13-独立避障动力学与实时限额共享.md。原严格阈值与所有失败证据完整保留，不以启动/话题可见或合成测试替代实飞。

## 部署差异

本次仅开发机源码/原生安装更新，refresh和new都待同步两个仓库及重建共享接口消费者，主MEMORY与独立MEMORY已标记。保留逐机配置，不能照搬相机/Tag/串口。Tag有效分支、Odin20°安装角、现场长时负载与实飞未在此次验收。
