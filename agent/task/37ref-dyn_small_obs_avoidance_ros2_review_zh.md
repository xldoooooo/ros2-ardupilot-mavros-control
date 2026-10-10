# dyn_small_obs_avoidance ROS1 / ROS2 对照审查报告

**主题：path_planning 重设计原因、运行流程取舍、算法结果对齐方案与待修复问题**

- 报告日期：2026-10-08（Asia/Shanghai）
- ROS1 基准：hku-mars/dyn_small_obs_avoidance，提交 **9d25dd6974e04518c892e5711c9be9082558e5eb**
- ROS2 基准：LostPatrol/dyn_small_obs_avoidance-ros2，提交 **d2e3a694a13de14d3af8969dfa0d468d23650d1c**
- 本次重新核对两个 main 分支，仍指向上述提交。
- 审查范围：path_searching、path_planning、相关参数、launch、消息、测试、移植与验证文档，以及异常点云转换涉及的 PCL 代码。
- 本报告中的兼容模式、状态机和修复方案均为**建议设计**；不表示当前仓库已经实现。本次交付为审查报告，未修改远端仓库。

## 阅读导航

1. 结论与证据边界
2. 为什么 path_planning 被重新设计
3. ROS1 的实际运行流程及状态死角
4. ROS2 的实际运行流程
5. 两个流程谁更好
6. “尽量复现 ROS1 结果”应如何定义
7. 必须逐项核对的行为差异
8. 推荐的兼容实现方案
9. ROS2 待修复问题清单
10. 差分回归与可靠性验收方案
11. 建议实施顺序
12. 来源索引与最小复现记录

---

## 1. 结论与证据边界

### 1.1 三个核心结论

**第一，path_searching 保留了上游算法主体，但 path_planning 不是逐行为移植。** ROS2 仓库初始导入的搜索头文件和源文件，其 Git blob 与上游一致；当前仍保留双积分器、常加速度原语、控制能量与时间代价、原启发式主要公式、三次终端连接、位置 hash 和双 KD-Tree 积累机制。后续主动改变了碰撞策略、终止语义和调度方式。[H01] [R02] [N02] [N16]

**第二，重设计主要是为了独立化和修复旧演示节点的耦合与状态问题，并非 ROS2 强制要求。** 上游 wrapper 接入 MAVROS 接口，并在收到执行索引时使用旧参考路径；新版把这些接口和分支移出模块，围绕已配准点云、里程计和位置目标建立独立节点。这一范围在项目自己的 THIRD_PARTY_NOTICES、PORTING 和 USAGE 中已有明确说明。[N12] [N14] [N16]

**第三，新版更适合作为可维护的独立组件；旧版“保留有效路径、必要时重规划、利用执行进度衔接”的意图更值得作为执行层设计参考。** 两者现有代码都不足以直接当作已经验收的闭环飞行系统。建议保留新版的输入防护、原子结果和清晰依赖，再补回经过修正的事件驱动与执行接续机制，而不是整体回滚旧 wrapper。

### 1.2 本报告怎样区分证据

| 标记 | 含义 | 本次例子 |
| --- | --- | --- |
| 源码确认 | 能从固定版本代码直接确认 | 新版固定 init=false；失败不清除可视化 Path |
| 最小复现 | 独立编译运行局部原样或等价抽取逻辑 | 小参数候选枚举；重复 XYZ 字段的 ASan 越界读取 |
| 作者记录 | 仓库文档声称完成，本次没有重新跑完整流程 | 两平台构建、12 项核心回归、Jetson 基准 |
| 条件推导 | 由代码路径和给定条件推出，尚未做完整系统实测 | 输出构造期间输入过期仍发布成功 |
| 设计建议 | 本报告提出的方案，尚未在当前仓库实现 | 参考模式、执行状态机、未来接轨点、差分测试 |

本次读取了相关源码与测试，复核了提交和导入关系，并执行了局部 C++ 复现。当前环境缺少完整 ROS2/PCL/colcon/CMake/gtest，因此**没有独立完成整包构建、仓库 gtest/pytest、DDS 回归、bag 回放、SITL 或实飞**。局部 ASan 结果不应写成“已经在完整 ROS2 节点中复现崩溃”；源码未发现默认场景错误也不等于证明其不存在。

---

## 2. 为什么 path_planning 被重新设计

### 2.1 哪些原因有项目自身的文字证据

| 有明确文档依据的决定 | 对应的代码改变 | 目的 |
| --- | --- | --- |
| 只保留搜索与规划组件，不包含传感器、SLAM、飞行演示控制器 | 移除 MAVROS 与外部 trajectory_time_index 依赖 | 让节点可单独构建、回放、接入不同状态估计器 |
| ROS API 留在 wrapper，核心只依赖普通 C++、Eigen、PCL | SearchConfig 替代核心 ROS 参数读取 | 降低核心对中间件的依赖 |
| 不保留旧演示器索引协议，改从实际状态重规划 | 最近里程计 p/v 作为每次搜索初态 | 消除外部索引越界及协议缺失带来的问题 |
| 修复起点 z 速度误写、未初始化、旧结果失效等问题 | 输入状态标志、有限性检查、失败空轨迹 | 让异常结果可检测、可恢复 |
| 输出精确多项式及原子状态 | PlanResult 同时携带状态、版本、输入时标和轨迹 | 避免消费者拼接多个不同步话题 |
| 不包含接轨、HOLD/LAND、yaw、控制租约等执行职责 | 输出独立规划结果 | 控制接入由外部完成 |

来源：PORTING:8–37、THIRD_PARTY_NOTICES:10–15、USAGE:133–156。[N12] [N16] [N14]

### 2.2 哪些属于本报告的工程推断

从上述变化可以推断，作者希望把“依赖特定演示环境才能工作的节点”收敛成“输入明确、输出明确、可独立测试的 ROS2 组件”。固定计时器也自然带来两个效果：

- 目标早于传感器到来时，输入后来齐备后仍会规划。
- 无路或超时之后，只要输入恢复有效，后续周期会继续尝试。

这些效果由新源码支持，但**不能把它们写成作者未明确表达过的个人动机**。报告将它们作为设计效果与合理推断，而非访问过作者或得到作者确认的结论。[N01:213–237](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L213-L237)

### 2.3 ROS2 本身并不要求改变规划策略

从这份代码的依赖关系看，以下工作足以构成通常意义上的中间件迁移：

- catkin 改为 ament；
- ROS1 消息与发布订阅 API 改为 ROS2 对应接口；
- 参数、时钟、QoS、launch 和消息生成适配；
- 将 callback 接入原有 plan/replan 逻辑。

**把点云事件触发改为固定频率全量搜索、从旧参考状态改为最近实测状态、把 init=true/false 两次尝试改成仅 false，都是算法编排决定，并非编译或 ROS2 API 的必然要求。**

因此，这个仓库同时完成了“移植”和“职责重划分”。这有工程合理性，也使其不能再声称完整复现上游 wrapper 的运行行为。

### 2.4 重设计的代价

新版省去了对旧执行器协议的依赖，但也省去了旧版中与执行进度相关的调度意图。现在的原子结果和多项式输出解决了“结果如何表达”，并没有解决“什么时候切换、以什么初态切换、旧轨迹如何继续、失败时如何制动”。

这不是单纯缺几个话题名的问题：执行状态、时间基准、轨迹有效期和控制反馈需要形成一致协议。项目自身也明确承认尚未提供这些内容。[N07] [N14:153–156](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/USAGE.md#L153-L156)

---

## 3. ROS1 的实际运行流程及状态死角

以下按源码实际行为描述；涉及未初始化数据的分支不能被当作可靠、确定的参考行为。

### 3.1 四类主要回调

1. **里程计回调**：更新 start_pt、start_vel、uav_odom 和全局 cur_pos，设置 set_start。它本身不调用 plan。
2. **目标回调**：setGoal 中的旧比较条件成立后才写入目标；如果已经 set_start，立即调用 plan。比较使用了未初始化状态，见第 3.4 节。
3. **点云回调**：先取原点云下标 0、2、4……，再按当时 cur_pos 裁剪逐轴 ±20 m 立方体，更新 KD-Tree，调用 replan。
4. **执行索引回调**：只保存 trajectory_time_index，并不立即重规划。

注意点云不是“先裁剪后隔点”，也不是“20 m 球形裁剪”；顺序与形状都会影响送入地图的点集合。[R01:52–89,364–425](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L52-L425)

~~~mermaid
flowchart TD
    O["里程计回调"] --> U["更新实测 p/v 与 set_start"]
    G["目标回调"] --> V["旧比较成立后写目标并检查 set_start"]
    V -->|已就绪| P["进入 plan"]
    V -->|未就绪| W["本次不规划"]
    C["点云回调"] --> F["隔点取样和 20 m 立方体裁剪"]
    F --> M["更新双 KD-Tree"]
    M --> D{"距目标小于 0.3 m？"}
    D -->|是| X["返回"]
    D -->|否| E{"已有路径且 set_start？"}
    E -->|否| X
    E -->|是| K["检查 replan_flag；每隔 40 点检查碰撞"]
    K --> H{"需要重规划？"}
    H -->|是| P
    H -->|否| X
    I["执行索引回调"] --> S["只更新索引"]
~~~

### 3.2 plan 内部并非始终从旧参考轨迹出发

旧版有两条起点选择分支：

- **首次规划，或执行索引仍为 -1**：使用最近里程计位置和速度。旧代码没有验证其他索引是否真正有效。
- **已有成功规划且收到非 -1 执行索引**：使用旧轨迹上的参考点；通常取 index+3，靠近末端时退回 index，并用附近路径点差分估计速度。

两条分支都向搜索传入 start_acc=0、goal_v=0；先 search(init=true)，仅返回 NO_PATH 时才 reset 并重试 init=false。init=true 限定首轮原语，但近目标时仍可直接进入 shot，不能由此推断输出轨迹的起始加速度必为零。NEAR_END 和 REACH_HORIZON 在 wrapper 中没有被当成需要再次求完整路径的失败处理。[R01:131–241](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L131-L241) [R02:193–237](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L193-L237)

~~~mermaid
flowchart TD
    P["进入 plan"] --> B{"首次规划或执行索引为 -1？"}
    B -->|是| A["用实测位置和速度"]
    B -->|否| D{"索引变化至少 10？"}
    D -->|否| X["直接返回"]
    D -->|是| R["旧轨迹取参考点并差分估速"]
    A --> T["传入 start_acc=0、goal_v=0"]
    R --> T
    T --> S["搜索 init=true"]
    S --> N{"返回 NO_PATH？"}
    N -->|是| F["重置后搜索 init=false"]
    N -->|否| E["导出路径"]
    F --> Q{"仍为 NO_PATH？"}
    Q -->|是| X
    Q -->|否| E
    E --> Y["以 0.01 s 采样并发布演示接口"]
~~~

### 3.3 旧版接续意图有价值，但实现不保证连续

旧版希望利用执行进度，在正在执行的参考轨迹前方重新规划。这比完全忽略当前执行轨迹更接近“轨迹替换”的需要，但不能称为已经完整解决接轨：

- index+3 只相当于**名义约 30 ms**，不等于按实测计算和通信延迟选定的未来时间。
- 速度使用固定 0.01 s 分母差分估计；旧采样存在边界重复和 shot 端点缺失，并不是完整的均匀时间序列。
- 传入的 start_acc 每次设零，并非旧曲线接点处的真实加速度；该参数也不保证所有搜索分支的输出起始加速度。
- 没有明确的未来切换时间、旧前缀保留、切换确认和跨进程有效期协议。
- 使用旧参考状态还隐含“车辆跟踪误差足够小”的条件；如果车辆已偏离参考，参考状态不能直接代表实际状态。

所以，应保留的是“规划应考虑执行进度与衔接状态”的设计意图，而不是照搬 +3、差分估速及索引门限。[R01:173–229,243–326](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L173-L326) [R02:754–796](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L754-L796)

### 3.4 旧版存在三个影响恢复和目标响应的状态死角

| 情况 | 源码路径 | 结果 |
| --- | --- | --- |
| 目标先到，里程计后到 | setGoal 在未 set_start 时不 plan；后续 odom 只更新状态；点云 replan 又要求已有路径 | 不一定会自动开始第一次规划 |
| 首次两次搜索均 NO_PATH | plan 直接返回，path_size 仍为 0；新点云 replan 因无旧路径而跳过 | 障碍后来移开，也未必自动重试 |
| 有旧路径和执行索引时收到新目标 | 新目标调用 plan，但索引变化小于 10 时直接返回；之后索引回调不触发 plan；旧路径若仍安全，点云也不触发 | 新目标可能长期得不到规划，需目标重发或其他触发 |

这里的第一、第三项还受 setGoal 的未初始化比较影响：prev_goal 从未初始化或更新，prev_start 初始也未定义。因此上述表格是**在目标确实进入 setGoal 更新路径的条件下**进一步指出的调度缺陷，不是假定原代码一定能稳定走到这些分支。[R01:70–89,101–129,143–188,335–425](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L70-L425)

这些问题解释了为什么新版“持续检查条件、自动再次尝试”的结构，在节点可用性上有明显优势。

---

## 4. ROS2 的实际运行流程

### 4.1 输入回调负责维护最新状态和地图

- 点云：严格 frame 与时间戳检查、布局检查、转换为 XYZ、可选盲区过滤、双树积累。
- 里程计：检查数值与四元数；默认把 child-frame 线速度旋转到世界系；保存最近 p/v。
- 目标：保存目标，增加 goal_revision；不直接搜索。
- 如果启用盲区过滤，点云最多缓冲一帧，等待按源时间戳匹配的里程计。**该匹配服务于盲区球心计算，不等于整条规划链已经完成状态同步或预测。**[N01:103–211](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L103-L211)

### 4.2 固定计时器负责搜索和发布

当前默认每 100 ms 尝试一次：

1. 处理待匹配点云。
2. 检查目标、里程计、点云状态。
3. 检查输入新鲜度。
4. 从最近实际 p/v 出发，传入 start_acc=0、goal_v=0，调用 search(init=false, dynamic=false)；由于采用自由初始加速度搜索，输出首段加速度不必为零。
5. 搜索结束后再检查一次输入新鲜度。
6. 成功时导出精确多项式、采样 p/v/a 和 Path。
7. 每次都发布 PlanResult；只有非空 Path 才发布 kino_path。

失败时，PlanResult 中无轨迹；RViz 可以继续显示旧成功预览。这两者具有不同用途。[N01:213–270](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L213-L270) [N07] [N14:106–117](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/USAGE.md#L106-L117)

~~~mermaid
flowchart TD
    I["输入回调"] --> C["更新状态和双树地图"]
    T["定时器触发"] --> P["处理待匹配点云"]
    P --> V{"输入有效且新鲜？"}
    C -.-> V
    V -->|否| F["构造失败状态和空轨迹"]
    V -->|是| S["从最近实测 p/v 搜索"]
    S --> E{"成功且搜索后仍新鲜？"}
    E -->|否| F
    E -->|是| B["构造多项式与采样输出"]
    B --> R["发布 PlanResult"]
    F --> R
    R --> Q{"Path 非空？"}
    Q -->|是| K["发布新的可视化 Path"]
    Q -->|否| X["本轮不发布 Path"]
~~~

### 4.3 这里的定时器频率不是实际输出保证

当前 YAML 的 planning_rate=10 Hz，但 search_budget=0.3 s；核心默认及 benchmark 直接启动路径仍是 0.08 s。搜索、输入处理和发布由同一线程串行执行，长搜索会推迟输入回调和后续周期。搜索预算还不包含建图、输出采样和消息序列化。[N01:29,51,77,300–304](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L29-L304) [N03:28](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L28) [N04:9,55–57](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml#L9-L57) [N11:55–60](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/test/benchmark.py#L55-L60)

因此，新版改善了“失败后仍有再次尝试机会”，但不能把这一点等同于“实时性已经优于上游”。

---

## 5. 两个流程谁更好

### 5.1 按目标比较，而不是只给一个总分

| 评价维度 | ROS1 原流程 | ROS2 当前流程 | 判断 |
| --- | --- | --- | --- |
| 独立构建、维护与测试 | 演示器/飞控协议耦合较多 | 核心与 ROS 清晰分离，原子结果和显式参数 | 新版更好 |
| 输入异常、等待与恢复 | 状态和有效性检查不足，有调度死角 | 有状态门控、过期判断和周期重试 | 新版更好，但仍有输入边界 bug |
| 有效轨迹保持与计算开销 | 旧路径有效时主要做重检查 | 每轮全量重搜 | 旧策略意图通常更节省搜索计算；实际要测量 |
| 碰撞检查强度 | 旧路径每隔 40 点，原语采样较粗 | 核心碰撞更密、膨胀更保守 | 新版几何检查更严谨 |
| 对新障碍的反应时机 | 点云回调可直接进入重检查和搜索，但可能被门限阻塞 | 至少受计时器、当前任务和搜索耗时影响 | 无法仅凭架构判谁延迟更小 |
| 与正在执行轨迹的关联 | 有执行索引与参考接点意图 | 当前没有执行反馈和接轨协议 | 旧版意图更接近执行层需求，旧实现仍不合格 |
| 贴近车辆最新测量状态 | 首次/无索引用实测，其他时候可能用旧参考 | 始终使用最近实测 p/v | 新版更直接，但没有外推到实际切换时刻 |
| ROS1 行为复现 | 自身就是基准，但有未定义行为 | 输入、调度、约束与输出均已变化 | 当前新版不满足逐行为复现 |
| 动态实飞可靠性 | 有原系统演示背景，当前源代码仍有明显边界 | 有独立组件防护，尚未完成对应闭环复现 | 都不能仅据代码结构认定可直接投入飞行 |

表内“通常更节省”“更接近执行层需求”是工程判断，不是本次基准测量。相关实现依据见前两章。[R01] [N01] [N12] [N14]

### 5.2 对当前项目的建议选择

**以 ROS2 新版作为工程底座，保留旧版按需重规划和执行进度接续的意图，重写其状态机。**

这意味着：

- 已有轨迹仍安全、目标未变、执行误差可接受时，不必无条件全量重搜。
- 新目标、无有效轨迹、路径将碰撞、跟踪误差过大、轨迹即将结束等应是独立重规划触发。
- 新目标不得被上一条轨迹的索引节流门限抑制。
- 失败后要有有界重试机制，不依赖“已有旧路径”才能再试。
- 旧路径可否继续执行，应由剩余路径检查和执行器有效性决定；不能因 RViz 还有路径就继续。
- 起点应对应计划切换时刻：根据有效参考轨迹和跟踪状态选择接点，或者从外推后的实际状态生成可行过渡。
- 保留周期性检查或 watchdog，但把“检查”和“每次必定搜索”分开。

这是一套**建议方案**，既不是当前 ROS2 已完成的功能，也不是原 ROS1 已具备的完整保证。

### 5.3 不建议用两种简单改法替代真正设计

**仅把 timer 改成点云回调中直接 search，不足以恢复 ROS1 行为。** 还缺旧路径复查、目标事件、起点选择、init 分支和执行状态等内容。

**仅换成 MultiThreadedExecutor，也不会自动解决时效问题。** 当前核心复用 KD 查询缓冲和搜索节点，不允许地图更新与搜索并发。若采用并行输入，需要不可变地图快照、状态版本和结果提交校验，或合适的互斥与作业调度；否则会引入数据竞争。[N03:94–100,117–140](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L94-L140)

---

## 6. “尽量复现 ROS1 结果”应如何定义

### 6.1 三种目标不能混在一起

| 目标 | 适合的验收方式 | 本报告建议 |
| --- | --- | --- |
| 算法机制复现 | 比较运动模型、候选原语、代价、启发式、连接公式与地图机制 | 当前已较充分保留，但需明确策略改动 |
| 修复后参考实现的结果复现 | 固定输入事务和共同策略，比较连续轨迹、搜索决策及状态 | **推荐作为正式目标** |
| 未修改旧二进制的逐字节输出复现 | 冻结旧二进制、依赖、平台和消息投递，记录经验输出 | 可作历史观察，不适合跨平台可靠合同 |

上游存在未初始化状态、未定义的地图原点/边界、缺返回值、索引越界等问题。由这些问题产生的某次输出，不是一个可唯一指定的算法结果。即使修复全部问题，ROS 调度、PCL/Eigen/libm、浮点运算和同分节点顺序也可能改变结果。[R01:52–61,70–82,173–210,325,335–361](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L52-L361) [R02:700–706,917–931](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L700-L931)

**应保存未改上游快照，再创建有补丁清单的“ROS1 修复参考版”。** 不应将修复后的参考结果伪称为未经修改上游的原始行为，也不应为追求一致重新引入越界或未初始化读取。

### 6.2 为什么同一个 bag、同一个目标仍然不够

规划结果可以按四层理解：

| 层次 | 决定结果的实际内容 |
| --- | --- |
| 地图 | 哪些点进入核心、以什么顺序积累、在哪一帧清图或换 bank |
| 搜索初态 | 当时实际采用的 p/v/a、参考执行进度、目标状态 |
| 搜索策略 | init、碰撞规则、shot、终止、队列和资源预算 |
| 输出 | 连续多项式怎样采样、怎样解释状态、何时发布和执行 |

两个节点即使订阅同一条里程计，也可能一个用实测 p/v、另一个用旧轨迹参考 p/v；即使读取同一段 bag，也可能处理不同的点云帧子集。先对齐这些输入事务，比先追求 RViz 路线相似更重要。

### 6.3 推荐采用双基线

- **原始快照**：未经修改的上游源码，只用于归因、查历史分支和记录已知不确定行为。
- **修复参考版**：明确初始化与边界，修复三轴速度、索引、队列、参数枚举和资源边界，加入诊断导出；所有会改变输出的修复逐条登记。

如果正式参考采用新版更严格的碰撞或 shot 策略，名称应明确是“采用共同安全策略的 ROS1 派生参考”。这验证的是移植在相同定义下的一致性，不能用它掩盖新版相对历史算法策略的改变。

历史策略差异可以在**单独的离线诊断入口**逐项重放，记录其候选轨迹及独立安全判定。正式执行接口不应把不满足共同约束的历史候选标成可执行成功。

---

## 7. 必须逐项核对的行为差异

下表的“优先”表示复现工作的依赖顺序，不是缺陷严重性。“仅配置”只指当前仓库已存在的参数；没有现成开关的项目需要改代码或增加离线适配器。

### 7.1 第一组：先对齐实际进入搜索的输入

| 编号 | 差异与证据 | 为复现需要做什么 | 当前仅配置能否完成 |
| --- | --- | --- | --- |
| A01 | 旧版先选原始偶数下标，再以 cur_pos 做逐轴 ±20 m 裁剪；新版无该预处理。[R01:375–388](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L375-L388) [N01:125–167](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L125-L167) | 固定原始点序、裁剪中心、处理顺序；保存真正传入 setKdtree 的点云。不要用球形裁剪或“先裁剪后隔点”替代 | 否 |
| A02 | 旧版直接取 twist，且 vz 写错分量；新版默认旋转 child-frame twist。[R01:52–61,393–401](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L52-L401) [N01:173–197](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L173-L197) | 根据生产者真实语义统一世界系 p/v/a；修正旧赋值 bug。若源确实发布世界系速度，才使用 body=false | 部分；坐标适配和旧 bug 仍需处理 |
| A03 | 旧版 goal/cloud 事件决定 search 调用；新版 timer 决定调用。[R01:70–82,101–129](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L70-L129) [N01:77,213–229](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L77-L229) | 显式记录并重放每次规划事件及当时输入；不能靠把 planning_rate 改成点云 Hz 获得等价 | 否 |
| A04 | 旧版有索引时可用旧参考 p/v，新版始终用实际 p/v。[R01:143–229](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L143-L229) [N01:228–229](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L228-L229) | 先确定实验有无执行器索引。核心对比固定共同初态；节点对比另测参考分支，保存所选索引和估计速度 | 否 |
| A05 | 旧版每轮 true，只有 NO_PATH 才 false；新版一次 false。[R01:149–157,222–229](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L149-L229) [N01:229](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L229) | 增加明确的两阶段调用策略；定义新增 TIMEOUT/NODE_LIMIT 是否重试、总预算如何共用 | 否 |
| A06 | 两端接收/接受的点云序列可能不同：新版拒绝重复时标、过期和坏输入，并可等待盲区位姿。[N01:103–171](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L103-L171) | 对齐 accepted-map-event 序列、reset/clear 事件及 odom/goal 快照。bag 发布时间不是核心接收序列 | 否 |
| A07 | 新版盲区可删除传感器球内所有点，上游没有对应过滤。[N01:142–165](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L142-L165) [N04:16–25](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml#L16-L25) | 历史输入对照关闭盲区；若生产必须去机体点，另立过滤基准，不能把任意近身障碍删除解释成还原 | 关闭可配置；精确输入仍需验证 |

**特别容易被静止测试掩盖的 A05：** 当起速为零、初始加速度为零、目标又不在 near-end 范围内时，旧版 init=true 的第一轮原语不移动，通常会被同格剪枝，最后转入 false。非零起速时则可以生成滑行原语，和新版自由初始加速度搜索存在实质差别。因此只做静止竖杆基准不足以证明初始模式一致。[R02:175–269,312–319](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L175-L319)

### 7.2 第二组：会直接改变安全判定和可行解集合的策略

| 编号 | 差异与证据 | 对结果的影响与对齐要求 | 当前仅配置能否完成 |
| --- | --- | --- | --- |
| A08 | 旧 origin/map_size 赋值被注释；旧原语 z 为 [0.2,1.3]，shot 又用另一套边界；新版 origin=lower，统一 bounds，默认 z 到 10。[R02:349–353,600–604,700–706](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L349-L706) [N02:48,91–92,261–272,478–498](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L48-L498) | 先为参考版显式定义 grid_origin 和 bounds；按实验选择共同高度范围。旧未初始化值不能被“对齐”。建议格原点与安全边界分离 | 仅高度可配；整体不行 |
| A09 | 旧最近点距离 <0.45 m 才拒绝；新版默认 <=0.648205 m。[R02:121](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L121) [N02:94–102](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L94-L102) | 共同参考必须明确膨胀策略和等号边界；历史 0.45 规则单独离线比较 | 否，当前公式固定包含补偿 |
| A10 | 旧原语按端点直线位移采样，新版按速度上界采样；默认 0.6 s 常规原语最多检查 42 个点，遇碰撞提前退出，init 时长另算。[R02:329–364](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L329-L364) [N02:295–316](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L295-L316) | 共用同一种采样器；旧的位移采样作为历史策略归因。仅调整 collision_step 无法对所有原语复现旧规则 | 否 |
| A11 | 旧 shot 只试一次启发式时长，新版试 1/1.5/2/3/4 倍。[R02:199–201](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L199-L201) [N02:183–190](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L183-L190) | 分别比较系数公式和时长候选集合；时长改变直接改变末段轨迹、总耗时和是否成功 | 否 |
| A12 | 旧 shot 的速度/加速度超限拒绝被注释，约 10 点检查碰撞；新版解析查极值并用沿程上界采样。[R02:550–620](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L550-L620) [N02:458–510](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L458-L510) | 正式两端共用强校验；历史不限导数候选只作为诊断输出，单独标出独立验证失败原因 | 否 |
| A13 | 旧 near-end shot 失败就 NEAR_END/NO_PATH；新版继续搜索，不实际输出 NEAR_END。[R02:221–237](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L221-L237) [N02:207–217](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L207-L217) | 搜索终止与 wrapper 状态消费一起对齐；不能把旧 NEAR_END 直接当 REACH_END | 否 |
| A14 | 新版搜索前拒绝无地图、非法状态、起终点占用/越界；旧版没有相同前置检查。[R02:92–149](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L92-L149) [N02:116–123](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L116-L123) | 正式比较限定共同合法输入域；对被新 guard 拒绝的旧样本独立分类。目标占用但允许局部 horizon 的政策也要单独说明 | 否；不建议回滚防护 |

A09 的数值关系：

$$
r_{\mathrm{new}}=safe\_distance+\sqrt{3}\,voxel\_size+\frac{collision\_step}{2}
=0.45+\sqrt3(0.1)+0.025\approx0.648205\ {\rm m}.
$$

这解释了旧版通过而新版无路的一部分情况。**不要通过随意减小物理安全距离，使总半径偶然等于 0.45 来宣称完成复现**；那既没有恢复旧采样和 shot，也混淆了参数的物理含义。[N04:48–54](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml#L48-L54)

A08 还需要一个细节：当前默认 lower 相对零原点为整数格偏移，理想算术下可以只是索引整体平移，不能说它必然改变路径。但旧原点未定义，且浮点 floor 边界可能分叉；应通过显式格原点和临界样例验证。[R02:917–931](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L917-L931) [N02:48,593–602](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L48-L602)

### 7.3 第三组：搜索顺序、数值和资源边界

| 编号 | 差异与证据 | 对齐方法 |
| --- | --- | --- |
| A15 | 双积分器、控制代价、四次启发式和主要输入格点已经保留。[R02] [N02] | 保持这些数学；不要为了“清理代码”同时改变代价或控制离散化 |
| A16 | tie_breaker=1.0001，启发式返回又乘 1+tie_breaker，默认 lambda=5 后实际倍率为 10.0005；两端相同。[R02:480,547](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L480-L547) [N03:178–180](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L178-L180) [N02:455](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L455) | 这是已对齐项。不要顺手改成约 5 或 5.0005 后仍声称同一算法 |
| A17 | 上游有 max_vel+vel_margin，新版无独立 vel_margin；resolution_astar 改名为 resolution。[R02:466–484](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L466-L484) [N03:19–24](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L19-L24) | 比较解析后的有效数值。若旧 vel_margin 非零，两端共同 max_vel 应使用相同有效和 |
| A18 | 旧堆中原地改 f；新版保存快照、重新入队。现比较器都没有稳定同分次序。[R02:380–445](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L380-L445) [N02:166–172,333–340,378–390](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L166-L390) | 参考版与 ROS2 同时采用修复，并共同定义等 f 时的稳定次序；不要回滚堆错误。若新加序号，要明确这也是参考补丁 |
| A19 | 候选枚举固定 +1e-3 的旧缺陷仍保留；新版有退化时间下界、finite 过滤、acos clamp。[R02:243–269,512–548,623–655](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L243-L655) [N02:229–249,435–445,541](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L229-L541) | 两端同步修正小参数枚举及退化保护；正常域比较数学，退化域比较明确的错误/终止语义 |
| A20 | 上游无墙钟预算；新版有合作式 TIMEOUT，默认核心 80 ms / YAML 300 ms。[N02:114–115,165,255](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L114-L255) [N04:55–57](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml#L55-L57) | 功能等价用共同确定性工作量限制和外部 watchdog；真实性能另测墙钟预算。当前无预算抽象，需改离线 harness/核心接口 |
| A21 | 旧版分配到最后一槽即返回 NO_PATH；新版下一次需要超容量时才 NODE_LIMIT。[R02:403–430](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L403-L430) [N02:351–375](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L351-L375) | 两端统一容量及失败语义，单独测试最后一槽；相同 allocate_num 不代表相同边界行为 |
| A22 | 位置 hash 不含速度、closed 不重开；dynamic 只加时间，仍是空间碰撞。[R03] [N03:78–90,133–144](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L78-L144) | 这些主体近似已保留，不应为本次复现顺手换六维状态格或动态预测模型。要增强应另立算法版本 |

A20 的“确定性工作量限制”是建议新增的测试能力，例如共同限制节点扩展/原语尝试数量。**当前 search_budget 不能写成 0 或无穷大来获得这一功能**：参数校验会拒绝；也不应在在线系统中无界运行。

### 7.4 第四组：地图积累和输出表达

| 编号 | 差异与证据 | 对齐方法 |
| --- | --- | --- |
| A23 | 两端都约每 50 次地图输入切换 bank，但新版接受/清图条件不同。[R02:23–89](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L23-L89) [N02:70–89](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L70-L89) [N01:98–171](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp#L98-L171) | 对齐真正的 setKdtree 和 clearMap 事件，而非只配 tree_period=50；保存 bank 索引、点数和内容摘要 |
| A24 | 两端都是“上次滤波质心+新帧”再体素化，不是整段原始历史点一次体素化。[R02:37–59](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L37-L59) [N02:75–87](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L75-L87) | 保持逐帧顺序与 float32/PCL 处理路径，固定 leaf size 和依赖版本；不要换成一次性合并 bag |
| A25 | 旧 getKinoTraj(0.01) 从原语末端倒采样，可能重复边界/缺段起点或 shot 精确终点；新默认 0.02，均分各段且包含精确端点。[R02:754–796](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L754-L796) [N02:634–655](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L634-L655) | 先对比连续段，再共同采样；仅 sample_step=0.01 不足以复刻旧点序和索引 |
| A26 | 旧 path_size 为实际点数+1、位置输出 latched；新版原子状态、精确系数和明确时间。[R01:315–326,459–462](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L315-L462) [N07] | 修正长度 bug，单独写旧接口适配层；ROS1/ROS2 消息不做逐字节等价要求 |
| A27 | 上游计算了 PX4 变换后的路径，但第 321 行实际发布原 kino_nav_path；新版统一 world frame。[R01:252–321](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L252-L321) | 以真正发布/消费的数据为准，不把恢复未使用变量或变换代码当作结果保真 |
| A28 | 旧 check_num、optimistic 等字段在现有实际碰撞路径未生效；launch 中 margin 也未进入有效逻辑。[R02:466–484](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L466-L484) [R04] | 对齐有效参数，不按字段名字或文件数量判断缺失；未使用的 DenseInput 等也不能当成算法漏移植 |

### 7.5 最小优先顺序

若资源有限，先做 A01–A06：**核心实际点云、共同状态、规划事件、起点选择、初始模式和接受帧序列**。这些没有相同，后面的轨迹差分没有诊断意义。

随后处理 A08–A14 的可行域与终止策略，再处理队列/数值/资源，最后才是 Path 采样和消息兼容。只调整 max_vel、max_acc、safe_distance、sample_step 等几个 YAML 值，不能完成这项任务。

---

## 8. 推荐的兼容实现方案

### 8.1 将三类变更分开登记

| 变更类别 | 例子 | 应如何管理 |
| --- | --- | --- |
| 中间件适配 | ament、rclcpp、消息类型、QoS、launch | 独立列出，原则上不改变搜索数学 |
| 确定性与健壮性修复 | 初始化、vz 赋值、边界、索引、堆、字段校验、参数枚举 | 保留；参考版同步修复并说明结果影响 |
| 可选策略改变 | timer、起点策略、碰撞膨胀、shot 重试、near-end 继续搜索 | 明确配置/实验入口、单项差分、独立验收 |

这能避免两种混乱：把安全修复当作不忠实而回滚，或把所有行为重设计都称作 ROS2 必需适配。

### 8.2 建议的代码职责

建议保留现有核心和 ROS 输入输出边界，增加一个可单独测试的规划管理层：

| 组件 | 职责 | 与复现的关系 |
| --- | --- | --- |
| 输入适配器 | 验证、坐标/速度语义、点云预处理 | 可以重放旧预处理，也可以使用生产过滤 |
| 地图事件记录与快照 | 记录接受帧、清图、bank、版本 | 固定同一次搜索实际使用的地图 |
| 规划管理器 | 目标 revision、触发原因、有效路径、执行进度和重试 | 对齐旧意图并修正旧状态死角 |
| 搜索策略 | init、碰撞、shot、终止和预算 | 进行受控策略对照 |
| 输出适配器 | 连续多项式、统一采样、旧接口兼容、原子状态 | 不混淆“轨迹不同”和“采样不同” |
| 独立验证器 | 原始障碍净空、导数、边界、连续性 | 实现间一致不能替代安全验证 |

上述是建议结构，不要求机械创建六个包；可以先以类或离线测试模块实现。

### 8.3 两个建议入口

**正式参考入口：修复后的共同定义。** 两端使用相同合法输入、明确边界、共同队列与数值保护、共同碰撞/shot 定义和相同终止语义。以连续多项式为核心输出，固定每次调用输入并找第一处分歧。

**历史行为观察入口：离线归因。** 一次只改变一项历史政策，例如旧隔点裁剪、旧 init 调用次序、旧半径与采样、旧一次 shot 或旧 near-end 退出。每个候选同时经过独立验证器，分别记录历史状态与是否满足共同约束。不能把这个入口生成的所有候选直接发布给飞行执行器。

这两个入口目前不存在；建议名称也不应写成用户现在可以运行的命令。

### 8.4 规划管理器建议保留哪些旧意图

- 输入首次齐备且存在目标：触发首次规划。
- 收到新目标：设置 pending revision，绕开旧轨迹进度节流。
- 没有有效轨迹：新地图或定时退避后重试。
- 已有轨迹：检查未执行的后缀，按需要触发重规划。
- 接近轨迹末端：按剩余时间、终点速度与任务要求决定是否重规划或结束。
- 接轨时刻：使用明确的未来生效时间；从精确参考曲线求 p/v/a，结合跟踪误差判断可否使用。
- 搜索完成：检查目标、地图与执行版本是否仍适用；若作业已被新目标取代，不能覆盖更新结果。
- 失败：发布可辨识状态。旧轨迹是否可暂时继续由独立执行管理器决定，不由可视化预览决定。

若仍需与旧 index 协议对照，适配层应绑定 trajectory ID、源时间和索引范围。只传一个裸 Int64 不能识别该索引属于哪条轨迹。[R01:85–89,419–425](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp#L85-L425)

### 8.5 哪些上游问题不应为“忠实”而重新引入

| 不应回滚的修复 | 原因 |
| --- | --- |
| 三轴速度正确赋值、状态初始化 | 未定义/错误初态不是有效算法参考 |
| timeToIndex 返回值、时间 hash 索引 | 属于确定性实现错误 |
| priority_queue 分数更新修复 | 保证队列语义；两端共同修正比复制错误更合理 |
| 空图/坏输入明确拒绝 | 不应把无障碍观测误当全空间已知安全 |
| 索引范围和真实路径长度 | 避免越界和错误执行进度 |
| 原子失败状态与空 PlanResult 轨迹 | 防止消费者误用失败结果 |
| 有限资源与参数检查 | 参考实验可有不同预算政策，在线不能无界 |
| 精确多项式导出 | 提供更好的对照对象；历史采样另做输出适配 |

对旧半径、旧采样、旧 shot 接受策略等，需要保留**可解释的离线对照**；正式执行仍应使用经过独立几何和动力学约束验证的结果。

---

## 9. ROS2 待修复问题清单

严重性在这里按“作为规划组件继续集成”的影响排序：P1 为优先处理的内存安全或结果有效性问题；P2 为非默认参数边界、文档和验证可追溯性问题。条件触发的 P1 不表示正常传感器每次都会导致故障。

| 编号 | 优先级 | 问题 | 证据状态 | 归因 |
| --- | --- | --- | --- | --- |
| B01 | P1 | 重复 XYZ 字段绕过布局校验，PCL 按另一字段读取导致越界读 | 源码确认＋隔离 ASan 最小复现 | 新增防护不完整；旧版也缺少充分校验 |
| B02 | P1 | 轨迹构造后、发布前没有再次检查输入新鲜度 | 源码确认＋条件推导，未完整 ROS 复现 | 当前结果时效契约的缺口 |
| B03 | P2 | 固定浮点余量产生超限候选、过多候选及极端零步长循环 | 原样枚举及参数校验的有界 C++ 复现 | 上游继承，新配置校验未覆盖 |
| B04 | P2 | PORTING 的 Path 清空描述与当前代码/测试相反 | 源码、文档和测试交叉确认 | 文档随行为变更未同步 |
| B05 | P2 | 80 ms 验证记录不能代表当前 300 ms launch；记录未绑定当前测试结果 | 配置与启动代码确认，旧记录为作者报告 | 配置/验证追溯不足 |

### 9.1 B01：重复字段造成越界读取

**位置：** ROS2 path_planning.cpp:112–126。[N01]

当前校验逻辑是：对 x、y、z 各自寻找至少一个合法 FLOAT32/count=1/offset 字段。它没有要求字段唯一，也没有保证 PCL 最后选择的字段就是校验通过的字段。

PCL 1.14.0 的 FieldMatches 检查名称、类型和数量，不检查 offset；FieldMapper 选到第一个匹配字段就返回。复制阶段再直接使用该 offset。[P01] [P02] [P03] [P04]

**最小畸形输入：**

- 正确 frame、有效且递增的新鲜时间戳；
- width=1、height=1；
- point_step=12、row_step=12、data.size=12；
- little-endian；
- fields 必须按下表顺序排列。

| 顺序 | name | offset | datatype | count |
| --- | --- | --- | --- | --- |
| 1 | x | 1024 | FLOAT32（7） | 1 |
| 2 | x | 0 | FLOAT32（7） | 1 |
| 3 | y | 4 | FLOAT32（7） | 1 |
| 4 | z | 8 | FLOAT32（7） | 1 |

wrapper 因第二个 x 合法而放行；PCL 却选择第一个 x，从 12 字节数据之外的偏移 1024 读取 4 字节。

隔离探针输出：

~~~text
wrapper_layout_accepts=true
PCL_mapping serialized_offset=1024 size=4 struct_offset=0 within_source=false
AddressSanitizer: heap-buffer-overflow
READ of size 4
~~~

**实验边界：** 探针抽取固定 PCL 版本的 FieldMatches/FieldMapper、wrapper 布局判断及相同复制表达式；消息结构与 PointXYZ traits 用轻量替身。没有运行完整 ROS2/PCL 节点。该结果证实的是越界读的代码路径，不能扩写成越界写、任意代码执行或已复现完整节点崩溃。

**修复建议：**

1. 要求 x、y、z 每个恰好一个，检查类型、count、offset 和点步长。
2. 确保后续转换只使用同一组经过验证的字段映射；可使用严格唯一字段后再调用 PCL，或自行按验证后的偏移提取 XYZ。
3. 对重叠 XYZ 字段定义明确政策；若不支持，拒绝，而不是让同一字节被默认为不同坐标。
4. 保留对额外非 XYZ 字段和合法 padding 的兼容，不应为修补而把所有正常扩展点云拒绝掉。

**回归验收：** 正常 XYZ、字段乱序、额外 intensity、合法行 padding、重复字段两种顺序、越界 offset、缺字段、错误类型/count、空数据、NaN/Inf。畸形输入应在转换前拒绝、节点存活、本轮无可执行轨迹；真实 ROS 进程与 sanitizer 构建都应覆盖。

### 9.2 B02：发布前最后一道时效检查缺失

**位置：** wrapper:234–270，核心 sampleTrajectory:634–651。[N01] [N02]

搜索完成后检查 fresh，随后才构造多项式消息、全部采样点和 Path，最后 publish。若输入在构造期间跨过 input_timeout，结果仍可能保留 REACH_END/REACH_HORIZON。

sample_step 只要求正有限值，核心容许最多一百万个采样点，因此非常密集的输出或慢平台能使构造耗时不可忽略。此项是条件明确的代码推导，本次未完整 ROS 复现。

**修复建议：**

- 输出构造完成后、publish 前再检查输入时效。
- 失败时改为 STALE_INPUT，并清空本轮 segments、trajectory.points 和本轮 Path。
- 对输出点数、轨迹时长和生成耗时设合理上限。
- 若将来引入异步搜索，还必须在提交结果时检查目标、地图和执行版本，防止过时作业覆盖较新目标。
- 消费者仍需检查接收时的有效期；最后一次 fresh 只覆盖发布调用前，不能覆盖 DDS 和执行器排队。

**回归验收：** 用受控的慢输出构造或测试时钟，使输入在搜索结束时有效、发布前失效；确认最终状态变为失败、轨迹为空。另测快速正常构造，不产生误拒绝。

这里清空的是本轮准备发布的数据，不代表一定要擦除 last-success RViz 预览；可视化政策应另行明确。

### 9.3 B03：小正参数突破候选枚举与预算边界

**位置：** SearchConfig::validate:32–40；初始时长/加速度枚举:231–249；下一次预算检查:255。[N02]

固定 +1e-3 余量配合按参数缩小的步长，会使“每轴五点、初始二十时长”不再成立。原语扩展之后主要检查速度，没有重新拒绝超出 max_acc 的 um。[N02:281–285](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L281-L285)

原样抽取校验和循环的运行结果：

| 已被接受的配置 | 计数/最大值 | 解释 |
| --- | --- | --- |
| max_acc=2 | 每轴 5，总计 125，最大 2 | 默认正常 |
| max_acc=1e-4 | 每轴 25，总计 15,625，最大 0.0011 | 候选达到限制的 11 倍 |
| max_acc=1e-8 | 每轴 200,005 | 理论三轴组合约 8.0006×10^15 |
| init_max_tau=0.8 | 20 项，最大 0.8 | 默认正常 |
| init_max_tau=1e-6 | 20,019 项，最大 0.00100095 | 不再满足最大时长含义 |
| denorm_min 级别正值 | validate 接受，但比例步长可下溢成 0 | 原浮点 for 不再前进 |

**已运行的是有界单轴计数、配置校验和步长检查**，没有分配极端数量的向量，也没有真的运行无限循环或完整搜索。因此准确结论是“生成候选和预算入口存在缺陷”，不能写成已经测到某架无人机输出了 11 倍加速度轨迹。

该问题上游就存在；默认 max_acc=2、init_max_tau=0.8 不触发。[R02:243–269](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L243-L269)

**修复建议：**

- 用固定整数索引生成每轴五个候选，例如明确的 -a、-a/2、0、a/2、a。
- 用整数 1…20 生成初始时长，并验证最小分段时长不下溢、所有候选有限且为正。
- 在接受原语前检查逐轴加速度，构造输入时设置有界容量与必要的截止检查。
- 针对不具有有效数值分辨率的极端正参数，给出清晰配置错误，而不是仅通过“大于零”校验。

**回归验收：** 默认候选集合不变；小值、临界值、denormal、大值等输入有限时间退出；所有候选满足声明的约束。ROS1 修复参考版同步采用相同修复。

### 9.4 B04：Path 文档与实际语义冲突

当前 wrapper:270 只发布非空 Path；test_ros_io:170–172 明确断言失败保留最后成功预览；USAGE:112–117 也描述了这一行为。PORTING:14 却仍称“Path 同步清空”。[N01] [N10] [N14] [N12]

**正确修复不是盲目回滚代码。** 应先确认当前产品语义，然后统一移植说明、验证记录、示例消费者和可视化说明：

- PlanResult 是当前规划有效性的权威状态。
- kino_path 是最后成功轨迹的预览，其旧时间戳不应被刷新为当前成功。
- 如果消费者只订阅 Path，必须增加状态协议或改用 PlanResult，不能靠“仍有线条”判定可以执行。

这是文档和接口使用风险，不是“当前失败 PlanResult 还偷偷携带旧轨迹”。

### 9.5 B05：默认配置和验证证据需要重新绑定

当前核心默认 0.08 s，benchmark.py 直接启动节点，未加载安装 YAML；planner.launch.py 则加载 search_budget=0.3 s 的 YAML。[N03:28](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L28) [N11:55–60](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/test/benchmark.py#L55-L60) [N05] [N04:55–57](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml#L55-L57)

旧验证记录报告 80 ms 预算下：

| 平台 | 目标轨迹/结果 | 搜索 P99 | 结果间隔 P99 |
| --- | --- | --- | --- |
| amd64 | 288/290，2 TIMEOUT | 73.50 ms | 121.67 ms |
| Jetson Orin NX | 1,189/1,190，1 TIMEOUT | 72.61 ms | 135.32 ms |

这些是**作者记录**，本次未独立重跑；文档已坦承未稳定满足全链 P99≤100 ms。当前普通 launch 允许 300 ms 搜索，更不能直接套用这一组数字。[N13:29–46](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/VALIDATION.md#L29-L46)

其他证据边界：

- 核心现有 12 项测试确有独立原始点几何检查，但多数用例预算为 2 秒。[N09:7,14–40](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/test/test_search.cpp#L7-L40)
- 当前 ROS2 进程测试参数化为 blind_radius=0 和 0.5 两组；旧“15 tests / 0 failures”的统计并非本次对当前源码重新运行所得。[N10:24–34](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/test/test_ros_io.py#L24-L34) [N13:24–27](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/VALIDATION.md#L24-L27)
- 真实 Odin 回放记录是 150 次 NO_PATH、停流后 30 次 STALE_INPUT，作者明确称其为输入接入与拒绝验证，不是避障成功证明。[N13:52–70](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/VALIDATION.md#L52-L70)
- 检查时 GitHub Actions 运行记录和该提交 check-runs 均为空；这不证明作者没在本地测试，只说明缺少相应公开自动化运行证据。[H05] [H06]

**修复建议：** benchmark 与正式 launch 接受共同配置入口，导出完整解析参数、提交 SHA、依赖/硬件/构建信息和原始结果；分别报告 80 ms 与 300 ms。更新验证文档，保留全部失败和超时，建立固定提交的 CI 或可复跑日志。

### 9.6 不应混称为“移植 bug”的系统建设项

| 建设项 | 当前边界 | 后续应做什么 |
| --- | --- | --- |
| 轨迹时刻与状态外推 | 初态来自最近 odom，header 为搜索开始；没有未来切换协议 | 明确状态时间、预测到生效时刻、接轨和执行确认 |
| 事件状态机与恢复 | 新版周期重试，但缺执行/取消/到达/保持状态 | 引入有版本的规划管理器，修复旧状态机意图 |
| 动态障碍预测 | 目前是空间点图和反复重规划，dynamic hash 不预测运动 | 根据应用要求另立预测或反应式安全边界与实验 |
| 未观测空间和盲区 | 非空点图不证明完整自由空间；盲区删除会同时删掉真实近身障碍 | 明确观测覆盖和过滤依据，不靠任意删点制造成功 |
| 真实飞行动力学 | 限制逐轴 v/a，轨迹主要 C1，加速度可跳变 | 验证控制跟踪、jerk/推力/姿态需求，按系统需要扩展 |
| 全链时效 | 单线程、合作式搜索预算，不含建图和全部输出 | 测端到端响应，设计有界调度和故障动作 |

这些大多已在项目文档中声明不包含或不保证。若目标是实际飞行，必须补齐；若目标只是核心算法离线复现，不应把它们全部当成“ROS2 移植没写对”。[N03:14–15,43–50,94–100,133–140](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h#L14-L140) [N12:28–37](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/PORTING.md#L28-L37) [N14:138–170](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/USAGE.md#L138-L170)

---

## 10. 差分回归与可靠性验收方案

### 10.1 先记录真正的搜索调用事务

每一个可重放样本至少保存：

- 两端提交、参考补丁清单、编译器、优化选项、CPU 架构、PCL/Eigen/标准库版本。
- 原始点云布局、顺序和时标；送入核心的实际 float32 XYZ。
- setKdtree/clearMap 事件顺序、接受或拒绝原因、bank 索引、构建后的点数/内容摘要。
- 每次 search 的 p0、v0、a0、goal_p、goal_v、init、dynamic、time_start。
- 全部解析后的有效参数，包括 grid origin、bounds、碰撞、shot、队列与预算策略。
- 触发原因、目标版本、所用里程计时间、若有执行索引则保存轨迹 ID 和所选参考状态。
- 状态、段数、时长、系数，以及第一处不同的搜索决策。

这是建议新增的诊断格式；不能只写两个 bag 路径就认为输入已固定。

### 10.2 分五层找第一处分歧

| 层次 | 对照对象 | 主要目的 |
| --- | --- | --- |
| 1 数学 | 相同输入的 stateTransit、heuristic 候选、shot 系数 | 排除公式和数值迁移错误 |
| 2 地图 | 相同逐帧输入后的 bank、质心、最近距离 | 排除预处理、积累和依赖差异 |
| 3 搜索 | 同一个地图/初态事务，逐节点记录 g/h/f、剪枝、shot、终止 | 找最早的离散决策分歧 |
| 4 wrapper | 固定事件顺序、目标/索引/点云的交错 | 验证何时搜索、从哪里搜索、何时恢复 |
| 5 闭环 | 车辆实际跟踪、延迟、动态障碍和失败动作 | 判断系统可靠性，不能用前四层代替 |

建议一次只改变一组策略，保留其他输入。全部回滚后只比较最终路线，会丢失差异来源。

### 10.3 不要逐下标比较旧 Path 和新轨迹采样数组

旧 getKinoTraj 和新 sampleTrajectory 的点序、边界和时距不同。正确方法是：

1. 在参考侧导出原始常加速度段与 shot 的系数/时长，不用旧 Path 重新拟合曲线。
2. 先比较总时长、分段结构、精确起终 p/v。
3. 在**相同曲线时间**上用共同采样器比较 p/v/a，包括每个段边界的左右极限。
4. 加速度在段间可能跳变，不能只比较一侧取样就称完全一致。
5. 不对两条轨迹各自做时间归一化后忽略总时长，否则会掩盖 shot 时长变化。
6. 点数组长度差异只作为输出协议差异，不自动等于算法差异。

来源：旧采样 R02:754–796、新采样 N02:619–655。[R02] [N02]

若比较完整轨迹代价，应使用共同公式，而不是直接把终止节点的 g_score 当成包含 shot 的总成本：

$$
J=\sum_s \int_0^{T_s}\left(\|a_s(t)\|^2+w_{\rm time}\right)\,dt.
$$

对 p(t)=c0+c1 t+c2 t^2+c3 t^3，单段可计算为：

$$
J_s=(4\|c2\|^2+w_{\rm time})T_s
+12(c2\cdot c3)T_s^2+12\|c3\|^2T_s^3.
$$

这是基于多项式求导的共同验证公式；它包含最后一段 shot，避免只比较原语累计分数。[R02:372–374,550–569](https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp#L372-L569) [N02:322–325,458–476](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L322-L476)

### 10.4 推荐回归样例

| 用例 | 要检查的差异或缺陷 | 合格判据 |
| --- | --- | --- |
| T01 静止直达且目标远于 near-end | 基本数学、true 无位移后 false | 共同政策下状态/曲线一致，明确记录 fallback |
| T02 非零三轴速度 | body/world、vz 修复、init 分支 | 共同世界系初态一致；初始导数、首原语和终点速度正确 |
| T03 非零初始加速度直接核心调用 | init=true 实际约束范围 | 第一扩展候选正确；不得声称直接 shot 也保证接轨加速度 |
| T04 近目标但直连被挡 | near-end 退出与继续搜索 | 历史前缀与共同完整解分别归类 |
| T05 shot 所需时长偏短 | 一次/五次时长、导数约束 | 记录每次候选与拒绝原因，不用放宽限制掩盖失败 |
| T06 细杆与原语中间障碍 | 旧稀疏检查/新版保守检查 | 原始几何独立验证；旧漏检不算新版误差 |
| T07 窄通道与 0.45/0.648 m 临界 | 半径与等号政策 | 可行域差别可解释，严格边界样例单列 |
| T08 高度与 grid 边界 | 1.3/10 m、原点相位和 floor | 显式 bounds；无未初始化参考 |
| T09 第 49/50/51、99/100/101 次接受更新 | bank 轮换、逐帧体素；更新次数从 1 开始，切换实际发生于第 51、101 次 | 同一事件序列后地图与查询一致 |
| T10 重复、倒退、断流与恢复 | 接受帧、清图、输入时效 | 事件和状态正确，旧数据不能无限续期 |
| T11 目标先于传感器；首次无路后路变通 | 旧状态死角、新管理器恢复 | 输入齐备/地图变化后有明确有界重试 |
| T12 新目标与冻结/迟到执行索引 | 新目标被门限吞掉、索引串轨 | 新 revision 不被旧门限抑制，索引必须绑定轨迹 |
| T13 末端、高速近目标及身后障碍 | 0.3 m 早退、只查剩余后缀 | 到达与停车分开定义，不为已执行段误触发 |
| T14 节点最后一槽、TIMEOUT | 资源终止差异 | 容量不越界，状态和空结果一致 |
| T15 极小参数与下溢 | B03 | 候选数有界、逐轴限幅、有限时间拒绝或完成 |
| T16 重复/越界 XYZ 字段 | B01 | 转换前拒绝，真实进程无 sanitizer 错误 |
| T17 搜索后有效、输出后过期 | B02 | 发布前转失败，本轮无有效轨迹 |
| T18 不整除的段时长与采样步长 | 旧点序和端点差异 | 连续曲线正确；统一采样含精确端点 |
| T19 真实点数下成功绕障 | 80/300 ms 与全链性能 | 记录全部成功、超时和过期，不只测起点被挡的快速拒绝 |
| T20 移动车辆与突然出现障碍 | 执行延迟、接轨与制动 | 使用闭环测得的跟踪/净空/失效响应，不由静止基准外推 |

现有 C++ 用例中的 dynamic=true 目标只有 0.5 m，起点已在 near-end 范围，可能直接 shot 成功；它不能充分覆盖动态时间 hash 的深度扩展。新增核心时间索引测试应强制实际展开多个时间层。[N09:91–95](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/test/test_search.cpp#L91-L95) [N02:161–190](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp#L161-L190)

### 10.5 功能一致性、几何有效性和性能分别验收

**功能一致性：** 两端共同输入和策略是否一致；必要时比较第一处分歧，不只比较最终路径。容差应按量纲、依赖环境和数值尺度预先登记。临界舍入导致的离散分支差异应单独分析，不能用放大位置容差掩盖另一条路径。

**几何与动力学有效性：** 使用未因历史隔点裁剪而丢失的原始几何，独立检查净空、bounds、逐轴 v/a、端点条件和 C1 连续性。1,001 点密集采样是有价值的回归检查，但本身不是连续时间安全性的完整数学证明。[N09:14–40](https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/test/test_search.cpp#L14-L40)

**真实性能：** 在实际 Release 配置下分别测 80 ms 和 300 ms，统计规划与端到端 P50/P95/P99/max、连续失败时长、输入年龄、结果年龄、CPU/RSS 与地图规模。保留全部 TIMEOUT、STALE_INPUT、NO_PATH 和资源失败。

功能差分可以使用共同确定性工作量 cap，并设独立 watchdog；性能测试必须使用部署实际墙钟约束。两类结果不能相互代替。

### 10.6 “通过”的定义应写清

正式验收至少区分：

- REACH_END：达到指定终点位置与终点速度；
- REACH_HORIZON：有效局部前缀，未必到目标，也未必停止；
- NEAR_END：仅历史兼容类别，不能标成目标达成；
- NO_PATH：在当前共同搜索定义下未找到路径，不是物理不可达证明；
- TIMEOUT/NODE_LIMIT：资源中断，应分别计数；可计入给定预算下的总体规划失败率，但不能解释为几何无路；
- INVALID_INPUT/NO_MAP/STALE_INPUT/FRAME_MISMATCH：输入或运行契约失败。

新版定义主要在 PlanResult 中，参考侧应显式建立状态映射。[N07]

---

## 11. 建议实施顺序

| 阶段 | 主要工作 | 阶段产物与停止条件 |
| --- | --- | --- |
| 1 固定基线 | 保存原始 ROS1、当前 ROS2、参考补丁清单和依赖版本 | 能说明每一项有意变化，避免一边改一边移动基准 |
| 2 修复组件缺陷 | B01、B02、B03；统一 B04/B05 文档/配置证据 | 真正回归覆盖畸形输入、参数边界和最后时效检查 |
| 3 捕获共同输入 | A01–A07、明确 origin/bounds、地图事件记录 | 同一搜索事务在两端有可核对的实际输入 |
| 4 对齐核心策略 | 碰撞、shot、near-end、队列、数值与资源 | 第一处分歧可定位；连续轨迹和状态有明确解释 |
| 5 补兼容编排与执行接续 | 事件触发、目标版本、重试、有效后缀、轨迹 ID 与未来切换 | 通过目标/索引/输入交错测试；不复制旧状态死角 |
| 6 目标平台验收 | 真实点数成功搜索、长时运行、SITL/受控闭环和动态障碍 | 用端到端延迟、跟踪误差、净空和失败动作支持可靠性结论 |

建议将“修复 bug”“恢复旧编排意图”“新增算法安全策略”拆成可分别审阅的变更。尤其不要在同一大改动里同时换碰撞半径、启发式权重、状态格、调度和控制接口，否则很难解释与 ROS1 的差异。

对当前目标，最适合的路线是：**修复版 ROS1 参考＋ROS2 共同输入/策略差分＋独立的节点和闭环验收**。这既能尽量保留上游算法结果，又能把必要修复、策略变化和系统建设各自量化。

---

## 12. 来源索引与最小复现记录

### 12.1 如何阅读源码引用

正文中 R 表示 ROS1 上游，N 表示 ROS2，P 表示外部 PCL 转换实现，H 表示提交或检查记录。带行号的引用对应固定提交；同一引用中的不连续行号，链接覆盖相关范围。仓库头部在报告日期已重新确认，API 检查记录则反映审查时刻。

| 编号 | 来源 |
| --- | --- |
| R01 | ROS1 path_planning/src/path_planning.cpp [R01] |
| R02 | ROS1 path_searching/src/kinodynamic_astar.cpp [R02] |
| R03 | ROS1 path_searching/include/path_searching/kinodynamic_astar.h [R03] |
| R04 | ROS1 path_planning/launch/demo.launch [R04] |
| R05 | ROS1 README [R05] |
| N01 | ROS2 path_planning/src/path_planning.cpp [N01] |
| N02 | ROS2 path_searching/src/kinodynamic_astar.cpp [N02] |
| N03 | ROS2 path_searching/include/path_searching/kinodynamic_astar.h [N03] |
| N04 | ROS2 planner.yaml [N04] |
| N05 | ROS2 planner.launch.py [N05] |
| N06 | ROS2 odin.launch.py [N06] |
| N07 | ROS2 PlanResult.msg [N07] |
| N08 | ROS2 PolynomialSegment.msg [N08] |
| N09 | ROS2 核心回归 test_search.cpp [N09] |
| N10 | ROS2 进程回归 test_ros_io.py [N10] |
| N11 | ROS2 benchmark.py [N11] |
| N12 | ROS2 PORTING.md [N12] |
| N13 | ROS2 VALIDATION.md [N13] |
| N14 | ROS2 USAGE.md [N14] |
| N15 | ROS2 README [N15] |
| N16 | ROS2 THIRD_PARTY_NOTICES [N16] |
| H01 | 原搜索源码精确导入提交 [H01] |
| H02 | ROS2 审查提交 [H02] |
| H03 | ROS1 审查提交 [H03] |
| H04 | ROS2 将搜索预算改为 300 ms 的提交 [H04] |
| H05 | ROS2 固定提交的 GitHub check-runs [H05] |
| H06 | ROS2 GitHub Actions 运行记录 [H06] |
| P01 | PCL 1.14.0 FieldMatches [P01] |
| P02 | PCL 1.14.0 FieldMapper [P02] |
| P03 | PCL 1.14.0 转换复制循环 [P03] |
| P04 | ROS2 pcl_conversions 的 fromROSMsg 调用路径 [P04] |

### 12.2 最小复现记录摘要

**重复字段：** 使用 ROS2 固定 wrapper 的布局条件与 PCL 固定源码中的字段匹配、映射和复制逻辑；消息/traits 为轻量替身。正常 schema 探针确认 wrapper 放行、PCL 选中 offset=1024。ASan 版本在复制 4 字节时报告 heap-buffer-overflow，位于 12 字节源区域之后。结论限于该输入边界代码路径。

**参数枚举：** 使用当前 SearchConfig、validate 和候选循环抽取代码，由 g++ 编译运行。测得 max_acc=1e-4 时单轴 25 项、最大 0.0011；init_max_tau=1e-6 时 20,019 项。另用 denorm_min 验证配置接受但增量可为零。极大三维组合数由单轴计数计算，未实际分配；无限循环未实际执行。

**未执行事项：** 完整 ROS2/PCL 构建及节点运行、所有现成 gtest/pytest、真实 bag、动态闭环、控制跟踪和实飞。本报告没有把这些列为已通过。

### 12.3 对上游性能声明的处理

原 README 描述了动态小障碍、20 mm 级细杆和 50 Hz 的原系统能力。[R05] 这些属于原系统的公开说明。当前移植仓库自身也明确表示没有复现相应飞行性能。[N15] 因此本报告不把源码继承、局部测试或静止基准当作该性能的移植证明。


<!-- 固定来源引用；保留此区块以便离线 Markdown 阅读器解析。 -->

[R01]: https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/src/path_planning.cpp
[R02]: https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/src/kinodynamic_astar.cpp
[R03]: https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_searching/include/path_searching/kinodynamic_astar.h
[R04]: https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/path_planning/launch/demo.launch
[R05]: https://github.com/hku-mars/dyn_small_obs_avoidance/blob/9d25dd6974e04518c892e5711c9be9082558e5eb/README.md
[N01]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/src/path_planning.cpp
[N02]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/src/kinodynamic_astar.cpp
[N03]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/include/path_searching/kinodynamic_astar.h
[N04]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/config/planner.yaml
[N05]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/launch/planner.launch.py
[N06]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/launch/odin.launch.py
[N07]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/msg/PlanResult.msg
[N08]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/msg/PolynomialSegment.msg
[N09]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_searching/test/test_search.cpp
[N10]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/test/test_ros_io.py
[N11]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/path_planning/test/benchmark.py
[N12]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/PORTING.md
[N13]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/VALIDATION.md
[N14]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/docs/USAGE.md
[N15]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/README.md
[N16]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/blob/d2e3a694a13de14d3af8969dfa0d468d23650d1c/THIRD_PARTY_NOTICES.md
[H01]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/commit/35d3be78fb3122f5b3b1ef9b0708752e5a8223d7
[H02]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/commit/d2e3a694a13de14d3af8969dfa0d468d23650d1c
[H03]: https://github.com/hku-mars/dyn_small_obs_avoidance/commit/9d25dd6974e04518c892e5711c9be9082558e5eb
[H04]: https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2/commit/b69fe4f1c0cdc843dcb649ca097ebd79de8c750a
[H05]: https://api.github.com/repos/LostPatrol/dyn_small_obs_avoidance-ros2/commits/d2e3a694a13de14d3af8969dfa0d468d23650d1c/check-runs
[H06]: https://api.github.com/repos/LostPatrol/dyn_small_obs_avoidance-ros2/actions/runs?per_page=20
[P01]: https://github.com/PointCloudLibrary/pcl/blob/f62c018b4fc7df3dc2c096918a8462a190f28bb8/common/include/pcl/PCLPointField.h#L63-L77
[P02]: https://github.com/PointCloudLibrary/pcl/blob/f62c018b4fc7df3dc2c096918a8462a190f28bb8/common/include/pcl/conversions.h#L80-L112
[P03]: https://github.com/PointCloudLibrary/pcl/blob/f62c018b4fc7df3dc2c096918a8462a190f28bb8/common/include/pcl/conversions.h#L210-L224
[P04]: https://github.com/ros-perception/perception_pcl/blob/6d44fb520f8fb2440f9801f6a8a977dbc751788a/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L576-L589
