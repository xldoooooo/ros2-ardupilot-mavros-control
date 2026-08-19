# 航点平稳飞行方法、GUI 切换与 SITL 调参简报

## 1. 完成结论

- 日期：2026-08-12。
- 保留原有 `STEP_POSITION + POSITION_PD_DOB` 基线及其原始增益、位置阶跃、到达判据和默认 GUI
  选择；新增功能不改变该分支的控制数值路径。
- 新增并跑通三组连续参考实验：
  1. `SECOND_ORDER_FILTER + TRAJECTORY_PD_DOB`；
  2. `TRAPEZOIDAL_PROFILE + TRAJECTORY_PD_DOB`；
  3. 主推荐 `JERK_LIMITED_S_CURVE + TRAJECTORY_PD_DOB`，轨迹反馈使用独立低带宽
     PD+DOB 增益。
- GUI 已删除“执行与进度”标题，进度条与发送按钮按约 2:1 同行排列；下一行按“避障策略 / 命令生成 /
  跟踪控制”三列横向排列。
- 避障策略仍为明确空壳；非直线选项会警告并按直线执行，没有伪造避障功能。
- 任意参考生成/跟踪组合仍可传递。未验证组合在仿真和实机模式都会先显示默认取消的确认框；本次只对
  基线和三种连续参考配轨迹 PD+DOB 的组合负责调试。
- 三项选择只允许在已降落、解除武装且待机时修改。GUI 起飞后禁用下拉框，机载服务还会拒绝绕过 GUI
  在同一武装周期内更换组合；解除武装后自动清锁。
- 本次只运行本机 domain 231 + `LOCALHOST` 隔离 SITL；没有连接、解锁或起飞实机。

## 2. GUI 与切换行为

### 2.1 布局

- 空标题 `Card` 会彻底省略标题行，不留下隐藏布局占位。
- 第一行：权威机载进度条 stretch=2，发送按钮 stretch=1。
- 第二行：三个等宽字段，每个字段采用标签在上、向下弹出的下拉框在下。
- 625×141 和最小 442×141 两种执行卡尺寸均完成离屏视觉检查；进度条/按钮保持同行，三列没有换行。
- 本地视觉证据：
  - `agent/codex/waypoint-method-layout.png`；
  - `agent/codex/waypoint-method-layout-minimum.png`。

### 2.2 冷切换而非飞行中热切换

完整选择随一次 `ExecuteWaypoints` 请求原子上传：

```text
flight_strategy
reference_generator
tracking_controller
```

地面 GUI 的 `waypoint_configuration` 门控要求：环境已连接、工作流不忙、机载/飞控在线、飞机未武装、
模式为 `IDLE`、当前没有航点任务。机载端再保存本武装周期的三项锁定值：

- 第一次航点任务锁定组合；
- 同一武装周期再次提交相同组合可以继续执行；
- 任一字段发生变化，服务直接拒绝并要求先降落解除武装；
- FCU 回报解除武装后清除锁，下一轮实验可以重新选择；
- 武装前发生启动失败时也清锁，不会留下无法恢复的陈旧配置。

SITL 中已连续完成“起飞—往返航点—降落解除武装—更换方法”的四轮实验；另在主推荐任务完成但仍
武装时提交不同方法，机载端按设计明确拒绝。

## 3. 控制架构

### 3.1 参考生成与跟踪控制解耦

新增统一 `ReferenceGenerator` 接口与工厂，所有方法输出同一结构：

```text
position reference
velocity reference
acceleration reference
yaw reference / yaw rate
reference phase / finished
```

四种生成器独立实现：位置阶跃、二阶命令滤波、普通梯形速度、七阶段限 jerk S 曲线。三种连续方法都用
一维航段进度映射到三维直线，避免逐轴限幅导致空间路径弯曲；XY 模长和 Z 分量限制会换算为同一航段
标量限制，因此三轴同步到达。

跟踪侧继续只有一套生产 `DobController`：

- `POSITION_PD_DOB` 使用原有位置/速度误差和原有增益，不启用参考加速度前馈；
- `TRAJECTORY_PD_DOB` 切换到独立低带宽增益，并加入参考加速度前馈；
- 两者共用已由 FCU 校准的悬停油门映射、重力补偿、倾角/加速度/扰动硬限幅；切换增益不会覆盖
  `MOT_THST_HOVER` 校准值。

### 3.2 100 Hz 执行路径与负载

生产调用链保持集中：

```text
OnboardControlNode::control_tick (100 Hz)
  -> update_waypoint_executor
     -> ReferenceGenerator::update
  -> publish_attitude_setpoint
     -> DobController::compute
```

S 曲线的峰值速度搜索只在航段初始化时做 60 次二分；100 Hz 循环只在最多七个解析相位中取样，没有
每周期优化器、矩阵求解或动态分配。最终主推荐往返航段实测：

- 机载 `ros2 run` 进程树约占单核 **3.92%**；加入 90° 航向变化时约 **4.04%**；
- 控制率均值约 **100.00 Hz**，最小值约 **99.98 Hz**；
- 航点任务期间 `deadline_miss_delta=0`。

上述 CPU 数值来自当前桌面 SITL，只用于评估算法量级，不等同于目标机载计算机的最终占用率。

## 4. 参数入口

全部生产参数位于 `src/onboard_control/config/control.yaml`，按方法分组并带用途注释，没有把调参值写死
在 GUI 或航点执行逻辑中。

| 方法/安全层 | 独立配置前缀或参数 | 当前默认意图 |
|---|---|---|
| 位置 PD+DOB 基线 | `hover_wn_*`、`hover_zeta_*`、`dob_L_*` | 完整保留原值 |
| 轨迹 PD+DOB | `trajectory_wn_*`、`trajectory_zeta_*`、`trajectory_dob_L_*` | 低带宽反馈，DOB XY/Z 为 0.5/0.3 |
| 二阶命令滤波 | `second_order_filter_*` | XY/Z 速度 0.30/0.15 m/s，独立频率、阻尼、加速度和完成阈值 |
| 普通梯形速度 | `trapezoidal_*` | XY/Z 速度 0.30/0.15 m/s，独立加速/减速上限 |
| 限 jerk S 曲线 | `s_curve_*` | XY/Z 速度 0.30/0.15 m/s，独立加速、减速和 jerk 上限 |
| 航点动态保护 | `waypoint_start_speed_tolerance`、`waypoint_arrival_speed_tolerance`、`waypoint_actual_speed_guard_*`、`waypoint_speed_guard_observations` | 平滑任务从稳定悬停开始，低速到达；持续超速则取消到悬停 |

当前 S 曲线关键默认值：

```yaml
s_curve_max_velocity_xy: 0.30
s_curve_max_velocity_z: 0.15
s_curve_max_acceleration_xy: 0.18
s_curve_max_acceleration_z: 0.10
s_curve_max_deceleration_xy: 0.20
s_curve_max_deceleration_z: 0.12
s_curve_max_jerk_xy: 0.15
s_curve_max_jerk_z: 0.08
```

参考速度是轨迹规划硬上限；真实机体受闭环动态影响可能略有跟踪超调，因此另设实际速度持续超限保护
（当前 XY/Z 为 0.42/0.24 m/s，连续 5 次观测触发）。这两个概念没有混为同一个“绝对物理限速”。

## 5. SITL 调参与结果

### 5.1 实验条件

- ArduCopter SITL + MAVROS + 本仓库 `onboard_control`，接口版本 3.0；
- 每轮起飞到约 1.5 m，等待连续稳定悬停；
- 航点为当前位置 → X 方向 +4 m → 返回当前位置，总航程 8 m；
- 每种方法完成后 LAND，等待 FCU 回报解除武装，再选择下一种方法；
- 采样约 10 Hz 权威 `ControlStatus`，控制器实际运行 100 Hz；
- 调参前后各完成一轮四方法闭环；主推荐随后又独立重复一次，并增加一次 90° 航向变化往返。

### 5.2 最终同条件对比

| 组合 | 用时/s | 实际峰值速度/(m/s) | 恒速实际均值±标准差/(m/s) | 最大倾角/° | 倾角 RMS/° | 姿态变化率 RMS/(°/s) |
|---|---:|---:|---:|---:|---:|---:|
| 位置阶跃 + 位置 PD+DOB 基线 | 5.75 | 3.707 | 无平台 | 25.15 | 18.09 | 52.88 |
| 二阶滤波 + 轨迹 PD+DOB | 48.73 | 0.376 | 无严格平台 | 1.68 | 0.50 | 0.57 |
| 梯形速度 + 轨迹 PD+DOB | 31.69 | 0.334 | 0.303±0.019 | 1.70 | 0.71 | 1.29 |
| **限 jerk S 曲线 + 轨迹 PD+DOB** | **34.29** | **0.327** | **0.303±0.019** | **1.73** | **0.61** | **0.64** |

结论：

- 基线被原样保留，也清楚复现了远航点位置阶跃导致的高速度、大倾角和大姿态变化率。
- 二阶滤波最柔和但耗时最长，且状态始终是滤波收敛过程，没有可解释的严格匀速平台，适合作为平滑
  对照而不是本项目主方案。
- 梯形速度具备最长的明确匀速段，但加速度在相位边界跳变，姿态变化率 RMS 约为 S 曲线的两倍。
- S 曲线保留约 20.9 秒的恒速样本，同时用连续 jerk 过渡显著降低姿态变化率，是本次需求下的主推荐。
- 主推荐独立重复时得到实际峰值 0.327 m/s、恒速标准差 0.0185 m/s、最大倾角 1.72°、姿态变化率
  RMS 0.63°/s，结果可重复。

### 5.3 90° 航向变化附加验证

主推荐在 4 m 航段中同步改变 90° 航向并返航：

- 实际峰值速度 0.324 m/s；恒速均值 0.300 m/s，标准差 0.019 m/s；
- 最大倾角 1.72°，姿态变化率 RMS 0.65°/s；
- 参考/实际最大航向速率约 6.75/7.49°/s；
- 100 Hz 零超期，进程树约占单核 4.04%。

平移稳定性没有因这一长航段转向退化。纯原地转向或极短距离大航向变化没有在本次范围内单独调参。

## 6. 自动化与质量检查

- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：三包成功。
- `colcon test` + `colcon test-result --verbose`：**13 tests，0 errors，0 failures，0 skipped**；包含
  6 个 PD+DOB 测试和 5 个参考生成器测试，其余为包级结果。
- 正式 Python 测试目录：**103 passed in 28.11s**。
- 参考生成器测试覆盖：基线语义、二阶限速收敛、梯形长匀速平台、S 曲线速度/加速度/jerk 上限、
  S 曲线短航段自动降峰值速度。
- GUI/协议测试覆盖：无标题布局、2:1 行布局、三下拉框、解除武装门控、未验证组合默认取消、三个字段
  原子排队和 ROS 服务载荷、接口 3.0 版本门。
- `compileall`、致命 flake8、任务新增行 88 字符检查、YAML 解析、shell 语法、任务范围
  `git diff --check`：通过。
- 仓库根目录无范围 `pytest` 会收集用户现有且未跟踪的
  `integration/websocket_test_demo/tests/test_protocol.py`，该目录因缺少 `ws_demo` 包在收集阶段失败。
  此目录与本任务无关且不属于正式 `tests/`，没有擅自修改；上述 103 个项目测试全部通过。
- 每次仿真结束均确认受管 SITL、MAVROS、onboard_control、RViz 四个进程已停止，残留和清理错误均为
  空。

## 7. 已知边界与现场建议

- 避障策略仅保留接口和 GUI 选项，当前任何非直线选择仍按直线执行；警告框会明确说明。
- 非推荐的自由组合只保证能够选择、传输和执行框架，不代表已调参或具备稳定性结论。
- 接口从 2.2 升级到 3.0，增加方法选择和轨迹诊断字段；地面站、`guided_interfaces` 与机载节点必须
  同步部署，版本门会拒绝混用。
- SITL 结果不能替代真实机体的质量、推力、桨叶、定位噪声和气流验证。现场前应由用户手动完成无桨
  检查和低风险分级试飞；本任务没有、也不得自行连接或起飞实机。
- 首次真实室内测试建议保持主推荐默认值，先用 2～4 m 航段核对实际速度保护、定位噪声和高度保持，
  再一次只调整一个参数组；不要同时提高速度、加速度、jerk 和 DOB 带宽。
