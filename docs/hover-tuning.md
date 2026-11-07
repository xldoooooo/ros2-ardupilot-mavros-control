<!-- 手动悬停采集与离线 PD+DOB 分析的使用和修改入口。 -->
# 悬停跟踪误差与 PD+DOB 调试

两个工具均手动运行，不接入开机服务、地面站或飞行状态机。

- `scripts/diagnostics/record_hover.py` 订阅 `/onboard_control/tracking_sample`，保存 CSV 和同名 `.csv.json` 元信息。
- `scripts/diagnostics/analyze_hover.py` 离线生成中文 `analysis.md`、`summary.json` 与每段 PNG 曲线，不需要 ROS。

## 启用和停止

首次使用须在采集端和机载端构建新增的 `guided_interfaces/TrackingSample`；机载节点也须原生重建，按现有部署流程在地面重启加载新产物。仅修改 Python 脚本无需重新构建。脚本不会自行构建、启动服务、解锁或起飞。

仓库根目录运行，ROS domain、发现范围和命名空间应与目标飞机一致；建议在机载端记录，减少无线链路缺口。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python scripts/diagnostics/record_hover.py \
  --output agent/codex/hover-test.csv
# Ctrl+C 结束；也可添加 --duration 120，表示从脚本启动起计时120秒。
```

可在起飞前启动脚本，用户手动进入悬停后保持固定目标至少 15–30 秒。采集期间也保存其他控制模式，方便保留边界；分析默认仅选择显式悬停 `MODE_HOVER`，排除自动校准、移动参考和大间隔，并丢弃每段开头 2 秒，要求剩余至少 10 秒。不会按实际速度或误差大小筛除“飞得不好”的样本。

有诊断订阅者时，机载才启用同周期快照及独立发布定时器；最后一个订阅者退出后，DDS 发现断开即取消定时器并清空快照。无订阅者时不采集、不发布、不写诊断数据盘，仅保留已注册话题和订阅匹配事件。任何直接订阅该诊断话题的工具也会启用采集；普通状态话题订阅不会启用。多个采集者同时运行时，最后一个退出才停。

CSV 写盘全部在脚本进程。机载仅保存最新一条快照，诊断 DDS 发布不持控制锁；慢订阅/调度可能覆盖快照，`session_id/sequence` 和源单调时间用于识别重启、跳号及缺口。快照来自控制计算阶段，不证明对应 setpoint 已被飞控接收。异常退出后，已有 CSV 可继续分析，最多损失尚未刷新的一小段缓冲；最后订阅断开的时机取决于 DDS 发现。

## 离线分析

分析依赖 NumPy，PNG 可选依赖 Matplotlib；依赖仅供手动工具使用，不加入机载服务启动链：

```bash
.venv/bin/python -m pip install -r scripts/diagnostics/requirements.txt
.venv/bin/python scripts/diagnostics/analyze_hover.py \
  agent/codex/hover-test.csv --output agent/codex/hover-test-analysis
```

不绘图可加 `--no-plots`。已有 CSV/分析目录不会覆盖。`--start 20 --end 60` 按第一条源单调时间选取相对区间；`--settle`、`--min-seconds` 调整分段；`--include-waypoint-holds` 允许参考已静止的航点保持段，其当前增益以样本为准。目标变动、参数切换、校准、时间回退和源变化都会断段，避免拼接出虚假频率。

## 数据与判断边界

位置、速度、参考、DOB 及加速度均采用 **FCU local ENU**，误差为“参考−实测”；不把任务 map 坐标与 FCU local 混用。样本记录真实参与本周期计算的速度，包括校准期间替代速度，但默认不分析校准段。

每轴给出偏差、RMS、P95、峰值、速度误差、P/D 项 RMS、DOB 均值/波动/限幅占比、摆动主频和输出修改占比；另有 XY 合成误差、姿态差、推力边界、遥测年龄、控制周期与快照缺失统计。

`pose_age_s/velocity_age_s` 是机载接收缓存年龄，不是传感器源数据年龄，不能单独排除上游延迟；链路异常还需对照现有 MAVROS/机载诊断日志。

`disturbance` 是控制器内部估计值，不是实测外力。`controller_acceleration` 是 DOB 使用的输入历史；当两个 jerk 上限均为零时，它可能位于倾角限制之前。`mapped_acceleration` 为倾角/最小竖直分量限制后的命令，扣除了重力，尚未计入推力截断影响，不是实测加速度。分析以当前 `wn/zeta` 重建 `P+D(+前馈)−DOB`，与 mapped 值比较；“输出修改”可能来自加速度、倾角或 jerk 限制，不等同单一硬饱和。

参数关系来自本项目控制器：`Kp=wn²`，`Kd=2*zeta*wn`。报告优先提示采样异常、限幅和内环跟随问题，再给出以下试验方向：周期摆动时调整阻尼/带宽；DOB 高频活动明显时减小 L 做对照；稳态偏差先查定位、姿态零偏与悬停推力。每次只改一组参数，XY 两轴共用增益；脚本不修改参数，也不能从静态悬停唯一确定最佳增益或替代实飞验收。

## 修改入口

新增采样量修改 `TrackingSample.msg` 和 `src/onboard_control/src/tracking_diagnostics.cpp`；采样点位于现有 `publish_attitude_setpoint()` 的有效输出计算后。修改统计/启发式只需编辑分析脚本，采集脚本不依赖分析代码。订阅匹配与独立回调组负责启停，不增加新的飞行命令或控制分支。
