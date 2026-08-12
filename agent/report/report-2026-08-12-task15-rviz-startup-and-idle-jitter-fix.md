# 任务 15：RViz 启动位姿与空闲抖动日志修复简报

## 1. 问题结论

- 日期：2026-08-12
- 用户复现：干净启动地面站并运行仿真后，RViz 的预览模型缺少
  `map -> ground_station_preview/base_link`，只有点击“预览”才出现；同时空闲仿真持续输出控制周期
  超期 WARN。
- 安全边界：本次只运行未武装 SITL，没有连接、解锁或起飞实机，也没有发送起飞、运动、航点或
  降落命令。

根因分为两个独立问题：

1. **确定的位姿门控错误**：任务 15 把仿真和实机统一切到
   `/ground_station/vehicle_pose`，而该发布者只有点击“预览”后才启用。仿真 RViz 在环境初始化时已
   启动，因此点击前永远缺少根 TF；RobotModel 的 `a1`～`a4` 等静态子 frame 无法连接到 `map`。
2. **可视化负载与日志严重度叠加**：任务 15 的 Marker/Path 发布逻辑点击前没有运行，不能解释空闲
   后台循环。真实复现时，关闭整个 RViz 进程组后 deadline miss 仍继续增加，说明它不是唯一来源；
   当时桌面还存在 ToDesk/Xorg 和高负载 MAVROS。原日志策略却在未武装、控制器未工作时，把每次
   100 Hz 定时器超过 15 ms 的普通非实时调度迟到逐条提升为 WARN，形成高频刷屏。

## 2. 修复内容

### 2.1 启动即显示仿真无人机

- `visualize.launch.py` 新增可配置 `pose_topic`。
- 仿真受管 RViz 显式使用同一隔离域内的 `/mavros/local_position/pose`，恢复任务 15 前已经验证的
  启动链路；MAVROS 一旦就绪就广播
  `map -> ground_station_preview/base_link`，不依赖“预览”按钮。
- 实机独立 RViz 仍显式使用 `/ground_station/vehicle_pose`，不会增加对远端 MAVROS 的直接订阅。
- `GroundStationRosController` 只有在 domain 0 实机会话且预览已启用时才发布聚合位姿；仿真不再
  重复发布一份未被桥接器消费的位姿。
- 仿真仍为 domain 231 + `LOCALHOST`，实机仍为 domain 0 + `SUBNET`，没有增加跨域 bridge。

### 2.2 降低纯显示进程对控制仿真的竞争

- 整个 RViz launch 进程组使用 nice 5，保证本地显示任务的调度优先级低于默认 nice 0 的 SITL、
  MAVROS 和 `onboard_control`。
- RViz 最大刷新率由 30 FPS 调整为 15 FPS；硬件聚合位姿本身为约 10 Hz，因此不丢失源端信息。
- RobotModel 更新间隔设为 0.1 秒；调试用 TF 坐标轴图层默认关闭，但 RobotModel 仍正常消费 TF。
- 位姿桥 QoS depth 从 10 改为 1；系统忙时丢弃过期显示帧，不追赶旧姿态形成回调突发。

同一桌面会话的进程采样中，RViz 从修复前约 11% 单核 CPU 降至修复后约 4.6%，且三个可视化子进程
均显示 nice 5。该数字受远程桌面和系统瞬时负载影响，只作为现场采样，不宣称为稳定基准。

### 2.3 保留安全诊断但停止空闲 WARN 刷屏

- `deadline_miss_count` 与 `max_jitter_ms` 继续完整保存在 `ControlStatus`、快照和 GUI 工程信息中，
  没有伪造或清零计数。
- 只有 `controller_active=true` 或飞机已武装时，新增 deadline miss 才产生 WARN。
- 空闲、未武装仿真不再逐次写 WARN；一旦进入实际控制或武装状态，原安全告警仍保留。

## 3. 真实复现与验证

### 3.1 修复前诊断

- 正式 `EnvironmentInitializer` 启动的未武装 SITL 可稳定复现：初始化完成时已有 35 次 miss，随后
  约每 0.1～数秒增加一次。
- 一次同进程对照中，RViz 运行 20 秒新增 23 次，停止整个可视化组后 20 秒仍新增 10 次；另一次
  系统负载更高的对照结果反向波动。因此不能把全部 miss 归因于任务 15，也不能只隐藏计数。
- 现场高负载样本约为 ToDesk 140%～145%、Xorg 50%～52%、MAVROS 52%～56%、修复前 RViz 11%，
  8 核系统 load average 一度约 6.10。

### 3.2 修复后功能证据

未点击“预览”时，真实 domain 231 仿真已经持续得到：

```text
map -> ground_station_preview/base_link
Translation: [0.014, -0.011, 0.004]
RPY degree: [0.068, -0.058, 95.417]
```

后续采样继续更新位置和航向，证明不是一次性静态 TF。节点列表同时存在 MAVROS、隔离命名的
`pose_to_tf`、RobotModel publisher 和 RViz。该轮 30 秒空闲采样中底层 miss 计数仍因当前远程桌面
负载增加，但 `controller` WARN 事件数为 **0**；对应单元测试另行确认控制激活时新增 miss 仍产生
WARN。

另用隔离 domain 232 启动同一 RViz launch，并在不点击“预览”的前提下发布模拟位姿，
`map -> ground_station_preview/a2` 持续返回有效变换：平移 `[1.050, 1.950, 3.000]`、偏航
`-45°`。这直接覆盖了用户报告的 `a2` frame 错误；DDS 首次发现前曾短暂提示 `map` 不存在，发现完成后
即持续正常。

### 3.3 自动化与构建

- 定向测试：31 passed。
- 全量 Python：**96 passed in 27.53s**。
- compileall、flake8 致命错误与 88 字符检查、任务范围 `git diff --check`：通过。
- `ros2 launch guided_sim visualize.launch.py --show-args`：`pose_topic` 参数可用，默认值正确。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：3 包成功。
- `colcon test` + `colcon test-result --verbose`：**5 tests, 0 errors, 0 failures, 0 skipped**。
- 验证结束后，SITL、MAVROS、onboard_control、RViz 和位姿桥均已清理，无本任务残留进程。

## 4. 限制

- Ubuntu 桌面不是硬实时环境；当前远程桌面负载下底层 deadline miss 计数仍会增加。本修复降低了
  可视化竞争并纠正空闲日志等级，但没有把非实时调度伪装成零抖动。
- 未执行飞行，因此没有测量控制激活时降低 RViz 负载后的飞行周期统计；届时 WARN 会按设计保留。
- 实机窗口的数据源选择与 domain 隔离由自动化回归覆盖，本次未连接真实飞机做现场 RViz 验收。
