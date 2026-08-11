# Task 16 beautify-3 执行报告

日期：2026-08-12

范围：地面站手动操纵区布局、状态块、电池/通讯阈值、滚转聚合遥测、响应式显示及真机兼容验证。

安全边界：真机调试全程未申请控制租约，未发送模式、解锁、起飞、降落、悬停、运动、航点或原点指令；所有观测均为 `armed=false`。服务重启仅用于加载已验证的新接口。

## 一、结论

任务 16 的五项界面要求均已实现，接口版本升级为 2.2，代码提交 `b84ca3e` 已推送到 `main`，真机也已同步并运行该提交。

| 任务要求 | 最终结果 |
| --- | --- |
| 默认坐标系 | 默认选择“本地 ENU”；初始化时不生成伪造的切换日志 |
| 六项摘要 | 严格为实际高度、实际速度、实际航向、指令水平速度、指令垂直速度、指令转向速度；实际速度使用三维速度模长 |
| 右侧详细状态 | 仅保留四项单次指令增量和“飞行姿态 / 俯仰/滚转”；删除地面↔机载、电池、飞控行 |
| 顶部五个状态块 | 严格为控制权、飞控模式、电池、通讯频率、最近指令；窄窗口自动换成坐标行 + 状态行 |
| 左侧详细状态 | 严格为实际位姿、目标位姿、实际速度、目标速度、控制周期、安全门控 |

## 二、实现说明

### 2.1 状态块与阈值

- 通讯频率仅显示两位小数和 `Hz`，不再显示 age；`9.00～11.00 Hz` 为绿色，其他有效频率为黄色。
- 实机电池只显示电压：`≥23.5 V` 绿色、`22.5～23.5 V` 黄色、`<22.5 V` 红色。
- 仿真电池只显示百分比：`≥50%` 绿色、`25～50%` 黄色、`<25%` 红色。
- 上述目标值、容差和电池边界均为 `ground_station_core/config.py` 中的命名常量，显示层没有重复硬编码。
- 1180×700 最小窗口下，较长的 `STABILIZE`、`23.45 V` 和 `355.7 s` 文本均完整显示，状态块无交叠或裁字。

### 2.2 滚转数据真实性

任务要求把“俯仰/偏航”改为“俯仰/滚转”。现有接口没有 roll，单纯改标签会把 yaw 伪装成 roll，因此进行了最小必要的机载接口扩展：

1. `ControlStatus.msg` 新增 `float64 roll`，接口版本由 2.1 升为 2.2。
2. 机载端从已有本地位姿四元数统一解算 roll/pitch/yaw，并在同一聚合状态发布。
3. `VehicleSnapshot` 和地面站状态存储保留 roll；详细区只在本地位姿有效时显示真实“俯仰/滚转”。

没有让地面站绕过机载聚合接口直接订阅 MAVROS，也没有修改控制算法或飞行指令语义。

## 三、自动化与视觉验证

### 3.1 本地验证

- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：成功。
- `colcon test` 后 `colcon test-result --verbose`：5 tests、0 errors、0 failures。
- 加载 Jazzy 与本地 overlay 后全量 pytest：76 passed（最终复核 23.80 s）。
- `ground_station.py --check-environment`：workspace environment OK。
- `compileall`、flake8 致命规则和提交 `b84ca3e` 的 whitespace 检查：通过。
- 新增回归覆盖默认 ENU、命令语义、六项摘要和六项详细顺序、真实 roll 映射、五个状态块顺序、频率/电池全部边界，以及 1180×700 响应式无裁切。

### 3.2 最终离屏渲染

- 主窗口：[task16-beautify-3-main.png](../codex/task16-beautify-3-main.png)
- 最小窗口：[task16-beautify-3-minimum.png](../codex/task16-beautify-3-minimum.png)
- 展开窗口：[task16-beautify-3-details.png](../codex/task16-beautify-3-details.png)
- 完整详细区：[task16-beautify-3-panel-details.png](../codex/task16-beautify-3-panel-details.png)

视觉复核确认默认 ENU、五个顶部状态块、六项摘要、左侧六行和右侧“俯仰/滚转”均符合任务文字；最小窗口的状态块按设计换行。

## 四、真机部署与只读验证

### 4.1 部署

- 部署前：服务 active，远端提交 `8ca0b61`，接口 2.1，原始状态为 `armed=false / STABILIZE`。
- 远端 `git pull --ff-only origin main` 快进到 `b84ca3e`。
- `onboard_workspace.sh verify` 在 Humble/aarch64 上完成两包构建、5 项测试和隔离 smoke；smoke 为 `interface=2.2, fcu_connected=false, armed=false, setpoint_messages=0`。
- 服务重启后保持 active；远端仓库为 `main...origin/main`，仅保留部署前已有的 `.deployment-backups/` 未跟踪目录。

### 4.2 数据与安全证据

最终重启前先放置同发行版 Humble 只读监视器，避免晚加入订阅者受到当前 Fast DDS 发现问题干扰。新服务发布后的有效窗口结果为：

| 指标 | 结果 |
| --- | ---: |
| `ControlStatus` | 164 条 / 16.501319 s，按首末时间戳为 9.8780 Hz |
| 接口 | 2.2 |
| 最后一帧 | FCU connected、local position valid、battery valid、message rates configured |
| roll | `0.0063949709 rad`，有限值 |
| 飞控模式 | STABILIZE |
| 电池 | 23.07 V |
| 武装 | 164/164 均为 false，true 为 0 |
| 租约 | `lease_active=true` 为 0 条 |
| 姿态 setpoint | 42 秒监视窗 0 条 |

另一次正式地面站只观察客户端在 12 秒内收到 109 条接口 2.2 状态，近 5 秒计算为 10.00 Hz，`control_enabled=false`、`control_authority=false`；新增 roll、pitch、yaw 均为有限值。原始 `/mavros/state` 的 7 秒窗口持续为 `connected=true / armed=false / STABILIZE`，4 秒原始姿态 setpoint 窗口为 0 条。

### 4.3 未通过项与限制

真机最终启动的 `thrust_mode_verified` 没有在监视窗内变为 true，`start_drone_all.sh` 在 120 秒后如实打印：

```text
[startup] WARNING: processes remain running, but full readiness was not reached in time
```

因此本报告只确认任务 16 的界面、接口、状态流和未解锁安全边界，**不宣称真机已达到飞行 READY**。现有安全门会因推力参数未验证而禁用起飞、悬停和运动控制；在单独查明 MAVROS 参数服务发现/同步问题并恢复 `thrust_mode_verified=true` 前，不应进行人工解锁或飞行验证。本任务没有绕过该门控，也没有擅自修改飞控参数。

同机晚启动的 Humble CLI 曾无法订阅现有状态/MAVROS 发布者，而开发机 Jazzy 正式只读客户端仍能收到数据；机载日志同时出现项目已记录的 Humble/Jazzy Fast DDS `sequence size exceeds remaining buffer`。退出 Jazzy 参与者并用重启前置 Humble 监视器复测后，FCU、位姿、电池和 9.878 Hz 状态流均恢复。该现象再次说明跨发行版发现链路不是受支持的实飞通信基线，不能用“部分话题可达”替代完整 READY。

## 五、主要改动文件

- `ground_station_core/config.py`
- `ground_station_core/models.py`
- `ground_station_core/ros_controller.py`
- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/operations_panel.py`
- `ground_station_core/qt_ui/theme.py`
- `src/guided_interfaces/msg/ControlStatus.msg`
- `src/onboard_control/include/onboard_control/onboard_control_node.hpp`
- `src/onboard_control/src/onboard_control_node.cpp`
- `src/guided_interfaces/package.xml`
- `src/onboard_control/package.xml`
- `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`
- `src/onboard_control/deploy/onboard_workspace.sh`
- `DEPLOY_UBUNTU_2204.md`
- `tests/test_bootstrap.py`
- `tests/test_onboard_truthfulness.py`
- `tests/test_qt_gui.py`
- `tests/test_ros_controller.py`
- 本报告、最终截图与 `MEMORY.md`
