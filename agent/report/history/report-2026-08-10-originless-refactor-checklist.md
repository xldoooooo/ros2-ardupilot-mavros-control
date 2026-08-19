# 不配置 GPS 原点改造清单

## 1. 报告信息

- 日期：2026-08-10
- 调研项目：`/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware`
- 调研基线：`main`，提交 `876ad22b39137d4fe32f9b0d040b4264f6c835d0`
- 对照的 ArduPilot：`/home/nvidia/scq/projects/ardupilot` 当前本地源码
- 本报告性质：后续实施参考清单；本轮只新增本报告，没有修改功能代码、飞控参数或实机状态
- 安全边界：后续自动化只能在 SITL 中执行解锁/起飞。实机解锁和起飞必须由操作者手动执行，本清单不授权任何自动实飞操作

## 2. 最终结论

可以做到，而且**不需要修改 ArduPilot 源码**。

需要实施的范围是：

1. 修改 `ros2-ardupilot-sitl-hardware`，删除“完整连接必须写 GPS 原点”的产品逻辑；
2. 把当前依赖 `/mavros/cmd/takeoff` 的原生 Guided 起飞，改为项目已有 PD+DOB/`AttitudeTarget` 链路上的本地 ENU 相对高度起飞；
3. 用 ArduPilot 参数把 EKF3 的位置源设为 ExternalNav，并禁用 GPS 和依赖全球位置的功能；
4. 增加真正关闭 GPS、未写原点的 SITL 回归，而不是继续用 SITL 默认 GPS/Home 掩盖问题。

这里的“不配置 GPS 原点”是指：

- 项目不再发布 `/mavros/global_position/set_gp_origin`；
- 地面站、机载服务和共享接口中不再出现经纬高原点；
- 起飞、悬停、键盘控制、局部航点、取消和降落都不依赖 WGS-84、Home、RTL 或全球航点；
- ArduPilot/EKF 内部仍可能维护自己的局部 NED 数学参考，这是估计器内部实现，不等于由本项目配置一个 GPS 地理原点。

## 3. 为什么只删 `set_gp_origin` 还不够

当前有两个独立的原点依赖，必须一起处理：

1. `ground_station_core/environment.py` 的实机连接流程把 `set_gp_origin` 成功作为连接成功的硬条件；
2. `src/onboard_control/src/onboard_control_node.cpp` 通过 MAVROS `CommandTOL` 请求 ArduPilot 原生 Guided 起飞。当前 ArduPilot 的 `ModeGuided::do_user_takeoff_start_m()` 通常会把“相对 Home 高度”转换为“相对 EKF origin 高度”，没有 Home/origin 时可以拒绝起飞。

ArduPilot 的普通 `GUIDED` 模式本身并不等同于“必须有 GPS”。当前源码的 `Copter::position_ok()` 接受 EKF 的绝对位置或相对位置；EKF3 ExternalNav 可以提供相对位置。因此应保留 `GUIDED + EKF3 ExternalNav`，只替换原生 `CommandTOL` 起飞，不需要切换到 ExternalAHRS，也不需要修改 ArduPilot 的模式或解锁源码。

## 4. 推荐接口决策

推荐做一次干净的协议 3.0 升级：

- 删除 `SetGpsOrigin.srv`，而不是保留误导性的死接口；
- `FlightCommand.srv` 为起飞同时传相对高度和爬升速度；
- `ControlStatus.msg` 增加估计器健康和着陆状态，供起飞门控、完成判定及 GUI 诊断使用；
- 地面站、机载端、`guided_interfaces` 与 `onboard_control` 版本统一升到 `3.0/3.0.0`。

若必须临时兼容 2.0 客户端，可以保留但不调用 `SetGpsOrigin`，并发布弃用告警；这会留下错误产品语义，不建议作为最终状态。

## 5. 需要修改的文件清单

### 5.1 共享 ROS 2 协议

| 文件 | 优先级 | 要修改的地方 | 验收点 |
|---|---:|---|---|
| `src/guided_interfaces/srv/SetGpsOrigin.srv` | 必须 | 删除文件。经纬高不再属于产品接口 | 构建产物中没有 `SetGpsOrigin` 类型和 `/onboard_control/set_gps_origin` 服务 |
| `src/guided_interfaces/srv/FlightCommand.srv` | 必须 | 保留 `value` 表示起飞相对高度，新增明确的 `speed` 字段表示本地 Z 轴最大爬升速度；注释其他命令忽略这两个字段 | 起飞请求同时携带高度和速度；非法/NaN/越界值在机载端拒绝 |
| `src/guided_interfaces/msg/ControlStatus.msg` | 必须（安全） | 增加高层估计器健康状态和着陆状态。建议用项目自有常量表示 `UNKNOWN/ON_GROUND/IN_AIR/TAKEOFF/LANDING`，不要把 GUI 直接耦合到 MAVROS 常量 | GUI 能区分“话题新鲜但 EKF 不健康”和“可靠局部位置”；起飞完成不能只靠高度阈值 |
| `src/guided_interfaces/CMakeLists.txt` | 必须 | 从接口列表删除 `srv/SetGpsOrigin.srv`；删除 `find_package(geographic_msgs)` 及 `DEPENDENCIES geographic_msgs` | `colcon build` 不再需要 `geographic_msgs` |
| `src/guided_interfaces/package.xml` | 必须 | 版本升为 `3.0.0`；删除 `geographic_msgs` 依赖；更新描述中的接口版本语义 | 包版本、Python 常量、C++ 常量一致 |

### 5.2 机载控制节点

| 文件 | 优先级 | 要修改的地方 | 验收点 |
|---|---:|---|---|
| `src/onboard_control/include/onboard_control/onboard_control_node.hpp` | 必须 | 删除 `geographic_msgs`、`SetGpsOrigin`、`CommandTOL` 的 include、别名、回调、publisher/service/client 成员和 `send_takeoff_request()`；加入 `mavros_msgs::msg::EstimatorStatus`、`ExtendedState` 的订阅与新起飞状态；声明本地起飞规划器接口 | 头文件不存在地理原点和原生 takeoff client |
| `src/onboard_control/src/onboard_control_node.cpp` | 必须 | 版本改为 `3.0`；删除 `/global_position/set_gp_origin` publisher、`/set_gps_origin` service、`/cmd/takeoff` client 和 `on_set_gps_origin()`；把起飞改为本地 ENU 参考轨迹；加入估计器/着陆状态门控和失效处理 | 起飞全程不调用 `CommandTOL`，目标为 `z_start + requested_height`，不是绝对 `requested_height` |
| `src/onboard_control/include/onboard_control/takeoff_reference.hpp` | 建议新增，安全相关 | 提取无 ROS 依赖的本地起飞参考生成器/状态机，保存起始姿态、相对目标、速度和加速度限制、完成驻留计时 | 核心算法能用 GTest 确定性测试，不依赖真实 ROS/飞控 |
| `src/onboard_control/src/takeoff_reference.cpp` | 建议新增，安全相关 | 实现限速、限加速度的 Z 参考斜坡；固定起飞点 `x0/y0/yaw0`；处理非有限输入、时间跳变和局部位姿重置 | 无一步目标跳变；不同初始 Z 下都得到相同相对爬升高度 |
| `src/onboard_control/config/control.yaml` | 必须 | 新增起飞参考加速度、默认/最大爬升速度、高度容差、垂直速度容差、完成驻留时间、预发送时间、估计器状态超时、允许的最大位姿跳变量等参数；保留并重新审查 `takeoff_timeout_seconds` | 所有安全阈值可配置、带单位注释、启动时做范围校验 |
| `src/onboard_control/CMakeLists.txt` | 必须 | 删除 `geographic_msgs`；编译并链接新的起飞规划器；注册起飞规划器 GTest | `BUILD_TESTING=ON` 时新旧 C++ 测试都执行 |
| `src/onboard_control/package.xml` | 必须 | 版本升为 `3.0.0`；删除 `geographic_msgs` 依赖 | Humble/Jazzy 两端依赖清单一致 |

#### `onboard_control_node.cpp` 的起飞状态机必须满足

1. 接收命令时要求 FCU 在线、局部位姿和速度新鲜、EKF 相对水平位置/垂直位置有效、着陆状态为 `ON_GROUND`、推力语义已验证、无 setpoint 冲突；
2. 在真正爬升前捕获 `x0/y0/z0/yaw0`，最终目标为 `z0 + height`；
3. 先安全预发送姿态 setpoint，再请求 `GUIDED`，确认模式遥测，随后请求武装并确认 `armed` 遥测；预发送阶段不得输出会意外离地的正推力；
4. 进入爬升阶段后才沿限速、限加速度的 Z 参考上升，同时保持 `x0/y0/yaw0`；
5. `MODE_TAKEOFF` 必须加入姿态 setpoint 发布分支和 `raw_control_mode` 安全分支。当前代码在这两处漏掉 TAKEOFF，不能沿用；
6. 完成条件至少同时包含：`IN_AIR`、高度误差小于容差、垂直速度小于容差，并连续维持配置的驻留时间；完成后无扰切换到 HOVER；
7. 模式切换失败、武装失败、估计器失效、局部位姿跳变、姿态发布冲突、超时或租约失效都必须生成唯一终态结果；已经武装/开始离地时进入现有 LAND 保护，未武装时只失败退出；
8. 不要在此任务中顺带实现 `TODO.md` 的降落速度、避障或控制器 jerk 优化；只实现原点移除所需的起飞参考平滑。

### 5.3 地面站逻辑与 GUI

| 文件 | 优先级 | 要修改的地方 | 验收点 |
|---|---:|---|---|
| `ground_station_core/config.py` | 必须 | `INTERFACE_VERSION` 改为 `3.0`；删除 `DEFAULT_GPS_ORIGIN`；把 `TAKEOFF_SPEED` 注释改为项目本地起飞参考速度，不再声称只是 ArduPilot `WP_SPD_UP`；增加 originless SITL 参数文件路径常量 | 配置层无经纬高默认值 |
| `ground_station_core/environment.py` | 必须 | `initialize_hardware()`、`_hardware_workflow()` 删除 `origin` 参数；实机第 3 步只配置消息频率并等待 ExternalNav/EKF 健康，不再调用 `request_set_gp_origin()`；重写所有“写入飞控原点”文案；仿真改用无 GPS Vicon 参数档 | 硬件连接不发布原点，且必须等待真实 ExternalNav 相对位置就绪 |
| `ground_station_core/ros_controller.py` | 必须 | 删除 `request_set_gp_origin()`、`SetGpsOrigin` import、client、实体和命令分发；`request_takeoff(altitude, speed)` 以结构化参数排队，并填写 `FlightCommand.speed`；映射新增状态字段 | 命令队列和服务请求保留高度、速度，不存在 origin client |
| `ground_station_core/models.py` | 必须（若扩展状态） | `VehicleSnapshot` 增加估计器健康和着陆状态；状态超时时把它们恢复为未知/无效 | GUI 不会在旧状态超时后继续显示可起飞 |
| `ground_station_core/qt_ui/main_window.py` | 必须 | `_initialize_hardware()` 不读取/展示/传递原点；风险提示改为“申请租约、配置频率、等待 ExternalNav/EKF”；`_takeoff()` 同时读取高度和速度、在确认框显示两者并传给 ROS 控制器 | 完整连接没有原点副作用；实机起飞仍需要明确人工点击和高风险确认 |
| `ground_station_core/qt_ui/operations_panel.py` | 必须 | 删除 `OriginConfigDialog`、`_origin`、原点齿轮、摘要、访问器和 tooltip；环境卡片说明改成局部 ExternalNav；起飞速度 tooltip 删除“暂不下传飞控”，明确其限速用途；辅助按钮尺寸逻辑只保留 Wi-Fi 按钮 | UI 不再提供无意义的经纬高设置，速度控件与实际请求一致 |
| `ground_station_core/qt_ui/state.py` | 必须 | 删除 `UiAvailability.origin_settings` 及其派生逻辑；起飞门控加入估计器健康和 `ON_GROUND`；其他局部位置模式继续要求健康相对位置 | 飞行器不在地面或估计器未就绪时起飞按钮不可用 |
| `ground_station_core/qt_ui/theme.py` | 必须 | 删除 `originSettingsButton` 的专用 QSS 选择器，确认移除后 Wi-Fi/连接按钮布局正常 | 没有悬空样式规则和布局空洞 |

### 5.4 无原点 SITL 配置与进程管理

| 文件 | 优先级 | 要修改的地方 | 验收点 |
|---|---:|---|---|
| `src/guided_sim/config/originless-vicon.parm` | 必须新增 | 保存关闭 GPS、启用 EKF3 ExternalNav/Vicon、真实推力语义和不复用记录原点的 SITL 参数，禁止依赖默认 GPS/Home | 清空 EEPROM 后使用该文件仍可得到健康相对位置 |
| `src/guided_sim/CMakeLists.txt` | 必须 | 安装新增的 `config/` 目录 | 源码运行和安装空间运行使用同一参数档 |
| `ground_station_core/environment.py` | 必须 | 启动 `sim_vehicle.py` 时增加 `--add-param-file <originless-vicon.parm>` 以及 `-A --serial5=sim:vicon:`；不要只写一个 `GPS1_TYPE=0` 就宣称 ExternalNav 已存在 | SITL 中 ExternalNav 数据来自 `sim:vicon`，GPS 为关闭状态 |
| `ground_station_core/process_manager.py` | 必须 | 更新 SITL 残留识别签名，使新增参数文件/`sim:vicon` 启动方式仍能被安全清理；继续基于 argv 精确匹配，不能扩大到所有 ArduPilot 进程 | 初始化失败或关闭后无 SITL/MAVProxy/MAVROS/onboard/RViz 残留 |
| `tests/test_process_manager.py` | 必须 | 增加新 SITL argv 的正例和无关 Vicon/ArduPilot 进程的反例 | 清理能力不因启动参数变化而回归，也不误杀其他项目 |

建议的 SITL 参数档至少覆盖以下项目，最终值必须以当前本地 ArduPilot 实测为准：

```text
AHRS_EKF_TYPE       3
AHRS_OPTIONS        0
ARMING_NEED_LOC     0
GPS1_TYPE           0
SIM_GPS1_TYPE       0
VISO_TYPE           1
SERIAL5_PROTOCOL    1
EK3_SRC1_POSXY      6
EK3_SRC1_VELXY      6
EK3_SRC1_POSZ       6
EK3_SRC1_VELZ       6
EK3_SRC1_YAW        1
GUID_OPTIONS        8
SIM_VICON_TMASK     3
```

说明：SITL 的 `sim:vicon` 是可重复的 ExternalNav 替身，验证“ArduPilot + 本项目在无 GPS、无人工原点下运行”；它不能替代真实 Odin/extnav 链路验收。

### 5.5 部署文件和文档

| 文件 | 优先级 | 要修改的地方 | 验收点 |
|---|---:|---|---|
| `src/onboard_control/deploy/onboard_workspace.sh` | 必须 | 依赖检查列表删除 `geographic_msgs`；烟雾测试期望接口改为 `3.0`；如状态消息新增字段，加入“未连接时安全默认值”断言 | 部署脚本仍不包含真实解锁/起飞动作 |
| `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md` | 必须 | 全部接口版本改为 3.0；删除正式连接“写 GPS 原点”的说明；新增 ExternalNav 参数审计、只读 estimator/extended state 检查和无原点台架顺序 | 文档不再把原点当成必需项，也不把 SITL 结果冒充真机结果 |
| `TODO.md` | 实现完成后再改 | 将“修改 AP 源码删除起飞原点依赖”改写为“ROS2 本地起飞与 ExternalNav 无原点验收”；仅在实际完成并取得用户授权后更新，不在本轮擅自实现其他 TODO | 后续人员不会再次走向无必要的 AP 源码分叉 |
| `MEMORY.md` | 实现完成后必须改 | 删除旧的“完整连接会写 GPS 原点”和默认 SITL GPS/Home 结论，记录协议 3.0、本地起飞状态机、参数档、实测范围与剩余限制 | 只记录已实现、已验证事实，不提前写完成 |
| `agent/report/report-<实施日期>-originless-refactor.md` | 实现完成后必须新增 | 如实记录修改、测试、未通过项、SITL 参数、日志路径和是否触及实机 | 不覆盖本报告；失败指标不得美化 |

### 5.6 现有测试修改

| 文件 | 必须修改的断言 |
|---|---|
| `tests/test_environment_communication.py` | 删除 fake `request_set_gp_origin()` 和 `("set_gp_origin", origin)` 断言；硬件完整连接测试改为确认租约、消息频率、ExternalNav/EKF 等待存在且原点调用不存在 |
| `tests/test_qt_gui.py` | 删除 `OriginConfigDialog`、原点按钮/摘要/可用性和 `last_origin` 测试；更新环境卡布局；起飞测试断言 `(height, speed)` 同时下传且确认框显示速度 |
| `tests/test_ros_controller.py` | 起飞排队参数改为结构化高度+速度；验证 ROS 请求的两个字段；接口版本断言改为 3.0；新增 origin 服务不存在的协议测试 |
| `tests/test_bootstrap.py` | 地面站/C++/两个 package.xml 的版本期望统一改为 `3.0/3.0.0` |
| `tests/test_onboard_deploy.py` | 依赖期望删除 `geographic_msgs`，烟雾接口改为 3.0；继续保证部署脚本不含 `/cmd/arming`、`/cmd/takeoff` 或 `COMMAND_TAKEOFF` |
| `tests/test_process_manager.py` | 覆盖新的 originless Vicon SITL argv 和精确清理边界 |

建议新增：

| 新文件 | 内容 |
|---|---|
| `src/onboard_control/test/test_takeoff_reference.cpp` | 覆盖非零初始 Z、限速/限加速度、XY/Yaw 保持、完成驻留、超时、位姿跳变、NaN 和中断 |
| `tests/test_originless_contract.py` | 静态协议回归：生产代码不再导入/创建 `SetGpsOrigin`、`CommandTOL`、`geographic_msgs`；SITL 参数档明确关闭 GPS 并启用 ExternalNav |

不要把会自动连接真实 MAVROS 的测试放进默认 `pytest`。SITL 飞行集成测试必须显式标记/单独运行，并在执行前校验 FCU URL 为 localhost、进程为 SITL、无真实串口。

## 6. ArduPilot 只改参数，不改源码

### 6.1 必须核对的参数

| 参数 | 建议 | 说明 |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | 使用 EKF3。不要设为 `11` ExternalAHRS；当前 ExternalAHRS 的相对位置输出显式依赖它自身提供 origin |
| `GPS1_TYPE`（以及实际启用的其他 GPS 实例） | `0` | 关闭 GPS 驱动；修改后通常需要重启飞控 |
| `EK3_SRC1_POSXY` | `6` | ExternalNav 是项目局部 X/Y 的主位置源 |
| `EK3_SRC1_VELXY` | `6`，前提是 Odin/extnav 速度质量已验证 | 若生产桥没有可靠速度，必须先确认实际消息内容，不能机械照抄 |
| `EK3_SRC1_POSZ` | `1`（Baro）或 `6`（ExternalNav）二选一 | 由高度需求、Odin Z 质量、气压漂移和实测决定；不是所有平台都应设为 6 |
| `EK3_SRC1_VELZ` | `6` 或 `0` | 只有 ExternalNav 垂直速度可靠时才用 6 |
| `EK3_SRC1_YAW` | `1`（Compass）或 `6`（ExternalNav） | 取决于磁环境和 ExternalNav 航向是否经过安装角、时间同步和跳变验证 |
| `VISO_TYPE` | 与实际 MAVLink vision/odometry 输入匹配，当前源码测试常用 `1` | 先确认 extnav bridge 最终发送的是 `VISION_POSITION_ESTIMATE`、速度消息还是 `ODOMETRY` |
| `GUID_OPTIONS` | 必须包含 bit 3，当前项目期望 `8` | 使 `SET_ATTITUDE_TARGET.thrust` 表示真实归一化推力；否则 PD+DOB 输出语义错误 |
| `ARMING_NEED_LOC` | `0` | 不额外强制全球位置；但它不能代替健康的相对位置，`GUIDED` 仍需 `position_ok()` |
| `AHRS_OPTIONS` | 保留其他需要的 bit，但清除 bit 3/4 | 不自动记录 origin，也不为 non-GPS 恢复记录过的 origin；不要在不审计其他 bit 的情况下盲目整值写 0 |
| 围栏、RTL、SmartRTL 和各类 failsafe 动作 | 禁用全球位置功能，或改为经安全评审的 LAND/人工接管策略 | 任一失败策略仍选择 RTL，就不能声称整套系统不依赖 Home/全球位置 |

ExternalNav 的 Z、垂直速度和 Yaw 来源是**现场选择项**，不是固定答案。修改前必须记录原参数，修改后读取回验并重启，再根据 `ESTIMATOR_STATUS`、创新量和日志判断，而不是只看 `/mavros/local_position/pose` 是否有数据。

当前未能连接 `192.168.112.186`，因此本报告没有核实真实飞控上现有参数、Odin 输出字段和 extnav 消息类型。实施者不得把上表直接批量写入真机后跳过验证。

### 6.2 明确不要修改的 ArduPilot 文件

以下文件用于解释结论，不在本次改造范围内：

- `/home/nvidia/scq/projects/ardupilot/ArduCopter/mode_guided.cpp`
- `/home/nvidia/scq/projects/ardupilot/ArduCopter/mode.cpp`
- `/home/nvidia/scq/projects/ardupilot/ArduCopter/system.cpp`
- `/home/nvidia/scq/projects/ardupilot/ArduCopter/AP_Arming_Copter.cpp`
- `/home/nvidia/scq/projects/ardupilot/libraries/AP_Arming/AP_Arming.cpp`
- `/home/nvidia/scq/projects/ardupilot/libraries/AP_AHRS/`
- `/home/nvidia/scq/projects/ardupilot/libraries/AP_NavEKF3/`

不要删除 Guided 的通用位置检查，不要伪造 Home/origin，不要把全球位置检查硬编码绕过，也不要改 `modules/`。项目已经有本地控制链，正确的边界是在 ROS2 项目内停止调用全球起飞接口。

## 7. 推荐实施顺序

1. 备份真实飞控完整参数和现有启动脚本，只做只读审计，不连接控制会话；
2. 先完成协议 3.0、删除 origin 接口和对应 Python/C++ 死代码，使项目能重新构建；
3. 提取并单测本地起飞参考生成器；
4. 接入机载起飞状态机、EstimatorStatus/ExtendedState 门控和失败保护；
5. 更新地面站 takeoff 高度+速度传输及 GUI，删除全部原点 UI；
6. 增加 originless Vicon SITL 参数档与进程清理规则；
7. 运行 Python、C++、colcon 和安全烟雾测试；
8. 清空 SITL EEPROM，从零启动无 GPS/无原点回归，依次验证起飞、悬停、键盘、本地航点、取消、LAND 和外部定位丢失保护；
9. 在拆桨、地面供电、实机保持未武装的条件下，只读验证真实 Odin/extnav、EKF3 状态和参数；
10. 只有前述证据全部通过后，才由用户另行决定并手动执行实机解锁/起飞测试。

## 8. 完成验收清单

### 8.1 静态和构建

- [ ] 生产源码中不存在 `SetGpsOrigin`、`set_gp_origin`、`CommandTOL` 和 `geographic_msgs`；
- [ ] `/onboard_control/set_gps_origin` 和 `/mavros/cmd/takeoff` 不再由本项目创建/调用；
- [ ] 地面站、机载常量和两个 package.xml 版本一致；
- [ ] `colcon build --packages-select guided_interfaces onboard_control guided_sim` 通过；
- [ ] `colcon test --packages-select onboard_control` 和项目 Python 测试通过；
- [ ] 关闭/失败清理后无受管进程残留。

### 8.2 无原点 SITL

- [ ] 清空 EEPROM 后启动，`GPS1_TYPE=0`、`SIM_GPS1_TYPE=0`；
- [ ] 没有任何 `set_gp_origin` 发布，AHRS 记录原点恢复 bit 未启用；
- [ ] ExternalNav 输入持续、EKF 相对水平/垂直状态健康，GUI 才开放控制；
- [ ] 在初始 Z 不为 0 的测试中，起飞高度仍等于 `z_start + requested_height`；
- [ ] 起飞参考满足速度/加速度限制并稳定驻留后进入 HOVER；
- [ ] 键盘、悬停和全部局部 ENU 航点不使用经纬度；
- [ ] 取消产生唯一终态；LAND 不依赖 Home/RTL；
- [ ] 断开 ExternalNav 或制造位姿超时会拒绝起飞，飞行中按既定保护进入 LAND；
- [ ] 测试日志中没有把默认 SITL GPS/Home 当作成功依据。

### 8.3 真机台架（禁止自动解锁/起飞）

- [ ] 实际 FCU 参数逐项读取并与备份比较；
- [ ] 确认 extnav 最终消息类型、坐标系、单位、安装偏置、时间戳、频率和协方差；
- [ ] `EstimatorStatus` 的相对水平位置、速度和垂直状态连续健康；
- [ ] `ExtendedState` 在未武装地面正确报告 `ON_GROUND`；
- [ ] 正式“连接实机”只申请租约、配置频率和等待健康状态，没有原点写入；
- [ ] 全程 `armed=false`，没有模式、解锁、起飞、姿态 setpoint 或飞控参数写入副作用；
- [ ] Humble/Jazzy DDS 跨版本告警重新记录，不能沿用旧结论直接宣称可实飞。

## 9. 主要风险和停止条件

遇到以下任一项，应停止进入更高风险阶段并报告，不能通过放宽门限“让测试通过”：

1. 真实 extnav 只提供位置、不提供可靠速度，或坐标轴/时间戳不明确；
2. GPS 关闭且未写原点后，目标 FCU 的 EKF3 不能稳定报告相对位置；
3. 起飞阶段出现 setpoint 空窗、推力阶跃、局部 Z 重置或错误地以绝对 Z 判定完成；
4. 任一 ArduPilot failsafe 仍会进入 RTL/SmartRTL 等 Home 相关模式；
5. LAND 在 ExternalNav 丢失条件下不能可靠执行；
6. 真机参数名/语义与当前本地 ArduPilot 源码不一致；
7. Humble/Jazzy DDS 或 MAVROS 版本导致状态字段/服务不兼容。

综上，下一步应在 `ros2-ardupilot-sitl-hardware` 内实施协议清理和本地起飞状态机，并通过参数配置使用 EKF3 ExternalNav。ArduPilot 源码修改不仅不需要，也会制造额外的安全分叉和维护负担。
