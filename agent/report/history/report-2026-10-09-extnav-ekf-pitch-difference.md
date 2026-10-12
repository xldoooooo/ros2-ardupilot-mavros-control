<!-- refresh 的只读融合参数检查及 EKF3 外部姿态数据路径说明。 -->
# extnav 与飞控最终俯仰角不同的原因

当前 extnav pitch 约−19.42°而飞控最终 pitch 约+0.62°，原因是 EKF3 的这条外部导航接口只从外部姿态提取 yaw，未将外部 roll/pitch 作为直接姿态观测融合。飞控机体的倾斜主要由自身惯性传感器与重力相关约束估计；机身水平而 Odin 倾斜安装时，两者可以同时成立。

2026-10-09 21:51:29 北京时间，通过 refresh 项目 `.venv` 调用 MAVROS 标准 GetParameters，读取参数缓存；同时被动订阅诊断和状态。未写参数、未切换 EKF 来源、未发布飞行指令、未重启服务。FCU connected=true、armed=false、STABILIZE。

| 参数 | 读回值 | 含义 |
|---|---:|---|
| AHRS_EKF_TYPE | 3 | 选择 EKF3 |
| EK3_ENABLE | 1 | EKF3 启用 |
| EK3_SRC1_POSXY | 6 | 水平位置来源 ExternalNav |
| EK3_SRC1_POSZ | 6 | 高度来源 ExternalNav |
| EK3_SRC1_YAW | 6 | yaw 来源 ExternalNav |
| EK3_SRC1_VELXY / VELZ | 0 / 0 | source1 不选择外部速度来源 |
| VISO_TYPE | 1 | MAVLink 视觉输入 |
| VISO_ORIENT | 0 | 无视觉安装方向配置 |
| VISO_POS_X / Y / Z | 0 / 0 / 0 | 飞控端没有额外视觉杆臂配置 |
| AHRS_ORIENTATION | 0 | 默认飞控安装方向 |

来源参数说明见 [ArduPilot EKF 来源说明](https://ardupilot.org/copter/docs/common-ekf-sources.html)。参数配置不单独证明每一时刻每一个观测成功融合；本次没有读取飞控内部 EKF 融合创新或当前来源切换状态。

源码路径中，`AP_VisualOdom_MAV::handle_pose_estimate` 将位置和四元数送到 `writeExtNavData`。EKF3 的该函数分别保存位置与 yaw：虽然四元数被转换为 roll/pitch/yaw，但只有 `yaw_rad` 写入 `storedExtNavYawAng`，没有相应的外部 roll/pitch 观测缓冲。因此当前差异无需解释成“EKF 把外部20°补偿成0°”或“外部消息完全没有进入飞控”。[EKF3 外部导航入口](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp)

EKF3 的 `InitialiseFilterBootstrap` 从飞控自身加速度向量计算初始 roll/pitch，并单独进行 yaw 对齐；随后依靠 IMU 预测及其他测量约束继续估计。位置、速度观测可通过滤波状态间的关联间接影响姿态，因此“没有直接融合外部 roll/pitch”不等于姿态与其他观测完全无关。[EKF3 姿态初始化](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp)

本次核对了本机 ArduPilot 源码及官方实现，尚未识别现场完整固件版本或校验其二进制；解释针对标准 EKF3 的上述输入路径，并与现场参数及位姿差异一致。

这补充了前次评估的边界：桥接器错误的 pitch 不会在该路径上直接成为飞控最终 pitch。但桥接器已用错误机体旋转计算杆臂位置，误差发生在飞控接收位置之前；飞控不会因为自身 pitch 正常而自动恢复正确中心。倾斜安装下动态 Euler yaw 的轴耦合也需单独处理。

只读参数证据保存在 `agent/codex/fcu-ekf-source-readonly-20261009.json`。生产源码与参数未改变。
