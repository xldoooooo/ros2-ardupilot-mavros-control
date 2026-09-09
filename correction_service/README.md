# AprilTag–Odin 多 Tag 滑窗修正服务

`correction_service` 2.0 用世界位置已测的不同 AprilTag 建立空间基线，估计
`世界 <- Odin` 的水平 `x / y / yaw` 刚体变换。首次观测提供完整 SE(3) 链得到的粗解；
从第二个不同 Tag 起，每次用服务端滑窗保存的原始世界中心 `P_i` 与 Odin 中心 `Q_i`
重新求完整加权 SE(2)，不累计历史增量，也不拿 corrected odometry 反算。

本模块不修正世界高度，不发送解锁、起飞、模式或轨迹命令。任何 ACK 只说明 extnav
接受了坐标变换，不代表 MAVROS EKF 已稳定，更不构成实飞许可。

## 数据链和中心语义

```text
/odin1/odometry_highfreq (raw，Odin IMU 中心 / O 局部系)
       |
       +--> extnav 左乘公共 C
              |
              +--> /odin1/odometry_highfreq_corrected
              |      (corrected，仍是 Odin IMU 中心)
              |
              +--> 同一 raw 回调冻结飞控中心结果
                     +--> /extnav/pose_fcu
                     +--> /mavros/vision_pose/pose (MAVROS 外部视觉输入)
                                  |
                                  +--> /mavros/local_position/pose (EKF 融合输出)
```

当前已核实零安装角，物理杆臂 `T=(0.06,-0.03,0.05)m` 表示飞控中心指向 Odin
中心的机体系向量。原始位姿为 `(p,R)`、水平修正为 `(Q,t)` 时：

```text
correction valid:
  corrected Odin: p' = Q*p+t, R' = Q*R
  FCU center xy:  (p' - R'*T).xy
  FCU center z:   p_raw.z + T.z - (R'*T).z

correction invalid 或 correction API 缺失:
  FCU center:     p_raw + T - R_raw*T
```

有效分支移除了旧实现多出的固定 `+T_xy`；即使 `valid=true` 且 `C` 为 identity，
也必须走有效分支。无效分支保留旧局部零点约定，避免 API 缺失破坏原链路。`z` 仍是旧局部
数值约定，不是已标定的 Tag 世界高度。若 extnav 回报非零安装角或杆臂与配置不一致，服务
拒绝应用，而不是机械套用只对当前零安装角推导的简式。

raw、corrected、最终 FCU pose、`valid/session/revision/reference_mode` 在同一个 raw 回调中
形成一致快照。修正改变或失效时先丢弃旧最终快照，必须等下一条 raw 才恢复最终发布；不会
把新状态套在旧 pose 上，也不会在断流后重复发布旧世界样本。

## 标定模型

每个停留段先按当前 `T_imu_camera`、PnP `T_camera_tag`、Odin `T_odin_imu` 和 Tag 世界
位姿完成三维链：

```text
Q_i^O = translation(T_odin_imu * T_imu_camera * T_camera_tag)
```

首次粗解仍从完整 `T_world_odin` 提取 `x/y/yaw`，同时保存第一个 `P_i/Q_i` keyframe。
窗口至少有两个不同且分离的 Tag 后，求：

```text
min Σ w_i ||P_i - (R(theta) Q_i + t)||²
w_i = 1 / sigma_i²
```

平移使用同一绝对逆方差权重下的加权质心；旋转限制为 `det(R)=+1`，不拟合尺度或镜像。
二维共线但充分分离的点仍可观 yaw，不错误套用三维满秩条件。每次都从窗口原始 `P/Q`
重算，single-Tag yaw 只参与首次粗解，后续精解不对各 Tag yaw 做平均。

默认窗口长度 5、最大 20、最小长度 2。长度 2 可正常求解和应用，但只能报告“离群识别
能力有限”。窗口满时先用“旧窗口 + 新点”检查整体一致性，再模拟 FIFO；若新点异常，旧窗、
累计次数和 active correction 均保持不变，不能靠先淘汰旧锚点隐藏矛盾。重复 Tag ID、过短
基线、O/W 长度比例异常、残差超限、时间跨度/间隔超限或混合 session/配置/时间源都会拒绝。

## 服务端权威状态和事务

服务端在内存维护：

- 独立 `service_instance_id`；进程重启后变化且窗口为空；
- 单调递增 `window_revision`；clear 后不会归零；
- 正式 keyframe FIFO、成功次数 `N` 和下一次序号；
- 保存候选、临时 pending 候选及 application 事实；
- Odin session、extnav revision 和配置 SHA-256 指纹。

GUI 重开只读取这些权威状态，不自计数，也不要求磁盘窗口数据库。Odin session 改变、明确
断流、header 回退或 frame 对变化会使窗口失效；同 session 只有 active revision 改变时保留
原始 Q，但刷新相对 active 的 delta，并要求新的显式确认。窗口或候选过期只阻止继续/应用，
不会清除 extnav active。

`first` 和 `next` 都是一次收敛操作：达到停留段质量门后立即冻结候选、关闭相机、销毁任务
专属 400 Hz raw 订阅，然后才保存或申请 extnav 写入。idle 即使保留窗口，也只保留低频状态
订阅/定时器，不持续打开相机或高频订阅。

操作语义：

- `apply=false`：资源释放后原子保存 keyframe/候选，`N` 与 `window_revision` 各增加一次，
  完全不调用 extnav；
- `apply=true`：先保留临时窗口，通过 Odin session + expected extnav revision CAS 提交；
  只有 ACK 或匹配 `applied_job_id/session/revision/value` 的新鲜权威状态确认后才保存窗口；
- `apply_saved`：不重开相机、不增加 N，只短时订阅 raw 复算当前 FCU 中心实际跳变，随后释放；
- `stop`：应用发出前丢弃本次临时 keyframe；应用发出后不能撤销，只继续 ACK/状态对账；
- `clear_window`：只清窗口、保存候选和 N，不调用 extnav maintenance clear，不改变 active；
- 明确拒绝时保留旧正式窗口；ACK 超时不能冒充“未应用”，对账仍无结论时锁存
  `application_unknown`，只允许同一 job/candidate 幂等重试，禁止刷新 CAS 覆盖第三方 revision。

结果分别表达 `window_saved`、`resources_released`、`application_state` 与
`application_confirmed_by_status`，不会压成一个含糊的 success。extnav 已确认应用、但本地窗口
提交失败时会如实报告 `applied_but_window_commit_failed`，不会伪装成 unknown 或回滚 active。

## 配置和质量门

`config/` 在节点创建 ROS 写接口前严格加载：

- `intrinsics.yaml`：1920×1080 相机内参；
- `extrinsics.yaml`：`T_imu_camera`；
- `tag_pose.csv`：`tag_id,x,y,z,yaw_deg,size_m`；每个 Tag 可有自己的真实边长；
- `camera.conf`：唯一校准相机的稳定设备路径、MJPEG 模式和硬件 PTS 驱动；
- `lens.conf`：开流后写入并逐项读回的 UVC 参数；
- `general_settings.yaml`：接口、同步、停留段、窗口、跳变、超时和日志门限。

关键默认值包括：停留段至少 24 个样本、至少 5 个独立时间块；有效位置 sigma 下限
0.012 m；最小空间基线 1.0 m；O/W 基线长度相对误差最多 15%；窗口 RMS/最大残差分别
0.080/0.150 m；候选最大年龄 300 s；窗口最长 1800 s、相邻观测最长 600 s；完整空间修正
tilt 上限 10°，实际 FCU 中心位置/yaw 跳变上限 1.0 m/45°。其中 tilt 与 yaw 门限于
2026-09-09 为未武装地面原理验证适度放宽，仍不是当前硬件精度测量值或实飞许可。

同一 keyframe 不混用 `header` 和 `arrival_history`。连续帧按时间块估计有效独立信息，位置
不确定度合并 Odin 噪声、Tag 地图、外参和运动×同步误差的非零下限，不能靠提高帧数把 sigma
虚假压到零。首次仍检查 single-Tag yaw 稳定性；后续最终 yaw 使用空间配准的模型 sigma。

## ROS 2 接口 2.0

独立 correction 接口与相关包版本为 `2.0.0`；无关的飞行控制接口仍为 3.2。

| 名称 | 类型 | 含义 |
|---|---|---|
| `/correction_service/start` | `StartCorrection` | `first/next`、Tag、窗口长度、instance/window/extnav revisions 和 apply |
| `/correction_service/stop` | `StopCorrection` | 按 instance/job 停止当前唯一操作 |
| `/correction_service/clear_window` | `ClearWindow` | CAS 清空服务窗口，不清 extnav active |
| `/correction_service/apply_saved` | `ApplySavedCorrection` | 显式应用保存候选或幂等重试 unknown |
| `/correction_service/status` | `CorrectionStatus` | transient-local 任务、窗口、P/Q、候选、质量和应用事实 |
| `/correction_service/result` | `CorrectionResult` | transient-local 可靠终态、独立保存/应用/资源事实和日志路径 |
| `/extnav/set_correction` | `SetCorrection` | Odin session + revision CAS 原子写入/维护性清除 |
| `/extnav/correction_status` | `ExtnavCorrectionStatus` | active、session/revision、杆臂/安装角及最终样本自身的冻结元数据 |

2.0 是破坏性接口变更。客户端看到旧版本时会显示不匹配并禁用写操作；部署脚本同时检查
source 与 install 的 `2.0.0` 清单，以及新消息/服务字段，避免旧 overlay 被静默加载。

## 面板

主 GUI 右上角可打开“AprilTag-Odin 修正面板”，也可独立运行：

```bash
./.venv/bin/python correction_service/correction_panel.py
```

面板使用独立 ROS context 和 `CORRECTION_ROS_DOMAIN_ID`（默认 0）。它显示服务端窗口长度、
N/下一次序号、Tag 顺序、每个 keyframe 的 P/Q/质量、最长 O/W 基线、RMS/最大残差、模型 yaw sigma、
候选/相对 active delta、最近实际跳变、instance/session/revisions、application 状态和日志路径。
四段位姿分别标为 raw Odin、corrected Odin、转换后 FCU 输入和 MAVROS EKF final，并显示消息
年龄；不同中心/参考系不能直接逐帧相减。

窗口非空时长度锁定，Tag ID/窗口数值框忽略滚轮。所有服务请求异步发送，状态过期、请求中、
活动采样或 unknown 时按语义禁用冲突按钮。面板的“收敛后确认应用”也先发 dry-run；候选保存
后才展示具体 `C`、粗/精阶段、window/session/revisions、最近跳变和 reset 风险并二次确认，
随后调用 `apply_saved` 重新获取新鲜 raw 复算。关闭面板只关闭地面订阅，不 stop、不 clear、
不重复提交。

## 构建、测试和部署

开发机：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select correction_interfaces correction_service --symlink-install
source install/setup.bash
ros2 run correction_service correction_node --ros-args \
  -p config_dir:="$PWD/correction_service/config"
```

节点启动后为 idle。生产安装只能在用户确认的未解锁维护窗口进行；安装前应由用户停止可能
冲突的机载服务。本仓库代理不得自行部署、重启、解锁或起飞实机。

```bash
./correction_service/deploy/install_extnav_correction.sh
./correction_service/deploy/install_correction_service.sh
```

extnav 安装器会拒绝运行中的飞控链路，先为指定源文件和 package manifest 创建 SHA-256
定点备份，只构建而不启动/重启服务。独立服务安装器同样拒绝活动飞控服务，构建并核验 2.0
source/install 接口后安装 `odin-correction.service`。它虽可启动独立节点，但节点保持 idle，
不会自动开相机。

常用只读诊断：

```bash
systemctl status odin-correction.service
ros2 topic echo --once /correction_service/status
ros2 topic echo --once /extnav/correction_status
ros2 topic hz /odin1/odometry_highfreq_corrected
```

## 日志与当前边界

轮转服务日志和每任务 JSONL/camera 日志位于 `correction_service/log/`。JSONL 记录实际配置
指纹/几何/Tag 地图、first/next/stop/clear/apply、时间源、session/revisions、窗口前后、逐帧
Q/质量、keyframe 的 P/Q/sigma/weight/residual、淘汰前后配准、门限、实际 FCU 跳变、请求、
ACK/状态对账和资源结果；非有限值写为 JSON `null`，不输出非法 NaN。

- 生产 `tag_pose.csv` 当前只有实测定义的 Tag 0；没有编造 Tag 1/2 坐标。因此真实 next
  操作需先加入经测量的不同 Tag，当前多 Tag 验收只使用测试夹具中的合成真值。
- 2026-09-09 已在飞机确认 USB `1.2` 下视相机可见 Tag 0，并完成未武装 dry-run、一次显式
  apply/ACK 和 identity 清除；相机曾拆装，当前外参仍可能偏离旧标定，云台同型号相机绝不能
  替代。固定碳板并重做外参、布设精测多 Tag、引入独立真值并跨 session 重复试验前，真实精度
  一律记为“未验证”。
- `0.1～0.2°` 是目标，不是本次实现或两点拟合残差证明出的实机精度。两点零拟合残差也不能
  识别哪个点有误；建议至少 3 个空间展开点，并用未参与拟合的独立检查点评估。
- 当前 MAVROS 输入是 `PoseStamped`，无法携带 MAVLink estimator reset counter。extnav 只在
  状态中维护内部 counter；应用/失效仍是坐标重置风险，保留 `TODO(task27-reset-counter)`。
- 水平 SE(2) 假设不能掩盖高度相关的外参误差或轨迹漂移；当前放宽门限只用于用户明确要求的
  未武装原理验证，必须保留实际 tilt/跳变数据，不能再放宽门限或修正 z 来美化精度结论。
