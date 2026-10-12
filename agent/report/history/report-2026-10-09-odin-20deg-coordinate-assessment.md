<!-- 实测 20° Odin 安装的坐标关系评估；refresh 被动检查、数学推导及明确的验证边界。 -->
# Odin 仰角 20 度安装对坐标变换的影响评估

以用户最终实测的 **20°** 为硬件安装角。refresh 的原始 Odin pitch 约 −19.42°，飞控最终 EKF pitch 约 +0.62°，与物理安装关系基本一致。根据完整旋转矩阵，两套世界系的重力方向残差约 0.156°，支持 Odin odom 在当前静止状态下基本与重力对齐。因此，正确相机外参下，Odin–Tag 的完整 SE(3) 链和最终 SE(2) 模型可继续使用；应重点适配的是 Odin 物理 IMU 到飞控机体的安装旋转，以及随之改变的中心和速度换算。

地面站显示近零俯仰来自飞控最终 EKF，不能解释成 Odin 自行把安装倾角归零。当前 extnav 实际仍配置为零安装角，输出姿态没有去掉物理 IMU 的约 20°安装倾角。最终 EKF 显示正常不能证明外部导航输入的坐标关系正确。

本次按要求将 Odin 到下视相机的外参视为后续会正确校准的已知量，不评估该外参的改变量。以下讨论的 Odin 到飞控的安装旋转和杆臂属于另一条链。未修改生产源码或参数，未启停服务、未启动 Tag 相机、未调用修正应用服务、未发布飞行指令。现场为 connected=true、armed=false、STABILIZE。

## refresh 现场证据

2026-10-09 21:38:43 北京时间，在 refresh 上使用项目 `.venv` 有限订阅 6 秒；开发机先前的被动采样得到一致结果。下列位姿话题在采样窗口内各有一个发布源。

| 话题 | 含义 | 最后样本 pitch |
|---|---|---:|
| `/odin1/odometry` | Odin 低频原始 odom→imu | −19.4202° |
| `/odin1/odometry_highfreq` | Odin 高频原始 odom→imu | −19.4210° |
| `/odin1/odometry_highfreq_corrected` | 修正后的 Odin IMU 位姿 | −19.4205° |
| `/extnav/pose_fcu` | 桥接器输出的 FCU 位姿 | −19.4200° |
| `/mavros/vision_pose/pose` | 外部视觉输入 | −19.4200° |
| `/mavros/local_position/pose` | 飞控 EKF 融合后的机体位姿 | +0.6164° |

extnav 回报 correction_valid=false、revision=0、local_identity_origin，安装 roll/pitch/yaw 均为 0，杆臂为 `(0.06,−0.03,0.05)m`；运行命令行也明确给出零安装角。refresh extnav 源码与仓库补丁 SHA-256 相同：`b15b05a3ba61d23e34a9075c3147f6ef2e33afb87081d0fa013c610d230753d1`。

地面站显示链由源码确认：`onboard_control_node.cpp:263` 订阅 `/mavros/local_position/pose`，`:457` 解算 RPY，`:2187` 发布 pitch；`ros_controller.py:108` 接收该字段；`operations_panel.py:1202` 显示俯仰。本次没有检查全部飞控融合参数，因此不对 EKF 为何未跟随视觉输入的倾角作进一步归因。

正在使用的 Odin 配置为 `/home/nvidia/catkin_ws/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml`，与源码副本哈希一致。`send_odom_baselink_tf=0`、`tf_extra_publish_rate=0`、`custom_map_mode=0`。采样没有收到 `odom→imu` 动态变换，仅收到 MAVROS 三条静态轴约定转换：`map→map_ned`、`odom→odom_ned`、`base_link→base_link_frd`。这些静态关系不是物理 IMU 姿态，不能用 RViz 中的“TF 水平”证明 Odin 已补偿安装角。

refresh 驱动 `host_sdk_sample.h:1260` 将设备 `orient` 直接换算成四元数，`:1357` 仅在 TF 开关开启时发布同一姿态的 `odom→imu`；主机这段代码没有将 pitch 归零。[Odin 官方驱动源码](https://github.com/manifoldsdk/odin_ros_driver/blob/main/include/host_sdk_sample.h)

## 坐标系和安装旋转

使用 `H_AB` 表示将 B 系坐标变换到 A 系。W 为水平布设 Tag 的世界系，O 为 Odin 原始 odom，I 为物理 Odin IMU，B 为飞机机体 FLU，C 为下视相机光学系，T 为配置后的 Tag 系。官方资料定义 `odom→imu` 为 IMU 在 odom 中的实时位姿；I 是设备本体系，安装倾斜后不能直接视为飞机 B 系。[Odin 官方坐标系说明](https://github.com/ManifoldTechLtd/wiki/blob/master/docs/odin_series/odin1/5.%20Data%20output_.md)

定义 `S=R_BI`，即 `v_B=S v_I`。若 +X 前、+Y 左、+Z 上，纯抬头安装 20°对应 `S=Ry(−20°)`。杆臂 `l_B` 从飞控中心指向 Odin IMU 中心，以 B 系表达。这一定义用于推导，不意味着现有 `pitch_cam` 可直接填成 −20°。

在物理安装为纯俯仰、飞控 EKF 准确反映重力方向的前提下：

```text
R_map_odom = R_map_B S inverse(R_odom_I)
```

代入静止采样完整 RPY 得到世界 Z 轴夹角约 **0.1561°**。仅看 pitch 的近似残差为 `+0.6164°−20°−(−19.4210°)=+0.0374°`。这支持当前 O 与重力基本对齐，但属于静止一致性检查，没有验收搬动后的轴向、长期漂移或绝对定位。

## Odin 与 Tag 的完整链保持成立

当前 `geometry.py:137` 先完成完整 SE(3)，再提取 x/y/yaw：

```text
H_WI = H_WT inverse(H_CT) inverse(H_IC)
C_full = H_WI inverse(H_OI)
H_OT = H_OI H_IC H_CT
```

若相机外参正确、O 和 W 都与重力对齐，物理安装旋转会同时出现在 Tag 解算的 `H_WI` 和 Odin 输出的 `H_OI` 中，在求 `C_full` 时抵消。仅看旋转即可说明：

```text
R_WI = R_WB S
R_OI = R_OW R_WB S
R_WI inverse(R_OI) = R_WO
```

因此，IMU pitch 为约 −20°并不意味着 Tag 修正 tilt 为 20°。`tilt=acos(C_full.R[2,2])` 测量的是 W 和 O 的世界 Z 轴差异。当前 10°门限不应因为模块安装角为 20°而被放宽。

首次直接提取 `C_full` 的 x/y/yaw、后续使用 Tag 中心 `Q_i^O=translation(H_OT)` 与 `P_i^W` 做加权 SE(2)，在上述条件下均无需因安装角而改写。Tag 图案方向、OpenCV 到配置 Tag 的基变换、PnP 模型也不随 Odin 安装角改变。不能给 Tag yaw 或修正 yaw 固定加减 20°。

本次没有启动 Tag 相机，所以没有测到当前安装的实际 `C_full.tilt` 或候选质量。这里的结论是：**后续正确校准外参后，现有观测链在数学上适用。** 相机内参、Tag 世界测量、同步和已有绝对精度风险仍需分别验证。

## Odin 到飞控的转换需要适配

原来 I 与 B 平行时，直接使用 Odin 旋转作为飞机旋转成立；新安装下应当右乘逆安装旋转：

```text
R_OB = R_OI Sᵀ
p_OB = p_OI − R_OB l_B
```

若水平 Tag 修正为 `(Q,t)`，且 O 已与重力对齐：

```text
R_WB = Q R_OI Sᵀ
p_WB = Q p_OI + t − R_WB l_B
```

安装旋转改变了位姿所代表的机体系，乘在右侧；Tag 修正改变世界系，乘在左侧。两者不能混成同一个“俯仰补偿”。当前 `extnav_to_vision_pose.py:683` 在零安装角时直接透传 Odin 旋转，现场 raw、pose_fcu 和 vision_pose 几乎相同，也确认这项安装旋转没有应用。

**姿态。** 视觉输入当前代表倾斜的 I，却被当成飞机 B 的姿态。最终 EKF pitch 仍接近零不消除这个语义错误。理想重力水平世界中，纯俯仰安装且飞机保持水平时，IMU 和飞机 yaw 可以相同；不能将约 20°俯仰差误说成固定 yaw 偏差。飞机有滚转、俯仰时，两套机体轴的 Euler yaw 可以发生耦合，应使用矩阵转换后提取机体 yaw。

**中心位置。** 当前杆臂在 B 系表达，原来的 `R_OI l_B` 应改为 `R_OI Sᵀ l_B`。即使暂不改变杆臂数值，旋转它所用的姿态也必须适配。保持旧杆臂、机身水平、纯抬头 20°的无误差合成例中，旧算法的中心误差约 `(+0.02072,0,−0.01751)m`。这是隔离旋转影响的数学例，不是现场误差测量，也没有包含机械重装造成的杆臂数值变化。

**速度。** 若原始 linear 属于 O/world、angular 属于物理 I，正确关系为：

```text
ω_B = S ω_I
v_OB = v_OI − R_OB (ω_B × l_B)
v_WB = Q v_OB
```

当前桥接器 `:693` 的叉乘补偿也要与安装角一致。静止时这项误差可能几乎不可见，因此本次零速采样不能验收动态速度换算。设备 twist 的实际坐标约定还需结合人工转动或位姿差分核实。

**局部零点和跳变预测。** 当前无效修正分支使用 `p+T−RT` 保持旧局部零点，z 也保留旧约定。推广到非零安装时，需要用正确机体旋转和初始参考处理这个常量，不能只替换一项然后沿用零安装假设。`geometry.py:187` 的候选 FCU 中心及应用跳变预测必须与桥接器一起更新。

只改 `pitch_cam` 不足以完成适配。当前非零角算法采用共轭 `R_IO R R_IOᵀ`，同时位置并未对应旋转；这不是一般情况下从物理 I 转到 B 的右乘关系。extnav `:536` 明确拒绝非零安装角下应用有效修正，修正服务 `node.py:1444` 也要求安装角为零。因此必须同步推导姿态、中心、速度、跳变预测和支持条件，而非只解除门控。

## 其他坐标运算的影响

| 链路 | 影响与边界 |
|---|---|
| ENU/NED、FLU/FRD | 固定轴约定公式不变；不会自动包含物理 I→B 安装旋转 |
| 机载 PD 与 DOB、轨迹生成 | 当前输入是 MAVROS EKF 位置/速度，算法不直接依赖 Odin 安装角；应修正上游输入。Z 上加 g 的前提仍是重力水平世界 |
| 地面站机体系操纵 | 用最终 EKF yaw 做水平旋转，公式保持；不应额外加入模块俯仰安装角 |
| 航点、上位机位置、RViz 机体预览 | 格式和显示公式保持；外部导航中心错误可能传入最终位置，显示平稳不能验证中心正确 |
| Odin 内部 IMU、LiDAR、相机外参 | 模块整体旋转不改变其内部刚体关系；物理 I 和飞机 B 仍应区分 |
| 注册点云与独立避障 odom | 同一重力水平 O 中的坐标组合可继续使用；若以后改变世界轴，应同步转换点云、位姿、速度和目标 |
| 点云视场 | 模块抬头改变可见障碍覆盖；坐标公式不能恢复未观测区域 |
| Odin map/odom 关系 | 不参与当前 Tag 计算链；Tag 直接使用 raw odometry。RViz fixed frame 水平也不等于 IMU 水平 |

独立避障当前仍以 Odin IMU 为位置参考，盲区球零偏移也以该中心为参考，不能将其自动解释成飞机中心。独立避障尚未接入本工程飞行执行链，本次未修改该仓库。

## 世界系倾斜这一条件性风险

本次实测支持 O 基本水平，不需要仅因模块抬头 20°就对全局位置和点云再旋转 20°。如果后续确认 O 本身倾斜，则应单独建立世界系水平化变换，并同步处理位置、姿态、速度和相关点云。将安装角直接用于左旋原始世界坐标，会混淆这两个问题。

现有 SE(2) 不能补偿世界系 roll/pitch。离线工具另外构造“物理 IMU 初始为零、O 随模块倾斜 20°”的假设：完整 Tag 链得到 tilt=20°，被现有 10°采样门拒绝；若人为绕过该门，丢掉 Q 的 Z 后会形成方向性缩放，二维滑窗可能拒绝长基线，也可能接受有偏候选。例如前向 3 m 基线的 RMS 为 0.09046 m，被拒绝；45°方向的 2 m 基线可通过，却有约 −1.7808°的候选 yaw 偏差。这些是模型边界，**不是当前 refresh 的实测问题**。

## 验证结果和后续范围

离线工具直接调用当前生产几何与滑窗函数，使用精确的合成相机外参；重力对齐情况下 20°物理安装可正确抵消，世界系倾斜的边界例也按当前门限表现。已有几何和配准专项回归使用项目 `.venv` 执行，结果 **6 passed，24 deselected**。这些结果不替代实际 Tag 采样、独立检查点和动态搬动验收。

后续最小适配范围是 extnav 的 I→B 姿态、杆臂位置、杆臂速度，以及修正服务的 FCU 跳变预测和非零安装角支持。Tag 完整三维链、首次投影和 SE(2) 滑窗不因这次 20°安装本身而需要改算法。是否需要重新定义局部 z/初始原点，应在推广中心转换时明确保持或重定义其语义。

本次仅新增报告、`agent/codex` 下的复核工具和只读证据，并更新 MEMORY；生产程序、运行参数和 refresh 服务状态保持检查前状态，未推送或部署。new 未检查该硬件变化。

可复核文件：

- `agent/codex/refresh-odin-mount-readonly-snapshot.json`：refresh 直接订阅的位姿、状态和 TF。
- `agent/codex/odin_mount_readonly_snapshot.json`：开发机收到的同一数据链。
- `agent/codex/refresh-odin-coordinate-source.txt`：refresh 驱动发布逻辑及 extnav 源码哈希摘录。
- `agent/codex/odin_tilt_coordinate_assessment.py` 与同名 `.json`：20°安装及条件性模型边界复核，使用项目 `.venv/bin/python` 运行。
- `agent/codex/odin_mount_readonly_snapshot.py`：有限被动订阅工具；加载 ROS 和项目接口 overlay 后运行，不发消息或调用服务。
