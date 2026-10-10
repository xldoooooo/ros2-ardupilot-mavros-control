# task29：多 Tag 跨位置滑窗标定与飞控参考中心修正

<!-- 完善后的任务规范，2026-09-09；合并 task29 多 Tag 需求与 task27 下游原点偏移修正。仅编写，尚未执行。 -->

## 0. 状态、目标与范围

本文件是 `29-window.md` 经数学和现有实现审查后的完整执行规范。当前用户仅要求编写、审查
任务，**不要据本文立即实现、构建、部署、重启服务或提交真机修正**。后续明确执行本任务时，
再按本文实施；不得顺带实现无关 TODO 或 task28。

目标包含两个必须一起验收的部分：

1. 在现有 `correction_service` 内实现依次观察多个 Tag、跨位置积累 keyframe 的滑窗标定，
   通过 Tag 中心的空间基线求完整 `世界 <- Odin` 水平 SE(2)，完成后端、接口、GUI、日志与测试。
2. 修复 task27 校准后，extnav 的飞控中心转换仍额外保留 `+T_xy` 而造成世界坐标偏移的问题。

单 Tag yaw 误差约 `0.5°` 仅作背景估计；本任务争取达到 `0.1～0.2°`，但必须分别报告模型
估算、合成数据误差、独立实测误差，不能把收敛、残差小或离线演示当作实机精度证据。

只做 x/y/yaw 校准，不校准绝对 z，不做尺度修正、不修改 Odin 内部融合，不做同帧多 Tag
联合识别，不做 onboard_control 自动触发或航点集成。保留现有同帧多 Tag 拒绝策略。

截至本次审查，下视相机被拆除，当前连接的是云台服务相机。不能用它替代下视相机，也不能
复用下视外参给它标定。可基于已有数据和合成场景完成离线验收；实机采集待正确相机恢复后
另行安排。严禁代理自行解锁、起飞或发送飞行命令；未经明确允许不部署或重启飞机服务。

## 1. 已核实基线与代码入口

- 新飞机：`nvidia@192.168.112.169`，工作区 `/home/nvidia/ros2-ardupilot-mavros-control`。
- task27 校准：`correction_service/correction_service/geometry.py`、`node.py`、`estimator.py`、
  `synchronizer.py`，相机外参为 `correction_service/config/extrinsics.yaml` 的 `T_imu_camera`。
- extnav 可审计模板：`correction_service/extnav_patch/extnav_to_vision_pose.py`；真机生产源码
  位于 `/home/nvidia/vrpn_mavros/src/extnav_bridge/extnav_bridge/extnav_to_vision_pose.py`。
- 独立接口：`src/correction_interfaces`；GUI：`correction_panel.py` 与 `ros_client.py`
  （位于 `correction_service/correction_service/`）。当前独立接口版本为 1.0。
- 本次只读检查确认本机与真机的 geometry、extnav 模板/生产源码、相机外参文件内容一致。
- 当前 `T=(0.06,-0.03,0.05)m`，extnav 安装角全为 0；真机 ArduCopter 4.6.3，
  `VISO_POS_X/Y/Z=0`、`INS_POS1/2_X/Y/Z=0`、`VISO_SCALE=1`。
- 检查时 `armed=false`、extnav correction 无效、revision=0。以上是观测快照，不是后续执行
  时的状态保证；连接后应重新只读核实，禁止自动写参数来配合预期结论。

实际数据链为：

```text
raw highfreq（Odin IMU）
    -> extnav 左乘 C（可选）
    -> corrected highfreq（仍为 Odin IMU）
    -> 飞控参考中心转换
    -> /mavros/vision_pose/pose 与 /extnav/pose_fcu
    -> ArduPilot EKF
    -> /mavros/local_position/pose（onboard_control 实际订阅）
```

旧文档把 corrected highfreq 描述为“最终飞控位姿”不准确，实施时同步修正文档与面板标签。
MAVROS 的 ENU/NED、FLU/FRD 转换是坐标表达转换，不能消除额外的原点平移。EKF 输出也不是
输入位姿的逐帧复制，不能用未经时间对齐的两条消息之差直接证明外参精度。

## 2. 坐标、时间与原始观测

记 `H_AB` 把 B 系坐标变换到 A 系；`W/O/I/C/F` 分别为 Tag 世界系、Odin 固定局部系、
Odin IMU、下视相机、飞控参考系。避免把修正矩阵、安装杆臂都简称为 T。

对 Tag i 的第 k 帧，用图像采集时间对应的原始 Odin 位姿计算：

```text
H_OT_i,k = H_OI_k * H_IC * H_CT_i,k
Q_i,k   = xy(H_OT_i,k * [0,0,0,1]^T)
P_i     = 已测量的 Tag i 世界中心的 x/y
```

`H_IC` 为当前 `T_imu_camera`，`H_CT` 为 `相机 <- Tag`。使用完整 SE(3) 后再取 xy，保留
相机平移、安装倾斜、机体 roll/pitch/yaw，不能直接二维相加。OpenCV 标准 Tag 系与项目配置
Tag 系的轴向转换沿用 `configured_tag_from_standard()`；中心不受纯轴旋转影响，但首次
单 Tag 完整姿态校准受影响，不得在首次与后续分支混用约定。

只从 `/odin1/odometry_highfreq` 求解。每帧都先生成 `Q_i,k` 再稳健汇总，不能分别平均欧拉角、
机体位置、PnP 位移后再拼接变换。保存得到的原始 O 系 keyframe，不把 active 修正烘焙进去。

飞机无需位于 Tag 正上方：`Q_i=p_i^O+r_i^O`，其中 `r_i^O` 包含相机杆臂以及 IMU 指向 Tag
的完整观测向量。两 Tag 基线为 `ΔQ=Δp+Δr`，不得只用飞机航迹 `Δp`。

时间匹配必须兼容真实基线：

- 同 epoch 优先严格匹配 header，超出门限拒绝；不能因匹配失败随意回退。
- 当前 Odin 可能使用设备时钟、相机 PTS 使用 ROS 时钟；仅 epoch 不兼容时沿用任务历史内的
  `arrival_history`，当前硬门为 30 ms。禁止匹配识别完成时的最新 Odin。
- 显示并记录时间源、误差、采集跨度与运动量。接收时间匹配不等于精确曝光同步；不能将
  “小于 30 ms”换算成高精度保证。运动引起的误差应进入门控或噪声保守下限。
- 同一 keyframe 不混用时间源；窗口混合来源时必须保守建模并标识，否则拒绝精标应用。
  主机时钟跳变应丢弃受影响的任务/历史，不得仅靠同一个 Odin session 判定时间仍可信。

## 3. 首次粗校准与多 Tag 配准

### 3.1 首次

窗口为空时，以 task27 的完整 Tag 位姿链求初始粗修正：

```text
H_WI = H_WT * inverse(H_CT) * inverse(H_IC)
C_full = H_WI * inverse(H_OI)
C = planar_xy_yaw(C_full)  # z 平移为 0，roll/pitch 不应用
```

同时从同一批合格观测生成首个 `Q_0` keyframe。粗修正与首个 keyframe 都合格才能提交，
不能只保存其中一个并计为首次成功。单 Tag 粗修正与多 Tag 精标使用不同的质量阶段标识和
门限，不能拿精标 yaw 目标强行拒绝所有粗校准。

### 3.2 至少两个不同空间点

同一 Odin session 内拟合：

```text
P_i = R(theta)*Q_i + t + error_i
min sum_i w_i * ||P_i - R(theta)*Q_i - t||^2
P_bar = sum(w_i*P_i)/sum(w_i)
Q_bar = sum(w_i*Q_i)/sum(w_i)
a = sum(w_i * dot(Q_i-Q_bar, P_i-P_bar))
b = sum(w_i * cross(Q_i-Q_bar, P_i-P_bar))
theta = atan2(b,a)
t = P_bar - R(theta)*Q_bar
cross(u,v) = u_x*v_y-u_y*v_x
```

两点时 yaw 等价于 `wrap(angle(P_1-P_0)-angle(Q_1-Q_0))`；等权时 t 是两个点的平均
平移，不等权时必须使用上面的加权质心。只允许 `det(R)=+1` 的旋转，不允许镜像或拟合尺度。

第 2 次起用 Tag 中心配准产生最终 yaw，单 Tag yaw 只用于诊断或宽松一致性检查，不加入
高精度 yaw 平均。每次从窗口保存的 `P_i/Q_i` 重算完整 C，不累加历史修正。

两点不重合时二维 yaw 可观；多个共线但空间分离的点也能求二维 yaw。不得误用三维配准规则，
把“二维点云矩阵必须满秩/必须非共线”作为硬条件。应检查加权空间展开量、有效基线和
`hypot(a,b)`，拒绝重合、过短或对应关系严重不一致的退化数据。

## 4. keyframe、质量与精度边界

每个 Tag 停留段只生成一个 keyframe，至少保存 Tag ID、P、Q、采集起止时间、有效/拒绝帧数、
时间源、重投影和匹配误差、稳健位置散布/不确定度、session、配置指纹和来源 job_id。

配置指纹至少覆盖 Tag 地图、相机内外参和会影响几何的采集配置；改变这些输入后不能复用旧
窗口。Tag 地图必须包含真实测量的坐标和尺寸，不能编造 Tag1/Tag2 生产坐标。首次还依赖
Tag 世界 yaw；后续中心配准不需要以每个 Tag 图案 yaw 作为最终 yaw 观测。

同一停留段连续帧高度相关，不能用总帧数 N 直接获得 `sigma/sqrt(N)` 的无限精度提升。
采用简单可审计的稳健汇总、有效独立样本估计/时间分块和非零误差下限。权重统一由公制位置
不确定度产生，例如 `w_i=1/sigma_i_eff^2`，设置合理上下界；重投影像素、同步毫秒用于门控或
有单位依据的误差传播，不能直接把 px、ms、样本数相乘拼成无解释权重。

`sigma_i_eff` 应保守考虑 O 系观测、Tag 世界测量、外参与同步误差；无法量化的系统项明确
标记未覆盖，不伪造精确协方差。标量加权公式按各向同性近似使用，不冒称已实现完整各向异性
最优估计。yaw 随机误差模型可参考：

```text
等噪声双点：sigma_theta ~= sqrt(2)*sigma_position/baseline  [rad]
各向同性多点：sigma_theta ~= 1/sqrt(sum_i w_i*||Q_i-Q_bar||^2)
```

后式中的 w 必须是绝对逆方差，不能使用任意归一化权重后仍当作角度标准差。不能仅从两点的
拟合残差估计 yaw 不确定度：等长但方向有偏的基线可给出零残差，yaw 仍然有偏。

窗口最小长度仍为 2，建议默认 5、最大 20，写入配置并校验边界。长度 2 保留完整求解和
显式 apply 能力，但标识为“两点解，离群识别能力有限”，不得宣称达到冗余验证。
默认精标建议积累至少 3 个空间分离 keyframe；这只是增加一致性检查机会，3 点也不能保证
识别所有错误对应。长度 2 的目标是可用的最小方案，不能把原需求偷偷改成必须 3 点才能工作。

窗口内重复 Tag ID 不作为新独立基线，拒绝且不增加 N；相距过近的不同 Tag 也拒绝。已淘汰
的 Tag 可再次观测，但不能据此假定多次世界坐标测量误差独立。至少门控：

- 单帧/停留段样本数、跨度、重投影、同步、位置散布、有限值与首次 tilt；
- O/W 两侧有效基线、基线长度差或比例、窗口时间跨度、距当前时间的候选年龄；
- 配准加权 RMS、最大公制残差、模型 yaw 标准差和当前位姿处的预计应用跳变；
- 新点在旧模型下的残差作为诊断/保守门控，不能用过严旧粗模型门限锁死合理的 yaw 改善。

新点异常时保留旧窗口，不能靠自动删掉所有不利点让指标通过。两点只提供一个基线，无法
可靠定位哪一点错；应报告能力限制，要求补充观测，而非宣称已可靠剔除错误 Tag。

固定 session 不代表轨迹无漂移。窗口跨度内若不存在同一个 SE(2)，应明确报模型不一致；
不能偷偷扩成尺度变换、非刚性拟合或持续变化的 C 来掩盖漂移。

水平 SE(2) 还依赖 O/W 重力轴足够一致。完整 SE(3) 中存在 roll/pitch 误差时，简单提取
x/y/yaw 只是受限近似，水平偏差可能随观测高度变化。保留首次和后续观测的完整变换 tilt
诊断及异常门控；旧 `max_correction_tilt_deg=8` 不能被解释为已满足精标精度要求。若跨高度
数据显著违背水平模型，应报告该假设不满足，不以放宽配准残差或修正 z 掩盖问题。

## 5. 修复飞控参考中心的水平原点

### 5.1 与 task27 校准的关系及证明

令 `E=H_IF`。由于 `(H_WI*E)*inverse(H_OI*E)=H_WI*inverse(H_OI)`，世界系校准 C 与
Odin→飞控外参无关；它仍依赖相机→Odin 外参。中心不同本身不要求重算 task27 校准。

当前安装角为 0，T 为飞控中心指向 Odin 中心的机体系固定向量。设原始位姿 `(p,R)`，
修正 `(Q,t)`，则 `p'=Q*p+t`、`R'=Q*R`：

```text
同一世界系内正确的中心转换：p_F = p' - R'*T
旧实现：                    p_old = p' + T - R'*T
额外原点偏移：              p_old - p_F = T
```

旧 `+T` 在 `p=0,R=I` 时使初始飞控输出归零，是旧局部零点约定；它不能继续作为 Tag 世界
原点的一部分。水平偏移为 `(+0.06,-0.03)m`，模长约 6.71 cm。旧处理与 C 交换顺序的差为
`(I-Q)*T`；例如 Q 为 90° 时为 `(0.03,-0.09,0)m`。两种差值不得混称为同一个误差。

### 5.2 指定的最小修复

保留物理 T、公共 C 以及 raw/corrected highfreq；仅调整最终飞控中心转换的原点项：

```text
correction valid：
    p_F.xy = (p_corrected - R_corrected*T).xy
    p_F.z  = p_raw.z + T.z - (R_corrected*T).z
correction invalid 或 correction API 缺失：
    p_F = p_raw + T - R_raw*T                  # 保持旧局部输出约定
```

保持 z 现有数值/约定；水平 yaw 左乘不改变 `(R*T).z`，因此同一 raw 样本上 valid 切换不能
因本次补丁额外改变 z。世界高度仍未校准，不把此输出宣传为完整 Tag 世界 SE(3)。

valid=true 且 C 为单位变换时也必须采用有效分支，不能根据候选数值是否为零判断是否校准。
T 不改为 `Q*T`；不要把候选 t 改成 `t-T_xy` 来隐藏偏移，否则 corrected Odin 的语义错误。
杆臂使用修正后的完整姿态，速度保留当前安装角为 0 时的 `v_F=v_corrected-R_corrected*(ω×T)`。
不能因本次位置常量调整额外引入速度脉冲；C 切换本身作为估计器坐标重置事件处理和记录。

`corrected`、最终姿态/速度、valid、session、revision、中心转换模式必须来自同一 raw 样本的
原子快照。当前定时器读取缓存消息，不得读取旧 corrected 同时套用新 valid 状态；应在生成
该样本时一并冻结转换所需元数据或最终结果。失效后在没有新鲜 raw 时不重复发布旧世界样本。

`/mavros/vision_pose/pose` 与 `/extnav/pose_fcu` 使用同一转换实现；raw 不变，corrected
仍是 Odin IMU 中心。明确记录有效时的水平世界参考与无效时的局部参考，不将状态变化仅靠
含糊的 `frame_id=odom` 隐去，也不发布未经推导的 TF 或擅自重定义 MAVROS map。

本修复的具体简式只对当前已核实的零安装角作保证。固定物理外参可右乘、世界变换可左乘的
一般原则成立，但现有非零 `q_IO*q*q_IO^-1` 与位置混合约定不能因此自动获得正确性保证。
若执行时安装角或物理参考点已改变，先重新推导该分支并报告，不能机械套用简式或改外参值。

## 6. 窗口状态、单次采样与提交语义

服务端权威维护窗口、累计成功 N、window revision 和独立 service instance ID；GUI 不自计数。
window revision 在进程实例内单调递增，clear 后 N 归零但 revision 不归零，防止旧请求误匹配。
进程重启生成新 instance，内存窗口为空；GUI 重开读取服务端状态，不要求新增磁盘窗口数据库。

首次仅在空窗口执行；后续任务需已有同 session 窗口。最小窗口 2 的 FIFO 为：

```text
成功1：[Tag0] -> 成功2：[Tag0,Tag1] -> 成功3：[Tag1,Tag2]
```

在旧窗口副本中加入新点、模拟淘汰后再求解。除目标窗口质量外，保留淘汰前的一致性诊断，
防止一淘汰旧锚点就隐藏矛盾。没有合格临时窗口不得修改正式窗口或 N。

本任务的 first/next 采用“单次收敛自动结束”：dry-run 与 apply 都在合格候选冻结后释放相机
和任务专属 highfreq 订阅，不沿用 task27 dry-run 必须 stop 才结束的隐含生命周期。此变化只
适用于新滑窗操作；如保留旧单 Tag 调试入口，其持续采样语义明确区分，不能静默串用。

执行顺序与结果分开定义：

1. start 校验 instance/window revision、session、配置、Tag ID 与唯一活动操作，返回 job_id。
2. 采样、汇总新 keyframe，构建临时窗口并冻结候选；每个阶段复查 session/配置未变化。
3. 释放采集资源。清理失败不得继续发起 extnav 应用，不允许新任务占用尚未释放的设备。
4. apply=false：再次校验窗口基线，原子保存窗口/候选，N 加一、window revision 加一；
   不调用 extnav，结果为 saved_unapplied。
5. apply=true：保留临时窗口为 pending，先通过 SetCorrection CAS 提交冻结候选；确认与
   本次操作关联的应用事实后才保存正式窗口并增加 N/revision。明确拒绝时保留旧窗口。
6. 结果独立表达“窗口是否保存、修正是否已应用/未应用/未知、资源是否释放”，不把这些状态
   压成一个 success 布尔值。重复响应不得重复增加 N。

**ACK 超时不等于未应用。**请求可能已在 extnav 生效但响应丢失，不能承诺“任何失败都使
active 不变”，也不能为恢复旧窗口自动回滚 active。沿用同一 job_id 和同一候选的幂等重试，
或检查新鲜 extnav 状态中的 applied_job_id、session、revision 和数值进行有限时间对账。
以状态对账确认时报告“经权威状态确认”，不得伪称收到了服务 ACK。

仍无法确定时明确标记 application_unknown，释放相机，保留 pending 诊断，禁止继续覆盖该
不确定状态。对账不需要持续 400 Hz 订阅或新建分布式事务框架；extnav 始终是 active 权威。
若已被别的操作覆盖，仅凭当前不匹配也不能断言本次从未应用。不能刷新 expected_revision
后自动强行覆盖他人的修正。后续明确解除不确定状态也不得偷偷清除 extnav active。

停止语义：冻结/提交前 stop 取消本次并丢弃临时 keyframe，保留正式窗口和 N；已发出应用
请求后 stop 不能撤销已发请求，须按上述对账报告真实结果；提交完成后 stop 不回滚。
clear 仅清空服务端窗口、候选、N，不清除 active。采样、提交或结果未对账期间拒绝 clear。
clear/restart 后已有 active 仍需用于首次应用的跳变检查，不能因窗口为空绕过。

## 7. session、闲置资源与候选重用

滑窗跨采样保存时必须依赖 extnav 权威 Odin session，dry-run 也不例外。否则任务之间相机与
400 Hz 订阅已关闭，无法证明期间没有 Odin 重启。没有可信 session 时可做孤立离线分析，
但不能保存成可继续或可应用的生产窗口。

idle 只保留已有低频 extnav 状态订阅/定时器。Odin session 改变、extnav 明确不可用、frame
改变或超过现有断流/回退阈值时，清空窗口和候选并取消采样，禁止跨 session 拼接。短时状态
过期先标记不可继续/应用；恢复后必须核对 session，不能把网络暂时丢一帧等同 Odin 重启。
同 session 仅 active revision 改变时，原始 Q 仍有效，不必无故清空窗口，但取消旧 expected
revision 的应用，刷新 delta 并要求新的显式提交。

最大单次采样时长、首帧/首 Tag 超时、窗口跨度、相邻观测最大间隔、冻结候选最大年龄应分别
配置并用单调时钟检查。窗口过期不清除 extnav active；标记窗口不可继续并要求清空重采，
不要悄悄淘汰至单点再把精标冒充成粗校准。service instance 或配置指纹变化使旧请求失效。

实现“应用当前滑窗结果”：只应用已保存的合格、未过期候选，不重新开相机、不增加 N。
检查 instance/window revision、session、extnav revision 和新鲜 raw 对应的实际跳变；需要
当前位置时只创建有界短时原始 Odin 订阅，结束即释放。候选 stale、unknown 或未达质量门
时拒绝。重复提交同一候选应幂等，不能产生重复 revision/N。

## 8. delta、跳变与 EKF 边界

GUI 可显示参数差 `wrap(theta_new-theta_old)`、`t_new-t_old`，但平移参数差不等于飞机当前
位置跳变。若需要显示完整相对变换：

```text
C_delta = C_new * inverse(C_old)
Q_delta = Q_new * Q_old^T
t_delta = t_new - Q_delta*t_old
```

在同一个新鲜 raw `(p,R)` 上分别执行旧/新最终飞控转换，求 `Δp_F` 与姿态差作为实际跳变
门控，必须包含 valid/invalid 分支的原点变化。两边都有效时：

```text
Δp_F.xy = ((Q_new-Q_old)*(p-R*T) + t_new-t_old).xy
```

门控在首次、后续及“应用当前结果”都适用；不能只限制 `||t_new-t_old||`。两套 PoseStamped
的发布速率不同，不拿未同步的消息直接代替此计算。原始位姿/active 状态不新鲜时拒绝应用。

当前 `/mavros/vision_pose/pose` 不能携带 extnav 内部 reset counter。按 task27 的既有边界，
本任务不默认扩展成 MAVLink 消息迁移或平滑坐标重建工程，但必须保留 TODO、状态/日志提示
和已应用时的跳变值；“收到 ACK”只证明桥接修正生效，不证明 EKF 已接受、稳定或可实飞。
invalid 回退也会改变坐标含义，不能宣称数据不断流就代表世界航点仍然有效。

## 9. ROS 接口与 GUI

最小扩展现有独立 correction 接口，支持 first/next、window_size、expected instance/window
revision、clear、apply_saved、窗口快照与 pending/application 状态。保留 source_id、sequence、
job_id 和原有 extnav session/revision CAS，不通过 GUI 维护第二套状态机或数学求解器。

破坏性接口变更应升级独立 correction 接口及相关包/配置版本（建议 2.0），更新消息生成、
README、构建/部署与 source/install 版本检查；不因本任务改动无关 guided_interfaces 3.2。
新客户端遇旧服务需明确禁用不支持的写操作；不能把版本不匹配伪装成普通 dry-run 成功。

面板保留原样式和独立 ROS context，至少提供：

- 预期 Tag ID、滑窗长度（空窗可改，非空锁定；数值框禁滚轮）；
- 开始首次校准/动态“开始第 N 次校准”、停止本次采样、清空滑窗；
- apply/dry-run 选择，以及“应用当前滑窗结果”；
- apply 确认明确候选、粗/精阶段、窗口/session/revision、预计位置/yaw 跳变和 EKF reset
  限制；不能为后台自动每帧应用添加确认后即视为长期授权。

状态过期、请求未完成或 application_unknown 时禁用冲突写操作；请求异步，不阻塞 Qt。
按钮从权威状态派生；面板关闭/重开不丢服务端窗口，也不重复自动提交。

至少展示窗口数/长度、N/下一次序号、Tag 顺序、P/Q 与质量、O/W 基线（多点注明展示哪一对，
如最长基线）、RMS/最大残差、模型 yaw 标准差、候选/delta/预计跳变、可应用原因、窗口与
应用状态、session/instance/window/extnav revision、日志路径。

位姿对照区明确分为 raw Odin、corrected Odin、转换后的飞控输入（可订阅现有 pose_fcu）、
MAVROS EKF final；标明不同中心、有效时水平参考系及消息年龄。不要把 corrected 误标为
已转换飞控中心，也不要仅根据文本 frame_id 认定四者可直接相减。

## 10. 配置与可复算日志

`general_settings.yaml` 增加窗口默认/最大长度、keyframe 质量与权重误差下限、有效基线、
基线长度一致性、采样与候选/窗口年龄、RMS/最大残差、粗/精 yaw 门限、实际位置/yaw 跳变
门限等。所有阈值写清单位和作用阶段；不能将演示噪声或目标精度假装成已测硬件参数。

每个 job 日志保存实际几何配置/Tag 地图快照或可还原引用与哈希，记录 first/next/stop/clear/
apply、输入时钟、session/revisions、窗口前后、淘汰/拒绝原因、每帧用于汇总的 Q 与质量、
keyframe P/Q/有效不确定度/weight/residual、解算与门限、最终中心模式和预计跳变、请求与
ACK/对账证据。日志足以重建汇总与配准，不要求为了复算长期记录 400 Hz 全量 Odin或视频。

非有限/未知指标使用明确缺失标识，不输出虚假零误差或非法 JSON NaN。idle 资源不因窗口
存在增长；计算、日志与 GUI 不得阻塞原始 Odin→extnav→MAVROS 输出链。

## 11. 测试与验收

后续执行时使用项目 `.venv`，隔离 ROS 测试使用项目规定的独立 domain/localhost，禁止将
合成 Odin 或 SetCorrection 请求发进真机 domain。按改变运行必要测试，不以 GUI 按钮存在
或服务 ACK 代替端到端行为验收。

### 数学和质量

- 两点等/不等权、多点、共线分离点恢复已知 C；yaw wrap、零基线、镜像错误对应、非有限值。
- 非零起飞位置、非正上方、机体转向/倾斜、真实相机外参和相机轴约定，验证完整链生成 Q。
- 固定 FCU 外参在 C 解算中抵消；公共候选/raw/corrected 不受中心补丁污染。
- 重复/近邻 Tag、不同 Tag 尺寸、离群 keyframe、O/W 基线长度不一致、尺度/时间漂移拒绝。
- 连续重复帧不能虚增独立基线或产生趋零不确定度；两点零残差仍可能 yaw 有偏的反例。
- header/arrival_history、错误 epoch、延迟/乱序、时钟跳变、运动同步误差的门控。

### 最终中心转换

- 当前 T、任意 roll/pitch/yaw、修正 yaw 为 0/±90/180°，证明有效时水平为
  `(Q*p+t-Q*R*T).xy`，旧多余偏移 T_xy 已消除。
- 与物理中心转换的交换关系；单独验证旧公式 `(I-Q)*T` 反例，避免混淆两种误差。
- valid=true 且 C=identity、首次 valid、更新、clear/断流 invalid、API 缺失与 T=0。
- 同一 raw 上 z 与旧算法一致、速度关系一致、两路最终 pose 共用实现；定时器和回调交错
  不混用 pose/valid/revision。验收报告不把 z 的局部约定说成绝对高度准确。

### 状态、接口和集成

- FIFO 淘汰、N、revision/instance 防旧请求、失败保留旧窗；重复 job 不重复记数。
- dry-run 自动收敛保存且不调用 extnav，stop 各阶段、clear 与 active 独立、apply_saved
  不增 N/不开相机、两点解状态和首次粗解独立门限。
- CAS 拒绝、服务不可用、请求未到、已应用但 ACK 丢失、晚到 ACK、状态对账、被第三方覆盖、
  超时未知、应用成功后本地清理/报告失败，真实区分 saved/applied/unknown/cleanup。
- 采样间 idle 时 Odin 重启、断流、frame/session 变化，网络状态暂时过期、候选过期、
  配置变化、service 重启及 GUI 重开；不跨 session 拼接。
- GUI 异步/滚轮/确认/动态 N/按钮门控、旧接口拒绝、四类位姿标签与中心语义。
- correction_service、extnav、GUI、构建部署脚本相关回归；idle CPU/内存与高频订阅释放
  对照，必要时复测输出频率和延迟，保留测量条件。

### 精度与交付

完整交付后端、接口、GUI、配置、日志与测试，不能以无功能入口交差。用已知真值合成数据
与已有可信录包证明实现；精度统计必须区分拟合内残差与未参与配准的独立检查点误差。
真实精度评估需准确测量的 Tag 场地、正确相机、独立真值和多次 session/重复试验，并记录
基线、观察距离/角度、误差分布；不能凭一次两点拟合或模型标准差宣称达到 0.1～0.2°。

当前没有下视相机，实机精度标记为“未验证”，列出恢复条件，不降低验收标准或用云台相机
替代。当前未解决的 reset 语义意味着本任务完成也不自动等于实飞许可。

实施完成时更新 `correction_service/README.md`，修正 MEMORY 中 corrected 与最终飞控位置
混同的旧表述，维护为当前真实基线；新建 `agent/report/report-<日期>-<任务标题>.md`，如实
记录未完成项。过程文件放 `agent/codex/`。本次编写任务文件不提前把这些待实现项写成已实现。

## 12. 本次审查对原稿的主要修正

1. 合入飞控中心 `+T_xy` 原点偏移的证明、最小修复、快照一致性与非零安装角适用边界。
2. 明确 corrected 仍是 Odin 中心；区分桥接输入和 EKF 融合输出，补齐面板比较语义。
3. 纠正“严格图像时间匹配”对双时钟基线的信息缺失，保留现有有界历史匹配及精度限制。
4. 补齐加权质心、非独立帧、绝对逆方差权重和两点可观/不可可靠辨别离群的边界；不把共线
   二维点误判为 yaw 不可观，不强行取消原稿要求的长度 2。
5. 将 apply=true 的正式窗口提交放到应用事实确认之后；修正“任何失败 active 都不变”这一
   不可能覆盖丢 ACK 情形的承诺，补入幂等、对账和 unknown 状态。
6. 明确 dry-run 自动结束、停止与清空区别、保存候选单独应用、N/revision/instance 及
   idle session 监测，避免跨任务误用旧数据或长期保留高频订阅。
7. 用当前飞控中心处的实际跳变代替仅比较平移参数，保留 reset 风险，不擅自扩大实现范围。
8. 补齐当前相机缺席、输入配置变化、候选老化、模型漂移、独立实测精度与相关验收条件。
