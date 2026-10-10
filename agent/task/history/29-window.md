# 多 Tag 跨位置滑窗标定

<!-- 任务草案：2026-09-09 合入飞控中心转换问题；审查后的完整执行规范见 task29-window-refine.md。 -->



## 额外合入任务：修正世界系对齐后的飞控中心转换

当前odin外参处理存在问题，本次任务中一同修正。

2026-09-09 已通过本机源码、真机生产源码、task27 部署前备份和运行参数确认：

- 原始 `/odin1/odometry_highfreq` 表示 Odin IMU 中心；task27 的 corrected highfreq 仍表示该中心，之后 extnav 才执行 Odin 到飞控参考中心的转换。
- 当前安装杆臂 `T=(0.06,-0.03,0.05)m`，定义为飞控中心指向 Odin 中心的机体系向量；extnav 的 `roll_cam/pitch_cam/yaw_cam` 都为 0。
- task27 前后都沿用 `p_fcu = p_odin + T - R_body*T`。`-R_body*T` 是物理中心转换，`+T` 是旧局部坐标的原点偏移；对齐 Tag 世界系后不能继续把二者混为一个安装外参操作。
- 真机检查时 `VISO_POS_X/Y/Z=0`、两组 `INS_POS*=0`；未发现 EKF 再补偿同一杆臂。这是检查时配置，未来执行前仍须只读核实，不能自动写参数。

证明：设 `H_AB` 把 B 系变换到 A 系，`E=H_IF` 为飞控参考系在 Odin IMU 系下的固定位姿（将 F 系坐标变换到 I 系，在位姿链中右乘），
则 `(H_WI*E)*inverse(H_OI*E)=H_WI*inverse(H_OI)`。因此 task27 的世界系修正与该杆臂无关，但依然依赖相机到 Odin 的标定外参。

设原始 Odin 位姿为 `(p,R)`，世界修正为 `(Q,t)`，则正确飞控中心位置为`Q*p+t-Q*R*T`；旧公式得到 `Q*p+t+T-Q*R*T`，多出固定 `T`。水平额外偏移为
`(+0.06,-0.03)m`，模长约 `0.0671m`。另外，旧转换与世界修正交换顺序的差为 `(I-Q)*T`，它与上述绝对世界坐标偏差不是同一指标。

最佳最小改正方法：

1. 保留 task27 校准公式、公共候选、T 数值与 corrected highfreq 的 Odin 中心语义。
2. correction 有效时使用修正后的姿态 `R'`，最终输出水平位置改为`p_fcu_xy=(p_corrected-R'*T)_xy`，去掉额外的 `+T_xy`。
3. z 保持旧公式 `p_fcu_z=p_raw_z+T_z-(R'*T)_z`；不扩大成世界高度校准。
4. correction 无效或 API 缺失时保持旧局部零点约定；位姿、valid 和 revision 必须来自同一输出快照，禁止定时器混用新状态与旧缓存。
5. 不把 T 改为 `Q*T`，不通过修改公共候选来抵消最终输出偏移，不额外引入重复杆臂补偿。
6. 同时验证 MAVROS 输入与 `/extnav/pose_fcu`，并区分 corrected Odin、转换后的飞控输入和`/mavros/local_position/pose` 融合结果。修正切换造成的跳变及 reset counter 风险一并记录。

必须加入非零杆臂、任意原始姿态、不同修正 yaw、零修正但 valid=true、valid 切换与 z 不变的回归测试。下视相机当前已安装，且能直接拍摄到地面上的一个tag0（画面上方为tag上方），另有一个同型号相机用于云台服务，不得作为校准相机使用。



以下为task29本体任务：



## 背景与目标

当前 `correction_service` 使用单个世界位姿已知的 AprilTag，通过同一 Tag 的多帧观测估计`世界 <- Odin` 的 `x / y / yaw` 修正。单 Tag yaw 误差约 `0.5°` 是背景估计，不是当前安装的测量级验收结论；本任务增加不同Tag、不同观察位置之间的空间基线，争取把精度提高到 `0.1～0.2°`。

需要完成后端计算、ROS 接口、状态、日志、测试，并把入口与必要数据展示放入现有`AprilTag-Odin 修正面板`。不能只添加无实际功能的 GUI 按钮。

本任务的“多 Tag”是依次观察 Tag0、Tag1、Tag2……，每个 Tag 内先做多帧稳健汇总，再把结果作为一个空间 keyframe。暂不实现同一帧多个 Tag 联合识别，也不实现 onboard_control 自动触发。



## 核心方法

记 `T_AB` 为把 B 系坐标变换到 A 系，`W/O/I/C` 分别为世界、Odin、IMU、相机坐标系。

看到 Tag `i` 时，使用图像时间严格匹配的原始 Odin 位姿，先按完整 SE(3) 重建 Tag 中心在Odin 坐标系中的位置：

```text
T_OT_i = T_OI_i * T_IC * T_CT_i
Q_i^O  = xy(T_OT_i * [0, 0, 0, 1]^T)
```

`T_IC` 是当前 `T_imu_camera`，`T_CT_i` 是 PnP 得到的 `相机 <- Tag`。必须保留相机杆臂、飞机姿态和相机安装倾斜的影响，不能直接退化成二维相加。

设 Tag `i` 的已测世界中心为 `P_i`，固定 Odin session 内：

```text
P_i = R(theta) * Q_i^O + t + error_i
```

两个 Tag 时：

```text
d_W = P_1 - P_0
d_O = Q_1^O - Q_0^O

theta = wrap(
    atan2(d_W.y, d_W.x)
  - atan2(d_O.y, d_O.x)
)

t = (P_0 + P_1) / 2
  - R(theta) * (Q_0^O + Q_1^O) / 2
```

GUI 可以显示 `delta_theta = wrap(theta - theta_old)`，但后端每次必须使用保存的原始`Q_i^O` 重新求完整 `theta,t`，不能累计修正，也不能用 corrected odometry 作为求解输入。

飞机不需要位于 Tag 正上方：

```text
Q_i^O = p_i^O + r_i^O
d_O = (p_1^O-p_0^O) + (r_1^O-r_0^O)
```

`r_i^O` 是相机观测到的 IMU—Tag 偏移。起飞点不在 `(0,0)`、飞机侧飞/转向、第二次未飞到Tag1 正上方都不破坏算法，但不能省略该偏移，直接把飞机航迹当成 Tag 连线。

多个 keyframe 时使用加权二维刚体配准：

```text
min_(R,t) sum_i w_i ||P_i - (R Q_i^O + t)||^2

C = sum_i w_i dot(Q'_i, P'_i)
S = sum_i w_i cross(Q'_i, P'_i)
theta = atan2(S, C)
t = P_bar - R(theta) Q_bar
```

权重来自每个 keyframe 的位置离散度、重投影误差、同步误差和有效样本数。单 Tag yaw 只作为诊断或弱一致性检查，第 2 次以后不能通过平均单 Tag yaw 得到最终结果。



## 校准流程

1. 设置滑窗长度，最小为 2，默认值和最大值放入配置。
2. 点击“开始首次校准”，沿用现方法得到初始粗修正，同时保存该 Tag 的 `Q_0^O` keyframe。
3. 飞机由用户移动到下一 Tag 附近后，输入 Tag ID，点击“开始第 N 次校准”。
4. 当前 Tag 多帧收敛后，先加入临时窗口并完成配准和质量检查。
5. 结果合格后才原子替换当前窗口；apply=true 时再通过现有 session + revision CAS 提交 extnav。
6. 单次采样结束、失败或停止后释放相机和任务专属 Odin 高频订阅。

窗口长度为 2 时：

```text
第1次成功：[Tag0]
第2次成功：[Tag0, Tag1]
第3次成功：[Tag1, Tag2]
```

“第 N 次”是自上次清空后成功保存 keyframe 的累计序号；失败不增加 N，清空后重新从 1 开始。

新观测退化、离群或发散时，保留此前有效窗口和 active correction，不能先淘汰旧 keyframe 再失败。所有 keyframe 必须属于同一 Odin session；session 改变、时间戳回退、frame 改变或断流时立即清空窗口，禁止跨 session 拼接。



## 校准面板 GUI

在当前面板增加“多 Tag 滑窗标定”区域，保留现有独立 ROS context、窗口样式、extnav 状态和raw/corrected/MAVROS final 对照。

至少包含：

- `预期 Tag ID`；
- `滑窗长度`；
- `开始首次校准`；
- 动态显示的 `开始第 N 次校准`；
- `停止本次采样`；
- `清空滑窗`；
- `收敛后提交 extnav（需要 ACK）` 的 apply/dry-run 选择。

建议增加“应用当前滑窗结果”，用于显式应用 dry-run 得到的合格候选，无需重新采集；仍需检查Odin session、窗口 revision、extnav revision 并弹出确认。

按钮语义：

- 首次校准仅在窗口为空时可用，第 N 次仅在已有 keyframe 且 session 一致时可用；
- 停止只终止当前采样，不清空窗口；
- 清空只删除 correction_service 中的 keyframe、候选和累计 N，不清除 extnav active correction；
- 活动采样时拒绝清空；窗口非空时锁定滑窗长度；
- 状态过期或请求处理中禁用写操作；请求必须异步，不阻塞 Qt 主线程；
- 数值输入框禁止使用鼠标滚轮修改。

至少展示：

- 当前窗口状态、`keyframe 数/窗口长度`、成功次数和下一次 N；
- 窗口 Tag ID 顺序及每个 keyframe 的世界/Odin 坐标和主要质量；
- Odin session、窗口 revision、extnav revision；
- 世界/Odin 基线长度和方向；
- 窗口候选 `x/y/yaw`、相对 active 的 delta；
- 配准 RMS、最大残差、模型 yaw 标准差、可应用状态、失败原因和日志路径。



## 后端、接口与质量

- 滑窗、累计 N 和 window revision 由 correction_service 权威维护，GUI 不得自行计数。
- 扩展 ROS 接口以支持首次/后续校准、窗口长度、expected revision、clear 和窗口状态。
- clear 不得与 extnav maintenance clear 混用；面板重新打开后应能恢复服务端窗口状态。
- extnav 继续只维护最终 active SE(2)，沿用现有 Odin session + revision CAS。
- 若修改 ROS 消息/服务，提升独立 correction 接口版本，并同步包版本、README、构建部署脚本和source/install 接口检查。

`general_settings.yaml` 至少增加：默认/最大窗口长度、最小有效基线、keyframe 样本和离散度、最大观测间隔、配准 RMS/最大残差/yaw 标准差门限、相对 active 的最大允许跳变量。

每个 Tag 停留段先对 `Q_i^O` 做稳健汇总。相同或距离过近的 Tag 不能提供 yaw 基线，应拒绝。
不能编造 Tag1/Tag2 的生产世界坐标；测试坐标只放在测试夹具中。

模型随机误差可参考：

```text
sigma_theta ~= sqrt(2) * sigma_position / baseline
```

该值只能标为模型估算，不能当成实测精度。Tag 世界测量、外参、PnP 系统偏差、同步误差和 Odin漂移不会通过重复采样自动消失。

`apply=false` 完成计算、日志和显示但不调用 extnav；`apply=true` 只有收到 accepted、applied、同一 session 和新 revision 后才报告已应用。任意失败不能覆盖 active correction。

日志需记录 first/next/stop/clear/apply、window/session/revision、keyframe 汇总、`P_i/Q_i^O/weight/residual`、基线、求解结果、门限、窗口淘汰和 extnav ACK，保证可离线复算。



## 测试与验收

至少测试：

- 两 Tag/N Tag 恢复已知 `x/y/yaw`；
- 非零起飞点、飞机不在 Tag 正上方、机体转向、相机杆臂和安装倾角；
- yaw wrap、过短基线、错误 Tag、离群 keyframe 和非有限输入；
- 滑窗淘汰、累计 N、事务失败保留旧窗口、session 改变自动清空；
- apply=false、apply CAS/ACK、stop/clear 语义和故障资源释放；
- GUI 按钮门控、动态 N、数据显示、滚轮禁用、确认框和异步请求；
- correction_service、extnav、面板及全项目相关回归测试。

最终应满足：

- GUI 设置真实作用于服务端滑窗；
- 首次校准同时产生粗修正和第一个 Tag 中心 keyframe；
- 第 2 次以后使用 Tag 世界中心与 Odin Tag 中心做 SE(2) 配准；
- 失败不会破坏旧窗口、active correction 或原始 Odin/extnav 链路；
- session、clear、dry-run、apply 和 revision 语义可验证；
- idle 资源占用不退化；
- 没有测量级真实数据时，不得声称已达到 `0.1～0.2°` 实机精度。

真机可连接，可按需进行机载服务测试，但严禁自行解锁或起飞实机。本任务不发送任何飞行命令。

当前真机下视摄像头已安装，但经过一次拆除和重装，1080p下标定的内参仍然保持不变，但外参可能会与上次标定出的结果有一些差距。需要注意这一点。后续我会将相机固定在碳板，这个外参误差届时将会消除。本次只进行原理验证即可。

下视相机当前直接开启即可拍摄到地面的一个tag0. 相机上方为tag的上方。你可以借此进行一些测试。本次没有多个tag可用。当前飞机处于一个小角度倾斜；tag0可近似认为是水平摆放的。需要注意另有云台相机，不要用错。

当前 `PoseStamped` 无 estimator reset counter 的风险必须保留。完成后更新`correction_service/README.md`，新建详细的任务报告并维护 `MEMORY.md`。

未实现的功能，遇到的无法解决的困难，或本任务描述中存在的歧义/错误等问题，均记录到报告中。

本次任务复杂，请认真反复调试，全力以赴，不得破坏其他功能，不得引入新bug。做最小化改动，不牵扯无关项目内容。
