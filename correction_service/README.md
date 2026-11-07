<!-- AprilTag correction geometry, configuration and deployment. -->
# AprilTag–Odin 多 Tag 滑窗修正服务

`correction_service` 2.0 用世界位置已测的不同 AprilTag 建立空间基线，估计
`世界 <- Odin` 的水平 `x / y / yaw` 刚体变换。首次观测提供完整 SE(3) 链得到的粗解；
从第二个不同 Tag 起，每次用服务端滑窗保存的世界系 Tag 中心 `P_i` 与 odom 系 Tag 中心 `Q_i`
重新求完整加权 SE(2)，不累计历史增量，也不拿 corrected odometry 反算。

本模块不修正世界高度，不发送解锁、起飞、模式或轨迹命令。任何 ACK 只说明 extnav
接受了坐标变换，不代表 MAVROS EKF 已稳定，更不构成实飞许可。

Odin→FCU 安装外参的唯一来源是
`config/aircraft/<AIRCRAFT_ID>/odin_extrinsics.yaml` 的 `/**/ros__parameters`。
`pitch_cam` 单位 rad，定义 `R_FI=Ry(pitch_cam)`，把 Odin IMU 轴变换到 FCU 前/左/上轴；
`odin_xyz` 为 FCU→Odin IMU 在 FCU 轴表达的米制位移。host_sdk 在发布层右乘 `T_IF`，
同时转换位姿、速度及协方差。固定安装角补偿只作用于机体关系，不旋转 odom/map 世界系；
`auto_installation` 默认关闭。refresh 的固定仰装25°配置对应 `pitch_cam=-0.4363323129985824`，
`odin_xyz=(0.100,-0.029,0.060)m`。下游直接使用 FCU 中心，不重复补偿安装杆臂。

## 数据链和中心语义

```text
host_sdk 原始估计右乘 T_IF
       |
/odin1/odometry_highfreq (raw，odom←FCU，child=fcu，twist 在 FCU 轴)
       |                          |
       |                          +--> Tag 采样：只读此未经修正的 raw
       +--> extnav 左乘水平修正 C
              |
              +--> /odin1/odometry_highfreq_corrected (FCU 中心)
              +--> /extnav/pose_fcu (最终导航约定)
              +--> /mavros/vision_pose/pose
                           +--> /mavros/local_position/pose (EKF 融合输出)
```

raw 位姿为 `(p,R)`、水平修正为 `(Q,t)`、既有局部原点偏移为 `b=odin_xyz` 时：

```text
correction valid:
  corrected FCU: p' = Q*p+t, R' = Q*R
  final FCU xy:  p'.xy
  final FCU z:   p_raw.z + b.z

correction invalid 或 correction API 缺失:
  final FCU:     p_raw + b, R_raw
```

`b` 是保留原导航数值约定的固定偏移，不随姿态转动；安装杆臂效应已在驱动完成。
即使有效修正为 identity，也走有效分支，水平零点相对无效分支相差 `-b.xy`，
应用跳变门会在同一条 raw FCU 样本上完整计算该差异。Z 保留局部数值约定，
不使用 Tag 世界高度；Tag 的 roll/pitch 不应用。非零固定安装角可正常标定和应用，
服务继续核对 extnav 与共享配置的 `b`，以及 session/revision 和最终实际跳变。

raw、corrected、最终 FCU pose、`valid/session/revision/reference_mode` 在同一个 raw 回调中
形成一致快照。修正改变或失效时先丢弃旧最终快照，等下一条 raw 恢复最终发布。
Tag 服务只接受 `frame_id=odom, child_frame_id=fcu`，旧 IMU 流必须先迁移。
map 导航与 Tag 的组合仍待适配，本次统一参考点不启用该组合。

## 标定模型

### Tag 图案方向与相机外参

`tag_pose.csv` 的 yaw 指向 **AprilRobotics 官方 PNG 的图案上方**，不是 OpenCV
字典生成图片的上方；Tag 系为 `+X上 / +Y左 / +Z朝上`。现场 Tag0 手写“上”的一侧与
[官方 Tag0 原图](https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00000.png)
一致。OpenCV 36h11 的角点零位相对它旋转了 180°：

| 官方纸张角点 | 左上 | 右上 | 右下 | 左下 |
|---|---|---|---|---|
| OpenCV `detectMarkers` 点号 | 2 | 3 | 0 | 1 |

PnP 的标准 Tag 轴为 `+X右 / +Y上 / +Z朝观察者`，因此
`T_OpenCVTag_ConfiguredTag` 的旋转为 `[[0,1,0],[-1,0,0],[0,0,1]]`，右乘
`T_camera_OpenCVTag`。Tag 图案约定由 `geometry.py` 处理，不通过改变相机外参补符号。

相机唯一外参为 `extrinsics.yaml` 的 `T_fcu_camera`：
`p_F=R_FC p_C+t_FC`，C 为右/下/前的光学轴，F 为前/左/上。
当前 refresh 相机在 FCU 前方107 mm、下方15 mm，
`t_FC=(0.107,0,-0.015)m`；画面上方朝机前、右方朝机右、光轴向下，
`R_FC=[[0,-1,0],[-1,0,0],[0,0,-1]]`。这是用户提供的机械安装近似。
其他飞机档案保留原物理相机关系并转换为 FCU 表达；历史相机快照若仍为
`T_imu_camera`，须先迁移，服务会拒绝把旧参考系当作 FCU。
配置变更后重启独立修正服务并重新采样，不能复用旧窗口；服务重启不会自动清除
extnav 已生效的修正。

### 首次与滑窗计算

每个停留段先按当前 `T_fcu_camera`、PnP `T_camera_tag`、原始 `T_odin_fcu` 和 Tag 世界
位姿完成三维链：

```text
T_world_fcu = T_world_tag * inverse(T_camera_tag) * inverse(T_fcu_camera)
T_world_odin = T_world_fcu * inverse(T_odin_fcu)
Q_i^O = translation(T_odin_fcu * T_fcu_camera * T_camera_tag)
```

首次粗解严格沿用 Task27：先计算完整 `T_world_odin`，再直接取该矩阵的 x/y 平移与 yaw；z
平移和 roll/pitch 不应用，tilt 只作诊断/门控。不能用单个 `P_i/Q_i` 将首次平移改写成
`P_i-R(yaw)Q_i`；非零 tilt 下这种单点重锚会把观测高度和被丢弃的倾斜分量灌入坐标系原点
平移。首次的 x/y/yaw 离群筛选和稳健汇总也保持 Task27 语义。

同一批观测另外生成第一个 `P_i/Q_i` keyframe。新增 Q 只接受自身质量门检查，不参与或改写
首次粗修正；从第二个不同且分离的 Tag 起，才按滑窗模型求：

```text
min Σ w_i ||P_i - (R(theta) Q_i + t)||²
w_i = 1 / sigma_i²
```

平移使用同一绝对逆方差权重下的加权质心；旋转限制为 `det(R)=+1`，不拟合尺度或镜像。
二维共线但充分分离的点仍可观 yaw，不错误套用三维满秩条件。每次都从窗口原始 `P/Q`
重算，single-Tag yaw 只参与首次粗解，后续精解不对各 Tag yaw 做平均。

检测器在原始畸变图上定位亚像素角点，并把与处理分辨率匹配的内参 `K` 和标定畸变系数 `D`
直接传给 OpenCV `solvePnP`；这与先对角点执行 `undistortPoints`、再使用 `K` 和零畸变求解等价。
不得先整图去畸变后又重复传入 `D`。

印刷纸张有额外黑色外框时，OpenCV 4.6 默认候选合并距离可能将内部 Tag 丢弃，表现为
静止时仍间歇“未检测到 Tag”。检测器显式设置 `minMarkerDistanceRate=0.02`，保留内部
编码框参与解码，不改变纠错或质量门。`size_m` 必须测量**编码区域最外侧黑边**的边长，
不能测装饰外框或整张纸；配置多个 Tag 不改变图像解码。同帧多个 Tag 仍按现有规则拒绝。
解码后，角点精修半径按 `min(7, floor(最短像素边长 / 16))` 选择（至少1px）：
36h11含黑边共8格，窗口半径不超过半格，避免9.9cm小Tag的固定7px窗口跨入邻格或外框。
检测率、重投影残差与短时稳定性都不能替代独立的公制位置精度验收。

默认窗口长度 5、最大 20、最小长度 2。长度 2 可正常求解和应用，但只能报告“离群识别
能力有限”。窗口满时先用“旧窗口 + 新点”检查整体一致性，再模拟 FIFO；若新点异常，旧窗、
累计次数和 active correction 均保持不变，不能靠先淘汰旧锚点隐藏矛盾。重复 Tag ID、过短
基线、O/W 长度比例异常、残差超限或混合 session/配置/时间源都会拒绝。
窗口时间跨度/间隔超过配置仅记录日志警告，不改变窗口、修正或继续/应用资格。

## 服务端权威状态和事务

服务端在内存维护：

- 独立 `service_instance_id`；进程重启后变化且窗口为空；
- 单调递增 `window_revision`；clear 后不会归零；
- 正式 keyframe FIFO、成功次数 `N` 和下一次序号；
- 保存候选、临时 pending 候选及 application 事实；
- Odin session、extnav revision 和配置 SHA-256 指纹。

GUI 重开只读取这些权威状态，不自计数，也不要求磁盘窗口数据库。Odin session 改变、明确
断流、header 回退或 frame 对变化会使窗口失效；同 session 只有 active revision 改变时保留
原始 Q，但刷新相对 active 的 delta，并要求新的显式确认。候选过期仍阻止应用；
窗口时间跨度/间隔超限仅告警，不清除 extnav active。相机重复/倒序帧直接丢弃，保留当前采样任务。

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

项目 `config/aircraft/<AIRCRAFT_ID>/correction/` 在节点创建 ROS 写接口前严格加载：

- `intrinsics.yaml`：与 `camera.conf` 分辨率对应的相机内参；
- `extrinsics.yaml`：`T_fcu_camera`（必须显式匹配此 transform_name）；
- `tag_pose.csv`：`tag_id,x,y,z,yaw_deg,size_m`；每个 Tag 可有自己的真实边长；
- `camera.conf`：唯一校准相机的稳定设备路径、MJPEG 模式和本包 UVC/V4L2 驱动；
- `lens.conf`：预先保存的下视相机 UVC 参数；收到首帧并等待视频流稳定后，按文件顺序分步写入，
  丢弃切换期帧并最终逐项读回；任务日志保存 requested/readback；
- `general_settings.yaml`：接口、同步、停留段、窗口、跳变、超时和日志门限。
- `../odin_extrinsics.yaml`：与 host_sdk/extnav 共用安装配置，服务从共享参数段读取局部偏移 `b`；由 `fcu_reference.extrinsics_file` 指定并参与配置指纹。

面向 refresh 飞机的活动配置对应 UQ212 小视场镜头：1920×1200 MJPEG、原生 60 fps，
镜头控制与标定时一致。内参采用用户指定的
`uq212_narrow_runs/20261102-171744-403936` 原始30张标定结果，fx/fy=2259.1502/2259.4921px，
RMS=0.8135px，保留全部五个实测畸变系数及原始逐帧误差。10/15/18号样本存在明显异常角点，
独立精度仍待验证；外参沿用机械尺寸与垂直光轴近似，换镜头后的光心位置尚未复核。
修正服务仅使用 UQ212，活动配置位于对应飞机的 `correction/` 目录；
不再保存 Wasintek 配置或读取模块安装目录中的副本。

uav002 的 `correction/UQ212/` 保存换镜头前的完整六文件配置，`correction/UQ212-narrow/`
保存小视场镜头配置。服务只读取 `correction/` 根目录，不自动选择子目录。切换时将所选目录
内的六份文件复制回根目录并重启独立修正服务；子目录保留按根目录解释的相对路径，不直接用作 `config_dir`。

### 采集与发布帧率

`camera.conf` 的 `camera.fps` 请求原生采集模式，refresh 小视场镜头保持 1920×1200 MJPEG
60 fps。采集线程持续读取、归还 V4L2 缓冲区，单槽保存最新压缩帧及其原始采集时间戳，
不受另一线程的解码和发布阻塞。

正式校准唯一节拍是 `general_settings.yaml` 的 `detection.processing_rate_hz`（当前 10 Hz）：
相机调度器按采集单调时间选帧，仅传递选中的帧；检测端不再进行第二次限频。
refresh 使用原始 JPEG 传递，在检测进程内解码并做原分辨率 ROI 搜索；旧 mono8 模式仍支持前置缩图。
原始时间戳、相机内外参和质量门不随调度变化。
`camera.publish_fps` 仅保留旧配置/独立相机工具兼容，正式校准会用上述检测频率覆盖它；
调整校准频率只改 `processing_rate_hz`，重启独立修正服务后生效。

调度跳过错过的节拍，不补发旧图、不生成重复帧，也不改写采集时间戳；源帧不足时按实际可用
帧率输出。`CAMERA_HEALTH` 的 `capture_rate` 为采集线程收到的完整压缩帧频率，`rate` 为
选帧后的发布频率，`rate_limited` 为调度器主动跳过数，与过期/非单调丢帧分别统计。
USB 原始带宽保持原生采集模式，原始里程计仍全频缓存，以真实图像时间匹配历史样本。

校准节点使用单线程 ROS executor 执行默认互斥组回调，图像检测和事务等待仍由独立任务线程
运行。结束、停止或失败时先关闭并等待采集线程，再释放 V4L2 设备及任务订阅。

### 更换下视相机时修改哪里

设备入口是 `config/aircraft/<AIRCRAFT_ID>/correction/camera.conf` 中的 `camera.device`。应优先填写
`/dev/v4l/by-id/...-video-index0`；它比 `/dev/videoN` 稳定。地面站标定面板本身不枚举飞机 USB
设备，而是连接 `correction_service`。服务只在收到 `first/next` 时按该路径打开相机，因此设备
路径正确、服务重启并重新发布状态后，面板即可使用新设备。

- 仅同一颗相机换 USB 口且 by-id 不变：无需改配置；
- 同型号相机但 by-id/序列号变化：只改该机 `correction/camera.conf` 的 `device`，然后重启独立修正服务；
- 更换相机型号或镜头：核验并更新该机 `correction/` 的 `camera.conf`、
  `intrinsics.yaml`、`extrinsics.yaml`、`lens.conf`，然后
  重启独立修正服务。不得只改设备路径后继续沿用未知内外参或不兼容的 V4L2 控制；
- 纯配置变更无需重新编译，但会改变配置指纹；重启后旧窗口/候选不得继续使用。

可在飞机上先执行 `v4l2-ctl --list-devices`、`--list-formats-ext` 和 `--list-ctrls-menus` 核对路径、
模式和镜头控制。面板目前没有远程编辑配置或自动切换命名档案的功能，避免在校准过程中静默换用
另一套几何参数。

关键默认值包括：停留段至少 24 个样本、至少 5 个独立时间块；有效位置 sigma 下限
0.012 m；最小空间基线 1.0 m；O/W 基线长度相对误差最多 15%；窗口 RMS/最大残差分别
0.080/0.150 m；候选最大年龄 300 s；窗口跨度 1800 s、相邻观测间隔 600 s 仅作告警；完整空间修正
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

主 GUI 右上角可打开“标记校准面板”，也可独立运行：

```bash
./.venv/bin/python correction_service/correction_panel.py
```

校准操作与Tag配置在同一页直接展示：宽屏上方左右并排，下方整行显示候选、当前生效修正和FCU位姿对照；
窄屏自动上下排列，依靠纵向滚动访问全部内容。Tag表格随可用空间伸缩。配准、采样、session等信息在“展开诊断与任务详情”中。
Tag 配置区自动读取实际连接的修正服务所用 `tag_pose.csv`，支持增删和编辑
`tag_id,x,y,z,yaw_deg,size_m`，按数值 ID 升序排列。点击“保存并应用”才通过
`/correction_service/configure_tags` 原子写入飞机端文件并更新内存，无需重启；界面显示实际文件路径。
ID 必须唯一非负，坐标/角度必须有限，边长必须为正，至少保留一个 Tag。
表格沿用主GUI航点列表的主题和整行选择，双击编辑。上方单行填写 X、Y、Z、Yaw、Size，
点击“+”将这一组参数追加为最大ID后的新行；输入保留，首次读取时边长默认采用文件中首个Tag的边长。
左下箭头只交换选中行与相邻行的五项参数，ID保持原位升序；减号始终删除末行，“清空”删除全部草稿。
空表需新增Tag后才能保存；服务不可用、请求中或其他禁用状态下，“保存并应用”显示灰色。
活动校准或应用结果未知期间不能保存；配置变化后清空旧几何的窗口、候选和累计次数，
当前 extnav 生效修正保持。重新读取会替换未保存草稿，普通状态刷新不会覆盖编辑。
该入口为独立新增服务，已有飞行/校准协议不变；升级时地面与飞机均须重建
`correction_interfaces` 和 `correction_service`，旧服务仍可校准但配置页会提示缺少接口。

面板使用独立 ROS context 和 `CORRECTION_ROS_DOMAIN_ID`（默认 0）。它显示服务端窗口长度、
N/下一次序号、Tag 顺序、每个 keyframe 的 P/Q/质量、最长 O/W 基线、RMS/最大残差、模型 yaw sigma、
候选/相对 active delta、最近实际跳变、instance/session/revisions、application 状态和日志路径。
四段生产位姿分别标为 raw FCU、corrected FCU、最终导航 FCU 输入和 MAVROS EKF final，
并显示消息年龄；前三段已采用相同机体参考点，但参考系和局部原点约定不同，不能直接逐帧相减。

窗口非空时长度锁定，Tag ID/窗口数值框忽略滚轮。所有服务请求异步发送，状态过期、请求中、
活动采样或 unknown 时按语义禁用冲突按钮。面板的“收敛后确认应用”也先发 dry-run；候选保存
后才展示具体 `C`、粗/精阶段、window/session/revisions、最近跳变和 reset 风险并二次确认，
随后调用 `apply_saved` 重新获取新鲜 raw 复算。关闭面板只关闭地面订阅，不 stop、不 clear、
不重复提交。窗口小于 820 px 时，输入、五个操作按钮以及候选/extnav 状态组会改为窄屏排列；
最小 560×520 窗口依靠纵向滚动承载内容，不需要横向放大才能访问顶部按钮。

## 构建、测试和部署

开发机：

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --packages-select correction_interfaces correction_native correction_service \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
source install/setup.bash
ros2 run correction_service correction_node --ros-args \
  -p config_dir:="$PWD/config/aircraft/uav002/correction"
```

原生数据面位于 `src/correction_native/`：C++ 接收并校验完整频率的 Odin 消息、维护历史和时间匹配，
Python 仅在处理观测或检查应用跳变时读取状态；异常通过 guard condition 交给原状态机。
原生 DDS 发现上下文随服务保留，任务结束后仍实际销毁高频订阅并清空历史；idle 不接收400Hz数据。
滑窗、质量门、服务接口、CAS 应用与未知结果对账仍由 `node.py` 管理。AprilTag 搜索、原有缩图、
自适应亚像素精修及 IPPE square PnP 在同库 C++ 中执行，检测默认 `image_scale` 保持配置值。

采集入口 `correction_service/uvc_camera_node` 用 `exec` 加载原生节点，直接打开 `camera.conf` 指定的系统
UVC 设备，以 V4L2 mmap 持续读取 MJPEG。`detection.roi_enabled=true` 时，相机只把调度选中的原始
JPEG 发布到 `camera_image` 话题加 `/compressed`，不重编码；检测进程在 C++ 中解码一次，完整像素
不经过 Python 或原始图像 ROS 传输。refresh 的 `image_scale=1.0`，内参、角点和 PnP 均使用原分辨率。

首次全图定位目标，之后在上帧目标周围搜索；每侧至少留96像素或半个目标尺寸，并根据角点运动
预测中心、扩展余量。ROI 没有目标或目标靠近人工裁剪边缘时，同一帧立即回退全图；丢失后不复用旧
位姿。轮廓周长门按全图尺寸归一化，角点恢复全图坐标后在完整像素上亚像素精修，PnP与质量门不变。
晃动超出ROI时仍尝试同帧全图找回；回退帧需承担ROI与全图两次搜索，CPU会升高。新采样任务清除跟踪区域。

`roi_enabled=false` 保留原 mono8 链路：只解码选中帧，发布前按 `image_scale` 用 `INTER_AREA` 缩图，
检测不再二次缩图，内参只缩放一次。例如scale=0.5只传960×600。独立相机入口的 `compressed`
参数默认false、`image_scale`默认1，校准任务统一传入配置。采集模式始终与原始标定分辨率一致。
`detection.opencv_threads` 大于0时设置检测进程的OpenCV线程数；0或缺省保持库默认。refresh设为1，
降低并行调度CPU开销；修改分辨率、场景或平台后应同时测量单帧时间与总CPU，保持处理周期内完成。

采集时间直接来自内核 `CLOCK_MONOTONIC` 缓冲区时间戳，按帧龄映射至 ROS 系统时钟；
不按声明 fps 推算时间，不以到达时间掩盖无效时间戳。超过原有 `max_capture_age_ms` 的帧仍丢弃；
JPEG模式在检测侧解码和搜索后再次核对帧龄。任务结束后释放相机和图像订阅。
构建增加 `pybind11-dev` 和 ROS `pybind11_vendor`；安装入口自动补齐。原生模块选择 Python `cv2`
实际加载的 OpenCV 动态库。若 JetPack 开发头与 Python 版本不同，构建入口仅下载并解包匹配的
Ubuntu 头文件到 `build/correction_native/opencv/`，不替换系统 OpenCV。首次构建需要可访问 apt
软件源，后续复用缓存。运行仍使用 Linux V4L2、设备访问权限及已有的 Python/OpenCV/ROS、`v4l2-ctl`，无 Jetson 专属解码
或联合标定工作区依赖。启动脚本只加载 ROS 与本工作区 `local_setup.bash`，避免旧构建记录的外部
underlay 被重新引入。升级时须更新活动及命名档案的 `[driver]` 为 `package = correction_service`、
`executable = uvc_camera_node`，重建本包并重启独立修正服务；旧外部 driver 配置会明确拒绝。
标定配置中 `source_path` 仅记录外参数据或机械推导报告的来源，不在运行时读取那个路径。

节点启动后为 idle。部署与重启按用户已授权的地面维护范围进行；严禁代理自行解锁或起飞实机。

```bash
./correction_service/deploy/install_extnav_correction.sh
./correction_service/deploy/install_correction_service.sh
```

安装后的 unit 位于 `/etc/systemd/system/odin-correction.service`，开机自启状态由
systemd 管理。unit 与人工前台启动共用根目录入口：

```bash
./scripts/onboard/start_onboard_correction.sh
./scripts/onboard/stop_onboard_correction.sh
```

`scripts/onboard/start_onboard_correction.sh` 只在前台启动默认 idle 的修正节点，不启动 Odin、extnav、
MAVROS、onboard_control 或视频。已有修正节点时它会拒绝重复启动。
`scripts/onboard/stop_onboard_correction.sh` 停止 unit 并清理手工残留的修正进程，但不会清除
extnav 内已应用的 active correction，也不停止任何其他机载服务。

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

- Tag 方向以官方原图为准；现场相机固定方式和机械外参变动后必须重新核对。现场 CSV
  由用户维护，部署不覆盖。已有采样/apply 流程通过不代表独立真值精度验收完成。
- `0.1～0.2°` 是目标，不是本次实现或两点拟合残差证明出的实机精度。两点零拟合残差也不能
  识别哪个点有误；建议至少 3 个空间展开点，并用未参与拟合的独立检查点评估。
- 当前 MAVROS 输入是 `PoseStamped`，无法携带 MAVLink estimator reset counter。extnav 只在
  状态中维护内部 counter；应用/失效仍是坐标重置风险，保留 `TODO(task27-reset-counter)`。
- 水平 SE(2) 假设不能掩盖高度相关的外参误差或轨迹漂移；当前放宽门限只用于用户明确要求的
  未武装原理验证，必须保留实际 tilt/跳变数据，不能再放宽门限或修正 z 来美化精度结论。
