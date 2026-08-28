# 任务 27：AprilTag-Odin 定位修正（refined）执行报告

## 1. 结论

任务要求的独立 AprilTag-Odin `x/y/yaw` 修正链已经实现、部署并在新飞机
`nvidia@192.168.112.169` 上完成未解锁台架验证：

- extnav 始终订阅原始 `/odin1/odometry_highfreq`，无修正时 identity 透传；
- active 修正按左乘 SE(2) 应用于位置、姿态、世界系线速度和有效协方差，`z` 不平移；
- 实际送往 MAVROS 的数据同步发布为
  `/odin1/odometry_highfreq_corrected`；
- correction_service 只在显式任务期间订阅 400 Hz Odin 并打开下视相机，idle 时二者均释放；
- 候选经过 expected Tag、完整 SE(3)、时间历史匹配、样本数、残差、离散度、范围和 tilt
  质量门后，才以 `Odin session + expected revision` CAS 提交；
- 只有 extnav 返回 accepted/applied/new revision 后，`apply=true` 才成功；
- correction_service 失败、人工停止或崩溃不会终止 extnav、MAVROS 或飞控主服务，也不会清除
  extnav 已 ACK 的修正；
- Odin 断流、时间戳回退或 frame 改变会使旧修正失效并回到 identity；
- 地面站新增 detached 修正子面板，入口位于“摄像头配置面板”右侧；旧“在此处打开终端”入口及
  功能已删除；
- `apply=false`、`apply=true`、ACK、服务退出保留修正、显式 clear、失败保底、相机释放、
  raw/corrected/final 对照均已在真机验证；
- 最终飞机状态为 connected、`armed=false`、STABILIZE，修正已清除为 identity，摄像头空闲。

本次没有发送解锁、起飞、模式切换、姿态、速度、位置、航点或其他飞行控制命令。

## 2. 先行全量备份

在读取、修改或部署任务 27 代码前，先创建并校验了完整备份目录：

`/home/nvidia/scq/backups/task27-20260828-225107/`

| 归档 | 大小 | SHA-256 |
|---|---:|---|
| `local-ros2-ardupilot-sitl-hardware-full.tar.zst` | 432,073,883 B | `f0181641ada12ec8f12155fb860f8b710b96790b890ec340f6fb1c5aaf7b396e` |
| `remote-nvidia-192.168.112.169-project-and-runtime-full.tar.gz` | 11,132,075 B | `855f7f6b734210872b51ceef0a66c397fb30af6cb1208327390bb41717fc590a` |
| `remote-nvidia-192.168.112.169-odin-extnav-calibration-full.tar.gz` | 209,399,650 B | `f489b664363edde79ea781aca5cc61a0c7f99ebb94cdea33403f8fc549693fff` |

本地归档包含工作树、`.git`、未跟踪文件以及现有 build/install/log；远端归档覆盖项目源码与运行
配置，以及 Odin、extnav、相机内参、外参与标定输出。三个独立 SHA-256 校验均通过；zstd/gzip
压缩流完整性测试也通过。本地 zstd 归档解压测试大小为 1,321,369,600 B。

在覆盖生产 extnav 源码前，部署脚本又在飞机上创建定点备份：

`/home/nvidia/backups/extnav-task27-20260828-234049/`

- 旧 `extnav_to_vision_pose.py`：
  `9fba48a5a54e73862fc5fe241d96b260585810ef638e6749b73f6267a95bfd5b`
- 旧 `package.xml`：
  `2386f9d3569b70a6d12f3667aab50dff7ca5f0f47dc30ffed4efd71e1ecef38a`
- `SHA256SUMS` 回读验证通过。

飞机项目工作树原本已有接口 3.2 的选择性部署改动，Git HEAD 仍为 `6a40713`；
`/home/nvidia/vrpn_mavros` 也有既有 build/install 状态。全程没有 reset、clean、pull 或覆盖这些
不相关现场改动，只同步本任务新目录和明确目标文件。

## 3. 实现内容

### 3.1 独立接口 `correction_interfaces` 1.0

新增三个消息和三个服务：

- `CorrectionStatus`：任务状态、Tag、候选、质量、时间源、性能、ACK；
- `CorrectionResult`：可靠终态、最终候选、是否 applied、revision、日志路径；
- `ExtnavCorrectionStatus`：Odin session、active correction、revision、内部 reset counter、
  raw/corrected 计数；
- `StartCorrection`：`expected_tag_id + apply`；
- `StopCorrection`：显式停止唯一 job；
- `SetCorrection`：extnav 的 session/revision CAS 应用或显式 clear。

该接口版本独立为 1.0，不修改现有飞行/视频协议 3.2。

### 3.2 `correction_service`

根目录新增独立 ament_python 包，主要模块包括：

- 严格配置加载与刚体矩阵校验；
- OpenCV ArUco `DICT_APRILTAG_36h11` 检测；
- 公制 Tag PnP、完整 SE(3) 坐标链和最终 SE(2) 提取；
- 带 header/本机接收双时间轴的 Odin 历史缓冲；
- 圆统计、MAD 离群剔除、样本数/跨度/标准差/范围/发散质量门；
- 单 job 状态机、start/stop、相机进程组、镜头参数写入与读回；
- extnav CAS 客户端和明确 ACK；
- 模块内轮转日志和逐任务严格 JSONL；
- detached Qt 面板和独立 ROS context 客户端。

idle 节点只保留低频 extnav/status 与相机话题入口，不再长期订阅 400 Hz Odin。start 接受后创建
任务专属 Odin 订阅，先等到新鲜样本，再打开相机；最终 result 发布前先关闭相机、销毁高频订阅
并清空历史。该优化把真机 idle CPU 从约 0.866 核降到 0.018～0.021 核。

### 3.3 时间同步

真机发现两个 header 的 epoch 不一致：

- Odin header 是设备运行时钟，约 34,031 s；
- 相机硬件 PTS 被驱动映射到主机 ROS 时钟，约 1,787,932,xxx s。

因此它们不可能直接按 header 数值相减。最终策略为：

1. 若两个 header 属于同一 epoch，必须在 `max_header_delta_ms=20` 内匹配，否则拒绝；
2. 只有 epoch 明确不兼容时，才在任务期间的历史缓冲中按本机接收时间匹配；
3. 该路径标记为 `arrival_history`，仍受 `max_arrival_delta_ms=30` 限制；
4. 禁止使用识别完成时的最新 odometry，也禁止 mixed/未知时间源提交 apply。

真机成功任务的平均匹配误差为 0.74～0.84 ms。该实现符合“按图像采集时刻匹配历史”，但接收
时间仍包含 DDS/调度延迟，精度边界在“已测约 1 ms、硬门 30 ms”，不能包装成硬件级公共时钟。

### 3.4 extnav 修正与保底

生产 extnav 补丁位于 `correction_service/extnav_patch/`，部署到飞机
`/home/nvidia/vrpn_mavros/src/extnav_bridge/extnav_bridge/extnav_to_vision_pose.py`。

关键行为：

- 原始 Odin 始终是唯一输入；
- 无 valid 修正时 deepcopy identity 透传；
- 有 valid 修正时执行
  `p_world = Rz(yaw) * p_odin + [x,y,0]`；
- 左乘 yaw 四元数，旋转世界系线速度与有效 pose/twist covariance，保持 z 平移和机体系角速度；
- 同一单线程 executor 内更新完整 T/R/valid/revision，数据回调只会看到旧快照或新快照；
- `job_id + session + revision` 支持幂等 ACK 与冲突拒绝；
- correction API 缺失时捕获 ImportError，保持原链 identity；飞控总启动脚本不把该可选接口设为
  硬门，避免重新制造单点故障；
- Odin 超过 2 s 断流、时间戳回退 0.25 s 或 frame pair 改变时立即使旧修正失效；
- correction_service 不在 extnav 的进程/systemd 生命周期中，退出不会清除 active T/R。

### 3.5 真机配置

- 内参来自飞机 `/home/nvidia/camera_calib/camera_calibration.yaml`：1920×1080，
  `fx=1143.4239585813639`、`fy=1144.0407728492289`、
  `cx=960.6748716257138`、`cy=547.8450918370406`，RMS 1.44906 px；
- 外参来自
  `/home/nvidia/vins_odin_calib/output/success01-run_20260827_233838/extrinsic_parameter.csv`，
  配置为严格校验的 `T_imu_camera`；
- Tag 0 配置为 `(0,0,0)`、yaw 0、边长 0.170 m；
- 下视相机使用稳定 by-path，1920×1080@30 MJPEG，`wasintek_gst_camera` 硬件 PTS 节点；
- 镜头参数为 brightness 10、manual exposure mode 1、exposure 25、gain 240、zoom 10；
- 开流后逐项写入并读回，不一致即明确失败。

首次真机任务暴露 `v4l2-ctl` 会返回
`auto_exposure: 1 (Manual Mode)`。旧解析器只接受纯数字，任务按设计失败且相机释放；随后解析器
改为接受数值后的枚举说明，并增加回归测试。

### 3.6 地面站 UI

- 主 GUI 右上角在“摄像头配置面板”右侧增加“Tag-Odin 修正面板”；
- 删除“在此处打开终端”按钮、终端查找/启动函数和不再使用的导入；
- 子面板通过 `QProcess.startDetached` 独立运行，不加入地面站飞行/仿真清理链；
- 默认 `apply=false`，展示 start/stop、expected Tag、候选、质量、性能、时间源、extnav session/
  revision/reset、最近 result，以及 raw/corrected/MAVROS final 三路位姿；
- apply=true 前有人工确认；关闭面板只关闭本地订阅，不 stop job、不 clear correction。

### 3.7 部署

- `install_extnav_correction.sh`：拒绝 active 飞控 unit 或 extnav 进程，先构建接口、备份并校验旧
  extnav，再覆盖和原生构建 extnav；不 start/restart 飞控；
- `install_correction_service.sh`：拒绝共享 overlay 正被飞控使用的更新窗口，构建接口/节点，安装
  独立 `odin-correction.service`；
- correction unit 不含 `Requires=`、`PartOf=` 或 `BindsTo=` 飞控，默认 idle、相机关闭，
  `Restart=on-failure`；
- sparse checkout、地面/机载构建脚本和部署文档已加入 correction 包；
- `start_extnav.sh` 在存在时加载项目 overlay，以发现 correction interface；接口缺失仍由 extnav
  源码 identity 保底。

## 4. 真机台架验证

### 4.1 identity 基线

在未应用修正时观测：

- raw 约 399.21 Hz；
- corrected 约 398.97 Hz；
- MAVROS final 约 40.00 Hz；
- 1,430 对相同 header 的 raw/corrected pose 分量最大绝对差为 0；
- extnav 内部 raw/corrected 计数始终相等。

clear 后再次匹配 1,346 对相同 header 样本，`x/y/z/yaw` 差异全部精确为 0；单线程诊断订阅自身
会在 400 Hz 下丢部分观测样本，因此权威不断流证据采用 extnav 内部相等计数。

### 4.2 apply=false 成功

修复 V4L2 枚举解析后，首轮 dry-run 显式 stop 的终态：

- success=true、applied=false、outcome=`stopped_converged`；
- 366 accepted / 0 rejected；
- `x=-0.014268 m`、`y=-0.258763 m`、`yaw=89.733164°`；
- tilt 5.64745°；
- position std 0.007765 m、yaw std 0.02713°；
- reprojection 0.06753 px、odom match 0.99492 ms；
- 处理约 7.84 Hz / 14.24 ms；
- extnav 保持 revision 0、valid=false，相机释放。

按需高频订阅优化后的最终 dry-run 再次成功：

- job `4cd8873babd5`；
- 171 accepted / 0 rejected；
- `x=-0.012303 m`、`y=-0.271591 m`、`yaw=89.725306°`；
- tilt 6.19582°；
- position std 0.007190 m、yaw std 0.02943°；
- reprojection 0.06948 px、odom match 0.83584 ms；
- 处理 7.76 Hz / 13.97 ms；
- stop 后 raw 订阅从节点图中消失，相机 free，严格 JSONL 解析通过；
- extnav 保持 revision 2 identity。

### 4.3 apply=true、ACK、故障域和 clear

短时 apply=true job `bc2f0c0527ff` 的 ACK 终态：

- 24 accepted / 0 rejected，duration 6.14 s；
- `x=-0.013156 m`、`y=-0.261111 m`、`yaw=89.759921°`；
- tilt 5.70498°；
- position std 0.007026 m、yaw std 0.03157°；
- reprojection 0.06964 px、odom match 0.73945 ms；
- processing 8.12 Hz / 14.89 ms；
- extnav 明确返回 accepted=true、applied=true、revision 1；
- correction status/result 只有收到该 ACK 后才为 success/applied。

应用后短时 raw/corrected 对照中，1,359 对同 header 样本的 yaw 差均为约
`+89.759921°`，z 差为 0；x/y 差随原始位置按左乘旋转变化，符合 SE(2)，不是错误地分别加固定
常数。

随后人工停止 `odin-correction.service`：

- extnav valid/revision/T/R 保持不变；
- 5.5 s 内 raw 与 corrected 计数各增加 2,196，且始终相等；
- 飞控主 unit 保持 active，飞机保持 armed=false；
- 证明 correction_service 退出不影响 extnav 保存的 active 修正或数据链。

该人工 stop 首次暴露 rclpy SIGINT 后二次 `shutdown()` 会让正常退出被 systemd 标为 failed。
入口改为 `rclpy.try_shutdown()` 后，复验结果为 `inactive/dead`、ExecMainStatus 0、Result success；
重新启动仍为 idle，extnav active 修正未丢失。

由于桌面 Tag 未按世界 yaw 0 严格摆正，revision 1 只用于短时链路验证。验证后显式 CAS clear：

- clear accepted=true、applied=false；
- revision 1 → 2、reset counter 1 → 2；
- correction_valid=false，T/R 全零；
- identity raw/corrected 精确恢复；
- 没有把桌面约 90° 坐标修正留在飞机上。

### 4.4 失败保底

已观察两类真实失败：

1. V4L2 枚举读回格式不兼容：任务明确 failed，相机释放，extnav 未改变；随后修复；
2. 一轮性能采样中 20 s 未检测到 Tag：130 帧 rejected，任务明确 failed，相机释放，revision 2
   identity 不变；之后同一最终代码再次识别成功，说明失败路径和恢复路径均成立。

隔离 DDS 集成测试还覆盖：identity、CAS ACK、correction_service 不存在时 active 持续生效、
Odin 2 s gap 后旧修正失效、新 session identity 恢复。没有为此在真机上重启 Odin；真机 Odin
session 失效仍应在后续专门维护窗口补做，而不是为了本任务扩大实机扰动。

### 4.5 性能

Jetson 真机测量：

| 状态 | CPU | cgroup 内存 | 说明 |
|---|---:|---:|---|
| 优化前 idle | 0.8659 核 | 约 225 MB | 原因是长期订阅 400 Hz Odin |
| 优化后首次 idle | 0.0178 核 | 205 MB | 不含 raw 高频订阅 |
| 优化后 job 结束 idle | 0.0209 核 | 241 MB | OpenCV/allocator 保留部分内存 |
| 采样活跃 | 1.136 核 | current 383 MB、peak 392 MB | 1920×1080 相机 + 半分辨率 Python 检测 |

活跃时 `ps` 观测 correction_node RSS 约 383 MB、camera_node RSS 约 450 MB；二者包含共享映射，
不可相加当作 cgroup 独占内存。实际检测吞吐约 7.76～8.12 Hz，低于配置上限 10 Hz。

当前 Python 版本足以完成桌面收敛和架构验证，但不建议在完成长时间飞前资源/调度评审前直接把
它当成最终实时实现。后续若要提高频率、降低单核占用或与其他视觉任务并行，优先考虑 C++
AprilTag/OpenCV、ROI/分辨率优化或独立 CPU 调度；不能只把配置改成更高 Hz 后声称性能提升。

## 5. 本地构建与自动化验证

最终代码完成后重新执行：

- `colcon build`：
  `guided_interfaces correction_interfaces onboard_control guided_sim correction_service` 五包通过；
- 项目正式 `tests/`：198 passed in 47.50 s；
- colcon：22 tests、0 errors、0 failures、0 skipped；
- correction_service 包内 3 项 ament pytest 覆盖配置、SE(2) 和真机兼容助手；
- 隔离 ROS extnav 集成测试通过；
- Qt offscreen 主窗口和修正子面板测试通过；
- Ruff format check 与 E/F/I 检查通过；
- `git diff --check` 通过；
- 所有变更 shell 脚本 `bash -n` 通过；
- 系统没有安装 `shellcheck`，因此没有伪称其通过；部署边界由 Python 测试和真机安装/启动结果
  共同覆盖。

## 6. 已知限制和未完成边界

1. **世界坐标精度未验收。** 当前 Tag 没有严格按 `tag_pose.csv` 的 yaw 0 摆正，约 89.7° 候选
   只证明计算和数据链闭环，不能证明车间世界航向精度，更不能据此承诺 30 m/0.1 m 指标。
2. **没有实飞。** 全程未解锁、未起飞；悬停识别、飞行振动/模糊、EKF 创新、世界航点稳定性均
   未验证。
3. **MAVLink estimator reset 未通知。** 当前 extnav 实际向
   `/mavros/vision_pose/pose` 发布 `geometry_msgs/PoseStamped`，ROS 输入没有
   `VISION_POSITION_ESTIMATE.reset_counter` 字段。源码只维护/发布内部 reset counter，并保留
   `TODO(task27-reset-counter)`；修正跳变目前仍作为普通连续 PoseStamped 进入 MAVROS。若要正确
   通知，需改用能承载 reset counter 的 MAVLink/ROS 边界或修改 MAVROS，属于更大改动。
4. **多 Tag 未实现。** 首版明确拒绝同帧多 Tag，保留 `TODO(task27-multitag)`。
5. **onboard_control 自动航点触发未实现。** 本次只实现地面直接 start/stop/status/result；未来必须
   另设双方 watchdog、相机占用协调和应用后定位稳定判据。
6. **接收时间同步不是硬件公共时钟。** 当前实测误差约 1 ms、硬门 30 ms；若未来相机/Odin 能
   输出统一时基，应切回 header 严格匹配。
7. **实际 Odin session 重建只做了隔离测试。** 真机没有为此额外重启 Odin；后续维护窗口需补测。

## 7. 过程异常与处置

- 一次多源 rsync 把 `node.py`、`synchronizer.py`、`general_settings.yaml` 的副本误放到远端
  `correction_service/` 顶层。发现后只删除这三个本任务刚创建的明确副本，再按正确相对路径同步；
  正确文件、本地/远端 SHA-256 和服务运行均已核对。没有删除用户数据，副本内容可由本地源码或
  全量备份恢复。
- 首次独立服务 stop 被 rclpy 二次 shutdown 标为 failed，已修复并真机复验退出码 0。
- 首次 V4L2 枚举格式失败、一次无 Tag 超时均按真实结果保留在日志和本报告中，没有美化为成功。

## 8. 最终实机状态

截至 2026-08-29 00:12 CST：

- `ros2-ardupilot-onboard.service`：active/running，Result success，NRestarts 0；
- `odin-correction.service`：active/running，Result success，NRestarts 0；
- MAVROS：connected=true、armed=false、mode=STABILIZE；
- extnav：Odin available，correction_valid=false，revision 2，reset counter 2；
- extnav raw/corrected 权威计数相等；
- correction_service：idle，高频 Odin 订阅已销毁；
- 下视相机：free；
- 桌面测试修正已 clear，没有 active 世界坐标偏移留在飞机上。

真机任务 JSONL 位于飞机：

`/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/`

其中最终按需订阅 dry-run 为：

`job-20260829-001105-4cd8873babd5.jsonl`
