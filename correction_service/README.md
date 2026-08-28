# AprilTag-Odin 平面修正服务

本模块用世界位姿已知的 AprilTag 估计 Odin 自建坐标系到固定世界坐标系的
`x / y / yaw` 变换。它不修正 `z`，不会发送解锁、起飞、模式或飞行控制命令。

## 数据链与故障边界

`extnav_bridge` 始终直接订阅原始 `/odin1/odometry_highfreq`。没有有效修正、
接口包缺失、修正服务退出或修正计算异常时，extnav 都按 identity 继续向 MAVROS
传递原始 Odin 数据。有效修正由 extnav 原子保存并应用：

```text
/odin1/odometry_highfreq ──> extnav ──> /mavros/vision_pose/pose
                                  └──> /odin1/odometry_highfreq_corrected
correction_service ── SetCorrection(session + revision CAS) ──┘
```

修正服务只在一个显式任务期间订阅 400 Hz 原始 Odin 并启动下视相机；idle 时同时释放
高频订阅和相机，只保留低频状态接口。候选逐帧更新但不会逐帧写入 extnav；
只有全部质量门通过且 `apply=true` 时才提交冻结值，收到 extnav 的
`accepted=true`、`applied=true` 和新 revision 后才报告成功。失败不改变当前
active correction。Odin 断流超过 2 秒、时间戳回退或 frame 改变时，extnav 立即
清除旧修正、发布 `correction_valid=false`，恢复 identity 数据流。

## 坐标和计算约定

- `tag_pose.csv` 的 Tag 坐标系为 `+X` 指向图案上方、`+Y` 指向图案左方、`+Z`
  离开地面向上；`yaw_deg` 绕世界 `+Z`。
- OpenCV PnP 使用 `+X` 向右、`+Y` 向上、`+Z` 朝观察者；源码显式完成二者转换。
- `extrinsics.yaml` 是 `T_imu_camera`，即把相机坐标中的点变换到 Odin IMU。
- 先组合完整 SE(3) 链并检查非平面 tilt，再从 `T_world_odin` 提取 `x/y/yaw`。
- extnav 应用左乘 SE(2)：`p_world = Rz(yaw) p_odin + [x,y,0]`；姿态、水平速度
  和有效协方差也按同一 yaw 旋转，`z` 不平移。

## 配置

`config/` 中的文件均在节点创建接口前严格校验：

- `intrinsics.yaml`：飞机 `/home/nvidia/camera_calib` 的 1920×1080 内参；
- `extrinsics.yaml`：飞机
  `/home/nvidia/vins_odin_calib/output/success01-run_20260827_233838` 的外参；
- `tag_pose.csv`：`tag_id,x,y,z,yaw_deg,size_m`，首版包含 0 号、边长 0.170 m；
- `general_settings.yaml`：话题、10 Hz 检测、时间匹配和质量/超时门；
- `camera.conf`：下视 Wasintek 的稳定 by-path、1920×1080@30 MJPEG 和硬件 PTS
  相机节点；
- `lens.conf`：与标定一致、开流后写入并逐项读回的 UVC 参数。

当前 `tag_pose.csv` 把 Tag 0 定义为世界原点、yaw 0。只有在 Tag 的实际位置和朝向
已经按该定义精确测量时，`apply=true` 的世界坐标才有物理意义。桌面上随意摆放的
Tag 只适合 `apply=false` 验证识别、同步和离散度。

## ROS 2 接口

接口版本为独立的 `1.0`，不改变飞行控制/视频协议 `3.2`。

| 名称 | 类型 | 含义 |
|---|---|---|
| `/correction_service/start` | `correction_interfaces/srv/StartCorrection` | 指定 `expected_tag_id` 和 `apply`，异步创建唯一任务 |
| `/correction_service/stop` | `correction_interfaces/srv/StopCorrection` | 停止当前 job；不会清除已生效修正 |
| `/correction_service/status` | `CorrectionStatus` | transient-local 实时候选、质量、性能与任务状态 |
| `/correction_service/result` | `CorrectionResult` | transient-local 可靠终态和任务日志路径 |
| `/extnav/set_correction` | `SetCorrection` | session+expected revision 原子提交或维护性清除 |
| `/extnav/correction_status` | `ExtnavCorrectionStatus` | active 修正、session、revision、reset counter 和数据计数 |
| `/odin1/odometry_highfreq_corrected` | `nav_msgs/msg/Odometry` | extnav 此刻实际使用的 raw 或 corrected odometry |

状态主流程为 `idle → starting → sampling → converged → applying → succeeded`；
任一失败进入 `failed`，显式停止/相机清理期间为 `stopping`。`apply=false` 收敛后仍
保持运行和更新候选，直到地面端明确 stop；若 stop 时已有合格候选，result 为成功但
`applied=false`。

命令带每个 `source_id` 单调递增的 sequence，防止重放。同一时刻只允许一个 job。
`apply=true` 还要求新鲜 Odin/extnav session、有界历史时间匹配和 revision ACK。若相机与
Odin header 同 epoch，直接按 header 匹配；当前实机 Odin header 是设备时钟、相机 PTS 是
主机 ROS 时钟，因此明确按图像采集时间匹配 Odin 历史接收时刻（`arrival_history`），绝不
读取识别完成时的“最新 odom”。每个样本仍受 `max_arrival_delta_ms` 限制，混合时间源禁止提交。

## 构建与运行

开发机：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select correction_interfaces correction_service --symlink-install
source install/setup.bash
ros2 run correction_service correction_node --ros-args \
  -p config_dir:="$PWD/correction_service/config"
```

节点启动后处于 idle，相机关闭。地面面板可从主 GUI 右上角“Tag-Odin 修正面板”
打开，也可直接运行：

```bash
./.venv/bin/python correction_service/correction_panel.py
```

面板使用 `CORRECTION_ROS_DOMAIN_ID`（默认 0），直接显示候选/质量、extnav active
状态，以及 raw、corrected、MAVROS final 三路位姿。关闭面板只关闭本地订阅，不会
把 UI 生命周期误当作 stop 或 clear。

真机安装分两步，且必须在已确认安全、未解锁的维护窗口执行：

```bash
# 飞控主服务必须已经停止；安装器会再次拒绝活动状态。
./correction_service/deploy/install_extnav_correction.sh
./correction_service/deploy/install_correction_service.sh
```

第一步在覆盖 `/home/nvidia/vrpn_mavros` 的源文件前创建带 SHA-256 的定点备份，
只构建 extnav，不启动或重启飞控。第二步构建接口/节点并安装独立
`odin-correction.service`；服务常驻 idle，不打开相机，也不依赖飞控 systemd 单元。

常用只读诊断：

```bash
systemctl status odin-correction.service
ros2 topic echo --once /correction_service/status
ros2 topic echo --once /extnav/correction_status
ros2 topic hz /odin1/odometry_highfreq_corrected
```

维护性清除 active correction 需要先读取当前 session/revision，再显式调用：

```bash
ros2 service call /extnav/set_correction correction_interfaces/srv/SetCorrection \
  "{job_id: maintenance-clear, odin_session_id: '<当前session>', expected_revision: <当前revision>, valid: false}"
```

## 日志与已知限制

服务轮转日志和每任务 JSONL/camera 日志位于 `correction_service/log/`。JSONL 包括
Tag ID、候选、样本/离散度、重投影误差、odom 匹配误差、处理性能、ACK 和错误。

- 首版明确拒绝同帧多个 Tag；联合估计保留 `TODO(task27-multitag)`。
- 当前生产链向 MAVROS 发布 `geometry_msgs/PoseStamped`，该 ROS 消息没有 MAVLink
  `VISION_POSITION_ESTIMATE.reset_counter` 字段。因此源码只维护可观测的内部
  reset counter，并保留 `TODO(task27-reset-counter)`；修正跳变目前仍作为普通连续
  PoseStamped 发送，报告中必须据实说明。
- Python/OpenCV 在 Jetson 上按 10 Hz 处理半分辨率图像；是否迁移 C++ 应以实测 CPU、
  延迟和识别稳定性决定。
- correction_service 崩溃不会清除 extnav 已 ACK 的修正；只有 Odin session 失效或
  明确 clear 才会回到 identity。
- 本版不实现 onboard_control 自动触发，也不提供飞机端 bash job start/stop；这些由
  后续航点编排在独立 watchdog 和双方超时语义明确后接入。
