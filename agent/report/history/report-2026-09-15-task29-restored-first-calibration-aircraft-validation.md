# Task29 恢复版首次标定实机部署与验证简报

## 任务范围与安全边界

- 目标：把已经恢复的“Task27 首次标定 + Task29 有效分支删除多余 `+T_xy` + 第二个
  keyframe 起才进行多 Tag 配准”部署到新飞机，并在未武装状态下验证下视相机、候选生成、
  apply_saved、extnav、MAVROS 和旧 T 对照链路。
- 飞机：`drone-new`，项目目录
  `/home/nvidia/ros2-ardupilot-mavros-control`。
- 全程只进行地面静态测试；没有解锁、没有起飞、没有发送控制指令。每次校准/应用前均确认
  `armed=false`、`STABILIZE`、`controller_active=false`、`lease_active=false`。
- 本轮没有修改标定算法、门限、接口、onboard_control、Odin 或视频服务源码。

## 部署前发现与处理

飞机部署前的 correction 工作树并不是本地恢复版。关键几何、估计器和节点源码哈希与错误提交
`49f5b32` 一致，仍包含已撤销的首次单点 `P-RQ` 重锚。因此用户此前在飞机上得到的约
`(0.24,0.15)m` 结果，不能视为恢复版 Task27 算法的实测结果。

部署前先确认飞控连接、未武装、STABILIZE、控制器和租约均未激活，然后只停止
`odin-correction.service` 与 `ros2-ardupilot-onboard.service`。创建了精确的远端备份：

```text
/home/nvidia/backups/task29-correction-before-20260915-gv1sQj
```

备份包含部署前的 geometry、estimator、node、correction_panel、ros_client、包级测试和 README，
并带 SHA-256 清单。随后仅同步以下文件，没有覆盖飞机的其他脏工作树内容：

```text
correction_service/README.md
correction_service/correction_service/geometry.py
correction_service/correction_service/estimator.py
correction_service/correction_service/node.py
correction_service/correction_service/correction_panel.py
correction_service/correction_service/ros_client.py
correction_service/test/test_correction_package.py
```

extnav 生产源码部署前已与本地 Task29 正确版本哈希一致，故没有再次覆盖。机上 ARM64 colcon 构建
两个 correction 包成功，随后重启两个服务；onboard_control 在新运行目录
`/tmp/ros2_ardupilot_onboard/20260915-192714` 达到 READY。无显示环境下 Odin launch 自带的 RViz
退出仍是既有现象，不影响 Odin、extnav、MAVROS 或 onboard_control。

## 下视相机与首次校准

### 第一次任务：真实失败且未应用

首次 dry-run job `62d1fbb982af` 使用正确的 USB `1.2` 下视相机、1920×1080@30 MJPEG 和 mono8
话题。相机健康约 30 Hz，无 stale/nonmonotonic 丢帧，但在 20 s 内：

```text
frames_received=379
frames_processed=123
detections=0
samples=0
rejected=123
```

任务如实失败为“20.0s 内未识别到预期 Tag 0”，资源完整释放，窗口仍为空，extnav 保持 identity
revision 0，没有应用任何修正。

任务结束后从同一 `/dev/video2` 直接抓取的帧能看到 Tag 0。该帧虽整体偏亮，本地 OpenCV 4.6 和
飞机 OpenCV 4.14 均可用生产检测器解出 ID 0，重投影误差约 0.23 px。因此不能把第一次失败归因
为固定代码回归、相机选错或单纯曝光；当时被处理的 123 帧未保存，缺少逐帧证据，根因只能标记为
暂时无法复现。

### 第二次任务：同步抓取 ROS 原始帧并成功

第二次 dry-run job `97c6da443e1c` 期间，额外订阅了 correction_service 真正收到的 mono8 话题。
保存的第 1、10、30、60 帧均在飞机上立即识别出 Tag 0，随后任务自动收敛：

```text
accepted=24
rejected=0
independent_blocks=14
span=3.333332 s
position scatter=0.000280 m
effective sigma=0.012247 m
yaw std=0.015674 deg
reprojection RMS=0.264280 px
Odin match error=0.728853 ms (arrival_history)
tilt=4.469751 deg
```

候选严格来自 Task27 完整 SE(3) 首次链，保存为首个 keyframe，未调用多 Tag 配准：

```text
C_world_odin.x=+0.060286803 m
C_world_odin.y=+0.011825798 m
C_world_odin.yaw=-23.168353704 deg
expected FCU jump=0.065356952 m / 23.168353704 deg
candidate stage=rough_single_tag
window=1/5, revision=1
```

用户估计飞机相对 Tag 朝右偏约 30°，实测修正约 -23.17°，方向和粗略量级相符。tilt 低于 10°，
预计 yaw 跳变低于已授权的 45°，所有应用门均通过。

## apply_saved、旧 T 对照和稳态链路

在再次硬性检查未武装、无控制器和无租约后，对同一冻结候选执行 apply_saved。job
`97c6da443e1c-apply` 获得 extnav ACK：

```text
correction_valid=true
revision=1
reset_counter=1
session=odin-3021677342734-02d72bda
final_sample_revision=1
final_sample_correction_valid=true
horizontal_reference_mode=tag_world_xy_local_z
```

3 秒稳态统计结果：

| 数据 | x (m) | y (m) | yaw (deg) | 样本数 |
|---|---:|---:|---:|---:|
| `/extnav/pose_fcu` | 0.018747 | 0.066333 | -23.418919 | 202 |
| `/mavros/local_position/pose` | 0.018110 | 0.067406 | -23.418155 | 106 |

两条链路位置均值相差约 1.25 mm，yaw 均值相差约 0.0008°。同一时段 extnav 的 raw/corrected
内部计数各增加 799 且始终相等，证明应用没有打断输入链。

新面板客户端的旧公式只读对照也在飞机 DDS 域实测：

```text
current pose_fcu = (+0.018649, +0.066580) m
legacy +T pose   = (+0.078649, +0.036580) m
difference       = (+0.060000, -0.030000) m
```

差值严格等于生产杆臂 `T_xy=(+0.06,-0.03)m`。对照值没有 ROS publisher，也没有写入 extnav、
MAVROS 或飞控。

## 当前物理位置一致性审计

本次输出不能判为世界坐标精度合格。用户给出的当前粗略真值是 FCU 中心位于 Tag0 后方约 20 cm、
y 偏差约 5 cm 以内；按 Tag 图案上方为世界 +X，应约为 `x=-0.20m, y≈0`。实际 FCU 输入却约为
`(+0.019,+0.066)m`，表面差异约 22 cm / 7 cm。

对同步保存的 ROS 相机帧独立复算得到：

```text
camera_from_tag translation ≈ (-0.0860,-0.0898,+0.6396) m
world camera position       ≈ (+0.119,+0.026) m
world Odin IMU position     ≈ (+0.061～+0.063,+0.012～+0.015) m
configured FCU->camera xy   = (+0.107803,-0.002361) m (body frame)
```

最后一项由飞控中心到 Odin 的 `(+0.06,-0.03)m` 与 `T_imu_camera` 的相机平移叠加而来。它与当前
PnP 和姿态共同得到实测的 FCU 输出，代数闭环，没有发现额外平移、重复 T 或遗漏去畸变：检测在
原始畸变角点上执行，`solvePnP/projectPoints` 均显式传入缩放后的 K 和原 D。

若把用户的粗略 FCU 真值 `(-0.20,0±0.05)m` 当作准确测量，结合本次 PnP 相机位置和 yaw，反推
实际 FCU→相机水平杆臂应约为：

```text
y=-0.05m: (+0.263,+0.196)m
y= 0.00m: (+0.283,+0.150)m
y=+0.05m: (+0.303,+0.104)m
```

这与配置杆臂相差约 0.22～0.25 m。该证据更指向相机拆装后的外参平移、物理参考点定义或人工
位置估计之一不准确，而不是 Task29 删除 `+T_xy` 的代码错误；旧 `+T` 只会把当前值变为约
`(+0.079,+0.037)m`，也无法解释约 20 cm 的 x 差异。

由于用户位置只是粗略估计，且本轮无人能代替用户精确移动/测量飞机，尚不能在三者中定责。最有
判别力的下一步是在不重启 Odin/extnav、不重新校准的情况下，由用户用铅垂线或固定治具把 FCU
参考中心移动到 Tag0 中心正上方，再同时读取 current、legacy 和 MAVROS；若 current 仍显著不为
零，应实测 FCU/Odin/相机三个参考点并重标 `T_imu_camera`，而不是继续修改首次算法或放宽门限。

## 机上测试与最终状态

- ARM64 部署构建：`correction_interfaces`、`correction_service` 两包成功。
- 机上包级 pytest：4 passed。
- 机上 `colcon test-result --all --verbose`：23 tests、0 errors、0 failures、0 skipped。
- 校准任务结束后 `/dev/video2` 无占用，correction 相机进程不存在。
- `odin-correction.service`：active/running，idle，资源已释放。
- `ros2-ardupilot-onboard.service`：active/running，FCU connected，STABILIZE。
- 飞机：`armed=false`、controller inactive、lease inactive。
- extnav：revision 1 修正保持生效，窗口 1/5 和候选保留，方便用户下一步搬到 Tag 中心复测。

关键任务日志位于飞机：

```text
/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260915-192946-62d1fbb982af.jsonl
/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260915-193451-97c6da443e1c.jsonl
/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260915-193630-97c6da443e1c-apply.jsonl
```

同步抓取的诊断帧保存在本机忽略提交的 `agent/codex/` 目录。响应式 GUI 已在上一轮通过 Qt
offscreen 几何/截图测试；本轮验证了它的 ROS 客户端与旧 T 数据路径，但未在飞机的实际桌面显示器
上进行人工拖拽视觉验收。
