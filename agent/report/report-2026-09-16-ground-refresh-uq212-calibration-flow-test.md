# 地面站到 refresh 的 UQ212 标定流程实机测试

## 结论

本次在飞机静止于桌面、下视相机朝向 Tag 0 的条件下，从地面站真实 Qt 标定面板发起了一次
`first` dry-run。请求成功到达 refresh，但流程在相机首帧阶段失败，尚未进入 AprilTag 检测与
标定计算。

失败原因不是 Tag 摆放、Odin 离线或设备被占用，而是 UQ212 在当前默认曝光下的实际出帧率与
GStreamer 协商帧率不一致，导致现有相机节点产生持续落后的 PTS 并把首批图像全部当作过期帧
丢弃。标定服务的首帧等待时间短于相机节点的时间戳回退时间，因此每次都会先超时。

本次没有修改生产代码或配置，没有应用候选，没有改变 extnav active correction，没有解锁、
起飞或发送任何飞行命令。

## 测试入口与前置状态

- 地面站使用仓库 `.venv`、ROS domain 0 和真实 `CorrectionPanelWindow`/
  `CorrectionPanelClient`；Qt 仅以 offscreen 方式显示，按钮逻辑与正式面板一致。
- 面板前置条件全部满足：correction/extnav 接口均为 2.0、服务可用、Odin 在线且有新鲜样本。
- correction 实例：`313a61545f0f4693bf2800b4d6395575`。
- Odin session：`odin-5197594959732-1a2cbdeb`。
- 标定前 extnav 为 identity：`valid=false`、`revision=0`。
- 标定窗口为空：`window_revision=0`、`window_count=0`。
- 面板“开始首次校准”按钮实际处于 enabled。

## 地面站面板操作

面板输入：

- operation：`first`
- expected Tag ID：`0`
- window size：`5`
- apply：`false`（“应用”未勾选）

点击后服务创建 job `1061820d4c57`，状态依次为：

1. `starting`：正在启动下视相机；
2. `stopping`：首帧等待超时，开始释放采集资源；
3. `failed`：`相机节点启动后未在时限内收到实际图像`。

任务持续 `12.6867 s`，统计为 `frames_received=0`、`frames_processed=0`、
`detections_total=0`、`samples_accepted=0`、`samples_rejected=0`。因此本次结果不能用于判断
Tag 0 解码质量、理论内参质量或沿用外参的正确性。

refresh 上的任务证据：

- `/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260916-225107-1061820d4c57.jsonl`
- `/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260916-225107-1061820d4c57.camera.log`

## 相机链路诊断

### V4L2 设备层

UQ212 by-id 正常指向 `/dev/video0`。使用与活动配置一致的 MJPEG 1920×1080@120 参数执行
V4L2 mmap 采集，连续取得 30 帧，无设备错误；实际帧间隔主要约 `16.0 ms`，即约 60 fps。
这证明相机设备、USB 链路和 MJPEG 模式本身可出图。

### ROS/GStreamer 驱动层

以标定服务完全相同的参数独立启动 `wasintek_gst_camera/camera_node`：

- `/correction_service/image_raw` publisher 正常建立；
- 启动后的首批图像没有发布；
- 约 19.5 秒后日志出现
  `GStreamer PTS could not be mapped; falling back to ROS arrival timestamps`；
- 回退后话题实测约 `58.968 Hz`；
- 首份 health 日志记录 `stale_dropped=1173`。

源码中的行为与实测一致：GStreamer caps 使用 `framerate=120/1`，驱动以 pipeline running time
减 PTS 计算帧龄；帧龄超过活动配置的 `200 ms` 时直接丢弃，只有帧龄超过 `10 s` 才认为 PTS
不可映射并改用 ROS 到达时间。在当前约 60 fps 的实际输出下，PTS 仍按 120 fps 前进，时间戳
每秒约落后 0.5 秒，最终形成“先持续丢帧、约 20 秒后才回退”的确定性窗口。correction_service
的约 12 秒首帧时限在回退前已经结束。

UQ212 的 1080p MJPEG 枚举仍只有 120 fps；本次没有把活动配置改写成未枚举的 60 或 30 fps，
也没有通过放大等待超时来掩盖时间戳问题。

## 安全与终态核对

- 请求和结果均为 `apply=false` / `not_requested`；
- `candidate_saved=false`、`window_saved=false`、`extnav_applied=false`；
- extnav 始终为 `revision=0`、`valid=false`；
- 标定窗口仍为空，不需要执行 clear；
- `resources_released=true`，测试后 UQ212 没有遗留占用进程；
- correction 服务仍在线，保留失败终态供面板显示；Odin/extnav 正常持续运行。

## 未完成项与建议

完整标定流程目前被相机时间戳处理阻断。下一步应修复 UQ212 在“协商 120 fps、实际约 60 fps”
时的 PTS 判定/回退，使节点在服务首帧时限内发布具有可信时间戳的图像，再重新执行同一组 Tag 0
dry-run。不能仅延长 camera start timeout，因为那会保留约 20 秒无图且时间戳错误的根因；也不应
在未经验证时直接 apply 沿用外参生成的候选。
