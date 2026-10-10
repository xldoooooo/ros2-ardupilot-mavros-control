# 任务 29 真机编译部署与下视相机台架简报

- 日期：2026-09-09（Asia/Shanghai）
- 飞机：`nvidia@192.168.112.169`，Jetson Orin NX，Ubuntu 24.04 / ROS 2 Jazzy
- 主要任务依据：`agent/task/29-window.md`
- 补充澄清依据：`agent/task/task29-window-refine.md`
- 测试边界：全程地面、未解锁、未起飞；未调用任何武装、起飞、模式切换或运动控制接口

## 结论

任务 29 的 `correction_interfaces`、`correction_service` 和 extnav 2.0 已在飞机原生编译、选择性
部署并核对运行版本。当前真正朝下且能看到 Tag0 的 Wasintek 相机是 USB `1.2`（`/dev/video2`），
不是原配置的 USB `2`（`/dev/video0`）；生产配置已改为稳定的 by-path 路径。

两轮真实 Tag0 采样均以 `apply=false` 执行并成功形成单点粗候选。两次候选的预计 yaw 跳变分别约
`14.329°` 和 `14.353°`，均超过 `10°` 安全门，`can_apply=false`。没有调用 extnav 写入接口，
extnav 始终为 `correction_valid=false / revision=0`。因此本次只能证明采集、识别、同步、窗口事务、
质量门、安全拒绝和资源释放链可工作，不能证明当前外参、Tag 世界布设或最终世界坐标精度可用。

真机过程中还发现 2.0 状态消息在任务结束后继续累加 `elapsed_s`。代码已改为在终态冻结完成时刻，
新增回归测试，并在第二轮真机任务后以相隔 3 秒的两次状态采样验证数值都精确保持
`53.98436244200275 s`。

## 部署前状态与保护措施

- 飞机项目仍位于 `/home/nvidia/ros2-ardupilot-mavros-control`，Git HEAD 为历史 `6a40713`，且有
  多项既有部署改动和未跟踪文件。本次没有执行 `git pull`、`reset`、`clean` 或全目录覆盖。
- `ros2-ardupilot-onboard.service` 在测试前已经停止并呈既有 `failed/status 130`；
  `odin-correction.service` 运行旧 1.0 且处于 idle；`video-service.service` 未运行。
- 启动飞行链仅用于被动获得 MAVROS/Odin/extnav 数据。每次启动后先核对
  `connected=true, armed=false, controller_active=false, lease_owner=''`，没有发送控制命令。
- 部署前定点备份：`/home/nvidia/backups/task29-predeploy-20260909-2205/`，校验值：

  ```text
  88d296ccc969573dab960f7b1887a1fb623c271bb0893f577e252700f5f328f2  extnav-source.tar.gz
  46e23cbd99d570075f44aaa5d763837707a3073947bb7cdf3b2da8ba8a33a989  odin-correction.service
  7f71743783837045e3502aa4ec05d474ba8853e0b24f32dda068ccfcc6e3fccd  project-correction-source.tar.gz
  ```

- extnav 安装器另生成 `/home/nvidia/backups/extnav-task29-20260909-220308/`；旧脚本与包清单
  SHA-256 分别为 `b6f2fe23a808cc7c31cb697e7db68b28753117ae18474ab9830da6dc0cac24e9` 和
  `ac1bc42b2ed1397ad90ad26a2581a4a619e53aecf8af1d6714240493e6b913de`。

## 下视相机确认

飞机上两台 Wasintek 型号和序列号相同，`/dev/v4l/by-id` 无法唯一表达物理端口，因此必须使用
`/dev/v4l/by-path`：

| USB 拓扑 | 视频节点 | 单帧现场内容 | 用途判断 |
| --- | --- | --- | --- |
| `3610000.usb-2` | `/dev/video0`, `/dev/video1` | 房间/工作台视角，Tag0 只在画面边缘很小区域出现 | 不是本次下视校准相机 |
| `3610000.usb-1.2` | `/dev/video2`, `/dev/video3` | 镜头正对地面，Tag0 清晰且占据主要视野 | 本次下视校准相机 |

最终配置为：

```ini
device = /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0
```

逐路确认只各取一帧，没有启动视频服务。正式任务使用 Jetson MJPEG 硬件流水线
`1920x1080@30 -> mono8`。第二轮相机健康日志稳定在约 `30.00 Hz`，采集年龄均值约
`115.15～118.15 ms`、最大 `157.50 ms`，`stale_dropped=0`、`nonmonotonic_dropped=0`；任务结束后
日志明确记录相机节点停止并释放设备。

## 编译与部署

1. 在确认 FCU `armed=false` 后停止飞行链与旧修正服务。
2. 只同步 `correction_service/` 和 `src/correction_interfaces/`，不触碰飞机既有 guided、onboard、
   video 或 Odin 工作树改动。
3. 运行 `install_extnav_correction.sh`，原生编译 `correction_interfaces` 与 `extnav_bridge`。
4. 运行 `install_correction_service.sh`，原生编译 `correction_interfaces` 与 `correction_service`，
   安装并启动独立 idle unit。
5. 最终源码、安装链接和运行日志均报告接口/包版本 `2.0.0`；三处包清单已核对为 2.0.0。

首次补传计时修复时，两个文件被 rsync 临时放到了 `correction_service/` 顶层，未进入 Python 包；
该问题在哈希核对时立即发现。飞行链当时已停止，随后停止修正服务、将 `node.py` 同步到正确包路径、
删除本次误建的两个精确顶层文件并重新编译启动。最终机载源码哈希为：

```text
b5c8c41b8ac768a3edffbb536ac72cfafec2f54e53b7047f4f32e55bbf673183  config/camera.conf
8c2edf43de550b5985d6ee2eb69952ddf7d5dcca1f23fb968ad389e3e0d521ff  correction_service/node.py
```

## 真机功能测试

### 第一轮 Tag0 dry-run

- job：`324ed1a48347`
- 结果：success，窗口保存，资源释放，`application=not_requested`，extnav revision 仍为 0
- 耗时：`7.432 s`
- 样本：24 accepted / 2 rejected，11 个独立时间块
- `P=(0,0)`，`Q=(-0.085313, 0.040516) m`
- 候选：`x=0.068995 m, y=0.042176 m, yaw=-14.329108°`
- 有效位置标准差：`0.012247 m`；观测 Q 标准差：`0.000189 m`
- yaw 标准差：`0.016095°`；重投影误差：`0.289740 px`
- Odin 匹配误差：`0.822444 ms`，时间源 `arrival_history`
- tilt：`7.884626°`
- 预计 FCU 跳变：位置 `0.088017 m`，yaw `14.329108°`
- 安全结论：`can_apply=false`，原因“当前姿态预计 yaw 跳变超限”

### 第二轮 Tag0 dry-run（部署计时修复后）

- job：`0d7e96fe5256`
- 结果：success，窗口保存，资源释放，`application=not_requested`，extnav revision 仍为 0
- 耗时：`53.984362442 s`，相隔 3 秒复读保持完全相同
- 样本：24 accepted / 375 rejected，23 个独立时间块；采样跨度 `48.700001 s`
- 大量拒绝的直接原因是当前估计 tilt 在 `8°` 门限附近波动，没有放宽门限
- `P=(0,0)`，`Q=(-0.085936, 0.042894) m`
- 候选：`x=0.072386 m, y=0.040729 m, yaw=-14.353351°`
- 有效位置标准差：`0.012247 m`；观测 Q 标准差：`0.000339 m`
- yaw 标准差：`0.016248°`；重投影误差：`0.315893 px`
- Odin 匹配误差：`0.941676 ms`，最大同步运动误差 `0.0000054 m`
- tilt：`7.976455°`；检测处理约 `8.04 Hz`
- 预计 FCU 跳变：位置 `0.087498 m`，yaw `14.353351°`
- 安全结论：`can_apply=false`，原因仍为预计 yaw 跳变超限

两轮保存的单点窗口均在验证后显式清空。clear 只改变 correction_service 的窗口 revision，不改变
extnav active；最终 correction_service 重启为空窗口实例。

### 事务与失败分支

在第一轮窗口上还验证了以下安全拒绝，不启动危险动作：

- `next` 重复 Tag0：拒绝“当前窗口已含相同 Tag ID”。
- `next` Tag1：因生产 `tag_pose.csv` 未配置 Tag1 而拒绝，没有伪造世界坐标。
- `apply_saved` 携带错误 extnav revision 999：同步拒绝，没有创建应用工作线程。
- `clear_window`：成功，window revision 从 1 增至 2，extnav active 明确不变。
- 携带旧 window revision 的新 first：按 CAS 冲突拒绝。

真实多 Tag 配准未测，因为生产配置目前只有经过定义的 Tag0。这是未满足的现场前置条件，不以重复
使用 Tag0 或临时编造 Tag1 坐标替代。

### Odin/extnav/FCU 发布链

未应用修正的 identity 分支中，状态计数保持 raw 与 corrected 相等，且 final sample 与同一
session/revision 一致。约 7 秒窗口测得：

| 话题 | 实测频率 |
| --- | ---: |
| `/odin1/odometry_highfreq` | 约 398～400 Hz |
| `/odin1/odometry_highfreq_corrected` | 约 399 Hz |
| `/extnav/pose_fcu` | 约 100.0 Hz |
| `/mavros/vision_pose/pose` | 稳态约 38.8 Hz |

由于两次真实候选都被安全门拒绝，本次没有在真实 FCU 域写入 valid 修正，因此不能声称真实 valid
分支或飞控 EKF 对该修正的响应已验证。相关变换公式仍以任务 29 的合成真值和 localhost 隔离 ROS
测试为证据。

## 自动化验证

- 本机项目环境：`229 passed in 83.92s`（正式 `tests/` 范围）。
- 本次修改文件 Ruff：通过。
- 本机专项：修正服务、部署与状态事务共 27 项通过。
- 飞机 aarch64：`colcon test --packages-select correction_interfaces correction_service` 后
  `22 tests, 0 errors, 0 failures, 0 skipped`；包内 Python 3 项通过。
- 全仓 Ruff 仍报告 24 个既有问题，集中在本次未修改的 GUI/旧测试导入顺序、过期 noqa 等；未把
  无关格式修复混入本次提交。
- 从仓库根直接执行无范围 `pytest` 会误收集独立 `integration/websocket_test_demo`，其未安装
  `ws_demo` 而收集失败；正式项目测试按既有约定限定为 `tests/`，结果如上。

## 最终飞机状态

- `odin-correction.service`：`active / enabled`，2.0，idle。
- correction window：空，N=0；无保存候选；`resources_released=true`。
- correction_service 仅订阅低频 `/extnav/correction_status`，没有图像或任务 raw 订阅。
- `/dev/video0` 与 `/dev/video2` 均无人占用；无 correction camera、FFmpeg 或 GStreamer 采集进程。
- `ros2-ardupilot-onboard.service`：已恢复为测试前的停止状态；既有停止脚本使 unit 显示
  `failed/status 130`，但 MAVROS、Odin、extnav 和 onboard 进程均已退出。
- `video-service.service`：inactive，未因本任务启动。
- 测试期间所有 FCU 快照均为 `armed=false`；最后一次在线快照为 connected、STABILIZE、
  controller inactive、无租约、无 setpoint 冲突。

## 未解决风险与下一步人工前置

1. 当前相机/Tag 组合实测 tilt 约 `7.9°`，紧贴 8° 门限；第二轮 375 个拒绝样本说明安装几何并不
   具备充足余量。应先固定相机并重新做外参标定，不能靠放宽门限获得“成功”。
2. 两轮候选 yaw 都约 `-14.3°`，可能来自相机拆装后的外参偏移、Tag 实际朝向或二者共同作用。
   必须精测 Tag0 的世界朝向并从零重复外参校准。
3. 生产表只有 Tag0，尚不能验证任务 29 的真实双点/多点加权 SE(2)、FIFO 淘汰或离群识别。
4. 没有执行 apply、没有验证真实 valid 修正对 EKF 的影响，也没有产生任何世界坐标精度结论。
5. 本次只做地面静止台架；所有解锁、起飞、返航及空中表现仍未测试，且必须由用户人工操作。

## 本次代码与配置改动

- `correction_service/config/camera.conf`：将生产下视相机改为现场确认的 USB `1.2` by-path。
- `correction_service/correction_service/node.py`：终态冻结单调完成时间，统一 result/status 耗时。
- `tests/test_correction_node_state.py`：新增终态耗时不随心跳增长的回归测试。
- `MEMORY.md`：更新当前部署、相机映射、真机验证和风险基线。
