# 任务 29 放宽实验门限与真机修正效果测试简报

- 日期：2026-09-09（Asia/Shanghai）
- 飞机：`nvidia@192.168.112.169`，Jetson Orin NX，Ubuntu 24.04 / ROS 2 Jazzy
- 前置报告：`report-2026-09-09-task29-aircraft-deployment-camera-bench.md`
- 用户要求：适当放宽实验门限，将偏航修正应用跳变上限先放宽到 45°，避免原理验证一直被门限锁死
- 安全边界：全程未解锁、未起飞、未切换飞行模式、未申请控制租约、未发送运动或飞控参数命令

## 结果

根据前一轮真实数据，只放宽两个已经明确造成阻塞的门限：

| 门限 | 原值 | 新值 | 作用 |
| --- | ---: | ---: | --- |
| `detection.max_correction_tilt_deg` | 8° | 10° | 避免实测约 8° 的边界抖动大量拒绝采样 |
| `window.max_apply_yaw_jump_deg` | 10° | 45° | 允许未武装地面实验观察约 14.4° 的世界系 yaw 修正 |

其余重投影、同步误差、位置/yaw 离散度、独立时间块、最大位置跳变、session/revision CAS 和候选
年龄门限均保持不变。配置及文档明确注明这是用户授权的地面原理验证范围，不是外参精度合格、
世界坐标准确或可实飞的证据。

放宽后一次有效 dry-run 为 24 accepted / 0 rejected、7.48 s，候选通过全部现有门限。随后按
dry-run → 显式 `apply_saved` 流程成功应用，extnav ACK 将 revision 从 0 增至 1；四段位姿与
MAVROS EKF 均观测到预期修正。完成观察后通过 extnav CAS 显式清回 identity，revision 增至 2，
再清空 correction_service 窗口。最终没有残留 active 修正。

## 配置部署与验证

部署前备份位于：

```text
/home/nvidia/backups/task29-gates-predeploy-20260909-2325/
```

旧文件 SHA-256：

```text
2bb8543db130522346d53f6a7fc4adccf27f22607309159557eb4f478ae33354  general_settings.yaml
343903219443425addb156c8e5510ff672aeca26a27d24310d1c402b612ce622  README.md
d79482243e004ef5bbd9e051ab12d44c81457b0e65e75ea16a918509d335764c  test_correction_package.py
```

只同步上述三个 correction_service 文件；飞机既有 dirty 工作树、guided/onboard/video/Odin 文件和
历史 Git HEAD `6a40713` 均未改写。停止 idle 修正服务后运行正式安装器，飞机原生重建
`correction_interfaces` 与 `correction_service`，随后源码与 install share 配置均回读为 10°/45°。

最终本机与飞机源码 SHA-256 一致：

```text
7cd1b28609ec46988abe4266ed567af0aba081610bdff3b9830904051ba9dc92  general_settings.yaml
37d34ea9c2005c047e806ca66309fb80643a1573f54b2d7ad74e619073ed64e3  README.md
a3e03aae9ec1583d1708f93942e5b16cd732cee96f25202417c8630e716a228c  test_correction_package.py
```

测试结果：

- 本机门限配置及 correction 专项：38 passed。
- 本机完整正式范围（`tests/` 加 correction 包内测试）：232 passed in 83.03 s。
- 飞机 aarch64 包测试：22 tests、0 errors、0 failures、0 skipped；包内 Python 3 项通过。
- 本次 Python 文件 Ruff 与 `git diff --check`：通过。

## 真机采样

### 首次尝试：现场短时未识别到 Tag

- job：`ae2cedd31088`
- 60.41 s 内 437 次处理，只有 1 次检测/接受，436 次拒绝，最终按最大运行时长失败。
- 唯一有效帧 tilt 为 9.079°，说明它已经通过新的 10° 门；失败主因是其余帧“未检测到 Tag”，
  不是 yaw 或 tilt 门限。
- 任务失败后 `resources_released=true`，窗口和 extnav 均未改变。
- 随后从同一 USB `1.2` 下视相机取当前帧，Tag0 在画面中清晰完整；使用项目同一 OpenCV
  AprilTag 36h11 检测器离线立即识别 ID 0，重投影误差约 0.317 px。因此没有继续放宽检测门限，
  只在画面恢复后原配置重试。

### 有效 dry-run

- job：`444c7fa09581`
- 24 accepted / 0 rejected，12 个独立时间块，跨度 2.867 s，总耗时 7.480 s。
- keyframe `Q=(-0.084624, 0.049016) m`，世界 Tag0 中心 `P=(0,0)`。
- 候选 `x=0.070969 m, y=0.043864 m, yaw=-14.419973°`。
- tilt `8.657214°`，有效位置 sigma `0.012247 m`，yaw sigma `0.022272°`。
- 重投影误差 `0.310361 px`；Odin 到达时间历史匹配误差 `0.578659 ms`。
- 预计 FCU 中心位置跳变 `0.089769 m`，yaw 跳变 `14.419973°`。
- `can_apply=true`；相机与任务 raw 订阅在候选冻结后先释放。

## 应用与效果

应用前近同时快照：

| 阶段 | x (m) | y (m) | yaw (°) |
| --- | ---: | ---: | ---: |
| raw Odin | 0.002711 | -0.000912 | 0.01965 |
| corrected Odin | 0.002711 | -0.000912 | 0.01965 |
| FCU 输入 | 0.002578 | -0.001894 | 0.01942 |
| MAVROS EKF | 0.002930 | -0.001405 | 0.03000 |

`apply_saved` job `444c7fa09581-apply` 使用同一 service instance、window revision 1、Odin session
和 expected extnav revision 0。新鲜 raw 复算结果为位置跳变 `0.089470 m`、yaw 跳变
`14.419973°`，随后 extnav 返回：

```text
success=true
application_state=applied_ack
revision=1
duration=0.038049 s
```

extnav 权威状态同步变为 `correction_valid=true`、`reset_counter=1`、参考模式
`tag_world_xy_local_z`，且 raw/corrected/final sample 的 session/revision 一致。

应用约 8 秒后的稳定快照：

| 阶段 | x (m) | y (m) | yaw (°) |
| --- | ---: | ---: | ---: |
| raw Odin | 0.003589 | -0.001216 | 0.01772 |
| corrected Odin | 0.074142 | 0.041793 | -14.40225 |
| FCU 输入 | 0.023121 | 0.084878 | -14.40213 |
| MAVROS EKF | 0.023364 | 0.086052 | -14.39901 |

稳定时 EKF 与 FCU 输入水平位置相差约 1.2 mm，yaw 相差约 0.003°，证明 extnav 修正、Odin→FCU
中心转换、MAVROS 输入和 EKF 响应链实际生效。它只证明“系统按候选工作”，不能证明候选本身是
准确世界真值；当前仍只有单 Tag，且相机外参在拆装后未重标。

应用期间 FCU 始终 `armed=false / STABILIZE`，onboard 为待机、controller inactive、无租约、
无 setpoint 冲突和 failsafe。

## 清除与 EKF reset 瞬态

观察完成后直接调用 extnav 的 CAS 维护接口，使用相同 Odin session 和 expected revision 1，
提交 `valid=false` identity。响应 accepted，revision `1→2`；权威状态为：

```text
correction_valid=false
revision=2
reset_counter=2
horizontal_reference_mode=local_identity_origin
applied_job_id=''
```

输入链立即恢复 identity，但 MAVROS EKF 因 PoseStamped 没有 estimator reset counter 出现可见瞬态：

| 清除后时刻 | vision 输入 yaw | EKF yaw |
| --- | ---: | ---: |
| 立即 | 0.020° | -1.289° |
| 约 8 s | 0.032° | +1.994° |
| 约 20 s | 0.030° | +0.286° |
| 约 30 s | 0.027° | +0.064° |

这验证了原任务中“ACK 不等于 EKF 已稳定、PoseStamped 缺少 reset counter”的风险不是理论问题。
飞机未武装时瞬态最终衰减；不得据此推断空中切换安全。

## 最终状态

- 实验服务实例的窗口已显式清空到 window revision 2、N=0；最终同步 README 后安装器重启了
  correction_service，新实例为 revision 0、窗口为空、无候选且资源已释放。
- extnav 最后在线状态为 `correction_valid=false / revision=2 / reset_counter=2`，identity passthrough。
- 两台摄像头均无人占用；无 correction camera、FFmpeg 或 GStreamer 进程。
- `ros2-ardupilot-onboard.service` 已恢复测试前的停止状态；既有停止脚本使 unit 显示
  `failed/status 130`，但 MAVROS、Odin、extnav、onboard 相关进程均已退出。
- `odin-correction.service` 保持 active/enabled 且 idle；`video-service.service` 保持 inactive。
- 全程没有解锁或起飞。

## 仍未解决

1. 当前单 Tag 粗候选约 -14.4° 的来源仍可能是相机外参偏移、Tag 实际世界朝向或两者共同作用。
2. 当前 tilt 8.66° 表明水平 SE(2) 近似余量有限；10° 门仅用于这次原理验证。
3. 没有 Tag1/Tag2 的测量世界坐标，真实多 Tag 空间配准和独立精度仍未验证。
4. EKF reset 瞬态已实测存在；在解决 reset counter/切换策略并完成空中安全评审前，不允许把
   在线应用修正作为实飞操作。
5. 后续若继续实验，应优先固定下视相机、重标外参和精测 Tag 朝向，而不是继续放宽其他质量门。
