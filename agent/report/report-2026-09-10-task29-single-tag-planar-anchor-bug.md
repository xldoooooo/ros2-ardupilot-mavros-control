# Task 29 首次粗标定水平锚点 Bug 修复与真机复验简报

- 日期：2026-09-10（Asia/Shanghai）
- 飞机：`nvidia@192.168.112.169`，Ubuntu 24.04 / ROS 2 Jazzy
- 用户现象：校准后把飞控中心大致移到 Tag0 正上方，地面站仍显示约 `x=0.20 m, y=0.10 m`
- 安全边界：全程 `armed=false / STABILIZE`；未解锁、未起飞、未切模式、未申请控制租约，未发送
  姿态、速度、位置、航点或飞控参数命令

## 结论

确认并修复了一处会直接产生厘米级水平偏差的代码 Bug：首次粗标定从完整 SE(3) 丢弃
roll/pitch、只应用 yaw 时，旧代码仍照搬完整三维变换的 x/y 平移。非零 tilt 下，该受限 SE(2)
甚至不能把本次观测到的 Tag 中心 `Q` 映射到配置中的世界中心 `P`，但旧日志把首点残差固定写成
了 0。

修复后保持完整相机/IMU/Tag SE(3) 链和 yaw/tilt 计算不变，只把最终可应用的水平平移改为：

```text
t_xy = P_xy - Rz(yaw) * Q_xy
```

停留段分别稳健汇总 x/y/yaw/Q 后，冻结首次候选时还会用汇总后的 P/Q/yaw 再锚定一次。因而最终
候选严格满足 Task 29 的同一模型 `P=RQ+t`。没有修改物理杆臂 `T=(0.06,-0.03,0.05)m`，也没有
把 T 烘焙进公共 correction。

这处 Bug 能解释原结果中的显著一部分，但现有证据不能把用户报告的全部 `20/10 cm` 都归因于它。
修复后的同位置真机链路仍显示约 `(4.2,12.9) cm`；进一步检查表明该剩余量已经存在于“相机外参
估计的 Tag 中心”和“当前 T 换算的 FCU 中心”之间，而不是 extnav/MAVROS 再次多加了平移。
若物理中心确已精确重合，剩余问题应优先归入当前相机外参/安装 tilt/T 实测值，而不是继续改公式
或靠放宽门限掩盖。

## 旧版 Bug 的现场证据

读取飞机 2026-09-10 当晚全部 7 个成功首标候选，将日志中的候选重新代回
`P_hat=R(yaw)Q+t`。Tag0 配置均为 `P=(0,0)`：

| job | yaw | tilt | 旧候选得到的 P_hat (m) | 错误闭环残差 |
| --- | ---: | ---: | ---: | ---: |
| `8415e9311278` | -9.61° | 5.64° | (-0.0169, -0.0694) | 0.0715 m |
| `ab261433a9a1` | -23.20° | 3.12° | (-0.0309, -0.0263) | 0.0406 m |
| `48fd548d738a` | +155.89° | 6.77° | (-0.0614, -0.0601) | 0.0859 m |
| `f3a1f857450c` | -24.57° | 5.55° | (-0.0604, -0.0356) | 0.0701 m |
| `fdc3ecceb514` | -22.74° | 7.31° | (-0.0178, -0.0870) | 0.0888 m |
| `9335db6ebf27` | +13.59° | 6.16° | (-0.0068, -0.0784) | 0.0787 m |
| `211234485348` | -15.46° | 7.58° | (-0.0278, -0.0923) | 0.0964 m |

与用户现象时间最接近的 `211234485348` 保存了：

```text
P = ( 0.000000,  0.000000) m
Q = (-0.048990, -0.128901) m
旧 t = (0.053816, 0.018912) m
yaw = -15.462725 deg
```

旧候选把该 Tag 中心映射到 `(-0.027767,-0.092263)m`，残差 `0.096350m`。对相同 Q/yaw，正确
锚定平移应为 `(0.081584,0.111174)m`；旧版平移本身少了 `(0.027767,0.092263)m`。这不是手工
摆放误差，而是候选内部已经违反自身 P/Q 约束的确定性代码错误。

根因可直接由变换写出。完整链满足三维关系 `P3=R_full*Q3+t_full`；但实际应用的是
`Rz(yaw)` 且丢弃 roll/pitch。旧代码使用 `Rz(yaw)*Q_xy+t_full_xy`，非零 tilt 时通常不再等于
`P_xy`。原测试只构造 tilt=0 的完整修正，所以未覆盖到这条生产路径。

## 代码修改

- `correction_service/correction_service/geometry.py`
  - 新增给定 P/Q/yaw 的唯一水平平移求解；
  - 每帧完整 SE(3) 后以 Tag 中心锚定受限 SE(2)，不再照搬 full translation。
- `correction_service/correction_service/node.py`
  - 首次候选冻结时用稳健汇总后的 P/Q/yaw 再求一次平移；
  - registration 中的零残差现在与真实候选一致。
- `tests/test_correction_service.py`
  - 新增 8° tilt 的完整合成真值，证明旧公式产生大于 5 cm 残差而新公式闭环。
- `tests/test_correction_node_state.py`
  - 新增独立汇总 x/y/Q/yaw 后的冻结候选闭环回归。
- `correction_service/test/test_correction_package.py`
  - 加入可在 Jetson `colcon test` 中运行的 P/Q 锚点快速回归。
- `correction_service/README.md`
  - 明确首次 SE(3) 到受限 SE(2) 的投影语义。

## 本机验证

- correction 专项：`41 passed`
- 正式全量范围：`235 passed in 83.63s`
- Ruff（全部本次 Python 文件）：通过
- `git diff --check`：通过
- `colcon build --packages-select correction_interfaces correction_service --symlink-install`：通过

## 飞机部署

飞机 Git HEAD 仍为历史 `6a40713` 且工作树有既有选择性部署内容。本次没有 pull/reset/clean，
只同步 4 个修正服务文件。部署前定点备份：

```text
/home/nvidia/backups/task29-planar-anchor-predeploy-20260910-011345/
```

旧文件 SHA-256：

```text
68ef92aa637c0adf2eed3aa450ee30ee2ea2339e7c24af0bd9e6a5ed430720cc  geometry.py
8c2edf43de550b5985d6ee2eb69952ddf7d5dcca1f23fb968ad389e3e0d521ff  node.py
37d34ea9c2005c047e806ca66309fb80643a1573f54b2d7ad74e619073ed64e3  README.md
a3e03aae9ec1583d1708f93942e5b16cd732cee96f25202417c8630e716a228c  test_correction_package.py
```

同步后飞机源码与本机哈希一致。正式安装器在 Jetson 原生重建并启动 idle 服务。飞机包测试为
`23 tests, 0 errors, 0 failures, 0 skipped`，其中包内 Python `4 passed`。

首次启动飞行链时，诊断轮询 shell 命令行自身包含了启动脚本防重检查的进程名称，被安全检查当作
疑似重复进程，因此前两次在任何飞行组件启动前主动拒绝；轮询退出后 systemd 第三次重试正常启动。
这是本次诊断命令造成的无害假阳性，不是产品节点故障；没有绕过防重检查。

## 修复后真机 dry-run

- job：`0e8730adb3b5`
- 24 accepted / 0 rejected，12 个独立时间块，采样跨度 2.867 s，总耗时 6.659 s
- P：`(0,0)m`
- Q：`(-0.069268650,-0.107408604)m`
- 候选：`t=(0.093497801,0.087137335)m`，yaw `-14.198361°`
- tilt：`6.251024°`
- 重投影：`0.350920px`；Odin 匹配：`0.948836ms`，来源 `arrival_history`
- 预计当前 FCU 水平跳变：`0.137717m`；yaw 跳变 `14.198361°`
- 相机与任务 raw 订阅均在候选冻结后释放

逐帧重新代入 24 个样本，最大 `||P-RQ-t||=1.96e-17m`，平均 `7.76e-18m`；最终稳健候选
同样为数值精度零残差。旧版的 4～10 cm 内部矛盾已经消失。

## apply、输出链与清理

再次读取 FCU 为 `armed=false / STABILIZE` 后，显式 apply_saved 得到 ACK，extnav revision
`0→1`。稳定快照（话题顺序读取，非同一纳秒）为：

| 阶段 | x (m) | y (m) |
| --- | ---: | ---: |
| raw Odin | -0.000068 | -0.000644 |
| corrected Odin | +0.093099 | +0.086735 |
| FCU 输入 | +0.0415～+0.0421 | +0.1287～+0.1295 |
| MAVROS EKF | +0.0420～+0.0421 | +0.1275～+0.1280 |

FCU 输入与 EKF 稳定后相差约 1 mm，证明候选、corrected Odin、杆臂换算、MAVROS 输入和 EKF
链路均实际生效；它不证明候选对应绝对物理真值。

随后再次确认 `armed=false`，直接调用 extnav CAS 清除，revision `1→2`、reset counter `1→2`，
并清空 correction_service 窗口到 revision 2。约 20 秒后：

- extnav：`correction_valid=false`、identity/local reference、raw/corrected 计数相等；
- FCU：约 `(-0.00118,-0.00343)m`；
- MAVROS EKF：约 `(-0.00002,-0.00463)m`；
- 两路相机均无人占用。

## 剩余非零量的分离检查

在飞机未移动、修正已清回 identity 后，用同一 USB `1.2` 下视相机和生产镜头参数取稳定帧，
项目同一检测器得到：

```text
Tag0 full-resolution center ~= (801.97, 379.36) px
camera principal point      = (960.67, 547.85) px
T_camera_tag translation    = (-0.09233, -0.09892, +0.65415) m
T_imu_tag translation       = (-0.06607, -0.08696, -0.72670) m
reprojection RMS            = 0.36901 px
```

在 Task 29 已指定的零安装角和 `T_FI=(+0.06,-0.03,+0.05)m` 下，如果只看水平量，当前相机链
给出的 `F→Tag` 约为：

```text
T_FI.xy + T_IT.xy = (-0.00607, -0.11696) m
```

也就是说，当前传感器/配置本身认为 Tag 中心相对 FCU 中心约偏了 11.7 cm，方向与修复后
FCU/EKF 的主要剩余 y 量一致。Tag 在原始图像中也明显不位于按现有 T/外参预测的水平位置。
因此这部分不是 extnav 有效分支又多加了一次 T；当前公式的合成测试与真实 FCU/EKF 对照均一致。

这项检查还不能区分以下三者：

1. “大约重合”的人工摆放本身仍有约 10 cm 横向误差；
2. 相机拆装后 `T_imu_camera` 的平移/倾角已不再代表当前机械安装；
3. `T_FI=(0.06,-0.03,0.05)m` 的物理参考点或实测值与用户所指“飞控中心”不一致。

当前 full correction tilt 为 6.25°；在约 0.65～0.73 m 高度上，单是数度重力轴/安装倾角误差
就可产生约 7 cm 的高度相关水平分量。历史独立外参候选还曾相差 4.66 cm。因此若用量具确认
F/Tag 水平中心确实重合，下一步应固定相机后重做外参，并重新实测 T 和两个参考点；不能通过
修改 T 符号、把 T 烘焙进 correction 或继续放宽门限来制造零点。

## 最终飞机状态

- `ros2-ardupilot-onboard.service`：已停止；现有停止脚本按历史行为显示 failed/status 130，但
  MAVROS、Odin、extnav、onboard 进程均已退出。
- `odin-correction.service`：active/enabled、idle；窗口 N=0，候选为空，资源已释放。
- `video-service.service`：inactive。
- extnav 在停止前已明确为 correction invalid、revision 2 identity；没有残留 active 修正。
- `/dev/video0`、`/dev/video2` 均无人占用。
- 全程未解锁、未起飞。

后续最有价值的人工复验是：用铅垂线或固定治具而非目测，把 Task 29 定义的 FCU 参考点与 Tag0
中心对齐；在另一个初始位置完成 first 后再搬到该检查点，记录同步 raw/Q/FCU，而不是只抄地面站
最终 EKF。该试验无需解锁或起飞。
