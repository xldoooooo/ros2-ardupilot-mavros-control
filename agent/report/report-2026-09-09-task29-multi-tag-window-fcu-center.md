# Task 29 多 Tag 滑窗标定与飞控参考中心修正执行报告

- 日期：2026-09-09
- 主要任务规范：`agent/task/29-window.md`
- 歧义参考：`agent/task/task29-window-refine.md`
- 执行范围：本机源码、接口、配置、GUI、日志、测试与部署脚本
- 实机动作：未连接飞机、未部署、未停止或重启机载服务、未解锁、未起飞、未发送任何飞行命令

## 1. 结论

Task 29 的本机工程实现已完成。`correction_service` 现在支持按顺序观察不同 Tag、将每个停留段
汇总为一个原始 Odin 坐标系 keyframe、由服务端维护 FIFO 滑窗，并从保存的 `P_i/Q_i` 每次
重新求解完整的加权水平 SE(2)。首次仍用完整 SE(3) 位姿链产生粗修正和第一个 keyframe；第二次
起不再平均单 Tag yaw，也不累计历史修正。

独立 correction 接口及相关包已从 1.0 升级到 2.0.0，加入 first/next、服务实例与窗口 CAS、
clear、apply_saved、完整窗口快照和应用事实。GUI 已接入这些真实服务端能力，并将勾选“应用”
实现为先 dry-run、冻结并展示实际候选，再针对该候选二次确认并调用 apply_saved。

extnav 最终飞控中心水平转换已改为：

```text
correction valid:
    p_fcu.xy = (p_corrected - R_corrected*T).xy
    p_fcu.z  = p_raw.z + T.z - (R_corrected*T).z

correction invalid/API missing:
    p_fcu = p_raw + T - R_raw*T
```

因此 valid 世界水平分支已移除旧实现多出的固定 `+T_xy=(+0.06,-0.03)m`，同时保留旧 z 数值
约定和 invalid 局部原点约定。raw、corrected Odin、FCU 输入和 MAVROS EKF final 的语义已在
README 与面板中分开。

所有本机验收均通过：全目录 Python 回归 228 项通过，任务专项 46 项通过，工作区 5 个包构建
成功，ROS/CTest 汇总 22 项无失败，项目 `./build_onboard_control.sh --verify` 通过隔离 smoke。
这些结果证明实现和合成真值行为，不代表已经达到 `0.1～0.2°` 实机精度。

## 2. 多 Tag 滑窗实现

### 2.1 keyframe 几何与稳健汇总

- 仅从 `/odin1/odometry_highfreq` 取原始 Odin 位姿，不使用 corrected 结果回灌求解。
- 对每个图像时间匹配的样本执行完整变换链
  `H_OT = H_OI * H_IC * H_CT`，再取 Tag 中心 `Q_i^O` 的 xy。
- 保留相机杆臂、安装倾角、机体 roll/pitch/yaw 和飞机不在 Tag 正上方时的观测偏移。
- 每个停留段先完成多帧稳健汇总，再形成一个不可变 keyframe；保存 Tag ID、`P/Q`、采集时间、
  有效/拒绝样本数、独立时间块、位置散布、有效公制 sigma、重投影、同步误差、运动同步误差、
  tilt、单 Tag yaw、时间源、Odin session、配置指纹和来源 job。
- 连续帧按时间块估计独立信息，并叠加 Odin、Tag 世界测量和当前外参不确定度下限；不会随帧数
  增加把 sigma 无限压到零。

### 2.2 加权 SE(2)

- 使用 `w_i=1/sigma_i_eff^2` 的绝对公制逆方差权重和加权质心。
- 通过加权 dot/cross 求 `theta=atan2(S,C)`，只产生 `det(R)=+1` 的旋转，不拟合镜像或尺度。
- 两点、非等权、多点和共线但空间分离的二维点均支持；不误用三维“必须非共线”规则。
- 每次从窗口原始 `P_i/Q_i` 重算完整 `x/y/yaw`；不累计 delta，不将 active 修正烘焙进 keyframe，
  第 2 次以后不平均单 Tag yaw。
- yaw 模型标准差使用绝对权重的空间展开量计算；明确标记为模型估算而非实测精度。

### 2.3 质量门与 FIFO

配置中新增并校验：

- 默认窗口 5、最大 20、最小可用长度 2；建议精标 keyframe 数为 3；
- O/W 最小有效基线、基线长度比例误差；
- 相邻 keyframe 最大间隔、窗口最大跨度、候选最大年龄；
- keyframe 独立时间块、非零误差下限、同步运动误差；
- 配准 RMS、最大残差、模型 yaw 标准差、新点旧模型残差；
- 当前 FCU 中心的预计位置/yaw 跳变上限。

窗口拒绝重复 Tag、过近基线、O/W 比例不一致、镜像/尺度异常、非有限输入、混合 session、混合
配置指纹、混合时间源、时间跨度/间隔过大、残差或 yaw sigma 超限。长度 2 可正常解算和应用，
但状态明确说明其离群识别能力有限。

FIFO 在临时副本中先检查“淘汰前全部证据”，通过后才得到目标窗口。例如长度 2 时，第三次
候选先审查 `[Tag0,Tag1,Tag2]`，再形成 `[Tag1,Tag2]`。异常新点不会先删旧锚点，不会修改正式
窗口、成功 N 或 extnav active，也不会自动删除不利点来美化残差。

## 3. 服务端权威状态与应用事务

- 服务启动生成独立 `service_instance_id`；服务端权威维护窗口、window revision、成功 N、
  窗口长度、保存候选和 application 状态。clear 后 N 归零、revision 继续递增。
- first 只允许空窗，next 只允许已有同 session 窗口；窗口非空时长度锁定。
- 所有写请求检查 source sequence、service instance、window revision，并在需要时检查 extnav
  revision 和 Odin session。
- first/next 都是一次收敛自动结束。成功、失败和 stop 均释放任务相机、图像订阅和 400 Hz Odin
  订阅；idle 不创建这两类高负载订阅。
- dry-run 在资源释放后原子保存窗口、候选并增加 N/revision，完全不调用 extnav。
- 直接 API 的 apply=true 在资源释放后提交冻结候选；只有 ACK 或同 job/session/数值/revision 的
  新鲜权威状态对账确认后，才保存正式窗口并增加 N/revision。
- ACK 超时不会被误报为“未应用”。无法确认时锁存 `application_unknown`，保留 pending 诊断并
  阻止冲突写入；只允许同一 job/candidate 幂等重试。第三方 revision 覆盖后不会刷新 CAS 强行
  覆盖对方结果。
- 若 extnav 已确认应用、但本地窗口提交失败，会明确报告
  `applied_but_window_commit_failed`，不伪装成 unknown 或自动回滚 active。
- stop 在应用发出前取消并丢弃临时 keyframe；应用发出后只记录并继续 ACK/状态对账，不承诺撤销。
- clear 只清 correction_service 窗口/N/候选，绝不调用 extnav maintenance clear。
- apply_saved 只应用已保存且未过期的候选，不开相机、不增加 N；它短时订阅 fresh raw，在同一
  raw 样本上复算实际 FCU 中心跳变后立即释放订阅。

## 4. session、时间与配置一致性

- dry-run 保存也必须依赖新鲜的 extnav 2.0 权威 Odin session。
- idle 保留低频 extnav 状态订阅，因此相机和 400 Hz raw 订阅关闭期间仍能发现 session 改变。
- extnav 明确报告 Odin 不可用或 session 改变时清空窗口；活动采样同时取消。
- 任务 raw 断流、header 回退、主机 ROS 时钟回退或 frame pair 改变时取消任务并清空窗口。
- extnav 短暂状态过期只禁止继续/应用，不立即把网络抖动等同于 session 重启；恢复后重新核对。
- 同 epoch 使用严格 header 历史匹配；仅确认 epoch 不兼容时使用任务历史内的 arrival_history，
  硬门仍为 30 ms。不会使用识别完成时的“最新 Odin”。
- 配置指纹使用全部配置文件内容的 SHA-256；Tag 地图、内参、外参或采集/质量配置变化后不会复用
  旧窗口。

## 5. 飞控参考中心修正

### 5.1 数据链语义

```text
raw highfreq：Odin IMU 中心 / Odin 局部系
  -> 左乘公共水平 C
corrected highfreq：仍为 Odin IMU 中心 / 世界水平或 identity
  -> 物理杆臂中心转换
/extnav/pose_fcu 与 /mavros/vision_pose/pose：同一冻结 FCU 中心结果
  -> ArduPilot EKF
/mavros/local_position/pose：MAVROS EKF final 融合输出
```

公共校准候选和 corrected Odin 语义没有为抵消下游偏移而被污染，杆臂也没有错误改成 `Q*T`。

### 5.2 原子快照

- 一个 raw 回调内冻结 corrected、最终 FCU 位姿/速度、valid、revision、Odin session 和水平参考
  模式；两个最终 pose 话题读取同一结果。
- apply、clear、失效和断流状态切换会先清除最终样本缓存，等待下一条 raw 以新状态重建，禁止
  定时器把旧 pose 与新 valid/revision 混用。
- 2.0 `ExtnavCorrectionStatus` 同时报告 active 状态和最后一个最终样本自身的
  `final_sample_*` 元数据；GUI 将二者分开显示。
- valid=true 且 C 为单位变换仍进入世界有效分支，不以候选数值是否为零判断模式。
- 当前公式只对已经核实的零安装角作保证；extnav 回报安装角非零时拒绝 SetCorrection，服务端也
  拒绝应用并要求重新推导，不会机械套用当前简式。
- 速度仍沿用当前生产约定 `v_fcu=v_corrected-R_corrected*(omega×T)`；本任务没有人为插入额外
  速度脉冲。

合成集成测试使用非零杆臂 `(0.06,-0.03,0.05)m` 和 90° 世界 yaw：同一 raw 得到 corrected Odin
`(8.00,-2.00,0.70)m`，两个最终 FCU 话题均为 `(7.97,-2.06,0.70)m`。随后模拟 Odin session
失效，确认旧世界样本不再重复；下一条 raw 才以新 invalid/local 元数据恢复输出。

## 6. ROS 接口与 GUI

独立接口/相关包版本已统一为 2.0.0：

- 新增 `CalibrationKeyframe.msg`；
- 扩展 `CorrectionStatus.msg` 和 `CorrectionResult.msg`；
- 扩展 `ExtnavCorrectionStatus.msg`，加入中心参数、模式和原子最终样本元数据；
- 扩展 `StartCorrection.srv`，加入 first/next、window size 和 instance/window/extnav revisions；
- 扩展 `StopCorrection.srv` 的 instance 校验；
- 新增 `ClearWindow.srv` 和 `ApplySavedCorrection.srv`。

面板新增或完成：

- 预期 Tag ID、滑窗长度、首次、动态第 N 次、stop、clear、应用保存候选；
- 窗口非空锁定长度，数值框忽略滚轮；状态过期、请求处理中、unknown 或 revision 不匹配时门控；
- 所有 ROS 请求异步执行，不阻塞 Qt 主线程；面板重开从服务端恢复窗口/N；
- 展示 Tag 顺序和每个 keyframe 的 P/Q、sigma、样本/独立块、时间源；
- 展示最长 O/W 基线、方向、RMS、最大残差、模型 yaw sigma、候选、相对 active delta、当前 FCU
  位置/yaw 预计跳变、可应用原因、application 事实、资源状态和日志路径；
- 分开展示 raw Odin、corrected Odin、FCU 输入、MAVROS EKF final，并提示不同中心/参考系不可
  直接逐帧相减；
- “本次收敛后弹窗确认应用”即使选中，也先发送 apply=false。只有匹配本 job 的 dry-run 成功、
  候选保存且 revision 一致后，才展示实际候选、阶段、window/session/revisions、预计跳变和
  reset 限制，并由用户二次确认 apply_saved。确认不被解释为对未来未知候选的长期授权。

## 7. 日志与可复算性

任务和服务日志使用严格 JSON，不写非法 NaN；不可用指标写为明确缺失值。日志记录：

- first/next/stop/clear/apply_saved、source/job、instance/session/window/extnav revisions；
- 实际 Tag 地图快照、`T_imu_camera` 和覆盖全部配置文件的指纹；
- 每个合格样本的图像时间、Tag ID、P/Q、单 Tag 粗候选、tilt、重投影、匹配误差、时间源和
  Odin 速度；
- keyframe 汇总、独立时间块、有效 sigma、同步运动误差、weight；
- 窗口前后、拒绝证据、淘汰 Tag、每点 residual、最长基线、解算值和全部门限；
- FCU 杆臂、应用前后中心模式、实际预计跳变、SetCorrection 请求、ACK 或状态对账证据及终态。

日志足以离线重建 keyframe 汇总后的窗口配准；按任务范围没有长期保存 400 Hz 全量 raw 或视频。

## 8. 构建与测试证据

### 8.1 静态与接口

```text
.venv/bin/ruff format ...
  通过；最终无待格式化文件

.venv/bin/ruff check ...
  All checks passed

bash -n install_correction_service.sh install_extnav_correction.sh
  通过

ros2 interface show correction_interfaces/{msg,srv}/...
  通过；可见 2.0 first/next、clear、apply_saved、final_sample_* 字段
```

部署脚本同时核验 source/install 的 2.0.0 包清单和新接口字段，且在活动飞控链/冲突目标进程
存在时拒绝覆盖。extnav 安装器只覆盖、构建，不启动或重启飞控服务；独立 correction 安装器会
安装并 `enable --now` 启动保持 idle 的 correction 节点，因此本次未经允许没有执行任一安装器。

### 8.2 任务专项回归

命令：

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q \
  tests/test_correction_window.py \
  tests/test_correction_service.py \
  tests/test_correction_node_state.py \
  tests/test_correction_panel.py \
  tests/test_extnav_correction_integration.py \
  tests/test_correction_deploy.py
```

结果：`46 passed in 6.20s`。

覆盖两点/多点/非等权/共线、yaw wrap、短基线、尺度/镜像、非有限值、FIFO、时间跨度、重复 Tag、
时间块与误差下限、同步运动、完整 SE(3)、不同 Tag 尺寸、FCU 外参抵消、非零杆臂、任意姿态、
valid identity、valid/invalid session 切换、两路最终 pose、CAS/ACK/状态对账/unknown/第三方覆盖、
应用后窗口提交失败、stop/clear/apply_saved、资源故障、GUI 动态 N/门控/确认/滚轮和部署守卫。

专项第一次组合运行发现最后一个“订阅清理失败”测试的 monkeypatch 作用域延伸到了 fixture 销毁
阶段，导致 ROS 节点 teardown 循环。功能断言已经通过，但进程不能退出。已将故障注入限制在
被测 `_stop_image_capture()` 调用内；单测随后 `1 passed in 0.72s`，完整专项恢复为 46/46。
这属于测试隔离修复，没有掩盖产品失败。

### 8.3 全项目回归

```text
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests
  228 passed in 84.14s

colcon build --symlink-install
  5 packages finished

colcon test
colcon test-result --verbose
  Summary: 22 tests, 0 errors, 0 failures, 0 skipped

./build_onboard_control.sh --verify
  4 delivery packages Release build passed
  22 ROS/C++ tests passed
  isolated smoke passed:
    interface=3.2
    fcu_connected=false
    armed=false
    setpoint_messages=0
```

所有合成 ROS 集成使用 localhost-only 独立 domain；没有向实机 domain 发布 Odin、MAVROS 或
SetCorrection 消息。

## 9. 已知边界、未完成的实测与文档歧义

以下不是被隐藏的通过项：

1. 生产 `tag_pose.csv` 仍只有真实 Tag 0。未编造 Tag 1/2 生产坐标，因此无法在当前场地执行真实
   多 Tag 标定；多 Tag 功能目前以合成真值和隔离 ROS 验证。
2. `29-window.md` 说下视相机已安装且可看到 Tag0；refine 文件的较早现场描述说下视相机已拆除、
   当前连接云台相机。两份描述互相冲突。依用户指示以主任务为主要参考，但本次未连接飞机，故不
   推测当前硬件状态，也绝不使用云台相机替代下视标定相机。
3. 主任务同时说明下视相机经过拆装，旧内参仍保留而外参可能变化。本次配置中的 0.010 m 外参
   sigma 只是原理验证的保守下限，不是重装后的实测标定结果。相机固定到碳板后仍需重新标定外参。
4. 没有准确测量的多 Tag 场地、独立真值、检查点误差或跨 session 重复试验。因此不能声称已达到
   `0.1～0.2°`。拟合残差和模型 yaw sigma 都不是独立实测精度。
5. 水平模型假定窗口期间存在一个固定 SE(2)，且 O/W 重力轴足够一致。Odin 漂移、Tag 世界坐标
   系统误差、外参误差、曝光时间误差和跨高度 tilt 系统误差不会靠重复帧自动消失；不一致数据会被
   质量门拒绝，而不会通过尺度/非刚性模型掩盖。
6. z 继续使用旧局部原点约定，本任务没有世界高度校准；有效输出不能宣传成完整世界 SE(3)。
7. 当前飞控中心有效简式只对零安装角完成推导；安装角非零时本版明确拒绝应用，尚未实现一般化
   非零安装角位置/速度模型。
8. `/mavros/vision_pose/pose` 是 `PoseStamped`，不能携带 MAVLink estimator reset counter。
   extnav 内部 counter、状态和跳变均已记录，但“收到 ACK”不证明 EKF 已接受或稳定，更不是实飞
   许可。该接口迁移仍保留为既有 TODO，未擅自扩大本任务范围。
9. 同帧多 Tag 联合识别、onboard_control 自动航点触发、自动移动/起飞和真实飞行验收均不在本次
   范围，未实现。
10. 仓库 2.0 源码尚未部署到飞机；最后有记录的飞机运行基线仍是 task 27 的 1.0。部署前必须在
    明确维护窗口重新只读核对杆臂、安装角、飞控参数、source/install/runtime 版本。停止/重启机载
    服务、应用修正、解锁和起飞均需要用户明确安排或人工操作。

## 10. 主要文件

- 算法与状态机：
  `correction_service/correction_service/window.py`、`estimator.py`、`geometry.py`、`node.py`
- GUI/客户端：
  `correction_service/correction_service/correction_panel.py`、`ros_client.py`
- extnav：
  `correction_service/extnav_patch/extnav_to_vision_pose.py`
- 接口：
  `src/correction_interfaces/msg/`、`src/correction_interfaces/srv/`
- 配置：
  `correction_service/config/general_settings.yaml`
- 文档：
  `correction_service/README.md`、`correction_service/extnav_patch/README.md`、`MEMORY.md`
- 测试：
  `tests/test_correction_window.py`、`test_correction_node_state.py`、
  `test_correction_panel.py`、`test_correction_service.py`、
  `test_extnav_correction_integration.py`、`test_correction_deploy.py`

## 11. 后续实测建议

这不是本次未实现的软件补丁，而是取得实机精度证据所必需的后续条件：

1. 将正确的下视相机固定到最终碳板结构，重新标定并复核内外参；确认不是云台相机。
2. 精确测量至少 3 个空间分离 Tag 的世界中心、尺寸和朝向，并记录测量不确定度。
3. 在保持人工解锁/起飞的前提下，先做无桨台架和手动移动采集，再安排受控飞行；任务代理不发送
   飞行命令。
4. 每次部署前只读核对 `T`、安装角、`VISO_POS_*`、`INS_POS*`、Odin session 和运行接口版本。
5. 分别报告拟合内 RMS、未参与配准的检查点误差、不同 session 重复性和应用时 EKF 行为；只有
   独立真值结果才可用于判断是否达到 `0.1～0.2°`。
