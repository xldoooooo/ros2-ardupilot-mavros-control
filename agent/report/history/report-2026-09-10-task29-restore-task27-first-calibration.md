# Task 29 恢复 Task27 首次标定算法简报

- 日期：2026-09-10（Asia/Shanghai）
- 范围：仅本地源码、测试与构建；飞机已断开，本轮未连接、未部署、未运行任何机载操作
- 目标：严格恢复“Task27 首次粗标定 + Task29 有效分支删除多余 `+T_xy` + 从第二个
  keyframe 起才执行多 Tag 配准”

## 结论

本地源码已恢复到要求的三段语义：

1. 首次标定逐帧继续按 Task27 的完整 SE(3) 链计算 `C_full=H_WI*inverse(H_OI)`，候选直接取
   `C_full` 的 x/y 平移与 yaw；没有单点 `P-RQ` 重锚。
2. 首次的 x/y/yaw 离群筛选和稳健中值/圆均值恢复 Task27 特征集合。新增 Q 只作为首个
   keyframe 的记录和质量证据，不参与或改写首次候选。
3. extnav 的 Task29 有效修正分支保持 `p_F.xy=(p_corrected-R'*T).xy`，没有恢复错误的固定
   `+T_xy`；无效分支继续保留旧局部零点兼容语义。
4. 只有 `operation=next` 且窗口已有首个 keyframe 时，才从滑窗 `P_i/Q_i` 调用加权 SE(2)
   配准；首次不调用多点配准求解器。

上一轮 `49f5b32` 的“首次单点重锚”是错误回归，已经从本地生产路径和包级测试中移除。原报告
`report-2026-09-10-task29-single-tag-planar-anchor-bug.md` 已增加醒目的“结论作废”说明，保留为
可追溯的错误过程记录。

## 历史源码审计

以 Task29 前最终基线 `6098d68`、Task29 初始提交 `f83b872` 和错误提交 `49f5b32` 逐文件比较：

- `f83b872` 没有改写 `compute_planar_correction()` 的首次 x/y/yaw 公式，只增加了原始
  `Q_i`、窗口数据结构和 FCU 中心计算辅助函数。
- `49f5b32` 才新增 `planar_translation_from_correspondence()`，将逐帧和平滑冻结后的首次平移
  都改为 `P-Rz(yaw)Q`；这与 Task29 细化规范第 3.1 节明确给出的
  `C=planar_xy_yaw(C_full)` 冲突。
- 当前 `geometry.py` 相对 `6098d68` 的首次链只多计算并返回 `odin_from_tag/Q_i`；
  `world_from_camera`、`world_from_imu`、`world_from_odin`、x/y/yaw/tilt 的 Task27 计算次序与
  数值表达均已恢复。
- 当前 AprilTag `detector.py` 与 `6098d68` 逐字节无差异。
- Task29 曾把新增 Qx/Qy 加入首次 MAD 特征，理论上会改变 Task27 入选帧。现首次特征恢复为
  `[x,y,yaw_delta]`，后续 keyframe 才使用 `[Qx,Qy]`；首次候选的中值和圆均值公式与 Task27
  一致。
- 本轮没有修改 `extnav_to_vision_pose.py`，因此 Task29 删除有效分支 `+T_xy`、原子快照、
  session/revision 等机载链路逻辑均保持不变。

## 为什么撤销单点重锚

Task27 首次计算的是坐标系变换：

```text
H_WI   = H_WT * inverse(H_CT) * inverse(H_IC)
C_full = H_WI * inverse(H_OI)
C_first = [C_full.tx, C_full.ty, yaw(C_full)]
```

若 `C_full` 存在 roll/pitch，最终只应用 yaw 后，某个离坐标原点有高度的三维 Tag 点一般不再严格
满足二维 `P=Rz(yaw)Q+t`。这个差值是“丢弃 tilt 后的点投影差”，不是证明 `C_full.tx/ty` 错误的
残差。把首次平移强制改成 `P-Rz(yaw)Q` 会将 Tag 观测高度和被丢弃的 tilt 分量耦合进世界/Odin
坐标系原点平移，使结果随当前观察位置和高度变化。

保存的真机帧离线复算曾显示该错误改动相对 Task27 单独增加约 `(+1.98,+5.55)cm`，与用户实测
由约 `(20,10)cm` 恶化到 `(25,15)cm` 的方向和量级一致。该帧并非用户搬动全过程的严格同步
真值，因此只作为回归机理证据，不用来逐厘米解释最终 EKF 数值。

## 去畸变审计

当前生产流程没有先 remap 整张图，而是使用数学等价且避免二次校正的路径：

```text
原始畸变图检测/亚像素角点
  -> cv2.solvePnP(object_points, distorted_points, scaled_K, D)
  -> cv2.projectPoints(..., scaled_K, D) 计算重投影残差
```

`image_scale=0.5` 时同步缩放 `fx/fy/cx/cy`，畸变系数 D 保持不变。新增自动化回归使用明显偏轴
的合成 Tag 角点，验证“原始角点 + K,D”和“`undistortPoints` 后角点 + K,0”恢复的平移、旋转
一致到 `1e-10`。此前保存的真机帧 A/B 结果为平移差 `1.1e-16m`、旋转向量差约 `1e-13rad`。
因此生产 PnP 没有遗漏镜头畸变；若未来改为整图去畸变，必须同时改用对应新内参并传零畸变，
不能再次传 D。

## 用户场景端到端回归

新增完整合成链模拟：

- 校准时飞机实际位于 Tag0 附近；
- Odin 自建系中的 IMU 读数为 `x=5m, y=6m, yaw=9deg`；
- 使用生产 `T_imu_camera`、Tag0 世界位姿和 `T_FI=(0.06,-0.03,0.05)m`；
- 首次由 Task27 完整链恢复 `世界<-Odin`；
- 随后把 FCU 中心搬到 Tag0 的水平正上方，并用同一固定修正计算最终 FCU 中心。

测试结果：

```text
恢复候选 t = (-5.4670484932, -5.3939577184)m
恢复候选 yaw = -9deg
Task29 有效 FCU 输出 xy = (-3.89e-16, +6.38e-16)m
若错误保留旧 +T_xy，输出将固定变成 (+0.06,-0.03)m
```

该测试证明代码代数满足用户要求的预期效果；它是无噪声合成真值，不能替代下一轮真实相机、
外参、Odin 和 EKF 的实机精度验收。

## 修改范围

- `correction_service/correction_service/geometry.py`
  - 删除错误单点重锚函数；恢复 Task27 的 `C_full.tx/ty/yaw` 直接提取。
- `correction_service/correction_service/node.py`
  - 首次冻结候选恢复直接采用 Task27 稳健汇总结果；Q 只写入 keyframe。
- `correction_service/correction_service/estimator.py`
  - 首次 MAD 特征恢复 Task27 的 x/y/yaw；后续仍使用 Qx/Qy。
- `tests/test_correction_service.py`
  - 反向锁定非零 tilt 时的 Task27 投影语义；增加用户 5/6/9° 场景端到端回归和畸变等价回归。
- `tests/test_correction_node_state.py`
  - 锁定首次冻结不得被 P/Q 重锚。
- `tests/test_correction_window.py`
  - 锁定新增 Q 不得改变首次 Task27 入选帧；Q 异常仍可阻止坏 keyframe 保存。
- `correction_service/test/test_correction_package.py`
  - 包级非零 tilt 回归改为锁定 Task27 `C_full` 平移。
- `correction_service/README.md`、`MEMORY.md` 和旧错误报告
  - 删除错误基线，记录本地/机载版本边界。

未修改 detector、相机采集、extnav、MAVROS、onboard_control、视频服务、接口定义、部署脚本或
其他机载服务代码。

## 本地验证

- 修正专项：`44 passed in 1.42s`
- 项目正式范围 `tests/ + correction_service/test`：`238 passed in 83.75s`
- Ruff format/check：通过
- `git diff --check`：通过
- 500 组随机完整 SE(3) 复算：
  - `world_from_odin` 最大矩阵误差 `1.07e-14`
  - Task27 x/y 提取最大误差 `8.88e-15m`
  - Task27 yaw 提取最大误差 `4.44e-16rad`
- 1000 组随机水平修正、任意 FCU 位置/yaw 与非零 T 的有效分支复算：FCU xy 最大误差
  `1.07e-14m`
- `colcon build --packages-select correction_interfaces correction_service --symlink-install`：2 包成功
- colcon：`23 tests, 0 errors, 0 failures, 0 skipped`；包内 Python `4 passed`
- detector 与 Task29 前基线：无源码差异
- extnav 相对本轮 HEAD：无工作树差异

曾有两次扩大 pytest 收集范围的命令分别因独立 websocket 示例未安装 `ws_demo`、系统 pytest
解释器看不到项目虚拟环境 PySide6 而在收集阶段中止；随后按项目规范使用项目 `.venv` 的 Python
模块入口和正式测试范围完成上述 238 项验证。没有将收集环境问题伪报为通过，也没有出现产品
断言失败。

## 机载状态与下一轮边界

飞机本轮已断开，未执行 SSH、部署、服务启停或相机访问。飞机上一次已知部署仍包含错误的
`49f5b32` 首次重锚语义，因此在下一轮明确部署本次本地版本前，不应继续用当前机载首次标定结果
做精度结论。

下一轮真机测试应先选择性部署 correction_service，再在始终未武装状态下：

1. 确认机载源码/install/runtime 的 geometry 和 node 哈希一致；
2. clear 旧窗口和 extnav active，重新建立同一 Odin session 的首次候选；
3. 记录同步 PnP `H_CT`、raw `H_OI`、`C_full`、Task27 候选、corrected Odin、FCU 输入和 EKF；
4. 由用户使用固定治具或铅垂线把 FCU 参考中心搬到 Tag0 正上方；
5. 检查 FCU 输入是否接近 `(0,0)`，并把 EKF 动态响应与输入几何误差分开。

用户此前报告的约 `(20,10)cm` 原始偏差仍然没有归因。本次只撤销已确定的回归并建立正确算法
基线，不再用错误单点残差将其武断归因于相机外参、T、人工摆放或 EKF。
