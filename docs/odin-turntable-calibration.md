<!-- 水平固定转心的 Odin 杆臂校准方法、手动使用入口与可辨识范围。 -->
# Odin 水平转台校准

`scripts/diagnostics/calibrate_odin_turntable.py` 在既有固定安装外参方法上估计
`odin_x/y`，不改驱动、extnav 或世界原点。将 FCU 中心与转台轴重合、飞机保持水平，
缓慢匀速转满360°；建议一圈30–60秒。起止处可短暂停留，角度分组权重避免停留段支配拟合。
验收看各路 XY 是否保持稳定，允许 highfreq 与 vision 存在常数差值。

## 采集与计算

在飞机项目根目录运行，使用已在运行的项目 Odin/extnav：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
mkdir -p agent/codex/odin-turntable
.venv/bin/python scripts/diagnostics/calibrate_odin_turntable.py record \
  --config config/aircraft/uav002/odin_extrinsics.yaml \
  --seconds 90 --output agent/codex/odin-turntable/turn-1.jsonl
```

看到 `READY` 后开始手动旋转。程序只订阅原始诊断 IMU、标准/高频FCU里程计、vision和修正状态，
不发送飞行或参数命令。保存设备时间戳、主机接收时间、配置快照和发布者身份。
`--output -` 可通过 SSH 将数据直接写到开发机。缺少依赖时使用项目 Python 安装
`scripts/diagnostics/requirements.txt`；采集另需现有 ROS 及 correction_interfaces 环境。

离线计算不需要 ROS：

```bash
.venv/bin/python scripts/diagnostics/calibrate_odin_turntable.py fit \
  --input agent/codex/odin-turntable/turn-1.jsonl \
  --output agent/codex/odin-turntable/fit-1
```

输出 `candidate.yaml`、`result.json` 和三路 CSV；CSV 保留校准前位置与候选外参的离线预测，
可在 PlotJuggler 对照。候选文件不是已生效配置，工具不自动覆盖生产文件。
需要采用时，仅将候选的 `odin_x/y` 写入该飞机权威 `odin_extrinsics.yaml`，保留其他现场设置，
并重启 Odin/extnav 使两者加载同一配置；既有 Tag 校准需按现有流程重新建立。

用另一圈独立数据评估同一候选，不能重拟合后再称为独立验证：

```bash
.venv/bin/python scripts/diagnostics/calibrate_odin_turntable.py fit \
  --input agent/codex/odin-turntable/turn-2.jsonl \
  --candidate agent/codex/odin-turntable/fit-1/candidate.yaml \
  --output agent/codex/odin-turntable/check-2
```

这是离线预测。实际更新并重启后应再录一圈，检查 `streams.highfreq.before` 和
`streams.vision.before` 的实测峰峰值及径向RMS；生产采集的配置快照必须是当时实际加载的版本。
程序用原始IMU与标准FCU同时间戳配对核对配置，不能配对或不一致会明确告警。

## 数学与边界

固定 `pitch_cam`，沿用驱动 `R_OF=R_OI*Ry(-pitch_cam)`，对原始IMU位置拟合：

```text
p_IMU.xy = center.xy + drift.xy*t + (R_OF*[odin_x, odin_y, 固定odin_z]).xy
```

常量中心、缓慢线性漂移作为拟合辅助量，候选仍只更新水平杆臂。
**输出波动统计不扣除拟合漂移**，不会把漂移隐藏成校准成功。
highfreq 的预测减去 `R_OF*杆臂增量`；vision 使用自身最终姿态旋转同一增量，
兼容固定 Tag yaw 修正。原点随参数变化产生的常量差值不影响稳定性指标。

- 单次水平旋转不能可靠独立标定 Z、全部安装角、机械转心偏差或设备定位畸变；这些量不自动学习。
- 转台轴未对准 FCU 时，计算得到的是转台中心外参；恒定转速下设备位置/姿态延迟也可能混入杆臂。
  最终宜用反向或不同速度再转一圈，检查候选是否仍有效。
- 缺少转角覆盖会告警；完全静止、秩不足或混合设备会话不能生成有效拟合。
  Tag 修正切换、定位跳变、地图变化会影响评估，应使用同一次稳定定位会话。
- 参考点和数学验证通过不等于实飞验收。工具不修改固件、设备SLAM、地图、TF或飞行控制算法。
