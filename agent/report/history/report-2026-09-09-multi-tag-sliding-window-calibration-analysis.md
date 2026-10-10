# 多 Tag 滑窗标定数学分析与三维动画简报

日期：2026-09-09

## 任务结论

本次完成双 Tag/多 Tag 跨位置标定的数学审计、离线数值场景和 Python 三维动画，没有修改生产
`correction_service`，没有连接、解锁或起飞实机。

用户提出的方向可行，但不能在飞机没有精准飞到两个 Tag 正上方时直接比较“飞机航迹”和
“Tag0–Tag1 连线”。正确观测量是每个 Tag 中心在 Odin 固定局部坐标系中的位置：

```text
a_i = xy(T_OI_i T_IC T_CT_i [0,0,0,1]^T) = q_i + r_i^O
```

再把 `a_i` 与已测的 Tag 世界中心 `s_i` 做二维刚体配准：

```text
s_i = R(psi) a_i + t
```

两点时：

```text
psi = angle(s_1-s_0) - angle(a_1-a_0)
t   = mean_i(s_i - R(psi) a_i)
```

其中相机相对偏移 `r_i^O` 自动补偿起飞点不在 `(0,0)`、两次飞机不在 Tag 正上方和机体朝向
变化。机体系自身不是固定地图系，必须通过同步的 `T_OI_i` 把相机相对向量旋转到 Odin 系。

## 交付物

- `agent/codex/two-tag-sliding-window-calibration.md`：完整坐标定义、两点闭式解、增量形式、N 点
  加权 Kabsch 解、精度下限和生产建议；
- `agent/codex/two_tag_calibration_animation.py`：离线数值自检及三维 MP4/GIF 生成脚本；
- `agent/codex/two_tag_calibration_animation.mp4`：H.264 三维动画；
- `agent/codex/two_tag_calibration_animation.gif`：便于直接预览的动画。

`agent/codex/` 按项目规范属于本地过程目录，并由 `.gitignore` 排除；本报告和 `MEMORY.md` 维护
长期结论。

## 数值验证

命令：

```bash
./.venv/bin/python agent/codex/two_tag_calibration_animation.py --check-only
./.venv/bin/python -m py_compile agent/codex/two_tag_calibration_animation.py
```

离线场景结果：

- Tag 世界基线：6.324555 m；
- 首次单 Tag yaw 误差：0.500000°；
- 双 Tag 中心配准 yaw 误差：0.120000°；
- 首次修正后的第二端点误差：0.056663 m；
- 双 Tag 修正后的第二端点误差：0.016938 m；
- 飞机相对 Tag0/Tag1 水平偏离：0.448219 m / 0.777817 m。

动画按 220 帧、24 fps、1206×756 渲染，时长约 9.17 秒。抽帧检查确认包含：首次粗校准、
非正上方飞行、Odin/世界 Tag 基线向量比较、增量旋转与重拟合平移、最终对齐五个阶段。

## 精度判定

随机位置误差近似满足：

```text
sigma_psi ~= sqrt(2) sigma_p / L
```

5 m 基线若要达到 0.2°，每个 keyframe 的横向误差约需不高于 12.3 mm；达到 0.1°约需不高于
6.2 mm。若 keyframe 横向误差为 20 mm，则 0.2°/0.1°分别约需 8.1 m/16.2 m 基线。

该公式只描述独立随机误差。Tag 中心测量、相机外参、PnP 位置随视角偏差、同步误差、Odin
跨基线漂移等系统项必须通过测量级实验单独验收，不能仅凭动画或同一 Tag 重复帧声称达标。

## 未实现边界与风险

- 生产 `correction_service` 仍明确拒绝同帧多 Tag，也没有跨航点 keyframe 滑窗；
- 两点只能给出一个基线，无法可靠识别错 Tag/离群点，生产建议至少三个空间分离 keyframe；
- 同一 Tag 停留段的多帧高度相关，应先汇总为一个带协方差的 keyframe，不能虚增独立样本数；
- 更新 active `世界<-Odin` 会给 ArduPilot 外部定位造成坐标跳变；当前 `PoseStamped` 无 reset
  counter，在 reset 通知或经验证的过渡策略完成前不能直接作为可实飞方案；
- 本次未做真实相机、Odin 数据和测量级 Tag 场地实验，因此 0.1–0.2°只是在设定噪声的离线示例
  中达到，不是实机精度验收结果。
