# Task29 下视相机镜头预设与 Tag 计数说明简报

## 任务范围与安全边界

- 解释首次校准失败画面中 `tag=0` 的真实含义，并消除地面站歧义。
- 参照 video_service 的实机时序，为 correction_service 保存一组完整下视相机参数；必须在相机
  已开始采集后写入，且检测不能使用参数切换期间的残留帧。
- 在新飞机 `drone-new` 上编译、部署并执行未武装首次校准 dry-run；不应用修正，不改飞控、Odin、
  extnav、首次标定几何或滑窗算法。
- 全程没有解锁、没有起飞、没有发送飞行控制命令。测试前后均确认 `armed=false`、`STABILIZE`、
  `controller_active=false`、`lease_active=false`。

## 截图中 `tag=0` 的含义

截图原行：

```text
recv/proc=447/137  tag=0  accept/reject=0/137
```

这里的 `tag` 不是 Tag ID，而是所有已处理帧中成功解码出的目标总次数。该行表示：收到 447 帧，
处理 137 帧，137 帧均没有解出任何 AprilTag，所以解码次数为 0、接受样本为 0、拒绝帧为 137，
最后触发“20.0s 内未识别到预期 Tag 0”。地面站现已把标签改为 `Tag解码次数=...`，避免再误读为
“识别到了 ID 0”。

## 故障证据与结论边界

飞机上截图对应的最新失败任务为：

```text
job=51e9548ecb24
frames_processed=137
detections=0
samples=0
rejected=137
```

该任务使用正确的 USB `1.2` 下视相机 `/dev/video2`、1920×1080@30，采集健康约 30 Hz，没有 stale
或时间戳倒退丢帧。任务结束后 V4L2 回读为 brightness 10、manual exposure 1、exposure 25、gain
240、zoom 10。由于失败任务没有保存实际参与检测的帧，不能从现有证据证明曝光导致失败，也不能
证明某个固定检测代码缺陷。

随后在不应用修正的隔离 dry-run `2901780b3e45` 中，同步保存 correction_service 实际订阅的 ROS
帧。保存的第 1、10、30、45、60 帧均成功解出 ID 0，重投影误差约 0.27～0.31 px；任务 24 个样本
全部接受、0 拒绝并收敛。可视帧中 Tag0 清晰且尺度足够，画面虽偏亮但生产检测器可稳定识别。

因此本轮确认的是“此前失败属于当时处理帧没有解出 Tag，现有证据不足以定位其根因”；不能把它
描述为已经证明由镜头参数导致，更不能宣称间歇失败已经根治。

## 镜头参数与生效时序

原实现实际上已经做到“首帧后写入”，但只固定少量控制项，且每写一项立即读回，日志没有保留
完整 requested/readback。现按 video_service 的实机经验做最小增强，保留 correction_service 已
验证的下视相机曝光/增益值，不盲目照搬另一相机的亮度参数：

```text
auto_exposure=1
exposure_time_absolute=25
gain=240
brightness=10
contrast=6
saturation=6
hue=0
sharpness=6
power_line_frequency=1
zoom_absolute=10
```

启动顺序固定为：

1. 启动相机进程并等待首帧，确认设备已经真正开流；
2. 再等待 1.0 秒让视频流稳定；
3. 按 `lens.conf` 文件顺序写入控制项；写 manual exposure 和 exposure value 后各等待 0.2 秒；
4. 全部写完后统一逐项读回，任一值不一致则任务失败；
5. 清空单槽图像队列，丢弃默认参数和切换期残留帧；
6. 此后才进入 Tag 采样，并把 requested/readback 与 `after_first_image=true` 写入任务 JSONL。

这保证不会出现“先写参数、开流时又被设备恢复默认值”的顺序错误，也避免仅验证控制项写入瞬间
却被后续控制项联动改变。

## 修改文件

- `correction_service/config/lens.conf`：保存完整、确定顺序的下视相机预设。
- `correction_service/correction_service/config.py`：允许新增的标准 V4L2 控制项。
- `correction_service/correction_service/camera_process.py`：分步写入、稳定等待、最终统一回读。
- `correction_service/correction_service/node.py`：首帧后应用、清空切换期帧、记录参数审计事件。
- `correction_service/correction_service/correction_panel.py`：把歧义标签改为 `Tag解码次数`。
- `correction_service/README.md`：记录参数保存、时序与审计语义。
- `tests/test_correction_service.py`、`tests/test_correction_panel.py`、
  `correction_service/test/test_correction_package.py`：锁定顺序、最终回读、配置值和 GUI 文案。

没有修改首次标定、PnP、去畸变参数传递、SE(3)、多 Tag 滑窗、extnav 或 onboard_control；没有
新增依赖和接口版本。

## 本地验证

在加载 ROS 2 Jazzy 与本仓库构建 overlay 后：

```text
targeted pytest                                  28 passed
ruff format --check                             passed
ruff check                                      passed
git diff --check                                passed
colcon build correction_interfaces/service      2 packages passed
pytest tests + correction_service/test           243 passed
colcon correction package tests                  4 passed
colcon aggregate result                          23 tests, 0 failures
```

第一次未加载生成接口 overlay 的全量 pytest 得到 222 passed、1 skipped、7 failed、13 errors；失败
和错误均是找不到生成的 `guided_interfaces`/`correction_interfaces`，不是产品断言失败。补齐标准 ROS
构建/overlay 后全量 243 项通过。

## 实机部署与验证

部署前确认飞机未武装且无控制租约。精确备份位于：

```text
/home/nvidia/backups/task29-lens-controls-before-20260915-lbH7BA
```

只停止并重启了 `odin-correction.service`，只同步 correction_service 本次涉及的源码、配置、README
和包级测试。没有重启或修改 Odin、extnav、MAVROS、onboard_control 或 video_service。飞机上仅重建
`correction_service`，ARM64 包级测试 4 passed，服务启动后为 active/running、NRestarts=0。

部署后的首次校准 dry-run：

```text
service instance=ab234859c22e4ea2af8d145f0aed4762
job=25e7b882fa68
accepted=24
rejected=0
detected Tag ID=0
reprojection RMS=0.3190 px
window=1/5, revision=1 (dry-run only)
```

任务 JSONL 明确记录 `after_first_image=true`、`stream_settle_seconds=1.0`，上述 10 个控制项的
requested 与 readback 完全一致。任务结束后再次用 `v4l2-ctl` 独立检查，各项仍与配置一致，且
`/dev/video2` 已释放。关键日志：

```text
/home/nvidia/ros2-ardupilot-mavros-control/correction_service/log/job-20260915-200628-25e7b882fa68.jsonl
```

该任务只保存 dry-run 候选，未执行 apply。最终 extnav 仍为 identity、revision 0，飞机保持
`connected=true`、`armed=false`、`STABILIZE`、controller inactive、lease inactive。

当前 `ros2-ardupilot-onboard.service` systemd unit 因用户/地面站此前终止而处于 failed/stopped，
但另一条由地面站启动的 onboard ROS 进程仍在运行且 `/onboard_control/status` 正常发布。为避免打断
用户当前会话，本轮没有重启或接管 onboardcontrol；这不影响本轮独立 correction_service 的编译、
相机和 dry-run 验证结论。

## 最终结论

- 截图中的 `tag=0` 表示零次 Tag 解码，不表示已经识别到 ID 0；GUI 歧义已修正。
- 完整下视相机参数已持久化，并经代码和实机日志证明是在开流首帧之后写入、最终统一回读。
- 部署后的同链路首次校准为 24 接受、0 拒绝，证明当前预设和时序下可以识别 Tag0。
- 此前 137 帧均失败的根因因缺少失败帧仍未定位；如再现，下一步应在失败任务内按限额保存诊断帧，
  而不是仅凭参数回读推断曝光原因。
