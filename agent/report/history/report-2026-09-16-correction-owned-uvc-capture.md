# correction_service 自有 UVC 采集与 refresh 静态标定验证

本报告记录跨项目相机依赖修复、实际测试结果及未完成的应用边界；不代表相机精度验收。

## 实现范围

- 新增本包 `uvc_capture.py` / `uvc_camera_node.py`，直接打开系统 `/dev/v4l/...` UVC 设备。
  使用 Linux V4L2 单平面 mmap MJPEG，OpenCV 解码为原始 `mono8`，保持图像尺寸、方向、topic、
  frame_id 和按需采集生命周期。不需要联合标定目录、外部 ROS 相机包或 Jetson 专有解码插件。
- 时间戳直接取 V4L2 `CLOCK_MONOTONIC` 缓冲区时间，按实际帧龄映射到 ROS 系统时钟。
  不从声明 120 fps 推算实际 60 fps 图像的时间，不把失效时间戳改写成到达时间；继续执行
  原有 200 ms 帧龄限制，包含解码开销。有限排空积压队列，只解码最新完整帧。
- 新入口为 `ros2 run correction_service uvc_camera_node`。活动配置、UQ212/Wasintek 档案均
  切换到本包驱动；遗留外部驱动配置明确拒绝。启动脚本、安装器、systemd 模板和环境示例移除
  camera overlay；ROS 与本项目 `local_setup.bash` 即可运行，避免加载历史 underlay。
- 更新相应部署说明与回归测试。依赖使用已有 ROS/OpenCV/NumPy/V4L2 工具及 Python 标准库，
  无新增第三方依赖、独立 C++ 包或构建步骤。
- 未修改 `node.py`、`geometry.py`、检测/同步/滑窗算法、extnav、飞控、视频服务、Tag 地图、
  内参、外参、镜头参数或质量/跳变门限。外参 `source_path` 保留历史溯源字符串；加载器不读取
  那个外部路径。用户已有 README/TODO/部署文档移动等工作树改动未纳入本任务。

## 地面与 refresh 验证

- 地面 ROS Jazzy 构建通过；8 个 correction 相关测试文件共 **71 passed**；包内 **5 passed**。
- refresh 原生构建通过；相机/配置专项 **22 passed**；包内 **5 passed**。
- 新测试使用本机 C 编译器和系统 `videodev2.h` 独立比对 ctypes 结构尺寸、字段偏移及 ioctl
  请求值，amd64/aarch64 均通过；覆盖实际 60 Hz 连续 20 秒与声明 120 Hz 不一致、真实旧帧、
  无效/未来时间戳、初始化失败释放、STREAMOFF 失败释放以及拒绝外部驱动配置。
- 地面无本地 UVC 摄像头，硬件流程通过地面真实 `CorrectionPanelWindow` / `CorrectionPanelClient`
  连接 refresh 执行。Qt 使用 offscreen，但实际点击正式“开始首次校准”按钮，使用生产 DDS
  端点，未使用模拟相机或模拟标定结果。

共完成六次 Tag0 首次 dry-run；每次之后清空本次测试窗口，不改变 extnav。最终两次结果：

| 指标 | f714e836ee6d | 7656d4c05258 |
| --- | ---: | ---: |
| 结果 | succeeded / window_saved | succeeded / window_saved |
| 总时长 | 7.067 s | 7.139 s |
| 接收图像 | 232 | 220 |
| 检测/处理帧 | 24 / 24 | 24 / 24 |
| 接受/拒绝样本 | 24 / 0 | 24 / 0 |
| 候选 x / y | -0.794432 / +0.107190 m | -0.794250 / +0.107430 m |
| 候选 yaw | -108.475525° | -108.478641° |
| tilt | 3.152661° | 3.077012° |
| Odin 匹配误差 | 2.079 ms | 2.292 ms |
| 时间源 | arrival_history | arrival_history |
| 处理频率 | 8.321 Hz | 8.321 Hz |
| 资源释放 | true | true |

前两次任务为 `8d7a790c8615` / `b5e3aad99cf3`，分别 6.868 / 8.747 秒，同样 24 accepted、
0 rejected。中间两次 `fb717fc6abe8` / `db159b2885e7` 为 8.965 / 9.038 秒。
镜头设置及读回沿用既有流程。六轮均获得实际 Tag0，旧“零首帧”故障未再出现。

refresh 任务日志位于：

- `correction_service/log/job-20260916-232800-8d7a790c8615.jsonl`
- 其他任务按上述 job ID 查询相同目录中的 JSONL/camera.log。

地面过程证据保存在 `agent/codex/correction-uvc-20260916/`：正式 Qt 面板快照、各轮 JSON、
状态迁移和采集日志。该目录按项目规则保留本地，不提交运行产物。

## 独立采集及退出检查

在 refresh 的 `env -i` 清空环境下只加载系统 ROS 与主项目 local setup，外部
`wasintek_gst_camera` 查询返回不存在；本包 UVC 节点仍可直接采集。最终 25 秒探针的稳定
速率为 60.10 Hz，平均采集到解码帧龄约 25.64～26.28 ms；启动阶段最大 156.34 ms，后续
5 秒分段最大约 30.50～30.99 ms。`stale_dropped=0`、`nonmonotonic_dropped=0`，无按
120 fps 推算导致的累计延迟。

退出调试曾发现 ROS/进程组重复 SIGINT 可在 STREAMOFF 中再次抛中断，以及后台 shell 会
继承忽略 SIGINT。最终入口显式恢复 Python SIGINT 处理、关闭 ROS 的异步信号处理，并在
清理阶段忽略重复 SIGINT。后台启动后连续发送两次 SIGINT 的最终探针退出码 **0**，无
traceback；设备 fd/mmap 均释放，`fuser /dev/video0` 无占用。

## 未实现或未验证的指标

- **未应用修正。** 六次候选的 yaw 修正约 -108.46°，服务明确报告 `can_apply=false`、
  “当前姿态预计 yaw 跳变超限”，超过原有 45°上限。没有放宽门限或修改算法；extnav 始终
  `valid=false / revision=0`。这个角表示 Odin 与 Tag 世界系的修正，不能当作飞机物理偏航角。
  因此本次通过采集、Tag 检测、首次收敛、窗口保存和资源释放，未完成应用 ACK/EKF 跟随验收。
- 当前 MAVROS/飞控服务在测试前即未运行，仅 Odin/extnav/独立 correction 在线；没有启动
  飞行链来制造完整系统验收结论。飞机一直按用户给定条件静止桌面，未发送解锁、起飞或模式指令。
- UQ212 仍使用既有理论内参和沿用外参；单 Tag 静止收敛不等于绝对位置/航向精度已验证，
  更不构成多 Tag、动态同步、长时间高负载或 Wasintek 实物验收。

## 部署与同步

- 只更新 refresh 的 correction 相关源码、安装产物、配置及独立 unit，原 Odin PID 21114、
  extnav PID 21278 未改变。独立修正服务保持测试前的 active/disabled 策略，窗口为空、相机
  无占用。未改联合标定目录。
- refresh 定点备份：
  `/home/nvidia/ros2-ardupilot-mavros-control/agent/codex/correction-uvc-20260916/predeploy.tar.gz`，
  SHA-256 `c85a22653bbd5e313436d27e6a76b4c6a04e1a33ebde444e4a394736891b9e1b`。
- 地面与 refresh 已同步；drone-new 和独立 USB Jetson 未连接、未同步，已写入 MEMORY 的
  明确待同步项。后续同步须一起更新本包、相机 driver 配置和 unit/启动入口。
