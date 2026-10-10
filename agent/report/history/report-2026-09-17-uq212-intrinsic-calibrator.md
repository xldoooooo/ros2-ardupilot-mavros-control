<!-- 同板 UQ212 内参标定程序适配、refresh 部署与验证范围。 -->
# UQ212 同板内参标定程序

日期：2026-09-17。用户要求检查 refresh 的旧内参程序，并使用同一块标定板编写新程序。

## 来源与适配

实际旧目录为 `/home/nvidia/camera_int_calib`；原脚本 `wainstek_cam_calib.py`，
旧结果 `camera_calibration.yaml`。旧 README 中 camera_calib/realtime_camera_calibration.py
属于历史名称，不代表当前入口。

保留原板 6×6 tag36h11、ID0~35、55mm 黑框、16.5mm 净距、两位黑边及已验证的角点次序。
保留自动30张/至少15张、清晰度/稳定性/覆盖/姿态去重、手动强制采样及不删帧求解的流程。

新源码：`tools/camera_int_calib/uq212_cam_calib.py`。

- 稳定 by-id 路径取自活动相机配置，固定 UQ212 原生 MJPEG 1920×1080@120。
- 复用项目 UVC mmap/latest-frame 模块，不使用旧脚本的 video0 和 30fps 请求。
- 开流后共用生产镜头控制写入/读回逻辑，保留对焦和变焦状态；不操作其他服务。
- 兼容项目 OpenCV 4.6（没有 ArucoDetector）及新版 API，无新依赖安装。
- 使用原生分辨率角点估计 fx/fy/cx/cy 和 k1/k2/p1/p2/k3，不强制零畸变。
- 每次启动/重置创建新批次，保留原始 PNG、求解角点 NPZ、全部每视图残差。
- 输出 OpenCV YAML 和修正服务可直接解析的普通 intrinsics.yaml，后者保留真实原图 K/D，
  不是去畸变预览的 new_K。不会自动应用生产配置或宣称独立精度已验收。
- 预览分辨率必须一致；相机占用/采集超时/模式不符/控制读回失败会报错，退出释放设备。

## refresh 部署

工具已随 main 同步，机载新增软链接：

- `/home/nvidia/camera_int_calib/uq212_cam_calib.py`
- `/home/nvidia/camera_int_calib/run_uq212_calib.sh`

启动器使用主项目 `.venv/bin/python`。GUI 应从 refresh NoMachine/本机桌面终端运行。
结果默认在 `/home/nvidia/camera_int_calib/uq212_runs/<时间戳>/`。

最初远端测试提示文件不存在，原因是飞机为非 cone 稀疏检出；已追加
`/tools/camera_int_calib/` 与 `/tests/test_uq212_intrinsic_calibration.py`，保留原有路径。
这是独立源码工具，不需要 colcon 构建；部署步骤已写入工具 README。
new 飞机未连接/部署，MEMORY 已标记。

## 实际验证

- 开发机相关测试 32 passed：新增两项覆盖整板 ID/角点方向、带非零畸变的多姿态参数恢复、
  全视图保留、生产加载/双格式预览与独立输出批次；另含现有修正链30项回归。
- refresh 项目 OpenCV 4.6：新增两项测试 2 passed。
- refresh `--check-camera` 实际读30帧成功，协商 MJPEG 1920×1080@120，13项镜头参数全部
  与活动配置一致。测试后设备无人占用。
- refresh `QT_QPA_PLATFORM=offscreen` 创建真实窗口并处理一帧真实相机图像、模拟 Q 退出成功。
  未执行真实桌面人工交互全流程；空 smoke 目录位于机载 agent/codex，未写生产配置。
- 独立修正服务测试前后 MainPID 均为64616、active；本次未停止/重启任何服务，未发飞行命令。
- 旧脚本 SHA-256 前后一致：
  `1894f4bffb1930e960221a57cbd4b70dc6126e689b5e85f7fad006595989aaf7`。
- 旧标定 YAML SHA-256 前后一致：
  `e0a4cf9b137a699068e50d280fbf460468089016428c0ca6507c81e3930ed578`。
- 当前生产理论内参 SHA-256 保持
  `0104bdf7972f35011cad137e724d7049099f616b34d38939959da9e5a1fc4ce6`。

## 未完成的实测指标

用户尚未持 AprilGrid 进行本轮多姿态采集，因此没有 UQ212 实测内参/RMS/独立精度结果。
合成恢复与真实读帧成功不等于已完成镜头标定；待用户覆盖画面边缘、距离和倾角采样后，
检查独立视图，再决定是否同步生产内参。未使用旧 Wasintek 数值冒充 UQ212 标定。
