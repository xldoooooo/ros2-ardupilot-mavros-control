<!-- UQ212 同板内参标定：采集适配、机载入口、结果格式和应用边界。 -->
# UQ212 相机内参标定

从 refresh `/home/nvidia/camera_int_calib/wainstek_cam_calib.py` 改编。
旧脚本、旧 `camera_calibration.yaml` 和 Wasintek 档案保持原样。

## 与旧程序的区别

- 从活动 `correction_service/config/camera.conf` 读取 UQ212 稳定 by-id 路径，不使用 video0。
- 使用项目原生 UVC mmap 采集：MJPEG 1920×1080@120。逐次排空旧帧，只处理最新帧；
  120 是设备模式，不代表 AprilGrid 检测达到 120 Hz。检测和求解均使用原始全分辨率。
- 开流后按活动 `lens.conf` 设置并读回镜头参数；与生产链的对焦、变焦等状态一致。
  不引入旧相机不支持的控制，也不将采集请求改成设备未声明的 30 fps。
- 兼容项目环境 OpenCV 4.6 的旧 ArUco API 和新版本 API；不需升级系统 OpenCV。
- 标定完整估计 fx、fy、cx、cy、k1、k2、p1、p2、k3，不固定为零畸变，也不使用理论 89°内参作真值。
- 每批独立目录，保存原始样本图、求解用角点、每视图残差以及两种 YAML；不筛掉高残差帧。
  `R` 新建批次，旧结果仍保留；不自动写入生产内参。

## 同一块标定板

沿用原脚本的 6×6 AprilGrid：tag36h11、ID 0～35、Tag 黑框边长 55 mm、净间隙 16.5 mm，
OpenCV `markerBorderBits=2`。ID 从左下角 0 起向右、向上递增，保留原角点顺序。
不需要换板，不要改成单个 17 cm 定位 Tag 的尺寸。板应保持平整，尺寸指黑框外缘而非白边。

## 在 refresh 上运行

通过 NoMachine/本机桌面终端运行（需要图形会话），保持修正面板不在采样状态，不同时打开另一
个下视相机程序。程序遇到相机已占用会报错，不会停止其他服务。

```bash
cd /home/nvidia/camera_int_calib
./run_uq212_calib.sh
```

机载这两个新增入口是仓库文件的软链接：
`uq212_cam_calib.py`、`run_uq212_calib.sh`。启动器使用主项目 `.venv/bin/python`。
也可以在主项目运行 `./tools/camera_int_calib/run_uq212_calib.sh`。

无需图形界面的采集检查（实际开流、恢复镜头参数、读 30 帧后释放设备，不做标定）：

```bash
/home/nvidia/camera_int_calib/run_uq212_calib.sh --check-camera
```

现有依赖：项目 `.venv` 的 OpenCV（含 aruco 和 GUI）、NumPy、PyYAML，系统 `v4l2-ctl` 和
`fuser`。本次不引入新的 pip 包，也不修改 ROS/Odin 的 OpenCV 环境。

## 采集过程与按键

默认自动收集 30 张，至少 15 张可按 C 提前求解。保留旧脚本的清晰度、至少 10 个 Tag、
板面占比、稳定性和姿态去重门限。让板覆盖中央、四边和四角，变换距离并加入明显俯仰/侧倾；
每个姿态停稳约一秒。只有平行正视、中央区域或重复姿态的数据不适合可靠标定畸变。
照明充分，避免反光和移动模糊；不要在采集中改变对焦/变焦。

- A：切换自动采样。
- 空格：强制采纳当前原始画面的检测，绕过质量/去重门限，谨慎使用。
- C：至少 15 张时使用全部已采样视图求解，不自动剔除高残差视图。
- U：标定后切换原图/去畸变预览。
- S：保存本批结果。
- R：开始新批次，不覆盖先前批次。
- Q / Esc：退出并释放相机。

## 输出及预览

结果位于 `/home/nvidia/camera_int_calib/uq212_runs/<时间戳>/`：

- `view-000.png` 等：实际参与采样的原始图像。
- `observations.npz`：进入标定时保存的全部三维点/二维角点，可不依赖重新检测而重算。
- `camera_calibration.yaml`：OpenCV FileStorage 格式，包含 K/D、总 RMS、各视图 RMS、
  参数标准差和标定板规格。
- `intrinsics.yaml`：修正服务可加载的普通 YAML，包含原始图像对应的 K/D、RMS、采集模式与
  镜头参数。状态为 `calibrated_pending_independent_validation`。

两种 YAML 都可预览，例如：

```bash
/home/nvidia/camera_int_calib/run_uq212_calib.sh \
  --preview-yaml /home/nvidia/camera_int_calib/uq212_runs/<时间戳>/intrinsics.yaml
```

预览可能使用新的去畸变投影矩阵，但保存的始终是原始图像的 K/D；不会把预览用 new_K
错误写入生产配置，也不会接受非 1920×1080 的 YAML 后静默缩放。

标定完成后应检查每视图残差、整幅边缘直线和未参与标定的新姿态；低训练 RMS 不等于独立
精度验收。通过后再将 `intrinsics.yaml` 同步到活动及 UQ212 档案、安装目录并重启修正服务，
重新采样定位修正。本程序不会自动执行这些步骤，也不会覆盖 Wasintek 文件。
