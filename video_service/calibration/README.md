<!-- Wasintek intrinsic calibration entry point and production configuration contract. -->
# Wasintek 内参标定

使用原有 6×6 AprilGrid：tag36h11、ID 0～35、Tag 边长 5.5 cm、黑框净间距
1.65 cm。保持原程序的角点顺序、30 张自动采样、15 张手动求解及五参数畸变模型。

## 运行

关闭 video service 的摄像头采集后，在飞机桌面终端执行：

```bash
cd ~/ros2-ardupilot-mavros-control
python3 video_service/calibration/wainstek_cam_calib.py
```

脚本会自动进入项目 `.venv`。refresh 上也可使用同内容的现场入口：

```bash
python3 ~/camera_int_calib/wainstek_cam_calib.py
```

相机模式直接复用 video service 的配置解析器；配置路径与
`scripts/onboard/start_onboard_video.sh` 保持相同优先级：
`VIDEO_SERVICE_ONBOARD_CONFIG` / `VIDEO_SERVICE_LENS_CONFIG` 环境变量、
`/etc/ros2-ardupilot/camera.conf` / `lens.conf`、仓库 `video_service/config/`。
也可用 `--camera-config` / `--lens-config` 显式指定生产文件。

设备、编码、分辨率、帧率均来自 `camera.conf`，`device=auto` 也使用生产选择逻辑。
镜头控制来自同一份 `lens.conf`；开流稳定一秒后按生产顺序写入、等待并读回。
不单独维护标定分辨率、曝光或变焦默认值。refresh 当前生产模式为
**H.264 1920×1080@60**；如果生产改变分辨率、裁剪、对焦或变焦，须重新标定。

SSH 下可进行无窗口检查，不采集标定样本：

```bash
python3 ~/camera_int_calib/wainstek_cam_calib.py --check-camera
```

## 采样与输出

缓慢移动标定板，覆盖中心、四角和边缘，改变距离、俯仰、偏航与旋转角；每个姿态稍作停顿。
`A` 开关自动采样，空格强制加入有效角点，`C` 在至少 15 张时求解，
`R` 清空当前样本，`U` 对比去畸变，`S` 再次保存，`Q` / Esc 退出。

每次启动交互标定均分配独立输出：
`~/camera_int_calib/wainstek_runs/<时间戳>/camera_calibration.yaml`。
文件包含原始 K/D、重投影误差及采集模式、镜头参数、环境等元数据。
这是 OpenCV FileStorage 格式；不会覆盖历史结果或自动改写生产内参。
video service 当前仅存放内参档案，视频流水线尚未应用去畸变。

加载同分辨率的 OpenCV 标定 YAML 预览：

```bash
python3 ~/camera_int_calib/wainstek_cam_calib.py --preview-yaml /absolute/path/camera_calibration.yaml
```

依赖复用项目环境中的 OpenCV contrib、NumPy 及系统 `v4l2-ctl`、`fuser`。
H.264 通过 OpenCV 的 GStreamer 后端执行 `h264parse → openh264dec → BGR` 真正解码；
不能直接把 V4L2 返回的 H.264 数据当像素。解码只影响标定侧读取，不改变生产相机模式。
需要 OpenCV 构建时启用 GStreamer，以及以下插件（refresh 已具备）：

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

兼容 refresh 当前的 OpenCV 4.6 检测器 API，无需替换系统 OpenCV。
`--check-camera` 同时打印图像均值与标准差，方便排查异常黑屏。
