# 地面站本机摄像头 MediaMTX 故障修复简报

## 故障结论

面板报出的仓库内 `video_service/bin/mediamtx/mediamtx` 路径并非当前代码仍有旧依赖，而是删除
仓库二进制以前启动的 `camera_service.py serve` 后台仍在运行，进程内存保留了旧默认路径。
同时，本机尚未安装改造后的系统依赖 `/usr/local/bin/mediamtx`，所以仅重启后台仍会因缺少系统
二进制失败。

## 已完成修复

- 通过 Socket `shutdown` 正常退出旧后台，确认 8554 端口与摄像头设备均未被残留进程占用；
- 从官方 v1.20.0 发布下载 amd64 归档，SHA-256 校验为
  `952d5f7d31d1b448ab4da4509550594c511d42636db9d7bb175d377f4ede81df`，安装到
  `/usr/local/bin/mediamtx`，`file` 确认为 x86-64，版本输出为 `v1.20.0`；
- `setup_ground_station.sh` 新增 FFmpeg、ffprobe、v4l2-ctl、固定 MediaMTX 路径及本机可执行性
  检查；根 README 与视频 README 补齐地面站 amd64 的下载、校验、安装和旧后台处理步骤；
- 没有修改用户已经保存的本机摄像头配置，也没有连接、解锁或起飞真机。

## 真实摄像头闭环

使用当前 Wasintek USB 摄像头和用户现存的 MJPEG 1920×1080@30 配置完成：

1. 摄像头后台进入 `running`，MediaMTX 与 FFmpeg 正常启动；
2. FFprobe 从 `rtsp://127.0.0.1:8554/camera` 读到 MJPEG、1920×1080、30/1；
3. FFmpeg 经 RTSP/TCP 连续解码 60 帧成功；
4. 人工抓拍生成 1920×1080 JPEG；
5. 停止后录像正常封装为 34.9 秒 MP4，MediaMTX、FFmpeg、8554 监听和摄像头占用均释放；
6. 摄像头 Socket 后台保留在 `stopped`，面板可直接再次开启。

本次测试媒体位于：

- `/home/nvidia/Videos/ros2-ardupilot-camera/recording-20260820-020751-410519.mp4`
- `/home/nvidia/Pictures/ros2-ardupilot-camera/snapshot-20260820-020755-461434-manual-1.jpg`

## 测试结果

- `bash -n setup_ground_station.sh`：通过；
- `pytest -q tests/test_onboard_deploy.py tests/test_camera_service.py`：45 passed；
- source ROS Jazzy 与本工作区 overlay 后，`pytest -q tests`：164 passed；
- 无范围的仓库根 `pytest` 会额外收集 `integration/websocket_test_demo`，因该演示目录未配置
  `ws_demo` 导入路径而在收集阶段失败；正式 `tests/` 测试集不受影响，本次未擅自修改 demo。
