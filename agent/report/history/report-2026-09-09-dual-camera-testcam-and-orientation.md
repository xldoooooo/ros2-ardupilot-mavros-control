# 双摄像头 testcam 修复与下视安装方向检查简报

日期：2026-09-09
目标机：`nvidia@192.168.112.169`（Ubuntu 24.04 / ROS 2 Jazzy）

## 安全边界

- 全程未解锁、未起飞，也未发送任何飞控或 MAVROS 命令。
- 仅检查 USB/V4L2 状态、抓取静态画面并启动人工预览。
- 检查前摄像头节点无占用；无界面验证退出后确认节点均已释放。

## 根因

1. `/home/nvidia/testcam.py` 强制使用 Windows 专用 `cv2.CAP_DSHOW`。Linux 上该后端无法打开
   V4L2 设备，所以原脚本一个摄像头也打不开。
2. 两台 Wasintek 型号和序列号完全相同，均报告序列号 `00.00.01`。udev 的
   `/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-*` 名称冲突，当前只指向后接入的一台，
   因而不能用 by-id 区分两台设备。
3. 每台相机各暴露两个节点：`video-index0` 是 `Video Capture` 图像节点，`video-index1` 只有
   `Metadata Capture`。本次枚举中 `/dev/video0`、`/dev/video2` 可采图，`/dev/video1`、
   `/dev/video3` 不能作为 OpenCV 图像源。
4. `qv4l2` 单个进程只操作一个设备，默认启动落到第一台。需要分别指定两个 capture 节点并打开
   两个实例；不能把 metadata 节点当作第二台相机。

## 当前接线与画面

| USB 物理路径 | 本次节点 | 连接方式 | 实拍内容 |
| --- | --- | --- | --- |
| `platform-3610000.usb-usb-0:2:1.0-video-index0` | `/dev/video0` | Jetson 直连 | 前方/侧方工作区；远处检测到 ID 18 |
| `platform-3610000.usb-usb-0:1.2:1.0-video-index0` | `/dev/video2` | Genesys Logic 扩展坞 | 下视地面；清楚检测到 tag36h11 ID 0 |

两路位于同一 USB 2.0 根总线上，但显式使用 MJPEG 后已同时成功采集
`1920×1080 @ 30 FPS`。因此本次故障不是扩展坞断连、供电不足或并发带宽耗尽。原始 YUYV 双路
1080p/30 会远超 USB 2.0 带宽，测试脚本必须先选择 MJPEG。

## 修改

飞机 `/home/nvidia/testcam.py` 已完成以下调整：

- 使用 Linux `cv2.CAP_V4L2`，移除 `CAP_DSHOW`；
- 优先枚举 `/dev/v4l/by-path/*-video-index0`，解析真实节点并去除 Jetson `usb/usbv2` 重复链接；
- 不再盲扫 `0..9`，从源头排除 metadata 节点；
- 按 `MJPG -> 1920×1080 -> 30 FPS` 顺序配置，保留两个句柄验证真正并发；
- 首帧增加有限预热，打印实际生效编码、分辨率、帧率和物理路径；
- 两个窗口以 960×540 并排显示，`q` 或 `Esc` 统一退出；
- 新增 `--no-gui --snapshot-dir DIR`，可在 SSH/服务环境中做抓帧自检；
- 原文件备份为 `/home/nvidia/testcam.py.bak-20260909-1850`。

## 验证结果

- 本地与目标机 `python -m py_compile`：通过。
- 目标机执行 `python3 /home/nvidia/testcam.py --no-gui --snapshot-dir <临时目录>`：退出码 0。
- 输出确认 `/dev/video0`、`/dev/video2` 均实际工作在 `MJPG 1920×1080 @ 30.0 FPS`。
- 两路并发抓帧：通过，分别生成有效 JPEG。
- 无界面进程退出后 `fuser /dev/video0 /dev/video2`：均无占用。
- 桌面预览已作为临时用户服务 `testcam-preview.service` 启动；验证时两个 capture 节点均由同一
  `testcam.py` 进程持有。
- 摄像头固件的 MJPEG 尾部含约 300 字节额外数据，OpenCV/libjpeg 会输出
  `Corrupt JPEG data: ... extraneous bytes before marker 0xd9`，但连续解码、显示和保存均成功；
  本次未把该非致命固件兼容告警伪装为零告警。

## 下视相机方向结论

OpenCV AprilTag 36h11 检测器在 `/dev/video2` 画面中识别到 ID 0。解码后的 Tag 标准上边缘由角点
`(902, 464)` 到 `(926, 751)`，其中点位于 Tag 中心右侧，即 Tag 的“上方”在当前图像中朝右，
而不是朝图像上方，安装绕光轴偏转约 90°。

调整方式：站在地面、面向下视相机镜头向上看，将相机本体绕镜头光轴**顺时针旋转 90°**。
等价的验收目标是让预览中当前位于画面右侧的 Tag 上方转到画面上方。机械调整后必须重新抓一帧，
再次以 ID 0 标准角点顺序确认；本次没有代替用户进行任何实体拆装。

## qv4l2 使用方式

先退出 `testcam.py` 释放设备，再开两个终端分别执行：

```bash
qv4l2 -d /dev/v4l/by-path/platform-3610000.usb-usb-0:2:1.0-video-index0
qv4l2 -d /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0
```

两边都应在格式设置中选择 MJPEG 1920×1080、30 fps。当前 `/dev/videoN` 映射可能在拔插或重启后
变化，因此长期操作应使用上述 by-path，而不是记死 `/dev/video0`、`/dev/video2`。
