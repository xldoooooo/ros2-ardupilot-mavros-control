# 真机摄像头 120 FPS 录像加速问题修复简报

## 任务结果

- 新建真机脚本 `/home/xld/Project/python_demo/demo2_save_fix.py`。
- 原 `/home/xld/Project/python_demo/demo2.py` 和 `demo2_save.py` 均未修改。
- 新脚本基于原 `demo2.py` 保留摄像头 0、预览窗口和按 `q` 退出的基本操作，不加入业务层健壮性
  设计。
- 摄像头使用其原生 1280×720、120 FPS H.264 输出；录像分支只执行 `h264parse` 和 `mp4mux`，
  不再把 MJPEG 解码为 BGR 后用 CPU 同步执行 `mp4v` 二次编码。
- GStreamer `tee` 将录像和预览分离。录像直接封装原生 H.264；预览使用 Jetson
  `nvv4l2decoder`/`nvvidconv` 硬件链路解码，`appsink` 只保留最新一帧。
- `v4l2src do-timestamp=true` 使用真实采集时间戳，MP4 不再把每秒约 28 帧错误标记为固定
  120 FPS；按 `q` 或正常异常退出时发送 EOS，让 `mp4mux` 写完索引。
- 输出文件位于脚本同目录，名称为 `demo_2_fix_YYYYMMDD_HHMMSS.mp4`。

## 问题根因

- 旧 `demo2_save.py` 在同一个循环内顺序执行摄像头读取、CPU `mp4v` 编码和窗口显示。
- 真实样本约 6.99 秒仅写入 194 帧，实际约 27.74 FPS，却按 120 FPS 封装，导致播放器仅显示
  1.617 秒，约 4.33 倍加速。
- 摄像头能力枚举确认其原生支持 1280×720 H.264/MJPEG 120 FPS，因此采用 H.264 原码流直封装，
  消除软件重编码瓶颈和二次有损压缩。

## 验证结果

- 真机 Python 语法编译和依赖导入通过；GStreamer 1.20.3、PyGObject、OpenCV 4.5.4、NumPy，
  以及 `v4l2src`、`h264parse`、`mp4mux`、`nvv4l2decoder`、`nvvidconv`、`appsink` 等插件均可用。
- 原生 H.264 单录像分支：240 帧约 2.14 秒完成，管线正常 EOS。
- 录像/硬件预览完整分支：240 帧约 2.17 秒完成，管线正常 EOS。
- 修改后脚本较长实测：539 帧、4.5271 秒、平均 119.06 FPS。
- 最终源码实测：H.264 (`avc1`)、1280×720、374 帧、3.1615 秒、平均 118.30 FPS，文件
  8,066,899 字节；帧数、时间戳和时长校验通过，未再出现加速播放。
- 最终 SHA-256：`demo2.py` 为
  `4e94df16f007a62d1c25b8481a4ea56a8af333ba24bb03a831eda823728d2f95`，`demo2_save.py` 为
  `882c347073f1053c3903d8870d0d758a10a77e5ff2947cce476fd845c6cf6b73`，新
  `demo2_save_fix.py` 为
  `761af08c3cad07f5edb28fd1b16752bf99cd1797f255e335d2df826e32b8a95c`。
- 验证后 `/dev/video0` 无占用；四个 `/tmp` 测试视频均已移入真机回收站，无测试进程残留。

## 使用方法

```bash
cd /home/xld/Project/python_demo
python3 demo2_save_fix.py
```

预览窗口中按 `q` 正常结束并完成 MP4 索引写入。

## 范围与限制

- SSH 自动测试运行了完整采集、H.264 直封装、硬件解码、缓冲映射和退出链路，但将 OpenCV
  窗口函数替换为空操作，未在远程桌面上实际绘制窗口；原 `demo2.py` 的窗口操作方式保持不变。
- 脚本依赖当前 Jetson 和 Wasintek 摄像头已经验证存在的原生 H.264 120 FPS 与 NVIDIA GStreamer
  插件，不作为其他硬件平台的通用实现。
- 按用户要求采用最小实现；断电或 SIGKILL 无法执行 EOS 时，当前 MP4 仍可能来不及写完索引。
- 本任务未启动、停止或修改 MAVROS、Odin、extnav、机载控制服务或飞控；未解锁、起飞或发送
  任何飞行命令。
