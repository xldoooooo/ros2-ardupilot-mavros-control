# 真机摄像头 MJPEG 120 FPS AVI 录像脚本简报

## 任务结果

- 新建 `/home/xld/Project/python_demo/demo2_save_fix_avi.py`。
- 原 `demo2.py`、`demo2_save.py` 和 `demo2_save_fix.py` 均未修改。
- 摄像头固定使用已验证节奏稳定的 1280×720@120 FPS MJPEG 输出。
- GStreamer 录像分支通过 `jpegparse` 和 `avimux` 将摄像头 MJPEG 原码流直接封装为 AVI，
  不解码、不重新编码，因此不增加录像画质损失，也不受 Python预览速度限制。
- 独立预览分支的 `appsink` 只保留最新一帧；Python/OpenCV 使用 `imdecode()` 解码预览，保留原
  `demo2.py` 的窗口和按 `q` 退出方式。
- 输出文件位于脚本同目录，名称为 `demo_2_fix_avi_YYYYMMDD_HHMMSS.avi`。
- 正常结束时发送 EOS，让 `avimux` 写完 AVI索引。

## 设计选择

- 同一摄像头的 MJPEG 原始缓冲实测平均 119.77 FPS，中位帧间隔 8.43 ms，99%帧间隔不超过
  9.81 ms，大于12.5 ms的间隔为0；明显优于其原生 H.264 输出的成对扎堆。
- Jetson `nvjpegdec` 能把该摄像头 JPEG 解码为 NVMM Y42B，但当前 `nvvidconv` 不能把这一路
  协商为 OpenCV所需 BGRx；强行使用会导致 `not-negotiated`。最终只在预览分支使用
  OpenCV JPEG解码，录像仍为原码直封装。
- 预览解码不足120 FPS时，`appsink drop=true/max-buffers=1` 丢弃旧预览帧，不反压录像分支。

## 验证结果

- 真机 Python语法编译及依赖导入通过。
- 纯 GStreamer 录像/预览分流先以240帧验证：2.126秒正常 EOS，AVI大小22,596,392字节。
- 最终脚本完整真机测试：AVI编码标识 `MJPG`，1280×720、120 FPS，共601帧，播放时长
  5.0083秒，文件56,673,588字节，码率约90.53 Mbps；601帧全部可解码，验证通过。
- 无窗口自动测试中的预览 JPEG解码循环为578帧/5.0035秒，约115.32 FPS；录像仍完整保存
  601帧，证明预览未限制录像。
- 该码率对应约11.3 MB/s、约680 MB/min、约40.7 GB/h，长时间录像前必须规划磁盘容量。
- OpenCV/FFmpeg回读摄像头原始 MJPEG AVI时会打印
  `unable to decode APP fields`，属于摄像头 JPEG厂商 APP元数据兼容性告警；本轮全部601帧仍
  成功解码，未观察到数据帧丢失。其他播放器兼容性仍应以实际目标播放器复验。
- 最终 SHA-256：`demo2.py` 为
  `4e94df16f007a62d1c25b8481a4ea56a8af333ba24bb03a831eda823728d2f95`，`demo2_save.py` 为
  `882c347073f1053c3903d8870d0d758a10a77e5ff2947cce476fd845c6cf6b73`，`demo2_save_fix.py`
  为 `761af08c3cad07f5edb28fd1b16752bf99cd1797f255e335d2df826e32b8a95c`，新脚本为
  `e098089b8fa9bb7d0bcf0c835f801b16342cd6f682344d45e4daf6d9211524f0`。
- 测试后 `/dev/video0` 无占用，两个 `/tmp` AVI测试文件均已移入真机回收站，无测试进程残留。

## 使用方法

```bash
cd /home/xld/Project/python_demo
python3 demo2_save_fix_avi.py
```

预览窗口中按 `q` 正常结束并完成 AVI索引写入。

## 范围与限制

- 自动测试执行了真实摄像头采集、MJPEG原码封装、OpenCV预览解码和 EOS退出，但把窗口绘制
  替换为空操作；真机桌面窗口仍沿用已工作的 OpenCV方式。
- 按用户要求采用最小实现；断电或 SIGKILL 无法执行 EOS时，AVI索引可能来不及写完。
- 本任务未启动、停止或修改 MAVROS、Odin、extnav、机载控制服务或飞控；未解锁、起飞或发送
  任何飞行命令。
