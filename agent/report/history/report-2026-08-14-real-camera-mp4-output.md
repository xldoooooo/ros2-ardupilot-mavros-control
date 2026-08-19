# 真机摄像头录像改为 MP4 执行简报

## 任务结果

- 复查时上次生成的脚本已由外部从 `demo_2_save.py` 改名为
  `/home/xld/Project/python_demo/demo2_save.py`，文件内容哈希与上次生成版本一致；本次修改当前
  实际文件 `demo2_save.py`，没有擅自改回文件名。
- 输出容器由 AVI 改为 MP4，输出文件名为
  `/home/xld/Project/python_demo/demo_2_YYYYMMDD_HHMMSS_ffffff.mp4`。
- 视频写入编码由 MJPG 改为真机 OpenCV/FFmpeg 已验证支持的 `mp4v`。
- 摄像头采集端仍保留原 `demo2.py` 的 MJPG 请求、120 FPS 请求、预览窗口和按 `q` 退出行为；
  只有保存格式发生变化。
- 原基础文件 `/home/xld/Project/python_demo/demo2.py` 未修改。

## 验证结果

- 真机 Python 源码内存编译：通过。
- 直接加载修改后的 `demo2_save.py` 并调用其 `create_video_writer()`：成功创建 `.mp4` 文件，
  以 `mp4v` 写入并回读 3 帧，文件大小 1824 字节。
- 验证文件位于 `/tmp`，验证完成后已删除；目标目录没有因本次测试产生 AVI 或 MP4。
- 原 `demo2.py` SHA-256 仍为
  `4e94df16f007a62d1c25b8481a4ea56a8af333ba24bb03a831eda823728d2f95`。
- 修改后 `demo2_save.py` SHA-256 为
  `882c347073f1053c3903d8870d0d758a10a77e5ff2947cce476fd845c6cf6b73`，权限 `0644`，
  所有者 `xld:xld`。

## 使用方法

```bash
cd /home/xld/Project/python_demo
python3 demo2_save.py
```

预览窗口中按 `q` 结束后，MP4 文件保存在当前目录，终端会打印其绝对路径。

## 范围与限制

- 本次仍未远程打开摄像头，避免抢占用户正在使用的摄像头；已验证修改后脚本自身的 MP4 写入
  与回读链路。
- 没有启动、停止或修改 MAVROS、Odin、extnav、机载控制服务或飞控；没有解锁、起飞或发送任何
  飞行命令。
