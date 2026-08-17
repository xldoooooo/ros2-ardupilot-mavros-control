# 真机摄像头本地录像脚本执行简报

## 任务结果

- 真机实际基础文件为 `/home/xld/Project/python_demo/demo2.py`，并不存在用户表述中的
  `demo_2.py`；已按“在 demo2 基础上”的要求处理。
- 新建 `/home/xld/Project/python_demo/demo_2_save.py`，没有修改原 `demo2.py`。
- 新脚本保留摄像头 0、MJPG 采集、120 FPS 请求、OpenCV 预览窗口和按 `q` 退出的原有行为。
- 首次成功取帧后，在脚本所在的 `/home/xld/Project/python_demo` 目录创建带微秒时间戳的
  `demo_2_YYYYMMDD_HHMMSS_ffffff.avi`，以 MJPG 编码保存每一帧。
- 视频尺寸以第一帧实际尺寸为准；录像帧率优先使用摄像头报告值，无效、非有限或非正值时
  回退为 30 FPS。
- 正常退出和异常退出均释放摄像头、视频写入器与 OpenCV 窗口；视频写入器创建失败会明确报错。

## 验证结果

- 真机 Python 源码内存编译：通过。
- 真机 OpenCV MJPG/AVI 合成帧写入及回读：通过，写入并回读 3 帧，测试文件 7720 字节；
  测试临时文件已删除。
- 原文件修改前后 SHA-256 均为
  `4e94df16f007a62d1c25b8481a4ea56a8af333ba24bb03a831eda823728d2f95`。
- 新文件 SHA-256 为
  `1510bd4f844eda4edab621cb8905c1f9b5dcdfc5a24863b4403749cba5aa1bb5`，权限为 `0644`，
  所有者为 `xld:xld`。
- 验证过程没有打开 `/dev/video0`，没有在目标目录产生意外录像文件。

## 使用方法

```bash
cd /home/xld/Project/python_demo
python3 demo_2_save.py
```

预览窗口中按 `q` 结束，终端会打印最终视频的绝对路径。

## 范围与限制

- 为避免远程 SSH 验证抢占摄像头、影响用户现有摄像头会话，本次没有实际打开摄像头录制；
  真实摄像头取帧和显示能力沿用用户已确认可工作的 `demo2.py` 逻辑。
- 本次没有启动、停止或修改 MAVROS、Odin、extnav、机载控制服务或飞控；没有解锁、起飞或发送
  任何飞行命令。
