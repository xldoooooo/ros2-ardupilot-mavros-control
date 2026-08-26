# Windows 摄像头 RTSP 使用教程任务简报

## 任务结果

已新增面向甲方非技术人员的 Windows USB 摄像头调参与局域网 RTSP 推拉流教程：

- 文件：`docs/Windows-USB摄像头调参与RTSP推流教程.md`；
- 目标设备按项目现有真实基线写明为 Wasintek UVC（USB VID:PID `2aad:6373`）；
- 采用 OBS Studio、MediaMTX、VLC 三款成熟工具，不要求用户编写程序；
- 覆盖摄像头预览、曝光/增益等驱动参数、分辨率/FPS/采集格式、H.264 输出编码、RTSP 服务、跨设备拉流、停止顺序和故障排查；
- 使用 `rtsp://192.168.112.101:8556/camera` 作为贯穿示例，并明确 IP 必须取推流电脑的真实局域网 IPv4，只有端口和路径可自行决定；
- 给出从 1080p30 基线开始逐项升级到 1080p60、720p120 的测试顺序，避免一次修改多个变量后无法定位故障。

## 方案依据

- OBS 官方 Windows 视频采集文档确认可选择 DirectShow 摄像头、分辨率、FPS、视频格式并打开驱动配置界面；
- MediaMTX 官方文档推荐 OBS 通过本机 RTMP 向 MediaMTX 发布，由 MediaMTX 在相同路径提供 RTSP；
- MediaMTX Windows 独立二进制可直接解压运行，符合不要求甲方编译或编写代码的约束；
- 教程仅要求在现成 `mediamtx.yml` 中把 RTSP 传输固定为 TCP，并按需修改监听端口。

## 验证与边界

- 已检查 Markdown 标题、代码块、表格、链接和章节结构；
- 已对照项目 `camera.conf`、`lens.conf`、`MEMORY.md` 和真实摄像头台架报告核对型号及已验证模式；
- 本任务仅新增/维护文档，没有启动或占用摄像头，没有运行 FFmpeg/MediaMTX/OBS，没有连接飞机或操作任何飞控功能；
- Windows 实际控件名称会随 OBS、系统语言和摄像头驱动略有差异，教程已同时给出关键英文名称和排查边界；
- 未在本机 Ubuntu 环境执行 Windows GUI 实机复测，实际甲方 Windows 驱动是否暴露全部曝光/增益控制项，应以其电脑显示为准，文档中已如实说明。
