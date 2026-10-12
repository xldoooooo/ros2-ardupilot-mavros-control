<!-- Odin raw ROS cloud publication removal and deployment evidence. -->
# Odin 只发布降采样 SLAM 云

日期：2026-10-13。

## 修改与用途

SDK维护仓库 main `16735e8`，仅删除header的9行：ROS1/ROS2原始cloud_slam发布调用、发布器初始化和成员声明。保留SDK SLAM输入、内部临时点云转换和原有降采样算法；down继续为XYZ、12字节/点。README说明关闭降采样时不再输出SLAM云。

正式避障launch默认订阅 `/odin1/cloud_slam_down`。new运行现场查询：原始云0订阅者；down只有RViz订阅者，当时避障未运行。refresh当时无Odin进程，不能据此描述其运行时订阅者。SDK `sendcloudslam=1` 仍须保留，否则主机down无输入。仅依赖原始话题的外部工具需要改订阅down。

删除ROS发布可减少DDS写入、缓存等开销；有原始云订阅者时还可减少序列化和网络传输。当前原始云无订阅者，因此收益预期有限。未测CPU前后百分比，不宣称显著改善；设备SLAM计算、USB输入及主机临时转换没有减少。

## 验证与部署

- x86原有12个核心过滤用例全部通过，过滤代码未改。
- 两机ARM原生编译、安装成功，现场配置和标定未改；new既有OpenCV链接版本警告保留。
- refresh隔离domain221实际运行：raw收到0帧、发布器0；down收到20帧、发布器1；debug发布器0；XYZ格式及全部坐标有限值通过，退出后测试进程停止。
- 两机header SHA256：`995993f274428d2413841f6970f069a12307a2bdc173cf2ee8ee852ecf1de4fd`。
- refresh build/install BuildID：`0a40869dbc7094516ab2ddc9ec52b7ed5ddb2506`；new：`5eedcc7e5076adb8bb0c5c7b4f9b9c30e65b5c26`。
- new实查MAVROS connected=true、armed=false、STABILIZE，完整机载进程链正在运行。新版已安装，旧Odin进程保持运行，下次重启生效；未在new进行新版运行验收，不把安装成功算作已生效。

备份及构建日志：两机 `/home/nvidia/ros2-ardupilot-maintenance/task2-down-only-20261013/`。过程脚本位于开发机 `agent/codex/task2-down-only/`。未解锁或起飞实机。
