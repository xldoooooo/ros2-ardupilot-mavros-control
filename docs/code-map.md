<!-- 按任务定位源码、跨模块边界与验证入口；不维护逐文件摘要或设备状态。 -->
# 代码地图

已知文件、符号或报错时直接定位；不清楚入口时只读下表相关行，再沿调用方、消息生产者/消费者和配置继续查。路径相对仓库根目录；表中入口不是完整改动范围。

## 任务与实现入口

| 任务 | 先读哪里 | 跨模块边界与说明 |
| --- | --- | --- |
| 地面站显示、刷新、窗口 | [qt_ui](../ground_station_core/qt_ui/)、[models.py](../ground_station_core/models.py) | `main_window.py` 组合各面板，`state.py` 投影状态；业务数据继续查 [ros_controller.py](../ground_station_core/ros_controller.py)，媒体查独立服务 |
| 命令、权限、飞控状态 | [ros_controller.py](../ground_station_core/ros_controller.py)、[onboard_control_node.cpp](../src/onboard_control/src/onboard_control_node.cpp) | 地面站为薄客户端；机载是唯一飞控权威与姿态 setpoint 输出源；协议在 [guided_interfaces](../src/guided_interfaces/) |
| 航点、坐标、参考及跟踪 | [waypoint_io.py](../ground_station_core/waypoint_io.py)、[机载源码](../src/onboard_control/src/)中的 navigation_frame、reference_generator、dob_controller、avoidance_execution | [坐标约定](odom-map.md)；区分任务 odom/map 与 FCU local、参考限额与实测运动；接口变更检查地面、机载、仿真与桥 |
| CPU / FPGA 避障 | [避障桥说明](../src/avoidance_bridge/README.md)、[FPGA说明](../fpga_avoidance/README.md) | CPU 地图/搜索在独立 `dyn_small_obs_avoidance-ros2` 工作区（`AVOIDANCE_WORKSPACE`，默认同级目录）；桥仅适配目标、坐标和结果，机载负责执行。外部算法改动使用该工程自身规则与验证 |
| Odin、点云、安装外参 | [项目驱动](../drivers/odin/README.md)、[驱动源码](../drivers/odin/src/odin_ros_driver/) | 以本项目驱动为准；业务 Odometry 为 FCU 中心，下游不重复补偿安装杆臂；配置见 [配置索引](../config/README.md) |
| Tag 采样、估计、定位修正 | [修正服务](../correction_service/README.md)、[Python状态机/滑窗](../correction_service/correction_service/)、[原生接收/采集/检测](../src/correction_native/) | [生产 extnav](../correction_service/extnav_patch/README.md)、[correction_interfaces](../src/correction_interfaces/)；自动航点校准还需查 [waypoint_calibration.cpp](../src/onboard_control/src/waypoint_calibration.cpp)及[工作流](tag-waypoint-workflow.md)。修正 ACK 不等于 EKF 已融合 |
| 上位机业务、巡检回执 | [upstream](../ground_station_core/upstream/)、[协议说明](websocket/上位机-WebSocket%20对接.md) | 经 ROS 客户端到机载；图片/录像结果来自 [ground_media_reader.py](../ground_station_core/ground_media_reader.py)及媒体服务 |
| 视频、录像、媒体交付 | [video_service](../video_service/README.md)、[ground_media_reader.py](../ground_station_core/ground_media_reader.py) | 飞机采集与地面处理为独立进程；与飞行事件、上位机结果及本机媒体偏好关联 |
| 配置、启动、部署、日志 | [配置索引](../config/README.md)、[启动与日志](startup-and-logs.md)、[机载部署](../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md) | [onboard脚本](../scripts/onboard/)、[ground脚本](../scripts/ground/)、[共享构建依赖](../scripts/lib/build_dependencies.bash)；按设备核对配置、源码、原生构建与运行产物 |

## 验证与按需资料

- 产品测试以 [测试指南](../tests/README.md)和 [tests/run.py](../tests/run.py) 的影响映射为准，不在地图复制测试组规则。先运行 `bash tests/run.sh --files <本次相关文件...> --list`，再按结果及实际消费者验证，避免混入工作区其他任务。
- 接口/C++ 修改先按测试指南运行 `bash tests/build_native.sh`，不能用旧二进制通过替代新源码验证。FPGA 使用独立 `bash fpga_avoidance/test.sh`；外部 CPU 规划器还有自身验证范围。
- 部署使用 [project-deployment](../.agents/skills/project-deployment/SKILL.md)，故障复盘使用 [project-forensics](../.agents/skills/project-forensics/SKILL.md)；项目级维护才使用 [project-maintenance](../.agents/skills/project-maintenance/SKILL.md)。
- 本机 `MEMORY.md` 存在时，修改前按标题查相关风险；部署前查目标设备、漏同步项及现场保留约束。只有需要历史证据时才查本机 `agent/report/topics/INDEX.md`，再定向读报告。
- MEMORY、报告和根 AGENTS 不随 Git 分发；新检出缺失不妨碍源码定位，但设备基线、当前进程和现场差异须现场核实，不能从仓库版本推断。
- 只在入口或职责变化时更新本表；参数值以配置为准，设备状态留在本机 MEMORY，设计细节留在模块文档。
