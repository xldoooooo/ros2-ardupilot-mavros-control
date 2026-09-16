<!-- Task33 follow-up: change inventory and refresh aircraft deployment. -->
# Task33 变动统计与 refresh 机载代码同步

日期：2026-09-17。用户补充确认 refresh 当前没有飞控。

## 上次 task33 变更

统计提交 `82f5e0435761e7dca4b8a3327b28d1745b028abd`，依据 `git show --numstat`，包含注释与空行。
共25文件，新增885行、删除23行，净增862行。其中：

- 实现、接口与构建脚本16文件：+496/-12。
- 测试6文件：+243/-7。
- 文档与记录3文件：+146/-4。

| 文件 | 新增 | 删除 |
|---|---:|---:|
| `MEMORY.md` | 24 | 3 |
| `agent/report/report-2026-09-17-task33-fcu-hot-reboot.md` | 80 | 0 |
| `ground_station_core/config.py` | 2 | 2 |
| `ground_station_core/models.py` | 2 | 0 |
| `ground_station_core/qt_ui/main_window.py` | 27 | 0 |
| `ground_station_core/qt_ui/state.py` | 8 | 1 |
| `ground_station_core/ros_controller.py` | 7 | 0 |
| `reboot_fcu.sh` | 19 | 0 |
| `src/guided_interfaces/msg/ControlStatus.msg` | 3 | 0 |
| `src/guided_interfaces/package.xml` | 1 | 1 |
| `src/guided_interfaces/srv/FlightCommand.srv` | 1 | 0 |
| `src/onboard_control/CMakeLists.txt` | 1 | 0 |
| `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md` | 42 | 1 |
| `src/onboard_control/deploy/onboard_workspace.sh` | 5 | 1 |
| `src/onboard_control/include/onboard_control/onboard_control_node.hpp` | 29 | 0 |
| `src/onboard_control/package.xml` | 1 | 1 |
| `src/onboard_control/src/fcu_reboot.cpp` | 237 | 0 |
| `src/onboard_control/src/onboard_control_node.cpp` | 35 | 6 |
| `tests/test_bootstrap.py` | 3 | 3 |
| `tests/test_fcu_reboot.py` | 196 | 0 |
| `tests/test_onboard_truthfulness.py` | 1 | 1 |
| `tests/test_qt_gui.py` | 39 | 0 |
| `tests/test_ros_controller.py` | 2 | 1 |
| `tests/test_upstream_communication.py` | 2 | 2 |
| `tools/reboot_fcu.py` | 118 | 0 |

## 本次同步

- 目标：`ssh drone-refresh`，当前解析 `192.168.112.186`，工作区
  `/home/nvidia/ros2-ardupilot-mavros-control`。
- 初始 HEAD 为 `a10ef76`，跟踪文件干净，存在用户未跟踪的 `start_drone/image/`；完整保留。
- 已备份 `src/guided_interfaces`、`src/onboard_control` 和两包安装产物到机上
  `agent/codex/task33-refresh/pre-sync.tgz`，另保存 sparse checkout 原配置。
- 通过 Git 快进同步 main，追加 sparse 路径以包含 `reboot_fcu.sh`、`tools/reboot_fcu.py` 和任务报告，
  不替换已有 sparse 配置，不清理其他本地文件。
- 发现自检脚本仍硬编码接口3.2，本次修正3处为3.3（+3/-3）。未改变控制/重启功能实现。
- 保持 `ros2-ardupilot-onboard.service`、`odin-correction.service`、`video-service.service`
  原有 disabled/inactive；没有启动真实机载链，没有执行飞控重启、解锁或起飞。

## 验证

- `./build_onboard_control.sh --verify` 成功：四包 ARM64 Release 原生构建通过，
  colcon 汇总24 tests、0 errors、0 failures、0 skipped。
- localhost/domain231隔离smoke确认：`interface=3.3`、`fcu_connected=false`、
  `armed=false`、`setpoint_messages=0`，结束后隔离进程退出。
- 共享状态消息、包版本和部署自检脚本的源码/install逐字比较一致；重启C++源码、主节点源码、
  `reboot_fcu.sh` 和Python客户端四文件的开发机/refresh SHA-256逐一匹配。
- 开发机部署脚本定向回归20项通过，Bash语法检查通过。
- 无实机飞控，因此不声称完成refresh上的真实重启或完整飞行链路验收；此前new上的结果仍以
  原task33报告为准。new运行功能已是3.3，但本轮未连接new，自检脚本的3.3校验修正待后续同步。
- 过程日志与备份保存在两机 `agent/codex/task33-refresh/`；本次报告和MEMORY单独提交main。

