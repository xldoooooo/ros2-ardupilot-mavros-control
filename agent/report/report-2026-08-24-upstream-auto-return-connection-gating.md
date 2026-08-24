# 上位机自动返航连接门控修改简报

## 任务结论

已按要求收紧低电量和无人机异常自动返航的启用条件：

| WebSocket | 飞行环境 | 低电量 / 无人机异常处理 |
| --- | --- | --- |
| 断线 | 仿真或实机 | 不启动自动返航，不下发航点或 LAND |
| 在线 | 仿真 | 保持既有自动返航、降落和异常入库恢复流程 |
| 在线 | 实机 | 地面站日志和横幅只提示一次；飞行实现为 TODO + `pass` |

本次未连接、解锁或起飞实机，也未启动或停止任何机载服务。

## 实现内容

- `ground_station_core/upstream/service.py`
  - 状态投影仍可在断线时维护内部边沿，但低电量动作触发结果只有在 WebSocket
    `connected=true` 时才向主窗口开放。
- `ground_station_core/qt_ui/main_window.py`
  - 每次刷新读取真实 WebSocket 连接状态，无法读取时按断线故障关闭。
  - 无人机异常按断线、在线仿真、在线实机三路分流。
  - 低电量按断线、在线仿真、在线实机三路分流。
  - 在线实机两处仅写结构化日志和活动横幅；使用一次性锁存避免 10 Hz/1 Hz 重复刷屏。
  - 在线实机两处均保留明确 TODO，并按要求显式 `pass`，不调用航点、返航或 LAND 接口。
- `ground_station_core/qt_ui/upstream_panel.py`
  - 同步更新 0C 和无人机异常说明，明确三种连接/环境语义。
- `tests/test_upstream_communication.py`、`tests/test_qt_gui.py`
  - 新增服务层断线门控、断线仿真/实机零飞行动作、在线实机仅提示，以及在线仿真保持原行为的回归覆盖。

## 验证结果

- 新增与直接相关用例：`4 passed`。
- 上位机通讯与 Qt GUI 相关回归：`64 passed`。
- Python 基础静态检查：`ruff --select E4,E7,E9,F` 通过。
- 加载 `/opt/ros/jazzy/setup.bash` 和仓库 `install/setup.bash` 后，全量 Python 回归：
  `180 passed in 42.18s`。
- 第一次未加载仓库 ROS 安装环境运行全量测试时，7 项因找不到 `guided_interfaces` 失败；
  按项目标准环境重新运行后全部通过，不属于源码缺陷。

## 边界与未完成项

- 实机低电量返航和实机无人机异常返航按任务要求尚未实现，源码中保留 TODO。
- 当前门控控制“是否新触发自动组合”。若 WebSocket 在机载端已经接收飞行命令后断开，本次不会
  粗暴撤销已下发命令；后续如需断线中止策略，必须单独定义悬停、继续或降落的安全语义。
- 未进行真实 WebSocket + SITL 端到端飞行验证；本次使用单元与 Qt 回归验证命令调用边界。
