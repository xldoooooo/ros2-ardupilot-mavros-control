# 192.168.112.101 地面站仿真启动故障排查与修复报告

## 任务范围与安全边界

- 目标机：`scq@192.168.112.101`，Ubuntu 24.04 / ROS 2 Jazzy。
- 目标：修复 Qt 地面站点击“启动本地仿真”后初始化失败的问题。
- 全程未连接、解锁、起飞或控制实机，未调用起飞、降落、航点或运动命令。
- 现场验证只使用 domain 231 + LOCALHOST 的本地 SITL；验证过程中持续检查
  `armed=false`，结束后释放仿真租约并清理本地受管进程。

## 故障证据与根因

目标机 `/tmp/ros2_ardupilot_ground_station/sitl.log` 连续四次记录：

```text
[Errno 2] No such file or directory: 'mavproxy.py'
```

ArduPilot 源码和已构建的 `arducopter` 均存在，MAVProxy 也已安装在
`/home/scq/venv-ardupilot/bin`，但地面站启动器使用项目 `.venv` 的 Python，且没有激活
ArduPilot venv；`sim_vehicle.py` 内部按命令名执行 `mavproxy.py`，因此无法从子进程 `PATH`
找到它。失败尝试还留下了一个 03:21 启动、占用 TCP 5760 的旧 SITL；已按明确 PID 终止，
未扫描或停止其他 ROS 工作负载。

第一阶段修复后又暴露第二个独立问题：MAVProxy 1.8.74 wheel 的运行代码直接导入
`future`，但 wheel 元数据没有声明该依赖。项目 `.venv` 因此出现：

```text
ModuleNotFoundError: No module named 'future'
```

这会使 MAVProxy 启动即退出，继而让 `sim_vehicle.py` 以 code 0 结束。RViz 日志中的
`KeyboardInterrupt` 是仿真失败后统一清理的次生结果，不是首要根因。

## 修复内容

1. `requirements-gui.txt` 纳入 `MAVProxy>=1.8,<2` 与其缺失的运行依赖
   `future>=1,<2`，使 `setup_project.sh` 可在新电脑的项目 `.venv` 中完整安装本地 SITL
   Python 依赖。
2. `ground_station_core.config.find_mavproxy()` 按以下顺序定位可执行入口：
   `GROUND_STATION_MAVPROXY`、当前 `PATH`、当前 Python/项目 `.venv`、常见 ArduPilot venv。
3. 仿真编排只把已验证的 MAVProxy `bin` 目录加入 SITL 子进程 `PATH`，不修改地面站全局
   环境，也不影响 domain 0 实机会话。
4. 启动任何 SITL/RViz 进程前真实执行 `mavproxy.py --version`；入口缺失或依赖损坏时明确
   前检查失败，避免再次产生半启动进程。
5. 增加 MAVProxy 非全局 PATH 发现、运行探针及 SITL 子进程 PATH 注入回归；部署文档同步
   说明自动安装和发现规则。

## 目标机验证结果

- 项目 `.venv`：MAVProxy 1.8.74、future 1.0.0，`mavproxy.py --version` 成功。
- 定向回归：22 passed。
- 正式 Python 测试范围 `pytest tests`：105 passed。
- `ground_station.py --check-environment`：通过。
- 完整未武装 SITL 初始化：成功启动 SITL、MAVROS、`onboard_control_node` 与 RViz。
- 就绪快照：`connected=true`、`armed=false`、`mode=STABILIZE`、
  `local_position_valid=true`、`control_authority=true`、`thrust_mode_verified=true`。
- 清理结果：4 个受管进程停止，`remaining=()`、`errors=()`；5760/5762 无监听，项目仿真
  相关进程零残留。
- 本机同一正式 Python 测试范围：105 passed；环境自检、compileall、致命级 flake8 与
  `git diff --check` 通过。

根目录无范围 `pytest` 仍会在收集用户既有
`integration/websocket_test_demo/tests/test_protocol.py` 时因缺少 `ws_demo` 失败；这是项目
记忆中已有的无关集成示例收集问题，本次没有修改或掩盖。正式维护测试范围 `tests/` 全部通过。

## 最终状态

目标机“启动本地仿真”的两层依赖故障均已修复，并完成一次与 GUI 相同后端工作流的真实闭环
验证。验证结束后目标机保持无本地仿真进程、无仿真端口监听；没有执行任何实机解锁或起飞。
