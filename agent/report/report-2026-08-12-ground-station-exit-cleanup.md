# 地面站异常退出无残留修复简报

日期：2026-08-12

任务：关闭地面站 GUI 时完整清理本地 SITL、MAVROS、机载仿真节点与 RViz 进程

## 结论

已完成最小范围的生命周期修复。正常点击 GUI 退出仍保留原有默认取消确认和后台安全清理；关闭
运行 `./start_ground_all.sh` 的终端、按 Ctrl+C、发送常规终止信号、Qt 事件循环绕过
`closeEvent` 返回，以及 GUI 进程被 `SIGKILL` 强制结束时，启动壳均能把本项目本机相关进程清理
干净。没有修改仿真启动顺序、ROS 协议、飞行控制、飞控参数或实机服务管理边界。

## 根因与修复前复现

`start_ground_all.sh` 原先最后使用 `exec`，启动壳会被 GUI Python 进程替换。终端窗口关闭时：

1. 终端向前台进程组发送 `SIGHUP`；
2. Python 默认动作直接结束 GUI，Qt `closeEvent` 和 `_begin_shutdown()` 没有执行机会；
3. `ProcessSupervisor` 为了可靠分组管理，刻意用独立 session 启动 SITL/RViz 等进程；
4. 因此 GUI 消失，但独立 session 中的 AP、MAVROS、onboard 与 RViz 继续运行。

修复前使用无飞行的受管休眠进程复现：GUI PID `713429` 收到 `SIGHUP` 后退出，受管 PID
`713476` 仍存活；随后只用既有 `ProcessSupervisor.terminate_all()` 即可将其清除。这证明残留
终止能力本身有效，缺口位于 GUI 异常退出没有进入清理链。

## 修改内容

### `ground_station.py`

- 捕获 `SIGHUP`、`SIGINT`、`SIGQUIT`、`SIGTERM`；信号处理器只记录待处理信号，由 50 ms Qt
  定时器在主事件循环发起安全退出，避免在信号回调中直接操作 Qt。
- 信号退出跳过无人可操作的确认框，复用窗口现有的租约释放、本地进程清理和 ROS 停止流程。
- Qt 事件循环无论通过何种非正常路径返回，都会同步调用一次幂等后端清理兜底。
- 增加内部 `--cleanup-local-processes` 入口，只调用项目既有的严格 argv 匹配扫描；不创建 Qt、
  不创建 ROS 客户端、不连接或管理远端实机进程。

### `ground_station_core/qt_ui/main_window.py`

- 将窗口退出、外部信号退出和事件循环兜底统一到 `_cleanup_backend_once()`。
- 清理结果由锁保护并缓存，避免信号、窗口关闭和 `finally` 路径重复释放或并发清理。
- 右上角关闭按钮和窗口管理器关闭仍走原有二次确认；外部终止信号直接开始安全退出。

### `start_ground_all.sh`

- 不再用 `exec` 丢弃启动壳，而是保留一个只负责生命周期监督的父进程。
- `EXIT/HUP/INT/QUIT/TERM` 均进入统一 trap。若 GUI 尚存活，先发送 `SIGTERM` 并给现有安全退出
  最多 30 秒；超时才结束 GUI。
- GUI 结束后始终执行项目专属本地残留扫描。正常退出时这是无操作的幂等复核；即使 GUI 遭
  `SIGKILL`、来不及执行任何 Python 代码，父级启动壳仍能清除本地残留。
- 保留 GUI 原始退出码；只有 GUI 原本成功而兜底清理失败时才返回清理错误码。

## 兼容性与安全边界

- 未改变正常 GUI 退出的默认取消确认，也未改变飞行中退出提示。
- 未修改 `ProcessSupervisor` 的匹配范围：通用 MAVROS/RViz/SITL 仍必须带本项目专属参数，避免
  误停其他 ROS 工作负载。
- 实机会话仍只释放地面控制租约并停止本地 ROS 客户端；不会停止远端机载 systemd、MAVROS、
  Odin、extnav 或控制节点。
- 启动壳兜底覆盖 GUI 的不可捕获 `SIGKILL`；整台机器掉电、内核崩溃或启动壳本身同时遭
  `SIGKILL` 时，任何用户态清理代码都不可能继续运行。
- 本任务没有连接实机，没有发送飞行命令，没有解锁或起飞。

## 验证结果

### 自动回归

在正确 source Jazzy 与当前 workspace overlay 后：

```text
100 passed in 27.97s
```

新增回归覆盖：

- 四种终止信号注册、转交和原处理器恢复；
- 外部终止跳过确认并只清理一次环境/ROS；
- Qt 事件循环绕过 `closeEvent` 时的同步兜底与幂等性；
- 独立本地清理 CLI 不创建 GUI；
- 原有右上退出按钮确认语义继续通过。

第一次未 source workspace 直接运行全量测试时为 `96 passed, 3 failed`，三个失败均为
`No module named 'guided_interfaces'`；按项目环境要求 source 后全部通过，未将该环境错误计为
产品通过结果。

### 真实启动入口与进程级验证

均通过真实 `./start_ground_all.sh` 启动 GUI，但只使用不含飞行逻辑的休眠替身，不启动真实
飞行仿真：

1. 仅向 GUI 发送 `SIGHUP`：GUI 与替身退出，`related=[]`，启动脚本返回 0；
2. 向终端前台进程组发送 `SIGHUP`：启动壳、GUI、ArduCopter/MAVROS/RViz 三类生产 argv 形态
   的替身全部退出，三者在退出前均被项目扫描器识别，最终 `related=[]`；
3. 向 GUI 发送不可捕获 `SIGKILL`：GUI 返回 137，父级启动壳仍清除替身，最终 `related=[]`。

多进程终端关闭验证记录：

```text
recognized=[769803, 769804, 769805]
MULTI_PROCESS_TERMINAL_HUP_PASS related=[]
SIGKILL_FALLBACK_PASS related=[]
```

### 其他检查

- `python -m compileall`：通过；
- flake8 致命错误检查：通过；
- 本次新增行 88 字符检查：通过；
- `bash -n start_ground_all.sh`：通过；
- `shellcheck start_ground_all.sh`：通过；
- `./start_ground_all.sh --check-environment`：通过；
- 本任务文件 `git diff --check`：通过；
- 最终项目相关进程扫描：`final_related_processes=[]`。

工作区中原有的 `integration/`、RViz 配置和任务 18 文件改动未被本任务修改或纳入修复范围。
