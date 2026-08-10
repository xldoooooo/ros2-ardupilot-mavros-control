# 任务 09 调整简报：独立 Wi-Fi 通讯检测与完整实机连接恢复

日期：2026-08-08
任务性质：修正此前对“本次测试不得解锁/起飞”的产品语义误读

## 结论

调整已完成：原“连接实机”入口恢复完整正式功能；原先临时放入该入口的只读通讯能力已抽离为独立 Wi-Fi 图标按钮，位于原点齿轮正右侧。

- “连接实机”会在风险确认后申请控制租约、发送租约心跳、配置飞控消息频率、写入 GUI 缓存的 GPS 原点，并等待远端本地位置就绪；连接成功后，实机会话按与仿真一致的权威状态门控开放起降、手动运动和航点功能。
- “连接实机”动作本身不发送解锁或起飞请求；这两个高风险动作仍只能由操作者分别点击并再次确认。
- 独立 Wi-Fi 按钮仅订阅 `/onboard_control/status` 与远端 `/rosout`，在 3 秒窗口内计算状态样本数、接收频率和最大接收间隔，并把飞控、武装、租约和日志接收结果写入 GUI 日志。
- Wi-Fi 检测不申请/释放控制租约，不发布心跳，不调用消息频率、GPS 原点、模式、解锁、起飞、运动或航点入口，不启动/停止任何本地或远端服务，也不建立 GUI 环境会话。

本次调整过程中没有向真机执行完整连接、解锁或起飞，也没有改动真机环境。

## 设计边界

### 完整实机连接

生产 ROS 客户端仍以“默认仅观察”方式启动，避免 GUI 启动即自动申请租约。只有操作者明确确认完整仿真或实机连接后，环境工作流才调用 `enable_control()`；命令传输层也会直接拒绝观察态中意外排队的命令。

恢复后的实机流程依次执行：

1. 只清理本项目在地面机上的旧仿真进程并释放旧租约，不管理远端进程；
2. 启用远端日志接收并显式开启控制租约；
3. 等待接口 2.0、控制权、FCU 连接和 `GUID_OPTIONS` 推力语义；
4. 调用 `set_rates` 与 `set_gp_origin`；
5. 等待远端本地位置有效，然后建立 `hardware` 环境会话。

GUI 不再对 `hardware` 会话附加强制只读条件。飞行按钮仍必须同时满足 ROS、机载状态、FCU、租约、位置、推力语义和发布源冲突等既有安全门控。

### 独立 Wi-Fi 通讯检测

Wi-Fi 路径采用独立完成信号，不复用会把 `_environment_active` 置真的环境完成信号。检测前若当前客户端已经允许控制或持有租约，会直接拒绝；检测期间也持续检查这一不变量。

诊断使用机载状态默认 10 Hz 的事实，成功阈值设为至少 5 Hz、最大接收断流不超过 0.5 秒。成功日志包含：

- 新收 `ControlStatus` 数量和实测速率；
- 最大本地接收间隔；
- FCU 是否连接、是否武装、当前租约持有者；
- 检测窗内进入 GUI 的远端 rosout 条数；
- 明确的零租约、零命令、零进程管理声明。

普通环境工作流失败时会执行安全清理；Wi-Fi 诊断刻意绕开该清理分支，失败或取消时只关闭本地 rosout 接收开关，避免“只检测”按钮间接触发租约释放或进程终止。

## GUI 调整

- 环境操作行现在依次为“启动仿真”“连接实机”“齿轮”“Wi-Fi”。
- Wi-Fi 使用仓库内 SVG 图标、独立 object name、无文字按钮和中文可访问名称/说明。
- 齿轮与 Wi-Fi 均为 36×36 正方形；最小 1180×700 窗口下两个主按钮文字完整可读。
- 原点对话框、摘要和 tooltip 已恢复为“完整实机连接写入，SITL/Wi-Fi 检测不写入”的一致语义。
- 通讯检测运行时会互斥禁用两个环境入口、齿轮和 Wi-Fi 自身；结束后不改变环境标签或连接模式。

离屏实际 Qt 渲染证据：`agent/codex/task09-wifi-button-ui.png`。

## 修改文件

- `ground_station_core/environment.py`
  - 恢复完整实机工作流；增加零命令通讯工作流、链路指标和独立失败收尾。
- `ground_station_core/ros_controller.py`
  - 保留默认仅观察门控与远端 rosout 接收；增加状态累计数和有界接收时间戳读取。
- `ground_station_core/qt_ui/main_window.py`
  - 恢复完整实机确认/原点传递；增加独立通讯忙状态和完成信号。
- `ground_station_core/qt_ui/operations_panel.py`
  - 增加齿轮右侧 Wi-Fi 按钮；恢复实机连接、原点和断开文案。
- `ground_station_core/qt_ui/state.py`
  - 恢复实机会话控制门控；增加通讯检测可用状态。
- `ground_station_core/qt_ui/theme.py`
  - 为四个环境入口调整紧凑、清晰的最小窗口样式。
- `ground_station_core/qt_ui/assets/wifi.svg`
  - 新增 Wi-Fi 矢量图标。
- `ground_station_core/config.py`
  - 恢复默认原点的完整实机连接用途说明。
- `tests/test_environment_communication.py`
  - 新增完整连接恢复及诊断成功/失败零副作用回归。
- `tests/test_qt_gui.py`、`tests/test_ros_controller.py`
  - 覆盖按钮位置、信号、环境不变、实机门控、原点传递和状态接收统计。
- `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`
  - 区分 Wi-Fi 零命令检测与完整“连接实机”。
- `agent/codex/task09_gui_communication_probe.py`
  - 替换会点击旧“只读连接”入口的探针，改为点击新 Wi-Fi 按钮。
- `MEMORY.md`
  - 记录纠正后的长期产品语义和验证基线。

`TODO.md` 是任务开始前已有的用户修改，本次未编辑其内容。

## 验证结果

### 自动回归

```text
Python 全量：                 37 passed
colcon build：               guided_interfaces/onboard_control/guided_sim 成功
ROS/C++ test-result：        5 tests, 0 errors, 0 failures, 0 skipped
ground_station environment：OK
compileall：                 通过
flake8 E9/F63/F7/F82/E501： 通过
git diff --check：           通过
```

关键测试使用“任何租约、维护、飞行或进程调用都会立即抛错”的替身验证 Wi-Fi 成功路径，并另行覆盖异常收尾。完整实机工作流的替身回归确认实际调用 `enable_control`、`request_set_rates` 和带原值的 `request_set_gp_origin`。

### 生产 Qt→ROS 隔离集成

在本机 ROS domain 231、localhost-only 发现范围内启动生产 `onboard_control_node`，使用独立 `/_task09_wifi_mavros` 前缀且不启动 MAVROS；随后由真实 Qt Wi-Fi 按钮驱动生产 ROS 客户端与生产通讯工作流：

```text
结果：                       PASS
ControlStatus：              30 条 / 9.98 Hz
最大接收间隔：               100.2 ms
远端 rosout：                1 条，GUI 原文可见
FCU：                        未连接
armed：                      false
control_enabled/authority：  false / false
lease/command_result：       0 / 0
GUI environment/mode：       false / none
飞行按钮：                   全部禁用
```

测试后本地 `onboard_control_node` 与 GUI 探针均无残留。第一次选择 domain 239 时 Fast DDS 因端口计算上限在节点创建前拒绝启动，随后改用合法的 domain 231 完成上述验证；该过程未产生飞控连接或控制命令。

### 真机范围说明

本次调整未重新启动或修改真机服务。此前任务 09 的真实链路数据仍有效：两个 60 秒窗口各收到 601/601 条状态，最大接收间隔约 104.43/110.39 ms，FCU 守护窗始终 `armed=false`，命令与姿态 setpoint 均为 0；详见 `report-2026-08-08-task09-communication.md`。

没有在真机上点击恢复后的完整“连接实机”，因为该路径按正式设计会申请租约、配置消息频率和写 GPS 原点，已经超出本次零命令通讯复验的必要范围。也没有发送解锁、起飞、模式、运动或航点指令。

## 已知限制

Humble `rmw_fastrtps_cpp 6.2.10` 与 Jazzy `8.4.3` 的跨发行版发现仍会出现 `sequence size exceeds remaining buffer`，且 Jazzy 图侧可见未知节点名/type hash `INVALID`。状态数据路径的既有实测稳定性不代表完整 ROS 图兼容，更不构成实飞授权；在统一 ROS/DDS 或加入受支持桥接并重新验证前，应继续按部署文档的限制处理。
