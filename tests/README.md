<!-- 自动化测试入口、功能闭环归属与环境要求。 -->
# 自动化测试

日常修改按实际影响选择业务闭环或界面专项；共享代码覆盖实际消费者，不自动等同全量。
广泛改变公共执行机制、跨模块重构或明确整体验收时才运行全部测试。
所有命令从仓库根目录执行，不连接飞机；ROS 测试使用 localhost 和隔离 domain。

```bash
# 日常首选：限定本次任务文件，先预览命中规则和执行计划
bash tests/run.sh --files ground_station_core/qt_ui/main_window.py --list
bash tests/run.sh --files ground_station_core/qt_ui/main_window.py

# 整个工作区均属于本次任务时，比较 HEAD（包含暂存、未暂存及未跟踪文件）
bash tests/run.sh --changed HEAD --list
bash tests/run.sh

# 验证已提交改动：基线之后的差异也会计入
bash tests/run.sh --changed HEAD~1

# 单个文件或用例沿用统一环境，不附带业务组和 C++
bash tests/run.sh --tests tests/waypoints/test_waypoint_panel.py

# 显式选择一个或多个闭环
bash tests/run.sh --group video
bash tests/run.sh --group flight waypoints
bash tests/run.sh --group gui

# 功能组内只运行轻量检查，或只运行真实 ROS/媒体等集成检查
bash tests/run.sh --group correction --level unit
bash tests/run.sh --group waypoints --level integration

# pytest 选项放在 -- 后；C++ 检查仍按所选闭环执行
bash tests/run.sh --group upstream -- -x

# 跨模块重构或整体验收时明确执行全量
bash tests/run.sh --all
```

| 组 | 功能闭环 | 包含的检查 |
| --- | --- | --- |
| `runtime` | 启动 → 环境连接/切换 → 清理退出 | 自举、进程、环境通讯、部署入口、Qt 连接/退出 |
| `flight` | 权限/就绪 → 命令 → 权威状态/恢复 | ROS 客户端、参数读取、遥测、热重启、飞行门控、DOB C++ |
| `waypoints` | 导入/编辑 → 预览/上传 → 到点进度/终态 | CSV、Qt 航点、ROS 传输、RViz、导航/参考/到点 C++、原生避障桥 |
| `upstream` | WebSocket 指令 → GUI/任务编排 → 状态回传 | 协议、映射、通讯、巡检/返航/入库、上位机面板 |
| `video` | 配置/采集 → 推流/录像/抓拍 → 回执/关闭 | 独立进程/面板、ROS 视频、飞行事件联动、视频部署 |
| `correction` | 采样 → 几何/滑窗 → CAS 应用 → 面板/释放 | 相机、Tag 方向、同步、估计、窗口、节点事务、生产 C++ extnav 等价、包级回归 |
| `ui` | 界面基础状态与呈现 | Qt 平台、日志、窗口、刷新行为 |

七个业务组保持完整闭环，另有以下可与业务组取并集的专项。混合选择时用例去重：

| 专项 | 范围 |
| --- | --- |
| `waypoint-gui` | 航点面板编辑、预览交互、执行状态和后端按钮门控 |
| `ground-gui` | 地面主窗口与面板、连接恢复、手动/上位机起飞门控、倒计时和独立面板启动 |
| `gui` | `ground-gui` 加摄像头、文件管理、Tag 修正独立窗口；保留相关模态/本机 HTTP 交互 |
| `configuration` | Python/Shell 档案选择、设备身份与主机设置契约 |

界面专项不附带控制 C++、原生避障桥、推流/转码或修正算法回归。ROS 客户端、消息、算法、
进程生命周期发生变化时，仍须选择相应业务组及集成层，不能用界面专项替代。

`support/` 保存共享 Qt 替身、DDS 节点 fixture 与 extnav 进程助手，不包含测试。大型 Qt 用例按上述闭环拆开，
不能再通过导入某个 `test_*.py` 来复用替身，否则可能重复收集或产生跨组依赖。

## 改动影响规则

`impact.py` 的 `RULES` 和专项清单是唯一影响映射；`run.py` 负责选择与执行，命中多条规则时取并集：

- 航点面板选择 `waypoint-gui`；主窗口、操作面板及状态投影选择 `ground-gui`。
- 共享主题、控件、阴影、窗口装饰及 Qt 替身选择 `gui`；CSV、协议等契约选择实际消费业务组。
- Odin 驱动触发运行、飞行和修正；导航切换与独立避障入口触发运行、飞行和航点。
- 修改机载主节点会触发飞行、航点和视频，因为它发布视频事件；不触发独立 Tag 修正。
- 配置按控制、导航、相机、修正及主机设置等实际消费者选择，不再把整个 `config/` 映射到全量。
- 接口、环境和部署共用代码按实际消费方联动；修改某个测试文件仅执行该文件，再与产品影响范围合并。
- 组内共享夹具/辅助代码和删除的用例仍选择所属业务组；全局共享夹具覆盖全部业务组。
- 修改映射数据只执行选择器自检，并与本次产品修改的范围合并；修改测试构建入口只执行其隔离调用检查。
- 执行器、全局夹具和测试环境变化仍触发七个业务组；明确全量还包含测试系统自身检查。
- Markdown、文档与任务记录不触发回归。未知代码路径会报错并要求补充映射或显式选择，
  不会静默漏测，也不会自动扩大为全量。
- Git 重命名按旧路径删除和新路径增加处理。默认 `HEAD` 不包含已提交且工作区干净的修改，
  此时必须指定提交前的基线。

预览显示选择来源及逐文件命中规则，不导入或收集未选范围。映射粒度为文件；同一文件增加新的
跨模块调用时，需同步补充消费方，不能仅凭旧映射判断影响。新增独立界面用例时同步维护专项清单。
选择器自检核对实际已跟踪源码/配置和专项目标是否有效；影响边界变化时更新必要边界用例。
现有 `dev/` 示例不属于产品闭环，不自动执行；变更这些代码时应使用该示例自身的测试入口。
FPGA 模块保留独立的 `bash fpga_avoidance/test.sh`；修改该目录时统一入口会提示此命令，
不会把可拆卸模块导入默认产品组。

## 环境与结果边界

`run.sh` 使用项目 `.venv`，加载 Jazzy、可用的原生规划器 overlay 和本仓库 overlay；不会构建、部署、
重启服务或操作实机。接口/C++ 修改后先构建相关产物，测试范围仍按实际影响选择：

```bash
bash tests/build_native.sh --packages correction_native --list
bash tests/build_native.sh --packages correction_native
bash tests/run.sh --files src/correction_native/src/odometry.cpp

# 接口变化需显式包含全部消费者；完整构建仍可不带参数执行
bash tests/build_native.sh
```

构建入口使用全部本机 CPU，明确设置 `BUILD_TESTING=ON`，不启动服务。`--packages` 构建指定包及
本仓库依赖，不自动重建下游消费者；修改消息接口时必须带上其消费者，或执行完整构建。
只有构建 `avoidance_bridge` 才要求独立规划器 overlay，位置由 `AVOIDANCE_WORKSPACE` 指定，
缺省为同级 `dyn_small_obs_avoidance-ros2`。只构建 `extnav_bridge` 时同时构建所需修正接口。
包内 Python 测试使用 `ament_cmake_pytest`，机载依赖安装与检查入口已包含此依赖。
extnav 专用产物安装在 `build/test-extnav/install/`，不会替换生产外部桥。
也可用 `EXTNAV_CPP_EXECUTABLE` 显式选择对照产物。

选择 `flight`/`waypoints` 时执行已有 `build/onboard_control` 内对应 CTest；缺少构建产物会失败，
还会核对所选套件全部注册，不能把旧构建只跑部分测试当作完整通过。入口不会自动判定源码与
二进制是否一致，不能把旧二进制通过当成新源码验证通过。`waypoints` 的集成层分别执行
identity/extnav 两种桥模式、后端切换及三入口参数回读；这些也注册为包内 CTest。
`correction` 同时执行包内 Python 回归；生产 C++ 等价测试缺产物会明确失败，不再默认跳过。

`--level all` 为默认完整检查；`unit` 执行纯逻辑、Qt 替身与纯 C++，`integration` 执行标记的
真实 ROS/媒体/网络、模态倒计时或进程生命周期测试。新增真实外部交互用例应添加 `pytest.mark.integration`。
层级选择使用 pytest 的 `-m`，自定义标记表达式请在默认 `all` 层级使用。
`unit` 仍包含所选业务组的纯 C++；只需界面或某个 Python 用例时使用专项或 `--tests`。
层级标记在收集后过滤，所选文件的导入依赖仍需可用。测试系统自检始终完整执行，不应用产品的 pytest 过滤选项。

测试进程清理仅扫描本 pytest 的后代；启动偏好、通知队列和媒体配置使用临时目录。
媒体后台轮询默认隔离，真实 HTTP 用例使用临时本机服务，不读取现场媒体状态。

优先用 `--tests` 执行单个目录或用例。直接调用 pytest 时需自行 source ROS 与本地 overlay：

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests/waypoints/test_waypoint_io.py
```

直接 `pytest tests` 是显式全量 Python 测试，不附带 C++/包内测试；推荐使用上述统一入口。
测试失败不会被吞掉；pytest 的 skip 仍会如实显示，不能当作通过。
