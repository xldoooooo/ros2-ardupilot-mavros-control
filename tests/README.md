<!-- 自动化测试入口、功能闭环归属与环境要求。 -->
# 自动化测试

日常修改按功能闭环回归，只有共享模块变更或明确验收才运行全部测试。
所有命令从仓库根目录执行，不连接飞机；ROS 测试使用 localhost 和隔离 domain。

```bash
# 默认比较 HEAD 与当前工作区，包含暂存、未暂存和未跟踪文件
bash tests/run.sh

# 先看选择结果；不会导入或收集未选组的测试
bash tests/run.sh --changed HEAD --list

# 验证已提交改动：基线之后的差异也会计入
bash tests/run.sh --changed HEAD~1

# 按本次任务实际涉及的文件选择，避免其他工作区改动干扰
bash tests/run.sh --files correction_service/correction_service/window.py

# 显式选择一个或多个闭环
bash tests/run.sh --group video
bash tests/run.sh --group flight waypoints

# 功能组内只运行轻量检查，或只运行真实 ROS/媒体等集成检查
bash tests/run.sh --group correction --level unit
bash tests/run.sh --group waypoints --level integration

# pytest 选项放在 -- 后；C++ 检查仍按所选闭环执行
bash tests/run.sh --group upstream -- -x

# 重构或发布前明确执行全量
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

`support/` 保存共享 Qt 替身、DDS 节点 fixture 与 extnav 进程助手，不包含测试。大型 Qt 用例按上述闭环拆开，
不能再通过导入某个 `test_*.py` 来复用替身，否则可能重复收集或产生跨组依赖。

## 改动影响规则

`run.py` 的 `RULES` 是唯一影响映射，命中多条规则时取并集：

- 独立面板按功能选择；CSV、接口、状态投影等共享契约同时选择实际消费组。
- Odin 驱动触发运行、飞行和修正；导航切换与独立避障入口触发运行、飞行和航点。
- 修改机载主节点会触发飞行、航点和视频，因为它发布视频事件；不触发独立 Tag 修正。
- `main_window.py` 是所有面板的组合入口，`window_chrome.py` 被三个窗口共用，触发相关全部组。
- 接口、环境和部署共用代码按实际消费方联动；测试目录的修改触发所属组。
- Markdown、文档与任务记录不触发回归。未知代码路径会报错并要求补充映射或显式选择，
  不会静默漏测，也不会自动扩大为全量。
- Git 重命名按旧路径删除和新路径增加处理。默认 `HEAD` 不包含已提交且工作区干净的修改，
  此时必须指定提交前的基线。

增加功能模块或调用依赖时，同步更新规则和 `test_selection.py` 的边界用例。
选择器还检查已跟踪产品源码是否全部登记，避免新增源码长期游离于影响映射之外。
现有 `dev/` 示例不属于产品闭环，不自动执行；变更这些代码时应使用该示例自身的测试入口。
FPGA 模块保留独立的 `bash fpga_avoidance/test.sh`；修改该目录时统一入口会提示此命令，
不会把可拆卸模块导入默认产品组。

## 环境与结果边界

`run.sh` 使用项目 `.venv`，加载 Jazzy、可用的原生规划器 overlay 和本仓库 overlay；不会构建、部署、
重启服务或操作实机。接口/C++ 修改后先构建当前产物：

```bash
bash tests/build_native.sh
bash tests/run.sh --all
```

构建入口使用全部本机 CPU，不启动服务；要求独立规划器工作区已经构建，位置由
`AVOIDANCE_WORKSPACE` 指定，缺省为同级 `dyn_small_obs_avoidance-ros2`。
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

测试进程清理仅扫描本 pytest 的后代；启动偏好、通知队列和媒体配置使用临时目录。
媒体后台轮询默认隔离，真实 HTTP 用例使用临时本机服务，不读取现场媒体状态。

仍可直接执行单个目录或用例（需自行 source ROS 与本地 overlay）：

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests/waypoints/test_waypoint_io.py
```

直接 `pytest tests` 是显式全量 Python 测试，不附带 C++/包内测试；推荐使用上述统一入口。
测试失败不会被吞掉；pytest 的 skip 仍会如实显示，不能当作通过。
