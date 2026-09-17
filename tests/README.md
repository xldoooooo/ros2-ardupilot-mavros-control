<!-- 自动化测试入口、功能闭环归属和本次重构验收记录。 -->
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

# pytest 选项放在 -- 后；C++ 检查仍按所选闭环执行
bash tests/run.sh --group upstream -- -x

# 重构或发布前明确执行全量
bash tests/run.sh --all
```

| 组 | 功能闭环 | 包含的检查 |
| --- | --- | --- |
| `runtime` | 启动 → 环境连接/切换 → 清理退出 | 自举、进程、环境通讯、部署入口、Qt 连接/退出 |
| `flight` | 权限/就绪 → 命令 → 权威状态/恢复 | ROS 客户端、参数读取、遥测、热重启、飞行门控、DOB C++ |
| `waypoints` | 导入/编辑 → 预览/上传 → 到点进度/终态 | CSV、Qt 航点、ROS 传输、RViz、参考生成/到点判定 C++ |
| `upstream` | WebSocket 指令 → GUI/任务编排 → 状态回传 | 协议、映射、通讯、巡检/返航/入库、上位机面板 |
| `video` | 配置/采集 → 推流/录像/抓拍 → 回执/关闭 | 独立进程/面板、ROS 视频、飞行事件联动、视频部署 |
| `correction` | 采样 → 几何/滑窗 → CAS 应用 → 面板/释放 | 相机、Tag 方向、同步、估计、窗口、节点事务、extnav、包级回归 |
| `ui` | 界面基础状态与呈现 | Qt 平台、日志、窗口、刷新行为 |

`support/qt.py` 仅保存共享替身与窗口清理，不包含测试。大型 Qt 用例按上述闭环拆开，
不能再通过导入某个 `test_*.py` 来复用替身，否则可能重复收集或产生跨组依赖。

## 改动影响规则

`run.py` 的 `RULES` 是唯一影响映射，命中多条规则时取并集：

- 独立视频、修正、上位机、CSV 或航点面板只触发对应闭环。
- 修改机载主节点会触发飞行、航点和视频，因为它发布视频事件；不触发独立 Tag 修正。
- `main_window.py` 是所有面板的组合入口，`window_chrome.py` 被三个窗口共用，触发相关全部组。
- 接口、环境和部署共用代码按实际消费方联动；测试目录的修改触发所属组。
- Markdown、文档与任务记录不触发回归。未知代码路径会报错并要求补充映射或显式选择，
  不会静默漏测，也不会自动扩大为全量。
- Git 重命名按旧路径删除和新路径增加处理。默认 `HEAD` 不包含已提交且工作区干净的修改，
  此时必须指定提交前的基线。

增加功能模块或调用依赖时，同步更新规则和 `test_selection.py` 的边界用例。
现有 `dev/` 示例不属于产品闭环，不自动执行；变更这些代码时应使用该示例自身的测试入口。

## 环境与结果边界

`run.sh` 使用项目 `.venv`，加载本机 Jazzy 和当前 `install/local_setup.bash`；不会构建、部署、
重启服务或操作实机。修改接口/C++ 后，先按项目现有流程构建对应包，再运行测试。
选择 `flight`/`waypoints` 时执行已有 `build/onboard_control` 内对应 CTest；缺少构建产物会失败，
不能把旧二进制通过当成新源码验证通过。选择 `correction` 时同时执行其 ament 包内 Python 回归。

仍可直接执行单个目录或用例（需自行 source ROS 与本地 overlay）：

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests/waypoints/test_waypoint_io.py
```

直接 `pytest tests` 是显式全量 Python 测试，不附带 C++/包内测试；推荐使用上述统一入口。
测试失败不会被吞掉；pytest 的 skip 仍会如实显示，不能当作通过。

## 2026-09-17 重构记录

按“只改 test”要求，所有修改与本记录均位于 `tests/`，未更新根文档、MEMORY 或 agent/report。
已有生产代码、文档工作区改动未纳入此次重构。

删除三项不验证功能结果的检查：Qt success hover CSS 字符串、视频面板 disabled CSS 字符串、
Qt 内部 opaque paint 标志。保留禁用/可用状态、确认交互、窗口行为、故障隔离和全部算法/事务回归。
原自举测试中夹带的视频抓拍顺序与 QoS 检查迁至视频组，飞行协议版本检查迁至飞行组。

初始未 source overlay 的全量结果为 257 passed、12 failed、13 errors、1 skipped，
失败原因是找不到生成的 `guided_interfaces` / `correction_interfaces`。统一入口加载现有
overlay 后验证，不通过增加 skip 或删除这些用例规避环境问题。

最终验证：全量功能/包级 Python **286 passed**，选择器 **17 passed**，C++ **3 个套件通过**。
七组还分别在独立 pytest 进程执行通过：runtime 51、flight 34、waypoints 32、upstream 25、
video 42、correction（含包级）91、ui 11；未出现 skip。Shell 语法、未使用导入/未定义名称检查、
Git diff 空白检查通过。C++ 使用当前已有构建产物，本次没有修改或重新构建生产源码。

修正组独立执行退出时观察到一次 rclpy 异步清理消息：
`The following exception was never retrieved: cannot use Destroyable because destruction was requested`。
该次 91 项断言全部通过且退出码为 0；未屏蔽消息，也未据此修改生产节点。该异步清理现象尚未定位，
不能将本轮结果表述为退出日志完全无异常。
