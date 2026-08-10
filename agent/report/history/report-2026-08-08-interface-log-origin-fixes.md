# 协议版本、MAVROS 日志与原点文案修复报告

日期：2026-08-08

## 1. 结论

按审计结果完成三项限定修复，未改动 PD+DOB 算法、控制参数、航点策略行为或其他无关功能：

1. `ExecuteWaypoints` 请求结构对应的高层协议由 `1.0` 升级为 `2.0`，两个相关 ROS 包同步升为 `2.0.0`；旧 `1.0` 机载端会在命令进入传输层前明确拒绝。
2. MAVROS 启动日志分类覆盖生产受管进程名 `mavros_sim`，普通 ROS `[INFO]` 降为 DEBUG，显式 WARN/ERROR 保持原等级。
3. GUI 原点文案全部统一为“缓存原点仅在连接实机时写入；本地 SITL 使用自身 Home”。

用户已有的未提交 `TODO.md` 修改和先前审计报告均原样保留。

## 2. 修改内容

### 2.1 协议版本同步

- `ground_station_core/config.py`：`INTERFACE_VERSION = "2.0"`；
- `src/onboard_control/src/onboard_control_node.cpp`：机载发布版本同步为 `2.0`；
- `src/guided_interfaces/package.xml`：包版本升级为 `2.0.0`；
- `src/onboard_control/package.xml`：包版本升级为 `2.0.0`；
- 测试不再把正常快照硬编码为 `1.0`，统一引用地面站版本常量；
- 新增版本同步回归，校验 Python、C++ 和两个包的版本一致；
- 新增旧版门控回归：新地面站收到 `interface_version=1.0` 时，在读取任何 ROS 传输实体前返回“接口版本不兼容”。

选择主版本 `2.0` 的原因：新增 `flight_strategy` 改变了 ROS 服务请求结构，与旧生成类型不是线级兼容变更；使用主版本升级能避免把旧机载端错误识别为兼容节点。

### 2.2 真实 MAVROS 源名日志分类

- `ProcessSupervisor._CHATTY_SOURCES` 增加 `mavros_sim`；
- 单元测试直接使用 `EnvironmentInitializer` 的生产名称 `mavros_sim`；
- 测试样本使用不依赖额外 marker 的普通 MAVROS ROS `[INFO]`，避免因 `plugin loaded` 等关键字偶然通过；
- WARN/ERROR 优先级逻辑未修改。

对审计期间保存的真实 `/tmp/ros2_ardupilot_ground_station/mavros_sim.log` 重放结果：

```text
ROS [INFO] 样本总数：410
分类结果：DEBUG 410
显式 WARN：仍为 WARN
```

这覆盖了此前审计发现的 `Starting mavros_node container`、`GCS URL`、`Starting mavros router node` 等实际刷屏行。

### 2.3 原点 UI 文案

仅修改 `ground_station_core/qt_ui/operations_panel.py` 中与原点语义直接相关的内容：

- 原点配置对话框提示；
- 环境卡说明；
- 原点齿轮初始 tooltip；
- 原点访问函数文档；
- 原点摘要；
- “启动本地仿真”按钮 tooltip。

现在界面明确区分：

- 本地 SITL：使用自身 Home，不读取或写入 GUI 缓存原点；
- 实机连接：确认后把 GUI 缓存原点写入飞控。

现有功能测试继续断言仿真工作流不接收原点、实机工作流接收缓存原点，并新增界面字符串断言防止旧文案回归。

## 3. 验证结果

### 3.1 Python/Qt

```text
QT_QPA_PLATFORM=offscreen python3 -m pytest -q
27 passed in 24.10s
```

定向测试最终结果：

```text
tests/test_bootstrap.py
tests/test_event_log.py
tests/test_ros_controller.py
tests/test_qt_gui.py
24 passed in 24.61s
```

第一次运行新增版本门控测试时，测试夹具直接写入快照但没有设置状态新鲜度，因此先得到“机载控制服务不可用”，没有走到版本分支。该问题只存在于新测试夹具；补充新鲜时间后，测试按预期验证旧 `1.0` 被版本门控拒绝。没有通过放宽断言掩盖该失败。

### 3.2 ROS 构建与 C++ 测试

```text
colcon build --packages-select guided_interfaces onboard_control guided_sim
三个包 colcon_build.rc 均为 0

colcon test --packages-select guided_interfaces onboard_control guided_sim
colcon test-result --verbose
Summary: 5 tests, 0 errors, 0 failures, 0 skipped
```

安装空间校验：

```text
install/guided_interfaces/.../package.xml = 2.0.0
install/onboard_control/.../package.xml = 2.0.0
```

### 3.3 静态与环境检查

```text
python3 -m compileall -q ...
通过

flake8（本次修改的 Python 文件，E9/F63/F7/F82/E501）
通过

git diff --check
通过

python3 ground_station.py --check-environment
[GS] workspace environment OK: guided_interfaces + rclpy available
```

## 4. 部署注意事项

- 此版本地面站与机载服务必须一起部署 `guided_interfaces 2.0.0`；旧 `1.0` 节点会被有意拒绝。
- 机载端升级后需要重新构建并重启 `onboard_control_node`，否则它仍会发布旧版本状态或使用旧服务类型。
- 本轮没有改变未实现策略的行为：自动避障/遇障悬停仍会告警并按直线执行。

## 5. 文件清单

- `ground_station_core/config.py`
- `ground_station_core/process_manager.py`
- `ground_station_core/qt_ui/operations_panel.py`
- `src/guided_interfaces/package.xml`
- `src/onboard_control/package.xml`
- `src/onboard_control/src/onboard_control_node.cpp`
- `tests/test_bootstrap.py`
- `tests/test_event_log.py`
- `tests/test_qt_gui.py`
- `tests/test_ros_controller.py`
- `MEMORY.md`
- `agent/report/report-2026-08-08-interface-log-origin-fixes.md`
