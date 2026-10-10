# 任务 24：多个 Bug 修复执行简报

## 任务结论

任务 24 的四项问题均已定位并修复：

1. Ubuntu 24.04 / GNOME Wayland 下，上位机通讯面板和摄像头面板改为在 Qt 初始化前优先选择
   `adwaita` Wayland 窗口装饰；X11、非 Wayland 会话以及用户显式指定的装饰插件不受影响。
2. 飞机已武装且可靠控制链可用时，LAND 不再被 GUI 工作流忙碌、当前飞行模式、重复 LAND、
   本地位置或推力诊断错误地置灰；上位机组合任务中的重复降落会接管新命令票据，避免旧票据终态
   中断组合流程。
3. 摄像头无有效帧时由普通 Qt 控件逐像素绘制不透明纯黑背景，原生视频 surface 保持隐藏；只有
   收到首个有效视频帧才显示画面，断帧、播放停止、无效媒体和 1.5 秒帧超时都会恢复纯黑。
4. 航点可靠终态中的 `waypoint_index/waypoint_count` 现会完整传递到 GUI；仅当前任务的最终成功
   结果可把进度补为 `N/N`，不会被 LAND 状态切换覆盖，也不会被旧任务的迟到结果误填。

任务没有修改机载 C++ 控制器或线协议定义，现有接口版本保持 3.2。

## 根因与修复

### Bug 1：另一台 Ubuntu 24.04 缺少窗口边框阴影

在 `scq@192.168.112.101` 的实际 GNOME Wayland 会话中，Qt 6.11.1 默认加载
`libbradient.so` 装饰插件。诊断显示其窗口 frame margins 为 `(3, 30, 3, 3)`；改用系统已有的
`libadwaita.so` 后为 `(11, 49, 11, 11)`，恢复了边框留白和阴影范围。

地面站与独立摄像头两个入口现在仅在原生 Wayland 会话中、且用户没有显式配置时设置：

```text
QT_WAYLAND_DECORATION=adwaita
```

设置发生在导入 PySide6 和创建 `QApplication` 之前，避免 Qt 过早固定默认插件。

### Bug 2：飞行中 LAND 可能置灰，以及落地后的组合任务状态矛盾

原门控把 LAND 和普通动作共用 `busy`、当前模式及 pending 锁。因而以下路径都可能在已武装时
错误禁用 LAND：正在执行组合工作流、已有命令尚未结束、已进入 LAND 模式，或重复发送 LAND。

修复后 LAND 的 GUI 条件收敛为“可靠 ROS/机载链路存在、持有控制权、飞机已武装”，不受普通
工作流互斥、飞行模式、本地位姿、推力诊断或已有 LAND pending 影响。离线、关闭中、无控制权、
机载不可用或多个机载状态端点冲突时仍安全禁用，因为这些状态下 GUI 无法可靠下发命令。上位机的
正常/紧急 LAND 同样可绕过普通工作流 busy；重复 LAND 产生新票据后，组合任务及其降落状态投影
会重新绑定新票据，不会被旧票据的取消结果误判失败。

任务描述中的第二种现象确实可能发生：03 巡检组合在可靠落地后仍需等待默认 60 秒并满足机库
范围判定，组合状态不会立刻结束。旧实现却仅依据解除武装把“起飞”显示为可用，导致 GUI 和
上位机状态矛盾。本次没有删除机库判定，而是在组合真正结束前保持起飞禁用；只有组合释放后才
允许再次起飞。

### Bug 3：无画面时预览区透底、残缺或抽搐

旧实现把 `LoadedMedia/BufferedMedia` 当成已有视频帧，过早显示带原生子窗口的 `QVideoWidget`。
Wayland 合成器在尚无有效帧、断流或窗口拖动重绘时可能暴露未更新的原生 surface，表现为透出
下层窗口、残片或抽搐。

修复包括：

- 新增带 `WA_OpaquePaintEvent` 的纯黑预览控件，并在每次 paint 时覆盖完整脏区；
- 预览栈和占位控件均显式使用不透明黑色背景，黑区内不绘制文字；
- `LoadedMedia/BufferedMedia` 只表示已连接，不再显示视频 surface；
- 只有 video sink 收到有效帧才显示 `QVideoWidget`；无效帧、stalled/error/stop/no-media/end
  和播放期间超过 1.5 秒无新帧都会隐藏原生 surface；
- 暂停保留最后一帧，恢复播放时重置 watchdog 周期；状态说明移到预览区外。

### Bug 4：已执行全部航点并落地，进度仍为 `N-1/N`

航点完成后立即进入 LAND 时，`ControlStatus` 可能先切换到 LAND，最后一次航点进度快照来不及
把 GUI 更新为 `N/N`。机载可靠 `RemoteCommandResult` 已携带最终索引和总数，但地面端转换为
`CommandResult` 时丢弃了这两个字段。

现在可靠结果保留这两个字段；当前活动票据收到“最终成功且 index == count”的结果时补齐进度。
旧票据、失败结果、非最终结果以及非法计数均不能改写进度，避免修复竞态时引入跨任务污染。

## 变更文件

- `ground_station.py`
- `video_service/camera_panel.py`
- `video_service/camera_app/panel.py`
- `ground_station_core/models.py`
- `ground_station_core/ros_controller.py`
- `ground_station_core/qt_ui/state.py`
- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/waypoint_panel.py`
- `tests/test_qt_platform.py`
- `tests/test_camera_service.py`
- `tests/test_qt_gui.py`
- `tests/test_ros_controller.py`

## 验证结果

### 自动化与构建

- 定向 Qt/相机/ROS 控制器回归：`101 passed in 26.70s`。
- 项目完整 Python 测试：`176 passed in 40.59s`。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：三包全部成功。
- `colcon test`：`19 tests, 0 errors, 0 failures, 0 skipped`。
- `git diff --check`：通过。

新增/强化用例覆盖 Wayland 环境选择且不覆盖显式配置、首帧前纯黑、媒体仅加载不暴露 surface、
断帧 watchdog、暂停保帧、所有已武装飞行模式及 busy/pending 下 LAND 可用、重复 LAND 票据接管、
组合任务结束前起飞锁、最终航点进度补齐，以及旧票据迟到结果隔离。

### Ubuntu 24.04 / Wayland 目标机验证

在 `scq@192.168.112.101` 的真实 GNOME Wayland 会话中进行了独立验证：

- Qt 插件诊断确认加载 `libadwaita.so`，窗口 frame margins 从默认插件的 `(3, 30, 3, 3)` 变为
  `(11, 49, 11, 11)`；
- 无媒体和仅 LoadedMedia 状态下，预览中心及四角采样均为 `#000000`、alpha 255；
- 视频 surface 在首个有效帧前保持隐藏，预览区文字为空；
- 未启动摄像头后台或真实流，验证后无测试进程残留。

GNOME 的截图 D-Bus 接口因桌面权限返回 `AccessDenied`，因此没有绕过权限弹出截图门户；窗口装饰
采用 Qt 插件加载日志和实际 frame margins 验证，预览黑屏采用实际控件像素采样验证。

### 上位机 + 地面站 + SITL 全流程

第一次使用 `QT_QPA_PLATFORM=offscreen` 启动完整流程时，RViz/OGRE 因无 X11 parent window 报
`Invalid parentWindowHandle`，流程在起飞前安全终止；这是测试显示后端限制，不是本次功能失败。
随后改用本机真实 X11 会话重新执行完整 JAR + Qt + ROS + MAVROS + ArduPilot SITL 流程并通过：

- 接口版本 3.2；上位机命令 01、02、03、05、07 均收到 ACK；
- 巡检落地点约 `(0.477, 0.509, 0.003)`，返航落地点约 `(-0.068, 0.030, 0.016)`；
- 航点失败计数为 0，最终 `armed=false`、控制模式 idle；
- 已武装期间 LAND 按钮不可用的记录：`[]`；
- 组合任务仍处于待机阶段而起飞按钮误启用的记录：`[]`；
- 最终航点进度断言为 `2/2`；
- 一次平滑启动余速 `1/10` 重试按原逻辑恢复，未破坏既有异常恢复功能；
- 测试启动的 SITL、MAVROS、onboard、RViz 和临时 JAR 均已清理。

### 实机安全检查

本任务没有在实机上执行解锁、起飞、降落命令、控制租约操作或服务重启。最终只读检查
`nvidia@192.168.112.169` 返回：`connected=true`、`armed=false`、`mode=STABILIZE`。
飞行行为验证全部在 ArduPilot SITL 中完成。

## 未覆盖边界

- 未在真实飞行中复现和验证 LAND/航点竞态；这是安全边界要求，实机解锁与起飞只能由用户手动
  操作。本次通过状态机单测和完整 SITL 覆盖对应组合路径。
- 未获得 101 的桌面级截图；原因和替代证据已在上文如实说明。
