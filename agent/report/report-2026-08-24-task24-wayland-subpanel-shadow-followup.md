# 任务 24 复验：Wayland 子面板自绘阴影修复简报

## 问题复验

任务 24 第一版在 Ubuntu 24.04 原生 Wayland 会话中优先选择 Qt `adwaita` 窗口装饰。该修改恢复了
窗口装饰留边，但在 `scq@192.168.112.101` 的 GNOME Wayland 桌面上，合成器仍只显示很窄的白色
边框，没有开发机 X11 窗口管理器提供的外部投影。用户现场复验确认第一版未达到视觉要求。

根因是外部窗口阴影属于桌面合成器策略，切换 Qt 客户端装饰插件不能保证 GNOME Wayland 绘制
与 X11 相同的阴影。因此本次不再依赖外部装饰器，改为两个面板自己绘制完整投影。

## 修复内容

上位机通讯面板和独立摄像头配置面板现统一采用与地面站主窗口相同的窗口结构：

- 普通顶层窗口保持独立任务栏和非置顶语义；
- 使用 `FramelessWindowHint` 与 `WA_TranslucentBackground` 创建透明顶层表面；
- 四周保留 14 px 透明投影区，背景框使用 30 px blur、Y 偏移 3 px、
  `QColor(16, 30, 44, 118)` 的 Qt 自绘阴影；
- 自绘标题栏提供拖动、双击最大化、最小化、最大化/还原和关闭；
- 透明外沿仍支持四边和四角原生缩放，并显示对应缩放指针；
- 最大化时自动移除阴影留边、关闭投影并取消圆角，还原后恢复；
- 摄像头 `QVideoWidget` 没有挂载图形特效，仍由独立背景框在其后方绘制阴影，避免破坏原生视频
  surface、纯黑断流遮罩和播放性能；
- 关闭上位机面板仍只保存配置而不断开连接；关闭摄像头面板仍不停止摄像头后台。

第一版的 Wayland 装饰环境选择仍保留，但两个子面板的阴影已不依赖它。

## 自动化验证

- 新增/强化窗口回归覆盖：frameless/透明属性、独立窗口层级、30 px 投影、14 px 留边、实际半透明
  阴影像素、最大化去阴影、还原恢复以及最小化后从主界面重新打开。
- 两个面板定向回归：`82 passed in 26.83s`。
- 项目完整 Python 测试：`177 passed in 41.54s`。
- `git diff --check`：通过。
- 本次只修改 Qt/Python 显示层，没有修改 ROS 接口、飞行门控、机载控制器、摄像头后台或部署依赖。

## 101 真实 Wayland 验证

为避免验证阶段修改 101 的正式工作树，先在 `/tmp` 创建只含相关源码的临时树，并使用该机当前
GNOME Wayland 会话渲染两个无后台副作用的测试窗口。测试服务固定为断开状态，不连接 WebSocket、
不写用户配置；摄像头面板使用 `auto_bootstrap=False`，不启动摄像头服务或媒体进程。

验证结果：

- Qt 平台：`wayland`；
- 上位机面板：frameless=true、translucent=true、blur=30、offsetY=3；
- 摄像头面板：frameless=true、translucent=true、blur=30、offsetY=3；
- 两个窗口阴影采样均为 `RGBA(15, 31, 46, 33)`，证明透明留边内实际存在半透明投影；
- 两个窗口边框采样均为 `RGBA(133, 149, 165, 255)`，边框和窗口内容保持不透明；
- 客户区截图确认自绘标题栏、窗口控制按钮、内容布局和摄像头纯黑预览正常。

验证过程中没有停止或重启正式地面站、摄像头后台、ROS、MAVROS 或机载服务，也没有执行任何
实机解锁、起飞或降落操作。

## 变更文件

- `ground_station_core/qt_ui/upstream_panel.py`
- `ground_station_core/qt_ui/theme.py`
- `ground_station_core/qt_ui/window_chrome.py`
- `video_service/camera_app/panel.py`
- `video_service/camera_panel.py`
- `video_service/README.md`
- `tests/test_qt_gui.py`
- `tests/test_camera_service.py`
