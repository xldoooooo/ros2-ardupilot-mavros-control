# 摄像头面板 Dock 名称乱码修复报告

日期：2026-08-17

## 修复结果

Ubuntu Dock 中摄像头程序名称的乱码来自独立 Qt 进程使用的中文应用名被桌面环境按错误
编码解释。面板内部中文界面不受影响，所有可能被桌面环境读取的名称现统一为纯 ASCII：

`ROS 2 ArduPilot Camera Panel`

具体包括：

- Qt `applicationName`；
- Qt `applicationDisplayName`；
- 顶层窗口 `windowTitle`。

应用名和显示名在创建 `QApplication` 之前设置，避免桌面连接初始化后才修改元数据。面板
内部标题、按钮和配置项继续使用中文。已经运行的旧面板仍保留启动时的旧名称，需要关闭
并重新从地面站打开一次才能看到新 Dock 名称。

## 验证

- Qt 离屏实例实测三个字段均等于 `ROS 2 ArduPilot Camera Panel`；
- 新增窗口标题为纯 ASCII 的自动化回归测试；
- 摄像头定向测试：14 passed；
- source ROS Jazzy 与项目 overlay 后全量测试：122 passed；
- `py_compile`、致命 Flake8（E9/F63/F7/F82）和 `git diff --check`：全部通过。

本轮没有启动摄像头，没有改变摄像头后台和用户配置；没有连接或控制飞控，没有解锁、
起飞或发送飞行命令。用户已有 `TODO.md`、任务文件和旧 `video-service/demo/` 未修改或
纳入提交。
