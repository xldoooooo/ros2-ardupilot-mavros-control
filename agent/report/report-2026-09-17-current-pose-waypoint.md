# 当前位置追加航点执行报告

本报告记录地面站航点打点功能的实现与验证范围。

- 在航点输入行“+”右侧增加 28×28 定位准星按钮，提供中文悬停提示与无障碍名称。
- 主窗口将显示实际位姿的同一份 VehicleSnapshot 传给航点面板，点击时直接追加原始
  x/y/z/yaw 至末尾并选中；保持输入框与既有航点，不取目标位姿、不取整或限幅、不自动上传。
- 沿用已有编辑门控；机载离线、飞控断开、位置无效或非有限数时禁用打点。
- 未改动机载逻辑、协议或依赖，未连接、解锁或起飞任何实机；两台飞机均无需同步本功能。
- 更新对应 TODO 和 MEMORY 当前基线；保留任务开始前已有的 README、部署文档和 TODO 其他改动。

## 验证

运行项目环境：
`QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests/test_qt_gui.py tests/test_waypoint_io.py tests/test_waypoint_visualization.py`

最终 **68 passed（28.86 秒）**。新增测试覆盖原始精度、追加顺序、同一显示帧、更新后再次
打点、输入不变、未发送命令、无效位姿禁用，以及图标位置与正方形尺寸。
首轮为 1 failed / 67 passed：通用 compact 样式把按钮撑高至 34px，修复为复用“+”按钮
专用尺寸样式后全部通过。离屏 Qt 截图已人工检查，证据在
`agent/codex/current-pose-waypoint.png`（本地过程文件）。

未进行真实遥测或飞行验收；本次验证使用 Qt 窗口与替身快照。全工作区 diff 检查发现
任务前已有的 README.md 第8行尾随空格，本次未修改该文件。
