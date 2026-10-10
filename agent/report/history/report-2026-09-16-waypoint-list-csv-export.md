# 地面站航点列表 CSV 导出

## 1. 结论

已在地面站航点编辑区的“从文件导入”右侧增加“导出到文件”按钮。按钮只读取 GUI 当前实际
列表，不发送航点、不查询机载任务，也未修改任何机载服务。

- 列表为空时按钮为灰色禁用状态；列表非空后启用。
- 点击后打开系统保存文件选择器，可选择目录和文件名。
- 默认目录为项目根目录 `export/`，默认文件名为
  `waypoints-export-YYYYMMDD-HHMMSS.csv`。
- 保存选择器只显示 CSV；未输入扩展名时自动补 `.csv`，显式输入其他扩展名会拒绝写入。
- 导出列严格为 `index,x,y,z,yaw`，编号从 1 开始，位置单位为米，Yaw 单位为角度，与现有
  导入格式一致。
- 导出在任务执行中或环境会话不可编辑时仍可使用，因为它是对当前 GUI 列表的只读快照。

## 2. 修改范围

- `ground_station_core/waypoint_io.py`
  - 增加默认文件名、CSV 保存过滤器、导出校验和 CSV 写入。
  - 保持导入使用的五列顺序、数量限制、数值边界和角度语义。
- `ground_station_core/qt_ui/waypoint_panel.py`
  - 增加导出按钮、信号、空列表门控、提示和无障碍名称。
  - 将操作行间距压缩到 2 px，确认在 420 px 面板宽度下两个文件按钮文字均完整显示。
- `ground_station_core/qt_ui/main_window.py`
  - 创建默认 `export/` 目录，打开保存选择器，记录成功或失败日志。
- `ground_station_core/qt_ui/theme.py`
  - 导出按钮沿用导入按钮的白色、悬停和灰色禁用样式。
- `.gitignore`
  - 忽略本地运行时生成的 `/export/` 航点文件。
- `tests/test_waypoint_io.py`、`tests/test_qt_gui.py`
  - 增加格式往返、命名、扩展名限制、按钮状态、位置、默认路径和不触发机载发送的回归。
- `TODO.md`、`MEMORY.md`
  - 标记本项完成并维护当前功能基线。

## 3. 验证

在项目 `.venv` 中完成以下验证：

1. 航点 I/O 与 Qt GUI 定向测试：`64 passed`。
2. 加载 `/opt/ros/jazzy/setup.bash` 和工作区 `install/setup.bash` 后运行完整测试：
   `248 passed`。
3. Flake8 致命错误、未定义名称和 88 字符行宽检查：通过。
4. `git diff --check`：通过。
5. 420 px 宽离屏布局检查：导入、导出按钮实际宽度均为 85 px，与各自 size hint 一致，
   没有文字裁切。

首次未加载工作区 `install/setup.bash` 的完整测试出现 `guided_interfaces` 和
`correction_interfaces` 无法导入；按项目规定加载 ROS/工作区环境后全部通过，确认不是本次
功能回归。

## 4. 未覆盖与风险

- 未连接或操作实机，未启动仿真，未执行解锁或起飞。
- 没有修改 ROS 接口、机载服务、飞控参数或已发送到飞机的任务。
- 系统保存对话框的外观与覆盖已有文件时的确认行为由桌面环境提供；应用层仍会校验最终扩展名
  和写入错误。
