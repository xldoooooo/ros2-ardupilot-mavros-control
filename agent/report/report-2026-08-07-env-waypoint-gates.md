# 环境入口与航点组件门控

## 结论

已按需求收紧按钮策略，未改飞控算法或 ROS 接口：

1. **仿真或实机会话已建立**（`environment_active`）时，禁用「启动本地仿真」与「连接实机服务」；须先断开/清理后再开新会话；
2. **无环境会话**时，禁用航点面板全部操作组件（数值框、增删移清、表格、发送）；
3. 会话建立后仍可编辑航点；发送继续受原有起飞/租约/武装等飞行门控。

## 实现

- `ground_station_core/qt_ui/state.py`：`start_environment` 增加 `not environment_active`；`waypoint_edit` 要求 `environment_active`；
- 面板 tooltips 同步说明禁用原因；
- 回归：`test_availability_requires_explicit_environment_and_preserves_land` 加强断言；新增 `test_environment_session_gates_start_buttons_and_waypoint_widgets`。
