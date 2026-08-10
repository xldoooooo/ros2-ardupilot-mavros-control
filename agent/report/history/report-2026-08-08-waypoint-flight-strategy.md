# 航点飞行策略下拉与接口预留

## 结论

航点「发送并执行」区拆为左按钮 + 右策略下拉；三种策略均已走通 UI→ROS 参数路径，**执行仍统一为直线飞行**。

| 选项 | 协议值 | 当前行为 |
| --- | --- | --- |
| 直线飞行 | `0` `STRATEGY_STRAIGHT` | 已实现 |
| 自动避障 | `1` `STRATEGY_AVOID` | 预留，按直线执行 |
| 遇到障碍悬停 | `2` `STRATEGY_HOVER_ON_OBSTACLE` | 预留，按直线执行 |

## 改动

- `guided_interfaces/srv/ExecuteWaypoints.srv`：`flight_strategy` 与常量
- `models.WaypointFlightStrategy`：GUI/客户端枚举
- `waypoint_panel`：半宽按钮 + `QComboBox`
- `ros_controller.request_waypoints(..., strategy=)` 填入服务请求
- 机载 `on_execute_waypoints`：非直线策略打 WARN 后仍直线执行
- 测试与 `MEMORY.md`

## 验证

```text
colcon build --packages-select guided_interfaces onboard_control
pytest -q tests/test_qt_gui.py tests/test_ros_controller.py
```
