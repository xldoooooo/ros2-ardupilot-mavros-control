# 位姿发散 / RViz 三轴回归修复报告

## 1. 任务结论

已按审计结论完全修复「GUI 实际位姿间歇性天文数字」与「RViz 无人机三轴/机身不可见」问题，**效果恢复为 Grok 破坏仿真 `set_gp_origin` 之前的语义**：

| 项 | 修复后 |
| --- | --- |
| 本地仿真初始化 | **不再**调用 `set_gp_origin`；沿用 SITL 自身 Home 建 EKF |
| 实机连接初始化 | 仍写入 GUI 缓存原点（与历史正确行为一致） |
| 残留 `fake_pose` | 已终止；`/mavros/local_position/pose` 不再被调试假位姿争用 |
| 错误 MEMORY | 已删除/改写，禁止再写「仿真启动也 set_gp_origin」 |
| GUI 布局/组件 | **未改**（按用户要求只修功能回归） |

## 2. 根因回顾（与修复对应）

1. **主因**：工作区曾将杭州默认缓存原点在**仿真**路径强制 `set_gp_origin`，而 SITL 默认 Home 在 CMAC（澳洲）。local ENU 偏移约数百万米，与用户样本 `X≈+3.22e6 Y≈-7.30e6` 指纹一致。  
2. **间歇性**：调试残留的 `fake_pose` 进程向同一话题发布近原点假位姿，与 MAVROS 真值交替 → GUI/RViz 跳变。  
3. **RViz**：TF `map→base_link` 跟随后地位姿；机身跑到数百万米外时，默认 10 m 视角看不到三轴/模型。

## 3. 代码改动

### 3.1 `ground_station_core/environment.py`

- `initialize_simulation(status, done)`：去掉 `origin` 参数。  
- `_simulation_workflow`：恢复 1/5…5/5 步骤；`set_rates` 后直接 `_wait_local_position`，**不**调用 `request_set_gp_origin`。  
- 注释明确禁止在 SITL 上写与 Home 不一致的原点。  
- `_hardware_workflow`：保留 `set_gp_origin`。

### 3.2 `ground_station_core/qt_ui/main_window.py`

- `_initialize_simulation`：仅调用 `initialize_simulation(status, done)`，不再传入原点。  
- 不改动界面控件与布局。

### 3.3 `ground_station_core/config.py`

- `DEFAULT_GPS_ORIGIN` 注释改为：**仅实机**默认虚拟原点；本地 SITL 不用其建 EKF。

### 3.4 `tests/test_qt_gui.py`

- Fake 环境 `initialize_simulation` 不再接收 origin。  
- `test_origin_settings_are_local_and_applied_on_hardware_only`：断言仿真 `last_origin is None`，实机才应用缓存原点。

### 3.5 运行时清理

- 终止本机占用 `/mavros/local_position/pose` 的 `fake_pose` 调试进程。  
- 删除含临时 `vehicle_markers` 痕迹的 `pose_to_tf` 过期 pyc。

## 4. MEMORY 修正

删除错误记忆「启动仿真或连接实机时一并 `set_gp_origin`」。

正确记忆：

- 齿轮仅缓存原点；  
- **仅实机连接**写飞控原点；  
- **本地仿真禁止**写 GUI 缓存原点，否则 local ENU 可发散至数百万米。

## 5. 验证

```text
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest -q tests/test_qt_gui.py tests/test_event_log.py \
  tests/test_process_manager.py tests/test_ros_controller.py tests/test_bootstrap.py
24 passed in 21.73s
```

静态检查：

- `_simulation_workflow` 无 `request_set_gp_origin` 调用；  
- `_hardware_workflow` 仍有；  
- 本机无残留 `fake_pose` 进程。

## 6. 未做 / 限制

- **未改 GUI 文案**（齿轮摘要仍可能写「启动时写入原点」等）；功能上仿真已不写原点。若需文案与实机/仿真语义完全一致，可另开 UI 文案任务。  
- **未做完整在线 SITL 目视复飞**（本轮以代码回退 + 单元回归 + 进程清理为准）。操作者下次「启动本地仿真」后应看到：实际位姿在米级、RViz 默认视角可见机身/TF。  
- 若本机仍有其他匿名节点发布 `/mavros/local_position/pose`，需人工 `ros2 topic info` 确认发布者数为 1（MAVROS）。

## 7. 使用提示（修复后）

1. `python ground_station.py` 启动后点「启动本地仿真」。  
2. 初始化完成后 GUI「实际位姿」X/Y 应接近 0（起飞前通常亚米～数米量级），不应再出现数百万。  
3. RViz Fixed Frame=`map` 时，默认 Orbit 距离约 10 m 应能看到机身与 TF 轴。  
4. 实机连接仍会写入齿轮中缓存的原点；与仿真语义不同，属预期。
