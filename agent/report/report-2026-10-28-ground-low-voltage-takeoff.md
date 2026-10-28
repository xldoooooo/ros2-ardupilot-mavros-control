<!-- 本次任务的行为范围、验证结果与部署状态。 -->
# 落地低电压起飞门控

日期：2026-10-28。

实机会话在机载回报 `on_ground=true`、电池遥测有效且整组电压低于22.2 V时，禁用起飞按钮并拒绝起飞请求。人工操作、上位机01和包含起飞的03巡检组合共用门控；与 WebSocket 是否连接无关。人工确认或上位机倒计时结束后再次检查最新状态，低电压时不调用 ROS 起飞接口、不遗留待发送命令或巡检组合。

- 唯一阈值移至 `ground_station_core/config.py:HARDWARE_LOW_VOLTAGE`，上位机0C告警复用该值。
- `ground_station_core/qt_ui/state.py` 统一计算落地低电压原因和按钮状态；`main_window.py` 在起飞入口复核并记录拒绝日志。首次进入禁用状态也记录日志，周期刷新不重复刷屏。
- 电压等于或高于阈值时解除本项限制，其余既有门控仍然生效。缺失、非有限或非正电压不新增拒绝条件。
- 离地后本项逻辑失效，保持现有空中控制、航点、悬停和降落入口；未修改机载代码或飞控电池保护参数。既有实机低电量提示/0C上报及仿真百分比返航行为保持。

验证命令（项目 `.venv`）：

```bash
.venv/bin/python -m pytest -q tests/flight/test_takeoff_voltage.py tests/flight/test_flight_gui.py tests/upstream/test_upstream_gui.py tests/upstream/test_upstream_countdown.py tests/upstream/test_upstream_communication.py
```

结果：57项通过，用时75.61秒。新增验证覆盖落地低电压按钮与01/03拒绝、确认期间电压下降、阈值边界恢复、空中控制不受影响，以及无效遥测和仿真模式不引入新限制；已有人工确认、上位机真实倒计时和协议投影测试同时通过。指定源码及新增测试的 `git diff --check` 通过。

部署：本次仅修改开发机源码，重开本机地面站加载；无新依赖，无需重建机载包。mini-ground、hp及两架飞机源码尚未同步，已记入MEMORY。未连接、解锁或起飞实机，现场遥测及实飞未验收。工作区原有修改与已暂存变更保留，不纳入本次提交。
