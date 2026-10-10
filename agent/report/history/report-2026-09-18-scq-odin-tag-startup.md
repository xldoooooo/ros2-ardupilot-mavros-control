# scq 地面机 Tag-Odin 面板启动修复

记录现场根因、部署修复与验证边界。

- 目标：`scq@192.168.112.101`，项目 `/home/scq/ros2-ardupilot-mavros-control`，源码 HEAD `59df1d6`，修复前工作树干净。
- SSH 主机密钥变化经用户确认后更新，本次未连接或部署飞机。
- 复现：面板 ROS 客户端无法从已安装 `correction_interfaces.srv` 导入 `ApplySavedCorrection`。
  源码已同步，但 install 仍为旧接口，最近构建日志停留在 9 月 1 日。
- 修复：加载 Jazzy，执行 `colcon build --packages-select correction_interfaces correction_service --cmake-args -DCMAKE_BUILD_TYPE=Release`；两包成功，耗时 14.4 秒。无需修改面板业务代码。
- 文档：README 补充源码同步后的构建步骤与该导入错误的处理方法；MEMORY 更新部署事实。

## 验证

- 目标机器使用项目 `.venv`，实际创建 Qt 面板，离屏事件循环运行 5 秒，ROS 线程存活且 `startup_error` 为空。
- 收到新鲜 correction/extnav 2.0 状态、raw/corrected Odin、FCU 输入位姿。仅订阅，未发送校准、应用、清空、飞行或解锁命令；收到的 applied_ack 是既有服务状态，不是本次操作。
- 面板专项：`test_correction_panel.py`、`test_correction_gui.py` 共 11 passed。
- correction_service 包测试：4 passed、1 failed。失败为旧测试要求 Tag0 尺寸 0.170 m，但当前生产配置为 0.099 m；未更改生产尺寸或测试断言来掩盖失败。
- 未验证人工桌面显示或飞行；本次订阅未收到 MAVROS final 位姿，不将其报告为整条飞控链路正常。
- 验证面板已正常关闭。用户重新启动地面站及独立面板即可加载修复后的接口。
