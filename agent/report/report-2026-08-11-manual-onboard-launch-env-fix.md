# 真机交互启动配置修复执行简报

日期：2026-08-11
任务：修复真机交互终端执行 `start_drone_all.sh` 时无法复用 systemd 已确认硬件配置的问题

## 一、问题与原因

真机交互终端直接执行 `./start_drone_all.sh` 时发现 `/dev/ttyTHS1` 与
`/dev/ttyTHS2` 两个候选，并按安全设计拒绝猜测。飞控使用的 `/dev/ttyTHS1`
此前只配置在 systemd drop-in；systemd 的环境不会自动进入用户的交互 shell。
串口歧义解除后，无启动检查又确认 Odin 和 extnav 的 overlay 路径也存在相同的
环境隔离问题。

## 二、改动

- `start_drone_all.sh` 在运行时发现前读取 `/etc/ros2-ardupilot/onboard.env`；可用
  `ONBOARD_ENV_FILE` 覆盖该路径。
- 文件存在但当前用户不可读时明确失败；文件不存在时保留原来的安全发现逻辑，
  不会自动选择多个串口或多个 overlay。
- 部署示例和文档补充 FCU、Odin overlay、extnav overlay 的机型专用配置方法。
- 真机 `/etc/ros2-ardupilot/onboard.env` 写入此前部署已经验证的三项选择：
  - `MAVROS_FCU_DEVICE=/dev/ttyTHS1`
  - `ODIN_OVERLAY_SETUP=/home/xld/ws/install/setup.bash`
  - `EXTNAV_OVERLAY_SETUP=/home/xld/vrpn_mavros/install/setup.bash`

## 三、本地验证

- `bash -n start_drone_all.sh`：通过。
- `tests/test_onboard_deploy.py`：10 passed。
- 完整 Python 测试：74 passed。
- 涉及文件 `git diff --check`：通过。

## 四、真机同步与无启动验证

- 真机正式工作树从 `50c5a18` 快进到修复提交 `72cc836`。
- 在没有给当前命令临时设置任何变量的情况下执行
  `./start_drone_all.sh --check`，成功识别：
  - ROS：Humble
  - 飞控：`/dev/ttyTHS1:460800`
  - Odin：`/home/xld/ws/install/odin_ros_driver`
  - extnav：`/home/xld/vrpn_mavros/install/extnav_bridge`
- 检查明确返回 `discovery check passed; no component was started`。
- 检查后 systemd 服务仍为 `inactive`、`Result=success`，四类飞行进程计数为 0，
  `/dev/ttyTHS1` 无占用。

## 五、安全边界与结果

本次没有启动四组件飞行栈，没有请求模式切换、解锁、起飞或控制指令。
真机交互启动的配置隔离问题已经修复；用户现在可以自行直接执行
`./start_drone_all.sh` 启动。解锁与起飞仍只能由用户手动操作。
