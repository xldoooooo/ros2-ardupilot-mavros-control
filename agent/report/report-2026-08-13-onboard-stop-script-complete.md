# 真机机载服务彻底停止脚本修改简报

日期：2026-08-13
目标：最小修改真机 `/home/xld/stop_onboard_service.sh`，确保 systemd 或用户手工启动的机载 ROS/Odin 进程均被结束，执行后能够再次通过 `start_drone_all.sh` 启动前检查。

## 原因确认

- 原脚本只有 `sudo systemctl stop ros2-ardupilot-onboard.service` 一行。
- 该命令只能停止服务 cgroup，不能结束 GNOME Terminal 等其他 scope 中手工启动的进程。
- 故障现场残留 PID 29919（`ros2 launch odin_ros_driver odin1_ros2.launch.py`）及其 PID 29921（`host_sdk_sample`），因此 `start_drone_all.sh` 的重复进程保护正确拒绝启动。
- 本次现场还确认：独立 Odin 是 15:55:28 从桌面终端启动；原 systemd 栈内 MAVROS 于 15:55:39 因 `std::future_error: Promise already satisfied` 崩溃，服务清理自身组件后，独立 Odin 继续存活并阻断自动重启。

## 修改内容

真机脚本现在执行以下步骤：

1. 停止 `ros2-ardupilot-onboard.service`，阻止其自动重启。
2. 匹配所有来源的机载组件启动器和节点：`start_drone_all.sh`、MAVROS、Odin launch/driver、extnav、onboard_control 和 Odin 启动的 RViz。
3. 递归纳入匹配启动器的全部后代，避免遗漏命令名不明显的子进程。
4. 依次执行 SIGINT（等待 5 秒）、SIGTERM（等待 5 秒）、必要时 SIGKILL（等待 2 秒）。
5. 最终检查 systemd 服务 inactive 且目标进程为零；否则返回非零并打印残留。

停止入口随后正式加入仓库根目录 `stop_onboard_service.sh` 并设置 Git 可执行位；同步更新了：

- 机载 non-cone sparse checkout 清单和布局校验；
- 首次最小部署文件列表；
- README、Ubuntu 22.04/Humble 部署说明；
- 部署专项自动测试。

修改前备份：

```text
/home/xld/stop_onboard_service.sh.pre-codex-20260813-163351
```

修改后 SHA-256：

```text
87dc3c0d7675e96aa7553306cb19f124c9ee2510c549ef9ad24e3f8b35f502e3
```

## 真实验收

- `bash -n /home/xld/stop_onboard_service.sh`：通过。
- 首次执行识别 PID 29919/29921；SIGINT 等待后仍存活，SIGTERM 阶段成功退出，未使用 SIGKILL。
- `systemctl is-active ros2-ardupilot-onboard.service`：`inactive`。
- MAVROS、Odin、extnav、onboard_control、RViz 目标进程：零残留。
- `/dev/ttyTHS1`：无占用。
- `/home/onboard/ros2-ardupilot-mavros-control/start_drone_all.sh --check`：返回 0，输出 `discovery check passed; no component was started`。
- 检查后目标进程仍为零、串口仍无占用。
- 停止脚本第二次重复执行：返回 0，保持服务 inactive 和零残留。
- 本地部署专项测试：13 passed。
- 加载 Jazzy 和项目 overlay 后的正式 Python 回归：107 passed。
- 未加载项目 overlay 的一次全量运行结果为 104 passed、3 failed，失败均为
  `ModuleNotFoundError: guided_interfaces`；按项目运行环境 source 后全部通过，未隐瞒该环境错误。

本次没有正式启动机载四组件，没有发送飞行命令，没有解锁或起飞。

## 仓库发布与飞机同步

- 功能提交 `b6f9544` 已推送到远端 `main`。
- 飞机工作树从 `c71df1b` 快进到 `b6f9544`，`HEAD` 与 `origin/main` 一致。
- 飞机 sparse checkout 已加入 `/stop_onboard_service.sh`；Git 索引模式为 `100755`，实际文件可执行。
- 飞机项目文件 SHA-256：

```text
4e2f0ec30f3f9b2c39ee2c77d20ab4b05e4f8fb9ee8c087b2572b39d7c7ac248
```

- `/home/xld/stop_onboard_service.sh` 已移为
  `/home/xld/stop_onboard_service.sh.pre-project-move-20260813-210247`；原 51 字节脚本备份
  `/home/xld/stop_onboard_service.sh.pre-codex-20260813-163351` 继续保留。
- 同步时飞机 systemd 服务已由外部操作启动。同步前后均保持主 PID 4130、启动时间
  `2026-08-13 20:40:18 CST`、`NRestarts=0`、active/running，说明 Git 同步没有重启或扰动服务。
- 由于服务正在运行，`start_drone_all.sh --check` 正确报告现有 MAVROS/Odin/extnav/onboard
  进程；该检查仅适用于停止后的启动前状态。本次没有为获得通过结果而停止现有服务。

## 限制

- 本机和真机均未安装 ShellCheck，因此只完成 Bash 语法、真实进程清理、幂等停止和启动前检查。
- 飞机保留既有未跟踪 `.deployment-backups/`，本次没有修改或删除该目录。
