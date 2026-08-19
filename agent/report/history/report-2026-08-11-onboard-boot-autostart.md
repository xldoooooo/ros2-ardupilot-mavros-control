# 真机机载启动脚本开机自启动配置简报

日期：2026-08-11
目标机：`xld@192.168.112.186`（Ubuntu 22.04.5 / ROS 2 Humble / aarch64）

## 目标与安全边界

- 将真机已有的
  `/home/onboard/ros2-ardupilot-mavros-control/start_drone_all.sh` 配置为开机自动启动。
- 不解锁、不起飞，不发送飞行动作；本次不重启真机，也不在配置完成后立即启动服务。
- 不覆盖真机仓库已有未提交改动，不更新脚本或 ROS 包。

## 实施内容

安装并启用了系统级 unit：

```text
/etc/systemd/system/ros2-ardupilot-onboard.service
```

服务关键配置如下：

- `User=xld`、`Group=xld`；
- 工作目录为 `/home/onboard/ros2-ardupilot-mavros-control`；
- 明确加载 `/opt/ros/humble`、项目 install、`/home/xld/ws` Odin install 和
  `/home/xld/vrpn_mavros` extnav install；
- `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`；
- `Restart=on-failure`、`RestartSec=10s`；
- `KillMode=mixed`、`KillSignal=SIGINT`，让现有监督脚本优先按自身清理路径停止四个组件；
- `WantedBy=multi-user.target`。

仅执行了 `systemctl enable ros2-ardupilot-onboard.service`，没有执行 `systemctl start` 或
`enable --now`。

## 检查与验证

- 真机 ROS、四个 overlay、`ros2/setsid/stdbuf/tee/timeout/pgrep` 命令均可用；
- `mavros`、`odin_ros_driver`、`extnav_bridge`、`onboard_control` 包索引均可解析；
- `xld` 对 `/dev/ttyTHS1` 具有读写权限；
- `systemd-analyze verify` 对新增 unit 无本服务错误；输出中 snapd/NVIDIA 既有 unit 的警告与
  本服务无关；
- `systemctl is-enabled`：`enabled`；
- `systemctl is-active`：`inactive`；
- `ActiveState=inactive`、`SubState=dead`、`UnitFileState=enabled`；
- 最终 MAVROS、Odin、extnav、onboard 和启动脚本进程均为零，`/dev/ttyTHS1` 无占用。

因此自动启动配置已持久化，但本次没有通过重启做首次真实开机验证。用户下次手动开机后应检查：

```bash
systemctl status ros2-ardupilot-onboard.service --no-pager
journalctl -u ros2-ardupilot-onboard.service -b --no-pager
```

如需阻止下一次开机启动：

```bash
sudo systemctl disable ros2-ardupilot-onboard.service
```

## 执行中发现的问题与处置

远端脚本 SHA-256 为
`4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`，是可移植入口改造前的
旧版本。它依赖调用终端预先 source 四个 ROS 环境，而且并未实现后来文档所述的真正
`--check` 分支。

第一次在纯净环境调用 `--check` 时因 `ros2` 不在 PATH 而安全退出。随后为验证完整依赖而预先
source 四个环境后再次调用时，旧脚本忽略 `--check` 并短暂启动了 MAVROS、Odin、extnav 和
onboard。发现后立即向监督脚本发送 SIGINT，由脚本完成整组清理。只读 `/mavros/state` 样本为：

```text
connected: true
armed: false
guided: false
mode: STABILIZE
```

清理后复核四组件零残留、串口无人占用。期间没有人工请求模式切换、控制租约、解锁或起飞；
该事件未被隐瞒，也不能把这次调用报告成成功的“无启动检查”。后续在远端脚本升级前，不得对
该版本使用 `--check` 期待只读行为。

## 未验证项

- 未重启真机，尚未实测本 unit 在真实 boot ordering 下的首次启动结果；
- 未主动启动新增服务，因此本次没有生成该 unit 的完整 readiness journal；
- 真机仓库已有未提交修改和未跟踪部署文件，本次保持原状。
