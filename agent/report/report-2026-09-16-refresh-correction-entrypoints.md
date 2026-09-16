# refresh 飞机 correction_service 独立启停入口同步简报

## 目标与结果

将新飞机已验证的 correction_service 独立一键启停入口同步到
`drone-refresh (192.168.112.186)`：

```bash
cd /home/nvidia/ros2-ardupilot-mavros-control
./start_onboard_correction.sh
./stop_onboard_correction.sh
```

`start_onboard_correction.sh` 前台启动默认 idle 的 correction 2.0 节点；
`stop_onboard_correction.sh` 停止 systemd unit 和仅属于 correction_service 的手工残留进程。
两者均不启停 Odin、extnav、MAVROS、onboard_control 或视频服务。

## 现场基线与原因

refresh 飞机已有：

- correction_interfaces/correction_service 2.0 的 source 和 install overlay。
- `/home/nvidia/vins_odin_calib/camera_ws/install/setup.bash` 相机 overlay。
- `/etc/ros2-ardupilot/correction.env` 和完整校准配置。
- 旧版 `/etc/systemd/system/odin-correction.service`，状态为 disabled/inactive。

该机缺少根目录两个启停脚本，且 unit 仍内联了一整段 ROS source/
`ros2 run` 命令，尚未与人工启动共用同一入口。

## 备份与同步

覆盖前备份：

```text
/home/nvidia/backups/correction-entrypoints-refresh-20260916-cVSTEO/files-before.tar.gz
SHA-256: d6c54db92a39c1baae2ac0222f21392a4a3925b65675458b979e1ef322c16c80
/home/nvidia/backups/correction-entrypoints-refresh-20260916-cVSTEO/odin-correction.service.before
SHA-256: 46e23cbd99d570075f44aaa5d763837707a3073947bb7cdf3b2da8ba8a33a989
```

同步了两个根目录脚本，并同步与其相关的 unit 模板、安装器、sparse checkout
工作区脚本和 correction_service README。六个文件的 SHA-256 均与本地/main 一致。

实际 unit 已重新渲染为：

```text
ExecStart=/home/nvidia/ros2-ardupilot-mavros-control/start_onboard_correction.sh
```

`systemd-analyze verify` 通过该 unit；另外仅报告两个 NVIDIA 系统 unit 使用旧
`StandardOutput=syslog`，与本任务无关。本次未启用开机自启，保留 refresh 飞机原有
disabled 状态。

## 未解锁实测

- 通过 systemd 启动 unit，验证它实际调用新根目录启动脚本。
- 日志确认 correction_service 2.0 启动，默认 idle、相机保持关闭，`NRestarts=0`。
- 运行 `./stop_onboard_correction.sh` 后 unit 变为 inactive，无 `correction_node` 残留。
- 验收前后 `ros2-ardupilot-onboard.service` 和 `video-service.service` 均为 inactive，
  也没有 Odin、extnav、MAVROS 或 onboard_control 进程。
- 最终 `odin-correction.service` 为 disabled/inactive，与任务前状态一致。

## 安全边界

全程未解锁、未起飞，未发送飞行、模式或轨迹命令，未修改飞控参数。
本次只同步与验证 correction_service 启停边界，没有执行任何校准 apply/clear。
