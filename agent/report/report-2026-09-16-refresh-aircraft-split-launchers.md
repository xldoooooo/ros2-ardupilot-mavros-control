# refresh 飞机 Odin/extnav 独立启动入口同步简报

## 目标与结果

将已在新飞机验证的 Odin 和 extnav 独立一键启动入口同步到 refresh 飞机
`drone-refresh (192.168.112.186)`，解决用户直接执行 `./start_extnav.sh` 时的：

```text
[runtime-discovery] ERROR: ROS package extnav_bridge is not visible and no owning overlay was found
```

修复后可在 refresh 飞机的 `start_drone/` 目录中分别直接执行：

```bash
./start_odin.sh
./start_extnav.sh
```

两个入口仍彼此独立，没有新增合并启动脚本，也不会带起 MAVROS、
onboard_control、correction_service 或视频服务。

## 现场原因

refresh 飞机已有正确的现场配置：

- `ODIN_OVERLAY_SETUP=/home/nvidia/catkin_ws/install/setup.bash`
- `EXTNAV_OVERLAY_SETUP=/home/nvidia/vrpn_mavros/install/setup.bash`
- `ROS_DOMAIN_ID=0`
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`

`/home/nvidia/vrpn_mavros/install/setup.bash` 也能正常使
`ros2 pkg prefix extnav_bridge` 解析到
`/home/nvidia/vrpn_mavros/install/extnav_bridge`。报错的直接原因是 refresh 飞机仍使用旧启动脚本，
没有自动读取 `/etc/ros2-ardupilot/onboard.env`。

## 备份与同步

覆盖前备份：

```text
/home/nvidia/backups/split-odin-extnav-refresh-20260916-4jO7Q9/files-before.tar.gz
SHA-256: b770c7f6415c1f8bb0d5f2bb078784eebb68702da9e8c0a514ec938307100e5c
```

定点同步了：

- `start_drone/runtime_common.bash`
- `start_drone/start_odin.sh`
- `start_drone/start_extnav.sh`

远程 SHA-256 与本地/main 完全一致：

```text
f6ca2c87d2bebe07f26828e86cf9db322cba0e801a6b5b31bd094d80917e262d  runtime_common.bash
37b1e7f7dd5fa867181383da4a150884fb16455dcba445b1d3851ee469641907  start_odin.sh
2c2bc6b02f265ec76987ab3e237111dec32fdad7821e991949672540f3e46fdd  start_extnav.sh
```

refresh 飞机保留自身现场工作树，本次未执行 `git pull`、`reset` 或 `clean`。

## 未解锁实测

- 从用户报错时相同的 `start_drone/` 目录直接执行 `./start_extnav.sh`，成功进入
  `Bridge started`，配置为 vision 40 Hz、control 100 Hz。
- 直接执行 `./start_odin.sh`，成功识别 USB 3.2 Odin，完成软件连接并进入
  `Device ready and streams activated`。
- Odin 主进程在 SIGINT 后完成 stream stop 和 SDK deinit；extnav 也已停止。
- 最终无 Odin、extnav、RViz、MAVROS 或 onboard_control 残留进程。
- `ros2-ardupilot-onboard.service`、`video-service.service` 和
  `odin-correction.service` 验证前后均为 inactive。

## 已知边界与安全边界

- 纯 SSH 没有 `DISPLAY`，Odin 厂商 launch 带起的 RViz 会报 xcb 错误；Odin 硬件连接和
  主数据进程不受影响。
- extnav 外部 Python 节点在 SIGINT 后仍会因重复 `rcl_shutdown` 打印 traceback，
  但进程能停止且无残留。
- 全程未解锁、未起飞，未发送飞行、模式或轨迹命令，未修改飞控参数。
