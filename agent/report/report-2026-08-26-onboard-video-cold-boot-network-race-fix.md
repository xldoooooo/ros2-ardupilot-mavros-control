# 机载视频冷启动网络竞态修复简报

## 任务结论

经用户明确授权对当前 Jetson 执行两次整机 `sudo reboot`。第一次冷启动准确复现“飞机本机视频
服务正常、同网段 scq 无法发现”的问题；加入最小局域网路由就绪门后，第二次冷启动跨机发现、状态
地址和飞控未武装状态均通过验证。

本次没有恢复外网校时依赖，也没有使用固定开机延迟。启动脚本只等待本机出现非 `linkdown`、带
真实源 IPv4 的默认路由；该检查不发网络包、不访问互联网。

## 复现证据

第一次冷启动 boot ID：`d4449e30-b933-4a97-aa03-de3d14c717d1`。

- `network-online.target`：约 6.57 秒；
- `video-service.service` 启动：约 6.64 秒；
- 视频 ROS 节点创建：约 9.02 秒；
- Wi-Fi DHCP 获得 `192.168.112.169`：约 10.38 秒；
- 视频进程没有绑定 `192.168.112.169` DDS 单播套接字；
- 飞机本机可匹配服务并收到状态，但 RTSP 地址错误成为
  `rtsp://192.168.55.1:8554/camera`；
- scq 跨重启观测器在 130 秒窗口内始终没有重新发现服务。

上述事实证明：`network-online.target` 在本机不能代表生产 Wi-Fi IPv4 已经就绪，Fast DDS
participant 在 Wi-Fi 地址出现前创建后没有可靠刷新跨机网络定位信息。手动重启服务之所以恢复，
是因为重启时 Wi-Fi 已经拥有生产地址。

## 最小修复

`start_onboard_video.sh` 在创建 ROS participant 前：

1. 每 0.25 秒读取一次 `ip -4 route show default`；
2. 忽略带 `linkdown` 标记的默认路由；
3. 只接受带非回环 `src` IPv4 的主路由；
4. 最多等待 30 秒；
5. 超时明确失败，由既有 `Restart=on-failure`/`RestartSec=2` 重新尝试。

因此 systemd 与手工运行脚本继续使用同一行为；局域网就绪即可启动，无需外网时间或互联网探测。
`tests/test_onboard_deploy.py` 同步锁定上述最小门控存在。

## 部署过程与验证

部署前再次通过真实 `ControlStatus` 确认 `armed=false`、FCU connected、STABILIZE。旧启动脚本备份
为：

`/home/nvidia/backups/start_onboard_video-pre-lan-wait-fix-20260826-2132.sh`

首次部署时，Jetson 的 awk 实现把循环变量 `index` 与内建函数冲突处理，视频 unit 因语法错误发生
9 次自动重试。问题被如实保留在 journal；随后把变量改为 `field`，在开发机实际执行同一解析表达式
后重新部署。修正后视频 unit 手工重启，当前重启计数归零，飞控 PID 全程未变化。

第二次冷启动 boot ID：`39da3869-21d8-43a3-9674-aac481801a17`。

- 视频 unit 进入启动：约 7.39 秒；
- Wi-Fi DHCP 获得 `192.168.112.169`：约 10.41 秒；
- 启动脚本确认 LAN 源地址：约 10.44 秒；
- 视频 ROS 节点创建：约 11.23 秒；
- 视频进程实际绑定 `192.168.112.169` DDS 单播套接字；
- scq 观测器自动恢复为服务 ready、状态 fresh，RTSP 地址为
  `rtsp://192.168.112.169:8554/camera`；
- 最终视频与飞控 unit 均 active，`NRestarts=0`、`Result=success`；
- 最终真实 `ControlStatus` 为 `armed=false`。

验证结果：

- `bash -n start_onboard_video.sh`：通过；
- 路由解析表达式在开发机实际执行：得到当前主路由源 IPv4；
- 视频部署专项：21 passed；
- 完整 `tests/`：182 passed，耗时 42.75 秒；
- 第二次真实冷启动与 scq 跨机持续观测：通过。

全过程没有调用模式切换、解锁、起飞、降落、运动、航点、姿态、推力或飞控参数接口。
