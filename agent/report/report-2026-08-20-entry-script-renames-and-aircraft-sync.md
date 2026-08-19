# 入口脚本重命名与真机同步简报（2026-08-20）

## 仓库修改

- `start_drone_all.sh` 重命名为根目录 `start_onboard_control.sh`。
- `stop_onboard_service.sh` 重命名为根目录 `stop_onboard_control.sh`，残留进程识别规则同步新名称。
- `setup_project.sh` 重命名为根目录 `setup_ground_station.sh`，运行日志前缀改为 `ground-setup`。
- systemd 模板、飞控/视频安装器、机载 sparse checkout、环境诊断提示、Ubuntu 部署文档、根
  README、视频 README、测试和 MEMORY 全部适配新名称；历史任务报告保留原名作为当时事实。
- README 明确：`setup_ground_station.sh` 是完整地面站唯一项目安装入口；飞机专用的
  `src/onboard_control/deploy/install_onboard_service.sh` 与
  `video_service/deploy/install_onboard_video_service.sh` 不在地面站执行。

## 真机修改

- 修改前只读确认 `ros2-ardupilot-onboard.service`、`video-service.service` 均 active，MAVROS
  `connected=true`、`armed=false`、`mode=STABILIZE`。
- 备份旧脚本、飞控 unit 和 sparse 配置到
  `/home/nvidia/backups/onboard-entry-rename-20260820/`。
- 安装 `start_onboard_control.sh` 与 `stop_onboard_control.sh`，删除工作区旧名称；同步机载 deploy
  源码及 install/share 副本，更新 sparse checkout 条目。
- 将 `/etc/systemd/system/ros2-ardupilot-onboard.service` 的 ExecStart 修改为
  `/home/nvidia/ros2-ardupilot-mavros-control/start_onboard_control.sh`，通过
  `systemd-analyze verify` 后执行 `daemon-reload`。
- 未停止或重启飞控服务：MainPID 保持 23582，启动时间仍为 2026-08-19 23:34:33；修改后再次确认
  飞控 unit enabled+active、视频 unit active、MAVROS `armed=false`。
- 扫描真机工作区（排除历史报告）、install/share 和 `/etc/systemd/system`，旧三个入口名称零残留。

## 验证

- 新旧入口存在性、shell 语法、ShellCheck、sparse 路径、systemd 模板与生命周期边界由自动化测试
  覆盖。
- 项目正式 Python 全量 `tests/`：164 passed；整个过程未发送模式、解锁、起飞或控制命令。
