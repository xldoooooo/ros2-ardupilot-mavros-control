# new 飞机重启客户端迁移同步简报

本报告记录客户端路径迁移和部署脚本修正的同步范围、验证结果及现场限制。

- 日期：2026-09-17；目标：`drone-new`（192.168.112.169），工作区 `/home/nvidia/ros2-ardupilot-mavros-control`。
- 同步来源：开发机 main `c9119e3`；refresh 已下线，本次未连接。
- new 仍为旧 Git HEAD `6a40713` 加现场部署改动；采用备份后定点同步，未执行 pull/reset，未覆盖其他现场改动。

## 同步内容

- `tools/reboot_fcu.py` 原样迁至 `src/onboard_control/scripts/reboot_fcu_client.py`，内容逐字节一致。
- 更新根入口 `reboot_fcu.sh`，先部署客户端，再原子替换入口。
- 更新 `src/onboard_control/deploy/onboard_workspace.sh`：去掉旧客户端 sparse 条目，补齐接口3.3自检修正。
- 更新 `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`，同时更新以上两个部署文件在 `install/onboard_control/share/onboard_control/deploy/` 下的副本。
- 删除旧客户端后移除空 tools 目录；new 原 tools 内只有该文件，没有项目相机标定目录。未改动项目外标定程序和标定结果。
- 本次没有新增运行逻辑或修改 C++/接口，无需重建。四个同步源文件 SHA-256 均与开发机一致。

## 验证与安全

- 同步前后各3秒只读观测：接口3.3、FCU已连接、armed=false、on_ground=true、控制器未激活、无租约；观测窗口没有姿态推力输出。
- Bash语法、项目 `.venv` Python AST检查通过；安装副本与源码一致；旧 tools 路径不存在。
- new 上 localhost/domain231 隔离smoke通过：接口3.3、FCU未连接、未解锁、setpoint_messages=0。
- 三项生产服务的 MainPID 和启动时间前后一致：onboard=2498、correction=2496、video=2503。未重启飞控或生产服务，未执行重启客户端、解锁或起飞命令。
- 本次仅验证迁移与同步，不重复真实FCU重启验收；此前实机验收见 Task33 报告。

## 留档

- 飞机备份：`agent/codex/new-client-sync/before.tgz`，包括旧入口、tools、部署源码与安装部署目录。
- 飞机服务快照及同步包：`agent/codex/new-client-sync/`。
- 开发机同名过程目录保存同步日志、同步前后只读遥测和同步包。
- MEMORY.md 已清除 new 待同步标记；new 旧HEAD加现场改动的维护限制仍保留。
