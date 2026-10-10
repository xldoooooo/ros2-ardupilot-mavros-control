# refresh 机载代码同步与 sparse checkout 收紧

本报告记录刷新机 Git 同步、机载目录精简、Shell 部署路径更新及原生验证结果。

## 实际基线与操作

- 目标：`drone-refresh`，Ubuntu 24.04/Jazzy，工作区 `/home/nvidia/ros2-ardupilot-mavros-control`。
- 操作前为 main/c9119e3，已启用 non-cone sparse checkout，并非完整检出；但旧规则额外保留 MEMORY、历史报告、测试和旧 Shell 入口。
- 已从 origin/main 快进同步，纳入 57cf960 的全部 Shell 迁移及之前的构建入口迁移；本报告同批提交维护 `.gitignore` 检出规则，并在目标机继续快进同步。
- 检出范围仅为 `.gitignore`、`src/guided_interfaces/`、`src/correction_interfaces/`、`src/onboard_control/`、`correction_service/`、`video_service/`、机载 Shell 与共享函数。当前132个受版本控制文件。
- Git sparse 移除不需要的受版本控制文件；未执行 git reset/clean。历史调试材料、缓存与现场标定文件先归档至仓库外，旧 agent/tests/image/start_drone 目录不再留在运行工作区。
- 保留目标机原生 `.venv/`、`build/`、`install/`、`log/`，未复制开发机安装产物，未删除Git历史。sparse checkout控制工作区检出范围，不声称缩减全部Git历史。
- 三个已安装systemd unit仅替换ExecStart的Shell路径，执行daemon-reload及systemd-analyze verify；未改现场环境或相机配置。

## 备份与验证

- 飞机备份目录：`/home/nvidia/ros2-ardupilot-maintenance/refresh-sparse-20260918-101142/`。
- 目录包含原HEAD/status/sparse规则、原三个unit、`/etc/ros2-ardupilot`配置归档及local-artifacts。原`start_drone/image/cam_in_ex.txt`保存于`local-artifacts/start-drone-image/`。
- 现场标定文件、当时根image副本及原Git版本SHA256相同：`131b0762eddcca02e55170eecb6fbcfacfaff8d0df06d8757a6242f953d86867`。
- 新构建入口`--verify`完成四包原生构建；24项包测试通过，0错误、0失败、0跳过；隔离smoke通过：interface=3.3、fcu_connected=false、armed=false、setpoint_messages=0。日志保存于备份目录`verify.log`。
- guided_interfaces/onboard_control源码与install均为3.3.0；correction_interfaces/correction_service均为2.0.0。
- 本地 sparse 规则维护回归17项通过。systemd验证返回成功，仅提示既有NVIDIA系统unit使用过时syslog输出类型，未修改无关系统unit。
- 三个项目unit操作前后均disabled/inactive。既有独立Odin进程PID9792、启动时间保持不变；未干扰其用户会话。

## 边界与待同步事项

- 未连接或同步new；其Shell与构建入口迁移仍待后续部署，已更新MEMORY。
- 未启动真实飞控/视频/修正服务，未发送飞行命令，未解锁或起飞；验证不能代替实飞验收。
- 用户开发机原有README、TODO与odom_pose_in_map.py改动不纳入本次提交。
