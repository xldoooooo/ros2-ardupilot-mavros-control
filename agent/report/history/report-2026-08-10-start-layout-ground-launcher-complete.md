# 启动脚本结构同步与地面站入口完成简报（2026-08-10）

## 完成情况

真机恢复连接后，已从
`/home/onboard/ros2-ardupilot-mavros-control` 原样拉取以下内容到本地仓库根目录：

```text
start_drone_all.sh
start_drone/
├── start_link.sh
├── start_mavros.sh
├── start_odin.sh
└── start_extnav.sh
```

本地与真机逐文件 SHA-256 完全一致：

- `start_drone_all.sh`：`4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`
- `start_extnav.sh`：`5fb5f9ebabb377cd417c8a7232572ce75c45a10ddfd5c69b6bf1fdb8b5ffb337`
- `start_link.sh`：`a4d435b042b18c7bf27fd01d1e9743179716f9e3ae4f55e8788a9f91459826c8`
- `start_mavros.sh`：`350a50257b03c3f1fc7a65113fdda00662bec694885372c27b48e86aaf5234be`
- `start_odin.sh`：`aae5fa7b3abc2c5db0b64f2e86ccea7add8afddf8ea7ad9b6f29090725a244ed`

真机分步脚本权限 `664`、集成脚本权限 `775` 也按原样保留。`check.sh` 不存在。

## 本地入口整理

新增只属于地面开发机的 `start_ground_all.sh`：

```bash
cd /home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware
bash start_ground_all.sh
```

脚本加载 ROS 2 Jazzy、当前工作区 overlay 和项目 Python，默认使用 domain 0 与子网发现，
然后以 `exec` 启动 `ground_station.py`。它不包含 MAVROS 串口、Odin、extnav 或任何飞行命令。
可在启动前给两端统一设置其他 `ROS_DOMAIN_ID`，也可透传
`--check-environment` 做不创建 GUI 的环境检查。

远端复核明确返回 `ground_launcher_absent`，没有把 `start_ground_all.sh` 上传到无人机。

确认本地旧 `start_all.sh` 与新 `start_drone_all.sh` 字节一致后，已将下列旧入口移入系统
回收站，而非不可恢复删除：

- `start_all.sh`；
- `start_drone.sh`；
- `start_ground.sh`。

## 检查结果

- 真机与本地五个机载启动文件的 SHA-256 一致。
- `bash -n`：两个一键脚本及四个分步脚本全部通过。
- ShellCheck：`start_drone_all.sh`、`start_ground_all.sh` 通过。
- 地面入口在 domain 231 / localhost 隔离环境执行
  `--check-environment`，确认 `guided_interfaces + rclpy available`。
- Python 全量回归：52 passed，14.82 秒。
- 修改范围 `git diff --check` 通过。

验证过程没有启动真机 MAVROS、Odin、extnav 或 onboard 节点，没有连接控制会话，也没有执行
模式、解锁、起飞或飞行指令。

## 分步脚本的既有注意事项

同步以“真机原样”为准，因此没有擅自修正分步文件。`start_extnav.sh` 在
`yaw_cam` 参数的反斜杠续行后插入了三行注释，再在注释后写 Odin XYZ 偏移。Shell 的续行与
注释组合可能导致 `odin_x/odin_y/odin_z` 没有作为预期参数传入。

`start_drone_all.sh` 不存在该写法，集成命令明确传入
`odin_x=0.06`、`odin_y=-0.03`、`odin_z=0.05`，此前真机运行日志也已确认这一组值生效。
如需继续保留分步启动作为等价回退，应另行同步修正真机和本地的 `start_extnav.sh`，不能只
修改本地副本。

## 两种启动方式的实际差异

一键脚本中真正启动飞行链的仍是与分步脚本对应的四条 ROS 命令。额外代码负责环境与路径
检查、统一 domain、重复实例保护、四个进程组与日志管理、只读就绪检查、异常退出检测和
`Ctrl+C` 整组清理；不包含解锁、起飞或控制动作。

分步方式更直接，允许逐个终端独立重启，但启动顺序、环境一致性、故障判断、日志与退出清理
都由操作者承担。一键方式没有改变核心数据链，主要是把这些重复且容易遗漏的运维步骤变成
可检查、可重复的监督流程。
