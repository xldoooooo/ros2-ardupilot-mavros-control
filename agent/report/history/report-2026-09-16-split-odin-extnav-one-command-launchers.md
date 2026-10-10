# Odin 与 extnav 独立一键启动修复简报

## 目标与结果

修复现有的两个分立入口，使新飞机在不启动 MAVROS、`onboard_control`、
`correction_service` 或视频服务的情况下，能从普通 SSH 终端分别直接启动：

```bash
./start_drone/start_odin.sh
./start_drone/start_extnav.sh
```

两个入口彼此独立，没有新增“Odin + extnav 合并启动”脚本。全程未解锁、
未起飞，也未发送模式或轨迹命令。

## 原因与修复

原脚本可以在已手动 source 现场 overlay 的终端中运行，但普通 SSH 环境中没有
`ODIN_OVERLAY_SETUP` 和 `EXTNAV_OVERLAY_SETUP`，自动搜索也找不到飞机当前的深层工作区，
因此会在启动硬件前报 `odin_ros_driver is not visible`。另外，飞机上的旧
`start_extnav.sh` 没有 source 项目 overlay，存在无法发现 `correction_interfaces` 的风险。

本次修复：

- 在共享运行辅助脚本中增加可选环境文件读取，并将其变量导出给 ROS 子进程。
- Odin 和 extnav 脚本默认自动读取 `/etc/ros2-ardupilot/onboard.env`，仍允许用
  `ONBOARD_ENV_FILE` 显式覆盖。
- Jazzy 默认设置 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`，Humble 继续使用
  `ROS_LOCALHOST_ONLY=0`，避免从不同 shell 继承不相容的 ROS 发现配置。
- extnav 在独立 overlay 前先加载项目 `install/setup.bash`，保证修正接口可见。
- 部署文档和静态回归测试明确固化“两个独立入口”与不带起其他服务的边界。

## 本地验证

- `bash -n` 通过两个启动脚本与共享运行辅助脚本。
- `ruff` 通过部署测试文件。
- 定向部署测试：`19 passed in 0.50s`。
- 完整测试：`250 passed in 91.98s`。

## 新飞机同步与实测

目标为 `drone-new`，工作区为
`/home/nvidia/ros2-ardupilot-mavros-control`。远程保留历史选择性部署工作树，
本次没有执行 `git pull`、`reset` 或 `clean`，仅定点备份并同步相关文件。

覆盖前备份：

```text
/home/nvidia/backups/split-odin-extnav-launchers-20260916-R9Cvpl/files-before.tar.gz
SHA-256: 5c1350cdc88cf6d08ad9985e5eb76132e61d1f2ad480f6bad67eadde2acf4e87
```

同步后的 5 个文件均与本地 SHA-256 一致。在两个全新 SSH 终端中，没有手动
source 任何环境文件，分别直接执行两个脚本：

- Odin 成功连接硬件并发布 `/odin1/odometry_highfreq`，观测平均约
  `398.949 Hz`。
- extnav 成功订阅 Odin，稳定 session 为 `odin-7712475688718-e04dc6c2`，
  `/extnav/pose_fcu` 约 `99.9–100.0 Hz`。
- 修正状态显示 Odin 可用、最终样本可用，但未应用修正：
  `correction_valid=false`、`revision=0`。
- 未启动 MAVROS、onboard_control 或视频节点；飞控总服务保持 failed/inactive，
  视频服务保持 inactive，correction_service 保持 active/idle。
- Odin 通过 Ctrl+C 正常停止；extnav 也已停止且无残留进程。

## 已知边界

- Odin 厂商 launch 会无条件启动 RViz。纯 SSH 无 `DISPLAY` 时 RViz 会报错退出，
  但 Odin 主进程与数据发布继续正常；本次未修改外部厂商 launch。
- extnav 外部 Python 节点在 Ctrl+C 时会重复调用 `rcl_shutdown`，因此打印 traceback
  并让 `ros2 run` 返回 1；进程已停止且无残留。这是外部 extnav 节点的既有停止
  行为，不影响本任务的独立一键启动结论，但后续若要求干净零码退出需另行修复。
- 本次只验证未解锁的地面数据链，不代表已完成校准精度或实飞验收。

## 终态

- 两个独立启动进程均已停止，无 Odin、extnav、RViz、MAVROS 或 onboard 残留。
- 飞控总服务仍为 failed/inactive，视频服务仍为 inactive。
- `odin-correction.service` 仍为 active/idle；本任务未停止或重启它。
