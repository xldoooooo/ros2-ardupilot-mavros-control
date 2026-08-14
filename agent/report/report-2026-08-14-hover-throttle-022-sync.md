# 悬停油门回退值 0.22 同步简报

日期：2026-08-14

## 目标与范围

将用户已修改的 `src/onboard_control/config/control.yaml` 中 `hover_throttle` 从 0.39 同步为
0.22，推送远端 main，并同步、构建到真机源码与 install。未提交用户同时存在的 `TODO.md` 和
`agent/task/20-check.md` 改动。

## 完成结果

- 功能提交：`7e8e219 config: set hover throttle fallback to 0.22`。
- 远端仓库：main 已包含 0.22。
- 地面开发机：源码和 `install/onboard_control/share/onboard_control/config/control.yaml` 均为 0.22。
- 飞机：源码和 install 均为 0.22，`HEAD` 与 `origin/main` 一致。
- 没有执行解锁、起飞、飞控参数写入或主动服务重启。

## 验证

- 使用项目 `.venv` 解析 YAML：`hover_throttle=0.22`。
- 地面端 `build_onboard_control.sh`：两包构建成功。
- 地面端 ROS/C++：13 tests、0 errors、0 failures。
- 飞机 `build_onboard_control.sh`：两包构建成功，返回 0。
- 飞机构建前后服务保持 PID 2430、启动时间 `2026-08-14 11:18:16 CST`、`NRestarts=0`。
- 飞机只读状态：`armed=false`、`thrust_mode_verified=true`。

## 生效边界

本参数是启动回退值，不是飞行期间强制固定值：

- 当前运行节点在本次构建前已经启动，ROS 参数仍为 0.39；本次没有重启，因此不会热加载新 YAML。
- 权威状态中的控制器实际值为 `0.20000000298023224`，表明飞控 `MOT_THST_HOVER` 已覆盖回退值。
- 将来重启节点后会先读取 0.22，但 MAVROS 参数同步完成后仍会使用飞控当前标定值。
- 本次没有擅自修改飞控 `MOT_THST_HOVER`。

## 异常记录

首次飞机 `git pull` 时 SSH 断开，随后飞机一度不可达。网络恢复后确认该 pull 未落地，飞机系统/服务
已于 11:18:16 由外部重启；本次没有发出 reboot、shutdown、systemctl restart 或 stop。恢复后重新
执行 `pull --ff-only` 和构建成功。
