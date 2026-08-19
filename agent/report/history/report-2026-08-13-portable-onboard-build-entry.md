# 地面站/飞机通用机载构建入口简报

## 完成内容

- 日期：2026-08-13。
- 根目录新增可执行文件 `build_onboard_control`，地面开发机和飞机使用同一入口。
- 默认命令：

```bash
./build_onboard_control
```

  自动检测当前主机 ROS 发行版，以 Release 模式构建 `guided_interfaces` 及其上游目标
  `onboard_control`。
- 完整验证命令：

```bash
./build_onboard_control --verify
```

  在默认构建外执行依赖检查、ROS/C++ 单元测试与隔离 smoke。
- 支持 `-h`/`--help`；未知参数以退出码 2 拒绝，不会静默传递任意命令。

## 实现边界

- 快捷入口只解析自身所在仓库根目录，并复用
  `src/onboard_control/deploy/onboard_workspace.sh`，没有复制第二套构建逻辑。
- ROS 发行版沿用可移植发现：Ubuntu 24.04 地面机选择 Jazzy，Ubuntu 22.04 飞机选择 Humble，也
  可继续使用既有 `ONBOARD_ROS_DISTRO` 显式覆盖。
- 不调用 systemd，不启动、停止或重启机载服务，不连接真实 MAVROS，不申请租约，不发送解锁、
  起飞或任何飞行命令。
- 如果构建时服务正在运行，Linux 当前进程继续使用已加载的旧映像；新产物要生效仍需由操作者
  选择安全窗口另行重启服务，脚本不会替操作者决定。

## Sparse checkout 与文档

- 机载 sparse 路径新增 `/build_onboard_control`；`onboard_workspace.sh update` 会与两个 ROS 包、
  `start_drone/` 和 `start_drone_all.sh` 一起维护该入口。
- 初次 sparse clone 清单、最小检出树、更新说明、部署指南和根目录 README 均已增加新命令。
- `validate_workspace_layout()` 要求该入口存在且可执行，避免 sparse 配置遗漏后构建流程表面成功。

## 验证结果

- `bash -n build_onboard_control src/onboard_control/deploy/onboard_workspace.sh`：通过。
- 部署/启动回归：19 passed；新增测试覆盖可执行位、帮助文本、默认 build/verify 路由、禁止
  systemctl/解锁/起飞，以及 sparse 清单。
- `./build_onboard_control` 在本地 Jazzy 实际完成两包 Release 构建。
- `./build_onboard_control --verify` 在本地 Jazzy 完成 16 项 ROS 依赖检查、两包构建、13 tests
  零失败，并通过接口 3.0、FCU 未连接、未武装、零姿态 setpoint 的隔离 smoke。
- 正式 Python 测试目录：106 passed。
- `git diff --check`：通过。

## 飞机同步与验证

- 飞机网络短暂不可达，恢复后确认目标仓库位于
  `/home/onboard/ros2-ardupilot-mavros-control`，同步前为 `f403f92`，仅有既存未跟踪目录
  `.deployment-backups/`。
- 通过离线 Git bundle 将飞机 `main` 快进至 `0d5fb42`，没有覆盖或删除飞机上的既有文件；随后将
  `/build_onboard_control` 加入 non-cone sparse checkout，入口存在且保持可执行。
- `show-config` 在飞机确认 `ros_distro=humble`、`architecture=aarch64`、`revision=0d5fb42`。
- 飞机根目录实际执行 `./build_onboard_control`：`guided_interfaces` 1.36 秒、
  `onboard_control` 0.36 秒，总计 2.15 秒，两包成功。
- 构建前后 `ros2-ardupilot-onboard.service` 均保持 active；主 PID 2455、服务启动时间和运行中
  `onboard_control_node` PID 2932 均未变化，证明脚本没有重启运行栈。临时 bundle 已从飞机 `/tmp`
  删除。
- 只读 ROS 话题查询在超时内没有收到状态样本，因此没有把“已确认未武装”记作验证结论；本任务
  没有调用任何控制接口，没有解锁或起飞，也没有启动、停止或重启真机服务。
