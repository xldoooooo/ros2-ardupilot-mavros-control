# Ubuntu 22.04 / Humble 免硬编码部署适配简报

日期：2026-08-11

## 任务结论

已把项目部署入口从开发机/无人机专用绝对路径改为基于当前检出和系统信息的自动发现。
师兄在 Ubuntu 22.04 上正常只需克隆仓库后执行：

```bash
bash setup_project.sh
bash start_ground_all.sh --check-environment
```

Ubuntu 22.04 会优先选择 ROS 2 Humble，即使环境中残留 `ROS_DISTRO=jazzy` 且两个发行版
同时存在。机载四组件可先用 `bash start_drone_all.sh --check` 只做发现检查，不启动进程。

本任务没有连接真实飞控，没有启动 MAVROS/Odin/extnav，没有申请租约、写入原点、切换模式、
解锁或起飞实机。

## 主要改动

### 自动部署

新增 `setup_project.sh`：

- 从脚本自身位置解析仓库根目录；
- Ubuntu 22.04 自动选择 Humble，24.04 保留 Jazzy 开发兼容；
- 创建仓库内 `.venv`，使用 `--system-site-packages` 继承 ROS Python；
- 安装 PySide6、构建三个 ROS 包、执行 ROS/C++ 测试和环境检查；
- 不调用 `apt`、`sudo`、MAVROS、串口或飞行命令。

新增 `DEPLOY_UBUNTU_2204.md`，正常流程中的命令不要求填写工作区、ROS、Python 或飞控
源码绝对路径。

### 自动运行时发现

新增 `start_drone/runtime_common.bash`，供地面与机载入口共同使用：

- ROS：显式覆盖优先，其次按 Ubuntu 版本选择 Humble/Jazzy；
- Python：优先仓库 `.venv`，其次仓库 `venv` 和系统 `python3`；
- Odin/extnav：根据 ament package index 反查实际 install prefix，不依赖用户名或工作区名；
- 串口：优先唯一 `/dev/serial/by-id`，再检查 `ttyTHS/ttyACM/ttyUSB`；多个候选时拒绝猜测；
- ArduPilot：依次检查环境、PATH、仓库同级、当前用户、`/opt` 和 `/usr/local/src` 常见布局。

`start_drone_all.sh` 新增 `--check`，输出自动解析的 ROS、工作区、FCU、Odin 和 extnav，
随后退出且不启动任何组件。四个 `start_drone/*.sh` 同步移除固定 `/home/xld`、
`/home/onboard`、`/opt/ros/humble` 和 `/dev/ttyTHS1`。

### Humble DDS 适配

地面站 Python 入口和 ROS context 切换不再默认写死 Jazzy：

- Humble 本地仿真使用 `ROS_LOCALHOST_ONLY=1`；
- Humble 实机使用 `ROS_LOCALHOST_ONLY=0`；
- Jazzy 继续使用 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST/SUBNET`；
- 仿真 domain 231、实机 domain 0 和显式 peer 清理边界保持不变。

## 验证结果

- 新建仓库 `.venv` 后完整执行 `bash setup_project.sh`：成功；
- Python/Qt 全量：73 passed；
- 自动发现专项：26 passed；
- 三包 Release 构建：成功；
- ROS/C++：5 tests，0 errors，0 failures，0 skipped；
- 干净环境执行 `bash start_ground_all.sh --check-environment`：成功，自动选择项目 `.venv`；
- 所有新增/修改 shell 脚本 `bash -n`：通过；
- Python `compileall`、flake8 F/E9/E501、`git diff --check`：通过。

构建验证首轮曾用 `--symlink-install` 叠加到既有普通 install 构建目录，生成接口目录形态冲突，
`guided_interfaces` 构建失败。已把部署脚本固定为普通 Release install，并用 CMake clean target
与 `--cmake-clean-cache` 修复当前生成目录；之后三包构建和测试全部通过。没有删除源码或用户文件。

## 保留的安全例外

程序不会在多个串口或多个同名 ROS overlay 中自动猜测。只有真实存在歧义时，操作者才需先
确认硬件身份，再通过一次性环境变量选择；这不是路径硬编码，而是防止连接错误飞控的必要
安全边界。默认波特率仍为当前实机已验证的 460800，若硬件配置不同也必须由操作者确认。

当前开发机是 Ubuntu 24.04/Jazzy，因此本轮没有在师兄的 22.04 目标机实际安装。22.04/Humble
选择逻辑已由双发行版夹具回归覆盖，而机载 C++ 核心此前已在真实 22.04/Humble/aarch64 上完成
构建、5 项测试和安全烟雾验证。部署到师兄电脑后仍应先执行自动部署和无桨检查，不得直接实飞。
