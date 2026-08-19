# 当前改动发布与机载启动文件更新范围简报

日期：2026-08-11

## 目标

- 将当前工作树中的全部已有改动统一提交并推送到 `origin/main`。
- 把无人机新增的 `start_drone/` 四个分步脚本和根目录
  `start_drone_all.sh` 纳入版本库。
- 确保后续机载 sparse checkout 更新会与两个机载 ROS 包一起更新上述启动文件，且不会
  把地面端专用的 `start_ground_all.sh` 同步到无人机。

## 本次补充

- `onboard_workspace.sh update` 的 sparse checkout 范围新增
  `/start_drone/` 与 `/start_drone_all.sh`。
- 工作区布局校验新增机载启动目录和一键脚本检查，避免更新完成后静默缺文件。
- 首次部署说明、后续更新说明和自动化测试同步维护这四类机载路径。
- `start_drone/` 下四个 shell 脚本设置为可执行文件。
- `MEMORY.md` 记录长期更新边界：机载更新包含两个 ROS 包和两类机载启动入口，排除
  `start_ground_all.sh`。

## 验证结果

- `bash -n`：机载工作区助手、机载一键脚本、四个分步脚本和地面一键脚本均通过。
- `tests/test_onboard_deploy.py`：5 passed。
- 加载 `/opt/ros/jazzy/setup.bash` 与本地 `install/setup.bash` 后运行 Python 全量回归：
  62 passed。
- `git diff --check`：通过。
- 提交前 `git fetch origin main` 显示本地与远端基线均无领先或落后。

第一次在未加载本地 ROS overlay 的普通 shell 中运行全量测试时，有 2 项因找不到
`libguided_interfaces__rosidl_generator_py.so` 失败；加载项目规定的 Jazzy underlay 与本地
overlay 后复测全部通过，确认是测试终端环境问题而非代码回归。

## 安全说明

- 本次发布阶段没有连接、启动、解锁或起飞真实无人机，也没有改动无人机当前运行状态。
- 机载 smoke 流程仍固定为非零 domain 231 和 localhost-only，不会发现真实 MAVROS，也不
  发送飞行指令。
