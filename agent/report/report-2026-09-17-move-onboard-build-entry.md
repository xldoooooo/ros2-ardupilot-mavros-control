# 机载构建入口目录调整

本报告记录构建脚本迁移、引用更新及验证范围。

- 将根目录 `build_onboard_control.sh` 移至 `src/onboard_control/deploy/`，保留可执行权限，修正相对工作区根目录解析和帮助命令。
- 更新飞控/视频安装器、工作区布局检查、README、部署文档和现有回归测试；构建脚本由已有 `/src/onboard_control/` sparse 规则覆盖，移除多余的根文件规则。
- MEMORY.md 已更新当前入口与两台飞机待同步说明；既有历史报告保持原样。
- 验证：部署回归测试 17 项全部通过；四个相关 Shell 脚本语法检查通过；工作区 show-config 正确识别当前仓库/Jazzy；隔离替身验证从其他目录调用、路径带空格及 build/verify 参数转发全部通过；git diff --check 通过。
- 未执行完整 ROS 重建或实机测试；未连接飞机、未重启服务，未解锁或起飞。两台飞机下次同步时需一并更新脚本与调用方。
- 提交仅包含本任务变更，保留用户此前 README、TODO 及 odom_pose_in_map.py 迁移的未提交修改。
