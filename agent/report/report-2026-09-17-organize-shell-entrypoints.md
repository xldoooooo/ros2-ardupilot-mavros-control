# Shell 操作入口目录整理

本报告记录 Shell 目录迁移、调用关系适配与验证结果。

## 完成内容

- 按既定方案迁移 14 个文件：地面安装/启动归入 `scripts/ground/`；机载启停与飞控重启归入 `scripts/onboard/`；四个分组件启动归入 `scripts/onboard/components/`；共享函数归入 `scripts/lib/`。
- 保留 13 个操作脚本的可执行权限及共享函数库的原有 0644 权限，修正相对工作区根目录与共享函数路径；移除已空的本地 `start_drone/`，不保留旧根入口。
- 更新安装器、三个 systemd 模板、机载 sparse checkout 清单与布局检查、帮助提示、文档、测试及增量测试选择规则。
- 构建/部署脚本继续保留在组件 `deploy/`，requirements、主体代码目录、示例和资料布局不变。Python 文件仅更新相关入口路径引用或提示。
- MEMORY.md 更新当前入口与两台飞机待同步事项，保留旧飞机路径的历史事实；用户原有 README、TODO 与 odom_pose_in_map.py 迁移未纳入本次提交。

## 验证

- 首轮测试：62 项通过、2 项因未加载本地 ROS overlay 而缺少 guided_interfaces 失败；加载 `/opt/ros/jazzy/setup.bash` 与项目 `install/setup.bash` 后复测并扩展相关范围，102 项全部通过。
- 测试范围包含 runtime、机载启动参数/就绪检查、启动取消后进程清理、视频与修正部署、UVC 配置引用和增量选择；ROS 测试使用 localhost 隔离环境。
- 14 个迁移文件的权限与 Shell 语法检查通过，相关部署脚本语法及 `git diff --check` 通过。
- 临时带空格工作区内，从 `/tmp` 调用四个分组件入口及地面启动入口成功；ROS/Python 命令由替身截获，核对地面清理调用，未启动真实服务。
- 使用暂存树和真实 Git sparse checkout 验证全部机载脚本与共享函数被检出，地面脚本未检出，旧根入口不存在；从 `/tmp` 执行检出副本 `onboard_workspace.sh show-config` 正确识别工作区。

## 未执行与后续同步

- 未执行完整 ROS 重建；本次未改动控制算法和 ROS 包实现。
- 未连接两台飞机，未更新已安装 unit，未停止/重启真实服务，未解锁或起飞。
- 两台飞机下次同步须同时纳入本次 Shell 迁移和此前构建入口迁移。旧 sparse checkout 应先加入新路径，再拉取；在维护窗口更新三个 systemd unit 的 ExecStart 并 daemon-reload，详细步骤已写入 ONBOARD_DEPLOYMENT.md。
- 飞机旧 `start_drone/image/` 等未跟踪现场数据不可随目录迁移删除。
