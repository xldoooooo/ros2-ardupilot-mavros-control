<!-- 笔记本主工程与独立避障库同步、现场保留和部署验证结果。 -->
# 笔记本代码同步简报

日期：2026-10-13。目标：`ssh hp-desktop`，`scq@192.168.112.101`，Ubuntu 24.04 / ROS 2 Jazzy / x86_64。

## 同步结果

| 仓库 | 笔记本路径 | 同步前 | 功能基线 |
| --- | --- | --- | --- |
| 主工程 | `/home/scq/ros2-ardupilot-mavros-control` | `591f071` | `62ce78e` |
| 独立避障 | `/home/scq/dyn_small_obs_avoidance-ros2` | `0d86e7e` | `b527fbc` |

两个main均通过开发机Git bundle快进同步；主工程README和独立库.gitignore补齐开发机当前工作区版本。
在笔记本干净重建独立库、guided_interfaces、correction_interfaces、avoidance_bridge、onboard_control、guided_sim，另重建correction_service。
未复制开发机build/install或Python环境；笔记本使用其原有主工程`.venv`。

## 现场保留

- 完整旧Git历史、跟踪源码、现场diff及两个仓库原有build/install/log归档在`/home/scq/backups/code-sync-20261013/`；主工程现场改动另保存在Git stash `laptop-pre-sync-20261013`。
- 保留笔记本修正服务测试中取消固定Tag尺寸断言的修改；没有覆盖相机标定、运行环境、导出文件或实验日志。
- 旧避障launch曾将safe_distance从0.45改为0.35；该文件已归档，新入口使用公共planner.yaml，当前默认safe_distance=0.25。没有将旧桥profile重新带入新版规划器。
- 未修改或实现TODO条目，未提交的任务目录整理不纳入此次代码同步；main历史中的既有文档提交随仓库同步。

## 验证

- 主工程运行源码与独立库path_searching/path_planning源码SHA256对照一致；唯一排除项是明确保留的修正测试改动。
- 安装后的PlanResult、SetGoal、AvoidanceResult、公共planner.yaml、control.yaml及桥launch逐文件与源码一致。
- 原生C++核心22项、输出合同6项、动力学适配4项通过；控制端C++24项通过。
- 原生重规划DDS测试9项通过；桥公共参数、identity坐标流与修正服务测试合计7项通过；extnav坐标流另1项通过。
- 地面站`--check-environment`通过，避障launch参数解析通过，项目Python依赖`pip check`通过。
- DDS验证使用LOCALHOST及非零域，只有本机合成输入；收尾测试节点退出。未连接飞机或执行实机解锁、起飞。

所有同步与构建检查完成；本次没有新增功能，也未进行SITL飞行或真实硬件闭环验收。既有净空、安装角和实飞限制保持。
验证日志位于笔记本备份目录；两个仓库MEMORY已更新笔记本部署基线。
