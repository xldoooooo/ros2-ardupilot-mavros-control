# 机载启动校时缺陷与等待时间修复

本报告记录保持离线启动能力的最小改动、本地验证以及 drone-new 未武装实测。

## 结果

new 实机从 systemd 开始执行启动器到 READY，由基线 **54.143 秒**缩短至 **39.368 / 39.607 秒**，
两轮分别减少约 27.3% / 26.8%。不包含 restart 命令停止旧服务约 5 秒的时间。
未将进程启动当成就绪，仍要求同一条新状态中飞控连接、未武装、消息频率配置、必要推力参数值
验证和本地位姿全部通过。

离线能力保持：unit 仍只等 network-online.target，没有加入 NTP/time-sync 依赖或固定开机延迟。

## 修改

- `start_onboard_control.sh`：删除四组件之间 3 次 sleep 1；四组件并行启动、继续共同监督。
  单个持续 ROS CLI 订阅替换重复创建的 topic echo；GNU timeout 用 120 秒相对定时限时，
  避免 Bash SECONDS 因墙钟前跳提前到期。匹配状态/错误写入 readiness.log；取消时清理探针进程组。
- `onboard_control_node.cpp/.hpp`：飞控连接后服务可用即异步发起一次 ParamPull(force_pull=false)，
  不阻塞控制线程，不强制清空 MAVROS 缓存；断线后允许下一连接重新请求。
  拉表 ACK 不代替 GUID_OPTIONS 与 MOT_THST_HOVER 的值校验。
- `control.yaml` 与 C++ 默认：首次参数校验由 40 秒改成 2 秒；未通过时每秒重试，通过后仍每 5 秒复核。
- 部署文档和 MEMORY 同步维护。未修改飞控参数、MAVROS 软件或协议，不增加依赖。

## 校时问题与验证边界

8 月 11 日曾添加等待首次校时的依赖；8 月 25 日为支持无外网启动取消该依赖，但启动器仍使用
Bash SECONDS。此次 new 自然开机在单调时间 43.99 秒校时，45.87 秒就报告 120 秒等待超时，实际
只等约 27 秒。细节见同日 new 启动实测报告。

新增测试通过 LD_PRELOAD **仅在测试子进程**内注入两天墙钟前跳：旧 Bash 循环提前结束，新相对
计时器按期返回 124。未设置开发机或飞机系统时钟。本次没有重启 Linux、断开 WAN 或主动制造真实
NTP 校时；下一次现场冷启动仍须保留 journal 核对，不将子进程测试冒充完整实机冷启动验证。

## 验证

- 本地相关 ROS/部署/环境/状态测试 36 项通过；另新增校时回归与取消清理 2 项通过，共 38 项。
  最终 startup 专项 3 项一起执行全部通过，包括真实隔离 ROS 节点的早拉参数、错误/缺失参数不就绪、
  重连重新请求、armed=true 不就绪，以及取消启动时四组件与持续订阅均退出。
- 初版测试未给自定义 ROS Context 指定 executor，测试自身失败；修正测试 executor 后通过，未隐去失败。
- 本地 C++ 构建通过、19 tests/0 failures；new ARM64 原生构建通过、19 tests/0 failures。
- 脚本 bash -n 和本次改动范围 git diff --check 通过。用户原有 README 空白错误未改动。
- SITL 完整生产初始化成功、未解锁/未起飞：参数校验约 9.84 秒（原 17.64～20.33 秒），
  完整就绪 44.24 秒，仍受 EKF/GPS 冷启动限制；4 个受管进程清理成功，无残留。

## new 实机部署与实测

- 正确 unit 为 ros2-ardupilot-onboard.service，入口位于 /home/nvidia/ros2-ardupilot-mavros-control。
- 部署前发现服务已于 01:56 被外部停止（journal 有 stop 事务，来源未知），未沿用旧状态启动。
  先完成停机状态的构建，再短时启动仅 sys_status 的 MAVROS 只读探针，收到 connected=true、
  armed=false，探针自动退出后才启动全套服务。该最小探针因未加载 command 插件出现版本查询警告，
  不是生产全套服务故障；没有发起解锁/模式/运动命令。
- 飞机源文件与开发机本次 C++ 源码一致，补丁用 git apply --check 后定点应用，保留飞机现场其他修改；
  配置文件头部的现场注释也保留。部署指南另用定点补丁同步。
- 部署前备份：飞机 agent/codex/startup-fix-20260917/predeploy.tar.gz，包含四个生产源文件与
  install/onboard_control；SHA-256：6cb4feb7367f3e252de5f4a17a7438d0745d81e7c03cf58a7aea63bd97886220。

| 阶段（相对启动，秒） | 修改前 | 修改后第一轮 | 修改后第二轮 |
| --- | ---: | ---: | ---: |
| 四组件全部拉起 | 9.44 | 6.46 | 6.67 |
| 飞控心跳连接 | 8.32 | 8.87 | 9.17 |
| 主动拉取参数请求 | 无 | 8.90 | 9.17 |
| 必要参数校验通过 | 50.75 | 38.91 | 39.23 |
| 参数整表接收完成 | 44.01 | 39.26 | 39.38 |
| 打印 READY | 54.14 | 39.37 | 39.61 |

原点/外部定位约 11～12 秒有效，不是此次剩余等待的主因。两轮新流程均有少量参数缺失补传：
第一轮缺 3 项、第二轮缺 2 项，约在主动请求 30 秒后补传完成。保留日志中的 STAT_RUNTIME index
提示及首轮额外 remote address 检测，未删掉不利数据。未采集串口原始数据，不能断言丢包根因。
若需进一步显著提速，应单独评估按名称优先读取必要参数；本轮未扩大到修改 MAVROS 参数插件。

## 结束状态与同步

new 服务 active、NRestarts=0，已打印 READY；最终状态为 armed=false、STABILIZE、无租约、
controller_active=false，本地位姿/频率/参数校验均通过，控制约 100 Hz、deadline_miss_count=0。
未发送解锁、起飞、运动、模式或原点命令；生产节点正常启动会只读拉参数并配置消息频率。

源码与部署脚本哈希和开发机一致；new 源配置与 install 配置一致；运行中的 /proc/PID/exe 与安装二进制
SHA-256 同为 b82edf55370475b1616704f183955b65dfc5a76cd1dc9d8fe7c1a74c0da5937e。
refresh 与独立 USB Jetson 未部署此次改动，已在 MEMORY 标注，下次连接同步时一并处理。
本地过程证据在 agent/codex/startup-fix-20260917/，包含构建/测试、状态流及单调 journal。
