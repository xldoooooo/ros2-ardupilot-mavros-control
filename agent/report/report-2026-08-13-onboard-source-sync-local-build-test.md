# 机载服务源码同步与本地构建测试简报

## 完成结论

- 日期：2026-08-13。
- 已审查真机当前版本 `31b19fc` 之后的机载相关提交，重点覆盖接口 3.0、四种航点参考生成器、
  轨迹 PD+DOB 增益切换、航点速度保护、同一武装周期配置锁和解除武装后的状态清理。
- 未发现阻止源码同步的编译、接口或明显安全状态机缺陷。
- 真机 `/home/onboard/ros2-ardupilot-mavros-control` 已从 `31b19fc` 快进到 `4741066`；
  `src/onboard_control` 与 `src/guided_interfaces` 的 Git tree 对象和本地完全一致。
- 本次只同步 sparse 源码，没有重启 systemd 服务，没有在真机构建或替换 `install/` 产物；当前
  运行实例仍是接口 2.2。没有申请控制租约、发送飞行命令、解锁或起飞。

## 代码审查要点

- `e830f01` 将 `ExecuteWaypoints`/`ControlStatus` 升级到接口 3.0，并新增位置阶跃、二阶滤波、
  梯形速度和限 jerk S 曲线参考生成器。连续参考统一沿三维直线规划，速度、加速度和 jerk 限制
  由 XY 模长及 Z 分量共同换算；100 Hz 路径没有每周期优化器或动态工厂构造。
- `DobController` 的轨迹模式只切换独立反馈/DOB 增益并按需加入参考加速度前馈，保留运行时从
  `MOT_THST_HOVER` 获得的推力标定和既有姿态、加速度、扰动硬限幅。
- 机载服务校验参考生成与跟踪控制枚举，平滑任务要求低速起步、低速到达并监控持续超速；同一
  武装周期禁止更换实验组合，解除武装后清锁。任务失败会转入悬停或既有 LAND 保护路径。
- `aaddce7` 仅把 `control.yaml` 参数说明改为中文，没有改变参数键值。
- `4741066` 并非纯文档修改：平滑航点水平参考速度从 0.30 提高到 1.00 m/s，水平实际速度保护
  从 0.42 放宽到 1.20 m/s，相关加速度、jerk、到达速度及保持时间也有调整。数值仍处于代码
  接受范围并受控制器硬限幅约束，但本次没有真机飞行验证，不能把它们报告为已验证实机参数。

## 真机同步证据

- 真机系统：Ubuntu 22.04/Humble/aarch64，服务
  `ros2-ardupilot-onboard.service` 在整个同步期间保持同一主进程 PID 2452，启动时间为
  `2026-08-13 02:53:50 CST`，最终仍为 `active/running`。
- 同步前备份：
  `/home/onboard/ros2-ardupilot-mavros-control/.deployment-backups/`
  `pre-source-sync-20260813-025708.tar.gz`。
- 备份 SHA-256：
  `092f15ea2497218ce5af47d21ad0338980e0dea1d3b415c6027c1bba10f64fab`。
- 第一次 partial-clone 按需对象下载出现 Git HTTP 408；快进实际已完成。随后独立复核确认
  `HEAD=origin/main=4741066`、已跟踪工作树干净、新参考生成器存在，最终 tree 对象为：
  - `src/onboard_control`: `6ccbeb9dfee24e9cd08178ee7680e5f64819888f`；
  - `src/guided_interfaces`: `94bcf3065421d14f6178853eebb9947c63681940`。
- `.deployment-backups/` 是预期的未跟踪备份目录，没有覆盖或删除其中既有内容。

## 本地构建与测试

- `colcon build --packages-select guided_interfaces onboard_control guided_sim --cmake-args
  -DCMAKE_BUILD_TYPE=Release`：3 个包成功。
- `colcon test`：13 tests，0 errors，0 failures，0 skipped；其中 PD+DOB 6 项、参考生成器 5 项。
- 项目 Python 环境加载 Jazzy 与本地 overlay 后：`103 passed in 27.86s`。
- Python 首轮未 source `install/setup.bash` 时有 3 项因无法导入 `guided_interfaces` 失败；修正测试
  环境后全量复测通过，该首轮环境错误未被隐瞒或计为代码通过。
- `control.yaml` 用项目 Python/PyYAML 解析成功，共 67 个参数；启动脚本 Bash 语法和
  `git diff --check` 通过。
- domain 232 + `LOCALHOST` 隔离 smoke 通过：接口 3.0、`fcu_connected=false`、
  `armed=false`、2 秒观察窗内零姿态 setpoint；未接触 MAVROS 或真机。

## 现场边界

- 真机源码已经同步，但运行中的 `install/` 仍是接口 2.2；要让接口 3.0 实际上线，后续必须在
  真机完成 Humble/aarch64 构建、测试、隔离 smoke，并由用户选择安全窗口重启服务。
- 当前启动日志明确提示 full readiness 未达到；历史与本次日志均不能证明当前真机飞行 READY。
  在另行查清参数同步/状态发现并完成无桨验证前，禁止据本次源码同步进行飞行。
- 本次没有停止自启动栈；Odin 日志仍可见其既有配置目录/校准文件告警，与本次源码同步无关，
  未擅自扩展任务范围处理。
