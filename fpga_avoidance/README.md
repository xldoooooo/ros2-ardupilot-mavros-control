<!-- 可移除 FPGA 旁路的独立安装、配置及启停入口。 -->
# FPGA 避障旁路

此目录封装 CPU A* + FPGA Ising/QUBO 后端，复用现有 `path_planning` 协议、坐标桥和机载执行器。默认避障仍启动 CPU；未选择 FPGA 时不加载本目录的配置、overlay 或 Python 依赖，也不探测板卡。`COLCON_IGNORE` 将整个目录排除在主工程默认构建之外。

## 安装与配置

先按原方式构建主工程及独立 CPU 规划器，再单独运行：

```bash
./fpga_avoidance/install.sh
./fpga_avoidance/start.sh --check
```

`install.sh` 在 `workspace/.venv` 创建独立 Python 环境，仅在缺少时安装 NumPy/PyYAML，并调用 `build.sh` 构建 `fpga_uav_native`、`fpga_uav_planner_ros2`。这些步骤均不启动板卡探测。已有环境后可只运行 `./fpga_avoidance/build.sh`。

默认源码和产物位于本目录 `workspace/src`、`workspace/install`。可用 `FPGA_AVOIDANCE_WORKSPACE` 指定另一个工作区（支持包含 `ros2_ws` 的外部工程根目录），用 `AVOIDANCE_WORKSPACE` 指定现有 CPU 协议工作区。各机均需本机原生构建，不复制跨架构产物。若环境在旧外部工程根目录，其 Python 环境可通过本工作区创建，不依赖原根目录 `.venv`。

专用设置集中于 `config/backend.yaml`：`fpga_ip`、`bind_ip`、`udp_port` 指定 UDP 链路，`mock` 默认为 false；其余参数配置原规划器的栅格、机体净空、轨迹速度/加速度上限及重规划周期。可用 `FPGA_AVOIDANCE_CONFIG` 或 `start.sh --config /absolute/backend.yaml` 覆盖。规划 frame、点云、速度坐标约定和搜索边界由当前 CPU 启动配置传入；适配后的 FCU 中心里程计沿用 `/avoidance/odom`。

`max_velocity_xy/z` 和 `max_acceleration_xy/z` 为执行轨迹的方向限额，结果向原执行器回显相同值。规划对象从内部航点构造真实 Hermite 三次曲线，保留实测起速、终端零速，并解析检查速度/加速度；整条曲线通过占据体素的 Bezier 凸包细分验证。体素膨胀含输入量化余量，查询与曲线使用同一地图，净空不能直接等同于 CPU 的质心距离阈值。当前只保证单条曲线 C1，不保证跨计划加速度或 jerk 连续。

默认几何与运动参数对齐 `config/aircraft/uav002/avoidance/planner.yaml`：分辨率 0.1 m、名义净空 0.35 m、速度 2 m/s、加速度 1 m/s²，输出采样 0.02 s。这里以 `uav_radius_m=0.35`、`safety_margin_m=0` 表达名义净空，并非新增机体尺寸标定。FPGA 另加完整体素对角线补偿，传给膨胀函数的实际半径为 `0.35 + sqrt(3)*0.1 ≈ 0.5232 m`；CPU 仍比较质心距离 0.35 m，两者几何定义不同。`safe_clearance_m` 是原 QUBO 的额外净空偏好，不是地图膨胀半径，不按名称照搬 CPU 阈值。其他机型和仿真档案可能不同，需在专用配置中显式调整。

机型由各主机 `config/aircraft.local.env` 独立选择，FPGA 参数不会自动跟随所选 CPU 档案；不能根据主机名称推断档案编号。`max_z_cells=100` 支持 0.1 m 栅格覆盖 uav001 的 0.2–10 m 高度范围；这是地图容量上限，实际搜索边界仍继承当前机型。

`near_goal_replan_distance=0.3` 与 CPU 的末段调度一致：仅在已有成功曲线仍通过当前地图、输入及板卡检查时保留原零速末段，避免不断重拟合追逐终点；到点仍由原控制器判定。板卡 PING 持续进行，堵路、失效或超时立即发布空失败，不继续旧结果。原规划器的近点直线和 CPU 质量/耗时回退仍会发生，界面显示实际来源；可显式配置 `near_goal_direct_m`、`strict_spine_latency_fuse_ms`，后者默认 3.5 ms。

## 选择与停止

正常启动原避障后，在地面站选择“FPGA 加速”，或在相同 ROS domain 下调用：

```bash
ros2 service call /avoidance/backend/select std_srvs/srv/SetBool '{data: true}'
./fpga_avoidance/stop.sh
```

切换要求机载权威状态为未解锁、已落地、空闲且无航点任务。管理器先启动独立进程、确认实际板卡后端 ready，再修改桥的 `native_namespace`；失败不会重启 CPU、飞控或 Odin。MOCK 结果不能选入飞行链路。Linux 子进程在启动脚本前绑定管理器死亡信号，管理器异常退出时模块也收到 SIGTERM，避免孤留探板；此退出不会自动切换桥来源。`stop.sh` 通过同一门控选回 CPU，再结束管理器拥有的 FPGA 进程组。直接在终端运行 `start.sh` 用于单独诊断时，使用 Ctrl-C 结束该进程。

`/avoidance/backend/status` 为每秒发布的可靠、持久最近一条 JSON 状态，包含 `backend`、`available`（入口及独立构建标记存在）、`ready`、`error`、`source`、`transitioning`。CPU 的 `ready` 来自新鲜机载 `ControlStatus.avoidance_ready`，仅用于显示当前规划链路状态。未选中时仅检查入口和安装标记；实际硬件就绪在主动选择后确认。FPGA 原始诊断位于 `/fpga_avoidance/status`，服务及结果亦隔离在 `/fpga_avoidance`。真实求解来源、硬件事务数及参与率照实显示；选中后故障保留当前来源并通过原执行链路处理，不在空中切回 CPU。

失败计划也记录本次 `planning_ms`、`solve_ms` 和 `cpu_ms`；CPU 时间为该任务提交至处理完成期间的进程 CPU 时间，含 ROS 回调及最新地图复核，不是整机占用百分比。规划错误保留到成功规划、更换目标或取消，不因一次成功 PING 隐藏；板卡连接正常也可能因当前轨迹不合格而显示未就绪。

## 移除

先在落地未解锁状态调用 `stop.sh`，再删除或移出整个 `fpga_avoidance/` 目录即可。原构建与启动不依赖本目录，旧的 FPGA 界面偏好不会自动启用模块；下一次启动始终默认 CPU。无需修改 CPU 配置或重新安装公共飞行协议。

## 本地验证

安装后执行 `./fpga_avoidance/test.sh`，使用独立 Python 环境、domain 226 / LOCALHOST，验证真实多项式、DDS 目标生命周期、终端保持及故障撤销。测试只有软件 MOCK 求解，不接板卡或飞控；生产管理器拒绝选择 MOCK。真实板卡与 SITL 联合验收仍需实际硬件，不能由这些测试代替。
