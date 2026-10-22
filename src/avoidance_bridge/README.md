<!-- Native planner coordinate/goal adapter: inputs, lifecycle and deployment. -->
# 机载避障接入

本包将 FCU 状态、业务目标与原生规划器结果做坐标和身份适配。地图、搜索及目标/时间/位移/碰撞触发全部由独立 `dyn_small_obs_avoidance-ros2` 的规划节点负责，采用原生 `dual_tree`。桥不建图、不做射线覆盖/unknown 检查，也不周期请求搜索；没有观测到障碍的区域可通过规划检查。

`onboard_control` 是唯一飞控输出源，以现有 PD+DOB 执行规划器精确段的 p/v/a，保留原段时间尺度；只裁掉规划延迟期间已越过的前缀，不再统一放慢曲线。直线航点策略不依赖规划器；遇障悬停策略用同一地图的 CHECK_LINE，正常飞行和正常到点使用普通直线的参考参数，遇障制动使用自主避障的加速度限额。

## 构建与启动

```bash
bash src/onboard_control/deploy/build_avoidance.sh
bash scripts/onboard/start_avoidance.sh --check
bash scripts/onboard/start_avoidance.sh
bash scripts/onboard/stop_avoidance.sh
```

停止脚本清理本机所有 domain/工作区中的避障启动器、`path_planning_node`、`avoidance_bridge_node` 及已捕获的后代进程，包括孤儿节点；逐级发送 SIGINT、SIGTERM、SIGKILL，确认无活动残留后才成功返回。不会停止 MAVROS、Odin 或 onboard_control；若最终仍有活动进程则返回非零并列出残留。

构建脚本先构建独立库，再重建共享接口、桥、onboard 和仿真入口；不启动硬件或服务。独立库默认位于主工程同级目录，可通过 `AVOIDANCE_WORKSPACE` 指定。旧安装缺少 SetGoal 或新的结果身份字段时必须重新构建，不能只更新 Python/launch。

默认输入为 `/odin1/cloud_slam_down` 与 `/mavros/local_position/odom`。桥将 FCU 实际里程计转换到规划 frame 并转发 `/avoidance/odom`；MAVROS 的 twist 属于机体系，公共 `odometry_twist_in_body=true` 将其转回世界系。独立 `odin.launch.py` 默认使用 Odin 已在世界系的线速度，因此该入口默认为 false；更换里程计来源时须对应设置。SITL 使用 `coordinate_mode:=identity`、map 点云及真实模拟 FCU odom；纯隔离台架可显式提供同 frame 的 odom 输入。

正式输入已经上游降采样并剔除近点；不同时启动独立 Python 降采样进程。规划器盲区默认 0，不再二次剔除。若自行开启盲区过滤，云与 odom 须满足原生源时间配对规则，设备云和 FCU ROS 时钟通常不适合直接配对。

## 配置与目标来源

正式机载入口直接读取主工程 `config/aircraft/<编号>/avoidance/planner.yaml`，仿真固定使用 `simulation` 档案；修改后重启避障，无需重新构建独立库。独立库自身的演示入口仍由独立库维护。

```bash
bash scripts/onboard/start_avoidance.sh search.safe_distance:=0.3
ros2 launch path_planning odin.launch.py search.safe_distance:=0.3
```

直线模式在 `control.yaml` 配置 `straight_max_velocity_xy/z`（1.0/0.2 m/s）和 `straight_max_acceleration_xy/z`（0.35/0.15 m/s²），三个平滑生成器默认继承这组值，原有方法级参数仍可显式覆盖。

自主避障及遇障悬停的紧急制动由原生 `search.max_vel_xy/max_vel_z/max_acc_xy/max_acc_z` 独立配置，默认水平合速度 1.0 m/s、垂直速度绝对值 0.2 m/s，水平合加速度与垂直绝对加速度均为 2.0 m/s²。正常原语和末端曲线在原时间内满足这些约束；实际初速度超限时，保留真实初值并搜索有界减速首段，降入包线后继续正常搜索。减速首段仍检查碰撞、位置边界和加速度；无法安全减速时返回失败。旧 scalar CLI 是两方向的简写，同次显式方向 CLI 优先。原生结果携带解析限额，遇障悬停仅在制动时使用加速度项；正常飞行的速度、加速及到点减速与直线模式一致。遇障后从实际位置和速度生成恒减速度参考，零速后保持预计停点，重复失败不重启制动。自主避障段尾制动预留仍使用原生限额；PD+DOB 输出限幅仍为 4.0/4.0 m/s²，保留跟踪纠偏余量。S 曲线 jerk 与滤波器形状参数沿用现有配置。

原生检查频率默认 10 Hz，时间重规划 0.2 s、位移 1 m、双树轮换 20 个有效点云更新，实际结果发布频率受搜索和输入负载影响。从实际状态规划时直接搜索加速度，不反复插入零加速度滑行首段；窄垂直速度轴补充精确停速候选。其余搜索参数均来自公共配置；没有 bridge 专属规划 profile。

启动正式接入后，手动规划可向 `/avoidance/goal` 发规划 frame 的 PoseStamped；地面站目标由 onboard 经桥调用 `/avoidance/set_goal`。两者进入同一原生目标处理、地图和触发逻辑。独立 `odin.launch.py` 的手动入口仍为 `/goal`，输入参考点默认是 Odin 里程计；正式桥的参考点是 FCU 中心，比较结果时应提供相同状态及转换后的目标。

单个任务航点只设置一次持久目标，投递失败重试保持同一身份；正常重规划无需重发 goal。新 SetGoal 服务支持 active/cancel、target_id 与幂等，取消保留双树地图。任务完成、取消和换点都撤销旧目标；手动目标替换不会自动获得 onboard 飞行控制权。

## 执行与坐标

PlanResult 带进程 session、target_id、sequence 和原搜索时间；桥回传业务身份及递增 plan_sequence。执行器只接收当前目标的新结果，失败时制动等待，最长 15 s 转 LAND；不使用可能停留的旧 kino_path 作为许可。完整轨迹接近终点后可执行完并保持末点，不要求规划器持续刷新结果；原生输入故障和适配器失联仍撤销参考。

执行接入仅在搜索/传输延迟可解释的前缀内找最近点，同位置回返分支按实际速度区分。基础位置/速度接入容差叠加原生加速度对应的时延余量，不修改多项式或时间尺度。自主避障收到首条可接入结果即可恢复；遇障悬停继续使用原停稳与连续通畅窗口。原生失败原因区分越界、净空、超速恢复受阻及搜索预算，便于定位实际等待原因。

跟踪误差超过 `max_reference_error_xy` 的制动判断仅用于自主避障。遇障悬停与普通直线一样继续跟踪正常参考，不因参考位置领先超过0.8 m自动停车；通道阻挡、感知失效及结果超时仍会制动等待。

REACH_HORIZON 只表示中间段，不能触发航点完成或拍照；没有后续结果时在段尾前按避障加速度限额预留时间并等待。到点、低速、停留、拍照和推进航点仍由 onboard 判断。轨迹 yaw 沿现有航点语义管理，规划器只提供平移参考。

extnav 模式使用未应用 Tag 的 local-identity 基线。外参唯一入口为项目 `config/aircraft/<编号>/odin_extrinsics.yaml`，由本机固定编号选择。`R_FI=Rz(yaw_cam)Ry(pitch_cam)Rx(roll_cam)` 将 Odin IMU 轴映射到 FCU FLU 轴，角度单位 rad，向上仰装约 22°对应 `pitch_cam=-0.3839724354`。`T=(odin_x,odin_y,odin_z)` 是 FCU 中心到 Odin IMU 的向量，以 FCU FLU 表达，单位 m。

extnav 对原始 Odin 位姿使用 `R_OF=R_OI R_IF`、`p_LF=p_OI+b-R_OF T`，对速度减去 `R_OF(omega_F×T)`；`b=T` 是保留旧本地原点约定的常量。桥的输入已经是实际 FCU 中心，因此只需从 FCU 本地系减 `b` 映射到 Odin 世界，目标也减 `b`，返回多项式常数项加 `b`。这里减去的不是实时杆臂，不应再次按姿态旋转。MAVROS 的 FCU body twist 和姿态保持不变；Odin 已重力对齐的世界点云不另按安装 pitch 旋转。extnav session/revision/外参变化撤销旧目标与结果。TODO(task6-tag)：Tag 有效分支仍拒绝接入，非零安装角的 Tag 跳变预测尚未适配。

FCU 状态沿用输入超时；2 Hz 的 extnav 坐标身份消息使用独立 2 s 接收超时，避免一个发布周期的普通抖动撤销整个规划输入。消息主动报告失效或身份变化时立即处理。

可传 `preview:=true`，订阅 RViz Obstacle Preview；桥仅限量转发规划器 filtered_cloud，最多 10000 XYZ 点/2 Hz，不另建预览地图。

## 验证

在 source 两工作区后，使用本项目 `.venv`、独立域和 LOCALHOST 运行：

```bash
ROS_DOMAIN_ID=145 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_bridge_ros.py
AVOIDANCE_COORD_MODE=extnav ROS_DOMAIN_ID=146 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_bridge_ros.py
ROS_DOMAIN_ID=147 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_launch_parameters.py
```

原生地图只检查障碍点距离，不保证动态预测、完整表面包络或连续全曲线净空；末端 shot 和几何查询仍有离散碰撞检查。DDS、合成几何及 SITL 不能替代现场传感器/外参和飞行验收。实机解锁与起飞只能由用户手动执行。
