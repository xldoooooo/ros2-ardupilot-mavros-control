<!-- 用户追加距离参数对齐的源码依据、修改对比、验证结果与 new 台架操作说明。 -->
# Task37追加：按ROS1原项目对齐碰撞距离

日期：2026-10-08。用户要求：当前约0.64 m过于保守，采用ROS1原项目相关设置，
给出参数修改前后对比及飞机上简要验证步骤。

## 来源与改动

只读原始HKU-MARS检出 `agent/codex/task36/upstream-review`，HEAD为
`9d25dd6974e04518c892e5711c9be9082558e5eb`，工作树干净，没有改原始ROS1项目。
默认参数来自 `path_planning/launch/demo.launch`，距离判定来自
`path_searching/src/kinodynamic_astar.cpp:121`，硬编码SAFE_DIST来自搜索头文件。
当前独立库修改前为 `bde0647`；本次代码提交 `01b3785`，两平台验证文档 `5f9e5c7`，
main已推送并同步new原目录；文档提交不改变已测二进制。主工程仅更新MEMORY与本新报告。

原ROS1查询KD-Tree中的体素质心，判断 `distance² < 0.45²` 才拒绝；体素叶大小0.1 m。
ROS2之前额外加了 `sqrt(3)*0.1 + 0.05/2`。按本次明确授权直接移除两项附加半径，
没有通过把safe_distance伪设为0.251795 m抵消补偿；配置值保持真实的0.45 m。
比较符号由小于等于改为严格小于，数值等号边界与ROS1一致（PCL距离为float，有舍入误差）。

| 参数/行为 | 修改前ROS2 | 修改后ROS2 | ROS1原项目 |
| --- | --- | --- | --- |
| `search.safe_distance` | 0.45 m | 0.45 m | `SAFE_DIST=0.45` m |
| 附加体素位移半径 | sqrt(3)×0.1=0.173205 m | 0 | 无 |
| 附加采样间隙半径 | 0.05/2=0.025 m | 0 | 无 |
| 实际查询距离阈值 | 0.648205 m | **0.45 m** | **0.45 m** |
| 占用比较 | distance² ≤ radius² | **distance² < radius²** | **distance² < SAFE_DIST²** |
| `search.voxel_size` | 0.1 m | 0.1 m | 0.1 m |
| `search.max_tau` | 0.6 s | 0.6 s | 0.6 s |
| `search.init_max_tau` | 0.8 s | 0.8 s | 0.8 s |
| `search.max_vel` | 2 m/s，每轴 | 2 m/s，每轴 | 2 m/s，每轴 |
| `search.max_acc` | 2 m/s²，每轴 | 2 m/s²，每轴 | 2 m/s²，每轴 |
| `search.w_time` | 10 | 10 | 10 |
| `search.horizon` | 100 m | 100 m | 100 m |
| `search.lambda_heu` | 5 | 5 | 5 |
| `search.resolution` | 0.1 m | 0.1 m | `resolution_astar=0.1` m |
| `search.time_resolution` | 0.8 s | 0.8 s | 0.8 s |
| `search.allocate_num` | 100000 | 100000 | 100000 |
| `search.tree_period` / 树数 | 50帧 / 2 | 50帧 / 2 | KT_HORIZON=50 / KT_NUM=2 |

程序对比ROS1 launch与实际ROS2 YAML中的10项共有搜索参数，全部一致；
结果为 `agent/codex/task37/ros1-clearance-parameter-comparison.json`。
该目录的 `compare_ros1_parameters.py` 可复核，不连接ROS或FCU。

以下是ROS2独有或并非距离参数，本次保持原设置：`search.collision_step=0.05 m`、
`search.search_budget=0.3 s`（核心直接调用0.08 s）、planning_rate=10 Hz、input_timeout=0.5 s、
sample_step=0.02 s、输出10000点/60 s/0.1 s构造预算、通用盲区默认0。
Odin入口仍为odom/receive、z下界-2 m，台架可显式使用0.5 m盲区。
ROS1的collision采样按端点位移决定数量，并没有0.05 m同名距离参数；
原launch的 `search/margin=0.2` 未读入有效碰撞规则，`search/check_num=1` 也未被实际循环使用，
不能误把这两项当安全距离。ROS1输出轨迹采样为0.01 s，不是碰撞采样间距。
本次对齐距离与共有搜索默认值，没有恢复ROS1粗采样或其原始高度硬编码。

## 验证与限制

- 本地两包Release构建通过；18/19核心、6/6输出helper、5/5真实ROS进程、3/3基准工具通过。
  即33个逻辑用例32通过、1失败；colcon汇总37项、0 errors、2 failures、0 skipped，
  失败为同一个核心用例和它的CTest包装项，不能计成两个独立失败。
- 新增 `Ros1ClearanceHasNoExtraInflationAndUsesStrictBoundary` 验证0.44 m拒绝、0.46 m接受，
  自定义可精确表示的0.5 m半径验证0.49拒绝、0.5接受、0.51接受。
- 原盲区回归把0.55 m障碍当起点阻塞，其预期仅适用于旧0.648 m判定。
  改用距起点0.4 m、距盲区球心0.6 m的障碍，保留“球外真实点不删除且阻塞起点”检查，
  所有ROS进程用例通过。首轮失败日志没有删除。
- **原严格连续净空回归保留，未放宽断言。** `CollisionBetweenPrimitiveEndpoints` 使用
  自定义safe_distance=0.1 m、voxel=0.01 m、collision_step=0.025 m，
  其独立密采样实际最小距离0.099990361693828175 m，比声明距离少0.000009638306 m，
  因此仍失败。移除ROS2附加余量后恢复的是ROS1离散点判定，不能同时宣称连续净空保证。
  这不是默认0.45 m场景的现场数据。原测试结果如实保留，不标成通过或跳过。
- 修改前afc868b下599次成功的台架数据、0.648 m曲线审核与性能数字仍为历史证据，
  不将其套用于新0.45 m策略；本轮不替用户发布现场目标或执行飞行验证。

日志：`agent/codex/task37/ros1-clearance-{build,test,results}-amd64*.log`及对应arm64日志。

new原目录已整体快进代码 `01b3785`，两包原生Release构建完成（约1分27秒），
原生正式测试同样33个逻辑用例32通过、1失败，colcon37项/0 errors/2 failures/0 skipped。
该唯一失败的实际距离也是0.099990361693828175 m，未放宽或跳过；其余18核心、6 helper、
5 ROS进程、3基准工具均通过。

安装节点在localhost/domain 229隔离启动，不消费飞机domain 0输入，也未发布任何目标，
实际读回：safe_distance=0.45、voxel_size=0.1、collision_step=0.05、search_budget=0.3，
blind_radius=0.5（显式Odin台架参数）、planning_frame=odom、stamp_clock=receive。
首次参数探针误用233域，超出本机FastDDS可用端口范围，未能启动；完整错误日志保留为
`ros1-clearance-installed-node-domain233-failed.log`。改用229域后正常启动、参数读回并SIGINT退出。
这项错误仅属于隔离探针，不是生产域启动失败或源码参数问题。

安装证据 `ros1-clearance-install-identity-arm64.txt`：

- 核心库SHA256：`2fec36c15dbd5be4c289eca6a84ca8a812551ffc72eb07caf674a829d31a8e5d`。
- YAML SHA256：`57770959f1e9854685f5886c497bab2c299a8ea17560351c3f550ea12707a152`。
- wrapper SHA256仍为 `e241704773af15d25a40b87341b262aedecc1c2ee24f9872eef5b9b784c6cefe`；
  它动态链接更新后的核心库，wrapper代码未变，不能仅凭wrapper哈希判断此次策略是否更新。

直接回读FCU connected=true、armed=false、STABILIZE；原Odin launch3508、Odin3600、
onboard_control3597、MAVROS3625保持原PID。原生产服务没有重启，隔离验证节点已退出。
本轮未自行解锁、起飞或发送飞行指令。refresh未连接、未同步，下一次须补同步本次及前次修改。

## 飞机操作（new，独立规划台架）

已有Odin/MAVROS等服务无需重启；本独立包只发布规划结果，没有接入飞行跟踪。
下面的目标是odom绝对坐标，示例并不是自动计算的相对前向4 m；按现场实际位姿设置。

1. 第一终端SSH并启动规划器，保持此终端运行：

   ```bash
   ssh drone-new
   cd /home/nvidia/dyn_small_obs_avoidance-ros2
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   .venv/bin/python /opt/ros/jazzy/bin/ros2 launch path_planning odin.launch.py \
     blind_radius:=0.5 goal:=/avoidance_test/goal
   ```

2. 第二终端同样SSH、cd并source上述环境后，读当前里程计、确认参数，发布一次规划目标：

   ```bash
   .venv/bin/python /opt/ros/jazzy/bin/ros2 param get /path_planning search.safe_distance
   .venv/bin/python /opt/ros/jazzy/bin/ros2 topic echo /odin1/odometry --once --field pose.pose.position
   .venv/bin/python /opt/ros/jazzy/bin/ros2 topic pub --once /avoidance_test/goal geometry_msgs/msg/PoseStamped \
     '{header: {frame_id: odom}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
   .venv/bin/python /opt/ros/jazzy/bin/ros2 topic echo /plan_result --field status
   ```

   2=REACH_END（到达目标轨迹）、1=REACH_HORIZON（部分轨迹）、3=NO_PATH。
   其他失败读完整 `/plan_result` 的detail；出现旧Path线条不代表本轮有效。

3. RViz2的Fixed Frame设为odom，添加原始 `/odin1/cloud_slam`、过滤 `/filtered_cloud`
   （PointCloud2/Best Effort）和 `/kino_path`（Path）。结束时第一终端Ctrl+C。
   本次验证规划输出即可，解锁/起飞只能由用户人工操作；本独立库尚不控制飞机沿曲线飞行。
