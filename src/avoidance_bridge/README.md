<!-- Independent Task38 observation bridge: deployment, coordinate contract and known limits. -->
# 机载独立避障桥

本包只订阅状态/点云，发布地图、预览和带请求身份的规划结果，不发布飞控指令。
控制器保持独立进程；规划器来自独立 `dyn_small_obs_avoidance-ros2` 工作区，不复制其算法。

先构建独立库，再构建主工作区：

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/dyn_small_obs_avoidance-ros2
colcon build --packages-up-to path_planning --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd /path/to/ros2-ardupilot-sitl-hardware
colcon build --base-paths src --packages-up-to avoidance_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/onboard/start_avoidance.sh --check
bash scripts/onboard/start_avoidance.sh
```

独立库路径可用 `AVOIDANCE_WORKSPACE` 覆盖。启动/停止入口为 `scripts/onboard/start_avoidance.sh`
和 `stop_avoidance.sh`，不接管原机载总服务，也不启动硬件。默认 `coordinate_mode:=extnav`；
SITL必须显式传 `coordinate_mode:=identity cloud:=/你的已注册map扫描 odom:=/对应map里程计`。
规划器 `service_only=true` 禁用旧目标定时搜索，`cloud.map_mode=latest_observation` 接收桥的完整地图；
公共YAML默认关闭点云二次盲区滤波。输入超时采用飞机YAML（当前0.5s），桥地图最多4Hz，可选预览仍最多2Hz。
正式桥已提供持续观测地图；默认不再进行规划器近身过滤，也不依赖规划器独立 `/odom` 输入。
桥模式若开启规划器盲区过滤，还需要为规划器另外提供同frame/source时钟的里程计；通常在上游完成过滤。

独立台架入口 `ros2 launch path_planning odin.launch.py` 直接订阅Odin点云；启用半径时要求
云和里程计具有相同frame/source时基，并满足配置的配对时差。
旧版独立入口保留169现场行为：默认订阅 `/odin1/cloud_slam_down`，
盲区由公共YAML配置；显式传 `blind_radius` 时才覆盖。

两个入口默认读取独立库安装目录同一份 `config/planner.yaml`。参数优先级为：显式启动参数 →
YAML（节点专属段优先于公共段）→ launch/节点默认值。文件无法读取、YAML语法或段结构错误时，
输出警告并回退；非法参数值仍由节点校验，不会被静默换成其他值。修改源码YAML后重新构建独立库，
或者用 `config:=/绝对路径/planner.yaml` 直接读取源码；修改配置后须重启节点。

公共 `path_planning.ros__parameters` 保存搜索、时钟、frame、盲区等参数；
`/avoidance/path_planning.ros__parameters` 仅指定 `service_only: true` 和
`cloud.map_mode: latest_observation`。启动文件不再强制覆盖YAML的盲区、搜索下界或时钟。
`avoidance_bridge.ros__parameters` 保存桥的坐标模式、输入话题、预览及地图参数；
未单独填写桥的 `input_timeout` 时继承规划器值。`odin_launch`/`planner_launch` 保存独立入口话题。

可显式覆盖任何规划器参数，例如：

```bash
bash scripts/onboard/start_avoidance.sh search.safe_distance:=0.3 'search.lower:=[-50,-50,-3]'
ros2 launch path_planning odin.launch.py stamp_clock:=receive blind_radius:=0.0
```

`blind_radius`、`map_mode`、`search_lower` 是对应ROS参数的短名，同一次启动不要同时使用短名和完整名。
桥专属参数使用 `bridge.<参数>:=值`，如 `bridge.map_radius:=25.0`；输入话题仍用 `cloud`/`odom`，
预览用 `preview:=true`。显式 `coordinate_mode:=identity` 同时选择 `planning_frame=map`，
可再用 `planning_frame` 显式覆盖；若在YAML选择identity，应同时将规划器frame配置为map。
桥输出 `/avoidance/map` 与规划服务 `/avoidance/plan_motion` 是内部接口连接，不是搜索调参。

`latest_observation` 每次替换的是桥提供的完整观测地图，并非最新一帧雷达；桥负责保存占用、
射线清障及unknown检查。`dual_tree` 由规划器按 `search.tree_period` 交替累积和更换两棵KD树，
按有效点云帧数淘汰历史。二者不可视为相同地图语义。`service_only=true` 禁用独立目标/定时重规划，
保留云更新及PlanMotion服务；`latest_observation` 当前要求 `service_only=true`。

`/odin1/cloud_slam_down` 由Odin host SDK直接提供，默认0.1m降采样及0.4m球形剔除，
配置在驱动 `control_command.yaml` 的 `cloud_downsample` 段；不要再同时运行独立Python降采样脚本。
上游剔除可能删除真实近障；规划器 `cloud.blind_radius=0` 仅表示不再进行第二次剔除。

回归验证使用项目自己的Python环境（先source两个install）：

```bash
ctest --test-dir build/avoidance_bridge --output-on-failure
ROS_DOMAIN_ID=143 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_bridge_ros.py
AVOIDANCE_REAL_PLANNER=1 ROS_DOMAIN_ID=144 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_bridge_ros.py
AVOIDANCE_COORD_MODE=extnav ROS_DOMAIN_ID=145 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_bridge_ros.py
ROS_DOMAIN_ID=146 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python src/avoidance_bridge/test/test_launch_parameters.py
```

三种ROS回归分别使用mock服务、真实独立规划器、mock extnav元数据；全部使用隔离域合成扫描，
不能替代实机外参/覆盖和飞行验收。
参数回归启动独立入口和桥入口，验证CLI/YAML/默认值优先级、节点模式及错误文件回退；
不启动传感器或飞控、无扫描输入，不能据此宣称实际滤波/规划有效。
日志留在 `agent/codex/new-1-path-delay/blind-radius/`。

## 坐标与时效

extnav模式只支持无Tag修正基线：`correction_valid=false`、最新final sample session/revision一致、
新鲜status/raw/FCU pose；有效Tag修正或session/revision/reset/杆臂变化清空地图并拒绝旧结果。
原始障碍在Odin世界 `odom` 中；送规划器的请求位置为 `p_map-T`，速度原样，目标同样减T，
多项式仅常数项减T；返回段仅常数项加T。地面预览 `map` 世界点为原始障碍加T。
T来自权威extnav杆臂字段，不从一次位置差标定。姿态不旋转世界点云，也不按约20°仰角另加旋转。

只读验证按extnav源码计算 `p_FCU=raw+T-(R_IO R_raw R_IO^-1)T`，与最新MAVROS pose差≤0.25m。
Odin设备时钟和MAVROS ROS时钟不同，二者仅按稳态**接收时间**≤0.1s配对，不能宣称采样同步，
运动/传输延迟可能导致保守unknown；不是物理外参或绝对定位精度验收。
原始云与raw里程计则严格按**源header时标**≤0.1s匹配，不能用接收时标冒充。
identity模式不依赖extnav或Odin安装姿态，但仍需与扫描源时钟匹配的 `map` 原点里程计。

## 观测地图

0.1m体素保存占用和自由观测证据，以首个扫描原点为中心半径20m，最多300000格。
没有看见的格始终unknown；遮挡、漏帧和时间流逝不会删除障碍。只有当前扫描射线穿越原格、
且量测终点在其后方至少一个体素对角线，才退役该格占用；本帧命中终点最后写入以覆盖自由证据。
射线按半体素间距离散，最多取10000条，未采到的射线格仍unknown；全部有限终点仍插入占用。
原始输入最多500000点。容量饱和变unknown而不擦除旧障碍，越界区域unknown，重启/坐标重置清图。
最新原始云为0点、schema错误、无有限点、配对/时效失败均unknown，不发布伪造空闲地图。
输入必须是**当前扫描的已注册世界点**；若上游cloud_slam实际包含历史累计点，不能把它的旧点
当成当前射线量测，部署前需确认驱动输出语义。桥本身持有持续地图，服务模式每次替换完整快照；独立Odin入口按 `search.tree_period` 轮换双树，当前为20帧。

XYZ必须各自唯一、FLOAT32/count=1且字节区域不重叠，data长度必须恰等于row_step×height。
源Time要求sec非负、nanosec<10⁹；原始云/里程计源时标必须严格前进，重复不刷新接收年龄并
转unknown，回退清图且拒绝该帧。schema与源序检查发生在2Hz计算节流之前，不能通过重发/节流
掩盖源时钟重置。容量饱和直接拒绝地图输出，需要坐标重置或重启后重新建立证据。

CHECK_LINE要求整条中心线观测格明确自由；PLAN返回轨迹和VALIDATE剩余轨迹同样逐格检查，
未知返回NO_MAP并交给控制器等待，不能把障碍点缺失当作路径clear。曲线以0.01s采样并对相邻点
按半体素间距检查，最多100000样点；这是一项离散中心线观测假设，**不是整个机体扫掠体积、
连续曲线净空或全视野覆盖证明**。稀疏雷达/遮挡可能导致无可执行路径。规划器使用独立库当前核心及0.25m质心阈值；普通原语已有净空证书，
末端shot和直线/剩余曲线服务仍离散调用 `isSafe`，不承诺完整曲线连续净空或
每个原始点/真实障碍表面的包络保证。

自由证据同样按静态场景持久保存：后续没有扫描到的格可能已有新障碍，不能声称动态预测/全场景
持续覆盖保证。扫描射线原点暂用raw IMU里程计中心，尚无经过验收的实际雷达测量中心外参。
FCU中心可能处于前向扫描origin邻近后方的未观测格；严格unknown规则会因此长期等待，未擅自
增加盲区球或把近身未知当free。现场必须确认覆盖/外参后才能声称实际执行可用。

旧规划器的空间曲线直接复用，不降低现场搜索速度/加速度。控制层对超出原Task38包线的完整
曲线做统一时间缩放：段时长乘s，k次系数除以s^k，位置曲线不变，速度/加速度降低。
已满足包线的曲线保持原时间；执行仍限水平1m/s、垂直0.2m/s、加速度0.35/0.15m/s²，
C1、完整末点、60s时长与实际p/v接入检查继续生效。运动中的旧计划若缩放后速度无法接入，
按原逻辑制动等待再从停稳状态规划；没有放宽接入门槛，也没有重写PD+DOB。

## 身份、可视化

控制层收到完整轨迹后，在所有连续三次段上求到最新FCU位置的三维最近点，裁掉此前前缀，
解析平移首段系数；新的零秒用于PD+DOB参考、剩余轨迹校验和RViz路径。等距时取较晚点，
后续只按正时间执行。位置/速度接入容差改在最近点检查，超限仍制动等待；不平移整条空间
路径、不通过回到旧起点补偿延迟。终点保持及原有包线、时效、会话身份检查继续生效。

桥异步最多一个active请求与一个最新pending；超时0.8s，坐标变化或旧请求结果不会接受。
若桥已收到新地图而规划器服务尚未消费该DDS样本，在原请求内间隔50ms重试：PLAN旧图成功
先保留候选段，再对最新图调用VALIDATE_TRAJECTORY；CHECK_LINE/已有VALIDATE重复原查询。
仅与桥当前地图源stamp一致的成功会回控制器；最新图几何失败/unknown仍失败。重试不改变
controller身份、不刷新原始请求时效和active起点，0.8s总期限不延长；候选验证期间planner
session改变也拒绝。避免把一次云/服务回调顺序竞争直接当成执行故障，也不会放行旧图成功。
请求的controller_session/task_revision/waypoint_index/request_id/mode原样回传，bridge_session每进程变化，
coordinate_revision清图时递增，planner_session取服务响应。1Hz request_id=0心跳标明ready，
未发目标时status=WAITING_FOR_INPUT而ready可为true。控制器负责发布其已接受轨迹的RViz Path和清除。
可传 `preview:=true` 开启 `/onboard_control/obstacle_preview`；只有真实订阅者才生成最多10000 XYZ点，
上限2Hz，payload约240kB/s另加DDS开销，局域网实际带宽/CPU成本须实测，默认关闭。

Odin仰角对应的物理外参尚未验收；本包未改Odin-Tag代码，Tag valid分支暂不支持。部署与地面
台架检查不能替代用户手动解锁/飞行验收。
