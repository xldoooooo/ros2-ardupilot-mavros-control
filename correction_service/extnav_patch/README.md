# extnav_bridge

Odin 到 MAVROS 的 C++ 外部导航桥，使用普通单线程 `rclcpp` 和固定尺寸双精度数学，保留 correction 2.0 接口。
生产入口保持 `ros2 run extnav_bridge extnav_to_vision_pose`，机载启动器仍配置 vision 40 Hz、辅助输出100 Hz。

节点直接订阅 `/odin1/odometry_highfreq`，每条输入都处理会话和时间戳；corrected 仅在有匹配订阅者时按原始频率发布，读者每0.1秒查询。
100 Hz 辅助timer和原计数保持；pose/velocity各自在无匹配读者时跳过消息构造及publish，读者加入后按原频率恢复。vision和2 Hz状态不使用此门控。
`/odin1/odometry_highfreq_corrected` 保留 Odin IMU 参考点、原始 header、child frame 和完整 covariance。
`/extnav/pose_fcu`、`/extnav/velocity_fcu` 和 `/mavros/vision_pose/pose` 使用同一条 raw 冻结的 FCU 中心数值及修正身份，最终输出仍使用发布时的 ROS 时间和 `odom` frame。

安装参数与杆臂来自现有 `config/odin_extrinsics.yaml` 或逐机 `ODIN_EXTRINSICS_FILE`。`R_FI` 将 Odin IMU 轴转为 FCU FLU，`T` 是 FCU 指向 IMU 的杆臂、以 FCU 轴表示。
identity 使用 `p_raw + T - R_OF*T`；修正有效时 XY 使用 `p_corrected - R_OF*T`，Z 保留原局部约定。
非零安装角仍拒绝设置有效 Tag correction；没有借本次重写扩大支持范围。

修正服务采用 session/revision CAS，保留幂等、reset计数和原校验顺序。apply、clear、失效后必须等待下一条 raw 建立新快照。
Odin 断流、时间戳回退或 frame 改变使旧修正失效；退出修正服务本身不清除 active。
状态仍为2 Hz加事件，分别报告 active 身份和最终样本冻结身份。

## 构建与部署

先构建项目 `correction_interfaces` 并 source 对应 overlay。隔离构建不修改现有生产安装：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
cmake -S correction_service/extnav_patch -B agent/codex/extnav-candidate/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/agent/codex/extnav-candidate/install"
cmake --build agent/codex/extnav-candidate/build -j2
cmake --install agent/codex/extnav-candidate/build
```

仅在已确认未解锁的维护窗口、飞控栈停止后部署：

```bash
./correction_service/deploy/install_extnav_correction.sh
```

安装器原生构建 Release，备份并迁移外部 `extnav_bridge` 的 source/build/install；从 `ament_python` 切到 `ament_cmake` 时不混用旧缓存，构建失败恢复旧包。
它不启动、重启或解锁飞机。构建环境不变，不需要引入 Eigen 或并发框架。

入口只在启动时选择原生程序。可选 correction 动态库缺失时使用独立编译的 C++ identity 程序，保持原来的缺接口降级，不执行 Python 业务。
缺接口环境可用 `-DEXTNAV_WITH_CORRECTION=OFF` 构建相同 identity 程序。
`extnav_to_vision_pose_native` 与 `extnav_to_vision_pose_identity` 是内部程序，日常使用公共入口即可。

## 维护

- `include/extnav_bridge/math.hpp`：坐标、杆臂、协方差和幂等数值比较；固定修正矩阵按 revision 缓存。
- `src/extnav_to_vision_pose.cpp`：参数、会话、修正事务、快照和 ROS 发布；所有业务回调串行执行。
- `extnav_to_vision_pose.py`：保留的原 Python 对照源码，供现有数学回归与跨实现测试使用，不再是 C++ 安装的生产入口。
- `tests/correction/test_extnav_cpp_equivalence.py`：设置 `EXTNAV_CPP_EXECUTABLE` 后，对照两种实现的公共 ROS 契约。

PoseStamped 仍不能携带 MAVLink estimator reset_counter，原有修正瞬态及非零安装角 Tag 限制保持。
