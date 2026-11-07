<!-- FCU odometry input, planar correction and local navigation conventions. -->
# extnav_bridge

Odin 到 MAVROS 的单线程 C++ 外部导航桥，保留 correction 2.0 的 session/revision CAS、幂等请求和计数接口。生产入口为 `ros2 run extnav_bridge extnav_to_vision_pose`。

## 坐标与输入

唯一输入 `/odin1/odometry_highfreq` 必须为标准 `nav_msgs/Odometry`：`header.frame_id=odom`，`child_frame_id=fcu`，pose 是 `O←F`，线速度与角速度均以 FCU FLU 机体系表达。旧 IMU 中心输入会停止最终输出并报告错误，避免重复安装补偿。

固定安装外参由项目 Odin 驱动处理；桥不再旋转机体轴或补偿杆臂。`odin_extrinsics.yaml` 的 `/**/ros__parameters` 与驱动共享安装参数，节点专用 `/extnav_to_vision_pose` 保留频率、话题等设置。`roll_cam/pitch_cam/yaw_cam` 只作为状态元数据；`odin_xyz` 是 FCU→Odin IMU、以水平 FLU 表达的米制向量，同时保留原有局部原点偏移 `b`。自动安装俯仰已禁用；旧 `auto_installation: true` 只产生提示，不修改外参。

最终输出采用以下约定：

```text
无有效 Tag：p_final = p_raw + b，R_final = R_raw
有效 Tag：  p_final.xy = (Rz*p_raw+t).xy
            p_final.z  = p_raw.z+b.z，R_final = Rz*R_raw
```

`b` 是常量局部原点约定，不是再次随姿态旋转的安装杆臂。Tag 仅修正 XY/yaw，Z 继续使用原局部高度。非零安装俯仰可正常应用 Tag 修正。

`/odin1/odometry_highfreq_corrected` 仍表示 FCU 中心：pose 左乘世界修正，pose 的完整 6×6 covariance 按同一 yaw 旋转；FCU 机体系 twist 及其 covariance 原样保留。最终 `/extnav/pose_fcu`、`/mavros/vision_pose/pose` 使用同一 raw 冻结快照。`/extnav/velocity_fcu` 的 header 为 `odom`，线速度及角速度均按最终姿态从 F 旋到该世界轴。

## 时序与修正接口

每条合格 raw 都参与会话处理。接纳数据超时及不合格 frame 使旧修正失效；修正服务退出本身不清 active。apply、clear 或失效后须等待下一条 raw 建立新快照。状态保持 2 Hz 加事件发布，分别报告 active 修正与最终冻结样本身份；安装元数据和 `lever_arm_*` 字段保留原接口。

有效正时间戳的重复样本始终丢弃；距上次接纳样本未超过 `odin_session_gap_seconds`（默认 2 秒）时，所有倒退样本均丢弃，不覆盖缓存或重建 session。超过该超时后允许接纳更小时间戳并建立新会话，使设备重启后可以恢复；仅重复包不能恢复可用状态。旧 `odin_stamp_regression_seconds` 参数仅保留配置兼容，不再参与判断。

`raw_messages` 和 `raw_age_s` 对应接纳的 raw，后者是接收年龄，不是设备端采样延迟。vision 保留源时间戳，每个选中样本最多发送一次；零/负时间戳输入兼容使用接收时的 ROS 时间。辅助 pose/velocity 也不重复发送同一样本，仍使用主机发布时间以兼容自动校准。没有新输入时停止刷新输出，不提前改变现有 session 超时、修正事务和飞行门限。

源时间戳保留不保证任意延迟都能被飞控正确补偿；ArduPilot 的时间偏移估计和 EKF 历史窗口仍需端到端验证。

vision 默认 30 Hz，辅助输出默认 100 Hz，机载启动器可配置为 40/100 Hz。corrected 在有读者时按 raw 频率发布，读者状态每 0.1 秒查询；辅助 pose/velocity 各自在无读者时跳过构造及发布，控制 timer 计数仍推进。

## 构建与部署

先构建项目 `correction_interfaces` 并 source 对应 overlay。隔离构建示例：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
cmake -S correction_service/extnav_patch -B agent/codex/extnav-candidate/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/agent/codex/extnav-candidate/install"
cmake --build agent/codex/extnav-candidate/build --parallel "$(nproc)"
cmake --install agent/codex/extnav-candidate/build
```

部署入口 `./correction_service/deploy/install_extnav_correction.sh` 原生构建 Release 并备份旧包，不启动或解锁飞机。可选 correction 动态库缺失时公共入口选择独立 C++ identity 程序，仍要求标准 FCU 输入。可用 `-DEXTNAV_WITH_CORRECTION=OFF` 单独构建此程序。

## 维护

- `include/extnav_bridge/math.hpp`：世界修正、局部原点、body/world 速度与协方差转换。
- `src/extnav_to_vision_pose.cpp`：参数、会话、修正事务、冻结快照和 ROS 发布。
- `extnav_to_vision_pose.py`：同步语义的 Python 数学及公共接口比较参考，非生产入口。
- `tests/correction/test_extnav_cpp_equivalence.py`：设置 `EXTNAV_CPP_EXECUTABLE` 后验证两种实现的 ROS 契约。
- `tests/correction/test_extnav_installation.py`：验证已补偿 FCU 输入、非零安装元数据和 body twist。
- `tests/correction/test_extnav_auto_installation.py`：验证旧自动补偿参数不再触发第二次转换。

`PoseStamped` 无法携带 MAVLink estimator reset_counter；该计数仍仅在修正状态中报告。
