# map 系无人机位姿终端解算脚本执行简报

日期：2026-08-31

## 任务结果

- 新增独立脚本 `odom_pose_in_map.py`。
- 已部署到当前飞机：
  `/home/nvidia/ros2-ardupilot-mavros-control/odom_pose_in_map.py`。
- 脚本只订阅 `/tf` 与 `/odin1/odometry_highfreq`，不发布话题、不调用服务、不修改飞控状态。
- 已为 `nvidia@192.168.112.169` 配置专用 Ed25519 SSH 密钥；本机别名 `drone-new` 已通过
  `BatchMode` 无密码登录验证。
- 全程未解锁、未起飞，未停止或重启任何机载服务。

## 实机接口核对

2026-08-31 在飞机当前 ROS 2 Jazzy 会话只读检查得到：

- `/tf`：`tf2_msgs/msg/TFMessage`，实际目标变换为
  `header.frame_id=odom, child_frame_id=map`；
- `/odin1/odometry_highfreq`：`nav_msgs/msg/Odometry`，实际为
  `header.frame_id=odom, child_frame_id=imu`。

因此 `/tf` 给出的不是 `map->odom`，而是 map 坐标系原点及朝向在 odom 坐标系中的位姿
`T_odom_map`。Odin 给出 `T_odom_imu`，所求为：

```text
T_map_imu = inverse(T_odom_map) * T_odom_imu
```

展开到位置与姿态：

```text
p_map_imu = R_odom_map^T * (p_odom_imu - p_odom_map)
q_map_imu = conjugate(q_odom_map) * q_odom_imu
```

四元数采用 ROS 的 `(x, y, z, w)` 顺序，乘法为 Hamilton 积。脚本会归一化输入/输出四元数，
并拒绝非有限值、零四元数和 frame 不匹配的数据。

## 运行方法

在飞机执行：

```bash
ssh drone-new
cd /home/nvidia/ros2-ardupilot-mavros-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./odom_pose_in_map.py
```

默认每秒打印 10 次 map 系下的 `x/y/z`、四元数和 `roll/pitch/yaw`。如需打印每一帧：

```bash
./odom_pose_in_map.py --ros-args -p output_rate_hz:=0.0
```

## 验证情况

- 项目 `.venv` 执行 `py_compile`：通过。
- 90°旋转加平移的独立 SE(3) 逆变换/复合断言：通过。
- `git diff --check`：通过。
- 飞机端运行约 3 秒：成功接收两个话题并连续输出 map 系位姿。
- 静止台架输出约为 `x=0.413 m, y=-0.187 m, z=0.055 m, yaw=1.57°`，数值连续稳定。

## 边界说明

- 当前实现对每帧 odometry 使用回调前最近收到的 `odom->map` TF；适用于当前两个实时连续话题。
- 输出的子坐标系沿用 Odin 消息的 `child_frame_id=imu`，因此这是 IMU 原点位姿，不是相机光心或
  其他机体参考点位姿；若需要 `base_link`，还需再复合已标定的 `imu->base_link` 外参。
- 此脚本是终端诊断工具，没有加入飞控、extnav 或 systemd 自动启动链。
