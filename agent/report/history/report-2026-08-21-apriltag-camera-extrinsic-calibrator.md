# AprilTag 相机外参标定工具执行简报

- 日期：2026-08-21
- 目标飞机：`nvidia@192.168.112.169`
- 部署目录：`/home/nvidia/AprilTag/`

## 完成内容

在不接管、重启或停止现有机载程序的前提下，部署了一个独立的 Python/ROS 2 标定工具：

- `calibrate_camera_extrinsic.py`：读取 USB 相机中的 AprilTag 36h11，并订阅 Odin 高频里程计；
- `calibration.yaml`：相机、标签地图、粗略外参和采样阈值配置；
- `run_calibration.sh`：加载 ROS 2 Jazzy 与 `/home/nvidia/catkin_ws/install` 后启动程序；
- `README.md`：模型、限制和使用说明。

工具估计二维平面外参 `T_odin_imu_camera=[x, y, yaw]`，并同时为每个有效观测估计
`map→odom`。它使用静止门限、重投影误差门限、鲁棒最小二乘和可观测性检查，退出时写出
YAML 结果及逐帧 CSV。

## 真机只读检查

- 相机为 Wasintek USB Camera，稳定路径为
  `/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0`；
- 相机支持 MJPEG 1280×720，配置采用 30 FPS；
- ROS 2 Jazzy、`/home/nvidia/catkin_ws/install/setup.bash`、OpenCV 4.14、SciPy、NumPy、
  PyYAML 和 rclpy 可用；
- OpenCV 已包含 AprilTag 36h11 字典，不需要新增 Python 依赖；
- 检查时相机无人占用，Odin、MAVROS、外部定位和本标定程序均未运行；
- `/odin1/odometry_highfreq` 在检查时没有数据发布。

## 验证结果

- 本地 Python 编译检查通过；
- shell 语法检查通过；
- 本地合成 SE(2) 自测通过；
- 部署后在真机 ROS/Python 环境中再次运行合成自测通过：已知外参
  `[0.11 m, -0.045 m, 7 deg]` 被准确恢复，Jacobian 秩为 6，条件数约 3.96；
- 自测结束后确认无新增相机占用、Odin/MAVROS/控制进程或后台服务。

## 未完成与风险

- 未执行真实相机 + Odin 联合标定，因为检查时 Odin 话题没有发布，且本任务不允许为验证而
  擅自启停整套机载服务；
- 配置中的相机内参/畸变系数来自此前粗略值，标签边长、ID 和图案朝向也是待确认项；错误值会
  直接形成外参系统误差；
- 当前方案只标定 `x/y/yaw`。若需要精确的完整六自由度外参，应使用标定板或运动捕捉等更强
  约束方法；
- 直接 V4L2 采集没有硬件时间戳，因此首版只在人工静止时采样，运动帧不应作为标定数据；
- 得到的外参在用于飞行前，必须用独立往返数据验证残差与重复性。

## 安全说明

整个部署和验证过程没有解锁、起飞、发布控制命令、修改飞控参数，也没有停止或重启任何现有
机载服务。真实移动采样须由用户保持飞机未解锁并人工携带完成。
