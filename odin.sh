#!/bin/bash

source ~/ws/install/setup.bash 
ros2 launch odin_ros_driver  odin1_ros2.launch.py &
sleep 5

# 启动apm.launch
ros2 launch mavros apm.launch fcu_url:=/dev/ttyTHS1:460800 &
sleep 3
# roslaunch livox_ros_driver2 msg_MID360.launch &
# sleep 0.5
# roslaunch fast_lio mapping_mid360.launch &



echo "odin start success!"
sleep 2

source ~/vrpn_mavros/install/setup.bash

# OdometryBridge: 订阅 Odometry，变换逻辑对标 vision_to_mavros
ros2 run extnav_bridge extnav_to_vision_pose --ros-args \
  -p vision_rate_hz:=40.0 \
  -p ctrl_rate_hz:=100.0 \
  -p odom_topic:=/odin1/odometry_highfreq \
  -p roll_cam:=0.0 \
  -p pitch_cam:=0.0 \
  -p yaw_cam:=0.0 \
  -p odin_x:=0.06 \
  -p odin_y:=-0.03 \
  -p odin_z:=0.05 &
echo "extnav bridge (Odin→FCU) start success!"
sleep 0.2

# 定义一个函数来调用服务，使代码简洁
set_rate() {
    local msg_id=$1
    local rate=$2
    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: $msg_id, message_rate: $rate}"
}

# 提高 local position (ID 32) 频率
set_rate 32 100

# 提高 imu/data (ID 31) 频率
set_rate 31 100

# 提高 imu/data_raw (ID 105) 频率
set_rate 105 100

echo "========================================="
echo "Message intervals set successfully!"
echo "========================================="
echo ""


wait
exit 0

