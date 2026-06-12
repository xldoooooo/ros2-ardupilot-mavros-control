#!/bin/bash
## 等到GPS/SLAM定位稳定再启动
echo "========================================="
echo "Starting Guided Control"
echo "========================================="
echo ""

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

source ~/ros2-ardupilot-mavros-control/install/setup.bash

echo "========================================="
echo "Launching RViz2 visualization (background)..."
echo "========================================="
ros2 launch guided_sim visualize.launch.py &
VIZ_PID=$!
echo "RViz2 started (PID=$VIZ_PID)"
sleep 5

echo ""
echo "========================================="
echo "Launching keyboard_vel_controller..."
echo "========================================="

# ros2 run keeps real stdin — required for keyboard input.
PARAMS_FILE="$HOME/ros2-ardupilot-mavros-control/install/guided_sim/share/guided_sim/params/keyboard_vel_controller.yaml"
ros2 run guided_sim keyboard_vel_controller --ros-args --params-file "$PARAMS_FILE"