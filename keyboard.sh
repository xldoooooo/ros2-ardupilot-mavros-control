#!/bin/bash
## 等到GPS/SLAM定位稳定再启动
echo "========================================="
echo "开始设置原点"
echo "========================================="
echo ""

ros2 topic pub --once /mavros/global_position/set_gp_origin geographic_msgs/msg/GeoPointStamped "{header: {stamp: now, frame_id: 'map'}, position: {latitude: 30.2489634, longitude: 120.2052342, altitude: 488.0}}"


source ~/ros2-ardupilot-mavros-control/install/setup.bash

echo ""
echo "========================================="
echo "Launching keyboard_vel_controller..."
echo "========================================="

# Run directly (not through launch) so stdin is the real terminal.
# Launch doesn't forward keyboard input — use ros2 run + params file instead.
PARAMS_FILE="$HOME/ros2-ardupilot-mavros-control/src/guided_sim/params/keyboard_vel_controller.yaml"
ros2 run guided_sim keyboard_vel_controller --ros-args --params-file "$PARAMS_FILE"

