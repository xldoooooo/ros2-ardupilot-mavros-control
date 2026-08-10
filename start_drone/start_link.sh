#!/usr/bin/env bash
# Start the onboard control link from the deployed aircraft workspace.

cd /home/onboard/ros2-ardupilot-mavros-control
source /opt/ros/humble/setup.bash
source install/setup.bash
# export ROS_DOMAIN_ID=42
# export ROS_LOCALHOST_ONLY=0
ros2 launch onboard_control control.launch.py
