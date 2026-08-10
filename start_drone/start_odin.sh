#!/usr/bin/env bash
# Start the Odin positioning driver with the aircraft's existing Humble overlay.

source /opt/ros/humble/setup.bash
source /home/xld/ws/install/setup.bash

# export ROS_DOMAIN_ID=42
# export ROS_LOCALHOST_ONLY=0

ros2 launch odin_ros_driver odin1_ros2.launch.py
