#!/usr/bin/env bash
# Start MAVROS on the aircraft flight-controller serial link.

source /opt/ros/humble/setup.bash
# export ROS_DOMAIN_ID=42
# export ROS_LOCALHOST_ONLY=0

ros2 launch mavros apm.launch fcu_url:=/dev/ttyTHS1:460800
