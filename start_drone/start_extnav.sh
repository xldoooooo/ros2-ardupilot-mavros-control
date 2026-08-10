#!/usr/bin/env bash
# Start the external-navigation bridge with the aircraft's existing Humble overlays.

source /opt/ros/humble/setup.bash
source /home/xld/ws/install/setup.bash
source /home/xld/vrpn_mavros/install/setup.bash

# export ROS_DOMAIN_ID=42
# export ROS_LOCALHOST_ONLY=0

ros2 run extnav_bridge extnav_to_vision_pose --ros-args \
  -p vision_rate_hz:=40.0 \
  -p ctrl_rate_hz:=100.0 \
  -p odom_topic:=/odin1/odometry_highfreq \
  -p roll_cam:=0.0 \
  -p pitch_cam:=0.0 \
  -p yaw_cam:=0.0 \
  # -p odin_x:=0.00 \
  # -p odin_y:=-0.00 \
  # -p odin_z:=0.00
  -p odin_x:=0.06 \
  -p odin_y:=-0.03 \
  -p odin_z:=0.05
