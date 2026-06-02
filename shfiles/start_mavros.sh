#!/bin/bash
# Script to start MAVROS connected to ArduPilot SITL

echo "========================================="
echo "Starting MAVROS for ArduPilot SITL"
echo "========================================="

# Start MAVROS with direct command (more reliable than launch file)
ros2 run mavros mavros_node --ros-args -p fcu_url:="tcp://127.0.0.1:5762"
