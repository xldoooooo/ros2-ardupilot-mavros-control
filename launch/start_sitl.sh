#!/bin/bash
# Script to start ArduPilot SITL for use with MAVROS

echo "========================================="
echo "Starting ArduPilot SITL - ArduCopter"
echo "========================================="
echo ""

cd ~/ardupilot

# 定义清理函数，用于优雅地退出
cleanup() {
    echo "Exiting ArduPilot SITL..."
    exit 0
}

# 捕获 Ctrl+C 和 SIGTERM 信号
trap cleanup SIGINT SIGTERM

echo "Starting ArduPilot SITL..."

# 启动原生 SITL 仿真，直接运行原生 SITL
sim_vehicle.py -v ArduCopter

# 当 SITL 退出后，打印提示信息
echo "ArduPilot SITL has exited."