#!/bin/bash
# Script to start ArduPilot SITL for use with MAVROS

echo "========================================="
echo "Starting ArduPilot SITL - ArduCopter"
echo "========================================="
echo ""
echo "SITL will start on the following ports:"
echo "  - MAVLink output 1: 127.0.0.1:14550"
echo "  - MAVLink output 2: 127.0.0.1:14551"
echo ""
echo "MAVROS will connect to port 14550"
echo ""
echo "========================================="

cd ~/ardupilot

# 定义清理函数
cleanup() {
    echo "Shutting down Gazebo gracefully..."
    if [ -n "$GAZEBO_PID" ]; then
        # 先发送 SIGINT (Ctrl+C) 让 Gazebo 自行清理
        kill -SIGINT "$GAZEBO_PID" 2>/dev/null
        # 等待最多 5 秒，让它自己退出
        for i in {1..5}; do
            if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        # 如果还没退出，再强制终止
        kill -9 "$GAZEBO_PID" 2>/dev/null
    fi
    exit 0
}

# 捕获 SIGINT (Ctrl+C) 和 SIGTERM
trap cleanup SIGINT SIGTERM

echo "Starting Gazebo..."
gz sim -v4 -r iris_runway.sdf &
GAZEBO_PID=$!

sleep 5

echo "Starting ArduPilot SITL..."
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON

# 当 SITL 退出后，自动关闭 Gazebo
echo "Shutting down Gazebo..."
kill $GAZEBO_PID 2>/dev/null
