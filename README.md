# MAVROS + ArduPilot SITL Configuration

This workspace contains the configuration for running MAVROS with ArduPilot SITL (Software In The Loop) simulation.

## Setup Complete ✅

Your workspace is ready! Here's what was created:

### Workspace Structure
```
~/ros2_ws/
├── src/
│   └── mavros_sitl_config/          # MAVROS configuration package
│       ├── launch/
│       │   └── ardupilot_sitl.launch.py   # Launch file for SITL connection
│       └── config/
│           └── mavros_params.yaml         # MAVROS parameters
├── start_sitl.sh                    # Helper script to start SITL
└── start_mavros.sh                  # Helper script to start MAVROS
```

## How to Use

### Option 1: Using Helper Scripts (Easiest)

**Terminal 1 - Start SITL:**
```bash
cd ~/ros2_ws
./start_sitl.sh
```

**Terminal 2 - Start MAVROS:**
```bash
cd ~/ros2_ws
./start_mavros.sh
```

### Option 2: Manual Commands

**Terminal 1 - Start ArduPilot SITL:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --map --console
```

**Terminal 2 - Start MAVROS:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch mavros_sitl_config ardupilot_sitl.launch.py
```

## Verify Connection

**Terminal 3 - Check MAVROS topics:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# List all MAVROS topics
ros2 topic list

# Check heartbeat
ros2 topic echo /mavros/state

# Check GPS position
ros2 topic echo /mavros/global_position/global

# Check local position
ros2 topic echo /mavros/local_position/pose
```

## Connection Details

- **SITL MAVLink Output:** `127.0.0.1:14550` and `127.0.0.1:14551`
- **MAVROS Connection:** `udp://:14550@127.0.0.1:14555`
- **Protocol:** MAVLink v2.0
- **Target System ID:** 1
- **Target Component ID:** 1

## Common MAVROS Topics

### State & Status
- `/mavros/state` - Connection status and flight mode
- `/mavros/battery` - Battery status
- `/mavros/sys_status` - System status

### Position & Navigation
- `/mavros/global_position/global` - GPS position (lat/lon/alt)
- `/mavros/local_position/pose` - Local position (x/y/z)
- `/mavros/imu/data` - IMU data
- `/mavros/altitude` - Altitude data

### Control
- `/mavros/setpoint_position/local` - Set target position
- `/mavros/setpoint_velocity/cmd_vel_unstamped` - Set velocity
- `/mavros/rc/override` - RC override

### Services
- `/mavros/cmd/arming` - Arm/disarm vehicle
- `/mavros/set_mode` - Change flight mode
- `/mavros/cmd/takeoff` - Takeoff command
- `/mavros/cmd/land` - Land command

## Testing Basic Commands

### Arm the Vehicle (in SITL console)
```
arm throttle
```

### Change to GUIDED mode (in SITL console)
```
mode GUIDED
```

### Via ROS2 Service (from Terminal 3)
```bash
# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Set mode to GUIDED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# Takeoff to 10m
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"
```

## Troubleshooting

### MAVROS not connecting
1. Make sure SITL is running first
2. Check that SITL shows MAVLink output on port 14550
3. Look for "IMU: Stabilized" in MAVROS output

### No topics appearing
1. Verify MAVROS is running: `ros2 node list`
2. Source the workspace: `source ~/ros2_ws/install/setup.bash`
3. Check MAVROS logs for errors

### Build issues
If you modify the package, rebuild with:
```bash
cd ~/ros2_ws
colcon build --packages-select mavros_sitl_config
source install/setup.bash
```

## Next Steps

1. Try the basic testing commands above
2. Write custom ROS2 nodes to control the drone
3. Test waypoint missions
4. Experiment with different flight modes

Enjoy flying! 🚁
