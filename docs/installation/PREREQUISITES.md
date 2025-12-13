# Installation Prerequisites

Complete guide to install all dependencies for this framework.

---

## System Requirements

- **OS:** Ubuntu 22.04 LTS (recommended)
- **RAM:** Minimum 4GB (8GB recommended for SITL)
- **Disk:** 10GB free space

---

## Step 1: Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

**Verify installation:**
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

---

## Step 2: Install MAVROS

```bash
# Install MAVROS packages
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y

# Install GeographicLib datasets (required for GPS)
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

**Verify installation:**
```bash
ros2 pkg list | grep mavros
```

---

## Step 3: Install ArduPilot SITL (For Simulation)

```bash
# Install dependencies
sudo apt install git python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y

# Clone ArduPilot
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install MAVProxy and tools
pip3 install MAVProxy

# Setup environment (add to ~/.bashrc)
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
source ~/.bashrc
```

**Verify installation:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --help
```

---

## Step 4: Additional Tools (Optional)

### Mission Planner (Windows) or QGroundControl (Linux/Mac/Windows)

**QGroundControl (Recommended for Linux):**
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

# Download and install
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

---

## Step 5: Setup ROS2 Environment

Add to `~/.bashrc`:
```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Auto-source workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
```

Apply changes:
```bash
source ~/.bashrc
```

---

## Troubleshooting

### ROS2 command not found
```bash
source /opt/ros/humble/setup.bash
```

### MAVROS GeographicLib error
```bash
# Re-run the install script
sudo ./install_geographiclib_datasets.sh
```

### SITL won't start
```bash
# Check Python dependencies
pip3 install --upgrade pymavlink MAVProxy
```

---

## Next Steps

✅ All dependencies installed  
→ Continue to [SITL Setup Guide](../guides/SITL_SETUP.md)

---

**Author:** Sidharth Mohan Nair  
**GitHub:** https://github.com/sidharthmohannair
