# ROS2-ArduPilot SITL & Hardware Integration

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter%204.x-green)](https://ardupilot.org/)

> Production-ready framework to control ArduPilot drones with ROS2. **Seamlessly switch between SITL simulation and real hardware** using the same code.

**Battle-tested** on real flights with Cube Orange flight controller and Raspberry Pi 4.

---

## ✨ Key Features

- 🔄 **Dual Environment** - Same mission code works in SITL and real hardware
- ✅ **Flight Tested** - Validated on actual drone flights
- 🛡️ **Safety First** - Comprehensive safety checks and procedures
- 📡 **Ground Control** - Mission Planner/QGroundControl integration included
- 📚 **Complete Docs** - Step-by-step guides for every scenario
- 🎯 **Ready to Use** - Working mission examples included

---

## 🚀 Quick Start

### Prerequisites
- **ROS2 Humble** on Ubuntu 22.04
- **ArduPilot SITL** (for simulation)
- **MAVROS** installed

**Don't have these?** See [Installation Guide](docs/installation/PREREQUISITES.md)

### Test in Simulation (5 minutes)

```bash
# 1. Clone this repository
git clone https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware.git
cd ros2-ardupilot-sitl-hardware

# 2. Build packages
colcon build
source install/setup.bash

# 3. Start SITL (Terminal 1)
./launch/start_sitl.sh

# 4. Start MAVROS (Terminal 2)
./launch/start_mavros.sh

# 5. Run autonomous mission (Terminal 3)
source install/setup.bash
python3 scripts/missions/mission_simple.py
```

**Watch your drone takeoff in the SITL map!** 🎉

---

## 📂 Repository Structure

```
├── src/                          # ROS2 packages
│   ├── simtofly_mavros_sitl/    # SITL simulation configuration
│   └── simtofly_mavros_real/    # Real hardware configuration
├── scripts/missions/             # Autonomous mission examples
├── launch/                       # Helper scripts (start/stop)
├── docs/                         # Complete documentation
└── README.md                     # You are here
```

---

## 📖 Documentation

### Getting Started
- **[Installation Prerequisites](docs/installation/PREREQUISITES.md)** - Install ROS2, MAVROS, ArduPilot
- **[SITL Simulation Guide](docs/guides/SITL_SETUP.md)** - Test safely in simulation

### Hardware Deployment
- **[Real Hardware Setup](docs/guides/REAL_HARDWARE_SETUP.md)** - Deploy on Cube Orange/Pixhawk
- **[Mission Planner Connection](docs/guides/MISSION_PLANNER_SETUP.md)** - Connect ground control station

### Usage
- **[Creating Missions](scripts/missions/README.md)** - Write autonomous flight missions
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions

---

## 🛠️ Tested Hardware

| Component | Tested Model | Status |
|-----------|-------------|---------|
| **Flight Controller** | Cube Orange | ✅ Validated |
| **Companion Computer** | Raspberry Pi 4 (4GB) | ✅ Validated |
| **ROS2 Version** | Humble Hawksbill | ✅ Validated |
| **OS** | Ubuntu 22.04 | ✅ Validated |

**Should work with:** Pixhawk 4, Pixhawk 6C, Cube Black, Jetson Nano

---

## ⚠️ Safety Notice

**Before testing on real hardware:**
- ✅ Remove all propellers during bench testing
- ✅ Secure drone on stable surface
- ✅ Have RC transmitter ready for manual override
- ✅ Read the [Real Hardware Setup Guide](docs/guides/REAL_HARDWARE_SETUP.md)
- ✅ Follow local drone regulations

**Motors WILL spin when armed - propellers off = safe testing!**

---

## 🤝 Contributing

Contributions welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

**Ways to contribute:**
- Report bugs or suggest features
- Test on different hardware
- Improve documentation
- Share your missions

---

## 📜 License

This project is licensed under the Apache License 2.0 - see [LICENSE](LICENSE) file.

**When using this work, please credit:**
```
Based on work by Sidharth Mohan Nair
https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware
```

---

## 🙏 Acknowledgments

- ROS2 community
- ArduPilot developers  
- MAVROS maintainers

---

## 📞 Support

- **Issues:** [GitHub Issues](https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware/issues)
- **Discussions:** [GitHub Discussions](https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware/discussions)
- **Author:** [Sidharth Mohan Nair](https://github.com/sidharthmohannair)

---

**Star ⭐ this repository if it helps your project!**
