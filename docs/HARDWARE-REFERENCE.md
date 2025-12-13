# Hardware Reference

**Test Platform:** FuryVision AAV

This framework was developed and tested using the **FuryVision AAV** - a fully open-source autonomous air vehicle.

---

## About FuryVision AAV

**Repository:** https://github.com/sidharthmohannair/Fury-Drone-Project

**Design:** Open Drone Team, CDOH Lab, ICFOSS, Thiruvananthapuram  
**Based on:** Requirements from SPIN Laboratory, IIT Kanpur

### Key Specifications

| Component | Details |
|-----------|---------|
| **Frame** | Custom carbon fiber with 3D-printed parts |
| **Flight Controller** | Pixhawk Orange Cube+ |
| **Companion Computer** | Raspberry Pi 4, 8GB |
| **GPS** | Here 3+ |
| **Motors** | Emax 2306-2400KV Brushless |
| **ESC** | Holybro Tekko 4in1 50A |
| **Battery** | 14.8V 2200mAh LiPo |
| **All-Up Weight** | 1161g |
| **Flight Time** | 8 minutes 40 seconds |
| **Sensors** | 4x Intel RealSense cameras |
| **Firmware** | ArduCopter v4.5.6 |

### Flight Performance

**Validated Flight Modes:**
- ✅ Stabilize
- ✅ Altitude Hold
- ✅ Loiter
- ✅ Guided (Autonomous)
- ✅ RTL (Return to Launch)

**Temperature Performance:**
- Motors: Max 50.1°C post-flight
- ESC: Max 53.5°C post-flight

**Battery Performance:**
- Pre-flight: 16.15V
- Post-flight (8m 40s): 13.5V
- RTL voltage: 3.3V per cell

---

## Why This Platform?

### ✅ Fully Open Source

**Everything is documented:**
- CAD files (STL, design files)
- Bill of Materials (BOM)
- Assembly instructions (PDF guide)
- Wiring diagrams
- Pre-tuned parameters
- Flight test results

### ✅ Production Ready

**Real-world validated:**
- Multiple successful flights
- Stable performance across all modes
- Thermal performance documented
- Battery endurance tested
- Component reliability proven

### ✅ Developer Friendly

**Perfect for integration:**
- USB connection to Raspberry Pi
- Network accessible via WiFi
- Standard MAVLink protocol
- Mission Planner compatible
- ROS2 integration ready

---

## Getting the Hardware

### Option 1: Build Your Own

**Full documentation available:**
- Assembly Guide: [PDF](https://github.com/sidharthmohannair/Fury-Drone-Project/blob/main/versions/2_furyvision_aav/docs/Assembly%20of%20FuryVision%20AAV.pdf)
- CAD Files: [Hardware folder](https://github.com/sidharthmohannair/Fury-Drone-Project/tree/main/versions/2_furyvision_aav/hardware)
- Parameters: [fury_drone1.param](https://github.com/sidharthmohannair/Fury-Drone-Project/blob/main/versions/2_furyvision_aav/firmware/fury_drone1.param)

### Option 2: Use Compatible Hardware

This framework works with any:
- Cube Orange / Cube Orange+ / Pixhawk 4 / Pixhawk 6C
- Raspberry Pi 4/5 or Jetson Nano
- ArduCopter firmware (4.3+)

**Just adapt the connection settings for your specific hardware.**

---

## Integration with This Framework

### Serial Connection

**FuryVision AAV setup:**
```bash
# Flight controller connected via USB
/dev/ttyACM0:921600
```

**MAVROS configuration:**
```bash
./launch/start_mavros_real.sh
# Uses /dev/ttyACM0:921600 by default
```

### Network Setup

**WiFi configuration:**
- Raspberry Pi: 10.73.2.222
- Laptop (Mission Planner): 10.73.2.236
- Protocol: UDP port 14550

**Automatically configured** when using `start_mavros_real.sh`

---

## Test Results

**This framework validated on FuryVision AAV:**

✅ **SITL Testing** - All missions work in simulation  
✅ **Bench Testing** - Commands accepted, motors respond  
✅ **Flight Testing** - Autonomous missions completed  
✅ **Mission Planner** - Telemetry and control confirmed  
✅ **Network Integration** - UDP forwarding works  
✅ **Safety Procedures** - Emergency stop tested  

**Performance:**
- Mission execution: Stable
- GPS lock: < 30 seconds outdoor
- Mode transitions: Smooth
- RTL behavior: Reliable
- Network latency: Acceptable for telemetry

---

## Additional Resources

**FuryVision AAV Project:**
- Main Repository: https://github.com/sidharthmohannair/Fury-Drone-Project
- Version 2 Details: https://github.com/sidharthmohannair/Fury-Drone-Project/tree/main/versions/2_furyvision_aav
- Assembly Guide: [PDF](https://github.com/sidharthmohannair/Fury-Drone-Project/blob/main/versions/2_furyvision_aav/docs/Assembly%20of%20FuryVision%20AAV.pdf)
- Test Results: [Tests folder](https://github.com/sidharthmohannair/Fury-Drone-Project/tree/main/versions/2_furyvision_aav/tests)

**Related:**
- SPIN Lab, IIT Kanpur: Original Fury Drone design
- CDOH Lab, ICFOSS: FuryVision modifications
- Open Drone Team: Development and testing

---

## Using Different Hardware?

**This framework is hardware-agnostic** but validated on FuryVision AAV.

**If you use different hardware:**
1. Adjust serial port in launch files (`/dev/ttyACM0` may vary)
2. Modify baud rate if needed (921600 is standard)
3. Update network IPs in scripts
4. Tune PID parameters for your frame
5. Test thoroughly in SITL first!

**Community contributions welcome** - share your hardware configurations!

---

**Author:** Sidharth Mohan Nair  
**Test Platform:** FuryVision AAV  
**Repository:** https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware
