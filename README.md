# Rovo ROS 2 Workspace

This repository contains the ROS 2 based software stack for the mobile robot [ROVO from HAWE Mattro](https://www.hawe.com/products/robot-platform/). The workspace bundles all essential packages for hardware control, localization, autonomous navigation, and a web user interface for control and monitoring.

## Core Features

- **Hardware Interface (`rovo_driver`)**: CAN bus communication and a robust mode system (Manual, Automatic, Automatic Watchdog for safe operation).
- **Localization & Sensors**: positioning using RTK-GPS (u-blox ZED-F9P) and IMU data fusion (BNO085) via Robot Localization.
- **Trajectory Following & Autonomous Navigation (`trajectory_control`)**: Path planning and precise following of GPS waypoints in the local coordinate system.
- **Interactive Web UI**: A browser-based frontend featuring a live map (Leaflet), click-to-draw route creation, live telemetry visualization (current position, heading, operating mode), and remote control elements.

---

## Building
Build entire workspace:
```bash
colcon build
```

Build specific package:
```bash
colcon build --packages-select <PACKAGE_NAME>
```

## Launching
```bash
source install/setup.bash
ros2 launch bringup main_bringup.launch.py
```

## Useful Tutorials
**Creating new ROS2 packages:** [ROS2 Tutorial: Creating Your First ROS2 Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Example to create a new python package:
```bash
# --build-type ament_python tells ROS this is a Python package
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

**Add Launch Files to Packages:** [Launch System Tutorial](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html)

**RTK Setup Guide:** [Ardusimple Integration Guide](https://www.ardusimple.com/how-to-integrate-u-blox-zed-f9p-gnss-rtk-receiver-into-ros-2-jazzy/)
