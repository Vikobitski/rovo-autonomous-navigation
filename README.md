# Rovo Workspace
Information collected from `README.txt` and formatted as Markdown.

## Sourcing
```bash
source ~/rovo_ws/install/setup.bash
# Global ROS sourcing
source /opt/ros/jazzy/setup.bash
```

## Launching
```bash
source install/setup.bash
ros2 launch bringup main_bringup.launch.py
```
Or launch specific package:
```bash
ros2 launch rovo_launch rovo_launch.py
```
Or bringup:
```bash
ros2 launch rovo_bringup rovo.launch.py
```

## Building
Build specific package:
```bash
colcon build --packages-select <DEIN_PAKETNAME>
```
Build entire workspace:
```bash
colcon build
```

**Clean Re-Build:**
```bash
cd ~/rovo_ws
rm -rf build install log
# CLOSE TERMINAL and open new one
cd ~/rovo_ws
colcon build
source install/setup.bash
```

## Creating New Packages
[ROS2 Tutorial: Creating Your First ROS2 Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Create Python package:
```bash
# --build-type ament_python tells ROS this is a Python package
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

## How-Tos
**Add Launch Files to Packages:** [Launch System Tutorial](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html)

**RTK Setup Guide:** [Ardusimple Integration Guide](https://www.ardusimple.com/how-to-integrate-u-blox-zed-f9p-gnss-rtk-receiver-into-ros-2-jazzy/)

### Checking RTK Fix Status
*   Monitor `/ublox_gps_node/fix`: Check `status.status` field.
*   Monitor `/diagnostics`: Often provides a readable summary like "3D fix".
*   Use `/ublox/navpvt`:
    *   `fix_type`: Should be 3 for 3D fix.
    *   `flags`: Should indicate corrections usage.
    *   `flags2`: RTK status (67 = RTK Float, 128 = RTK Fix). Lower values likely mean no corrections.

## WiFi Configuration
List networks:
```bash
nmcli dev wifi list
```
Connect:
```bash
sudo nmcli dev wifi connect "SSID" password "PASSWORT"
```

## Git
```bash
git add .
git commit -m "..."
```

## Python
Global installation (use with caution):
```bash
--break-system-packages
```

## Useful Commands
List all ROS topics:
```bash
ros2 topic list
```
Run specific node:
```bash
ros2 run rovo_hardware imu_node
```
