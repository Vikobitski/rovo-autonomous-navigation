SOURCEN:
source ~/rovo_ws/install/setup.bash

LAUNCHEN
source install/setup.bash
ros2 launch bringup main_bringup.launch.py

BAUEN 
colcon build --packages-select <DEIN_PAKETNAME>
oder 
colcon build

ECHO

ros2 topic list

Neues Package (Ordner) erstellen:
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html


# --build-type ament_python sagt ROS, dass dies ein Python-Paket ist
ros2 pkg create --build-type ament_python --node-name my_node my_package


Sourcen
source /opt/ros/jazzy/setup.bash
source ~/rovo_ws/install/setup.bash


Bauen & Testen

cd ~/rovo_ws
colcon build
source install/setup.bash
ros2 run rovo_hardware imu_node


GIT
git add . und git commit -m "..." speichern.

Python 

Global installieren:
--break-system-packages


ROS2 HowTo:
Wie man Launch Files in packages einfügt: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html

launchen 
ros2 launch rovo_launch rovo_launch.py



Clean von neu Bauen

cd ~/rovo_ws
rm -rf build install log

-->Terminal SCHLIESSEN-->

cd ~/rovo_ws
colcon build
source install/setup.bash
ros2 launch rovo_bringup rovo.launch.py


WLan verbindung ändern:

rovo@rovo:~/rovo_ws/src$ nmcli dev wifi list

rovo@rovo:~/rovo_ws/src$ sudo nmcli dev wifi connect "SSID" password "PASSWORT"


GUTES HOW TO FÜR RTK SETUP: https://www.ardusimple.com/how-to-integrate-u-blox-zed-f9p-gnss-rtk-receiver-into-ros-2-jazzy/

Key tips for checking fix status:
Monitor /ublox_gps_node/fix in NavSatFix messages (status.status field) or diagnostic messages for more detailed text-based statuses.
The /diagnostics topic often provides a readable summary like “3D fix”.
Use /ublox/navpvt to get detailed information:
The fix_type field should be 3 for 3D fix.
The flags field should indicate that corrections are being used.
The flags2 field includes the RTK status: 67 RTK Float, 128 RTK Fix. If the value is significantly lower, it likely means no RTK corrections are being applied.