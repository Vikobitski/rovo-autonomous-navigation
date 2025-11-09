Sourcen:
source ~/rovo_ws/install/setup.bash

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