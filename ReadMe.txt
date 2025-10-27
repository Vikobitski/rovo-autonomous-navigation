
Neues Package (Ordner) erstellen:

# --build-type ament_python sagt ROS, dass dies ein Python-Paket ist
ros2 pkg create rovo_hardware --build-type ament_python



Bauen & Testen

cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run rovo_hardware imu_node


GIT
git add . und git commit -m "..." speichern.

Python 

Global installieren:
--break-system-packages