# ROVO Autonomous Offroad Robot

This repository contains the software for the autonomous offroad robot (ROVO), based on ROS 2. The software enables precise navigation and path tracking (trajectory control) of the robot using RTK-GPS and an IMU.

## Web-based User Interface (Web UI)

A central, web-based dashboard is available for the convenient monitoring and control of the ROVO robot. The communication between the web application and the ROS 2 network is asynchronous via the `rosbridge_suite` package using WebSockets, so no ROS knowledge or installation is required on the client device.

The interface, based on HTML5, JavaScript, Leaflet, and `roslib.js`, is divided into three main areas:

1. **Interactive Map View:** 
   A map based on OpenStreetMap visualizes the robot's current position, orientation, and traveled trajectory in real-time.
   
2. **Telemetry and Status Display:**
   Continuous display of all critical system states:
   - **RTK-GPS (ZED-F9P):** Coordinates and connection status (e.g., RTK Fix).
   - **IMU (BNO085):** Orientation, rotational speed, acceleration, and calibration status.
   - **Local Odometry:** Position and orientation relative to the starting point.
   - **Speed:** Current linear and rotational speed values.
   - **ROS Logs:** Direct display of system logs (`/rosout`) for error diagnosis in the browser.

3. **Control Panel:**
   - **Route Planning:** Waypoints can be clicked directly onto the map in an interactive drawing mode. The robot calculates the path and follows it upon confirmation. An emergency stop button clears the path and stops the robot immediately.
   - **Operating Mode:** Switching between three states:
     - *MANUAL:* Autonomous driving commands are deactivated; control only via physical remote control.
     - *AUTOMATIC:* Autonomous path tracking active.
     - *AUTO WATCHDOG:* Like AUTOMATIC, but with a heartbeat signal (2 Hz). If the browser connection drops, the robot stops automatically and switches to MANUAL mode.

## Starting the Application

The main software runs on a Raspberry Pi, which is operated via SSH from the control PC.

### 1. Establish Remote Connection
Make sure the control PC and Raspberry Pi are on the same network and open a terminal. Execute the following command:
```bash
ssh rovo@rovo.local
```
*(Alternatively, via a direct Ethernet connection: `ssh rovo@192.168.0.153`)*

After entering the password, you are connected to the Raspberry Pi.

### 2. Load Workspace and Start System
Switch to the main workspace in the SSH session and source the ROS 2 environment:

```bash
cd ~/rovo_ws
source install/setup.bash
```

Start the entire software architecture (sensors, drivers, controllers, and web server) centrally via the main launch file:

```bash
ros2 launch bringup main_bringup.launch.py
```

### 3. Operation
After the successful initialization of the system, the robot is fully ready for use. Open the following address in the web browser on your control PC to access the Web UI:

**[http://rovo.local:8000/](http://rovo.local:8000/)**
