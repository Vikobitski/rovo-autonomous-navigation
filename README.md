# ROVO Autonomous Offroad Robot

This repository contains the software for the autonomous offroad robot based on the ["ROVO 3 Heavy Duty" platform by HAWE Hydraulik](https://www.mattro.com/products#rovo). The system is a fully electric, differential-drive tracked vehicle specifically designed for demanding off-road environments such as vineyards. 

Developed as part of a master's project, this ROS 2-based software system enables precise and reproducible trajectory tracking in open fields. By fusing high-precision RTK GPS data with an IMU, the navigation stack allows the robot to reliably follow user-defined paths. This autonomous mobility serves as the foundational platform for automating agricultural processes, such as the adaptive spraying of vines.

## Web-based User Interface (Web UI)

A central, web-based dashboard is available for convenient monitoring and control of the ROVO robot. Communication between the web application and the ROS 2 network occurs asynchronously via the `rosbridge_suite` package using WebSockets. Therefore, no ROS knowledge or installations are required on the client device.

Based on HTML5, JavaScript, Leaflet, and `roslib.js`, the interface is divided into three main areas:

1. **Interactive Map View:** 
   An OpenStreetMap-based map visualizes the robot's current position, orientation, and tracked trajectory in real-time.
   
2. **Telemetry and Status Display:**
   Continuous display of all critical system states:
   - **RTK GPS (ZED-F9P):** Coordinates and connection status (e.g., RTK Fix).
   - **IMU (BNO085):** Orientation, angular velocity, acceleration, and calibration status.
   - **Local Odometry:** Position and orientation relative to the starting point.
   - **Velocity:** Current linear and rotational speeds.
   - **ROS Logs:** Direct display of system logs (`/rosout`) for troubleshooting directly in the browser.

3. **Control Panel:**
   - **Route Planning:** Entering the interactive drawing mode allows you to click waypoints directly on the map. The robot calculates the path and follows it upon confirmation. An emergency stop button clears the path and halts the robot immediately.
   - **Operation Mode:** Switch between three states:
     - *MANUAL:* Autonomous drive commands are disabled; control only via the physical radio remote.
     - *AUTOMATIC:* Autonomous trajectory tracking is active.
     - *AUTO WATCHDOG:* Same as AUTOMATIC, but includes a heartbeat signal (2 Hz). If the browser connection drops, the robot automatically stops and reverts to MANUAL mode.

## Starting the Application

The core software runs on a Raspberry Pi, which is operated via SSH from the control PC.

### 1. Establishing a Remote Connection
Ensure that the control PC and the Raspberry Pi are on the same network and open a terminal. Execute the following command:
```bash
ssh rovo@rovo.local
```
*(Alternatively, via a direct Ethernet connection: `ssh rovo@192.168.0.153`)*

After entering the password, you are connected to the Raspberry Pi.

### 2. Loading the Workspace and Starting the System
Navigate to the main workspace in your SSH session and source the ROS 2 environment:

```bash
cd ~/rovo_ws
source install/setup.bash
```

Launch the entire software architecture (sensors, drivers, controllers, and the web server) centrally via the main launch file:

```bash
ros2 launch bringup main_bringup.launch.py
```

### 3. Operation
After successful initialization of the system, the robot is fully operational. Open the following address in the web browser on your control PC to access the Web UI:

**[http://rovo.local:8000/](http://rovo.local:8000/)**

### 4. Defining and Executing a Trajectory
To make the robot drive along a desired route, proceed as follows in the Web UI:

1. **Enable Drawing Mode:** Activate the route planning mode in the right control panel.
2. **Set Waypoints:** Click consecutively on the desired target coordinates on the interactive map. The system visually connects these points into a path.
3. **Send Route to Robot:** Confirm the drawn route. The path is then transmitted to the robot's control system.
4. **Start Driving:** Change the operation mode in the control panel to **AUTOMATIC** (or **AUTO WATCHDOG**). The robot immediately starts driving the trajectory autonomously.
5. **Abort Drive:** To stop the robot, either click the stop button (clears the active route) or switch the operation mode back to **MANUAL**.
