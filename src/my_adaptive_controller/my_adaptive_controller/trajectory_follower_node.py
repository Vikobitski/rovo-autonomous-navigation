import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from .pid_nav import PIDController

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        # 1. PID PARAMS (Tune here according to your needs
        params = {
            'Kp': 1.5, 'Ki': 0.0, 'Kd': 0.0, 'I_max': 0.2,
            'v_des': 0.28, 'v_max': 0.3, 'lookahead_dist': 0.5,
            'dist_thresh': 0.25, 'w_sat': 0.5, 
            'accel_lin_max': 1, 'accel_ang_max': 3.14
        }

        # 2. PATH
        self.path = None

        # 3. COMPONENTS
        self.nav = PIDController(params, self.get_logger())

        # 4. STATE VARIABLES
        self.x_est, self.y_est, self.yaw_est = 0.0, 0.0, 0.0
        self.odom_received = False
        self.path_received = False

        # 5. PUBS/SUBS
        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Path, '/set_trajectory', self.path_cb, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/rovo/set_speed', 10)

        # 6. SRV CLIENTS
        self.from_ll_client = self.create_client(FromLL, '/fromLL', callback_group=self.cb_group)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Trajectory Follower Active. Waiting for /set_trajectory and /odometry/filtered.")

    def odom_cb(self, msg):
        self.x_est = msg.pose.pose.position.x
        self.y_est = msg.pose.pose.position.y
        
        # Quaternion to Yaw calculation
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_est_off = np.arctan2(siny_cosp, cosy_cosp)

        self.yaw_est = yaw_est_off 

        
        self.odom_received = True

    def path_cb(self, msg):
        import threading
        # Offload logic to a background thread so we don't block the executor
        threading.Thread(target=self._process_path_thread, args=(msg,), daemon=True).start()

    def _process_path_thread(self, msg):
        import time
        # We assume the UI sends Path where x = Latitude and y = Longitude
        gps_points = []
        for pose_stamped in msg.poses:
            lat = pose_stamped.pose.position.x
            lon = pose_stamped.pose.position.y
            gps_points.append((lat, lon))

        if len(gps_points) == 0:
            self.nav.finished = True
            #self.path_received = False
            self.get_logger().info("Empty path received, deactivating controller.")
            return

        self.get_logger().info(f"Received new GPS path with {len(gps_points)} points. Converting to map frame...")
        
        if not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /fromLL is not available. Cannot convert path!")
            return

        # Convert all points using the service
        converted_path = []
        for lat, lon in gps_points:
            req = FromLL.Request()
            req.ll_point.latitude = float(lat)
            req.ll_point.longitude = float(lon)
            req.ll_point.altitude = 0.0
            
            future = self.from_ll_client.call_async(req)
            # Wait for future without blocking the global ROS executor
            start_wait = time.time()
            while not future.done():
                if time.time() - start_wait > 2.0:
                    self.get_logger().error("Timeout waiting for /fromLL response")
                    break
                time.sleep(0.01)

            if future.done():
                try:
                    response = future.result()
                    if response is not None:
                        map_pt = response.map_point
                        converted_path.append([map_pt.x, map_pt.y])
                except Exception as e:
                    self.get_logger().warn(f"Failed to convert point {lat}, {lon}: {e}")
        
        if len(converted_path) > 0:
            # Interpolate points every 5mm (0.005 meters)
            interpolated_path = []
            interpolated_path.append(converted_path[0])

            for i in range(len(converted_path) - 1):
                p1 = np.array(converted_path[i])
                p2 = np.array(converted_path[i + 1])
                dist = np.linalg.norm(p2 - p1)

                step_size = 0.005  # 5 mm
                num_steps = int(np.ceil(dist / step_size))

                if num_steps == 0:
                    continue

                for j in range(1, num_steps + 1):
                    interp_pt = p1 + (p2 - p1) * (j / float(num_steps))
                    interpolated_path.append(interp_pt.tolist())

            self.path = np.array(interpolated_path)
            self.path_received = True
            
            # Reset controller state for new path
            self.nav.finished = False 
            self.nav.last_closest_idx = 0
            self.get_logger().info(f"Successfully converted, interpolated and loaded path with {len(self.path)} points.")
        else:
            self.nav.finished = True
            self.path_received = False
            self.get_logger().error("Path conversion failed completely.")

    def control_loop(self):
        if not self.odom_received or not self.path_received:
            return

        # B. Compute Control
        v_cmd, w_cmd = self.nav.compute([self.x_est, self.y_est, self.yaw_est], self.path, 0.1)

        # C. Publish
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

        # D. Hardware Telemetry
        if self.nav.finished:
            # We log once, but finished remains true preventing consecutive logs and movement in compute
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()