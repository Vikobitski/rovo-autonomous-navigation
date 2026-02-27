import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from .pid_nav import PIDController

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        # 1. PID PARAMS (Tune here according to your needs)
        # params = {
        #     'Kp': 1.5, 'Ki': 0.0, 'Kd': 0.6, 'I_max': 0.2,
        #     'v_des': 0.15, 'v_max': 0.3, 'lookahead_dist': 0.10,
        #     'dist_thresh': 0.03, 'w_sat': 0.1, #1.5,
        #     'accel_lin_max': 0.1, 'accel_ang_max': 1.0
        # }

        params = {
            'Kp': 0.35, 'Ki': 0.0, 'Kd': 0.0, 'I_max': 0.2,
            'v_des': 0.15, 'v_max': 0.3, 'lookahead_dist': 0.5,
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
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Path, '/set_trajectory', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rovo/set_speed', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Trajectory Follower Active. Waiting for /set_trajectory and /odometry/filtered.")

    def odom_cb(self, msg):
        self.x_est = msg.pose.pose.position.x
        self.y_est = msg.pose.pose.position.y

        # var siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        # var cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

        # var yawRad = Math.atan2(siny_cosp, cosy_cosp);
        
        # Quaternion to Yaw calculation
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_est_off = np.arctan2(siny_cosp, cosy_cosp)

        self.yaw_est = yaw_est_off 

        
        self.odom_received = True

    def path_cb(self, msg):
        # Extract points from Path message
        raw_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            raw_points.append([x, y])
        
        path_points = []
        if len(raw_points) > 1:
            for i in range(len(raw_points) - 1):
                p1 = np.array(raw_points[i])
                p2 = np.array(raw_points[i+1])
                # Insert 200 points between p1 and p2 (total 202 points per segment, excluding last to avoid duplicates)
                segment = np.linspace(p1, p2, 202)[:-1]
                path_points.extend(segment)
            path_points.append(raw_points[-1])
        elif len(raw_points) == 1:
            path_points = raw_points
        
        if len(path_points) > 0:
            self.path = np.array(path_points)
            self.path_received = True
            
            # Reset controller state for new path
            self.nav.finished = False 
            self.nav.last_closest_idx = 0
            self.get_logger().info(f"Received new path with {len(self.path)} interpolated points (from {len(raw_points)} original).")

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
    rclpy.spin(TrajectoryFollower())
    rclpy.shutdown()