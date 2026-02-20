import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from .geo_filter import GeoFilter
from .pid_nav import PIDController

class Pi5Navigator(Node):
    def __init__(self):
        super().__init__('pi5_navigator')

        # 1. PID PARAMS (Tune here according to your needs)
        params = {
            'Kp': 1.5, 'Ki': 0.0, 'Kd': 0.6, 'I_max': 0.2,
            'v_des': 0.15, 'v_max': 0.22, 'lookahead_dist': 0.25,
            'dist_thresh': 0.03, 'w_sat': 1.5,
            'accel_lin_max': 0.1, 'accel_ang_max': 1.0
        }

        # 2. PATH (The 2m x 2m Square)
        # Note: Adjust the corners if your robot's starting position is different
        corners = np.array([[0,0], [2,0], [2,2], [0,2], [0,0]])
        self.path = []
        for i in range(4):
            seg = np.linspace(corners[i], corners[i+1], 200)
            self.path.extend(seg[:-1])
        self.path = np.array(self.path)

        # 3. COMPONENTS
        # Note: Ensure these match your actual hardware specs for accurate state estimation and control
        self.filter = GeoFilter(r_wheel=0.175, l_base=0.777) # Adjusted for ROVO dimensions
        self.nav = PIDController(params)

        # 4. STATE VARIABLES
        self.lat, self.lon = 0.0, 0.0
        self.w_l, self.w_r = 0.0, 0.0 # From real encoders

        # 5. PUBS/SUBS
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Pi 5 Navigator Active. Path: 2m Square.")

    def gps_cb(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude

    def odom_cb(self, msg):
        # NOTE: On real hardware, these would be the actual rad/s from your encoders
        self.w_l = msg.twist.twist.linear.x
        self.w_r = msg.twist.twist.linear.y

    def control_loop(self):
        if self.lat == 0.0: return # Wait for GPS fix

        # A. State Estimation
        x_est, y_est, yaw_est = self.filter.update(self.lat, self.lon, self.w_l, self.w_r, 0.1, 0.6)

        # B. Compute Control
        v_cmd, w_cmd = self.nav.compute([x_est, y_est, yaw_est], self.path, 0.1)

        # C. Publish
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

        # D. Hardware Telemetry
        self.get_logger().info(f"Target Corner Check: Dist={self.nav.dist_to_target:.2f}m", once=False)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Pi5Navigator())
    rclpy.shutdown()