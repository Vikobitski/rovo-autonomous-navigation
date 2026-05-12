import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from .geo_filter import GeoFilter
from .pid_nav import PIDController

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # 1. Config & Parameters (Spyder Section 1)
        params = {
            'Kp': 1.5, 'Ki': 0.0, 'Kd': 0.6, 'I_max': 0.2,
            'v_des': 0.15, 'v_max': 0.22, 'lookahead_dist': 0.25,
            'dist_thresh': 0.03, 'w_sat': 1.5,
            'accel_lin_max': 0.1, 'accel_ang_max': 1.0
        }

        # 2. Path Generation (Spyder Section 2)
        corners = np.array([[0,0], [2,0], [2,2], [0,2], [0,0]])
        self.path = []
        for i in range(4):
            seg = np.linspace(corners[i], corners[i+1], 200)
            self.path.extend(seg[:-1])
        self.path = np.array(self.path)

        # 3. Initialization
        self.filter = GeoFilter(r_wheel=0.033, l_base=0.160)
        self.nav = PIDController(params)
        
        self.lat, self.lon = 0.0, 0.0
        self.w_l, self.w_r = 0.0, 0.0
        self.sim_time = 0.0

        # ROS Comm
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def gps_cb(self, msg): self.lat, self.lon = msg.latitude, msg.longitude
    def odom_cb(self, msg): self.w_l, self.w_r = msg.twist.twist.linear.x, msg.twist.twist.linear.y

    def control_loop(self):
        if self.lat == 0.0: return

        # State Estimation (Spyder Section C)
        x_est, y_est, yaw_est = self.filter.update(self.lat, self.lon, self.w_l, self.w_r, 0.1, 0.6)

        # Compute Control (Spyder Section D)
        v_cmd, w_cmd = self.nav.compute([x_est, y_est, yaw_est], self.path, 0.1)

        # Send to Hardware (Spyder Section E)
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

        # Telemetry (Spyder Section F)
        self.sim_time += 0.1
        if self.sim_time > 10.0 and int(self.sim_time * 10) % 20 == 0:
            self.get_logger().info(f"Time: {self.sim_time:.1f}s | Pose: [{x_est:.2f}, {y_est:.2f}] | V: {v_cmd:.2f}")

        if self.nav.finished:
            self.get_logger().info("MISSION COMPLETE!")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavigationNode())
    rclpy.shutdown()