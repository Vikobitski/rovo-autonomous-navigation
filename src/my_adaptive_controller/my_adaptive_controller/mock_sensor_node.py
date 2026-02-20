import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class MockSensorNode(Node):
    def __init__(self):
        super().__init__('mock_sensor_node')
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Config from Spyder
        self.ts = 0.1
        self.r_earth = 6371000.0
        self.lat0, self.lon0 = 52.5200, 13.4050
        self.gps_noise_std = 0.03  # 3cm noise
        self.l_base = 0.160
        self.r_wheel = 0.033

        self.true_pose = np.array([0.0, 0.0, 0.0])
        self.v_act, self.w_act = 0.0, 0.0
        
        self.timer = self.create_timer(self.ts, self.simulation_step)
        self.get_logger().info("Mock Sensor with 3cm Noise Started.")

    def cmd_callback(self, msg):
        self.v_act = msg.linear.x
        self.w_act = msg.angular.z

    def simulation_step(self):
        # Physics Update (Spyder Section A)
        self.true_pose[0] += self.v_act * np.cos(self.true_pose[2]) * self.ts
        self.true_pose[1] += self.v_act * np.sin(self.true_pose[2]) * self.ts
        self.true_pose[2] += self.w_act * self.ts

        # Generate Noisy Coordinates (Spyder Section B)
        mx = self.true_pose[0] + self.gps_noise_std * np.random.randn()
        my = self.true_pose[1] + self.gps_noise_std * np.random.randn()

        # Publish GPS
        gps_msg = NavSatFix()
        gps_msg.latitude = self.lat0 + np.degrees(my / self.r_earth)
        gps_msg.longitude = self.lon0 + np.degrees(mx / (self.r_earth * np.cos(np.radians(self.lat0))))
        self.gps_pub.publish(gps_msg)

        # Publish Wheel Speeds via Odom (for the GeoFilter)
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = float(self.true_pose[0]) # For Ground Truth tracking
        odom_msg.pose.pose.position.y = float(self.true_pose[1])
        
        # Calculate wheel speeds (Spyder Section B)
        w_l = (self.v_act - self.w_act * self.l_base / 2.0) / self.r_wheel
        w_r = (self.v_act + self.w_act * self.l_base / 2.0) / self.r_wheel
        
        odom_msg.twist.twist.linear.x = float(w_l) # Hijacking x/y for wheel speeds
        odom_msg.twist.twist.linear.y = float(w_r)
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MockSensorNode())
    rclpy.shutdown()