import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
import csv
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/home/rovo/rovo_ws/logs')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('log_gps', True)
        self.declare_parameter('log_pose', True)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.log_gps = self.get_parameter('log_gps').value
        self.log_pose = self.get_parameter('log_pose').value
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.gps_csv_path = os.path.join(self.output_dir, f'gps_log_{timestamp}.csv')
        self.pose_csv_path = os.path.join(self.output_dir, f'pose_log_{timestamp}.csv')
        
        # Initialize CSV files with headers
        if self.log_gps:
            self._init_gps_csv()
        if self.log_pose:
            self._init_pose_csv()
        
        # Subscriptions
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
        
        if self.log_gps:
            self.gps_sub = self.create_subscription(
                NavSatFix, self.gps_topic, self._gps_callback, qos_profile
            )
        
        if self.log_pose:
            self.pose_sub = self.create_subscription(
                Pose, self.pose_topic, self._pose_callback, qos_profile
            )
        
        self.get_logger().info(f'CSV Logger started')
        if self.log_gps:
            self.get_logger().info(f'GPS logging to: {self.gps_csv_path}')
        if self.log_pose:
            self.get_logger().info(f'Pose logging to: {self.pose_csv_path}')
    
    def _init_gps_csv(self):
        """Initialize GPS CSV file with headers"""
        with open(self.gps_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'seconds', 'latitude', 'longitude', 
                'altitude', 'position_covariance_type', 'status'
            ])
    
    def _init_pose_csv(self):
        """Initialize Pose CSV file with headers"""
        with open(self.pose_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'seconds', 'x', 'y', 'z',
                'qx', 'qy', 'qz', 'qw'
            ])
    
    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS NavSatFix messages"""
        try:
            timestamp = datetime.now().isoformat()
            seconds = self.get_clock().now().nanoseconds / 1e9
            
            with open(self.gps_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    f'{seconds:.3f}',
                    f'{msg.latitude:.8f}',
                    f'{msg.longitude:.8f}',
                    f'{msg.altitude:.3f}',
                    msg.position_covariance_type,
                    msg.status.status
                ])
        except Exception as e:
            self.get_logger().error(f'Error writing GPS data: {e}')
    
    def _pose_callback(self, msg: Pose):
        """Handle Pose messages"""
        try:
            timestamp = datetime.now().isoformat()
            seconds = self.get_clock().now().nanoseconds / 1e9
            
            with open(self.pose_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    f'{seconds:.3f}',
                    f'{msg.position.x:.6f}',
                    f'{msg.position.y:.6f}',
                    f'{msg.position.z:.6f}',
                    f'{msg.orientation.x:.6f}',
                    f'{msg.orientation.y:.6f}',
                    f'{msg.orientation.z:.6f}',
                    f'{msg.orientation.w:.6f}'
                ])
        except Exception as e:
            self.get_logger().error(f'Error writing Pose data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CSVLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
