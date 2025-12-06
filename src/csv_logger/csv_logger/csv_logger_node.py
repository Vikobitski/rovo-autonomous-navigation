import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/home/rovo/rovo_ws/csv_logs')
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
        self.declare_parameter('log_gps', True)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.log_gps = self.get_parameter('log_gps').value
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.gps_csv_path = os.path.join(self.output_dir, f'gps_log_{timestamp}.csv')
        
        # Initialize CSV file with headers
        if self.log_gps:
            self._init_gps_csv()
        
        # Subscriptions
        # QoSProfile requires history and/or depth — set KEEP_LAST with depth 10
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        
        if self.log_gps:
            self.gps_sub = self.create_subscription(
                NavSatFix, self.gps_topic, self._gps_callback, qos_profile
            )
        
        self.get_logger().info('CSV Logger started')
        if self.log_gps:
            self.get_logger().info(f'GPS logging to: {self.gps_csv_path}')
    
    def _init_gps_csv(self):
        """Initialize GPS CSV file with headers"""
        with open(self.gps_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'seconds', 'latitude', 'longitude', 
                'altitude', 'position_covariance_type', 'status'
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


def main(args=None):
    rclpy.init(args=args)
    node = CSVLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
