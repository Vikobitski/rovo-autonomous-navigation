import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import glob
import re
from datetime import datetime as dt


class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/home/rovo/rovo_ws/csv_logs')
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
        self.declare_parameter('log_gps', True)
        self.declare_parameter('max_files', 10)
        self.declare_parameter('max_lines_per_file', 10000)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.log_gps = self.get_parameter('log_gps').value
        self.max_files = int(self.get_parameter('max_files').value)
        self.max_lines_per_file = int(self.get_parameter('max_lines_per_file').value)
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file setup (handle rotation / existing files)
        self.gps_csv_path = None
        self.current_lines = 0
        if self.log_gps:
            self._scan_existing_logs_and_prepare()
        
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
        # header counts as one line
        self.current_lines = 1

    def _scan_existing_logs_and_prepare(self):
        """Scan output dir for existing gps logs, set current file or create new one.

        Ensures at most `max_files` exist and that the current file has fewer
        than `max_lines_per_file` lines. If the latest file is full, create a new one.
        """
        pattern = os.path.join(self.output_dir, 'gps_log_*.csv')
        files = glob.glob(pattern)
        # parse timestamp from filename gps_log_YYYYMMDD_HHMMSS.csv
        parsed = []
        regex = re.compile(r'gps_log_(\d{8}_\d{6})\.csv$')
        for f in files:
            m = regex.search(os.path.basename(f))
            if m:
                try:
                    ts = dt.strptime(m.group(1), '%Y%m%d_%H%M%S')
                    parsed.append((ts, f))
                except Exception:
                    continue
        parsed.sort()

        # enforce max files (remove oldest first)
        while len(parsed) > self.max_files:
            old_ts, old_file = parsed.pop(0)
            try:
                os.remove(old_file)
                self.get_logger().info(f'Removed old gps log: {old_file}')
            except Exception as e:
                self.get_logger().warn(f'Could not remove old log {old_file}: {e}')

        if parsed:
            # latest existing file
            latest_ts, latest_file = parsed[-1]
            # count lines
            try:
                with open(latest_file, 'r') as f:
                    line_count = sum(1 for _ in f)
            except Exception:
                line_count = 0

            if line_count >= self.max_lines_per_file:
                # create a new file
                self._open_new_gps_file()
            else:
                # append to existing latest file
                self.gps_csv_path = latest_file
                self.current_lines = line_count
        else:
            # no existing files -> create new
            self._open_new_gps_file()

        # ensure still within max_files after potential new file creation
        self._enforce_max_files()

    def _open_new_gps_file(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.gps_csv_path = os.path.join(self.output_dir, f'gps_log_{timestamp}.csv')
        self._init_gps_csv()

    def _enforce_max_files(self):
        pattern = os.path.join(self.output_dir, 'gps_log_*.csv')
        files = glob.glob(pattern)
        regex = re.compile(r'gps_log_(\d{8}_\d{6})\.csv$')
        parsed = []
        for f in files:
            m = regex.search(os.path.basename(f))
            if m:
                try:
                    ts = dt.strptime(m.group(1), '%Y%m%d_%H%M%S')
                    parsed.append((ts, f))
                except Exception:
                    continue
        parsed.sort()
        while len(parsed) > self.max_files:
            old_ts, old_file = parsed.pop(0)
            try:
                os.remove(old_file)
                self.get_logger().info(f'Removed old gps log: {old_file}')
            except Exception as e:
                self.get_logger().warn(f'Could not remove old log {old_file}: {e}')
    
    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS NavSatFix messages"""
        try:
            # rotate file if too many lines
            if self.current_lines >= self.max_lines_per_file:
                self._open_new_gps_file()
                self._enforce_max_files()

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
                    msg.statuubls.status
                ])
            # increment line counter
            self.current_lines += 1
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
