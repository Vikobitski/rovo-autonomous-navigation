import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import math
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
            # Subscribe to IMU data
            self.imu_sub = self.create_subscription(
                Imu, '/imu/data', self._imu_callback, 10
            )
            self.latest_imu_msg = None
            
            # Subscribe to Calibration Status
            self.calib_sub = self.create_subscription(
                Int8, '/imu/calibration_status', self._calib_callback, 10
            )
            self.latest_calib_msg = None

            # Subscribe to Global Odometry
            self.odom_sub = self.create_subscription(
                Odometry, '/odometry/filtered', self._odom_callback, 10
            )
            self.latest_odom_msg = None
        
        self.get_logger().info('CSV Logger started')
        if self.log_gps:
            self.get_logger().info(f'GPS logging to: {self.gps_csv_path}')
    
    def _init_gps_csv(self):
        """Initialize GPS CSV file with headers"""
        with open(self.gps_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'seconds', 'latitude', 'longitude', 
                'altitude', 'position_covariance_type', 'RTK-status',
                'BNO085-absolute-heading','BNO085-quaternion' , 'BNO085-angular-velocity', 'BNO085-linear-acceleration',
                'BNO085-calibration-status',
                'odom_x', 'odom_y', 'odom_heading', 'odom_quat_w', 'odom_quat_x', 'odom_quat_y', 'odom_quat_z'
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
            
            # Prepare BNO085 data
            bno_heading = ""
            bno_quat = ""
            bno_ang_vel = ""
            bno_lin_acc = ""
            bno_calib = ""
            
            if self.latest_imu_msg:
                # Calculate heading (Yaw)
                q = self.latest_imu_msg.orientation
                yaw = self._get_yaw_from_quaternion(q)
                bno_heading = f"{yaw:.4f}"
                bno_quat = f"[{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]"
                
                av = self.latest_imu_msg.angular_velocity
                bno_ang_vel = f"[{av.x:.4f}, {av.y:.4f}, {av.z:.4f}]"
                
                la = self.latest_imu_msg.linear_acceleration
                bno_lin_acc = f"[{la.x:.4f}, {la.y:.4f}, {la.z:.4f}]"

            if self.latest_calib_msg:
                bno_calib = str(self.latest_calib_msg.data)

            # Prepare Odometry data
            odom_x = ""
            odom_y = ""
            odom_heading = ""
            odom_qw = ""
            odom_qx = ""
            odom_qy = ""
            odom_qz = ""

            if self.latest_odom_msg:
                pos = self.latest_odom_msg.pose.pose.position
                odom_x = f"{pos.x:.4f}"
                odom_y = f"{pos.y:.4f}"
                
                ori = self.latest_odom_msg.pose.pose.orientation
                odom_qw = f"{ori.w:.4f}"
                odom_qx = f"{ori.x:.4f}"
                odom_qy = f"{ori.y:.4f}"
                odom_qz = f"{ori.z:.4f}"
                
                # reusing _get_yaw_from_quaternion for heading in degrees
                heading_deg = self._get_yaw_from_quaternion(ori)
                odom_heading = f"{heading_deg:.4f}"


            with open(self.gps_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    f'{seconds:.3f}',
                    f'{msg.latitude:.8f}',
                    f'{msg.longitude:.8f}',
                    f'{msg.altitude:.3f}',
                    msg.position_covariance_type,
                    msg.status.status,
                    bno_heading,
                    bno_quat,
                    bno_ang_vel,
                    bno_lin_acc,
                    bno_calib,
                    odom_x,
                    odom_y,
                    odom_heading,
                    odom_qw,
                    odom_qx,
                    odom_qy,
                    odom_qz
                ])
            # increment line counter
            self.current_lines += 1
        except Exception as e:
            self.get_logger().error(f'Error writing GPS data: {e}')

    def _imu_callback(self, msg: Imu):
        """Store the latest IMU message"""
        self.latest_imu_msg = msg

    def _calib_callback(self, msg: Int8):
        """Store the latest calibration status"""
        self.latest_calib_msg = msg

    def _odom_callback(self, msg: Odometry):
        """Store the latest odometry message"""
        self.latest_odom_msg = msg

    def _get_yaw_from_quaternion(self, q):
        """
        Convert quaternion (x, y, z, w) to heading in degrees (0-360).
        """
        # standard conversion from quaternion to euler roll, pitch, yaw
        t0 = +2.0 * (q.w * q.z + q.x * q.y)
        t1 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(t0, t1)
        
        yaw_deg = math.degrees(yaw_rad)
        
        # Calculate display heading based on JS logic
        # let displayHeading = (90 - yawDeg);
        display_heading = 90.0 - yaw_deg
        
        # displayHeading = (displayHeading + 360) % 360;
        display_heading = (display_heading + 360.0) % 360.0
        
        return display_heading

def main(args=None):
    rclpy.init(args=args)
    node = CSVLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
