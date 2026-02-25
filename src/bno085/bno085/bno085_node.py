import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

class Bno085Node(Node):
    """
    ROS 2 Wrapper für den BNO085 Sensor.
    Liest die fusionierte Quaternion (Rotation Vector), Beschleunigung
    und Gyroskop-Daten und veröffentlicht sie als sensor_msgs/Imu.
    
    Veröffentlicht außerdem den Kalibrierungsstatus auf /imu/calibration_status.
    """
    def __init__(self):
        super().__init__('bno085_node')

        # --- Parameter ---
        self.declare_parameter('frame_id', 'imu_link')
        self.frame_id = self.get_parameter('frame_id').value
        
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.yaw_offset_deg = self.get_parameter('yaw_offset_deg').value
        
        # --- Hardware-Initialisierung ---
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.bno = BNO08X_I2C(self.i2c)
        except Exception as e:
            self.get_logger().fatal(f"Konnte BNO085 nicht initialisieren: {e}")
            self.get_logger().fatal("Stelle sicher, dass I2C aktiviert ist und der Sensor verbunden ist.")
            rclpy.shutdown()
            return

        # --- Sensor-Features aktivieren ---
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        
        self.get_logger().info("BNO085 Sensor initialisiert und Features aktiviert.")

        # --- ROS Publisher & Timer ---
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.calib_publisher = self.create_publisher(Int8, '/imu/calibration_status', 10)
        
        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Liest Sensordaten UND Kalibrierungsstatus und veröffentlicht beides.
        """
        current_time = self.get_clock().now().to_msg()
        
        try:
            # --- 1. Imu-Daten lesen und veröffentlichen ---
            
            # .quaternion nutzt den ROTATION_VECTOR (absolut, mit Magnetometer)
            quat = self.bno.quaternion 
            accel = self.bno.acceleration
            gyro = self.bno.gyro

            if not all([quat, accel, gyro]):
                self.get_logger().warn("Einige Sensordaten sind 'None'. Überspringe Tick.")
                return

            # --- Yaw Offset für Konvertierung von Norden=0 auf Osten=0 ---
            yaw_offset_rad = math.radians(self.yaw_offset_deg)
            q_off_w = math.cos(yaw_offset_rad / 2.0)
            q_off_x = 0.0
            q_off_y = 0.0
            q_off_z = math.sin(yaw_offset_rad / 2.0)

            qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]

            # Quaternionen-Multiplikation für Z-Achsen-Rotation (q_offset * q_orig)
            new_w = q_off_w * qw - q_off_x * qx - q_off_y * qy - q_off_z * qz
            new_x = q_off_w * qx + q_off_x * qw + q_off_y * qz - q_off_z * qy
            new_y = q_off_w * qy - q_off_x * qz + q_off_y * qw + q_off_z * qx
            new_z = q_off_w * qz + q_off_x * qy - q_off_y * qx + q_off_z * qw

            msg = Imu()
            msg.header.stamp = current_time
            msg.header.frame_id = self.frame_id

            msg.orientation.x = float(new_x)
            msg.orientation.y = float(new_y)
            msg.orientation.z = float(new_z)
            msg.orientation.w = float(new_w)

            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            
            # Kovarianzen (Standardwerte)
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
            msg.angular_velocity_covariance[0] = 0.001
            msg.angular_velocity_covariance[4] = 0.001
            msg.angular_velocity_covariance[8] = 0.001
            msg.linear_acceleration_covariance[0] = 0.01
            msg.linear_acceleration_covariance[4] = 0.01
            msg.linear_acceleration_covariance[8] = 0.01

            self.imu_publisher.publish(msg)

            # --- 2. Kalibrierungsstatus lesen und veröffentlichen ---
            
            # .calibration_status gibt einen einzelnen Integer (0-3) zurück.
            cal_status = self.bno.calibration_status 

            calib_msg = Int8()
            calib_msg.data = cal_status
            self.calib_publisher.publish(calib_msg)


        except Exception as e:
            self.get_logger().warn(f"Fehler im Timer-Callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    bno085_node = Bno085Node()
    
    try:
        rclpy.spin(bno085_node)
    except KeyboardInterrupt:
        pass
    finally:
        bno085_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
