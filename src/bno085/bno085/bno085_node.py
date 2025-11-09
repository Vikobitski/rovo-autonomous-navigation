import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

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
    
    Veröffentlicht außerdem den Kalibrierungsstatus auf /diagnostics.
    """
    def __init__(self):
        super().__init__('bno085_node')

        # --- Parameter ---
        self.declare_parameter('frame_id', 'imu_link')
        self.frame_id = self.get_parameter('frame_id').value
        
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
        self.diag_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
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

            msg = Imu()
            msg.header.stamp = current_time
            msg.header.frame_id = self.frame_id

            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]

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
            sys_cal = self.bno.calibration_status 
            
            diag_array = DiagnosticArray()
            diag_array.header.stamp = current_time

            status_msg = DiagnosticStatus()
            status_msg.name = f"{self.get_name()}: Calibration Status" 
            status_msg.hardware_id = self.frame_id

            # Level 0-2 sind WARN, Level 3 ist OK
            if sys_cal < 3:
                status_msg.level = DiagnosticStatus.WARN 
                status_msg.message = f"IMU is not fully calibrated (System Status: {sys_cal})"
            else:
                status_msg.level = DiagnosticStatus.OK
                status_msg.message = f"IMU is fully calibrated (System Status: {sys_cal})"

            # Füge nur den System-Kalibrierungswert hinzu
            status_msg.values.append(KeyValue(key="System Calibration", value=str(sys_cal)))

            diag_array.status.append(status_msg)
            self.diag_publisher.publish(diag_array)


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
