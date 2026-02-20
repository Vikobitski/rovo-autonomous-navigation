import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import lgpio # Standard for Pi 5

class Pi5MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # --- 1. GPIO SETUP (Adjust pins to your wiring) ---
        self.h = lgpio.gpiochip_open(4) # GPIO chip 4 is standard on Pi 5
        self.L_PWM, self.R_PWM = 12, 13 # PWM Pins
        self.L_DIR, self.R_DIR = 20, 21 # Direction Pins
        
        for pin in [self.L_PWM, self.R_PWM, self.L_DIR, self.R_DIR]:
            lgpio.gpio_claim_output(self.h, pin)

        # --- 2. ROS SETUP ---
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # Optional: Publisher for wheel encoders if you have them
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        
        self.get_logger().info("Pi 5 Motor Driver Initialized.")

    def listener_callback(self, msg):
        # Differential Drive Kinematics
        v = msg.linear.x
        w = msg.angular.z
        l_base = 0.777# Changes for ROVO dimensions
        
        # Convert to wheel velocities
        v_l = v - (w * l_base / 2.0)
        v_r = v + (w * l_base / 2.0)

        # Execute on Hardware
        self.set_motor(self.L_PWM, self.L_DIR, v_l)
        self.set_motor(self.R_PWM, self.R_DIR, v_r)

    def set_motor(self, pwm_pin, dir_pin, velocity):
        # 1. Handle Direction
        direction = 1 if velocity >= 0 else 0
        lgpio.gpio_write(self.h, dir_pin, direction)
        
        # 2. Handle PWM (Scale velocity to 0-100 duty cycle)
        duty = min(abs(velocity) * 500, 100) # Adjust multiplier for your motors
        lgpio.tx_pwm(self.h, pwm_pin, 800, duty) # 800Hz frequency

def main(args=None):
    rclpy.init(args=args)
    node = Pi5MotorDriver()
    rclpy.spin(node)
    # Cleanup on exit
    lgpio.gpiochip_close(node.h)
    rclpy.shutdown()