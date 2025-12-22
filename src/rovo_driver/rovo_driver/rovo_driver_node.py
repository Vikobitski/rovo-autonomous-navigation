#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
import can
import threading
import time
import subprocess
import os
import math 
from enum import Enum

class RovoMode(Enum):
    IDLE = 0
    JOYSTICK = 1
    AUTOMATIC = 2

class RovoDriverNode(Node):
    def __init__(self):
        super().__init__('rovo_driver_node')

        # Rovo Mechanical parameters
        self.wheelbase = 0.96 # [m] German: Radabstand
        self.gear_ratio = 16
        self.drive_diameter = 0.35 # [m]

        # Operating mode
        self.shutdown_requested = False
        self.mode = RovoMode.IDLE
        
        # CAN interface configuration
        self.can_interface = 'can0'
        self.can_bitrate = 500000
        self.shutdown_can_on_exit = False  # Set to True in production, False for testing
        self.can_started_by_us = False  # Track if we started the interface

        # Threading control
        self.running = True
        self.alive_counter = 0
        self.cyclic_messages = {}  # Dictionary for multiple messages
        self.cyclic_lock = threading.Lock()  # Lock for thread safety

        # Controller pre-allocs
        self.gear = 0 # virtual gear for joystick mode
        self.rb_pressed = False
        self.lb_pressed = False
        self.mag_x = 0 
        self.mag_y = 0
        self.left_rpm = 0
        self.right_rpm = 0
                
        # Publisher for rovo mode
        self.rovo_mode_publisher = self.create_publisher(
            Int32,
            '/rovo/mode',
            10
        )
        
        # Controller and ROS2 checks FIRST (before CAN setup)
        if not self.check_controller_and_joy():
            self.get_logger().error("\033[91mController or Joy node not available - terminating program\033[0m")
            return

        # # WIEDER EINKOMMENTIEREN, WENN REGELUNG IMPLEMENTIERT IST
        # if not self.check_trajectory_control():
        #     self.get_logger().error("\033[91mTrajectory control node not available - terminating program\033[0m")
        #     return
        
        # Setup CAN interface only after controller checks pass
        if not self.setup_can_interface():
            self.get_logger().error("\033[91mCAN interface setup failed!\033[0m")
            return
        
        # Initialize CAN interface
        try:
            self.can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
            self.get_logger().info("CAN interface initialized")
        except Exception as e:
            self.get_logger().error(f"CAN interface error: {e}")
            return
        
        # Start cyclic sender thread
        self.cyclic_thread = threading.Thread(target=self.cyclic_sender)
        self.cyclic_thread.daemon = True
        self.cyclic_thread.start()
        
        # Start initialization
        self.get_logger().info("ROVO Controller started - ready for controller inputs:")
        self.get_logger().info("  B = Joystick mode")
        self.get_logger().info("  Y = Automatic mode") 
        self.get_logger().info("  A = Idle mode")

        
        # Publisher for speed data
        self.speed_publisher = self.create_publisher(
            Twist,
            '/rovo/speed',
            10
        )


        # Start CAN Notifier with a listener for speed updates
        # This will call self.speed_monitor whenever a message arrives
        self.notifier = can.Notifier(self.can_bus, [self.speed_monitor])

    def check_controller_and_joy(self):
        """Check if controller exists and joy node is running"""
        # Check if controller device exists
        if not os.path.exists('/dev/input/js0'):
            self.get_logger().error("No joystick found at /dev/input/js0")
            return False
        else:
            self.get_logger().info("Xbox controller found")

        # Check if joy node is publishing
        self.joy_received = False
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Wait for Joy messages (max 7 seconds)
        self.get_logger().info("Waiting for Joy node...")
        start_time = time.time()
        timeout = 7.0

        while not self.joy_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks for 0.1s
            
        if not self.joy_received:
            self.get_logger().error(f"Joy node is not running")
            return False
        else:
            self.get_logger().info("Joy node is active")
            return True

    def check_trajectory_control(self):
        """Check if trajectory control is active"""
        self.trajectory_control_subscription = self.create_subscription(
            Twist,
            '/rovo/speed_setpoint',
            self.trajectory_control_callback,
            20
        )

        self.get_logger().info("Waiting for Trajectory control node...")
        start_time = time.time()
        timeout = 7.0
        
        while not self.speed_setpoint_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks for 0.1s

        if not self.speed_setpoint_received:
            self.get_logger().error("Trajectory control node is not running")
            return False
        else:
            self.get_logger().info("Trajectory control node is active")
            return True

    def setup_can_interface(self):
        """Setup CAN interface with automatic configuration"""
        try:
            # Check if interface already exists and is up
            result = subprocess.run(['ip', 'link', 'show', self.can_interface], 
                                  capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f"CAN interface {self.can_interface} does not exist!")
                return False
            
            # Check if interface is already UP
            if 'state UP' in result.stdout:
                self.get_logger().info(f"CAN interface {self.can_interface} is already active")
                return True
            
            # Interface exists but is DOWN - configure it
            self.get_logger().info(f"Configuring CAN interface {self.can_interface}...")
            
            # Set bitrate and bring up interface
            commands = [
                ['sudo', 'ip', 'link', 'set', self.can_interface, 'up', 'type', 'can', 'bitrate', str(self.can_bitrate)]
            ]
            
            for cmd in commands:
                result = subprocess.run(cmd, capture_output=True, text=True)
                if result.returncode != 0:
                    self.get_logger().error(f"Error executing: {' '.join(cmd)}")
                    self.get_logger().error(f"Error: {result.stderr}")
                    return False
            
            self.can_started_by_us = True
            self.get_logger().info(f"CAN interface {self.can_interface} successfully started (Bitrate: {self.can_bitrate})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during CAN interface setup: {e}")
            return False

    def shutdown_can_interface(self):
        """Shutdown CAN interface if we started it and shutdown is enabled"""
        if self.shutdown_can_on_exit and self.can_started_by_us:
            try:
                self.get_logger().info(f"Shutting down CAN interface {self.can_interface}...")
                result = subprocess.run(['sudo', 'ip', 'link', 'set', 'down', self.can_interface], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info(f"CAN interface {self.can_interface} successfully shut down")
                else:
                    self.get_logger().warning(f"Error during shutdown: {result.stderr}")
            except Exception as e:
                self.get_logger().error(f"Error shutting down CAN interface: {e}")
        elif self.can_started_by_us:
            self.get_logger().info(f"CAN interface {self.can_interface} remains active (shutdown_can_on_exit=False)")
        else:
            self.get_logger().info(f"CAN interface {self.can_interface} was not started by us - no changes")

    def set_cyclic_message(self, can_id, data):
        """Set a message to be sent cyclically every 10ms"""
        with self.cyclic_lock:
            self.cyclic_messages[can_id] = data.copy() if data else None
        
    def stop_cyclic_message(self, can_id=None):
        """Stop sending a specific cyclic message or all messages"""
        with self.cyclic_lock:
            if can_id is None:
                # Stop all
                self.cyclic_messages.clear()
            else:
                # Stop only a specific ID
                if can_id in self.cyclic_messages:
                    del self.cyclic_messages[can_id]

    def cyclic_sender(self):
        """Thread that sends all cyclic messages every 10ms"""
        while self.running:
            with self.cyclic_lock:
                for can_id, data in self.cyclic_messages.items():
                    if data is not None:
                        data_to_send = data.copy()
                        
                        # Special handling for alive counter on 0x28A
                        if can_id == 0x28A and len(data_to_send) >= 8:
                            data_to_send[7] = self.alive_counter
                            self.alive_counter = (self.alive_counter + 1) % 256
                        
                        # Send message
                        self.send_can_message(can_id, data_to_send)
                
            time.sleep(0.01)  # 10ms
    
    def send_can_message(self, can_id, data):
        """Send single CAN message"""
        try:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.can_bus.send(msg)
            # self.get_logger().info(f"Sending ID={hex(can_id)}: DLC={len(msg.data)}, DATA={msg.data}")
            return True
        except Exception as e:
            self.get_logger().error(f"CAN send error {hex(can_id)}: {e}")
            return False
    
    def receive_can_message(self, expected_id, timeout=1.0):
        """Receive and filter CAN messages"""
        try:
            msg = self.can_bus.recv(timeout=timeout)
            if msg and msg.arbitration_id == expected_id:
                return msg
        except Exception as e:
            self.get_logger().error(f"Error receiving CAN message: {e}")
        return None
    
    def wait_for_response(self, expected_id, expected_byte0, expected_byte2, timeout=5.0):
        """Wait for specific response with timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.receive_can_message(expected_id, 0.1)
            if msg and len(msg.data) >= 3:
                # self.get_logger().info(f"Received: ID=0x{msg.arbitration_id:X}, DATA={[hex(b) for b in msg.data]}")
                if msg.data[0] == expected_byte0 and msg.data[2] == expected_byte2:
                    return True
            time.sleep(0.01)
        return False
    
    def activate_external_control(self):
        """Complete ROVO initialization"""
        self.get_logger().info("Start activation of external control...")
        
        try:
            # Step 0 - Start alive counter with 0x28A
            # self.get_logger().info("Step 0, start alive counter on 28A")
            self.set_cyclic_message(0x28A, [0] * 8)  # Will be updated in cyclic_sender

            # Step 1
            # self.get_logger().info("Step 1, sending to 18A once --> 00 00 00")
            if not self.send_can_message(0x18A, [0x00, 0x00, 0x00]):
                raise Exception("Step 1 failed")
            
            # Step 2
            # self.get_logger().info("Step 2, checking on 20A for 02 00 00")
            if not self.wait_for_response(0x20A, 0x02, 0x00):
                raise Exception("Step 2 failed")
            else:
                # self.get_logger().info("In Step 2: 02 00 00 received")
                pass
            
            # Step 3 - Start cyclic 0x18A with 0x01
            # self.get_logger().info("Step 3, sending to 18A cyclic (every 10 ms) --> 01 00 00")
            self.set_cyclic_message(0x18A, [0x01, 0x00, 0x00])

            # Step 4
            # self.get_logger().info("Step 4, checking on 20A for 02 00 01")
            if not self.wait_for_response(0x20A, 0x02, 0x01):
                raise Exception("Step 3/4 failed")
            else:
                # self.get_logger().info("In Step 4: 02 00 01 received")
                pass
            
            # Step 5 - Change to 0x02
            # self.get_logger().info("Step 5, sending to 18A cyclic (every 10 ms) --> 02 00 00")
            self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])
            
            # Step 6.1 - Send password
            # self.get_logger().info("Step 6.1, sending password")
            if not self.send_can_message(0x38A, [0x4D, 0x41, 0x54, 0x54, 0x52, 0x4F]):
                raise Exception("Step 6 failed")
            
            # Step 6.2
            # self.get_logger().info("Step 6.2, checking on 20A for 02 00 02")
            if not self.wait_for_response(0x20A, 0x02, 0x02):
                raise Exception("Step 6.1 failed")
            else:
                # self.get_logger().info("In Step 6.2: 02 00 02 received")
                pass
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            return False
        
    def speed_monitor(self, message):
        """
        Callback for CAN Notifier. 
        Called immediately when a message is received.
        """
        if message.arbitration_id == 0x30A and len(message.data) >= 4:
            try:
                # Extract RPM values
                self.left_rpm = int.from_bytes(message.data[0:2], byteorder='little', signed=True)
                self.right_rpm = int.from_bytes(message.data[2:4], byteorder='little', signed=True)
                
                # Calculate speed in m/s from RPM
                # v = (rpm / 60 s/min) * d * pi * 1/ü 
                left_velocity = (self.left_rpm / 60.0) * self.drive_diameter * math.pi / self.gear_ratio
                right_velocity = (self.right_rpm / 60.0) * self.drive_diameter * math.pi / self.gear_ratio
                
                # Calculate average speed
                linear_velocity = (left_velocity + right_velocity) / 2.0

                # experimental correction factor if needed:
                # avg_speed = avg_speed * 1.033 # for gear 1 at 0.16 m/S
                # # avg_speed = avg_speed * 1.04 # for gear 2 at 0.62 m/S

                # Calculate rotational speed in rad/s
                # w = (v_r - v_l) / 2a 
                # 2a == wheelbase
                angular_velocity = (right_velocity - left_velocity) / self.wheelbase
                
                speed_msg = Twist()
                speed_msg.linear.x = linear_velocity * 3.6 # m/s to km/h -- REMOVE---
                speed_msg.angular.z = angular_velocity * 57.2958 # rad/s to deg/s -- REMOVE---
                self.speed_publisher.publish(speed_msg)
                
                # # Optional: Log occasionally
                # if self.alive_counter % 100 == 0 and avg_speed > 0:
                #     self.get_logger().info(f"Speed: {avg_speed:.3f}m/s (L:{left_rpm}, R:{right_rpm}RPM)")
            
            except Exception as e:
                self.get_logger().warning(f"Speed calculation error: {e}")

    def joy_callback(self, msg):
        """Handle controller input"""
        if not self.joy_received:
            self.joy_received = True  # Just for checking if joy publishes
            self.get_logger().info("First joystick message received!")
            return
        
        # Mode selection based on button presses

        if msg.buttons[0] == 1:  # A - Idle mode
            if self.mode != RovoMode.IDLE:
                self.enter_idle_mode()

        elif msg.buttons[1] == 1:  # B - Joystick mode
            if self.mode != RovoMode.JOYSTICK:
                self.enter_joystick_mode()

        elif msg.buttons[4] == 1:  # Y - Automatic mode
            self.get_logger().info("Automatic mode not implemented")
            # if self.mode != RovoMode.AUTOMATIC:
            #     self.enter_automatic_mode()

        # Handle driving input only in active modes
        if self.mode == RovoMode.JOYSTICK:
            self.update_joystick_mode(msg)

    def trajectory_control_callback(self, msg):
        """Handle trajectory control input"""
        if not self.speed_setpoint_received:
            self.speed_setpoint_received = True
            self.get_logger().info("First speed setpoint message received!")

        if self.mode == RovoMode.AUTOMATIC:
            self.update_automatic_mode(msg)

    def enter_joystick_mode(self):
        """Switch to joystick mode"""

        self.gear = 0
        self.mode = RovoMode.JOYSTICK
        
        if not self.activate_external_control():
            self.get_logger().error("Failed to activate external control")
            return
        self.get_logger().info("External control activated")

        # # self.send_can_message(0x28A, [0] * 8)
        # self.set_cyclic_message(0x28A, [0] * 8)
        self.set_cyclic_message(0x18A, [0x02, 0x01, 0x00])  # Joystick mode command

        rovo_status_msg = Int32()
        rovo_status_msg.data = RovoMode.JOYSTICK.value
        self.rovo_mode_publisher.publish(rovo_status_msg)
        self.get_logger().info("→ JOYSTICK MODE activated")

    def enter_automatic_mode(self):
        """Switch to automatic mode"""
        self.get_logger().info("→ AUTOMATIC MODE activated")
        self.mode = RovoMode.AUTOMATIC
        rovo_status_msg = Int32()
        rovo_status_msg.data = RovoMode.AUTOMATIC.value
        self.rovo_mode_publisher.publish(rovo_status_msg)
        self.send_can_message(0x28A, [0] * 8)
        self.set_cyclic_message(0x18A, [0x02, 0x02, 0x00])  # Speed mode command

    def enter_idle_mode(self):
       
        """Switch to idle mode"""
        # set speed control outputs to 0
        # self.send_can_message(0x28A, [0] * 8)
        self.gear = 0
        self.set_cyclic_message(0x28A, [0] * 8)
        self.get_logger().info("Speed control outputs set to 0")

        #self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])  # Idle mode command
        self.set_cyclic_message(0x18A, [0x00, 0x00, 0x00])  # Idle mode command
        self.mode = RovoMode.IDLE
        rovo_status_msg = Int32()
        rovo_status_msg.data = RovoMode.IDLE.value
        self.rovo_mode_publisher.publish(rovo_status_msg)
        self.get_logger().info("→ IDLE MODE activated (Controller inputs ignored)")

    def change_gear(self, direction: int):
        """
        Changes gear by +1 or -1 (direction) if allowed.
        direction = +1 → shift up
        direction = -1 → shift down
        """

        if self.gear_change_possible():
            new_gear = self.gear + direction
            self.gear = self.clamp(new_gear, 0, 4) # Clamp gear to valid range (0-4)
            self.get_logger().info(f"→ Gear changed to: {self.gear}")

    def clamp(self, n, minn, maxn):
        return max(minn, min(n, maxn))

    def gear_change_possible(self) -> bool:
        """Checks if gear change is allowed (only at zero speed)."""
        if (self.left_rpm == 0 and self.right_rpm == 0):
            return True
        else:
            self.get_logger().warn("Gear change blocked – speed not equal to zero")
            return False

    def update_joystick_mode(self, msg):

        # --- Joystick control ---
        joystick_x = msg.axes[0]  # Left stick horizontal
        joystick_y = msg.axes[1]  # Left stick vertical

        # Direction X (Byte0)
        if joystick_x < -0.05:      # Right (negative)
            dir_x = 1
        elif joystick_x > 0.05:     # Left (positive)
            dir_x = 2
        else:
            dir_x = 0

        # Direction Y (Byte1)
        if joystick_y > 0.05:       # Forward (positive)
            dir_y = 1
        elif joystick_y < -0.05:    # Backward (negative)
            dir_y = 2
        else:
            dir_y = 0

        # Magnitude X (Byte3+4)
        self.mag_x = self.scale_joystick_value(joystick_x)
        mag_x_l = self.mag_x & 0xFF
        mag_x_h = (self.mag_x >> 8) & 0xFF

        # Magnitude Y (Byte5+6)
        self.mag_y = self.scale_joystick_value(joystick_y)
        mag_y_l = self.mag_y & 0xFF
        mag_y_h = (self.mag_y >> 8) & 0xFF

        # --- Gear control ---
        if (msg.buttons[7] == 1) and not self.rb_pressed:  # RB pressed
            self.change_gear(+1)
            self.rb_pressed = True
        
        self.rb_pressed = (msg.buttons[7] == 1)

        if (msg.buttons[6] == 1) and not self.lb_pressed:  # LB pressed
            self.change_gear(-1)
            self.lb_pressed = True
        
        self.lb_pressed = (msg.buttons[6] == 1)

        # Update 0x28A (Bytes 0–6)
        data = [dir_x, dir_y, self.gear, mag_x_l, mag_x_h, mag_y_l, mag_y_h, 0]

        # self.get_logger().info(f"0x28A Bytes: {[hex(b) for b in data_28A]}")
        # self.get_logger().info(
        #     f"0x28A Update: dir_x={dir_x}, dir_y={dir_y}, gear={self.gear}, "
        #     f"mag_x={self.mag_x} (L={mag_x_l},H={mag_x_h}), "
        #     f"mag_y={self.mag_y} (L={mag_y_l},H={mag_y_h})"
        # )
        self.set_cyclic_message(0x28A, data)

    def scale_joystick_value(self, raw_value: float):
        """
        Scales joystick value (-1 to 1) to current (virtual)gear max
        Values <20 are set to 0 automaticly in rovo (deadzone)
        """
        max_per_gear = {1: 320, 2: 500, 3: 750, 4: 1000}

        # Scale linearly to gear maximum
        target_max = max_per_gear.get(self.gear, 0)
        scaled_value = abs(raw_value) * target_max

        if scaled_value < 20:
            return 0

        return int(scaled_value)
    
    def update_automatic_mode(self, msg):
        """
        Sets CAN frame for 0x28A in speed mode (RPM mode) based on Twist message (linear.x, angular.z).
        Calculates left/right wheel speeds and converts to RPM.
        """
        linear_velocity = msg.linear.x 
        angular_velocity = msg.angular.z

        left_velocity = linear_velocity + (self.wheelbase * angular_velocity / 2)
        right_velocity = linear_velocity - (self.wheelbase * angular_velocity / 2)

        # Direction
        if left_velocity >= 0:      
            left_dir = 1            # Forwards
        else:                       
            left_dir = 2            # Backwards

        if right_velocity >= 0:     
            right_dir = 1           # Forwards
        else:                       
            right_dir = 2           # Backwards

        # --- Convert speeds to RPM ---
        # n = v * 60 s/min * ü * 1/d*π 
        left_rpm = int(abs(left_velocity) * 60.0 * self.gear_ratio * (1/(self.drive_diameter * math.pi)))
        right_rpm = int(abs(right_velocity) * 60.0 * self.gear_ratio * (1/(self.drive_diameter * math.pi)))

        # --- CAN message for 0x28A ---
        data = [
            left_dir,            # Byte0
            right_dir,           # Byte1
            1,                        # Byte2 - set to 1 to enable motor control
            left_rpm & 0xFF,   # Byte3 (low)
            (left_rpm >> 8) & 0xFF,  # Byte4 (high)
            right_rpm & 0xFF,  # Byte5
            (right_rpm >> 8) & 0xFF, # Byte6
            self.alive_counter & 0xFF # Byte7 (Alive)
        ]

        # Debug log (optional)
        # self.get_logger().info(
        #     f"28A-RPM Mode | DirL={self.dir_left} SpeedL={self.speed_left} | DirR={self.dir_right} SpeedR={self.speed_right} | Gear={self.gear}"
        # )

        # Send
        self.set_cyclic_message(0x28A, data)

    def destroy_node(self):
        """Clean shutdown"""
        print("Shutting down...")
        self.running = False
        self.stop_cyclic_message()  # Stop all
        if hasattr(self, 'notifier'):
            self.notifier.stop()
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
        
        # Shutdown CAN interface if configured
        self.shutdown_can_interface()
        self.shutdown_requested = True
        
def main(args=None):
    rclpy.init(args=args)
    controller = RovoDriverNode()
    
    try:
        # Use spin_once with shutdown check
        while rclpy.ok() and not controller.shutdown_requested:
            rclpy.spin_once(controller, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        print("Terminated by user")
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()