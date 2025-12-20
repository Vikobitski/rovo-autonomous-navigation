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
from enum import Enum

''' README
- to start from terminal:
    --> # Start spray
        ros2 topic pub --once /spray_control std_msgs/msg/String "data: 'spray_start'"
        # Pause spray
        ros2 topic pub --once /spray_control std_msgs/msg/String "data: 'spray_pause'"
    --> # Stop peripherie
        ros2 topic pub --once /peripherie/stop std_msgs/msg/Bool "data: true"
'''

class ROVOMode(Enum):
    IDLE = 0
    JOYSTICK_MODE = 1
    SPEED_MODE = 2

class ROVOController(Node):
    def __init__(self):
        super().__init__('rovo_controller')

        # Operating mode
        self.shutdown_requested = False
        self.mode = ROVOMode.IDLE
        
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
        self.gear = 0
        self.rb_pressed = False
        self.lb_pressed = False
        self.x_pressed = False
        self.x_press_count = 0  # Counter for X button presses
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.mag_x = 0
        self.mag_y = 0
        self.speed_left = 0
        self.speed_right = 0
        self.last_left_dir = 1
        self.last_right_dir = 1
        self.dir_left = 0
        self.dir_right = 0

        # Speed calculation variables
        self.actual_speed_left_rpm = 0.0  # From CAN 0x30A Byte 0-1
        self.actual_speed_right_rpm = 0.0  # From CAN 0x30A Byte 2-3
        self.current_speed_ms = 0.0  # Calculated average speed in m/s
                
        # Publisher for periphery stop signal
        self.periphery_stop_publisher = self.create_publisher(
            Bool,
            '/peripherie/stop',
            10
        )
        
        # Controller and ROS2 checks FIRST (before CAN setup)
        if not self.check_controller_and_joy():
            self.get_logger().error("\033[91mController or Joy node not available - terminating program\033[0m")
            self.emergency_shutdown()
            return
        
        # Setup CAN interface only after controller checks pass
        if not self.setup_can_interface():
            self.get_logger().error("\033[91mCAN interface setup failed!\033[0m")
            self.emergency_shutdown()
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
        self.get_logger().info("ROVO Controller started - starting initialization...")
        success = self.initialization_sequence()
        
        if success:
            self.get_logger().info("ROVO successfully initialized!")
            self.get_logger().info("Ready for controller inputs:")
            self.get_logger().info("  B = Joystick mode")
            self.get_logger().info("  Y = Speed mode") 
            self.get_logger().info("  A = Idle mode")
            self.get_logger().info("  X = Stop periphery (1st press) / Exit program (2nd press)")
        else:
            self.get_logger().error("ROVO initialization failed!")
            return
        
        # Publisher for spray control
        self.spray_publisher = self.create_publisher(
            String,
            '/spray_control',
            10
        )

        # Publisher for speed data
        self.speed_publisher = self.create_publisher(
            Twist,
            '/rovo/speed',
            10
        )

        # Start CAN receiver thread for speed data
        self.can_receiver_thread = threading.Thread(target=self.speed_monitor)
        self.can_receiver_thread.daemon = True
        self.can_receiver_thread.start()
        
        # Status variables
        self.spray_active = False

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

        # Wait for Joy messages (max 2 seconds)
        self.get_logger().info("Waiting for Joy node...")
        start_time = time.time()
        timeout = 7.0

        while not self.joy_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks for 0.1s
            
        if not self.joy_received:
            self.get_logger().error(f"Joy node is not running: joy_received: {self.joy_received}")
            return False
        else:
            self.get_logger().info("Joy node is active")
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
    
    def initialization_sequence(self):
        """Complete ROVO initialization"""
        self.get_logger().info("Starting initialization...")
        
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
        
    def speed_monitor(self):
        """Monitor CAN 0x30A messages for speed calculation"""
        while self.running:
            try:
                message = self.can_bus.recv(timeout=0.1)
                if message and message.arbitration_id == 0x30A and len(message.data) >= 4:
                    # Extract RPM values
                    left_rpm = int.from_bytes(message.data[0:2], byteorder='little', signed=True)
                    right_rpm = int.from_bytes(message.data[2:4], byteorder='little', signed=True)
                    
                    # Calculate and publish speed in m/s
                    avg_speed = (left_rpm + right_rpm) * 0.00412125 / 2.0 / 3.6
                    avg_speed = avg_speed * 1.033 # for gear 1 at 0.16 m/S
                    # avg_speed = avg_speed * 1.04 # for gear 2 at 0.62 m/S
                    
                    speed_msg = Twist()
                    speed_msg.linear.x = avg_speed
                    self.speed_publisher.publish(speed_msg)
                    
                    # # Optional: Log occasionally
                    # if self.alive_counter % 100 == 0 and avg_speed > 0:
                    #     self.get_logger().info(f"Speed: {avg_speed:.3f}m/s (L:{left_rpm}, R:{right_rpm}RPM)")
                        
            except Exception as e:
                if self.running:
                    self.get_logger().warning(f"Speed monitoring error: {e}")
                time.sleep(0.01)

    def joy_callback(self, msg):
        """Handle controller input"""
        if not self.joy_received:
            self.joy_received = True  # Just for checking if joy publishes
            self.get_logger().info("First joystick message received!")
            return
        
        # --- Gear control ---
        if msg.buttons[5] == 1 and not self.rb_pressed:  # RB pressed
            self.change_gear(+1)
            self.rb_pressed = True
        elif msg.buttons[5] == 0:
            self.rb_pressed = False

        if msg.buttons[4] == 1 and not self.lb_pressed:  # LB pressed
            self.change_gear(-1)
            self.lb_pressed = True
        elif msg.buttons[4] == 0:
            self.lb_pressed = False
        
        # Mode selection based on button presses
        if msg.buttons[1] == 1:  # B - Joystick mode
            if self.mode != ROVOMode.JOYSTICK_MODE:
                self.enter_joystick_mode()
        elif msg.buttons[3] == 1:  # Y - Speed mode
            if self.mode != ROVOMode.SPEED_MODE:
                self.enter_speed_mode()
        elif msg.buttons[0] == 1:  # A - Idle mode
            if self.mode != ROVOMode.IDLE:
                self.enter_idle_mode()
        elif msg.buttons[2] == 1 and not self.x_pressed:  # X - Stop periphery / Exit
            self.x_pressed = True
            self.x_press_count += 1
            
            if self.x_press_count == 1:
                # First press: Stop periphery nodes
                self.get_logger().info("X pressed (1st time) - Stopping periphery nodes...")
                stop_msg = Bool()
                stop_msg.data = True
                self.periphery_stop_publisher.publish(stop_msg)
                self.get_logger().info("Periphery stop signal sent. Press X again to exit program.")
            elif self.x_press_count >= 2:
                # Second press: Exit program
                self.get_logger().info("X pressed (2nd time) - Exiting program...")
                self.running = False
                # self.shutdown_requested = True
                self.destroy_node()
                # rclpy.shutdown()
                return
        elif msg.buttons[2] == 0:
            self.x_pressed = False

        axis_7 = msg.axes[7]
        
        # Start spray nodes, i.e. pump-bldc, valves, and propellers
        if axis_7 > 0.0 and not self.spray_active:
            spray_msg = String()
            spray_msg.data = "spray_start"
            self.spray_publisher.publish(spray_msg)
            self.spray_active = True
            self.get_logger().info("Spray nodes started!")
            
        # Stop spray nodes, i.e. pump-bldc and valves
        elif axis_7 < 0.0 and self.spray_active:
            spray_msg = String()
            spray_msg.data = "spray_pause"
            self.spray_publisher.publish(spray_msg)
            self.spray_active = False
            self.get_logger().info("Spray nodes paused!")
        
        # Handle driving input only in active modes
        if self.mode == ROVOMode.JOYSTICK_MODE:
            self.update_joy_mode(msg)
        elif self.mode == ROVOMode.SPEED_MODE:
            self.update_speed_mode(msg)

    def enter_joystick_mode(self):
        """Switch to joystick mode"""
        self.get_logger().info("→ JOYSTICK MODE activated")
        self.mode = ROVOMode.JOYSTICK_MODE
        self.send_can_message(0x28A, [0] * 8)
        self.get_logger().info("Gear 0 sent")
        self.set_cyclic_message(0x18A, [0x02, 0x01, 0x00])  # Joystick mode command

    def enter_speed_mode(self):
        """Switch to speed mode"""
        self.get_logger().info("→ SPEED MODE activated")
        self.mode = ROVOMode.SPEED_MODE
        self.last_left_dir = 1
        self.last_right_dir = 1
        self.send_can_message(0x28A, [0] * 8)
        self.set_cyclic_message(0x18A, [0x02, 0x02, 0x00])  # Speed mode command

    def enter_idle_mode(self):
        """Switch to idle mode"""
        self.get_logger().info("→ IDLE MODE activated (Controller inputs ignored)")
        self.mode = ROVOMode.IDLE
        self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])  # Idle mode command

    def change_gear(self, direction: int):
        """
        Changes gear by +1 or -1 (direction) if allowed.
        direction = +1 → shift up
        direction = -1 → shift down
        """
        if not self.can_change_gear():
            return  # Abort if speed != 0

        new_gear = self.gear + direction
        if 0 <= new_gear <= 4:
            self.gear = new_gear
            if direction > 0:
                self.get_logger().info(f"→ Gear up: {self.gear}")
            else:
                self.get_logger().info(f"→ Gear down: {self.gear}")
            
            # Send CAN frame directly
            data_28A = [0, 0, self.gear, 0, 0, 0, 0, 0]
            self.set_cyclic_message(0x28A, data_28A)

    def can_change_gear(self) -> bool:
        """Checks if gear change is allowed (only at zero speed)."""
        if ((self.speed_left == 0 and self.speed_right == 0) and (self.mag_x == 0 and self.mag_y == 0)):
            return True
        else:
            self.get_logger().warn("Gear change blocked – speed not equal to zero")
            return False

    def update_joy_mode(self, msg):
        self.joystick_x = msg.axes[0]  # Left stick horizontal
        self.joystick_y = msg.axes[1]  # Left stick vertical

        # Direction X (Byte0)
        if self.joystick_x < -0.05:      # Right (negative)
            dir_x = 1
        elif self.joystick_x > 0.05:     # Left (positive)
            dir_x = 2
        else:
            dir_x = 0

        # Direction Y (Byte1)
        if self.joystick_y > 0.05:       # Forward (positive)
            dir_y = 1
        elif self.joystick_y < -0.05:    # Backward (negative)
            dir_y = 2
        else:
            dir_y = 0

        # Magnitude X (Byte3+4)
        self.mag_x = self.scale_joystick_value(self.joystick_x)
        mag_x_l = self.mag_x & 0xFF
        mag_x_h = (self.mag_x >> 8) & 0xFF

        # Magnitude Y (Byte5+6)
        self.mag_y = self.scale_joystick_value(self.joystick_y)
        mag_y_l = self.mag_y & 0xFF
        mag_y_h = (self.mag_y >> 8) & 0xFF

        # Update 0x28A (Bytes 0–6)
        data_28A = [dir_x, dir_y, self.gear, mag_x_l, mag_x_h, mag_y_l, mag_y_h, 0]
        # self.get_logger().info(f"0x28A Bytes: {[hex(b) for b in data_28A]}")
        # self.get_logger().info(
        #     f"0x28A Update: dir_x={dir_x}, dir_y={dir_y}, gear={self.gear}, "
        #     f"mag_x={self.mag_x} (L={mag_x_l},H={mag_x_h}), "
        #     f"mag_y={self.mag_y} (L={mag_y_l},H={mag_y_h})"
        # )
        self.set_cyclic_message(0x28A, data_28A)

    def scale_joystick_value(self, raw_value: float):
        """
        Scales joystick value (-1 to 1) to current gear:
        - Linear from 0-1000 to 0-max_per_gear[self.gear]
        - Values <20 are set to 0 (deadzone)
        """
        max_per_gear = {1: 320, 2: 500, 3: 750, 4: 1000}
        value = abs(raw_value) * 1000
        # Scale linearly to gear maximum
        scaled = value * max_per_gear.get(self.gear, 1000) / 1000
        if scaled < 20:
            return 0
        return int(scaled)
    
    def update_speed_mode(self, joy_msg):
        """
        Sets CAN frame for 0x28A in speed mode (RPM mode).
        - Left stick controls direction + gear-dependent speed with LT
        - Right stick controls direction + gear-dependent speed with RT
        """

        # --- Axes for directions (e.g. left_stick_y = axis 1, right_stick_y = axis 4) ---
        left_axis = joy_msg.axes[1]   # Left stick (forward/backward)
        right_axis = joy_msg.axes[4]  # Right stick (forward/backward)

        # --- Triggers for speeds (e.g. LT = axis 2, RT = axis 5) ---
        lt = (1 - joy_msg.axes[2]) / 2  # Axis -1 to 1 -> normalized to 0–1
        rt = (1 - joy_msg.axes[5]) / 2

        # --- Gear-dependent max RPM ---
        max_per_gear = {1: 800, 2: 1600, 3: 2400, 4: 3200}
        max_rpm = max_per_gear.get(self.gear, 0)

        # --- Helper function: Scales trigger to max_rpm ---
        def scale_trigger(value):
            rpm = int(value * max_rpm)
            return rpm if rpm >= 20 else 0

        # Left joystick Y-axis → Byte0
        if joy_msg.axes[1] > 0:      # Upward
            self.last_left_dir = 1
        elif joy_msg.axes[1] < 0:    # Downward
            self.last_left_dir = 2
        # Right joystick Y-axis → Byte1
        if joy_msg.axes[4] > 0:      # Upward
            self.last_right_dir = 1
        elif joy_msg.axes[4] < 0:    # Downward
            self.last_right_dir = 2

        # Set Byte0/Byte1 to stored direction
        self.dir_left = self.last_left_dir
        self.dir_right = self.last_right_dir

        # --- Speeds from triggers ---
        self.speed_left = scale_trigger(lt)
        self.speed_right = scale_trigger(rt)

        # --- CAN message for 0x28A ---
        data = [
            self.dir_left,            # Byte0
            self.dir_right,           # Byte1
            self.gear,                # Byte2
            self.speed_left & 0xFF,   # Byte3 (low)
            (self.speed_left >> 8) & 0xFF,  # Byte4 (high)
            self.speed_right & 0xFF,  # Byte5
            (self.speed_right >> 8) & 0xFF, # Byte6
            self.alive_counter & 0xFF # Byte7 (Alive)
        ]

        # Debug log (optional)
        # self.get_logger().info(
        #     f"28A-RPM Mode | DirL={self.dir_left} SpeedL={self.speed_left} | DirR={self.dir_right} SpeedR={self.speed_right} | Gear={self.gear}"
        # )

        # Send
        self.set_cyclic_message(0x28A, data)

    def emergency_shutdown(self):
        """Emergency shutdown: stop all nodes and self"""
        try:
            # Stop-Signal for all other nodes
            time.sleep(3.0)
            stop_msg = Bool()
            stop_msg.data = True
            self.periphery_stop_publisher.publish(stop_msg)
            self.running = False
            self.get_logger().warn("Emergency stop signal sent to all periphery nodes")
            time.sleep(0.5)
            
            # Stop node
            self.get_logger().info("Shutting down xbox controller node...")

            # Force exit the entire program
            import sys
            sys.exit(1)
            
        except Exception as e:
            self.get_logger().error(f"Error during emergency shutdown: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        print("Shutting down...")
        self.running = False
        self.stop_cyclic_message()  # Stop all
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
        
        # Shutdown CAN interface if configured
        self.shutdown_can_interface()
        self.shutdown_requested = True
        
def main(args=None):
    rclpy.init(args=args)
    controller = ROVOController()
    
    try:
        # Use spin_once with shutdown check like BNO085
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