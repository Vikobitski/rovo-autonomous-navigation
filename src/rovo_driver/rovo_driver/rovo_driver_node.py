#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import can
import threading
import time
import subprocess

import math 
from enum import Enum

class RovoMode(Enum):
    MANUAL = 0 # Manual / Passive
    AUTOMATIC = 1 # Automatic / Active
    AUTOMATIC_WATCHDOG = 2 # Automatic with 1s timeout

class RovoDriverNode(Node):
    def __init__(self):
        super().__init__('rovo_driver_node')

        # Rovo Mechanical parameters
        self.wheelbase = 0.96 # [m] German: Radabstand
        self.gear_ratio = 16
        self.drive_diameter = 0.35 # [m]

        # Operating mode
        self.shutdown_requested = False
        self.mode = RovoMode.MANUAL
        
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



        self.speed_setpoint_received = False
                
        # Publisher for rovo mode
        self.mode_publisher = self.create_publisher(
            Int32,
            '/rovo/mode',
            10
        )
        
        # Subscriber for mode setting
        self.mode_subscription = self.create_subscription(
            Int32,
            '/rovo/set_mode',
            self.set_mode_callback,
            10
        )

        # Watchdog Timer (1.0s timeout)
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_timeout)
        self.watchdog_timer.cancel() # Start cancelled
        # Publisher for speed data
        self.speed_publisher = self.create_publisher(
            TwistStamped,
            '/rovo/speed',
            10
        )
        
        # Subscription for speed setpoint
        self.trajectory_control_subscription = self.create_subscription(
            TwistStamped,
            '/rovo/set_speed',
            self.trajectory_control_callback,
            20
        )

        # Subscriber for heartbeat
        self.heartbeat_subscription = self.create_subscription(
            Bool,
            '/rovo/heartbeat',
            self.heartbeat_callback,
            10
        )

        
        # Diagnostics Publisher
        self.diag_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Setup CAN interface
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
        self.get_logger().info("ROVO driver started - ready for commands")
        self.get_logger().info("  Send 1 to /rovo/set_mode for AUTOMATIC mode") 
        self.get_logger().info("  Send 0 to /rovo/set_mode for MANUAL mode")

        # Start CAN Notifier with a listener for speed updates
        # This will call self.speed_monitor whenever a message arrives
        self.notifier = can.Notifier(self.can_bus, [self.speed_monitor])

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
                if msg.data[0] == expected_byte0 and msg.data[2] == expected_byte2:
                    return True
            time.sleep(0.01)
        return False
    
    def activate_external_control(self):
        """Complete ROVO initialization"""
        self.get_logger().info("Start activation of external control...")
        
        try:
            # Step 0 - Start alive counter with 0x28A
            self.set_cyclic_message(0x28A, [0] * 8)  # Will be updated in cyclic_sender

            # Step 1
            if not self.send_can_message(0x18A, [0x00, 0x00, 0x00]):
                raise Exception("Step 1 failed")
            
            # Step 2
            if not self.wait_for_response(0x20A, 0x02, 0x00):
                raise Exception("Step 2 failed")
            
            # Step 3 - Start cyclic 0x18A with 0x01
            self.set_cyclic_message(0x18A, [0x01, 0x00, 0x00])

            # Step 4
            if not self.wait_for_response(0x20A, 0x02, 0x01):
                raise Exception("Step 3/4 failed")
            
            # Step 5 - Change to 0x02
            self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])
            
            # Step 6.1 - Send password
            if not self.send_can_message(0x38A, [0x4D, 0x41, 0x54, 0x54, 0x52, 0x4F]):
                raise Exception("Step 6.1 failed")
            
            # Step 6.2
            if not self.wait_for_response(0x20A, 0x02, 0x02):
                raise Exception("Step 6.2 failed")
            
            # Send Success Diagnostic
            self.get_logger().info("External control activated")
            #self.publish_diagnostic(DiagnosticStatus.OK, "External control activated successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"External control activation failed: {e}")
            #self.publish_diagnostic(DiagnosticStatus.ERROR, f"External control activation failed: {e}")
            return False

    def publish_diagnostic(self, level, message):
        """Helper to publish a single diagnostic message"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status_msg = DiagnosticStatus()
        status_msg.name = "rovo_driver_node: External Control"
        status_msg.hardware_id = self.can_interface
        status_msg.level = level
        status_msg.message = message
        
        diag_array.status.append(status_msg)
        self.diag_publisher.publish(diag_array)

        
    def speed_monitor(self, message):
        """
        Callback for CAN Notifier. 
        Called immediately when a message is received.
        """
        if message.arbitration_id == 0x30A and len(message.data) >= 4:
            try:
                # Extract RPM values
                left_rpm = int.from_bytes(message.data[0:2], byteorder='little', signed=True)
                right_rpm = int.from_bytes(message.data[2:4], byteorder='little', signed=True)
                
                # Calculate speed in m/s from RPM
                left_velocity = (left_rpm / 60.0) * self.drive_diameter * math.pi / self.gear_ratio
                right_velocity = (right_rpm / 60.0) * self.drive_diameter * math.pi / self.gear_ratio
                
                # Calculate average speed
                linear_velocity = (left_velocity + right_velocity) / 2.0

                # Calculate rotational speed in rad/s
                angular_velocity = (right_velocity - left_velocity) / self.wheelbase
                
                speed_msg = TwistStamped()
                speed_msg.header.stamp = self.get_clock().now().to_msg()
                speed_msg.header.frame_id = "base_link"
                speed_msg.twist.linear.x = linear_velocity
                speed_msg.twist.angular.z = angular_velocity
                self.speed_publisher.publish(speed_msg)
                
            except Exception as e:
                self.get_logger().warning(f"Speed calculation error: {e}")

    def set_mode_callback(self, msg):
        """Handle mode switching from external topic"""
        requested_mode = msg.data
        
        if requested_mode == 0: # Manual request
            #if self.mode != RovoMode.MANUAL:
                self.enter_manual_mode()
                
        elif requested_mode == 1: # Automatic request
            #if self.mode != RovoMode.AUTOMATIC:
                self.enter_automatic_mode()

        elif requested_mode == 2: # Automatic Watchdog request
            #if self.mode != RovoMode.AUTOMATIC_WATCHDOG:
                self.enter_automatic_watchdog_mode()

        else:
            self.get_logger().warn(f"Unknown mode requested: {requested_mode}")

    def trajectory_control_callback(self, msg):
        """Handle trajectory control input"""
        if not self.speed_setpoint_received:
            self.speed_setpoint_received = True
            self.get_logger().info("First speed setpoint message received!")

        if self.mode == RovoMode.AUTOMATIC or self.mode == RovoMode.AUTOMATIC_WATCHDOG:
            self.update_automatic_mode(msg)

    def enter_automatic_mode(self):
        """Switch to automatic mode"""
        
        # Ensure watchdog is off
        self.watchdog_timer.cancel()
        
        if not self.activate_external_control():
            self.get_logger().error("Failed to activate external control")
            return

        self.mode = RovoMode.AUTOMATIC
        rovo_status_msg = Int32()
        rovo_status_msg.data = RovoMode.AUTOMATIC.value
        self.mode_publisher.publish(rovo_status_msg)
        
        # Initialize speed mode command
        self.set_cyclic_message(0x18A, [0x02, 0x02, 0x00])  # Speed mode command
        self.send_can_message(0x28A, [0] * 8)

        self.get_logger().info("→ AUTOMATIC MODE activated")

    def enter_automatic_watchdog_mode(self):
        """Switch to automatic mode with watchdog"""
               
        if not self.activate_external_control():
            self.get_logger().error("Failed to activate external control")
            return

        self.mode = RovoMode.AUTOMATIC_WATCHDOG
        self.watchdog_timer.reset() # Start/Reset watchdog timer
        
        rovo_status_msg = Int32()
        rovo_status_msg.data = RovoMode.AUTOMATIC_WATCHDOG.value
        self.mode_publisher.publish(rovo_status_msg)
        
        self.set_cyclic_message(0x18A, [0x02, 0x02, 0x00])  # Speed mode command
        self.send_can_message(0x28A, [0] * 8)

        self.get_logger().info("→ AUTOMATIC WATCHDOG MODE activated")

    def enter_manual_mode(self):     
        """Switch to manual mode"""
        self.watchdog_timer.cancel() # Stop watchdog if running
        self.set_cyclic_message(0x28A, [0] * 8) # reset speed outputs
        self.set_cyclic_message(0x18A, [0x00, 0x00, 0x00])  # Deactivate external control
        self.get_logger().info("Deactivated external control")
        #self.publish_diagnostic(DiagnosticStatus.OK, "External control deactivated")
        self.mode = RovoMode.MANUAL
        
        mode_msg = Int32()
        mode_msg.data = RovoMode.MANUAL.value
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info("→ ROVO JOYSTICK (MANUAL) MODE activated")

    
    def update_automatic_mode(self, msg):
        """
        Sets CAN frame for 0x28A in speed mode (RPM mode) based on Twist message (linear.x, angular.z).
        Calculates left/right wheel speeds and converts to RPM.
        """
        linear_velocity = msg.twist.linear.x 
        angular_velocity = msg.twist.angular.z

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
            left_dir,                   # Byte0
            right_dir,                  # Byte1
            1,                          # Byte2 - set to 1 to enable motor control
            left_rpm & 0xFF,            # Byte3 (low)
            (left_rpm >> 8) & 0xFF,     # Byte4 (high)
            right_rpm & 0xFF,           # Byte5
            (right_rpm >> 8) & 0xFF,    # Byte6
            self.alive_counter & 0xFF   # Byte7 (Alive)
        ]

        # Send
        self.set_cyclic_message(0x28A, data)

    def heartbeat_callback(self, msg):
        """Update heartbeat - resets watchdog timer"""
        if self.mode == RovoMode.AUTOMATIC_WATCHDOG:
            self.watchdog_timer.reset()

    def watchdog_timeout(self):
        """Called when watchdog timer expires"""
        self.get_logger().warn("Watchdog timeout! Switching to MANUAL mode.")
        self.enter_manual_mode()
    
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
