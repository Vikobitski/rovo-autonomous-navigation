#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import can
import threading
import time
import subprocess
from enum import Enum

class RovoMode(Enum):
    IDLE = 0
    JOYSTICK_MODE = 1
    SPEED_MODE = 2

class RovoDriverNode(Node):
    def __init__(self):
        super().__init__('rovo_driver_node')

        # Operating mode
        self.mode = RovoMode.IDLE
        self.shutdown_requested = False
        
        # CAN interface configuration
        self.can_interface = 'can0'
        self.can_bitrate = 500000
        self.shutdown_can_on_exit = False  # Set to True in production
        self.can_started_by_us = False
        self.can_bus = None

        # Threading control
        self.running = True
        self.alive_counter = 0
        self.cyclic_messages = {}
        self.cyclic_lock = threading.Lock()

        # State variables
        self.gear = 0
        self.speed_left = 0
        self.speed_right = 0
        self.dir_left = 0
        self.dir_right = 0
        
        # Publishers
        self.speed_publisher = self.create_publisher(Twist, '/rovo/speed', 10)
        self.status_publisher = self.create_publisher(String, '/rovo/status', 10)

        # Setup CAN interface
        if not self.setup_can_interface():
            self.get_logger().error("CAN interface setup failed!")
            self.destroy_node()
            return
        
        # Initialize CAN bus
        try:
            self.can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
            self.get_logger().info("CAN interface initialized")
        except Exception as e:
            self.get_logger().error(f"CAN interface error: {e}")
            self.destroy_node()
            return

        # Start cyclic sender thread
        self.cyclic_thread = threading.Thread(target=self.cyclic_sender)
        self.cyclic_thread.daemon = True
        self.cyclic_thread.start()

        # Start initialization sequence
        success = self.initialization_sequence()
        if success:
            self.get_logger().info("ROVO successfully initialized!")
            self.enter_idle_mode()
        else:
            self.get_logger().error("ROVO initialization failed!")

        # Start speed monitor thread
        self.can_receiver_thread = threading.Thread(target=self.speed_monitor)
        self.can_receiver_thread.daemon = True
        self.can_receiver_thread.start()

    def setup_can_interface(self):
        """Setup CAN interface with automatic configuration"""
        try:
            # Check interface status
            result = subprocess.run(['ip', 'link', 'show', self.can_interface], 
                                  capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f"CAN interface {self.can_interface} does not exist!")
                return False
            
            if 'state UP' in result.stdout:
                self.get_logger().info(f"CAN interface {self.can_interface} is already active")
                return True
            
            # Configure interface
            self.get_logger().info(f"Configuring CAN interface {self.can_interface}...")
            cmd = ['sudo', 'ip', 'link', 'set', self.can_interface, 'up', 'type', 'can', 'bitrate', str(self.can_bitrate)]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().error(f"Error bringing up CAN: {result.stderr}")
                return False
            
            self.can_started_by_us = True
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during CAN setup: {e}")
            return False

    def shutdown_can_interface(self):
        """Shutdown CAN interface if we started it"""
        if self.shutdown_can_on_exit and self.can_started_by_us:
            try:
                subprocess.run(['sudo', 'ip', 'link', 'set', 'down', self.can_interface], 
                             capture_output=True, text=True)
                self.get_logger().info(f"CAN interface {self.can_interface} shut down")
            except Exception as e:
                self.get_logger().error(f"Error shutting down CAN: {e}")

    def set_cyclic_message(self, can_id, data):
        """Set a message to be sent cyclically every 10ms"""
        with self.cyclic_lock:
            self.cyclic_messages[can_id] = data.copy() if data else None

    def cyclic_sender(self):
        """Thread that sends all cyclic messages every 10ms"""
        while self.running:
            with self.cyclic_lock:
                for can_id, data in self.cyclic_messages.items():
                    if data is not None:
                        data_to_send = data.copy()
                        
                        # Update alive counter for 0x28A
                        if can_id == 0x28A and len(data_to_send) >= 8:
                            data_to_send[7] = self.alive_counter
                            self.alive_counter = (self.alive_counter + 1) % 256
                        
                        self.send_can_message(can_id, data_to_send)
                
            time.sleep(0.01)

    def send_can_message(self, can_id, data):
        """Send single CAN message"""
        try:
            if self.can_bus:
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                self.can_bus.send(msg)
                return True
        except Exception as e:
            self.get_logger().error(f"CAN send error {hex(can_id)}: {e}")
        return False

    def receive_can_message(self, expected_id, timeout=1.0):
        """Receive specific CAN message"""
        try:
            if self.can_bus:
                msg = self.can_bus.recv(timeout=timeout)
                if msg and msg.arbitration_id == expected_id:
                    return msg
        except Exception as e:
            self.get_logger().error(f"Error receiving CAN: {e}")
        return None

    def wait_for_response(self, expected_id, byte0, byte2, timeout=5.0):
        """Wait for specific response bytes"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.receive_can_message(expected_id, 0.1)
            if msg and len(msg.data) >= 3:
                if msg.data[0] == byte0 and msg.data[2] == byte2:
                    return True
            time.sleep(0.01)
        return False

    def initialization_sequence(self):
        """Perform Rovo startup handshake"""
        self.get_logger().info("Starting initialization sequence...")
        try:
            # Step 0: Start alive counter
            self.set_cyclic_message(0x28A, [0] * 8)

            # Step 1: Send to 18A once
            if not self.send_can_message(0x18A, [0x00, 0x00, 0x00]):
                return False

            # Step 2: Check 20A for 02 00 00
            if not self.wait_for_response(0x20A, 0x02, 0x00):
                return False

            # Step 3: Start cyclic 18A
            self.set_cyclic_message(0x18A, [0x01, 0x00, 0x00])

            # Step 4: Check 20A for 02 00 01
            if not self.wait_for_response(0x20A, 0x02, 0x01):
                return False

            # Step 5: Change to 0x02
            self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])

            # Step 6: Send password
            if not self.send_can_message(0x38A, [0x4D, 0x41, 0x54, 0x54, 0x52, 0x4F]):
                return False

            # Step 6.2: Check confirmation
            if not self.wait_for_response(0x20A, 0x02, 0x02):
                return False

            return True
        except Exception as e:
            self.get_logger().error(f"Initialization exception: {e}")
            return False

    def speed_monitor(self):
        """Monitor speed feedback from CAN 0x30A"""
        while self.running:
            try:
                if self.can_bus:
                    msg = self.can_bus.recv(timeout=0.1)
                    if msg and msg.arbitration_id == 0x30A and len(msg.data) >= 4:
                        left_rpm = int.from_bytes(msg.data[0:2], byteorder='little', signed=True)
                        right_rpm = int.from_bytes(msg.data[2:4], byteorder='little', signed=True)
                        
                        # Calculate m/s
                        avg_speed = (left_rpm + right_rpm) * 0.00412125 / 2.0 / 3.6
                        avg_speed = avg_speed * 1.033 # for gear 1 at 0.16 m/S
                        # avg_speed = avg_speed * 1.04 # for gear 2 at 0.62 m/S
                        
                        twist = Twist()
                        twist.linear.x = avg_speed
                        self.speed_publisher.publish(twist)
            except Exception:
                time.sleep(0.01)

    def enter_idle_mode(self):
        self.mode = RovoMode.IDLE
        self.set_cyclic_message(0x18A, [0x02, 0x00, 0x00])
        self.get_logger().info("Switched to IDLE mode")

    def enter_speed_mode(self):
        self.mode = RovoMode.SPEED_MODE
        self.set_cyclic_message(0x18A, [0x02, 0x02, 0x00])
        self.get_logger().info("Switched to SPEED mode")

    def destroy_node(self):
        self.running = False
        try:
            if self.cyclic_thread.is_alive():
                self.cyclic_thread.join(timeout=1.0)
            if self.can_bus:
                self.can_bus.shutdown()
        except:
            pass
        self.shutdown_can_interface()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RovoDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
