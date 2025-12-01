#!/usr/bin/env python3
"""
rtcm_bridge_node.py

Bridge node: subscribes to /rtcm (rtcm_msgs/Message or mavros_msgs/RTCM)
and forwards the raw RTCM bytes to a u‑blox device via serial (default)
or TCP/UDP (optional).
"""

import rclpy
from rclpy.node import Node

# Prefer imports but handle optional presence for mavros_msgs
from rtcm_msgs.msg import Message as RtcmMsg
try:
    from mavros_msgs.msg import RTCM as MavrosRtcm
    _HAVE_MAVROS = True
except Exception:
    MavrosRtcm = None
    _HAVE_MAVROS = False

import serial
import socket
import threading
import time

class RtcmBridge(Node):
    def __init__(self):
        super().__init__('rtcm_bridge')

        # Parameters
        self.declare_parameter('transport', 'serial')        # serial | tcp | udp | none
        self.declare_parameter('device', '/dev/ttyACM0')    # serial device
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('tcp_host', '127.0.0.1')
        self.declare_parameter('tcp_port', 5000)
        self.declare_parameter('use_mavros_msgs', False)    # allow mavros_msgs/RTCM
        self.declare_parameter('write_timeout', 0.1)

        self._transport = self.get_parameter('transport').value
        self._device = self.get_parameter('device').value
        self._baud = int(self.get_parameter('baudrate').value)
        self._tcp_host = self.get_parameter('tcp_host').value
        self._tcp_port = int(self.get_parameter('tcp_port').value)
        self._use_mavros = bool(self.get_parameter('use_mavros_msgs').value)
        self._write_timeout = float(self.get_parameter('write_timeout').value)

        self._serial = None
        self._sock = None
        self._sock_lock = threading.Lock()

        if self._transport == 'serial':
            try:
                self._serial = serial.Serial(self._device, self._baud, timeout=1, write_timeout=self._write_timeout)
                self.get_logger().info(f'Opened serial {self._device} @ {self._baud}')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port {self._device}: {e}')
                self._serial = None
        elif self._transport in ('tcp', 'udp'):
            try:
                if self._transport == 'tcp':
                    self._sock = socket.create_connection((self._tcp_host, self._tcp_port), timeout=5)
                else:
                    # UDP socket (connect sets remote endpoint)
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect((self._tcp_host, self._tcp_port))
                    self._sock = s
                self.get_logger().info(f'Opened {self._transport} socket to {self._tcp_host}:{self._tcp_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to open {self._transport} socket: {e}')
                self._sock = None

        # subscribe
        self._rtcm_sub = self.create_subscription(RtcmMsg, '/rtcm', self._on_rtcm, 10)

        # optionally subscribe mavros_msgs/RTCM if requested and available
        if self._use_mavros and _HAVE_MAVROS:
            self._mav_sub = self.create_subscription(MavrosRtcm, '/rtcm', self._on_mavros_rtcm, 10)

        self.get_logger().info('rtcm_bridge ready (transport=%s)' % self._transport)

    def _on_mavros_rtcm(self, msg):
        # Convert mavros_msgs/RTCM.data -> bytes and forward
        data = bytes(msg.data) if hasattr(msg, 'data') else b''
        self._forward_bytes(data)

    def _on_rtcm(self, msg):
        # rtcm_msgs/Message field name differs by package versions; try common fields
        if hasattr(msg, 'message'):
            data = bytes(msg.message)
        elif hasattr(msg, 'data'):
            data = bytes(msg.data)
        else:
            self.get_logger().warn('RTCM message had no data/message field; ignoring')
            return
        self._forward_bytes(data)

    def _forward_bytes(self, data: bytes):
        if not data:
            self.get_logger().debug('Empty RTCM payload, skipping')
            return

        # Prefer serial (if open)
        if self._serial:
            try:
                self._serial.write(data)
            except Exception as e:
                self.get_logger().error(f'Failed to write to serial: {e}')
        elif self._sock:
            try:
                # guard socket writes
                with self._sock_lock:
                    self._sock.sendall(data)
            except Exception as e:
                self.get_logger().error(f'Failed to send to socket: {e}')
        else:
            self.get_logger().warn('No transport open; drop RTCM')

    def destroy_node(self):
        try:
            if self._serial:
                self._serial.close()
                self.get_logger().info('Closed serial')
            if self._sock:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self._sock.close()
                self.get_logger().info('Closed socket')
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RtcmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
