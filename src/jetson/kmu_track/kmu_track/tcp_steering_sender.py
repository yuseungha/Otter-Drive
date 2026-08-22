"""Forward laptop preview steering to a Jetson over a guarded TCP link."""

from __future__ import annotations

import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

from rc_car_teleop.tcp_steering_core import encode_steering_packet


class TcpSteeringSender(Node):
    """Send only steering; throttle is deliberately absent from the wire."""

    def __init__(self) -> None:
        super().__init__('tcp_steering_sender')
        self.declare_parameter('host', '192.168.1.5')
        self.declare_parameter('port', 9100)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.30)
        self.declare_parameter('max_abs_steering', 650)
        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        rate = float(self.get_parameter('send_rate_hz').value)
        self._timeout = float(
            self.get_parameter('input_timeout_sec').value)
        self._maximum = int(
            self.get_parameter('max_abs_steering').value)
        if rate <= 0.0 or not 0.05 <= self._timeout < 0.40:
            raise ValueError('sender timing parameters are unsafe')
        self._socket = None
        self._last_connect_attempt = float('-inf')
        self._last_input_at = float('-inf')
        self._steering = 0
        self._sequence = 0
        self._status_pub = self.create_publisher(
            String, '/rc_car/tcp_sender_status', 10)
        self.create_subscription(
            Int32MultiArray,
            '/rc_car/drive_cmd_preview',
            self._on_preview,
            10,
        )
        self.create_timer(1.0 / rate, self._send_tick)
        self.get_logger().info(
            f'Steering-only sender targeting {self._host}:{self._port}')

    def _on_preview(self, message: Int32MultiArray) -> None:
        if len(message.data) != 3:
            return
        throttle, steering, gear = [int(value) for value in message.data]
        if throttle != 0 or gear != -1 or abs(steering) > self._maximum:
            self.get_logger().error(
                'Rejected unsafe preview command', throttle_duration_sec=1.0)
            return
        self._steering = steering
        self._last_input_at = time.monotonic()

    def _disconnect(self) -> None:
        connection = self._socket
        self._socket = None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    def _connect(self, now: float) -> None:
        if self._socket is not None or now - self._last_connect_attempt < 1.0:
            return
        self._last_connect_attempt = now
        try:
            connection = socket.create_connection(
                (self._host, self._port), timeout=0.30)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._socket = connection
            self.get_logger().info('Connected to Jetson steering receiver')
        except OSError:
            self._socket = None

    def _send_tick(self) -> None:
        now = time.monotonic()
        self._connect(now)
        fresh = now - self._last_input_at <= self._timeout
        steering = self._steering if fresh else 0
        connected = self._socket is not None
        if connected:
            self._sequence += 1
            try:
                self._socket.sendall(
                    encode_steering_packet(self._sequence, steering))
            except OSError:
                self._disconnect()
                connected = False
        status = (
            f'connected={str(connected).lower()} fresh={str(fresh).lower()} '
            f'seq={self._sequence} steering={steering}')
        self._status_pub.publish(String(data=status))

    def destroy_node(self) -> bool:
        if self._socket is not None:
            for _ in range(3):
                self._sequence += 1
                try:
                    self._socket.sendall(
                        encode_steering_packet(self._sequence, 0))
                    time.sleep(0.03)
                except OSError:
                    break
        self._disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the laptop-side steering sender."""
    rclpy.init(args=args)
    node = TcpSteeringSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
