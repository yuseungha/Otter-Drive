"""Receive steering-only TCP packets and publish guarded ROS commands."""

from __future__ import annotations

import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32MultiArray, String

from rc_car_teleop.tcp_steering_core import (
    decode_steering_packet,
    select_forwarded_steering,
)


class TcpSteeringReceiver(Node):
    """Jetson-side TCP receiver with a mandatory stale-to-neutral gate."""

    def __init__(self) -> None:
        super().__init__('tcp_steering_receiver')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 9100)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('max_abs_steering', 600)
        self._bind_host = str(self.get_parameter('bind_host').value)
        self._port = int(self.get_parameter('port').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timeout = float(
            self.get_parameter('command_timeout_sec').value)
        self._maximum = int(
            self.get_parameter('max_abs_steering').value)
        if not 1 <= self._port <= 65535:
            raise ValueError('port must be in 1..65535')
        if rate <= 0.0 or not 0.05 <= self._timeout < 0.40:
            raise ValueError('receiver timing parameters are unsafe')
        if not 1 <= self._maximum <= 1000:
            raise ValueError('max_abs_steering must be in 1..1000')

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = False
        self._serial_ready = False
        self._steering = 0
        self._last_received_at = float('-inf')
        self._last_sequence = -1
        self._server = self._open_server()
        self._client = None
        self._invalid_packets = 0

        self._command_pub = self.create_publisher(
            Int32MultiArray, '/rc_car/drive_cmd', 10)
        self._connected_pub = self.create_publisher(
            Bool, '/rc_car/tcp_steering_connected', 10)
        self._status_pub = self.create_publisher(
            String, '/rc_car/tcp_steering_status', 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            '/rc_car/serial_ready',
            self._on_serial_ready,
            state_qos,
        )
        self.create_timer(1.0 / rate, self._publish_command)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Steering-only TCP receiver listening on '
            f'{self._bind_host}:{self._port}; timeout={self._timeout:.2f}s')

    def _on_serial_ready(self, message: Bool) -> None:
        """Gate TCP steering until the local serial bridge is armed."""
        self._serial_ready = bool(message.data)

    def _open_server(self) -> socket.socket:
        """Bind before publishing commands so a duplicate instance fails safe."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((self._bind_host, self._port))
            server.listen(1)
            server.settimeout(0.50)
        except OSError as error:
            server.close()
            self.get_logger().fatal(
                f'Cannot bind {self._bind_host}:{self._port}: {error}; '
                'another steering receiver may already be running')
            raise RuntimeError(
                f'steering receiver port {self._port} is unavailable') \
                from error
        return server

    def _set_client(self, client) -> None:
        with self._lock:
            self._client = client
            self._connected = client is not None
            self._last_sequence = -1
            if client is None:
                self._steering = 0
                self._last_received_at = float('-inf')

    def _serve_client(self, client: socket.socket) -> None:
        client.settimeout(0.20)
        buffer = bytearray()
        while not self._stop.is_set():
            try:
                chunk = client.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buffer.extend(chunk)
            if len(buffer) > 4096:
                buffer.clear()
                self._invalid_packets += 1
                continue
            while b'\n' in buffer:
                line, _, remainder = buffer.partition(b'\n')
                buffer = bytearray(remainder)
                try:
                    sequence, steering = decode_steering_packet(
                        line, self._maximum)
                except ValueError:
                    self._invalid_packets += 1
                    continue
                with self._lock:
                    if sequence <= self._last_sequence:
                        self._invalid_packets += 1
                        continue
                    self._last_sequence = sequence
                    self._steering = steering
                    self._last_received_at = time.monotonic()

    def _serve(self) -> None:
        server = self._server
        try:
            while not self._stop.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._set_client(client)
                self.get_logger().info(
                    f'Steering client connected: {address[0]}')
                try:
                    self._serve_client(client)
                finally:
                    self._set_client(None)
                    try:
                        client.close()
                    except OSError:
                        pass
                    if not self._stop.is_set():
                        self.get_logger().warn(
                            'Steering client disconnected; forcing neutral')
        finally:
            try:
                server.close()
            except OSError:
                pass

    def _publish_command(self) -> None:
        now = time.monotonic()
        with self._lock:
            connected = self._connected
            age = now - self._last_received_at
            fresh = connected and age <= self._timeout
            serial_ready = self._serial_ready
            steering = select_forwarded_steering(
                self._steering,
                connected=connected,
                fresh=fresh,
                serial_ready=serial_ready,
            )
            sequence = self._last_sequence
            invalid = self._invalid_packets
        self._command_pub.publish(
            Int32MultiArray(data=[0, int(steering), -1]))
        self._connected_pub.publish(Bool(data=connected))
        status = (
            f'connected={str(connected).lower()} fresh={str(fresh).lower()} '
            f'serial_ready={str(serial_ready).lower()} '
            f'seq={sequence} steering={steering} '
            f'age_ms={max(0.0, age) * 1000.0:.1f} invalid={invalid}')
        self._status_pub.publish(String(data=status))

    def destroy_node(self) -> bool:
        for _ in range(3):
            self._command_pub.publish(Int32MultiArray(data=[0, 0, -1]))
            time.sleep(0.05)
        self._stop.set()
        for connection in (self._client, self._server):
            try:
                if connection is not None:
                    connection.close()
            except OSError:
                pass
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the Jetson-side steering receiver."""
    rclpy.init(args=args)
    node = TcpSteeringReceiver()
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
