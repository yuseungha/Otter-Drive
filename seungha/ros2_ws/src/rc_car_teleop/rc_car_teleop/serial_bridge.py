#!/usr/bin/env python3
"""Bridge atomic ROS drive commands to the Arduino over USB serial."""

import re
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int32MultiArray, String

THROTTLE_MIN = -1000
THROTTLE_MAX = 1050
STEERING_MIN = -1000
STEERING_MAX = 1000
GEAR_LOW = -1
GEAR_HIGH = 1
MAX_RX_BUFFER = 4096

DEBUG_PATTERN = re.compile(
    r'\[DEBUG\] Throttle: (-?\d+) \| SteerTarget\(ADC\): (-?\d+) '
    r'\| SteerCurrent\(ADC\): (-?\d+) \| Error: (-?\d+)')


class SerialBridge(Node):
    """ROS-to-serial bridge with persistent commands and automatic reconnect."""

    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('reset_guard_sec', 3.2)

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._send_rate = float(self.get_parameter('send_rate_hz').value)
        self._reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self._reset_guard = float(self.get_parameter('reset_guard_sec').value)
        if self._send_rate <= 0.0:
            raise ValueError('send_rate_hz must be greater than zero')
        if self._reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval_sec must be greater than zero')
        if self._reset_guard < 0.0:
            raise ValueError('reset_guard_sec cannot be negative')

        self._serial = None
        self._rx_buffer = bytearray()
        self._throttle = 0
        self._steering = 0
        self._gear = GEAR_LOW
        self._last_connect_attempt_at = float('-inf')
        self._connected_at = None
        self._source_neutral_seen = False

        self.create_subscription(
            Int32MultiArray, '/rc_car/drive_cmd', self._drive_callback, 10)
        self._feedback_pub = self.create_publisher(
            String, '/rc_car/feedback', 10)
        self._steering_adc_pub = self.create_publisher(
            Int32, '/rc_car/steering_adc', 10)
        self._steering_error_pub = self.create_publisher(
            Int32, '/rc_car/steering_error', 10)
        self._serial_connected_pub = self.create_publisher(
            Bool, '/rc_car/serial_connected', 10)

        self.create_timer(1.0 / self._send_rate, self._io_tick)
        self.get_logger().info(
            f'Serial bridge starting: {self._port} @ {self._baud}')

    @staticmethod
    def _limit_throttle(value):
        return max(THROTTLE_MIN, min(THROTTLE_MAX, int(value)))

    @staticmethod
    def _limit_steering(value):
        return max(STEERING_MIN, min(STEERING_MAX, int(value)))

    def _drive_callback(self, message):
        if len(message.data) not in (2, 3):
            self.get_logger().warn(
                'Ignoring drive_cmd: expected '
                '[throttle, steering] or [throttle, steering, gear]',
                throttle_duration_sec=1.0)
            return
        if len(message.data) == 3:
            gear = int(message.data[2])
            if gear not in (GEAR_LOW, GEAR_HIGH):
                self.get_logger().warn(
                    'Ignoring drive_cmd: gear must be -1 (LOW) or 1 (HIGH)',
                    throttle_duration_sec=1.0)
                return
            self._gear = gear
        self._throttle = self._limit_throttle(message.data[0])
        self._steering = self._limit_steering(message.data[1])
        if self._throttle == 0 and self._steering == 0:
            self._source_neutral_seen = True

    def _publish_connected(self, connected):
        message = Bool()
        message.data = connected
        self._serial_connected_pub.publish(message)

    def _connect_if_due(self, now):
        if self._serial is not None:
            return
        if now - self._last_connect_attempt_at < self._reconnect_interval:
            return
        self._last_connect_attempt_at = now
        try:
            connection = serial.Serial(
                self._port,
                self._baud,
                timeout=0,
                write_timeout=0.10)
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(
                f'Cannot open {self._port}: {error}; retrying',
                throttle_duration_sec=5.0)
            return

        self._serial = connection
        self._rx_buffer.clear()
        self._connected_at = now
        # Requiring a new source-neutral after every reconnect prevents an old
        # keyboard speed from resuming when opening USB resets the Arduino.
        self._source_neutral_seen = False
        self._throttle = 0
        self._steering = 0
        self._gear = GEAR_LOW
        self._publish_connected(True)
        self.get_logger().info(
            f'Serial connected: {self._port} @ {self._baud}; '
            f'forcing neutral for {self._reset_guard:.1f}s')

    def _disconnect(self, reason):
        connection = self._serial
        self._serial = None
        self._connected_at = None
        self._rx_buffer.clear()
        self._source_neutral_seen = False
        self._throttle = 0
        self._steering = 0
        self._gear = GEAR_LOW
        if connection is not None:
            try:
                connection.close()
            except (serial.SerialException, OSError):
                pass
        self._publish_connected(False)
        self.get_logger().error(f'Serial disconnected: {reason}')

    def _command_to_send(self, now):
        guard_active = (
            self._connected_at is None
            or now - self._connected_at < self._reset_guard)

        # During USB reset/arming, or until the keyboard supplies a full
        # neutral frame, send no valid D frame. This keeps Arduino steering
        # disabled instead of making it move to center before teleop starts.
        if guard_active or not self._source_neutral_seen:
            return None
        return self._throttle, self._steering, self._gear

    def _write_command(self, now):
        command = self._command_to_send(now)
        if command is None:
            return
        throttle, steering, gear = command
        try:
            self._serial.write(
                f'D{throttle} {steering} {gear}\n'.encode('ascii'))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as error:
            self._disconnect(f'write failed: {error}')

    def _read_available(self):
        try:
            waiting = self._serial.in_waiting
            if waiting:
                self._rx_buffer.extend(self._serial.read(waiting))
        except (serial.SerialException, OSError) as error:
            self._disconnect(f'read failed: {error}')
            return

        if len(self._rx_buffer) > MAX_RX_BUFFER:
            self.get_logger().warn('Serial receive buffer overflow; clearing')
            self._rx_buffer.clear()
            return

        while b'\n' in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            line = raw_line.rstrip(b'\r').decode('utf-8', errors='replace')
            if line:
                self._handle_line(line)

    def _handle_line(self, line):
        feedback = String()
        feedback.data = line
        self._feedback_pub.publish(feedback)

        match = DEBUG_PATTERN.search(line)
        if not match:
            return
        _, _, current_adc, steering_error = map(int, match.groups())
        adc = Int32()
        adc.data = current_adc
        error = Int32()
        error.data = steering_error
        self._steering_adc_pub.publish(adc)
        self._steering_error_pub.publish(error)

    def _io_tick(self):
        now = time.monotonic()
        self._connect_if_due(now)
        if self._serial is None:
            return
        # Nodes in a launch start concurrently. Re-publish the ready state so
        # a safety gate that subscribes just after the initial connection does
        # not remain stuck in serial_not_ready.
        self._publish_connected(True)
        self._read_available()
        if self._serial is not None:
            self._write_command(now)

    def destroy_node(self):
        if self._serial is not None:
            try:
                for _ in range(3):
                    self._serial.write(b'D0 0\n')
                self._serial.flush()
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                pass
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
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
