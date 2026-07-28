"""PID steering controller with lane validity, deadman, and E-stop interlocks."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class Drive(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.declare_parameter('control_enable_topic', '/control/enable')
        self.declare_parameter('emergency_stop_topic', '/control/emergency_stop')
        self.declare_parameter('deadman_timeout_sec', 0.5)
        self.declare_parameter('speed_mps', 0.35)
        self.declare_parameter('input_timeout_sec', 0.5)
        self._error = self._integral = self._previous_error = 0.0
        self._last_error = None
        self._last_enable = None
        self._lane_valid = False
        self._enabled = False
        self._emergency_stop = False
        self._deadman_timeout = float(self.get_parameter('deadman_timeout_sec').value)
        self._input_timeout = float(self.get_parameter('input_timeout_sec').value)
        self._speed = float(self.get_parameter('speed_mps').value)

        self.create_subscription(Float32, '/lane_detection/steering_error', self._error_callback, 10)
        self.create_subscription(Bool, '/lane_detection/lane_valid', self._lane_valid_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter('control_enable_topic').value), self._enable_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter('emergency_stop_topic').value), self._estop_callback, 10)
        self._command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._active_publisher = self.create_publisher(Bool, '/control/active', 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Drive interlocks active: lane validity + deadman + E-stop')

    def _error_callback(self, message):
        self._error = message.data
        self._last_error = self.get_clock().now()
        # A fresh validity result must accompany every new error sample.
        self._lane_valid = False

    def _lane_valid_callback(self, message):
        self._lane_valid = message.data

    def _enable_callback(self, message):
        self._enabled = message.data
        if message.data:
            self._last_enable = self.get_clock().now()

    def _estop_callback(self, message):
        self._emergency_stop = message.data
        if message.data:
            self._enabled = False
            self._reset_controller()

    def _reset_controller(self):
        self._integral = 0.0
        self._previous_error = 0.0

    def _deadman_is_fresh(self, now):
        return self._enabled and self._last_enable is not None and (
            now - self._last_enable
        ).nanoseconds < int(self._deadman_timeout * 1e9)

    def _input_is_fresh(self, now):
        return self._last_error is not None and (
            now - self._last_error
        ).nanoseconds < int(self._input_timeout * 1e9)

    def _tick(self):
        command = Twist()
        now = self.get_clock().now()
        active = (
            not self._emergency_stop
            and self._deadman_is_fresh(now)
            and self._lane_valid
            and self._input_is_fresh(now)
        )
        if active:
            self._integral = max(-1.0, min(1.0, self._integral + self._error * 0.05))
            steering = max(
                -0.8,
                min(0.8, 1.2 * self._error + 0.05 * self._integral + 0.12 * (self._error - self._previous_error) / 0.05),
            )
            command.linear.x = self._speed
            command.angular.z = steering
            self._previous_error = self._error
        else:
            self._reset_controller()
        self._command_publisher.publish(command)
        status = Bool()
        status.data = active
        self._active_publisher.publish(status)


def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
