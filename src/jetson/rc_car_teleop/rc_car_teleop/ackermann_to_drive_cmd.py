"""Convert stamped cone-controller SI commands to the two-count protocol."""

import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from rc_car_teleop.autonomous_drive_core import (
    AutonomousDriveConfig,
    convert_command,
)


class AutonomousDriveAdapter(Node):
    def __init__(self) -> None:
        super().__init__('ackermann_to_drive_cmd')
        defaults = AutonomousDriveConfig()
        self.declare_parameter(
            'input_topic', 'cone_controller/command_vector')
        self.declare_parameter('output_topic', '/vehicle/cone_drive_cmd')
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('publish_rate_hz', 20.0)
        for name in AutonomousDriveConfig.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self._config = AutonomousDriveConfig(**{
            name: self.get_parameter(name).value
            for name in AutonomousDriveConfig.__dataclass_fields__
        })
        self._config.validate()
        self._timeout = float(self.get_parameter('command_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeouts and rates must be positive')
        self._publisher = self.create_publisher(
            Int32MultiArray,
            str(self.get_parameter('output_topic').value), 10)
        self._last_received = None
        self._last_command = (0, 0)
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter('input_topic').value), self._on_command, 10)
        self.create_timer(1.0 / rate, self._tick)

    def _on_command(self, message: Vector3Stamped) -> None:
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - stamp_ns) * 1.0e-9
        if stamp_ns <= 0 or age < -0.05 or age > self._timeout:
            self._last_command = (0, 0)
            return
        self._last_command = convert_command(
            float(message.vector.x), float(message.vector.y), self._config)
        self._last_received = time.monotonic()

    def _tick(self) -> None:
        command = self._last_command
        if (self._last_received is None
                or time.monotonic() - self._last_received > self._timeout):
            command = (0, 0)
        self._publisher.publish(Int32MultiArray(data=list(command)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousDriveAdapter()
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
