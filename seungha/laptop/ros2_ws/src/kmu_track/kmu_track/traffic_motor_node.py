"""ROS adapter that maps traffic-light states to guarded drive commands."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32MultiArray, String

from kmu_track.traffic_motor_core import TrafficMotorController


class TrafficMotorNode(Node):
    """Publish preview or live RC-car commands from traffic-light states."""

    def __init__(self) -> None:
        super().__init__('traffic_motor_controller')
        self.declare_parameter('enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('hardware_confirmed', False)
        self.declare_parameter('require_serial_ready', True)
        self.declare_parameter('go_throttle', 150)
        self.declare_parameter('turn_throttle', 100)
        self.declare_parameter('left_steering', 350)
        self.declare_parameter('gear', -1)
        self.declare_parameter('signal_timeout_sec', 0.50)
        self.declare_parameter('speed_timeout_sec', 0.50)
        self.declare_parameter('stopped_speed_mps', 0.03)
        self.declare_parameter('center_fallback_delay_sec', 0.75)
        self.declare_parameter('throttle_ramp_per_sec', 300.0)
        self.declare_parameter('arming_neutral_sec', 1.0)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('manual_mode', False)
        self.declare_parameter(
            'manual_command_topic', '/rc_car/manual_drive_cmd')
        self.declare_parameter('manual_timeout_sec', 0.50)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        hardware_confirmed = bool(
            self.get_parameter('hardware_confirmed').value)
        requested_enabled = bool(self.get_parameter('enabled').value)
        self.manual_mode = bool(self.get_parameter('manual_mode').value)
        if not self.dry_run and not hardware_confirmed:
            raise RuntimeError(
                'live output requires hardware_confirmed:=true')

        self.controller = TrafficMotorController(
            enabled=requested_enabled,
            go_throttle=int(self.get_parameter('go_throttle').value),
            turn_throttle=int(self.get_parameter('turn_throttle').value),
            left_steering=int(self.get_parameter('left_steering').value),
            gear=int(self.get_parameter('gear').value),
            signal_timeout_sec=float(
                self.get_parameter('signal_timeout_sec').value),
            speed_timeout_sec=float(
                self.get_parameter('speed_timeout_sec').value),
            stopped_speed_mps=float(
                self.get_parameter('stopped_speed_mps').value),
            center_fallback_delay_sec=float(
                self.get_parameter('center_fallback_delay_sec').value),
            throttle_ramp_per_sec=float(
                self.get_parameter('throttle_ramp_per_sec').value),
            arming_neutral_sec=float(
                self.get_parameter('arming_neutral_sec').value),
            require_serial_ready=(
                False
                if self.dry_run
                else bool(self.get_parameter('require_serial_ready').value)
            ),
            manual_mode=self.manual_mode,
            manual_timeout_sec=float(
                self.get_parameter('manual_timeout_sec').value),
            clock=self._now_sec,
        )

        command_topic = (
            '/rc_car/drive_cmd_preview'
            if self.dry_run
            else '/rc_car/drive_cmd'
        )
        self.command_pub = self.create_publisher(
            Int32MultiArray, command_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/vehicle/traffic_motor_status', 10)
        self.create_subscription(
            String,
            '/perception/traffic_light_state',
            lambda msg: self.controller.update_signal(msg.data),
            10,
        )
        if self.manual_mode:
            self.create_subscription(
                Int32MultiArray,
                str(self.get_parameter('manual_command_topic').value),
                self._on_manual_command,
                10,
            )
        self.create_subscription(
            Float32,
            '/vehicle/speed_mps',
            lambda msg: self.controller.update_speed(msg.data),
            10,
        )
        self.create_subscription(
            Bool,
            '/rc_car/serial_connected',
            lambda msg: self.controller.update_serial_ready(msg.data),
            10,
        )
        rate = float(self.get_parameter('command_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('command_rate_hz must be positive')
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        mode = 'PREVIEW' if self.dry_run else 'LIVE'
        self.get_logger().info(
            f'Traffic motor controller: mode={mode}, '
            f'enabled={requested_enabled}, manual_gate={self.manual_mode}, '
            f'topic={command_topic}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_command(self, command) -> None:
        message = Int32MultiArray()
        message.data = [
            command.throttle,
            command.steering,
            command.gear,
        ]
        self.command_pub.publish(message)
        status = {
            'mode': 'preview' if self.dry_run else 'live',
            'signal': self.controller.signal,
            'throttle': command.throttle,
            'steering': command.steering,
            'gear': command.gear,
            'reason': command.reason,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _on_manual_command(self, message: Int32MultiArray) -> None:
        if len(message.data) not in (2, 3):
            self.get_logger().warn(
                'Ignoring manual command: expected '
                '[throttle, steering] or [throttle, steering, gear]',
                throttle_duration_sec=1.0,
            )
            return
        gear = message.data[2] if len(message.data) == 3 else -1
        self.controller.update_manual_command(
            message.data[0], message.data[1], gear)

    def _on_timer(self) -> None:
        command = self.controller.command()
        if command is not None:
            self._publish_command(command)

    def destroy_node(self) -> bool:
        # External shutdown can invalidate the ROS context before ``finally``
        # reaches this method. Publishing in that state raises RCLError and
        # hides the real shutdown reason, so only send the final neutral burst
        # while the context is still usable. The downstream serial controller
        # must retain its own command watchdog for process/power failures.
        if self.controller.enabled and rclpy.ok(context=self.context):
            for _ in range(3):
                self.controller.current_throttle = 0.0
                self.controller.current_steering = 0
                command = self.controller._neutral('node_shutdown')
                self._publish_command(command)
                time.sleep(0.05)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficMotorNode()
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
