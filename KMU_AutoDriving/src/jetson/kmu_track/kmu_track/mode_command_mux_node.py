"""Fail-closed command mux: only the active mission controller can actuate."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32MultiArray, String

from kmu_track.drive_mode_core import DriveMode, selected_command_source


class ModeCommandMux(Node):
    def __init__(self) -> None:
        super().__init__('mode_command_mux')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('output_rate_hz', 20.0)
        self.declare_parameter('lane_command_topic', '/vehicle/lane_drive_cmd')
        self.declare_parameter('cone_command_topic', '/vehicle/cone_drive_cmd')
        self.declare_parameter(
            'preview_output_topic', '/rc_car/drive_cmd_preview')
        self.declare_parameter('live_output_topic', '/rc_car/drive_cmd')
        self._timeout = float(self.get_parameter('command_timeout_sec').value)
        rate = float(self.get_parameter('output_rate_hz').value)
        if self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError(
                'command timeout and output rate must be positive')
        output_topic = str(self.get_parameter(
            'preview_output_topic' if bool(self.get_parameter('dry_run').value)
            else 'live_output_topic').value)
        self._publisher = self.create_publisher(
            Int32MultiArray, output_topic, 10)
        self._status_pub = self.create_publisher(
            String, '/vehicle/command_mux_status', 10)
        self._mode = DriveMode.SAFE_STOP
        self._stop_requested = True
        self._commands = {'lane': None, 'cone': None}

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/mission/state', self._on_mode, latched)
        self.create_subscription(
            Bool, '/vehicle/stop_requested', self._on_stop, latched)
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter('lane_command_topic').value),
            lambda message: self._on_command('lane', message), 10)
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter('cone_command_topic').value),
            lambda message: self._on_command('cone', message), 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Mode command mux ready: output={output_topic}')

    def _publish_neutral(self) -> None:
        self._publisher.publish(Int32MultiArray(data=[0, 0]))

    def _on_mode(self, message: String) -> None:
        try:
            mode = DriveMode(str(message.data).strip().upper())
        except ValueError:
            mode = DriveMode.SAFE_STOP
        if mode != self._mode:
            self._commands = {'lane': None, 'cone': None}
            self._publish_neutral()
        self._mode = mode

    def _on_stop(self, message: Bool) -> None:
        self._stop_requested = bool(message.data)
        if self._stop_requested:
            self._publish_neutral()

    def _on_command(self, source: str, message: Int32MultiArray) -> None:
        if len(message.data) != 2:
            return
        self._commands[source] = (
            [int(message.data[0]), int(message.data[1])], time.monotonic())

    def _tick(self) -> None:
        source = selected_command_source(self._mode, self._stop_requested)
        record = self._commands.get(source)
        reason = 'transition_or_stop'
        command = [0, 0]
        if record is not None:
            values, received_at = record
            if time.monotonic() - received_at <= self._timeout:
                command = values
                reason = 'active_fresh'
            else:
                reason = 'active_command_stale'
        elif source in {'lane', 'cone'}:
            reason = 'active_command_missing'
        self._publisher.publish(Int32MultiArray(data=command))
        self._status_pub.publish(String(data=(
            f'{{"state":"{self._mode.value}","source":"{source}",'
            f'"reason":"{reason}","throttle":{command[0]},'
            f'"steering":{command[1]}}}')))

    def destroy_node(self) -> bool:
        if rclpy.ok(context=self.context):
            for _ in range(3):
                self._publish_neutral()
                time.sleep(0.02)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeCommandMux()
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
