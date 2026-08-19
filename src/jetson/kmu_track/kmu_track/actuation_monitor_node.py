"""Correlate lane commands, actual serial TX, and steering feedback."""

from collections import deque
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, Int32MultiArray, String


ADC_LEFT = 712
ADC_CENTER = 601
ADC_RIGHT = 488
STEER_DEADBAND = 7


def _arduino_map(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (
        in_max - in_min)) + out_min


def steering_command_to_adc(command):
    command = max(-1000, min(1000, int(command)))
    if command >= 0:
        return _arduino_map(command, 0, 1000, ADC_CENTER, ADC_LEFT)
    return _arduino_map(command, -1000, 0, ADC_RIGHT, ADC_CENTER)


class ActuationMonitor(Node):
    """Publish evidence without estimating serial TX from ROS topic rates."""

    def __init__(self):
        super().__init__('actuation_monitor')
        self.declare_parameter('dry_run', True)
        self._commanded = 0
        self._deadband = 0
        self._side = 'CENTER'
        self._expected_adc = ADC_CENTER
        self._measured_adc = None
        self._steering_error = None
        self._feedback_at = None
        self._command_changed_at = None
        self._command_start_adc = None
        self._motion_ms = None
        self._settle_ms = None
        self._tx_stats = {}
        self._command_times = deque()
        self._serial_connected = False
        self._serial_ready = False
        self._command_stale = False
        self._estop_latched = False
        self._lane_valid = False
        self._target_source = 'UNKNOWN'
        self._fault_flags = []

        dry_run = bool(self.get_parameter('dry_run').value)
        command_topic = (
            '/rc_car/drive_cmd_preview' if dry_run else '/rc_car/drive_cmd')
        self.create_subscription(
            Int32MultiArray, command_topic, self._on_command, 10)
        self.create_subscription(
            Int32, '/rc_car/steering_adc', self._on_adc, 10)
        self.create_subscription(
            Int32, '/rc_car/steering_error', self._on_error, 10)
        self.create_subscription(
            String, '/rc_car/tx_stats', self._on_tx_stats, 10)
        self.create_subscription(
            String, '/rc_car/feedback', self._on_feedback, 10)
        self.create_subscription(
            String, '/vehicle/lane_control_status', self._on_control, 10)
        self.create_subscription(
            Bool, '/lane/valid', lambda msg: setattr(
                self, '_lane_valid', bool(msg.data)), 10)
        self.create_subscription(
            String, '/lane/lane_geometry', self._on_geometry, 10)
        self.create_subscription(
            Bool, '/rc_car/serial_connected', lambda msg: setattr(
                self, '_serial_connected', bool(msg.data)), 10)
        self.create_subscription(
            Bool, '/rc_car/command_stale', lambda msg: setattr(
                self, '_command_stale', bool(msg.data)), 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/rc_car/serial_ready', lambda msg: setattr(
                self, '_serial_ready', bool(msg.data)), state_qos)
        self.create_subscription(
            Bool, '/vehicle/estop_latched', lambda msg: setattr(
                self, '_estop_latched', bool(msg.data)), state_qos)
        self._publisher = self.create_publisher(
            String, '/vehicle/actuation_status', 10)
        self.create_timer(0.1, self._publish)

    @staticmethod
    def _now():
        return time.monotonic()

    def _on_command(self, message):
        if len(message.data) < 2:
            return
        now = self._now()
        self._command_times.append(now)
        steering = int(message.data[1])
        if steering != self._commanded:
            self._commanded = steering
            self._expected_adc = steering_command_to_adc(steering)
            self._command_changed_at = now
            self._command_start_adc = self._measured_adc
            self._motion_ms = None
            self._settle_ms = None

    def _on_adc(self, message):
        now = self._now()
        self._measured_adc = int(message.data)
        self._feedback_at = now
        if self._command_changed_at is None:
            return
        elapsed_ms = (now - self._command_changed_at) * 1000.0
        if (
            self._motion_ms is None
            and self._command_start_adc is not None
            and abs(self._measured_adc - self._command_start_adc)
            > STEER_DEADBAND
        ):
            self._motion_ms = elapsed_ms
        if (
            self._settle_ms is None
            and abs(self._expected_adc - self._measured_adc)
            <= STEER_DEADBAND
        ):
            self._settle_ms = elapsed_ms

    def _on_error(self, message):
        self._steering_error = int(message.data)

    def _on_tx_stats(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            self._tx_stats = data

    def _on_control(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self._deadband = int(data.get('deadband_applied', 0))
        self._side = str(data.get('side', 'CENTER'))

    def _on_geometry(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self._target_source = str(data.get('target_source', 'UNKNOWN'))

    def _on_feedback(self, message):
        line = str(message.data)
        flags = []
        if 'Fault=YES' in line:
            flags.append('STEERING_FAULT')
        if '[WATCHDOG]' in line:
            flags.append('FIRMWARE_WATCHDOG')
        if '[ERROR]' in line:
            flags.append('FIRMWARE_ERROR')
        if flags:
            self._fault_flags = flags

    def _publish(self):
        now = self._now()
        while self._command_times and now - self._command_times[0] > 1.0:
            self._command_times.popleft()
        measured = self._measured_adc
        data = {
            'commanded_steering': self._commanded,
            'side': self._side,
            'deadband_applied': self._deadband,
            'expected_adc': self._expected_adc,
            'measured_adc': measured,
            'adc_error': (
                None if measured is None else self._expected_adc - measured),
            'command_to_motion_ms': self._motion_ms,
            'settle_ms': self._settle_ms,
            'timing_budget_ms': 167.0,
            'tx_hz': self._tx_stats.get('tx_hz', 0.0),
            'cmd_pub_hz': float(len(self._command_times)),
            'feedback_age_ms': (
                None if self._feedback_at is None
                else (now - self._feedback_at) * 1000.0),
            'serial_connected': self._serial_connected,
            'serial_ready': self._serial_ready,
            'command_stale': self._command_stale,
            'estop_latched': self._estop_latched,
            'fault_flags': self._fault_flags,
            'lane_valid': self._lane_valid,
            'target_source': self._target_source,
            'tx_stats': self._tx_stats,
        }
        message = String()
        message.data = json.dumps(data, sort_keys=True)
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ActuationMonitor()
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
