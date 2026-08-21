"""Throttle-invariant, independently repeated left/right steering sweep."""

import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Int32, Int32MultiArray


BASE_STEPS = [0, 50, 65, 71, 73, 80, 100, 150, 200, 400, 1000, 0]


class SteeringSweep(Node):
    """Publish only zero-throttle steering steps and record ADC evidence."""

    def __init__(self):
        super().__init__('steering_sweep')
        self.declare_parameter('enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('hardware_confirmed', False)
        self.declare_parameter('require_serial_ready', True)
        self.declare_parameter('arming_neutral_sec', 1.0)
        self.declare_parameter('step_sec', 1.0)
        self.declare_parameter('repeats', 3)
        self.declare_parameter('max_counts_left', 600)
        self.declare_parameter('max_counts_right', 600)
        enabled = bool(self.get_parameter('enabled').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        if not self.dry_run and not bool(
            self.get_parameter('hardware_confirmed').value
        ):
            raise RuntimeError('live sweep requires hardware_confirmed:=true')
        self.enabled = enabled
        self.require_serial_ready = bool(
            self.get_parameter('require_serial_ready').value) and not self.dry_run
        self.serial_ready = False
        self.serial_ready_at = None
        topic = (
            '/rc_car/drive_cmd_preview' if self.dry_run
            else '/rc_car/drive_cmd')
        self.publisher = self.create_publisher(Int32MultiArray, topic, 10)
        self.create_subscription(
            Int32, '/rc_car/steering_adc', self._on_adc, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/rc_car/serial_ready', self._on_serial_ready, state_qos)
        self.arming_neutral_sec = max(
            0.0, float(self.get_parameter('arming_neutral_sec').value))
        self.step_sec = max(0.1, float(self.get_parameter('step_sec').value))
        repeats = max(1, int(self.get_parameter('repeats').value))
        left_max = max(1, int(self.get_parameter('max_counts_left').value))
        right_max = max(1, int(self.get_parameter('max_counts_right').value))
        left = [min(value, left_max) for value in BASE_STEPS]
        right = [-min(value, right_max) for value in BASE_STEPS]
        self.steps = [
            (repeat + 1, side, command)
            for repeat in range(repeats)
            for side, commands in (('LEFT', left), ('RIGHT', right))
            for command in commands
        ]
        self.index = -1
        self.step_started_at = None
        self.start_adc = None
        self.latest_adc = None
        self.latest_adc_at = None
        self.motion_at = None
        self.settle_at = None
        self.rows = []
        Path('runs').mkdir(exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_path = Path('runs') / f'steering_sweep_{stamp}.csv'
        self.timer = self.create_timer(0.05, self._tick)
        if not self.enabled:
            self.get_logger().warn(
                'Sweep disabled by default; no commands will be published')

    def _on_adc(self, message):
        now = time.monotonic()
        self.latest_adc = int(message.data)
        self.latest_adc_at = now
        if self.step_started_at is None or self.start_adc is None:
            return
        command = self.steps[self.index][2]
        if (
            self.motion_at is None
            and abs(self.latest_adc - self.start_adc) > 7
        ):
            self.motion_at = now
        expected = self._expected_adc(command)
        if self.settle_at is None and abs(expected - self.latest_adc) <= 7:
            self.settle_at = now

    def _on_serial_ready(self, message):
        ready = bool(message.data)
        now = time.monotonic()
        if ready and not self.serial_ready:
            self.serial_ready_at = now
        if not ready:
            self.serial_ready_at = None
        self.serial_ready = ready

    @staticmethod
    def _expected_adc(command):
        if command >= 0:
            return int(command * (712 - 601) / 1000) + 601
        return int(
            (command + 1000) * (601 - 488) / 1000
        ) + 488

    def _publish_command(self, steering):
        # Throttle is deliberately hard-coded; there is no throttle parameter.
        self.publisher.publish(Int32MultiArray(data=[0, int(steering), -1]))

    def _finish_step(self, now):
        if self.index < 0:
            return
        repeat, side, command = self.steps[self.index]
        final_adc = self.latest_adc
        delta = None if final_adc is None or self.start_adc is None else (
            final_adc - self.start_adc)
        self.rows.append({
            'repeat': repeat,
            'side': side,
            'command': command,
            'start_adc': self.start_adc,
            'final_adc': final_adc,
            'delta_adc': delta,
            'moved': bool(delta is not None and abs(delta) > 7),
            'motion_ms': (
                None if self.motion_at is None
                else round((self.motion_at - self.step_started_at) * 1000, 1)),
            'settle_ms': (
                None if self.settle_at is None
                else round((self.settle_at - self.step_started_at) * 1000, 1)),
        })

    def _tick(self):
        if not self.enabled:
            return
        now = time.monotonic()
        if self.require_serial_ready and (
            not self.serial_ready
            or self.serial_ready_at is None
            or now - self.serial_ready_at < self.arming_neutral_sec
        ):
            self._publish_command(0)
            return
        if self.step_started_at is not None and (
            now - self.step_started_at < self.step_sec
        ):
            return
        self._finish_step(now)
        self.index += 1
        if self.index >= len(self.steps):
            self._publish_command(0)
            self._write_results()
            self.enabled = False
            self.get_logger().info(f'Sweep complete: {self.output_path}')
            return
        _, side, command = self.steps[self.index]
        self.step_started_at = now
        self.start_adc = self.latest_adc
        self.motion_at = None
        self.settle_at = None
        self._publish_command(command)
        self.get_logger().info(
            f'step {self.index + 1}/{len(self.steps)} {side} {command:+d}')

    def _write_results(self):
        fields = list(self.rows[0]) if self.rows else []
        with self.output_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        summary = {}
        for side in ('LEFT', 'RIGHT'):
            side_rows = [row for row in self.rows if row['side'] == side]
            thresholds = []
            for repeat in sorted({row['repeat'] for row in side_rows}):
                moved = [
                    abs(row['command']) for row in side_rows
                    if row['repeat'] == repeat and row['moved']
                    and row['command'] != 0]
                if moved:
                    thresholds.append(min(moved))
            scales = [
                abs(row['final_adc'] - 601)
                * 1000
                / abs(row['command'])
                for row in side_rows
                if row['final_adc'] is not None and row['command'] not in (0,)
            ]
            settles = [
                row['settle_ms'] for row in side_rows
                if row['settle_ms'] is not None]
            direction_samples = [
                row['final_adc'] - row['start_adc']
                for row in side_rows
                if row['final_adc'] is not None and row['start_adc'] is not None
                and row['command'] != 0 and row['moved']]
            direction_delta = (
                statistics.median(direction_samples)
                if direction_samples else 0)
            summary[side] = {
                'deadband_command_median': (
                    statistics.median(thresholds) if thresholds else None),
                'adc_counts_per_1000_median': (
                    statistics.median(scales) if scales else None),
                'settle_ms_median': (
                    statistics.median(settles) if settles else None),
                'sign_observed': (
                    'ADC_INCREASE' if direction_delta > 0 else (
                        'ADC_DECREASE' if direction_delta < 0 else 'NO_MOTION')),
            }
        summary_path = self.output_path.with_suffix('.summary.json')
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')

    def destroy_node(self):
        if self.enabled:
            for _ in range(3):
                self._publish_command(0)
                time.sleep(0.05)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringSweep()
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
