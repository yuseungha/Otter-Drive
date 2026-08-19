"""Correlate the complete steering command chain once per second."""

from __future__ import annotations

import json
import math
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32MultiArray, String

from rc_car_teleop.firmware_model import STEER_DEADBAND


def _parse_key_value_status(value: str) -> dict[str, str]:
    """Parse the whitespace-separated TCP receiver status payload."""
    fields: dict[str, str] = {}
    for token in value.split():
        if '=' in token:
            key, field_value = token.split('=', 1)
            fields[key] = field_value
    return fields


def _counter_delta(
    current: Optional[dict[str, Any]],
    previous: Optional[dict[str, Any]],
    key: str,
) -> Optional[int]:
    if current is None or previous is None:
        return None
    try:
        return int(current.get(key, 0)) - int(previous.get(key, 0))
    except (TypeError, ValueError):
        return None


class SteeringChainProbe(Node):
    """Expose the first failing boundary after TCP steering reception."""

    def __init__(self) -> None:
        super().__init__('steering_chain_probe')
        self._tcp: Optional[dict[str, str]] = None
        self._steering: Optional[dict[str, Any]] = None
        self._tx: Optional[dict[str, Any]] = None
        self._firmware_info: Optional[dict[str, Any]] = None
        self._previous_tx: Optional[dict[str, Any]] = None
        self._commanded_steering = 0
        self._serial_ready: Optional[bool] = None
        self._previous_abs_error: Optional[int] = None

        self.create_subscription(
            String, '/rc_car/tcp_steering_status', self._on_tcp, 10)
        self.create_subscription(
            String, '/rc_car/steering_status', self._on_steering, 10)
        self.create_subscription(String, '/rc_car/tx_stats', self._on_tx, 10)
        self.create_subscription(
            Int32MultiArray, '/rc_car/drive_cmd', self._on_command, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/rc_car/serial_ready', self._on_serial_ready, state_qos)
        self.create_subscription(
            String,
            '/rc_car/firmware_info',
            self._on_firmware_info,
            state_qos,
        )
        self.create_timer(1.0, self._report)

    def _on_tcp(self, message: String) -> None:
        self._tcp = _parse_key_value_status(message.data)

    def _on_steering(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Invalid /rc_car/steering_status JSON',
                throttle_duration_sec=1.0,
            )
            return
        if isinstance(value, dict):
            self._steering = value

    def _on_tx(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Invalid /rc_car/tx_stats JSON',
                throttle_duration_sec=1.0,
            )
            return
        if isinstance(value, dict):
            self._tx = value

    def _on_command(self, message: Int32MultiArray) -> None:
        if len(message.data) >= 2:
            self._commanded_steering = int(message.data[1])

    def _on_serial_ready(self, message: Bool) -> None:
        self._serial_ready = bool(message.data)

    def _on_firmware_info(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self._firmware_info = value

    def _suppressed_deltas(self) -> dict[str, int]:
        if self._tx is None or self._previous_tx is None:
            return {}
        deltas = {}
        for key in sorted(self._tx):
            if not key.startswith('suppressed_'):
                continue
            delta = _counter_delta(self._tx, self._previous_tx, key)
            if delta is not None and delta > 0:
                deltas[key] = delta
        return deltas

    def _verdict(
        self,
        frames_delta: Optional[int],
        suppressed: dict[str, int],
    ) -> str:
        if self._serial_ready is False and frames_delta == 0:
            detail = ','.join(
                f'{key}=+{value}' for key, value in suppressed.items())
            if 'suppressed_no_source_neutral' in suppressed:
                detail = f'중립 인터록 교착; {detail}'
            return f'BRIDGE_NOT_ARMED ({detail or "suppressed 증가 없음"})'

        if (
            self._firmware_info is not None
            and self._firmware_info.get('model_match') is False
        ):
            return 'FIRMWARE_MODEL_MISMATCH'

        if self._steering is not None and bool(
            self._steering.get('fault', False)
        ):
            return 'FIRMWARE_FAULT_LATCHED'

        if frames_delta == 0:
            return 'BRIDGE_NOT_WRITING'

        if self._steering is None:
            return 'NO_FIRMWARE_FEEDBACK(펌웨어/시리얼 확인)'

        target = int(self._steering.get('target_adc', 0))
        current = int(self._steering.get('current_adc', 0))
        error = abs(target - current)
        pwm = int(self._steering.get('pwm', 0))
        if self._commanded_steering != 0 and error <= STEER_DEADBAND:
            return 'BELOW_FIRMWARE_DEADBAND'
        if (
            pwm > 0
            and self._previous_abs_error is not None
            and error >= self._previous_abs_error
        ):
            return 'DRIVING_BUT_NO_PROGRESS(방향 반전 의심)'
        if (
            self._previous_abs_error is None
            or error < self._previous_abs_error
            or (pwm == 0 and error <= STEER_DEADBAND)
        ):
            return 'OK'
        return 'BRIDGE_NOT_WRITING'

    @staticmethod
    def _display(value: Any, default: str = '-') -> Any:
        return default if value is None else value

    def _report(self) -> None:
        frames_delta = _counter_delta(
            self._tx, self._previous_tx, 'frames_written_total')
        suppressed = self._suppressed_deltas()
        steering = self._steering or {}
        tcp = self._tcp or {}
        age_ms = tcp.get('age_ms')
        try:
            age_ms = f'{float(age_ms):.1f}'
        except (TypeError, ValueError):
            age_ms = '-'
        if isinstance(age_ms, float) and math.isinf(age_ms):
            age_ms = 'inf'

        summary = (
            'CHAIN '
            f'tcp={tcp.get("connected", "-")}/{tcp.get("fresh", "-")} '
            f'seq={tcp.get("seq", "-")} age_ms={age_ms} '
            f'cmd={self._commanded_steering} '
            f'ready={self._display(self._serial_ready)} '
            f'tx_delta={self._display(frames_delta)} '
            f'suppressed={suppressed or {}} '
            f'target={self._display(steering.get("target_adc"))} '
            f'current={self._display(steering.get("current_adc"))} '
            f'error={self._display(steering.get("error_adc"))} '
            f'pwm={self._display(steering.get("pwm"))} '
            f'drive={self._display(steering.get("drive"))} '
            f'enabled={self._display(steering.get("enabled"))} '
            f'fault={self._display(steering.get("fault"))}'
        )
        self.get_logger().info(summary)
        self.get_logger().info(
            f'VERDICT {self._verdict(frames_delta, suppressed)}')

        self._previous_tx = dict(self._tx) if self._tx is not None else None
        if self._steering is not None:
            self._previous_abs_error = abs(
                int(self._steering.get('target_adc', 0))
                - int(self._steering.get('current_adc', 0)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SteeringChainProbe()
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
