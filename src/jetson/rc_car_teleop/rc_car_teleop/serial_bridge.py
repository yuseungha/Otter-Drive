#!/usr/bin/env python3
"""Bridge atomic ROS drive commands to Arduino with host-side safety gates."""

from collections import deque
import json
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, Int32MultiArray, String

from rc_car_teleop.serial_bridge_core import (
    BridgeSafetyState,
    parse_debug_line,
    parse_steering_status_line,
    serial_ready_condition,
    guard_steering_command_by_adc,
    validate_bridge_config,
)


THROTTLE_MIN = -1000
THROTTLE_MAX = 1000
STEERING_MIN = -1000
STEERING_MAX = 1000
GEAR_LOW = -1
MAX_RX_BUFFER = 4096
EXPECTED_FIRMWARE_CONFIG_BANNER = (
    '[CONFIG] UNO_FINAL_20260813_ADC747_602_462_ESC1620_1500_1350_WD300'
)


class SerialBridge(Node):
    """ROS-to-serial bridge with freshness, readiness, E-stop, and TX metrics."""

    def __init__(self):
        super().__init__('serial_bridge')
        parameters = {
            'serial_port': (
                '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
            'baud_rate': 115200,
            'send_rate_hz': 20.0,
            'reconnect_interval_sec': 2.0,
            'reset_guard_sec': 3.5,
            'command_timeout_sec': 0.20,
            'write_timeout_sec': 0.05,
            'drive_enabled': False,
            'limits_confirmed': False,
            'throttle_min': 0,
            'throttle_max': 0,
            'steering_min': 0,
            'steering_max': 0,
            'allow_fault_reset': False,
            'steering_feedback_guard': False,
            'steering_feedback_timeout_sec': 0.30,
            'steering_adc_max_error': 22,
            'steering_adc_left': 747,
            'steering_adc_center': 602,
            'steering_adc_right': 462,
            'stale_steer_hold_sec': 0.20,
            'stale_steer_ramp_counts_per_tick': 120,
            'estop_center_rate_counts_per_tick': 120,
            'estop_center_timeout_sec': 1.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._send_rate = float(self.get_parameter('send_rate_hz').value)
        self._reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self._reset_guard = float(self.get_parameter('reset_guard_sec').value)
        self._write_timeout = float(
            self.get_parameter('write_timeout_sec').value)
        self._drive_enabled = bool(
            self.get_parameter('drive_enabled').value)
        self._limits_confirmed = bool(
            self.get_parameter('limits_confirmed').value)
        self._throttle_min = int(
            self.get_parameter('throttle_min').value)
        self._throttle_max = int(
            self.get_parameter('throttle_max').value)
        self._steering_min = int(
            self.get_parameter('steering_min').value)
        self._steering_max = int(
            self.get_parameter('steering_max').value)
        self._allow_fault_reset = bool(
            self.get_parameter('allow_fault_reset').value)
        self._steering_feedback_guard = bool(
            self.get_parameter('steering_feedback_guard').value)
        self._steering_feedback_timeout = float(
            self.get_parameter('steering_feedback_timeout_sec').value)
        self._steering_adc_max_error = int(
            self.get_parameter('steering_adc_max_error').value)
        self._steering_adc_left = int(
            self.get_parameter('steering_adc_left').value)
        self._steering_adc_center = int(
            self.get_parameter('steering_adc_center').value)
        self._steering_adc_right = int(
            self.get_parameter('steering_adc_right').value)
        if not 0.05 <= self._steering_feedback_timeout <= 1.0:
            raise ValueError('steering_feedback_timeout_sec must be in 0.05..1.0')
        # Validate the calibration and the firmware-stall safety margin.
        guard_steering_command_by_adc(
            0,
            self._steering_adc_center,
            max_error_adc=self._steering_adc_max_error,
            adc_left=self._steering_adc_left,
            adc_center=self._steering_adc_center,
            adc_right=self._steering_adc_right,
        )
        command_timeout = float(
            self.get_parameter('command_timeout_sec').value)
        stale_steer_hold = float(
            self.get_parameter('stale_steer_hold_sec').value)
        stale_steer_ramp = int(
            self.get_parameter('stale_steer_ramp_counts_per_tick').value)
        estop_center_rate = int(
            self.get_parameter('estop_center_rate_counts_per_tick').value)
        estop_center_timeout = float(
            self.get_parameter('estop_center_timeout_sec').value)
        validate_bridge_config(
            serial_port=self._port,
            baud_rate=self._baud,
            send_rate_hz=self._send_rate,
            reconnect_interval_sec=self._reconnect_interval,
            reset_guard_sec=self._reset_guard,
            command_timeout_sec=command_timeout,
            write_timeout_sec=self._write_timeout,
            drive_enabled=self._drive_enabled,
            limits_confirmed=self._limits_confirmed,
            throttle_min=self._throttle_min,
            throttle_max=self._throttle_max,
            steering_min=self._steering_min,
            steering_max=self._steering_max,
            stale_steer_hold_sec=stale_steer_hold,
            stale_steer_ramp_counts_per_tick=stale_steer_ramp,
            estop_center_rate_counts_per_tick=estop_center_rate,
            estop_center_timeout_sec=estop_center_timeout,
        )
        self._safety = BridgeSafetyState(
            command_timeout_sec=command_timeout,
            stale_steer_hold_sec=stale_steer_hold,
            stale_steer_ramp_counts_per_tick=stale_steer_ramp,
            estop_center_rate_counts_per_tick=estop_center_rate,
            estop_center_timeout_sec=estop_center_timeout,
        )
        self._serial = None
        self._rx_buffer = bytearray()
        self._last_connect_attempt_at = float('-inf')
        self._connected_at = None
        self._source_neutral_seen = False
        self._serial_ready = False
        self._startup_stop_sent = False
        self._startup_stop_ack = False
        self._firmware_config_verified = False
        self._operator_armed = False
        self._operator_deadman = False
        self._stop_pending = False
        self._host_timeout_latched = False
        self._recovery_blocked = False
        self._last_stale_state = None
        self._last_stale_warning_at = float('-inf')
        self._last_stats_at = time.monotonic()
        self._last_debug_warning_at = float('-inf')
        self._steering_current_adc = None
        self._steering_feedback_at = None
        self._write_times = deque()
        self._rx_line_times = deque()
        self._stats = {
            'frames_written_total': 0,
            'bytes_written_total': 0,
            'write_errors_total': 0,
            'suppressed_reset_guard': 0,
            'suppressed_no_source_neutral': 0,
            'suppressed_estop': 0,
            'stale_neutral_frames': 0,
            'last_frame_ascii': '',
            'debug_parse_ok': 0,
            'debug_parse_fail': 0,
        }

        self.create_subscription(
            Int32MultiArray, '/rc_car/drive_cmd', self._drive_callback, 10)
        self.create_subscription(
            Bool, '/rc_car/operator_armed', self._operator_armed_callback, 10)
        self.create_subscription(
            Bool, '/rc_car/operator_deadman', self._operator_deadman_callback, 10)
        self.create_subscription(
            Bool, '/vehicle/estop', self._estop_callback, 10)
        self.create_subscription(
            Bool, '/vehicle/estop_reset', self._estop_reset_callback, 10)
        self._feedback_pub = self.create_publisher(
            String, '/rc_car/feedback', 10)
        self._steering_adc_pub = self.create_publisher(
            Int32, '/rc_car/steering_adc', 10)
        self._steering_error_pub = self.create_publisher(
            Int32, '/rc_car/steering_error', 10)
        self._steering_status_pub = self.create_publisher(
            String, '/rc_car/steering_status', 10)
        self._serial_connected_pub = self.create_publisher(
            Bool, '/rc_car/serial_connected', 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._serial_ready_pub = self.create_publisher(
            Bool, '/rc_car/serial_ready', state_qos)
        self._estop_latched_pub = self.create_publisher(
            Bool, '/vehicle/estop_latched', state_qos)
        self._command_stale_pub = self.create_publisher(
            Bool, '/rc_car/command_stale', 10)
        self._recovery_blocked_pub = self.create_publisher(
            Bool, '/rc_car/recovery_blocked', state_qos)
        self._tx_stats_pub = self.create_publisher(
            String, '/rc_car/tx_stats', 10)
        self._publish_ready(False, force=True)
        self._publish_estop(False)
        self._publish_recovery_blocked(False)
        self.create_timer(1.0 / self._send_rate, self._io_tick)
        self.get_logger().info(
            f'Serial bridge starting: {self._port} @ {self._baud}; '
            f'drive_enabled={self._drive_enabled} '
            f'limits=T[{self._throttle_min},{self._throttle_max}] '
            f'S[{self._steering_min},{self._steering_max}]')

    @staticmethod
    def _normalize_throttle(value):
        return max(THROTTLE_MIN, min(THROTTLE_MAX, int(value)))

    @staticmethod
    def _normalize_steering(value):
        return max(STEERING_MIN, min(STEERING_MAX, int(value)))

    def _ensure_safety_state(self):
        """Keep source-level compatibility with older direct bridge tests."""
        if hasattr(self, '_safety'):
            return
        self._safety = BridgeSafetyState()
        self._safety.throttle = int(getattr(self, '_throttle', 0))
        self._safety.steering = int(getattr(self, '_steering', 0))
        self._safety.gear = int(getattr(self, '_gear', GEAR_LOW))

    def _drive_callback(self, message):
        self._ensure_safety_state()
        legacy_direct_test = not hasattr(self, '_drive_enabled')
        if len(message.data) != 2:
            self.get_logger().warn(
                'Ignoring drive_cmd: expected exactly [throttle, steering]',
                throttle_duration_sec=1.0)
            self._request_stop('invalid_command_shape')
            return
        try:
            throttle = int(message.data[0])
            steering = int(message.data[1])
        except (TypeError, ValueError, OverflowError):
            self.get_logger().error('Invalid non-integer drive command')
            self._request_stop('invalid_command')
            return
        if legacy_direct_test:
            # Older protocol-only tests constructed this class through
            # ``__new__`` and exercised the historical hard normalization.
            # Live nodes always take the strict configured-range path below.
            throttle = self._normalize_throttle(throttle)
            steering = self._normalize_steering(steering)

        # A neutral source sample may establish the post-reset neutral state;
        # the guarded startup X below establishes the same state for the web
        # client, which intentionally publishes no drive_cmd while disarmed.
        # Neither path authorizes D without ARM and deadman.
        if throttle == 0 and steering == 0:
            self._source_neutral_seen = True

        if hasattr(self, '_drive_enabled') and (
            not self._drive_enabled or not self._limits_confirmed
        ):
            return
        if hasattr(self, '_operator_armed') and not self._operator_armed:
            return
        if hasattr(self, '_operator_deadman') and not self._operator_deadman:
            return

        throttle_min = getattr(self, '_throttle_min', THROTTLE_MIN)
        throttle_max = getattr(self, '_throttle_max', THROTTLE_MAX)
        steering_min = getattr(self, '_steering_min', STEERING_MIN)
        steering_max = getattr(self, '_steering_max', STEERING_MAX)
        if not throttle_min <= throttle <= throttle_max:
            self.get_logger().error(
                f'Rejected throttle outside confirmed range: {throttle}')
            self._request_stop('throttle_range')
            return
        if not steering_min <= steering <= steering_max:
            self.get_logger().error(
                f'Rejected steering outside confirmed range: {steering}')
            self._request_stop('steering_range')
            return

        if not self._safety.accept_command(
            throttle, steering, GEAR_LOW, time.monotonic()
        ):
            self._stats['suppressed_estop'] += 1
            return
        self._throttle = self._safety.throttle
        self._steering = self._safety.steering
        self._gear = self._safety.gear

    def _operator_armed_callback(self, message):
        requested = bool(message.data)
        if not requested:
            was_armed = self._operator_armed
            self._operator_armed = False
            self._operator_deadman = False
            self._safety.last_command_at = None
            if was_armed:
                self._request_stop('operator_disarmed')
            # A fresh explicit false transition acknowledges a prior timeout
            # and permits a later manual arm. No automatic re-arm occurs.
            self._host_timeout_latched = False
            self._safety.command_stale = False
            self._command_stale_pub.publish(self._bool_message(False))
            return
        if not self._drive_enabled or not self._limits_confirmed:
            self.get_logger().error('Operator arm rejected: drive is locked')
            self._operator_armed = False
            return
        if self._safety.estop_latched or self._recovery_blocked:
            self.get_logger().error('Operator arm rejected: fault is latched')
            self._operator_armed = False
            return
        if not self._serial_ready or not self._source_neutral_seen:
            self.get_logger().warn('Operator arm rejected: serial is not ready')
            self._operator_armed = False
            return
        if self._host_timeout_latched:
            self.get_logger().error(
                'Operator arm rejected: disarm transition required after timeout')
            return
        if not self._operator_armed:
            self._safety.last_command_at = None
            self._operator_armed = True
            self.get_logger().info('Operator arm accepted; awaiting fresh drive_cmd')

    def _operator_deadman_callback(self, message):
        requested = bool(message.data)
        if not requested:
            was_active = self._operator_deadman
            self._operator_deadman = False
            self._safety.last_command_at = None
            if was_active:
                self._stop_pending = True
                self.get_logger().warn('Deadman released; sending X')
            return
        if not self._serial_ready:
            self._operator_deadman = False
            return
        # Arm and deadman use separate transient topics. Remember a fresh
        # deadman press even if DDS delivers it just before the arm sample;
        # actual D output still requires both gates simultaneously.
        self._operator_deadman = True

    def _request_stop(self, reason):
        self._operator_armed = False
        self._operator_deadman = False
        self._safety.last_command_at = None
        self._safety.throttle = 0
        self._safety.steering = 0
        self._stop_pending = True
        self.get_logger().warn(f'Safe stop requested: {reason}')

    def _estop_callback(self, message):
        if not bool(message.data):
            return
        if self._safety.latch_estop(time.monotonic()):
            self._operator_armed = False
            self._operator_deadman = False
            self._safety.last_command_at = None
            self._safety.throttle = 0
            self._safety.steering = 0
            self._stop_pending = True
            self._publish_estop(True)
            self.get_logger().error('E-stop latched; sending X')

    def _estop_reset_callback(self, message):
        if not bool(message.data):
            return
        if not self._allow_fault_reset:
            self.get_logger().error(
                'E-stop reset rejected: allow_fault_reset=false')
            return
        if not self._safety.reset_estop():
            return
        # R is never used as a stop. It is permitted only after the host latch
        # has already sent X and an operator explicitly enables this path.
        if self._serial is not None and not self._write_frame(b'R\n'):
            return
        self._source_neutral_seen = False
        self._publish_ready(False)
        self._publish_estop(False)
        self.get_logger().warn('E-stop reset; source neutral and re-arming required')

    @staticmethod
    def _bool_message(value):
        message = Bool()
        message.data = bool(value)
        return message

    def _publish_connected(self, connected):
        self._serial_connected_pub.publish(self._bool_message(connected))

    def _publish_ready(self, ready, force=False):
        ready = bool(ready)
        if not force and ready == self._serial_ready:
            return
        self._serial_ready = ready
        self._serial_ready_pub.publish(self._bool_message(ready))

    def _publish_estop(self, latched):
        self._estop_latched_pub.publish(self._bool_message(latched))

    def _publish_recovery_blocked(self, blocked):
        self._recovery_blocked_pub.publish(self._bool_message(blocked))

    def _update_ready(self, now):
        ready = serial_ready_condition(
            self._serial is not None and self._connected_at is not None,
            0.0 if self._connected_at is None else now - self._connected_at,
            self._reset_guard,
            self._source_neutral_seen,
            self._safety.estop_latched,
        ) and (
            self._startup_stop_sent
            and self._startup_stop_ack
            and self._firmware_config_verified
            and not self._recovery_blocked
        )
        if self._startup_stop_sent and not self._firmware_config_verified:
            self.get_logger().error(
                'Firmware config banner not verified; drive remains locked',
                throttle_duration_sec=5.0)
        elif self._startup_stop_sent and not self._startup_stop_ack:
            self.get_logger().warn(
                'Waiting for firmware [STOP] acknowledgement; drive locked',
                throttle_duration_sec=5.0)
        self._publish_ready(ready)
        return ready

    def _connect_if_due(self, now):
        if self._serial is not None:
            return
        if self._recovery_blocked:
            return
        if now - self._last_connect_attempt_at < self._reconnect_interval:
            return
        self._last_connect_attempt_at = now
        try:
            connection = serial.Serial(
                self._port,
                self._baud,
                timeout=0,
                write_timeout=self._write_timeout,
                exclusive=True,
            )
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(
                f'Cannot open {self._port}: {error}; retrying',
                throttle_duration_sec=5.0)
            return
        self._serial = connection
        self._rx_buffer.clear()
        # Opening CH340 may block and toggle DTR. The complete reset guard must
        # begin only after open has actually returned.
        self._connected_at = time.monotonic()
        self._source_neutral_seen = False
        self._startup_stop_sent = False
        self._startup_stop_ack = False
        self._firmware_config_verified = False
        self._operator_armed = False
        self._operator_deadman = False
        self._stop_pending = False
        self._steering_current_adc = None
        self._steering_feedback_at = None
        self._safety.last_command_at = None
        self._safety.throttle = 0
        self._safety.steering = 0
        self._safety.gear = GEAR_LOW
        self._publish_ready(False)
        self._publish_connected(True)
        self.get_logger().info(
            f'Serial connected: {self._port} @ {self._baud}; '
            f'no D frames for {self._reset_guard:.1f}s')

    def _disconnect(self, reason, *, block_recovery=False):
        connection = self._serial
        self._serial = None
        self._connected_at = None
        self._rx_buffer.clear()
        self._source_neutral_seen = False
        self._startup_stop_sent = False
        self._startup_stop_ack = False
        self._firmware_config_verified = False
        self._operator_armed = False
        self._operator_deadman = False
        self._stop_pending = False
        self._steering_current_adc = None
        self._steering_feedback_at = None
        self._safety.last_command_at = None
        if block_recovery:
            self._recovery_blocked = True
            self._publish_recovery_blocked(True)
        if connection is not None:
            try:
                connection.close()
            except (serial.SerialException, OSError):
                pass
        self._publish_ready(False)
        self._publish_connected(False)
        self.get_logger().error(f'Serial disconnected: {reason}')

    def _write_frame(self, payload):
        connection = self._serial
        if connection is None:
            return False
        try:
            written = connection.write(payload)
            if written != len(payload):
                raise serial.SerialTimeoutException(
                    f'partial write {written}/{len(payload)}')
            connection.flush()
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as error:
            self._stats['write_errors_total'] += 1
            # A failed/partial write can leave an unterminated D or X frame.
            # Never append another frame or automatically reopen this stream.
            self._disconnect(
                f'write failed: {error}', block_recovery=True)
            return False
        now = time.monotonic()
        self._write_times.append(now)
        self._stats['frames_written_total'] += 1
        self._stats['bytes_written_total'] += len(payload)
        self._stats['last_frame_ascii'] = payload.decode(
            'ascii', errors='replace').rstrip('\r\n')
        return True

    def _write_decision(self, now):
        if self._stop_pending:
            if self._startup_stop_sent and self._serial is not None:
                if self._write_frame(b'X\n'):
                    self._stop_pending = False
            return
        if not self._operator_armed or not self._operator_deadman:
            return
        decision = self._safety.decision(now)
        stale_message = self._bool_message(self._safety.command_stale)
        self._command_stale_pub.publish(stale_message)
        if (
            self._safety.command_stale
            and now - self._last_stale_warning_at >= 1.0
        ):
            self.get_logger().warn('drive_cmd stale; throttle neutralized')
            self._last_stale_warning_at = now
        self._last_stale_state = self._safety.command_stale
        if decision is None:
            if self._safety.estop_latched:
                self._stats['suppressed_estop'] += 1
            return
        if decision.stale:
            self._host_timeout_latched = True
            self._request_stop('host_command_timeout')
            if self._startup_stop_sent and self._serial is not None:
                if self._write_frame(b'X\n'):
                    self._stop_pending = False
            return
        if self._safety.estop_latched:
            ready = True
        else:
            guard_active = (
                self._connected_at is None
                or now - self._connected_at < self._reset_guard)
            if guard_active:
                self._stats['suppressed_reset_guard'] += 1
                return
            if not self._source_neutral_seen:
                self._stats['suppressed_no_source_neutral'] += 1
                return
            ready = self._update_ready(now)
        if not ready:
            return
        if decision.kind == 'X':
            self._request_stop('estop')
            return
        # Final Arduino protocol is exactly "D <throttle> <steering>\n".
        steering = decision.steering
        if getattr(self, '_steering_feedback_guard', False):
            feedback_at = getattr(self, '_steering_feedback_at', None)
            if (
                self._steering_current_adc is None
                or feedback_at is None
                or now - feedback_at > self._steering_feedback_timeout
            ):
                self._request_stop('steering_feedback_stale')
                return
            steering = guard_steering_command_by_adc(
                steering,
                self._steering_current_adc,
                max_error_adc=self._steering_adc_max_error,
                adc_left=self._steering_adc_left,
                adc_center=self._steering_adc_center,
                adc_right=self._steering_adc_right,
            )
        payload = f'D {decision.throttle} {steering}\n'.encode('ascii')
        if self._write_frame(payload) and decision.stale:
            if decision.throttle == 0 and decision.steering == 0:
                self._stats['stale_neutral_frames'] += 1

    def _write_command(self, now):
        """Compatibility shim for the pre-safety direct unit-test API."""
        self._ensure_safety_state()
        if self._safety.last_command_at is None:
            self._safety.last_command_at = float(now)
        guard_active = (
            getattr(self, '_connected_at', None) is None
            or now - self._connected_at < getattr(self, '_reset_guard', 0.0))
        if guard_active or not getattr(self, '_source_neutral_seen', False):
            return
        decision = self._safety.decision(now)
        if decision is None or decision.kind != 'D':
            return
        self._serial.write(
            f'D {decision.throttle} {decision.steering}\n'.encode('ascii'))

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
        now = time.monotonic()
        self._rx_line_times.append(now)
        if line == EXPECTED_FIRMWARE_CONFIG_BANNER:
            if not self._firmware_config_verified:
                self.get_logger().info('Verified final Arduino firmware config')
            self._firmware_config_verified = True
        if self._startup_stop_sent and line == '[STOP]':
            self._startup_stop_ack = True
        feedback = String()
        feedback.data = line
        self._feedback_pub.publish(feedback)
        parsed = parse_debug_line(line)
        steering_status = parse_steering_status_line(line)
        if steering_status is not None:
            self._steering_current_adc = steering_status['current_adc']
            self._steering_feedback_at = now
            status_message = String()
            status_message.data = json.dumps(
                steering_status, sort_keys=True)
            self._steering_status_pub.publish(status_message)
        if parsed is None:
            if '[DEBUG]' in line:
                self._stats['debug_parse_fail'] += 1
                if now - self._last_debug_warning_at >= 1.0:
                    self.get_logger().warn(
                        'Unrecognized firmware DEBUG feedback format')
                    self._last_debug_warning_at = now
            return
        self._stats['debug_parse_ok'] += 1
        current_adc, steering_error = parsed
        adc = Int32()
        adc.data = current_adc
        error = Int32()
        error.data = steering_error
        self._steering_adc_pub.publish(adc)
        self._steering_error_pub.publish(error)

    @staticmethod
    def _trim_window(samples, now):
        while samples and now - samples[0] > 1.0:
            samples.popleft()

    def _publish_stats_if_due(self, now):
        if now - self._last_stats_at < 1.0:
            return
        self._trim_window(self._write_times, now)
        self._trim_window(self._rx_line_times, now)
        data = dict(self._stats)
        data['tx_hz'] = float(len(self._write_times))
        data['rx_lines_hz'] = float(len(self._rx_line_times))
        message = String()
        message.data = json.dumps(data, sort_keys=True)
        self._tx_stats_pub.publish(message)
        self._last_stats_at = now

    def _io_tick(self):
        now = time.monotonic()
        self._connect_if_due(now)
        if self._serial is not None:
            self._publish_connected(True)
            self._read_available()
        if self._serial is not None:
            now = time.monotonic()
            guard_complete = (
                self._connected_at is not None
                and now - self._connected_at >= self._reset_guard
            )
            if guard_complete and not self._startup_stop_sent:
                try:
                    self._serial.reset_input_buffer()
                except (AttributeError, serial.SerialException, OSError):
                    pass
                if self._write_frame(b'X\n'):
                    self._startup_stop_sent = True
                    self._source_neutral_seen = True
                    # The startup X also satisfies any stop request that
                    # arrived during reset guard; never append a duplicate X.
                    self._stop_pending = False
            self._update_ready(now)
            self._write_decision(now)
        self._publish_stats_if_due(now)

    def destroy_node(self):
        if self._serial is not None:
            try:
                if self._startup_stop_sent:
                    self._write_frame(b'X\n')
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                pass
            connection = self._serial
            if connection is not None:
                try:
                    connection.close()
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
