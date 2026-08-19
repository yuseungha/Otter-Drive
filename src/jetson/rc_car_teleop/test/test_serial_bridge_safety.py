"""Hardware-free safety tests for the ROS-to-Arduino serial bridge."""

from collections import deque
from types import SimpleNamespace

import pytest

from rc_car_teleop.serial_bridge import (
    EXPECTED_FIRMWARE_CONFIG_BANNER,
    SerialBridge,
)
from rc_car_teleop.serial_bridge_core import (
    BridgeSafetyState,
    validate_bridge_config,
)
import rc_car_teleop.serial_bridge as serial_bridge_module


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class CaptureLogger:
    def __init__(self):
        self.lines = []

    def _capture(self, message, **_kwargs):
        self.lines.append(str(message))

    debug = _capture
    info = _capture
    warn = _capture
    warning = _capture
    error = _capture


class ScriptedSerial:
    def __init__(self, write_results=None, *, flush_error=None):
        self.writes = []
        self.write_results = deque(write_results or [])
        self.flush_error = flush_error
        self.closed = False
        self.reset_input_calls = 0
        self.in_waiting = 0

    def write(self, payload):
        self.writes.append(payload)
        if self.write_results:
            result = self.write_results.popleft()
            if isinstance(result, BaseException):
                raise result
            return result
        return len(payload)

    def flush(self):
        if self.flush_error is not None:
            raise self.flush_error

    def reset_input_buffer(self):
        self.reset_input_calls += 1

    def close(self):
        self.closed = True


def make_bridge(connection=None):
    bridge = SerialBridge.__new__(SerialBridge)
    bridge._serial = connection or ScriptedSerial()
    bridge._rx_buffer = bytearray()
    bridge._last_connect_attempt_at = float('-inf')
    bridge._connected_at = 100.0
    bridge._source_neutral_seen = False
    bridge._serial_ready = False
    bridge._startup_stop_sent = False
    bridge._startup_stop_ack = False
    bridge._firmware_config_verified = False
    bridge._operator_armed = False
    bridge._operator_deadman = False
    bridge._stop_pending = False
    bridge._host_timeout_latched = False
    bridge._recovery_blocked = False
    bridge._last_stale_state = None
    bridge._last_stale_warning_at = float('-inf')
    bridge._last_stats_at = 100.0
    bridge._last_debug_warning_at = float('-inf')
    bridge._write_times = deque()
    bridge._rx_line_times = deque()
    bridge._stats = {
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
    bridge._port = '/dev/serial/by-id/fake-controller'
    bridge._baud = 115200
    bridge._write_timeout = 0.05
    bridge._reconnect_interval = 2.0
    bridge._reset_guard = 3.5
    bridge._drive_enabled = True
    bridge._limits_confirmed = True
    bridge._throttle_min = 0
    bridge._throttle_max = 150
    bridge._steering_min = -350
    bridge._steering_max = 350
    bridge._allow_fault_reset = False
    bridge._safety = BridgeSafetyState(
        command_timeout_sec=0.20,
        stale_steer_hold_sec=0.0,
    )
    bridge._logger = CaptureLogger()
    bridge.get_logger = lambda: bridge._logger
    for name in (
        '_serial_connected_pub',
        '_serial_ready_pub',
        '_estop_latched_pub',
        '_command_stale_pub',
        '_recovery_blocked_pub',
        '_tx_stats_pub',
        '_feedback_pub',
        '_steering_status_pub',
        '_steering_adc_pub',
        '_steering_error_pub',
    ):
        setattr(bridge, name, CapturePublisher())
    return bridge


def ready_bridge(connection=None):
    bridge = make_bridge(connection)
    bridge._startup_stop_sent = True
    bridge._startup_stop_ack = True
    bridge._firmware_config_verified = True
    bridge._source_neutral_seen = True
    bridge._serial_ready = True
    return bridge


def test_reset_guard_emits_one_startup_x_and_no_d(monkeypatch):
    connection = ScriptedSerial()
    bridge = make_bridge(connection)
    now = [103.49]
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: now[0])

    bridge._io_tick()
    assert connection.writes == []

    now[0] = 103.50
    bridge._io_tick()
    now[0] = 103.55
    bridge._io_tick()

    assert connection.writes == [b'X\n']
    assert connection.reset_input_calls == 1
    assert bridge._startup_stop_sent
    assert not bridge._serial_ready

    bridge._handle_line(EXPECTED_FIRMWARE_CONFIG_BANNER)
    bridge._handle_line('[STOP]')
    bridge._update_ready(now[0])
    assert bridge._serial_ready


def test_wrong_firmware_banner_never_unlocks_drive(monkeypatch):
    connection = ScriptedSerial()
    bridge = make_bridge(connection)
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 103.5)

    bridge._handle_line('[CONFIG] OLD_OR_UNKNOWN')
    bridge._io_tick()
    bridge._handle_line('[STOP]')
    bridge._update_ready(103.6)

    assert connection.writes == [b'X\n']
    assert bridge._startup_stop_ack
    assert not bridge._firmware_config_verified
    assert not bridge._serial_ready


def test_reset_guard_clock_starts_after_blocking_open_returns(monkeypatch):
    connection = ScriptedSerial()
    bridge = make_bridge()
    bridge._serial = None
    now = [10.0]

    def open_serial(*_args, **_kwargs):
        now[0] = 50.0
        return connection

    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        serial_bridge_module.serial, 'Serial', open_serial, raising=False)

    bridge._connect_if_due(10.0)
    assert bridge._connected_at == 50.0

    now[0] = 53.49
    bridge._io_tick()
    assert connection.writes == []
    now[0] = 53.50
    bridge._io_tick()
    assert connection.writes == [b'X\n']


def test_startup_x_satisfies_pending_stop_without_duplicate(monkeypatch):
    connection = ScriptedSerial()
    bridge = make_bridge(connection)
    bridge._stop_pending = True
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 103.5)

    bridge._io_tick()

    assert connection.writes == [b'X\n']
    assert not bridge._stop_pending


def test_disarmed_drive_sample_never_emits_d(monkeypatch):
    connection = ScriptedSerial()
    bridge = ready_bridge(connection)
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 104.0)

    bridge._drive_callback(SimpleNamespace(data=[100, 25]))
    bridge._write_decision(104.0)

    assert connection.writes == []
    assert bridge._safety.last_command_at is None


def test_legacy_three_field_command_fails_closed_without_d(monkeypatch):
    connection = ScriptedSerial()
    bridge = ready_bridge(connection)
    bridge._operator_armed = True
    bridge._operator_deadman = True
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 104.0)

    bridge._drive_callback(SimpleNamespace(data=[100, 25, -1]))
    bridge._write_decision(104.0)

    assert connection.writes == [b'X\n']
    assert not bridge._operator_armed
    assert not bridge._operator_deadman


def test_deadman_fall_emits_exactly_one_x(monkeypatch):
    connection = ScriptedSerial()
    bridge = ready_bridge(connection)
    bridge._operator_armed = True
    bridge._operator_deadman = True
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 104.0)
    bridge._drive_callback(SimpleNamespace(data=[100, -25]))
    bridge._write_decision(104.0)

    bridge._operator_deadman_callback(SimpleNamespace(data=False))
    bridge._write_decision(104.01)
    bridge._write_decision(104.02)

    assert connection.writes == [b'D 100 -25\n', b'X\n']
    assert bridge._operator_armed
    assert not bridge._operator_deadman


def test_host_timeout_latches_disarm_and_emits_x_not_stale_d(monkeypatch):
    connection = ScriptedSerial()
    bridge = ready_bridge(connection)
    bridge._operator_armed = True
    bridge._operator_deadman = True
    now = [104.0]
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: now[0])
    bridge._drive_callback(SimpleNamespace(data=[100, 25]))
    bridge._write_decision(now[0])

    now[0] = 104.201
    bridge._write_decision(now[0])
    now[0] = 104.5
    bridge._write_decision(now[0])

    assert connection.writes == [b'D 100 25\n', b'X\n']
    assert bridge._host_timeout_latched
    assert not bridge._operator_armed
    assert not bridge._operator_deadman


def test_partial_d_blocks_recovery_and_never_appends_x(monkeypatch):
    expected = b'D 100 25\n'
    connection = ScriptedSerial(write_results=[len(expected) - 1])
    bridge = ready_bridge(connection)
    bridge._operator_armed = True
    bridge._operator_deadman = True
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 104.0)
    bridge._drive_callback(SimpleNamespace(data=[100, 25]))

    bridge._write_decision(104.0)

    assert connection.writes == [expected]
    assert connection.closed
    assert bridge._serial is None
    assert bridge._recovery_blocked

    open_calls = []
    monkeypatch.setattr(
        serial_bridge_module.serial,
        'Serial',
        lambda *_args, **_kwargs: open_calls.append(True),
        raising=False,
    )
    bridge._connect_if_due(200.0)
    assert open_calls == []
    assert connection.writes == [expected]


def test_partial_stop_blocks_recovery_without_retry(monkeypatch):
    connection = ScriptedSerial(write_results=[1])
    bridge = ready_bridge(connection)
    bridge._operator_armed = True
    bridge._operator_deadman = True
    bridge._stop_pending = True

    bridge._write_decision(104.0)
    bridge._write_decision(104.1)

    assert connection.writes == [b'X\n']
    assert connection.closed
    assert bridge._recovery_blocked


def test_partial_startup_x_blocks_recovery_without_second_frame(monkeypatch):
    connection = ScriptedSerial(write_results=[1])
    bridge = make_bridge(connection)
    monkeypatch.setattr(serial_bridge_module.time, 'monotonic', lambda: 103.5)

    bridge._io_tick()
    bridge._io_tick()

    assert connection.writes == [b'X\n']
    assert connection.closed
    assert bridge._serial is None
    assert bridge._recovery_blocked


def valid_config(**overrides):
    values = {
        'serial_port': '/dev/serial/by-id/fake-controller',
        'baud_rate': 115200,
        'send_rate_hz': 20.0,
        'reconnect_interval_sec': 2.0,
        'reset_guard_sec': 3.5,
        'command_timeout_sec': 0.20,
        'write_timeout_sec': 0.05,
        'drive_enabled': True,
        'limits_confirmed': True,
        'throttle_min': 0,
        'throttle_max': 150,
        'steering_min': -350,
        'steering_max': 350,
        'stale_steer_hold_sec': 0.0,
        'stale_steer_ramp_counts_per_tick': 120,
        'estop_center_rate_counts_per_tick': 120,
        'estop_center_timeout_sec': 1.0,
    }
    values.update(overrides)
    return values


def test_live_competition_config_is_valid():
    validate_bridge_config(**valid_config())


def test_default_locked_config_is_valid():
    validate_bridge_config(**valid_config(
        drive_enabled=False,
        limits_confirmed=False,
        throttle_min=0,
        throttle_max=0,
        steering_min=0,
        steering_max=0,
    ))


@pytest.mark.parametrize(
    ('overrides', 'message'),
    [
        ({'serial_port': '/dev/ttyUSB0'}, 'serial_port'),
        ({'baud_rate': 9600}, 'baud_rate'),
        ({'send_rate_hz': float('nan')}, 'finite'),
        ({'send_rate_hz': 9.0}, 'send_rate_hz'),
        ({'send_rate_hz': 10.0, 'command_timeout_sec': 0.05}, 'send period'),
        ({'reconnect_interval_sec': float('inf')}, 'finite'),
        ({'reset_guard_sec': 3.49}, 'reset_guard_sec'),
        ({'reset_guard_sec': 31.0}, 'reset_guard_sec'),
        ({'write_timeout_sec': 0.201}, 'write_timeout_sec'),
        ({'drive_enabled': True, 'limits_confirmed': False,
          'throttle_min': 0, 'throttle_max': 0,
          'steering_min': 0, 'steering_max': 0}, 'limits_confirmed'),
        ({'drive_enabled': False, 'limits_confirmed': False,
          'throttle_max': 1}, 'unconfirmed'),
        ({'throttle_min': 1}, 'include zero'),
        ({'steering_max': 1001}, 'steering_max'),
    ],
)
def test_invalid_config_is_rejected(overrides, message):
    with pytest.raises(ValueError, match=message):
        validate_bridge_config(**valid_config(**overrides))


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_safety_state_rejects_nonfinite_timing(value):
    with pytest.raises(ValueError, match='finite'):
        BridgeSafetyState(command_timeout_sec=value)
