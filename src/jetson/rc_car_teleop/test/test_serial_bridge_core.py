"""Regression tests for bridge safety state and firmware feedback parsing."""

from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from rc_car_teleop.serial_bridge_core import (  # noqa: E402
    BridgeSafetyState,
    FaultResetPolicy,
    FrameDecision,
    guard_steering_command_by_adc,
    parse_debug_line,
    parse_steering_status_line,
    parse_version_line,
    serial_ready_condition,
    steering_command_to_adc,
)


from rc_car_teleop import firmware_model as FAKE  # noqa: E402


def test_both_firmware_debug_formats_and_mismatch() -> None:
    legacy = (
        '[DEBUG] Throttle: 0 | SteerTarget(ADC): 620 | '
        'SteerCurrent(ADC): 615 | Error: 5')
    active = (
        '[DEBUG] Current=594 Target=590 Error=-4 PWM=114 '
        'Drive=ADC_DOWN Enabled=YES Fault=NO')
    assert parse_debug_line(legacy) == (615, 5)
    assert parse_debug_line(active) == (594, -4)
    assert parse_debug_line('[DEBUG] unknown') is None


def test_active_firmware_full_steering_status() -> None:
    active = (
        '[DEBUG] Command=-106 Current=594 Target=589 Error=-5 PWM=115 '
        'Drive=ADC_DOWN Enabled=YES Fault=NO')
    assert parse_steering_status_line(active) == {
        'command': -106,
        'current_adc': 594,
        'target_adc': 589,
        'error_adc': -5,
        'pwm': 115,
        'drive': 'ADC_DOWN',
        'enabled': True,
        'fault': False,
    }


def test_adc_feedback_guard_releases_full_lock_as_hardware_moves() -> None:
    calibration = {
        'adc_left': 747,
        'adc_center': 602,
        'adc_right': 462,
    }
    first = guard_steering_command_by_adc(
        650, 602, max_error_adc=22, **calibration)
    assert first == 152
    assert steering_command_to_adc(first, **calibration) == 624

    advanced = guard_steering_command_by_adc(
        650, 650, max_error_adc=22, **calibration)
    assert advanced > first
    assert steering_command_to_adc(advanced, **calibration) == 672

    reached = guard_steering_command_by_adc(
        650, 695, max_error_adc=22, **calibration)
    assert reached == 650


def test_adc_feedback_guard_crosses_center_without_exceeding_error_budget() -> None:
    calibration = {
        'adc_left': 747,
        'adc_center': 602,
        'adc_right': 462,
    }
    guarded = guard_steering_command_by_adc(
        -650, 640, max_error_adc=22, **calibration)
    target = steering_command_to_adc(guarded, **calibration)
    assert guarded > 0
    assert target == 618
    assert abs(target - 640) == 22


def test_active_firmware_version_banner() -> None:
    line = (
        '[VERSION] sketch=rc_car_controller_safe build=2026-08-12 '
        'LEFT=710 CENTER=588 RIGHT=466 DEADBAND=10 '
        'MINPWM=110 MAXPWM=180 ENA=5 IN1=12 IN2=13 FB=A5 '
        'HIGH_LOW_INCREASES_ADC=0 WATCHDOG_MS=400')
    parsed = parse_version_line(line)
    assert parsed is not None
    assert parsed['sketch'] == 'rc_car_controller_safe'
    assert parsed['build'] == '2026-08-12'
    assert parsed['adc_center'] == FAKE.ADC_CENTER
    assert parsed['steer_deadband'] == FAKE.STEER_DEADBAND
    assert parsed['steer_min_pwm'] == FAKE.STEER_MIN_PWM
    assert parsed['steer_max_pwm'] == FAKE.STEER_MAX_PWM
    assert parsed['steer_feedback_pin'] == 'A5'
    assert parsed['high_low_increases_adc'] is False
    assert parse_version_line('[VERSION] old firmware') is None


def test_freshness_neutralizes_before_firmware_watchdog_deadline() -> None:
    state = BridgeSafetyState(
        command_timeout_sec=0.30,
        stale_steer_hold_sec=0.0,
        stale_steer_ramp_counts_per_tick=120,
    )
    assert state.accept_command(300, -300, -1, 0.0)
    assert state.decision(0.29).throttle == 300
    stale = state.decision(0.31)
    assert stale.stale
    assert stale.throttle == 0
    assert stale.steering == -180
    assert 0.31 < FAKE.WATCHDOG_SEC


def test_estop_is_latched_discards_drive_and_centers_before_single_x() -> None:
    state = BridgeSafetyState(
        estop_center_rate_counts_per_tick=120,
        estop_center_timeout_sec=1.0)
    state.accept_command(200, -300, -1, 0.0)
    assert state.latch_estop(0.01)
    assert not state.accept_command(800, 900, 1, 0.02)
    assert state.decision(0.02) == FrameDecision('D', 0, -300, -1, False)
    assert state.decision(0.07).steering == -180
    assert state.decision(0.12).steering == -60
    centered = state.decision(0.17)
    assert centered.steering == 0
    assert state.decision(0.22).kind == 'X'
    assert state.decision(0.27) is None
    assert state.estop_latched
    assert state.reset_estop()
    assert not state.estop_latched
    assert state.last_command_at is None


def test_false_estop_cannot_clear_latch_semantics() -> None:
    state = BridgeSafetyState()
    state.latch_estop(0.0)
    # There is intentionally no "set_estop(False)" path.
    assert state.estop_latched
    assert not state.accept_command(0, 0, -1, 0.1)


def test_reconnect_requires_source_neutral_before_ready() -> None:
    assert serial_ready_condition(True, 3.2, 3.2, True, False)
    # Reconnect resets source_neutral_seen even when the guard has elapsed.
    assert not serial_ready_condition(True, 3.2, 3.2, False, False)
    assert not serial_ready_condition(True, 3.2, 3.2, True, True)


def test_reconnect_nonzero_is_suppressed_until_one_neutral_command() -> None:
    writes = []
    source_neutral_seen = False
    for throttle, steering in ((0, 240), (0, 240), (0, 0), (0, 240)):
        if throttle == 0 and steering == 0:
            source_neutral_seen = True
        if serial_ready_condition(
            True, 3.2, 3.2, source_neutral_seen, False
        ):
            writes.append((throttle, steering))
    assert writes == [(0, 0), (0, 240)]


def test_fault_reset_never_runs_while_steering_is_nonzero() -> None:
    policy = FaultResetPolicy()
    assert policy.observe(True, 240, 0.0) == 'none'
    assert policy.observe(True, -240, 10.0) == 'none'
    assert policy.consecutive_attempts == 0


def test_fault_reset_runs_at_neutral_and_stops_after_three_attempts() -> None:
    policy = FaultResetPolicy()
    assert policy.observe(True, 0, 0.0) == 'reset'
    assert policy.observe(True, 0, 1.9) == 'none'
    assert policy.observe(True, 0, 2.0) == 'reset'
    assert policy.observe(True, 0, 4.0) == 'reset'
    assert policy.observe(True, 0, 6.0) == 'give_up'
    assert policy.observe(True, 0, 8.0) == 'none'
    assert policy.consecutive_attempts == 3


def test_fault_reset_success_clears_consecutive_attempts() -> None:
    policy = FaultResetPolicy()
    assert policy.observe(True, 0, 0.0) == 'reset'
    assert policy.observe(False, 0, 0.1) == 'success'
    assert policy.consecutive_attempts == 0
    assert policy.observe(True, 0, 0.2) == 'none'


def test_zero_steering_maps_to_calibrated_center() -> None:
    # ROS/TCP steering uses normalized counts. Zero must remain the firmware's
    # calibrated center target; callers must not send raw ADC 588 as a command.
    assert FAKE.steering_command_to_adc(0) == FAKE.ADC_CENTER == 588


def test_arduino_mapping_has_asymmetric_motion_boundaries() -> None:
    assert FAKE.steering_command_to_adc(90) == 598
    assert FAKE.steering_command_to_adc(91) == 599
    # Arduino map() anchors its integer division at -1000, so the actual
    # right-side boundary differs from the algebraic shortcut in the prompt.
    assert FAKE.steering_command_to_adc(-81) == 578
    assert FAKE.steering_command_to_adc(-82) == 577
    assert FAKE.steering_command_to_adc(1000) == FAKE.ADC_LEFT
    assert FAKE.steering_command_to_adc(-1000) == FAKE.ADC_RIGHT
