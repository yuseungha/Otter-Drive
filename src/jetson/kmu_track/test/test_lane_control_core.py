"""Unit tests for the hardware-independent lane controller."""

from pathlib import Path
import re

from kmu_track.lane_control_core import (
    LaneControlConfig,
    LaneControlController,
)
from rc_car_teleop import firmware_model


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _integer_setting(path, pattern):
    match = re.search(pattern, path.read_text(encoding='utf-8'), re.MULTILINE)
    assert match is not None
    return int(match.group(1))


def _float_setting(path, name):
    match = re.search(
        rf'^\s*{re.escape(name)}:\s*([-+]?[0-9]*\.?[0-9]+)\s*$',
        path.read_text(encoding='utf-8'),
        re.MULTILINE,
    )
    assert match is not None
    return float(match.group(1))


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _controller(clock, **overrides):
    parameters = {
        'enabled': True,
        'require_serial_ready': False,
        'ignore_mission_state': True,
        'recover_frames': 1,
    }
    parameters.update(overrides)
    return LaneControlController(LaneControlConfig(**parameters), clock=clock)


def _valid_sample(controller, clock, error=0.1, confidence=0.9):
    controller.update_lane_sample(error, True, confidence, clock())


def test_positive_error_commands_negative_right_steering() -> None:
    clock = FakeClock()
    controller = _controller(clock, max_delta_counts_per_tick=1000)
    _valid_sample(controller, clock, error=0.2)
    output = controller.command()
    assert output.gate_reason == 'running'
    assert output.steering < 0


def test_deadband_leaves_tiny_error_zero_then_jumps_above_minimum() -> None:
    clock = FakeClock()
    controller = _controller(clock, max_delta_counts_per_tick=1000)
    _valid_sample(controller, clock, error=0.001)
    assert controller.command().steering == 0
    clock.advance(0.05)
    _valid_sample(controller, clock, error=0.1)
    assert abs(controller.command().steering) >= 70


def test_minimum_nonzero_output_exceeds_firmware_adc_deadband() -> None:
    config_path = PACKAGE_ROOT / 'config' / 'lane_control.yaml'
    deadband = _integer_setting(
        config_path, r'^\s*deadband_counts:\s*(\d+)\s*$')
    maximum = _integer_setting(
        config_path, r'^\s*max_counts:\s*(\d+)\s*$')
    epsilon = _float_setting(config_path, 'steer_epsilon')
    steering_sign = _integer_setting(
        config_path, r'^\s*steering_sign:\s*(-?\d+)\s*$')
    clock = FakeClock()
    controller = _controller(
        clock,
        kp=1.0,
        kd=0.0,
        ki=0.0,
        k_heading=0.0,
        error_lpf_alpha=0.0,
        steering_sign=steering_sign,
        deadband_counts=deadband,
        max_counts=maximum,
        steer_epsilon=epsilon,
        max_delta_counts_per_tick=maximum,
    )
    _valid_sample(controller, clock, error=epsilon)
    steering = controller.command().steering
    assert steering != 0
    target_adc = firmware_model.steering_command_to_adc(steering)
    assert (
        abs(target_adc - firmware_model.ADC_CENTER)
        > firmware_model.STEER_DEADBAND
    )


def test_lane_sender_and_receiver_launch_share_maximum() -> None:
    config_path = PACKAGE_ROOT / 'config' / 'lane_control.yaml'
    sender_path = PACKAGE_ROOT / 'kmu_track' / 'tcp_steering_sender.py'
    launch_path = (
        WORKSPACE_SRC / 'rc_car_teleop' / 'launch'
        / 'jetson_tcp_steering.launch.py')
    maximums = {
        _integer_setting(config_path, r'^\s*max_counts:\s*(\d+)\s*$'),
        _integer_setting(
            sender_path,
            r"declare_parameter\('max_abs_steering',\s*(\d+)\)"),
        _integer_setting(
            launch_path, r"'max_abs_steering':\s*(\d+)"),
    }
    assert len(maximums) == 1
    assert 1 <= maximums.pop() <= 1000


def test_rate_limit_and_clip_are_enforced() -> None:
    clock = FakeClock()
    limited = _controller(clock, max_delta_counts_per_tick=120)
    _valid_sample(limited, clock, error=1.0)
    assert limited.command().steering == -120

    clipped = _controller(
        clock, max_counts=600, max_delta_counts_per_tick=1000, kp=3.0)
    _valid_sample(clipped, clock, error=1.0)
    output = clipped.command()
    assert output.steering == -600
    assert output.saturated


def test_stale_input_returns_zero_throttle() -> None:
    clock = FakeClock()
    controller = _controller(clock)
    _valid_sample(controller, clock)
    controller.command()
    clock.advance(0.31)
    output = controller.command()
    assert output.gate_reason == 'input_stale'
    assert output.throttle == 0


def test_lane_loss_transitions_hold_decay_stop() -> None:
    clock = FakeClock()
    controller = _controller(clock, max_delta_counts_per_tick=1000)
    _valid_sample(controller, clock, error=0.2)
    running = controller.command()
    controller.update_lane_sample(0.0, False, 0.0, clock())
    hold = controller.command()
    assert hold.gate_reason == 'lane_lost_hold'
    assert hold.steering == running.steering

    clock.advance(0.60)
    controller.update_lane_sample(0.0, False, 0.0, clock())
    decay = controller.command()
    assert decay.gate_reason == 'lane_lost_decay'
    assert abs(decay.steering) < abs(hold.steering)

    clock.advance(0.50)
    controller.update_lane_sample(0.0, False, 0.0, clock())
    stopped = controller.command()
    assert stopped.gate_reason == 'lane_lost'
    assert stopped.throttle == 0


def test_stop_request_does_not_center_while_speed_is_nonzero() -> None:
    clock = FakeClock()
    controller = _controller(clock, max_delta_counts_per_tick=1000)
    _valid_sample(controller, clock, error=0.2)
    moving = controller.command()
    controller.update_speed(0.5, clock())
    controller.update_stop_requested(True)
    stopped = controller.command()
    assert stopped.gate_reason == 'stop_requested'
    assert stopped.throttle == 0
    assert stopped.steering == moving.steering


def test_disabled_controller_does_not_publish() -> None:
    clock = FakeClock()
    controller = LaneControlController(
        LaneControlConfig(enabled=False), clock=clock)
    output = controller.command()
    assert output.gate_reason == 'disabled'
    assert not output.publish


def test_side_specific_deadband_and_maximum_with_legacy_fallback() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        steering_only=True,
        kp=1.0,
        max_delta_counts_per_tick=2000,
        deadband_counts_left=80,
        deadband_counts_right=90,
        max_counts_left=500,
        max_counts_right=700,
    )
    _valid_sample(controller, clock, error=-1.0)
    left = controller.command()
    assert (left.side, left.steering, left.deadband_applied) == (
        'LEFT', 500, 80)
    clock.advance(0.05)
    controller.update_lane_sample(1.0, True, 0.9, clock())
    controller.filtered_error = 1.0
    right = controller.command()
    assert (right.side, right.steering, right.max_applied) == (
        'RIGHT', -700, 700)

    legacy = _controller(
        clock, kp=1.0, max_delta_counts_per_tick=1000,
        deadband_counts_left=0, max_counts_left=0)
    _valid_sample(legacy, clock, error=-0.5)
    output = legacy.command()
    assert output.deadband_applied == 70
    assert output.max_applied == 600


def test_side_specific_config_rejects_deadband_at_or_above_max() -> None:
    try:
        LaneControlConfig(deadband_counts_left=500, max_counts_left=500)
    except ValueError as error:
        assert 'left deadband' in str(error)
    else:
        raise AssertionError('invalid left side configuration was accepted')


def test_serial_ready_arming_starts_only_on_rising_edges() -> None:
    clock = FakeClock()
    controller = _controller(
        clock, require_serial_ready=True, arming_neutral_sec=1.0)
    for _ in range(100):
        controller.update_serial_ready(True, clock())
        clock.advance(0.001)
    first_edge_at = controller.serial_ready_at
    assert first_edge_at == 10.0
    assert controller.command().gate_reason == 'serial_arming_neutral'

    controller.update_serial_ready(False, clock())
    clock.advance(0.1)
    controller.update_serial_ready(True, clock())
    assert controller.serial_ready_at != first_edge_at
    assert controller.command().steering == 0


def test_estop_gate_has_priority_over_other_lane_gates() -> None:
    clock = FakeClock()
    controller = _controller(clock)
    controller.update_command_stale(True)
    controller.update_estop_latched(True)
    output = controller.command()
    assert output.gate_reason == 'estop_latched'
    assert output.throttle == 0
    controller.update_estop_latched(False)
    assert controller.command().gate_reason == 'command_stale'
