"""Unit tests for guarded traffic-light motor commands."""

from kmu_track.traffic_motor_core import TrafficMotorController


class FakeClock:
    """Deterministic monotonic clock for motor tests."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _controller(clock: FakeClock, **kwargs) -> TrafficMotorController:
    return TrafficMotorController(
        enabled=True,
        require_serial_ready=False,
        throttle_ramp_per_sec=300.0,
        clock=clock,
        **kwargs,
    )


def test_disabled_controller_publishes_nothing() -> None:
    clock = FakeClock()
    controller = TrafficMotorController(enabled=False, clock=clock)
    controller.update_signal('GO')
    assert controller.command() is None


def test_go_ramps_forward_with_centered_steering() -> None:
    clock = FakeClock()
    controller = _controller(clock)
    controller.update_signal('GO')
    clock.advance(0.25)
    command = controller.command()
    assert command.throttle == 75
    assert command.steering == 0
    assert command.gear == -1


def test_turn_left_uses_lower_throttle_and_positive_steering() -> None:
    clock = FakeClock()
    controller = _controller(clock)
    controller.update_signal('TURN LEFT')
    clock.advance(0.50)
    command = controller.command()
    assert command.throttle == 100
    assert command.steering == 350
    assert command.reason == 'turn_left'


def test_stop_removes_throttle_before_centering() -> None:
    clock = FakeClock()
    controller = _controller(clock)
    controller.update_signal('TURN LEFT')
    clock.advance(0.50)
    controller.command()
    controller.update_signal('STOP')
    controller.update_speed(0.20)
    command = controller.command()
    assert command.throttle == 0
    assert command.steering == 350
    assert command.reason == 'stop_waiting_to_center'
    controller.update_speed(0.0)
    command = controller.command()
    assert command.steering == 0
    assert command.reason == 'stop_centered'


def test_stop_centers_after_fallback_without_speed_feedback() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        center_fallback_delay_sec=0.75,
        signal_timeout_sec=2.0,
    )
    controller.update_signal('TURN LEFT')
    clock.advance(0.50)
    controller.command()
    controller.update_signal('STOP')
    clock.advance(0.74)
    assert controller.command().steering == 350
    clock.advance(0.02)
    assert controller.command().steering == 0


def test_stale_signal_forces_full_neutral() -> None:
    clock = FakeClock()
    controller = _controller(clock, signal_timeout_sec=0.50)
    controller.update_signal('TURN LEFT')
    clock.advance(0.25)
    controller.command()
    clock.advance(0.30)
    command = controller.command()
    assert command.throttle == 0
    assert command.steering == 0
    assert command.reason == 'signal_stale'


def test_serial_connection_requires_neutral_arming_time() -> None:
    clock = FakeClock()
    controller = TrafficMotorController(
        enabled=True,
        require_serial_ready=True,
        arming_neutral_sec=1.0,
        clock=clock,
    )
    controller.update_signal('GO')
    assert controller.command().reason == 'serial_not_ready'
    controller.update_serial_ready(True)
    assert controller.command().reason == 'serial_arming_neutral'
    clock.advance(1.1)
    controller.update_signal('GO')
    assert controller.command().reason == 'go_straight'


def test_manual_go_passes_operator_command_with_ramp() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        manual_mode=True,
        manual_timeout_sec=2.0,
        signal_timeout_sec=2.0,
    )
    controller.update_manual_command(180, -120, -1)
    controller.update_signal('GO')
    clock.advance(0.50)
    command = controller.command()
    assert command.throttle == 150
    assert command.steering == -120
    assert command.reason == 'manual_go'


def test_manual_command_clamps_throttle_and_steering_independently() -> None:
    clock = FakeClock()
    controller = _controller(clock, manual_mode=True)
    controller.update_manual_command(1200, -1200, -1)
    assert controller.manual_throttle == 1050
    assert controller.manual_steering == -1000


def test_manual_red_stops_then_centers_and_green_resumes() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        manual_mode=True,
        manual_timeout_sec=3.0,
        signal_timeout_sec=3.0,
        center_fallback_delay_sec=0.75,
    )
    controller.update_manual_command(180, 90, -1)
    controller.update_signal('GO')
    clock.advance(0.60)
    assert controller.command().throttle == 180

    controller.update_signal('STOP')
    stopped = controller.command()
    assert stopped.throttle == 0
    assert stopped.steering == 90
    assert stopped.reason == 'stop_waiting_to_center'

    clock.advance(0.76)
    controller.update_signal('STOP')
    assert controller.command().steering == 0

    controller.update_manual_command(180, 90, -1)
    controller.update_signal('GO')
    clock.advance(0.25)
    resumed = controller.command()
    assert resumed.throttle == 75
    assert resumed.steering == 90
    assert resumed.reason == 'manual_go'


def test_manual_left_arrow_limits_speed_and_overrides_steering() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        manual_mode=True,
        manual_timeout_sec=2.0,
        signal_timeout_sec=2.0,
    )
    controller.update_manual_command(250, -200, -1)
    controller.update_signal('TURN LEFT')
    clock.advance(0.50)
    command = controller.command()
    assert command.throttle == 100
    assert command.steering == 350
    assert command.reason == 'manual_turn_left'


def test_stale_manual_command_forces_neutral() -> None:
    clock = FakeClock()
    controller = _controller(
        clock,
        manual_mode=True,
        manual_timeout_sec=0.50,
        signal_timeout_sec=2.0,
    )
    controller.update_manual_command(150, 100, -1)
    controller.update_signal('GO')
    clock.advance(0.51)
    controller.update_signal('GO')
    command = controller.command()
    assert command.throttle == 0
    assert command.steering == 0
    assert command.reason == 'manual_command_stale'
