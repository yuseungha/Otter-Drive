from laptop_teleop.control_core import ControlState


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_starts_neutral_and_disarmed():
    state = ControlState(0.25)
    assert state.command_for_publish() == (0, 0)
    assert not state.snapshot().armed


def test_only_active_operator_can_command():
    state = ControlState(0.25)
    assert state.arm("laptop-a", allowed=True)[0]
    assert not state.update("laptop-b", 100, 0)[0]
    assert state.update("laptop-a", 100, -200)[0]
    assert state.command_for_publish() == (100, -200)


def test_heartbeat_timeout_disarms_and_neutralizes():
    clock = FakeClock()
    state = ControlState(0.25, clock=clock)
    assert state.arm("laptop", allowed=True)[0]
    assert state.update("laptop", 150, 300)[0]
    clock.now += 0.26
    assert state.command_for_publish() == (0, 0)
    snapshot = state.snapshot()
    assert not snapshot.armed
    assert snapshot.stop_reason == "browser_heartbeat_timeout"


def test_commands_are_clamped():
    state = ControlState(0.25)
    assert state.arm("laptop", allowed=True)[0]
    assert state.update("laptop", 9999, -9999)[0]
    assert state.command_for_publish() == (1050, -1000)


def test_live_mode_rejects_arm_without_serial():
    state = ControlState(0.25)
    ok, reason = state.arm("laptop", allowed=False)
    assert not ok
    assert reason == "serial_not_ready"
