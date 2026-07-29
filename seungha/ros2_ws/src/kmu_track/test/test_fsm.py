"""Mission FSM unit tests."""

from kmu_track.fsm import MissionFSM, MissionState


LAP_EVENTS = (
    'cone_complete',
    'static_obstacle_zone',
    'fixed_obstacle_clear',
    'overtake_complete',
    'left_signal',
    'shortcut_complete',
    'lap_complete',
)


def test_nominal_three_laps_finish() -> None:
    fsm = MissionFSM(now=0.0)
    now = 1.0
    assert fsm.handle_event('start_signal', now)
    for lap in range(3):
        for event in LAP_EVENTS:
            now += 0.1
            assert fsm.handle_event(event, now)
        assert fsm.completed_laps == lap + 1
    assert fsm.state == MissionState.FINISHED
    assert fsm.snapshot(now).stop_requested


def test_out_of_order_event_is_ignored() -> None:
    fsm = MissionFSM(now=0.0)
    assert not fsm.handle_event('overtake_complete', 1.0)
    assert fsm.state == MissionState.WAIT_START
    assert fsm.completed_laps == 0


def test_stall_requests_recovery_then_aborts_before_rule_limit() -> None:
    fsm = MissionFSM(
        stall_warning_sec=3.0,
        stop_guard_sec=55.0,
        now=0.0,
    )
    assert fsm.handle_event('start_signal', 1.0)
    fsm.tick(1.0, 0.0)
    warning = fsm.tick(4.1, 0.0)
    assert warning.stall_detected
    assert warning.recovery_requested
    aborted = fsm.tick(56.1, 0.0)
    assert aborted.state == MissionState.ABORTED
    assert aborted.abort_reason == 'continuous_stop_guard'
    assert aborted.stop_requested


def test_mission_time_guard_uses_safety_margin() -> None:
    fsm = MissionFSM(mission_timeout_sec=235.0, now=0.0)
    assert fsm.handle_event('start_signal', 10.0)
    snapshot = fsm.tick(245.0, 0.5)
    assert snapshot.state == MissionState.ABORTED
    assert snapshot.abort_reason == 'mission_time_guard'


def test_reset_clears_terminal_state_and_timers() -> None:
    fsm = MissionFSM(mission_timeout_sec=20.0, now=0.0)
    fsm.handle_event('start_signal', 1.0)
    fsm.tick(21.0, 0.5)
    assert fsm.state == MissionState.ABORTED
    assert fsm.handle_event('reset', 22.0)
    snapshot = fsm.snapshot(23.0)
    assert snapshot.state == MissionState.WAIT_START
    assert snapshot.elapsed_sec == 0.0
    assert not snapshot.stop_requested
