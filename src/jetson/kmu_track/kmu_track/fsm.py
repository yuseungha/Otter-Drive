"""Hardware-independent mission finite-state machine."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class MissionState(str, Enum):
    """Competition mission phases."""

    WAIT_START = 'WAIT_START'
    CONE_SLALOM = 'CONE_SLALOM'
    LANE_FOLLOW = 'LANE_FOLLOW'
    STATIC_AVOID = 'STATIC_AVOID'
    OVERTAKE = 'OVERTAKE'
    SHORTCUT_WAIT = 'SHORTCUT_WAIT'
    SHORTCUT_LEFT = 'SHORTCUT_LEFT'
    LAP_RUN = 'LAP_RUN'
    FINISHED = 'FINISHED'
    ABORTED = 'ABORTED'


MOVING_STATES = {
    MissionState.CONE_SLALOM,
    MissionState.LANE_FOLLOW,
    MissionState.STATIC_AVOID,
    MissionState.OVERTAKE,
    MissionState.SHORTCUT_LEFT,
    MissionState.LAP_RUN,
}


TRANSITIONS = {
    (MissionState.WAIT_START, 'start_signal'): MissionState.CONE_SLALOM,
    (MissionState.CONE_SLALOM, 'cone_complete'): MissionState.LANE_FOLLOW,
    (MissionState.LANE_FOLLOW, 'static_obstacle_zone'): MissionState.STATIC_AVOID,
    (MissionState.STATIC_AVOID, 'fixed_obstacle_clear'): MissionState.OVERTAKE,
    (MissionState.OVERTAKE, 'overtake_complete'): MissionState.SHORTCUT_WAIT,
    (MissionState.SHORTCUT_WAIT, 'left_signal'): MissionState.SHORTCUT_LEFT,
    (MissionState.SHORTCUT_LEFT, 'shortcut_complete'): MissionState.LAP_RUN,
}


EVENT_ALIASES = {
    'green': 'start_signal',
    'manual_start': 'start_signal',
    'start_green': 'start_signal',
    'left_arrow_green': 'left_signal',
}


@dataclass(frozen=True)
class MissionSnapshot:
    """Observable FSM state at one instant."""

    state: MissionState
    completed_laps: int
    elapsed_sec: float
    remaining_sec: float
    stopped_sec: float
    stall_detected: bool
    recovery_requested: bool
    stop_requested: bool
    abort_reason: str
    last_event: str


class MissionFSM:
    """Deterministic mission FSM with competition safety guards."""

    def __init__(
        self,
        target_laps: int = 3,
        mission_timeout_sec: float = 235.0,
        stall_warning_sec: float = 3.0,
        stop_guard_sec: float = 55.0,
        stopped_speed_mps: float = 0.03,
        now: float = 0.0,
    ) -> None:
        if target_laps < 1:
            raise ValueError('target_laps must be at least 1')
        if not 0.0 < mission_timeout_sec <= 240.0:
            raise ValueError('mission_timeout_sec must be in (0, 240]')
        if not 0.0 <= stall_warning_sec < stop_guard_sec <= 60.0:
            raise ValueError('stall thresholds must satisfy 0 <= warning < guard <= 60')

        self.target_laps = target_laps
        self.mission_timeout_sec = mission_timeout_sec
        self.stall_warning_sec = stall_warning_sec
        self.stop_guard_sec = stop_guard_sec
        self.stopped_speed_mps = stopped_speed_mps
        self.reset(now)

    def reset(self, now: float) -> None:
        """Reset all run state and return to the start signal wait."""
        self.state = MissionState.WAIT_START
        self.completed_laps = 0
        self.mission_started_at: Optional[float] = None
        self.state_entered_at = now
        self.stopped_since: Optional[float] = None
        self.stall_detected = False
        self.recovery_requested = False
        self.abort_reason = ''
        self.last_event = 'reset'

    def _set_state(self, state: MissionState, now: float) -> None:
        self.state = state
        self.state_entered_at = now
        self.stopped_since = None
        self.stall_detected = False
        self.recovery_requested = False

    def abort(self, reason: str, now: float) -> None:
        """Enter the terminal safe-stop state."""
        self.abort_reason = reason
        self._set_state(MissionState.ABORTED, now)

    def handle_event(self, event: str, now: float) -> bool:
        """Apply one normalized perception or course event."""
        event = EVENT_ALIASES.get(event.strip().lower(), event.strip().lower())
        self.last_event = event

        if event == 'reset':
            self.reset(now)
            return True
        if self.state in {MissionState.FINISHED, MissionState.ABORTED}:
            return False

        if self.state == MissionState.LAP_RUN and event == 'lap_complete':
            self.completed_laps += 1
            if self.completed_laps >= self.target_laps:
                self._set_state(MissionState.FINISHED, now)
            else:
                self._set_state(MissionState.CONE_SLALOM, now)
            return True

        next_state = TRANSITIONS.get((self.state, event))
        if next_state is None:
            return False

        if self.state == MissionState.WAIT_START:
            self.mission_started_at = now
        self._set_state(next_state, now)
        return True

    def tick(self, now: float, speed_mps: float) -> MissionSnapshot:
        """Update time and stall guards, then return the current snapshot."""
        if self.mission_started_at is not None and self.state not in {
            MissionState.FINISHED,
            MissionState.ABORTED,
        }:
            if now - self.mission_started_at >= self.mission_timeout_sec:
                self.abort('mission_time_guard', now)

        if self.state in MOVING_STATES:
            if abs(speed_mps) <= self.stopped_speed_mps:
                if self.stopped_since is None:
                    self.stopped_since = now
                stopped_sec = now - self.stopped_since
                self.stall_detected = stopped_sec >= self.stall_warning_sec
                self.recovery_requested = self.stall_detected
                if stopped_sec >= self.stop_guard_sec:
                    self.abort('continuous_stop_guard', now)
            else:
                self.stopped_since = None
                self.stall_detected = False
                self.recovery_requested = False
        else:
            self.stopped_since = None
            self.stall_detected = False
            self.recovery_requested = False

        return self.snapshot(now)

    def snapshot(self, now: float) -> MissionSnapshot:
        """Return state without mutating timers."""
        if self.mission_started_at is None:
            elapsed_sec = 0.0
        else:
            elapsed_sec = max(0.0, now - self.mission_started_at)
        stopped_sec = 0.0 if self.stopped_since is None else max(
            0.0, now - self.stopped_since)
        remaining_sec = max(0.0, self.mission_timeout_sec - elapsed_sec)
        return MissionSnapshot(
            state=self.state,
            completed_laps=self.completed_laps,
            elapsed_sec=elapsed_sec,
            remaining_sec=remaining_sec,
            stopped_sec=stopped_sec,
            stall_detected=self.stall_detected,
            recovery_requested=self.recovery_requested,
            stop_requested=self.state in {
                MissionState.FINISHED,
                MissionState.ABORTED,
            },
            abort_reason=self.abort_reason,
            last_event=self.last_event,
        )
