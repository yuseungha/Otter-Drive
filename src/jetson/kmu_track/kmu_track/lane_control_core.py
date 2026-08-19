"""ROS-independent guarded lane-following controller."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Callable, Optional, Sequence


@dataclass(frozen=True)
class LaneControlConfig:
    """Tunable lane controller and safety-gate parameters."""

    enabled: bool = False
    steering_only: bool = True
    require_serial_ready: bool = True
    ignore_mission_state: bool = False
    active_states: Sequence[str] = ('LANE_FOLLOW', 'LAP_RUN')
    error_timeout_sec: float = 0.30
    min_confidence: float = 0.30
    error_jump_limit: float = 0.45
    error_lpf_alpha: float = 0.65
    kp: float = 0.80
    kd: float = 0.12
    ki: float = 0.0
    k_heading: float = 0.35
    integral_limit: float = 0.20
    steering_sign: int = -1
    max_counts: int = 600
    deadband_counts: int = 70
    deadband_counts_left: int = 0
    deadband_counts_right: int = 0
    max_counts_left: int = 0
    max_counts_right: int = 0
    steer_epsilon: float = 0.03
    max_delta_counts_per_tick: int = 120
    lost_hold_sec: float = 0.40
    lost_decay_sec: float = 1.00
    recover_frames: int = 3
    throttle_base: int = 300
    throttle_curve: int = 180
    throttle_lost: int = 0
    gear: int = -1
    arming_neutral_sec: float = 1.0
    speed_timeout_sec: float = 0.50
    stopped_speed_mps: float = 0.03
    center_fallback_delay_sec: float = 0.75

    def __post_init__(self) -> None:
        if self.error_timeout_sec <= 0.0:
            raise ValueError('error_timeout_sec must be positive')
        if not 0.0 <= self.error_lpf_alpha < 1.0:
            raise ValueError('error_lpf_alpha must be in [0, 1)')
        if self.max_counts <= 0:
            raise ValueError('max_counts must be positive')
        if not 0 <= self.deadband_counts < self.max_counts:
            raise ValueError('deadband_counts must be below max_counts')
        for side in ('left', 'right'):
            deadband = getattr(self, f'deadband_counts_{side}')
            maximum = getattr(self, f'max_counts_{side}')
            effective_deadband = (
                self.deadband_counts if deadband <= 0 else deadband)
            effective_maximum = self.max_counts if maximum <= 0 else maximum
            if not 0 <= effective_deadband < effective_maximum:
                raise ValueError(
                    f'{side} deadband must be non-negative and below max')
        if self.max_delta_counts_per_tick <= 0:
            raise ValueError('max_delta_counts_per_tick must be positive')
        if self.lost_hold_sec < 0.0 or self.lost_decay_sec < self.lost_hold_sec:
            raise ValueError('lane-loss durations are invalid')
        if self.recover_frames < 1:
            raise ValueError('recover_frames must be at least one')
        if self.gear not in (-1, 1):
            raise ValueError('gear must be -1 (LOW) or 1 (HIGH)')
        if self.steering_sign not in (-1, 1):
            raise ValueError('steering_sign must be -1 or 1')


@dataclass(frozen=True)
class LaneControlOutput:
    """One command plus the exact values shown by the visualizer HUD."""

    publish: bool
    throttle: int
    steering: int
    gear: int
    gate_reason: str
    raw_error: float
    filtered_error: float
    heading: float
    p_term: float
    d_term: float
    i_term: float
    h_term: float
    saturated: bool
    rate_limited: bool
    lost_sec: float
    tick_age_ms: float
    sample_rejected: bool
    deadband_applied: int = 0
    max_applied: int = 0
    side: str = 'CENTER'

    def status(self) -> dict:
        """Return a JSON-ready status dictionary."""
        data = asdict(self)
        data['steering_counts'] = data.pop('steering')
        return data


class LaneControlController:
    """Filter lane error and map it to guarded RC-car commands."""

    def __init__(
        self,
        config: LaneControlConfig = LaneControlConfig(),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self.raw_error = 0.0
        self.filtered_error = 0.0
        self.derivative = 0.0
        self.integral = 0.0
        self.heading = 0.0
        self.confidence = 0.0
        self.last_error_at: Optional[float] = None
        self.last_filter_at: Optional[float] = None
        self.last_heading_at: Optional[float] = None
        self.lane_valid = False
        self.lost_since: Optional[float] = None
        self.recover_count = 0
        self.mission_state = 'WAIT_START'
        self.stop_requested = False
        self.estop_latched = False
        self.command_stale = False
        self.speed_mps = 0.0
        self.last_speed_at: Optional[float] = None
        self.serial_ready = False
        self.serial_ready_at: Optional[float] = None
        self.last_steering = 0
        self.last_throttle = 0
        self._loss_start_steering = 0
        self._loss_start_throttle = 0
        self._stop_since: Optional[float] = None
        self.sample_rejected = False

    def update_lane_sample(
        self,
        error: float,
        valid: bool,
        confidence: float,
        now: Optional[float] = None,
    ) -> None:
        """Consume one synchronized perception sample."""
        timestamp = self._clock() if now is None else float(now)
        self.last_error_at = timestamp
        self.raw_error = max(-1.0, min(1.0, float(error)))
        self.confidence = max(0.0, min(1.0, float(confidence)))
        sample_valid = bool(valid) and self.confidence >= self.config.min_confidence
        self.sample_rejected = False

        if sample_valid:
            if self.lost_since is not None:
                self.recover_count += 1
                if self.recover_count >= self.config.recover_frames:
                    self.lost_since = None
                    self.recover_count = 0
                    self.derivative = 0.0
                    self.integral = 0.0
                    self.last_filter_at = timestamp
            else:
                self.recover_count = self.config.recover_frames
            self.lane_valid = self.lost_since is None
        else:
            self.recover_count = 0
            self.lane_valid = False
            if self.lost_since is None:
                self.lost_since = timestamp
                self._loss_start_steering = self.last_steering
                self._loss_start_throttle = self.last_throttle

        if not sample_valid:
            return
        if (
            self.last_filter_at is not None
            and abs(self.raw_error - self.filtered_error) > self.config.error_jump_limit
        ):
            self.sample_rejected = True
            return

        if self.last_filter_at is None:
            self.filtered_error = self.raw_error
            self.derivative = 0.0
            self.last_filter_at = timestamp
            return
        previous = self.filtered_error
        alpha = self.config.error_lpf_alpha
        self.filtered_error = alpha * previous + (1.0 - alpha) * self.raw_error
        dt = max(1e-3, timestamp - self.last_filter_at)
        self.derivative = (self.filtered_error - previous) / dt
        self.last_filter_at = timestamp

    def update_heading(self, heading: float, now: Optional[float] = None) -> None:
        """Store an optional curvature look-ahead input."""
        self.heading = max(-1.0, min(1.0, float(heading)))
        self.last_heading_at = self._clock() if now is None else float(now)

    def update_mission_state(self, state: str) -> None:
        """Store the existing mission FSM state without changing the FSM."""
        self.mission_state = str(state).strip().upper()

    def update_stop_requested(self, requested: bool) -> None:
        """Store the highest-priority external stop request."""
        self.stop_requested = bool(requested)

    def update_estop_latched(self, latched: bool) -> None:
        """Store the bridge-owned emergency-stop latch state."""
        self.estop_latched = bool(latched)

    def update_command_stale(self, stale: bool) -> None:
        """Store the bridge freshness gate for status and defense in depth."""
        self.command_stale = bool(stale)

    def update_speed(self, speed_mps: float, now: Optional[float] = None) -> None:
        """Store optional speed feedback used for safe centering."""
        self.speed_mps = float(speed_mps)
        self.last_speed_at = self._clock() if now is None else float(now)

    def update_serial_ready(self, ready: bool, now: Optional[float] = None) -> None:
        """Start a neutral arming interval after a serial connection."""
        timestamp = self._clock() if now is None else float(now)
        ready = bool(ready)
        if ready and not self.serial_ready:
            self.serial_ready_at = timestamp
        if not ready:
            self.serial_ready_at = None
        self.serial_ready = ready

    def _heading_term(self, now: float) -> float:
        if (
            self.last_heading_at is None
            or now - self.last_heading_at > self.config.error_timeout_sec
        ):
            return 0.0
        return self.config.k_heading * self.heading

    def _rate_limit(self, target: int) -> tuple[int, bool]:
        delta = int(target) - self.last_steering
        maximum = self.config.max_delta_counts_per_tick
        if abs(delta) <= maximum:
            return int(target), False
        limited = self.last_steering + (maximum if delta > 0 else -maximum)
        return int(limited), True

    def _can_center(self, now: float) -> bool:
        if (
            self.last_speed_at is not None
            and now - self.last_speed_at <= self.config.speed_timeout_sec
        ):
            return abs(self.speed_mps) <= self.config.stopped_speed_mps
        return (
            self._stop_since is not None
            and now - self._stop_since >= self.config.center_fallback_delay_sec
        )

    def _make_output(
        self,
        publish: bool,
        throttle: int,
        steering: int,
        gate_reason: str,
        now: float,
        p_term: float = 0.0,
        d_term: float = 0.0,
        i_term: float = 0.0,
        h_term: float = 0.0,
        saturated: bool = False,
        rate_limited: bool = False,
        deadband_applied: int = 0,
        max_applied: int = 0,
        side: str = 'CENTER',
    ) -> LaneControlOutput:
        self.last_throttle = int(throttle)
        self.last_steering = int(steering)
        age_ms = (
            float('inf')
            if self.last_error_at is None
            else max(0.0, now - self.last_error_at) * 1000.0
        )
        lost_sec = (
            0.0 if self.lost_since is None
            else max(0.0, now - self.lost_since)
        )
        return LaneControlOutput(
            publish=publish,
            throttle=int(throttle),
            steering=int(steering),
            gear=self.config.gear,
            gate_reason=gate_reason,
            raw_error=self.raw_error,
            filtered_error=self.filtered_error,
            heading=self.heading if h_term != 0.0 else 0.0,
            p_term=p_term,
            d_term=d_term,
            i_term=i_term,
            h_term=h_term,
            saturated=saturated,
            rate_limited=rate_limited,
            lost_sec=lost_sec,
            tick_age_ms=age_ms,
            sample_rejected=self.sample_rejected,
            deadband_applied=int(deadband_applied),
            max_applied=int(max_applied),
            side=str(side),
        )

    def _safe_stop(self, reason: str, now: float) -> LaneControlOutput:
        if self._stop_since is None:
            self._stop_since = now
        target = 0 if self._can_center(now) else self.last_steering
        steering, rate_limited = self._rate_limit(target)
        return self._make_output(
            True, 0, steering, reason, now, rate_limited=rate_limited)

    def _lane_loss_output(self, now: float) -> LaneControlOutput:
        elapsed = max(0.0, now - float(self.lost_since))
        if elapsed <= self.config.lost_hold_sec:
            self._stop_since = None
            return self._make_output(
                True,
                self._loss_start_throttle,
                self._loss_start_steering,
                'lane_lost_hold',
                now,
            )
        if elapsed <= self.config.lost_decay_sec:
            span = max(1e-6, self.config.lost_decay_sec - self.config.lost_hold_sec)
            remaining = 1.0 - (elapsed - self.config.lost_hold_sec) / span
            steering_target = int(round(self._loss_start_steering * remaining))
            throttle = int(round(
                self.config.throttle_lost
                + (self._loss_start_throttle - self.config.throttle_lost) * remaining
            ))
            steering, rate_limited = self._rate_limit(steering_target)
            return self._make_output(
                True,
                throttle,
                steering,
                'lane_lost_decay',
                now,
                rate_limited=rate_limited,
            )
        return self._safe_stop('lane_lost', now)

    def command(self, now: Optional[float] = None) -> LaneControlOutput:
        """Return the next guarded command and its display status."""
        timestamp = self._clock() if now is None else float(now)
        if not self.config.enabled:
            return self._make_output(False, 0, 0, 'disabled', timestamp)
        if self.estop_latched:
            return self._safe_stop('estop_latched', timestamp)
        if self.command_stale:
            return self._safe_stop('command_stale', timestamp)
        if self.config.require_serial_ready and not self.serial_ready:
            return self._safe_stop('serial_not_ready', timestamp)
        if (
            self.config.require_serial_ready
            and self.serial_ready_at is not None
            and timestamp - self.serial_ready_at < self.config.arming_neutral_sec
        ):
            return self._safe_stop('serial_arming_neutral', timestamp)
        if (
            self.last_error_at is None
            or timestamp - self.last_error_at > self.config.error_timeout_sec
        ):
            return self._safe_stop('input_stale', timestamp)
        if (
            not self.config.ignore_mission_state
            and self.mission_state not in set(self.config.active_states)
        ):
            return self._safe_stop('state_not_active', timestamp)
        if self.stop_requested:
            return self._safe_stop('stop_requested', timestamp)
        if not self.lane_valid or self.lost_since is not None:
            return self._lane_loss_output(timestamp)

        self._stop_since = None
        p_term = self.config.kp * self.filtered_error
        d_term = self.config.kd * self.derivative
        h_term = self._heading_term(timestamp)
        candidate_integral = max(
            -self.config.integral_limit,
            min(
                self.config.integral_limit,
                self.integral + self.filtered_error * 0.05,
            ),
        )
        i_term = self.config.ki * candidate_integral
        normalized = p_term + d_term + i_term + h_term
        saturated = abs(normalized) > 1.0
        if not saturated:
            self.integral = candidate_integral
        normalized = max(-1.0, min(1.0, normalized))
        signed = self.config.steering_sign * normalized
        if abs(signed) < self.config.steer_epsilon:
            target_steering = 0
            side = 'CENTER'
            deadband_applied = 0
            max_applied = 0
        else:
            side = 'LEFT' if signed > 0.0 else 'RIGHT'
            suffix = 'left' if signed > 0.0 else 'right'
            configured_deadband = getattr(
                self.config, f'deadband_counts_{suffix}')
            configured_maximum = getattr(self.config, f'max_counts_{suffix}')
            deadband_applied = (
                self.config.deadband_counts
                if configured_deadband <= 0 else configured_deadband)
            max_applied = (
                self.config.max_counts
                if configured_maximum <= 0 else configured_maximum)
            magnitude = (
                deadband_applied
                + (max_applied - deadband_applied) * abs(signed)
            )
            target_steering = int(round(magnitude if signed > 0.0 else -magnitude))
        target_steering = max(
            -max_applied if side != 'CENTER' else 0,
            min(max_applied if side != 'CENTER' else 0, target_steering),
        )
        steering, rate_limited = self._rate_limit(target_steering)
        steering_ratio = abs(steering) / max(
            1.0, float(max_applied or self.config.max_counts))
        throttle = 0 if self.config.steering_only else int(round(
            self.config.throttle_base
            + (self.config.throttle_curve - self.config.throttle_base) * steering_ratio
        ))
        return self._make_output(
            True,
            throttle,
            steering,
            'running',
            timestamp,
            p_term=p_term,
            d_term=d_term,
            i_term=i_term,
            h_term=h_term,
            saturated=saturated,
            rate_limited=rate_limited,
            deadband_applied=deadband_applied,
            max_applied=max_applied,
            side=side,
        )
