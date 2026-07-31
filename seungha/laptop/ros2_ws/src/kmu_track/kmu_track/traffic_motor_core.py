"""ROS-independent safe motor command mapping for traffic-light states."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Optional


THROTTLE_MIN = -1000
THROTTLE_MAX = 1050
STEERING_MIN = -1000
STEERING_MAX = 1000
GEAR_LOW = -1
VALID_SIGNALS = {'STOP', 'GO', 'TURN LEFT'}


@dataclass(frozen=True)
class DriveCommand:
    """One normalized throttle, steering, and gear command."""

    throttle: int
    steering: int
    gear: int
    reason: str


class TrafficMotorController:
    """
    Map filtered traffic signals to guarded RC-car drive commands.

    STOP always removes throttle immediately. Steering is centered only after
    fresh speed feedback says the car is stopped, or after a configurable
    no-speed-feedback fallback delay. GO and TURN LEFT ramp throttle upward.
    """

    def __init__(
        self,
        enabled: bool = False,
        go_throttle: int = 150,
        turn_throttle: int = 100,
        left_steering: int = 350,
        gear: int = GEAR_LOW,
        signal_timeout_sec: float = 0.50,
        speed_timeout_sec: float = 0.50,
        stopped_speed_mps: float = 0.03,
        center_fallback_delay_sec: float = 0.75,
        throttle_ramp_per_sec: float = 300.0,
        arming_neutral_sec: float = 1.0,
        require_serial_ready: bool = True,
        manual_mode: bool = False,
        manual_timeout_sec: float = 0.50,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if signal_timeout_sec <= 0.0:
            raise ValueError('signal_timeout_sec must be positive')
        if speed_timeout_sec <= 0.0:
            raise ValueError('speed_timeout_sec must be positive')
        if center_fallback_delay_sec < 0.0:
            raise ValueError('center_fallback_delay_sec cannot be negative')
        if throttle_ramp_per_sec <= 0.0:
            raise ValueError('throttle_ramp_per_sec must be positive')
        if arming_neutral_sec < 0.0:
            raise ValueError('arming_neutral_sec cannot be negative')
        if manual_timeout_sec <= 0.0:
            raise ValueError('manual_timeout_sec must be positive')
        if gear not in (-1, 1):
            raise ValueError('gear must be -1 (LOW) or 1 (HIGH)')
        self.enabled = bool(enabled)
        self.go_throttle = self._limit_throttle(go_throttle)
        self.turn_throttle = self._limit_throttle(turn_throttle)
        self.left_steering = self._limit_steering(left_steering)
        self.gear = int(gear)
        self.signal_timeout_sec = float(signal_timeout_sec)
        self.speed_timeout_sec = float(speed_timeout_sec)
        self.stopped_speed_mps = abs(float(stopped_speed_mps))
        self.center_fallback_delay_sec = float(center_fallback_delay_sec)
        self.throttle_ramp_per_sec = float(throttle_ramp_per_sec)
        self.arming_neutral_sec = float(arming_neutral_sec)
        self.require_serial_ready = bool(require_serial_ready)
        self.manual_mode = bool(manual_mode)
        self.manual_timeout_sec = float(manual_timeout_sec)
        self._clock = clock

        now = self._clock()
        self.signal = 'STOP'
        self.last_signal_at: Optional[float] = None
        self.speed_mps = 0.0
        self.last_speed_at: Optional[float] = None
        self.serial_ready = False
        self.serial_ready_at: Optional[float] = None
        self.manual_throttle = 0
        self.manual_steering = 0
        self.manual_gear = self.gear
        self.last_manual_at: Optional[float] = None
        self.stop_since = now
        self.last_tick_at = now
        self.current_throttle = 0.0
        self.current_steering = 0

    @staticmethod
    def _limit_throttle(value: int) -> int:
        return max(THROTTLE_MIN, min(THROTTLE_MAX, int(value)))

    @staticmethod
    def _limit_steering(value: int) -> int:
        return max(STEERING_MIN, min(STEERING_MAX, int(value)))

    def update_signal(self, signal: str) -> None:
        """Store a perception state; unknown values become STOP."""
        now = self._clock()
        normalized = str(signal).strip().upper()
        if normalized not in VALID_SIGNALS:
            normalized = 'STOP'
        if normalized == 'STOP' and self.signal != 'STOP':
            self.stop_since = now
        self.signal = normalized
        self.last_signal_at = now

    def update_speed(self, speed_mps: float) -> None:
        """Store the latest vehicle speed feedback."""
        self.speed_mps = float(speed_mps)
        self.last_speed_at = self._clock()

    def update_serial_ready(self, ready: bool) -> None:
        """Start a neutral arming interval after each serial connection."""
        now = self._clock()
        ready = bool(ready)
        if ready and not self.serial_ready:
            self.serial_ready_at = now
        if not ready:
            self.serial_ready_at = None
        self.serial_ready = ready

    def update_manual_command(
        self,
        throttle: int,
        steering: int,
        gear: int = GEAR_LOW,
    ) -> None:
        """Store the operator request consumed by manual safety-gate mode."""
        normalized_gear = int(gear)
        if normalized_gear not in (-1, 1):
            normalized_gear = GEAR_LOW
        self.manual_throttle = self._limit_throttle(throttle)
        self.manual_steering = self._limit_steering(steering)
        self.manual_gear = normalized_gear
        self.gear = normalized_gear
        self.last_manual_at = self._clock()

    def _neutral(self, reason: str) -> DriveCommand:
        self.current_throttle = 0.0
        self.current_steering = 0
        return DriveCommand(0, 0, self.gear, reason)

    def _signal_is_fresh(self, now: float) -> bool:
        return (
            self.last_signal_at is not None
            and now - self.last_signal_at <= self.signal_timeout_sec
        )

    def _speed_is_fresh(self, now: float) -> bool:
        return (
            self.last_speed_at is not None
            and now - self.last_speed_at <= self.speed_timeout_sec
        )

    def _manual_is_fresh(self, now: float) -> bool:
        return (
            self.last_manual_at is not None
            and now - self.last_manual_at <= self.manual_timeout_sec
        )

    def _can_center(self, now: float) -> bool:
        if self._speed_is_fresh(now):
            return abs(self.speed_mps) <= self.stopped_speed_mps
        return now - self.stop_since >= self.center_fallback_delay_sec

    def _ramp_throttle(self, target: int, now: float) -> int:
        elapsed = max(0.0, now - self.last_tick_at)
        max_step = self.throttle_ramp_per_sec * elapsed
        delta = float(target) - self.current_throttle
        if abs(delta) <= max_step:
            self.current_throttle = float(target)
        elif delta > 0.0:
            self.current_throttle += max_step
        else:
            self.current_throttle -= max_step
        return int(round(self.current_throttle))

    def command(self) -> Optional[DriveCommand]:
        """Return the next command, or None while output is disabled."""
        now = self._clock()
        if not self.enabled:
            self.last_tick_at = now
            return None
        if self.require_serial_ready and not self.serial_ready:
            self.last_tick_at = now
            return self._neutral('serial_not_ready')
        if (
            self.require_serial_ready
            and self.serial_ready_at is not None
            and now - self.serial_ready_at < self.arming_neutral_sec
        ):
            self.last_tick_at = now
            return self._neutral('serial_arming_neutral')
        if not self._signal_is_fresh(now):
            if self.current_throttle != 0.0:
                self.stop_since = now
            self.last_tick_at = now
            return self._neutral('signal_stale')
        if self.manual_mode and not self._manual_is_fresh(now):
            if self.current_throttle != 0.0:
                self.stop_since = now
            self.last_tick_at = now
            return self._neutral('manual_command_stale')

        if self.signal == 'STOP':
            self.current_throttle = 0.0
            if self._can_center(now):
                self.current_steering = 0
                reason = 'stop_centered'
            else:
                reason = 'stop_waiting_to_center'
            self.last_tick_at = now
            return DriveCommand(
                0,
                self.current_steering,
                self.gear,
                reason,
            )

        if self.signal == 'TURN LEFT':
            if self.manual_mode:
                target_throttle = max(
                    0,
                    min(self.turn_throttle, self.manual_throttle),
                )
                reason = 'manual_turn_left'
            else:
                target_throttle = self.turn_throttle
                reason = 'turn_left'
            self.current_steering = self.left_steering
        else:
            if self.manual_mode:
                target_throttle = self.manual_throttle
                self.current_steering = self.manual_steering
                reason = 'manual_go'
            else:
                target_throttle = self.go_throttle
                self.current_steering = 0
                reason = 'go_straight'
        throttle = self._ramp_throttle(target_throttle, now)
        self.last_tick_at = now
        return DriveCommand(
            throttle,
            self.current_steering,
            self.gear,
            reason,
        )
