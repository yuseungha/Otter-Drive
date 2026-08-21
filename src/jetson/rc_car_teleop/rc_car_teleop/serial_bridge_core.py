"""ROS-independent safety state for the serial bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Optional


LEGACY_DEBUG_PATTERN = re.compile(
    r'\[DEBUG\] Throttle: (-?\d+) \| SteerTarget\(ADC\): (-?\d+) '
    r'\| SteerCurrent\(ADC\): (-?\d+) \| Error: (-?\d+)')
ACTIVE_DEBUG_PATTERN = re.compile(
    r'\[DEBUG\]\s+(?:Command=(?P<command>-?\d+)\s+)?'
    r'Current=(?P<current>-?\d+)\s+Target=(?P<target>-?\d+)\s+'
    r'Error=(?P<error>-?\d+)\s+PWM=(?P<pwm>-?\d+)\s+'
    r'Drive=(?P<drive>\S+)\s+Enabled=(?P<enabled>YES|NO)\s+'
    r'Fault=(?P<fault>YES|NO)')
VERSION_PATTERN = re.compile(
    r'^\[VERSION\]\s+sketch=(?P<sketch>\S+)\s+'
    r'build=(?P<build>\S+)\s+'
    r'LEFT=(?P<adc_left>-?\d+)\s+'
    r'CENTER=(?P<adc_center>-?\d+)\s+'
    r'RIGHT=(?P<adc_right>-?\d+)\s+'
    r'DEADBAND=(?P<steer_deadband>\d+)\s+'
    r'MINPWM=(?P<steer_min_pwm>\d+)\s+'
    r'MAXPWM=(?P<steer_max_pwm>\d+)\s+'
    r'ENA=(?P<steer_enable_pin>\S+)\s+'
    r'IN1=(?P<steer_in1_pin>\S+)\s+'
    r'IN2=(?P<steer_in2_pin>\S+)\s+'
    r'FB=(?P<steer_feedback_pin>\S+)\s+'
    r'HIGH_LOW_INCREASES_ADC=(?P<high_low_increases_adc>[01])\s+'
    r'WATCHDOG_MS=(?P<watchdog_ms>\d+)$')

SERIAL_BY_ID_DIR = Path('/dev/serial/by-id')
MIN_RESET_GUARD_SEC = 3.5
MAX_RESET_GUARD_SEC = 30.0
MIN_SEND_RATE_HZ = 10.0
MAX_SEND_RATE_HZ = 100.0
MIN_COMMAND_TIMEOUT_SEC = 0.05
MAX_COMMAND_TIMEOUT_SEC = 0.20


def validate_bridge_config(
    *,
    serial_port: str,
    baud_rate: int,
    send_rate_hz: float,
    reconnect_interval_sec: float,
    reset_guard_sec: float,
    command_timeout_sec: float,
    write_timeout_sec: float,
    drive_enabled: bool,
    limits_confirmed: bool,
    throttle_min: int,
    throttle_max: int,
    steering_min: int,
    steering_max: int,
    stale_steer_hold_sec: float,
    stale_steer_ramp_counts_per_tick: int,
    estop_center_rate_counts_per_tick: int,
    estop_center_timeout_sec: float,
    competition_no_stop_enabled: bool = False,
    competition_minimum_throttle_counts: int = 1,
) -> None:
    """Validate every timing and motion gate before serial can be opened."""

    serial_path = Path(serial_port)
    if (
        not serial_path.is_absolute()
        or serial_path.parent != SERIAL_BY_ID_DIR
        or serial_path.name in ('', '.', '..')
    ):
        raise ValueError('serial_port must be a direct /dev/serial/by-id path')
    if baud_rate != 115200:
        raise ValueError('baud_rate must remain 115200')

    finite_values = (
        ('send_rate_hz', send_rate_hz),
        ('reconnect_interval_sec', reconnect_interval_sec),
        ('reset_guard_sec', reset_guard_sec),
        ('command_timeout_sec', command_timeout_sec),
        ('write_timeout_sec', write_timeout_sec),
        ('stale_steer_hold_sec', stale_steer_hold_sec),
        ('estop_center_timeout_sec', estop_center_timeout_sec),
    )
    for name, value in finite_values:
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if not MIN_SEND_RATE_HZ <= send_rate_hz <= MAX_SEND_RATE_HZ:
        raise ValueError(
            f'send_rate_hz must be in {MIN_SEND_RATE_HZ:g}..'
            f'{MAX_SEND_RATE_HZ:g}')
    if 1.0 / send_rate_hz > command_timeout_sec:
        raise ValueError(
            'send period must not exceed command_timeout_sec')
    if not 0.1 <= reconnect_interval_sec <= 60.0:
        raise ValueError('reconnect_interval_sec must be in 0.1..60')
    if not MIN_RESET_GUARD_SEC <= reset_guard_sec <= MAX_RESET_GUARD_SEC:
        raise ValueError(
            f'reset_guard_sec must be in {MIN_RESET_GUARD_SEC:g}..'
            f'{MAX_RESET_GUARD_SEC:g}')
    if not MIN_COMMAND_TIMEOUT_SEC <= command_timeout_sec <= MAX_COMMAND_TIMEOUT_SEC:
        raise ValueError('command_timeout_sec must be in 0.05..0.20')
    if not 0.01 <= write_timeout_sec <= command_timeout_sec:
        raise ValueError(
            'write_timeout_sec must be in 0.01..command_timeout_sec')
    if not 0.0 <= stale_steer_hold_sec <= 1.0:
        raise ValueError('stale_steer_hold_sec must be in 0..1')
    if (
        isinstance(stale_steer_ramp_counts_per_tick, bool)
        or not 1 <= stale_steer_ramp_counts_per_tick <= 1000
    ):
        raise ValueError(
            'stale_steer_ramp_counts_per_tick must be in 1..1000')
    if (
        isinstance(estop_center_rate_counts_per_tick, bool)
        or not 1 <= estop_center_rate_counts_per_tick <= 1000
    ):
        raise ValueError(
            'estop_center_rate_counts_per_tick must be in 1..1000')
    if not 0.05 <= estop_center_timeout_sec <= 5.0:
        raise ValueError('estop_center_timeout_sec must be in 0.05..5')

    bounds = (
        ('throttle_min', throttle_min),
        ('throttle_max', throttle_max),
        ('steering_min', steering_min),
        ('steering_max', steering_max),
    )
    for name, value in bounds:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f'{name} must be an integer')
        if not -1000 <= value <= 1000:
            raise ValueError(f'{name} must be in -1000..1000')
    if throttle_min > throttle_max:
        raise ValueError('throttle_min exceeds throttle_max')
    if steering_min > steering_max:
        raise ValueError('steering_min exceeds steering_max')
    if drive_enabled and not limits_confirmed:
        raise ValueError('drive_enabled requires limits_confirmed=true')
    if not limits_confirmed and any(
        (throttle_min, throttle_max, steering_min, steering_max)
    ):
        raise ValueError('unconfirmed limits must all remain zero')
    if limits_confirmed and not (
        throttle_min <= 0 <= throttle_max
        and steering_min <= 0 <= steering_max
    ):
        raise ValueError('confirmed throttle and steering ranges must include zero')
    if competition_no_stop_enabled:
        if (
            isinstance(competition_minimum_throttle_counts, bool)
            or not throttle_min
            <= competition_minimum_throttle_counts
            <= throttle_max
            or competition_minimum_throttle_counts <= 0
        ):
            raise ValueError(
                'competition_minimum_throttle_counts must be positive and '
                'inside the confirmed throttle range')


def parse_debug_line(line: str) -> Optional[tuple[int, int]]:
    """Return current ADC and error for either supported firmware format."""
    match = LEGACY_DEBUG_PATTERN.search(line)
    if match:
        return int(match.group(3)), int(match.group(4))
    match = ACTIVE_DEBUG_PATTERN.search(line)
    if match:
        return int(match.group('current')), int(match.group('error'))
    return None


def parse_steering_status_line(line: str) -> Optional[dict]:
    """Return all steering telemetry fields from the active firmware."""
    match = ACTIVE_DEBUG_PATTERN.search(line)
    if not match:
        return None
    command = match.group('command')
    return {
        'command': int(command) if command is not None else None,
        'current_adc': int(match.group('current')),
        'target_adc': int(match.group('target')),
        'error_adc': int(match.group('error')),
        'pwm': int(match.group('pwm')),
        'drive': match.group('drive'),
        'enabled': match.group('enabled') == 'YES',
        'fault': match.group('fault') == 'YES',
    }


def parse_version_line(line: str) -> Optional[dict]:
    """Parse a structured active-firmware version banner."""
    match = VERSION_PATTERN.fullmatch(line.strip())
    if not match:
        return None
    values = match.groupdict()
    integer_fields = (
        'adc_left',
        'adc_center',
        'adc_right',
        'steer_deadband',
        'steer_min_pwm',
        'steer_max_pwm',
        'watchdog_ms',
    )
    for field in integer_fields:
        values[field] = int(values[field])
    values['high_low_increases_adc'] = (
        values['high_low_increases_adc'] == '1')
    return values


def steering_command_to_adc(
    command: int,
    *,
    adc_left: int,
    adc_center: int,
    adc_right: int,
) -> int:
    """Map the final-firmware steering command to its ADC target."""
    command = max(-1000, min(1000, int(command)))
    if command >= 0:
        return int(round(
            adc_center + (adc_left - adc_center) * command / 1000.0
        ))
    return int(round(
        adc_center + (adc_center - adc_right) * command / 1000.0
    ))


def steering_adc_to_command(
    target_adc: int,
    *,
    adc_left: int,
    adc_center: int,
    adc_right: int,
) -> int:
    """Invert the final-firmware piecewise steering calibration."""
    target_adc = max(adc_right, min(adc_left, int(target_adc)))
    if target_adc >= adc_center:
        command = 1000.0 * (target_adc - adc_center) / (
            adc_left - adc_center)
    else:
        command = -1000.0 * (adc_center - target_adc) / (
            adc_center - adc_right)
    return max(-1000, min(1000, int(round(command))))


def guard_steering_command_by_adc(
    desired_command: int,
    current_adc: int,
    *,
    max_error_adc: int,
    adc_left: int,
    adc_center: int,
    adc_right: int,
) -> int:
    """Keep a steering target close enough to feedback to avoid stall latch.

    The desired command remains the eventual destination.  Each call advances
    the firmware target by at most ``max_error_adc`` from measured position,
    so movement automatically releases more steering authority.
    """
    if not adc_right < adc_center < adc_left:
        raise ValueError('steering ADC calibration must satisfy right<center<left')
    if not 1 <= int(max_error_adc) <= 24:
        raise ValueError('max_error_adc must be in 1..24')
    desired_adc = steering_command_to_adc(
        desired_command,
        adc_left=adc_left,
        adc_center=adc_center,
        adc_right=adc_right,
    )
    delta = desired_adc - int(current_adc)
    if abs(delta) <= int(max_error_adc):
        return max(-1000, min(1000, int(desired_command)))
    bounded_target = int(current_adc) + max(
        -int(max_error_adc), min(int(max_error_adc), delta)
    )
    return steering_adc_to_command(
        bounded_target,
        adc_left=adc_left,
        adc_center=adc_center,
        adc_right=adc_right,
    )


def ramp_to_zero(value: int, step: int) -> int:
    """Move an integer toward zero without crossing it."""
    if value > 0:
        return max(0, value - step)
    if value < 0:
        return min(0, value + step)
    return 0


def serial_ready_condition(
    connected: bool,
    connected_elapsed_sec: float,
    reset_guard_sec: float,
    source_neutral_seen: bool,
    estop_latched: bool,
) -> bool:
    """Define readiness as write permission, not merely an open port."""
    return bool(
        connected
        and connected_elapsed_sec >= reset_guard_sec
        and source_neutral_seen
        and not estop_latched
    )


class FaultResetPolicy:
    """Rate-limit a neutral-only firmware fault reset sequence.

    The policy has no serial side effects. A caller may send an ``R`` frame
    only when ``observe`` returns ``reset``. After three failed resets it
    returns ``give_up`` once and remains blocked until a non-fault sample is
    observed.
    """

    def __init__(
        self,
        retry_interval_sec: float = 2.0,
        max_attempts: int = 3,
    ) -> None:
        if not math.isfinite(retry_interval_sec) or retry_interval_sec <= 0.0:
            raise ValueError('retry_interval_sec must be positive and finite')
        if isinstance(max_attempts, bool) or max_attempts <= 0:
            raise ValueError('max_attempts must be a positive integer')
        self.retry_interval_sec = float(retry_interval_sec)
        self.max_attempts = int(max_attempts)
        self.consecutive_attempts = 0
        self._last_attempt_at: Optional[float] = None
        self._gave_up = False

    def observe(self, fault: bool, steering: int, now: float) -> str:
        """Return ``none``, ``reset``, ``success``, or ``give_up``."""
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        if not fault:
            if self.consecutive_attempts:
                self.consecutive_attempts = 0
                self._gave_up = False
                return 'success'
            return 'none'
        if int(steering) != 0 or self._gave_up:
            return 'none'
        if (
            self._last_attempt_at is not None
            and now - self._last_attempt_at < self.retry_interval_sec
        ):
            return 'none'
        if self.consecutive_attempts >= self.max_attempts:
            self._gave_up = True
            return 'give_up'
        self.consecutive_attempts += 1
        self._last_attempt_at = now
        return 'reset'


@dataclass(frozen=True)
class FrameDecision:
    """One bridge-owned output decision."""

    kind: str
    throttle: int = 0
    steering: int = 0
    gear: int = -1
    stale: bool = False


class BridgeSafetyState:
    """Freshness and latched E-stop logic independent of ROS and serial I/O."""

    def __init__(
        self,
        command_timeout_sec: float = 0.30,
        stale_steer_hold_sec: float = 0.20,
        stale_steer_ramp_counts_per_tick: int = 120,
        estop_center_rate_counts_per_tick: int = 120,
        estop_center_timeout_sec: float = 1.0,
        competition_no_stop_enabled: bool = False,
        competition_minimum_throttle_counts: int = 1,
    ) -> None:
        timing_values = (
            command_timeout_sec,
            stale_steer_hold_sec,
            estop_center_timeout_sec,
        )
        if any(not math.isfinite(value) for value in timing_values):
            raise ValueError('safety timing values must be finite')
        if command_timeout_sec <= 0.0:
            raise ValueError('command_timeout_sec must be positive')
        if stale_steer_hold_sec < 0.0:
            raise ValueError('stale_steer_hold_sec cannot be negative')
        if stale_steer_ramp_counts_per_tick <= 0:
            raise ValueError('stale ramp must be positive')
        if estop_center_rate_counts_per_tick <= 0:
            raise ValueError('estop center rate must be positive')
        if estop_center_timeout_sec <= 0.0:
            raise ValueError('estop center timeout must be positive')
        if competition_minimum_throttle_counts <= 0:
            raise ValueError('competition minimum throttle must be positive')
        self.command_timeout_sec = float(command_timeout_sec)
        self.stale_steer_hold_sec = float(stale_steer_hold_sec)
        self.stale_ramp = int(stale_steer_ramp_counts_per_tick)
        self.estop_ramp = int(estop_center_rate_counts_per_tick)
        self.estop_timeout_sec = float(estop_center_timeout_sec)
        self.competition_no_stop_enabled = bool(
            competition_no_stop_enabled)
        self.competition_minimum_throttle_counts = int(
            competition_minimum_throttle_counts)
        self.continuous_drive_started = False
        self.throttle = 0
        self.steering = 0
        self.gear = -1
        self.last_command_at: Optional[float] = None
        self.command_stale = False
        self.estop_latched = False
        self._estop_started_at: Optional[float] = None
        self._estop_steering = 0
        self._estop_first_frame = False
        self._estop_centered_sent = False
        self._estop_x_sent = False

    def accept_command(
        self, throttle: int, steering: int, gear: int, now: float,
    ) -> bool:
        """Store a fresh command unless the E-stop latch is active."""
        if self.estop_latched:
            return False
        if (
            self.competition_no_stop_enabled
            and self.continuous_drive_started
            and int(throttle) <= 0
        ):
            # A fresh upstream soft-stop sample must not replace the last
            # valid forward command after competition departure.
            self.last_command_at = float(now)
            self.command_stale = False
            return True
        self.throttle = int(throttle)
        self.steering = int(steering)
        self.gear = int(gear)
        self.last_command_at = float(now)
        self.command_stale = False
        if self.competition_no_stop_enabled and self.throttle > 0:
            self.throttle = max(
                self.throttle,
                self.competition_minimum_throttle_counts,
            )
            self.continuous_drive_started = True
        return True

    def latch_estop(self, now: float) -> bool:
        """Latch once and initialize the defined center-then-X sequence."""
        if self.estop_latched:
            return False
        self.estop_latched = True
        self._estop_started_at = float(now)
        self._estop_steering = int(self.steering)
        self._estop_first_frame = True
        self._estop_centered_sent = False
        self._estop_x_sent = False
        return True

    def reset_estop(self) -> bool:
        """Clear the latch only through the dedicated reset path."""
        if not self.estop_latched:
            return False
        self.estop_latched = False
        self._estop_started_at = None
        self._estop_first_frame = False
        self._estop_centered_sent = False
        self._estop_x_sent = False
        self.throttle = 0
        self.steering = 0
        self.last_command_at = None
        self.command_stale = False
        self.continuous_drive_started = False
        return True

    def clear_continuity(self) -> None:
        """Require a new valid departure after a real stop or bridge fault."""

        self.continuous_drive_started = False

    def decision(self, now: float) -> Optional[FrameDecision]:
        """Return the frame the bridge may write on this timer tick."""
        now = float(now)
        if self.estop_latched:
            return self._estop_decision(now)
        if self.last_command_at is None:
            self.command_stale = False
            return None
        age = max(0.0, now - self.last_command_at)
        if age <= self.command_timeout_sec:
            self.command_stale = False
            return FrameDecision(
                'D', self.throttle, self.steering, self.gear, False)
        self.command_stale = True
        if (
            self.competition_no_stop_enabled
            and self.continuous_drive_started
        ):
            return FrameDecision(
                'D',
                max(self.throttle, self.competition_minimum_throttle_counts),
                self.steering,
                self.gear,
                True,
            )
        if age <= self.command_timeout_sec + self.stale_steer_hold_sec:
            return FrameDecision('D', 0, self.steering, self.gear, True)
        self.steering = ramp_to_zero(self.steering, self.stale_ramp)
        return FrameDecision('D', 0, self.steering, self.gear, True)

    def _estop_decision(self, now: float) -> Optional[FrameDecision]:
        if self._estop_x_sent:
            return None
        if self._estop_first_frame:
            self._estop_first_frame = False
            return FrameDecision('D', 0, self._estop_steering, self.gear)
        elapsed = now - float(self._estop_started_at)
        if not self._estop_centered_sent:
            self._estop_steering = ramp_to_zero(
                self._estop_steering, self.estop_ramp)
            if (
                self._estop_steering == 0
                or elapsed >= self.estop_timeout_sec
            ):
                self._estop_steering = 0
                self._estop_centered_sent = True
            return FrameDecision('D', 0, self._estop_steering, self.gear)
        self._estop_x_sent = True
        return FrameDecision('X')
