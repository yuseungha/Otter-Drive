"""Thread-safe, ROS-independent state machine for manual-drive commands."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional


THROTTLE_MIN = -1000
THROTTLE_MAX = 1050
STEERING_MIN = -1000
STEERING_MAX = 1000


@dataclass(frozen=True)
class ControlSnapshot:
    armed: bool
    throttle: int
    steering: int
    active_client: Optional[str]
    command_age_ms: Optional[int]
    stop_reason: str


class ControlState:
    """Own the single-client lease and fail neutral on a missed heartbeat."""

    def __init__(
        self,
        timeout_sec: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        self._timeout_sec = timeout_sec
        self._clock = clock
        self._lock = threading.Lock()
        self._armed = False
        self._throttle = 0
        self._steering = 0
        self._active_client: Optional[str] = None
        self._last_command_at: Optional[float] = None
        self._stop_reason = "startup"

    @staticmethod
    def _limit(value: object, minimum: int, maximum: int) -> int:
        if isinstance(value, bool):
            raise ValueError("boolean is not a drive command")
        numeric = int(value)
        return max(minimum, min(maximum, numeric))

    def arm(self, client: str, allowed: bool) -> tuple[bool, str]:
        now = self._clock()
        with self._lock:
            if not allowed:
                self._force_stop_locked("serial_not_ready")
                return False, "serial_not_ready"
            if self._active_client not in (None, client):
                return False, "another_operator_is_active"
            self._active_client = client
            self._armed = True
            self._throttle = 0
            self._steering = 0
            self._last_command_at = now
            self._stop_reason = ""
            return True, "armed"

    def update(self, client: str, throttle: object, steering: object) -> tuple[bool, str]:
        now = self._clock()
        with self._lock:
            if not self._armed:
                return False, "not_armed"
            if self._active_client != client:
                return False, "not_active_operator"
            try:
                throttle_value = self._limit(
                    throttle, THROTTLE_MIN, THROTTLE_MAX)
                steering_value = self._limit(
                    steering, STEERING_MIN, STEERING_MAX)
            except (TypeError, ValueError, OverflowError):
                self._force_stop_locked("invalid_command")
                return False, "invalid_command"
            self._throttle = throttle_value
            self._steering = steering_value
            self._last_command_at = now
            return True, "accepted"

    def stop(self, reason: str = "operator_stop") -> None:
        with self._lock:
            self._force_stop_locked(reason)

    def command_for_publish(self) -> tuple[int, int]:
        now = self._clock()
        with self._lock:
            if (
                self._armed
                and self._last_command_at is not None
                and now - self._last_command_at > self._timeout_sec
            ):
                self._force_stop_locked("browser_heartbeat_timeout")
            return self._throttle, self._steering

    def snapshot(self) -> ControlSnapshot:
        now = self._clock()
        with self._lock:
            age = None
            if self._last_command_at is not None:
                age = max(0, int((now - self._last_command_at) * 1000))
            return ControlSnapshot(
                armed=self._armed,
                throttle=self._throttle,
                steering=self._steering,
                active_client=self._active_client,
                command_age_ms=age,
                stop_reason=self._stop_reason,
            )

    def _force_stop_locked(self, reason: str) -> None:
        self._armed = False
        self._throttle = 0
        self._steering = 0
        self._active_client = None
        self._last_command_at = None
        self._stop_reason = reason
