"""Hardware-independent state and safety logic for sensor-mode driving."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import isfinite


class DriveMode(str, Enum):
    LANE_FOLLOW = "LANE_FOLLOW"
    CONE_INIT = "CONE_INIT"
    CONE_SLALOM = "CONE_SLALOM"
    LANE_REACQUIRE = "LANE_REACQUIRE"
    SAFE_STOP = "SAFE_STOP"


class DriveEvent(str, Enum):
    CONE_CONFIRMED = "cone_confirmed"
    CONE_CANCELLED = "cone_cancelled"
    VALID_CONE_PATH = "valid_cone_path"
    CONE_FINISHED = "cone_finished"
    LANE_STABLE = "lane_stable"
    SENSOR_TIMEOUT = "sensor_timeout"
    PATH_TIMEOUT = "path_timeout"
    TIMEOUT = "timeout"


class DriveModeMachine:
    """The only legal transition table for the integrated mission manager."""

    def __init__(self, initial_mode: DriveMode | str = DriveMode.LANE_FOLLOW) -> None:
        initial = DriveMode(initial_mode)
        if initial not in {DriveMode.LANE_FOLLOW, DriveMode.CONE_INIT}:
            raise ValueError(
                'initial_mode must be LANE_FOLLOW or fail-closed CONE_INIT')
        self.mode = initial

    def apply(self, event: DriveEvent | str) -> DriveMode:
        event = DriveEvent(event)
        if event in {DriveEvent.SENSOR_TIMEOUT, DriveEvent.PATH_TIMEOUT}:
            self.mode = DriveMode.SAFE_STOP
            return self.mode
        transitions = {
            (DriveMode.LANE_FOLLOW, DriveEvent.CONE_CONFIRMED):
                DriveMode.CONE_INIT,
            (DriveMode.CONE_INIT, DriveEvent.CONE_CANCELLED):
                DriveMode.LANE_REACQUIRE,
            (DriveMode.CONE_INIT, DriveEvent.VALID_CONE_PATH):
                DriveMode.CONE_SLALOM,
            (DriveMode.CONE_INIT, DriveEvent.TIMEOUT): DriveMode.SAFE_STOP,
            (DriveMode.CONE_SLALOM, DriveEvent.CONE_FINISHED):
                DriveMode.LANE_REACQUIRE,
            (DriveMode.LANE_REACQUIRE, DriveEvent.LANE_STABLE):
                DriveMode.LANE_FOLLOW,
            (DriveMode.LANE_REACQUIRE, DriveEvent.TIMEOUT):
                DriveMode.SAFE_STOP,
        }
        self.mode = transitions.get((self.mode, event), self.mode)
        return self.mode


class ConsecutiveThreshold:
    def __init__(self, required: int) -> None:
        if required < 1:
            raise ValueError("required must be >= 1")
        self.required = required
        self.count = 0

    def update(self, detected: bool) -> bool:
        self.count = self.count + 1 if detected else 0
        return self.count >= self.required

    def reset(self) -> None:
        self.count = 0


class RollingConfirmation:
    def __init__(self, window: int, required: int) -> None:
        if window < 1 or required < 1 or required > window:
            raise ValueError("require 1 <= required <= window")
        self.window = window
        self.required = required
        self.samples: deque[bool] = deque(maxlen=window)

    def update(self, valid: bool) -> bool:
        self.samples.append(bool(valid))
        return (
            len(self.samples) == self.window
            and sum(self.samples) >= self.required
        )

    def reset(self) -> None:
        self.samples.clear()


@dataclass(frozen=True)
class SubscriptionDecision:
    camera: bool
    lidar: bool


def desired_subscriptions(mode: DriveMode | str) -> SubscriptionDecision:
    mode = DriveMode(mode)
    return SubscriptionDecision(
        camera=mode in {DriveMode.LANE_FOLLOW, DriveMode.LANE_REACQUIRE},
        lidar=mode in {DriveMode.CONE_INIT, DriveMode.CONE_SLALOM},
    )


class SubscriptionInterlock:
    """Interlock destination activation until the source reports inactive."""

    def __init__(self) -> None:
        self.mode = DriveMode.LANE_FOLLOW
        self.camera_active = False
        self.lidar_active = False

    def set_mode(self, mode: DriveMode | str) -> None:
        self.mode = DriveMode(mode)

    def camera_allowed(self) -> bool:
        return (
            desired_subscriptions(self.mode).camera
            and not self.lidar_active
        )

    def lidar_allowed(self) -> bool:
        return (
            desired_subscriptions(self.mode).lidar
            and not self.camera_active
        )

    def safe(self) -> bool:
        return not (self.camera_active and self.lidar_active)


def stamp_age_sec(stamp_ns: int, now_ns: int) -> float:
    return (now_ns - stamp_ns) * 1.0e-9


def stamp_is_fresh(
    stamp_ns: int,
    now_ns: int,
    max_age_sec: float,
    max_future_sec: float = 0.05,
) -> bool:
    if stamp_ns <= 0 or now_ns <= 0 or max_age_sec <= 0.0:
        return False
    age = stamp_age_sec(stamp_ns, now_ns)
    return isfinite(age) and -max_future_sec <= age <= max_age_sec


def selected_command_source(
    mode: DriveMode | str, stop_requested: bool
) -> str:
    if stop_requested:
        return "neutral"
    mode = DriveMode(mode)
    if mode == DriveMode.LANE_FOLLOW:
        return "lane"
    if mode == DriveMode.CONE_SLALOM:
        return "cone"
    return "neutral"


def largest_bbox_area_ratio(
    boxes: list[tuple[int, int, int, int]],
    image_width: int,
    image_height: int,
) -> float:
    """Compute the largest ``(x, y, width, height)`` box/image area ratio."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    largest = max(
        (max(0, box[2]) * max(0, box[3]) for box in boxes),
        default=0,
    )
    return float(largest / (image_width * image_height))
