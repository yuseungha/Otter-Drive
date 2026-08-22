"""ROS-independent LiDAR cone-corridor end detection."""

from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ConeEndConfig:
    x_min: float = 0.3
    x_max: float = 0.7
    left_y_min: float = 0.15
    left_y_max: float = 1.0
    right_y_min: float = -1.0
    right_y_max: float = -0.15
    empty_scans: int = 5
    min_mode_duration_sec: float = 1.0

    def __post_init__(self) -> None:
        if not self.x_min < self.x_max:
            raise ValueError('cone end x limits are reversed')
        if not 0.0 < self.left_y_min < self.left_y_max:
            raise ValueError('left cone end y limits are invalid')
        if not self.right_y_min < self.right_y_max < 0.0:
            raise ValueError('right cone end y limits are invalid')
        if self.empty_scans < 1:
            raise ValueError('empty_scans must be at least one')
        if self.min_mode_duration_sec < 0.0:
            raise ValueError('min_mode_duration_sec must be nonnegative')


class ConeEndDetector:
    """Finish only after a seen corridor becomes empty on both sides."""

    def __init__(self, config: ConeEndConfig = ConeEndConfig()) -> None:
        self.config = config
        self.entered_at = 0.0
        self.observed_valid_cluster = False
        self.consecutive_empty = 0
        self.finished = False

    def enter(self, now: float) -> None:
        self.entered_at = float(now)
        self.observed_valid_cluster = False
        self.consecutive_empty = 0
        self.finished = False

    def update(
        self,
        clusters: Iterable[Sequence[float]],
        now: float,
    ) -> bool:
        points = [(float(point[0]), float(point[1])) for point in clusters]
        if points:
            self.observed_valid_cluster = True
        left_present = any(
            self.config.x_min <= x <= self.config.x_max
            and self.config.left_y_min <= y <= self.config.left_y_max
            for x, y in points
        )
        right_present = any(
            self.config.x_min <= x <= self.config.x_max
            and self.config.right_y_min <= y <= self.config.right_y_max
            for x, y in points
        )
        if not left_present and not right_present:
            self.consecutive_empty += 1
        else:
            # Losing only one side must never count as an empty corridor.
            self.consecutive_empty = 0

        qualifies = (
            self.observed_valid_cluster
            and float(now) - self.entered_at
            >= self.config.min_mode_duration_sec
            and self.consecutive_empty >= self.config.empty_scans
        )
        newly_finished = qualifies and not self.finished
        self.finished = self.finished or qualifies
        return newly_finished
