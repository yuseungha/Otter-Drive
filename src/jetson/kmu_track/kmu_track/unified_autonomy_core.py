"""Three-mode path selection and continuous Pure Pursuit control.

Both camera-lane and LiDAR-cone planners express paths in ``base_link`` with
``x`` forward and ``y`` left.  The selector coordinates normal lane driving,
opposite-lane obstacle avoidance and cone driving.  A missing path keeps a
forward command while preserving the last steering demand.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import atan, isfinite, radians
import time
from typing import Callable, Iterable, Sequence

import numpy as np


class PlannerMode(str, Enum):
    LANE = 'LANE'
    OBSTACLE_AVOID = 'OBSTACLE_AVOID'
    CONE = 'CONE'


def yolo_activity_for_mode(mode: str) -> bool | None:
    """Return the requested YOLO activity, or ``None`` for unknown states."""

    normalized = str(mode).strip().upper()
    if normalized in {
        'LANE', 'LANE_FOLLOW', 'LANE_REACQUIRE', 'OBSTACLE_AVOID',
    }:
        return True
    if normalized in {'CONE', 'CONE_INIT', 'CONE_SLALOM'}:
        return False
    return None


@dataclass(frozen=True)
class ConeSwitchConfig:
    """Geometry and hysteresis for detecting a left/right cone gate."""

    enter_distance_m: float = 0.90
    minimum_forward_m: float = 0.05
    minimum_lateral_m: float = 0.08
    minimum_track_width_m: float = 0.35
    maximum_track_width_m: float = 1.20
    maximum_pair_dx_m: float = 0.25
    minimum_cone_pairs: int = 2
    enter_confirm_frames: int = 1
    exit_missing_sec: float = 0.80

    def validate(self) -> None:
        positive = (
            self.enter_distance_m,
            self.minimum_track_width_m,
            self.maximum_track_width_m,
            self.maximum_pair_dx_m,
            self.exit_missing_sec,
        )
        if not all(
            isfinite(float(value)) and value > 0.0 for value in positive
        ):
            raise ValueError(
                'cone switch distances and times must be positive')
        if self.minimum_forward_m < 0.0 or self.minimum_lateral_m < 0.0:
            raise ValueError('cone switch minima must be nonnegative')
        if self.minimum_track_width_m >= self.maximum_track_width_m:
            raise ValueError('cone track-width range is invalid')
        if self.minimum_cone_pairs < 1:
            raise ValueError('minimum_cone_pairs must be at least one')
        if self.enter_confirm_frames < 1:
            raise ValueError('enter_confirm_frames must be at least one')


@dataclass(frozen=True)
class ConePair:
    left_x_m: float
    left_y_m: float
    right_x_m: float
    right_y_m: float
    center_x_m: float
    center_y_m: float
    width_m: float


def valid_cone_pairs(
    cone_points: Iterable[Sequence[float]],
    config: ConeSwitchConfig,
) -> tuple[ConePair, ...]:
    """Return a maximum set of plausible pairs without reusing a cone."""

    config.validate()
    try:
        points = np.asarray(list(cone_points), dtype=np.float64)
    except (TypeError, ValueError):
        return ()
    if points.size == 0:
        return ()
    if points.ndim != 2 or points.shape[1] != 2:
        return ()
    points = points[np.all(np.isfinite(points), axis=1)]
    points = points[points[:, 0] >= config.minimum_forward_m]
    left = points[points[:, 1] >= config.minimum_lateral_m]
    right = points[points[:, 1] <= -config.minimum_lateral_m]
    candidates: dict[int, list[tuple[int, ConePair]]] = {}
    for left_index, (left_x, left_y) in enumerate(left):
        for right_index, (right_x, right_y) in enumerate(right):
            width = float(left_y - right_y)
            if not (
                config.minimum_track_width_m
                <= width
                <= config.maximum_track_width_m
            ):
                continue
            if abs(float(left_x - right_x)) > config.maximum_pair_dx_m:
                continue
            pair = ConePair(
                left_x_m=float(left_x),
                left_y_m=float(left_y),
                right_x_m=float(right_x),
                right_y_m=float(right_y),
                center_x_m=float((left_x + right_x) * 0.5),
                center_y_m=float((left_y + right_y) * 0.5),
                width_m=width,
            )
            candidates.setdefault(left_index, []).append((right_index, pair))

    for edges in candidates.values():
        edges.sort(key=lambda edge: (
            abs(edge[1].left_x_m - edge[1].right_x_m),
            edge[1].center_x_m,
            abs(edge[1].center_y_m),
        ))

    matches: dict[int, tuple[int, ConePair]] = {}

    def assign(left_index: int, visited_right: set[int]) -> bool:
        for right_index, pair in candidates.get(left_index, ()):
            if right_index in visited_right:
                continue
            visited_right.add(right_index)
            current = matches.get(right_index)
            if current is None or assign(current[0], visited_right):
                matches[right_index] = (left_index, pair)
                return True
        return False

    for left_index in sorted(candidates):
        assign(left_index, set())

    return tuple(sorted(
        (match[1] for match in matches.values()),
        key=lambda pair: (pair.center_x_m, abs(pair.center_y_m)),
    ))


def nearest_cone_pair(
    cone_points: Iterable[Sequence[float]],
    config: ConeSwitchConfig,
) -> ConePair | None:
    """Find the nearest plausible non-overlapping left/right pair."""

    pairs = valid_cone_pairs(cone_points, config)
    return pairs[0] if pairs else None


class PlannerModeSelector:
    """Coordinate lane, obstacle-avoidance and cone planner selection."""

    def __init__(
        self,
        config: ConeSwitchConfig = ConeSwitchConfig(),
        obstacle_clear_sec: float = 0.60,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        config.validate()
        if (
            not isfinite(float(obstacle_clear_sec))
            or obstacle_clear_sec <= 0.0
        ):
            raise ValueError('obstacle_clear_sec must be finite and positive')
        self.config = config
        self.obstacle_clear_sec = float(obstacle_clear_sec)
        self._clock = clock
        self.mode = PlannerMode.LANE
        self.enter_count = 0
        self.last_cone_line_at: float | None = None
        self.last_pair: ConePair | None = None
        self.last_pair_count = 0
        self.last_obstacle_at: float | None = None
        self.reason = 'startup_lane'

    def update(
        self,
        cone_points: Iterable[Sequence[float]],
        *,
        cone_path_valid: bool,
        obstacle_detected: bool = False,
        now: float | None = None,
    ) -> PlannerMode:
        timestamp = self._clock() if now is None else float(now)
        pairs = valid_cone_pairs(cone_points, self.config)
        pair = pairs[0] if pairs else None
        self.last_pair = pair
        self.last_pair_count = len(pairs)
        line_valid = pair is not None and bool(cone_path_valid)

        near_gate = (
            line_valid
            and pair is not None
            and len(pairs) >= self.config.minimum_cone_pairs
            and pair.center_x_m <= self.config.enter_distance_m
        )

        if self.mode != PlannerMode.CONE:
            self.enter_count = self.enter_count + 1 if near_gate else 0
            if self.enter_count >= self.config.enter_confirm_frames:
                self.mode = PlannerMode.CONE
                self.last_cone_line_at = timestamp
                self.last_obstacle_at = None
                self.enter_count = 0
                self.reason = 'near_cone_pair'
                return self.mode

        if self.mode == PlannerMode.LANE:
            if obstacle_detected:
                self.mode = PlannerMode.OBSTACLE_AVOID
                self.last_obstacle_at = timestamp
                self.reason = 'obstacle_vehicle_on_path'
            return self.mode

        if self.mode == PlannerMode.OBSTACLE_AVOID:
            if obstacle_detected:
                self.last_obstacle_at = timestamp
                self.reason = 'obstacle_vehicle_visible'
            elif self.last_obstacle_at is None:
                self.last_obstacle_at = timestamp
            elif timestamp - self.last_obstacle_at >= self.obstacle_clear_sec:
                self.mode = PlannerMode.LANE
                self.last_obstacle_at = None
                self.reason = 'obstacle_vehicle_cleared'
            return self.mode

        if line_valid:
            self.last_cone_line_at = timestamp
            self.reason = 'cone_lines_visible'
        elif self.last_cone_line_at is None:
            self.last_cone_line_at = timestamp
        elif (
            timestamp - self.last_cone_line_at
            >= self.config.exit_missing_sec
        ):
            self.mode = PlannerMode.LANE
            self.last_cone_line_at = None
            self.reason = 'cone_lines_ended'
        return self.mode


@dataclass(frozen=True)
class PurePursuitConfig:
    wheelbase_m: float = 0.20
    lane_lookahead_m: float = 0.55
    cone_lookahead_m: float = 0.35
    maximum_steering_angle_rad: float = radians(35.0)
    maximum_steering_rate_rad_s: float = 3.0
    lane_cruise_speed_mps: float = 0.15
    cone_cruise_speed_mps: float = 0.12
    minimum_speed_mps: float = 0.08
    cone_minimum_speed_mps: float = 0.08
    curvature_slowdown_gain: float = 0.35
    minimum_target_forward_m: float = 0.03

    def validate(self) -> None:
        for name, value in vars(self).items():
            if not isfinite(float(value)):
                raise ValueError(f'{name} must be finite')
        for name in (
            'wheelbase_m', 'lane_lookahead_m', 'cone_lookahead_m',
            'maximum_steering_angle_rad', 'maximum_steering_rate_rad_s',
            'lane_cruise_speed_mps', 'cone_cruise_speed_mps',
            'minimum_speed_mps', 'cone_minimum_speed_mps',
            'minimum_target_forward_m',
        ):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.curvature_slowdown_gain < 0.0:
            raise ValueError('curvature_slowdown_gain must be nonnegative')
        if self.minimum_speed_mps > self.lane_cruise_speed_mps:
            raise ValueError('minimum speed exceeds lane cruise speed')
        if self.cone_minimum_speed_mps > self.cone_cruise_speed_mps:
            raise ValueError('cone minimum speed exceeds cone cruise speed')


@dataclass(frozen=True)
class PurePursuitResult:
    path_valid: bool
    reason: str
    speed_mps: float
    steering_angle_rad: float
    raw_steering_angle_rad: float
    curvature_1pm: float
    lookahead_m: float
    target_x_m: float
    target_y_m: float


def _prepare_path(path_points: Iterable[Sequence[float]]) -> np.ndarray | None:
    try:
        path = np.asarray(list(path_points), dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        return None
    if not np.all(np.isfinite(path)):
        return None
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    path = path[np.concatenate(([True], lengths > 1.0e-5))]
    return path if len(path) >= 2 else None


def _closest_arc(path: np.ndarray, arc: np.ndarray) -> float:
    best_distance = float('inf')
    best_arc = 0.0
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1.0e-12:
            continue
        ratio = float(np.clip(-np.dot(start, segment) / length_sq, 0.0, 1.0))
        projection = start + ratio * segment
        distance = float(np.dot(projection, projection))
        if distance < best_distance:
            best_distance = distance
            best_arc = float(arc[index] + ratio * np.sqrt(length_sq))
    return best_arc


def _interpolate(
    path: np.ndarray, arc: np.ndarray, target_arc: float
) -> np.ndarray:
    target_arc = float(np.clip(target_arc, arc[0], arc[-1]))
    upper = int(np.searchsorted(arc, target_arc, side='right'))
    if upper <= 0:
        return path[0].copy()
    if upper >= len(path):
        return path[-1].copy()
    lower = upper - 1
    span = float(arc[upper] - arc[lower])
    ratio = 0.0 if span <= 1.0e-12 else (target_arc - arc[lower]) / span
    return path[lower] + ratio * (path[upper] - path[lower])


class ContinuousPurePursuit:
    """Basic Pure Pursuit with a non-stopping last-steering fallback."""

    def __init__(
        self,
        config: PurePursuitConfig = PurePursuitConfig(),
    ) -> None:
        config.validate()
        self.config = config
        self.last_steering_angle_rad = 0.0

    def _cruise(self, mode: PlannerMode) -> float:
        return (
            self.config.cone_cruise_speed_mps
            if mode == PlannerMode.CONE
            else self.config.lane_cruise_speed_mps
        )

    def _minimum_speed(self, mode: PlannerMode) -> float:
        return (
            self.config.cone_minimum_speed_mps
            if mode == PlannerMode.CONE
            else self.config.minimum_speed_mps
        )

    def _fallback(self, mode: PlannerMode, reason: str) -> PurePursuitResult:
        return PurePursuitResult(
            path_valid=False,
            reason=f'FORWARD_FALLBACK_{reason}',
            speed_mps=self._cruise(mode),
            steering_angle_rad=self.last_steering_angle_rad,
            raw_steering_angle_rad=self.last_steering_angle_rad,
            curvature_1pm=0.0,
            lookahead_m=(
                self.config.cone_lookahead_m
                if mode == PlannerMode.CONE
                else self.config.lane_lookahead_m),
            target_x_m=0.0,
            target_y_m=0.0,
        )

    def command(
        self,
        path_points: Iterable[Sequence[float]],
        mode: PlannerMode | str,
        *,
        dt_s: float,
    ) -> PurePursuitResult:
        mode = PlannerMode(mode)
        if not isfinite(float(dt_s)) or dt_s <= 0.0:
            return self._fallback(mode, 'BAD_DT')
        path = _prepare_path(path_points)
        if path is None:
            return self._fallback(mode, 'NO_PATH')
        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        arc = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        if arc[-1] <= 1.0e-6:
            return self._fallback(mode, 'SHORT_PATH')
        lookahead = (
            self.config.cone_lookahead_m
            if mode == PlannerMode.CONE
            else self.config.lane_lookahead_m
        )
        start_arc = _closest_arc(path, arc)
        target = _interpolate(path, arc, min(start_arc + lookahead, arc[-1]))
        target_x, target_y = float(target[0]), float(target[1])
        distance_sq = target_x * target_x + target_y * target_y
        if (
            target_x < self.config.minimum_target_forward_m
            or distance_sq <= 1.0e-8
        ):
            return self._fallback(mode, 'NO_FORWARD_TARGET')
        curvature = 2.0 * target_y / distance_sq
        raw_steering = atan(self.config.wheelbase_m * curvature)
        steering_target = float(np.clip(
            raw_steering,
            -self.config.maximum_steering_angle_rad,
            self.config.maximum_steering_angle_rad,
        ))
        maximum_step = self.config.maximum_steering_rate_rad_s * dt_s
        steering = float(np.clip(
            steering_target,
            self.last_steering_angle_rad - maximum_step,
            self.last_steering_angle_rad + maximum_step,
        ))
        self.last_steering_angle_rad = steering
        cruise = self._cruise(mode)
        speed = max(
            self._minimum_speed(mode),
            cruise / (
                1.0 + self.config.curvature_slowdown_gain * abs(curvature)
            ),
        )
        return PurePursuitResult(
            path_valid=True,
            reason='PURE_PURSUIT',
            speed_mps=float(min(cruise, speed)),
            steering_angle_rad=steering,
            raw_steering_angle_rad=float(raw_steering),
            curvature_1pm=float(curvature),
            lookahead_m=float(lookahead),
            target_x_m=target_x,
            target_y_m=target_y,
        )


@dataclass(frozen=True)
class DriveCountConfig:
    lane_throttle_counts: int = 550
    cone_throttle_counts: int = 500
    minimum_throttle_counts: int = 280
    maximum_steering_counts: int = 650
    steering_gain: float = 1.0
    steering_gain_right: float = 0.0
    steering_sign: int = -1

    def validate(self) -> None:
        if self.lane_throttle_counts <= 0 or self.cone_throttle_counts <= 0:
            raise ValueError('cruise throttle counts must be positive')
        if self.minimum_throttle_counts <= 0:
            raise ValueError('minimum_throttle_counts must be positive')
        if self.minimum_throttle_counts > min(
            self.lane_throttle_counts, self.cone_throttle_counts
        ):
            raise ValueError('minimum throttle exceeds a cruise throttle')
        if self.maximum_steering_counts <= 0:
            raise ValueError('maximum_steering_counts must be positive')
        if not isfinite(self.steering_gain) or self.steering_gain <= 0.0:
            raise ValueError('steering_gain must be positive and finite')
        if (
            not isfinite(self.steering_gain_right)
            or self.steering_gain_right < 0.0
        ):
            raise ValueError('steering_gain_right must be finite and nonnegative')
        if self.steering_sign not in (-1, 1):
            raise ValueError('steering_sign must be -1 or 1')


def command_to_counts(
    result: PurePursuitResult,
    mode: PlannerMode | str,
    pursuit_config: PurePursuitConfig,
    count_config: DriveCountConfig,
) -> tuple[int, int]:
    """Map the metric Pure Pursuit command to ``[throttle, steering]``."""

    mode = PlannerMode(mode)
    pursuit_config.validate()
    count_config.validate()
    cruise_speed = (
        pursuit_config.cone_cruise_speed_mps
        if mode == PlannerMode.CONE
        else pursuit_config.lane_cruise_speed_mps
    )
    cruise_throttle = (
        count_config.cone_throttle_counts
        if mode == PlannerMode.CONE
        else count_config.lane_throttle_counts
    )
    speed_ratio = float(np.clip(result.speed_mps / cruise_speed, 0.0, 1.0))
    throttle = max(
        count_config.minimum_throttle_counts,
        int(round(cruise_throttle * speed_ratio)),
    )
    right_demand = (
        result.steering_angle_rad * count_config.steering_sign < 0.0
    )
    steering_gain = (
        count_config.steering_gain_right
        if right_demand and count_config.steering_gain_right > 0.0
        else count_config.steering_gain
    )
    steering_ratio = float(np.clip(
        result.steering_angle_rad
        / pursuit_config.maximum_steering_angle_rad
        * steering_gain,
        -1.0,
        1.0,
    ))
    steering = int(round(
        steering_ratio
        * count_config.maximum_steering_counts
        * count_config.steering_sign
    ))
    return throttle, steering
