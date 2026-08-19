"""ROS-independent, fail-closed Pure Pursuit calculations.

The path is expressed in the vehicle control frame: x points forward and y
points left.  This module never retains a previous path and never produces a
non-zero command from invalid input; message freshness and planner-status
pairing are enforced by :mod:`cone_pure_pursuit`.
"""

from dataclasses import dataclass
from math import atan, isfinite, pi, radians, sqrt
from typing import Iterable, Sequence

import numpy as np


@dataclass
class ControllerConfig:
    """Vehicle geometry, lookahead and command limits."""

    # These are placeholders that must be replaced by measured values before
    # the ROS wrapper can be armed (geometry_confirmed defaults to false).
    wheelbase_m: float = 0.20
    max_steering_angle_rad: float = radians(35.0)

    lookahead_min_m: float = 0.25
    lookahead_max_m: float = 0.55
    lookahead_time_s: float = 0.75
    min_target_forward_m: float = 0.08
    min_remaining_path_m: float = 0.25

    max_speed_mps: float = 0.15
    max_lateral_accel_mps2: float = 0.15
    max_accel_mps2: float = 0.20
    max_decel_mps2: float = 0.50
    stopping_buffer_m: float = 0.12
    max_steering_rate_rad_s: float = 1.50

    min_plan_confidence: float = 0.40
    confidence_speed_floor: float = 0.35
    virtual_speed_factor: float = 0.60
    reject_steering_saturation: bool = True

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        positive = {
            "wheelbase_m": self.wheelbase_m,
            "max_steering_angle_rad": self.max_steering_angle_rad,
            "lookahead_min_m": self.lookahead_min_m,
            "lookahead_max_m": self.lookahead_max_m,
            "min_target_forward_m": self.min_target_forward_m,
            "min_remaining_path_m": self.min_remaining_path_m,
            "max_speed_mps": self.max_speed_mps,
            "max_lateral_accel_mps2": self.max_lateral_accel_mps2,
            "max_accel_mps2": self.max_accel_mps2,
            "max_decel_mps2": self.max_decel_mps2,
            "max_steering_rate_rad_s": self.max_steering_rate_rad_s,
        }
        for name, value in positive.items():
            if not isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

        nonnegative = {
            "lookahead_time_s": self.lookahead_time_s,
            "stopping_buffer_m": self.stopping_buffer_m,
        }
        for name, value in nonnegative.items():
            if not isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")

        if self.lookahead_min_m > self.lookahead_max_m:
            raise ValueError("lookahead_min_m cannot exceed lookahead_max_m")
        if self.max_steering_angle_rad >= 0.5 * pi:
            raise ValueError("max_steering_angle_rad must be smaller than pi/2")
        if not 0.0 <= self.min_plan_confidence <= 1.0:
            raise ValueError("min_plan_confidence must be in [0, 1]")
        if not 0.0 <= self.confidence_speed_floor <= 1.0:
            raise ValueError("confidence_speed_floor must be in [0, 1]")
        if not 0.0 < self.virtual_speed_factor <= 1.0:
            raise ValueError("virtual_speed_factor must be in (0, 1]")


@dataclass(frozen=True)
class ControlResult:
    """One finite command or an explicit zero-command rejection reason."""

    valid: bool
    reason: str
    speed_mps: float = 0.0
    steering_angle_rad: float = 0.0
    raw_steering_angle_rad: float = 0.0
    curvature_1pm: float = 0.0
    lookahead_m: float = 0.0
    target_x_m: float = 0.0
    target_y_m: float = 0.0
    remaining_path_m: float = 0.0


def stop_result(reason: str) -> ControlResult:
    """Return the canonical immediate-stop command."""

    return ControlResult(False, reason)


def _as_path(points: Iterable[Sequence[float]]) -> np.ndarray | None:
    try:
        path = np.asarray(list(points), dtype=float)
    except (TypeError, ValueError):
        return None
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        return None
    if not np.all(np.isfinite(path)):
        return None

    # Consecutive duplicates make arc-length interpolation ambiguous.
    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    keep = np.concatenate(([True], segment_lengths > 1.0e-7))
    path = path[keep]
    return path if len(path) >= 2 else None


def _closest_arc_position(path: np.ndarray, arc: np.ndarray) -> float:
    """Project the vehicle origin onto the path and return its arc position."""

    best_distance_sq = float("inf")
    best_arc = 0.0
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1.0e-14:
            continue
        ratio = float(np.clip(-np.dot(start, segment) / length_sq, 0.0, 1.0))
        projected = start + ratio * segment
        distance_sq = float(np.dot(projected, projected))
        candidate_arc = float(
            arc[index] + ratio * sqrt(length_sq)
        )
        if (
            distance_sq < best_distance_sq - 1.0e-12
            or (
                abs(distance_sq - best_distance_sq) <= 1.0e-12
                and candidate_arc < best_arc
            )
        ):
            best_distance_sq = distance_sq
            best_arc = candidate_arc
    return best_arc


def _interpolate_at_arc(path: np.ndarray, arc: np.ndarray, target_arc: float) -> np.ndarray:
    target_arc = float(np.clip(target_arc, arc[0], arc[-1]))
    upper = int(np.searchsorted(arc, target_arc, side="right"))
    if upper <= 0:
        return path[0].copy()
    if upper >= len(path):
        return path[-1].copy()
    lower = upper - 1
    span = float(arc[upper] - arc[lower])
    ratio = 0.0 if span <= 1.0e-12 else (target_arc - arc[lower]) / span
    return path[lower] + ratio * (path[upper] - path[lower])


def _confidence_speed_factor(confidence: float, config: ControllerConfig) -> float:
    if config.min_plan_confidence >= 1.0 - 1.0e-12:
        return 1.0
    normalized = (confidence - config.min_plan_confidence) / (
        1.0 - config.min_plan_confidence
    )
    normalized = float(np.clip(normalized, 0.0, 1.0))
    return config.confidence_speed_floor + (
        1.0 - config.confidence_speed_floor
    ) * normalized


def compute_pure_pursuit(
    path_points: Iterable[Sequence[float]],
    *,
    current_speed_mps: float,
    plan_confidence: float,
    previous_speed_mps: float,
    previous_steering_angle_rad: float,
    dt_s: float,
    config: ControllerConfig,
    virtual_path: bool = False,
) -> ControlResult:
    """Calculate a speed-adaptive Pure Pursuit command.

    Normal valid-to-valid commands obey speed and steering slew limits.  The
    caller must publish :func:`stop_result` directly for invalid input so an
    emergency stop is never delayed by these normal-operation limits.
    """

    config.validate()
    path = _as_path(path_points)
    if path is None:
        return stop_result("BAD_PATH")

    scalars = (
        current_speed_mps,
        plan_confidence,
        previous_speed_mps,
        previous_steering_angle_rad,
        dt_s,
    )
    if not all(isfinite(float(value)) for value in scalars):
        return stop_result("NONFINITE_CONTROL_INPUT")
    if dt_s <= 0.0:
        return stop_result("BAD_CONTROL_DT")
    if not 0.0 <= plan_confidence <= 1.0:
        return stop_result("BAD_CONFIDENCE")
    if plan_confidence < config.min_plan_confidence:
        return stop_result("LOW_CONFIDENCE")

    current_speed = abs(float(current_speed_mps))
    previous_speed = max(0.0, float(previous_speed_mps))
    previous_steering = float(previous_steering_angle_rad)
    if previous_speed > config.max_speed_mps + 1.0e-9:
        return stop_result("PREVIOUS_SPEED_OUT_OF_RANGE")
    if abs(previous_steering) > config.max_steering_angle_rad + 1.0e-9:
        return stop_result("PREVIOUS_STEERING_OUT_OF_RANGE")
    lookahead = float(
        np.clip(
            config.lookahead_min_m + config.lookahead_time_s * current_speed,
            config.lookahead_min_m,
            config.lookahead_max_m,
        )
    )

    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if not isfinite(float(arc[-1])) or arc[-1] <= 1.0e-7:
        return stop_result("BAD_PATH_LENGTH")
    start_arc = _closest_arc_position(path, arc)
    remaining = float(arc[-1] - start_arc)
    if remaining < config.min_remaining_path_m:
        return stop_result("PATH_TOO_SHORT_CONTROL")

    target = _interpolate_at_arc(path, arc, min(start_arc + lookahead, arc[-1]))
    target_x = float(target[0])
    target_y = float(target[1])
    target_distance_sq = target_x * target_x + target_y * target_y
    if target_x < config.min_target_forward_m or target_distance_sq <= 1.0e-8:
        return stop_result("NO_FORWARD_TARGET")

    curvature = 2.0 * target_y / target_distance_sq
    raw_steering = atan(config.wheelbase_m * curvature)
    if (
        config.reject_steering_saturation
        and abs(raw_steering) > config.max_steering_angle_rad + 1.0e-9
    ):
        return stop_result("STEERING_LIMIT")
    steering_target = float(
        np.clip(
            raw_steering,
            -config.max_steering_angle_rad,
            config.max_steering_angle_rad,
        )
    )

    if abs(curvature) <= 1.0e-9:
        curvature_speed = config.max_speed_mps
    else:
        curvature_speed = sqrt(
            config.max_lateral_accel_mps2 / abs(curvature)
        )
    usable_stop_distance = max(0.0, remaining - config.stopping_buffer_m)
    stopping_speed = sqrt(2.0 * config.max_decel_mps2 * usable_stop_distance)
    confidence_speed = (
        config.max_speed_mps * _confidence_speed_factor(plan_confidence, config)
    )
    virtual_speed = (
        config.max_speed_mps * config.virtual_speed_factor
        if virtual_path
        else config.max_speed_mps
    )
    target_speed = min(
        config.max_speed_mps,
        curvature_speed,
        stopping_speed,
        confidence_speed,
        virtual_speed,
    )

    if target_speed >= previous_speed:
        speed = min(target_speed, previous_speed + config.max_accel_mps2 * dt_s)
    else:
        speed = max(target_speed, previous_speed - config.max_decel_mps2 * dt_s)
    speed = float(np.clip(speed, 0.0, config.max_speed_mps))
    steering_step = config.max_steering_rate_rad_s * dt_s
    steering = float(
        np.clip(
            steering_target,
            previous_steering - steering_step,
            previous_steering + steering_step,
        )
    )
    steering = float(
        np.clip(
            steering,
            -config.max_steering_angle_rad,
            config.max_steering_angle_rad,
        )
    )

    values = (speed, steering, raw_steering, curvature, lookahead, target_x, target_y)
    if not all(isfinite(float(value)) for value in values):
        return stop_result("NONFINITE_COMMAND")
    return ControlResult(
        valid=True,
        reason="OK_VIRTUAL" if virtual_path else "OK",
        speed_mps=max(0.0, speed),
        steering_angle_rad=steering,
        raw_steering_angle_rad=raw_steering,
        curvature_1pm=curvature,
        lookahead_m=lookahead,
        target_x_m=target_x,
        target_y_m=target_y,
        remaining_path_m=remaining,
    )
