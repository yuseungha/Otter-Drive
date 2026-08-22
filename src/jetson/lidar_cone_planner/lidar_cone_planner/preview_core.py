"""ROS-independent lookahead preview for a stationary validation vehicle."""

from dataclasses import dataclass
from math import atan, atan2, isfinite
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class PreviewConfig:
    """Geometry used only to visualize a future Pure Pursuit command."""

    wheelbase_m: float = 0.20
    lookahead_min_m: float = 0.25
    lookahead_max_m: float = 0.45
    lookahead_time_s: float = 0.75
    validation_speed_mps: float = 0.0

    def validate(self) -> None:
        values = (
            self.wheelbase_m,
            self.lookahead_min_m,
            self.lookahead_max_m,
            self.lookahead_time_s,
            self.validation_speed_mps,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("preview parameters must be finite")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be > 0")
        if self.lookahead_min_m <= 0.0:
            raise ValueError("lookahead_min_m must be > 0")
        if self.lookahead_max_m < self.lookahead_min_m:
            raise ValueError("lookahead_max_m cannot be below lookahead_min_m")
        if self.lookahead_time_s < 0.0:
            raise ValueError("lookahead_time_s must be >= 0")
        if self.validation_speed_mps < 0.0:
            raise ValueError("validation_speed_mps must be >= 0")


@dataclass(frozen=True)
class PreviewResult:
    """Finite direction values or an explicit invalid reason."""

    valid: bool
    reason: str
    lookahead_m: float = 0.0
    target_x_m: float = 0.0
    target_y_m: float = 0.0
    heading_rad: float = 0.0
    curvature_1pm: float = 0.0
    steering_angle_rad: float = 0.0


def invalid_preview(reason: str) -> PreviewResult:
    return PreviewResult(False, reason)


def _project_origin_to_arc(path: np.ndarray, arc: np.ndarray) -> float:
    best_distance_sq = float("inf")
    best_arc = 0.0
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1.0e-14:
            continue
        ratio = float(np.clip(-np.dot(start, segment) / length_sq, 0.0, 1.0))
        point = start + ratio * segment
        distance_sq = float(np.dot(point, point))
        candidate_arc = float(arc[index] + ratio * np.sqrt(length_sq))
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_arc = candidate_arc
    return best_arc


def _interpolate(path: np.ndarray, arc: np.ndarray, target_arc: float) -> np.ndarray:
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


def compute_path_preview(
    path_points: Iterable[Sequence[float]], config: PreviewConfig
) -> PreviewResult:
    """Select an arc-length lookahead target and calculate steering geometry."""

    config.validate()
    try:
        path = np.asarray(list(path_points), dtype=float)
    except (TypeError, ValueError):
        return invalid_preview("BAD_PATH")
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        return invalid_preview("BAD_PATH")
    if not np.all(np.isfinite(path)):
        return invalid_preview("NONFINITE_PATH")

    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    keep = np.concatenate(([True], segment_lengths > 1.0e-7))
    path = path[keep]
    if len(path) < 2:
        return invalid_preview("BAD_PATH")
    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if not isfinite(float(arc[-1])) or arc[-1] <= 1.0e-7:
        return invalid_preview("BAD_PATH_LENGTH")

    lookahead = float(
        np.clip(
            config.lookahead_min_m
            + config.lookahead_time_s * config.validation_speed_mps,
            config.lookahead_min_m,
            config.lookahead_max_m,
        )
    )
    start_arc = _project_origin_to_arc(path, arc)
    target = _interpolate(path, arc, min(start_arc + lookahead, float(arc[-1])))
    target_x = float(target[0])
    target_y = float(target[1])
    distance_sq = target_x * target_x + target_y * target_y
    if target_x <= 0.0 or distance_sq <= 1.0e-10:
        return invalid_preview("NO_FORWARD_TARGET")

    heading = atan2(target_y, target_x)
    curvature = 2.0 * target_y / distance_sq
    steering = atan(config.wheelbase_m * curvature)
    output = (lookahead, target_x, target_y, heading, curvature, steering)
    if not all(isfinite(value) for value in output):
        return invalid_preview("NONFINITE_PREVIEW")
    return PreviewResult(
        True,
        "OK",
        lookahead_m=lookahead,
        target_x_m=target_x,
        target_y_m=target_y,
        heading_rad=heading,
        curvature_1pm=curvature,
        steering_angle_rad=steering,
    )
