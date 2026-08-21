"""Pure LANE/OBSTACLE_AVOID/CONE mode-decision logic.

This module intentionally contains no ROS or vehicle-control code.  It only
turns already processed perception observations into a mission mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
import time
from typing import Callable, Iterable, Sequence

import numpy as np


class DecisionMode(str, Enum):
    """Modes selected from the currently visible course geometry."""

    LANE = 'LANE'
    OBSTACLE_AVOID = 'OBSTACLE_AVOID'
    CONE = 'CONE'


@dataclass(frozen=True)
class ConeModeConfig:
    """Geometry and hysteresis for entering and leaving the cone course."""

    enter_distance_m: float = 0.90
    minimum_forward_m: float = 0.05
    minimum_lateral_m: float = 0.08
    # The measured fixed course is 0.65--0.80 m wide.  These defaults retain
    # a small LiDAR margin without accepting the old 0.35--1.20 m range.
    minimum_track_width_m: float = 0.60
    maximum_track_width_m: float = 0.90
    maximum_pair_dx_m: float = 0.25
    enter_confirm_frames: int = 2
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
            isfinite(float(value)) and float(value) > 0.0
            for value in positive
        ):
            raise ValueError('cone mode distances and times must be positive')
        if self.minimum_forward_m < 0.0 or self.minimum_lateral_m < 0.0:
            raise ValueError('cone mode minima must be nonnegative')
        if self.minimum_track_width_m >= self.maximum_track_width_m:
            raise ValueError('cone track-width range is invalid')
        if self.enter_confirm_frames < 1:
            raise ValueError('enter_confirm_frames must be at least one')


@dataclass(frozen=True)
class ConeGate:
    """One plausible left/right cone pair in vehicle coordinates."""

    left_x_m: float
    left_y_m: float
    right_x_m: float
    right_y_m: float
    center_x_m: float
    center_y_m: float
    width_m: float


def nearest_cone_gate(
    cone_points: Iterable[Sequence[float]],
    config: ConeModeConfig,
) -> ConeGate | None:
    """Return the nearest plausible fixed-course gate, if one is visible."""

    config.validate()
    try:
        points = np.asarray(list(cone_points), dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if points.size == 0 or points.ndim != 2 or points.shape[1] != 2:
        return None
    points = points[np.all(np.isfinite(points), axis=1)]
    points = points[points[:, 0] >= config.minimum_forward_m]
    left = points[points[:, 1] >= config.minimum_lateral_m]
    right = points[points[:, 1] <= -config.minimum_lateral_m]

    candidates: list[ConeGate] = []
    for left_x, left_y in left:
        for right_x, right_y in right:
            width = float(left_y - right_y)
            if not (
                config.minimum_track_width_m
                <= width
                <= config.maximum_track_width_m
            ):
                continue
            if abs(float(left_x - right_x)) > config.maximum_pair_dx_m:
                continue
            candidates.append(ConeGate(
                left_x_m=float(left_x),
                left_y_m=float(left_y),
                right_x_m=float(right_x),
                right_y_m=float(right_y),
                center_x_m=float((left_x + right_x) * 0.5),
                center_y_m=float((left_y + right_y) * 0.5),
                width_m=width,
            ))
    return min(
        candidates,
        key=lambda gate: (gate.center_x_m, abs(gate.center_y_m)),
        default=None,
    )


class ModeDecisionMachine:
    """Select a mode without producing any steering or throttle command."""

    _NO_OBSERVATION = object()

    def __init__(
        self,
        config: ConeModeConfig = ConeModeConfig(),
        obstacle_clear_sec: float = 0.60,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        config.validate()
        if (
            not isfinite(float(obstacle_clear_sec))
            or float(obstacle_clear_sec) <= 0.0
        ):
            raise ValueError('obstacle_clear_sec must be positive')
        self.config = config
        self.obstacle_clear_sec = float(obstacle_clear_sec)
        self._clock = clock
        self.mode = DecisionMode.LANE
        self.reason = 'startup_lane'
        self.enter_count = 0
        self.last_gate: ConeGate | None = None
        self.last_cone_line_at: float | None = None
        self.last_obstacle_at: float | None = None
        self._last_cone_observation_id = self._NO_OBSERVATION

    def update(
        self,
        cone_points: Iterable[Sequence[float]],
        *,
        cone_path_valid: bool,
        obstacle_detected: bool = False,
        cone_observation_id=None,
        now: float | None = None,
    ) -> DecisionMode:
        """Update the state from one coherent perception snapshot.

        ``cone_observation_id`` prevents a high-rate timer from counting one
        LiDAR scan as multiple confirmation frames.  Callers that omit it get
        the simple one-call-per-observation behavior used by unit tests.
        """

        timestamp = self._clock() if now is None else float(now)
        gate = nearest_cone_gate(cone_points, self.config)
        self.last_gate = gate
        line_valid = gate is not None and bool(cone_path_valid)
        near_gate = bool(
            line_valid
            and gate is not None
            and gate.center_x_m <= self.config.enter_distance_m
        )

        observation_is_new = (
            cone_observation_id is None
            or cone_observation_id != self._last_cone_observation_id
        )
        if self.mode != DecisionMode.CONE:
            if not near_gate:
                self.enter_count = 0
            elif observation_is_new:
                self.enter_count += 1
                self._last_cone_observation_id = cone_observation_id
            if self.enter_count >= self.config.enter_confirm_frames:
                self.mode = DecisionMode.CONE
                self.last_cone_line_at = timestamp
                self.last_obstacle_at = None
                self.enter_count = 0
                self.reason = 'near_fixed_cone_gate'
                return self.mode

        # A valid cone gate has priority over an obstacle observation.
        if self.mode == DecisionMode.LANE:
            if obstacle_detected:
                self.mode = DecisionMode.OBSTACLE_AVOID
                self.last_obstacle_at = timestamp
                self.reason = 'obstacle_vehicle_on_lane_path'
            return self.mode

        if self.mode == DecisionMode.OBSTACLE_AVOID:
            if obstacle_detected:
                self.last_obstacle_at = timestamp
                self.reason = 'obstacle_vehicle_visible'
            elif self.last_obstacle_at is None:
                self.last_obstacle_at = timestamp
            elif timestamp - self.last_obstacle_at >= self.obstacle_clear_sec:
                self.mode = DecisionMode.LANE
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
            self.mode = DecisionMode.LANE
            self.last_cone_line_at = None
            self.reason = 'cone_lines_ended'
        return self.mode
