"""Project an image-space YOLO lane fit into a vehicle-frame path.

The segmentation planner works in normalized image coordinates.  Pure
Pursuit, however, needs metric points where ``x`` is forward and ``y`` is
left.  This module supplies the small four-point perspective calibration that
connects those two representations and deliberately has no ROS dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class LanePathProjectionConfig:
    """Normalized image trapezoid and matching ground-plane rectangle."""

    far_left_x_ratio: float = 0.38
    far_right_x_ratio: float = 0.62
    far_y_ratio: float = 0.50
    near_left_x_ratio: float = 0.10
    near_right_x_ratio: float = 0.90
    near_y_ratio: float = 0.98
    near_forward_m: float = 0.10
    far_forward_m: float = 2.00
    near_half_width_m: float = 0.45
    far_half_width_m: float = 0.45
    minimum_forward_m: float = 0.02

    def validate(self) -> None:
        values = tuple(float(value) for value in vars(self).values())
        if not all(isfinite(value) for value in values):
            raise ValueError('projection values must be finite')
        for name in (
            'far_left_x_ratio', 'far_right_x_ratio', 'far_y_ratio',
            'near_left_x_ratio', 'near_right_x_ratio', 'near_y_ratio',
        ):
            if not 0.0 <= float(getattr(self, name)) <= 1.0:
                raise ValueError(f'{name} must lie in [0, 1]')
        if self.far_left_x_ratio >= self.far_right_x_ratio:
            raise ValueError('far image points must be ordered left to right')
        if self.near_left_x_ratio >= self.near_right_x_ratio:
            raise ValueError('near image points must be ordered left to right')
        if self.far_y_ratio >= self.near_y_ratio:
            raise ValueError('far_y_ratio must be above near_y_ratio')
        if self.near_forward_m < 0.0:
            raise ValueError('near_forward_m must be nonnegative')
        if self.far_forward_m <= self.near_forward_m:
            raise ValueError('far_forward_m must exceed near_forward_m')
        if self.near_half_width_m <= 0.0 or self.far_half_width_m <= 0.0:
            raise ValueError('ground half widths must be positive')
        if self.minimum_forward_m < 0.0:
            raise ValueError('minimum_forward_m must be nonnegative')


def _homography(source: np.ndarray, destination: np.ndarray) -> np.ndarray:
    """Return the projective transform mapping four 2D source points."""

    if source.shape != (4, 2) or destination.shape != (4, 2):
        raise ValueError('homography requires exactly four point pairs')
    matrix = []
    vector = []
    for (u, v), (x, y) in zip(source, destination):
        matrix.append((u, v, 1.0, 0.0, 0.0, 0.0, -u * x, -v * x))
        matrix.append((0.0, 0.0, 0.0, u, v, 1.0, -u * y, -v * y))
        vector.extend((x, y))
    try:
        coefficients = np.linalg.solve(
            np.asarray(matrix, dtype=np.float64),
            np.asarray(vector, dtype=np.float64),
        )
    except np.linalg.LinAlgError as error:
        raise ValueError('projection quadrilateral is degenerate') from error
    return np.append(coefficients, 1.0).reshape(3, 3)


class LanePathProjector:
    """Convert ``SegmentationLanePlanner.fit_path`` to ``base_link`` points."""

    def __init__(
        self,
        config: LanePathProjectionConfig = LanePathProjectionConfig(),
    ) -> None:
        config.validate()
        self.config = config
        source = np.asarray((
            (config.far_left_x_ratio, config.far_y_ratio),
            (config.far_right_x_ratio, config.far_y_ratio),
            (config.near_left_x_ratio, config.near_y_ratio),
            (config.near_right_x_ratio, config.near_y_ratio),
        ), dtype=np.float64)
        destination = np.asarray((
            (config.far_forward_m, config.far_half_width_m),
            (config.far_forward_m, -config.far_half_width_m),
            (config.near_forward_m, config.near_half_width_m),
            (config.near_forward_m, -config.near_half_width_m),
        ), dtype=np.float64)
        self._transform = _homography(source, destination)

    def project_normalized(
        self, points: Sequence[Sequence[float]]
    ) -> np.ndarray:
        """Project normalized image ``(u, v)`` points to metric ``(x, y)``."""

        values = np.asarray(points, dtype=np.float64)
        if values.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        if values.ndim != 2 or values.shape[1] != 2:
            raise ValueError('points must have shape (N, 2)')
        if not np.all(np.isfinite(values)):
            raise ValueError('points must be finite')
        homogeneous = np.column_stack((values, np.ones(len(values))))
        projected = homogeneous @ self._transform.T
        scale = projected[:, 2]
        if np.any(np.abs(scale) <= 1.0e-9):
            raise ValueError('point projects to infinity')
        return projected[:, :2] / scale[:, None]

    def project_geometry(self, geometry: Mapping[str, object]) -> np.ndarray:
        """Return a forward-ordered path or empty output for an invalid fit."""

        if not bool(geometry.get('valid', False)):
            return np.empty((0, 2), dtype=np.float64)
        width = int(geometry.get('image_w', 0))
        height = int(geometry.get('image_h', 0))
        fit_path = geometry.get('fit_path', [])
        if width < 2 or height < 2 or not isinstance(fit_path, Sequence):
            return np.empty((0, 2), dtype=np.float64)
        normalized = []
        for point in fit_path:
            if not isinstance(point, Mapping):
                continue
            try:
                normalized.append((
                    float(point['x']) / (width - 1.0),
                    float(point['y']) / (height - 1.0),
                ))
            except (KeyError, TypeError, ValueError):
                continue
        if len(normalized) < 2:
            return np.empty((0, 2), dtype=np.float64)
        projected = self.project_normalized(normalized)
        projected = projected[np.all(np.isfinite(projected), axis=1)]
        projected = projected[
            projected[:, 0] >= self.config.minimum_forward_m]
        if len(projected) < 2:
            return np.empty((0, 2), dtype=np.float64)
        projected = projected[np.argsort(projected[:, 0])]
        segment_lengths = np.linalg.norm(np.diff(projected, axis=0), axis=1)
        keep = np.concatenate(([True], segment_lengths > 1.0e-4))
        projected = projected[keep]
        if len(projected) < 2:
            return np.empty((0, 2), dtype=np.float64)
        # Pure Pursuit then measures its lookahead from the rear-axle origin.
        return np.vstack((np.asarray(((0.0, 0.0),)), projected))
