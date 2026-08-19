"""Pure coordinate helpers shared by the OpenCV viewer and its tests."""

from dataclasses import dataclass
from math import cos, isfinite, sin
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class BevGeometry:
    width_px: int = 900
    height_px: int = 900
    range_forward_m: float = 2.5
    range_lateral_m: float = 1.5

    def validate(self) -> None:
        if int(self.width_px) != self.width_px or self.width_px < 100:
            raise ValueError("width_px must be an integer >= 100")
        if int(self.height_px) != self.height_px or self.height_px < 100:
            raise ValueError("height_px must be an integer >= 100")
        if not isfinite(self.range_forward_m) or self.range_forward_m <= 0.0:
            raise ValueError("range_forward_m must be finite and > 0")
        if not isfinite(self.range_lateral_m) or self.range_lateral_m <= 0.0:
            raise ValueError("range_lateral_m must be finite and > 0")


def metric_to_pixel(x_m: float, y_m: float, geometry: BevGeometry) -> tuple[int, int]:
    """Map vehicle x-forward/y-left coordinates to a top-forward image."""

    geometry.validate()
    if not isfinite(float(x_m)) or not isfinite(float(y_m)):
        raise ValueError("metric coordinates must be finite")
    u = (0.5 - float(y_m) / (2.0 * geometry.range_lateral_m)) * (
        geometry.width_px - 1
    )
    v = (1.0 - float(x_m) / geometry.range_forward_m) * (
        geometry.height_px - 1
    )
    return int(round(u)), int(round(v))


def transform_scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    sensor_to_planning: Sequence[float],
) -> np.ndarray:
    """Convert finite LaserScan samples to planning-frame Cartesian points."""

    scalars = (angle_min, angle_increment, range_min, range_max)
    if not all(isfinite(float(value)) for value in scalars):
        raise ValueError("scan metadata must be finite")
    if abs(float(angle_increment)) < 1.0e-12 or range_min < 0.0 or range_min >= range_max:
        raise ValueError("invalid scan geometry")
    if len(sensor_to_planning) != 3:
        raise ValueError("sensor_to_planning must contain x, y and yaw")
    tx, ty, yaw = (float(value) for value in sensor_to_planning)
    if not all(isfinite(value) for value in (tx, ty, yaw)):
        raise ValueError("transform must be finite")

    values = np.asarray(ranges, dtype=float)
    angles = float(angle_min) + np.arange(len(values), dtype=float) * float(
        angle_increment
    )
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    if not np.any(valid):
        return np.empty((0, 2), dtype=float)
    distances = values[valid]
    angles = angles[valid]
    sensor_x = distances * np.cos(angles)
    sensor_y = distances * np.sin(angles)
    c = cos(yaw)
    s = sin(yaw)
    return np.column_stack(
        (tx + c * sensor_x - s * sensor_y, ty + s * sensor_x + c * sensor_y)
    )
