"""LiDAR obstacle-vehicle detection and opposite-lane path generation."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from math import floor, isfinite
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class ObstacleAvoidanceConfig:
    """Geometry for treating a non-cone cluster as a lane-blocking vehicle."""

    vehicle_half_width_m: float = 0.15
    detection_min_forward_m: float = 0.20
    detection_max_forward_m: float = 2.00
    cone_exclusion_radius_m: float = 0.20
    cluster_distance_m: float = 0.14
    minimum_cluster_points: int = 3
    minimum_cluster_span_m: float = 0.08
    maximum_cluster_span_m: float = 1.20
    confirm_frames: int = 1
    clear_sec: float = 0.60
    opposite_lane_offset_m: float = 0.55
    lane_change_distance_m: float = 0.80
    preferred_offset_sign: int = 1

    def validate(self) -> None:
        for name, value in vars(self).items():
            if name in {
                'minimum_cluster_points', 'confirm_frames',
                'preferred_offset_sign',
            }:
                continue
            if not isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.detection_min_forward_m >= self.detection_max_forward_m:
            raise ValueError('obstacle detection forward range is invalid')
        if self.minimum_cluster_span_m >= self.maximum_cluster_span_m:
            raise ValueError('obstacle cluster span range is invalid')
        if self.minimum_cluster_points < 2:
            raise ValueError('minimum_cluster_points must be at least two')
        if self.confirm_frames < 1:
            raise ValueError('confirm_frames must be at least one')
        if self.preferred_offset_sign not in (-1, 1):
            raise ValueError('preferred_offset_sign must be -1 or 1')


@dataclass(frozen=True)
class ObstacleVehicle:
    center_x_m: float
    center_y_m: float
    path_distance_m: float
    signed_path_offset_m: float
    point_count: int
    span_m: float
    avoidance_sign: int


def _points(values: Iterable[Sequence[float]]) -> np.ndarray:
    try:
        points = np.asarray(list(values), dtype=np.float64)
    except (TypeError, ValueError):
        return np.empty((0, 2), dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        return np.empty((0, 2), dtype=np.float64)
    return points[np.all(np.isfinite(points), axis=1)]


def exclude_cone_points(
    lidar_points: Iterable[Sequence[float]],
    cone_centers: Iterable[Sequence[float]],
    exclusion_radius_m: float,
) -> np.ndarray:
    """Remove scan endpoints belonging to cone-sized detections."""

    if not isfinite(float(exclusion_radius_m)) or exclusion_radius_m <= 0.0:
        raise ValueError('exclusion_radius_m must be finite and positive')
    points = _points(lidar_points)
    cones = _points(cone_centers)
    if len(points) == 0 or len(cones) == 0:
        return points
    delta = points[:, None, :] - cones[None, :, :]
    distance_sq = np.sum(delta * delta, axis=2)
    keep = np.all(distance_sq > exclusion_radius_m ** 2, axis=1)
    return points[keep]


def cluster_obstacle_points(
    values: Iterable[Sequence[float]],
    distance_m: float,
) -> list[np.ndarray]:
    """Cluster unordered 2D points with a small spatial-hash flood fill."""

    if not isfinite(float(distance_m)) or distance_m <= 0.0:
        raise ValueError('distance_m must be finite and positive')
    points = _points(values)
    if len(points) == 0:
        return []
    cells: dict[tuple[int, int], list[int]] = defaultdict(list)
    for index, point in enumerate(points):
        key = (
            floor(float(point[0]) / distance_m),
            floor(float(point[1]) / distance_m),
        )
        cells[key].append(index)

    visited = np.zeros(len(points), dtype=bool)
    distance_sq = distance_m * distance_m
    clusters = []
    for seed in range(len(points)):
        if visited[seed]:
            continue
        visited[seed] = True
        members = []
        pending = deque((seed,))
        while pending:
            index = pending.popleft()
            members.append(index)
            point = points[index]
            cell_x = floor(float(point[0]) / distance_m)
            cell_y = floor(float(point[1]) / distance_m)
            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    for candidate in cells.get(
                        (cell_x + offset_x, cell_y + offset_y), ()
                    ):
                        if visited[candidate]:
                            continue
                        delta = points[candidate] - point
                        if float(np.dot(delta, delta)) <= distance_sq:
                            visited[candidate] = True
                            pending.append(candidate)
        clusters.append(points[np.asarray(members, dtype=int)])
    return clusters


def _project_to_path(
    point: np.ndarray,
    path: np.ndarray,
) -> tuple[float, float]:
    best_distance = float('inf')
    best_signed = 0.0
    for start, end in zip(path[:-1], path[1:]):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1.0e-12:
            continue
        ratio = float(np.clip(
            np.dot(point - start, segment) / length_sq, 0.0, 1.0))
        projection = start + ratio * segment
        delta = point - projection
        distance = float(np.linalg.norm(delta))
        if distance < best_distance:
            tangent = segment / np.sqrt(length_sq)
            left_normal = np.asarray((-tangent[1], tangent[0]))
            best_distance = distance
            best_signed = float(np.dot(delta, left_normal))
    return best_distance, best_signed


def detect_obstacle_vehicle(
    lidar_points: Iterable[Sequence[float]],
    cone_centers: Iterable[Sequence[float]],
    path_points: Iterable[Sequence[float]],
    config: ObstacleAvoidanceConfig = ObstacleAvoidanceConfig(),
) -> ObstacleVehicle | None:
    """Return the nearest non-cone cluster intersecting the ego path width."""

    config.validate()
    path = _points(path_points)
    if len(path) < 2:
        return None
    non_cone = exclude_cone_points(
        lidar_points, cone_centers, config.cone_exclusion_radius_m)
    candidates = []
    for cluster in cluster_obstacle_points(
        non_cone, config.cluster_distance_m
    ):
        if len(cluster) < config.minimum_cluster_points:
            continue
        center = np.mean(cluster, axis=0)
        if not (
            config.detection_min_forward_m
            <= center[0]
            <= config.detection_max_forward_m
        ):
            continue
        span = float(np.linalg.norm(np.ptp(cluster, axis=0)))
        if not (
            config.minimum_cluster_span_m
            <= span
            <= config.maximum_cluster_span_m
        ):
            continue
        distances = [_project_to_path(point, path)[0] for point in cluster]
        path_distance = float(min(distances))
        if path_distance > config.vehicle_half_width_m:
            continue
        _center_distance, signed_offset = _project_to_path(center, path)
        if abs(signed_offset) <= 0.03:
            avoidance_sign = config.preferred_offset_sign
        else:
            avoidance_sign = -1 if signed_offset > 0.0 else 1
        candidates.append(ObstacleVehicle(
            center_x_m=float(center[0]),
            center_y_m=float(center[1]),
            path_distance_m=path_distance,
            signed_path_offset_m=signed_offset,
            point_count=len(cluster),
            span_m=span,
            avoidance_sign=avoidance_sign,
        ))
    return min(candidates, key=lambda item: item.center_x_m, default=None)


def make_opposite_lane_path(
    path_points: Iterable[Sequence[float]],
    signed_offset_m: float,
    lane_change_distance_m: float,
) -> np.ndarray:
    """Smoothly shift a path by one lane along its local left normal."""

    if not isfinite(float(signed_offset_m)) or signed_offset_m == 0.0:
        raise ValueError('signed_offset_m must be finite and non-zero')
    if (
        not isfinite(float(lane_change_distance_m))
        or lane_change_distance_m <= 0.0
    ):
        raise ValueError('lane_change_distance_m must be finite and positive')
    path = _points(path_points)
    if len(path) < 2:
        return np.empty((0, 2), dtype=np.float64)
    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    shifted = path.copy()
    for index in range(len(path)):
        if index == 0:
            tangent = path[1] - path[0]
        elif index == len(path) - 1:
            tangent = path[-1] - path[-2]
        else:
            tangent = path[index + 1] - path[index - 1]
        length = float(np.linalg.norm(tangent))
        if length <= 1.0e-9:
            continue
        tangent /= length
        left_normal = np.asarray((-tangent[1], tangent[0]))
        ratio = float(np.clip(arc[index] / lane_change_distance_m, 0.0, 1.0))
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        shifted[index] += left_normal * signed_offset_m * smooth
    shifted[0] = path[0]
    return shifted
