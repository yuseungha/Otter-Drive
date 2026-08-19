"""ROS-independent 2D LiDAR cone detection and local corridor planning.

The core intentionally depends only on NumPy.  ROS integration, TF lookup and
watchdogs live in :mod:`cone_line_planner`; this module can therefore be tested
with synthetic scans on any development machine.
"""

from dataclasses import dataclass
from math import atan2, cos, exp, radians, sin
from typing import Iterable, Sequence

import numpy as np


@dataclass
class PlannerConfig:
    """Tunable detection, pairing and path-validity limits."""

    # Sensor range filtering is followed by angular and Cartesian ROI checks in
    # planning_frame.  This keeps a rotated/translated LiDAR mount from
    # redefining vehicle forward.
    range_min_m: float = 0.15
    range_max_m: float = 5.0
    front_angle_min_deg: float = -180.0
    front_angle_max_deg: float = 180.0
    planning_min_forward_m: float = 0.10
    planning_max_forward_m: float = 5.0
    planning_max_abs_lateral_m: float = 2.5

    # Scan-order clustering. A short invalid-return hole may be bridged only
    # when the two real endpoints remain close in Cartesian and radial space.
    cluster_base_gap_m: float = 0.055
    cluster_adaptive_gap_factor: float = 2.5
    cluster_max_skip_beams: int = 1
    cluster_bridge_gap_multiplier: float = 1.35
    cluster_min_points: int = 2
    cluster_max_points: int = 60
    cluster_min_independent_ranges: int = 2
    cluster_same_range_epsilon_m: float = 0.0005
    cluster_min_angular_span_deg: float = 0.20
    cluster_max_angular_span_deg: float = 14.0
    cone_min_width_m: float = 0.015
    cone_max_width_m: float = 0.25
    cone_max_radial_depth_m: float = 0.18
    # A 2D scan sees the near surface, not the physical cone centre.  A
    # positive measured correction moves the cluster centre away from LiDAR.
    cone_center_radial_offset_m: float = 0.0
    cone_candidate_dedup_m: float = 0.07
    max_cone_candidates: int = 48

    # Temporal confirmation suppresses angle-compensation replicas and one-scan
    # glints. Missed tracks are remembered for association but never emitted as
    # current cones, so stale geometry cannot create a path.
    track_confirmation_scans: int = 2
    track_match_distance_m: float = 0.18
    track_max_missed_scans: int = 2
    track_smoothing_alpha: float = 0.75

    # Cone-pair geometry and multi-hypothesis path search.
    track_width_m: float = 0.60
    track_width_min_m: float = 0.42
    track_width_max_m: float = 0.82
    expected_cone_spacing_m: float = 0.297
    pair_max_along_error_m: float = 0.18
    pair_max_width_change_m: float = 0.18
    pair_max_boundary_step_difference_m: float = 0.20
    pair_forward_band_m: float = 0.22
    pair_beam_width: int = 32
    first_pair_min_distance_m: float = 0.12
    first_pair_max_distance_m: float = 1.50
    first_pair_max_lateral_m: float = 0.65
    require_first_pair_straddle: bool = False
    first_pair_non_straddle_penalty: float = 0.22
    min_forward_progress_m: float = 0.04
    max_midpoint_step_m: float = 0.78
    max_turn_angle_deg: float = 70.0
    max_pairs: int = 20
    min_pairs_for_path: int = 3
    heading_update_gain: float = 0.70

    # A short one-sided tail can be reconstructed only after a reliable,
    # two-sided corridor has already established boundary identity and width.
    # This borrows the useful "virtual cone" idea from Formula Student
    # planners without allowing an ambiguous single boundary to start a path.
    enable_single_side_fallback: bool = True
    min_real_pairs_before_virtual: int = 3
    max_virtual_pairs: int = 2
    max_virtual_fraction: float = 0.40
    virtual_cone_lateral_tolerance_m: float = 0.10
    virtual_cone_step_tolerance_m: float = 0.16
    virtual_boundary_uncertainty_m: float = 0.015
    virtual_cone_confidence_weight: float = 0.70

    # Path construction and fail-closed validity limits.
    include_vehicle_origin: bool = True
    smoothing_weight: float = 0.22
    smoothing_iterations: int = 2
    max_centerline_deviation_m: float = 0.025
    path_resolution_m: float = 0.05
    min_path_length_m: float = 0.80
    vehicle_width_m: float = 0.50
    safety_margin_m: float = 0.02
    # Track width is configured from cone-centre to cone-centre, so the default
    # envelope uses the vehicle half-width plus safety margin only.  Set this
    # to a measured cone footprint radius when the configured corridor is wide
    # enough to preserve that additional clearance.
    cone_obstacle_radius_m: float = 0.0
    max_path_curvature_1pm: float = 3.5
    curvature_sample_distance_m: float = 0.18
    confidence_full_pairs: int = 5
    min_plan_confidence: float = 0.40

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Reject unsafe or internally inconsistent parameter sets."""

        for name in ("front_angle_min_deg", "front_angle_max_deg"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

        positive = {
            "range_min_m": self.range_min_m,
            "range_max_m": self.range_max_m,
            "planning_max_forward_m": self.planning_max_forward_m,
            "planning_max_abs_lateral_m": self.planning_max_abs_lateral_m,
            "cluster_base_gap_m": self.cluster_base_gap_m,
            "cluster_bridge_gap_multiplier": self.cluster_bridge_gap_multiplier,
            "cluster_same_range_epsilon_m": self.cluster_same_range_epsilon_m,
            "cluster_max_angular_span_deg": self.cluster_max_angular_span_deg,
            "cone_max_width_m": self.cone_max_width_m,
            "track_match_distance_m": self.track_match_distance_m,
            "track_width_m": self.track_width_m,
            "track_width_min_m": self.track_width_min_m,
            "track_width_max_m": self.track_width_max_m,
            "expected_cone_spacing_m": self.expected_cone_spacing_m,
            "pair_max_along_error_m": self.pair_max_along_error_m,
            "pair_max_width_change_m": self.pair_max_width_change_m,
            "pair_max_boundary_step_difference_m": (
                self.pair_max_boundary_step_difference_m
            ),
            "pair_forward_band_m": self.pair_forward_band_m,
            "first_pair_max_distance_m": self.first_pair_max_distance_m,
            "first_pair_max_lateral_m": self.first_pair_max_lateral_m,
            "max_midpoint_step_m": self.max_midpoint_step_m,
            "curvature_sample_distance_m": self.curvature_sample_distance_m,
            "max_midpoint_step_m": self.max_midpoint_step_m,
            "path_resolution_m": self.path_resolution_m,
            "min_path_length_m": self.min_path_length_m,
            "vehicle_width_m": self.vehicle_width_m,
            "confidence_full_pairs": self.confidence_full_pairs,
        }
        for name, value in positive.items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and > 0")

        nonnegative = {
            "planning_min_forward_m": self.planning_min_forward_m,
            "cluster_adaptive_gap_factor": self.cluster_adaptive_gap_factor,
            "cluster_max_skip_beams": self.cluster_max_skip_beams,
            "cluster_min_angular_span_deg": self.cluster_min_angular_span_deg,
            "cone_min_width_m": self.cone_min_width_m,
            "cone_max_radial_depth_m": self.cone_max_radial_depth_m,
            "cone_center_radial_offset_m": self.cone_center_radial_offset_m,
            "cone_candidate_dedup_m": self.cone_candidate_dedup_m,
            "track_max_missed_scans": self.track_max_missed_scans,
            "first_pair_min_distance_m": self.first_pair_min_distance_m,
            "first_pair_non_straddle_penalty": self.first_pair_non_straddle_penalty,
            "min_forward_progress_m": self.min_forward_progress_m,
            "smoothing_weight": self.smoothing_weight,
            "smoothing_iterations": self.smoothing_iterations,
            "max_centerline_deviation_m": self.max_centerline_deviation_m,
            "safety_margin_m": self.safety_margin_m,
            "cone_obstacle_radius_m": self.cone_obstacle_radius_m,
            "max_path_curvature_1pm": self.max_path_curvature_1pm,
            "max_turn_angle_deg": self.max_turn_angle_deg,
            "virtual_cone_lateral_tolerance_m": self.virtual_cone_lateral_tolerance_m,
            "virtual_cone_step_tolerance_m": self.virtual_cone_step_tolerance_m,
            "virtual_boundary_uncertainty_m": self.virtual_boundary_uncertainty_m,
        }
        for name, value in nonnegative.items():
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and >= 0")

        integer_minimums = {
            "cluster_min_points": (self.cluster_min_points, 1),
            "cluster_max_points": (self.cluster_max_points, 1),
            "cluster_min_independent_ranges": (
                self.cluster_min_independent_ranges,
                1,
            ),
            "max_cone_candidates": (self.max_cone_candidates, 2),
            "track_confirmation_scans": (self.track_confirmation_scans, 1),
            "pair_beam_width": (self.pair_beam_width, 1),
            "max_pairs": (self.max_pairs, 1),
            "min_pairs_for_path": (self.min_pairs_for_path, 2),
            "min_real_pairs_before_virtual": (
                self.min_real_pairs_before_virtual,
                2,
            ),
            "max_virtual_pairs": (self.max_virtual_pairs, 0),
            "cluster_max_skip_beams": (self.cluster_max_skip_beams, 0),
            "track_max_missed_scans": (self.track_max_missed_scans, 0),
            "smoothing_iterations": (self.smoothing_iterations, 0),
            "confidence_full_pairs": (self.confidence_full_pairs, 1),
        }
        for name, (value, minimum) in integer_minimums.items():
            if int(value) != value or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")

        if self.range_min_m >= self.range_max_m:
            raise ValueError("range_min_m must be smaller than range_max_m")
        if self.planning_min_forward_m >= self.planning_max_forward_m:
            raise ValueError(
                "planning_min_forward_m must be smaller than planning_max_forward_m"
            )
        if self.front_angle_min_deg >= self.front_angle_max_deg:
            raise ValueError("front_angle_min_deg must be smaller than front_angle_max_deg")
        if self.front_angle_max_deg - self.front_angle_min_deg > 360.0 + 1.0e-9:
            raise ValueError("scan angle window cannot exceed 360 degrees")
        if self.cluster_min_points > self.cluster_max_points:
            raise ValueError("cluster_min_points cannot exceed cluster_max_points")
        if self.cluster_min_independent_ranges > self.cluster_max_points:
            raise ValueError(
                "cluster_min_independent_ranges cannot exceed cluster_max_points"
            )
        if self.cluster_min_angular_span_deg > self.cluster_max_angular_span_deg:
            raise ValueError("cluster angular span limits are reversed")
        if self.cone_min_width_m >= self.cone_max_width_m:
            raise ValueError("cone width limits are reversed")
        if not self.track_width_min_m < self.track_width_m < self.track_width_max_m:
            raise ValueError("track_width_m must lie inside its min/max limits")
        if self.first_pair_min_distance_m >= self.first_pair_max_distance_m:
            raise ValueError("first-pair distance limits are reversed")
        if not 0.0 < self.max_turn_angle_deg < 180.0:
            raise ValueError("max_turn_angle_deg must be in (0, 180)")
        if not 0.0 <= self.heading_update_gain <= 1.0:
            raise ValueError("heading_update_gain must be in [0, 1]")
        if not 0.0 < self.track_smoothing_alpha <= 1.0:
            raise ValueError("track_smoothing_alpha must be in (0, 1]")
        if not 0.0 <= self.smoothing_weight <= 1.0:
            raise ValueError("smoothing_weight must be in [0, 1]")
        if not 0.0 <= self.min_plan_confidence <= 1.0:
            raise ValueError("min_plan_confidence must be in [0, 1]")
        if not 0.0 < self.virtual_cone_confidence_weight <= 1.0:
            raise ValueError("virtual_cone_confidence_weight must be in (0, 1]")
        if not 0.0 <= self.max_virtual_fraction < 1.0:
            raise ValueError("max_virtual_fraction must be in [0, 1)")
        for name in (
            "require_first_pair_straddle",
            "enable_single_side_fallback",
            "include_vehicle_origin",
        ):
            if not isinstance(getattr(self, name), (bool, np.bool_)):
                raise ValueError(f"{name} must be boolean")


@dataclass
class PlanResult:
    """Selected boundaries, local path and an actionable validity reason."""

    left_boundary: np.ndarray
    right_boundary: np.ndarray
    raw_centerline: np.ndarray
    path: np.ndarray
    status: str = "NO_PATH"
    confidence: float = 0.0
    path_length_m: float = 0.0
    max_curvature_1pm: float = 0.0
    smoothing_applied: bool = False
    candidate_count: int = 0
    real_pair_count: int = 0
    virtual_pair_count: int = 0

    @property
    def valid(self) -> bool:
        return self.status in {"OK", "OK_VIRTUAL"} and len(self.path) >= 2

    @property
    def matched_pair_count(self) -> int:
        return len(self.raw_centerline)


def _empty_points() -> np.ndarray:
    return np.empty((0, 2), dtype=float)


def empty_plan_result(status: str, candidate_count: int = 0) -> PlanResult:
    empty = _empty_points()
    return PlanResult(empty, empty, empty, empty, status=status, candidate_count=candidate_count)


def _normalise(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < 1.0e-9:
        return np.zeros(2, dtype=float)
    return vector / length


def left_normal_from_tangent(tangent: Sequence[float]) -> np.ndarray:
    """Return the unit left normal ``(-ty, tx)`` of a station tangent.

    The tangent and resulting normal are expressed in ``planning_frame``.  In
    the real node that frame is normally ``base_link`` at the rear-axle
    centre; no global x/y-axis offset is introduced here.
    """

    tangent_value = np.asarray(tangent, dtype=float)
    if tangent_value.shape != (2,) or not np.all(np.isfinite(tangent_value)):
        raise ValueError("tangent must be a finite 2D vector")
    unit_tangent = _normalise(tangent_value)
    if float(np.linalg.norm(unit_tangent)) < 1.0e-9:
        raise ValueError("tangent must be non-zero")
    return np.array([-unit_tangent[1], unit_tangent[0]], dtype=float)


def station_center_from_boundaries(
    left_boundary: Sequence[float] | None,
    right_boundary: Sequence[float] | None,
    tangent: Sequence[float],
    estimated_track_width_m: float,
) -> np.ndarray:
    """Compute a station centre using only its local tangent/left normal.

    Two real boundaries use their midpoint.  A single real boundary is offset
    by half the estimated cone-centre track width along the station's local
    normal.  The function deliberately has no fixed-y or curvature/racing-line
    offset input.
    """

    if left_boundary is None and right_boundary is None:
        raise ValueError("at least one boundary point is required")
    width = float(estimated_track_width_m)
    if not np.isfinite(width) or width <= 0.0:
        raise ValueError("estimated_track_width_m must be finite and > 0")
    normal = left_normal_from_tangent(tangent)

    def point(name: str, value: Sequence[float] | None) -> np.ndarray | None:
        if value is None:
            return None
        result = np.asarray(value, dtype=float)
        if result.shape != (2,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite 2D point")
        return result

    left = point("left_boundary", left_boundary)
    right = point("right_boundary", right_boundary)
    if left is not None and right is not None:
        return 0.5 * (left + right)
    if left is not None:
        return left - 0.5 * width * normal
    assert right is not None
    return right + 0.5 * width * normal


def _wrapped_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def _angle_is_selected(angle: float, config: PlannerConfig) -> bool:
    if config.front_angle_max_deg - config.front_angle_min_deg >= 360.0 - 1.0e-9:
        return True
    wrapped = _wrapped_angle(angle)
    return radians(config.front_angle_min_deg) <= wrapped <= radians(
        config.front_angle_max_deg
    )


def _maximum_pairwise_distance(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    differences = points[:, None, :] - points[None, :, :]
    return float(np.sqrt(np.max(np.sum(differences * differences, axis=2))))


def _independent_range_count(values: np.ndarray, epsilon: float) -> int:
    """Count distinct range levels after A1 angle-compensation replication."""

    if len(values) == 0:
        return 0
    ordered = np.sort(np.asarray(values, dtype=float))
    return 1 + int(np.count_nonzero(np.diff(ordered) > epsilon))


def detect_cones_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    config: PlannerConfig,
    sensor_to_planning: Sequence[float] = (0.0, 0.0, 0.0),
    sensor_range_min_m: float | None = None,
    sensor_range_max_m: float | None = None,
) -> np.ndarray:
    """Return cone-sized scan clusters in the configured planning frame.

    ``sensor_to_planning`` is ``(translation_x, translation_y, yaw)`` for the
    scan timestamp.  Clusters are connected in angular scan order, while the
    vehicle-forward ROI is evaluated after this transform.
    """

    config.validate()
    if len(ranges) == 0:
        return _empty_points()
    if not np.isfinite(angle_min) or not np.isfinite(angle_increment):
        raise ValueError("LaserScan angles must be finite")
    if abs(angle_increment) < 1.0e-12:
        raise ValueError("LaserScan angle_increment must be non-zero")
    if len(sensor_to_planning) != 3:
        raise ValueError("sensor_to_planning must contain x, y and yaw")

    tx, ty, yaw = (float(value) for value in sensor_to_planning)
    if not all(np.isfinite(value) for value in (tx, ty, yaw)):
        raise ValueError("sensor_to_planning must be finite")
    transform_cos = cos(yaw)
    transform_sin = sin(yaw)

    effective_min = config.range_min_m
    effective_max = config.range_max_m
    if sensor_range_min_m is not None and np.isfinite(sensor_range_min_m):
        effective_min = max(effective_min, float(sensor_range_min_m))
    if sensor_range_max_m is not None and np.isfinite(sensor_range_max_m):
        effective_max = min(effective_max, float(sensor_range_max_m))
    if effective_min >= effective_max:
        raise ValueError("configured and sensor range limits do not overlap")

    # Each sample is (planning point, sensor range, scan index).
    clusters: list[list[tuple[np.ndarray, float, int]]] = []
    active: list[tuple[np.ndarray, float, int]] = []
    previous_point: np.ndarray | None = None
    previous_range = 0.0
    skipped_beams = 0

    def finish_active() -> None:
        nonlocal active, previous_point, previous_range, skipped_beams
        if active:
            clusters.append(active)
        active = []
        previous_point = None
        previous_range = 0.0
        skipped_beams = 0

    for index, raw_range in enumerate(ranges):
        angle = float(angle_min + index * angle_increment)
        distance = float(raw_range)
        if not (
            np.isfinite(distance) and effective_min <= distance <= effective_max
        ):
            if active and skipped_beams < config.cluster_max_skip_beams:
                skipped_beams += 1
            else:
                finish_active()
            continue

        sensor_x = distance * cos(angle)
        sensor_y = distance * sin(angle)
        point = np.array(
            [
                tx + transform_cos * sensor_x - transform_sin * sensor_y,
                ty + transform_sin * sensor_x + transform_cos * sensor_y,
            ],
            dtype=float,
        )
        # Both the angular gate and Cartesian ROI are vehicle/planning-frame
        # quantities.  Applying this gate to the raw laser angle would rotate
        # the ROI with a non-zero lidar_yaw_rad mount.
        planning_angle = atan2(float(point[1]), float(point[0]))
        if not _angle_is_selected(planning_angle, config):
            finish_active()
            continue
        in_planning_roi = (
            config.planning_min_forward_m
            <= point[0]
            <= config.planning_max_forward_m
            and abs(float(point[1])) <= config.planning_max_abs_lateral_m
        )
        if not in_planning_roi:
            finish_active()
            continue

        if previous_point is None:
            active = [(point, distance, index)]
        else:
            beam_steps = skipped_beams + 1
            adaptive_gap = (
                config.cluster_adaptive_gap_factor
                * min(previous_range, distance)
                * abs(angle_increment)
                * beam_steps
            )
            allowed_gap = max(config.cluster_base_gap_m, adaptive_gap)
            if skipped_beams:
                allowed_gap *= config.cluster_bridge_gap_multiplier
            radial_limit = config.cone_max_radial_depth_m * (
                config.cluster_bridge_gap_multiplier if skipped_beams else 1.0
            )
            connected = (
                float(np.linalg.norm(point - previous_point)) <= allowed_gap
                and abs(distance - previous_range) <= radial_limit
            )
            if connected:
                active.append((point, distance, index))
            else:
                finish_active()
                active = [(point, distance, index)]

        previous_point = point
        previous_range = distance
        skipped_beams = 0

    finish_active()

    candidates: list[tuple[np.ndarray, int]] = []
    increment_deg = abs(float(np.degrees(angle_increment)))
    for cluster in clusters:
        count = len(cluster)
        if not config.cluster_min_points <= count <= config.cluster_max_points:
            continue
        points = np.asarray([sample[0] for sample in cluster], dtype=float)
        distances = np.asarray([sample[1] for sample in cluster], dtype=float)
        independent_ranges = _independent_range_count(
            distances, config.cluster_same_range_epsilon_m
        )
        angular_span_deg = increment_deg * abs(cluster[-1][2] - cluster[0][2])
        extent = _maximum_pairwise_distance(points)
        radial_depth = float(np.ptp(distances))
        if not (
            config.cluster_min_angular_span_deg
            <= angular_span_deg
            <= config.cluster_max_angular_span_deg
        ):
            continue
        if independent_ranges < config.cluster_min_independent_ranges:
            continue
        if not config.cone_min_width_m <= extent <= config.cone_max_width_m:
            continue
        if radial_depth > config.cone_max_radial_depth_m:
            continue

        # The median is less sensitive than the mean to a grazing edge return.
        # It still lies on the visible near surface, so an optional measured
        # correction moves it radially away from the sensor toward cone centre.
        center = np.median(points, axis=0)
        radial = center - np.array([tx, ty], dtype=float)
        radial_length = float(np.linalg.norm(radial))
        if config.cone_center_radial_offset_m > 0.0 and radial_length > 1.0e-9:
            center = center + (
                config.cone_center_radial_offset_m * radial / radial_length
            )
        candidates.append((center, count))

    # Merge split views of the same physical object deterministically.
    candidates.sort(key=lambda item: (-item[1], float(item[0][0]), float(item[0][1])))
    accepted: list[np.ndarray] = []
    for center, _ in candidates:
        if any(
            float(np.linalg.norm(center - existing)) < config.cone_candidate_dedup_m
            for existing in accepted
        ):
            continue
        accepted.append(center)
        if len(accepted) >= config.max_cone_candidates:
            break

    if not accepted:
        return _empty_points()
    result = np.asarray(accepted, dtype=float)
    order = np.lexsort((result[:, 1], result[:, 0]))
    return result[order]


@dataclass
class _ConeTrack:
    position: np.ndarray
    hits: int
    misses: int = 0


class ConeTrackFilter:
    """Confirm candidates across scans without outputting missed/stale tracks."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self._tracks: list[_ConeTrack] = []

    def reset(self) -> None:
        self._tracks = []

    def update(self, candidates: Iterable[Sequence[float]]) -> np.ndarray:
        values = np.asarray(list(candidates), dtype=float)
        if values.size == 0:
            values = _empty_points()
        elif values.ndim != 2 or values.shape[1] != 2:
            raise ValueError("candidates must have shape (N, 2)")
        values = values[np.all(np.isfinite(values), axis=1)]

        edges: list[tuple[float, int, int]] = []
        for track_index, track in enumerate(self._tracks):
            for candidate_index, candidate in enumerate(values):
                distance = float(np.linalg.norm(track.position - candidate))
                if distance <= self.config.track_match_distance_m:
                    edges.append((distance, track_index, candidate_index))
        edges.sort()

        matched_tracks: set[int] = set()
        matched_candidates: set[int] = set()
        current_confirmed: list[np.ndarray] = []
        alpha = self.config.track_smoothing_alpha
        for _, track_index, candidate_index in edges:
            if track_index in matched_tracks or candidate_index in matched_candidates:
                continue
            track = self._tracks[track_index]
            candidate = values[candidate_index]
            track.position = alpha * candidate + (1.0 - alpha) * track.position
            track.hits += 1
            track.misses = 0
            matched_tracks.add(track_index)
            matched_candidates.add(candidate_index)
            if track.hits >= self.config.track_confirmation_scans:
                current_confirmed.append(track.position.copy())

        for index, track in enumerate(self._tracks):
            if index not in matched_tracks:
                track.misses += 1
                # Confirmation is intentionally a consecutive-hit streak.
                # Remember the position briefly for association, but a gap
                # must not let intermittent glints become confirmed cones.
                track.hits = 0

        self._tracks = [
            track
            for track in self._tracks
            if track.misses <= self.config.track_max_missed_scans
        ]
        for candidate_index, candidate in enumerate(values):
            if candidate_index not in matched_candidates:
                track = _ConeTrack(candidate.copy(), hits=1)
                self._tracks.append(track)
                if self.config.track_confirmation_scans <= 1:
                    current_confirmed.append(track.position.copy())

        if not current_confirmed:
            return _empty_points()
        confirmed = np.asarray(current_confirmed, dtype=float)
        order = np.lexsort((confirmed[:, 1], confirmed[:, 0]))
        return confirmed[order]


@dataclass(frozen=True)
class _PairCandidate:
    first_index: int
    second_index: int
    midpoint: np.ndarray
    width: float


@dataclass(frozen=True)
class _SearchState:
    score: float
    used: frozenset[int]
    left: tuple[np.ndarray, ...]
    right: tuple[np.ndarray, ...]
    centers: tuple[np.ndarray, ...]
    widths: tuple[float, ...]
    along_errors: tuple[float, ...]
    steps: tuple[float, ...]
    heading: np.ndarray


def _pair_candidates(cones: np.ndarray, config: PlannerConfig) -> list[_PairCandidate]:
    pairs: list[_PairCandidate] = []
    for first_index in range(len(cones)):
        for second_index in range(first_index + 1, len(cones)):
            width = float(np.linalg.norm(cones[second_index] - cones[first_index]))
            if config.track_width_min_m <= width <= config.track_width_max_m:
                pairs.append(
                    _PairCandidate(
                        first_index,
                        second_index,
                        0.5 * (cones[first_index] + cones[second_index]),
                        width,
                    )
                )
    return pairs


def _evaluate_pair_extension(
    state: _SearchState,
    pair: _PairCandidate,
    cones: np.ndarray,
    config: PlannerConfig,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, float, float] | None:
    if pair.first_index in state.used or pair.second_index in state.used:
        return None

    first_pair = len(state.centers) == 0
    current = state.centers[-1] if state.centers else np.zeros(2, dtype=float)
    heading = state.heading
    travel = pair.midpoint - current
    step = float(np.linalg.norm(travel))
    if step < 1.0e-9:
        return None
    forward = float(np.dot(travel, heading))
    normal = np.array([-heading[1], heading[0]], dtype=float)
    lateral = abs(float(np.dot(travel, normal)))

    if first_pair:
        if not config.first_pair_min_distance_m <= step <= config.first_pair_max_distance_m:
            return None
        if forward <= config.min_forward_progress_m:
            return None
        if lateral > config.first_pair_max_lateral_m:
            return None
    else:
        if forward <= config.min_forward_progress_m or step > config.max_midpoint_step_m:
            return None
        direction = travel / step
        # On the second station the origin-to-first-center direction can be
        # dominated by a lateral entry offset. Let the pair progression seed
        # the course tangent before enforcing the normal turn gate.
        if len(state.centers) == 1:
            reference_heading = direction
        else:
            reference_heading = heading
        turn = float(
            np.arccos(np.clip(np.dot(direction, reference_heading), -1.0, 1.0))
        )
        if turn > radians(config.max_turn_angle_deg):
            return None

    first = cones[pair.first_index]
    second = cones[pair.second_index]
    span = second - first
    pair_heading = (
        travel / step
        if not first_pair and len(state.centers) == 1
        else heading
    )
    along_error = abs(float(np.dot(span, pair_heading)))
    if along_error > config.pair_max_along_error_m:
        return None

    first_lateral = float(np.dot(first - current, normal))
    second_lateral = float(np.dot(second - current, normal))
    if first_lateral >= second_lateral:
        left_point, right_point = first, second
    else:
        left_point, right_point = second, first

    non_straddle_penalty = 0.0
    if first_pair:
        first_side = first_lateral
        second_side = second_lateral
        straddles = first_side * second_side <= 0.0
        if config.require_first_pair_straddle and not straddles:
            return None
        if not straddles:
            non_straddle_penalty = config.first_pair_non_straddle_penalty
    else:
        # Preserve the boundary identity established by the first pair.  A
        # midpoint heading can change rapidly on an offset entry, but swapping
        # left/right rows would create crossed boundaries and false step gates.
        keep_cost = float(
            np.linalg.norm(first - state.left[-1])
            + np.linalg.norm(second - state.right[-1])
        )
        swap_cost = float(
            np.linalg.norm(second - state.left[-1])
            + np.linalg.norm(first - state.right[-1])
        )
        if swap_cost < keep_cost:
            left_point, right_point = second, first
        else:
            left_point, right_point = first, second
        width_change = abs(pair.width - state.widths[-1])
        if width_change > config.pair_max_width_change_m:
            return None
        left_step_vector = left_point - state.left[-1]
        right_step_vector = right_point - state.right[-1]
        left_step = float(np.linalg.norm(left_step_vector))
        right_step = float(np.linalg.norm(right_step_vector))
        if (
            float(np.dot(left_step_vector, heading)) < -0.02
            or float(np.dot(right_step_vector, heading)) < -0.02
        ):
            return None
        if abs(left_step - right_step) > config.pair_max_boundary_step_difference_m:
            return None

    # Missing complete stations are allowed when the step is near an integer
    # multiple of the expected spacing, but receive a small explicit penalty.
    if first_pair:
        target_step = min(config.expected_cone_spacing_m, step)
        skipped_station_penalty = 0.0
    else:
        multiple = max(1, int(round(step / config.expected_cone_spacing_m)))
        target_step = multiple * config.expected_cone_spacing_m
        skipped_station_penalty = 0.10 * max(0, multiple - 1)
    step_error = abs(step - target_step)
    width_error = abs(pair.width - config.track_width_m)
    direction = travel / step
    turn = float(
        np.arccos(
            np.clip(
                np.dot(
                    direction,
                    direction
                    if not first_pair and len(state.centers) == 1
                    else heading,
                ),
                -1.0,
                1.0,
            )
        )
    )
    extension_score = (
        2.7 * width_error
        + 2.3 * along_error
        + 1.5 * step_error
        + 0.35 * lateral
        + 0.30 * turn / np.pi
        + non_straddle_penalty
        + skipped_station_penalty
    )
    return extension_score, left_point, right_point, direction, along_error, step


def _search_pair_chain(
    cones: np.ndarray, config: PlannerConfig
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    pairs = _pair_candidates(cones, config)
    if not pairs:
        empty = _empty_points()
        return empty, empty, empty, (), (), ()

    initial = _SearchState(
        score=0.0,
        used=frozenset(),
        left=(),
        right=(),
        centers=(),
        widths=(),
        along_errors=(),
        steps=(),
        heading=np.array([1.0, 0.0], dtype=float),
    )
    beam = [initial]
    completed: list[_SearchState] = []

    for _ in range(config.max_pairs):
        next_states: list[_SearchState] = []
        for state in beam:
            viable: list[
                tuple[_PairCandidate, tuple[float, np.ndarray, np.ndarray, np.ndarray, float, float]]
            ] = []
            for pair in pairs:
                evaluated = _evaluate_pair_extension(state, pair, cones, config)
                if evaluated is not None:
                    viable.append((pair, evaluated))

            if not viable:
                completed.append(state)
                continue

            current = state.centers[-1] if state.centers else np.zeros(2, dtype=float)
            minimum_forward = min(
                float(np.dot(pair.midpoint - current, state.heading))
                for pair, _ in viable
            )
            viable = [
                item
                for item in viable
                if float(np.dot(item[0].midpoint - current, state.heading))
                <= minimum_forward + config.pair_forward_band_m
            ]

            for pair, evaluated in viable:
                extension_score, left, right, direction, along_error, step = evaluated
                blended = (
                    (1.0 - config.heading_update_gain) * state.heading
                    + config.heading_update_gain * direction
                )
                heading = _normalise(blended)
                next_states.append(
                    _SearchState(
                        score=state.score + extension_score,
                        used=state.used | {pair.first_index, pair.second_index},
                        left=state.left + (left,),
                        right=state.right + (right,),
                        centers=state.centers + (pair.midpoint,),
                        widths=state.widths + (pair.width,),
                        along_errors=state.along_errors + (along_error,),
                        steps=state.steps + (step,),
                        heading=heading,
                    )
                )

        if not next_states:
            break
        # All states at this depth have equal pair count. Keep diverse, low-cost
        # hypotheses instead of committing to one locally attractive pair.
        next_states.sort(
            key=lambda state: (
                state.score / max(1, len(state.centers)),
                -float(np.linalg.norm(state.centers[-1])),
                tuple(sorted(state.used)),
            )
        )
        beam = next_states[: config.pair_beam_width]

    completed.extend(beam)
    completed = [state for state in completed if state.centers]
    if not completed:
        empty = _empty_points()
        return empty, empty, empty, (), (), ()

    best = max(
        completed,
        key=lambda state: (
            len(state.centers),
            float(np.linalg.norm(state.centers[-1] - state.centers[0]))
            if len(state.centers) > 1
            else 0.0,
            -state.score / len(state.centers),
        ),
    )
    return (
        np.asarray(best.left, dtype=float),
        np.asarray(best.right, dtype=float),
        np.asarray(best.centers, dtype=float),
        best.widths,
        best.along_errors,
        best.steps,
    )


def _extend_with_virtual_pairs(
    cones: np.ndarray,
    left: np.ndarray,
    right: np.ndarray,
    centers: np.ndarray,
    widths: tuple[float, ...],
    along_errors: tuple[float, ...],
    steps: tuple[float, ...],
    config: PlannerConfig,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
    int,
    bool,
]:
    """Append a tightly bounded one-sided tail using virtual opposite cones.

    A virtual pair is never allowed to seed a path.  It is considered only
    after several real pairs establish left/right identity, local heading and
    track width.  If plausible observations exist on both sides, extension is
    refused so the ordinary two-sided search remains authoritative.
    """

    if (
        not config.enable_single_side_fallback
        or config.max_virtual_pairs == 0
        or len(centers) < config.min_real_pairs_before_virtual
        or len(centers) < 2
    ):
        return left, right, centers, widths, along_errors, steps, 0, False

    left_values = [point.copy() for point in left]
    right_values = [point.copy() for point in right]
    center_values = [point.copy() for point in centers]
    width_values = list(widths)
    along_values = list(along_errors)
    step_values = list(steps)

    used = np.zeros(len(cones), dtype=bool)
    for index, cone in enumerate(cones):
        used[index] = any(
            float(np.linalg.norm(cone - boundary_point)) <= 1.0e-6
            for boundary_point in (*left_values, *right_values)
        )

    virtual_count = 0
    virtual_limit_exceeded = False
    # One extra probe detects a plausible third missing-side station instead
    # of silently truncating the course and presenting a partial path as safe.
    for _ in range(config.max_virtual_pairs + 1):
        heading = _normalise(center_values[-1] - center_values[-2])
        if float(np.linalg.norm(heading)) < 1.0e-9:
            break
        normal = left_normal_from_tangent(heading)

        side_candidates: dict[str, list[tuple[float, int]]] = {
            "left": [],
            "right": [],
        }
        for index, cone in enumerate(cones):
            if used[index]:
                continue
            for side, previous in (
                ("left", left_values[-1]),
                ("right", right_values[-1]),
            ):
                delta = cone - previous
                forward = float(np.dot(delta, heading))
                lateral = abs(float(np.dot(delta, normal)))
                step = float(np.linalg.norm(delta))
                step_error = abs(step - config.expected_cone_spacing_m)
                if (
                    forward > config.min_forward_progress_m
                    and lateral <= config.virtual_cone_lateral_tolerance_m
                    and step_error <= config.virtual_cone_step_tolerance_m
                ):
                    score = (
                        step_error / max(config.virtual_cone_step_tolerance_m, 1.0e-6)
                        + lateral
                        / max(config.virtual_cone_lateral_tolerance_m, 1.0e-6)
                    )
                    side_candidates[side].append((score, index))

        has_left = bool(side_candidates["left"])
        has_right = bool(side_candidates["right"])
        if has_left == has_right:
            # No observation, or an ambiguous/two-sided station: do not
            # hallucinate a boundary.
            break

        side = "left" if has_left else "right"
        if virtual_count >= config.max_virtual_pairs:
            virtual_limit_exceeded = True
            break
        proposed_fraction = (virtual_count + 1) / (
            len(center_values) + 1
        )
        if proposed_fraction > config.max_virtual_fraction + 1.0e-9:
            virtual_limit_exceeded = True
            break
        _, cone_index = min(side_candidates[side])
        real_point = cones[cone_index].copy()
        previous_real = left_values[-1] if side == "left" else right_values[-1]
        new_heading = _normalise(real_point - previous_real)
        if float(np.linalg.norm(new_heading)) < 1.0e-9:
            break
        turn = float(
            np.arccos(np.clip(np.dot(heading, new_heading), -1.0, 1.0))
        )
        if turn > radians(config.max_turn_angle_deg):
            break
        local_width = float(
            np.median(width_values[-min(3, len(width_values)) :])
        )
        local_width = float(
            np.clip(local_width, config.track_width_min_m, config.track_width_max_m)
        )
        if side == "left":
            new_left = real_point
            new_center = station_center_from_boundaries(
                new_left, None, new_heading, local_width
            )
            new_right = 2.0 * new_center - new_left
        else:
            new_right = real_point
            new_center = station_center_from_boundaries(
                None, new_right, new_heading, local_width
            )
            new_left = 2.0 * new_center - new_right
        center_delta = new_center - center_values[-1]
        center_step = float(np.linalg.norm(center_delta))
        if (
            float(np.dot(center_delta, heading)) <= config.min_forward_progress_m
            or center_step > config.max_midpoint_step_m
            or new_center[0] < config.planning_min_forward_m
            or new_center[0] > config.planning_max_forward_m
            or abs(float(new_center[1])) > config.planning_max_abs_lateral_m
        ):
            break

        left_values.append(new_left)
        right_values.append(new_right)
        center_values.append(new_center)
        width_values.append(local_width)
        along_values.append(0.0)
        step_values.append(center_step)
        used[cone_index] = True
        virtual_count += 1

    return (
        np.asarray(left_values, dtype=float),
        np.asarray(right_values, dtype=float),
        np.asarray(center_values, dtype=float),
        tuple(width_values),
        tuple(along_values),
        tuple(step_values),
        virtual_count,
        virtual_limit_exceeded,
    )


def _smooth_polyline_bounded(
    points: np.ndarray, weight: float, iterations: int, max_displacement: float
) -> tuple[np.ndarray, bool]:
    if len(points) < 3 or iterations <= 0 or weight <= 0.0 or max_displacement <= 0.0:
        return points.copy(), False

    original = points.copy()
    smoothed = points.copy()
    for _ in range(iterations):
        previous = smoothed.copy()
        neighbour_average = 0.5 * (previous[:-2] + previous[2:])
        proposed = (1.0 - weight) * previous[1:-1] + weight * neighbour_average
        displacement = proposed - original[1:-1]
        lengths = np.linalg.norm(displacement, axis=1)
        scale = np.ones_like(lengths)
        too_far = lengths > max_displacement
        scale[too_far] = max_displacement / lengths[too_far]
        smoothed[1:-1] = original[1:-1] + displacement * scale[:, None]
    return smoothed, bool(np.max(np.linalg.norm(smoothed - original, axis=1)) > 1.0e-7)


def _resample_polyline(points: np.ndarray, resolution: float) -> np.ndarray:
    if len(points) < 2:
        return points.copy()
    segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate(([True], segments > 1.0e-6))
    points = points[keep]
    if len(points) < 2:
        return points.copy()
    arc = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )
    total = float(arc[-1])
    samples = np.arange(0.0, total, max(float(resolution), 0.01), dtype=float)
    if len(samples) == 0 or total - samples[-1] > 1.0e-6:
        samples = np.append(samples, total)
    return np.column_stack(
        (
            np.interp(samples, arc, points[:, 0]),
            np.interp(samples, arc, points[:, 1]),
        )
    )


def _polyline_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    if len(polyline) == 0:
        return float("inf")
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))
    starts = polyline[:-1]
    segments = polyline[1:] - starts
    lengths_sq = np.sum(segments * segments, axis=1)
    projection = np.zeros(len(segments), dtype=float)
    valid = lengths_sq > 1.0e-12
    projection[valid] = np.sum((point - starts[valid]) * segments[valid], axis=1) / lengths_sq[valid]
    projection = np.clip(projection, 0.0, 1.0)
    closest = starts + projection[:, None] * segments
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


def _distance_between_segments(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> float:
    """Return the shortest 2D distance between two closed line segments."""

    def orientation(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        ab = b - a
        ac = c - a
        return float(ab[0] * ac[1] - ab[1] * ac[0])

    first_side_a = orientation(first_start, first_end, second_start)
    first_side_b = orientation(first_start, first_end, second_end)
    second_side_a = orientation(second_start, second_end, first_start)
    second_side_b = orientation(second_start, second_end, first_end)
    if first_side_a * first_side_b <= 0.0 and second_side_a * second_side_b <= 0.0:
        return 0.0
    return min(
        _distance_to_polyline(first_start, np.vstack((second_start, second_end))),
        _distance_to_polyline(first_end, np.vstack((second_start, second_end))),
        _distance_to_polyline(second_start, np.vstack((first_start, first_end))),
        _distance_to_polyline(second_end, np.vstack((first_start, first_end))),
    )


def _minimum_path_to_boundary_distance(
    path: np.ndarray, boundary: np.ndarray
) -> float:
    if len(path) == 0 or len(boundary) == 0:
        return float("inf")
    if len(path) == 1 or len(boundary) == 1:
        return min(_distance_to_polyline(point, boundary) for point in path)
    minimum = float("inf")
    for path_start, path_end in zip(path[:-1], path[1:]):
        for boundary_start, boundary_end in zip(boundary[:-1], boundary[1:]):
            minimum = min(
                minimum,
                _distance_between_segments(
                    path_start,
                    path_end,
                    boundary_start,
                    boundary_end,
                ),
            )
    return minimum


def _maximum_path_curvature(
    path: np.ndarray, sample_distance: float
) -> float:
    if len(path) < 3:
        return 0.0
    resolution = max(_polyline_length(path) / max(1, len(path) - 1), 1.0e-3)
    stride = max(1, int(round(sample_distance / resolution)))
    maximum = 0.0
    for index in range(stride, len(path) - stride):
        first = path[index] - path[index - stride]
        second = path[index + stride] - path[index]
        first_length = float(np.linalg.norm(first))
        second_length = float(np.linalg.norm(second))
        if first_length < 1.0e-6 or second_length < 1.0e-6:
            continue
        turn = abs(
            atan2(
                float(first[0] * second[1] - first[1] * second[0]),
                float(np.dot(first, second)),
            )
        )
        curvature = turn / max(0.5 * (first_length + second_length), 1.0e-6)
        maximum = max(maximum, curvature)
    return maximum


def plan_centerline(
    cone_centers: Iterable[Sequence[float]], config: PlannerConfig
) -> PlanResult:
    """Build a confidence-gated center path through two cone boundaries."""

    config.validate()
    cones = np.asarray(list(cone_centers), dtype=float)
    if cones.size == 0:
        return empty_plan_result("NOT_ENOUGH_CONES")
    if cones.ndim != 2 or cones.shape[1] != 2:
        raise ValueError("cone_centers must have shape (N, 2)")
    cones = cones[np.all(np.isfinite(cones), axis=1)]
    cones = cones[
        (cones[:, 0] >= config.planning_min_forward_m)
        & (cones[:, 0] <= config.planning_max_forward_m)
        & (np.abs(cones[:, 1]) <= config.planning_max_abs_lateral_m)
    ]
    if len(cones) < 2:
        return empty_plan_result("NOT_ENOUGH_CONES", len(cones))
    order = np.lexsort((cones[:, 1], cones[:, 0]))
    cones = cones[order][: config.max_cone_candidates]

    left, right, centers, widths, along_errors, steps = _search_pair_chain(
        cones, config
    )
    if len(centers) == 0:
        return empty_plan_result("NO_VALID_PAIR", len(cones))
    real_pair_count = len(centers)
    if real_pair_count < config.min_pairs_for_path:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="INSUFFICIENT_PAIRS",
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
        )

    (
        left,
        right,
        centers,
        widths,
        along_errors,
        steps,
        virtual_pair_count,
        virtual_limit_exceeded,
    ) = _extend_with_virtual_pairs(
        cones,
        left,
        right,
        centers,
        widths,
        along_errors,
        steps,
        config,
    )
    if virtual_limit_exceeded:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="VIRTUAL_LIMIT_EXCEEDED",
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )

    minimum_half_clearance = min(widths) * 0.5 - config.vehicle_width_m * 0.5
    usable_deviation = minimum_half_clearance - config.safety_margin_m
    if usable_deviation <= 0.0:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="INSUFFICIENT_CLEARANCE",
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )

    control_points = centers.copy()
    if config.include_vehicle_origin:
        control_points = np.vstack((np.zeros((1, 2), dtype=float), control_points))
    maximum_deviation = min(config.max_centerline_deviation_m, usable_deviation)
    smoothed, smoothing_applied = _smooth_polyline_bounded(
        control_points,
        config.smoothing_weight,
        config.smoothing_iterations,
        maximum_deviation,
    )
    path = _resample_polyline(smoothed, config.path_resolution_m)
    path_length = _polyline_length(path)
    required_center_clearance = config.vehicle_width_m * 0.5 + config.safety_margin_m
    if virtual_pair_count:
        required_center_clearance += config.virtual_boundary_uncertainty_m
    clearance_path = path
    boundary_clearance = min(
        _minimum_path_to_boundary_distance(clearance_path, left),
        _minimum_path_to_boundary_distance(clearance_path, right),
    )
    if boundary_clearance + 1.0e-6 < required_center_clearance:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="PATH_OUTSIDE_CORRIDOR",
            path_length_m=path_length,
            smoothing_applied=smoothing_applied,
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )
    # Any unused observed cone is still a hard physical obstacle. This also
    # vetoes a virtual boundary that contradicts a real narrowed row.
    cone_clearance = min(
        _distance_to_polyline(cone, path) for cone in cones
    )
    required_cone_clearance = (
        config.vehicle_width_m * 0.5
        + config.safety_margin_m
        + config.cone_obstacle_radius_m
    )
    if cone_clearance + 1.0e-6 < required_cone_clearance:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="CONE_OBSTACLE_ON_PATH",
            path_length_m=path_length,
            smoothing_applied=smoothing_applied,
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )
    if path_length < config.min_path_length_m:
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="PATH_TOO_SHORT",
            path_length_m=path_length,
            smoothing_applied=smoothing_applied,
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )

    max_curvature = _maximum_path_curvature(path, config.curvature_sample_distance_m)
    if (
        config.max_path_curvature_1pm > 0.0
        and max_curvature > config.max_path_curvature_1pm
    ):
        return PlanResult(
            left,
            right,
            centers,
            _empty_points(),
            status="CURVATURE_LIMIT",
            path_length_m=path_length,
            max_curvature_1pm=max_curvature,
            smoothing_applied=smoothing_applied,
            candidate_count=len(cones),
            real_pair_count=real_pair_count,
            virtual_pair_count=virtual_pair_count,
        )

    # Synthetic pairs must never make width/alignment quality look better than
    # the real observations that justified them.
    real_widths = np.asarray(widths[:real_pair_count], dtype=float)
    real_along_errors = np.asarray(
        along_errors[:real_pair_count], dtype=float
    )
    width_rmse = float(
        np.sqrt(np.mean((real_widths - config.track_width_m) ** 2))
    )
    along_rmse = float(np.sqrt(np.mean(real_along_errors**2)))
    effective_support = (
        real_pair_count
        + config.virtual_cone_confidence_weight * virtual_pair_count
    )
    support_quality = min(1.0, effective_support / config.confidence_full_pairs)
    width_quality = exp(-width_rmse / max(0.05, 0.2 * config.track_width_m))
    alignment_quality = exp(-along_rmse / max(0.04, config.pair_max_along_error_m))
    real_steps = np.asarray(steps[:real_pair_count], dtype=float)
    if real_pair_count > 1:
        station_steps = real_steps[1:]
        station_multiple = np.maximum(
            1.0,
            np.rint(station_steps / config.expected_cone_spacing_m),
        )
        observation_density = float(
            np.clip(
                len(station_steps) / np.sum(station_multiple),
                0.0,
                1.0,
            )
        )
    else:
        observation_density = 0.0
    length_quality = min(1.0, path_length / config.min_path_length_m)
    confidence = float(
        np.clip(
            support_quality
            * width_quality
            * alignment_quality
            * observation_density
            * length_quality
            * config.virtual_cone_confidence_weight**virtual_pair_count,
            0.0,
            1.0,
        )
    )
    if confidence < config.min_plan_confidence:
        status = "LOW_CONFIDENCE"
    else:
        status = "OK_VIRTUAL" if virtual_pair_count else "OK"
    return PlanResult(
        left,
        right,
        centers,
        path if status in {"OK", "OK_VIRTUAL"} else _empty_points(),
        status=status,
        confidence=confidence,
        path_length_m=path_length,
        max_curvature_1pm=max_curvature,
        smoothing_applied=smoothing_applied,
        candidate_count=len(cones),
        real_pair_count=real_pair_count,
        virtual_pair_count=virtual_pair_count,
    )
