"""Deterministic, ROS-independent 2D LiDAR and vehicle simulation.

The simulator uses the same frame convention as ``LaserScan`` and the cone
planner: x is forward, y is left, and positive yaw is counter-clockwise.  All
world geometry is expressed in metres.  A scan is returned in the sensor
frame, while ``sensor_pose_in_vehicle`` describes the static sensor
extrinsic.  Invalid/no-return beams are represented by ``numpy.inf``.

Only NumPy and the Python standard library are used so the module can run in
unit tests without a ROS installation.
"""

from dataclasses import dataclass, field
from math import atan2, cos, pi, sin, tan
from typing import Iterable, Sequence

import numpy as np


_EPSILON = 1.0e-12


def _finite_scalar(name: str, value: float) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _point2(name: str, value: Sequence[float]) -> tuple[float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (2,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain two finite values")
    return float(array[0]), float(array[1])


def wrap_yaw(yaw_rad: float) -> float:
    """Wrap a finite angle to ``[-pi, pi)``."""

    yaw = _finite_scalar("yaw_rad", yaw_rad)
    return float((yaw + pi) % (2.0 * pi) - pi)


@dataclass(frozen=True)
class Pose2D:
    """Rigid 2D pose, using x-forward/y-left/CCW-yaw convention."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_scalar("pose.x", self.x))
        object.__setattr__(self, "y", _finite_scalar("pose.y", self.y))
        object.__setattr__(self, "yaw", wrap_yaw(self.yaw))


def compose_pose(parent: Pose2D, child: Pose2D) -> Pose2D:
    """Return the world pose of ``child`` expressed relative to ``parent``."""

    cosine = cos(parent.yaw)
    sine = sin(parent.yaw)
    return Pose2D(
        parent.x + cosine * child.x - sine * child.y,
        parent.y + sine * child.x + cosine * child.y,
        parent.yaw + child.yaw,
    )


def transform_points(points: Sequence[Sequence[float]], pose: Pose2D) -> np.ndarray:
    """Transform points from a local frame into the parent/world frame."""

    array = np.asarray(points, dtype=float)
    if array.size == 0:
        return np.empty((0, 2), dtype=float)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if not np.all(np.isfinite(array)):
        raise ValueError("points must be finite")
    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)
    rotation = np.array(((cosine, -sine), (sine, cosine)), dtype=float)
    return array @ rotation.T + np.array((pose.x, pose.y), dtype=float)


def inverse_transform_points(
    points: Sequence[Sequence[float]], pose: Pose2D
) -> np.ndarray:
    """Transform parent/world points into the local frame of ``pose``."""

    array = np.asarray(points, dtype=float)
    if array.size == 0:
        return np.empty((0, 2), dtype=float)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if not np.all(np.isfinite(array)):
        raise ValueError("points must be finite")
    translated = array - np.array((pose.x, pose.y), dtype=float)
    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)
    inverse_rotation = np.array(((cosine, sine), (-sine, cosine)), dtype=float)
    return translated @ inverse_rotation.T


@dataclass(frozen=True)
class CircleObstacle:
    """Circular obstacle; cones are represented by their LiDAR-height slice."""

    center: tuple[float, float]
    radius_m: float = 0.025
    label: str = "circle"

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _point2("circle.center", self.center))
        radius = _finite_scalar("circle.radius_m", self.radius_m)
        if radius <= 0.0:
            raise ValueError("circle.radius_m must be > 0")
        object.__setattr__(self, "radius_m", radius)


@dataclass(frozen=True)
class SegmentObstacle:
    """Infinitely thin closed line-segment obstacle."""

    start: tuple[float, float]
    end: tuple[float, float]
    label: str = "segment"

    def __post_init__(self) -> None:
        start = _point2("segment.start", self.start)
        end = _point2("segment.end", self.end)
        if np.linalg.norm(np.subtract(end, start)) <= _EPSILON:
            raise ValueError("segment endpoints must be distinct")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)


@dataclass(frozen=True)
class CapsuleObstacle:
    """Line segment swept by a circle of ``radius_m``."""

    start: tuple[float, float]
    end: tuple[float, float]
    radius_m: float
    label: str = "capsule"

    def __post_init__(self) -> None:
        start = _point2("capsule.start", self.start)
        end = _point2("capsule.end", self.end)
        if np.linalg.norm(np.subtract(end, start)) <= _EPSILON:
            raise ValueError("capsule endpoints must be distinct")
        radius = _finite_scalar("capsule.radius_m", self.radius_m)
        if radius <= 0.0:
            raise ValueError("capsule.radius_m must be > 0")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)
        object.__setattr__(self, "radius_m", radius)


Obstacle = CircleObstacle | SegmentObstacle | CapsuleObstacle


@dataclass(frozen=True)
class WorldConfig:
    """LiDAR geometry and deterministic sensor-fault configuration.

    Defaults intentionally match the planner's 0.15--5.0 m accepted range.
    The random stream for a frame is derived from ``(random_seed,
    scan_index)``.  Repeating a frame therefore produces bit-identical output
    regardless of call order.
    """

    angle_min_rad: float = -pi
    angle_max_rad: float = pi
    beam_count: int = 1081
    range_min_m: float = 0.15
    range_max_m: float = 5.0
    sensor_pose_in_vehicle: Pose2D = field(default_factory=Pose2D)
    random_seed: int = 7
    gaussian_noise_std_m: float = 0.0
    range_quantization_m: float = 0.0
    beam_dropout_probability: float = 0.0
    same_range_replica_probability: float = 0.0
    same_range_replica_span_beams: int = 0
    glint_probability: float = 0.0
    glint_range_min_m: float = 0.15
    glint_range_max_m: float = 2.0

    def __post_init__(self) -> None:
        self.validate()

    @property
    def angle_increment_rad(self) -> float:
        return (self.angle_max_rad - self.angle_min_rad) / (self.beam_count - 1)

    def validate(self) -> None:
        angle_min = _finite_scalar("angle_min_rad", self.angle_min_rad)
        angle_max = _finite_scalar("angle_max_rad", self.angle_max_rad)
        range_min = _finite_scalar("range_min_m", self.range_min_m)
        range_max = _finite_scalar("range_max_m", self.range_max_m)
        if angle_min >= angle_max:
            raise ValueError("angle_min_rad must be smaller than angle_max_rad")
        if angle_max - angle_min > 2.0 * pi + 1.0e-9:
            raise ValueError("LiDAR angular span cannot exceed 2*pi")
        if isinstance(self.beam_count, (bool, np.bool_)):
            raise ValueError("beam_count must be an integer >= 2")
        if int(self.beam_count) != self.beam_count or self.beam_count < 2:
            raise ValueError("beam_count must be an integer >= 2")
        if range_min <= 0.0 or range_min >= range_max:
            raise ValueError("range limits must satisfy 0 < min < max")
        if not isinstance(self.sensor_pose_in_vehicle, Pose2D):
            raise ValueError("sensor_pose_in_vehicle must be Pose2D")
        if isinstance(self.random_seed, (bool, np.bool_)):
            raise ValueError("random_seed must be a nonnegative integer")
        if int(self.random_seed) != self.random_seed or self.random_seed < 0:
            raise ValueError("random_seed must be a nonnegative integer")

        nonnegative = {
            "gaussian_noise_std_m": self.gaussian_noise_std_m,
            "range_quantization_m": self.range_quantization_m,
        }
        for name, value in nonnegative.items():
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
        for name in (
            "beam_dropout_probability",
            "same_range_replica_probability",
            "glint_probability",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        span = self.same_range_replica_span_beams
        if isinstance(span, (bool, np.bool_)) or int(span) != span or span < 0:
            raise ValueError("same_range_replica_span_beams must be an integer >= 0")
        if span >= self.beam_count:
            raise ValueError("same_range_replica_span_beams must be < beam_count")
        if self.same_range_replica_probability > 0.0 and span == 0:
            raise ValueError("replica span must be positive when replica fault is enabled")
        glint_min = _finite_scalar("glint_range_min_m", self.glint_range_min_m)
        glint_max = _finite_scalar("glint_range_max_m", self.glint_range_max_m)
        if not range_min <= glint_min < glint_max <= range_max:
            raise ValueError("glint range must lie inside the LiDAR range limits")


@dataclass(frozen=True)
class ScanFrame:
    """ROS-neutral equivalent of the geometric parts of ``LaserScan``."""

    ranges: np.ndarray
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    sensor_world_pose: Pose2D
    scan_index: int

    def __post_init__(self) -> None:
        ranges = np.asarray(self.ranges, dtype=float).copy()
        if ranges.ndim != 1 or len(ranges) < 2:
            raise ValueError("ranges must be a one-dimensional array with >= 2 beams")
        if np.any(np.isnan(ranges)) or np.any(np.isneginf(ranges)):
            raise ValueError("ranges may contain finite values or positive infinity")
        ranges.setflags(write=False)
        object.__setattr__(self, "ranges", ranges)

    @property
    def angle_max_rad(self) -> float:
        return self.angle_min_rad + (len(self.ranges) - 1) * self.angle_increment_rad

    @property
    def angles_rad(self) -> np.ndarray:
        return self.angle_min_rad + np.arange(len(self.ranges)) * self.angle_increment_rad


def _cross_2d(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    return first[..., 0] * second[..., 1] - first[..., 1] * second[..., 0]


def _ray_circle_distances(
    origin: np.ndarray,
    directions: np.ndarray,
    center: Sequence[float],
    radius_m: float,
) -> np.ndarray:
    relative = origin - np.asarray(center, dtype=float)
    projection = directions @ relative
    constant = float(relative @ relative - radius_m * radius_m)
    discriminant = projection * projection - constant
    result = np.full(len(directions), np.inf, dtype=float)
    visible = discriminant >= 0.0
    if not np.any(visible):
        return result
    root = np.sqrt(np.maximum(discriminant[visible], 0.0))
    near = -projection[visible] - root
    far = -projection[visible] + root
    distance = np.where(near >= 0.0, near, np.where(far >= 0.0, far, np.inf))
    result[visible] = distance
    if float(relative @ relative) < radius_m * radius_m:
        result.fill(0.0)
    return result


def _ray_circle_distances_batch(
    origin: np.ndarray,
    directions: np.ndarray,
    centers: np.ndarray,
    radii_m: np.ndarray,
) -> np.ndarray:
    """Return the nearest circle hit for every ray in one NumPy batch.

    The scalar helper above remains the reference used by capsule end caps.
    Grouping circles here avoids repeating Python dispatch and allocating a
    full beam result for every cone in a course.
    """

    if len(centers) == 0:
        return np.full(len(directions), np.inf, dtype=float)

    relative = origin[None, :] - centers
    projection = directions @ relative.T
    constant = np.sum(relative * relative, axis=1) - radii_m * radii_m
    discriminant = projection * projection - constant[None, :]
    visible = discriminant >= 0.0
    root = np.sqrt(np.maximum(discriminant, 0.0))
    near = -projection - root
    far = -projection + root
    distances = np.where(
        visible & (near >= 0.0),
        near,
        np.where(visible & (far >= 0.0), far, np.inf),
    )

    # Preserve _ray_circle_distances' convention: a sensor strictly inside a
    # circle reports contact at zero for every ray, rather than the exit hit.
    inside = constant < 0.0
    if np.any(inside):
        distances[:, inside] = 0.0
    return np.min(distances, axis=1)


def _ray_segment_distances(
    origin: np.ndarray,
    directions: np.ndarray,
    start: Sequence[float],
    end: Sequence[float],
) -> np.ndarray:
    segment_start = np.asarray(start, dtype=float)
    segment = np.asarray(end, dtype=float) - segment_start
    difference = segment_start - origin
    denominator = _cross_2d(directions, segment)
    result = np.full(len(directions), np.inf, dtype=float)
    nonparallel = np.abs(denominator) > _EPSILON
    if np.any(nonparallel):
        distance = _cross_2d(difference, segment) / denominator[nonparallel]
        fraction = (
            _cross_2d(difference, directions[nonparallel])
            / denominator[nonparallel]
        )
        hit = (distance >= 0.0) & (fraction >= 0.0) & (fraction <= 1.0)
        indexes = np.flatnonzero(nonparallel)
        result[indexes[hit]] = distance[hit]

    # A ray exactly collinear with a zero-width segment still sees the nearest
    # point of that segment.  Treating it as no return would create a narrow
    # but deterministic hole in a wall aligned with a LiDAR beam.
    parallel_indexes = np.flatnonzero(~nonparallel)
    if len(parallel_indexes):
        parallel_directions = directions[parallel_indexes]
        collinear = np.abs(
            _cross_2d(difference, parallel_directions)
        ) <= _EPSILON
        if np.any(collinear):
            indexes = parallel_indexes[collinear]
            directions_subset = directions[indexes]
            start_projection = directions_subset @ difference
            end_projection = directions_subset @ (
                np.asarray(end, dtype=float) - origin
            )
            near = np.minimum(start_projection, end_projection)
            far = np.maximum(start_projection, end_projection)
            distance = np.where(near >= 0.0, near, np.where(far >= 0.0, 0.0, np.inf))
            result[indexes] = distance
    return result


def _point_segment_distance(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    point_array = np.asarray(point, dtype=float)
    start_array = np.asarray(start, dtype=float)
    segment = np.asarray(end, dtype=float) - start_array
    length_squared = float(segment @ segment)
    fraction = float(np.dot(point_array - start_array, segment) / length_squared)
    fraction = float(np.clip(fraction, 0.0, 1.0))
    return float(np.linalg.norm(point_array - (start_array + fraction * segment)))


def _ray_capsule_distances(
    origin: np.ndarray,
    directions: np.ndarray,
    start: Sequence[float],
    end: Sequence[float],
    radius_m: float,
) -> np.ndarray:
    start_array = np.asarray(start, dtype=float)
    end_array = np.asarray(end, dtype=float)
    segment = end_array - start_array
    length = float(np.linalg.norm(segment))
    tangent = segment / length
    normal = np.array((-tangent[1], tangent[0]), dtype=float)
    local_origin = origin - start_array
    along_origin = float(local_origin @ tangent)
    normal_origin = float(local_origin @ normal)
    along_direction = directions @ tangent
    normal_direction = directions @ normal

    result = np.minimum(
        _ray_circle_distances(origin, directions, start_array, radius_m),
        _ray_circle_distances(origin, directions, end_array, radius_m),
    )
    for side in (-radius_m, radius_m):
        usable = np.abs(normal_direction) > _EPSILON
        distance = np.full(len(directions), np.inf, dtype=float)
        distance[usable] = (side - normal_origin) / normal_direction[usable]
        along = along_origin + distance * along_direction
        hit = usable & (distance >= 0.0) & (along >= 0.0) & (along <= length)
        result[hit] = np.minimum(result[hit], distance[hit])
    if _point_segment_distance(origin, start_array, end_array) < radius_m:
        result.fill(0.0)
    return result


class SyntheticWorld:
    """Static 2D world capable of producing deterministic synthetic scans."""

    def __init__(
        self,
        obstacles: Iterable[Obstacle],
        config: WorldConfig | None = None,
    ) -> None:
        self.config = config if config is not None else WorldConfig()
        self.config.validate()
        self.obstacles = tuple(obstacles)
        for obstacle in self.obstacles:
            if not isinstance(
                obstacle, (CircleObstacle, SegmentObstacle, CapsuleObstacle)
            ):
                raise TypeError("unsupported obstacle type")
        circle_obstacles = tuple(
            obstacle
            for obstacle in self.obstacles
            if isinstance(obstacle, CircleObstacle)
        )
        self._circle_centers = np.asarray(
            [obstacle.center for obstacle in circle_obstacles], dtype=float
        ).reshape((-1, 2))
        self._circle_radii_m = np.asarray(
            [obstacle.radius_m for obstacle in circle_obstacles], dtype=float
        )
        self._noncircle_obstacles = tuple(
            obstacle
            for obstacle in self.obstacles
            if not isinstance(obstacle, CircleObstacle)
        )
        self._circle_centers.setflags(write=False)
        self._circle_radii_m.setflags(write=False)

    def _ideal_ranges(self, sensor_pose: Pose2D) -> np.ndarray:
        angles = (
            self.config.angle_min_rad
            + np.arange(self.config.beam_count) * self.config.angle_increment_rad
            + sensor_pose.yaw
        )
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        origin = np.array((sensor_pose.x, sensor_pose.y), dtype=float)
        ranges = _ray_circle_distances_batch(
            origin,
            directions,
            self._circle_centers,
            self._circle_radii_m,
        )
        for obstacle in self._noncircle_obstacles:
            if isinstance(obstacle, SegmentObstacle):
                candidate = _ray_segment_distances(
                    origin, directions, obstacle.start, obstacle.end
                )
            else:
                candidate = _ray_capsule_distances(
                    origin,
                    directions,
                    obstacle.start,
                    obstacle.end,
                    obstacle.radius_m,
                )
            ranges = np.minimum(ranges, candidate)
        outside = (
            ~np.isfinite(ranges)
            | (ranges < self.config.range_min_m)
            | (ranges > self.config.range_max_m)
        )
        ranges[outside] = np.inf
        return ranges

    def _apply_faults(self, ideal_ranges: np.ndarray, scan_index: int) -> np.ndarray:
        seed = np.random.SeedSequence((int(self.config.random_seed), scan_index))
        generator = np.random.default_rng(seed)
        ranges = ideal_ranges.copy()
        finite = np.isfinite(ranges)

        if self.config.gaussian_noise_std_m > 0.0 and np.any(finite):
            ranges[finite] += generator.normal(
                0.0,
                self.config.gaussian_noise_std_m,
                int(np.count_nonzero(finite)),
            )
        if self.config.range_quantization_m > 0.0 and np.any(finite):
            quantum = self.config.range_quantization_m
            ranges[finite] = np.round(ranges[finite] / quantum) * quantum

        if self.config.glint_probability > 0.0:
            glint_mask = (
                generator.random(len(ranges)) < self.config.glint_probability
            )
            glint_values = generator.uniform(
                self.config.glint_range_min_m,
                self.config.glint_range_max_m,
                len(ranges),
            )
            ranges[glint_mask] = np.minimum(ranges[glint_mask], glint_values[glint_mask])

        span = self.config.same_range_replica_span_beams
        if self.config.same_range_replica_probability > 0.0 and span > 0:
            source_ranges = ranges.copy()
            sources = np.flatnonzero(
                np.isfinite(source_ranges)
                & (
                    generator.random(len(ranges))
                    < self.config.same_range_replica_probability
                )
            )
            # Copy from the frozen pre-replica array so faults cannot cascade.
            # Ascending source order gives deterministic overlap resolution.
            for source in sources:
                first = max(0, source - span)
                last = min(len(ranges), source + span + 1)
                ranges[first:last] = source_ranges[source]

        if self.config.beam_dropout_probability > 0.0:
            dropout = (
                generator.random(len(ranges))
                < self.config.beam_dropout_probability
            )
            ranges[dropout] = np.inf

        outside = (
            ~np.isfinite(ranges)
            | (ranges < self.config.range_min_m)
            | (ranges > self.config.range_max_m)
        )
        ranges[outside] = np.inf
        return ranges

    def scan(self, vehicle_pose: Pose2D, scan_index: int = 0) -> ScanFrame:
        """Ray-cast one scan at ``vehicle_pose``.

        ``scan_index`` must be explicitly monotonic in a live simulation, but
        may be repeated by tests to reproduce a frame exactly.
        """

        if not isinstance(vehicle_pose, Pose2D):
            raise TypeError("vehicle_pose must be Pose2D")
        if isinstance(scan_index, (bool, np.bool_)):
            raise ValueError("scan_index must be a nonnegative integer")
        if int(scan_index) != scan_index or scan_index < 0:
            raise ValueError("scan_index must be a nonnegative integer")
        sensor_pose = compose_pose(
            vehicle_pose, self.config.sensor_pose_in_vehicle
        )
        ranges = self._apply_faults(
            self._ideal_ranges(sensor_pose), int(scan_index)
        )
        return ScanFrame(
            ranges=ranges,
            angle_min_rad=self.config.angle_min_rad,
            angle_increment_rad=self.config.angle_increment_rad,
            range_min_m=self.config.range_min_m,
            range_max_m=self.config.range_max_m,
            sensor_world_pose=sensor_pose,
            scan_index=int(scan_index),
        )


@dataclass(frozen=True)
class VehicleState:
    """Ground-truth planar bicycle state."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed_mps: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_scalar("state.x", self.x))
        object.__setattr__(self, "y", _finite_scalar("state.y", self.y))
        object.__setattr__(self, "yaw", wrap_yaw(self.yaw))
        object.__setattr__(
            self, "speed_mps", _finite_scalar("state.speed_mps", self.speed_mps)
        )

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw)


def bicycle_yaw_rate(
    speed_mps: float, steering_angle_rad: float, wheelbase_m: float
) -> float:
    """Return ``v*tan(delta)/wheelbase`` after singularity-safe validation."""

    speed = _finite_scalar("speed_mps", speed_mps)
    steering = _finite_scalar("steering_angle_rad", steering_angle_rad)
    wheelbase = _finite_scalar("wheelbase_m", wheelbase_m)
    if wheelbase <= 0.0:
        raise ValueError("wheelbase_m must be > 0")
    if abs(steering) >= 0.5 * pi:
        raise ValueError("abs(steering_angle_rad) must be < pi/2")
    return float(speed * tan(steering) / wheelbase)


def step_bicycle(
    state: VehicleState,
    speed_mps: float,
    steering_angle_rad: float,
    dt_s: float,
    wheelbase_m: float,
) -> VehicleState:
    """Integrate a constant-speed kinematic bicycle exactly for one step."""

    if not isinstance(state, VehicleState):
        raise TypeError("state must be VehicleState")
    speed = _finite_scalar("speed_mps", speed_mps)
    steering = _finite_scalar("steering_angle_rad", steering_angle_rad)
    dt = _finite_scalar("dt_s", dt_s)
    if dt < 0.0:
        raise ValueError("dt_s must be >= 0")
    yaw_rate = bicycle_yaw_rate(speed, steering, wheelbase_m)
    yaw_change = yaw_rate * dt
    if abs(yaw_rate) <= _EPSILON:
        x = state.x + speed * dt * cos(state.yaw)
        y = state.y + speed * dt * sin(state.yaw)
    else:
        turn_radius = speed / yaw_rate
        x = state.x + turn_radius * (
            sin(state.yaw + yaw_change) - sin(state.yaw)
        )
        y = state.y - turn_radius * (
            cos(state.yaw + yaw_change) - cos(state.yaw)
        )
    return VehicleState(x, y, state.yaw + yaw_change, speed)


@dataclass
class ConeCourse:
    """Ground-truth centerline and left/right circular cone rows."""

    centerline: np.ndarray
    left_cones: np.ndarray
    right_cones: np.ndarray
    track_width_m: float
    cone_radius_m: float
    name: str

    def __post_init__(self) -> None:
        for field_name in ("centerline", "left_cones", "right_cones"):
            array = np.asarray(getattr(self, field_name), dtype=float).copy()
            if array.ndim != 2 or array.shape[1] != 2 or len(array) < 2:
                raise ValueError(f"{field_name} must have shape (N, 2), N >= 2")
            if not np.all(np.isfinite(array)):
                raise ValueError(f"{field_name} must be finite")
            setattr(self, field_name, array)
        if len(self.left_cones) != len(self.right_cones):
            raise ValueError("left_cones and right_cones must have equal length")
        if not np.isfinite(self.track_width_m) or self.track_width_m <= 0.0:
            raise ValueError("track_width_m must be finite and > 0")
        if not np.isfinite(self.cone_radius_m) or self.cone_radius_m <= 0.0:
            raise ValueError("cone_radius_m must be finite and > 0")

    @property
    def all_cones(self) -> np.ndarray:
        return np.vstack((self.left_cones, self.right_cones))

    def obstacles(self) -> tuple[CircleObstacle, ...]:
        left = (
            CircleObstacle(tuple(point), self.cone_radius_m, "left_cone")
            for point in self.left_cones
        )
        right = (
            CircleObstacle(tuple(point), self.cone_radius_m, "right_cone")
            for point in self.right_cones
        )
        return tuple((*left, *right))


def _polyline_arc_length(points: np.ndarray) -> np.ndarray:
    return np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )


def _interpolate_polyline(points: np.ndarray, distances: np.ndarray) -> np.ndarray:
    arc = _polyline_arc_length(points)
    return np.column_stack(
        (
            np.interp(distances, arc, points[:, 0]),
            np.interp(distances, arc, points[:, 1]),
        )
    )


def _validate_course_parameters(
    length_m: float,
    track_width_m: float,
    cone_spacing_m: float,
    cone_radius_m: float,
    first_cone_distance_m: float,
) -> None:
    values = {
        "length_m": length_m,
        "track_width_m": track_width_m,
        "cone_spacing_m": cone_spacing_m,
        "cone_radius_m": cone_radius_m,
    }
    for name, value in values.items():
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
    if not np.isfinite(first_cone_distance_m) or first_cone_distance_m < 0.0:
        raise ValueError("first_cone_distance_m must be finite and >= 0")
    if first_cone_distance_m >= length_m:
        raise ValueError("first_cone_distance_m must be smaller than length_m")


def _course_from_centerline(
    centerline: np.ndarray,
    track_width_m: float,
    cone_spacing_m: float,
    cone_radius_m: float,
    first_cone_distance_m: float,
    name: str,
) -> ConeCourse:
    total_length = float(_polyline_arc_length(centerline)[-1])
    stations = np.arange(
        first_cone_distance_m,
        total_length + 1.0e-9,
        cone_spacing_m,
        dtype=float,
    )
    if len(stations) < 2:
        raise ValueError("course must contain at least two cone stations")
    cone_centers = _interpolate_polyline(centerline, stations)
    tangent = np.gradient(cone_centers, axis=0)
    tangent_norm = np.linalg.norm(tangent, axis=1)
    if np.any(tangent_norm <= _EPSILON):
        raise ValueError("centerline contains a degenerate tangent")
    tangent /= tangent_norm[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    half_width = 0.5 * track_width_m
    return ConeCourse(
        centerline=centerline,
        left_cones=cone_centers + half_width * normal,
        right_cones=cone_centers - half_width * normal,
        track_width_m=track_width_m,
        cone_radius_m=cone_radius_m,
        name=name,
    )


def make_straight_course(
    length_m: float = 4.5,
    track_width_m: float = 0.60,
    cone_spacing_m: float = 0.297,
    cone_radius_m: float = 0.025,
    first_cone_distance_m: float = 0.30,
) -> ConeCourse:
    """Create a straight course beginning at the vehicle origin."""

    _validate_course_parameters(
        length_m,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
    )
    count = max(2, int(np.ceil(length_m / 0.02)) + 1)
    x = np.linspace(0.0, length_m, count)
    centerline = np.column_stack((x, np.zeros_like(x)))
    return _course_from_centerline(
        centerline,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
        "straight",
    )


def make_arc_course(
    length_m: float = 4.5,
    radius_m: float = 3.0,
    turn_left: bool = True,
    track_width_m: float = 0.60,
    cone_spacing_m: float = 0.297,
    cone_radius_m: float = 0.025,
    first_cone_distance_m: float = 0.30,
) -> ConeCourse:
    """Create a constant-curvature course tangent to +x at the origin."""

    _validate_course_parameters(
        length_m,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
    )
    if not np.isfinite(radius_m) or radius_m <= 0.5 * track_width_m:
        raise ValueError("radius_m must exceed half the track width")
    if not isinstance(turn_left, (bool, np.bool_)):
        raise ValueError("turn_left must be boolean")
    count = max(2, int(np.ceil(length_m / 0.02)) + 1)
    arc = np.linspace(0.0, length_m, count)
    direction = 1.0 if turn_left else -1.0
    heading = direction * arc / radius_m
    centerline = np.column_stack(
        (
            radius_m * np.sin(arc / radius_m),
            direction * radius_m * (1.0 - np.cos(arc / radius_m)),
        )
    )
    # ``heading`` is intentionally evaluated to document/check the tangent
    # convention without introducing numerical orientation ambiguity.
    if not np.all(np.isfinite(heading)):
        raise ValueError("arc heading is not finite")
    return _course_from_centerline(
        centerline,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
        "left_arc" if turn_left else "right_arc",
    )


def make_s_course(
    length_m: float = 4.5,
    amplitude_m: float = 0.35,
    track_width_m: float = 0.60,
    cone_spacing_m: float = 0.297,
    cone_radius_m: float = 0.025,
    first_cone_distance_m: float = 0.30,
) -> ConeCourse:
    """Create a smooth signed S-bend with zero end slope."""

    _validate_course_parameters(
        length_m,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
    )
    if not np.isfinite(amplitude_m) or amplitude_m <= 0.0:
        raise ValueError("amplitude_m must be finite and > 0")
    count = max(2, int(np.ceil(length_m / 0.02)) + 1)
    x = np.linspace(0.0, length_m, count)
    phase = x / length_m
    y = (
        amplitude_m
        * np.sin(2.0 * pi * phase)
        * np.sin(pi * phase) ** 2
    )
    centerline = np.column_stack((x, y))
    return _course_from_centerline(
        centerline,
        track_width_m,
        cone_spacing_m,
        cone_radius_m,
        first_cone_distance_m,
        "s_bend",
    )


@dataclass(frozen=True)
class PolylineProjection:
    """Nearest-point ground truth on a directed centerline."""

    nearest_point: tuple[float, float]
    signed_lateral_m: float
    absolute_lateral_m: float
    along_track_m: float
    segment_index: int


def project_to_polyline(
    point: Sequence[float], polyline: Sequence[Sequence[float]]
) -> PolylineProjection:
    """Project a point onto a directed polyline; left error is positive."""

    query = np.asarray(_point2("point", point), dtype=float)
    line = np.asarray(polyline, dtype=float)
    if line.ndim != 2 or line.shape[1] != 2 or len(line) < 2:
        raise ValueError("polyline must have shape (N, 2), N >= 2")
    if not np.all(np.isfinite(line)):
        raise ValueError("polyline must be finite")
    segments = np.diff(line, axis=0)
    lengths_squared = np.sum(segments * segments, axis=1)
    valid = lengths_squared > _EPSILON
    if not np.any(valid):
        raise ValueError("polyline must contain a nonzero segment")
    fraction = np.zeros(len(segments), dtype=float)
    fraction[valid] = (
        np.sum((query - line[:-1][valid]) * segments[valid], axis=1)
        / lengths_squared[valid]
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    nearest = line[:-1] + fraction[:, None] * segments
    distance_squared = np.sum((query - nearest) ** 2, axis=1)
    distance_squared[~valid] = np.inf
    index = int(np.argmin(distance_squared))
    offset = query - nearest[index]
    cross = float(_cross_2d(segments[index], offset))
    signed = float(np.sign(cross) * np.sqrt(distance_squared[index]))
    arc = _polyline_arc_length(line)
    along = float(
        arc[index] + fraction[index] * np.sqrt(lengths_squared[index])
    )
    return PolylineProjection(
        nearest_point=(float(nearest[index, 0]), float(nearest[index, 1])),
        signed_lateral_m=signed,
        absolute_lateral_m=abs(signed),
        along_track_m=along,
        segment_index=index,
    )


def lateral_error_m(
    point: Sequence[float], centerline: Sequence[Sequence[float]]
) -> float:
    """Convenience signed lateral-error metric (left is positive)."""

    return project_to_polyline(point, centerline).signed_lateral_m


def obstacle_clearance_m(
    point: Sequence[float],
    obstacles: Iterable[Obstacle],
    footprint_radius_m: float = 0.0,
) -> float:
    """Minimum signed clearance from a circular vehicle footprint.

    A positive result is free space, zero is contact, and a negative result is
    penetration.  With no obstacles the result is positive infinity.
    """

    query = np.asarray(_point2("point", point), dtype=float)
    footprint = _finite_scalar("footprint_radius_m", footprint_radius_m)
    if footprint < 0.0:
        raise ValueError("footprint_radius_m must be >= 0")
    minimum = np.inf
    for obstacle in obstacles:
        if isinstance(obstacle, CircleObstacle):
            distance = (
                float(np.linalg.norm(query - np.asarray(obstacle.center)))
                - obstacle.radius_m
            )
        elif isinstance(obstacle, SegmentObstacle):
            distance = _point_segment_distance(
                query, obstacle.start, obstacle.end
            )
        elif isinstance(obstacle, CapsuleObstacle):
            distance = (
                _point_segment_distance(query, obstacle.start, obstacle.end)
                - obstacle.radius_m
            )
        else:
            raise TypeError("unsupported obstacle type")
        minimum = min(minimum, distance - footprint)
    return float(minimum)


def path_minimum_clearance_m(
    path: Sequence[Sequence[float]],
    obstacles: Iterable[Obstacle],
    footprint_radius_m: float = 0.0,
) -> float:
    """Minimum sampled ground-truth clearance along a path."""

    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("path must have shape (N, 2), N >= 1")
    if not np.all(np.isfinite(points)):
        raise ValueError("path must be finite")
    obstacle_tuple = tuple(obstacles)
    return min(
        obstacle_clearance_m(point, obstacle_tuple, footprint_radius_m)
        for point in points
    )
