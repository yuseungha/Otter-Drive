"""Deterministic, ROS-independent closed-loop validation for cone driving.

The simulator deliberately exercises the production algorithm chain:

``ray/circle LaserScan -> cone detection -> temporal tracking -> planning
-> Pure Pursuit -> bicycle model``

It is intentionally small enough for normal CI.  It is not a replacement for
hardware validation: the circular cone cross-section is only a deterministic
geometric model of the return seen at the LiDAR scan height.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
from math import cos, isfinite, pi, sin, tan
from typing import Sequence

import numpy as np

from .planner_core import (
    ConeTrackFilter,
    PlannerConfig,
    detect_cones_from_scan,
    extract_obstacle_points_from_scan,
    plan_centerline,
)
from .pure_pursuit_core import (
    ControllerConfig,
    ControlResult,
    compute_pure_pursuit,
    stop_result,
)


# A road cone tapers substantially above its base.  A 15 mm radius is a
# plausible circular cross-section at a low 2D LiDAR scan height.
CONE_CROSS_SECTION_RADIUS_M = 0.015


@dataclass(frozen=True)
class SyntheticCourse:
    """World-frame centreline and the two rows of cone centres."""

    name: str
    reference_centerline: np.ndarray
    reference_arc_m: np.ndarray
    cone_centers: np.ndarray


@dataclass
class BicycleState:
    """Rear-axle-centre state used by the kinematic bicycle model."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    speed_mps: float = 0.0
    steering_angle_rad: float = 0.0


@dataclass(frozen=True)
class SyntheticValidationResult:
    """Compact safety and tracking metrics for one deterministic run."""

    scenario: str
    steps: int
    dt_s: float
    simulated_duration_s: float
    progress_m: float
    required_progress_m: float
    max_lateral_error_m: float
    p95_lateral_error_m: float
    min_clearance_m: float
    valid_plan_fraction: float
    mean_plan_confidence: float
    max_plan_confidence: float
    mean_confirmed_cones: float
    mean_real_pairs: float
    collisions: int
    positive_commands_after_fault: int
    post_fault_travel_m: float
    fault_step: int | None
    completed: bool
    max_speed_mps: float
    max_abs_steering_rad: float
    final_x_m: float
    final_y_m: float
    final_yaw_rad: float
    status_counts: dict[str, int]


def make_course(name: str, *, station_count: int = 28) -> SyntheticCourse:
    """Create a measured-width cone-centre corridor accepted by the planner."""

    if station_count < 8:
        raise ValueError("station_count must be at least 8")
    if name not in {"straight", "left_arc"}:
        raise ValueError("scenario must be 'straight' or 'left_arc'")

    spacing_m = PlannerConfig().expected_cone_spacing_m
    station_arc = 0.45 + spacing_m * np.arange(station_count, dtype=float)
    end_arc = float(station_arc[-1] + 0.60)
    dense_arc = np.linspace(0.0, end_arc, int(end_arc / 0.02) + 1)

    if name == "straight":
        reference = np.column_stack((dense_arc, np.zeros_like(dense_arc)))
        stations = np.column_stack((station_arc, np.zeros_like(station_arc)))
        normals = np.tile(np.array([0.0, 1.0]), (station_count, 1))
    else:
        # Radius 5 m (curvature 0.2 1/m) is well below the production
        # planner's 3.5 1/m gate while still exercising sustained steering.
        radius_m = 5.0
        dense_theta = dense_arc / radius_m
        station_theta = station_arc / radius_m
        reference = np.column_stack(
            (
                radius_m * np.sin(dense_theta),
                radius_m * (1.0 - np.cos(dense_theta)),
            )
        )
        stations = np.column_stack(
            (
                radius_m * np.sin(station_theta),
                radius_m * (1.0 - np.cos(station_theta)),
            )
        )
        normals = np.column_stack((-np.sin(station_theta), np.cos(station_theta)))

    # Use 0.64 m rather than the mathematical 0.60 m nominal.  The detector
    # reports the median visible cone surface (not its hidden centre), and the
    # extra 20 mm per side preserves the configured 20 mm safety margin on a
    # curve.  It remains well inside the production 0.42..0.82 m width gate.
    measured_track_width_m = 0.64
    half_width_m = 0.5 * measured_track_width_m
    cones = np.vstack(
        (stations + half_width_m * normals, stations - half_width_m * normals)
    )
    return SyntheticCourse(name, reference, dense_arc, cones)


def _world_to_body(points: np.ndarray, state: BicycleState) -> np.ndarray:
    translated = points - np.array([state.x_m, state.y_m], dtype=float)
    heading_cos = cos(state.yaw_rad)
    heading_sin = sin(state.yaw_rad)
    return np.column_stack(
        (
            heading_cos * translated[:, 0] + heading_sin * translated[:, 1],
            -heading_sin * translated[:, 0] + heading_cos * translated[:, 1],
        )
    )


def ray_circle_scan(
    cone_centers_body: np.ndarray,
    *,
    beam_count: int = 1441,
    cone_radius_m: float = CONE_CROSS_SECTION_RADIUS_M,
    range_min_m: float = 0.15,
    range_max_m: float = 5.0,
    noise_std_m: float = 0.0002,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, float, float]:
    """Ray-cast a 360-degree scan against circular cone cross-sections.

    The nearest positive ray/circle intersection naturally models occlusion.
    A tiny seeded range perturbation avoids relying on exact floating-point
    symmetry and remains below normal A1 measurement noise.
    """

    centers = np.asarray(cone_centers_body, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 2:
        raise ValueError("cone_centers_body must have shape (N, 2)")
    if beam_count < 3 or cone_radius_m <= 0.0:
        raise ValueError("beam_count must be >= 3 and cone_radius_m must be > 0")
    if not 0.0 < range_min_m < range_max_m:
        raise ValueError("scan range limits are invalid")

    angle_min = -pi
    angle_increment = 2.0 * pi / float(beam_count - 1)
    angles = angle_min + angle_increment * np.arange(beam_count, dtype=float)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))

    if len(centers) == 0:
        return np.full(beam_count, np.inf, dtype=float), angle_min, angle_increment

    projection = directions @ centers.T
    center_term = np.sum(centers * centers, axis=1) - cone_radius_m**2
    discriminant = projection * projection - center_term[None, :]
    intersections = projection - np.sqrt(np.maximum(discriminant, 0.0))
    valid = (
        (discriminant >= 0.0)
        & (projection > 0.0)
        & (intersections >= range_min_m)
        & (intersections <= range_max_m)
    )
    intersections = np.where(valid, intersections, np.inf)
    ranges = np.min(intersections, axis=1)

    finite = np.isfinite(ranges)
    if noise_std_m > 0.0 and np.any(finite):
        generator = rng if rng is not None else np.random.default_rng(0)
        ranges[finite] += generator.normal(0.0, noise_std_m, np.count_nonzero(finite))
        in_range = (ranges >= range_min_m) & (ranges <= range_max_m)
        ranges[~in_range] = np.inf
    return ranges, angle_min, angle_increment


def step_bicycle(
    state: BicycleState,
    command: ControlResult,
    *,
    dt_s: float,
    wheelbase_m: float,
) -> BicycleState:
    """Advance one exact-constant-command kinematic bicycle step."""

    if dt_s <= 0.0 or wheelbase_m <= 0.0:
        raise ValueError("dt_s and wheelbase_m must be positive")
    speed = float(command.speed_mps if command.valid else 0.0)
    steering = float(command.steering_angle_rad if command.valid else 0.0)
    if not all(isfinite(value) for value in (speed, steering)):
        raise ValueError("command must be finite")

    yaw_rate = speed * tan(steering) / wheelbase_m
    midpoint_yaw = state.yaw_rad + 0.5 * yaw_rate * dt_s
    return BicycleState(
        x_m=state.x_m + speed * cos(midpoint_yaw) * dt_s,
        y_m=state.y_m + speed * sin(midpoint_yaw) * dt_s,
        yaw_rad=state.yaw_rad + yaw_rate * dt_s,
        speed_mps=speed,
        steering_angle_rad=steering,
    )


def _project_to_reference(
    point: Sequence[float], course: SyntheticCourse
) -> tuple[float, float]:
    query = np.asarray(point, dtype=float)
    starts = course.reference_centerline[:-1]
    segments = np.diff(course.reference_centerline, axis=0)
    length_sq = np.sum(segments * segments, axis=1)
    ratios = np.clip(
        np.sum((query - starts) * segments, axis=1) / length_sq,
        0.0,
        1.0,
    )
    projected = starts + ratios[:, None] * segments
    distance_sq = np.sum((projected - query) ** 2, axis=1)
    index = int(np.argmin(distance_sq))
    segment_length = course.reference_arc_m[index + 1] - course.reference_arc_m[index]
    arc_m = course.reference_arc_m[index] + ratios[index] * segment_length
    return float(arc_m), float(np.sqrt(distance_sq[index]))


def _cone_clearance(
    state: BicycleState,
    course: SyntheticCourse,
    planner_config: PlannerConfig,
    cone_radius_m: float,
) -> float:
    position = np.array([state.x_m, state.y_m], dtype=float)
    center_distance = float(
        np.min(np.linalg.norm(course.cone_centers - position, axis=1))
    )
    # A circular footprint is conservative laterally and does not invent an
    # unconfigured body length.  Physical contact excludes safety_margin_m;
    # that margin remains enforced by the production path validator.
    return center_distance - 0.5 * planner_config.vehicle_width_m - cone_radius_m


def run_synthetic_validation(
    scenario: str,
    *,
    steps: int = 140,
    dt_s: float = 0.10,
    seed: int = 7,
    scan_dropout_step: int | None = None,
) -> SyntheticValidationResult:
    """Run one deterministic core-to-core closed-loop validation scenario.

    When ``scan_dropout_step`` is reached, the harness applies
    ``stop_result('SCAN_DROPOUT')`` on that same step and every later step.
    This is the ROS-independent fault contract; ROS watchdog timing belongs in
    wrapper/launch tests rather than being approximated with wall-clock sleeps.
    """

    if steps <= 0 or dt_s <= 0.0:
        raise ValueError("steps and dt_s must be positive")
    if scan_dropout_step is not None and not 0 <= scan_dropout_step < steps:
        raise ValueError("scan_dropout_step must lie inside the run")

    # The generated course is 0.64 m cone-centre to cone-centre.  Matching the
    # planner's measured-width parameter is normal calibration, not a relaxed
    # validity gate; all clearance, curvature and confidence checks stay on.
    planner_config = PlannerConfig(track_width_m=0.64)
    controller_config = ControllerConfig()
    course = make_course(scenario)
    tracker = ConeTrackFilter(planner_config)
    generator = np.random.default_rng(seed)
    state = BicycleState()

    status_counts: Counter[str] = Counter()
    lateral_errors: list[float] = []
    clearances: list[float] = []
    speeds: list[float] = []
    steerings: list[float] = []
    valid_plans = 0
    planning_attempts = 0
    plan_confidences: list[float] = []
    confirmed_counts: list[int] = []
    real_pair_counts: list[int] = []
    collisions = 0
    positive_after_fault = 0
    fault_progress: float | None = None

    initial_arc, initial_error = _project_to_reference((state.x_m, state.y_m), course)
    lateral_errors.append(initial_error)
    initial_clearance = _cone_clearance(
        state, course, planner_config, CONE_CROSS_SECTION_RADIUS_M
    )
    clearances.append(initial_clearance)
    collisions += int(initial_clearance < 0.0)

    # Cones beyond 1.65 m are deliberately reported as no-return.  This is a
    # realistic reflectivity/range envelope for the tiny target and keeps the
    # multi-hypothesis search bounded in CI without relaxing any planner gate.
    effective_scan_max_m = min(1.65, planner_config.range_max_m)

    for step in range(steps):
        fault_active = scan_dropout_step is not None and step >= scan_dropout_step
        if fault_active:
            if step == scan_dropout_step:
                tracker.reset()
                fault_progress, _ = _project_to_reference(
                    (state.x_m, state.y_m), course
                )
            command = stop_result("SCAN_DROPOUT")
            status_counts[command.reason] += 1
        else:
            planning_attempts += 1
            cone_centers_body = _world_to_body(course.cone_centers, state)
            ranges, angle_min, angle_increment = ray_circle_scan(
                cone_centers_body,
                range_min_m=planner_config.range_min_m,
                range_max_m=effective_scan_max_m,
                rng=generator,
            )
            candidates = detect_cones_from_scan(
                ranges,
                angle_min,
                angle_increment,
                planner_config,
                sensor_range_min_m=planner_config.range_min_m,
                sensor_range_max_m=effective_scan_max_m,
            )
            confirmed = tracker.update(candidates)
            obstacles = extract_obstacle_points_from_scan(
                ranges,
                angle_min,
                angle_increment,
                planner_config,
                sensor_range_min_m=planner_config.range_min_m,
                sensor_range_max_m=effective_scan_max_m,
            )
            plan = plan_centerline(
                confirmed,
                planner_config,
                obstacle_points=obstacles,
            )
            plan_confidences.append(plan.confidence)
            confirmed_counts.append(len(confirmed))
            real_pair_counts.append(plan.real_pair_count)
            status_counts[plan.status] += 1
            if plan.valid:
                valid_plans += 1
                command = compute_pure_pursuit(
                    plan.path,
                    current_speed_mps=state.speed_mps,
                    plan_confidence=plan.confidence,
                    previous_speed_mps=state.speed_mps,
                    previous_steering_angle_rad=state.steering_angle_rad,
                    dt_s=dt_s,
                    config=controller_config,
                    virtual_path=plan.virtual_pair_count > 0,
                )
                if not command.valid:
                    status_counts[f"CONTROL_{command.reason}"] += 1
            else:
                command = stop_result(f"PLANNER_{plan.status}")

        if fault_active and command.speed_mps > 1.0e-9:
            positive_after_fault += 1
        state = step_bicycle(
            state,
            command,
            dt_s=dt_s,
            wheelbase_m=controller_config.wheelbase_m,
        )
        speeds.append(state.speed_mps)
        steerings.append(state.steering_angle_rad)

        _, lateral_error = _project_to_reference((state.x_m, state.y_m), course)
        clearance = _cone_clearance(
            state, course, planner_config, CONE_CROSS_SECTION_RADIUS_M
        )
        lateral_errors.append(lateral_error)
        clearances.append(clearance)
        collisions += int(clearance < 0.0)

    final_arc, _ = _project_to_reference((state.x_m, state.y_m), course)
    progress = max(0.0, final_arc - initial_arc)
    post_fault_travel = (
        max(0.0, final_arc - fault_progress) if fault_progress is not None else 0.0
    )
    valid_fraction = valid_plans / planning_attempts if planning_attempts else 0.0
    required_progress_m = 1.50 if scenario == "straight" else 1.00
    completed = (
        scan_dropout_step is None
        and progress >= required_progress_m
        and collisions == 0
    )

    return SyntheticValidationResult(
        scenario=scenario,
        steps=steps,
        dt_s=dt_s,
        simulated_duration_s=steps * dt_s,
        progress_m=progress,
        required_progress_m=required_progress_m,
        max_lateral_error_m=float(np.max(lateral_errors)),
        p95_lateral_error_m=float(np.percentile(lateral_errors, 95.0)),
        min_clearance_m=float(np.min(clearances)),
        valid_plan_fraction=valid_fraction,
        mean_plan_confidence=(
            float(np.mean(plan_confidences)) if plan_confidences else 0.0
        ),
        max_plan_confidence=max(plan_confidences, default=0.0),
        mean_confirmed_cones=(
            float(np.mean(confirmed_counts)) if confirmed_counts else 0.0
        ),
        mean_real_pairs=float(np.mean(real_pair_counts)) if real_pair_counts else 0.0,
        collisions=collisions,
        positive_commands_after_fault=positive_after_fault,
        post_fault_travel_m=post_fault_travel,
        fault_step=scan_dropout_step,
        completed=completed,
        max_speed_mps=max(speeds, default=0.0),
        max_abs_steering_rad=max((abs(value) for value in steerings), default=0.0),
        final_x_m=state.x_m,
        final_y_m=state.y_m,
        final_yaw_rad=state.yaw_rad,
        status_counts=dict(sorted(status_counts.items())),
    )


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario", choices=("straight", "left_arc", "all"), nargs="?", default="all"
    )
    parser.add_argument("--steps", type=int, default=140)
    parser.add_argument("--dt", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--scan-dropout-step", type=int)
    arguments = parser.parse_args(argv)

    names = (
        ("straight", "left_arc")
        if arguments.scenario == "all"
        else (arguments.scenario,)
    )
    results = [
        asdict(
            run_synthetic_validation(
                name,
                steps=arguments.steps,
                dt_s=arguments.dt,
                seed=arguments.seed,
                scan_dropout_step=arguments.scan_dropout_step,
            )
        )
        for name in names
    ]
    payload: object = results if len(results) > 1 else results[0]
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
