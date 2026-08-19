from math import pi
import unittest

import numpy as np

from lidar_cone_planner.planner_core import (
    ConeTrackFilter,
    PlannerConfig,
    detect_cones_from_scan,
    left_normal_from_tangent,
    plan_centerline,
    station_center_from_boundaries,
)


def make_boundaries(center_points: np.ndarray, width: float) -> np.ndarray:
    tangents = np.gradient(center_points, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    left = center_points + 0.5 * width * normals
    right = center_points - 0.5 * width * normals
    return np.vstack((left, right))


class TestPlannerConfig(unittest.TestCase):
    def test_invalid_ranges_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            PlannerConfig(range_min_m=2.0, range_max_m=1.0)
        with self.assertRaises(ValueError):
            PlannerConfig(track_width_min_m=0.7, track_width_m=0.6)
        with self.assertRaises(ValueError):
            PlannerConfig(track_smoothing_alpha=0.0)
        with self.assertRaises(ValueError):
            PlannerConfig(virtual_cone_confidence_weight=0.0)
        with self.assertRaises(ValueError):
            PlannerConfig(max_turn_angle_deg=float("nan"))
        with self.assertRaises(ValueError):
            PlannerConfig(pair_max_along_error_m=-0.1)
        with self.assertRaises(ValueError):
            PlannerConfig(curvature_sample_distance_m=0.0)
        with self.assertRaises(ValueError):
            PlannerConfig(cluster_max_skip_beams=1.5)


class TestLocalStationGeometry(unittest.TestCase):
    def test_two_sided_station_is_exact_midpoint(self) -> None:
        center = station_center_from_boundaries(
            [0.20, 0.90], [0.80, -0.10], [0.60, 0.80], 0.64
        )
        np.testing.assert_allclose(center, [0.50, 0.40], atol=1.0e-12)

    def test_left_only_uses_rotated_local_normal_not_global_y(self) -> None:
        tangent = np.array([0.0, 1.0])
        normal = left_normal_from_tangent(tangent)
        expected_center = np.array([0.90, 0.30])
        left = expected_center + 0.32 * normal

        center = station_center_from_boundaries(left, None, tangent, 0.64)

        # For this 90-degree station the offset is purely global x, a direct
        # regression guard against adding/subtracting a fixed y value.
        np.testing.assert_allclose(normal, [-1.0, 0.0], atol=1.0e-12)
        np.testing.assert_allclose(center, expected_center, atol=1.0e-12)

    def test_right_only_uses_oblique_local_normal(self) -> None:
        tangent = np.array([0.60, 0.80])
        normal = left_normal_from_tangent(tangent)
        expected_center = np.array([1.10, -0.20])
        right = expected_center - 0.32 * normal

        center = station_center_from_boundaries(None, right, tangent, 0.64)

        np.testing.assert_allclose(normal, [-0.80, 0.60], atol=1.0e-12)
        np.testing.assert_allclose(center, expected_center, atol=1.0e-12)

    def test_invalid_station_geometry_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            station_center_from_boundaries(None, None, [1.0, 0.0], 0.64)
        with self.assertRaises(ValueError):
            station_center_from_boundaries([1.0, 0.2], None, [0.0, 0.0], 0.64)


class TestPlanCenterline(unittest.TestCase):
    def setUp(self) -> None:
        self.config = PlannerConfig(
            smoothing_iterations=0,
            path_resolution_m=0.05,
            min_path_length_m=0.60,
            min_plan_confidence=0.30,
        )

    def test_straight_course(self) -> None:
        x = 0.45 + np.arange(8) * 0.297
        expected = np.column_stack((x, np.zeros_like(x)))
        cones = make_boundaries(expected, 0.60)
        np.random.default_rng(7).shuffle(cones)

        result = plan_centerline(cones, self.config)

        self.assertTrue(result.valid, result.status)
        self.assertEqual(len(result.raw_centerline), len(expected))
        self.assertEqual(result.virtual_pair_count, 0)
        np.testing.assert_allclose(result.raw_centerline[:, 1], 0.0, atol=1.0e-7)
        self.assertTrue(np.all(np.diff(result.raw_centerline[:, 0]) > 0.0))
        self.assertGreaterEqual(result.path_length_m, self.config.min_path_length_m)

    def test_cone_input_order_does_not_change_selected_path(self) -> None:
        x = 0.35 + np.arange(7) * 0.297
        centers = np.column_stack((x, 0.04 * x**2))
        cones = make_boundaries(centers, 0.60)
        reference = plan_centerline(cones, self.config)
        self.assertTrue(reference.valid, reference.status)
        for seed in range(6):
            shuffled = cones.copy()
            np.random.default_rng(seed).shuffle(shuffled)
            result = plan_centerline(shuffled, self.config)
            self.assertEqual(result.status, reference.status)
            np.testing.assert_allclose(result.left_boundary, reference.left_boundary)
            np.testing.assert_allclose(result.right_boundary, reference.right_boundary)
            np.testing.assert_allclose(result.path, reference.path)

    def test_curved_course_is_ordered_and_inside_center_tolerance(self) -> None:
        x = 0.40 + np.arange(9) * 0.27
        y = 0.14 * x**2
        expected = np.column_stack((x, y))
        cones = make_boundaries(expected, 0.60)
        np.random.default_rng(11).shuffle(cones)

        result = plan_centerline(cones, self.config)

        self.assertTrue(result.valid, result.status)
        self.assertGreaterEqual(len(result.raw_centerline), 7)
        self.assertTrue(np.all(np.diff(result.raw_centerline[:, 0]) > 0.0))
        for midpoint in result.raw_centerline:
            error = np.min(np.linalg.norm(expected - midpoint, axis=1))
            self.assertLess(error, 0.04)

    def test_vehicle_outside_first_gate_is_fail_closed(self) -> None:
        centers = np.array(
            [[0.45, 0.40], [0.75, 0.40], [1.05, 0.40], [1.35, 0.40]]
        )
        cones = np.vstack(
            (
                centers + np.array([0.0, 0.20]),
                centers - np.array([0.0, 0.40]),
            )
        )

        result = plan_centerline(cones, self.config)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, "PATH_OUTSIDE_CORRIDOR")

    def test_clutter_pair_does_not_preempt_nearest_true_station(self) -> None:
        centers = np.array(
            [[0.30, 0.0], [0.60, 0.0], [0.90, 0.0], [1.20, 0.0]]
        )
        cones = make_boundaries(centers, 0.66)
        # An exact-width clutter pair slightly farther than the first true pair.
        clutter = np.array([[0.40, -0.30], [0.40, 0.30]])
        result = plan_centerline(np.vstack((cones, clutter)), self.config)

        self.assertTrue(result.valid, result.status)
        self.assertAlmostEqual(result.raw_centerline[0, 0], 0.30, delta=0.03)

    def test_missing_complete_station_is_skipped_without_diagonal_pair(self) -> None:
        centers = np.array(
            [[0.35, 0.0], [0.65, 0.0], [0.95, 0.0], [1.25, 0.0], [1.55, 0.0]]
        )
        cones = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        # Drop both cones at station 2. A valid chain should bridge the gap.
        remaining = np.vstack((np.delete(cones[0], 2, axis=0), np.delete(cones[1], 2, axis=0)))

        result = plan_centerline(remaining, self.config)

        self.assertTrue(result.valid, result.status)
        self.assertTrue(np.all(np.diff(result.raw_centerline[:, 0]) > 0.0))
        self.assertFalse(np.any(np.isclose(result.raw_centerline[:, 0], 0.95, atol=0.05)))

    def test_short_or_narrow_course_is_fail_closed(self) -> None:
        short = plan_centerline([[0.5, 0.3], [0.5, -0.3]], self.config)
        self.assertFalse(short.valid)
        self.assertEqual(short.status, "INSUFFICIENT_PAIRS")

        centers = np.column_stack((0.3 + np.arange(5) * 0.3, np.zeros(5)))
        narrow_config = PlannerConfig(
            track_width_m=0.50,
            track_width_min_m=0.35,
            track_width_max_m=0.65,
            vehicle_width_m=0.50,
            safety_margin_m=0.02,
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
        )
        narrow = plan_centerline(make_boundaries(centers, 0.50), narrow_config)
        self.assertFalse(narrow.valid)
        self.assertEqual(narrow.status, "INSUFFICIENT_CLEARANCE")

    def test_production_smoothing_is_displacement_bounded(self) -> None:
        centers = np.array(
            [[0.35, 0.0], [0.65, 0.0], [0.95, 0.10], [1.25, 0.22], [1.55, 0.35]]
        )
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
            max_path_curvature_1pm=0.0,
            max_centerline_deviation_m=0.02,
            smoothing_weight=0.8,
            smoothing_iterations=5,
        )
        result = plan_centerline(make_boundaries(centers, 0.60), config)
        self.assertTrue(result.valid, result.status)
        raw_control = np.vstack(([0.0, 0.0], result.raw_centerline))
        unsmoothed_path = []
        for first, second in zip(raw_control[:-1], raw_control[1:]):
            segment = second - first
            length = float(np.linalg.norm(segment))
            sample_count = max(2, int(np.ceil(length / config.path_resolution_m)) + 1)
            unsmoothed_path.extend(
                first + ratio * segment
                for ratio in np.linspace(0.0, 1.0, sample_count)
            )
        unsmoothed_path = np.asarray(unsmoothed_path)
        maximum_deviation = max(
            np.min(np.linalg.norm(unsmoothed_path - point, axis=1))
            for point in result.path
        )
        self.assertLessEqual(maximum_deviation, config.max_centerline_deviation_m + 0.01)

    def test_curved_path_that_violates_vehicle_clearance_is_rejected(self) -> None:
        centers = np.array(
            [[0.35, 0.0], [0.65, 0.0], [0.95, 0.16], [1.25, 0.36], [1.55, 0.58]]
        )
        config = PlannerConfig(
            track_width_m=0.60,
            track_width_min_m=0.45,
            track_width_max_m=0.75,
            vehicle_width_m=0.54,
            safety_margin_m=0.02,
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
            max_path_curvature_1pm=0.0,
            max_centerline_deviation_m=0.05,
            smoothing_weight=0.9,
            smoothing_iterations=6,
        )
        result = plan_centerline(make_boundaries(centers, 0.60), config)
        self.assertFalse(result.valid)
        self.assertIn(
            result.status,
            {"INSUFFICIENT_CLEARANCE", "PATH_OUTSIDE_CORRIDOR"},
        )

    def test_sharp_path_is_rejected_by_curvature_limit(self) -> None:
        centers = np.array(
            [[0.32, 0.0], [0.62, 0.0], [0.86, 0.18], [1.05, 0.43], [1.17, 0.70]]
        )
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
            max_path_curvature_1pm=0.75,
            smoothing_iterations=0,
            pair_max_along_error_m=0.22,
            enable_single_side_fallback=False,
        )
        result = plan_centerline(make_boundaries(centers, 0.60), config)
        self.assertFalse(result.valid)
        # Collision/clearance vetoes intentionally take priority over the
        # curvature diagnostic when both constraints are violated.
        self.assertIn(
            result.status,
            {"CURVATURE_LIMIT", "CONE_OBSTACLE_ON_PATH"},
        )

    def test_short_one_sided_tail_uses_confidence_penalized_virtual_pairs(self) -> None:
        centers = np.column_stack((0.35 + np.arange(6) * 0.30, np.zeros(6)))
        boundaries = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        # Keep three complete anchors and only the left boundary for two more
        # stations, exactly at the configured cap.
        cones = np.vstack((boundaries[0, :5], boundaries[1, :3]))
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.20,
            max_path_curvature_1pm=0.0,
            max_virtual_pairs=2,
        )

        result = plan_centerline(cones, config)
        full = plan_centerline(make_boundaries(centers, 0.60), config)

        self.assertTrue(result.valid, result.status)
        self.assertEqual(result.status, "OK_VIRTUAL")
        self.assertEqual(result.real_pair_count, 3)
        self.assertEqual(result.virtual_pair_count, 2)
        self.assertLess(result.confidence, full.confidence)
        np.testing.assert_allclose(result.raw_centerline[-1], centers[4], atol=0.04)

    def test_single_boundary_cannot_seed_virtual_course(self) -> None:
        x = 0.35 + np.arange(6) * 0.30
        single_boundary = np.column_stack((x, np.full_like(x, 0.30)))
        result = plan_centerline(single_boundary, self.config)

        self.assertFalse(result.valid)
        self.assertEqual(result.virtual_pair_count, 0)

    def test_curved_right_boundary_tail_uses_local_normal(self) -> None:
        x = 0.35 + np.arange(6) * 0.28
        centers = np.column_stack((x, 0.08 * x**2))
        boundaries = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        cones = np.vstack((boundaries[0, :3], boundaries[1, :5]))
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.15,
            max_path_curvature_1pm=0.0,
        )

        result = plan_centerline(cones, config)

        self.assertTrue(result.valid, result.status)
        self.assertEqual(result.virtual_pair_count, 2)
        np.testing.assert_allclose(result.raw_centerline[-1], centers[4], atol=0.05)

    def test_alternating_side_dropout_uses_virtual_not_diagonal_real_pair(self) -> None:
        centers = np.column_stack((0.35 + np.arange(6) * 0.30, np.zeros(6)))
        boundaries = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        left = np.delete(boundaries[0], 3, axis=0)
        right = np.delete(boundaries[1], 4, axis=0)
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
            max_path_curvature_1pm=0.0,
        )

        result = plan_centerline(np.vstack((left, right)), config)

        self.assertTrue(result.valid, result.status)
        self.assertEqual(result.real_pair_count, 3)
        self.assertEqual(result.virtual_pair_count, 2)
        for midpoint in result.raw_centerline:
            nearest = np.min(np.linalg.norm(centers - midpoint, axis=1))
            self.assertLess(nearest, 0.04)

    def test_virtual_fraction_limit_rejects_excess_missing_tail(self) -> None:
        centers = np.column_stack((0.35 + np.arange(6) * 0.30, np.zeros(6)))
        boundaries = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        cones = np.vstack((boundaries[0], boundaries[1, :3]))
        config = PlannerConfig(
            min_path_length_m=0.5,
            min_plan_confidence=0.0,
            max_path_curvature_1pm=0.0,
            max_virtual_fraction=0.25,
        )

        result = plan_centerline(cones, config)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, "VIRTUAL_LIMIT_EXCEEDED")
        self.assertEqual(result.real_pair_count, 3)
        self.assertEqual(result.virtual_pair_count, 1)

    def test_absolute_virtual_limit_rejects_third_missing_station(self) -> None:
        centers = np.column_stack((0.35 + np.arange(6) * 0.30, np.zeros(6)))
        boundaries = make_boundaries(centers, 0.60).reshape(2, len(centers), 2)
        cones = np.vstack((boundaries[0], boundaries[1, :3]))
        result = plan_centerline(
            cones,
            PlannerConfig(
                min_path_length_m=0.5,
                min_plan_confidence=0.0,
                max_path_curvature_1pm=0.0,
                max_virtual_pairs=2,
                max_virtual_fraction=0.49,
            ),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, "VIRTUAL_LIMIT_EXCEEDED")
        self.assertEqual(result.virtual_pair_count, 2)

    def test_unused_cone_on_centerline_vetoes_path(self) -> None:
        centers = np.column_stack((0.35 + np.arange(6) * 0.30, np.zeros(6)))
        cones = make_boundaries(centers, 0.60)
        result = plan_centerline(
            np.vstack((cones, [[1.10, 0.0]])),
            PlannerConfig(
                min_path_length_m=0.5,
                min_plan_confidence=0.0,
                max_path_curvature_1pm=0.0,
            ),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, "CONE_OBSTACLE_ON_PATH")

    def test_observed_narrow_row_vetoes_virtual_boundary(self) -> None:
        x = 0.35 + np.arange(5) * 0.30
        complete = np.vstack(
            (
                np.column_stack((x[:3], np.full(3, 0.30))),
                np.column_stack((x[:3], np.full(3, -0.30))),
            )
        )
        tail = np.vstack(
            (
                np.column_stack((x[3:], np.full(2, 0.30))),
                np.column_stack((x[3:], np.full(2, -0.10))),
            )
        )
        result = plan_centerline(
            np.vstack((complete, tail)),
            PlannerConfig(
                min_path_length_m=0.5,
                min_plan_confidence=0.0,
                max_path_curvature_1pm=0.0,
            ),
        )

        self.assertFalse(result.valid)
        self.assertIn(
            result.status,
            {"CONE_OBSTACLE_ON_PATH", "PATH_OUTSIDE_CORRIDOR"},
        )

    def test_sparse_station_density_reduces_confidence(self) -> None:
        dense_x = 0.35 + np.arange(6) * 0.30
        sparse_x = 0.35 + np.arange(5) * 0.60
        dense = plan_centerline(
            make_boundaries(
                np.column_stack((dense_x, np.zeros_like(dense_x))), 0.60
            ),
            self.config,
        )
        sparse = plan_centerline(
            make_boundaries(
                np.column_stack((sparse_x, np.zeros_like(sparse_x))), 0.60
            ),
            self.config,
        )

        self.assertTrue(dense.valid, dense.status)
        self.assertLess(sparse.confidence, dense.confidence)


class TestScanClustering(unittest.TestCase):
    def test_cone_sized_clusters_are_detected_and_wall_rejected(self) -> None:
        angle_increment = pi / 360.0
        ranges = np.full(721, np.inf, dtype=float)
        ranges[318:323] = [1.005, 1.001, 1.000, 1.002, 1.006]
        ranges[398:403] = [1.205, 1.201, 1.200, 1.202, 1.206]
        ranges[500:550] = 2.00

        cones = detect_cones_from_scan(
            ranges, -pi, angle_increment, PlannerConfig()
        )

        self.assertEqual(cones.shape, (2, 2))
        self.assertTrue(np.all(cones[:, 0] > 0.8))
        self.assertTrue(np.any(cones[:, 1] < 0.0))
        self.assertTrue(np.any(cones[:, 1] > 0.0))

    def test_one_invalid_beam_inside_cone_is_bridged(self) -> None:
        angle_increment = pi / 360.0
        ranges = np.full(721, np.inf, dtype=float)
        ranges[350:357] = [1.007, 1.003, 1.001, 1.0, 1.001, 1.003, 1.007]
        ranges[353] = np.inf

        cones = detect_cones_from_scan(
            ranges, -pi, angle_increment, PlannerConfig()
        )

        self.assertEqual(cones.shape, (1, 2))

    def test_a1_single_return_replication_is_rejected(self) -> None:
        angle_increment = pi / 360.0
        config = PlannerConfig()
        for distance in (0.5, 1.0, 3.0):
            for replicas in (3, 5, 8):
                ranges = np.full(721, np.inf, dtype=float)
                ranges[360 : 360 + replicas] = distance
                cones = detect_cones_from_scan(
                    ranges, -pi, angle_increment, config
                )
                self.assertEqual(len(cones), 0)

    def test_a1_compensated_multi_return_cone_keeps_one_stable_center(self) -> None:
        angle_increment = pi / 360.0
        raw = np.full(721, np.inf, dtype=float)
        raw[359:362] = [1.004, 1.000, 1.004]
        compensated = np.full(721, np.inf, dtype=float)
        compensated[357:363] = [1.004, 1.004, 1.000, 1.000, 1.004, 1.004]

        raw_cones = detect_cones_from_scan(
            raw, -pi, angle_increment, PlannerConfig()
        )
        compensated_cones = detect_cones_from_scan(
            compensated, -pi, angle_increment, PlannerConfig()
        )

        self.assertEqual(raw_cones.shape, (1, 2))
        self.assertEqual(compensated_cones.shape, (1, 2))
        self.assertLess(
            float(np.linalg.norm(raw_cones[0] - compensated_cones[0])),
            0.03,
        )

    def test_sensor_transform_and_planning_roi_are_applied(self) -> None:
        angle_increment = pi / 360.0
        ranges = np.full(721, np.inf, dtype=float)
        ranges[358:363] = [1.004, 1.001, 1.0, 1.001, 1.004]
        config = PlannerConfig(planning_min_forward_m=0.2)

        cones = detect_cones_from_scan(
            ranges,
            -pi,
            angle_increment,
            config,
            sensor_to_planning=(0.30, 0.20, 0.0),
        )

        self.assertEqual(cones.shape, (1, 2))
        self.assertAlmostEqual(cones[0, 0], 1.30, delta=0.03)
        self.assertAlmostEqual(cones[0, 1], 0.20, delta=0.03)

    def test_front_angle_roi_is_evaluated_after_lidar_yaw_transform(self) -> None:
        angle_increment = pi / 360.0
        ranges = np.full(721, np.inf, dtype=float)
        # A cluster at -90 degrees in the laser frame points forward after a
        # +90 degree mount yaw. It must survive a vehicle-frame +/-10 deg ROI.
        ranges[178:183] = [1.004, 1.001, 1.0, 1.001, 1.004]
        config = PlannerConfig(
            front_angle_min_deg=-10.0,
            front_angle_max_deg=10.0,
        )

        cones = detect_cones_from_scan(
            ranges,
            -pi,
            angle_increment,
            config,
            sensor_to_planning=(0.30, 0.0, pi / 2.0),
        )

        self.assertEqual(cones.shape, (1, 2))
        self.assertAlmostEqual(cones[0, 0], 1.30, delta=0.03)
        self.assertAlmostEqual(cones[0, 1], 0.0, delta=0.03)

    def test_bad_scan_metadata_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            detect_cones_from_scan([1.0, 1.0], 0.0, 0.0, PlannerConfig())
        with self.assertRaises(ValueError):
            detect_cones_from_scan(
                [1.0, 1.0],
                0.0,
                0.01,
                PlannerConfig(),
                sensor_range_min_m=2.0,
                sensor_range_max_m=1.0,
            )

    def test_measured_surface_bias_correction_moves_center_away_from_sensor(self) -> None:
        angle_increment = pi / 360.0
        ranges = np.full(721, np.inf, dtype=float)
        ranges[358:363] = [1.004, 1.001, 1.0, 1.001, 1.004]
        uncorrected = detect_cones_from_scan(
            ranges, -pi, angle_increment, PlannerConfig()
        )
        corrected = detect_cones_from_scan(
            ranges,
            -pi,
            angle_increment,
            PlannerConfig(cone_center_radial_offset_m=0.05),
        )
        self.assertEqual(corrected.shape, (1, 2))
        self.assertAlmostEqual(corrected[0, 0] - uncorrected[0, 0], 0.05, delta=0.003)


class TestConeTrackFilter(unittest.TestCase):
    def test_candidates_require_confirmation_and_misses_are_not_output(self) -> None:
        tracker = ConeTrackFilter(PlannerConfig(track_confirmation_scans=2))
        first = tracker.update([[1.0, 0.3]])
        second = tracker.update([[1.02, 0.29]])
        missed = tracker.update([])

        self.assertEqual(len(first), 0)
        self.assertEqual(len(second), 1)
        self.assertEqual(len(missed), 0)

    def test_one_scan_glint_never_becomes_confirmed(self) -> None:
        tracker = ConeTrackFilter(PlannerConfig(track_confirmation_scans=2))
        self.assertEqual(len(tracker.update([[0.8, -0.2]])), 0)
        self.assertEqual(len(tracker.update([])), 0)
        self.assertEqual(len(tracker.update([[1.4, 0.4]])), 0)

    def test_hit_streak_resets_after_miss(self) -> None:
        tracker = ConeTrackFilter(PlannerConfig(track_confirmation_scans=2))
        self.assertEqual(len(tracker.update([[1.0, 0.3]])), 0)
        self.assertEqual(len(tracker.update([])), 0)
        self.assertEqual(len(tracker.update([[1.01, 0.3]])), 0)
        self.assertEqual(len(tracker.update([[1.02, 0.3]])), 1)

    def test_confirmed_track_requires_reconfirmation_after_miss(self) -> None:
        tracker = ConeTrackFilter(PlannerConfig(track_confirmation_scans=2))
        self.assertEqual(len(tracker.update([[1.0, 0.3]])), 0)
        self.assertEqual(len(tracker.update([[1.01, 0.3]])), 1)
        self.assertEqual(len(tracker.update([])), 0)
        self.assertEqual(len(tracker.update([[1.02, 0.3]])), 0)
        self.assertEqual(len(tracker.update([[1.03, 0.3]])), 1)


if __name__ == "__main__":
    unittest.main()
