"""Tests for LiDAR obstacle-vehicle detection and lane shifting."""

import unittest

import numpy as np

from kmu_track.obstacle_avoidance_core import (
    ObstacleAvoidanceConfig,
    cluster_obstacle_points,
    detect_obstacle_vehicle,
    exclude_cone_points,
    make_opposite_lane_path,
)


STRAIGHT_PATH = ((0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (2.0, 0.0))


class TestObstacleVehicleDetection(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ObstacleAvoidanceConfig(
            minimum_cluster_points=3,
            minimum_cluster_span_m=0.08,
        )

    def test_cone_returns_are_removed_before_clustering(self) -> None:
        points = ((0.9, 0.25), (0.91, 0.27), (1.2, 0.0))
        filtered = exclude_cone_points(points, ((0.9, 0.25),), 0.10)
        np.testing.assert_allclose(filtered, ((1.2, 0.0),))

    def test_nearby_points_form_one_cluster(self) -> None:
        clusters = cluster_obstacle_points(
            ((1.0, -0.05), (1.0, 0.02), (1.01, 0.10)), 0.14)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(len(clusters[0]), 3)

    def test_non_cone_cluster_on_path_is_an_obstacle_vehicle(self) -> None:
        points = (
            (0.95, -0.10), (0.96, 0.0),
            (0.95, 0.10), (0.97, 0.16),
        )
        obstacle = detect_obstacle_vehicle(
            points, (), STRAIGHT_PATH, self.config)
        self.assertIsNotNone(obstacle)
        self.assertLessEqual(
            obstacle.path_distance_m, self.config.vehicle_half_width_m)
        self.assertEqual(obstacle.avoidance_sign, -1)

    def test_cluster_outside_vehicle_half_width_is_ignored(self) -> None:
        points = (
            (0.95, 0.35), (0.96, 0.44),
            (0.95, 0.52), (0.97, 0.59),
        )
        self.assertIsNone(detect_obstacle_vehicle(
            points, (), STRAIGHT_PATH, self.config))

    def test_cone_shaped_cluster_is_not_an_obstacle(self) -> None:
        points = (
            (0.95, -0.05), (0.96, 0.0),
            (0.95, 0.06), (0.97, 0.10),
        )
        self.assertIsNone(detect_obstacle_vehicle(
            points, ((0.96, 0.02),), STRAIGHT_PATH, self.config))


class TestOppositeLanePath(unittest.TestCase):
    def test_path_shifts_one_lane_left_smoothly(self) -> None:
        shifted = make_opposite_lane_path(STRAIGHT_PATH, 0.55, 0.80)
        np.testing.assert_allclose(shifted[0], STRAIGHT_PATH[0])
        self.assertGreater(shifted[1, 1], 0.0)
        self.assertAlmostEqual(shifted[-1, 1], 0.55)

    def test_negative_offset_shifts_right(self) -> None:
        shifted = make_opposite_lane_path(STRAIGHT_PATH, -0.55, 0.80)
        self.assertAlmostEqual(shifted[-1, 1], -0.55)


if __name__ == '__main__':
    unittest.main()
