import math
import unittest

import numpy as np

from lidar_cone_planner.preview_core import PreviewConfig, compute_path_preview
from lidar_cone_planner.viewer_core import (
    BevGeometry,
    metric_to_pixel,
    transform_scan_points,
)


class TestPreviewCore(unittest.TestCase):
    def test_straight_and_curved_outputs_are_finite(self) -> None:
        config = PreviewConfig(
            wheelbase_m=0.20,
            lookahead_min_m=0.25,
            lookahead_max_m=0.45,
            validation_speed_mps=0.0,
        )
        for path in (
            np.column_stack((np.linspace(0.0, 1.0, 21), np.zeros(21))),
            np.column_stack(
                (np.linspace(0.0, 1.0, 21), 0.15 * np.linspace(0.0, 1.0, 21) ** 2)
            ),
        ):
            result = compute_path_preview(path, config)
            self.assertTrue(result.valid, result.reason)
            values = (
                result.lookahead_m,
                result.target_x_m,
                result.target_y_m,
                result.heading_rad,
                result.curvature_1pm,
                result.steering_angle_rad,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertAlmostEqual(
            compute_path_preview(
                np.column_stack((np.linspace(0.0, 1.0, 21), np.zeros(21))),
                config,
            ).steering_angle_rad,
            0.0,
            places=9,
        )

    def test_invalid_path_returns_finite_zero_preview(self) -> None:
        result = compute_path_preview([[0.0, 0.0]], PreviewConfig())
        self.assertFalse(result.valid)
        self.assertTrue(
            all(
                math.isfinite(value)
                for value in (
                    result.target_x_m,
                    result.target_y_m,
                    result.heading_rad,
                    result.curvature_1pm,
                    result.steering_angle_rad,
                )
            )
        )


class TestViewerCore(unittest.TestCase):
    def test_metric_to_pixel_orientation_and_scale(self) -> None:
        geometry = BevGeometry(900, 900, 2.5, 1.5)
        self.assertEqual(metric_to_pixel(0.0, 0.0, geometry), (450, 899))
        self.assertEqual(metric_to_pixel(2.5, 0.0, geometry), (450, 0))
        self.assertEqual(metric_to_pixel(0.0, 1.5, geometry), (0, 899))
        self.assertEqual(metric_to_pixel(0.0, -1.5, geometry), (899, 899))

    def test_scan_transform_is_numeric_and_filters_invalid_ranges(self) -> None:
        points = transform_scan_points(
            [1.0, float("inf"), float("nan"), 0.05, 2.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.10,
            range_max=1.50,
            sensor_to_planning=(0.20, -0.10, math.pi / 2.0),
        )
        self.assertEqual(points.shape, (1, 2))
        np.testing.assert_allclose(points[0], [0.20, 0.90], atol=1.0e-8)
        self.assertTrue(np.all(np.isfinite(points)))


if __name__ == "__main__":
    unittest.main()
