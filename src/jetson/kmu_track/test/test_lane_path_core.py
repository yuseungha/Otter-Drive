"""Tests for camera-image to vehicle-frame lane projection."""

import unittest

import numpy as np

from kmu_track.lane_path_core import LanePathProjector


class TestLanePathProjector(unittest.TestCase):
    def setUp(self) -> None:
        self.projector = LanePathProjector()

    def test_image_center_projects_to_vehicle_center(self) -> None:
        ground = self.projector.project_normalized(
            ((0.50, 0.50), (0.50, 0.98)))
        np.testing.assert_allclose(ground[:, 1], (0.0, 0.0), atol=1.0e-8)
        self.assertGreater(ground[0, 0], ground[1, 0])

    def test_image_left_is_positive_vehicle_y(self) -> None:
        ground = self.projector.project_normalized(
            ((0.38, 0.50), (0.10, 0.98)))
        self.assertTrue(np.all(ground[:, 1] > 0.0))

    def test_geometry_becomes_forward_ordered_metric_path(self) -> None:
        geometry = {
            'valid': True,
            'image_w': 101,
            'image_h': 101,
            'fit_path': [
                {'x': 50.0, 'y': 50},
                {'x': 50.0, 'y': 65},
                {'x': 50.0, 'y': 80},
                {'x': 50.0, 'y': 98},
            ],
        }
        path = self.projector.project_geometry(geometry)
        np.testing.assert_allclose(path[0], (0.0, 0.0), atol=1.0e-8)
        self.assertGreaterEqual(len(path), 3)
        self.assertTrue(np.all(np.diff(path[:, 0]) >= 0.0))
        np.testing.assert_allclose(path[:, 1], 0.0, atol=1.0e-8)

    def test_invalid_geometry_publishes_no_path(self) -> None:
        self.assertEqual(
            self.projector.project_geometry({'valid': False}).shape,
            (0, 2),
        )


if __name__ == '__main__':
    unittest.main()
