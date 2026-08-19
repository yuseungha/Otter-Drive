import os
import tempfile
import unittest
from unittest import mock

try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter

    from lidar_cone_planner.cone_cv_viewer import ConeCvViewer

    ROS_VIEWER_AVAILABLE = True
except ImportError:
    ROS_VIEWER_AVAILABLE = False


@unittest.skipUnless(
    ROS_VIEWER_AVAILABLE, "ROS 2 and OpenCV Python packages are unavailable"
)
class ViewerRosHeadlessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.try_shutdown()

    def test_no_display_writes_image_without_opening_gui(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = os.path.join(directory, "headless.png")
            overrides = [
                Parameter("viewer_enabled", value=True),
                Parameter("viewer_record_path", value=output),
                Parameter("viewer_width_px", value=320),
                Parameter("viewer_height_px", value=320),
            ]
            with mock.patch.dict(
                os.environ, {"DISPLAY": "", "WAYLAND_DISPLAY": ""}
            ):
                viewer = ConeCvViewer(parameter_overrides=overrides)
                try:
                    self.assertFalse(viewer.gui_active)
                    viewer._render_timer()
                    self.assertTrue(os.path.isfile(output))
                    image = cv2.imread(output)
                    self.assertIsNotNone(image)
                    self.assertEqual(image.shape[:2], (320, 320))
                finally:
                    viewer.destroy_node()

    def test_viewer_enabled_false_is_safe_without_output(self) -> None:
        overrides = [Parameter("viewer_enabled", value=False)]
        with mock.patch.dict(os.environ, {"DISPLAY": "", "WAYLAND_DISPLAY": ""}):
            viewer = ConeCvViewer(parameter_overrides=overrides)
            try:
                self.assertFalse(viewer.gui_active)
                self.assertFalse(viewer.renderer_active)
                viewer._render_timer()
            finally:
                viewer.destroy_node()

    def test_invalid_plan_renders_center_direction_preview(self) -> None:
        overrides = [
            Parameter("viewer_enabled", value=False),
            Parameter("viewer_width_px", value=400),
            Parameter("viewer_height_px", value=400),
        ]
        with mock.patch.dict(os.environ, {"DISPLAY": "", "WAYLAND_DISPLAY": ""}):
            viewer = ConeCvViewer(parameter_overrides=overrides)
            try:
                viewer.status_values = {"status": "LOW_CONFIDENCE"}
                viewer.marker_points = {
                    "raw_center": np.asarray(
                        [[0.30, 0.02], [0.65, 0.08], [1.00, 0.18]],
                        dtype=float,
                    )
                }
                image = viewer.render_frame()
                preview_color = np.asarray([0, 170, 255], dtype=np.uint8)
                preview_pixels = np.all(image == preview_color, axis=2)
                self.assertGreater(int(np.count_nonzero(preview_pixels)), 0)
            finally:
                viewer.destroy_node()


if __name__ == "__main__":
    unittest.main()
