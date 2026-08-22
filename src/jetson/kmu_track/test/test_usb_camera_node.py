"""Tests for USB-camera source selection."""

import numpy as np
from builtin_interfaces.msg import Time

from kmu_track.image_message import bgr_to_image_message
from kmu_track.usb_camera_node import opencv_camera_source


def test_dev_video_path_becomes_numeric_index() -> None:
    assert opencv_camera_source('/dev/video0') == 0
    assert opencv_camera_source('/dev/video12') == 12


def test_non_v4l2_path_is_preserved() -> None:
    assert opencv_camera_source('camera.mp4') == 'camera.mp4'


def test_bgr_frame_converts_without_cv_bridge() -> None:
    frame = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    message = bgr_to_image_message(
        frame[:, ::-1], Time(sec=12, nanosec=34), 'front_camera')

    assert message.height == 2
    assert message.width == 3
    assert message.encoding == 'bgr8'
    assert message.is_bigendian == 0
    assert message.step == 9
    expected = np.ascontiguousarray(frame[:, ::-1]).tobytes()
    assert bytes(message.data) == expected
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 34
    assert message.header.frame_id == 'front_camera'
