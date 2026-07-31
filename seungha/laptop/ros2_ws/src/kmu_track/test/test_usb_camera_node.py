"""Tests for USB-camera source selection."""

from kmu_track.usb_camera_node import opencv_camera_source


def test_dev_video_path_becomes_numeric_index() -> None:
    assert opencv_camera_source('/dev/video0') == 0
    assert opencv_camera_source('/dev/video12') == 12


def test_non_v4l2_path_is_preserved() -> None:
    assert opencv_camera_source('camera.mp4') == 'camera.mp4'
