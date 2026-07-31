"""Synthetic-image tests for the lane detector."""

import cv2
import numpy as np

from kmu_track.lane_core import detect_lane_center, preprocess_lane_frame


def _lane_image(offset_px: int = 0) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(image, (190 + offset_px, 479), (270 + offset_px, 220), (255, 255, 255), 12)
    cv2.line(image, (450 + offset_px, 479), (370 + offset_px, 220), (0, 255, 255), 12)
    return image


def test_centered_two_lane_detection() -> None:
    result = detect_lane_center(_lane_image())
    assert result.valid
    assert result.lane_count == 2
    assert abs(result.center_error) < 0.05
    assert result.confidence > 0.5


def test_right_shift_has_positive_error() -> None:
    result = detect_lane_center(_lane_image(offset_px=55))
    assert result.valid
    assert result.center_error > 0.1


def test_roi_bev_hsv_preprocessing_outputs_expected_masks() -> None:
    result = preprocess_lane_frame(_lane_image())
    assert result.roi.shape == (264, 640, 3)
    assert result.bev.shape == (480, 640, 3)
    assert result.white_mask.shape == (480, 640)
    assert result.yellow_mask.shape == (480, 640)
    assert result.binary.shape == (480, 640)
    assert np.count_nonzero(result.white_mask) > 0
    assert np.count_nonzero(result.yellow_mask) > 0
    assert np.count_nonzero(result.binary) > 0
