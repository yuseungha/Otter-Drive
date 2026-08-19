"""Synthetic-frame tests for safe traffic-light decisions."""

import cv2
import numpy as np

from kmu_track.traffic_light_core import SignalState, TrafficLightDetector


def _frame(*colors) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    centers = [(260, 240), (380, 240)]
    for center, color in zip(centers, colors):
        cv2.circle(image, center, 55, color, -1)
    return image


def _mixed_frame(red_radius: int, green_radius: int) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (220, 240), red_radius, (0, 0, 255), -1)
    cv2.circle(image, (400, 240), green_radius, (0, 255, 0), -1)
    return image


def _arrow_frame(left: bool = True, red_noise_radius: int = 0) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    if left:
        points = np.array([
            [150, 240], [255, 150], [255, 195], [490, 195],
            [490, 285], [255, 285], [255, 330],
        ], dtype=np.int32)
    else:
        points = np.array([
            [490, 240], [385, 150], [385, 195], [150, 195],
            [150, 285], [385, 285], [385, 330],
        ], dtype=np.int32)
    cv2.fillPoly(image, [points], (0, 255, 0))
    if red_noise_radius:
        cv2.circle(
            image, (200, 100), red_noise_radius, (0, 0, 255), -1)
    return image


def _split_line_arrow_frame(left: bool = True) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    color = (0, 255, 0)
    if left:
        cv2.line(image, (165, 240), (270, 145), color, 34)
        cv2.line(image, (165, 240), (270, 335), color, 34)
        cv2.line(image, (305, 240), (500, 240), color, 34)
    else:
        cv2.line(image, (475, 240), (370, 145), color, 34)
        cv2.line(image, (475, 240), (370, 335), color, 34)
        cv2.line(image, (335, 240), (140, 240), color, 34)
    return image


def _detector(**kwargs) -> TrafficLightDetector:
    parameters = {
        'min_red_ratio': 0.005,
        'min_green_ratio': 0.005,
        'min_blob_area': 100.0,
        'confirm_red_frames': 5,
        'confirm_green_frames': 3,
        'lost_signal_frames': 2,
    }
    parameters.update(kwargs)
    return TrafficLightDetector(
        **parameters,
    )


def test_red_requires_five_consecutive_frames_before_stop() -> None:
    detector = _detector(confirm_green_frames=1)
    assert detector.analyze(_frame((0, 255, 0))).state == SignalState.GO
    red = _frame((0, 0, 255))
    for expected_streak in range(1, 5):
        result = detector.analyze(red)
        assert result.state == SignalState.GO
        assert result.red_streak == expected_streak
        assert result.reason == (
            f'RED DETECTED -> RED CHECK {expected_streak}/5')
    result = detector.analyze(red)
    assert result.state == SignalState.STOP
    assert result.red_streak == 5
    assert result.reason == 'RED DETECTED -> RED CONFIRMED'


def test_red_streak_resets_when_red_is_interrupted() -> None:
    detector = _detector(confirm_green_frames=1)
    assert detector.analyze(_frame((0, 255, 0))).state == SignalState.GO
    red = _frame((0, 0, 255))
    detector.analyze(red)
    detector.analyze(red)
    result = detector.analyze(_frame((0, 255, 0)))
    assert result.state == SignalState.GO
    assert result.red_streak == 0
    result = detector.analyze(red)
    assert result.state == SignalState.GO
    assert result.red_streak == 1
    assert result.reason == 'RED DETECTED -> RED CHECK 1/5'


def test_green_requires_confirmation_before_go() -> None:
    detector = _detector(confirm_green_frames=5)
    green = _frame((0, 255, 0))
    for expected_streak in range(1, 5):
        result = detector.analyze(green)
        assert result.state == SignalState.STOP
        assert result.green_streak == expected_streak
    result = detector.analyze(green)
    assert result.state == SignalState.GO
    assert result.reason == 'GREEN DETECTED -> GREEN CONFIRMED'


def test_red_overrides_go_after_confirmation() -> None:
    detector = _detector()
    green = _frame((0, 255, 0))
    for _ in range(3):
        result = detector.analyze(green)
    assert result.state == SignalState.GO
    red = _frame((0, 0, 255))
    for _ in range(4):
        result = detector.analyze(red)
        assert result.state == SignalState.GO
    result = detector.analyze(red)
    assert result.state == SignalState.STOP


def test_green_dominates_smaller_red_noise() -> None:
    detector = _detector()
    image = _mixed_frame(red_radius=25, green_radius=65)
    for _ in range(3):
        result = detector.analyze(image)
    assert result.state == SignalState.GO
    assert result.reason.startswith('GREEN DOMINANT -> GREEN CONFIRMED')
    assert result.evidence.green_ratio > result.evidence.red_ratio


def test_red_dominates_smaller_green_noise() -> None:
    detector = _detector()
    result = detector.analyze(_mixed_frame(red_radius=65, green_radius=25))
    assert result.state == SignalState.STOP
    assert result.reason.startswith('RED DOMINANT')
    assert result.evidence.red_ratio > result.evidence.green_ratio


def test_equal_red_green_ratio_fails_safe_to_stop() -> None:
    detector = _detector()
    result = detector.analyze(_mixed_frame(red_radius=45, green_radius=45))
    assert result.state == SignalState.STOP
    assert result.reason == 'RED/GREEN TIE - SAFE STOP'


def test_signal_loss_falls_back_to_stop() -> None:
    detector = _detector()
    green = _frame((0, 255, 0))
    for _ in range(3):
        result = detector.analyze(green)
    assert result.state == SignalState.GO
    assert detector.analyze(_frame()).state == SignalState.GO
    result = detector.analyze(_frame())
    assert result.state == SignalState.STOP
    assert result.reason == 'NO SIGNAL - SAFE STOP'


def test_green_can_require_red_transition() -> None:
    detector = _detector(require_red_before_green=True)
    green = _frame((0, 255, 0))
    assert detector.analyze(green).reason == 'WAITING FOR RED FIRST'
    for _ in range(5):
        detector.analyze(_frame((0, 0, 255)))
    for _ in range(3):
        result = detector.analyze(green)
    assert result.state == SignalState.GO


def test_left_arrow_displays_turn_left_after_confirmation() -> None:
    detector = _detector()
    arrow = _arrow_frame(left=True)
    for _ in range(3):
        result = detector.analyze(arrow)
    assert result.state == SignalState.TURN_LEFT
    assert result.reason == 'GREEN DETECTED -> LEFT ARROW CONFIRMED'
    assert result.evidence.left_arrow_active


def test_green_dominant_left_arrow_ignores_small_red_noise() -> None:
    detector = _detector()
    arrow = _arrow_frame(left=True, red_noise_radius=20)
    for _ in range(3):
        result = detector.analyze(arrow)
    assert result.state == SignalState.TURN_LEFT
    assert result.reason.startswith(
        'GREEN DOMINANT -> LEFT ARROW CONFIRMED')


def test_right_arrow_is_not_mislabeled_as_turn_left() -> None:
    detector = _detector()
    arrow = _arrow_frame(left=False)
    for _ in range(3):
        result = detector.analyze(arrow)
    assert result.state == SignalState.GO
    assert not result.evidence.left_arrow_active


def test_split_line_left_arrow_displays_turn_left() -> None:
    detector = _detector()
    arrow = _split_line_arrow_frame(left=True)
    for _ in range(3):
        result = detector.analyze(arrow)
    assert result.state == SignalState.TURN_LEFT
    assert result.evidence.left_arrow_active


def test_split_line_right_arrow_is_not_turn_left() -> None:
    detector = _detector()
    arrow = _split_line_arrow_frame(left=False)
    for _ in range(3):
        result = detector.analyze(arrow)
    assert result.state == SignalState.GO
    assert not result.evidence.left_arrow_active
