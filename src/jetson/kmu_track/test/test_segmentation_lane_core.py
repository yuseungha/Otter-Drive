"""Synthetic tests for the segmentation-mask lane planner."""

import numpy as np

from kmu_track.segmentation_lane_core import (
    SegmentationInstance,
    SegmentationLaneConfig,
    SegmentationLanePlanner,
)


HEIGHT = 240
WIDTH = 320


def _line_mask(x_at_y, thickness=5):
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    for y in range(HEIGHT):
        x = int(round(x_at_y(y / (HEIGHT - 1))))
        low = max(0, x - thickness // 2)
        high = min(WIDTH, x + thickness // 2 + 1)
        mask[y, low:high] = 1.0
    return mask


def _instance(name, mask, confidence=0.9):
    return SegmentationInstance(name, confidence, mask)


def test_two_boundaries_produce_centered_valid_path() -> None:
    planner = SegmentationLanePlanner()
    result = planner.plan([
        _instance('lane', _line_mask(lambda _y: 60.0)),
        _instance('lane', _line_mask(lambda _y: 260.0)),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['target_source'] == 'BOTH_LANES'
    assert abs(result['center_error']) < 0.01
    assert result['confidence'] > 0.8


def test_path_shifted_right_has_positive_cross_track_error() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('lane', _line_mask(lambda _y: 100.0)),
        _instance('lane', _line_mask(lambda _y: 300.0)),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['center_error'] > 0.20


def test_left_curve_has_negative_heading_error() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('lane', _line_mask(lambda y: 30.0 + 80.0 * y)),
        _instance('lane', _line_mask(lambda y: 190.0 + 80.0 * y)),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['heading_error'] < -0.15


def test_center_marking_is_a_valid_fallback() -> None:
    result = SegmentationLanePlanner().plan([
        _instance(
            'center', _line_mask(lambda y: 145.0 + 20.0 * y, 7), 0.8),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['target_source'] == 'CENTER_MARKING'
    assert result['confidence'] > 0.5


def test_one_boundary_without_center_is_fail_safe_invalid() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('lane', _line_mask(lambda _y: 60.0)),
    ], (HEIGHT, WIDTH))

    assert not result['valid']
    assert result['center_error'] is None
    assert result['target_source'] == 'NONE'


def test_disagreeing_center_sets_warning_and_reduces_confidence() -> None:
    planner = SegmentationLanePlanner(SegmentationLaneConfig(
        center_consistency_tol=0.05,
    ))
    without_center = planner.plan([
        _instance('lane', _line_mask(lambda _y: 60.0)),
        _instance('lane', _line_mask(lambda _y: 260.0)),
    ], (HEIGHT, WIDTH))
    with_bad_center = planner.plan([
        _instance('lane', _line_mask(lambda _y: 60.0)),
        _instance('lane', _line_mask(lambda _y: 260.0)),
        _instance('center', _line_mask(lambda _y: 230.0)),
    ], (HEIGHT, WIDTH))

    assert with_bad_center['valid']
    assert with_bad_center['consistency_warning']
    assert with_bad_center['confidence'] < without_center['confidence']


def test_unrelated_classes_are_ignored() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('cone', _line_mask(lambda _y: 160.0)),
    ], (HEIGHT, WIDTH))

    assert not result['valid']
    assert result['boundary_instances'] == 0
    assert result['center_instances'] == 0
