"""Synthetic tests for the IRE center-priority lane planner."""

import numpy as np

from kmu_ire_track.ire_segmentation_lane_core import (
    SegmentationInstance,
    SegmentationLaneConfig,
    SegmentationLanePlanner,
    normalized_roi_bounds,
)


HEIGHT = 240
WIDTH = 320


def test_camera_road_roi_pixel_bounds() -> None:
    assert normalized_roi_bounds(
        (1080, 1920, 3), 0.12, 0.43, 0.88, 1.0
    ) == (230, 464, 1690, 1080)


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


def test_center_marking_overrides_two_boundary_midpoint() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('lane', _line_mask(lambda _y: 60.0)),
        _instance('lane', _line_mask(lambda _y: 260.0)),
        _instance('center', _line_mask(lambda _y: 230.0), 0.9),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['target_source'] == 'CENTER_MARKING'
    assert result['center_error'] > 0.35
    assert all(
        row['target_source'] == 'CENTER_MARKING'
        for row in result['scan_rows']
    )


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


def test_short_center_marking_gap_reuses_last_planned_curve() -> None:
    planner = SegmentationLanePlanner(SegmentationLaneConfig(
        memory_max_frames=2,
        memory_confidence_decay=0.90,
    ))
    detected = planner.plan([
        _instance('center', _line_mask(lambda y: 145.0 + 20.0 * y, 7)),
    ], (HEIGHT, WIDTH))

    first_gap = planner.plan([], (HEIGHT, WIDTH))
    second_gap = planner.plan([], (HEIGHT, WIDTH))
    expired = planner.plan([], (HEIGHT, WIDTH))

    assert first_gap['valid']
    assert first_gap['memory_active']
    assert first_gap['memory_age_frames'] == 1
    assert first_gap['target_source'] == 'MEMORY_CENTER_MARKING'
    assert first_gap['center_error'] == detected['center_error']
    assert second_gap['valid']
    assert second_gap['memory_age_frames'] == 2
    assert not expired['valid']


def test_reacquired_center_marking_resets_memory_age() -> None:
    planner = SegmentationLanePlanner()
    instance = _instance(
        'center', _line_mask(lambda y: 145.0 + 20.0 * y, 7))
    planner.plan([instance], (HEIGHT, WIDTH))
    assert planner.plan([], (HEIGHT, WIDTH))['memory_active']

    reacquired = planner.plan([instance], (HEIGHT, WIDTH))

    assert reacquired['valid']
    assert not reacquired['memory_active']
    assert reacquired['memory_age_frames'] == 0


def test_memory_keeps_confidence_until_finite_frame_budget_expires() -> None:
    planner = SegmentationLanePlanner(SegmentationLaneConfig(
        memory_max_frames=24,
        memory_confidence_decay=1.0,
    ))
    detected = planner.plan([
        _instance('center', _line_mask(lambda y: 145.0 + 20.0 * y, 7)),
    ], (HEIGHT, WIDTH))

    remembered = [planner.plan([], (HEIGHT, WIDTH)) for _ in range(24)]
    expired = planner.plan([], (HEIGHT, WIDTH))

    assert all(item['valid'] for item in remembered)
    assert all(
        item['confidence'] == detected['confidence'] for item in remembered)
    assert remembered[-1]['memory_age_frames'] == 24
    assert not expired['valid']


def test_far_preview_clamps_to_visible_path_instead_of_invalidating() -> None:
    mask = _line_mask(lambda y: 145.0 + 20.0 * y, 7)
    mask[:int(HEIGHT * 0.65), :] = 0.0
    planner = SegmentationLanePlanner(SegmentationLaneConfig(
        look_ahead_ratio=0.50,
    ))

    result = planner.plan([_instance('center', mask)], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['look_ahead_clamped']
    assert result['requested_look_ahead_ratio'] == 0.50
    assert result['look_ahead_ratio'] > 0.50


def test_upper_half_marking_is_used_without_lower_frame_roi() -> None:
    mask = _line_mask(lambda y: 145.0 + 20.0 * y, 7)
    mask[int(HEIGHT * 0.48):, :] = 0.0

    result = SegmentationLanePlanner().plan([
        _instance('center', mask),
    ], (HEIGHT, WIDTH))

    assert result['valid']
    assert result['valid_rows'] >= 3
    assert min(row['ratio'] for row in result['scan_rows']) < 0.10
    assert result['look_ahead_clamped']


def test_heading_follows_vehicle_anchored_trajectory() -> None:
    result = SegmentationLanePlanner().plan([
        _instance('center', _line_mask(lambda y: 180.0 + 80.0 * y, 7)),
    ], (HEIGHT, WIDTH))

    # The raw marking leans back toward the left when read far-to-near, but
    # the drivable path starts at vehicle center and initially turns right.
    assert result['valid']
    assert result['center_error'] > 0.20
    assert result['heading_error'] > 0.20
