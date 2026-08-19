"""Synthetic scan-line and tracking tests for lane feature extraction."""

import numpy as np

from kmu_track.lane_feature_core import (
    LaneFeatureTracker,
    find_scanline_candidates,
)


HEIGHT = 240
WIDTH = 640
LEFT_BOX = [40, 120, 335, 239]
RIGHT_BOX = [305, 120, 600, 239]


def _gray_frame(lines=(), background=60, line_value=145):
    image = np.full((HEIGHT, WIDTH), background, dtype=np.uint8)
    for x, width in lines:
        low = int(round(x - width / 2))
        high = int(round(x + width / 2))
        image[:, low:high] = line_value
    return image


def _tracker(**overrides):
    parameters = {
        'lane_half_width_px': 220.0,
    }
    parameters.update(overrides)
    return LaneFeatureTracker(**parameters)


def test_scanline_accepts_local_contrast_and_measured_width() -> None:
    image = _gray_frame([(320, 16)])
    candidates = find_scanline_candidates(
        image, int(HEIGHT * 0.80), boxes=(LEFT_BOX, RIGHT_BOX))
    assert len(candidates) == 1
    assert abs(candidates[0].x - 320) <= 1
    assert 14 <= candidates[0].width_px <= 17
    assert candidates[0].contrast >= 80


def test_scanline_rejects_too_narrow_and_too_wide_runs() -> None:
    image = _gray_frame([(100, 2), (500, 50)])
    candidates = find_scanline_candidates(
        image, int(HEIGHT * 0.90), boxes=(LEFT_BOX, RIGHT_BOX))
    assert candidates == []


def test_scanline_rejects_missing_low_high_low_pair() -> None:
    image = _gray_frame([(320, 16)])
    image[:, 328:] = 145
    candidates = find_scanline_candidates(
        image, int(HEIGHT * 0.80), boxes=(LEFT_BOX, RIGHT_BOX))
    assert candidates == []


def test_target_priority_uses_both_edges_before_dashed() -> None:
    tracker = _tracker()
    geometry = tracker.process(
        _gray_frame([(100, 10), (320, 10), (540, 10)]),
        LEFT_BOX,
        RIGHT_BOX,
    )
    assert geometry['target_source'] == 'BOTH_EDGES'
    assert abs(geometry['lane_center_x'] - 320) < 2
    assert geometry['feature_validated']


def test_target_priority_falls_through_dashed_one_edge_and_box() -> None:
    dashed = _tracker().process(
        _gray_frame([(320, 10)]), LEFT_BOX, RIGHT_BOX)
    assert dashed['target_source'] == 'DASHED'

    one_edge = _tracker().process(
        _gray_frame([(100, 10)]), LEFT_BOX, RIGHT_BOX)
    assert one_edge['target_source'] == 'ONE_EDGE'
    assert one_edge['used_single_side']

    box = _tracker().process(_gray_frame([]), LEFT_BOX, RIGHT_BOX)
    assert box['target_source'] == 'BOX'
    assert not box['feature_validated']


def test_prediction_marks_dashed_gap_and_lambda_roi_expires() -> None:
    tracker = _tracker(
        lambda_roi_decay_frames=1,
        max_predicted_frames=3,
    )
    tracker.process(
        _gray_frame([(100, 10), (320, 10), (540, 10)]),
        LEFT_BOX,
        RIGHT_BOX,
    )
    predicted = tracker.process(_gray_frame([]), LEFT_BOX, RIGHT_BOX)
    assert predicted['predicted']
    assert all(row['source'] == 'predicted' for row in predicted['scan_rows'])

    expired = tracker.process(_gray_frame([]), LEFT_BOX, RIGHT_BOX)
    assert not expired['lambda_roi_active']


def test_center_consistency_warning_is_not_silently_averaged() -> None:
    geometry = _tracker().process(
        _gray_frame([(100, 10), (365, 10), (540, 10)]),
        LEFT_BOX,
        RIGHT_BOX,
    )
    assert geometry['target_source'] == 'BOTH_EDGES'
    assert geometry['consistency_warning']
    assert geometry['center_consistency_gap'] > 0.08


def test_long_prediction_is_clamped_inside_the_image() -> None:
    tracker = _tracker(max_predicted_frames=50)
    tracker.process(
        _gray_frame([(100, 10), (320, 10), (540, 10)]),
        LEFT_BOX,
        RIGHT_BOX,
    )
    shifted = tracker.process(
        _gray_frame([(140, 10), (360, 10), (580, 10)]),
        LEFT_BOX,
        RIGHT_BOX,
    )
    assert shifted['lane_center_x'] is not None
    for _ in range(40):
        geometry = tracker.process(_gray_frame([]), LEFT_BOX, RIGHT_BOX)
    for row in geometry['scan_rows']:
        for point in row['points'].values():
            assert 0.0 <= point['x'] < WIDTH
