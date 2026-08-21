"""LiDAR-only cone-finish tests."""

from lidar_cone_planner.cone_end_core import ConeEndConfig, ConeEndDetector


LEFT = [(0.5, 0.4)]
RIGHT = [(0.5, -0.4)]
BOTH = LEFT + RIGHT


def make_detector(minimum_duration: float = 0.0) -> ConeEndDetector:
    detector = ConeEndDetector(ConeEndConfig(
        empty_scans=5,
        min_mode_duration_sec=minimum_duration,
    ))
    detector.enter(0.0)
    return detector


def test_one_missing_side_does_not_finish() -> None:
    detector = make_detector()
    assert not detector.update(BOTH, 0.1)
    for index in range(10):
        assert not detector.update(LEFT, 0.2 + index * 0.1)
    assert detector.consecutive_empty == 0


def test_both_sides_need_exactly_five_empty_scans() -> None:
    detector = make_detector()
    detector.update(BOTH, 0.1)
    for index in range(4):
        assert not detector.update([], 0.2 + index * 0.1)
    assert detector.update([], 0.6)


def test_never_observed_cones_cannot_finish() -> None:
    detector = make_detector()
    for index in range(20):
        assert not detector.update([], index * 0.1)


def test_minimum_mode_duration_blocks_early_finish() -> None:
    detector = make_detector(minimum_duration=2.0)
    detector.update(BOTH, 0.1)
    for index in range(8):
        assert not detector.update([], 0.2 + index * 0.1)
    assert detector.update([], 2.0)
