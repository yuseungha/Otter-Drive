"""Tests for the perception-only three-mode decision machine."""

import unittest

from kmu_track.mode_decision_core import (
    ConeModeConfig,
    DecisionMode,
    ModeDecisionMachine,
    nearest_cone_gate,
)


GATE_065 = ((0.70, 0.325), (0.72, -0.325))
GATE_080 = ((0.60, 0.40), (0.62, -0.40))


class TestFixedConeGate(unittest.TestCase):
    def test_measured_course_width_range_is_accepted(self) -> None:
        config = ConeModeConfig()
        self.assertIsNotNone(nearest_cone_gate(GATE_065, config))
        self.assertIsNotNone(nearest_cone_gate(GATE_080, config))

    def test_unrelated_narrow_pair_is_rejected(self) -> None:
        gate = ((0.50, 0.20), (0.50, -0.20))
        self.assertIsNone(nearest_cone_gate(gate, ConeModeConfig()))


class TestModeDecisionMachine(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 10.0
        self.machine = ModeDecisionMachine(
            ConeModeConfig(enter_confirm_frames=2, exit_missing_sec=0.8),
            obstacle_clear_sec=0.6,
            clock=lambda: self.now,
        )

    def test_two_distinct_cone_observations_enter_cone_mode(self) -> None:
        mode = self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=1)
        self.assertEqual(mode, DecisionMode.LANE)
        mode = self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=2)
        self.assertEqual(mode, DecisionMode.CONE)

    def test_one_scan_is_not_counted_twice_by_timer(self) -> None:
        self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=7)
        mode = self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=7)
        self.assertEqual(mode, DecisionMode.LANE)
        self.assertEqual(self.machine.enter_count, 1)

    def test_cone_gate_has_priority_over_obstacle(self) -> None:
        self.machine.update(
            GATE_065,
            cone_path_valid=True,
            obstacle_detected=True,
            cone_observation_id=1,
        )
        mode = self.machine.update(
            GATE_065,
            cone_path_valid=True,
            obstacle_detected=True,
            cone_observation_id=2,
        )
        self.assertEqual(mode, DecisionMode.CONE)

    def test_missing_cone_lines_return_to_lane(self) -> None:
        self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=1)
        self.machine.update(
            GATE_065, cone_path_valid=True, cone_observation_id=2)
        self.now += 0.79
        self.machine.update((), cone_path_valid=False)
        self.assertEqual(self.machine.mode, DecisionMode.CONE)
        self.now += 0.02
        self.machine.update((), cone_path_valid=False)
        self.assertEqual(self.machine.mode, DecisionMode.LANE)

    def test_obstacle_mode_clears_after_hysteresis(self) -> None:
        mode = self.machine.update(
            (), cone_path_valid=False, obstacle_detected=True)
        self.assertEqual(mode, DecisionMode.OBSTACLE_AVOID)
        self.now += 0.59
        self.machine.update(
            (), cone_path_valid=False, obstacle_detected=False)
        self.assertEqual(self.machine.mode, DecisionMode.OBSTACLE_AVOID)
        self.now += 0.02
        self.machine.update(
            (), cone_path_valid=False, obstacle_detected=False)
        self.assertEqual(self.machine.mode, DecisionMode.LANE)


if __name__ == '__main__':
    unittest.main()
