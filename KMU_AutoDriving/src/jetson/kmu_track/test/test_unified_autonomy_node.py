"""Focused tests for the unified autonomy node's automatic drive gate."""

import json
import time
from types import SimpleNamespace

from kmu_track.unified_autonomy_node import UnifiedAutonomyNode
from kmu_track.unified_autonomy_core import PlannerMode


class _Publisher:
    def __init__(self) -> None:
        self.values = []

    def publish(self, message) -> None:
        self.values.append(bool(message.data))


def _node_with_gate(*, lane_ready: bool = False):
    node = UnifiedAutonomyNode.__new__(UnifiedAutonomyNode)
    node.serial_ready = False
    node.operator_armed_pub = _Publisher()
    node.operator_deadman_pub = _Publisher()
    node.selector = SimpleNamespace(mode=PlannerMode.LANE)
    node.lane_command_timeout_sec = 0.25
    node.ire_lane_command = (0, 0)
    node._ire_lane_command_at = None
    node._ire_lane_status_at = None
    node._ire_lane_gate_active = False
    node._ire_lane_gate_reason = 'waiting_for_status'
    if lane_ready:
        now = time.monotonic()
        node._ire_lane_command_at = now
        node._ire_lane_status_at = now
        node._ire_lane_gate_active = True
        node._ire_lane_gate_reason = 'running'
    return node


def test_drive_gate_stays_off_until_serial_is_ready() -> None:
    node = _node_with_gate()

    node._publish_drive_gate()

    assert node.operator_deadman_pub.values == [False]
    assert node.operator_armed_pub.values == [False]


def test_serial_ready_automatically_enables_arm_and_deadman() -> None:
    node = _node_with_gate(lane_ready=True)

    node._on_serial_ready(SimpleNamespace(data=True))

    assert node.serial_ready is True
    assert node.operator_deadman_pub.values == [True]
    assert node.operator_armed_pub.values == [True]


def test_serial_disconnect_automatically_disables_drive_gate() -> None:
    node = _node_with_gate(lane_ready=True)
    node._on_serial_ready(SimpleNamespace(data=True))

    node._on_serial_ready(SimpleNamespace(data=False))

    assert node.serial_ready is False
    assert node.operator_deadman_pub.values == [True, False]
    assert node.operator_armed_pub.values == [True, False]


def test_drive_gate_is_inactive_when_publishers_are_disabled() -> None:
    node = UnifiedAutonomyNode.__new__(UnifiedAutonomyNode)
    node.serial_ready = True
    node.operator_armed_pub = None
    node.operator_deadman_pub = None

    node._publish_drive_gate()


def test_serial_ready_waits_for_a_fresh_ire_lane_command() -> None:
    node = _node_with_gate()

    node._on_serial_ready(SimpleNamespace(data=True))

    assert node.operator_deadman_pub.values == [False]
    assert node.operator_armed_pub.values == [False]


def test_ire_command_and_running_status_enable_lane_gate() -> None:
    node = _node_with_gate()
    node._on_ire_lane_command(SimpleNamespace(data=[550, -120]))
    node._on_ire_lane_status(SimpleNamespace(data=json.dumps({
        'publish': True,
        'gate_reason': 'running',
    })))

    assert node.ire_lane_command == (550, -120)
    assert node._ire_lane_ready(time.monotonic()) is True


def test_cone_mode_does_not_depend_on_ire_lane_gate() -> None:
    node = _node_with_gate()
    node.selector.mode = PlannerMode.CONE

    node._on_serial_ready(SimpleNamespace(data=True))

    assert node.operator_deadman_pub.values == [True]
    assert node.operator_armed_pub.values == [True]
