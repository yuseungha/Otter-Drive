"""Deterministic no-hardware scenario for integration testing."""

from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


@dataclass(frozen=True)
class ScenarioAction:
    """One scheduled demo publication."""

    at_sec: float
    kind: str
    value: object


class ScenarioPlayerNode(Node):
    """Publish a complete three-lap nominal mission sequence."""

    def __init__(self) -> None:
        super().__init__('scenario_player')
        self.declare_parameter('target_laps', 3)
        self.declare_parameter('step_sec', 0.45)
        self.event_pub = self.create_publisher(String, '/mission/event', 10)
        self.start_pub = self.create_publisher(Bool, '/perception/start_signal', 10)
        self.left_pub = self.create_publisher(Bool, '/perception/left_signal', 10)
        self.speed_pub = self.create_publisher(Float32, '/vehicle/speed_mps', 10)
        self.actions = self._make_actions(
            int(self.get_parameter('target_laps').value),
            float(self.get_parameter('step_sec').value),
        )
        self.next_action = 0
        self.started_at = self.get_clock().now()
        self.timer = self.create_timer(0.02, self._on_timer)
        self.get_logger().info('Nominal demo scenario started')

    @staticmethod
    def _make_actions(target_laps: int, step: float) -> List[ScenarioAction]:
        actions: List[ScenarioAction] = []
        at = 0.6
        actions.append(ScenarioAction(at, 'start', True))
        at += step
        actions.append(ScenarioAction(at, 'start', False))
        for _ in range(target_laps):
            for event in (
                'cone_complete',
                'static_obstacle_zone',
                'fixed_obstacle_clear',
                'overtake_complete',
            ):
                at += step
                actions.append(ScenarioAction(at, 'event', event))
            at += step
            actions.append(ScenarioAction(at, 'left', True))
            at += step
            actions.append(ScenarioAction(at, 'left', False))
            at += step
            actions.append(ScenarioAction(at, 'event', 'shortcut_complete'))
            at += step
            actions.append(ScenarioAction(at, 'event', 'lap_complete'))
        at += step
        actions.append(ScenarioAction(at, 'speed', 0.0))
        return actions

    def _publish_action(self, action: ScenarioAction) -> None:
        if action.kind == 'event':
            self.event_pub.publish(String(data=str(action.value)))
        elif action.kind == 'start':
            self.start_pub.publish(Bool(data=bool(action.value)))
        elif action.kind == 'left':
            self.left_pub.publish(Bool(data=bool(action.value)))
        elif action.kind == 'speed':
            self.speed_pub.publish(Float32(data=float(action.value)))

    def _on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
        moving = self.next_action < len(self.actions)
        if moving:
            self.speed_pub.publish(Float32(data=0.5))
        while (
            self.next_action < len(self.actions)
            and elapsed >= self.actions[self.next_action].at_sec
        ):
            self._publish_action(self.actions[self.next_action])
            self.next_action += 1
        if self.next_action >= len(self.actions):
            self.speed_pub.publish(Float32(data=0.0))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioPlayerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
