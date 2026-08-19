"""ROS 2 adapter for the hardware-independent mission FSM."""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String, UInt8

from kmu_track.fsm import MissionFSM


class MissionManagerNode(Node):
    """Own mission state and publish safety decisions."""

    def __init__(self) -> None:
        super().__init__('mission_manager')
        self.declare_parameter('target_laps', 3)
        self.declare_parameter('mission_timeout_sec', 235.0)
        self.declare_parameter('stall_warning_sec', 3.0)
        self.declare_parameter('stop_guard_sec', 55.0)
        self.declare_parameter('stopped_speed_mps', 0.03)
        self.declare_parameter('update_rate_hz', 20.0)

        now = self._now_sec()
        self.fsm = MissionFSM(
            target_laps=self.get_parameter('target_laps').value,
            mission_timeout_sec=self.get_parameter('mission_timeout_sec').value,
            stall_warning_sec=self.get_parameter('stall_warning_sec').value,
            stop_guard_sec=self.get_parameter('stop_guard_sec').value,
            stopped_speed_mps=self.get_parameter('stopped_speed_mps').value,
            now=now,
        )
        self.speed_mps = 0.0

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.lap_pub = self.create_publisher(UInt8, '/mission/lap', state_qos)
        self.elapsed_pub = self.create_publisher(Float32, '/mission/elapsed_sec', 10)
        self.remaining_pub = self.create_publisher(Float32, '/mission/remaining_sec', 10)
        self.status_pub = self.create_publisher(String, '/mission/status_json', 10)
        self.stall_pub = self.create_publisher(Bool, '/safety/stall_detected', 10)
        self.recovery_pub = self.create_publisher(
            Bool, '/safety/recovery_requested', 10)
        self.stop_pub = self.create_publisher(Bool, '/vehicle/stop_requested', 10)

        self.create_subscription(String, '/mission/event', self._on_event, 20)
        self.create_subscription(
            Bool, '/perception/start_signal', self._on_start_signal, 10)
        self.create_subscription(
            Bool, '/perception/left_signal', self._on_left_signal, 10)
        self.create_subscription(
            Float32, '/vehicle/speed_mps', self._on_speed, 20)

        rate = float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._on_timer)
        self.get_logger().info('Mission manager ready in WAIT_START')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _apply_event(self, event: str) -> None:
        previous = self.fsm.state
        accepted = self.fsm.handle_event(event, self._now_sec())
        if accepted:
            self.get_logger().info(
                f'event={event}: {previous.value} -> {self.fsm.state.value}')
        else:
            self.get_logger().warn(
                f'ignored event={event} while state={previous.value}')

    def _on_event(self, msg: String) -> None:
        self._apply_event(msg.data)

    def _on_start_signal(self, msg: Bool) -> None:
        if msg.data and self.fsm.state.value == 'WAIT_START':
            self._apply_event('start_signal')

    def _on_left_signal(self, msg: Bool) -> None:
        if msg.data and self.fsm.state.value == 'SHORTCUT_WAIT':
            self._apply_event('left_signal')

    def _on_speed(self, msg: Float32) -> None:
        self.speed_mps = float(msg.data)

    def _on_timer(self) -> None:
        snapshot = self.fsm.tick(self._now_sec(), self.speed_mps)
        self.state_pub.publish(String(data=snapshot.state.value))
        self.lap_pub.publish(UInt8(data=snapshot.completed_laps))
        self.elapsed_pub.publish(Float32(data=float(snapshot.elapsed_sec)))
        self.remaining_pub.publish(Float32(data=float(snapshot.remaining_sec)))
        self.stall_pub.publish(Bool(data=snapshot.stall_detected))
        self.recovery_pub.publish(Bool(data=snapshot.recovery_requested))
        self.stop_pub.publish(Bool(data=snapshot.stop_requested))
        status = {
            'state': snapshot.state.value,
            'completed_laps': snapshot.completed_laps,
            'elapsed_sec': round(snapshot.elapsed_sec, 3),
            'remaining_sec': round(snapshot.remaining_sec, 3),
            'stopped_sec': round(snapshot.stopped_sec, 3),
            'abort_reason': snapshot.abort_reason,
            'last_event': snapshot.last_event,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
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
