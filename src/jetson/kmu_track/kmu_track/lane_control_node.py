"""ROS adapter for the guarded lane-following controller."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32MultiArray, String

from kmu_track.lane_control_core import (
    LaneControlConfig,
    LaneControlController,
    actuation_gate_active,
)


class LaneControlNode(Node):
    """Subscribe to lane state and publish preview or live drive commands."""

    def __init__(self) -> None:
        super().__init__('lane_control')
        defaults = LaneControlConfig()
        self.declare_parameter('enabled', defaults.enabled)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('hardware_confirmed', False)
        self.declare_parameter('steering_only', defaults.steering_only)
        self.declare_parameter(
            'require_serial_ready', defaults.require_serial_ready)
        self.declare_parameter(
            'ignore_mission_state', defaults.ignore_mission_state)
        self.declare_parameter('active_states', list(defaults.active_states))
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('command_topic', '')
        self.declare_parameter('manage_serial_gate', False)
        for name in (
            'error_timeout_sec',
            'min_confidence',
            'error_jump_limit',
            'error_lpf_alpha',
            'kp',
            'kd',
            'ki',
            'k_heading',
            'integral_limit',
            'steer_epsilon',
            'lost_hold_sec',
            'lost_decay_sec',
            'lost_coast_sec',
            'arming_neutral_sec',
            'speed_timeout_sec',
            'stopped_speed_mps',
            'center_fallback_delay_sec',
            'steering_gain',
            'steering_gain_right',
            'steering_curve_exponent',
            'full_lock_threshold',
            'rolling_start_sec',
        ):
            self.declare_parameter(name, float(getattr(defaults, name)))
        for name in (
            'steering_sign',
            'max_counts',
            'deadband_counts',
            'deadband_counts_left',
            'deadband_counts_right',
            'max_counts_left',
            'max_counts_right',
            'max_delta_counts_per_tick',
            'rolling_start_max_counts',
            'recover_frames',
            'throttle_base',
            'throttle_curve',
            'throttle_lost',
            'gear',
        ):
            self.declare_parameter(name, int(getattr(defaults, name)))

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.manage_serial_gate = bool(
            self.get_parameter('manage_serial_gate').value)
        enabled = bool(self.get_parameter('enabled').value)
        if (
            not self.dry_run
            and not bool(self.get_parameter('hardware_confirmed').value)
        ):
            raise RuntimeError('live output requires hardware_confirmed:=true')
        if self.dry_run and self.manage_serial_gate:
            raise RuntimeError('dry-run output cannot manage the serial gate')

        config_values = {}
        for field_name in LaneControlConfig.__dataclass_fields__:
            value = self.get_parameter(field_name).value
            if field_name == 'active_states':
                value = tuple(str(item).upper() for item in value)
            config_values[field_name] = value
        if self.dry_run:
            config_values['require_serial_ready'] = False
        self.controller = LaneControlController(
            LaneControlConfig(**config_values), clock=self._now_sec)

        command_topic = str(self.get_parameter('command_topic').value).strip()
        if not command_topic:
            command_topic = (
                '/rc_car/drive_cmd_preview'
                if self.dry_run else '/rc_car/drive_cmd'
            )
        self.command_pub = self.create_publisher(
            Int32MultiArray, command_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/vehicle/lane_control_status', 10)
        self.arm_pub = None
        self.deadman_pub = None
        self._actuation_gate_active = False
        if self.manage_serial_gate:
            self.arm_pub = self.create_publisher(
                Bool, '/rc_car/operator_armed', 10)
            self.deadman_pub = self.create_publisher(
                Bool, '/rc_car/operator_deadman', 10)
        self._lane_valid = False
        self._lane_confidence = 0.0

        self.create_subscription(
            Bool, '/lane/valid', self._on_lane_valid, 10)
        self.create_subscription(
            Float32, '/lane/confidence', self._on_lane_confidence, 10)
        self.create_subscription(
            Float32, '/lane/center_error', self._on_lane_error, 10)
        self.create_subscription(
            Float32, '/lane/heading_error', self._on_heading, 10)
        mission_qos = QoSProfile(depth=1)
        mission_qos.reliability = ReliabilityPolicy.RELIABLE
        mission_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/mission/state', self._on_mission_state, mission_qos)
        self.create_subscription(
            Bool,
            '/vehicle/stop_requested',
            lambda msg: self.controller.update_stop_requested(msg.data),
            10,
        )
        self.create_subscription(
            Float32,
            '/vehicle/speed_mps',
            lambda msg: self.controller.update_speed(msg.data),
            10,
        )
        self.create_subscription(
            Bool,
            '/rc_car/serial_connected',
            lambda _msg: None,
            10,
        )
        serial_ready_qos = QoSProfile(depth=1)
        serial_ready_qos.reliability = ReliabilityPolicy.RELIABLE
        serial_ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            '/rc_car/serial_ready',
            lambda msg: self.controller.update_serial_ready(msg.data),
            serial_ready_qos,
        )
        self.create_subscription(
            Bool,
            '/vehicle/estop_latched',
            lambda msg: self.controller.update_estop_latched(msg.data),
            serial_ready_qos,
        )
        self.create_subscription(
            Bool,
            '/rc_car/command_stale',
            lambda msg: self.controller.update_command_stale(msg.data),
            10,
        )
        rate = float(self.get_parameter('control_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        mode = 'PREVIEW' if self.dry_run else 'LIVE'
        self.get_logger().info(
            f'Lane controller ready: mode={mode}, enabled={enabled}, '
            f'topic={command_topic}, steering_sign='
            f'{self.controller.config.steering_sign} (hardware unverified)')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_lane_valid(self, message: Bool) -> None:
        self._lane_valid = bool(message.data)

    def _on_lane_confidence(self, message: Float32) -> None:
        self._lane_confidence = float(message.data)

    def _on_lane_error(self, message: Float32) -> None:
        self.controller.update_lane_sample(
            message.data,
            self._lane_valid,
            self._lane_confidence,
        )

    def _on_heading(self, message: Float32) -> None:
        self.controller.update_heading(message.data)

    def _on_mission_state(self, message: String) -> None:
        self.controller.update_mission_state(message.data)

    def _publish_command(self, output) -> None:
        message = Int32MultiArray()
        message.data = [output.throttle, output.steering]
        self.command_pub.publish(message)

    def _publish_actuation_gate(self, output) -> None:
        if self.arm_pub is None or self.deadman_pub is None:
            return
        active = actuation_gate_active(
            output, serial_ready=self.controller.serial_ready)
        message = Bool(data=active)
        if active:
            self.arm_pub.publish(message)
            self.deadman_pub.publish(message)
        else:
            # Match the verified manual sequence: release deadman, then disarm.
            self.deadman_pub.publish(message)
            self.arm_pub.publish(message)
        if active != self._actuation_gate_active:
            state = 'ACTIVE' if active else 'STOPPED'
            self.get_logger().info(
                f'Autonomous serial gate {state}: {output.gate_reason}')
        self._actuation_gate_active = active

    def _on_timer(self) -> None:
        output = self.controller.command()
        if output.publish:
            self._publish_command(output)
        self._publish_actuation_gate(output)
        status = output.status()
        status.update({
            'mode': 'preview' if self.dry_run else 'live',
            'mission_state': self.controller.mission_state,
            'confidence': self.controller.confidence,
            'throttle': output.throttle,
            'gear': output.gear,
            'steering_sign': self.controller.config.steering_sign,
        })
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self) -> bool:
        if self.controller.config.enabled and rclpy.ok(context=self.context):
            neutral = Int32MultiArray(data=[0, 0])
            for _ in range(3):
                self.command_pub.publish(neutral)
                time.sleep(0.05)
        if self.deadman_pub is not None and self.arm_pub is not None:
            disabled = Bool(data=False)
            self.deadman_pub.publish(disabled)
            self.arm_pub.publish(disabled)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the lane control ROS node."""
    rclpy.init(args=args)
    node = LaneControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
