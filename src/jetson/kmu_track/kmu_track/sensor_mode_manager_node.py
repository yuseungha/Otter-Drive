"""Sole owner of the five-state camera/LiDAR driving mission mode."""

import json
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import SetBool

from kmu_track.drive_mode_core import (
    DriveEvent,
    DriveMode,
    DriveModeMachine,
    RollingConfirmation,
    stamp_is_fresh,
)


class SensorModeManager(Node):
    """Consume perception events and exclusively publish ``/mission/state``."""

    def __init__(self) -> None:
        super().__init__('sensor_mode_manager')
        self.declare_parameter('lane_confirm_window', 5)
        self.declare_parameter('lane_confirm_required', 4)
        self.declare_parameter('lane_reacquire_timeout_sec', 3.0)
        self.declare_parameter('lidar_path_timeout_sec', 4.0)
        self.declare_parameter('perception_result_max_age_sec', 0.35)
        self.declare_parameter('sensor_timeout_sec', 0.60)
        self.declare_parameter('path_timeout_sec', 3.0)
        self.declare_parameter('subscription_switch_timeout_sec', 3.0)
        self.declare_parameter('startup_grace_sec', 15.0)
        self.declare_parameter('watchdog_rate_hz', 20.0)
        self.declare_parameter('automatic_cone_transition_enabled', False)
        self.declare_parameter('automatic_cone_exit_enabled', True)
        self.declare_parameter('initial_mode', DriveMode.LANE_FOLLOW.value)

        window = int(self.get_parameter('lane_confirm_window').value)
        required = int(self.get_parameter('lane_confirm_required').value)
        self._lane_confirmation = RollingConfirmation(window, required)
        self._max_age = self._positive('perception_result_max_age_sec')
        self._sensor_timeout = self._positive('sensor_timeout_sec')
        self._path_timeout = self._positive('path_timeout_sec')
        self._lidar_path_timeout = self._positive('lidar_path_timeout_sec')
        self._reacquire_timeout = self._positive(
            'lane_reacquire_timeout_sec')
        self._switch_timeout = self._positive(
            'subscription_switch_timeout_sec')
        self._startup_grace = self._positive('startup_grace_sec')
        self._automatic_cone_transition = bool(self.get_parameter(
            'automatic_cone_transition_enabled').value)
        self._automatic_cone_exit = bool(self.get_parameter(
            'automatic_cone_exit_enabled').value)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(
            String, '/mission/state', latched)
        self._event_pub = self.create_publisher(String, '/mission/event', 10)
        self._stop_pub = self.create_publisher(
            Bool, '/vehicle/stop_requested', latched)
        self._status_pub = self.create_publisher(
            String, '/mission/sensor_mode_status', latched)

        self._fsm = DriveModeMachine(
            str(self.get_parameter('initial_mode').value).strip().upper())
        now = time.monotonic()
        self._started_at = now
        self._mode_entered_at = now
        self._camera_active = False
        self._lidar_active = False
        self._camera_heartbeat_at = None
        self._lidar_heartbeat_at = None
        self._lane_path_at = None
        self._cone_path_at = None
        self._fault_reason = ''
        self._motion_ready = False

        self.create_subscription(
            Bool, '/perception/cone_confirmed', self._on_cone_confirmed, 10)
        self.create_subscription(
            Bool, '/perception/cone_finished', self._on_cone_finished, 10)
        self.create_subscription(
            String, '/perception/lane_result', self._on_lane_result, 10)
        self.create_subscription(
            Path, '/perception/cone_path', self._on_cone_path, 10)
        self.create_subscription(
            Header, '/perception/camera_heartbeat',
            self._on_camera_heartbeat, 10)
        self.create_subscription(
            Header, '/perception/lidar_heartbeat',
            self._on_lidar_heartbeat, 10)
        self.create_subscription(
            Bool, '/perception/camera_subscription_active',
            self._on_camera_active, latched)
        self.create_subscription(
            Bool, '/perception/lidar_subscription_active',
            self._on_lidar_active, latched)
        self.create_service(
            SetBool,
            'sensor_mode_manager/set_cone_mode',
            self._on_set_cone_mode,
        )

        rate = self._positive('watchdog_rate_hz')
        self.create_timer(1.0 / rate, self._watchdog)
        self.create_timer(0.5, self._publish_status)
        self._publish_mode('startup')

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be > 0')
        return value

    @staticmethod
    def _stamp_ns(header: Header) -> int:
        return (
            int(header.stamp.sec) * 1_000_000_000
            + int(header.stamp.nanosec)
        )

    def _header_is_fresh(self, header: Header) -> bool:
        return stamp_is_fresh(
            self._stamp_ns(header), self.get_clock().now().nanoseconds,
            self._max_age)

    @property
    def mode(self) -> DriveMode:
        return self._fsm.mode

    def _request_event(self, event: DriveEvent, reason: str) -> bool:
        previous = self.mode
        current = self._fsm.apply(event)
        if current == previous:
            return False
        self._fault_reason = reason if current == DriveMode.SAFE_STOP else ''
        self._mode_entered_at = time.monotonic()
        self._lane_confirmation.reset()
        # Old paths/commands are invalid across every mode boundary.
        self._lane_path_at = None
        self._cone_path_at = None
        self._camera_heartbeat_at = None
        self._lidar_heartbeat_at = None
        self._motion_ready = (
            current == DriveMode.CONE_SLALOM
            and event == DriveEvent.VALID_CONE_PATH
        ) or (
            current == DriveMode.LANE_FOLLOW
            and event == DriveEvent.LANE_STABLE
        )
        self._publish_mode(reason)
        return True

    def _on_set_cone_mode(self, request, response):
        """Request a legal cone-mode FSM transition without forcing state."""
        if request.data:
            if self.mode in {DriveMode.CONE_INIT, DriveMode.CONE_SLALOM}:
                response.success = True
                response.message = f'already in {self.mode.value}'
                return response
            if self.mode != DriveMode.LANE_FOLLOW:
                response.success = False
                response.message = (
                    f'cannot enter cone mode from {self.mode.value}')
                return response
            accepted = self._request_event(
                DriveEvent.CONE_CONFIRMED, 'manual_cone_mode_request')
        else:
            if self.mode in {DriveMode.LANE_FOLLOW, DriveMode.LANE_REACQUIRE}:
                response.success = True
                response.message = f'already in {self.mode.value}'
                return response
            if self.mode == DriveMode.CONE_INIT:
                accepted = self._request_event(
                    DriveEvent.CONE_CANCELLED, 'manual_cone_mode_cancel')
            elif self.mode == DriveMode.CONE_SLALOM:
                accepted = self._request_event(
                    DriveEvent.CONE_FINISHED, 'manual_cone_mode_exit')
            else:
                response.success = False
                response.message = (
                    f'cannot leave cone mode from {self.mode.value}')
                return response

        response.success = bool(accepted)
        response.message = (
            f'transitioned to {self.mode.value}'
            if accepted else f'no legal transition from {self.mode.value}')
        return response

    def _publish_mode(self, reason: str) -> None:
        stop = (
            self.mode not in {DriveMode.LANE_FOLLOW, DriveMode.CONE_SLALOM}
            or not self._motion_ready
        )
        # Publish stop first so downstream controllers fail closed before they
        # observe the new mode and clear their cached command.
        self._stop_pub.publish(Bool(data=stop))
        self._state_pub.publish(String(data=self.mode.value))
        self._event_pub.publish(String(data=json.dumps({
            'event': reason, 'state': self.mode.value,
            'stamp_ns': self.get_clock().now().nanoseconds,
        })))
        self.get_logger().info(f'mission state={self.mode.value}: {reason}')

    def _on_cone_confirmed(self, message: Bool) -> None:
        if (
            self._automatic_cone_transition
            and message.data
            and self.mode == DriveMode.LANE_FOLLOW
        ):
            self._request_event(DriveEvent.CONE_CONFIRMED, 'cone_confirmed')

    def _on_cone_finished(self, message: Bool) -> None:
        if (
            self._automatic_cone_exit
            and message.data
            and self.mode == DriveMode.CONE_SLALOM
        ):
            self._request_event(DriveEvent.CONE_FINISHED, 'cone_finished')

    def _on_lane_result(self, message: String) -> None:
        try:
            result = json.loads(message.data)
            stamp_ns = int(result['stamp_ns'])
            valid = bool(result['valid'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        now_ns = self.get_clock().now().nanoseconds
        if not stamp_is_fresh(stamp_ns, now_ns, self._max_age):
            return
        if valid:
            self._lane_path_at = time.monotonic()
        if (
            valid
            and self.mode == DriveMode.LANE_FOLLOW
            and not self._motion_ready
        ):
            self._motion_ready = True
            self._stop_pub.publish(Bool(data=False))
        if self.mode == DriveMode.LANE_REACQUIRE:
            if self._lane_confirmation.update(valid):
                self._request_event(DriveEvent.LANE_STABLE, 'lane_stable')

    def _on_cone_path(self, message: Path) -> None:
        if not self._header_is_fresh(message.header):
            return
        valid = len(message.poses) >= 2
        if valid:
            self._cone_path_at = time.monotonic()
        if valid and self.mode == DriveMode.CONE_INIT:
            self._request_event(DriveEvent.VALID_CONE_PATH, 'valid_cone_path')

    def _on_camera_heartbeat(self, message: Header) -> None:
        if self._header_is_fresh(message):
            self._camera_heartbeat_at = time.monotonic()

    def _on_lidar_heartbeat(self, message: Header) -> None:
        if self._header_is_fresh(message):
            self._lidar_heartbeat_at = time.monotonic()

    def _on_camera_active(self, message: Bool) -> None:
        self._camera_active = bool(message.data)
        self._check_interlock()

    def _on_lidar_active(self, message: Bool) -> None:
        self._lidar_active = bool(message.data)
        self._check_interlock()

    def _check_interlock(self) -> None:
        if self._camera_active and self._lidar_active:
            self._request_event(
                DriveEvent.SENSOR_TIMEOUT, 'perception_subscription_overlap')

    def _watchdog(self) -> None:
        if self.mode == DriveMode.SAFE_STOP:
            return
        now = time.monotonic()
        elapsed = now - self._mode_entered_at
        if (
            self.mode == DriveMode.LANE_FOLLOW
            and not self._motion_ready
            and now - self._started_at < self._startup_grace
        ):
            return

        if self.mode == DriveMode.CONE_INIT:
            if elapsed > self._lidar_path_timeout:
                self._request_event(DriveEvent.TIMEOUT, 'lidar_path_timeout')
                return
        elif self.mode == DriveMode.LANE_REACQUIRE:
            if elapsed > self._reacquire_timeout:
                self._request_event(
                    DriveEvent.TIMEOUT, 'lane_reacquire_timeout')
                return

        expects_camera = self.mode in {
            DriveMode.LANE_FOLLOW, DriveMode.LANE_REACQUIRE}
        expects_lidar = self.mode in {
            DriveMode.CONE_INIT, DriveMode.CONE_SLALOM}
        if elapsed > self._switch_timeout:
            if expects_camera and not self._camera_active:
                self._request_event(
                    DriveEvent.SENSOR_TIMEOUT, 'camera_subscription_timeout')
                return
            if expects_lidar and not self._lidar_active:
                self._request_event(
                    DriveEvent.SENSOR_TIMEOUT, 'lidar_subscription_timeout')
                return
        heartbeat = (
            self._camera_heartbeat_at if expects_camera
            else self._lidar_heartbeat_at)
        # A managed subscription may need the full switch window before its
        # first callback can publish a heartbeat.  Starting the heartbeat
        # deadline at the mode boundary made sensor_timeout shorter than the
        # permitted subscription switch and could fault a healthy handoff.
        first_heartbeat_timeout = self._switch_timeout + self._sensor_timeout
        if ((heartbeat is None and elapsed > first_heartbeat_timeout)
                or (heartbeat is not None
                    and now - heartbeat > self._sensor_timeout)):
            self._request_event(DriveEvent.SENSOR_TIMEOUT, 'sensor_timeout')
            return
        path_at = (
            self._lane_path_at if self.mode == DriveMode.LANE_FOLLOW
            else self._cone_path_at if self.mode == DriveMode.CONE_SLALOM
            else None)
        if self.mode in {DriveMode.LANE_FOLLOW, DriveMode.CONE_SLALOM} and (
            (path_at is None and elapsed > self._path_timeout)
            or (path_at is not None and now - path_at > self._path_timeout)
        ):
            self._request_event(DriveEvent.PATH_TIMEOUT, 'path_timeout')

    def _publish_status(self) -> None:
        self._publish_mode_status()

    def _publish_mode_status(self) -> None:
        self._status_pub.publish(String(data=json.dumps({
            'state': self.mode.value,
            'motion_ready': self._motion_ready,
            'automatic_cone_transition_enabled': (
                self._automatic_cone_transition),
            'automatic_cone_exit_enabled': self._automatic_cone_exit,
            'camera_subscription_active': self._camera_active,
            'lidar_subscription_active': self._lidar_active,
            'interlock_ok': not (self._camera_active and self._lidar_active),
            'fault': self._fault_reason,
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorModeManager()
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
