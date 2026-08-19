"""Four-panel visual verification window for lane following."""

from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String, UInt32


DEFAULT_THRESHOLDS = (
    0, 0, 130, 180, 80, 255,
    12, 70, 70, 42, 255, 255,
)


class TrackVisualizerNode(Node):
    """Display perception evidence and the controller's published status."""

    def __init__(self) -> None:
        super().__init__('track_visualizer')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('window_name', 'KMU Lane Drive')
        self.declare_parameter('display_width', 1280)
        self.declare_parameter('strip_seconds', 10.0)
        self.declare_parameter('rolling_stats_sec', 5.0)
        self.declare_parameter('capture_dir', 'runs')
        self.declare_parameter('show_mask_panel', False)
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('capture_gate_transitions', False)
        self.declare_parameter(
            'capture_times_sec', [10.0, 60.0, 90.0, 210.0])

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_overlay = None
        self.latest_binary = None
        self.latest_yolo = None
        self.geometry = {}
        self.control_status = {}
        self.actuation_status = {}
        self.mission_state = 'WAIT_START'
        self.lane_valid = False
        self.lane_confidence = 0.0
        self.inference_ms = 0.0
        self.video_time_sec = 0.0
        self.video_duration_sec = 0.0
        self.playback_rate = 1.0
        self.frame_index = 0
        self.paused = False
        self.panel_focus = 0
        self.history = deque()
        self.recording = False
        self.video_writer = None
        self.recording_path = None
        self.last_display = None
        self.auto_captured_times = set()
        self.captured_gate_reasons = set()
        self.pending_capture_label = None
        self.shutdown_at = None

        self.command_pub = self.create_publisher(String, '/video/command', 10)
        self.threshold_pub = self.create_publisher(
            Int32MultiArray, '/lane/hsv_thresholds/set', 10)
        self.estop_pub = self.create_publisher(Bool, '/vehicle/estop', 10)
        self._create_subscriptions()

        self.window_name = str(self.get_parameter('window_name').value)
        self.display_width = max(
            640, int(self.get_parameter('display_width').value))
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            self.window_name, self.display_width,
            int(self.display_width * 9 / 16))
        self.timer = self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            'Visualizer ready: Space pause | []/,. step | +/- rate | '
            'R restart | 1-4 focus | S capture | V record | Q quit')

    def _create_subscriptions(self) -> None:
        image_topic = str(self.get_parameter('image_topic').value)
        for topic, callback in (
            (image_topic, self._on_image),
            ('/lane/lane_overlay', self._on_overlay),
            ('/lane/debug_binary', self._on_binary),
            ('/lane/yolo_debug', self._on_yolo),
        ):
            self.create_subscription(
                Image, topic, callback, qos_profile_sensor_data)
        self.create_subscription(
            String, '/lane/lane_geometry', self._on_geometry, 10)
        self.create_subscription(
            String,
            '/vehicle/lane_control_status',
            self._on_control_status,
            10,
        )
        self.create_subscription(
            String,
            '/vehicle/actuation_status',
            self._on_actuation_status,
            10,
        )
        self.create_subscription(
            String, '/mission/state', self._on_state, 10)
        self.create_subscription(
            Bool, '/lane/valid', self._on_lane_valid, 10)
        self.create_subscription(
            Float32, '/lane/confidence', self._on_lane_confidence, 10)
        self.create_subscription(
            Float32, '/lane/inference_ms', self._on_inference, 10)
        self.create_subscription(
            Float32, '/video/time_sec', self._on_time, 10)
        self.create_subscription(
            UInt32, '/video/frame_index', self._on_frame, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/video/paused', self._on_paused, state_qos)
        self.create_subscription(
            Float32, '/video/playback_rate', self._on_rate, state_qos)
        self.create_subscription(
            Float32, '/video/duration_sec', self._on_duration, state_qos)

    def _on_image(self, message: Image) -> None:
        self.latest_frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')

    def _on_overlay(self, message: Image) -> None:
        self.latest_overlay = self.bridge.imgmsg_to_cv2(message, 'bgr8')

    def _on_binary(self, message: Image) -> None:
        self.latest_binary = self.bridge.imgmsg_to_cv2(message, 'mono8')

    def _on_yolo(self, message: Image) -> None:
        self.latest_yolo = self.bridge.imgmsg_to_cv2(message, 'bgr8')

    def _on_geometry(self, message: String) -> None:
        try:
            self.geometry = json.loads(message.data)
        except json.JSONDecodeError:
            self.geometry = {}

    def _on_control_status(self, message: String) -> None:
        try:
            self.control_status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.history.append({
            'time': now,
            'error': float(self.control_status.get('filtered_error', 0.0)),
            'steering': int(self.control_status.get('steering_counts', 0)),
            'saturated': bool(self.control_status.get('saturated', False)),
            'valid': self.lane_valid,
        })
        gate_reason = str(self.control_status.get('gate_reason', ''))
        capture_reasons = {
            'lane_lost_hold', 'lane_lost_decay', 'lane_lost',
        }
        if (
            bool(self.get_parameter('capture_gate_transitions').value)
            and gate_reason in capture_reasons
            and gate_reason not in self.captured_gate_reasons
        ):
            self.captured_gate_reasons.add(gate_reason)
            self.pending_capture_label = f'synthetic_{gate_reason}'
        keep = max(
            float(self.get_parameter('strip_seconds').value),
            float(self.get_parameter('rolling_stats_sec').value),
        ) + 1.0
        while self.history and now - self.history[0]['time'] > keep:
            self.history.popleft()

    def _on_actuation_status(self, message: String) -> None:
        try:
            self.actuation_status = json.loads(message.data)
        except json.JSONDecodeError:
            return

    def _on_state(self, message: String) -> None:
        self.mission_state = message.data

    def _on_lane_valid(self, message: Bool) -> None:
        self.lane_valid = bool(message.data)

    def _on_lane_confidence(self, message: Float32) -> None:
        self.lane_confidence = float(message.data)

    def _on_inference(self, message: Float32) -> None:
        self.inference_ms = float(message.data)

    def _on_time(self, message: Float32) -> None:
        self.video_time_sec = float(message.data)

    def _on_duration(self, message: Float32) -> None:
        self.video_duration_sec = float(message.data)

    def _on_rate(self, message: Float32) -> None:
        self.playback_rate = float(message.data)

    def _on_frame(self, message: UInt32) -> None:
        self.frame_index = int(message.data)

    def _on_paused(self, message: Bool) -> None:
        self.paused = bool(message.data)

    @staticmethod
    def _put_text(
        image: np.ndarray,
        text: str,
        origin,
        color=(235, 235, 235),
        scale=0.52,
        thickness=1,
    ) -> None:
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
            scale, color, thickness, cv2.LINE_AA)

    def _panel(self, image, label: str, width: int, height: int) -> np.ndarray:
        if image is None:
            panel = np.zeros((height, width, 3), dtype=np.uint8)
            self._put_text(panel, 'Waiting for topic...', (15, height // 2))
        else:
            panel = image.copy()
            if panel.ndim == 2:
                panel = cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)
            panel = cv2.resize(panel, (width, height))
        cv2.rectangle(panel, (0, 0), (width, 28), (18, 18, 18), -1)
        self._put_text(panel, label, (10, 20), (0, 255, 255), 0.55, 1)
        return panel

    def _follow_panel(self, width: int, height: int) -> np.ndarray:
        source = self.latest_overlay
        if source is None:
            source = self.latest_frame
        panel = self._panel(source, '1. FOLLOW VIEW', width, height)
        state = 'VALID' if self.lane_valid else 'LOST'
        color = (50, 230, 50) if self.lane_valid else (30, 30, 255)
        self._put_text(
            panel,
            f'{state} conf {self.lane_confidence:.2f} | '
            f'{self.video_time_sec:.1f}/{self.video_duration_sec:.1f}s | '
            f'{self.playback_rate:.2f}x {"PAUSED" if self.paused else "PLAY"}',
            (10, height - 12), color, 0.48, 1)
        return panel

    def _yolo_panel(self, width: int, height: int) -> np.ndarray:
        show_mask = bool(self.get_parameter('show_mask_panel').value)
        image = self.latest_binary if show_mask else self.latest_yolo
        label = '2. CLEAN MASK' if show_mask else '2. YOLO ROAD REGIONS'
        return self._panel(image, label, width, height)

    @staticmethod
    def _sign_flip_indices(samples) -> set:
        flips = set()
        previous = 0
        for index, sample in enumerate(samples):
            current = int(np.sign(sample['steering']))
            if current and previous and current != previous:
                flips.add(index)
            if current:
                previous = current
        return flips

    def _steering_panel(self, width: int, height: int) -> np.ndarray:
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (width, 28), (18, 18, 18), -1)
        self._put_text(panel, '3. STEERING', (10, 20), (0, 255, 255), 0.55)
        status = self.control_status
        steering = int(status.get('steering_counts', 0))
        max_counts = 600
        deadband = 70
        x0, x1 = 38, width - 38
        gauge_y = 76

        def gauge_x(value):
            ratio = (float(value) + max_counts) / (2.0 * max_counts)
            return int(round(x0 + np.clip(ratio, 0.0, 1.0) * (x1 - x0)))

        cv2.rectangle(
            panel,
            (gauge_x(-deadband), gauge_y - 14),
            (gauge_x(deadband), gauge_y + 14),
            (70, 70, 70),
            -1,
        )
        cv2.line(panel, (x0, gauge_y), (x1, gauge_y), (190, 190, 190), 2)
        cv2.line(
            panel, (gauge_x(0), gauge_y - 20),
            (gauge_x(0), gauge_y + 20), (255, 255, 255), 1)
        color = (0, 0, 255) if status.get('saturated') else (40, 220, 40)
        cv2.line(
            panel, (gauge_x(0), gauge_y),
            (gauge_x(steering), gauge_y), color, 9)
        direction = (
            'NO LANE' if steering == 0 and not self.lane_valid
            else ('LEFT' if steering > 0 else (
                'RIGHT' if steering < 0 else 'CENTER')))
        self._put_text(
            panel,
            f'{steering:+d} counts  {direction}  deadband +/-{deadband}',
            (38, 116), color, 0.58, 1)
        self._put_text(panel, '-600 RIGHT', (x0, 52), (180, 180, 180), 0.42)
        self._put_text(panel, '+600 LEFT', (x1 - 78, 52), (180, 180, 180), 0.42)

        strip_top = 142
        strip_bottom = height - 26
        cv2.rectangle(
            panel, (x0, strip_top), (x1, strip_bottom), (25, 25, 25), -1)
        zero_y = (strip_top + strip_bottom) // 2
        cv2.line(panel, (x0, zero_y), (x1, zero_y), (100, 100, 100), 1)
        now = self.get_clock().now().nanoseconds * 1e-9
        seconds = max(1.0, float(self.get_parameter('strip_seconds').value))
        samples = [sample for sample in self.history if now - sample['time'] <= seconds]
        flips = self._sign_flip_indices(samples)
        error_points = []
        steering_points = []
        for index, sample in enumerate(samples):
            x = int(x1 - (now - sample['time']) / seconds * (x1 - x0))
            amplitude = (strip_bottom - strip_top) * 0.43
            error_y = int(
                zero_y - np.clip(sample['error'], -1.0, 1.0) * amplitude)
            steering_y = int(
                zero_y
                - np.clip(
                    sample['steering'] / max_counts, -1.0, 1.0
                ) * amplitude
            )
            error_points.append((x, error_y))
            steering_points.append((x, steering_y))
            if index in flips:
                cv2.circle(panel, (x, steering_y), 3, (0, 0, 255), -1)
        if len(error_points) >= 2:
            cv2.polylines(panel, [np.asarray(error_points)], False, (255, 120, 40), 2)
            cv2.polylines(panel, [np.asarray(steering_points)], False, (40, 220, 40), 2)
        self._put_text(
            panel,
            f'{seconds:.0f}s strip: error BLUE | steering GREEN | flips RED',
            (x0, height - 8), (190, 190, 190), 0.40)
        return panel

    def _rolling_stats(self) -> dict:
        now = self.get_clock().now().nanoseconds * 1e-9
        seconds = max(
            1.0, float(self.get_parameter('rolling_stats_sec').value))
        samples = [sample for sample in self.history if now - sample['time'] <= seconds]
        if not samples:
            return {'valid': 0.0, 'avg': 0.0, 'max': 0.0, 'sat': 0.0, 'flips': 0.0}
        errors = [abs(sample['error']) for sample in samples]
        return {
            'valid': 100.0 * sum(sample['valid'] for sample in samples) / len(samples),
            'avg': float(np.mean(errors)),
            'max': float(np.max(errors)),
            'sat': 100.0 * sum(sample['saturated'] for sample in samples) / len(samples),
            'flips': len(self._sign_flip_indices(samples)) / seconds,
        }

    def _status_panel(self, width: int, height: int) -> np.ndarray:
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (width, 28), (18, 18, 18), -1)
        self._put_text(panel, '4. STATUS', (10, 20), (0, 255, 255), 0.55)
        status = self.control_status
        geometry = self.geometry
        rows = geometry.get('scan_rows', [])
        look = next((row for row in rows if row.get('look_ahead')), {})
        measured = sum(row.get('source') == 'measured' for row in rows)
        predicted = sum(row.get('source') == 'predicted' for row in rows)
        fallback = sum(row.get('source') == 'box_fallback' for row in rows)
        width_px = look.get('width_px')
        contrast = look.get('contrast')
        width_text = '-' if width_px is None else f'{width_px:.1f}'
        contrast_text = '-' if contrast is None else f'+{contrast:.0f}'
        steering = int(status.get('steering_counts', 0))
        direction = (
            'NO LANE' if steering == 0 and not self.lane_valid
            else ('LEFT' if steering > 0 else (
                'RIGHT' if steering < 0 else 'CENTER')))
        saturated_text = 'yes' if status.get('saturated') else 'no'
        lines = [
            (f'MODE      {str(status.get("mode", "WAITING")).upper()}', (80, 240, 255)),
            (f'GATE      {status.get("gate_reason", "waiting")}', (235, 235, 235)),
            (
                f'TARGET    {geometry.get("target_mode", "-")} via '
                f'{geometry.get("target_source", "-")}',
                (235, 235, 235),
            ),
            (f'LANE      {geometry.get("lane_state", "NONE")}', (235, 235, 235)),
            (
                f'SCAN      measured {measured} / predicted {predicted} '
                f'/ fallback {fallback}',
                (235, 235, 235),
            ),
            (
                f'LINE      w={width_text} px  contrast {contrast_text}',
                (235, 235, 235),
            ),
            (
                f'ERROR     raw {status.get("raw_error", 0.0):+.3f}  '
                f'filtered {status.get("filtered_error", 0.0):+.3f}',
                (235, 235, 235),
            ),
            (
                f'TERMS     P {status.get("p_term", 0.0):+.3f}  '
                f'D {status.get("d_term", 0.0):+.3f}  '
                f'H {status.get("h_term", 0.0):+.3f}',
                (235, 235, 235),
            ),
            (
                f'STEER     {steering:+d} counts ({direction})  '
                f'sat:{saturated_text}',
                (40, 220, 40),
            ),
            (f'THROTTLE  {int(status.get("throttle", 0))}', (235, 235, 235)),
            (
                f'TIMING    infer {self.inference_ms:.1f} ms | '
                f'tick {status.get("tick_age_ms", 0.0):.0f} ms',
                (235, 235, 235),
            ),
            (
                f'VIDEO     {self.video_time_sec:.1f}/'
                f'{self.video_duration_sec:.1f}s frame {self.frame_index}',
                (235, 235, 235),
            ),
        ]
        act = self.actuation_status
        tx_stats = act.get('tx_stats', {})
        measured_adc = act.get('measured_adc')
        measured_text = '-' if measured_adc is None else str(measured_adc)
        latency = act.get('command_to_motion_ms')
        latency_text = '-' if latency is None else f'{latency:.0f}ms'
        lines.extend([
            (
                f'ACT       cmd {int(act.get("commanded_steering", 0)):+d} '
                f'({act.get("side", "CENTER")} db='
                f'{int(act.get("deadband_applied", 0))}) adc exp '
                f'{act.get("expected_adc", "-")} meas {measured_text} '
                f'lat {latency_text}',
                (40, 220, 40),
            ),
            (
                f'LINK      tx {float(act.get("tx_hz", 0.0)):.1f}Hz '
                f'cmd {float(act.get("cmd_pub_hz", 0.0)):.1f}Hz '
                f'ready {"YES" if act.get("serial_ready") else "no"} '
                f'stale {"YES" if act.get("command_stale") else "no"} '
                f'estop {"YES" if act.get("estop_latched") else "-"}',
                (235, 235, 235),
            ),
            (
                f'SUPP      guard {tx_stats.get("suppressed_reset_guard", 0)} '
                f'/ neutral {tx_stats.get("suppressed_no_source_neutral", 0)} '
                f'/ estop {tx_stats.get("suppressed_estop", 0)} '
                f'err {tx_stats.get("write_errors_total", 0)}',
                (235, 235, 235),
            ),
        ])
        if geometry.get('consistency_warning'):
            lines.append(('WARNING   center consistency mismatch', (40, 40, 255)))
        stats = self._rolling_stats()
        lines.append((
            f'5s STATS  valid {stats["valid"]:.0f}% |err| avg {stats["avg"]:.2f} '
            f'max {stats["max"]:.2f} sat {stats["sat"]:.0f}% flip {stats["flips"]:.1f}/s',
            (0, 255, 255),
        ))
        y = 50
        step = max(18, int((height - 56) / max(1, len(lines))))
        for text, color in lines:
            self._put_text(panel, text, (12, y), color, 0.48)
            y += step
        return panel

    def _compose(self) -> np.ndarray:
        panel_width = self.display_width // 2
        panel_height = int(panel_width * 0.56)
        panels = [
            self._follow_panel(panel_width, panel_height),
            self._yolo_panel(panel_width, panel_height),
            self._steering_panel(panel_width, panel_height),
            self._status_panel(panel_width, panel_height),
        ]
        if self.panel_focus:
            focused = panels[self.panel_focus - 1]
            return cv2.resize(
                focused, (self.display_width, panel_height * 2))
        return cv2.vconcat([
            cv2.hconcat(panels[:2]),
            cv2.hconcat(panels[2:]),
        ])

    def _capture_path(self, suffix: str, label: str = '') -> Path:
        directory = Path(str(self.get_parameter('capture_dir').value))
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        label_text = f'_{label}' if label else ''
        return directory / f'lane_drive_{stamp}{label_text}.{suffix}'

    def _save_capture(self, label: str = '') -> None:
        if self.last_display is None:
            return
        path = self._capture_path('png', label)
        if cv2.imwrite(str(path), self.last_display):
            self.get_logger().info(f'Capture saved: {path.resolve()}')
        else:
            self.get_logger().error(f'Capture failed: {path.resolve()}')

    def _toggle_recording(self) -> None:
        if self.recording:
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            self.get_logger().info(f'Recording saved: {self.recording_path}')
            return
        if self.last_display is None:
            return
        self.recording_path = self._capture_path('mp4').resolve()
        height, width = self.last_display.shape[:2]
        self.video_writer = cv2.VideoWriter(
            str(self.recording_path),
            cv2.VideoWriter_fourcc(*'mp4v'),
            20.0,
            (width, height),
        )
        if not self.video_writer.isOpened():
            self.video_writer = None
            self.get_logger().error('Could not open MP4 writer')
            return
        self.recording = True
        self.get_logger().info(f'Recording started: {self.recording_path}')

    def _handle_key(self, key: int) -> None:
        commands = {
            ord(' '): 'toggle_pause',
            ord('.'): 'step_forward',
            ord(']'): 'step_forward',
            ord(','): 'step_back',
            ord('['): 'step_back',
            ord('+'): 'rate_up',
            ord('='): 'rate_up',
            ord('-'): 'rate_down',
            ord('_'): 'rate_down',
        }
        if key in commands:
            self.command_pub.publish(String(data=commands[key]))
        elif key in {ord('r'), ord('R')}:
            self.command_pub.publish(String(data='restart'))
        elif key in {ord('d'), ord('D')}:
            self.threshold_pub.publish(
                Int32MultiArray(data=list(DEFAULT_THRESHOLDS)))
            self.get_logger().info('HSV defaults restored')
        elif key in {ord('s'), ord('S')}:
            self._save_capture()
        elif key in {ord('v'), ord('V')}:
            self._toggle_recording()
        elif key in {ord('1'), ord('2'), ord('3'), ord('4')}:
            selected = key - ord('0')
            self.panel_focus = 0 if self.panel_focus == selected else selected
        elif key in {ord('q'), ord('Q'), 27}:
            if self.shutdown_at is None:
                self.estop_pub.publish(Bool(data=True))
                self.shutdown_at = time.monotonic() + 1.5
                self.get_logger().warn(
                    'Visualizer exit requested: E-stop centering sequence started')

    def _render(self) -> None:
        if self.shutdown_at is not None and time.monotonic() >= self.shutdown_at:
            self.get_logger().info('Visualizer closed after E-stop sequence')
            rclpy.shutdown()
            return
        display = self._compose()
        if self.recording:
            cv2.circle(display, (display.shape[1] - 18, 18), 7, (0, 0, 255), -1)
        self.last_display = display
        if self.pending_capture_label is not None:
            label = self.pending_capture_label
            self.pending_capture_label = None
            self._save_capture(label)
        if bool(self.get_parameter('auto_capture').value):
            for capture_time in self.get_parameter('capture_times_sec').value:
                capture_time = float(capture_time)
                if (
                    capture_time not in self.auto_captured_times
                    and self.video_time_sec >= capture_time
                ):
                    self.auto_captured_times.add(capture_time)
                    self._save_capture(f't{capture_time:06.1f}s')
        if self.recording and self.video_writer is not None:
            self.video_writer.write(display)
        cv2.imshow(self.window_name, display)
        self._handle_key(cv2.waitKey(1) & 0xFF)

    def destroy_node(self) -> bool:
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the lane verification visualizer."""
    rclpy.init(args=args)
    node = TrackVisualizerNode()
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
