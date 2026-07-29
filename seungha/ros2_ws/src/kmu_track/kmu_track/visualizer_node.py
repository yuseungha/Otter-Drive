"""OpenCV pipeline viewer and live HSV threshold controls."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String, UInt8, UInt32


COMPACT_TRACKBARS = (
    ('White brightness min', 255, 2),
    ('White saturation max', 255, 4),
    ('Yellow hue min', 180, 6),
    ('Yellow hue max', 180, 9),
    ('Yellow saturation min', 255, 7),
    ('Yellow brightness min', 255, 8),
)

DEFAULT_THRESHOLDS = (
    0, 0, 175, 180, 80, 255,
    12, 70, 70, 42, 255, 255,
)


class TrackVisualizerNode(Node):
    """Render three useful lane views and publish live HSV thresholds."""

    def __init__(self) -> None:
        super().__init__('track_visualizer')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('window_name', 'KMU Track Vision')
        self.declare_parameter('controls_window_name', 'HSV Threshold Controls')
        self.declare_parameter('display_width', 1200)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_binary = None
        self.latest_yolo = None
        self.mission_state = 'WAIT_START'
        self.completed_laps = 0
        self.lane_error = 0.0
        self.lane_confidence = 0.0
        self.lane_valid = False
        self.video_time_sec = 0.0
        self.frame_index = 0
        self.paused = False
        self.thresholds = list(DEFAULT_THRESHOLDS)
        self.last_sent_thresholds = None

        self.command_pub = self.create_publisher(String, '/video/command', 10)
        self.threshold_pub = self.create_publisher(
            Int32MultiArray, '/lane/hsv_thresholds/set', 10)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/lane/debug_binary',
            self._on_binary,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/lane/yolo_debug', self._on_yolo, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        self.create_subscription(UInt8, '/mission/lap', self._on_lap, 10)
        self.create_subscription(
            Float32, '/lane/center_error', self._on_lane_error, 10)
        self.create_subscription(
            Float32, '/lane/confidence', self._on_lane_confidence, 10)
        self.create_subscription(Bool, '/lane/valid', self._on_lane_valid, 10)
        self.create_subscription(Float32, '/video/time_sec', self._on_time, 10)
        self.create_subscription(UInt32, '/video/frame_index', self._on_frame, 10)
        self.create_subscription(Bool, '/video/paused', self._on_paused, 10)
        threshold_qos = QoSProfile(depth=1)
        threshold_qos.reliability = ReliabilityPolicy.RELIABLE
        threshold_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Int32MultiArray,
            '/lane/hsv_thresholds/current',
            self._on_threshold_state,
            threshold_qos,
        )

        self.window_name = str(self.get_parameter('window_name').value)
        self.controls_window_name = str(
            self.get_parameter('controls_window_name').value)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            self.window_name,
            int(self.get_parameter('display_width').value),
            360,
        )
        cv2.namedWindow(self.controls_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.controls_window_name, 560, 360)
        for label, maximum, threshold_index in COMPACT_TRACKBARS:
            cv2.createTrackbar(
                label,
                self.controls_window_name,
                self.thresholds[threshold_index],
                maximum,
                lambda _value: None,
            )
        self.timer = self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            'Visualizer ready: 6 compact HSV controls | '
            'Space=pause/play, D=defaults, R=restart, Q/Esc=quit')

    def _on_image(self, msg: Image) -> None:
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_binary(self, msg: Image) -> None:
        self.latest_binary = self.bridge.imgmsg_to_cv2(msg, 'mono8')

    def _on_yolo(self, msg: Image) -> None:
        self.latest_yolo = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_state(self, msg: String) -> None:
        self.mission_state = msg.data

    def _on_lap(self, msg: UInt8) -> None:
        self.completed_laps = int(msg.data)

    def _on_lane_error(self, msg: Float32) -> None:
        self.lane_error = float(msg.data)

    def _on_lane_confidence(self, msg: Float32) -> None:
        self.lane_confidence = float(msg.data)

    def _on_lane_valid(self, msg: Bool) -> None:
        self.lane_valid = bool(msg.data)

    def _on_time(self, msg: Float32) -> None:
        self.video_time_sec = float(msg.data)

    def _on_frame(self, msg: UInt32) -> None:
        self.frame_index = int(msg.data)

    def _on_paused(self, msg: Bool) -> None:
        self.paused = bool(msg.data)

    def _on_threshold_state(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != len(DEFAULT_THRESHOLDS):
            return
        incoming = [int(value) for value in msg.data]
        if incoming == self.thresholds:
            return
        self.thresholds = incoming
        for label, _maximum, threshold_index in COMPACT_TRACKBARS:
            cv2.setTrackbarPos(
                label,
                self.controls_window_name,
                self.thresholds[threshold_index],
            )

    @staticmethod
    def _put_text(image, text: str, origin, color=(255, 255, 255)) -> None:
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            1,
            cv2.LINE_AA,
        )

    def _raw_with_status(self):
        frame = self.latest_frame.copy()
        height, width = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (width, 70), (20, 20, 20), -1)
        valid_text = 'VALID' if self.lane_valid else 'LOST'
        valid_color = (40, 230, 40) if self.lane_valid else (30, 30, 255)
        self._put_text(
            frame,
            f'{self.mission_state} | LAP {self.completed_laps}/3 | '
            f'{self.video_time_sec:.1f}s',
            (10, 23),
            (0, 255, 255),
        )
        self._put_text(
            frame,
            f'YOLO {valid_text} | error {self.lane_error:+.3f} | '
            f'confidence {self.lane_confidence:.2f}',
            (10, 49),
            valid_color,
        )
        self._put_text(
            frame,
            'Space pause/play | D HSV defaults | R restart | Q/Esc quit',
            (10, height - 10),
        )
        return frame

    def _panel(self, image, label: str, mono: bool = False):
        panel_width = max(
            240, int(self.get_parameter('display_width').value) // 3)
        panel_height = int(panel_width * 0.75)
        if image is None:
            panel = np.zeros(
                (panel_height, panel_width, 3), dtype=np.uint8)
            self._put_text(
                panel, 'Waiting for topic...', (12, panel_height // 2))
        else:
            panel = image
            if mono or panel.ndim == 2:
                panel = cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)
            panel = cv2.resize(panel, (panel_width, panel_height))
        cv2.rectangle(
            panel, (0, 0), (panel_width, 28), (20, 20, 20), -1)
        self._put_text(panel, label, (10, 20), (0, 255, 255))
        return panel

    def _read_and_publish_thresholds(self) -> None:
        current = self.thresholds.copy()
        for label, _maximum, threshold_index in COMPACT_TRACKBARS:
            current[threshold_index] = cv2.getTrackbarPos(
                label, self.controls_window_name)
        self.thresholds = current
        if current != self.last_sent_thresholds:
            self.threshold_pub.publish(Int32MultiArray(data=current))
            self.last_sent_thresholds = current.copy()

    def _control_canvas(self):
        canvas = np.zeros((150, 540, 3), dtype=np.uint8)
        white = self.thresholds
        self._put_text(canvas, 'Live HSV thresholds', (12, 25), (0, 255, 255))
        self._put_text(
            canvas,
            f'WHITE lower {white[0:3]}  upper {white[3:6]}',
            (12, 58),
        )
        self._put_text(
            canvas,
            f'YELLOW lower {white[6:9]}  upper {white[9:12]}',
            (12, 88),
        )
        self._put_text(
            canvas,
            'D: restore defaults | fixed HSV limits are hidden',
            (12, 122),
            (180, 220, 180),
        )
        return canvas

    def _restore_default_thresholds(self) -> None:
        self.thresholds = list(DEFAULT_THRESHOLDS)
        for label, _maximum, threshold_index in COMPACT_TRACKBARS:
            cv2.setTrackbarPos(
                label,
                self.controls_window_name,
                self.thresholds[threshold_index],
            )

    def _render(self) -> None:
        self._read_and_publish_thresholds()
        cv2.imshow(self.controls_window_name, self._control_canvas())
        if self.latest_frame is not None:
            panels = [
                self._panel(self._raw_with_status(), '1. ORIGINAL'),
                self._panel(
                    self.latest_binary,
                    '2. COMBINED CLEAN MASK',
                    mono=True,
                ),
                self._panel(
                    self.latest_yolo,
                    '3. YOLO lane1 / lane2 + CENTER',
                ),
            ]
            display = cv2.hconcat(panels)
            cv2.imshow(self.window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self.command_pub.publish(String(data='toggle_pause'))
        elif key in {ord('d'), ord('D')}:
            self._restore_default_thresholds()
        elif key in {ord('r'), ord('R')}:
            self.command_pub.publish(String(data='restart'))
        elif key in {ord('q'), ord('Q'), 27}:
            self.get_logger().info('Visualizer closed by user')
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
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
