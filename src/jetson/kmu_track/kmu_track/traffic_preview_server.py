"""Serve an annotated traffic-light camera preview over local HTTP."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class PreviewHttpServer(ThreadingHTTPServer):
    """Threaded HTTP server carrying a reference to its ROS node."""

    daemon_threads = True
    allow_reuse_address = True


class PreviewHandler(BaseHTTPRequestHandler):
    """Serve a small page and an MJPEG stream."""

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ('/', '/index.html'):
            page = (
                '<!doctype html><html><head><meta charset="utf-8">'
                '<title>Jetson Traffic Light Preview</title>'
                '<style>html,body{margin:0;background:#111;color:#eee;'
                'font-family:Arial,sans-serif;height:100%}body{display:flex;'
                'align-items:center;justify-content:center;'
                'flex-direction:column}'
                'h2{margin:10px}img{max-width:96vw;max-height:88vh;'
                'border:2px solid #555;background:#000}</style></head><body>'
                '<h2>Jetson Traffic Light - DRY-RUN</h2>'
                '<img src="/stream.mjpg" alt="camera preview">'
                '</body></html>'
            ).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path == '/stream.mjpg':
            self._serve_stream()
            return
        self.send_error(404)

    def _serve_stream(self) -> None:
        node = self.server.preview_node
        self.send_response(200)
        self.send_header(
            'Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache')
        self.end_headers()
        last_sequence = -1
        try:
            while rclpy.ok(context=node.context):
                with node.frame_condition:
                    node.frame_condition.wait_for(
                        lambda: node.frame_sequence != last_sequence,
                        timeout=1.0,
                    )
                    jpeg = node.latest_jpeg
                    last_sequence = node.frame_sequence
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(
                    f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, _format: str, *_args) -> None:
        return


class TrafficPreviewServer(Node):
    """Overlay perception state on camera frames and serve them as MJPEG."""

    def __init__(self) -> None:
        super().__init__('traffic_preview_server')
        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 80)

        self.bridge = CvBridge()
        self.state = 'STOP'
        self.reason = 'WAITING FOR SIGNAL'
        self.start_green_ratio = 0.0
        self.left_green_ratio = 0.0
        self.left_arrow_score = 0.0
        self.motor_throttle = 0
        self.motor_steering = 0
        self.motor_gear = -1
        self.motor_reason = 'WAITING'
        self.latest_jpeg = None
        self.frame_sequence = 0
        self.frame_condition = threading.Condition()
        self.jpeg_quality = max(
            30,
            min(95, int(self.get_parameter('jpeg_quality').value)),
        )

        self.create_subscription(
            Image, '/camera/front/image_raw', self._on_image, 5)
        self.create_subscription(
            String,
            '/perception/traffic_light_state',
            lambda message: setattr(self, 'state', message.data),
            10,
        )
        self.create_subscription(
            String,
            '/perception/traffic_light_reason',
            lambda message: setattr(self, 'reason', message.data),
            10,
        )
        self.create_subscription(
            Float32,
            '/perception/start_green_ratio',
            lambda message: setattr(
                self, 'start_green_ratio', float(message.data)),
            10,
        )
        self.create_subscription(
            String,
            '/vehicle/traffic_motor_status',
            self._on_motor_status,
            10,
        )
        self.create_subscription(
            Float32,
            '/perception/left_green_ratio',
            lambda message: setattr(
                self, 'left_green_ratio', float(message.data)),
            10,
        )
        self.create_subscription(
            Float32,
            '/perception/left_arrow_score',
            lambda message: setattr(
                self, 'left_arrow_score', float(message.data)),
            10,
        )

        bind_address = str(self.get_parameter('bind_address').value)
        port = int(self.get_parameter('port').value)
        self.http_server = PreviewHttpServer(
            (bind_address, port), PreviewHandler)
        self.http_server.preview_node = self
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name='traffic-preview-http',
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f'Annotated preview ready at http://{bind_address}:{port}')

    def _on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        banner_height = min(130, max(90, height // 6))
        color = {
            'GO': (30, 165, 30),
            'TURN LEFT': (180, 110, 20),
            'STOP': (20, 20, 210),
        }.get(self.state, (80, 80, 80))
        cv2.rectangle(frame, (0, 0), (width, banner_height), color, -1)
        font_scale = max(0.8, min(1.8, width / 850.0))
        cv2.putText(
            frame,
            self.state,
            (24, int(banner_height * 0.56)),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            self.reason[:80],
            (24, banner_height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        metrics = (
            f'GREEN {self.start_green_ratio * 100:.2f}%   '
            f'LEFT {self.left_green_ratio * 100:.2f}%   '
            f'ARROW {self.left_arrow_score:.2f}'
        )
        cv2.putText(
            frame,
            metrics,
            (20, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        motor = (
            f'CMD throttle={self.motor_throttle}  '
            f'steering={self.motor_steering}  gear={self.motor_gear}  '
            f'{self.motor_reason}'
        )
        cv2.putText(
            frame,
            motor,
            (20, height - 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            return
        with self.frame_condition:
            self.latest_jpeg = encoded.tobytes()
            self.frame_sequence += 1
            self.frame_condition.notify_all()

    def _on_motor_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.motor_throttle = int(status.get('throttle', 0))
        self.motor_steering = int(status.get('steering', 0))
        self.motor_gear = int(status.get('gear', -1))
        self.motor_reason = str(status.get('reason', 'UNKNOWN'))[:40]

    def destroy_node(self) -> bool:
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficPreviewServer()
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
