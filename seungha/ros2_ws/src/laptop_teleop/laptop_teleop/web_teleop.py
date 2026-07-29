#!/usr/bin/env python3
"""Serve a laptop control page and publish fail-safe ROS drive commands."""

from __future__ import annotations

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray

from .control_core import ControlState


MAX_BODY_BYTES = 2048


class TeleopHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, node):
        super().__init__(address, handler)
        self.teleop_node = node


class TeleopRequestHandler(SimpleHTTPRequestHandler):
    server: TeleopHttpServer

    def log_message(self, format_string: str, *args: Any) -> None:
        self.server.teleop_node.get_logger().debug(format_string % args)

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self._send_json(200, self.server.teleop_node.status_payload())
            return
        if self.path in ("/", "/index.html"):
            self.path = "/index.html"
            super().do_GET()
            return
        self.send_error(404)

    def do_POST(self) -> None:
        try:
            body = self._read_json()
        except ValueError as error:
            self._send_json(400, {"ok": False, "reason": str(error)})
            return

        client = self.client_address[0]
        node = self.server.teleop_node

        if self.path == "/api/arm":
            ok, reason = node.arm(client)
            self._send_json(200 if ok else 409, {"ok": ok, "reason": reason})
        elif self.path == "/api/command":
            ok, reason = node.command(
                client, body.get("throttle", 0), body.get("steering", 0))
            self._send_json(200 if ok else 409, {"ok": ok, "reason": reason})
        elif self.path == "/api/estop":
            node.estop("operator_estop")
            self._send_json(200, {"ok": True, "reason": "stopped"})
        else:
            self.send_error(404)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("invalid_content_length") from error
        if length < 0 or length > MAX_BODY_BYTES:
            raise ValueError("request_too_large")
        if length == 0:
            return {}
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("invalid_json") from error
        if not isinstance(data, dict):
            raise ValueError("json_object_required")
        return data

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


class WebTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("web_teleop")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 8765)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("browser_timeout_sec", 0.25)
        self.declare_parameter("dry_run", True)

        host = str(self.get_parameter("listen_host").value)
        port = int(self.get_parameter("listen_port").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        timeout = float(self.get_parameter("browser_timeout_sec").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        if not 1 <= port <= 65535:
            raise ValueError("listen_port must be 1..65535")
        if rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self._control = ControlState(timeout)
        self._serial_connected = False
        self._publisher = self.create_publisher(
            Int32MultiArray, "/rc_car/drive_cmd", 10)
        self.create_subscription(
            Bool, "/rc_car/serial_connected", self._serial_status_callback, 10)
        self.create_timer(1.0 / rate_hz, self._publish_command)

        web_root = Path(get_package_share_directory("laptop_teleop")) / "web"
        handler = partial(TeleopRequestHandler, directory=str(web_root))
        self._http = TeleopHttpServer((host, port), handler, self)
        self._http_thread = threading.Thread(
            target=self._http.serve_forever,
            name="teleop-http",
            daemon=True,
        )
        self._http_thread.start()
        mode = "DRY-RUN" if self._dry_run else "HARDWARE"
        self.get_logger().info(
            f"Laptop teleop ready: http://{host}:{port} mode={mode}")

    def _serial_status_callback(self, message: Bool) -> None:
        self._serial_connected = bool(message.data)
        if not self._serial_connected and not self._dry_run:
            self._control.stop("serial_disconnected")

    def arm(self, client: str) -> tuple[bool, str]:
        allowed = self._dry_run or self._serial_connected
        result = self._control.arm(client, allowed)
        if result[0]:
            self.get_logger().info(f"Operator armed from {client}")
        else:
            self.get_logger().warn(f"Arm rejected from {client}: {result[1]}")
        return result

    def command(
        self, client: str, throttle: object, steering: object
    ) -> tuple[bool, str]:
        return self._control.update(client, throttle, steering)

    def estop(self, reason: str) -> None:
        was_armed = self._control.snapshot().armed
        self._control.stop(reason)
        self._publish_neutral_burst()
        if was_armed:
            self.get_logger().warn(f"Emergency stop: {reason}")

    def _publish_command(self) -> None:
        throttle, steering = self._control.command_for_publish()
        message = Int32MultiArray()
        message.data = [throttle, steering]
        self._publisher.publish(message)

    def _publish_neutral_burst(self) -> None:
        message = Int32MultiArray()
        message.data = [0, 0]
        for _ in range(3):
            self._publisher.publish(message)

    def status_payload(self) -> dict[str, Any]:
        snapshot = self._control.snapshot()
        return {
            "armed": snapshot.armed,
            "throttle": snapshot.throttle,
            "steering": snapshot.steering,
            "active_client": snapshot.active_client,
            "command_age_ms": snapshot.command_age_ms,
            "stop_reason": snapshot.stop_reason,
            "dry_run": self._dry_run,
            "serial_connected": self._serial_connected,
        }

    def destroy_node(self) -> None:
        self._control.stop("node_shutdown")
        if rclpy.ok():
            self._publish_neutral_burst()
            self._http.shutdown()
            self._http_thread.join(timeout=1.0)
        self._http.server_close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch can deliver a second SIGINT while resources close.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
