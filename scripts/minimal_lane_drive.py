#!/usr/bin/env python3
"""Minimal lane following: camera -> YOLO -> target lane -> steering -> serial.

This is the smallest runnable slice of the ROS stack.  It keeps only the two
steps that must exist for the car to drive itself:

1. ``plan_lane``  : YOLO ``lane``/``center`` masks -> one target lane center.
2. ``SteeringPD`` : that lane error -> a steering command in firmware counts.

Everything else in ``src/jetson`` (mission FSM, LiDAR cones, obstacle avoid,
overlays, dry-run gates) is intentionally absent.  Throttle stays at zero
unless ``--throttle`` is given, so the default run only moves the steering
actuator.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import serial
from ultralytics import YOLO

# Firmware contract, copied from rc_car_teleop/serial_bridge.py.
BAUD_RATE = 115200
RESET_GUARD_SEC = 3.5      # The Uno reboots when the serial port opens.
WATCHDOG_SEC = 0.400       # Send faster than this or the firmware neutralizes.
STEER_COUNTS_LIMIT = 1000  # Firmware clamps the D-frame steering field here.

# Lane geometry, copied from config/segmentation_lane.yaml.
SCAN_ROWS = (0.50, 0.58, 0.66, 0.74, 0.82, 0.90, 0.96)
LOOK_AHEAD_ROW = 0.74      # Row whose lane center becomes the steering target.
HEADING_FAR_ROW = 0.58
HEADING_NEAR_ROW = 0.90
MASK_THRESHOLD = 0.50
MIN_LANE_WIDTH_RATIO = 0.10
MIN_PIXELS_PER_ROW = 3
MIN_VALID_ROWS = 3


def sample_mask_x(mask, row_y: int, half_height: int = 5):
    """Return the median mask column inside a horizontal band, or None."""
    low = max(0, row_y - half_height)
    high = min(mask.shape[0], row_y + half_height + 1)
    _ys, xs = np.nonzero(mask[low:high] >= MASK_THRESHOLD)
    if xs.size < MIN_PIXELS_PER_ROW:
        return None
    return float(np.median(xs))


def masks_by_class(result, names, shape) -> dict:
    """Group YOLO segmentation masks by class name at full image resolution."""
    grouped = {'lane': [], 'center': []}
    if result.masks is None or result.boxes is None:
        return grouped
    height, width = shape[:2]
    raw = result.masks.data.float().cpu().numpy()
    classes = result.boxes.cls.int().cpu().numpy()
    for mask, class_id in zip(raw, classes):
        if mask.shape != (height, width):
            mask = cv2.resize(
                mask, (width, height), interpolation=cv2.INTER_LINEAR)
        name = str(names[int(class_id)])
        if name in grouped:
            grouped[name].append(mask)
    return grouped


def plan_lane(result, names, shape) -> dict:
    """Build the lane to follow and return its normalized steering errors.

    Per scan row the target is the midpoint of the two outermost ``lane``
    boundaries.  When only one boundary is visible the ``center`` marking is
    used instead.  A row with neither is dropped; too few rows means LOST and
    the caller must not steer from it.
    """
    height, width = shape[:2]
    grouped = masks_by_class(result, names, shape)
    minimum_lane_width = width * MIN_LANE_WIDTH_RATIO

    rows = []  # (row ratio, target x in pixels)
    for ratio in SCAN_ROWS:
        row_y = int(round(ratio * (height - 1)))
        boundary_x = sorted(
            x for x in (sample_mask_x(m, row_y) for m in grouped['lane'])
            if x is not None
        )
        center_x = next(
            (x for x in (sample_mask_x(m, row_y) for m in grouped['center'])
             if x is not None),
            None,
        )
        wide_enough = (
            len(boundary_x) >= 2
            and boundary_x[-1] - boundary_x[0] >= minimum_lane_width
        )
        if wide_enough:
            rows.append((ratio, (boundary_x[0] + boundary_x[-1]) * 0.5))
        elif center_x is not None:
            rows.append((ratio, center_x))

    if len(rows) < MIN_VALID_ROWS:
        return {
            'valid': False,
            'center_error': 0.0,
            'heading_error': 0.0,
            'rows': rows,
        }

    # One low-order fit through the target points is the followed lane.
    ratios = np.asarray([row[0] for row in rows], dtype=np.float64)
    targets = np.asarray([row[1] for row in rows], dtype=np.float64)
    degree = 2 if len(rows) >= 5 else 1
    curve = np.polynomial.polynomial.Polynomial.fit(ratios, targets, degree)

    half_width = max(1.0, width * 0.5)
    center_error = float(np.clip(
        (curve(LOOK_AHEAD_ROW) - half_width) / half_width, -1.0, 1.0))
    heading_error = float(np.clip(
        (curve(HEADING_FAR_ROW) - curve(HEADING_NEAR_ROW)) / half_width,
        -1.0, 1.0))
    return {
        'valid': True,
        'center_error': center_error,    # + means the lane is right of center.
        'heading_error': heading_error,  # + means the lane bends right ahead.
        'lane_center_x': float(curve(LOOK_AHEAD_ROW)),
        'rows': rows,
        'curve': curve,
    }


class SteeringPD:
    """Lane error -> steering counts, with the gains from lane_control.yaml."""

    def __init__(
        self,
        kp: float = 0.95,
        kd: float = 0.10,
        k_heading: float = 0.60,
        steering_sign: int = -1,      # Airborne sign check owns this value.
        max_counts: int = 650,
        deadband_counts: int = 110,   # Below this the actuator does not move.
        max_delta_counts: int = 120,  # Per control tick.
        lpf_alpha: float = 0.50,
    ) -> None:
        self.kp = kp
        self.kd = kd
        self.k_heading = k_heading
        self.steering_sign = steering_sign
        self.max_counts = max_counts
        self.deadband_counts = deadband_counts
        self.max_delta_counts = max_delta_counts
        self.lpf_alpha = lpf_alpha
        self.filtered_error = 0.0
        self.last_steering = 0
        self.last_time = None

    def update(self, sample: dict, now: float) -> int:
        """Return the next steering command in counts (-1000..1000)."""
        if not sample['valid']:
            # Lane lost: ramp to center instead of holding the last lock.
            self.filtered_error = 0.0
            return self._rate_limit(0)

        previous = self.filtered_error
        self.filtered_error = (
            self.lpf_alpha * previous
            + (1.0 - self.lpf_alpha) * sample['center_error']
        )
        dt = 0.1 if self.last_time is None else max(1e-3, now - self.last_time)
        self.last_time = now
        derivative = (self.filtered_error - previous) / dt

        demand = (
            self.kp * self.filtered_error
            + self.kd * derivative
            + self.k_heading * sample['heading_error']
        )
        # steering_sign maps "lane sits right of image center" to the physical
        # direction the linkage must turn.
        signed = float(np.clip(self.steering_sign * demand, -1.0, 1.0))

        if abs(signed) < 0.03:
            target = 0
        else:
            magnitude = (
                self.deadband_counts
                + (self.max_counts - self.deadband_counts) * abs(signed)
            )
            target = int(round(magnitude if signed > 0.0 else -magnitude))
        return self._rate_limit(target)

    def _rate_limit(self, target: int) -> int:
        delta = int(target) - self.last_steering
        if abs(delta) > self.max_delta_counts:
            target = self.last_steering + (
                self.max_delta_counts if delta > 0 else -self.max_delta_counts)
        self.last_steering = int(
            np.clip(target, -STEER_COUNTS_LIMIT, STEER_COUNTS_LIMIT))
        return self.last_steering


class SteeringLink:
    """Write ``D <throttle> <steering>`` frames to the Arduino."""

    def __init__(self, port: str) -> None:
        self.serial = serial.Serial(
            port, BAUD_RATE, timeout=0.05, write_timeout=0.05)
        print(f'serial open: {port} '
              f'(waiting {RESET_GUARD_SEC:.1f}s for the Uno reset)')
        time.sleep(RESET_GUARD_SEC)
        self.send(0, 0)

    def send(self, throttle: int, steering: int) -> None:
        throttle = int(
            np.clip(throttle, -STEER_COUNTS_LIMIT, STEER_COUNTS_LIMIT))
        steering = int(
            np.clip(steering, -STEER_COUNTS_LIMIT, STEER_COUNTS_LIMIT))
        self.serial.write(f'D {throttle} {steering}\n'.encode('ascii'))

    def stop(self) -> None:
        """Center the wheels, then latch the firmware stop."""
        self.send(0, 0)
        time.sleep(0.05)
        self.serial.write(b'X\n')
        self.serial.flush()
        self.serial.close()


def open_camera(device: str, width: int, height: int, fps: float):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f'camera did not open: {device}')
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def draw_overlay(frame, sample: dict, steering: int):
    """Draw the followed lane and the command, for eyeball verification."""
    view = frame.copy()
    height, width = view.shape[:2]
    cv2.line(view, (width // 2, height), (width // 2, int(height * 0.45)),
             (90, 90, 90), 1)
    for ratio, target_x in sample['rows']:
        cv2.circle(view, (int(target_x), int(ratio * (height - 1))), 4,
                   (0, 200, 255), -1)
    if sample['valid']:
        curve = sample['curve']
        points = [
            (int(curve(r)), int(r * (height - 1)))
            for r in np.linspace(SCAN_ROWS[0], SCAN_ROWS[-1], 20)
        ]
        cv2.polylines(view, [np.asarray(points, dtype=np.int32)], False,
                      (0, 255, 0), 2)
        cv2.circle(view, (int(sample['lane_center_x']),
                          int(LOOK_AHEAD_ROW * (height - 1))), 8, (0, 0, 255), 2)
    label = (
        f"{'LANE' if sample['valid'] else 'LOST'}  "
        f"err={sample['center_error']:+.3f}  steer={steering:+d}"
    )
    cv2.putText(view, label, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 0) if sample['valid'] else (0, 0, 255), 2, cv2.LINE_AA)
    return view


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='models/lane_seg_v3_e37.pt')
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--video', default='',
                        help='Use a recorded file instead of the camera.')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--device', default='0',
                        help="'0' for CUDA, 'cpu' otherwise.")
    parser.add_argument('--serial', default='',
                        help='Arduino port; empty means no motor output.')
    parser.add_argument('--throttle', type=int, default=0,
                        help='Forward counts. Keep 0 until the car is on stands.')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.is_file():
        print(f'model not found: {model_path}', file=sys.stderr)
        return 1
    model = YOLO(str(model_path))
    print(f'model classes: {model.names}')

    source = args.video if args.video else args.camera
    capture = (
        cv2.VideoCapture(source) if args.video
        else open_camera(source, args.width, args.height, args.fps)
    )
    controller = SteeringPD()
    link = SteeringLink(args.serial) if args.serial else None
    last_sent = time.monotonic()

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print('frame read failed', file=sys.stderr)
                break

            result = model.predict(
                frame, imgsz=args.imgsz, conf=args.conf, device=args.device,
                verbose=False)[0]
            sample = plan_lane(result, model.names, frame.shape)

            now = time.monotonic()
            steering = controller.update(sample, now)
            throttle = args.throttle if sample['valid'] else 0
            if link is not None:
                if now - last_sent > WATCHDOG_SEC:
                    print(f'WARN: {now - last_sent:.2f}s since the last frame; '
                          'the firmware watchdog may have neutralized',
                          file=sys.stderr)
                link.send(throttle, steering)
                last_sent = now

            if args.show:
                cv2.imshow('minimal lane drive',
                           draw_overlay(frame, sample, steering))
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if link is not None:
            link.stop()
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
