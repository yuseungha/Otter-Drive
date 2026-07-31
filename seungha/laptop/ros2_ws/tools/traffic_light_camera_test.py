#!/usr/bin/env python3
"""Show a live USB-camera RED/STOP and GREEN/GO test window."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = WORKSPACE_ROOT / 'src' / 'kmu_track'
sys.path.insert(0, str(PACKAGE_ROOT))

from kmu_track.traffic_light_core import (  # noqa: E402
    SignalDecision,
    SignalState,
    TrafficLightDetector,
)


WINDOW_NAME = 'Logitech Traffic Light Test'


def _parse_roi(value: str) -> Tuple[float, float, float, float]:
    try:
        roi = tuple(float(part.strip()) for part in value.split(','))
    except ValueError as error:
        raise argparse.ArgumentTypeError('ROI values must be numbers') from error
    if len(roi) != 4:
        raise argparse.ArgumentTypeError('ROI must be x,y,width,height')
    return roi  # type: ignore[return-value]


def _camera_candidates(
    requested_index: int,
    auto_fallback: bool,
) -> Sequence[int]:
    if not auto_fallback:
        return [requested_index]
    remaining = [index for index in range(5) if index != requested_index]
    return [requested_index, *remaining]


def _open_camera(
    requested_index: int,
    width: int,
    height: int,
    fps: int,
    auto_fallback: bool,
) -> Tuple[cv2.VideoCapture, int, str]:
    backends = []
    if sys.platform == 'win32':
        backends = [
            (cv2.CAP_DSHOW, 'DirectShow'),
            (cv2.CAP_MSMF, 'Media Foundation'),
        ]
    backends.append((cv2.CAP_ANY, 'Auto'))

    for index in _camera_candidates(requested_index, auto_fallback):
        for backend, backend_name in backends:
            capture = cv2.VideoCapture(index, backend)
            if not capture.isOpened():
                capture.release()
                continue
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                return capture, index, backend_name
            capture.release()
    attempted = 'indexes 0..4' if auto_fallback else f'index {requested_index}'
    raise RuntimeError(
        f'No camera could be opened at {attempted}. Close Camera/Teams/Zoom '
        'and retry, or pass the correct --camera N.'
    )


def _mask_preview(decision: SignalDecision, width: int) -> np.ndarray:
    red = cv2.cvtColor(decision.evidence.red_mask, cv2.COLOR_GRAY2BGR)
    green = cv2.cvtColor(decision.evidence.green_mask, cv2.COLOR_GRAY2BGR)
    red[:, :, 0:2] = 0
    green[:, :, (0, 2)] = 0
    combined = cv2.add(red, green)
    preview_height = max(1, int(combined.shape[0] * width / combined.shape[1]))
    return cv2.resize(combined, (width, preview_height))


def _draw_overlay(
    frame: np.ndarray,
    decision: SignalDecision,
    camera_index: int,
    backend_name: str,
) -> np.ndarray:
    output = frame.copy()
    height, width = output.shape[:2]
    banner_colors = {
        SignalState.STOP: (30, 30, 210),
        SignalState.GO: (35, 170, 35),
        SignalState.TURN_LEFT: (20, 155, 90),
    }
    banner_color = banner_colors[decision.state]
    text_color = (255, 255, 255)
    state_scale = 1.45 if decision.state == SignalState.TURN_LEFT else 2.1
    reason_x = 440 if decision.state == SignalState.TURN_LEFT else 220

    cv2.rectangle(output, (0, 0), (width, 112), banner_color, -1)
    cv2.putText(
        output,
        decision.state.value,
        (20, 68),
        cv2.FONT_HERSHEY_DUPLEX,
        state_scale,
        text_color,
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        decision.reason,
        (reason_x, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        text_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        f'R {decision.evidence.red_ratio * 100:5.2f}%   '
        f'G {decision.evidence.green_ratio * 100:5.2f}%   '
        f'LEFT {decision.evidence.left_arrow_score:.2f}   '
        f'CAM {camera_index} / {backend_name}',
        (220, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        text_color,
        1,
        cv2.LINE_AA,
    )

    x0, y0, x1, y1 = decision.evidence.roi_box
    cv2.rectangle(output, (x0, y0), (x1, y1), (0, 255, 255), 2)
    cv2.putText(
        output,
        'DETECTION ROI',
        (x0 + 6, min(height - 10, y0 + 24)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    arrow_box = decision.evidence.left_arrow_box
    if decision.evidence.left_arrow_active and arrow_box is not None:
        arrow_x0, arrow_y0, arrow_x1, arrow_y1 = arrow_box
        arrow_start = (x0 + arrow_x0, y0 + arrow_y0)
        arrow_end = (x0 + arrow_x1, y0 + arrow_y1)
        cv2.rectangle(output, arrow_start, arrow_end, (255, 255, 0), 3)
        cv2.putText(
            output,
            'LEFT ARROW',
            (arrow_start[0], max(20, arrow_start[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    preview_width = max(180, width // 4)
    preview = _mask_preview(decision, preview_width)
    preview_height = preview.shape[0]
    px0 = width - preview_width - 10
    py0 = height - preview_height - 38
    output[py0:py0 + preview_height, px0:px0 + preview_width] = preview
    cv2.rectangle(
        output,
        (px0, py0),
        (px0 + preview_width, py0 + preview_height),
        (255, 255, 255),
        1,
    )
    cv2.putText(
        output,
        'COLOR MASK',
        (px0, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        'Q/ESC quit | R reset filter',
        (12, height - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--camera',
        type=int,
        default=2,
        help='OpenCV camera index (default: 2, current Logitech BRIO)',
    )
    parser.add_argument(
        '--auto-fallback',
        action='store_true',
        help='try camera indexes 0..4 if the requested camera cannot open',
    )
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument(
        '--roi',
        type=_parse_roi,
        default=(0.20, 0.15, 0.60, 0.70),
        help='normalized x,y,width,height (default: 0.20,0.15,0.60,0.70)',
    )
    parser.add_argument('--min-ratio', type=float, default=0.010)
    parser.add_argument('--min-blob-area', type=float, default=250.0)
    parser.add_argument('--green-frames', type=int, default=5)
    parser.add_argument('--lost-frames', type=int, default=3)
    parser.add_argument('--require-red-first', action='store_true')
    parser.add_argument(
        '--max-frames',
        type=int,
        default=0,
        help='exit after N frames; 0 keeps running',
    )
    return parser


def run(args: argparse.Namespace) -> int:
    detector = TrafficLightDetector(
        roi=args.roi,
        min_red_ratio=args.min_ratio,
        min_green_ratio=args.min_ratio,
        min_blob_area=args.min_blob_area,
        confirm_green_frames=args.green_frames,
        lost_signal_frames=args.lost_frames,
        require_red_before_green=args.require_red_first,
    )
    capture: Optional[cv2.VideoCapture] = None
    consecutive_read_failures = 0
    processed_frames = 0
    try:
        capture, camera_index, backend_name = _open_camera(
            args.camera,
            args.width,
            args.height,
            args.fps,
            args.auto_fallback,
        )
        print(
            f'Opened camera {camera_index} via {backend_name}: '
            f'{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
        )
        window_name = f'{WINDOW_NAME} - CAM {camera_index}'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1100, 700)

        while True:
            ok, frame = capture.read()
            if not ok or frame is None or not frame.size:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 10:
                    raise RuntimeError('Camera stopped delivering frames')
                time.sleep(0.01)
                continue
            consecutive_read_failures = 0
            decision = detector.analyze(frame)
            cv2.imshow(
                window_name,
                _draw_overlay(frame, decision, camera_index, backend_name),
            )
            processed_frames += 1
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('r'), ord('R')):
                detector.reset()
            if args.max_frames and processed_frames >= args.max_frames:
                break
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    args = build_argument_parser().parse_args()
    try:
        return run(args)
    except (RuntimeError, ValueError, cv2.error) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
