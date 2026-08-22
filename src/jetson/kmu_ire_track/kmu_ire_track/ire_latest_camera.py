"""Threaded latest-frame camera capture for the IRE perception node."""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from kmu_track.usb_camera_node import opencv_camera_source


class LatestFrameCamera:
    """Capture a USB camera while retaining only its newest frame."""

    def __init__(
        self,
        *,
        device: str,
        width: int,
        height: int,
        fps: float,
        fourcc: str,
        reconnect_interval_sec: float,
        logger,
    ) -> None:
        if width <= 0 or height <= 0 or fps <= 0.0:
            raise ValueError('camera width, height, and fps must be positive')
        if len(fourcc) != 4:
            raise ValueError(
                'camera fourcc must contain exactly four characters')
        self.device = device
        self.capture_source = opencv_camera_source(device)
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc.upper()
        self.reconnect_interval_sec = max(0.05, reconnect_interval_sec)
        self.logger = logger

        self._frame_lock = threading.Lock()
        self._latest: Optional[tuple[int, int, np.ndarray]] = None
        self._sequence = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._capture: Optional[cv2.VideoCapture] = None

    def start(self) -> None:
        """Start the capture worker once."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name='ire-latest-camera',
            daemon=True,
        )
        self._thread.start()

    def latest(self) -> Optional[tuple[int, int, np.ndarray]]:
        """Return sequence, capture time, and newest frame by reference."""
        with self._frame_lock:
            return self._latest

    def stop(self) -> None:
        """Stop capture and release the camera device."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(2.0, 3.0 / self.fps))
        capture = self._capture
        if capture is not None:
            capture.release()
            self._capture = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None

    def _open(self) -> Optional[cv2.VideoCapture]:
        capture = cv2.VideoCapture(self.capture_source, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            self.logger.warning(
                f'Cannot open camera {self.device}; retrying')
            return None
        capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.fourcc),
        )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
        actual_format = ''.join(
            chr((actual_fourcc >> (8 * index)) & 0xFF)
            for index in range(4)
        )
        self.logger.info(
            f'IRE integrated camera opened: {self.device} '
            f'{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}@'
            f'{capture.get(cv2.CAP_PROP_FPS):g} {actual_format}')
        return capture

    def _run(self) -> None:
        consecutive_failures = 0
        try:
            while not self._stop_event.is_set():
                if self._capture is None:
                    self._capture = self._open()
                    if self._capture is None:
                        self._stop_event.wait(self.reconnect_interval_sec)
                        continue
                    consecutive_failures = 0

                ok, frame = self._capture.read()
                if not ok or frame is None or not frame.size:
                    consecutive_failures += 1
                    if consecutive_failures >= 10:
                        self.logger.error(
                            'IRE integrated camera disconnected after ten '
                            'consecutive read failures')
                        self._capture.release()
                        self._capture = None
                    continue

                consecutive_failures = 0
                captured_at_ns = time.time_ns()
                with self._frame_lock:
                    self._sequence += 1
                    self._latest = (self._sequence, captured_at_ns, frame)
        finally:
            if self._capture is not None:
                self._capture.release()
                self._capture = None
