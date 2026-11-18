"""Camera sources. Frames are always returned as RGB numpy arrays."""
from __future__ import annotations

import logging
import time
from typing import Optional, Tuple

import cv2
import numpy as np

log = logging.getLogger(__name__)

Resolution = Tuple[int, int]


class PiCamera:
    """Raspberry Pi camera through picamera2/libcamera."""

    def __init__(self, resolution: Resolution = (1920, 1080), autofocus: bool = True,
                 warmup_s: float = 1.0) -> None:
        from libcamera import controls
        from picamera2 import Picamera2

        self._cam = Picamera2()
        self._cam.configure(
            self._cam.create_still_configuration(main={"size": resolution, "format": "RGB888"})
        )
        if autofocus:
            self._cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        self._cam.start()
        time.sleep(warmup_s)
        self.resolution = resolution
        log.info("Pi camera started at %dx%d", *resolution)

    def read(self) -> Optional[np.ndarray]:
        return self._cam.capture_array()

    def close(self) -> None:
        self._cam.stop()


class OpenCVCamera:
    """Any webcam OpenCV can open (handy for testing on a laptop)."""

    def __init__(self, resolution: Resolution = (1280, 720), index: int = 0) -> None:
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise IOError(f"cannot open camera {index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.resolution = (
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        log.info("OpenCV camera %d started at %dx%d", index, *self.resolution)

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if ok else None

    def close(self) -> None:
        self._cap.release()


def open_camera(resolution: Resolution = (1280, 720), prefer_pi: bool = True):
    """Open the Pi camera when available, otherwise fall back to OpenCV."""
    if prefer_pi:
        try:
            return PiCamera(resolution)
        except ImportError:
            log.info("picamera2 not installed; using OpenCV camera")
        except Exception as exc:  # camera missing or busy
            log.warning("Pi camera failed (%s); using OpenCV camera", exc)
    return OpenCVCamera(resolution)


def parse_resolution(text: str) -> Resolution:
    width, height = (int(v) for v in text.replace(" ", "").split(","))
    return width, height
