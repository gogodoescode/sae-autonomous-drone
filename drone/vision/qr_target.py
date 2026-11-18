"""QR-code landing pad detection.

pyzbar is tried first because it also decodes the payload. When the code is
too small or blurry to decode (i.e. from altitude), OpenCV's QR detector can
still locate it from the finder patterns, which is enough to steer towards it.
"""
from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np
from pyzbar.pyzbar import decode

from drone.vision.tracker import Detection

log = logging.getLogger(__name__)


class QRDetector:
    def __init__(self, eps_x: float = 0.9, eps_y: float = 0.9) -> None:
        # Larger epsilons make finder-pattern detection more tolerant
        # (OpenCV defaults are 0.2 / 0.1).
        self._cv_detector = cv2.QRCodeDetector()
        self._cv_detector.setEpsX(eps_x)
        self._cv_detector.setEpsY(eps_y)

    def detect(self, frame_rgb: np.ndarray) -> Optional[Detection]:
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        decoded = decode(gray)
        if decoded:
            code = decoded[0]
            corners = np.array([(p.x, p.y) for p in code.polygon], dtype=np.float32)
            return Detection(corners, code.data.decode("utf-8", errors="replace"))

        found, points = self._cv_detector.detect(gray)
        if found and points is not None and len(points) > 0 and np.isfinite(points).all():
            log.debug("QR located from finder patterns (not decoded)")
            return Detection(points.reshape(-1, 2).astype(np.float32))
        return None
