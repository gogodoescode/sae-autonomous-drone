"""Combine a camera, a detector and calibration into landing-target observations."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from drone.vision.calibration import CameraIntrinsics
from drone.vision.geometry import TargetObservation, estimate_target


@dataclass(frozen=True, eq=False)
class Detection:
    corners: np.ndarray  # Nx2 image points
    data: Optional[str] = None  # decoded payload (QR text) if available


class LandingTargetTracker:
    def __init__(self, camera, detector, intrinsics: CameraIntrinsics, target_size_m: float) -> None:
        """``detector`` is anything with ``detect(rgb_frame) -> Optional[Detection]``."""
        self.camera = camera
        self.detector = detector
        self.intrinsics = intrinsics
        self.target_size_m = target_size_m

    def observe(self):
        """Grab one frame. Returns ``(observation, detection)`` or ``None``."""
        frame = self.camera.read()
        if frame is None:
            return None
        detection = self.detector.detect(frame)
        if detection is None:
            return None
        observation: Optional[TargetObservation] = estimate_target(
            detection.corners, self.target_size_m, self.intrinsics
        )
        if observation is None:
            return None
        return observation, detection
