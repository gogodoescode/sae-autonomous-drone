"""Pure geometry for turning detected corners into landing-target angles."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from drone.vision.calibration import CameraIntrinsics


@dataclass(frozen=True, eq=False)
class TargetObservation:
    """What ArduPilot's LANDING_TARGET message needs, plus pose for debugging.

    Angles are radians from the camera's optical axis (x right, y down).
    """

    angle_x: float
    angle_y: float
    distance_m: float
    size_x: float
    size_y: float
    rvec: np.ndarray
    tvec: np.ndarray
    corners: np.ndarray  # 4x2, ordered TL, TR, BR, BL


def order_corners(points) -> np.ndarray:
    """Return four points ordered top-left, top-right, bottom-right, bottom-left."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    by_y = pts[np.argsort(pts[:, 1])]
    top = by_y[:2][np.argsort(by_y[:2, 0])]
    bottom = by_y[2:][np.argsort(by_y[2:, 0])]
    return np.array([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)


def square_around(points) -> np.ndarray:
    """Approximate an arbitrary polygon by an axis-aligned square with the same
    centre and mean side length (used when a detector returns != 4 points)."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    cx, cy = pts.mean(axis=0)
    side = np.mean(np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1))
    h = side / 2
    return np.array(
        [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]],
        dtype=np.float32,
    )


def square_object_points(size_m: float) -> np.ndarray:
    """3D corners of a flat square target centred on the origin (TL, TR, BR, BL)."""
    h = size_m / 2
    return np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]], dtype=np.float32)


def estimate_target(
    corners, size_m: float, intrinsics: CameraIntrinsics
) -> Optional[TargetObservation]:
    """Estimate the target's direction and distance from its image corners."""
    pts = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 3:
        return None
    pts = order_corners(pts) if len(pts) == 4 else square_around(pts)

    ok, rvec, tvec = cv2.solvePnP(
        square_object_points(size_m), pts, intrinsics.camera_matrix, intrinsics.dist_coeffs
    )
    if not ok:
        return None

    center_x, center_y = pts.mean(axis=0)
    width = np.linalg.norm(pts[0] - pts[1])
    height = np.linalg.norm(pts[0] - pts[3])
    return TargetObservation(
        angle_x=math.atan2(center_x - intrinsics.cx, intrinsics.fx),
        angle_y=math.atan2(center_y - intrinsics.cy, intrinsics.fy),
        distance_m=float(tvec.flatten()[2]),
        size_x=math.atan2(width, intrinsics.fx),
        size_y=math.atan2(height, intrinsics.fy),
        rvec=rvec,
        tvec=tvec,
        corners=pts,
    )
