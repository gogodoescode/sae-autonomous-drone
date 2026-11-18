"""Camera intrinsics: loading, saving and estimating."""
from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


class CalibrationError(RuntimeError):
    pass


@dataclass(frozen=True, eq=False)
class CameraIntrinsics:
    camera_matrix: np.ndarray  # 3x3
    dist_coeffs: np.ndarray
    image_size: Optional[Tuple[int, int]] = None  # (width, height)

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1, 2])

    @classmethod
    def estimated(cls, width: int, height: int) -> "CameraIntrinsics":
        """Rough intrinsics for an uncalibrated camera (focal length ~ width)."""
        matrix = np.array(
            [[width, 0, width / 2], [0, width, height / 2], [0, 0, 1]], dtype=np.float64
        )
        return cls(matrix, np.zeros((5, 1)), (width, height))


def load_calibration(path: str) -> CameraIntrinsics:
    """Load a calibration file written by ``tools/calibrate_camera.py``.

    Only load files you produced yourself: the format is a Python pickle.
    """
    try:
        with open(path, "rb") as f:
            data = pickle.load(f)
        return CameraIntrinsics(
            camera_matrix=np.asarray(data["camera_matrix"]),
            dist_coeffs=np.asarray(data["dist_coeffs"]),
            image_size=tuple(data["img_size"]) if data.get("img_size") else None,
        )
    except (OSError, KeyError, pickle.UnpicklingError) as exc:
        raise CalibrationError(
            f"could not load camera calibration from {path!r}; "
            "run tools/calibrate_camera.py first"
        ) from exc


def save_calibration(path: str, intrinsics: CameraIntrinsics) -> None:
    data = {
        "camera_matrix": intrinsics.camera_matrix,
        "dist_coeffs": intrinsics.dist_coeffs,
        "img_size": intrinsics.image_size,
    }
    with open(path, "wb") as f:
        pickle.dump(data, f)
