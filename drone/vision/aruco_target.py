"""ArUco marker detection, tuned for small (distant) markers.

Works with both the OpenCV >= 4.7 ``ArucoDetector`` API and the older
function-based API that ships on many Raspberry Pi images.
"""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from drone.vision.tracker import Detection

DEFAULT_DICTIONARY = cv2.aruco.DICT_5X5_50
_SHARPEN_KERNEL = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])


def _get_dictionary(dictionary_id: int):
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _make_parameters():
    params = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "DetectorParameters")
        else cv2.aruco.DetectorParameters_create()
    )
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.minMarkerPerimeterRate = 0.01  # accept small, far-away markers
    params.adaptiveThreshConstant = 5
    return params


def enhance_contrast(gray: np.ndarray) -> np.ndarray:
    """CLAHE + sharpening; helps in harsh outdoor lighting."""
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    return cv2.filter2D(clahe.apply(gray), -1, _SHARPEN_KERNEL)


class ArucoDetector:
    def __init__(self, marker_id: Optional[int] = 0, dictionary_id: int = DEFAULT_DICTIONARY,
                 enhance: bool = True) -> None:
        """``marker_id=None`` accepts any marker from the dictionary."""
        self.marker_id = marker_id
        self.enhance = enhance
        dictionary = _get_dictionary(dictionary_id)
        params = _make_parameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect = detector.detectMarkers
        else:
            self._detect = lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)

    def detect(self, frame_rgb: np.ndarray) -> Optional[Detection]:
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY) if frame_rgb.ndim == 3 else frame_rgb
        if self.enhance:
            gray = enhance_contrast(gray)
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return None
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if self.marker_id is None or marker_id == self.marker_id:
                return Detection(marker_corners.reshape(4, 2), data=str(int(marker_id)))
        return None


def generate_marker(marker_id: int = 0, size_px: int = 1000,
                    dictionary_id: int = DEFAULT_DICTIONARY) -> np.ndarray:
    """Printable marker image (grayscale) with a white quiet zone and a label."""
    dictionary = _get_dictionary(dictionary_id)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, borderBits=1)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, size_px, borderBits=1)
    border = size_px // 10
    image = cv2.copyMakeBorder(marker, border, border, border, border,
                               cv2.BORDER_CONSTANT, value=255)
    cv2.putText(image, f"ArUco ID {marker_id}", (border, image.shape[0] - border // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1, cv2.LINE_AA)
    return image
