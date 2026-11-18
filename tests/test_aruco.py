import cv2
import numpy as np

from drone.vision.aruco_target import ArucoDetector, generate_marker


def scene_with_marker(marker_id, offset=(300, 200)):
    marker = generate_marker(marker_id, size_px=200)
    scene = np.full((720, 1280), 255, dtype=np.uint8)
    y, x = offset
    scene[y:y + marker.shape[0], x:x + marker.shape[1]] = marker
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2RGB)


def test_detects_requested_marker():
    det = ArucoDetector(marker_id=7).detect(scene_with_marker(7))
    assert det is not None
    assert det.data == "7"
    assert det.corners.shape == (4, 2)


def test_ignores_other_marker_ids():
    assert ArucoDetector(marker_id=3).detect(scene_with_marker(7)) is None


def test_any_marker_when_id_is_none():
    assert ArucoDetector(marker_id=None).detect(scene_with_marker(12)) is not None


def test_empty_scene():
    blank = np.full((720, 1280, 3), 255, dtype=np.uint8)
    assert ArucoDetector().detect(blank) is None
