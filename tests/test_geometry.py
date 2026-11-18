import math

import cv2
import numpy as np
import pytest

from drone.vision.calibration import CameraIntrinsics
from drone.vision.geometry import (
    estimate_target,
    order_corners,
    square_around,
    square_object_points,
)

INTRINSICS = CameraIntrinsics.estimated(1280, 720)


def project(size_m, tvec):
    pts, _ = cv2.projectPoints(
        square_object_points(size_m), np.zeros(3), np.asarray(tvec, dtype=np.float64),
        INTRINSICS.camera_matrix, INTRINSICS.dist_coeffs,
    )
    return pts.reshape(-1, 2)


def test_order_corners():
    shuffled = [[10, 10], [0, 10], [10, 0], [0, 0]]
    expected = [[0, 0], [10, 0], [10, 10], [0, 10]]
    np.testing.assert_array_equal(order_corners(shuffled), expected)


def test_square_around_keeps_center_and_side():
    sq = square_around([[0, 0], [4, 0], [4, 4], [0, 4]])
    np.testing.assert_allclose(sq.mean(axis=0), [2, 2])
    assert np.linalg.norm(sq[0] - sq[1]) == pytest.approx(4)


def test_target_straight_below():
    obs = estimate_target(project(0.285, [0, 0, 3.0]), 0.285, INTRINSICS)
    assert obs.distance_m == pytest.approx(3.0, rel=1e-3)
    assert obs.angle_x == pytest.approx(0, abs=1e-6)
    assert obs.angle_y == pytest.approx(0, abs=1e-6)


def test_target_offset_gives_angles():
    obs = estimate_target(project(0.285, [0.5, -0.25, 2.0]), 0.285, INTRINSICS)
    assert obs.angle_x == pytest.approx(math.atan(0.5 / 2.0), abs=1e-3)
    assert obs.angle_y == pytest.approx(math.atan(-0.25 / 2.0), abs=1e-3)
    assert obs.distance_m == pytest.approx(2.0, rel=1e-2)


def test_too_few_points():
    assert estimate_target([[0, 0], [1, 1]], 0.1, INTRINSICS) is None
