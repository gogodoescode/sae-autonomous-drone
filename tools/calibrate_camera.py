"""Chessboard camera calibration.

    python -m tools.calibrate_camera capture   # grab chessboard photos ('c' to save, 'q' to stop)
    python -m tools.calibrate_camera calibrate # compute intrinsics from the photos
    python -m tools.calibrate_camera test      # live original vs undistorted view
    python -m tools.calibrate_camera all       # the three steps in order
"""
from __future__ import annotations

import argparse
import glob
import os
import time

import cv2
import numpy as np

from drone.cli import setup_logging
from drone.vision.calibration import CameraIntrinsics, load_calibration, save_calibration
from drone.vision.camera import open_camera, parse_resolution

DEFAULT_OUTPUT = "calibration/camera_calibration.pkl"


def capture(args) -> int:
    os.makedirs(args.images, exist_ok=True)
    camera = open_camera(parse_resolution(args.resolution))
    count = 0
    try:
        while count < args.count:
            frame = camera.read()
            if frame is None:
                time.sleep(0.1)
                continue
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display = bgr.copy()
            for i, line in enumerate([f"Captured: {count}/{args.count}",
                                      "'c' capture   'q' quit"]):
                cv2.putText(display, line, (10, 30 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Calibration capture", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                path = os.path.join(args.images, f"calib_{count:02d}.jpg")
                cv2.imwrite(path, bgr)
                print(f"Saved {path}")
                count += 1
                time.sleep(0.5)  # avoid duplicate captures
    finally:
        camera.close()
        cv2.destroyAllWindows()
    print(f"Captured {count} images")
    return count


def calibrate(args):
    cols, rows = (int(v) for v in args.board.split(","))
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square_size

    images = sorted(glob.glob(os.path.join(args.images, "*.jpg")))
    if not images:
        raise SystemExit(f"No .jpg images found in {args.images!r}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points, img_points, size = [], [], None
    for path in images:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"  {path}: unreadable")
            continue
        size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        print(f"  {path}: {'ok' if found else 'no chessboard'}")
        if found:
            obj_points.append(objp)
            img_points.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria))

    if not obj_points:
        raise SystemExit("Chessboard not found in any image")

    _, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    intrinsics = CameraIntrinsics(matrix, dist, size)
    save_calibration(args.output, intrinsics)

    errors = [
        cv2.norm(img_points[i], cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], matrix, dist)[0],
                 cv2.NORM_L2) / len(img_points[i])
        for i in range(len(obj_points))
    ]
    print(f"Used {len(obj_points)}/{len(images)} images, image size {size}")
    print("Camera matrix:\n", matrix)
    print("Distortion coefficients:\n", dist.ravel())
    print(f"Mean reprojection error: {np.mean(errors):.4f} px (< 0.5 excellent, < 1.0 good)")
    print(f"Saved to {args.output}")
    return intrinsics


def test(args) -> None:
    intrinsics = load_calibration(args.output)
    camera = open_camera(parse_resolution(args.resolution))
    try:
        size = camera.resolution
        new_matrix, (x, y, w, h) = cv2.getOptimalNewCameraMatrix(
            intrinsics.camera_matrix, intrinsics.dist_coeffs, size, 1, size)
        print("Press 'q' to quit")
        while True:
            frame = camera.read()
            if frame is None:
                continue
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            fixed = cv2.undistort(bgr, intrinsics.camera_matrix, intrinsics.dist_coeffs,
                                  None, new_matrix)[y:y + h, x:x + w]
            both = np.hstack((cv2.resize(bgr, (w, h)), fixed))
            cv2.putText(both, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(both, "Undistorted", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Calibration test", both)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()


def run_all(args) -> None:
    if capture(args) > 0:
        calibrate(args)
        test(args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Chessboard camera calibration")
    parser.add_argument("step", choices=["capture", "calibrate", "test", "all"])
    parser.add_argument("--resolution", default="1920,1080",
                        help="camera resolution W,H; must match the mission (default: 1920,1080)")
    parser.add_argument("--images", default="calibration_images", help="image folder")
    parser.add_argument("--count", type=int, default=20, help="images to capture")
    parser.add_argument("--board", default="9,6", help="inner corners per row,column")
    parser.add_argument("--square-size", type=float, default=2.5, help="square size (cm)")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="calibration file")
    args = parser.parse_args()
    setup_logging()
    {"capture": capture, "calibrate": calibrate, "test": test, "all": run_all}[args.step](args)


if __name__ == "__main__":
    main()
