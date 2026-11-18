"""Live preview of landing-target detection (no drone needed).

    python -m tools.target_preview qr --size 28.5
    python -m tools.target_preview aruco --size 15 --marker-id 0

Keys: 'q' quit, 'e' toggle ArUco contrast enhancement.
"""
from __future__ import annotations

import argparse
import math

import cv2
import numpy as np

from drone.cli import setup_logging
from drone.vision.calibration import CalibrationError, CameraIntrinsics, load_calibration
from drone.vision.camera import open_camera, parse_resolution
from drone.vision.geometry import estimate_target


def build_detector(args):
    if args.target == "qr":
        from drone.vision.qr_target import QRDetector
        return QRDetector()
    from drone.vision.aruco_target import ArucoDetector
    marker_id = None if args.marker_id < 0 else args.marker_id
    return ArucoDetector(marker_id=marker_id)


def draw(image, k: CameraIntrinsics, obs, det) -> None:
    cv2.drawMarker(image, (int(k.cx), int(k.cy)), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
    if obs is None:
        cv2.putText(image, "No target", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return
    pts = obs.corners.astype(np.int32)
    cv2.polylines(image, [pts], True, (0, 255, 0) if det.data else (0, 0, 255), 2)
    center = tuple(int(v) for v in pts.mean(axis=0))
    cv2.line(image, (int(k.cx), int(k.cy)), center, (0, 255, 255), 2)
    lines = [
        f"angle x: {math.degrees(obs.angle_x):+.1f} deg",
        f"angle y: {math.degrees(obs.angle_y):+.1f} deg",
        f"distance: {obs.distance_m:.2f} m",
        f"data: {det.data[:20]}" if det.data else "data: (not decoded)",
    ]
    for i, line in enumerate(lines):
        cv2.putText(image, line, (10, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Landing-target detection preview")
    parser.add_argument("target", choices=["qr", "aruco"])
    parser.add_argument("--size", type=float, default=None,
                        help="target side length in cm (default: 28.5 for QR, 15 for ArUco)")
    parser.add_argument("--marker-id", type=int, default=0, help="ArUco id, -1 for any")
    parser.add_argument("--calibration", default="calibration/camera_calibration.pkl")
    parser.add_argument("--resolution", default="1280,720")
    args = parser.parse_args()
    setup_logging()

    size_cm = args.size or (28.5 if args.target == "qr" else 15.0)
    camera = open_camera(parse_resolution(args.resolution))
    try:
        intrinsics = load_calibration(args.calibration)
    except CalibrationError as exc:
        print(f"{exc}\nUsing estimated intrinsics: distances will be approximate.")
        intrinsics = CameraIntrinsics.estimated(*camera.resolution)

    detector = build_detector(args)
    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue
            det = detector.detect(frame)
            obs = estimate_target(det.corners, size_cm / 100, intrinsics) if det else None
            image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            draw(image, intrinsics, obs, det)
            cv2.imshow("Landing target preview", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("e") and hasattr(detector, "enhance"):
                detector.enhance = not detector.enhance
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
