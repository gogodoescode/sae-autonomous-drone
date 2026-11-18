"""Run the delivery mission.

    python -m drone --drop-location 12.9716,77.5946 --safe-height 15
"""
from __future__ import annotations

import argparse
import logging
from typing import List, Optional

from drone.cli import add_connection_args, connection_from_args, setup_logging
from drone.config import MissionConfig, parse_drop_location

log = logging.getLogger("drone")


def build_parser() -> argparse.ArgumentParser:
    d = MissionConfig(drop_location=None)  # defaults only
    p = argparse.ArgumentParser(prog="python -m drone", description=__doc__.splitlines()[0])
    p.add_argument("--drop-location", "--drop_location", required=True, type=parse_drop_location,
                   metavar="LAT,LON", help="drop zone coordinates")
    p.add_argument("--safe-height", "--safe_height", type=float, default=d.safe_height_m,
                   help=f"cruise altitude in metres (default: {d.safe_height_m})")
    p.add_argument("--detection-height", type=float, default=d.detection_height_m,
                   help=f"altitude to descend to before landing (default: {d.detection_height_m})")
    p.add_argument("--qr-size", "--qr_code_size", type=float, default=d.qr_size_cm,
                   help=f"QR landing pad side length in cm (default: {d.qr_size_cm})")
    p.add_argument("--aruco-size", type=float, default=d.aruco_size_cm,
                   help=f"home ArUco marker side length in cm (default: {d.aruco_size_cm})")
    p.add_argument("--no-vision", action="store_true",
                   help="plain GPS landing at the drop zone, no camera")
    p.add_argument("--precision-land-home", action="store_true",
                   help="use the ArUco marker for the landing at home")
    p.add_argument("--calibration", default=d.calibration_file,
                   help=f"camera calibration file (default: {d.calibration_file})")
    p.add_argument("--resolution", default="1920,1080", help="camera resolution W,H")
    add_connection_args(p)
    return p


def config_from_args(args: argparse.Namespace) -> MissionConfig:
    return MissionConfig(
        drop_location=args.drop_location,
        safe_height_m=args.safe_height,
        detection_height_m=args.detection_height,
        qr_size_cm=args.qr_size,
        aruco_size_cm=args.aruco_size,
        use_vision=not args.no_vision,
        precision_land_home=args.precision_land_home,
        calibration_file=args.calibration,
    )


def main(argv: Optional[List[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    setup_logging(args.verbose)
    config = config_from_args(args)

    # Imported here so --help works on machines without DroneKit / a camera.
    from drone.mission import DeliveryMission
    from drone.payload import PayloadReleaser
    from drone.vehicle import DroneController

    camera = drop_tracker = home_tracker = None
    if config.use_vision or config.precision_land_home:
        from drone.vision.calibration import load_calibration
        from drone.vision.camera import PiCamera, parse_resolution
        from drone.vision.tracker import LandingTargetTracker

        intrinsics = load_calibration(config.calibration_file)
        camera = PiCamera(parse_resolution(args.resolution))
        if config.use_vision:
            from drone.vision.qr_target import QRDetector
            drop_tracker = LandingTargetTracker(camera, QRDetector(), intrinsics,
                                                config.qr_size_cm / 100)
        if config.precision_land_home:
            from drone.vision.aruco_target import ArucoDetector
            home_tracker = LandingTargetTracker(camera, ArucoDetector(), intrinsics,
                                                config.aruco_size_cm / 100)

    drone = DroneController.connect(connection_from_args(args))
    try:
        DeliveryMission(drone, PayloadReleaser(drone), config,
                        drop_tracker=drop_tracker, home_tracker=home_tracker).run()
    finally:
        drone.close()
        if camera is not None:
            camera.close()


if __name__ == "__main__":
    main()
