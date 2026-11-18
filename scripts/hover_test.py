"""Take off to a low altitude, hover briefly, then land. FLIGHT TEST.

    python -m scripts.hover_test --device COM3 --height 5
"""
from __future__ import annotations

import argparse
import time

from drone.cli import add_connection_args, connection_from_args, setup_logging
from drone.vehicle import POLL_INTERVAL_S, DroneController


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=float, default=5, help="takeoff altitude (m)")
    parser.add_argument("--hover", type=float, default=0, help="seconds to hover before landing")
    add_connection_args(parser)
    args = parser.parse_args()
    setup_logging(args.verbose)

    drone = DroneController.connect(connection_from_args(args))
    try:
        drone.arm_and_takeoff(args.height)
        time.sleep(args.hover)
    finally:
        drone.land()
        while not drone.is_landed():
            print(f"Landing... altitude {drone.altitude_m:.1f} m")
            time.sleep(POLL_INTERVAL_S)
        print("Landed and disarmed")
        drone.close()


if __name__ == "__main__":
    main()
