"""Arm in GUIDED mode, wait, then disarm. Motors will spin: remove props or clear the area.

    python -m scripts.arm_test --device COM3 --seconds 10
"""
from __future__ import annotations

import argparse
import time

from drone.cli import add_connection_args, connection_from_args, setup_logging
from drone.vehicle import POLL_INTERVAL_S, DroneController


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=10, help="time to stay armed")
    add_connection_args(parser)
    args = parser.parse_args()
    setup_logging(args.verbose)

    drone = DroneController.connect(connection_from_args(args))
    v = drone.vehicle
    try:
        drone.set_mode("GUIDED")
        while not v.is_armable:
            print("Waiting for vehicle to become armable...")
            time.sleep(POLL_INTERVAL_S)
        v.armed = True
        while not v.armed:
            print("Waiting for vehicle to arm...")
            time.sleep(POLL_INTERVAL_S)
        print(f"Armed; disarming in {args.seconds:g} s")
        time.sleep(args.seconds)
    finally:
        v.armed = False
        while v.armed:
            print("Waiting for vehicle to disarm...")
            time.sleep(POLL_INTERVAL_S)
        print("Disarmed")
        drone.close()


if __name__ == "__main__":
    main()
