"""Connect to the flight controller and print its status. Props can stay off.

    python -m scripts.check_connection --device /dev/ttyAMA0
"""
from __future__ import annotations

import argparse

from drone.cli import add_connection_args, connection_from_args, setup_logging
from drone.vehicle import DroneController


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_connection_args(parser)
    args = parser.parse_args()
    setup_logging(args.verbose)

    drone = DroneController.connect(connection_from_args(args))
    try:
        v = drone.vehicle
        print(f"GPS:            {v.gps_0}")
        print(f"Battery:        {v.battery}")
        print(f"Last heartbeat: {v.last_heartbeat}")
        print(f"Armable:        {v.is_armable}")
        print(f"System status:  {v.system_status.state}")
        print(f"Mode:           {v.mode.name}")
        print(f"Position:       {drone.position}, alt {drone.altitude_m} m")
    finally:
        drone.close()


if __name__ == "__main__":
    main()
