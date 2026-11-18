"""Actuate the payload-release servo once, on the ground.

    python -m scripts.payload_test --device /dev/ttyAMA0
"""
from __future__ import annotations

import argparse

from drone.cli import add_connection_args, connection_from_args, setup_logging
from drone.config import PayloadConfig
from drone.payload import PayloadReleaser
from drone.vehicle import DroneController


def main() -> None:
    defaults = PayloadConfig()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--servo", type=int, default=defaults.servo_channel, help="servo channel")
    parser.add_argument("--open-pwm", type=int, default=defaults.open_pwm)
    parser.add_argument("--duration", type=float, default=defaults.open_duration_s,
                        help="seconds to keep the mechanism open")
    add_connection_args(parser)
    args = parser.parse_args()
    setup_logging(args.verbose)

    config = PayloadConfig(servo_channel=args.servo, open_pwm=args.open_pwm,
                           open_duration_s=args.duration)
    drone = DroneController.connect(connection_from_args(args))
    try:
        PayloadReleaser(drone, config).release()
    finally:
        drone.close()


if __name__ == "__main__":
    main()
