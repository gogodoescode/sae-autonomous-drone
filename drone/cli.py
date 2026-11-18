"""Argument helpers shared by the mission entry point and scripts."""
from __future__ import annotations

import argparse
import logging

from drone.config import ConnectionConfig


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    defaults = ConnectionConfig()
    parser.add_argument("--device", default=defaults.device,
                        help=f"serial port or connection string (default: {defaults.device})")
    parser.add_argument("--baud", type=int, default=defaults.baud,
                        help=f"serial baud rate (default: {defaults.baud})")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")


def connection_from_args(args: argparse.Namespace) -> ConnectionConfig:
    return ConnectionConfig(device=args.device, baud=args.baud)


def setup_logging(verbose: bool = False) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
