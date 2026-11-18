"""Generate a printable ArUco marker (DICT_5X5_50).

    python -m tools.generate_aruco_marker --id 0 --pixels 1000

Print it as large as possible on matte paper (at least 15-20 cm).
"""
from __future__ import annotations

import argparse
import os

import cv2

from drone.vision.aruco_target import generate_marker


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--id", type=int, default=0, help="marker id (0-49)")
    parser.add_argument("--pixels", type=int, default=1000, help="marker size in pixels")
    parser.add_argument("--out-dir", default="assets")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, f"aruco_5x5_50_id{args.id}.png")
    cv2.imwrite(path, generate_marker(args.id, args.pixels))
    print(f"Saved {path}")


if __name__ == "__main__":
    main()
