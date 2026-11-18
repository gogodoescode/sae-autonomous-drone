# SAE Autonomous Drone

Companion-computer software for an autonomous payload-delivery quadcopter built
for the SAE Aero Design / ADDC challenge. A Raspberry Pi talks to an ArduPilot
flight controller over MAVLink (DroneKit) and uses the Pi camera to
precision-land on a QR-code landing pad.

## Mission

```
record home ─► arm & take off to safe height ─► fly to drop zone
     ─► descend to detection height ─► LAND, steering onto the QR pad
     ─► release payload ─► take off ─► fly home ─► LAND (optionally onto an ArUco marker)
```

While landing at the drop zone the QR code is decoded, saved to
`qr_content.txt`, and shown in the ground control station as a status message.

## Hardware

| Part | Notes |
|---|---|
| Flight controller | ArduPilot Copter, TELEM port wired to the Pi UART (`/dev/ttyAMA0`, 57600 baud) |
| Companion computer | Raspberry Pi running Raspberry Pi OS |
| Camera | Pi camera module (picamera2), pointing straight down |
| Payload release | Servo on AUX output 9 |

Precision landing must be enabled on the flight controller:

```
PLND_ENABLED = 1
PLND_TYPE    = 1   # companion computer
```

`config/ardupilot.parm` is a full parameter dump from the team's vehicle, for reference.

## Layout

```
drone/                 the application package
  __main__.py          mission entry point (python -m drone)
  mission.py           the delivery sequence
  vehicle.py           blocking DroneKit wrapper (modes, takeoff, goto, MAVLink messages)
  payload.py           servo-driven payload release
  config.py            configuration dataclasses
  geo.py               GPS distance
  vision/
    camera.py          Pi camera, with an OpenCV webcam fallback
    calibration.py     load/save camera intrinsics
    geometry.py        corners -> angles/distance (solvePnP)
    qr_target.py       QR pad detection (pyzbar decode, OpenCV finder-pattern fallback)
    aruco_target.py    ArUco detection and marker generation
    tracker.py         camera + detector + calibration -> observations
tools/                 bench tools (calibration, live preview, marker generator)
scripts/               hardware checks (connection, arm, hover, payload)
tests/                 unit tests; run on any laptop
calibration/           camera calibration used by the mission
assets/                printable ArUco marker
config/                ArduPilot parameter dump
```

## Setup (Raspberry Pi)

DroneKit 2.9.2 doesn't work on Python 3.10 or newer, so use Python 3.9.

```bash
sudo apt install python3-picamera2 libzbar0
python3.9 -m venv --system-site-packages .venv   # system packages give access to picamera2
source .venv/bin/activate
pip install -r requirements.txt
```

## Running the mission

```bash
python -m drone --drop-location 12.9716,77.5946 --safe-height 15 --qr-size 28.5
```

| Option | Default | |
|---|---|---|
| `--drop-location LAT,LON` | required | drop zone coordinates |
| `--safe-height` | 15 m | cruise altitude |
| `--detection-height` | 10 m | altitude to descend to before landing |
| `--qr-size` | 28.5 cm | QR pad side length |
| `--no-vision` | off | GPS-only landing at the drop zone |
| `--precision-land-home` | off | land at home on the ArUco marker (`assets/`) |
| `--device` / `--baud` | `/dev/ttyAMA0` / 57600 | flight controller link |
| `--calibration` | `calibration/camera_calibration.pkl` | camera intrinsics |

The old underscore flags (`--drop_location`, `--safe_height`, `--qr_code_size`) still work.

## Bench tools

```bash
python -m tools.calibrate_camera all          # capture chessboard photos, calibrate, verify
python -m tools.target_preview qr             # live QR pad detection overlay
python -m tools.target_preview aruco          # live ArUco detection overlay
python -m tools.generate_aruco_marker --id 0  # printable marker into assets/
```

Calibrate at the same resolution the mission uses (1920x1080 by default).

## Hardware checks

Run these in order when bringing up a new build. Use `--device COM3` (or similar)
when connected from a laptop over USB telemetry.

```bash
python -m scripts.check_connection   # read-only status
python -m scripts.payload_test       # actuate the release servo
python -m scripts.arm_test           # arms the motors: props off
python -m scripts.hover_test         # FLIGHT: take off to 5 m and land
```

## Tests

```bash
pip install -r requirements-dev.txt
pytest
```

The tests cover the geometry, ArUco detection on synthetic images, config
parsing, and the mission sequence against a fake drone. They don't need
DroneKit, a camera, or a flight controller.

## Credits

Rewritten from the team's original prototype,
[anuragrao04/sae-addc-drone](https://github.com/anuragrao04/sae-addc-drone).
The precision-landing approach follows
[dronedojo/pidronescripts](https://github.com/dronedojo/pidronescripts).
