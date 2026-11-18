"""Configuration objects shared by the mission, CLI and helper scripts."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GeoPoint:
    """A latitude/longitude pair in decimal degrees."""

    lat: float
    lon: float


def parse_drop_location(text: str) -> GeoPoint:
    """Parse ``"lat,lon"`` (optionally followed by heading and altitude).

    The original mission accepted ``lat,lon,heading,alt``; only latitude and
    longitude are used, so the extra values are accepted and ignored.
    """
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if not 2 <= len(parts) <= 4:
        raise ValueError(f"expected 'lat,lon[,heading[,alt]]', got {text!r}")
    try:
        lat, lon = float(parts[0]), float(parts[1])
    except ValueError as exc:
        raise ValueError(f"latitude/longitude must be numbers: {text!r}") from exc
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError(f"latitude/longitude out of range: {text!r}")
    return GeoPoint(lat, lon)


@dataclass(frozen=True)
class ConnectionConfig:
    """How the companion computer reaches the flight controller."""

    # UART on the Raspberry Pi GPIO header. Use e.g. "COM3" from a laptop.
    device: str = "/dev/ttyAMA0"
    baud: int = 57600


@dataclass(frozen=True)
class PayloadConfig:
    """Servo driving the payload-release mechanism."""

    servo_channel: int = 9  # ArduPilot AUX output
    open_pwm: int = 3300
    stop_pwm: int = 0
    open_duration_s: float = 3.0


@dataclass(frozen=True)
class MissionConfig:
    drop_location: GeoPoint
    safe_height_m: float = 15.0
    # Height at which the camera starts looking for the landing pad.
    detection_height_m: float = 10.0
    arrival_radius_m: float = 1.0
    settle_after_drop_s: float = 5.0
    # Side length of the QR code on the drop-zone landing pad.
    qr_size_cm: float = 28.5
    # Side length of the ArUco marker at home (used only for distance estimates).
    aruco_size_cm: float = 15.0
    # Precision landing on the QR pad at the drop zone.
    use_vision: bool = True
    # Precision landing on the ArUco marker when returning home.
    precision_land_home: bool = False
    calibration_file: str = "calibration/camera_calibration.pkl"
    qr_output_file: str = "qr_content.txt"
