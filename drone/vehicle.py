"""Thin, blocking wrapper around a DroneKit vehicle."""
from __future__ import annotations

import logging
import time
from typing import Optional

from dronekit import LocationGlobalRelative, VehicleMode, connect
from pymavlink import mavutil

from drone.config import ConnectionConfig, GeoPoint
from drone.geo import distance_m

log = logging.getLogger(__name__)

POLL_INTERVAL_S = 1.0
# Treat 95% of the requested takeoff altitude as "reached".
TAKEOFF_ALTITUDE_RATIO = 0.95
# Give the motors time to spin up after arming before taking off.
MOTOR_SPINUP_S = 3.0
# MAVLink STATUSTEXT carries at most 50 characters.
STATUSTEXT_MAX_LEN = 50


class DroneController:
    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle
        self.home: Optional[GeoPoint] = None

    @classmethod
    def connect(cls, config: ConnectionConfig) -> "DroneController":
        log.info("Connecting to flight controller on %s @ %d baud", config.device, config.baud)
        vehicle = connect(config.device, baud=config.baud, wait_ready=True)
        log.info("Connected")
        return cls(vehicle)

    # --- state -----------------------------------------------------------

    @property
    def altitude_m(self) -> float:
        return self.vehicle.location.global_relative_frame.alt

    @property
    def position(self) -> GeoPoint:
        loc = self.vehicle.location.global_relative_frame
        return GeoPoint(loc.lat, loc.lon)

    def is_landed(self) -> bool:
        # ArduPilot disarms automatically after touchdown in LAND mode.
        return not self.vehicle.armed

    def record_home(self) -> GeoPoint:
        self.home = self.position
        log.info("Home recorded at %s", self.home)
        return self.home

    # --- commands --------------------------------------------------------

    def set_mode(self, name: str) -> None:
        self.vehicle.mode = VehicleMode(name)
        while self.vehicle.mode.name != name:
            log.info("Waiting for %s mode...", name)
            time.sleep(POLL_INTERVAL_S)
        log.info("Vehicle is in %s mode", name)

    def arm_and_takeoff(self, height_m: float) -> None:
        """Arm in GUIDED mode and climb to ``height_m``. Blocks until reached."""
        self.set_mode("GUIDED")
        while not self.vehicle.is_armable:
            log.info("Waiting for vehicle to become armable...")
            time.sleep(POLL_INTERVAL_S)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            log.info("Waiting for vehicle to arm...")
            time.sleep(POLL_INTERVAL_S)
        log.info("Armed")
        time.sleep(MOTOR_SPINUP_S)

        log.info("Taking off to %.1f m", height_m)
        self.vehicle.simple_takeoff(height_m)
        while self.altitude_m < height_m * TAKEOFF_ALTITUDE_RATIO:
            log.info("Altitude: %.1f m", self.altitude_m)
            time.sleep(POLL_INTERVAL_S)
        log.info("Reached %.1f m", height_m)

    def goto(self, target: GeoPoint, altitude_m: float, arrival_radius_m: float = 1.0) -> None:
        """Fly to ``target`` at ``altitude_m``. Blocks until within the radius."""
        self.vehicle.simple_goto(LocationGlobalRelative(target.lat, target.lon, altitude_m))
        while True:
            remaining = distance_m(self.position, target)
            log.info("Distance to target: %.1f m", remaining)
            if remaining <= arrival_radius_m:
                break
            time.sleep(POLL_INTERVAL_S)
        log.info("Reached %s", target)

    def change_altitude(self, altitude_m: float, tolerance_m: float = 0.5) -> None:
        """Climb or descend in place to ``altitude_m``. Blocks until reached."""
        here = self.position
        self.vehicle.simple_goto(LocationGlobalRelative(here.lat, here.lon, altitude_m))
        while abs(self.altitude_m - altitude_m) > tolerance_m:
            log.info("Altitude: %.1f m (target %.1f m)", self.altitude_m, altitude_m)
            time.sleep(POLL_INTERVAL_S)
        log.info("Holding at %.1f m", altitude_m)

    def land(self) -> None:
        """Switch to LAND mode. Use :meth:`is_landed` to wait for touchdown."""
        self.set_mode("LAND")

    def send_landing_target(self, angle_x: float, angle_y: float) -> None:
        """Send a LANDING_TARGET message used by ArduPilot precision landing.

        Requires PLND_ENABLED=1 and PLND_TYPE=1 (companion computer).
        """
        msg = self.vehicle.message_factory.landing_target_encode(
            0,  # time_usec
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            angle_x,
            angle_y,
            0,  # distance (unknown -> rangefinder/baro is used)
            0,  # size_x
            0,  # size_y
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_status_text(self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_ALERT) -> None:
        """Show ``text`` in the ground control station's message log."""
        payload = text[:STATUSTEXT_MAX_LEN].encode("utf-8", errors="replace")
        self.vehicle.message_factory.statustext_send(severity, payload)

    def set_servo(self, channel: int, pwm: int) -> None:
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # confirmation
            channel,
            pwm,
            0, 0, 0, 0, 0,
        )
        self.vehicle.send_mavlink(msg)

    def close(self) -> None:
        self.vehicle.close()
