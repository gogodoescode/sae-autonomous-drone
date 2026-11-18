"""The delivery mission: fly out, precision-land, drop the payload, fly home."""
from __future__ import annotations

import logging
import time
from typing import Optional

from drone.config import MissionConfig

log = logging.getLogger(__name__)

# How often the landing loop runs when no camera is used.
IDLE_POLL_S = 1.0
# Minimum time between repeated QR reports to the ground station.
QR_REPORT_INTERVAL_S = 2.0


class DeliveryMission:
    def __init__(self, drone, payload, config: MissionConfig,
                 drop_tracker=None, home_tracker=None) -> None:
        """
        drone:        a :class:`drone.vehicle.DroneController` (or compatible)
        payload:      a :class:`drone.payload.PayloadReleaser` (or compatible)
        drop_tracker: landing-target tracker for the drop-zone QR pad, or None
        home_tracker: landing-target tracker for the home ArUco marker, or None
        """
        self.drone = drone
        self.payload = payload
        self.config = config
        self.drop_tracker = drop_tracker
        self.home_tracker = home_tracker
        self.qr_data: Optional[str] = None
        self._last_qr_report = float("-inf")

    def run(self) -> None:
        cfg = self.config
        home = self.drone.record_home()

        log.info("== Outbound to drop zone")
        self.drone.arm_and_takeoff(cfg.safe_height_m)
        self.drone.goto(cfg.drop_location, cfg.safe_height_m, cfg.arrival_radius_m)
        self.drone.change_altitude(cfg.detection_height_m)

        log.info("== Landing at drop zone")
        self.drone.land()
        self._wait_for_landing(self.drop_tracker)

        log.info("== Dropping payload")
        self.payload.release()
        time.sleep(cfg.settle_after_drop_s)

        log.info("== Returning home")
        self.drone.arm_and_takeoff(cfg.safe_height_m)
        self.drone.goto(home, cfg.safe_height_m, cfg.arrival_radius_m)

        log.info("== Landing at home")
        self.drone.land()
        self._wait_for_landing(self.home_tracker)
        log.info("Mission complete")

    def _wait_for_landing(self, tracker) -> None:
        """Stay in LAND mode until touchdown, steering with the tracker if given."""
        while not self.drone.is_landed():
            if tracker is None:
                time.sleep(IDLE_POLL_S)
                continue
            result = tracker.observe()
            if result is None:
                log.debug("No landing target in view")
                continue
            observation, detection = result
            self.drone.send_landing_target(observation.angle_x, observation.angle_y)
            if tracker is self.drop_tracker and detection.data:
                self._handle_qr(detection.data)

    def _handle_qr(self, data: str) -> None:
        if self.qr_data is None:
            log.info("QR decoded: %s", data)
            self.qr_data = data
            with open(self.config.qr_output_file, "w", encoding="utf-8") as f:
                f.write(data)
        now = time.monotonic()
        if now - self._last_qr_report >= QR_REPORT_INTERVAL_S:
            self.drone.send_status_text(data)
            self._last_qr_report = now
