"""Payload release mechanism driven by a servo on the flight controller."""
from __future__ import annotations

import logging
import time

from drone.config import PayloadConfig

log = logging.getLogger(__name__)


class PayloadReleaser:
    def __init__(self, drone, config: PayloadConfig = PayloadConfig()) -> None:
        """``drone`` is anything with ``set_servo(channel, pwm)``,
        normally a :class:`drone.vehicle.DroneController`."""
        self.drone = drone
        self.config = config

    def release(self) -> None:
        """Open the mechanism long enough to drop the payload, then stop."""
        cfg = self.config
        log.info("Opening payload mechanism")
        self.drone.set_servo(cfg.servo_channel, cfg.open_pwm)
        time.sleep(cfg.open_duration_s)
        log.info("Stopping payload mechanism")
        self.drone.set_servo(cfg.servo_channel, cfg.stop_pwm)
