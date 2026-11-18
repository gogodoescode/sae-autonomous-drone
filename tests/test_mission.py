from types import SimpleNamespace

import pytest

from drone import mission as mission_module
from drone.config import GeoPoint, MissionConfig
from drone.mission import DeliveryMission
from drone.vision.tracker import Detection

HOME = GeoPoint(1.0, 2.0)
DROP = GeoPoint(1.001, 2.001)


class FakeDrone:
    """Records calls; 'lands' after a fixed number of is_landed() polls."""

    def __init__(self, polls_until_landed=3):
        self.calls = []
        self.polls_until_landed = polls_until_landed
        self._polls = 0

    def record_home(self):
        self.calls.append(("record_home",))
        return HOME

    def arm_and_takeoff(self, h):
        self.calls.append(("takeoff", h))

    def goto(self, target, alt, radius):
        self.calls.append(("goto", target, alt))

    def change_altitude(self, alt):
        self.calls.append(("change_altitude", alt))

    def land(self):
        self._polls = 0
        self.calls.append(("land",))

    def is_landed(self):
        self._polls += 1
        return self._polls > self.polls_until_landed

    def send_landing_target(self, ax, ay):
        self.calls.append(("landing_target", ax, ay))

    def send_status_text(self, text):
        self.calls.append(("status_text", text))


class FakePayload:
    def __init__(self, drone):
        self.drone = drone

    def release(self):
        self.drone.calls.append(("release",))


class FakeTracker:
    def __init__(self, data=None):
        self.data = data

    def observe(self):
        obs = SimpleNamespace(angle_x=0.1, angle_y=-0.2)
        return obs, Detection(corners=None, data=self.data)


@pytest.fixture(autouse=True)
def no_sleep(monkeypatch):
    monkeypatch.setattr(mission_module.time, "sleep", lambda s: None)


def config(tmp_path, **kw):
    return MissionConfig(drop_location=DROP, qr_output_file=str(tmp_path / "qr.txt"), **kw)


def names(calls):
    return [c[0] for c in calls]


def test_mission_sequence_without_vision(tmp_path):
    drone = FakeDrone()
    DeliveryMission(drone, FakePayload(drone), config(tmp_path, use_vision=False)).run()
    assert names(drone.calls) == [
        "record_home", "takeoff", "goto", "change_altitude", "land",
        "release", "takeoff", "goto", "land",
    ]
    gotos = [c for c in drone.calls if c[0] == "goto"]
    assert gotos[0][1] == DROP and gotos[1][1] == HOME


def test_precision_landing_sends_targets_and_saves_qr(tmp_path):
    drone = FakeDrone(polls_until_landed=3)
    cfg = config(tmp_path)
    m = DeliveryMission(drone, FakePayload(drone), cfg, drop_tracker=FakeTracker("HELLO"))
    m.run()

    first_land = names(drone.calls).index("land")
    release = names(drone.calls).index("release")
    between = drone.calls[first_land + 1:release]
    assert names(between).count("landing_target") == 3
    assert ("landing_target", 0.1, -0.2) in between
    # Reported once (throttled), saved once.
    assert names(between).count("status_text") == 1
    assert (tmp_path / "qr.txt").read_text() == "HELLO"
    # No tracker for home: no landing targets after the release.
    assert "landing_target" not in names(drone.calls[release:])


def test_home_tracker_does_not_report_qr(tmp_path):
    drone = FakeDrone()
    m = DeliveryMission(drone, FakePayload(drone), config(tmp_path, use_vision=False),
                        home_tracker=FakeTracker("0"))
    m.run()
    assert "status_text" not in names(drone.calls)
    assert names(drone.calls).count("landing_target") == 3
