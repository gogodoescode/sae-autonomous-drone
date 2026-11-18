import pytest

from drone.config import GeoPoint
from drone.geo import distance_m


def test_same_point_is_zero():
    p = GeoPoint(12.8, 77.8)
    assert distance_m(p, p) == 0


def test_one_degree_of_latitude():
    assert distance_m(GeoPoint(0, 0), GeoPoint(1, 0)) == pytest.approx(111_195, rel=1e-3)


def test_longitude_shrinks_with_latitude():
    at_equator = distance_m(GeoPoint(0, 0), GeoPoint(0, 1))
    at_60 = distance_m(GeoPoint(60, 0), GeoPoint(60, 1))
    assert at_60 == pytest.approx(at_equator / 2, rel=1e-2)
