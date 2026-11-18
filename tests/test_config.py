import pytest

from drone.config import GeoPoint, parse_drop_location


def test_parses_lat_lon():
    assert parse_drop_location("12.5, 77.25") == GeoPoint(12.5, 77.25)


def test_ignores_heading_and_altitude():
    assert parse_drop_location("12.5,77.25,180,20") == GeoPoint(12.5, 77.25)


@pytest.mark.parametrize("text", ["12.5", "a,b", "1,2,3,4,5", "91,0", "0,181"])
def test_rejects_bad_input(text):
    with pytest.raises(ValueError):
        parse_drop_location(text)
