"""Geographic helpers."""
from __future__ import annotations

import math

EARTH_RADIUS_M = 6_371_000.0


def distance_m(a, b) -> float:
    """Great-circle (haversine) distance in metres between two objects with
    ``lat``/``lon`` attributes (``GeoPoint``, DroneKit locations, ...)."""
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(h))
