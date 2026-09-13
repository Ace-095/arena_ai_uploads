"""
Local ENU (East-North-Up) conversion and polygon geometry helpers.

Uses an equirectangular (flat-earth) approximation around a local origin.
This is intentional, not a shortcut: at field scale (hundreds of meters),
the error versus a full geodesic/UTM projection is sub-millimeter, and it
avoids pulling in a projection library on the Pi. If this service is ever
reused for something spanning kilometers, switch to a proper projection —
the approximation degrades with distance from the origin and with latitude.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

EARTH_RADIUS_M = 6371000.0


@dataclass(frozen=True)
class ENU:
    x: float  # meters east of origin
    y: float  # meters north of origin


def latlon_to_enu(lat: float, lon: float, origin_lat: float, origin_lon: float) -> ENU:
    """Convert a lat/lon to local meters relative to origin_lat/origin_lon."""
    dlat_rad = math.radians(lat - origin_lat)
    dlon_rad = math.radians(lon - origin_lon)
    origin_lat_rad = math.radians(origin_lat)

    north = dlat_rad * EARTH_RADIUS_M
    east = dlon_rad * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    return ENU(x=east, y=north)


def enu_to_latlon(x: float, y: float, origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    """Inverse of latlon_to_enu."""
    origin_lat_rad = math.radians(origin_lat)
    dlat = math.degrees(y / EARTH_RADIUS_M)
    dlon = math.degrees(x / (EARTH_RADIUS_M * math.cos(origin_lat_rad)))
    return origin_lat + dlat, origin_lon + dlon


def polygon_centroid(points: List[ENU]) -> ENU:
    """
    Area-weighted centroid (not just the average of vertices, which biases
    toward wherever vertices happen to be clustered on an irregular polygon).
    Falls back to the vertex average for degenerate (near-zero-area) shapes.
    """
    n = len(points)
    area = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(n):
        x0, y0 = points[i].x, points[i].y
        x1, y1 = points[(i + 1) % n].x, points[(i + 1) % n].y
        cross = x0 * y1 - x1 * y0
        area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    area *= 0.5
    if abs(area) < 1e-9:
        avg_x = sum(p.x for p in points) / n
        avg_y = sum(p.y for p in points) / n
        return ENU(x=avg_x, y=avg_y)
    cx /= 6 * area
    cy /= 6 * area
    return ENU(x=cx, y=cy)


def max_radius_from_point(points: List[ENU], origin: ENU) -> float:
    """Largest distance from `origin` to any vertex — the search algorithm's
    worst-case range, used instead of a hardcoded 'search radius' constant."""
    return max(math.hypot(p.x - origin.x, p.y - origin.y) for p in points)


def point_in_polygon(point: ENU, polygon: List[ENU]) -> bool:
    """
    Standard ray-casting point-in-polygon test.

    This is a SOFTWARE-SIDE convenience check for the search algorithm
    (e.g. "am I still inside the area I'm supposed to be searching").
    It is NOT a safety boundary. The Pixhawk's own fence enforcement,
    running independently on the flight controller, is the safety-critical
    check — see mavlink_fence.py. Do not remove the FC-side fence and rely
    on this function instead.
    """
    x, y = point.x, point.y
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i].x, polygon[i].y
        xj, yj = polygon[j].x, polygon[j].y
        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi
        )
        if intersect:
            inside = not inside
        j = i
    return inside
