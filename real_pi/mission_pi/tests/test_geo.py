#!/usr/bin/env python3
"""Stdlib tests for geo.py (run anywhere: python tests/test_geo.py)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import geo  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


# ENU round-trip (~cm accuracy near origin)
olat, olon = 15.35, 75.12
x, y = geo.latlon_to_enu(15.3501, 75.1201, olat, olon)
la, lo = geo.enu_to_latlon(x, y, olat, olon)
check("enu roundtrip", abs(la - 15.3501) < 1e-7 and abs(lo - 75.1201) < 1e-7)
check("enu scale ~11m", 10.0 < abs(x) < 12.0 and 10.0 < abs(y) < 12.0, (x, y))

# point in polygon
sq = [(0, 0), (0, 1), (1, 1), (1, 0)]
check("pip inside", geo.point_in_polygon(0.5, 0.5, sq))
check("pip outside", not geo.point_in_polygon(2.0, 2.0, sq))

# lawnmower over a ~50x50 m square
import math  # noqa
d = 25.0 / 6371000.0 * 180.0 / math.pi
poly = [(olat - d, olon - d), (olat - d, olon + d), (olat + d, olon + d), (olat + d, olon - d)]
rows = geo.lawnmower_rows(poly, 10.0, origin=(olat, olon))
check("lawnmower count", 6 <= len(rows) <= 14, len(rows))
check("lawnmower inside", all(geo.point_in_polygon(la, lo, poly) for la, lo in rows))
w, h = geo.polygon_size_m(poly)
check("poly size ~50m", 45 < w < 60 and 45 < h < 60, (w, h))

# footprint: 15 m, 66 deg HFOV, 16:9 -> ~19.5 x 11 m
fw, fh = geo.footprint_m(15.0, 66.0, 1920, 1080)
check("footprint", 18 < fw < 21 and 10 < fh < 12, (fw, fh))

# nadir projection: center pixel -> ~0 offset; edge -> ~half footprint
e, n = geo.nadir_pixel_to_ground_m(960, 540, 1920, 1080, 66.0, 15.0, 0.0)
check("nadir center", abs(e) < 0.05 and abs(n) < 0.05, (e, n))
e, n = geo.nadir_pixel_to_ground_m(1919, 540, 1920, 1080, 66.0, 15.0, 0.0)
check("nadir right edge ~+9.7E", 9.0 < e < 10.5 and abs(n) < 0.5, (e, n))
e, n = geo.nadir_pixel_to_ground_m(1919, 540, 1920, 1080, 66.0, 15.0, 90.0)
check("nadir yaw90 rotates", abs(e) < 0.5 and -10.5 < n < -9.0, (e, n))

# front bearing: center -> yaw; right edge -> yaw + ~33
check("front bearing", geo.front_pixel_bearing_deg(960, 1920, 66.0, 10.0) == 10.0)
b = geo.front_pixel_bearing_deg(1919, 1920, 66.0, 10.0)
check("front edge bearing", 40 < b < 46, b)

print("FAILURES:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
