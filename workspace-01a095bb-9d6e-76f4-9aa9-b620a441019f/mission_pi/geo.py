"""Flat-earth ENU geo helpers for mission_pi.

Same convention as mission-ui js/geo.js: x = east, y = north, angles in
degrees, yaw clockwise from north. Stdlib only (no numpy/cv2) so the
mission math stays testable anywhere.
"""
import math

R_EARTH_M = 6371000.0


def latlon_to_enu(lat, lon, olat, olon):
    """(lat, lon) -> (x_east, y_north) meters about origin."""
    dlat = math.radians(lat - olat)
    dlon = math.radians(lon - olon)
    return (dlon * R_EARTH_M * math.cos(math.radians(olat)),
            dlat * R_EARTH_M)


def enu_to_latlon(x, y, olat, olon):
    """(x_east, y_north) meters about origin -> (lat, lon)."""
    dlat = y / R_EARTH_M
    dlon = x / (R_EARTH_M * math.cos(math.radians(olat)))
    return (olat + math.degrees(dlat), olon + math.degrees(dlon))


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Compass bearing from point 1 to point 2 (0=N, clockwise)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    b = math.atan2(math.sin(dl) * math.cos(p2),
                   math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return (math.degrees(b) + 360.0) % 360.0


def point_in_polygon(lat, lon, poly):
    """Ray-cast point-in-polygon. poly = [(lat, lon), ...] (any winding)."""
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        yi, xi = poly[i][0], poly[i][1]
        yj, xj = poly[j][0], poly[j][1]
        if ((yi > lat) != (yj > lat)) and \
                (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def polygon_centroid(poly):
    return (sum(p[0] for p in poly) / len(poly),
            sum(p[1] for p in poly) / len(poly))


def polygon_size_m(poly):
    """(width_east_m, height_north_m) of the polygon bbox."""
    olat, olon = polygon_centroid(poly)
    xs = [latlon_to_enu(p[0], p[1], olat, olon)[0] for p in poly]
    ys = [latlon_to_enu(p[0], p[1], olat, olon)[1] for p in poly]
    return (max(xs) - min(xs), max(ys) - min(ys))


def lawnmower_rows(poly, spacing_m, origin=None):
    """Serpentine coverage waypoints over a polygon (the geofence).

    Rows run east-west spaced `spacing_m` apart, clipped to the polygon.
    Returns [(lat, lon), ...] in flyable order. Falls back to the polygon
    centroid if nothing survives the clip (tiny fence).
    """
    if len(poly) < 3 or spacing_m <= 0:
        return [polygon_centroid(poly)] if poly else []
    olat, olon = origin or polygon_centroid(poly)
    enu = [latlon_to_enu(p[0], p[1], olat, olon) for p in poly]
    xs = [p[0] for p in enu]
    ys = [p[1] for p in enu]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    rows = max(1, int((y1 - y0) / spacing_m) + 1)
    wps = []
    for r in range(rows):
        y = y0 + min(r * spacing_m, y1 - y0) if rows > 1 else (y0 + y1) / 2.0
        # scan the row in fine steps, keep inside-points, take the run ends
        n = max(2, int((x1 - x0) / (spacing_m / 4.0)) + 1)
        inside = []
        for i in range(n):
            x = x0 + (x1 - x0) * i / (n - 1)
            la, lo = enu_to_latlon(x, y, olat, olon)
            if point_in_polygon(la, lo, poly):
                inside.append((x, y))
        if not inside:
            continue
        pts = [inside[0], inside[-1]] if len(inside) > 1 else inside
        if r % 2 == 1:
            pts = pts[::-1]
        wps.extend(pts)
    if not wps:
        return [polygon_centroid(poly)]
    return [enu_to_latlon(x, y, olat, olon) for x, y in wps]


def vfov_from_hfov(hfov_deg, img_w, img_h):
    return math.degrees(2.0 * math.atan(
        math.tan(math.radians(hfov_deg) / 2.0) * img_h / img_w))


def footprint_m(alt_m, hfov_deg, img_w, img_h):
    """Ground footprint (width, height) meters for a nadir camera."""
    w = 2.0 * alt_m * math.tan(math.radians(hfov_deg) / 2.0)
    h = 2.0 * alt_m * math.tan(math.radians(vfov_from_hfov(hfov_deg, img_w, img_h)) / 2.0)
    return (w, h)


def _rot_px(px, py, img_w, img_h, rot_deg):
    """Rotate a pixel for a physically rotated camera mount."""
    rot = rot_deg % 360
    if rot == 90:
        return (img_h - 1 - py, px)
    if rot == 180:
        return (img_w - 1 - px, img_h - 1 - py)
    if rot == 270:
        return (py, img_w - 1 - px)
    return (px, py)


def nadir_pixel_to_ground_m(px, py, img_w, img_h, hfov_deg, alt_m, yaw_deg, rot_deg=0):
    """Ground offset (east_m, north_m) of a pixel seen by a NADIR (down)
    camera at `alt_m` AGL with vehicle yaw `yaw_deg`.

    Assumes flat ground and the camera optical axis pointing straight down.
    """
    px, py = _rot_px(px, py, img_w, img_h, rot_deg)
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (img_h / 2.0) / math.tan(math.radians(vfov_from_hfov(hfov_deg, img_w, img_h)) / 2.0)
    # +x right, +y forward (image up = nose direction after rot fix)
    gx_right = alt_m * (px - img_w / 2.0) / fx
    gy_fwd = alt_m * (img_h / 2.0 - py) / fy
    t = math.radians(yaw_deg)
    east = gx_right * math.cos(t) + gy_fwd * math.sin(t)
    north = -gx_right * math.sin(t) + gy_fwd * math.cos(t)
    return (east, north)


def front_pixel_bearing_deg(px, img_w, hfov_deg, yaw_deg):
    """Compass bearing of a pixel seen by a FORWARD camera.

    Range is unobservable with a level monocular camera, so callers get a
    bearing to yaw-toward / step-toward only.
    """
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    off = math.degrees(math.atan((px - img_w / 2.0) / fx))
    return (yaw_deg + off + 360.0) % 360.0


def cover_cells(lat, lon, origin, cell_m, radius_m):
    """Cells (ix, iy) whose centre falls within `radius_m` of (lat, lon).

    Grid: ENU metres about `origin` (lat, lon); cell (ix, iy) spans
    [ix*cell, (ix+1)*cell) x [iy*cell, (iy+1)*cell). The live coverage
    heatmap marks these (mission) and draws them (UI). Set of tuples.
    """
    import math as _m
    cell_m = max(0.5, float(cell_m))
    radius_m = max(0.0, float(radius_m))
    e, n = latlon_to_enu(lat, lon, origin[0], origin[1])
    ix0 = int(_m.floor((e - radius_m) / cell_m))
    ix1 = int(_m.floor((e + radius_m) / cell_m))
    iy0 = int(_m.floor((n - radius_m) / cell_m))
    iy1 = int(_m.floor((n + radius_m) / cell_m))
    out = set()
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            cx, cy = (ix + 0.5) * cell_m, (iy + 0.5) * cell_m
            if (cx - e) ** 2 + (cy - n) ** 2 <= radius_m ** 2 + 1e-9:
                out.add((ix, iy))
    return out
