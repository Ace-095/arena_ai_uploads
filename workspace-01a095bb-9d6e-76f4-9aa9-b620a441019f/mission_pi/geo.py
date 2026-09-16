"""Flat-earth ENU geo helpers for mission_pi.

Same convention as mission-ui js/geo.js: x = east, y = north, angles in
degrees, yaw clockwise from north. Stdlib only (no numpy/cv2) so the
mission math stays testable anywhere.

Enhanced for arena/test1:
 - configurable max height via config.yaml (flight.max_alt_m / mission.max_alt_m)
 - camera FOV based coverage (cam.txt specs: Pi Cam3 66° HFOV, IMX477 100° HFOV)
 - polygon -> fence inclusion conversion helpers
 - optimal grid spacing ensuring max area coverage with no gaps
"""
import math

R_EARTH_M = 6371000.0

# --- Camera specs from cam.txt (arena/test1) ---
# Pi Camera Module 3 Standard
CAM3_STANDARD = {
    "model": "Pi Camera Module 3 Standard",
    "sensor": "IMX708",
    "hfov_deg": 66.0,
    "vfov_deg": 41.0,
    "diagonal_fov_deg": 75.0,
    "focal_mm": 4.74,
    "resolution": (4608, 2592),
}

# Waveshare IMX477 IR-CUT B, 113° diagonal
IMX477_B = {
    "model": "Waveshare IMX477 IR-CUT B",
    "sensor": "IMX477",
    "hfov_deg": 100.0,  # derived from 113° diagonal on 4:3
    "vfov_deg": 75.0,
    "diagonal_fov_deg": 113.0,
    "focal_mm": 2.7,
    "resolution": (4056, 3040),
}


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


def point_in_polygon_enu(x, y, enu_poly):
    """ENU version of point_in_polygon for faster checks."""
    inside = False
    n = len(enu_poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        yi, xi = enu_poly[i][1], enu_poly[i][0]  # y=lat-like, x=lon-like
        yj, xj = enu_poly[j][1], enu_poly[j][0]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def polygon_centroid(poly):
    return (sum(p[0] for p in poly) / len(poly),
            sum(p[1] for p in poly) / len(poly))


def polygon_area_m2_enu(enu):
    """Shoelace area in m² for ENU polygon."""
    a = 0.0
    for i in range(len(enu)):
        x0, y0 = enu[i]
        x1, y1 = enu[(i + 1) % len(enu)]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def polygon_area_m2(poly, origin=None):
    """Area in m² for latlon polygon."""
    if len(poly) < 3:
        return 0.0
    olat, olon = origin or polygon_centroid(poly)
    enu = [latlon_to_enu(p[0], p[1], olat, olon) for p in poly]
    return polygon_area_m2_enu(enu)


def polygon_size_m(poly):
    """(width_east_m, height_north_m) of the polygon bbox."""
    olat, olon = polygon_centroid(poly)
    xs = [latlon_to_enu(p[0], p[1], olat, olon)[0] for p in poly]
    ys = [latlon_to_enu(p[0], p[1], olat, olon)[1] for p in poly]
    return (max(xs) - min(xs), max(ys) - min(ys))


def vfov_from_hfov(hfov_deg, img_w, img_h):
    return math.degrees(2.0 * math.atan(
        math.tan(math.radians(hfov_deg) / 2.0) * img_h / img_w))


def vfov_from_diagonal(diagonal_deg, img_w, img_h):
    """Derive VFOV from diagonal FOV and aspect ratio."""
    diag_rad = math.radians(diagonal_deg)
    # diagonal pixels
    diag_px = math.hypot(img_w, img_h)
    # focal in px from diagonal
    f = (diag_px / 2.0) / math.tan(diag_rad / 2.0)
    vfov = 2.0 * math.atan((img_h / 2.0) / f)
    return math.degrees(vfov)


def hfov_from_diagonal(diagonal_deg, img_w, img_h):
    """Derive HFOV from diagonal FOV and aspect ratio."""
    diag_rad = math.radians(diagonal_deg)
    diag_px = math.hypot(img_w, img_h)
    f = (diag_px / 2.0) / math.tan(diag_rad / 2.0)
    hfov = 2.0 * math.atan((img_w / 2.0) / f)
    return math.degrees(hfov)


def footprint_m(alt_m, hfov_deg, img_w, img_h, vfov_deg=None):
    """Ground footprint (width, height) meters for a nadir camera.
    
    If vfov_deg is given (from cam.txt), use it directly; otherwise derive
    from hfov + aspect ratio.
    """
    if vfov_deg is None:
        vfov_deg = vfov_from_hfov(hfov_deg, img_w, img_h)
    w = 2.0 * alt_m * math.tan(math.radians(hfov_deg) / 2.0)
    h = 2.0 * alt_m * math.tan(math.radians(vfov_deg) / 2.0)
    return (w, h)


def footprint_at_max_alt(max_alt_m, camera="bottom"):
    """Footprint at max allowed altitude for given camera.
    
    camera: 'front' = Pi Cam3 Standard (66° HFOV), 'bottom' = IMX477 B (100° HFOV)
    Returns (width_m, height_m)
    """
    if camera == "front":
        spec = CAM3_STANDARD
        # Use actual res for footprint calc, but FOV dominates
        img_w, img_h = spec["resolution"]
        return footprint_m(max_alt_m, spec["hfov_deg"], img_w, img_h, spec["vfov_deg"])
    else:
        spec = IMX477_B
        img_w, img_h = spec["resolution"]
        return footprint_m(max_alt_m, spec["hfov_deg"], img_w, img_h, spec["vfov_deg"])


def optimal_spacing_m(alt_m, hfov_deg, img_w, img_h, overlap=0.3, vfov_deg=None):
    """Optimal lawnmower spacing ensuring max coverage with no gaps.
    
    Uses the smaller footprint dimension (height, since rows run E-W) 
    to guarantee overlap in both axes.
    """
    fw, fh = footprint_m(alt_m, hfov_deg, img_w, img_h, vfov_deg)
    # Use the smaller dimension for spacing to ensure full coverage
    # Overlap 0.3 means 70% of footprint is new area per row
    min_dim = min(fw, fh)
    spacing = max(1.0, min_dim * (1.0 - overlap))
    return spacing, (fw, fh)


def clamp_altitude(alt_m, max_alt_m, min_alt_m=1.0):
    """Clamp altitude to [min_alt_m, max_alt_m] from config.yaml."""
    return max(float(min_alt_m), min(float(max_alt_m), float(alt_m)))


def lawnmower_rows(poly, spacing_m, origin=None, edge_margin_m=2.0):
    """Serpentine coverage waypoints over a polygon (the geofence).

    Rows run east-west spaced `spacing_m` apart, clipped to the polygon,
    then inset `edge_margin_m` along-track so legs never target the fence
    line itself (overshoot at speed = fence-breach RTL, Rsn 10). Runs
    shorter than 2x margin are skipped. Falls back to the polygon
    centroid if nothing survives (tiny fence).
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
        x_lo, x_hi = inside[0][0] + edge_margin_m, inside[-1][0] - edge_margin_m
        if x_hi < x_lo:
            continue  # run shorter than 2x margin: never target the line
        pts = [(x_lo, y)] if x_hi == x_lo else [(x_lo, y), (x_hi, y)]
        if r % 2 == 1:
            pts = pts[::-1]
        wps.extend(pts)
    if not wps:
        return [polygon_centroid(poly)]
    return [enu_to_latlon(x, y, olat, olon) for x, y in wps]


def lawnmower_rows_fov_optimal(poly, alt_m, hfov_deg, img_w, img_h,
                               origin=None, edge_margin_m=2.0,
                               overlap=0.3, vfov_deg=None,
                               max_alt_m=None):
    """FOV-optimal lawnmower: max coverage using camera footprint at alt.
    
    This is the main function for arena/test1 — divides polygon area
    according to cam FOV so drone covers as much as possible.
    
    - alt_m is clamped to max_alt_m if given (from config.yaml)
    - spacing derived from footprint * (1 - overlap)
    - ensures no gaps, maximizes covered area
    """
    if max_alt_m is not None:
        alt_m = clamp_altitude(alt_m, max_alt_m)
    spacing, (fw, fh) = optimal_spacing_m(alt_m, hfov_deg, img_w, img_h,
                                          overlap=overlap, vfov_deg=vfov_deg)
    wps = lawnmower_rows(poly, spacing, origin=origin, edge_margin_m=edge_margin_m)
    return {
        "waypoints": wps,
        "spacing_m": spacing,
        "footprint_w_m": fw,
        "footprint_h_m": fh,
        "alt_m": alt_m,
        "overlap": overlap,
        "coverage_m2": fw * fh,
        "total_area_m2": polygon_area_m2(poly, origin),
    }


def divide_polygon_by_fov(poly, alt_m, camera_specs, overlap=0.3,
                          origin=None, max_alt_m=None):
    """Divide polygon into coverage cells based on camera FOV.
    
    For each camera (front 66° + bottom 100°), compute footprint at alt_m,
    then return grid that maximizes coverage. This is used to ensure
    drone covers as much as possible area according to cam FOV.
    
    camera_specs: dict like {"front": {"hfov_deg": 66, "size": [4608,2592]}, ...}
    Returns dict with per-camera plans + combined optimal.
    """
    if max_alt_m is not None:
        alt_m = clamp_altitude(alt_m, max_alt_m)
    if origin is None:
        origin = polygon_centroid(poly) if poly else (0, 0)
    
    results = {}
    best_area = -1
    best_cam = None
    # For max area coverage we prefer largest footprint (bottom cam 100° HFOV)
    # but we keep per-camera plans so UI can switch. The optimal is the one
    # with largest footprint_area (max coverage per image) — ensures drone
    # covers as much as possible area according to cam FOV at max_alt.
    for cam_name, spec in (camera_specs or {}).items():
        hfov = float(spec.get("hfov_deg", 66.0))
        size = spec.get("size", [1920, 1080])
        img_w, img_h = int(size[0]), int(size[1])
        vfov = spec.get("vfov_deg")
        if vfov is not None:
            vfov = float(vfov)
        spacing, (fw, fh) = optimal_spacing_m(alt_m, hfov, img_w, img_h,
                                              overlap=overlap, vfov_deg=vfov)
        wps = lawnmower_rows(poly, spacing, origin=origin, edge_margin_m=2.0)
        area = polygon_area_m2(poly, origin)
        footprint_area = fw * fh
        needed = area / (footprint_area * (1.0 - overlap)) if footprint_area > 0 else 0

        results[cam_name] = {
            "hfov_deg": hfov,
            "vfov_deg": vfov if vfov is not None else vfov_from_hfov(hfov, img_w, img_h),
            "footprint_w_m": fw,
            "footprint_h_m": fh,
            "footprint_area_m2": footprint_area,
            "spacing_m": spacing,
            "waypoints": wps,
            "waypoint_count": len(wps),
            "total_area_m2": area,
            "estimated_footprints_needed": needed,
            "alt_m": alt_m,
        }
        # Max coverage = largest footprint area (bottom 100° HFOV wins over front 66°)
        # If tie, prefer bottom camera explicitly
        is_better = footprint_area > best_area
        if cam_name == "bottom":
            is_better = is_better or footprint_area >= best_area * 0.99
        if best_cam is None or is_better:
            best_area = footprint_area
            best_cam = cam_name

    if best_cam and best_cam in results:
        results["optimal"] = results[best_cam]
        results["optimal"]["chosen_camera"] = best_cam
    elif results:
        first = next(iter(results))
        results["optimal"] = results[first]
        results["optimal"]["chosen_camera"] = first
    
    return results


def polygon_to_fence_inclusion(polygon_latlon, max_alt_m=None):
    """Convert a drawn polygon to fence inclusion vertices.
    
    In arena/test1, UI draws polygon (shows on Mission Planner as polygon),
    then button converts it to fence inclusion (MAV_CMD 5001) for FC.
    
    Returns list of (lat, lon) tuples, validated, clamped.
    """
    if not polygon_latlon or len(polygon_latlon) < 3:
        return []
    # Validate and normalize to (lat, lon) tuples
    verts = []
    for p in polygon_latlon:
        if isinstance(p, dict):
            lat = p.get("lat")
            lon = p.get("lon")
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            lat, lon = p[0], p[1]
        else:
            continue
        try:
            lat = float(lat)
            lon = float(lon)
            if -90 <= lat <= 90 and -180 <= lon <= 180:
                verts.append((lat, lon))
        except Exception:
            continue
    # Ensure at least 3
    if len(verts) < 3:
        return []
    # Clamp to max 255 (MAVLink limit)
    if len(verts) > 255:
        verts = verts[:255]
    return verts


def validate_max_alt(max_alt_m, default=15.0, min_alt=1.0, max_allowed=50.0):
    """Validate max altitude from config.yaml (5, 10, 15 m etc)."""
    try:
        v = float(max_alt_m)
        if v < min_alt:
            return float(min_alt)
        if v > max_allowed:
            return float(max_allowed)
        return v
    except Exception:
        return float(default)


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
