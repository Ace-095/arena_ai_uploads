"""Single inclusion polygon validation/readback helpers (no FC side effects)."""
import math


def normalize(vertices):
    if not isinstance(vertices, (list, tuple)):
        raise ValueError("vertices must be a list")
    out = []
    for v in vertices:
        if isinstance(v, dict):
            lat = v.get("lat", v.get("latitude", v.get("y")))
            lon = v.get("lon", v.get("longitude", v.get("x")))
        elif isinstance(v, (list, tuple)) and len(v) == 2:
            lat, lon = v
        else:
            raise ValueError("every vertex must contain lat and lon")
        try:
            lat, lon = float(lat), float(lon)
        except (TypeError, ValueError):
            raise ValueError("coordinates must be numeric") from None
        if not (math.isfinite(lat) and math.isfinite(lon) and
                -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("coordinates must be finite WGS84 lat/lon")
        out.append((round(lat, 7), round(lon, 7)))
    if len(out) > 1 and out[0] == out[-1]:
        out.pop()  # MAVLink polygons are NOT closed by repeating the first point
    if len(out) < 3 or len(set(out)) != len(out):
        raise ValueError("fence needs >=3 distinct vertices (no duplicates)")
    # Local coordinates avoid cancellation around large WGS84 coordinates.
    p = [(a-out[0][0], b-out[0][1]) for a, b in out]
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    if abs(sum(p[i][0]*p[(i+1)%len(p)][1] - p[(i+1)%len(p)][0]*p[i][1]
               for i in range(len(p)))) < 1e-14:
        raise ValueError("fence has zero area")
    def on(a, b, c):
        return (min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and
                min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))
    for i, a in enumerate(p):
        b = p[(i+1) % len(p)]
        for j in range(i+1, len(p)):
            if j == i+1 or (i == 0 and j == len(p)-1):
                continue
            c, d = p[j], p[(j+1) % len(p)]
            x, y, z, w = cross(a,b,c), cross(a,b,d), cross(c,d,a), cross(c,d,b)
            if ((x*y < 0 and z*w < 0) or
                (x == 0 and on(a,b,c)) or (y == 0 and on(a,b,d)) or
                (z == 0 and on(c,d,a)) or (w == 0 and on(c,d,b))):
                raise ValueError("fence edges intersect")
    return out


def _f32_ulp(x):
    """One ULP of the nearest float32 to |x| - pure stdlib, no numpy here."""
    try:
        import struct
        a = abs(float(x))
        if a == 0.0 or math.isnan(a) or math.isinf(a):
            return 0.0
        bits = struct.unpack("<I", struct.pack("<f", a))[0]
        nxt = struct.unpack("<f", struct.pack("<I", bits + 1))[0]
        return abs(nxt - a)
    except Exception:
        return 0.0


def quant_tolerance(deg, floor=1.1e-7):
    """The largest error the VEHICLE ITSELF can introduce for one coordinate.

    ArduPilot's AC_Fence keeps polygon vertices as `Vector2f` holding
    (degrees * 1e7), i.e. a float32 - so an int32 degree value cannot be stored
    exactly and comes back up to one float32 ULP away. MEASURED against a real
    ArduCopter SITL (see tests/test_sitl_flight.py): uploading 15.3647000 /
    75.1240800 returned 15.3647008 / 75.1240768 - exactly the float32 rounding
    of 153647000 and 751240800, i.e. +8 and -32 in 1e-7-degree units.

    Why this matters: the previous fixed tolerance of 1.1e-7 deg (~1.2 cm) is
    ~30x smaller than that quantization, so `matches()` returned False for a
    fence the FC had stored PERFECTLY. On a real Pixhawk every upload would
    therefore have answered 502 "FC did not store the fence we sent", while the
    fake vehicle used by the bench tests stores exact integers and so never
    showed it. The tolerance is derived from the wire/storage format rather than
    guessed, and it stays tight in absolute terms (<= ~0.7 m at these
    latitudes) so a genuinely different polygon still fails.
    """
    return max(floor, _f32_ulp(max(abs(float(deg)), 1e-9) * 1e7) / 1e7)


def matches(expected, actual, tolerance=None):
    """Accept quantization, cyclic shifts and reversed winding; not a different fence.

    `tolerance=None` (the default) derives a per-coordinate tolerance from
    `quant_tolerance` - see the note there for why the old fixed 1.1e-7 was
    wrong against real hardware. Pass a number to force a fixed tolerance.
    """
    if not expected or not actual:
        return not expected and not actual
    a, b = normalize(expected), normalize(actual)
    if len(a) != len(b):
        return False

    def close(x, y):
        tol = tolerance if tolerance is not None else quant_tolerance(max(abs(x), abs(y)))
        return abs(x - y) <= tol

    for seq in (b, list(reversed(b))):
        for offset in range(len(seq)):
            if all(close(x, seq[(i+offset)%len(seq)][0]) and
                   close(y, seq[(i+offset)%len(seq)][1])
                   for i, (x,y) in enumerate(a)):
                return True
    return False
