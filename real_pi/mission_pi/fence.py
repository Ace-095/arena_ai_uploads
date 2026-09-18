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


def matches(expected, actual, tolerance=1.1e-7):
    """Accept quantization, cyclic shifts and reversed winding; not a different fence."""
    if not expected or not actual:
        return not expected and not actual
    a, b = normalize(expected), normalize(actual)
    if len(a) != len(b):
        return False
    for seq in (b, list(reversed(b))):
        for offset in range(len(seq)):
            if all(abs(x-seq[(i+offset)%len(seq)][0]) <= tolerance and
                   abs(y-seq[(i+offset)%len(seq)][1]) <= tolerance
                   for i, (x,y) in enumerate(a)):
                return True
    return False
