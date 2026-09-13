// geo.js — flat-earth ENU math (identical to bridge/mp_bridge.py + ui-spec §4b)
export const EARTH_RADIUS_M = 6371000.0;

export function latlonToEnu(lat, lon, olat, olon) {
  const dlat = (lat - olat) * Math.PI / 180.0;
  const dlon = (lon - olon) * Math.PI / 180.0;
  return {
    x: dlon * EARTH_RADIUS_M * Math.cos(olat * Math.PI / 180.0), // east
    y: dlat * EARTH_RADIUS_M,                                     // north
  };
}

export function enuToLatlon(x, y, olat, olon) {
  return {
    lat: olat + (y / EARTH_RADIUS_M) * 180.0 / Math.PI,
    lon: olon + (x / (EARTH_RADIUS_M * Math.cos(olat * Math.PI / 180.0))) * 180.0 / Math.PI,
  };
}

// area-weighted (shoelace) polygon area, metres²
export function polyAreaM2(pts) {
  let a = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], q = pts[(i + 1) % pts.length];
    a += p.x * q.y - q.x * p.y;
  }
  return Math.abs(a) / 2.0;
}

export function centroid(pts) {
  const n = pts.length;
  let a = 0, cx = 0, cy = 0;
  for (let i = 0; i < n; i++) {
    const p = pts[i], q = pts[(i + 1) % n];
    const cross = p.x * q.y - q.x * p.y;
    a += cross; cx += (p.x + q.x) * cross; cy += (p.y + q.y) * cross;
  }
  a *= 0.5;
  if (Math.abs(a) < 1e-9) {
    return { x: pts.reduce((s, p) => s + p.x, 0) / n, y: pts.reduce((s, p) => s + p.y, 0) / n };
  }
  return { x: cx / (6 * a), y: cy / (6 * a) };
}

export function maxRadiusFrom(pts, c) {
  return Math.max(...pts.map(p => Math.hypot(p.x - c.x, p.y - c.y)));
}
