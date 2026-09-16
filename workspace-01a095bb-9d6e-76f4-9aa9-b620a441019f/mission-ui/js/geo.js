// geo.js — flat-earth ENU math + FOV coverage (arena/test1)
// Same convention as bridge/mp_bridge.py + mission_pi/geo.py
// Camera specs from cam.txt: Pi Cam3 66° HFOV, IMX477 100° HFOV (113° diag)

const EARTH_RADIUS_M = 6371000.0;

// Camera specs from cam.txt
const CAM_SPECS = {
  front: {
    model: "Pi Camera Module 3 Standard",
    sensor: "IMX708",
    hfov_deg: 66.0,
    vfov_deg: 41.0,
    diagonal_fov_deg: 75.0,
    focal_mm: 4.74,
    resolution: [4608, 2592],
  },
  bottom: {
    model: "Waveshare IMX477 IR-CUT B",
    sensor: "IMX477",
    hfov_deg: 100.0,
    vfov_deg: 75.0,
    diagonal_fov_deg: 113.0,
    focal_mm: 2.7,
    resolution: [4056, 3040],
  }
};

function latlonToEnu(lat, lon, olat, olon) {
  const dlat = (lat - olat) * Math.PI / 180.0;
  const dlon = (lon - olon) * Math.PI / 180.0;
  return {
    x: dlon * EARTH_RADIUS_M * Math.cos(olat * Math.PI / 180.0), // east
    y: dlat * EARTH_RADIUS_M,                                     // north
  };
}

function enuToLatlon(x, y, olat, olon) {
  return {
    lat: olat + (y / EARTH_RADIUS_M) * 180.0 / Math.PI,
    lon: olon + (x / (EARTH_RADIUS_M * Math.cos(olat * Math.PI / 180.0))) * 180.0 / Math.PI,
  };
}

// area-weighted (shoelace) polygon area, metres²
function polyAreaM2(pts) {
  let a = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], q = pts[(i + 1) % pts.length];
    a += p.x * q.y - q.x * p.y;
  }
  return Math.abs(a) / 2.0;
}

function centroid(pts) {
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

function maxRadiusFrom(pts, c) {
  return Math.max(...pts.map(p => Math.hypot(p.x - c.x, p.y - c.y)));
}

// --- FOV-based coverage (arena/test1) ---
function vfovFromHfov(hfov_deg, img_w, img_h) {
  return 2 * Math.atan(Math.tan(hfov_deg * Math.PI / 180 / 2) * img_h / img_w) * 180 / Math.PI;
}

function footprintM(alt_m, hfov_deg, img_w, img_h, vfov_deg) {
  if (vfov_deg == null) vfov_deg = vfovFromHfov(hfov_deg, img_w, img_h);
  const w = 2 * alt_m * Math.tan(hfov_deg * Math.PI / 180 / 2);
  const h = 2 * alt_m * Math.tan(vfov_deg * Math.PI / 180 / 2);
  return { w, h, area: w * h };
}

function optimalSpacingM(alt_m, hfov_deg, img_w, img_h, overlap, vfov_deg) {
  const fp = footprintM(alt_m, hfov_deg, img_w, img_h, vfov_deg);
  const minDim = Math.min(fp.w, fp.h);
  const spacing = Math.max(1.0, minDim * (1 - (overlap || 0.3)));
  return { spacing, footprint: fp };
}

function clampAlt(alt_m, max_alt_m, min_alt_m) {
  min_alt_m = min_alt_m || 1.0;
  max_alt_m = max_alt_m || 15.0;
  return Math.max(min_alt_m, Math.min(max_alt_m, alt_m));
}

function dividePolygonByFov(polygonLatLon, alt_m, cameraSpecs, overlap, max_alt_m) {
  // polygonLatLon: [{lat, lon}, ...]
  // Returns per-camera coverage plans ensuring max area coverage
  if (!polygonLatLon || polygonLatLon.length < 3) return {};
  const alt = clampAlt(alt_m, max_alt_m || 15.0);
  const olat = polygonLatLon.reduce((s, p) => s + p.lat, 0) / polygonLatLon.length;
  const olon = polygonLatLon.reduce((s, p) => s + p.lon, 0) / polygonLatLon.length;
  const enuPoly = polygonLatLon.map(p => {
    const e = latlonToEnu(p.lat, p.lon, olat, olon);
    return [e.x, e.y];
  });
  // bbox
  const xs = enuPoly.map(p => p[0]), ys = enuPoly.map(p => p[1]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs);
  const y0 = Math.min(...ys), y1 = Math.max(...ys);

  const results = {};
  const specs = cameraSpecs || CAM_SPECS;

  for (const camName in specs) {
    const spec = specs[camName];
    const hfov = spec.hfov_deg;
    const res = spec.resolution || [1920, 1080];
    const vfov = spec.vfov_deg;
    const fp = footprintM(alt, hfov, res[0], res[1], vfov);
    const spacing = Math.max(1.0, Math.min(fp.w, fp.h) * (1 - (overlap || 0.3)));

    // lawnmower waypoints inside polygon
    const rows = Math.max(1, Math.ceil((y1 - y0) / spacing) + 1);
    const wps = [];
    for (let r = 0; r < rows; r++) {
      const y = y0 + Math.min(r * spacing, y1 - y0);
      // scan row for inside points
      const n = Math.max(2, Math.ceil((x1 - x0) / (spacing / 4)) + 1);
      const inside = [];
      for (let i = 0; i < n; i++) {
        const x = x0 + (x1 - x0) * i / (n - 1);
        // point in polygon (ENU)
        let c = false, j = enuPoly.length - 1;
        for (let k = 0; k < enuPoly.length; k++) {
          const yi = enuPoly[k][1], xi = enuPoly[k][0];
          const yj = enuPoly[j][1], xj = enuPoly[j][0];
          if ((yi > y) !== (yj > y) && (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)) c = !c;
          j = k;
        }
        if (c) inside.push([x, y]);
      }
      if (!inside.length) continue;
      const margin = 2.0;
      let x_lo = inside[0][0] + margin, x_hi = inside[inside.length - 1][0] - margin;
      if (x_hi < x_lo) continue;
      let pts = (x_lo === x_hi) ? [[x_lo, y]] : [[x_lo, y], [x_hi, y]];
      if (r % 2 === 1) pts = pts.reverse();
      pts.forEach(p => {
        const ll = enuToLatlon(p[0], p[1], olat, olon);
        wps.push(ll);
      });
    }

    // area calc
    let area = 0;
    for (let i = 0; i < enuPoly.length; i++) {
      const p = enuPoly[i], q = enuPoly[(i + 1) % enuPoly.length];
      area += p[0] * q[1] - q[0] * p[1];
    }
    area = Math.abs(area) / 2;

    results[camName] = {
      hfov_deg: hfov,
      vfov_deg: vfov || vfovFromHfov(hfov, res[0], res[1]),
      footprint_w_m: fp.w,
      footprint_h_m: fp.h,
      footprint_area_m2: fp.area,
      spacing_m: spacing,
      waypoints: wps,
      waypoint_count: wps.length,
      total_area_m2: area,
      alt_m: alt,
      max_alt_m: max_alt_m || 15.0,
    };
  }

  // optimal = largest footprint area (max coverage per cam.txt: bottom 100° HFOV)
  // This ensures drone covers as much as possible area according to cam FOV
  // Bottom cam at 15m: ~35m wide vs front 66° ~19m -> bottom wins for max area
  let best = null, bestArea = -1;
  for (const k in results) {
    const area = results[k].footprint_area_m2;
    let isBetter = area > bestArea;
    if (k === 'bottom') isBetter = isBetter || area >= bestArea * 0.99;
    if (best === null || isBetter) {
      bestArea = area;
      best = k;
    }
  }
  if (best) {
    results.optimal = results[best];
    results.optimal.chosen_camera = best;
  }

  return results;
}

function polygonToFenceInclusion(polygon) {
  // Convert [{lat, lon}] polygon to fence inclusion vertices (same format, validated)
  if (!polygon || polygon.length < 3) return [];
  const out = [];
  for (const p of polygon) {
    const lat = parseFloat(p.lat), lon = parseFloat(p.lon);
    if (!isNaN(lat) && !isNaN(lon) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180) {
      out.push({ lat, lon });
    }
  }
  return out.length >= 3 ? out.slice(0, 255) : [];
}

// ES module exports (for future) + global for current app.js/map.js
if (typeof window !== 'undefined') {
  window.GeoUtils = {
    EARTH_RADIUS_M,
    CAM_SPECS,
    latlonToEnu,
    enuToLatlon,
    polyAreaM2,
    centroid,
    maxRadiusFrom,
    vfovFromHfov,
    footprintM,
    optimalSpacingM,
    clampAlt,
    dividePolygonByFov,
    polygonToFenceInclusion,
  };
}

// For ES module usage
try {
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = {
      EARTH_RADIUS_M,
      CAM_SPECS,
      latlonToEnu,
      enuToLatlon,
      polyAreaM2,
      centroid,
      maxRadiusFrom,
      vfovFromHfov,
      footprintM,
      optimalSpacingM,
      clampAlt,
      dividePolygonByFov,
      polygonToFenceInclusion,
    };
  }
} catch (e) {}

export {
  EARTH_RADIUS_M,
  CAM_SPECS,
  latlonToEnu,
  enuToLatlon,
  polyAreaM2,
  centroid,
  maxRadiusFrom,
  vfovFromHfov,
  footprintM,
  optimalSpacingM,
  clampAlt,
  dividePolygonByFov,
  polygonToFenceInclusion,
};
