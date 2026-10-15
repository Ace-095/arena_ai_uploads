/* map.js — UI v2 + arena/test1 polygon + FOV coverage
 *
 * v2 fixes + arena/test1 enhancements:
 *  - polygon drawing (primary, shows on Mission Planner) + button to convert polygon -> fence inclusion
 *  - configurable max height from config.yaml (flight.max_alt_m)
 *  - FOV-based coverage planning (cam.txt: Pi Cam3 66° HFOV, IMX477 100° HFOV)
 *  - coverage grid visualization that divides area according to cam footprint
 *  - curated sources: offline MBTiles + OSM + Carto-dark fallback
 *  - FIXED grid anchor: frozen at boot (or fence origin), not live centre
 *  - canvas renderer + viewport culling + auto-coarsen
 */
(function () {
'use strict';

const EARTH_R = 6371000;
const MPP_EQUATOR = 156543.03392;
const MAX_GRID_LINES = 280;
const SPACINGS = [1, 2, 5, 10, 20, 50, 100];
const fmtInt = (n) => Math.round(n).toLocaleString('en-US').replace(/,/g, ' ');

// --- Camera specs from cam.txt (arena/test1) ---
const CAM_SPECS = {
  front: {
    model: "Pi Camera Module 3 Standard",
    hfov_deg: 66.0,
    vfov_deg: 41.0,
    diagonal_fov_deg: 75.0,
    resolution: [4608, 2592],
  },
  bottom: {
    model: "Waveshare IMX477 IR-CUT B",
    hfov_deg: 100.0,
    vfov_deg: 75.0,
    diagonal_fov_deg: 113.0,
    resolution: [4056, 3040],
  }
};

function latlonToEnu(lat, lon, olat, olon) {
  const dlat = (lat - olat) * Math.PI / 180;
  const dlon = (lon - olon) * Math.PI / 180;
  return [dlon * EARTH_R * Math.cos(olat * Math.PI / 180), dlat * EARTH_R];
}

function enuToLatlon(x, y, olat, olon) {
  const dlat = y / EARTH_R;
  const dlon = x / (EARTH_R * Math.cos(olat * Math.PI / 180));
  return [olat + dlat * 180 / Math.PI, olon + dlon * 180 / Math.PI];
}

function polyAreaM2(verts, o) {
  if (verts.length < 3) return 0;
  const enu = verts.map((v) => latlonToEnu(v[0], v[1], o.lat, o.lon));
  let a = 0;
  for (let i = 0; i < enu.length; i++) {
    const p = enu[i], q = enu[(i + 1) % enu.length];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return Math.abs(a) / 2;
}

function vfovFromHfov(hfov_deg, img_w, img_h) {
  return 2 * Math.atan(Math.tan(hfov_deg * Math.PI / 180 / 2) * img_h / img_w) * 180 / Math.PI;
}

function footprintM(alt_m, hfov_deg, img_w, img_h, vfov_deg) {
  if (vfov_deg == null) vfov_deg = vfovFromHfov(hfov_deg, img_w, img_h);
  const w = 2 * alt_m * Math.tan(hfov_deg * Math.PI / 180 / 2);
  const h = 2 * alt_m * Math.tan(vfov_deg * Math.PI / 180 / 2);
  return [w, h];
}

function optimalSpacing(alt_m, hfov_deg, img_w, img_h, overlap, vfov_deg) {
  const fp = footprintM(alt_m, hfov_deg, img_w, img_h, vfov_deg);
  const minDim = Math.min(fp[0], fp[1]);
  return { spacing: Math.max(1.0, minDim * (1 - overlap)), footprint: fp };
}

function initMap(o) {
  o = o || {};
  const onLog = o.onLog || function () {};
  const tilesAvailable = o.tilesAvailable !== false;
  const tilesZmax = o.tilesZmax || 18;

  const start = { lat: 15.3647, lon: 75.1240 };
  const map = L.map('map', { zoomControl: true }).setView([start.lat, start.lon], 18);
  const canvas = L.canvas({ padding: 0.5 });

  // ---- configurable max height (from config.yaml) ----
  let maxAltM = 15.0;
  let sweepAltM = 15.0;
  let overlap = 0.3;
  let currentCamera = "bottom"; // bottom = IMX477 100° is primary search cam

  function setMaxAlt(v) {
    const val = parseFloat(v);
    if (!isNaN(val) && val >= 1 && val <= 50) {
      maxAltM = val;
      if (sweepAltM > maxAltM) sweepAltM = maxAltM;
      onLog('INFO', 'max altitude set to ' + maxAltM + ' m (from config.yaml)');
      updateFovCoverage();
    }
  }
  function setSweepAlt(v) {
    const val = parseFloat(v);
    if (!isNaN(val) && val >= 1) {
      sweepAltM = Math.min(val, maxAltM);
      onLog('INFO', 'sweep alt set to ' + sweepAltM + ' m (clamped to max ' + maxAltM + ' m)');
      updateFovCoverage();
    }
  }
  function getMaxAlt() { return maxAltM; }
  function getSweepAlt() { return sweepAltM; }

  // ---- fixed grid anchor ----
  const sessionAnchor = { lat: start.lat, lon: start.lon };
  let fenceAnchor = null;
  const anchor = () => fenceAnchor || sessionAnchor;

  function reanchorGrid() {
    const c = map.getCenter();
    sessionAnchor.lat = c.lat; sessionAnchor.lon = c.lng;
    fenceAnchor = null;
    drawGrid();
    onLog('INFO', 'grid re-anchored at ' + c.lat.toFixed(5) + ', ' + c.lng.toFixed(5));
  }

  function setOrigin(lat, lon) {
    fenceAnchor = { lat, lon };
    drawGrid();
  }

  // ---- tile sources ----
  const origin = window.location.origin;
  const SOURCES = {
    mbtiles: { label: 'offline-mbtiles', url: origin + '/tiles/{z}/{x}/{y}.png', maxZoom: tilesZmax,
               attribution: 'offline MBTiles (field pack)' },
    osm: { label: 'osm', url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png', maxZoom: 19,
           attribution: '© OpenStreetMap contributors' },
    cartoDark: { label: 'carto-dark', url: 'https://basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png', maxZoom: 20,
                 attribution: '© OpenStreetMap © CARTO' },
  };
  const ORDER = ['mbtiles', 'osm', 'cartoDark'];
  const selTile = document.getElementById('selTile');
  if (selTile) {
    ORDER.forEach((k) => {
      const opt = document.createElement('option');
      opt.value = k; opt.textContent = SOURCES[k].label;
      selTile.appendChild(opt);
    });
  }

  const mapHint = document.getElementById('mapHint');
  let layer = null, layerName = '';
  let errStreak = 0, blockStreak = 0, lastSwitchAt = 0;

  function setHint(t) { if (mapHint) mapHint.textContent = t || ''; }

  function applyTileLayer(name, why) {
    if (!SOURCES[name]) return;
    if (name === 'mbtiles' && !tilesAvailable) {
      onLog('WARN', 'offline tiles unavailable on bridge — ' + (why || 'skipping mbtiles'));
      name = 'osm';
    }
    if (layer) { map.removeLayer(layer); layer = null; }
    const s = SOURCES[name];
    layerName = name;
    if (selTile) selTile.value = name;
    layer = L.tileLayer(s.url, { maxZoom: s.maxZoom, attribution: s.attribution });
    layer.on('tileerror', () => {
      errStreak += 1;
      if (errStreak >= 8 && Date.now() - lastSwitchAt > 10000) autoSwitch('tile errors x' + errStreak);
    });
    layer.on('tileload', () => { errStreak = 0; blockStreak = 0; });
    layer.on('tileloadstart', (e) => {
      const img = e.tile;
      if (img && img.complete && img.naturalWidth > 0 && img.naturalWidth < 32) {
        blockStreak += 1;
        if (blockStreak >= 4 && Date.now() - lastSwitchAt > 10000) autoSwitch('blocked-tile placeholders');
      }
    });
    layer.addTo(map);
    errStreak = 0; blockStreak = 0;
    onLog('INFO', 'tile source → ' + s.label + (why ? ' (' + why + ')' : ''));
  }

  function autoSwitch(why) {
    const i = ORDER.indexOf(layerName);
    for (let k = 1; k <= ORDER.length; k++) {
      const next = ORDER[(i + k) % ORDER.length];
      if (next === 'mbtiles' && !tilesAvailable) continue;
      if (next === layerName) continue;
      lastSwitchAt = Date.now();
      onLog('WARN', 'auto-switching tiles: ' + layerName + ' → ' + next + ' (' + why + ')');
      applyTileLayer(next, 'auto: ' + why);
      return;
    }
  }

  function setTileSourceByName(name) {
    lastSwitchAt = Date.now();
    applyTileLayer(name, 'manual');
  }

  // ---- meter grid ----
  const gridSelect = document.getElementById('gridSelect');
  let gridGroup = L.layerGroup().addTo(map);

  function metersPerPixel(lat) {
    return MPP_EQUATOR * Math.cos(lat * Math.PI / 180) / Math.pow(2, map.getZoom());
  }

  function zoomForSpacing(spacing, lat) {
    const z = Math.ceil(Math.log2(MPP_EQUATOR * Math.cos(lat * Math.PI / 180) * 2 / spacing));
    return Math.max(1, Math.min(21, z));
  }

  function zoomToGrid() {
    const spacing = parseFloat(gridSelect ? gridSelect.value : 10) || 10;
    const c = map.getCenter();
    const z = zoomForSpacing(spacing, c.lat);
    map.setView([c.lat, c.lng], Math.max(z, map.getZoom()));
    onLog('INFO', 'zoomed to z' + Math.max(z, map.getZoom()) + ' for ' + spacing + ' m grid');
  }

  function drawGrid() {
    gridGroup.clearLayers();
    const requested = parseFloat(gridSelect ? gridSelect.value : 10) || 10;
    const c = map.getCenter();
    const mpp = metersPerPixel(c.lat);
    if (requested < 2 * mpp) {
      setHint('need z' + zoomForSpacing(requested, c.lat) + '+ for ' + requested + ' m grid (click to zoom)');
      return;
    }
    const o = anchor();
    const b = map.getBounds();
    const corners = [b.getSouthWest(), b.getNorthWest(), b.getNorthEast(), b.getSouthEast()]
      .map((p) => latlonToEnu(p.lat, p.lng, o.lat, o.lon));
    const xs = corners.map((p) => p[0]), ys = corners.map((p) => p[1]);
    let minX = Math.min.apply(null, xs), maxX = Math.max.apply(null, xs);
    let minY = Math.min.apply(null, ys), maxY = Math.max.apply(null, ys);

    let spacing = requested;
    for (let i = SPACINGS.indexOf(requested); i < SPACINGS.length; i++) {
      spacing = SPACINGS[i];
      const nx = Math.ceil((maxX - minX) / spacing) + 1;
      const ny = Math.ceil((maxY - minY) / spacing) + 1;
      if (nx + ny <= MAX_GRID_LINES) break;
      if (i === SPACINGS.length - 1) {
        setHint('viewport too wide even for 100 m grid — zoom in');
        return;
      }
    }
    const pad = spacing;
    minX -= pad; maxX += pad; minY -= pad; maxY += pad;
    const kMinX = Math.floor(minX / spacing), kMaxX = Math.ceil(maxX / spacing);
    const kMinY = Math.floor(minY / spacing), kMaxY = Math.ceil(maxY / spacing);
    const lines = [];
    for (let kx = kMinX; kx <= kMaxX; kx++) {
      const x = kx * spacing;
      const a = enuToLatlon(x, minY, o.lat, o.lon), d = enuToLatlon(x, maxY, o.lat, o.lon);
      lines.push(L.polyline([[a[0], a[1]], [d[0], d[1]]], {
        color: kx % 10 === 0 ? '#ffb020' : '#3a4a63', weight: kx % 10 === 0 ? 1.4 : 0.7,
        opacity: kx % 10 === 0 ? 0.85 : 0.55, interactive: false, renderer: canvas,
      }));
    }
    for (let ky = kMinY; ky <= kMaxY; ky++) {
      const y = ky * spacing;
      const a = enuToLatlon(minX, y, o.lat, o.lon), d = enuToLatlon(maxX, y, o.lat, o.lon);
      lines.push(L.polyline([[a[0], a[1]], [d[0], d[1]]], {
        color: ky % 10 === 0 ? '#ffb020' : '#3a4a63', weight: ky % 10 === 0 ? 1.4 : 0.7,
        opacity: ky % 10 === 0 ? 0.85 : 0.55, interactive: false, renderer: canvas,
      }));
    }
    lines.forEach((l) => gridGroup.addLayer(l));
    setHint(spacing !== requested
      ? ('grid auto → ' + spacing + ' m (viewport; requested ' + requested + ' m)')
      : ('grid ' + spacing + ' m · anchor ' + o.lat.toFixed(4) + ', ' + o.lon.toFixed(4) + ' · maxAlt ' + maxAltM + 'm'));
  }

  // ---- drone / target markers ----
  const droneIcon = L.divIcon({
    className: 'drone-marker',
    html: '<svg width="26" height=\"26\" viewBox=\"0 0 26 26\"><g><path d=\"M13 2 L18 16 L13 13 L8 16 Z\" fill=\"#ff5050\" stroke=\"#fff\" stroke-width=\"1.2\"/></g><circle cx=\"13\" cy=\"13\" r=\"2\" fill=\"#fff\"/></svg>',
    iconSize: [26, 26], iconAnchor: [13, 13],
  });
  let droneMarker = null, dronePos = null, droneSeen = false, following = false;
  function setDrone(lat, lon, hdg) {
    if (lat == null || lon == null) return;
    dronePos = [lat, lon];
    if (!droneMarker) {
      droneMarker = L.marker(dronePos, { icon: droneIcon, zIndexOffset: 1000 }).addTo(map);
      droneMarker.bindTooltip('vehicle', { direction: 'top', offset: [0, -14] });
    } else droneMarker.setLatLng(dronePos);
    if (hdg != null) {
      const el = droneMarker.getElement && droneMarker.getElement();
      const g = el && el.querySelector('g');
      if (g) g.setAttribute('transform', 'rotate(' + hdg + ' 13 13)');
    }
    if (!droneSeen) {
      droneSeen = true;
      map.setView(dronePos, Math.max(map.getZoom(), 17));
    } else if (following && !map.getBounds().pad(-0.15).contains(dronePos)) {
      map.panTo(dronePos);
    }
  }
  function setFollow(on) {
    following = !!on;
    if (following && dronePos) map.panTo(dronePos);
    return following;
  }

  let targetMarker = null;
  function setTarget(lat, lon, on) {
    if (on === false || lat == null) {
      if (targetMarker) { map.removeLayer(targetMarker); targetMarker = null; }
      return;
    }
    const p = (lat != null && lon != null) ? [lat, lon]
      : (function () { const c = map.getCenter(); return [c.lat, c.lng]; })();
    if (!targetMarker) {
      targetMarker = L.circleMarker(p, { radius: 9, color: '#ffb020', weight: 2, fillColor: '#ffb020',
        fillOpacity: 0.25, renderer: canvas }).addTo(map);
      targetMarker.bindTooltip('target', { direction: 'top', offset: [0, -10] });
    } else targetMarker.setLatLng(p);
  }

  // ---- coverage heatmap (Pi) ----
  const covGroup = L.layerGroup().addTo(map);
  const covCells = new Set();
  let covOrigin = null, covCellM = 5;
  function updateCovLegend(n) {
    const el = document.getElementById('covLegend');
    if (!el) return;
    el.style.cssText = 'margin-left:8px;font-size:11px;color:#9fb3c8;white-space:nowrap;';
    el.innerHTML = '<span style=\"display:inline-block;width:10px;height:10px;background:#2f8f6f;opacity:.8;margin-right:3px;\"></span>searched '
      + n + ' cells <span style=\"display:inline-block;width:10px;height:10px;background:#ffd23f;opacity:.9;margin:0 3px 0 8px;\"></span>scanning';
  }
  function setCoverage(msg) {
    if (!msg || !msg.origin) return;
    const o = { lat: msg.origin.lat, lon: msg.origin.lon };
    const cm = msg.cell_m || 5;
    if (!covOrigin || o.lat !== covOrigin.lat || o.lon !== covOrigin.lon || cm !== covCellM) {
      covOrigin = o; covCellM = cm;
      covCells.clear(); covGroup.clearLayers();
    }
    const hot = {};
    (msg.hot || []).forEach((c) => { hot[c[0] + ',' + c[1]] = 1; });
    (msg.cells || []).forEach((c) => {
      const k = c[0] + ',' + c[1];
      if (covCells.has(k) || covCells.size > 4000) return;
      covCells.add(k);
      const sw = enuToLatlon(c[0] * covCellM, c[1] * covCellM, o.lat, o.lon);
      const ne = enuToLatlon((c[0] + 1) * covCellM, (c[1] + 1) * covCellM, o.lat, o.lon);
      covGroup.addLayer(L.rectangle([[sw[0], sw[1]], [ne[0], ne[1]]], {
        stroke: false,
        fillColor: hot[k] ? '#ffd23f' : '#2f8f6f',
        fillOpacity: hot[k] ? 0.55 : 0.35,
        interactive: false, renderer: canvas,
      }));
    });
    updateCovLegend(msg.total != null ? msg.total : covCells.size);
  }
  function clearCoverage() {
    covCells.clear(); covGroup.clearLayers(); updateCovLegend(0);
  }

  // ---- FOV-based coverage planning (arena/test1) ----
  // Shows how drone will cover polygon according to cam FOV at max height
  const fovGroup = L.layerGroup().addTo(map);
  let fovPlan = null;

  function updateFovCoverage() {
    fovGroup.clearLayers();
    // Use polygon if available, else fence
    const verts = (polyVerts.length >= 3 ? polyVerts : drawVerts);
    if (verts.length < 3) return;

    const spec = CAM_SPECS[currentCamera] || CAM_SPECS.bottom;
    const o = anchor();
    const alt = Math.min(sweepAltM, maxAltM);

    // Calculate footprint at this alt
    const fp = footprintM(alt, spec.hfov_deg, spec.resolution[0], spec.resolution[1], spec.vfov_deg);
    const sp = optimalSpacing(alt, spec.hfov_deg, spec.resolution[0], spec.resolution[1], overlap, spec.vfov_deg);

    // Build lawnmower rows clipped to polygon
    const enuPoly = verts.map(v => latlonToEnu(v[0], v[1], o.lat, o.lon));
    const xs = enuPoly.map(p => p[0]), ys = enuPoly.map(p => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs);
    const y0 = Math.min(...ys), y1 = Math.max(...ys);

    const rows = Math.max(1, Math.ceil((y1 - y0) / sp.spacing) + 1);
    const waypoints = [];

    for (let r = 0; r < rows; r++) {
      const y = y0 + Math.min(r * sp.spacing, y1 - y0);
      // scan row
      const n = Math.max(2, Math.ceil((x1 - x0) / (sp.spacing / 4)) + 1);
      const inside = [];
      for (let i = 0; i < n; i++) {
        const x = x0 + (x1 - x0) * i / (n - 1);
        // point in polygon check (ENU)
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
      // inset by edge margin (2m)
      const margin = 2.0;
      let x_lo = inside[0][0] + margin, x_hi = inside[inside.length - 1][0] - margin;
      if (x_hi < x_lo) continue;
      let pts = (x_lo === x_hi) ? [[x_lo, y]] : [[x_lo, y], [x_hi, y]];
      if (r % 2 === 1) pts = pts.reverse();
      pts.forEach(p => {
        const ll = enuToLatlon(p[0], p[1], o.lat, o.lon);
        waypoints.push(ll);
      });
    }

    // Draw footprint rectangles at each waypoint (visualize coverage)
    waypoints.forEach((wp, idx) => {
      if (idx % 2 !== 0) return; // only draw every other to reduce clutter
      const e = latlonToEnu(wp[0], wp[1], o.lat, o.lon);
      const halfW = fp[0] / 2, halfH = fp[1] / 2;
      const sw = enuToLatlon(e[0] - halfW, e[1] - halfH, o.lat, o.lon);
      const ne = enuToLatlon(e[0] + halfW, e[1] + halfH, o.lat, o.lon);
      fovGroup.addLayer(L.rectangle([[sw[0], sw[1]], [ne[0], ne[1]]], {
        color: '#5fb3ff',
        weight: 0.8,
        opacity: 0.5,
        fillColor: '#5fb3ff',
        fillOpacity: 0.08,
        interactive: false,
        renderer: canvas,
      }));
    });

    // Draw lawnmower path
    if (waypoints.length >= 2) {
      fovGroup.addLayer(L.polyline(waypoints, {
        color: '#5fb3ff',
        weight: 2,
        opacity: 0.9,
        dashArray: '6,4',
        renderer: canvas,
      }));
    }

    // Update info display
    const area = polyAreaM2(verts, o);
    const fpArea = fp[0] * fp[1];
    const covEl = document.getElementById('fovCoverageInfo');
    if (covEl) {
      covEl.innerHTML = 'cam ' + currentCamera + ' (' + spec.hfov_deg + '° HFOV) @ ' + alt + 'm<br>' +
        'footprint ' + fp[0].toFixed(1) + '×' + fp[1].toFixed(1) + 'm (' + fpArea.toFixed(0) + 'm²)<br>' +
        'spacing ' + sp.spacing.toFixed(1) + 'm (overlap ' + (overlap * 100).toFixed(0) + '%)<br>' +
        'legs ' + waypoints.length + ' · area ' + fmtInt(area) + 'm² · est. ' + Math.ceil(area / (fpArea * (1 - overlap))) + ' footprints';
    }

    fovPlan = {
      waypoints,
      footprint: fp,
      spacing: sp.spacing,
      alt,
      camera: currentCamera,
      area_m2: area,
    };

    onLog('INFO', 'FOV coverage: ' + currentCamera + ' @ ' + alt + 'm footprint ' +
      fp[0].toFixed(1) + 'x' + fp[1].toFixed(1) + 'm spacing ' + sp.spacing.toFixed(1) + 'm ' +
      waypoints.length + ' wps area ' + area.toFixed(0) + 'm²');
  }

  function setFovCamera(cam) {
    if (CAM_SPECS[cam]) {
      currentCamera = cam;
      updateFovCoverage();
      onLog('INFO', 'FOV camera switched to ' + cam + ' (' + CAM_SPECS[cam].hfov_deg + '° HFOV)');
    }
  }

  function setOverlap(v) {
    const val = parseFloat(v);
    if (!isNaN(val) && val >= 0 && val < 0.9) {
      overlap = val;
      updateFovCoverage();
    }
  }

  function getFovPlan() { return fovPlan; }
  function clearFovPlan() { fovGroup.clearLayers(); fovPlan = null; }

  // ---- fence overlay (authoritative from FC) ----
  let fencePoly = null;
  function setFence(verts) {
    if (fencePoly) { map.removeLayer(fencePoly); fencePoly = null; }
    if (!verts || verts.length < 3) return;
    fencePoly = L.polygon(verts, { color: '#5fd97a', weight: 2, fillOpacity: 0.08, renderer: canvas }).addTo(map);
    updateFovCoverage();
  }

  // ---- POLYGON (arena/test1 primary) ----
  // Polygon is what user draws, shows on Mission Planner, then converts to fence inclusion
  let polyGroup = L.layerGroup().addTo(map);
  let polyVerts = []; // [[lat, lon], ...] primary polygon
  let polyDrawing = false;
  let polyFenceConverted = false;

  function updatePolygon() {
    polyGroup.clearLayers();
    const pDrawEl = document.getElementById('pDrawing');
    const pAreaEl = document.getElementById('pArea');
    const pVertsEl = document.getElementById('pVerts');

    if (polyVerts.length === 0) {
      if (pDrawEl) pDrawEl.textContent = '0 pts';
      if (pAreaEl) pAreaEl.textContent = '—';
      if (pVertsEl) pVertsEl.textContent = '—';
      updateFovCoverage();
      return;
    }

    // Draw vertices
    polyVerts.forEach((v) => polyGroup.addLayer(
      L.circleMarker(v, { radius: 5, color: '#a78bfa', fillColor: '#a78bfa', fillOpacity: 1, renderer: canvas })));

    // Draw polygon
    if (polyVerts.length >= 3) {
      polyGroup.addLayer(L.polygon(polyVerts, {
        color: '#a78bfa',
        weight: 2,
        fillColor: '#a78bfa',
        fillOpacity: polyFenceConverted ? 0.18 : 0.10,
        renderer: canvas
      }));
    } else if (polyVerts.length === 2) {
      polyGroup.addLayer(L.polyline(polyVerts, { color: '#a78bfa', weight: 2, renderer: canvas }));
    }

    if (pDrawEl) pDrawEl.textContent = polyVerts.length + ' pts' + (polyFenceConverted ? ' (→ fence ✓)' : '');
    if (pAreaEl) pAreaEl.textContent = polyVerts.length >= 3
      ? ('≈ ' + fmtInt(polyAreaM2(polyVerts, anchor())) + ' m²') : '—';
    if (pVertsEl) pVertsEl.textContent = polyVerts.length + ' verts';

    updateFovCoverage();
  }

  function setPolygonDrawing(on) {
    polyDrawing = !!on;
    if (!polyDrawing) {
      // keep verts when stopping, don't clear
    }
    return polyDrawing;
  }

  function getPolygonVertices() {
    return polyVerts.map((v) => ({ lat: v[0], lon: v[1] }));
  }

  function setPolygon(verts) {
    polyVerts = [];
    if (verts && verts.length) {
      verts.forEach((v) => {
        if (Array.isArray(v) && v.length >= 2) polyVerts.push([v[0], v[1]]);
        else if (v.lat != null && v.lon != null) polyVerts.push([v.lat, v.lon]);
      });
    }
    polyFenceConverted = false;
    updatePolygon();
  }

  function undoPolygonVertex() { polyVerts.pop(); updatePolygon(); }
  function clearPolygon() { polyVerts = []; polyFenceConverted = false; updatePolygon(); clearFovPlan(); }

  function polygonToFence() {
    // Convert polygon vertices to fence inclusion format
    if (polyVerts.length < 3) {
      onLog('ERROR', 'polygon needs ≥3 vertices to convert to fence (have ' + polyVerts.length + ')');
      return null;
    }
    const fenceVerts = polyVerts.map((v) => ({ lat: v[0], lon: v[1] }));
    polyFenceConverted = true;
    updatePolygon();
    onLog('INFO', 'polygon → fence inclusion: ' + fenceVerts.length + ' verts (shows on MP as fence)');
    return fenceVerts;
  }

  // ---- drawing (fence legacy) ----
  let drawing = false, drawVerts = [];
  const drawGroup = L.layerGroup().addTo(map);
  const drawAreaEl = document.getElementById('drawArea');
  const fDrawingEl = document.getElementById('fDrawing');

  function snapLatLon(lat, lon) {
    const snapEl = document.getElementById('snapInput');
    const snapM = (snapEl && parseFloat(snapEl.value)) || 1;
    const o = anchor();
    const e = latlonToEnu(lat, lon, o.lat, o.lon);
    const sx = Math.round(e[0] / snapM) * snapM, sy = Math.round(e[1] / snapM) * snapM;
    return enuToLatlon(sx, sy, o.lat, o.lon);
  }

  function updatePoly() {
    drawGroup.clearLayers();
    if (drawVerts.length === 0) {
      if (fDrawingEl) fDrawingEl.textContent = '0 pts';
      if (drawAreaEl) drawAreaEl.textContent = '—';
      return;
    }
    drawVerts.forEach((v) => drawGroup.addLayer(
      L.circleMarker(v, { radius: 4, color: '#ffb020', fillOpacity: 1, renderer: canvas })));
    if (drawVerts.length >= 3)
      drawGroup.addLayer(L.polygon(drawVerts, { color: '#ffb020', weight: 1.5,
        fillOpacity: 0.12, renderer: canvas }));
    else if (drawVerts.length === 2)
      drawGroup.addLayer(L.polyline(drawVerts, { color: '#ffb020', weight: 1.5, renderer: canvas }));
    if (fDrawingEl) fDrawingEl.textContent = drawVerts.length + ' pts';
    if (drawAreaEl) drawAreaEl.textContent = drawVerts.length >= 3
      ? ('≈ ' + fmtInt(polyAreaM2(drawVerts, anchor())) + ' m²') : '—';
  }

  function setDrawing(on) {
    drawing = !!on;
    if (!drawing) { drawVerts = []; updatePoly(); }
    return drawing;
  }

  function getVertices() { return drawVerts.map((v) => ({ lat: v[0], lon: v[1] })); }
  function undoVertex() { drawVerts.pop(); updatePoly(); }
  function clearVertices() { drawVerts = []; updatePoly(); }

  map.on('click', (e) => {
    const snapped = snapLatLon(e.latlng.lat, e.latlng.lng);
    if (polyDrawing) {
      polyVerts.push(snapped);
      updatePolygon();
      return;
    }
    if (!drawing) return;
    drawVerts.push(snapped);
    updatePoly();
  });

  // ---- boot ----
  applyTileLayer(tilesAvailable ? 'mbtiles' : 'osm', 'boot');
  drawGrid();

  // Load config.yaml if available (max height)
  fetch('/config.yaml')
    .then(r => r.ok ? r.text() : Promise.reject())
    .then(txt => {
      const m = txt.match(/max_alt_m:\s*([\d.]+)/);
      if (m) setMaxAlt(m[1]);
      const s = txt.match(/default_sweep_alt_m:\s*([\d.]+)/);
      if (s) setSweepAlt(s[1]);
      const o = txt.match(/overlap:\s*([\d.]+)/);
      if (o) setOverlap(o[1]);
    }).catch(() => {
      // try mission-ui/config.yaml path
      fetch('/api/config').then(r => r.ok ? r.json() : null).then(cfg => {
        if (cfg && cfg.flight && cfg.flight.max_alt_m) setMaxAlt(cfg.flight.max_alt_m);
      }).catch(() => {});
    });

  return {
    // legacy fence drawing
    setDrawing, getVertices, undoVertex, clearVertices,
    // polygon (arena/test1 primary)
    setPolygonDrawing, getPolygonVertices, setPolygon, undoPolygonVertex, clearPolygon, polygonToFence,
    // FOV coverage
    setMaxAlt, getMaxAlt, setSweepAlt, getSweepAlt, setFovCamera, setOverlap, getFovPlan, clearFovPlan, updateFovCoverage,
    // general
    setDrone, setTarget, setFence, setOrigin, setCoverage, clearCoverage, setFollow,
    setTileSourceByName, getTileSource: () => layerName,
    reanchorGrid, zoomToGrid, map,
    CAM_SPECS,
  };
}

function initTiles() {
  return Promise.resolve(null);
}

window.MissionMap = { initMap, initTiles };
})();
