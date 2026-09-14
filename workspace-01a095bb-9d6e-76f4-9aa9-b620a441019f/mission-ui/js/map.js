/* map.js — UI v2 Leaflet map.
 *
 * v2 fixes (vs v1 shallow + buggy grid):
 *  - curated sources: offline MBTiles (bridge /tiles, default when available)
 *    + OSM + Carto-dark fallback. Deep zoom (z18+) is served from the local
 *    MBTiles — no network needed in the field.
 *  - FIXED grid anchor: the meter grid is projected from one anchor frozen at
 *    boot (or the fence origin once applied), not from the live map centre.
 *    Panning no longer shifts the tape under a drawn polygon.
 *  - canvas renderer + viewport culling + auto-coarsen: the grid draws only
 *    the lines visible in the viewport (cap ~280 lines), stepping 1→2→5→…→100 m
 *    automatically when zoomed out. No more frozen tab at z10.
 *  - zoom-to-grid helper: click the map hint to jump to a zoom where the grid
 *    actually renders.
 */
(function () {
'use strict';

const EARTH_R = 6371000;
const MPP_EQUATOR = 156543.03392;
const MAX_GRID_LINES = 280;
const SPACINGS = [1, 2, 5, 10, 20, 50, 100];
const fmtInt = (n) => Math.round(n).toLocaleString('en-US').replace(/,/g, ' ');

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

function initMap(o) {
  o = o || {};
  const onLog = o.onLog || function () {};
  const tilesAvailable = o.tilesAvailable !== false;
  const tilesZmax = o.tilesZmax || 18;

  const start = { lat: 15.3647, lon: 75.1240 };
  const map = L.map('map', { zoomControl: true }).setView([start.lat, start.lon], 18);
  const canvas = L.canvas({ padding: 0.5 });

  // ---- fixed grid anchor: frozen at boot, fence origin wins once applied ----
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

  // ---- tile sources: offline-mbtiles default, OSM + Carto fallback ----
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
  ORDER.forEach((k) => {
    const opt = document.createElement('option');
    opt.value = k; opt.textContent = SOURCES[k].label;
    selTile.appendChild(opt);
  });

  const mapHint = document.getElementById('mapHint');
  let layer = null, layerName = '';
  let errStreak = 0, blockStreak = 0, lastSwitchAt = 0;

  function setHint(t) { mapHint.textContent = t || ''; }

  function applyTileLayer(name, why) {
    if (!SOURCES[name]) return;
    if (name === 'mbtiles' && !tilesAvailable) {
      onLog('WARN', 'offline tiles unavailable on bridge — ' + (why || 'skipping mbtiles'));
      name = 'osm';
    }
    if (layer) { map.removeLayer(layer); layer = null; }
    const s = SOURCES[name];
    layerName = name;
    selTile.value = name;
    layer = L.tileLayer(s.url, { maxZoom: s.maxZoom, attribution: s.attribution });
    layer.on('tileerror', () => {
      errStreak += 1;
      if (errStreak >= 8 && Date.now() - lastSwitchAt > 10000) autoSwitch('tile errors x' + errStreak);
    });
    layer.on('tileload', () => { errStreak = 0; blockStreak = 0; });
    layer.on('tileloadstart', (e) => {
      // blocked-tile placeholder sniff (hehe technique): tiny tiles = block page
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
    lastSwitchAt = Date.now(); // manual switch resets the auto cooldown
    applyTileLayer(name, 'manual');
  }

  // ---- meter grid: viewport-culled, canvas-rendered, auto-coarsened ----
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
    const spacing = parseFloat(gridSelect.value) || 10;
    const c = map.getCenter();
    const z = zoomForSpacing(spacing, c.lat);
    map.setView([c.lat, c.lng], Math.max(z, map.getZoom()));
    onLog('INFO', 'zoomed to z' + Math.max(z, map.getZoom()) + ' for ' + spacing + ' m grid');
  }

  function drawGrid() {
    gridGroup.clearLayers();
    const requested = parseFloat(gridSelect.value) || 10;
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
      : ('grid ' + spacing + ' m · anchor ' + o.lat.toFixed(4) + ', ' + o.lon.toFixed(4)));
  }

  // ---- drone / target markers (SVG divIcons) ----
  const droneIcon = L.divIcon({
    className: 'drone-marker',
    html: '<svg width="26" height="26" viewBox="0 0 26 26"><g><path d="M13 2 L18 16 L13 13 L8 16 Z" fill="#ff5050" stroke="#fff" stroke-width="1.2"/></g><circle cx="13" cy="13" r="2" fill="#fff"/></svg>',
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
      // First fix ever: jump to the vehicle (sim boots a continent away
      // from the session anchor — without this the marker is invisible).
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

  // ---- search coverage heatmap (Pi "coverage" channel; full set, self-healing) ----
  const covGroup = L.layerGroup().addTo(map);
  const covCells = new Set();   // "ix,iy" already drawn
  let covOrigin = null, covCellM = 5;
  function updateCovLegend(n) {
    const el = document.getElementById('covLegend');
    if (!el) return;
    el.style.cssText = 'margin-left:8px;font-size:11px;color:#9fb3c8;white-space:nowrap;';
    el.innerHTML = '<span style="display:inline-block;width:10px;height:10px;background:#2f8f6f;opacity:.8;margin-right:3px;"></span>searched '
      + n + ' cells <span style="display:inline-block;width:10px;height:10px;background:#ffd23f;opacity:.9;margin:0 3px 0 8px;"></span>scanning';
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

  // ---- fence overlay (authoritative copy from FC) ----
  let fencePoly = null;
  function setFence(verts) {
    if (fencePoly) { map.removeLayer(fencePoly); fencePoly = null; }
    if (!verts || verts.length < 3) return;
    fencePoly = L.polygon(verts, { color: '#5fd97a', weight: 2, fillOpacity: 0.08, renderer: canvas }).addTo(map);
  }

  // ---- drawing (click-to-add, snapped to the fixed-anchor grid) ----
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
      fDrawingEl.textContent = '0 pts';
      drawAreaEl.textContent = '—';
      return;
    }
    drawVerts.forEach((v) => drawGroup.addLayer(
      L.circleMarker(v, { radius: 4, color: '#ffb020', fillOpacity: 1, renderer: canvas })));
    if (drawVerts.length >= 3)
      drawGroup.addLayer(L.polygon(drawVerts, { color: '#ffb020', weight: 1.5,
        fillOpacity: 0.12, renderer: canvas }));
    else if (drawVerts.length === 2)
      drawGroup.addLayer(L.polyline(drawVerts, { color: '#ffb020', weight: 1.5, renderer: canvas }));
    fDrawingEl.textContent = drawVerts.length + ' pts';
    drawAreaEl.textContent = drawVerts.length >= 3
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
    if (!drawing) return;
    drawVerts.push(snapLatLon(e.latlng.lat, e.latlng.lng));
    updatePoly();
  });

  // ---- boot ----
  applyTileLayer(tilesAvailable ? 'mbtiles' : 'osm', 'boot');
  drawGrid();

  return {
    setDrawing, getVertices, undoVertex, clearVertices,
    setDrone, setTarget, setFence, setOrigin, setCoverage, clearCoverage, setFollow,
    setTileSourceByName, getTileSource: () => layerName,
    reanchorGrid, zoomToGrid, map,
  };
}

function initTiles() {
  // No-op hook: app.js boot fetches /api/mp/tiles-info and passes the result
  // into initMap opts, so the tiles pipeline keeps one documented entry point.
  return Promise.resolve(null);
}

window.MissionMap = { initMap, initTiles };
})();
