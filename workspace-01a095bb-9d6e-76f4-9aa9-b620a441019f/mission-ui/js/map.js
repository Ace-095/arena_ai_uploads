// map.js — Leaflet map: OSM/offline tiles, meter grid, fence drawing, drone + target markers
import { latlonToEnu, enuToLatlon } from './geo.js';

const $ = (id) => document.getElementById(id);

// 1x1 transparent tile — dark background shows through when OSM is unreachable (offline mode)
const EMPTY_TILE = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==';

const TILE_SOURCES = [
  { id: 'osm',       name: 'OSM standard',       url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', subs: 'abc',  attr: '&copy; OpenStreetMap contributors' },
  { id: 'cartoDark', name: 'Carto dark (UI match)', url: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', subs: 'abcd', attr: '&copy; CARTO &copy; OpenStreetMap' },
  { id: 'voyager',   name: 'Carto voyager (light)', url: 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png', subs: 'abcd', attr: '&copy; CARTO &copy; OpenStreetMap' },
  { id: 'esriSat',   name: 'Esri satellite',     url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', subs: '', attr: '&copy; Esri' },
  { id: 'esriTopo',  name: 'Esri topo',          url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}', subs: '', attr: '&copy; Esri' },
];

export function initMap(onLog = () => {}) {
  const map = L.map('map', { zoomControl: true }).setView([15.3697, 75.1235], 18);

  let tileLayer = null;
  let tileIdx = TILE_SOURCES.findIndex((s) => s.id === (localStorage.getItem('mc_tile') || 'osm'));
  if (tileIdx < 0) tileIdx = 0;

  // OSM serves its "Access blocked" policy page as a 200 PNG — detect it by
  // sampling the yellow/black warning stripes down the left edge of a tile.
  function looksLikeOsMBlock(img) {
    try {
      const c = document.createElement('canvas');
      c.width = 8; c.height = 256;
      const ctx = c.getContext('2d', { willReadFrequently: true });
      ctx.drawImage(img, 0, 0, 8, 256, 0, 0, 8, 256);
      const d = ctx.getImageData(0, 0, 8, 256).data;
      let yellow = 0, black = 0;
      for (let y = 0; y < 256; y++) {
        const r = d[(y * 8) * 4], g = d[(y * 8) * 4 + 1], b = d[(y * 8) * 4 + 2];
        if (r > 190 && g > 140 && b < 110) yellow++;
        else if (r < 70 && g < 70 && b < 70) black++;
      }
      // require BOTH yellow and black: the block page has alternating stripes;
      // a genuinely dark map edge (e.g. Carto dark over open sea) has no yellow
      return yellow > 40 && black > 40;
    } catch { return false; } // cross-origin tainted canvas — detection unavailable
  }

  function buildTileLayer(i) {
    const s = TILE_SOURCES[i];
    return L.tileLayer(s.url, {
      maxZoom: 20, minZoom: 13, subdomains: s.subs || 'abc',
      attribution: s.attr, errorTileUrl: EMPTY_TILE,
    });
  }

  let errStreak = 0, blockStreak = 0;
  function attachTileWatchers() {
    errStreak = 0; blockStreak = 0;
    tileLayer.on('tileerror', () => {
      errStreak += 1;
      if (errStreak >= 5) { errStreak = 0; autoSwitchTiles('repeated tile errors'); }
    });
    tileLayer.on('tileload', (e) => {
      errStreak = 0;
      if (looksLikeOsMBlock(e.tile)) {
        blockStreak += 1;
        if (blockStreak >= 3) { blockStreak = 0; autoSwitchTiles('OSM policy-block page detected'); }
      } else {
        blockStreak = 0;
      }
    });
  }

  function autoSwitchTiles(reason) {
    onLog(`tile source failed (${reason}) — auto-switching to "${TILE_SOURCES[(tileIdx + 1) % TILE_SOURCES.length].name}"`);
    setTileSource((tileIdx + 1) % TILE_SOURCES.length, true);
  }

  function setTileSource(i, silent = false) {
    tileIdx = ((i % TILE_SOURCES.length) + TILE_SOURCES.length) % TILE_SOURCES.length;
    if (tileLayer) map.removeLayer(tileLayer);
    tileLayer = buildTileLayer(tileIdx).addTo(map);
    attachTileWatchers();
    localStorage.setItem('mc_tile', TILE_SOURCES[tileIdx].id);
    const sel = $('selTile');
    if (sel && sel.value !== TILE_SOURCES[tileIdx].id) sel.value = TILE_SOURCES[tileIdx].id;
    if (!silent) onLog(`tile source: ${TILE_SOURCES[tileIdx].name}`);
  }

  setTileSource(tileIdx, true);

  const gridGroup = L.layerGroup().addTo(map);
  const gridOrigin = { lat: null, lon: null };   // EKF origin (from fence) or map center fallback
  let gridVisible = true;
  let drawing = false;
  const vMarkers = [];
  let vPoly = null;
  let droneMarker = null;
  let targetMarker = null;
  let confirmedFence = null;

  // ---------------- meter grid ----------------
  function metersPerPixel() {
    const c = map.getCenter();
    return (156543.03392 * Math.cos(c.lat * Math.PI / 180.0)) / Math.pow(2, map.getZoom());
  }

  function renderGrid() {
    gridGroup.clearLayers();
    const hint = $('mapHint');
    if (!gridVisible) { hint.style.display = 'none'; return; }
    const spacing = parseInt($('selGrid').value, 10);
    const mpp = metersPerPixel();
    if (spacing < 2 * mpp) {
      hint.textContent = `zoom in for the ${spacing} m grid (now ~${mpp.toFixed(2)} m/px)`;
      hint.style.display = 'block';
      return;
    }
    hint.style.display = 'none';
    const origin = (gridOrigin.lat != null)
      ? gridOrigin
      : { lat: map.getCenter().lat, lon: map.getCenter().lng };
    const range = 60; // ±60 m around origin
    const majorEvery = 10; // every 10th line = 10 m when spacing=1
    for (let x = -range; x <= range; x += spacing) {
      const p1 = enuToLatlon(x, -range, origin.lat, origin.lon);
      const p2 = enuToLatlon(x, range, origin.lat, origin.lon);
      const major = (Math.round(x / spacing) % majorEvery === 0);
      gridGroup.addLayer(L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], major
        ? { color: '#ffb020', weight: 1.2, opacity: 0.5 }
        : { color: '#52c7e0', weight: 0.6, opacity: 0.22 }));
    }
    for (let y = -range; y <= range; y += spacing) {
      const p1 = enuToLatlon(-range, y, origin.lat, origin.lon);
      const p2 = enuToLatlon(range, y, origin.lat, origin.lon);
      const major = (Math.round(y / spacing) % majorEvery === 0);
      gridGroup.addLayer(L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], major
        ? { color: '#ffb020', weight: 1.2, opacity: 0.5 }
        : { color: '#52c7e0', weight: 0.6, opacity: 0.22 }));
    }
    gridGroup.addLayer(L.circleMarker([origin.lat, origin.lon], {
      radius: 5, color: gridOrigin.lat != null ? '#ffb020' : '#ff5c5c',
      fillColor: gridOrigin.lat != null ? '#ffb020' : '#ff5c5c', fillOpacity: 0.9,
    }).bindTooltip(gridOrigin.lat != null ? 'EKF fence origin (0,0)' : 'Grid origin (map center fallback)'));
  }

  function snapToGrid(ll) {
    const spacing = parseInt($('selGrid').value, 10);
    const origin = (gridOrigin.lat != null) ? gridOrigin : { lat: map.getCenter().lat, lon: map.getCenter().lng };
    const e = latlonToEnu(ll.lat, ll.lng, origin.lat, origin.lon);
    const xs = Math.round(e.x / spacing) * spacing;
    const ys = Math.round(e.y / spacing) * spacing;
    return enuToLatlon(xs, ys, origin.lat, origin.lon);
  }

  // ---------------- fence drawing ----------------
  const dotIcon = L.divIcon({
    className: '',
    html: '<div style="width:10px;height:10px;border-radius:50%;background:#ffb020;border:2px solid #0a0e13;box-shadow:0 0 6px rgba(255,176,32,0.8)"></div>',
    iconSize: [10, 10], iconAnchor: [5, 5],
  });

  function updatePoly() {
    if (vPoly) { map.removeLayer(vPoly); vPoly = null; }
    const pts = vMarkers.map(m => m.getLatLng());
    if (pts.length >= 2) {
      vPoly = L.polygon(pts, { color: '#52c7e0', weight: 2, fillColor: '#52c7e0', fillOpacity: 0.08 }).addTo(map);
    }
    $('drawCount').textContent = `${pts.length} pts`;
  }

  map.on('click', (e) => {
    if (!drawing) return;
    const ll = $('chkSnap').checked ? snapToGrid(e.latlng) : { lat: e.latlng.lat, lng: e.latlng.lng };
    const m = L.marker([ll.lat, ll.lng], { icon: dotIcon, draggable: true }).addTo(map);
    m.on('drag', updatePoly);
    vMarkers.push(m);
    updatePoly();
  });

  // ---------------- markers ----------------
  function ensureDrone() {
    if (droneMarker) return;
    droneMarker = L.marker([0, 0], {
      icon: L.divIcon({
        className: '',
        html: '<div class="drone-marker"><div class="arrow"></div></div>',
        iconSize: [26, 26], iconAnchor: [13, 13],
      }),
      interactive: false,
    }).addTo(map);
  }

  // ---------------- confirmed fence render ----------------
  function renderConfirmedFence(st) {
    if (confirmedFence) { map.removeLayer(confirmedFence); confirmedFence = null; }
    targetMarker && map.removeLayer(targetMarker);
    targetMarker = null;
    if (!st || !st.loaded || !st.vertices_latlon || st.vertices_latlon.length < 3) return;
    confirmedFence = L.polygon(
      st.vertices_latlon.map(v => [v.lat, v.lon]),
      { color: st.confirmed ? '#34d17c' : '#ff5c5c', weight: 2.5, fillColor: st.confirmed ? '#34d17c' : '#ff5c5c', fillOpacity: 0.06 }
    ).addTo(map).bindTooltip(st.confirmed ? 'Fence (readback OK)' : 'Fence (READBACK MISMATCH)');
    if (st.origin_lat != null) {
      gridOrigin.lat = st.origin_lat; gridOrigin.lon = st.origin_lon;
      renderGrid();
    }
  }

  map.on('zoomend moveend', renderGrid);

  return {
    setDrawing(d) { drawing = d; map.getContainer().style.cursor = d ? 'crosshair' : ''; },
    getVertices() { return vMarkers.map(m => ({ lat: m.getLatLng().lat, lon: m.getLatLng().lng })); },
    undoVertex() { const m = vMarkers.pop(); if (m) map.removeLayer(m); updatePoly(); },
    clearVertices() { vMarkers.forEach(m => map.removeLayer(m)); vMarkers.length = 0; updatePoly(); },
    setDrone(t) {
      if (!t || t.lat == null) return;
      ensureDrone();
      droneMarker.setLatLng([t.lat, t.lon]);
      const el = droneMarker.getElement();
      if (el && t.hdg != null) {
        const arrow = el.querySelector('.arrow');
        if (arrow) arrow.style.transform = `rotate(${t.hdg}deg)`;
      }
    },
    setTarget(on) {
      if (!on) { if (targetMarker) { map.removeLayer(targetMarker); targetMarker = null; } return; }
      if (targetMarker) return;
      const c = map.getCenter();
      targetMarker = L.marker([c.lat, c.lng], {
        icon: L.divIcon({ className: '', html: '<div class="target-marker"></div>', iconSize: [18, 18], iconAnchor: [9, 9] }),
        interactive: false,
      }).addTo(map);
    },
    setFence(st) { renderConfirmedFence(st); },
    setOrigin(lat, lon) { gridOrigin.lat = lat; gridOrigin.lon = lon; renderGrid(); },
    setGridVisible(v) { gridVisible = v; renderGrid(); },
    getTileLayer() { return tileLayer; },
    setTileSourceByName(id) {
      const i = TILE_SOURCES.findIndex((s) => s.id === id);
      if (i >= 0) setTileSource(i);
    },
    getTileSource() { return TILE_SOURCES[tileIdx]; },
    map,
  };
}
