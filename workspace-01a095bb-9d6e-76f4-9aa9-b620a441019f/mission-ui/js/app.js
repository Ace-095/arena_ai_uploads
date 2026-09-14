/* app.js — UI v2 wiring.
 *
 * Data ownership (ui-spec v2):
 *  - flight truth (telemetry, mode, fence, plan, battery, MP console): the
 *    BRIDGE, sourced from MAVLink forwarded by Mission Planner (SSE).
 *  - mission brain (fsm, system, Pi QR receipts, target events): the PI
 *    (WebSocket, receive-only).
 *  - OUT to FC: fence upload/clear, RTL/LAND mode, param read — all through
 *    the bridge's MP link. OUT to Pi: camera tuning only (no MAVLink path).
 */
(function () {
'use strict';

const $ = (id) => document.getElementById(id);
const nowStr = () => new Date().toLocaleTimeString('en-GB');
const fmtInt = (n) => Math.round(n).toLocaleString('en-US').replace(/,/g, ' ');
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

const S = {
  piUrl: '', isMock: false, bridgeOk: false,
  pi: { connected: false },
  camsAvail: { cam1: true, cam2: true }, camsProbed: false,
  mav: null, fence: null, plan: null, fsm: null,
  qr: { payload: '', streak: 0, required: 3, confirmed: false },
  receipts: { pi: '', mavlink: '', mpfile: '', manual: '' },
  logs: [],
};

let map = null, pi = null, bridge = null, cams = null;

// ---------------------------------------------------------------- logs --
function log(level, msg) {
  S.logs.push({ ts: nowStr(), level, msg: String(msg) });
  if (S.logs.length > 500) S.logs.splice(0, S.logs.length - 500);
  renderLogs();
}

function renderLogs() {
  const f = $('logFilter').value;
  const box = $('logBox');
  box.innerHTML = S.logs
    .filter((l) => !f || l.level === f)
    .map((l) => '<div class="lr ' + l.level + '"><span class="lt">' + l.ts + '</span> ' +
      '<span class="ll">' + l.level + '</span> ' + esc(l.msg) + '</div>')
    .join('');
  box.scrollTop = box.scrollHeight;
}

function setLamp(id, on) {
  const el = $(id);
  el.classList.toggle('on', !!on);
  el.classList.toggle('off', !on);
}

// ------------------------------------------------------- bridge-owned --
function renderTelemetry(d) {
  $('tLatLon').textContent = d.lat.toFixed(7) + ' / ' + d.lon.toFixed(7);
  $('tAlt').textContent = d.alt_rel.toFixed(1) + ' / ' + d.alt_msl.toFixed(1) + ' m';
  $('tMode').textContent = (d.mode || '—') + (d.armed ? ' · ARMED' : '');
  const sp = Math.hypot(d.vx || 0, d.vy || 0);
  $('tVel').textContent = sp.toFixed(1) + ' m/s (vz ' + (d.vz != null ? d.vz.toFixed(1) : '—') + ')';
  $('tAtt').textContent = d.attitude
    ? d.attitude.roll.toFixed(1) + ' / ' + d.attitude.pitch.toFixed(1) + ' / ' + d.attitude.yaw.toFixed(1) : '—';
  $('tGps').textContent = d.gps ? ('fix ' + d.gps.fix + ' / ' + d.gps.sats + ' sats') : '—';
  $('tBatt').textContent = d.battery
    ? ((d.battery.v != null ? d.battery.v.toFixed(1) + ' V' : '—') +
       (d.battery.pct != null ? ' · ' + d.battery.pct + '%' : '') +
       (d.battery.current_a != null ? ' · ' + d.battery.current_a.toFixed(1) + ' A' : '')) : '—';
  $('tMis').textContent = d.mission ? ('seq ' + d.mission.seq + ' / ' + (d.mission.total || '?')) : '—';
  if (map) map.setDrone(d.lat, d.lon, d.hdg);
}

function renderMav(d) {
  S.mav = d;
  const b = $('mavBadge');
  b.textContent = d.connected ? ('CONNECTED · ' + (d.mode || '?')) : 'DISCONNECTED';
  b.classList.toggle('ok', !!d.connected);
  setLamp('lampMav', d.connected);
  $('mavVeh').textContent = d.vehicle_sysid != null
    ? ('sysid ' + d.vehicle_sysid + ' / type ' + d.vehicle_type) : '—';
  $('mavMode').textContent = (d.mode || '—') + (d.armed ? ' · ARMED' : '') +
    (d.hb_age_s != null ? ' · hb ' + d.hb_age_s + 's ago' : '');
  $('mavRx').textContent = d.rx_rate + ' msg/s · ' + fmtInt(d.rx_msgs) + ' total · :' + d.port;
  $('mavSender').textContent = (d.sender || 'no forwarder yet') + (d.tx_msgs != null ? ' · tx ' + fmtInt(d.tx_msgs) : '') + (d.sender_changes ? ' · mpswitch ' + d.sender_changes : '') + (d.link_flaps ? ' · flaps ' + d.link_flaps : '');
  renderPlan(S.plan || d.plan);
  renderArmable();
}

function renderFence(d) {
  S.fence = d;
  if (!d) {
    $('fState').textContent = '—'; $('fReadback').textContent = '—';
    $('fVerts').textContent = '—'; $('fArea').textContent = '—';
    $('fRadius').textContent = '—'; $('fOrigin').textContent = '—';
    return;
  }
  $('fState').textContent = d.loaded ? 'LOADED on FC' : 'not loaded';
  $('fState').style.color = d.loaded ? (d.confirmed ? '#5fd97a' : '#ffb020') : '';
  $('fReadback').textContent = d.loaded ? (d.confirmed ? 'MATCH ✓' : 'MISMATCH ✗') : '—';
  $('fVerts').textContent = d.vertex_count != null ? d.vertex_count : '—';
  $('fArea').textContent = d.area_m2 != null ? fmtInt(d.area_m2) + ' m²' : '—';
  $('fRadius').textContent = d.max_radius_m != null ? d.max_radius_m + ' m' : '—';
  $('fOrigin').textContent = (d.origin_lat != null)
    ? (d.origin_lat.toFixed(5) + ', ' + d.origin_lon.toFixed(5)) : '—';
  if (d.reason && d.reason !== S._lastFenceReason) {
    S._lastFenceReason = d.reason;
    log(d.confirmed ? 'INFO' : 'WARN', 'fence: ' + d.reason);
  }
  if (map) {
    const verts = (d.vertices_latlon || []).map((v) => [v.lat, v.lon]);
    map.setFence(verts);
    if (d.origin_lat != null) map.setOrigin(d.origin_lat, d.origin_lon);
  }
  renderArmable();
}

function renderPlan(p) {
  if (p) S.plan = p;
  p = S.plan;
  const txt = !p ? '—'
    : (p.synced ? (p.total + ' items' + (p.do_sprayer_seq != null ? ' · sprayer trig #' + p.do_sprayer_seq : ' · no sprayer cmd'))
      : 'empty');
  $('planInfo').textContent = txt;
  $('mavPlan').textContent = txt;
}

function renderArmable() {
  const ok = !!(S.mav && S.mav.connected && S.fence && S.fence.loaded && S.fence.confirmed);
  const el = $('armableInfo');
  el.textContent = ok ? 'YES — fence confirmed on FC' : 'NO — need MAVLink link + confirmed fence';
  el.style.color = ok ? '#5fd97a' : '#ff6b6b';
}

function renderBridgeState(d) {
  S.isMock = !!d.mock;
  $('mockBadge').classList.toggle('hidden', !d.mock);
  $('piMode').textContent = d.mock ? 'mock (bridge loopback)' : 'real';
  $('watchInfo').textContent = ([].concat(d.watching || []).join('; ')) || '—';
  if (d.qr && d.qr.payload && !S.qr.payload) {
    S.qr.payload = d.qr.payload;
    $('qrPayload').textContent = d.qr.payload;
    log('INFO', 'QR already latched on bridge: ' + d.qr.payload);
  }
  if (d.mavlink) {
    // lightweight summary until the first full mavlink-state arrives
    if (!S.mav) {
      $('mavBadge').textContent = d.mavlink.connected ? 'CONNECTED' : 'DISCONNECTED';
      setLamp('lampMav', d.mavlink.connected);
    }
  }
  if (d.tiles) {
    log('INFO', 'tiles: ' + (d.tiles.available
      ? (d.tiles.count + ' tiles z' + d.tiles.zmin + '–' + d.tiles.zmax) : 'OFFLINE PACK MISSING'));
  }
}

function renderMpConsole(m) {
  const box = $('mpConsole');
  const div = document.createElement('div');
  div.className = 'lr' + (m.severity <= 2 ? ' ERROR' : '');
  div.innerHTML = '<span class="lt">' + nowStr() + '</span> ' + esc(m.line);
  box.appendChild(div);
  while (box.children.length > 100) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
  if (m.severity <= 2) log('ERROR', '[FC] ' + m.line);
}

function onMpQr(m) {
  const t = nowStr();
  if (m.source === 'mavlink') { S.receipts.mavlink = t; $('qrSrcMav').textContent = t; }
  else if (m.source === 'manual') { S.receipts.manual = t; $('qrSrcMan').textContent = t; }
  else { S.receipts.mpfile = t; $('qrSrcMp').textContent = t + ' (' + m.source + ')'; }
  if (m.payload && !S.qr.payload) {
    S.qr.payload = m.payload;
    $('qrPayload').textContent = m.payload;
  }
  log('WARN', 'QR via ' + m.source + ': ' + m.payload);
}

// ----------------------------------------------------------- pi-owned --
function renderSystem(d) {
  if (d.temp_c != null) $('sTemp').textContent = d.temp_c.toFixed(1) + ' °C';
  else if (d.temp != null) $('sTemp').textContent = d.temp + ' °C';
  const cpu = d.cpu_pct != null ? d.cpu_pct : d.cpu;
  const mem = d.mem_pct != null ? d.mem_pct : d.mem;
  if (cpu != null || mem != null)
    $('sCpu').textContent = (cpu != null ? cpu.toFixed(0) + '%' : '?') + ' / ' + (mem != null ? mem.toFixed(0) + '%' : '?');
  if (d.disk_pct != null) $('sDisk').textContent = d.disk_pct.toFixed(0) + '%';
  else if (d.disk != null) $('sDisk').textContent = d.disk + '%';
  if (d.uptime_s != null) $('sUp').textContent = Math.floor(d.uptime_s / 60) + 'm ' + Math.floor(d.uptime_s % 60) + 's';
}

function renderFsm(d) {
  S.fsm = d;
  const state = d.state || d.to || '?';
  const reason = d.reason || (d.acknowledged && (d.acknowledged.reason || d.acknowledged.by)) || '—';
  $('fsmBadge').textContent = state;
  $('fsmState').textContent = state;
  $('fsmReason').textContent = reason;
}

function renderQrUpdate(d) {
  if (d.payload) {
    S.qr.payload = d.payload;
    $('qrPayload').textContent = d.payload;
  }
  if (d.streak != null) S.qr.streak = d.streak;
  else if (d.count != null) S.qr.streak = d.count;
  if (d.required != null) S.qr.required = d.required;
  S.qr.confirmed = d.confirmed != null ? !!d.confirmed : (S.qr.streak >= S.qr.required && S.qr.streak > 0);
  $('qrStreak').textContent = S.qr.streak + ' / ' + S.qr.required + (S.qr.confirmed ? ' ✓' : '');
  S.receipts.pi = nowStr();
  $('qrSrcPi').textContent = S.receipts.pi;
}

function renderEvent(d) {
  const t = d.type || d.event || '';
  if (/target/i.test(t)) {
    const lat = d.lat != null ? d.lat : d.latitude, lon = d.lon != null ? d.lon : d.longitude;
    if (map && lat != null) map.setTarget(lat, lon);
    log('WARN', 'target: ' + (d.confidence != null ? Math.round(d.confidence * 100) + '% ' : '') +
      (lat != null ? ('@ ' + lat.toFixed(5) + ', ' + lon.toFixed(5)) : '(no fix)'));
    return;
  }
  if (d.payload) {
    renderQrUpdate(d);
    log('WARN', 'QR via pi-ws: ' + d.payload);
    return;
  }
  if (/mission_result/i.test(t)) {
    log('INFO', 'mission result: ' + (d.routes ? d.routes.length + ' routes' : JSON.stringify(d).slice(0, 200)));
    return;
  }
  if (/abort/i.test(t)) { log('ERROR', 'ABORT from Pi: ' + (d.reason || d.from || '')); return; }
  log('INFO', '[event] ' + (t || JSON.stringify(d).slice(0, 160)));
}

function onEnvelope(env) {
  if (!env || !env.channel) return;
  const d = env.data || {};
  switch (env.channel) {
    case 'system': renderSystem(d); break;
    case 'event': renderEvent(d); break;
    case 'qr': renderQrUpdate(d); log('WARN', 'QR via pi-ws: ' + (d.payload || '?')); break;
    case 'fsm': renderFsm(d); if (map && (d.phase || d.state || d.to) === 'WAIT_LINK') map.clearCoverage(); break;
    case 'coverage': if (map) map.setCoverage(d); break;
    case 'log': log(d.level || 'INFO', d.msg || JSON.stringify(d).slice(0, 200)); break;
    default: break; // bridge-owned channels also arrive on WS in mock — ignore, SSE handles them
  }
}

// ------------------------------------------------- camera live controls --
const CAMS = [{ id: 'cam1', p: 'c1' }, { id: 'cam2', p: 'c2' }];
const camTimers = {};

function setSlider(p, name, v) {
  if (v == null) return;
  const s = $(p + name);
  if (s) { s.value = v; $(p + name + 'V').textContent = v; }
}

// One camera on the rig => one tile + one tab. The Pi reports its rig at
// GET /api/cameras; anything missing is hidden (and never polled). When the
// probe fails (old Pi, mock, Pi down) both tiles stay up and keep trying.
function probeCameras() {
  if (!S.piUrl) return Promise.resolve(null);
  return fetch(S.piUrl + '/api/cameras').then((r) => {
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then((st) => {
    const avail = { cam1: !!(st && st.cam1), cam2: !!(st && st.cam2) };
    if (!avail.cam1 && !avail.cam2) return null; // empty/ancient: keep trying both
    S.camsProbed = true;
    S.camsAvail = avail;
    applyCamVisibility();
    log('INFO', 'cameras on Pi: ' + (avail.cam1 ? 'cam1 ' : '') + (avail.cam2 ? 'cam2' : ''));
    return st;
  }).catch(() => null);
}

function applyCamVisibility() {
  const a = S.camsAvail;
  [['cam1', '1'], ['cam2', '2']].forEach(([id, n]) => {
    const show = a[id] !== false;
    $('camView' + n).classList.toggle('hidden', !show);
    $('tabCam' + n).classList.toggle('hidden', !show);
  });
  // keep a VISIBLE tab selected (single-cam rigs must not strand the panel)
  const v1 = a.cam1 !== false, v2 = a.cam2 !== false;
  const c1on = $('tabCam1').classList.contains('on');
  if (c1on && !v1 && v2) $('tabCam2').click();
  else if (!c1on && !v2 && v1) $('tabCam1').click();
}

function startAvailableCams() {
  CAMS.forEach((c) => {
    if (S.camsAvail[c.id] !== false && cams) cams.startCam(c.id);
  });
}

function seedCamControls() {
  if (!S.piUrl) return;
  CAMS.forEach((c) => {
    if (S.camsProbed && S.camsAvail[c.id] === false) return;
    postJson(S.piUrl + '/api/camera/status', { cam: c.id }).then((st) => {
      if (!st || !st.controls) return;
      const k = st.controls;
      setSlider(c.p, 'Exp', k.exposure_us);
      setSlider(c.p, 'Gain', k.gain_db != null ? k.gain_db : k.gain);
      setSlider(c.p, 'Bri', k.brightness);
      setSlider(c.p, 'Con', k.contrast);
      setSlider(c.p, 'Sat', k.saturation);
      setSlider(c.p, 'Sha', k.sharpness);
      if (k.af_mode) $(c.p + 'Af').value = k.af_mode;
      if (k.adaptive != null) $(c.p + 'Adapt').checked = !!k.adaptive;
    }).catch(() => {});
  });
}

function scheduleCamPush(camId, p) {
  $(p + 'Live').textContent = 'sending…';
  if (camTimers[camId]) clearTimeout(camTimers[camId]);
  camTimers[camId] = setTimeout(() => pushCamControls(camId, p), 250);
}

function pushCamControls(camId, p) {
  if (!S.piUrl) { $(p + 'Live').textContent = 'no Pi link'; return; }
  const body = {
    cam: camId,
    exposure_us: +$(p + 'Exp').value, gain_db: +$(p + 'Gain').value,
    af_mode: $(p + 'Af').value, adaptive: $(p + 'Adapt').checked,
    brightness: +$(p + 'Bri').value, contrast: +$(p + 'Con').value,
    saturation: +$(p + 'Sat').value, sharpness: +$(p + 'Sha').value,
  };
  postJson(S.piUrl + '/api/camera/controls', body).then(() => {
    $(p + 'Live').textContent = 'applied ✓ ' + nowStr();
  }).catch((e) => {
    $(p + 'Live').textContent = 'error: ' + e.message;
    log('ERROR', camId + ' controls push failed: ' + e.message);
  });
}

// ------------------------------------------------------- fetch helpers --
function postJson(url, body) {
  return fetch(url, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  }).then((r) => {
    if (!r.ok) return r.text().then((t) => { throw new Error('HTTP ' + r.status + ': ' + t.slice(0, 200)); });
    return r.json();
  });
}

// ------------------------------------------------------------------ boot --
function boot() {
  setInterval(() => { $('clock').textContent = nowStr(); }, 1000);
  $('clock').textContent = nowStr();
  $('logFilter').addEventListener('change', renderLogs);
  $('btnLogClear').addEventListener('click', () => { S.logs = []; renderLogs(); });
  $('btnLogDownload').addEventListener('click', () => {
    const blob = new Blob([S.logs.map((l) => l.ts + ' ' + l.level + ' ' + l.msg).join('\n')],
      { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = 'mission-ui.log'; a.click();
  });
  $('piUrl').value = window.location.origin;

  // ---- map (tiles-info first so the offline pack is default when present) --
  fetch('/api/mp/tiles-info').then((r) => r.json()).then((info) => {
    map = window.MissionMap.initMap({
      onLog: log,
      tilesAvailable: !!(info && info.available),
      tilesZmax: (info && info.zmax) || 18,
    });
    log('INFO', 'map ready (tiles: ' + (info && info.available ? 'offline pack' : 'online fallback') + ')');
  }).catch((e) => {
    map = window.MissionMap.initMap({ onLog: log, tilesAvailable: false });
    log('WARN', 'tiles-info failed, online tiles: ' + e.message);
  });

  // ---- fence tools ----
  $('btnDraw').addEventListener('click', () => {
    if (!map) return;
    const on = map.setDrawing($('btnDraw').classList.toggle('on'));
    if (!on) $('btnDraw').classList.remove('on');
  });
  $('btnUndo').addEventListener('click', () => { map && map.undoVertex(); });
  $('btnClearDraw').addEventListener('click', () => { map && map.clearVertices(); });
  $('btnReanchor').addEventListener('click', () => { map && map.reanchorGrid(); });
  $('btnFenceApply').addEventListener('click', () => {
    if (!map) return;
    const verts = map.getVertices();
    if (verts.length < 3) { log('ERROR', 'fence needs ≥3 drawn vertices (have ' + verts.length + ')'); return; }
    if (!window.confirm('Apply ' + verts.length + '-vertex fence via MP link to the FC?')) return;
    postJson('/api/mp/fence', { vertices: verts, mission_type: 'fence', fence_action: null })
      .then((st) => { renderFence(st); log('WARN', 'fence upload done: ' + st.vertex_count + ' verts, ' + (st.confirmed ? 'readback MATCH' : 'READBACK MISMATCH')); })
      .catch((e) => log('ERROR', 'fence upload failed: ' + e.message));
  });
  $('btnFenceClear').addEventListener('click', () => {
    if (!window.confirm('Clear the fence on the FC?')) return;
    fetch('/api/mp/fence', { method: 'DELETE' }).then((r) => {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    }).then((st) => { renderFence(st); log('WARN', 'fence cleared on FC'); })
      .catch((e) => log('ERROR', 'fence clear failed: ' + e.message));
  });
  $('btnFenceRefresh').addEventListener('click', () => {
    fetch('/api/mp/fence').then((r) => {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    }).then((st) => {
      renderFence(st);
      log(st.loaded ? 'WARN' : 'INFO', st.loaded
        ? ('fence refreshed from FC: ' + st.vertex_count + ' verts')
        : 'no fence stored on FC');
    }).catch((e) => log('ERROR', 'fence refresh failed: ' + e.message));
  });
  $('btnExportLap').addEventListener('click', () => {
    const verts = (S.fence && S.fence.vertices_latlon) || (map && map.getVertices()) || [];
    if (!verts.length) { log('ERROR', 'nothing to export (no fence, no drawing)'); return; }
    const txt = verts.map((v) => v.lat.toFixed(7) + ',' + v.lon.toFixed(7)).join('\n') + '\n';
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' }));
    a.download = 'fence.lap'; a.click();
    log('INFO', 'exported ' + verts.length + ' verts to fence.lap');
  });

  // ---- MP link tools ----
  $('btnMavPort').addEventListener('click', () => {
    const port = parseInt($('mavPort').value, 10);
    if (!port || port < 1 || port > 65535) { log('ERROR', 'bad UDP port'); return; }
    postJson('/api/mp/mavlink', { port }).then((d) => log('INFO', 'MAVLink listener → port ' + d.port))
      .catch((e) => log('ERROR', 'rebind failed: ' + e.message));
  });
  $('btnParam').addEventListener('click', () => {
    const name = $('paramName').value.trim().toUpperCase();
    if (!name) return;
    $('paramVal').textContent = '…';
    postJson('/api/mp/param', { name }).then((d) => {
      $('paramVal').textContent = d.name + ' = ' + d.value;
      log('INFO', 'param ' + d.name + ' = ' + d.value);
    }).catch((e) => { $('paramVal').textContent = 'error'; log('ERROR', 'param read failed: ' + e.message); });
  });

  // ---- actions (via MP link) ----
  $('btnAbort').addEventListener('click', () => {
    if (!window.confirm('ABORT → send RTL to the vehicle via MP?')) return;
    postJson('/api/mp/mode', { mode: 'RTL' })
      .then((d) => log('WARN', 'RTL commanded (' + d.status + (d.result_name ? ', ' + d.result_name + ' [' + d.result + ']' : '') + ')'))
      .catch((e) => log('ERROR', 'RTL failed: ' + e.message));
  });
  $('btnLand').addEventListener('click', () => {
    if (!window.confirm('Send LAND to the vehicle via MP?')) return;
    postJson('/api/mp/mode', { mode: 'LAND' })
      .then((d) => log('WARN', 'LAND commanded (' + d.status + (d.result_name ? ', ' + d.result_name + ' [' + d.result + ']' : '') + ')'))
      .catch((e) => log('ERROR', 'LAND failed: ' + e.message));
  });
  $('btnMockRestart').addEventListener('click', () => {
    postJson('/api/mp/mock/restart', {}).then(() => log('INFO', 'mock scenario restarted'))
      .catch((e) => log('ERROR', 'mock restart failed: ' + e.message));
  });

  // ---- QR manual + MP log watch ----
  $('btnQrManual').addEventListener('click', () => {
    const payload = $('qrManual').value.trim();
    if (!payload) return;
    postJson('/api/mp/qr', { payload }).then((d) => {
      const p = (d.qr && d.qr.payload) || d.payload || '';
      S.qr.payload = p;
      $('qrPayload').textContent = p;
      S.receipts.manual = nowStr();
      $('qrSrcMan').textContent = S.receipts.manual;
      log('WARN', 'manual QR latched: ' + p);
    }).catch((e) => log('ERROR', 'manual QR failed: ' + e.message));
  });
  $('btnWatch').addEventListener('click', () => {
    const path = $('watchPath').value.trim();
    if (!path) return;
    postJson('/api/mp/watch', { path }).then((d) => {
      $('watchInfo').textContent = ([].concat(d.watching || []).join('; ')) || '—';
      log('INFO', 'watching MP log: ' + d.watching);
    }).catch((e) => log('ERROR', 'watch failed: ' + e.message));
  });

  // ---- links ----
  pi = window.MissionLinks.initPiLink({
    onStatus: (s) => {
      const was = S.pi.connected;
      S.pi.connected = s.connected;
      $('piWs').textContent = s.connected
        ? ('open · ' + s.url) : ('down · retry ' + s.retry);
      setLamp('lampPi', s.connected);
      $('btnPiConnect').textContent = s.connected ? 'disconnect' : 'connect';
      // Fresh Pi (WS open <=> HTTP up, same server): probe the rig, show
      // only real tiles, and (re)start polling against THIS Pi URL — polls
      // bound at boot would otherwise stare at the old base forever.
      if (s.connected && !was) {
        probeCameras().then(() => { startAvailableCams(); seedCamControls(); });
      }
    },
    onEnvelope,
    onLog: log,
  });
  $('btnPiConnect').addEventListener('click', () => {
    if (S.pi.connected) {
      pi.close(true);
      S.pi.connected = false;
      $('piWs').textContent = 'closed by user';
      setLamp('lampPi', false);
      $('btnPiConnect').textContent = 'connect';
      return;
    }
    S.piUrl = $('piUrl').value.trim().replace(/\/$/, '') || window.location.origin;
    $('piUrl').value = S.piUrl;
    S.camsProbed = false; // new Pi => re-probe its rig on WS open
    S.camsAvail = { cam1: true, cam2: true };
    applyCamVisibility();
    pi.connect(S.piUrl);
  });

  bridge = window.MissionLinks.initBridge({
    onBridgeStatus: (ok) => { S.bridgeOk = ok; setLamp('lampBridge', ok); },
    onBridgeState: renderBridgeState,
    onMavState: renderMav,
    onFence: renderFence,
    onPlan: renderPlan,
    onTelemetry: (d) => { renderTelemetry(d); },
    onMpLine: (m) => log('MP', '[' + (m.source || 'mp-log') + '] ' + m.line),
    onMpQr,
    onMpConsole: renderMpConsole,
    onPiSse: (d, channel) => {
      if (!S.pi.connected) onEnvelope({ channel, t: Date.now() / 1000, data: d });
    },
  });
  bridge.connect('');

  // ---- cameras ----
  cams = window.MissionCameras.initCameras({
    getPiUrl: () => S.piUrl,
    onMode: (id, mode) => { $(id === 'cam1' ? 'c1Mode' : 'c2Mode').textContent = mode; },
    onLatency: (id, ms) => { $(id === 'cam1' ? 'c1Lat' : 'c2Lat').textContent = ms; },
    onStats: (id, t) => { $(id === 'cam1' ? 'camStats1' : 'camStats2').textContent = t; },
    onHint: (id, t) => { $(id === 'cam1' ? 'camHint1' : 'camHint2').textContent = t; },
    onLog: log,
  });
  $('tabCam1').addEventListener('click', () => {
    $('tabCam1').classList.add('on'); $('tabCam2').classList.remove('on');
    $('camCtls1').classList.remove('hidden'); $('camCtls2').classList.add('hidden');
  });
  $('tabCam2').addEventListener('click', () => {
    $('tabCam2').classList.add('on'); $('tabCam1').classList.remove('on');
    $('camCtls2').classList.remove('hidden'); $('camCtls1').classList.add('hidden');
  });
  CAMS.forEach((c) => {
    ['Exp', 'Gain', 'Bri', 'Con', 'Sat', 'Sha'].forEach((name) => {
      $(c.p + name).addEventListener('input', (e) => {
        $(c.p + name + 'V').textContent = e.target.value;
        scheduleCamPush(c.id, c.p);
      });
    });
    $(c.p + 'Af').addEventListener('change', () => scheduleCamPush(c.id, c.p));
    $(c.p + 'Adapt').addEventListener('change', () => scheduleCamPush(c.id, c.p));
  });
  $('btnModalCam1').addEventListener('click', () => cams.openModal('cam1'));
  $('btnModalCam2').addEventListener('click', () => cams.openModal('cam2'));
  $('btnModalClose').addEventListener('click', () => cams.closeModal());
  $('btnPauseCam1').addEventListener('click', (e) => {
    e.target.textContent = cams.togglePause('cam1') ? 'resume' : 'pause';
  });
  $('btnPauseCam2').addEventListener('click', (e) => {
    e.target.textContent = cams.togglePause('cam2') ? 'resume' : 'pause';
  });

  // ---- map toolbar ----
  $('selTile').addEventListener('change', (e) => { map && map.setTileSourceByName(e.target.value); });
  $('mapHint').addEventListener('click', () => { map && map.zoomToGrid(); });

  // ---- mock auto-connect (same origin serves UI + mock Pi + bridge) ----
  // No eager cam start here: probe + start + seed run on the first Pi WS
  // open (onStatus), so tiles always poll the connected Pi, not boot origin.
  S.piUrl = window.location.origin;
  $('piUrl').value = S.piUrl;
  pi.connect(S.piUrl);
  log('INFO', 'mission-ui v2 boot: Pi WS + bridge SSE starting on ' + S.piUrl);
}

document.addEventListener('DOMContentLoaded', boot);
})();
