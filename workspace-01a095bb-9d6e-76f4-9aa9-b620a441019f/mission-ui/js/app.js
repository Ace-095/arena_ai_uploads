/* app.js — UI v2 + arena/test1 wiring.
 *
 * Data ownership (ui-spec v2 + arena/test1):
 *  - flight truth (telemetry, mode, fence, plan, battery, MP console): BRIDGE, MAVLink via MP (SSE)
 *  - mission brain (fsm, system, Pi QR receipts, target events): PI (WebSocket, receive-only)
 *  - polygon (primary, shows on MP) + conversion to fence inclusion (button)
 *  - configurable max height via config.yaml (flight.max_alt_m) — 5/10/15 m etc
 *  - FOV-based coverage: drone covers max area according to cam FOV (cam.txt)
 *  - OUT to FC: fence/polygon upload/clear, RTL/LAND, param read — via bridge MP link
 *  - OUT to Pi: camera tuning only
 */
(function () {
'use strict';

const $ = (id) => document.getElementById(id);
const nowStr = () => new Date().toLocaleTimeString('en-GB');
const fmtInt = (n) => Math.round(n).toLocaleString('en-US').replace(/,/g, ' ');
const esc = (s) => String(s).replace(/[&<>\"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '\"': '&quot;' }[c]));

const S = {
  piUrl: '', isMock: false, bridgeOk: false,
  pi: { connected: false },
  camsAvail: { cam1: true, cam2: true }, camsProbed: false,
  mav: null, fence: null, polygon: null, plan: null, fsm: null,
  qr: { payload: '', streak: 0, required: 3, confirmed: false },
  receipts: { pi: '', mavlink: '', mpfile: '', manual: '' },
  logs: [],
  config: {
    max_alt_m: 15.0,
    sweep_alt_m: 15.0,
    overlap: 0.3,
    camera: 'bottom',
    source: 'default',
  },
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
  if (!box) return;
  box.innerHTML = S.logs
    .filter((l) => !f || l.level === f)
    .map((l) => '<div class="lr ' + l.level + '"><span class="lt">' + l.ts + '</span> ' +
      '<span class="ll">' + l.level + '</span> ' + esc(l.msg) + '</div>')
    .join('');
  box.scrollTop = box.scrollHeight;
}

function setLamp(id, on) {
  const el = $(id);
  if (!el) return;
  el.classList.toggle('on', !!on);
  el.classList.toggle('off', !on);
}

// ------------------------------------------------------- bridge-owned --
function renderTelemetry(d) {
  if ($('tLatLon')) $('tLatLon').textContent = d.lat.toFixed(7) + ' / ' + d.lon.toFixed(7);
  if ($('tAlt')) $('tAlt').textContent = d.alt_rel.toFixed(1) + ' / ' + d.alt_msl.toFixed(1) + ' m';
  if ($('tMode')) $('tMode').textContent = (d.mode || '—') + (d.armed ? ' · ARMED' : '');
  const sp = Math.hypot(d.vx || 0, d.vy || 0);
  if ($('tVel')) $('tVel').textContent = sp.toFixed(1) + ' m/s (vz ' + (d.vz != null ? d.vz.toFixed(1) : '—') + ')';
  if ($('tAtt')) $('tAtt').textContent = d.attitude
    ? d.attitude.roll.toFixed(1) + ' / ' + d.attitude.pitch.toFixed(1) + ' / ' + d.attitude.yaw.toFixed(1) : '—';
  if ($('tGps')) $('tGps').textContent = d.gps ? ('fix ' + d.gps.fix + ' / ' + d.gps.sats + ' sats') : '—';
  if ($('tBatt')) $('tBatt').textContent = d.battery
    ? ((d.battery.v != null ? d.battery.v.toFixed(1) + ' V' : '—') +
       (d.battery.pct != null ? ' · ' + d.battery.pct + '%' : '') +
       (d.battery.current_a != null ? ' · ' + d.battery.current_a.toFixed(1) + ' A' : '')) : '—';
  if ($('tMis')) $('tMis').textContent = d.mission ? ('seq ' + d.mission.seq + ' / ' + (d.mission.total || '?')) : '—';
  if (map) map.setDrone(d.lat, d.lon, d.hdg);
}

function renderMav(d) {
  S.mav = d;
  const b = $('mavBadge');
  if (b) {
    b.textContent = d.connected ? ('CONNECTED · ' + (d.mode || '?')) : 'DISCONNECTED';
    b.classList.toggle('ok', !!d.connected);
  }
  setLamp('lampMav', d.connected);
  if ($('mavVeh')) $('mavVeh').textContent = d.vehicle_sysid != null
    ? ('sysid ' + d.vehicle_sysid + ' / type ' + d.vehicle_type) : '—';
  if ($('mavMode')) $('mavMode').textContent = (d.mode || '—') + (d.armed ? ' · ARMED' : '') +
    (d.hb_age_s != null ? ' · hb ' + d.hb_age_s + 's ago' : '');
  if ($('mavRx')) $('mavRx').textContent = d.rx_rate + ' msg/s · ' + fmtInt(d.rx_msgs) + ' total · :' + d.port;
  if (d.port != null && $('mavPort') && document.activeElement !== $('mavPort')) $('mavPort').value = d.port;
  if ($('mavSender')) $('mavSender').textContent = (d.sender || 'no forwarder yet') + (d.tx_msgs != null ? ' · tx ' + fmtInt(d.tx_msgs) : '') + (d.sender_changes ? ' · mpswitch ' + d.sender_changes : '') + (d.link_flaps ? ' · flaps ' + d.link_flaps : '');
  renderPlan(S.plan || d.plan);
  renderArmable();
}

function renderFence(d) {
  S.fence = d;
  if (!d) {
    if ($('fState')) $('fState').textContent = '—';
    if ($('fReadback')) $('fReadback').textContent = '—';
    if ($('fVerts')) $('fVerts').textContent = '—';
    if ($('fArea')) $('fArea').textContent = '—';
    if ($('fRadius')) $('fRadius').textContent = '—';
    if ($('fOrigin')) $('fOrigin').textContent = '—';
    return;
  }
  if ($('fState')) {
    $('fState').textContent = d.loaded ? 'LOADED on FC' : 'not loaded';
    $('fState').style.color = d.loaded ? (d.confirmed ? '#5fd97a' : '#ffb020') : '';
  }
  if ($('fReadback')) $('fReadback').textContent = d.loaded ? (d.confirmed ? 'MATCH ✓' : 'MISMATCH ✗') : '—';
  if ($('fVerts')) $('fVerts').textContent = d.vertex_count != null ? d.vertex_count : '—';
  if ($('fArea')) $('fArea').textContent = d.area_m2 != null ? fmtInt(d.area_m2) + ' m²' : '—';
  if ($('fRadius')) $('fRadius').textContent = d.max_radius_m != null ? d.max_radius_m + ' m' : '—';
  if ($('fOrigin')) $('fOrigin').textContent = (d.origin_lat != null)
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

function renderPolygon(polyVerts) {
  S.polygon = polyVerts;
  if ($('pVerts')) $('pVerts').textContent = polyVerts ? polyVerts.length + ' verts' : '—';
  if (map && polyVerts && polyVerts.length >= 3) {
    // also update FOV coverage when polygon changes
    map.updateFovCoverage();
  }
}

function renderPlan(p) {
  if (p) S.plan = p;
  p = S.plan;
  const txt = !p ? '—'
    : (p.synced ? (p.total + ' items' + (p.do_sprayer_seq != null ? ' · sprayer trig #' + p.do_sprayer_seq : ' · no sprayer cmd'))
      : 'empty');
  if ($('planInfo')) $('planInfo').textContent = txt;
  if ($('mavPlan')) $('mavPlan').textContent = txt;
}

function renderArmable() {
  const ok = !!(S.mav && S.mav.connected && S.fence && S.fence.loaded && S.fence.confirmed);
  const el = $('armableInfo');
  if (!el) return;
  el.textContent = ok ? 'YES — fence confirmed on FC (from polygon)' : 'NO — need MAVLink link + confirmed fence (draw polygon → fence)';
  el.style.color = ok ? '#5fd97a' : '#ff6b6b';
}

function renderBridgeState(d) {
  S.isMock = !!d.mock;
  if ($('mockBadge')) $('mockBadge').classList.toggle('hidden', !d.mock);
  if ($('piMode')) $('piMode').textContent = d.mock ? 'mock (bridge loopback)' : 'real';
  if ($('watchInfo')) $('watchInfo').textContent = ([].concat(d.watching || []).join('; ')) || '—';
  if (d.qr && d.qr.payload && !S.qr.payload) {
    S.qr.payload = d.qr.payload;
    if ($('qrPayload')) $('qrPayload').textContent = d.qr.payload;
    log('INFO', 'QR already latched on bridge: ' + d.qr.payload);
  }
  if (d.mavlink) {
    if (!S.mav) {
      if ($('mavBadge')) $('mavBadge').textContent = d.mavlink.connected ? 'CONNECTED' : 'DISCONNECTED';
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
  if (!box) return;
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
  if (m.source === 'mavlink') { S.receipts.mavlink = t; if ($('qrSrcMav')) $('qrSrcMav').textContent = t; }
  else if (m.source === 'manual') { S.receipts.manual = t; if ($('qrSrcMan')) $('qrSrcMan').textContent = t; }
  else { S.receipts.mpfile = t; if ($('qrSrcMp')) $('qrSrcMp').textContent = t + ' (' + m.source + ')'; }
  if (m.payload && !S.qr.payload) {
    S.qr.payload = m.payload;
    if ($('qrPayload')) $('qrPayload').textContent = m.payload;
  }
  log('WARN', 'QR via ' + m.source + ': ' + m.payload);
}

// ----------------------------------------------------------- pi-owned --
function renderSystem(d) {
  if (d.temp_c != null && $('sTemp')) $('sTemp').textContent = d.temp_c.toFixed(1) + ' °C';
  else if (d.temp != null && $('sTemp')) $('sTemp').textContent = d.temp + ' °C';
  const cpu = d.cpu_pct != null ? d.cpu_pct : d.cpu;
  const mem = d.mem_pct != null ? d.mem_pct : d.mem;
  if ((cpu != null || mem != null) && $('sCpu'))
    $('sCpu').textContent = (cpu != null ? cpu.toFixed(0) + '%' : '?') + ' / ' + (mem != null ? mem.toFixed(0) + '%' : '?');
  if (d.disk_pct != null && $('sDisk')) $('sDisk').textContent = d.disk_pct.toFixed(0) + '%';
  else if (d.disk != null && $('sDisk')) $('sDisk').textContent = d.disk + '%';
  if (d.uptime_s != null && $('sUp')) $('sUp').textContent = Math.floor(d.uptime_s / 60) + 'm ' + Math.floor(d.uptime_s % 60) + 's';
  // arena/test1: show max alt from Pi
  if (d.max_alt_m != null && $('piMaxAlt')) $('piMaxAlt').textContent = d.max_alt_m + ' m';
  else if (d.fov && d.fov.max_alt_m != null && $('piMaxAlt')) $('piMaxAlt').textContent = d.fov.max_alt_m + ' m (sweep ' + (d.fov.alt_m || '?') + 'm)';
}

function renderFsm(d, via) {
  S.fsm = d;
  const state = (d.state || d.to || d.phase || '?') + (via === 'poll' ? ' (polled)' : '');
  const reason = d.reason || (d.acknowledged && (d.acknowledged.reason || d.acknowledged.by)) || '—';
  if ($('fsmBadge')) $('fsmBadge').textContent = state;
  if ($('fsmState')) $('fsmState').textContent = state;
  if ($('fsmReason')) $('fsmReason').textContent = reason;
  // arena/test1: update config display from FSM
  if (d.max_alt_m != null) {
    S.config.max_alt_m = d.max_alt_m;
    if ($('cfgMaxAlt')) $('cfgMaxAlt').textContent = d.max_alt_m + ' m';
    if ($('cfgMaxInput')) $('cfgMaxInput').value = d.max_alt_m;
  }
  if (d.sweep_alt_m != null) {
    S.config.sweep_alt_m = d.sweep_alt_m;
    if ($('cfgSweepAlt')) $('cfgSweepAlt').textContent = d.sweep_alt_m + ' m';
    if ($('cfgSweepInput')) $('cfgSweepInput').value = d.sweep_alt_m;
  }
  if (d.fov) {
    if ($('covFov')) $('covFov').textContent = (d.fov.cam || 'bottom') + ' ' + (d.fov.hfov_deg || '?') + '° HFOV @ ' + (d.fov.alt_m || '?') + 'm footprint ' + (d.fov.footprint_w_m || '?') + 'x' + (d.fov.footprint_h_m || '?') + 'm';
  }
}

function renderQrUpdate(d) {
  if (d.payload) {
    S.qr.payload = d.payload;
    if ($('qrPayload')) $('qrPayload').textContent = d.payload;
  }
  if (d.streak != null) S.qr.streak = d.streak;
  else if (d.count != null) S.qr.streak = d.count;
  if (d.required != null) S.qr.required = d.required;
  S.qr.confirmed = d.confirmed != null ? !!d.confirmed : (S.qr.streak >= S.qr.required && S.qr.streak > 0);
  if ($('qrStreak')) $('qrStreak').textContent = S.qr.streak + ' / ' + S.qr.required + (S.qr.confirmed ? ' ✓' : '');
  S.receipts.pi = nowStr();
  if ($('qrSrcPi')) $('qrSrcPi').textContent = S.receipts.pi;
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
    case 'fsm': renderFsm(d, env.via); if (map && (d.phase || d.state || d.to) === 'WAIT_LINK') map.clearCoverage(); break;
    case 'coverage': if (map) map.setCoverage(d); break;
    case 'log': log(d.level || 'INFO', d.msg || JSON.stringify(d).slice(0, 200)); break;
    default: break;
  }
}

// ------------------------------------------------- camera live controls --
const CAMS = [{ id: 'cam1', p: 'c1' }, { id: 'cam2', p: 'c2' }];
const camTimers = {};

function setSlider(p, name, v) {
  if (v == null) return;
  const s = $(p + name);
  if (s) { s.value = v; const out = $(p + name + 'V'); if (out) out.textContent = v; }
}

function probeCameras() {
  if (!S.piUrl) return Promise.resolve(null);
  return fetch(S.piUrl + '/api/cameras').then((r) => {
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then((st) => {
    const avail = { cam1: !!(st && st.cam1), cam2: !!(st && st.cam2) };
    if (!avail.cam1 && !avail.cam2) return null;
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
    const v1 = $('camView' + n), v2 = $('tabCam' + n);
    if (v1) v1.classList.toggle('hidden', !show);
    if (v2) v2.classList.toggle('hidden', !show);
  });
  const v1 = a.cam1 !== false, v2 = a.cam2 !== false;
  const tab1 = $('tabCam1');
  if (!tab1) return;
  const c1on = tab1.classList.contains('on');
  if (c1on && !v1 && v2) { const t2 = $('tabCam2'); if (t2) t2.click(); }
  else if (!c1on && !v2 && v1) tab1.click();
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
      const af = $(c.p + 'Af'); if (af && k.af_mode) af.value = k.af_mode;
      const ad = $(c.p + 'Adapt'); if (ad && k.adaptive != null) ad.checked = !!k.adaptive;
    }).catch(() => {});
  });
}

function scheduleCamPush(camId, p) {
  const live = $(p + 'Live'); if (live) live.textContent = 'sending…';
  if (camTimers[camId]) clearTimeout(camTimers[camId]);
  camTimers[camId] = setTimeout(() => pushCamControls(camId, p), 250);
}

function pushCamControls(camId, p) {
  if (!S.piUrl) { const live = $(p + 'Live'); if (live) live.textContent = 'no Pi link'; return; }
  const body = {
    cam: camId,
    exposure_us: +$(p + 'Exp').value, gain_db: +$(p + 'Gain').value,
    af_mode: $(p + 'Af').value, adaptive: $(p + 'Adapt').checked,
    brightness: +$(p + 'Bri').value, contrast: +$(p + 'Con').value,
    saturation: +$(p + 'Sat').value, sharpness: +$(p + 'Sha').value,
  };
  postJson(S.piUrl + '/api/camera/controls', body).then(() => {
    const live = $(p + 'Live'); if (live) live.textContent = 'applied ✓ ' + nowStr();
  }).catch((e) => {
    const live = $(p + 'Live'); if (live) live.textContent = 'error: ' + e.message;
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

function loadUiConfig() {
  // arena/test1: load config.yaml for max height + FOV
  fetch('/config.yaml').then(r => r.ok ? r.text() : Promise.reject()).then(txt => {
    let maxM = null, sweepM = null, overlap = null;
    const mMax = txt.match(/max_alt_m:\s*([\d.]+)/);
    if (mMax) maxM = parseFloat(mMax[1]);
    const mSweep = txt.match(/default_sweep_alt_m:\s*([\d.]+)/);
    if (mSweep) sweepM = parseFloat(mSweep[1]);
    const mOver = txt.match(/^\s*overlap:\s*([\d.]+)/m);
    if (mOver) overlap = parseFloat(mOver[1]);

    if (maxM != null) {
      S.config.max_alt_m = maxM;
      S.config.source = 'config.yaml';
      if ($('cfgMaxAlt')) $('cfgMaxAlt').textContent = maxM + ' m';
      if ($('cfgMaxInput')) $('cfgMaxInput').value = maxM;
      if (map) map.setMaxAlt(maxM);
      log('INFO', 'config.yaml max_alt_m = ' + maxM + ' m');
    }
    if (sweepM != null) {
      S.config.sweep_alt_m = Math.min(sweepM, S.config.max_alt_m);
      if ($('cfgSweepAlt')) $('cfgSweepAlt').textContent = S.config.sweep_alt_m + ' m';
      if ($('cfgSweepInput')) $('cfgSweepInput').value = S.config.sweep_alt_m;
      if (map) map.setSweepAlt(S.config.sweep_alt_m);
    }
    if (overlap != null) {
      S.config.overlap = overlap;
      if ($('cfgOverlapInput')) $('cfgOverlapInput').value = overlap;
      if (map) map.setOverlap(overlap);
    }
    if ($('cfgSource')) $('cfgSource').textContent = 'config.yaml (' + S.config.max_alt_m + 'm max)';
  }).catch(() => {
    log('INFO', 'no /config.yaml, using defaults (15m max)');
    if ($('cfgSource')) $('cfgSource').textContent = 'defaults (15m) — create config.yaml to change';
  });

  // also try /api/config (bridge may serve it)
  fetch('/api/config').then(r => r.ok ? r.json() : Promise.reject()).then(cfg => {
    if (cfg && cfg.flight && cfg.flight.max_alt_m) {
      S.config.max_alt_m = cfg.flight.max_alt_m;
      if ($('cfgMaxAlt')) $('cfgMaxAlt').textContent = cfg.flight.max_alt_m + ' m';
      if ($('cfgMaxInput')) $('cfgMaxInput').value = cfg.flight.max_alt_m;
      if (map) map.setMaxAlt(cfg.flight.max_alt_m);
      S.config.source = 'api/config';
      if ($('cfgSource')) $('cfgSource').textContent = '/api/config (' + cfg.flight.max_alt_m + 'm)';
    }
  }).catch(() => {});
}

// ------------------------------------------------------------------ boot --
function boot() {
  const clockEl = $('clock');
  if (clockEl) setInterval(() => { clockEl.textContent = nowStr(); }, 1000);
  if (clockEl) clockEl.textContent = nowStr();
  const logFilter = $('logFilter');
  if (logFilter) logFilter.addEventListener('change', renderLogs);
  const btnLogClear = $('btnLogClear');
  if (btnLogClear) btnLogClear.addEventListener('click', () => { S.logs = []; renderLogs(); });
  const btnLogDown = $('btnLogDownload');
  if (btnLogDown) btnLogDown.addEventListener('click', () => {
    const blob = new Blob([S.logs.map((l) => l.ts + ' ' + l.level + ' ' + l.msg).join('\n')],
      { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = 'mission-ui.log'; a.click();
  });
  const piUrlInput = $('piUrl');
  if (piUrlInput) piUrlInput.value = window.location.origin;

  // ---- map ----
  fetch('/api/mp/tiles-info').then((r) => r.json()).then((info) => {
    map = window.MissionMap.initMap({
      onLog: log,
      tilesAvailable: !!(info && info.available),
      tilesZmax: (info && info.zmax) || 18,
    });
    log('INFO', 'map ready (tiles: ' + (info && info.available ? 'offline pack' : 'online fallback') + ') · arena/test1 polygon mode');
    loadUiConfig();
    // after map ready, wire polygon buttons
    wirePolygonButtons();
  }).catch((e) => {
    map = window.MissionMap.initMap({ onLog: log, tilesAvailable: false });
    log('WARN', 'tiles-info failed, online tiles: ' + e.message);
    loadUiConfig();
    wirePolygonButtons();
  });

  function wirePolygonButtons() {
    if (!map) return;

    // ---- CONFIG card — UI now drives alt via Pi, MP sees instantly ----
    const btnCfgApply = $('btnCfgApply');
    if (btnCfgApply) btnCfgApply.addEventListener('click', () => {
      const maxV = parseFloat($('cfgMaxInput').value);
      const sweepV = parseFloat($('cfgSweepInput').value);
      if (isNaN(maxV) || maxV < 1 || maxV > 50) { log('ERROR', 'bad max height'); return; }
      if (isNaN(sweepV) || sweepV < 1) { log('ERROR', 'bad sweep alt'); return; }
      const clampedSweep = Math.min(sweepV, maxV);
      S.config.max_alt_m = maxV;
      S.config.sweep_alt_m = clampedSweep;
      map.setMaxAlt(maxV);
      map.setSweepAlt(clampedSweep);
      if ($('cfgMaxAlt')) $('cfgMaxAlt').textContent = maxV + ' m';
      if ($('cfgSweepAlt')) $('cfgSweepAlt').textContent = clampedSweep + ' m';
      if ($('cfgSource')) $('cfgSource').textContent = 'manual (' + maxV + 'm max, ' + clampedSweep + 'm sweep)';
      log('WARN', 'altitude config: max=' + maxV + 'm sweep=' + clampedSweep + 'm (pushing to Pi, MP sees via FENCE_ALT_MAX)');
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (piBase) {
        postJson(piBase + '/api/config', { max_alt_m: maxV, sweep_alt_m: clampedSweep })
          .then((st) => {
            log('INFO', 'Pi alt updated: max=' + st.max_alt_m + 'm sweep=' + (st.updated.sweep_alt_m || clampedSweep) + 'm — MP Fence alt instant via FENCE_ALT_MAX param');
          })
          .catch((e) => log('ERROR', 'Pi alt push failed: ' + e.message));
      }
    });

    const cfgCamSel = $('cfgCamSelect');
    if (cfgCamSel) cfgCamSel.addEventListener('change', (e) => {
      S.config.camera = e.target.value;
      map.setFovCamera(e.target.value);
      if ($('covFov')) {
        const spec = map.CAM_SPECS ? map.CAM_SPECS[e.target.value] : null;
        if (spec) $('covFov').textContent = e.target.value + ' ' + spec.hfov_deg + '° HFOV @ ' + S.config.sweep_alt_m + 'm';
      }
      log('INFO', 'FOV camera switched to ' + e.target.value);
    });

    const cfgOverlap = $('cfgOverlapInput');
    if (cfgOverlap) cfgOverlap.addEventListener('change', (e) => {
      const v = parseFloat(e.target.value);
      if (!isNaN(v) && v >= 0 && v < 0.9) {
        S.config.overlap = v;
        map.setOverlap(v);
        log('INFO', 'overlap set to ' + v);
      }
    });

    // ---- POLYGON (primary) ----
    const btnPolyDraw = $('btnPolyDraw');
    if (btnPolyDraw) btnPolyDraw.addEventListener('click', () => {
      const on = map.setPolygonDrawing(btnPolyDraw.classList.toggle('on'));
      if (!on) btnPolyDraw.classList.remove('on');
      log('INFO', on ? 'polygon drawing ON — click map to add vertices (shows on MP after convert)' : 'polygon drawing OFF');
    });

    const btnPolyUndo = $('btnPolyUndo');
    if (btnPolyUndo) btnPolyUndo.addEventListener('click', () => { map.undoPolygonVertex(); });

    const btnPolyClear = $('btnPolyClear');
    if (btnPolyClear) btnPolyClear.addEventListener('click', () => {
      map.clearPolygon();
      if ($('pConverted')) $('pConverted').textContent = 'no';
      log('INFO', 'polygon cleared');
    });

    const btnPolyToFence = $('btnPolyToFence');
    if (btnPolyToFence) btnPolyToFence.addEventListener('click', () => {
      const verts = map.polygonToFence();
      if (!verts) return;
      S.polygon = verts;
      renderPolygon(verts);
      if ($('pConverted')) { $('pConverted').textContent = 'yes → fence inclusion'; $('pConverted').style.color = '#5fd97a'; }
      log('WARN', 'polygon → fence inclusion: ' + verts.length + ' verts (now can APPLY FENCE → MP)');
    });

    const btnPolyApply = $('btnPolyApply');
    if (btnPolyApply) btnPolyApply.addEventListener('click', () => {
      let verts = map.getPolygonVertices();
      if (verts.length < 3) {
        // fallback to fence drawing if polygon empty
        verts = map.getVertices();
        if (verts.length < 3) { log('ERROR', 'need ≥3 polygon vertices (have ' + verts.length + ')'); return; }
      }
      if (!window.confirm('Apply ' + verts.length + '-vertex POLYGON as FENCE via Pi to FC? MP will see instantly.')) return;
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (piBase) {
        postJson(piBase + '/api/fence', { vertices: verts })
          .then((st) => {
            log('WARN', 'Pi polygon->fence: ' + st.count + ' verts, FC=' + st.fc_uploaded + ' — MP Fence tab instant');
            if ($('pConverted')) { $('pConverted').textContent = 'uploaded via Pi ✓ ' + st.count + ' verts'; $('pConverted').style.color = '#5fd97a'; }
            return postJson('/api/mp/fence', { vertices: verts, mission_type: 'fence', fence_action: null }).catch(()=>st);
          })
          .then((st) => { renderFence(st); })
          .catch((e) => log('ERROR', 'Pi polygon fence upload failed: ' + e.message));
      } else {
        postJson('/api/mp/fence', { vertices: verts, mission_type: 'fence', fence_action: null })
          .then((st) => {
            renderFence(st);
            if ($('pConverted')) { $('pConverted').textContent = 'uploaded ✓'; $('pConverted').style.color = '#5fd97a'; }
            log('WARN', 'polygon as fence upload done via bridge: ' + st.vertex_count + ' verts, ' + (st.confirmed ? 'readback MATCH' : 'READBACK MISMATCH') + ' — check MP Fence tab');
          })
          .catch((e) => log('ERROR', 'polygon fence upload failed: ' + e.message));
      }
    });

    const btnPolyExport = $('btnPolyExport');
    if (btnPolyExport) btnPolyExport.addEventListener('click', () => {
      const verts = map.getPolygonVertices();
      if (!verts.length) { log('ERROR', 'nothing to export (no polygon)'); return; }
      const txt = verts.map((v) => v.lat.toFixed(7) + ',' + v.lon.toFixed(7)).join('\n') + '\n';
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' }));
      a.download = 'polygon.poly'; a.click();
      log('INFO', 'exported ' + verts.length + ' verts to polygon.poly');
    });

    const btnPolyShowFov = $('btnPolyShowFov');
    if (btnPolyShowFov) btnPolyShowFov.addEventListener('click', () => {
      map.updateFovCoverage();
      const plan = map.getFovPlan();
      if (plan) {
        log('INFO', 'FOV coverage: ' + plan.waypoints.length + ' waypoints, footprint ' + plan.footprint[0].toFixed(1) + 'x' + plan.footprint[1].toFixed(1) + 'm @ ' + plan.alt + 'm');
      } else {
        log('ERROR', 'no polygon for FOV coverage — draw polygon first');
      }
    });

    // ---- FOV coverage ----
    const btnCovClear = $('btnCovClear');
    if (btnCovClear) btnCovClear.addEventListener('click', () => {
      map.clearFovPlan();
      log('INFO', 'FOV coverage viz cleared');
    });

    const btnCovExport = $('btnCovExport');
    if (btnCovExport) btnCovExport.addEventListener('click', () => {
      const plan = map.getFovPlan();
      if (!plan || !plan.waypoints.length) { log('ERROR', 'no FOV plan to export — draw polygon + show coverage first'); return; }
      const out = {
        camera: plan.camera,
        alt_m: plan.alt,
        max_alt_m: S.config.max_alt_m,
        footprint_w_m: plan.footprint[0],
        footprint_h_m: plan.footprint[1],
        spacing_m: plan.spacing,
        area_m2: plan.area_m2,
        waypoints: plan.waypoints.map((wp) => ({ lat: wp[0], lon: wp[1] })),
      };
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([JSON.stringify(out, null, 2)], { type: 'application/json' }));
      a.download = 'fov_coverage.json'; a.click();
      log('INFO', 'exported FOV coverage ' + out.waypoints.length + ' wps to fov_coverage.json');
    });

    // ---- QR BOOST (Kabaddi setting — make QR pop vs ground) ----
    const qrBoostProfile = $('qrBoostProfile');
    const qrSwMode = $('qrSwMode');
    const qrBoostInfo = $('qrBoostInfo');

    function applyQrBoostProfile(profileName) {
      const cam = S.camActive === 'cam1' ? 'cam1' : 'cam2';
      // Map profile to ISP tuning (same as qr_camera_boost.py)
      const profiles = {
        normal: { adaptive: true, exposure_us: 8333, gain_db: 6, brightness: 0, contrast: 1.0, saturation: 1.0, sharpness: 1.0 },
        qr_boost_day: { adaptive: false, exposure_us: 5000, gain_db: 2, brightness: -10, contrast: 1.8, saturation: 0.7, sharpness: 2.0 },
        qr_boost_aggressive: { adaptive: false, exposure_us: 4000, gain_db: 1, brightness: -15, contrast: 2.0, saturation: 0.5, sharpness: 2.2 },
        qr_boost_lowlight: { adaptive: false, exposure_us: 8000, gain_db: 6, brightness: 5, contrast: 1.6, saturation: 0.8, sharpness: 1.8 },
        bottom_qr_boost: { adaptive: false, exposure_us: 5000, gain_db: 2, brightness: -12, contrast: 1.9, saturation: 0.6, sharpness: 2.0, af_mode: 'manual' },
        front_qr_boost: { adaptive: false, exposure_us: 6000, gain_db: 3, brightness: -5, contrast: 1.6, saturation: 0.8, sharpness: 1.8, af_mode: 'continuous' },
      };
      const tuning = profiles[profileName] || profiles.qr_boost_day;
      tuning.cam = cam;
      // POST to Pi /api/camera/controls
      postJson('/api/camera/controls', tuning).then(() => {
        log('WARN', cam + ' QR boost ISP ' + profileName + ' applied — contrast ' + tuning.contrast + ' sat ' + tuning.saturation + ' sharp ' + tuning.sharpness + ' (QR pops vs ground)');
        if (qrBoostInfo) qrBoostInfo.textContent = profileName + ': contrast ' + tuning.contrast + ' sat ' + tuning.saturation + ' sharp ' + tuning.sharpness + ' bright ' + tuning.brightness + ' exp ' + tuning.exposure_us + 'us — ' + (profileName.includes('aggressive') ? 'ground almost gray, QR B/W stays' : 'QR B/W pops vs green/brown ground');
        // Also update sliders to reflect new values
        const pfx = cam === 'cam1' ? 'c1' : 'c2';
        if ($(pfx+'Con')) { $(pfx+'Con').value = tuning.contrast; if ($(pfx+'ConV')) $(pfx+'ConV').textContent = tuning.contrast; }
        if ($(pfx+'Sat')) { $(pfx+'Sat').value = tuning.saturation; if ($(pfx+'SatV')) $(pfx+'SatV').textContent = tuning.saturation; }
        if ($(pfx+'Sha')) { $(pfx+'Sha').value = tuning.sharpness; if ($(pfx+'ShaV')) $(pfx+'ShaV').textContent = tuning.sharpness; }
        if ($(pfx+'Bri')) { $(pfx+'Bri').value = tuning.brightness; if ($(pfx+'BriV')) $(pfx+'BriV').textContent = tuning.brightness; }
      }).catch((e) => log('ERROR', 'QR boost ISP failed: ' + e.message));
    }

    const btnQrBoostApply = $('btnQrBoostApply');
    if (btnQrBoostApply) btnQrBoostApply.addEventListener('click', () => {
      const prof = qrBoostProfile ? qrBoostProfile.value : 'qr_boost_day';
      applyQrBoostProfile(prof);
    });

    const btnQrBoostOff = $('btnQrBoostOff');
    if (btnQrBoostOff) btnQrBoostOff.addEventListener('click', () => {
      applyQrBoostProfile('normal');
      if (qrBoostInfo) qrBoostInfo.textContent = 'reset to normal — auto exposure, contrast 1.0 sat 1.0 sharp 1.0';
    });

    const btnQrSwApply = $('btnQrSwApply');
    if (btnQrSwApply) btnQrSwApply.addEventListener('click', () => {
      const cam = S.camActive === 'cam1' ? 'cam1' : 'cam2';
      const mode = qrSwMode ? qrSwMode.value : 'qr_boost';
      postJson('/api/camera/controls', { cam, qr_software_enhance: mode !== 'none', qr_enhance_mode: mode }).then(() => {
        log('INFO', cam + ' software QR enhance ' + mode + ' — CLAHE+unsharp+green suppress, QR pops vs ground');
      }).catch((e) => log('ERROR', 'software enhance failed: ' + e.message));
    });

    // Quick preset buttons
    const btnQrBoostDay = $('btnQrBoostDay');
    if (btnQrBoostDay) btnQrBoostDay.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'qr_boost_day'; applyQrBoostProfile('qr_boost_day'); });
    const btnQrBoostAgg = $('btnQrBoostAgg');
    if (btnQrBoostAgg) btnQrBoostAgg.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'qr_boost_aggressive'; applyQrBoostProfile('qr_boost_aggressive'); });
    const btnQrBoostBottom = $('btnQrBoostBottom');
    if (btnQrBoostBottom) btnQrBoostBottom.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'bottom_qr_boost'; applyQrBoostProfile('bottom_qr_boost'); });
    const btnQrBoostFront = $('btnQrBoostFront');
    if (btnQrBoostFront) btnQrBoostFront.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'front_qr_boost'; applyQrBoostProfile('front_qr_boost'); });
    const btnPresetDark = $('btnPresetDark');
    if (btnPresetDark) btnPresetDark.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'dark'; applyQrBoostProfile('dark'); applyPreset('dark', 'cam2', false); });
    const btnPresetDarkQr = $('btnPresetDarkQr');
    if (btnPresetDarkQr) btnPresetDarkQr.addEventListener('click', () => { if (qrBoostProfile) qrBoostProfile.value = 'dark_qr_boost'; applyQrBoostProfile('dark_qr_boost'); applyPreset('dark_qr_boost', 'cam2', false); });

    // ---- PRESETS + FAKE FILTER — integrated workflow (one click dark + zero fakes) ----
    const presetSelect = $('presetSelect');
    const presetActive = $('presetActive');
    const presetInfo = $('presetInfo');
    const fakeFilterInfo = $('fakeFilterInfo');

    function applyPreset(presetName, cam, allCams) {
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (!piBase) { log('ERROR', 'no Pi URL for preset apply'); return; }
      const body = { cam: cam || 'cam2', all: !!allCams };
      // Try new /api/presets endpoint (integrated)
      fetch(piBase + '/api/presets/' + presetName, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      }).then(r => r.ok ? r.json() : r.text().then(t=>{throw new Error(t)})).then(res => {
        log('WARN', 'Preset ' + presetName + ' → ' + (body.all ? 'ALL cams' : body.cam) + ': ' + (res.description || '') + ' — ' + JSON.stringify(res.detector || {}));
        if (presetActive) presetActive.textContent = presetName + ' → ' + (body.all ? 'ALL' : body.cam);
        if (presetInfo) presetInfo.textContent = presetName + ': ' + (res.description || '') + ' detector ' + JSON.stringify(res.detector || {});
        if (fakeFilterInfo) fakeFilterInfo.textContent = 'detector ' + JSON.stringify(res.detector || {}) + ' — grill 0.16/0.24 hidden';
        // Also update QR boost dropdown to reflect
        if ($('qrBoostProfile')) $('qrBoostProfile').value = presetName;
        // Refresh camera controls
        setTimeout(seedCamControls, 500);
      }).catch(e => {
        // Fallback to old /api/camera/controls
        log('INFO', 'presets API failed, fallback to camera controls: ' + e.message);
        applyQrBoostProfile(presetName);
      });
    }

    function loadPresets() {
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (!piBase) return;
      fetch(piBase + '/api/presets').then(r => r.ok ? r.json() : Promise.reject()).then(res => {
        if (res && res.presets) {
          if (presetActive) presetActive.textContent = (res.active || 'dark_qr_boost') + ' (' + (res.available ? res.available.length : '?') + ' available)';
          if (presetInfo && res.presets[res.active || 'dark_qr_boost']) {
            const p = res.presets[res.active || 'dark_qr_boost'];
            presetInfo.textContent = (res.active || 'dark_qr_boost') + ': ' + p.description + ' detector ' + JSON.stringify(p.detector || {});
          }
          log('INFO', 'presets loaded: ' + (res.available ? res.available.join(', ') : ''));
        }
        return fetch(piBase + '/api/detector/fake_filter');
      }).then(r => r && r.ok ? r.json() : Promise.reject()).then(res => {
        if (res && res.fake_filter) {
          const ff = res.fake_filter;
          if ($('fakeEnabled')) $('fakeEnabled').checked = ff.enabled !== false;
          if ($('fakeHide')) $('fakeHide').checked = ff.hide_fake !== false;
          if ($('fakeRequire')) $('fakeRequire').checked = !!ff.require_decode;
          if ($('fakeConf')) $('fakeConf').value = ff.conf_thr || 0.35;
          if ($('fakeMinSize')) $('fakeMinSize').value = ff.min_size || 15;
          if (fakeFilterInfo) fakeFilterInfo.textContent = JSON.stringify(ff) + ' — grill 0.16/0.24 ' + (ff.hide_fake ? 'hidden' : 'shown');
        }
      }).catch(()=>{});
    }

    // Wire preset buttons
    const btnPresetApplyCam2 = $('btnPresetApplyCam2');
    if (btnPresetApplyCam2) btnPresetApplyCam2.addEventListener('click', () => {
      const name = presetSelect ? presetSelect.value : 'dark_qr_boost';
      applyPreset(name, 'cam2', false);
    });
    const btnPresetApplyAll = $('btnPresetApplyAll');
    if (btnPresetApplyAll) btnPresetApplyAll.addEventListener('click', () => {
      const name = presetSelect ? presetSelect.value : 'dark_qr_boost';
      applyPreset(name, 'cam2', true);
    });
    const btnPresetDay = $('btnPresetDay');
    if (btnPresetDay) btnPresetDay.addEventListener('click', () => applyPreset('daylight', 'cam2', false));
    const btnPresetDark2 = $('btnPresetDark');
    if (btnPresetDark2 && btnPresetDark2 !== $('btnPresetDark')) btnPresetDark2.addEventListener('click', () => applyPreset('dark', 'cam2', false));
    // There are two btnPresetDark ids (QR boost card and presets card) — handle both
    document.querySelectorAll('#btnPresetDark').forEach(el => el.addEventListener('click', () => applyPreset('dark', 'cam2', false)));
    const btnPresetNight = $('btnPresetNight');
    if (btnPresetNight) btnPresetNight.addEventListener('click', () => applyPreset('night', 'cam2', false));
    const btnPresetDarkQr2 = $('btnPresetDarkQr2');
    if (btnPresetDarkQr2) btnPresetDarkQr2.addEventListener('click', () => applyPreset('dark_qr_boost', 'cam2', false));
    const btnPresetGzDark = $('btnPresetGzDark');
    if (btnPresetGzDark) btnPresetGzDark.addEventListener('click', () => applyPreset('gazebo_dark', 'cam2', false));

    const btnFakeApply = $('btnFakeApply');
    if (btnFakeApply) btnFakeApply.addEventListener('click', () => {
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (!piBase) { log('ERROR', 'no Pi URL'); return; }
      const body = {
        enabled: $('fakeEnabled') ? $('fakeEnabled').checked : true,
        hide_fake: $('fakeHide') ? $('fakeHide').checked : true,
        require_decode: $('fakeRequire') ? $('fakeRequire').checked : true,
        conf_thr: parseFloat($('fakeConf') ? $('fakeConf').value : '0.35'),
        min_size: parseInt($('fakeMinSize') ? $('fakeMinSize').value : '15', 10),
      };
      fetch(piBase + '/api/detector/fake_filter', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      }).then(r => r.ok ? r.json() : r.text().then(t=>{throw new Error(t)})).then(res => {
        log('WARN', 'Fake filter applied: ' + JSON.stringify(res.fake_filter || res) + ' — grill 0.16/0.24 ' + (body.hide_fake ? 'hidden' : 'shown') + (body.require_decode ? ' ONLY decoded' : ''));
        if (fakeFilterInfo) fakeFilterInfo.textContent = JSON.stringify(res.fake_filter || body) + ' — grill hidden, zero fakes for ground';
      }).catch(e => log('ERROR', 'fake filter failed: ' + e.message));
    });

    const btnFakeOff = $('btnFakeOff');
    if (btnFakeOff) btnFakeOff.addEventListener('click', () => {
      const piBase = (S.piUrl || '').replace(/\/$/, '');
      if (!piBase) return;
      const body = { enabled: false, hide_fake: false, require_decode: false, conf_thr: 0.10, min_size: 5 };
      fetch(piBase + '/api/detector/fake_filter', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      }).then(r => r.json()).then(res => {
        log('INFO', 'Fake filter OFF — showing all raw boxes (debug): ' + JSON.stringify(res.fake_filter));
        if (fakeFilterInfo) fakeFilterInfo.textContent = 'OFF — showing all raw (debug) ' + JSON.stringify(res.fake_filter);
        if ($('fakeEnabled')) $('fakeEnabled').checked = false;
        if ($('fakeHide')) $('fakeHide').checked = false;
        if ($('fakeRequire')) $('fakeRequire').checked = false;
      });
    });

    // Load presets after Pi probe
    setTimeout(loadPresets, 2000);
    // Also reload on Pi connect
    const origOnStatus = pi ? pi.onStatus : null;
    // Hook via polling probe result already calls probeCameras — we also load presets there
    // For simplicity, reload presets every time cameras probed
    const _origProbe = probeCameras;
    probeCameras = function() {
      const p = _origProbe.apply(this, arguments);
      if (p && p.then) p.then(() => loadPresets());
      else loadPresets();
      return p;
    };

    // ---- legacy fence tools (still supported) ----
    const btnDraw = $('btnDraw');
    if (btnDraw) btnDraw.addEventListener('click', () => {
      const on = map.setDrawing(btnDraw.classList.toggle('on'));
      if (!on) btnDraw.classList.remove('on');
    });
    const btnUndo = $('btnUndo');
    if (btnUndo) btnUndo.addEventListener('click', () => { map.undoVertex(); });
    const btnClearDraw = $('btnClearDraw');
    if (btnClearDraw) btnClearDraw.addEventListener('click', () => { map.clearVertices(); });
    const btnReanchor = $('btnReanchor');
    if (btnReanchor) btnReanchor.addEventListener('click', () => { map.reanchorGrid(); });
  }

  // ---- fence tools (wired early, but also in wirePolygonButtons) ----
  const btnFenceApply = $('btnFenceApply');
  if (btnFenceApply) btnFenceApply.addEventListener('click', () => {
    if (!map) return;
    // prefer polygon if available
    let verts = map.getPolygonVertices ? map.getPolygonVertices() : [];
    if (verts.length < 3) verts = map.getVertices();
    if (verts.length < 3) { log('ERROR', 'fence needs ≥3 vertices (polygon or fence drawing, have ' + verts.length + ')'); return; }
    if (!window.confirm('Apply ' + verts.length + '-vertex fence via Pi to FC? MP will see instantly.')) return;
    // UI now drives fence via Pi — Pi owns fence, MP sees instantly because FC broadcasts
    // Keep MP link as fallback, but Pi is primary (MP not involved much)
    const piBase = (S.piUrl || '').replace(/\/$/, '');
    if (piBase) {
      postJson(piBase + '/api/fence', { vertices: verts })
        .then((st) => {
          log('WARN', 'Pi fence upload: ' + st.count + ' verts, FC uploaded=' + st.fc_uploaded + (st.fc_error ? ' err:'+st.fc_error : '') + ' — MP Fence tab should show instantly');
          // Also push to bridge for MP instant viz
          return postJson('/api/mp/fence', { vertices: verts, mission_type: 'fence', fence_action: null }).catch(()=>st);
        })
        .then((st) => { renderFence(st); })
        .catch((e) => log('ERROR', 'Pi fence upload failed: ' + e.message));
    } else {
      postJson('/api/mp/fence', { vertices: verts, mission_type: 'fence', fence_action: null })
        .then((st) => { renderFence(st); log('WARN', 'fence upload done via bridge: ' + st.vertex_count + ' verts, ' + (st.confirmed ? 'readback MATCH' : 'READBACK MISMATCH')); })
        .catch((e) => log('ERROR', 'fence upload failed: ' + e.message));
    }
  });

  const btnFenceClear = $('btnFenceClear');
  if (btnFenceClear) btnFenceClear.addEventListener('click', () => {
    if (!window.confirm('Clear the fence on the FC via Pi?')) return;
    const piBase = (S.piUrl || '').replace(/\/$/, '');
    const p1 = piBase ? postJson(piBase + '/api/fence', { clear: true }).catch(()=>{}) : Promise.resolve();
    const p2 = fetch('/api/mp/fence', { method: 'DELETE' }).then((r) => {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    }).catch(()=>{});
    Promise.all([p1,p2]).then(() => {
      renderFence({ loaded: false, vertex_count: 0, vertices_latlon: [] });
      log('WARN', 'fence cleared on FC via Pi+bridge — MP Fence tab should clear instantly');
    }).catch((e) => log('ERROR', 'fence clear failed: ' + e.message));
  });

  const btnFenceRefresh = $('btnFenceRefresh');
  if (btnFenceRefresh) btnFenceRefresh.addEventListener('click', () => {
    const piBase = (S.piUrl || '').replace(/\/$/, '');
    if (piBase) {
      fetch(piBase + '/api/fence').then((r) => r.ok ? r.json() : Promise.reject()).then((d) => {
        if (d.fence && d.fence.length) {
          log('INFO', 'Pi fence: ' + d.count + ' verts, FC: ' + d.fc_count + ' verts');
          if (map) map.setFence(d.fence.map((v) => [v[0], v[1]]));
        }
      }).catch(()=>{});
    }
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

  const btnExportLap = $('btnExportLap');
  if (btnExportLap) btnExportLap.addEventListener('click', () => {
    const verts = (S.fence && S.fence.vertices_latlon) || (map && map.getVertices()) || (map && map.getPolygonVertices()) || [];
    if (!verts.length) { log('ERROR', 'nothing to export (no fence, no drawing, no polygon)'); return; }
    const txt = verts.map((v) => v.lat.toFixed(7) + ',' + v.lon.toFixed(7)).join('\n') + '\n';
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' }));
    a.download = 'fence.lap'; a.click();
    log('INFO', 'exported ' + verts.length + ' verts to fence.lap');
  });

  // ---- MP link tools ----
  const btnMavPort = $('btnMavPort');
  if (btnMavPort) btnMavPort.addEventListener('click', () => {
    const port = parseInt($('mavPort').value, 10);
    if (!port || port < 1 || port > 65535) { log('ERROR', 'bad UDP port'); return; }
    postJson('/api/mp/mavlink', { port }).then((d) => log('INFO', 'MAVLink listener → port ' + d.port))
      .catch((e) => log('ERROR', 'rebind failed: ' + e.message));
  });

  const btnParam = $('btnParam');
  if (btnParam) btnParam.addEventListener('click', () => {
    const name = $('paramName').value.trim().toUpperCase();
    if (!name) return;
    const pv = $('paramVal'); if (pv) pv.textContent = '…';
    postJson('/api/mp/param', { name }).then((d) => {
      if (pv) pv.textContent = d.name + ' = ' + d.value;
      log('INFO', 'param ' + d.name + ' = ' + d.value);
    }).catch((e) => { if (pv) pv.textContent = 'error'; log('ERROR', 'param read failed: ' + e.message); });
  });

  // ---- actions ----
  const btnAbort = $('btnAbort');
  if (btnAbort) btnAbort.addEventListener('click', () => {
    if (!window.confirm('ABORT → send RTL to the vehicle via MP?')) return;
    postJson('/api/mp/mode', { mode: 'RTL' })
      .then((d) => log('WARN', 'RTL commanded (' + d.status + (d.result_name ? ', ' + d.result_name + ' [' + d.result + ']' : '') + ')'))
      .catch((e) => log('ERROR', 'RTL failed: ' + e.message));
  });

  const btnLand = $('btnLand');
  if (btnLand) btnLand.addEventListener('click', () => {
    if (!window.confirm('Send LAND to the vehicle via MP?')) return;
    postJson('/api/mp/mode', { mode: 'LAND' })
      .then((d) => log('WARN', 'LAND commanded (' + d.status + (d.result_name ? ', ' + d.result_name + ' [' + d.result + ']' : '') + ')'))
      .catch((e) => log('ERROR', 'LAND failed: ' + e.message));
  });

  const btnMockRestart = $('btnMockRestart');
  if (btnMockRestart) btnMockRestart.addEventListener('click', () => {
    postJson('/api/mp/mock/restart', {}).then(() => log('INFO', 'mock scenario restarted'))
      .catch((e) => log('ERROR', 'mock restart failed: ' + e.message));
  });

  // ---- QR manual + MP log watch ----
  const btnQrManual = $('btnQrManual');
  if (btnQrManual) btnQrManual.addEventListener('click', () => {
    const payload = $('qrManual').value.trim();
    if (!payload) return;
    postJson('/api/mp/qr', { payload }).then((d) => {
      const p = (d.qr && d.qr.payload) || d.payload || '';
      S.qr.payload = p;
      if ($('qrPayload')) $('qrPayload').textContent = p;
      S.receipts.manual = nowStr();
      if ($('qrSrcMan')) $('qrSrcMan').textContent = S.receipts.manual;
      log('WARN', 'manual QR latched: ' + p);
    }).catch((e) => log('ERROR', 'manual QR failed: ' + e.message));
  });

  const btnWatch = $('btnWatch');
  if (btnWatch) btnWatch.addEventListener('click', () => {
    const path = $('watchPath').value.trim();
    if (!path) return;
    postJson('/api/mp/watch', { path }).then((d) => {
      if ($('watchInfo')) $('watchInfo').textContent = ([].concat(d.watching || []).join('; ')) || '—';
      log('INFO', 'watching MP log: ' + d.watching);
    }).catch((e) => log('ERROR', 'watch failed: ' + e.message));
  });

  // ---- links ----
  let probeCamsAt = 0;
  function renderPiProbe(res) {
    const el = $('piHealth');
    const hint = $('piHint');
    if (!el) return;
    if (!res) { el.textContent = '—'; return; }
    if (res.ok) {
      const h = res.health || {};
      const bu = h.bringup || {};
      const bad = Object.keys(bu.state || {}).filter((k) => bu.state[k] === 'failed');
      el.textContent = 'up ' + res.ms + ' ms · link ' + (h.link ? 'OK' : 'down')
        + ' · cams ' + (Object.keys(h.cams || {}).join(',') || 'none')
        + (bu.stage ? (' · ' + bu.stage) : '')
        + (S.pi.connected ? '' : ' · WS REFUSED');
      el.style.color = S.pi.connected ? '#5fd97a' : '#ffb020';
      const last = (bu.errors || []).slice(-1)[0];
      if (hint) hint.textContent = bad.length
        ? ('Pi up but bring-up failed: ' + bad.join(', ') + (last ? ' — ' + last.error : ''))
        : (S.pi.connected ? 'Pi link healthy.'
          : ('Pi answers HTTP but the websocket is refused — on the Pi run: '
           + 'pip install "uvicorn[standard]" and restart mission_pi. '
           + 'Polling /api/fsm/status meanwhile.'));
      if (!S.pi.connected && Date.now() - probeCamsAt > 5000) {
        probeCamsAt = Date.now();
        probeCameras().then(() => { startAvailableCams(); seedCamControls(); });
      }
      // show max alt from Pi health
      if (h.mission && h.mission.max_alt_m && $('piMaxAlt')) $('piMaxAlt').textContent = h.mission.max_alt_m + ' m';
      else if (h.fov && h.fov.max_alt_m && $('piMaxAlt')) $('piMaxAlt').textContent = h.fov.max_alt_m + ' m';
    } else {
      el.textContent = 'unreachable (' + res.error + ')';
      el.style.color = '#ff6b6b';
      if (hint) hint.textContent = 'Cannot reach ' + S.piUrl + ' — is mission_pi running '
        + 'there (python3 main.py --config config.laptop.yaml)? Right IP '
        + '(hostname -I on the Pi box)? Firewall open (sudo ufw allow 8000/tcp)? '
        + 'Diagnose: tools/link_doctor.py on the Pi box, or '
        + 'tools/link_doctor.py --target ' + S.piUrl + ' from here.';
    }
  }

  pi = window.MissionLinks.initPiLink({
    onStatus: (s) => {
      const was = S.pi.connected;
      S.pi.connected = s.connected;
      const piWsEl = $('piWs');
      if (piWsEl) piWsEl.textContent = s.connected
        ? ('open · ' + s.url)
        : ('down · retry ' + s.retry + (s.polling ? ' · polling HTTP' : ''));
      setLamp('lampPi', s.connected);
      const btnPiConn = $('btnPiConnect');
      if (btnPiConn) btnPiConn.textContent = s.connected ? 'disconnect' : 'connect';
      if (s.connected && !was) {
        probeCamsAt = Date.now();
        probeCameras().then(() => { startAvailableCams(); seedCamControls(); });
        const piHint = $('piHint'); if (piHint) piHint.textContent = 'Pi link healthy.';
        const piHealth = $('piHealth'); if (piHealth) piHealth.style.color = '#5fd97a';
      }
    },
    onProbe: renderPiProbe,
    onEnvelope,
    onLog: log,
  });

  const btnPiProbe = $('btnPiProbe');
  if (btnPiProbe) btnPiProbe.addEventListener('click', () => {
    let v = $('piUrl').value.trim().replace(/\/$/, '');
    if (v && !/^[a-zA-Z][a-zA-Z0-9+.-]*:\/\//.test(v)) v = 'http://' + v;
    if (v && v !== S.piUrl) { S.piUrl = v; $('piUrl').value = v; }
    const piHealth = $('piHealth'); if (piHealth) piHealth.textContent = 'probing…';
    if (!S.piUrl) { log('ERROR', 'no Pi URL to probe'); return; }
    pi.probe().then((res) => {
      log(res && res.ok ? 'INFO' : 'ERROR',
        'pi probe ' + S.piUrl + ': ' + (res && res.ok
          ? ('HTTP up in ' + res.ms + ' ms' + (res.isPi ? '' : ' (NOT mission_pi!)'))
          : ('unreachable — ' + (res && res.error))));
    });
  });

  const btnPiConnect = $('btnPiConnect');
  if (btnPiConnect) btnPiConnect.addEventListener('click', () => {
    let v = $('piUrl').value.trim().replace(/\/$/, '');
    if (v && !/^[a-zA-Z][a-zA-Z0-9+.-]*:\/\//.test(v)) v = 'http://' + v;
    v = v || window.location.origin;
    if (S.pi.connected && v === S.piUrl) {
      pi.close(true);
      S.pi.connected = false;
      const piWsEl = $('piWs'); if (piWsEl) piWsEl.textContent = 'closed by user';
      setLamp('lampPi', false);
      btnPiConnect.textContent = 'connect';
      return;
    }
    S.piUrl = v;
    if ($('piUrl')) $('piUrl').value = S.piUrl;
    S.camsProbed = false;
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
    onMode: (id, mode) => { const el = $(id === 'cam1' ? 'c1Mode' : 'c2Mode'); if (el) el.textContent = mode; },
    onLatency: (id, ms) => { const el = $(id === 'cam1' ? 'c1Lat' : 'c2Lat'); if (el) el.textContent = ms; },
    onStats: (id, t) => { const el = $(id === 'cam1' ? 'camStats1' : 'camStats2'); if (el) el.textContent = t; },
    onHint: (id, t) => { const el = $(id === 'cam1' ? 'camHint1' : 'camHint2'); if (el) el.textContent = t; },
    onLog: log,
  });

  const tabCam1 = $('tabCam1'), tabCam2 = $('tabCam2');
  if (tabCam1) tabCam1.addEventListener('click', () => {
    tabCam1.classList.add('on'); const t2 = $('tabCam2'); if (t2) t2.classList.remove('on');
    const c1 = $('camCtls1'); if (c1) c1.classList.remove('hidden');
    const c2 = $('camCtls2'); if (c2) c2.classList.add('hidden');
    S.camActive = 'cam1';
  });
  if (tabCam2) tabCam2.addEventListener('click', () => {
    tabCam2.classList.add('on'); if (tabCam1) tabCam1.classList.remove('on');
    const c2 = $('camCtls2'); if (c2) c2.classList.remove('hidden');
    const c1 = $('camCtls1'); if (c1) c1.classList.add('hidden');
    S.camActive = 'cam2';
  });
  // default active cam
  S.camActive = 'cam2';

  CAMS.forEach((c) => {
    ['Exp', 'Gain', 'Bri', 'Con', 'Sat', 'Sha'].forEach((name) => {
      const el = $(c.p + name);
      if (el) el.addEventListener('input', (e) => {
        const out = $(c.p + name + 'V'); if (out) out.textContent = e.target.value;
        scheduleCamPush(c.id, c.p);
      });
    });
    const af = $(c.p + 'Af'); if (af) af.addEventListener('change', () => scheduleCamPush(c.id, c.p));
    const ad = $(c.p + 'Adapt'); if (ad) ad.addEventListener('change', () => scheduleCamPush(c.id, c.p));
  });

  const btnModalCam1 = $('btnModalCam1'), btnModalCam2 = $('btnModalCam2');
  if (btnModalCam1) btnModalCam1.addEventListener('click', () => cams.openModal('cam1'));
  if (btnModalCam2) btnModalCam2.addEventListener('click', () => cams.openModal('cam2'));
  const btnModalClose = $('btnModalClose');
  if (btnModalClose) btnModalClose.addEventListener('click', () => cams.closeModal());
  const btnPauseCam1 = $('btnPauseCam1'), btnPauseCam2 = $('btnPauseCam2');
  if (btnPauseCam1) btnPauseCam1.addEventListener('click', (e) => {
    e.target.textContent = cams.togglePause('cam1') ? 'resume' : 'pause';
  });
  if (btnPauseCam2) btnPauseCam2.addEventListener('click', (e) => {
    e.target.textContent = cams.togglePause('cam2') ? 'resume' : 'pause';
  });

  // ---- map toolbar ----
  const selTile = $('selTile');
  if (selTile) selTile.addEventListener('change', (e) => { map && map.setTileSourceByName(e.target.value); });
  const mapHintEl = $('mapHint');
  if (mapHintEl) mapHintEl.addEventListener('click', () => { map && map.zoomToGrid(); });
  const btnFollow = $('btnFollow');
  if (btnFollow) btnFollow.addEventListener('click', () => {
    if (!map) return;
    const on = map.setFollow(btnFollow.classList.toggle('on'));
    if (!on) btnFollow.classList.remove('on');
  });

  // ---- mock auto-connect ----
  S.piUrl = window.location.origin;
  const piUrlEl = $('piUrl'); if (piUrlEl) piUrlEl.value = S.piUrl;
  fetch('/api/mp/state').then((r) => r.json()).then((st) => {
    const piWsEl = $('piWs');
    if (st && st.mock) pi.connect(S.piUrl);
    else if (piWsEl) piWsEl.textContent = 'not connected — paste Pi URL';
    log('INFO', 'mission-ui v2 + arena/test1 boot: bridge SSE on ' + S.piUrl +
        (st && st.mock ? ' (+ mock Pi WS)' : ' (real mode: Pi link manual)') +
        ' · polygon→fence, config.yaml max height, FOV coverage');
  }).catch(() => { pi.connect(S.piUrl); });
}

document.addEventListener('DOMContentLoaded', boot);
})();
