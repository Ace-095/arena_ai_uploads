// app.js — Mission Companion: state, wiring, panels
import { initMap } from './map.js';
import { initPiLink, initBridge } from './links.js';
import { initCamera } from './camera.js';
import { latlonToEnu, polyAreaM2, centroid, maxRadiusFrom } from './geo.js';

const $ = (id) => document.getElementById(id);
const escapeHtml = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const nowStr = () => new Date().toTimeString().slice(0, 8);
const fmt = (v, d = 2, suf = '') => (v == null || isNaN(v)) ? '—' : Number(v).toFixed(d) + suf;

const FSM_ORDER = ['IDLE', 'PLAN_SYNC', 'WAITING_TRIGGER', 'SEARCH', 'TARGET_FOUND', 'DESCEND',
  'DECODED', 'TRANSMIT', 'RTL', 'LAND', 'COMPLETE', 'FAILSAFE'];

// ---------------- state ----------------
const S = {
  piUrl: localStorage.getItem('mc_pi_url') || '',
  isMock: false,
  fence: null,
  qr: { payload: null, streak: 0, required: 3, confirmed: false },
  receipts: { pi: null, mp: null, man: null },
  logs: [],
};

// ---------------- logging ----------------
function renderLogLine(ts, level, src, msg, box = $('logBox')) {
  const filter = $('logFilter').value.toLowerCase();
  if (filter && !(String(msg).toLowerCase().includes(filter) || level.toLowerCase().includes(filter) || src.toLowerCase().includes(filter))) return;
  const d = document.createElement('div');
  d.className = 'ln';
  d.innerHTML = `<span class="ts">${ts}</span><span class="${level === 'MP' ? 'MP' : level}">[${src}]</span> ${escapeHtml(msg)}`;
  box.appendChild(d);
  while (box.children.length > 500) box.removeChild(box.firstChild);
  if (box.scrollTop > box.scrollHeight - box.clientHeight - 30) box.scrollTop = box.scrollHeight;
}

function log(level, src, msg) {
  const ts = nowStr();
  S.logs.push({ ts, level, src, msg });
  if (S.logs.length > 800) S.logs.shift();
  renderLogLine(ts, level, src, msg);
}

function renderAllLogs() {
  const box = $('logBox');
  box.innerHTML = '';
  S.logs.forEach((l) => renderLogLine(l.ts, l.level, l.src, l.msg));
  box.scrollTop = box.scrollHeight;
}

// ---------------- API helpers ----------------
async function api(method, path, body) {
  const r = await fetch(S.piUrl + path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  let j = null;
  try { j = await r.json(); } catch { /* no body */ }
  if (!r.ok) throw new Error((j && j.detail) || `HTTP ${r.status}`);
  return j;
}
const apiGet = (p) => api('GET', p);
const apiPost = (p, b) => api('POST', p, b || {});
const apiDel = (p) => api('DELETE', p);

// ---------------- panel renderers ----------------
function setLamp(id, on, warn = false) {
  const el = $(id);
  el.className = 'lamp ' + (on ? (warn ? 'warn' : 'on') : 'off');
}

function renderTelemetry(t) {
  if (!t) return;
  $('tLat').textContent = fmt(t.lat, 7);
  $('tLon').textContent = fmt(t.lon, 7);
  $('tAlt').textContent = `${fmt(t.alt_rel, 1, ' m')} / ${fmt(t.alt_msl, 1, ' m')}`;
  $('tMode').textContent = `${t.mode || '—'} / ${t.armed ? 'ARMED' : 'disarmed'}`;
  $('tVel').textContent = t.vx != null ? `${fmt(t.vx, 1)}, ${fmt(t.vy, 1)}, ${fmt(t.vz, 1)}` : '—';
  if (t.attitude) $('tAtt').textContent = `${fmt(t.attitude.roll, 1)}°, ${fmt(t.attitude.pitch, 1)}°, ${fmt(t.attitude.yaw, 1)}°`;
  if (t.gps) $('tGps').textContent = `fix ${t.gps.fix ?? '—'} · ${t.gps.sats ?? '—'} sats`;
  if (t.battery) $('tBat').textContent = `${fmt(t.battery.v, 2, ' V')} ${t.battery.pct != null ? `· ${t.battery.pct}%` : ''}`;
}

function renderSystem(s) {
  if (!s) return;
  $('sysBadge').textContent = 'live';
  $('sysBadge').className = 'badge ok';
  $('sCpu').textContent = `${fmt(s.cpu, 0, '%')} / ${fmt(s.temp_c, 1, '°C')}`;
  $('sRam').textContent = s.ram != null ? `${fmt(s.ram, 0, '%')} (${(s.ram_used_mb / 1024).toFixed(1)}/${(s.ram_total_mb / 1024).toFixed(1)} GB)` : '—';
  $('sDisk').textContent = `${fmt(s.disk, 0, '%')} / ${fmt(s.freq_mhz, 0, ' MHz')}`;
  if (s.uptime_s != null) {
    const h = Math.floor(s.uptime_s / 3600), m = Math.floor((s.uptime_s % 3600) / 60);
    $('sUp').textContent = `${h}h ${m}m`;
  }
  if (s.network) {
    const r = s.network.router === 'up', l = s.network.lte === 'up';
    const el = $('sNet');
    el.textContent = `${r ? 'router ●' : 'router ○'}  ${l ? 'lte ●' : 'lte ○'}`;
    el.style.color = (r || l) ? 'var(--green)' : 'var(--text-dim)';
  }
}

function renderFsm(f) {
  if (!f) return;
  const st = $('fsmState');
  st.textContent = f.state || '—';
  st.className = 'fsm-state ' + ({
    FAILSAFE: 'bad', COMPLETE: 'good',
    DECODED: 'good', TRANSMIT: 'good',
  }[f.state] || (['IDLE', 'PLAN_SYNC', 'WAITING_TRIGGER'].includes(f.state) ? '' : 'run'));
  $('fsmReason').textContent = f.reason || (f.state || 'waiting for Pi…');
  $('fsmBadge').textContent = f.state || '—';
  $('fsmBadge').className = 'badge ' + (f.state === 'FAILSAFE' ? 'bad' : (f.state === 'COMPLETE' ? 'ok' : 'cyan'));
  $('armableInfo').textContent = f.armable ? 'YES' : 'no';
  $('planInfo').textContent = f.plan && f.plan.synced
    ? `${f.plan.items} items · trigger #${f.plan.trigger_seq} ✓` : 'not synced';
  const chips = $('fsmChips');
  chips.innerHTML = '';
  FSM_ORDER.forEach((s) => {
    const c = document.createElement('span');
    c.textContent = s;
    if (s === f.state) c.className = (s === 'FAILSAFE') ? 'fail' : 'cur';
    else if (s === f.previous) c.className = 'pv';
    chips.appendChild(c);
  });
}

function renderQrQ(q) {
  if (!q) return;
  S.qr.streak = q.streak || 0;
  S.qr.required = q.required || 3;
  if (q.confirmed && q.payload) { S.qr.payload = q.payload; S.qr.confirmed = true; }
  renderQr();
}

function renderQr() {
  const dots = $('qrStreak').children;
  for (let i = 0; i < 3; i++) {
    dots[i].className = (S.qr.confirmed || i < S.qr.streak) ? (S.qr.confirmed ? 'max' : 'on') : '';
  }
  const box = $('qrPayload');
  if (S.qr.payload) {
    box.textContent = S.qr.payload;
    box.classList.add('ok');
    $('qrBadge').textContent = 'decoded';
    $('qrBadge').className = 'badge ok';
  } else {
    $('qrBadge').textContent = S.qr.streak ? `streak ${S.qr.streak}/3` : 'awaiting';
    $('qrBadge').className = 'badge ' + (S.qr.streak ? 'warn' : 'idle');
  }
  $('qrSrcPi').textContent = S.receipts.pi ? `✓ ${S.qr.payload || ''} @ ${S.receipts.pi}` : '—';
  $('qrSrcMp').textContent = S.receipts.mp ? `✓ ${S.qr.payload || ''} @ ${S.receipts.mp}` : '—';
  $('qrSrcMan').textContent = S.receipts.man ? `✓ @ ${S.receipts.man}` : '—';
}

function renderFence(st) {
  if (!st) return;
  S.fence = st;
  const badge = $('fenceBadge');
  if (st.loaded && st.vertices_latlon && st.vertices_latlon.length) {
    badge.textContent = st.confirmed ? 'on FC ✓' : 'MISMATCH';
    badge.className = 'badge ' + (st.confirmed ? 'ok' : 'bad');
    $('fenceState').textContent = st.reason || 'loaded';
    $('fenceState').style.color = st.confirmed ? 'var(--green)' : 'var(--red)';
    $('fenceReadback').textContent = st.confirmed ? 'matched' : 'MISMATCH';
    $('fenceVerts').textContent = st.vertex_count;
    $('fenceArea').textContent = st.area_m2 != null ? `${Math.round(st.area_m2)} m²` : '—';
    $('fenceRadius').textContent = st.max_radius_m != null ? `${st.max_radius_m.toFixed(1)} m` : '—';
    $('fenceOrigin').textContent = st.origin_lat != null ? `${st.origin_lat.toFixed(6)}, ${st.origin_lon.toFixed(6)}` : '—';
  } else {
    badge.textContent = 'none';
    badge.className = 'badge idle';
    $('fenceState').textContent = st.reason || 'no fence loaded';
    $('fenceState').style.color = '';
    ['fenceReadback', 'fenceVerts', 'fenceArea', 'fenceRadius', 'fenceOrigin'].forEach((id) => $(id).textContent = '—');
  }
  map.setFence(st);
  if (st.origin_lat != null) map.setOrigin(st.origin_lat, st.origin_lon);
}

// ---------------- events ----------------
function onEvent(d) {
  const v = d.value || {};
  switch (d.type) {
    case 'fsm_state':
      log('INFO', 'PI', `FSM ${v.from} -> ${v.to}${v.reason ? ' | ' + v.reason : ''}`);
      break;
    case 'qr_decoded':
      S.receipts.pi = S.receipts.pi || nowStr();
      if (!S.qr.payload) { S.qr.payload = v.payload; S.qr.confirmed = true; renderQr(); }
      log('WARN', 'PI', `QR decoded via PI (WS): ${v.payload}`);
      break;
    case 'mission_result':
      S.receipts.pi = S.receipts.pi || nowStr();
      if (!S.qr.payload && v.payload) { S.qr.payload = v.payload; S.qr.confirmed = true; renderQr(); }
      log('WARN', 'PI', `mission result via PI: ${v.payload}`);
      break;
    case 'target_detected':
      map.setTarget(true);
      log('INFO', 'PI', `target detected (confidence ${v.confidence ?? '?'})`);
      break;
    case 'fence_updated':
      log('INFO', 'PI', 'fence state updated on FC');
      break;
    case 'plan_synced':
      log('INFO', 'PI', `plan synced: ${v.items} items, DO_SPRAYER trigger seq ${v.trigger_seq}`);
      break;
    case 'abort':
      log('WARN', 'PI', `abort from Pi (was ${v.from})`);
      break;
    default:
      log('INFO', 'PI', `event: ${d.type}`);
  }
}

function onEnvelope(env) {
  const d = env.data || {};
  switch (env.channel) {
    case 'telemetry': renderTelemetry(d); map.setDrone(d); break;
    case 'system': renderSystem(d); break;
    case 'fsm': renderFsm(d); break;
    case 'qr': renderQrQ(d); break;
    case 'fence': renderFence(d); break;
    case 'log': if (d.msg) log(d.level || 'INFO', 'PI', d.msg); break;
    case 'event': onEvent(d); break;
  }
}

// ---------------- map ----------------
const map = initMap((msg) => log('INFO', 'MAP', msg));

// ---------------- Pi link ----------------
let pi = null;
pi = initPiLink({
  getPiUrl: () => S.piUrl,
  onEnvelope,
  onStatus(s) {
    $('piWs').textContent = s;
    setLamp('lampPi', s === 'connected');
    if (s === 'connected') {
      log('INFO', 'UI', `Pi WS connected (${S.piUrl})`);
      $('mockBadge').hidden = !S.isMock;
    } else if (s !== 'disconnected') {
      /* keep log quiet for transient reconnects */
    }
  },
});

// ---------------- Bridge link ----------------
initBridge({
  onEnvelope,
  // SSE also mirrors the mock Pi's stream; use it only when the direct WS is down
  onPiSse(env) { if (!pi.connected) onEnvelope(env); },
  onMpLine(d) { if (d.line) log('MP', 'MP', String(d.line).slice(0, 200)); },
  onMpQr(d) {
    S.receipts.mp = nowStr();
    if (!S.qr.payload && d.payload) { S.qr.payload = d.payload; S.qr.confirmed = true; renderQr(); }
    log('WARN', 'MP', `QR via MP log bridge (source: ${d.source}): ${d.payload} — no router/LTE needed`);
  },
  onBridgeState(st) {
    S.isMock = !!st.mock;
    $('mockBadge').hidden = !st.mock;
    $('btnRestartMock').hidden = !st.mock;
    $('mpWatch').textContent = st.watching && st.watching.length ? st.watching.join(', ') : 'none';
    $('mpDfr').textContent = st.bin_parser_available ? 'available' : 'not installed (optional)';
    if (st.mock && !S.piUrl) {
      S.piUrl = location.origin;
      $('piUrl').value = S.piUrl;
      log('INFO', 'UI', `mock Pi detected at ${S.piUrl} — auto-connecting`);
      pi.connect();
      camera.start();
    }
  },
  onBridgeStatus(up) {
    setLamp('lampBridge', up);
    if (up) log('INFO', 'UI', 'MP log bridge connected (local)');
  },
});

// ---------------- camera ----------------
const camera = initCamera({ getPiUrl: () => S.piUrl });

// ---------------- wiring: map tile source ----------------
$('selTile').addEventListener('change', (e) => map.setTileSourceByName(e.target.value));

// ---------------- wiring: Pi link ----------------
function connectPi() {
  const v = $('piUrl').value.trim();
  if (v) { S.piUrl = v; localStorage.setItem('mc_pi_url', v); }
  $('piMode').textContent = S.piUrl ? (S.isMock ? 'mock (same host)' : 'live') : '—';
  if (!S.piUrl) { log('WARN', 'UI', 'No Pi URL set — enter the Pi address (e.g. http://192.168.1.50:8000)'); return; }
  pi.connect();
  camera.start();
}
$('btnPiConnect').addEventListener('click', connectPi);
$('piUrl').addEventListener('change', connectPi);

// ---------------- wiring: fence ----------------
let drawing = false;
$('btnDraw').addEventListener('click', () => {
  drawing = !drawing;
  map.setDrawing(drawing);
  $('btnDraw').textContent = drawing ? 'Stop drawing' : 'Start drawing';
  log('INFO', 'UI', drawing ? 'fence drawing ON — click the map to add points' : 'fence drawing off');
});
$('btnUndoV').addEventListener('click', () => map.undoVertex());
$('btnClearV').addEventListener('click', () => { map.clearVertices(); log('INFO', 'UI', 'drawing cleared'); });
$('selGrid').addEventListener('change', () => log('INFO', 'UI', `grid spacing: ${$('selGrid').value} m`));

function busy(btnId, on, label) {
  const b = $(btnId);
  b.disabled = on;
  b.dataset.label = b.dataset.label || b.textContent;
  b.textContent = on ? label : (b.dataset.label || label);
}

$('btnApplyFence').addEventListener('click', async () => {
  const verts = map.getVertices();
  if (verts.length < 3) { alert('Need at least 3 points to form a polygon fence.'); return; }
  if (!S.piUrl) { alert('Set the Pi address first (or run the bridge with --mock).'); return; }
  busy('btnApplyFence', true, 'Applying…');
  try {
    const st = await apiPost('/api/geofence', { vertices: verts });
    renderFence(st);
    log('WARN', 'UI', `FENCE APPLIED: ${st.vertex_count} verts, area ${Math.round(st.area_m2)} m², readback ${st.confirmed ? 'OK' : 'MISMATCH'} — search area = this polygon`);
  } catch (e) {
    log('ERROR', 'UI', 'fence apply failed: ' + e.message);
  }
  busy('btnApplyFence', false, 'Apply fence → Pi → FC');
});

$('btnClearFence').addEventListener('click', async () => {
  if (!S.piUrl) return;
  try {
    const st = await apiDel('/api/geofence');
    renderFence(st);
    log('INFO', 'UI', 'fence cleared on FC + locally');
  } catch (e) { log('ERROR', 'UI', 'fence clear failed: ' + e.message); }
});

$('btnExportLap').addEventListener('click', () => {
  const verts = (S.fence && S.fence.vertices_latlon && S.fence.vertices_latlon.length)
    ? S.fence.vertices_latlon : map.getVertices();
  if (!verts.length) { alert('No fence vertices to export.'); return; }
  const n = verts.length;
  const lines = verts.map((v) =>
    `WPL,0,5001,26,${n},0,0,0,${v.lat.toFixed(7)},${v.lon.toFixed(7)},0,0,0,0,0,0,0`);
  const txt = ['# Mission Companion fence export',
    '# cmd 5001 = NAV_FENCE_POLYGON_VERTEX_INCLUSION, frame 26 = fence, p1 = vertex count (every row)',
    '# NEED TEST: load via MP "Load Mission" and confirm the fence tab shows the polygon',
    ...lines, ''].join('\n');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' }));
  a.download = 'geofence.lap';
  a.click();
  URL.revokeObjectURL(a.href);
  log('INFO', 'UI', `exported geofence.lap (${n} vertices) — load in MP as fallback path`);
});

// ---------------- wiring: telemetry/QR/system panels receive via onEnvelope ----------------

// ---------------- wiring: QR ----------------
function showPayload(p, srcName) {
  if (!S.qr.payload && p) { S.qr.payload = p; S.qr.confirmed = true; }
  renderQr();
  log('WARN', srcName, `QR payload displayed: ${p}`);
}
$('btnManualQr').addEventListener('click', async () => {
  const p = $('manualQr').value.trim();
  if (!p) return;
  try {
    const r = await fetch('/api/mp/qr', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ payload: p }) });
    if (!r.ok) throw new Error('bridge rejected');
    S.receipts.man = nowStr();
    showPayload(p, 'MANUAL');
    $('manualQr').value = '';
  } catch (e) { log('ERROR', 'UI', 'manual QR entry failed: ' + e.message); }
});

// ---------------- wiring: camera controls ----------------
[['cExp', 'cExpV', v => v], ['cGain', 'cGainV', v => v], ['cBri', 'cBriV', v => Number(v).toFixed(2)],
 ['cCon', 'cConV', v => Number(v).toFixed(2)], ['cSat', 'cSatV', v => Number(v).toFixed(2)],
 ['cSha', 'cShaV', v => Number(v).toFixed(2)]].forEach(([id, vid, fmtFn]) => {
  $(id).addEventListener('input', () => { $(vid).textContent = fmtFn($(id).value); });
});
$('btnCamApply').addEventListener('click', async () => {
  if (!S.piUrl) { log('WARN', 'UI', 'camera settings: no Pi connected'); return; }
  const body = {
    exposure_us: parseInt($('cExp').value, 10),
    gain: parseFloat($('cGain').value),
    af_mode: $('cAf').value,
    brightness: parseFloat($('cBri').value),
    contrast: parseFloat($('cCon').value),
    saturation: parseFloat($('cSat').value),
    sharpness: parseFloat($('cSha').value),
    adaptive: $('cAdaptive').checked,
  };
  try {
    await apiPost('/api/camera/controls', body);
    log('INFO', 'UI', `camera settings applied: exp=${body.exposure_us}µs gain=${body.gain} af=${body.af_mode} adaptive=${body.adaptive}`);
    $('camCtlHint').textContent = 'applied ✓ (verify effect in feed — bench-test on Pi)';
  } catch (e) { log('ERROR', 'UI', 'camera settings failed: ' + e.message); }
});

// ---------------- wiring: MP bridge panel ----------------
$('btnWatch').addEventListener('click', async () => {
  const p = $('watchPath').value.trim();
  if (!p) return;
  try {
    const r = await fetch('/api/mp/watch', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ path: p }) });
    const j = await r.json();
    log('INFO', 'UI', `bridge now watching: ${j.watching.join(', ') || p}`);
    $('watchPath').value = '';
  } catch (e) { log('ERROR', 'UI', 'watch failed: ' + e.message); }
});

// ---------------- wiring: logs ----------------
$('logFilter').addEventListener('input', renderAllLogs);
$('btnLogClr').addEventListener('click', () => { S.logs = []; renderAllLogs(); });
$('btnLogDl').addEventListener('click', () => {
  const txt = S.logs.map((l) => `[${l.ts}] ${l.level} [${l.src}] ${l.msg}`).join('\n');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' }));
  a.download = 'mission-companion-log.txt';
  a.click();
  URL.revokeObjectURL(a.href);
});

// ---------------- wiring: controls ----------------
$('btnAbort').addEventListener('click', async () => {
  if (!S.piUrl) { log('WARN', 'UI', 'abort: no Pi connected'); return; }
  if (!confirm('Abort the autonomous mission? Pi will go FAILSAFE → RTL.')) return;
  try {
    const st = await apiPost('/api/fsm/abort');
    log('WARN', 'UI', `ABORT sent — Pi state: ${st.state}`);
  } catch (e) { log('ERROR', 'UI', 'abort failed: ' + e.message); }
});
$('btnRestartMock').addEventListener('click', async () => {
  try { await apiPost('/api/fsm/start'); log('INFO', 'UI', 'mock mission (re)started (debug)'); }
  catch (e) { log('ERROR', 'UI', 'mock restart failed: ' + e.message); }
});

// ---------------- wiring: camera enlarge ----------------
$('camView1').addEventListener('click', () => camera.openModal());
$('modalClose').addEventListener('click', () => camera.closeModal());
$('modalBg').addEventListener('click', () => camera.closeModal());
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') camera.closeModal(); });

// ---------------- boot ----------------
function tickClock() { $('clock').textContent = new Date().toLocaleTimeString(); }
setInterval(tickClock, 1000);
tickClock();

$('piUrl').value = S.piUrl;
log('INFO', 'UI', 'Mission Companion UI started — MP stays the mission head (arm/plan in MP)');
if (S.piUrl) {
  log('INFO', 'UI', `connecting Pi: ${S.piUrl}`);
  pi.connect();
  camera.start();
}
// if no Pi URL yet, the bridge's state (SSE, same origin) auto-connects us
// when it reports mock mode; otherwise wait for the user to set the Pi address.
