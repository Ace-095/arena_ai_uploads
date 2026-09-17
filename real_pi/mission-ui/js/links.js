/* links.js — UI v2 transports.
 *
 * Two links, both receive-heavy:
 *  1) Pi WS  (ws[s]://<pi>/ws/telemetry): mission/fsm/system/qr/event/log
 *     envelopes {channel, t, data}. Mission-ui NEVER sends flight data here.
 *     When the socket will not open, this module now PROBES the Pi over HTTP
 *     (/health) and says WHY (nothing listening / not mission_pi / WS route
 *     missing), then falls back to polling /api/fsm/status + /api/qr/status
 *     so the mission panels keep working with a broken socket.
 *  2) Bridge SSE (/api/mp/stream, same origin as this page): every hub
 *     channel — bridge-owned (mavlink-state, fence, plan, telemetry,
 *     mp-line, mp-qr, mp-console, bridge-state, log, keepalive) plus Pi
 *     channels relayed in mock mode (system, event, qr, fsm, log).
 */
(function () {
'use strict';

function initPiLink(o) {
  o = o || {};
  const st = { ws: null, retry: 0, url: '', opened: false,
               timer: null, openedAt: 0, lastMsgAt: 0,
               probe: null, poll: null, lastReason: '' };

  function status() {
    return {
      connected: !!st.ws && st.ws.readyState === 1,
      url: st.url,
      retry: st.retry,
      uptime_s: st.opened ? Math.round((Date.now() - st.openedAt) / 1000) : 0,
      last_msg_age_s: st.opened && st.lastMsgAt ? Math.round((Date.now() - st.lastMsgAt) / 1000) : null,
      probe: st.probe,
      polling: !!st.poll,
      reason: st.lastReason,
    };
  }

  function wsUrl(httpUrl) {
    const u = httpUrl.replace(/\/$/, '');
    return (u.startsWith('https') ? u.replace(/^https/, 'wss') : u.replace(/^http/, 'ws')) + '/ws/telemetry';
  }

  // ---- HTTP probe: turns "PI down · retry 3" into an actionable reason ----
  function probe() {
    if (!st.url) return Promise.resolve(null);
    const ctl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
    const to = ctl ? setTimeout(() => { try { ctl.abort(); } catch (e) { /* noop */ } }, 4000) : null;
    const opt = { cache: 'no-store' };
    if (ctl) opt.signal = ctl.signal;
    const t0 = Date.now();
    return fetch(st.url.replace(/\/$/, '') + '/health', opt)
      .then((r) => {
        if (to) clearTimeout(to);
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json().then((h) => ({
          ok: true, ms: Date.now() - t0, health: h, isPi: !!(h && (h.cams || h.bringup || h.ws || h.link != null)),
        }));
      })
      .catch((e) => {
        if (to) clearTimeout(to);
        return { ok: false, ms: Date.now() - t0, error: (e && e.message) || String(e) };
      })
      .then((res) => {
        st.probe = res;
        st.lastReason = explain(res);
        o.onProbe && o.onProbe(res);
        o.onStatus && o.onStatus(status());
        return res;
      });
  }

  function explain(res) {
    if (!res) return '';
    if (res.ok && res.isPi) {
      const bu = res.health.bringup || {};
      const bad = Object.keys(bu.state || {}).filter((k) => bu.state[k] === 'failed');
      if (bad.length) return 'Pi HTTP up (' + res.ms + ' ms) but bring-up failed: ' + bad.join(', ');
      return 'Pi HTTP up (' + res.ms + ' ms) — websocket route refused/blocked';
    }
    if (res.ok && !res.isPi) {
      return 'something answers ' + st.url + ' but it is not mission_pi (/health unknown) — wrong port?';
    }
    return 'cannot reach ' + st.url + ' (' + res.error + ') — mission_pi not running, wrong IP, or firewall on :8000';
  }

  // ---- HTTP fallback: keep the mission panels alive without the socket ----
  function startPolling() {
    if (st.poll || !st.url) return;
    const base = st.url.replace(/\/$/, '');
    const tick = () => {
      if (!st.url || (st.ws && st.ws.readyState === 1)) return;
      fetch(base + '/api/fsm/status', { cache: 'no-store' })
        .then((r) => (r.ok ? r.json() : null))
        .then((d) => {
          if (!d) return;
          st.lastMsgAt = Date.now();
          o.onEnvelope && o.onEnvelope({ channel: 'fsm', t: Date.now() / 1000, data: d, via: 'poll' });
        })
        .catch(() => { /* probe() reports the reason */ });
      fetch(base + '/api/qr/status', { cache: 'no-store' })
        .then((r) => (r.ok ? r.json() : null))
        .then((d) => {
          if (!d || !d.payload) return;
          o.onEnvelope && o.onEnvelope({ channel: 'qr', t: Date.now() / 1000, data: d, via: 'poll' });
        })
        .catch(() => { /* noop */ });
    };
    tick();
    st.poll = setInterval(tick, 2000);
    o.onLog && o.onLog('WARN', 'pi ws down → polling ' + base + '/api/fsm/status every 2 s');
  }

  function stopPolling() {
    if (st.poll) { clearInterval(st.poll); st.poll = null; }
  }

  function schedule() {
    if (st.timer) return;
    st.retry += 1;
    const wait = Math.min(20000, 1000 * Math.pow(2, Math.min(st.retry, 5)));
    o.onStatus && o.onStatus(status());
    st.timer = setTimeout(() => { st.timer = null; if (st.url) connect(st.url); }, wait);
  }

  function connect(httpUrl) {
    st.url = httpUrl;
    close(false);
    let sock;
    try { sock = new WebSocket(wsUrl(httpUrl)); }
    catch (e) {
      o.onLog && o.onLog('ERROR', 'bad Pi URL ' + httpUrl + ': ' + e.message);
      probe().then(startPolling);
      schedule();
      return;
    }
    st.ws = sock;
    sock.onopen = () => {
      st.retry = 0; st.opened = true; st.openedAt = Date.now(); st.lastMsgAt = 0;
      st.lastReason = '';
      stopPolling();
      o.onStatus && o.onStatus(status());
      o.onLog && o.onLog('INFO', 'pi ws open: ' + wsUrl(httpUrl));
    };
    sock.onmessage = (ev) => {
      st.lastMsgAt = Date.now();
      let env;
      try { env = JSON.parse(ev.data); } catch (e) { return; }
      o.onEnvelope && o.onEnvelope(env);
    };
    sock.onerror = () => { try { sock.close(); } catch (e) { /* noop */ } };
    sock.onclose = (ev) => {
      if (st.ws !== sock) return;
      st.ws = null; st.opened = false;
      o.onStatus && o.onStatus(status());
      const why = (ev && ev.code && ev.code !== 1000) ? ' (code ' + ev.code + ')' : '';
      o.onLog && o.onLog('WARN', 'pi ws closed' + why + ' (retry ' + (st.retry + 1) + ')');
      if (st.url) {
        // Diagnose + degrade: an HTTP probe says whether the box is even
        // there, and polling keeps the panels live while the socket is not.
        probe().then((res) => {
          if (res && res.ok) o.onLog && o.onLog('WARN', st.lastReason);
          else o.onLog && o.onLog('ERROR', st.lastReason);
          startPolling();
        });
        schedule();
      }
    };
  }

  function close(clearUrl) {
    if (st.timer) { clearTimeout(st.timer); st.timer = null; }
    if (st.ws) { const s = st.ws; st.ws = null; try { s.close(); } catch (e) { /* noop */ } }
    stopPolling();
    st.opened = false;
    if (clearUrl) { st.url = ''; st.probe = null; st.lastReason = ''; }
  }

  return { connect, close, status, probe };
}

function initBridge(o) {
  const SSE_MAP = {
    'mp-line': 'onMpLine', 'mp-qr': 'onMpQr', 'mp-console': 'onMpConsole',
    'bridge-state': 'onBridgeState', 'mavlink-state': 'onMavState',
    'fence': 'onFence', 'plan': 'onPlan', 'telemetry': 'onTelemetry',
    // Pi channels relayed over the same SSE in mock mode:
    'system': 'onPiSse', 'event': 'onPiSse', 'qr': 'onPiSse',
    'fsm': 'onPiSse', 'log': 'onPiSse',
  };
  let es = null;

  function connect(base) {
    disconnect();
    const url = (base || '').replace(/\/$/, '') + '/api/mp/stream';
    try { es = new EventSource(url); } catch (e) { return; }
    es.addEventListener('keepalive', () => { o.onBridgeStatus && o.onBridgeStatus(true); });
    Object.keys(SSE_MAP).forEach((ev) => {
      es.addEventListener(ev, (m) => {
        let d;
        try { d = JSON.parse(m.data); } catch (e) { return; }
        o.onBridgeStatus && o.onBridgeStatus(true);
        const fn = o[SSE_MAP[ev]];
        if (fn) fn(d, ev);
      });
    });
    es.onerror = () => { o.onBridgeStatus && o.onBridgeStatus(false); };
  }

  function disconnect() { if (es) { try { es.close(); } catch (e) { /* noop */ } es = null; } }

  return { connect, disconnect };
}

window.MissionLinks = { initPiLink, initBridge };
})();
