/* links.js — UI v2 transports.
 *
 * Two links, both receive-heavy:
 *  1) Pi WS  (ws[s]://<pi>/ws/telemetry): mission/fsm/system/qr/event/log
 *     envelopes {channel, t, data}. Mission-ui NEVER sends flight data here.
 *  2) Bridge SSE (/api/mp/stream, same origin as this page): every hub
 *     channel — bridge-owned (mavlink-state, fence, plan, telemetry,
 *     mp-line, mp-qr, mp-console, bridge-state, log, keepalive) plus Pi
 *     channels relayed in mock mode (system, event, qr, fsm, log).
 */
(function () {
'use strict';

function initPiLink(o) {
  const st = { ws: null, retry: 0, url: '', opened: false,
               timer: null, openedAt: 0, lastMsgAt: 0 };

  function status() {
    return {
      connected: !!st.ws && st.ws.readyState === 1,
      url: st.url,
      retry: st.retry,
      uptime_s: st.opened ? Math.round((Date.now() - st.openedAt) / 1000) : 0,
      last_msg_age_s: st.opened && st.lastMsgAt ? Math.round((Date.now() - st.lastMsgAt) / 1000) : null,
    };
  }

  function wsUrl(httpUrl) {
    const u = httpUrl.replace(/\/$/, '');
    return (u.startsWith('https') ? u.replace(/^https/, 'wss') : u.replace(/^http/, 'ws')) + '/ws/telemetry';
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
    try { sock = new WebSocket(wsUrl(httpUrl)); } catch (e) { schedule(); return; }
    st.ws = sock;
    sock.onopen = () => {
      st.retry = 0; st.opened = true; st.openedAt = Date.now(); st.lastMsgAt = 0;
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
    sock.onclose = () => {
      if (st.ws !== sock) return;
      st.ws = null; st.opened = false;
      o.onStatus && o.onStatus(status());
      o.onLog && o.onLog('WARN', 'pi ws closed (retry ' + (st.retry + 1) + ')');
      if (st.url) schedule();
    };
  }

  function close(clearUrl) {
    if (st.timer) { clearTimeout(st.timer); st.timer = null; }
    if (st.ws) { const s = st.ws; st.ws = null; try { s.close(); } catch (e) { /* noop */ } }
    st.opened = false;
    if (clearUrl) st.url = '';
  }

  return { connect, close, status };
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
