// links.js — Pi WebSocket client + local bridge SSE client
// Envelope schema: docs/api-contract.md

const CHANNELS = ['telemetry', 'system', 'fsm', 'qr', 'fence', 'log', 'event'];

// ---------------- Pi link (WS) ----------------
export function initPiLink(handlers) {
  let ws = null, timer = null, stopped = true;

  function connect() {
    const url = handlers.getPiUrl();
    if (!url || stopped) return;
    stopped = false;
    try { ws = new WebSocket(url.replace(/^http/, 'ws') + '/ws/telemetry'); }
    catch { handlers.onStatus('error'); schedule(); return; }
    ws.onopen = () => { handlers.onStatus('connected'); };
    ws.onmessage = (ev) => {
      try {
        const m = JSON.parse(ev.data);
        if (m && m.channel) handlers.onEnvelope(m);
      } catch { /* non-envelope (e.g. pong) */ }
    };
    ws.onclose = () => { handlers.onStatus('disconnected'); schedule(); };
    ws.onerror = () => { try { ws.close(); } catch { /* ignore */ } };
  }

  function schedule() {
    if (stopped || timer) return;
    timer = setTimeout(() => { timer = null; connect(); }, 2000);
  }

  return {
    connect,
    reconnect() { try { ws && ws.close(); } catch { /* ignore */ } connect(); },
    stop() { stopped = true; clearTimeout(timer); timer = null; try { ws && ws.close(); } catch { /* ignore */ } },
    get connected() { return !!(ws && ws.readyState === 1); },
  };
}

// ---------------- Bridge link (SSE, same origin) ----------------
export function initBridge(handlers) {
  const es = new EventSource('/api/mp/stream');
  const fwd = (name) => es.addEventListener(name, (e) => {
    try {
      const d = JSON.parse(e.data);
      if (name === 'mp-line') handlers.onMpLine(d);
      else if (name === 'mp-qr') handlers.onMpQr(d);
      else if (name === 'bridge-state') handlers.onBridgeState(d);
      else if (name === 'keepalive') { handlers.onBridgeStatus(true); return; }
      else handlers.onPiSse({ channel: name, t: d.ts || Date.now() / 1000, data: d });
    } catch { /* ignore */ }
  });
  CHANNELS.concat(['mp-line', 'mp-qr', 'bridge-state', 'keepalive']).forEach(fwd);
  es.onopen = () => handlers.onBridgeStatus(true);
  es.onerror = () => handlers.onBridgeStatus(false); // EventSource auto-reconnects
  return es;
}
