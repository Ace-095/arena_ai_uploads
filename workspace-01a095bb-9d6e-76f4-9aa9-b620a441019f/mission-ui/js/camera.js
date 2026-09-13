// camera.js — WebRTC primary, JPEG/SVG polling fallback, click-to-enlarge modal
const $ = (id) => document.getElementById(id);

export function initCamera(handlers) {
  let mode = 'off';
  let pollTimer = null;
  let pc = null, ws = null, stream = null;

  const vMain = $('videoCam1'), iMain = $('imgCam1');
  const vMod = $('videoModal'), iMod = $('imgModal');

  function setMode(m, extra = '') {
    mode = m;
    $('cMode').textContent = m + (extra ? ' ' + extra : '');
  }

  function stopAll() {
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
    try { ws && ws.close(); } catch { /* ignore */ }
    ws = null;
    try { pc && pc.close(); } catch { /* ignore */ }
    pc = null; stream = null;
    try { vMain.srcObject = null; } catch { /* ignore */ }
    try { vMod.srcObject = null; } catch { /* ignore */ }
  }

  function usePollingElements() {
    vMain.hidden = true; iMain.hidden = false;
    vMod.hidden = true; iMod.hidden = false;
  }

  function useWebRtcElements(s) {
    vMain.hidden = false; iMain.hidden = true;
    vMod.hidden = false; iMod.hidden = true;
    vMain.srcObject = s; vMod.srcObject = s;
  }

  function startPolling() {
    stopAll();
    usePollingElements();
    setMode('poll', '(fallback)');
    const tick = () => {
      const pi = handlers.getPiUrl();
      if (!pi) { $('cLat').textContent = '—'; return; }
      const t0 = performance.now();
      const src = pi + '/api/camera/frame/cam1?t=' + Date.now();
      iMain.src = src;
      iMain.onload = () => { $('cLat').textContent = (performance.now() - t0).toFixed(0) + ' ms'; };
      if (!$('modal').hidden) iMod.src = src;
      $('camStats1').textContent = 'poll ~3 fps';
    };
    tick();
    pollTimer = setInterval(tick, 300);
  }

  function tryWebRtc() {
    const pi = handlers.getPiUrl();
    if (!pi) { $('cMode').textContent = 'no Pi'; return; }
    stopAll();
    setMode('webrtc…');
    try {
      pc = new RTCPeerConnection({ iceServers: [] }); // LAN direct
      pc.addTransceiver('video', { direction: 'recvonly' });
      const t0 = performance.now();
      pc.ontrack = (ev) => {
        stream = ev.streams[0];
        useWebRtcElements(stream);
        setMode('webrtc');
        $('cLat').textContent = 'track in ' + (performance.now() - t0).toFixed(0) + ' ms';
        $('camStats1').textContent = 'WebRTC';
      };
      pc.onerror = () => { stopAll(); startPolling(); };

      ws = new WebSocket(pi.replace(/^http/, 'ws') + '/ws/webrtc/cam1');
      const to = setTimeout(() => { if (mode !== 'webrtc') { stopAll(); startPolling(); } }, 4000);
      ws.onopen = async () => {
        try {
          const offer = await pc.createOffer();
          await pc.setLocalDescription(offer);
          ws.send(JSON.stringify({ type: 'offer', sdp: offer.sdp }));
        } catch { stopAll(); startPolling(); }
      };
      ws.onmessage = async (ev) => {
        try {
          const m = JSON.parse(ev.data);
          if (m.type === 'answer') {
            await pc.setRemoteDescription(new RTCSessionDescription(m));
            clearTimeout(to);
          } else if (m.type === 'error') {
            clearTimeout(to); stopAll(); startPolling();
          }
        } catch { /* ignore */ }
      };
      ws.onerror = () => { clearTimeout(to); stopAll(); startPolling(); };
      ws.onclose = () => { if (mode !== 'webrtc' && mode !== 'poll') { stopAll(); startPolling(); } };
    } catch { startPolling(); }
  }

  return {
    start() { tryWebRtc(); },
    stop: stopAll,
    get mode() { return mode; },
    openModal() {
      $('modal').hidden = false;
      if (mode === 'poll') iMod.src = iMain.src;
    },
    closeModal() { $('modal').hidden = true; },
    isModalOpen() { return !$('modal').hidden; },
  };
}
