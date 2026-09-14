/* camera.js — UI v2 dual-camera viewer (CAM1 pi-cam3 + CAM2 imx477).
 * Each camera points <img> at the MJPEG stream (/api/camera/stream/<cam>,
 * smooth ~12 fps) and falls back to snapshot polling (/api/camera/frame/<cam>)
 * when the stream errors. WebRTC (/ws/webrtc/<cam>) is reserved for a future
 * server. One shared enlarge modal.
 */
(function () {
'use strict';

function initCameras(o) {
  o = o || {};
  const getPiUrl = o.getPiUrl || (() => '');
  const onMode = o.onMode || function () {};
  const onLatency = o.onLatency || function () {};
  const onStats = o.onStats || function () {};
  const onHint = o.onHint || function () {};
  const onLog = o.onLog || function () {};

  const cams = {};

  function els(id) {
    const s = id === 'cam1' ? 'Cam1' : 'Cam2';
    return {
      video: document.getElementById('video' + s),
      img: document.getElementById('img' + s),
    };
  }

  function stopAll(id, silent) {
    const c = cams[id];
    if (!c) return;
    if (c.pollTimer) { clearInterval(c.pollTimer); c.pollTimer = null; }
    if (c.ws) { try { c.ws.close(); } catch (e) { /* noop */ } c.ws = null; }
    if (c.pc) { try { c.pc.close(); } catch (e) { /* noop */ } c.pc = null; }
    if (c.stream) {
      c.stream.getTracks().forEach((t) => { try { t.stop(); } catch (e) { /* noop */ } });
      c.stream = null;
    }
    const e = els(id);
    if (e.video) { try { e.video.pause(); e.video.srcObject = null; } catch (err) { /* noop */ } }
    if (e.img) { e.img.onerror = null; }  // don't trip the mjpeg->polling fallback on purpose
    if (c.mode === 'mjpeg' && e.img) {
      try { e.img.removeAttribute('src'); } catch (err) { /* noop */ }  // kills the <img> stream
    }
    if (!silent) { c.mode = 'off'; onMode(id, c.mode); }
  }

  function startPolling(id) {
    const c = cams[id];
    const e = els(id);
    stopAll(id, true);
    c.mode = 'polling'; onMode(id, c.mode);
    onHint(id, 'polling snapshots from Pi…');
    const base = getPiUrl().replace(/\/$/, '');
    const tick = () => {
      if (c.paused) return;
      const t0 = performance.now();
      const img = new Image();
      img.onload = () => {
        if (c.mode !== 'polling') return;
        e.img.src = img.src; e.img.style.display = 'block'; e.video.style.display = 'none';
        e.img.dataset.ts = String(Date.now());
        c.frames += 1;
        onLatency(id, Math.round(performance.now() - t0) + ' ms');
        onStats(id, c.frames + ' frames');
        onHint(id, '');
      };
      img.onerror = () => { onHint(id, 'frame fetch failed — is the Pi reachable?'); };
      img.src = base + '/api/camera/frame/' + id + '?t=' + Date.now();
    };
    tick();
    c.pollTimer = setInterval(tick, 1000);
  }

  function startWebrtc(id) {
    const c = cams[id];
    const e = els(id);
    stopAll(id, true);
    const base = getPiUrl().replace(/\/$/, '');
    const wsUrl = (base.startsWith('https') ? base.replace(/^https/, 'wss') : base.replace(/^http/, 'ws'))
      + '/ws/webrtc/' + id;
    let ws;
    try { ws = new WebSocket(wsUrl); } catch (err) { startPolling(id); return; }
    c.ws = ws;
    c.mode = 'webrtc?'; onMode(id, c.mode);
    onHint(id, 'negotiating WebRTC…');
    const giveUp = setTimeout(() => {
      if (c.mode === 'webrtc?') { onLog('WARN', id + ': WebRTC timeout → polling fallback'); startPolling(id); }
    }, 6000);
    ws.onopen = () => {
      let pc;
      try { pc = new RTCPeerConnection(); } catch (err) { clearTimeout(giveUp); startPolling(id); return; }
      c.pc = pc;
      pc.onicecandidate = (ev) => {
        if (ev.candidate && ws.readyState === 1) ws.send(JSON.stringify({ candidate: ev.candidate }));
      };
      pc.ontrack = (ev) => {
        clearTimeout(giveUp);
        c.stream = ev.streams[0];
        e.video.srcObject = c.stream;
        e.video.play().catch(() => {});
        e.video.style.display = 'block'; e.img.style.display = 'none';
        c.mode = 'webrtc'; onMode(id, c.mode);
        onHint(id, '');
        onLog('INFO', id + ': WebRTC live');
        pc.getStats().then((stats) => {
          stats.forEach((r) => {
            if (r.type === 'inbound-rtp' && r.kind === 'video' && r.jitter != null)
              onLatency(id, Math.round(r.jitter * 1000) + ' ms jit');
          });
        }).catch(() => {});
      };
      pc.createOffer({ offerToReceiveVideo: true }).then((off) => {
        pc.setLocalDescription(off).then(() => {
          if (ws.readyState === 1) ws.send(JSON.stringify({ sdp: off.sdp, type: off.type }));
        });
      }).catch(() => { clearTimeout(giveUp); startPolling(id); });
    };
    ws.onmessage = (ev) => {
      let m;
      try { m = JSON.parse(ev.data); } catch (err) { return; }
      if (m.sdp && c.pc) c.pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: m.sdp })).catch(() => {});
      else if (m.candidate && c.pc) c.pc.addIceCandidate(new RTCIceCandidate(m.candidate)).catch(() => {});
      else if (m.error) { clearTimeout(giveUp); startPolling(id); }
    };
    ws.onerror = () => { clearTimeout(giveUp); startPolling(id); };
    ws.onclose = () => {
      if (c.ws !== ws) return;
      c.ws = null;
      clearTimeout(giveUp);
      if (c.mode === 'webrtc?') startPolling(id);
      else if (c.mode === 'webrtc' && !c.paused) startPolling(id);
    };
  }

  function startMjpeg(id) {
    const c = cams[id];
    const e = els(id);
    stopAll(id, true);
    c.mode = 'mjpeg'; onMode(id, c.mode);
    onHint(id, '');
    onStats(id, 'live');
    onLatency(id, '');
    const base = getPiUrl().replace(/\/$/, '');
    e.img.style.display = 'block'; e.video.style.display = 'none';
    e.img.onerror = () => {
      if (c.mode !== 'mjpeg') return;
      onLog('WARN', id + ': MJPEG stream failed → polling fallback');
      startPolling(id);
    };
    e.img.src = base + '/api/camera/stream/' + id;
  }

  function startCam(id) {
    if (!cams[id]) cams[id] = { mode: 'off', pollTimer: null, pc: null, ws: null, stream: null, paused: false, frames: 0 };
    cams[id].paused = false;
    // MJPEG first (smooth, instant). startWebrtc is kept for a future
    // server that implements /ws/webrtc — today it would only add a 6 s
    // black tile before the fallback. startPolling stays as the fallback
    // for servers without /api/camera/stream.
    startMjpeg(id);
  }

  function togglePause(id) {
    const c = cams[id];
    if (!c) return false;
    c.paused = !c.paused;
    if (c.paused) { stopAll(id, true); c.mode = 'paused'; onMode(id, c.mode); onHint(id, 'paused'); }
    else startCam(id);
    return c.paused;
  }

  // ---- shared enlarge modal ----
  const modal = document.getElementById('modal');
  const modalLabel = document.getElementById('modalCamLabel');
  const videoModal = document.getElementById('videoModal');
  const imgModal = document.getElementById('imgModal');
  let modalTimer = null, modalId = null;

  function openModal(id) {
    const c = cams[id] || {};
    modalId = id;
    modalLabel.textContent = (id === 'cam1' ? 'CAM1 · pi-cam3' : 'CAM2 · imx477');
    videoModal.style.display = 'none'; imgModal.style.display = 'none';
    if (c.mode === 'webrtc' && c.stream) {
      videoModal.srcObject = c.stream;
      videoModal.play().catch(() => {});
      videoModal.style.display = 'block';
    } else if (c.mode === 'mjpeg') {
      const base = getPiUrl().replace(/\/$/, '');
      imgModal.src = base + '/api/camera/stream/' + id;
      imgModal.style.display = 'block';
    } else {
      const base = getPiUrl().replace(/\/$/, '');
      const tick = () => { imgModal.src = base + '/api/camera/frame/' + id + '?t=' + Date.now(); };
      tick();
      imgModal.style.display = 'block';
      modalTimer = setInterval(tick, 1000);
    }
    modal.classList.remove('hidden');
  }

  function closeModal() {
    if (modalTimer) { clearInterval(modalTimer); modalTimer = null; }
    try { videoModal.pause(); videoModal.srcObject = null; } catch (e) { /* noop */ }
    imgModal.removeAttribute('src');
    modalId = null;
    modal.classList.add('hidden');
  }

  function refreshModalFrame() {
    // keep the enlarged polling view in sync with the grid view
    if (modalId == null || modal.classList.contains('hidden')) return;
    const c = cams[modalId] || {};
    if (c.mode !== 'polling') return;
    const e = els(modalId);
    if (e.img && e.img.src) imgModal.src = e.img.src;
  }
  setInterval(refreshModalFrame, 1000);

  modal.addEventListener('click', (ev) => { if (ev.target === modal) closeModal(); });

  return { startCam, togglePause, openModal, closeModal };
}

window.MissionCameras = { initCameras };
})();
