/* camera.js — UI v2 dual-camera viewer (CAM1 pi-cam3 + CAM2 imx477).
 * TRUE LIVE FEED: MJPEG first (12fps smooth), polling fallback only if stream 404.
 * User fix 2026-09-17: live feed was showing as photos (1s snapshots) because
 * MJPEG failed and never retried. Now MJPEG auto-retries every 3s and polling
 * is 200ms for live feel, not 1s photos.
 *
 * Each camera points <img> at the MJPEG stream (/api/camera/stream/<cam>,
 * smooth ~12 fps) and falls back to snapshot polling (/api/camera/frame/<cam>)
 * when the stream errors. WebRTC (/ws/webrtc/<cam>) reserved for future.
 * One shared enlarge modal — also live MJPEG.
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
    if (c.retryTimer) { clearTimeout(c.retryTimer); c.retryTimer = null; }
    if (c.ws) { try { c.ws.close(); } catch (e) { /* noop */ } c.ws = null; }
    if (c.pc) { try { c.pc.close(); } catch (e) { /* noop */ } c.pc = null; }
    if (c.stream) {
      c.stream.getTracks().forEach((t) => { try { t.stop(); } catch (e) { /* noop */ } });
      c.stream = null;
    }
    const e = els(id);
    if (e.video) { try { e.video.pause(); e.video.srcObject = null; } catch (err) { /* noop */ } }
    if (e.img) { e.img.onerror = null; e.img.onload = null; }
    if (c.mode === 'mjpeg' && e.img) {
      try { e.img.removeAttribute('src'); } catch (err) { /* noop */ }
    }
    if (!silent) { c.mode = 'off'; onMode(id, c.mode); }
  }

  function startPolling(id) {
    const c = cams[id];
    const e = els(id);
    stopAll(id, true);
    c.mode = 'polling'; onMode(id, c.mode);
    onHint(id, 'live snapshots (MJPEG unavailable)…');
    const base = getPiUrl().replace(/\/$/, '');
    let failCount = 0;
    const tick = () => {
      if (c.paused) return;
      const t0 = performance.now();
      const img = new Image();
      img.onload = () => {
        if (c.mode !== 'polling') return;
        e.img.src = img.src; e.img.style.display = 'block'; e.video.style.display = 'none';
        e.img.dataset.ts = String(Date.now());
        c.frames += 1;
        failCount = 0;
        onLatency(id, Math.round(performance.now() - t0) + ' ms');
        onStats(id, c.frames + ' frames live');
        onHint(id, '');
      };
      img.onerror = () => {
        failCount += 1;
        if (failCount > 5) onHint(id, 'frame fetch failed — is Pi reachable?');
        // Auto-retry MJPEG every 3s even while polling — true live recovery
        if (failCount % 15 === 0) {
          onLog('INFO', id + ': polling retry → MJPEG');
          startMjpeg(id);
        }
      };
      img.src = base + '/api/camera/frame/' + id + '?t=' + Date.now();
    };
    tick();
    c.pollTimer = setInterval(tick, 200); // 200ms = 5fps live, not 1s photos
    // Also schedule MJPEG retry every 3s
    const retryMjpeg = () => {
      if (c.mode === 'polling' && !c.paused) {
        onLog('INFO', id + ': polling → retry MJPEG live stream');
        startMjpeg(id);
      }
    };
    c.retryTimer = setTimeout(function retryLoop() {
      retryMjpeg();
      if (c.mode === 'polling') c.retryTimer = setTimeout(retryLoop, 3000);
    }, 3000);
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
      if (c.mode === 'webrtc?') { onLog('WARN', id + ': WebRTC timeout → MJPEG'); startMjpeg(id); }
    }, 6000);
    ws.onopen = () => {
      let pc;
      try { pc = new RTCPeerConnection(); } catch (err) { clearTimeout(giveUp); startMjpeg(id); return; }
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
      }).catch(() => { clearTimeout(giveUp); startMjpeg(id); });
    };
    ws.onmessage = (ev) => {
      let m;
      try { m = JSON.parse(ev.data); } catch (err) { return; }
      if (m.sdp && c.pc) c.pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: m.sdp })).catch(() => {});
      else if (m.candidate && c.pc) c.pc.addIceCandidate(new RTCIceCandidate(m.candidate)).catch(() => {});
      else if (m.error) { clearTimeout(giveUp); startMjpeg(id); }
    };
    ws.onerror = () => { clearTimeout(giveUp); startMjpeg(id); };
    ws.onclose = () => {
      if (c.ws !== ws) return;
      c.ws = null;
      clearTimeout(giveUp);
      if (c.mode === 'webrtc?' || c.mode === 'webrtc') startMjpeg(id);
    };
  }

  function startMjpeg(id) {
    const c = cams[id];
    const e = els(id);
    stopAll(id, true);
    c.mode = 'mjpeg'; onMode(id, c.mode);
    onHint(id, 'live MJPEG…');
    onStats(id, 'live 12fps');
    onLatency(id, '');
    const base = getPiUrl().replace(/\/$/, '');
    e.img.style.display = 'block'; e.video.style.display = 'none';
    let loaded = false;
    e.img.onload = () => {
      if (c.mode !== 'mjpeg') return;
      if (!loaded) {
        loaded = true;
        onHint(id, '');
        onLog('INFO', id + ': MJPEG live feed started');
        onStats(id, 'live 12fps MJPEG');
      }
      // For MJPEG, onload fires once when stream starts, not per frame
      // Keep hint clear
      onHint(id, '');
    };
    e.img.onerror = () => {
      if (c.mode !== 'mjpeg') return;
      onLog('WARN', id + ': MJPEG stream failed → polling fallback (retry in 3s)');
      startPolling(id);
    };
    // True live MJPEG stream — no cache bust, browser keeps multipart open
    // Add fps param to match server's stream config
    e.img.src = base + '/api/camera/stream/' + id + '?fps=12&t=' + Date.now();
  }

  function startCam(id) {
    if (!cams[id]) cams[id] = { mode: 'off', pollTimer: null, retryTimer: null, pc: null, ws: null, stream: null, paused: false, frames: 0 };
    cams[id].paused = false;
    cams[id].frames = 0;
    // MJPEG first (smooth 12fps live, instant). Polling is fallback with auto-retry.
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

  // ---- shared enlarge modal — also true live MJPEG ----
  const modal = document.getElementById('modal');
  const modalLabel = document.getElementById('modalCamLabel');
  const videoModal = document.getElementById('videoModal');
  const imgModal = document.getElementById('imgModal');
  let modalTimer = null, modalRetry = null, modalId = null;

  function openModal(id) {
    const c = cams[id] || {};
    modalId = id;
    if (modalTimer) { clearInterval(modalTimer); modalTimer = null; }
    if (modalRetry) { clearTimeout(modalRetry); modalRetry = null; }
    modalLabel.textContent = (id === 'cam1' ? 'CAM1 · pi-cam3' : 'CAM2 · imx477');
    videoModal.style.display = 'none'; imgModal.style.display = 'none';
    if (c.mode === 'webrtc' && c.stream) {
      videoModal.srcObject = c.stream;
      videoModal.play().catch(() => {});
      videoModal.style.display = 'block';
    } else if (c.mode === 'mjpeg') {
      const base = getPiUrl().replace(/\/$/, '');
      imgModal.src = base + '/api/camera/stream/' + id + '?fps=12&t=' + Date.now();
      imgModal.style.display = 'block';
      imgModal.onerror = () => {
        // Fallback to live polling 200ms in modal
        const base2 = getPiUrl().replace(/\/$/, '');
        const tick = () => { imgModal.src = base2 + '/api/camera/frame/' + id + '?t=' + Date.now(); };
        tick();
        modalTimer = setInterval(tick, 200);
      };
    } else {
      const base = getPiUrl().replace(/\/$/, '');
      const tick = () => { imgModal.src = base + '/api/camera/frame/' + id + '?t=' + Date.now(); };
      tick();
      imgModal.style.display = 'block';
      modalTimer = setInterval(tick, 200);
      // Try to upgrade to MJPEG in modal too
      modalRetry = setTimeout(() => {
        if (modalId === id) {
          imgModal.src = base + '/api/camera/stream/' + id + '?fps=12&t=' + Date.now();
          if (modalTimer) { clearInterval(modalTimer); modalTimer = null; }
        }
      }, 3000);
    }
    modal.classList.remove('hidden');
  }

  function closeModal() {
    if (modalTimer) { clearInterval(modalTimer); modalTimer = null; }
    if (modalRetry) { clearTimeout(modalRetry); modalRetry = null; }
    try { videoModal.pause(); videoModal.srcObject = null; } catch (e) { /* noop */ }
    imgModal.removeAttribute('src'); imgModal.onerror = null;
    modalId = null;
    modal.classList.add('hidden');
  }

  function refreshModalFrame() {
    if (modalId == null || modal.classList.contains('hidden')) return;
    const c = cams[modalId] || {};
    if (c.mode !== 'polling') return;
    const e = els(modalId);
    if (e.img && e.img.src) imgModal.src = e.img.src;
  }
  setInterval(refreshModalFrame, 200);

  modal.addEventListener('click', (ev) => { if (ev.target === modal) closeModal(); });

  return { startCam, togglePause, openModal, closeModal };
}

window.MissionCameras = { initCameras };
})();
