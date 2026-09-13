/**
 * cameras.js — Camera Pipeline & WebRTC Client Manager (Phase 5).
 * 
 * Connects to the backend WebRTC signaling endpoint (/ws/webrtc/{camera_id})
 * and renders live video feeds into HTML5 <video> elements.
 * 
 * Falls back gracefully to single-frame polling (/api/camera/frame/{camera_id})
 * if WebRTC or aiortc is unavailable on the backend.
 */

export class CameraStreamManager {
  /**
   * @param {function(): string} getBackendUrl - Function returning backend base URL.
   */
  constructor(getBackendUrl) {
    this.getBackendUrl = getBackendUrl;
    this.peerConnections = new Map(); // cameraId -> RTCPeerConnection
    this.sockets = new Map();         // cameraId -> WebSocket
    this.pollTimers = new Map();      // cameraId -> interval ID (fallback)
    this.onStatusChange = null;
    this.onLatencyUpdate = null;
  }

  /**
   * Change camera mode via backend API.
   * @param {'cam1' | 'cam2' | 'dual' | 'none'} mode
   */
  async setMode(mode) {
    const url = this.getBackendUrl();
    if (!url) throw new Error("Backend URL not configured");

    const res = await fetch(`${url}/api/camera/mode`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode }),
    });

    if (!res.ok) {
      const err = await res.json();
      throw new Error(err.detail || "Failed to set camera mode");
    }

    const status = await res.json();
    if (this.onStatusChange) this.onStatusChange(status);
    return status;
  }

  /**
   * Get camera pipeline status from backend API.
   */
  async getStatus() {
    const url = this.getBackendUrl();
    if (!url) return null;

    const res = await fetch(`${url}/api/camera/status`);
    if (!res.ok) return null;
    return await res.json();
  }

  /**
   * Set hardware focus position (0.0 to 1.0).
   * @param {number} position
   */
  async setFocus(position) {
    const url = this.getBackendUrl();
    if (!url) return;

    await fetch(`${url}/api/camera/focus`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ position: parseFloat(position) }),
    });
  }

  /**
   * Attach camera stream (WebRTC or fallback img polling) to element.
   * @param {string} cameraId - 'cam1' or 'cam2'
   * @param {HTMLVideoElement | HTMLImageElement} element
   */
  async attachStream(cameraId, element) {
    this.detachStream(cameraId);

    const baseUrl = this.getBackendUrl();
    if (!baseUrl) return;

    const wsUrl = baseUrl.replace(/^http/, "ws") + `/ws/webrtc/${cameraId}`;
    const startTime = performance.now();

    try {
      // 1. Create WebRTC PeerConnection
      const pc = new RTCPeerConnection({
        iceServers: [] // LAN stream, direct peer connection
      });

      this.peerConnections.set(cameraId, pc);

      pc.addTransceiver("video", { direction: "recvonly" });

      pc.ontrack = (event) => {
        if (element.tagName === "VIDEO") {
          element.srcObject = event.streams[0];
          element.play().catch(e => console.warn(`Playback error on ${cameraId}:`, e));
        }
        const latency = performance.now() - startTime;
        if (this.onLatencyUpdate) this.onLatencyUpdate(cameraId, latency);
      };

      // 2. Open WebSocket signaling channel
      const ws = new WebSocket(wsUrl);
      this.sockets.set(cameraId, ws);

      ws.onopen = async () => {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        ws.send(JSON.stringify({ type: "offer", sdp: offer.sdp }));
      };

      ws.onmessage = async (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === "answer") {
          await pc.setRemoteDescription(new RTCSessionDescription(msg));
        } else if (msg.type === "error") {
          console.warn(`WebRTC error for ${cameraId}, engaging fallback:`, msg.message);
          this._startFallbackPolling(cameraId, element);
        }
      };

      ws.onerror = (err) => {
        console.warn(`WebSocket error for ${cameraId}, fallback:`, err);
        this._startFallbackPolling(cameraId, element);
      };

    } catch (err) {
      console.warn(`WebRTC init failed for ${cameraId}:`, err);
      this._startFallbackPolling(cameraId, element);
    }
  }

  /**
   * Fallback image polling when WebRTC is unavailable.
   */
  _startFallbackPolling(cameraId, element) {
    this.detachStream(cameraId);

    const baseUrl = this.getBackendUrl();
    if (!baseUrl) return;

    const timer = setInterval(async () => {
      const t0 = performance.now();
      const frameUrl = `${baseUrl}/api/camera/frame/${cameraId}?t=${Date.now()}`;
      
      if (element.tagName === "IMG") {
        element.src = frameUrl;
      } else if (element.tagName === "VIDEO") {
        // Replace video element display or update poster/canvas
        element.poster = frameUrl;
      }

      const elapsed = performance.now() - t0;
      if (this.onLatencyUpdate) this.onLatencyUpdate(cameraId, elapsed);
    }, 200); // 5 FPS fallback

    this.pollTimers.set(cameraId, timer);
  }

  /**
   * Detach camera stream and stop connections for camera ID.
   * @param {string} cameraId
   */
  detachStream(cameraId) {
    if (this.sockets.has(cameraId)) {
      try { this.sockets.get(cameraId).close(); } catch(e){}
      this.sockets.delete(cameraId);
    }

    if (this.peerConnections.has(cameraId)) {
      try { this.peerConnections.get(cameraId).close(); } catch(e){}
      this.peerConnections.delete(cameraId);
    }

    if (this.pollTimers.has(cameraId)) {
      clearInterval(this.pollTimers.get(cameraId));
      this.pollTimers.delete(cameraId);
    }
  }

  /**
   * Detach all active streams.
   */
  detachAll() {
    for (const camId of Array.from(this.peerConnections.keys())) {
      this.detachStream(camId);
    }
    for (const camId of Array.from(this.pollTimers.keys())) {
      this.detachStream(camId);
    }
  }
}
