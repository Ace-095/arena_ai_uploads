"""
camera_manager.py — Dual/single switchable camera pipeline manager.

Built on CameraSource abstraction. Manages WebRTC streaming via aiortc over
FastAPI WebSocket signaling, per-camera lifecycle, mode selection, and
focus control.

Ground Rule Invariants:
- All camera consumers ingest frames strictly via CameraSource.get_frame().
- Fail closed: missing/failing camera source defaults to stopped/error.
- Single point of truth for camera state.
"""

import asyncio
import logging
import time
from typing import Dict, Literal, Optional, Set
import numpy as np

from camera_source import CameraSource

log = logging.getLogger("camera_manager")

# --- Optional aiortc / av imports ---
try:
    import av
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    AIORTC_AVAILABLE = True
except ImportError:
    AIORTC_AVAILABLE = False
    VideoStreamTrack = object


# ---------------------------------------------------------------------------
# WebRTC Video Track (aiortc)
# ---------------------------------------------------------------------------

if AIORTC_AVAILABLE:
    class CameraStreamTrack(VideoStreamTrack):
        """
        VideoStreamTrack implementation consuming frames from a CameraSource.
        """
        kind = "video"

        def __init__(self, camera_id: str, source: CameraSource):
            super().__init__()
            self.camera_id = camera_id
            self.source = source
            self._frame_count = 0
            self._start_time = time.time()

        async def recv(self):
            pts, time_base = await self.next_timestamp()

            loop = asyncio.get_running_loop()
            try:
                # get_frame blocks until next frame is ready
                frame_bgr = await loop.run_in_executor(None, self.source.get_frame)
            except Exception as exc:
                log.warning("CameraStreamTrack[%s]: frame acquisition error: %s", self.camera_id, exc)
                # Fallback blank frame on failure
                frame_bgr = np.zeros((480, 640, 3), dtype=np.uint8)

            self._frame_count += 1
            video_frame = av.VideoFrame.from_ndarray(frame_bgr, format="bgr24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame
else:
    class CameraStreamTrack:
        def __init__(self, camera_id: str, source: CameraSource):
            raise NotImplementedError("aiortc is not installed in this environment.")


# ---------------------------------------------------------------------------
# Camera Manager
# ---------------------------------------------------------------------------

class CameraManager:
    """
    Coordinates camera sources, WebRTC peer connections, and streaming modes.

    Precedence Rule:
    Starts/stops per-camera encode+publish. Manual API override (via
    set_active or POST /api/camera/mode) wins until the FSM (Phase 8)
    calls set_active again — state that precedence rule explicitly:
    manual user API requests immediately override FSM camera mode;
    FSM state changes update the mode during autonomous state transitions.
    """

    def __init__(self, sources: Dict[str, CameraSource]):
        """
        Parameters
        ----------
        sources : dict[str, CameraSource]
            Dictionary mapping camera identifiers (e.g. 'cam1', 'cam2') to
            concrete CameraSource instances.
        """
        self._sources: Dict[str, CameraSource] = sources
        self._active_mode: Literal["cam1", "cam2", "dual", "none"] = "none"
        self._peer_connections: Dict[str, Set] = {cam_id: set() for cam_id in sources.keys()}

    # ------------------------------------------------------------------
    # Mode & Lifecycle Control
    # ------------------------------------------------------------------

    def set_active(self, mode: Literal["cam1", "cam2", "dual", "none"]) -> None:
        """
        Starts/stops per-camera encode+publish. Manual API override
        wins until the FSM (Phase 8) calls this again — manual API requests
        take immediate effect; subsequent FSM state transitions may update it.
        """
        if mode not in ("cam1", "cam2", "dual", "none"):
            raise ValueError(f"Invalid camera mode '{mode}'. Must be one of: cam1, cam2, dual, none")

        log.info("Setting camera active mode from '%s' to '%s'", self._active_mode, mode)
        self._active_mode = mode

        # Determine which sources should be running
        active_cams = set()
        if mode == "cam1":
            active_cams.add("cam1")
        elif mode == "cam2":
            active_cams.add("cam2")
        elif mode == "dual":
            active_cams.add("cam1")
            active_cams.add("cam2")

        for cam_id, source in self._sources.items():
            if cam_id in active_cams:
                try:
                    source.start()
                    log.info("Started camera source '%s'", cam_id)
                except Exception as e:
                    log.error("Failed to start camera source '%s': %s", cam_id, e)
            else:
                try:
                    source.stop()
                    log.info("Stopped camera source '%s'", cam_id)
                except Exception as e:
                    log.error("Failed to stop camera source '%s': %s", cam_id, e)

    def get_source(self, camera_id: str) -> Optional[CameraSource]:
        """
        Return the CameraSource instance for a given camera id (e.g.
        'cam1', 'cam2'), or None if unknown. Used by the FSM to repoint
        qr_pipeline_runner at whichever camera is currently active — see
        fsm.py's _switch_camera().
        """
        return self._sources.get(camera_id)

    def status(self) -> dict:
        """
        Return pipeline status including mode, source states, and WebRTC metrics.
        """
        sources_status = {}
        for cam_id, source in self._sources.items():
            started = getattr(source, "_started", False)
            pc_count = len(self._peer_connections.get(cam_id, set()))
            sources_status[cam_id] = {
                "type": source.__class__.__name__,
                "active": started,
                "peer_connections": pc_count,
            }

        return {
            "mode": self._active_mode,
            "aiortc_available": AIORTC_AVAILABLE,
            "sources": sources_status,
        }

    def set_focus(self, position: float) -> None:
        """
        Real-hardware-only (Arducam IMX477 motorized focus). No-op or
        NotImplementedError against sim/synthetic sources.
        """
        if not (0.0 <= position <= 1.0):
            raise ValueError("Focus position must be float between 0.0 and 1.0")

        handled = False
        for cam_id, source in self._sources.items():
            if hasattr(source, "set_focus"):
                try:
                    source.set_focus(position)
                    handled = True
                    log.info("Set focus on hardware camera '%s' to %.2f", cam_id, position)
                except Exception as e:
                    log.error("Error setting focus on '%s': %s", cam_id, e)
            else:
                log.info(
                    "set_focus(%.2f) called on camera '%s' (%s): no-op against non-hardware source",
                    position,
                    cam_id,
                    source.__class__.__name__,
                )

    def get_frame(self, camera_id: str) -> Optional[np.ndarray]:
        """
        Retrieve latest frame for single-frame GET API or test inspection.
        """
        source = self._sources.get(camera_id)
        if source is None:
            return None

        # Check if active/started
        if not getattr(source, "_started", False):
            # Attempt lazy start if source is enabled in current mode
            if self._active_mode in (camera_id, "dual"):
                source.start()
            else:
                return None

        try:
            return source.get_frame()
        except Exception as e:
            log.error("get_frame failed for '%s': %s", camera_id, e)
            return None

    # ------------------------------------------------------------------
    # WebRTC PeerConnection Management
    # ------------------------------------------------------------------

    async def create_webrtc_offer_answer(self, camera_id: str, offer_sdp: str) -> dict:
        """
        Process incoming SDP offer from client over WebSocket, attach CameraTrack,
        create SDP answer, and return answer dict.
        """
        if not AIORTC_AVAILABLE:
            raise RuntimeError("aiortc is not installed. WebRTC streaming unavailable.")

        source = self._sources.get(camera_id)
        if source is None:
            raise ValueError(f"Unknown camera_id '{camera_id}'")

        # Ensure source is started
        source.start()

        pc = RTCPeerConnection()
        self._peer_connections[camera_id].add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            log.info("WebRTC[%s] connection state: %s", camera_id, pc.connectionState)
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                self._peer_connections[camera_id].discard(pc)

        # Attach video track
        track = CameraStreamTrack(camera_id, source)
        pc.addTrack(track)

        # Process offer
        offer = RTCSessionDescription(sdp=offer_sdp, type="offer")
        await pc.setRemoteDescription(offer)

        # Create answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return {
            "type": "answer",
            "sdp": pc.localDescription.sdp,
        }

    async def close_all(self):
        """Clean up WebRTC connections and stop sources."""
        if AIORTC_AVAILABLE:
            for cam_id, pcs in self._peer_connections.items():
                for pc in list(pcs):
                    await pc.close()
                pcs.clear()

        for source in self._sources.values():
            try:
                source.stop()
            except Exception:
                pass
