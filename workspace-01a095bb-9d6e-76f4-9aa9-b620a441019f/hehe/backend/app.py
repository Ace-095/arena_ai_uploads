"""
Geofence pipeline service (Milestone 1 + Milestone 2a).

Browser draws a polygon -> POSTs here -> we upload it to the Pixhawk as an
ArduPilot FENCE, read it back to confirm, and derive the local ENU frame
that the search algorithm (Milestone 5) will consume later.

INVARIANT enforced throughout: if any step fails, the fence is reported
`armable: false`. There is no fallback default area, and there is no
"partial success" state — either the confirmed, FC-verified fence exists,
or the system reports itself not ready to search. See README for why.
"""

import logging
import os
import sqlite3
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from config import config
from models import GeofenceUploadRequest, GeofenceStatus, CameraModeRequest, CameraFocusRequest, SearchStartRequest, SearchStatus, QRDetectionResult, FSMStatus
import state
import mavlink_fence as mf
from mavlink_bus import MavlinkBus
from telemetry import init_telemetry_hub, shutdown_telemetry_hub, get_telemetry_hub, websocket_endpoint
from camera_source import get_camera_source
from camera_manager import CameraManager
from search_algorithm import search_controller
from qr_pipeline import qr_pipeline_runner
import fsm
import cv2

# --- Logging ---
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("geofence")

# --- MAVLink Bus (single instance, created at startup) ---
_mavlink_bus: MavlinkBus | None = None


def get_mavlink_bus() -> MavlinkBus:
    """Get the global MavlinkBus instance."""
    if _mavlink_bus is None:
        raise RuntimeError("MAVLinkBus not initialized — call init_mavlink_bus() first")
    return _mavlink_bus


def init_mavlink_bus() -> MavlinkBus:
    """Initialize and start the global MavlinkBus instance."""
    global _mavlink_bus
    if _mavlink_bus is not None:
        return _mavlink_bus
    _mavlink_bus = MavlinkBus(
        device=config.MAVLINK_DEVICE,
        baud=config.MAVLINK_BAUD,
        timeout_s=config.MAVLINK_TIMEOUT_S,
    )
    _mavlink_bus.start()
    log.info("MAVLinkBus started on %s @ %d baud", config.MAVLINK_DEVICE, config.MAVLINK_BAUD)
    return _mavlink_bus


def shutdown_mavlink_bus() -> None:
    """Stop the global MavlinkBus instance."""
    global _mavlink_bus
    if _mavlink_bus is not None:
        _mavlink_bus.stop()
        log.info("MAVLinkBus stopped")
        _mavlink_bus = None


# --- Camera Manager (single instance, created at startup) ---
_camera_manager: CameraManager | None = None


def get_camera_manager() -> CameraManager:
    """Get the global CameraManager instance."""
    if _camera_manager is None:
        raise RuntimeError("CameraManager not initialized — call init_camera_manager() first")
    return _camera_manager


def init_camera_manager() -> CameraManager:
    """Initialize global CameraManager instance."""
    global _camera_manager
    if _camera_manager is not None:
        return _camera_manager

    source_name = config.CAMERA_SOURCE
    log.info("Initializing camera sources for mode '%s'", source_name)
    if source_name == "synthetic":
        cam1 = get_camera_source("synthetic", payload="CAM1-QR-TEST", fps=15.0)
        cam2 = get_camera_source("synthetic", payload="CAM2-QR-TEST", fps=15.0)
    elif source_name == "gazebo":
        cam1 = get_camera_source("gazebo", topic=config.ROS2_CAM1_TOPIC)
        cam2 = get_camera_source("gazebo", topic=config.ROS2_CAM2_TOPIC)
    else:
        # Auto-detect what's physically connected rather than assuming
        # camera_index 0/1 always map to cam1/cam2 — device enumeration
        # order isn't guaranteed to be stable across boots/wiring.
        from camera_source import detect_cameras, _SEARCH_CAM_MODEL_HINTS, _DECODE_CAM_MODEL_HINTS

        detected = detect_cameras()
        log.info("detect_cameras() found %d camera(s): %s", len(detected), detected)

        if len(detected) == 0:
            # Fail closed. A silent fallback here would mean the FSM's
            # camera-switch calls (see fsm.py::_switch_camera) succeed
            # against nothing, and the mission would run blind without
            # any clear signal why.
            raise RuntimeError(
                "detect_cameras() found 0 cameras. Check physical connections "
                "(rpicam-hello --list-cameras should show at least one) before "
                "starting the service — refusing to start with no camera source."
            )

        elif len(detected) == 1:
            # Single-camera mode: use the one physical camera for both
            # roles. Whatever its FOV/focal-length actually is now
            # determines BOTH the search altitude AND the decode
            # altitude — there's no separate long-focal-length camera to
            # get you the extra resolution DECODE needs. Expect to need
            # to descend closer than a dual-camera setup would require
            # (see the GSD math from our camera-altitude discussion).
            only = detected[0]
            log.warning(
                "Only 1 camera detected (model='%s') — running in SINGLE-CAMERA "
                "mode. Both cam1 (search) and cam2 (decode) will read the same "
                "physical camera. Decode altitude requirements change — this is "
                "degraded operation, not equivalent to the dual-camera design.",
                only["model"],
            )
            shared = get_camera_source("picamera2", camera_index=only["index"])
            cam1 = shared
            cam2 = shared

        else:
            # 2+ cameras: assign roles by sensor model rather than
            # trusting index order, which isn't guaranteed to correspond
            # to "search cam" vs "decode cam" just because it's first.
            def _match(info, hints):
                return any(h in info["model"].lower() for h in hints)

            search_matches = [c for c in detected if _match(c, _SEARCH_CAM_MODEL_HINTS)]
            decode_matches = [c for c in detected if _match(c, _DECODE_CAM_MODEL_HINTS)]

            if search_matches and decode_matches:
                search_cam, decode_cam = search_matches[0], decode_matches[0]
                log.info(
                    "Assigned by sensor model: cam1=search (index=%d, model=%s), "
                    "cam2=decode (index=%d, model=%s)",
                    search_cam["index"], search_cam["model"],
                    decode_cam["index"], decode_cam["model"],
                )
            else:
                # Models didn't match known hints — fall back to index
                # order, but this is exactly the kind of silent-guess
                # situation that should be loud, not quiet.
                search_cam, decode_cam = detected[0], detected[1]
                log.warning(
                    "Could not identify cameras by sensor model (expected hints "
                    "%s / %s, got models %s) — falling back to index order "
                    "(index 0 -> cam1/search, index 1 -> cam2/decode). Verify "
                    "this is actually correct for your wiring before trusting it.",
                    _SEARCH_CAM_MODEL_HINTS, _DECODE_CAM_MODEL_HINTS,
                    [c["model"] for c in detected],
                )

            if len(detected) > 2:
                log.warning(
                    "detect_cameras() found %d cameras, more than the 2 this "
                    "system expects — ignoring extras beyond the 2 assigned above.",
                    len(detected),
                )

            cam1 = get_camera_source("picamera2", camera_index=search_cam["index"])
            cam2 = get_camera_source("picamera2", camera_index=decode_cam["index"])

    _camera_manager = CameraManager({"cam1": cam1, "cam2": cam2})
    _camera_manager.set_active(config.DEFAULT_CAMERA_MODE)
    log.info("CameraManager initialized with default mode '%s'", config.DEFAULT_CAMERA_MODE)
    return _camera_manager


# --- FastAPI lifespan ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    bus = init_mavlink_bus()
    init_telemetry_hub(bus, position_hz=config.TELEMETRY_POSITION_HZ, stats_hz=config.TELEMETRY_STATS_HZ)
    cam_mgr = init_camera_manager()
    search_controller.set_bus(bus)
    qr_pipeline_runner.set_search_controller(search_controller)
    cam1 = cam_mgr.get_source("cam1")
    if cam1 is not None:
        qr_pipeline_runner.set_source(cam1)
    qr_pipeline_runner.start()
    fsm.init_fsm(bus, camera_manager=cam_mgr)
    yield
    # Shutdown
    qr_pipeline_runner.stop()
    search_controller.stop_search("Server shutting down")
    if _camera_manager is not None:
        await _camera_manager.close_all()
    shutdown_telemetry_hub()
    shutdown_mavlink_bus()


app = FastAPI(title="Geofence Pipeline", lifespan=lifespan)

# CORS for frontend on same LAN (adjust if needed)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve frontend static files if available
frontend_dir = "frontend" if os.path.exists("frontend") else "../frontend"
if os.path.exists(frontend_dir):
    app.mount("/static", StaticFiles(directory=frontend_dir), name="static")


# --- API Endpoints ---
@app.post("/api/geofence", response_model=GeofenceStatus)
async def upload_geofence(request: GeofenceUploadRequest):
    """Upload a polygon fence to the flight controller."""
    bus = get_mavlink_bus()
    vertices = request.vertices

    try:
        # 1. Request EKF origin from flight controller
        origin_lat, origin_lon = mf.request_ekf_origin(bus, config.MAVLINK_TIMEOUT_S)
        log.info("EKF origin: lat=%.7f lon=%.7f", origin_lat, origin_lon)

        # 2. Upload polygon fence
        mf.upload_polygon_fence(bus, vertices, config.MAVLINK_TIMEOUT_S)
        log.info("Fence upload sent (%d vertices)", len(vertices))

        # 3. Download fence back for readback verification
        readback = mf.download_fence(bus, config.MAVLINK_TIMEOUT_S)
        log.info("Fence readback received (%d vertices)", len(readback))

        # 4. Compare readback against what we sent (allow 1e-7 deg tolerance ~1cm)
        matched = True
        if len(readback) != len(vertices):
            matched = False
        else:
            for sent, recv in zip(vertices, readback):
                if abs(sent.lat - recv.lat) > 1e-7 or abs(sent.lon - recv.lon) > 1e-7:
                    matched = False
                    break

        if not matched:
            log.warning("Fence readback mismatch — FC stored different vertices")
            # Still proceed to enable fence, but mark as mismatch

        # 5. Enable fence on FC
        mf.enable_fence(bus, config.FENCE_ACTION, config.MAVLINK_TIMEOUT_S)
        log.info("Fence enabled (FENCE_ACTION=%d)", config.FENCE_ACTION)

        # 6. Build status object
        status = mf.build_geofence_status(
            bus,
            vertices,
            origin_lat,
            origin_lon,
            config.FENCE_ACTION,
            config.MAVLINK_TIMEOUT_S,
            fc_readback_matched=matched,
        )

        # 7. Persist to local cache (for UI polling)
        state.set(status)
        return status

    except mf.ConnectionError as e:
        log.error("Connection error: %s", e)
        state.clear()
        raise HTTPException(status_code=503, detail=f"Flight controller unreachable: {e}")
    except mf.FenceError as e:
        log.error("Fence error: %s", e)
        state.clear()
        raise HTTPException(status_code=500, detail=f"Fence operation failed: {e}")


@app.get("/api/geofence", response_model=GeofenceStatus)
async def get_geofence():
    """Get current cached fence status."""
    return state.get()


@app.get("/api/geofence/armable")
async def get_armable():
    """Lightweight armable check for arm sequence / search algorithm gating."""
    status = state.get()
    return {"armable": status.armable, "reason": status.reason}


@app.delete("/api/geofence", response_model=GeofenceStatus)
async def clear_geofence():
    """Clear fence on flight controller and local cache."""
    bus = get_mavlink_bus()
    try:
        mf.clear_fence(bus, config.MAVLINK_TIMEOUT_S)
        log.info("Fence cleared on flight controller")
    except mf.FenceError as e:
        log.error("Fence clear failed: %s", e)
        raise HTTPException(status_code=500, detail=f"Fence clear failed: {e}")
    except mf.ConnectionError as e:
        log.error("Connection error on clear: %s", e)
        raise HTTPException(status_code=503, detail=f"Flight controller unreachable: {e}")

    state.clear()
    return state.get()


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.get("/tiles/{z}/{x}/{y}.png")
async def get_tile(z: int, x: int, y: int):
    """
    Serve offline tiles from MBTiles SQLite database.
    Performs standard XYZ to TMS tile row conversion: tms_y = (2^z - 1) - y
    """
    mbtiles_path = config.MBTILES_PATH
    if not os.path.exists(mbtiles_path):
        raise HTTPException(status_code=404, detail="Tile database not found")

    tms_y = (1 << z) - 1 - y

    try:
        conn = sqlite3.connect(mbtiles_path)
        cur = conn.cursor()
        cur.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, tms_y),
        )
        row = cur.fetchone()
        conn.close()

        if row and row[0]:
            return Response(content=row[0], media_type="image/png")
        else:
            raise HTTPException(status_code=404, detail="Tile not found")
    except HTTPException:
        raise
    except Exception as e:
        log.error("Error reading tile z=%d x=%d y=%d: %s", z, x, y, e)
        raise HTTPException(status_code=404, detail="Error fetching tile")




@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    """WebSocket endpoint for real-time telemetry streaming."""
    async with websocket_endpoint(ws):
        try:
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass


# --- Camera Endpoints (Phase 5) ---
@app.post("/api/camera/mode")
async def set_camera_mode(request: CameraModeRequest):
    """
    Set active camera mode (cam1, cam2, dual, none).
    Manual API override wins until FSM state transition (Phase 8).
    """
    mgr = get_camera_manager()
    try:
        mgr.set_active(request.mode)
        return mgr.status()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/camera/status")
async def get_camera_status():
    """Get active camera mode, source status, and WebRTC metrics."""
    mgr = get_camera_manager()
    return mgr.status()


@app.post("/api/camera/focus")
async def set_camera_focus(request: CameraFocusRequest):
    """Set motorized camera focus position (real hardware Arducam IMX477)."""
    mgr = get_camera_manager()
    try:
        mgr.set_focus(request.position)
        return {"status": "ok", "position": request.position}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/camera/frame/{camera_id}")
async def get_camera_frame(camera_id: str):
    """Fetch single latest frame as JPEG image."""
    mgr = get_camera_manager()
    frame_bgr = mgr.get_frame(camera_id)
    if frame_bgr is None:
        raise HTTPException(status_code=404, detail=f"No frame available for camera '{camera_id}'")

    success, jpeg_bytes = cv2.imencode(".jpg", frame_bgr)
    if not success:
        raise HTTPException(status_code=500, detail="Frame encoding failed")

    return Response(content=jpeg_bytes.tobytes(), media_type="image/jpeg")


@app.websocket("/ws/webrtc/{camera_id}")
async def ws_webrtc(ws: WebSocket, camera_id: str):
    """
    WebSocket endpoint for WebRTC signaling exchange (SDP offer/answer).
    Signaled in-process with aiortc.
    """
    await ws.accept()
    mgr = get_camera_manager()
    try:
        while True:
            data = await ws.receive_json()
            if data.get("type") == "offer" and "sdp" in data:
                try:
                    answer = await mgr.create_webrtc_offer_answer(camera_id, data["sdp"])
                    await ws.send_json(answer)
                except Exception as exc:
                    log.error("WebRTC offer failed for '%s': %s", camera_id, exc)
                    await ws.send_json({"type": "error", "message": str(exc)})
            else:
                log.warning("Unknown WebRTC payload on '%s': %s", camera_id, data)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("WebRTC WebSocket disconnected for '%s': %s", camera_id, exc)


# --- Search Endpoints (Phase 6) ---
@app.post("/api/search/start", response_model=SearchStatus)
async def start_search(request: SearchStartRequest = SearchStartRequest()):
    """Start autonomous search algorithm sweep (or dry-run). Fails closed if geofence unarmable."""
    try:
        return search_controller.start_search(
            strategy_name=request.strategy,
            dry_run=request.dry_run,
            altitude_m=request.altitude_m,
            step_m=request.step_m,
        )
    except RuntimeError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/search/stop", response_model=SearchStatus)
async def stop_search():
    """Stop autonomous search algorithm sweep."""
    return search_controller.stop_search()


@app.post("/api/search/dry-run", response_model=SearchStatus)
async def dry_run_search(request: SearchStartRequest = SearchStartRequest()):
    """Compute and return search sweep waypoints without sending flight commands."""
    try:
        search_controller.run_dry_run(
            strategy_name=request.strategy,
            altitude_m=request.altitude_m,
            step_m=request.step_m,
        )
        return search_controller.get_status()
    except RuntimeError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/api/search/status", response_model=SearchStatus)
async def get_search_status():
    """Get current search algorithm execution status."""
    return search_controller.get_status()


# --- QR Pipeline Endpoints (Phase 7) ---
@app.get("/api/qr/status", response_model=QRDetectionResult)
async def get_qr_status():
    """Get current QR detection and consensus status."""
    return qr_pipeline_runner.consensus.get_status()


@app.post("/api/qr/process-frame")
async def process_qr_frame(camera_id: str = "cam1"):
    """Fetch latest frame from specified camera and run single-frame QR detection/decode."""
    mgr = get_camera_manager()
    frame = mgr.get_frame(camera_id)
    if frame is None:
        raise HTTPException(status_code=404, detail=f"No frame available for camera '{camera_id}'")

    newly_confirmed, result = qr_pipeline_runner.process_single_frame(frame)
    return result


# --- FSM Endpoints (Phase 8) ---
@app.post("/api/fsm/start", response_model=FSMStatus)
async def start_mission():
    """Start autonomous mission execution starting from PRE_FLIGHT_CHECK."""
    mission_fsm = fsm.get_fsm()
    try:
        return mission_fsm.start()
    except RuntimeError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/fsm/abort", response_model=FSMStatus)
async def abort_mission():
    """Trigger emergency manual FAILSAFE -> RTL."""
    mission_fsm = fsm.get_fsm()
    return mission_fsm.abort()


@app.get("/api/fsm/status", response_model=FSMStatus)
async def get_mission_status():
    """Return current FSM state and metrics."""
    mission_fsm = fsm.get_fsm()
    return mission_fsm.get_status()