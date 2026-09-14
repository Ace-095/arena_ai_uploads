"""Pi HTTP + WebSocket server — the receive-only end the laptop UI reads.

Contract (must match mission-ui js/links.js + js/camera.js):
  GET /api/camera/frame/{cam}   JPEG snapshot (cam1=front, cam2=bottom).
                                The UI polls this (its WebRTC attempt is
                                optional and falls back here — v1 serves
                                snapshots only, no WebRTC).
  GET /api/camera/stream/{cam}  MJPEG multipart/x-mixed-replace (?fps=12).
                                The UI points each tile's <img> here FIRST
                                (smooth ~12 fps); snapshots stay as fallback.
                                Needs the StreamManager (streamm1.py) passed
                                to create_app, else 404 -> UI falls back.
  WS  /ws/telemetry             envelopes {channel, t, data}; Pi channels:
                                qr / fsm / event / log / system.
  GET /health                   {ok, cams, link}
  GET /api/cameras              rig status {cam1: {...}, cam2: {...}}
  GET/POST /api/camera/status   {cam?} -> {cam, model, controls} (contract)
  POST /api/camera/controls     UI slider tuning -> {ok, cam, controls}
  GET/POST /api/camera/{cam}/controls  RAW driver props (bench/SSH only)
  GET /api/mission/status       mission snapshot passthrough.
  POST /api/mission/takeover    LOCAL USE ONLY (bench/SSH): force GUIDED
  POST /api/mission/abort       takeover now / RTL + stop. The flight UI
                                never calls these (Pi link is receive-only).

Run behind the field router: Pi + laptop join the same LAN (router may
carry LTE for remote viewing, but streaming itself is plain LAN HTTP).
"""
import asyncio
import json
import logging
import threading
import time

log = logging.getLogger("server")

try:
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import JSONResponse, Response, StreamingResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
    _HAVE_API = True
except Exception as e:
    _HAVE_API = False
    _API_ERR = e


class Hub:
    """Thread-safe WS broadcast hub. Mission threads call push(); the
    FastAPI loop owns delivery."""

    def __init__(self):
        self._clients = set()
        self._lock = threading.Lock()
        self._loop = None

    def attach(self, loop):
        self._loop = loop

    def push(self, channel, data):
        env = json.dumps({"channel": channel, "t": time.time(), "data": data})
        with self._lock:
            clients = list(self._clients)
        if not clients or self._loop is None:
            return
        for ws in clients:
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(env), self._loop)
            except Exception:
                pass

    def add(self, ws):
        with self._lock:
            self._clients.add(ws)

    def drop(self, ws):
        with self._lock:
            self._clients.discard(ws)

    @property
    def n_clients(self):
        with self._lock:
            return len(self._clients)


def _require_api():
    if not _HAVE_API:
        raise RuntimeError("fastapi/uvicorn not installed (%r)" % (_API_ERR,))


def create_app(rig, mission, fc, streams=None):
    """rig: CameraRig, mission: Mission (or None pre-start), fc: FCLink.
    streams: streamm1.StreamManager (or None -> /stream/* answers 404 and
    the UI falls back to snapshot polling)."""
    _require_api()
    app = FastAPI(title="mission_pi")
    # The UI is served from the BRIDGE origin, not the Pi — without CORS
    # every UI fetch() to the Pi (camera probe, slider tuning) dies in the
    # browser while <img> polling and WS sail through. Open LAN tool: allow all.
    app.add_middleware(CORSMiddleware, allow_origins=["*"],
                       allow_methods=["*"], allow_headers=["*"])
    hub = Hub()

    @app.on_event("startup")
    async def _startup():
        hub.attach(asyncio.get_running_loop())

    @app.get("/health")
    async def health():
        return {"ok": True, "ts": time.time(),
                "link": fc.link_ok() if fc else False,
                "cams": rig.status() if rig else {},
                "ws_clients": hub.n_clients}

    @app.get("/api/cameras")
    async def cameras():
        return rig.status() if rig else {}

    @app.get("/api/camera/frame/{cam}")
    async def frame(cam: str):
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        jpg = c.jpeg()
        if jpg is None:
            return JSONResponse({"detail": "no frame yet from %s" % cam}, 503)
        return Response(jpg, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.get("/api/camera/stream/{cam}")
    async def stream(cam: str, request: Request, fps: int = 0):
        """MJPEG multipart stream for the UI tiles (<img> points here).

        ?fps= caps the serve rate (default: the broadcaster's fps; never
        above it — stale ticks are skipped, never duplicated). 404/503
        answers make the tile fall back to snapshot polling.
        """
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        st = streams.get(cam) if streams else None
        if st is None:
            return JSONResponse({"detail": "streaming disabled on this server"}, 404)
        if not getattr(c, "running", False):
            return JSONResponse({"detail": "%s is not running" % cam}, 503)
        period = 1.0 / min(max(int(fps or st.fps), 1), 30)

        async def gen():
            last = -1
            try:
                while True:
                    try:
                        if await request.is_disconnected():
                            break
                    except Exception:
                        pass
                    jpg, seq = st.latest()
                    if jpg is not None and seq != last:
                        last = seq
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                               b"Content-Length: %d\r\n\r\n" % len(jpg)
                               + jpg + b"\r\n")
                    await asyncio.sleep(period)
            except asyncio.CancelledError:
                pass
            except Exception:
                pass

        return StreamingResponse(gen(),
                                 media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"Cache-Control": "no-store",
                                          "X-Accel-Buffering": "no"})

    @app.api_route("/api/camera/status", methods=["GET", "POST"])
    async def camera_status(req: Request):
        """Contract: {cam?} -> {cam, model, controls} (UI seeds sliders here)."""
        cam = req.query_params.get("cam", "cam1")
        if req.method == "POST":
            try:
                body = await req.json()
            except Exception:
                body = None
            if isinstance(body, dict) and body.get("cam"):
                cam = str(body["cam"])
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        return {"cam": cam, "model": c.model, "controls": c.get_tuning()}

    @app.post("/api/camera/controls")
    async def camera_controls(req: Request):
        """Contract: {cam, exposure_us, gain_db, af_mode, adaptive,
        brightness, contrast, saturation, sharpness} -> {ok, cam, controls}.
        The UI sliders POST here (debounced)."""
        try:
            body = await req.json()
        except Exception:
            body = None
        if not isinstance(body, dict):
            return JSONResponse({"detail": "want a JSON tuning object {cam, ...}"}, 400)
        cam = str(body.get("cam", "cam1"))
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        applied = c.apply_tuning(body)
        return {"ok": True, "status": "ok", "cam": cam, "controls": applied}

    @app.get("/api/camera/{cam}/controls")
    async def get_controls(cam: str):
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        fn = getattr(c, "get_controls", None)
        if fn is None:
            return JSONResponse({"detail": "%s has no tunable controls" % cam}, 400)
        return {"cam": cam, "controls": fn()}

    @app.post("/api/camera/{cam}/controls")
    async def set_controls(cam: str, req: Request):
        """BENCH/SSH-LOCAL tuning (like /takeover — the flight UI never
        calls this): POST {"contrast": 40, "saturation": 60} and watch the
        tile. Mirrors forward to their source device."""
        c = rig.get(cam) if rig else None
        if c is None:
            return JSONResponse({"detail": "unknown camera (want cam1|cam2)"}, 404)
        fn = getattr(c, "apply_controls", None)
        if fn is None:
            return JSONResponse({"detail": "%s has no tunable controls" % cam}, 400)
        try:
            body = await req.json()
        except Exception:
            body = None
        if not isinstance(body, dict):
            return JSONResponse({"detail": "want a JSON object {name: value}"}, 400)
        return {"cam": cam, "applied": fn(body)}

    @app.get("/api/mission/status")
    async def mission_status():
        if mission is None:
            return {"phase": "BOOT", "detail": "mission not started"}
        return mission.status()

    @app.post("/api/mission/takeover")
    async def takeover():
        if mission is None:
            return JSONResponse({"detail": "mission not started"}, 503)
        mission.request_takeover()
        return {"status": "takeover requested"}

    @app.post("/api/mission/abort")
    async def abort():
        if mission is None:
            return JSONResponse({"detail": "mission not started"}, 503)
        mission.request_abort()
        return {"status": "abort requested (RTL)"}

    @app.websocket("/ws/telemetry")
    async def ws_telemetry(ws: WebSocket):
        await ws.accept()
        hub.add(ws)
        try:
            while True:
                await ws.receive_text()  # ignore inbound; link is read-only
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            hub.drop(ws)

    app.state.hub = hub
    return app


def create_server(app, host="0.0.0.0", port=8000):
    """A stoppable uvicorn Server (lets main shut the loop down cleanly —
    killing the process with a live uvloop in a daemon thread segfaults)."""
    _require_api()
    return uvicorn.Server(uvicorn.Config(app, host=host, port=port,
                                         log_level="warning"))


def serve_forever(app, host="0.0.0.0", port=8000):
    create_server(app, host, port).run()
