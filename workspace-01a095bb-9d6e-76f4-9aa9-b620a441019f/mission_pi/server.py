"""Pi HTTP + WebSocket server — the receive-only end the laptop UI reads.

Contract (must match mission-ui js/links.js + js/camera.js):
  GET /api/camera/frame/{cam}   JPEG snapshot (cam1=front, cam2=bottom).
                                The UI polls this (its WebRTC attempt is
                                optional and falls back here — v1 serves
                                snapshots only, no WebRTC).
  WS  /ws/telemetry             envelopes {channel, t, data}; Pi channels:
                                qr / fsm / event / log / system.
  GET /health                   {ok, cams, link}
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
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import JSONResponse, Response
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


def create_app(rig, mission, fc):
    """rig: CameraRig, mission: Mission (or None pre-start), fc: FCLink."""
    _require_api()
    app = FastAPI(title="mission_pi")
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


def serve_forever(app, host="0.0.0.0", port=8000):
    _require_api()
    uvicorn.run(app, host=host, port=port, log_level="warning")
