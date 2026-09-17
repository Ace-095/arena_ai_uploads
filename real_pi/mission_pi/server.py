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
  GET /api/fsm/status           contract alias -> {state, phase, detail, ...}
  GET /api/qr/status            contract alias -> {payload, streak, required,
                                confirmed} (curl-able from the laptop)
  POST /api/fsm/start|abort     contract aliases of takeover/abort (bench).
  GET /                         human landing page: alive? bring-up? urls?
  POST /api/mission/takeover    LOCAL USE ONLY (bench/SSH): force GUIDED
  POST /api/mission/abort       takeover now / RTL + stop. The flight UI
                                never calls these (Pi link is receive-only).

Run behind the field router: Pi + laptop join the same LAN (router may
carry LTE for remote viewing, but streaming itself is plain LAN HTTP).

Liveness rule (2026-09-14 fix): this server starts BEFORE the FC link, the
cameras and the detector, and it outlives all of them. A Pi that cannot
answer http://<ip>:8000 is a Pi the operator cannot debug — so bring-up
failures are recorded in bringup.Bringup and reported here (/health plus
the WS `log` / `system` channels) instead of killing the process. The WS
also replays recent envelopes to every new client and pushes `system` on a
timer, so a browser that joins mid-mission sees state immediately and can
tell a live Pi from a half-open socket.
"""
import asyncio
import json
import logging
import threading
import time

try:
    import bringup as _bringup
except Exception:            # pragma: no cover - same package, always there
    _bringup = None

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
    FastAPI loop owns delivery.

    Keeps a ring of recent envelopes so a browser that connects mid-mission
    (the normal case — the operator opens the UI after the drone is already
    searching) gets the current picture instead of a blank panel until the
    next phase change. The contract calls this "replays a short history
    burst on every WS connect"; the mock bridge does it, the Pi now does too.
    """

    REPLAY_CHANNELS = ("fsm", "system", "qr", "event")

    def __init__(self, replay_cap=40):
        self._clients = set()
        self._lock = threading.Lock()
        self._loop = None
        self._replay_cap = max(4, int(replay_cap))
        self._recent = {}          # channel -> deque-ish list of envelopes
        self._sent = 0
        self._dropped = 0

    def attach(self, loop):
        self._loop = loop

    def remember(self, channel, env):
        """Store one envelope for late-joining clients (last per channel for
        state channels, a short tail for events)."""
        with self._lock:
            if channel not in self.REPLAY_CHANNELS:
                return
            cur = self._recent.setdefault(channel, [])
            cur.append(env)
            cap = 1 if channel in ("fsm", "system") else self._replay_cap
            del cur[:-cap]

    def recent(self, channels=None):
        """Envelopes to replay on connect, oldest first."""
        with self._lock:
            out = []
            for ch in (channels or ("system", "fsm", "qr", "event")):
                out.extend(self._recent.get(ch, []))
            return out

    def push(self, channel, data):
        """Broadcast to WS clients. Returns clients handed to (0 = nobody
        listening — the store&forward UI-route proxy reads this)."""
        env = json.dumps({"channel": channel, "t": time.time(), "data": data})
        self.remember(channel, env)
        with self._lock:
            clients = list(self._clients)
            self._sent += 1
        if not clients or self._loop is None:
            return 0
        for ws in clients:
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(env), self._loop)
            except Exception:
                with self._lock:
                    self._dropped += 1
        return len(clients)

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

    def stats(self):
        with self._lock:
            return {"clients": len(self._clients), "pushed": self._sent,
                    "send_errors": self._dropped,
                    "loop": self._loop is not None}


def _require_api():
    if not _HAVE_API:
        raise RuntimeError("fastapi/uvicorn not installed (%r)" % (_API_ERR,))


def create_app(rig, mission, fc, streams=None, bringup=None, pulse_s=2.0,
               port=8000):
    """rig: CameraRig, mission: Mission (or None pre-start), fc: FCLink.
    streams: streamm1.StreamManager (or None -> /stream/* answers 404 and
    the UI falls back to snapshot polling).
    bringup: bringup.Bringup ledger (or None) — published at /health so the
    operator can see WHICH subsystem is missing without SSH.
    pulse_s: `system` channel cadence (also the WS keepalive)."""
    _require_api()
    app = FastAPI(title="mission_pi")
    # The UI is served from the BRIDGE origin, not the Pi — without CORS
    # every UI fetch() to the Pi (camera probe, slider tuning) dies in the
    # browser while <img> polling and WS sail through. Open LAN tool: allow all.
    app.add_middleware(CORSMiddleware, allow_origins=["*"],
                       allow_methods=["*"], allow_headers=["*"])
    hub = Hub()
    t0 = time.time()
    pulse_s = max(0.5, float(pulse_s or 2.0))

    def _mission_snapshot():
        if mission is None:
            return {"state": "NO_MISSION", "phase": "NO_MISSION",
                    "detail": "mission object not created"}
        try:
            st = mission.status()
        except Exception as e:
            return {"state": "ERROR", "phase": "ERROR", "detail": repr(e)}
        # contract `fsm`: {state, reason?} — the UI reads state ?? to ?? phase
        st.setdefault("state", st.get("phase"))
        st.setdefault("reason", st.get("detail"))
        return st

    def _qr_snapshot():
        st = _mission_snapshot()
        streak = str(st.get("streak") or "0/3")
        try:
            have, need = streak.split("/")
            have, need = int(have), int(need)
        except Exception:
            have, need = 0, 3
        return {"payload": st.get("payload"), "streak": have, "required": need,
                "confirmed": bool(st.get("payload"))}

    @app.on_event("startup")
    async def _startup():
        hub.attach(asyncio.get_running_loop())
        asyncio.get_running_loop().create_task(_pulse())

    async def _pulse():
        """`system` on a timer (UI contract + WS keepalive) and an `fsm`
        refresh every third tick, so a connected UI never looks dead and a
        dead socket is noticed by the UI's last-message age."""
        n = 0
        last_errs = 0
        while True:
            await asyncio.sleep(pulse_s)
            n += 1
            try:
                hub.push("system", _bringup.system_stats())
            except Exception as e:
                log.debug("system pulse failed: %r", e)
            if n % 3 == 0:
                try:
                    hub.push("fsm", _mission_snapshot())
                except Exception:
                    pass
            if bringup is not None:
                try:
                    errs = bringup.snapshot().get("errors") or []
                    if len(errs) != last_errs:
                        last_errs = len(errs)
                        e = errs[-1]
                        hub.push("log", {"level": "ERROR",
                                         "msg": "bring-up %s — %s"
                                                % (e.get("stage"), e.get("error"))})
                except Exception:
                    pass

    @app.get("/")
    async def index():
        """Human landing page: paste the Pi URL in a browser tab and this
        answers — the fastest possible 'is mission_pi even alive?' check."""
        bu = bringup.snapshot() if bringup else {}
        rows = "".join(
            "<tr><td>%s</td><td><b>%s</b></td><td>%s</td></tr>" % (
                k, v, (bu.get("detail") or {}).get(k, ""))
            for k, v in sorted((bu.get("state") or {}).items()))
        errs = "".join("<li>%s: %s</li>" % (e.get("stage"), e.get("error"))
                       for e in (bu.get("errors") or []))
        body = (
            "<!doctype html><meta charset='utf-8'><title>mission_pi</title>"
            "<body style='font:14px/1.5 system-ui,sans-serif;margin:2em'>"
            "<h1>mission_pi is up</h1>"
            "<p>uptime %.0f s &middot; ws clients %d &middot; FC link %s</p>"
            "<h2>bring-up</h2><table border='1' cellpadding='4'>%s</table>"
            "%s<h2>endpoints</h2><ul>"
            "<li><a href='/health'>/health</a> (JSON: link, cams, bring-up, urls)</li>"
            "<li><a href='/api/cameras'>/api/cameras</a></li>"
            "<li>/api/camera/frame/cam1|cam2 &middot; /api/camera/stream/cam1|cam2</li>"
            "<li><a href='/api/fsm/status'>/api/fsm/status</a> &middot; "
            "<a href='/api/qr/status'>/api/qr/status</a> &middot; "
            "<a href='/api/mission/status'>/api/mission/status</a></li>"
            "<li>ws://this-host:%s/ws/telemetry (the UI's Pi link)</li>"
            "</ul><p>Paste <b>http://this-host:%s</b> into the UI's Pi link box.</p>"
            "</body>" % (time.time() - t0, hub.n_clients,
                         "OK" if (fc and fc.link_ok()) else "down",
                         rows,
                         ("<h2>errors</h2><ul>%s</ul>" % errs) if errs else "",
                         port, port))
        return Response(body, media_type="text/html; charset=utf-8")

    @app.get("/health")
    async def health():
        cams = {}
        try:
            cams = rig.status() if rig else {}
        except Exception as e:
            cams = {"error": repr(e)}
        link = False
        try:
            link = bool(fc.link_ok()) if fc else False
        except Exception:
            link = False
        out = {"ok": True, "status": "ok", "ts": time.time(),
               "uptime_s": round(time.time() - t0, 1),
               "link": link, "cams": cams,
               "ws_clients": hub.n_clients, "ws": hub.stats(),
               "streams": streams.status() if streams else {}}
        if mission is not None:
            try:
                out["mission"] = {"phase": mission.phase, "detail": mission.detail}
            except Exception:
                pass
        if fc is not None:
            try:
                fn = getattr(fc, "link_state", None)
                out["fc"] = fn() if callable(fn) else {
                    "device": getattr(fc, "device", None),
                    "mode": getattr(fc, "mode", None),
                    "armed": getattr(fc, "armed", None)}
            except Exception:
                pass
        if bringup is not None:
            try:
                out["bringup"] = bringup.snapshot()
                out["urls"] = bringup.urls
                out["ready"] = bringup.ok()
            except Exception as e:
                out["bringup"] = {"error": repr(e)}
        else:
            out["ready"] = link
        return out

    @app.get("/api/bringup")
    async def api_bringup():
        """Bring-up ledger on its own — the "why isn't my Pi link working"
        endpoint. Same data as /health's `bringup` block, minus the rest, so
        the UI diagnostics panel and tools/link_doctor.py can poll it cheaply
        while the FC/cameras are still coming up (or never did)."""
        if bringup is None:
            return {"ok": True, "stage": "ready", "state": {}, "detail": {},
                    "errors": [], "urls": [], "done": True,
                    "note": "no bring-up ledger (server started standalone)"}
        snap = bringup.snapshot()
        snap["ok"] = bool(bringup.ok())
        return snap

    @app.get("/api/cameras")
    async def cameras():
        return rig.status() if rig else {}

    @app.post("/api/cameras/rescan")
    async def cameras_rescan():
        """Hot-plug recovery: re-run camera auto-detect (CSI + V4L2/USB),
        start newly found cameras, sync the MJPEG streams. Returns the
        before/after assignment so the UI can refresh its tiles. The
        auto-rescan timer in main.py does this on its own; this endpoint
        is the manual trigger (UI button / SSH / link_doctor)."""
        if rig is None:
            return JSONResponse({"detail": "no camera rig"}, 404)
        before = {n: (c.kind, c.model, c.facing)
                  for n, c in (rig.cams or {}).items()}
        try:
            changed = rig.rescan()
        except Exception as e:
            return JSONResponse({"detail": "rescan failed: %r" % e}, 500)
        after = {n: (c.kind, c.model, c.facing)
                 for n, c in (rig.cams or {}).items()}
        n_streams = 0
        if streams is not None:
            try:
                n_streams = streams.sync(rig)
                streams.start_all()
            except Exception as e:
                log.warning("stream sync after rescan failed: %r", e)
        hub.push("log", {"level": "INFO",
                         "msg": "camera rescan: changed=%s cams=%s"
                                % (sorted(changed or []), sorted(after))})
        hub.push("event", {"type": "cameras", "rescan": True,
                           "added": sorted(set(after) - set(before)),
                           "removed": sorted(set(before) - set(after)),
                           "cams": sorted(after)})
        return {"ok": True, "changed": sorted(changed or []),
                "before": before, "after": after, "streams": n_streams}

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

        # ---- real_pi video-stuck fix ------------------------------------
        # OLD BUG: the generator only emitted bytes when a NEW frame seq
        # appeared. Before the first frame existed (CSI start-up, AE/AWB
        # settle, YOLO load — seconds on a Pi 5) it yielded NOTHING, so the
        # browser <img> sat on an open multipart connection with zero bytes:
        # the tile "always stuck" on a black box.
        #
        # FIX A: wait a BOUNDED time for the first real frame. Still nothing
        # -> 503, which makes the UI fall back to snapshot polling (which
        # also fails, but loudly retries) instead of a silent dead socket.
        if not await asyncio.to_thread(st.first_frame,
                                       streams.first_frame_wait_s):
            log.warning("%s: no frame within %.0fs — 503 (UI falls back to "
                        "polling); stream watchdog will retry", cam,
                        streams.first_frame_wait_s)
            return JSONResponse(
                {"detail": "no frame yet from %s (camera starting or dead; "
                           "watchdog will restart it)" % cam}, 503)
        # ------------------------------------------------------------------

        # Serve rate: never above the broadcaster's current (possibly
        # adaptive) fps — we can only re-send what was already encoded.
        eff_fps = min(max(int(fps or st.fps), 1), max(1, st.fps), 30)
        period = 1.0 / eff_fps

        from streamm1 import SendGate
        # Liveness: if no NEW frame arrives within this window (camera
        # lag/stall on the Pi 5), re-send the latest frame so the browser
        # always has fresh bytes + the freshest image and a dead camera
        # cannot produce a frozen tile. Duplicates cost at most one
        # JPEG/second — negligible on LAN, fine on LTE (the adaptive
        # ladder already cut the rate there).
        gate = SendGate(resend_after=max(3 * period, 1.0))

        async def gen():
            try:
                while True:
                    try:
                        if await request.is_disconnected():
                            break
                    except Exception:
                        pass
                    jpg, seq = st.latest()
                    if jpg is not None and gate.should_send(seq):
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
        brightness, contrast, saturation, sharpness, qr_boost_mode, qr_enhance_mode} -> {ok, cam, controls}.
        The UI sliders POST here (debounced). QR boost profiles also via this endpoint.
        Supports: qr_boost_profile / qr_boost_mode / profile for ISP, qr_enhance_mode / qr_software_enhance for SW."""
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
        # QR boost profile — if profile specified, apply ISP profile first (Kabaddi setting)
        profile = body.get("qr_boost_profile") or body.get("qr_boost_mode") or body.get("profile") or body.get("qr_profile")
        if profile:
            try:
                # apply_qr_profile handles ISP tuning for QR pop vs ground
                c.apply_qr_profile(profile)
            except Exception as e:
                log.warning("qr boost profile %r failed: %r", profile, e)
        # Software enhance mode
        if "qr_enhance_mode" in body or "qr_software_enhance" in body:
            try:
                sw = body.get("qr_enhance_mode")
                if sw is None:
                    # bool flag
                    en = body.get("qr_software_enhance")
                    if isinstance(en, bool):
                        c.qr_boost["qr_software_enhance"] = en
                    elif isinstance(en, str):
                        c.qr_boost["qr_software_enhance"] = True
                        c.qr_boost["qr_enhance_mode"] = en
                else:
                    if sw == "none":
                        c.qr_boost["qr_software_enhance"] = False
                    else:
                        c.qr_boost["qr_software_enhance"] = True
                        c.qr_boost["qr_enhance_mode"] = sw
            except Exception as e:
                log.debug("qr software enhance set failed: %r", e)
        applied = c.apply_tuning(body)
        return {"ok": True, "status": "ok", "cam": cam, "controls": applied, "qr_boost": dict(c.qr_boost)}

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

    # ---- contract aliases (docs/api-contract.md v2 Pi REST table) --------
    # The UI drives itself from the WS; these exist so a human with curl (or
    # the Windows laptop with no SSH) can read the same state, and so the Pi
    # answers the paths the contract publishes instead of 404-ing them.
    @app.get("/api/fsm/status")
    async def fsm_status():
        return _mission_snapshot()

    @app.get("/api/qr/status")
    async def qr_status():
        return _qr_snapshot()

    # ---- real_pi QR environment presets (fake-QR / window-grill fix) ----
    @app.get("/api/qr/presets")
    async def qr_presets():
        """List every preset the mission can switch to (same table the
        webcam bench tool uses, from config.yaml `qr_presets:`)."""
        try:
            from qr_filter import load_presets, preset_names
            return {"active": getattr(mission, "preset_name", "day"),
                    "names": preset_names(cfg),
                    "presets": load_presets(cfg),
                    "filter": (mission.qf.as_dict()
                               if hasattr(mission, "qf") else None)}
        except Exception as e:
            return JSONResponse({"detail": "presets: %r" % e}, 500)

    @app.post("/api/qr/preset")
    async def qr_preset_switch(req: Request):
        """Switch the live environment preset:
        POST /api/qr/preset {"preset": "dark"}
        Applies detector conf, tile grid, decode cadence, cue strictness,
        and the ISP boost profile to the bottom cam — no restart."""
        try:
            body = await req.json()
        except Exception:
            body = None
        if not isinstance(body, dict) or not body.get("preset"):
            return JSONResponse({"detail": 'want {"preset": "day|far|dark|..."}'}, 400)
        name = str(body.get("preset"))
        try:
            from qr_filter import normalize_preset_name
            name = normalize_preset_name(name)
        except Exception:
            pass
        try:
            applied = mission.set_preset(name)
        except Exception as e:
            return JSONResponse({"detail": "preset switch failed: %r" % e}, 500)
        if applied is None:
            from qr_filter import preset_names as _pn
            return JSONResponse(
                {"detail": "unknown preset %r (have %s)" % (name, _pn(cfg))}, 404)
        return {"ok": True, "preset": name, "applied": applied,
                "filter": mission.qf.as_dict()}

    # ---- UI-driven fence + alt — Pi owns fence, MP sees instantly ----
    @app.get("/api/fence")
    async def get_fence():
        """Return current fence from mission (Pi-owned) + FC readback if available."""
        fence = []
        try:
            fence = list(getattr(mission, "fence", []) or [])
        except Exception:
            fence = []
        fc_fence = []
        try:
            if fc and fc.link_ok():
                fc_fence = fc.read_fence(timeout=4.0)
        except Exception as e:
            log.debug("fc read_fence failed: %r", e)
        return {"ok": True, "fence": fence, "fc_fence": fc_fence, "count": len(fence), "fc_count": len(fc_fence)}

    @app.post("/api/fence")
    async def post_fence(req: Request):
        """UI pushes fence polygon to Pi — Pi stores + uploads to FC, MP sees instantly.

        Body: {vertices: [[lat,lon], ...] or [{lat,lon}, ...], clear: bool}
        Returns: {ok, count, fc_uploaded}
        """
        try:
            body = await req.json()
        except Exception:
            body = None
        if not isinstance(body, dict):
            return JSONResponse({"detail": "want JSON {vertices: [[lat,lon],...]}"}, 400)
        if body.get("clear"):
            try:
                if fc and fc.link_ok():
                    fc.clear_fence()
                if mission is not None:
                    mission.fence = []
                hub.push("event", {"type": "fence", "action": "clear", "source": "ui"})
                return {"ok": True, "cleared": True, "count": 0}
            except Exception as e:
                return JSONResponse({"detail": "clear failed: %r" % e}, 500)
        verts = body.get("vertices") or body.get("polygon") or body.get("fence") or []
        # Normalize: [[lat,lon]] or [{lat,lon}]
        norm = []
        for v in verts:
            try:
                if isinstance(v, dict):
                    lat = float(v.get("lat") or v.get("y") or v.get("latitude"))
                    lon = float(v.get("lon") or v.get("x") or v.get("longitude"))
                elif isinstance(v, (list, tuple)) and len(v) >= 2:
                    lat, lon = float(v[0]), float(v[1])
                else:
                    continue
                norm.append((lat, lon))
            except Exception:
                continue
        if len(norm) < 3:
            return JSONResponse({"detail": "fence needs >=3 vertices, got %d" % len(norm)}, 400)
        # Store in mission
        try:
            if mission is not None:
                mission.fence = list(norm)
        except Exception as e:
            log.warning("mission.fence set failed: %r", e)
        # Upload to FC if link ok — MP sees instantly because FC broadcasts
        fc_ok = False
        fc_err = None
        try:
            if fc and fc.link_ok():
                fc.upload_fence(norm, timeout=10.0)
                fc_ok = True
            else:
                fc_err = "no FC link"
        except Exception as e:
            fc_err = str(e)
            log.warning("fc upload_fence failed: %r", e)
        hub.push("event", {"type": "fence", "action": "upload", "count": len(norm), "fc_ok": fc_ok, "source": "ui"})
        hub.push("fsm", _mission_snapshot())
        return {"ok": True, "count": len(norm), "fc_uploaded": fc_ok, "fc_error": fc_err, "fence": norm}

    @app.delete("/api/fence")
    async def delete_fence():
        try:
            if fc and fc.link_ok():
                fc.clear_fence()
            if mission is not None:
                mission.fence = []
            hub.push("event", {"type": "fence", "action": "clear", "source": "ui"})
            return {"ok": True, "cleared": True}
        except Exception as e:
            return JSONResponse({"detail": "clear failed: %r" % e}, 500)

    @app.get("/api/config")
    async def get_config():
        """Return current flight config — max_alt, sweep_alt, etc."""
        try:
            max_alt = getattr(mission, "max_alt_m", None) if mission else None
            sweep_alt = getattr(mission, "sweep_alt", None) if mission else None
            cfg = {}
            if mission and hasattr(mission, "cfg"):
                cfg = mission.cfg or {}
            flight = cfg.get("flight", {}) if isinstance(cfg, dict) else {}
            mission_cfg = cfg.get("mission", {}) if isinstance(cfg, dict) else {}
            return {"ok": True, "max_alt_m": max_alt, "sweep_alt_m": sweep_alt,
                    "flight": flight, "mission": mission_cfg,
                    "fence_count": len(getattr(mission, "fence", []) or []) if mission else 0}
        except Exception as e:
            return {"ok": False, "error": repr(e)}

    @app.post("/api/config")
    async def post_config(req: Request):
        """UI sets alt — Pi owns alt, MP sees via FC params or mission status.

        Body: {max_alt_m: 15, sweep_alt_m: 12} or {flight: {max_alt_m: 15}}
        """
        try:
            body = await req.json()
        except Exception:
            body = None
        if not isinstance(body, dict):
            return JSONResponse({"detail": "want JSON {max_alt_m: 15, sweep_alt_m: 12}"}, 400)
        max_alt = body.get("max_alt_m")
        if max_alt is None:
            flight = body.get("flight") or {}
            if isinstance(flight, dict):
                max_alt = flight.get("max_alt_m")
        sweep_alt = body.get("sweep_alt_m")
        if sweep_alt is None:
            mission_cfg = body.get("mission") or {}
            if isinstance(mission_cfg, dict):
                sweep_alt = mission_cfg.get("sweep_alt_m")
        updated = {}
        try:
            if max_alt is not None:
                v = float(max_alt)
                # Clamp 1..50
                v = max(1.0, min(50.0, v))
                if mission is not None:
                    mission.max_alt_m = v
                    # Also update cfg for persistence in status
                    try:
                        if hasattr(mission, "cfg") and isinstance(mission.cfg, dict):
                            mission.cfg.setdefault("flight", {})["max_alt_m"] = v
                            mission.cfg.setdefault("mission", {})["max_alt_m"] = v
                    except Exception:
                        pass
                # Try set param on FC so MP sees instantly — WPNAV_SPEED or WP_SPD or FENCE_ALT_MAX
                try:
                    if fc and fc.link_ok():
                        # Set FENCE_ALT_MAX if available, else just log
                        try:
                            fc.set_param("FENCE_ALT_MAX", v, timeout=3.0)
                        except Exception:
                            pass
                except Exception:
                    pass
                updated["max_alt_m"] = v
            if sweep_alt is not None:
                v = float(sweep_alt)
                if mission is not None:
                    # Clamp to max_alt
                    max_a = getattr(mission, "max_alt_m", 15.0) or 15.0
                    v = max(1.0, min(max_a, v))
                    mission.sweep_alt = v
                    try:
                        if hasattr(mission, "cfg") and isinstance(mission.cfg, dict):
                            mission.cfg.setdefault("mission", {})["sweep_alt_m"] = v
                    except Exception:
                        pass
                updated["sweep_alt_m"] = v
            hub.push("event", {"type": "config", "updated": updated, "source": "ui"})
            hub.push("fsm", _mission_snapshot())
            return {"ok": True, "updated": updated, "max_alt_m": getattr(mission, "max_alt_m", None) if mission else None}
        except Exception as e:
            return JSONResponse({"detail": "config update failed: %r" % e}, 500)

    @app.post("/api/fsm/start")
    async def fsm_start():
        """Contract's debug start. The Pi mission auto-starts with the
        process, so this is the manual-takeover nudge (bench/SSH only)."""
        if mission is None:
            return JSONResponse({"detail": "mission not started"}, 503)
        mission.request_takeover()
        return {"status": "started", "detail": "manual takeover requested",
                "fsm": _mission_snapshot()}

    @app.post("/api/fsm/abort")
    async def fsm_abort():
        if mission is None:
            return JSONResponse({"detail": "mission not started"}, 503)
        mission.request_abort()
        return {"status": "aborted", "detail": "RTL + stop requested",
                "fsm": _mission_snapshot()}

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
        log.info("UI websocket connected (%d client(s)) from %s",
                 hub.n_clients, getattr(getattr(ws, "client", None), "host", "?"))
        # History burst: a browser joining mid-mission gets the current
        # picture at once (system + fsm + last qr/events) instead of an
        # empty panel until the next phase change.
        try:
            if _bringup is not None:
                await ws.send_text(json.dumps(
                    {"channel": "system", "t": time.time(),
                     "data": _bringup.system_stats()}))
            await ws.send_text(json.dumps(
                {"channel": "fsm", "t": time.time(), "data": _mission_snapshot()}))
            if bringup is not None:
                await ws.send_text(json.dumps(
                    {"channel": "log", "t": time.time(),
                     "data": {"level": "INFO", "msg": bringup.one_line()}}))
            for env in hub.recent(("qr", "event")):  # system/fsm sent fresh above
                await ws.send_text(env)
        except Exception as e:
            log.debug("replay burst failed: %r", e)
        try:
            while True:
                await ws.receive_text()  # ignore inbound; link is read-only
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            hub.drop(ws)
            log.info("UI websocket closed (%d client(s) left)", hub.n_clients)

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
