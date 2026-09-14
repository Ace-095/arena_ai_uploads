"""MJPEG UI streaming: smooth per-camera motion-JPEG for the mission-ui tiles.

Why this exists: the UI used to poll GET /api/camera/frame/<cam> once a
second (1 fps snapshots, after a 6 s WebRTC timeout that can never succeed
— v1 serves no WebRTC). That is the lag/chop: tiles can never be smooth on
1 Hz snapshots. This module runs ONE background thread per camera that
JPEG-encodes the latest captured frame at a capped rate (default 12 fps,
960 px wide, q70) and shares those bytes with ALL HTTP stream clients, so
N browser tiles cost 1 encode per tick, not N.

Resolution split (deliberate): capture + detection + decode stay FULL-res
(the A3 panel needs every pixel — see SIM_GUIDE.md section 10.3). Only the
human-facing UI stream is downscaled, exactly like the old snapshot path.

Wire-up (main.py):
    streams = StreamManager(rig, cfg.get("stream", {}))
    streams.start_all()
    app = create_app(rig, mission, fc, streams)   # streams=None disables
    ...
    finally: streams.stop_all()

Endpoint (server.py): GET /api/camera/stream/<cam>?fps=12 ->
multipart/x-mixed-replace. The UI points each tile's <img> at it (MJPEG
first, snapshot polling as fallback); any plain browser can open the URL
directly too. No third-party imports: stdlib only, cameras are duck-typed.
"""
import logging
import threading
import time

log = logging.getLogger("stream")

DEFAULTS = {"fps": 12, "width": 960, "quality": 70}


class CamStream:
    """One camera's shared MJPEG bytes: encode once per tick, serve many."""

    def __init__(self, cam, fps=12, width=960, quality=70):
        self.cam = cam
        self.fps = min(30, max(1, int(fps or 12)))
        self.width = min(1920, max(320, int(width or 960)))
        self.quality = min(95, max(30, int(quality or 70)))
        self._lock = threading.Lock()
        self._jpg = None
        self._seq = 0
        self._ts = 0.0
        self._stop = threading.Event()
        self._thread = None
        self.running = False

    def start(self):
        if self.running:
            return
        self._stop.clear()
        name = getattr(self.cam, "name", "?")
        self._thread = threading.Thread(target=self._loop, name="stream-" + name,
                                        daemon=True)
        self._thread.start()
        self.running = True
        log.info("%s stream: %dfps %dw q%d", name, self.fps, self.width,
                 self.quality)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.running = False

    def _loop(self):
        period = 1.0 / self.fps
        while not self._stop.is_set():
            t0 = time.time()
            try:
                jpg = self.cam.jpeg(max_width=self.width, quality=self.quality)
            except TypeError:
                # old camera objects without the quality kwarg — width only
                try:
                    jpg = self.cam.jpeg(max_width=self.width)
                except Exception:
                    jpg = None
            except Exception:
                jpg = None
            if jpg:
                with self._lock:
                    self._jpg = jpg
                    self._seq += 1
                    self._ts = time.time()
            dt = period - (time.time() - t0)
            self._stop.wait(max(0.0, dt))

    def latest(self):
        """(jpg_bytes_or_None, seq). seq rises once per encoded tick."""
        with self._lock:
            return self._jpg, self._seq

    def status(self):
        with self._lock:
            jpg, seq, ts = self._jpg, self._seq, self._ts
        return {"fps": self.fps, "width": self.width, "quality": self.quality,
                "seq": seq, "bytes": len(jpg) if jpg else 0,
                "age_s": round(time.time() - ts, 2) if ts else None,
                "running": self.running}


class StreamManager:
    """Owns the per-camera CamStreams; tolerates missing/extra cameras."""

    def __init__(self, rig, cfg=None):
        cfg = cfg or {}
        try:
            fps = int(cfg.get("fps", DEFAULTS["fps"]))
            width = int(cfg.get("width", DEFAULTS["width"]))
            quality = int(cfg.get("quality", DEFAULTS["quality"]))
        except Exception:
            fps, width, quality = (DEFAULTS["fps"], DEFAULTS["width"],
                                   DEFAULTS["quality"])
        self.fps, self.width, self.quality = fps, width, quality
        cams = getattr(rig, "cams", None) or {}
        self.streams = {name: CamStream(cam, fps, width, quality)
                        for name, cam in cams.items()}

    def describe(self):
        return "%dfps %dw q%d x%d" % (self.fps, self.width, self.quality,
                                      len(self.streams))

    def start_all(self):
        for s in self.streams.values():
            try:
                s.start()
            except Exception as e:
                log.error("stream start failed: %r", e)

    def stop_all(self):
        for s in self.streams.values():
            try:
                s.stop()
            except Exception:
                pass

    def get(self, name):
        return self.streams.get(name)

    def status(self):
        return {n: s.status() for n, s in self.streams.items()}
