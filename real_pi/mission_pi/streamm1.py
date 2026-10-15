"""MJPEG UI streaming: smooth per-camera motion-JPEG for the mission-ui tiles.

WHY THIS FILE EXISTS (real_pi, 2026-09-17 rewrite)
---------------------------------------------------
The Pi link's video "always stuck" bug had three compounding causes:

1. The HTTP generator (server.py) only emitted bytes when a NEW frame seq
   appeared. On a Pi 5 the first frame can take seconds (CSI start, AE/AWB
   settle, YOLO-tile load) — so the browser <img> opened the multipart
   connection and received ZERO bytes: a permanently black tile.
2. No watchdog: if a capture thread died (camera unplug, libcamera crash),
   the seq simply stopped and nothing recovered.
3. Fixed 12 fps / 960 px / q70: on a Pi 5 busy with 3x3 YOLO tiles the
   encoder starves, frames back up and the stream visibly freezes.

Fixes implemented here + in server.py:
  * the generator now serves the LATEST JPEG every tick, even if the seq
    did not change — the browser always has bytes + the freshest frame;
  * CamStream WATCHDOG: camera frames older than `watchdog_s` -> the camera
    thread is restarted automatically (bounded, with cooldown), logged, and
    reported at /api/cameras + /health — a stuck camera self-heals;
  * ADAPTIVE quality: the encoder watches its own encode time; when it
    exceeds 60% of the tick period it degrades one step (fps, then width,
    then quality) and recovers when headroom returns. The stream degrades
    gracefully instead of freezing;
  * `first_frame_wait_s` in server.py: the stream waits (bounded) for the
    first real frame and answers 503 otherwise, so the UI's snapshot
    polling fallback engages instead of a dead socket.

Everything is stdlib; cameras are duck-typed (name/running/latest/jpeg/
start/stop/size). Wire-up (main.py) is unchanged:

    streams = StreamManager(rig, cfg.get("stream", {}))
    streams.start_all()
    app = create_app(rig, mission, fc, streams)

Endpoint (server.py): GET /api/camera/stream/<cam>?fps=12 ->
multipart/x-mixed-replace. The UI points each tile's <img> at it; any plain
browser can open the URL directly.
"""
import logging
import threading
import time

log = logging.getLogger("stream")

DEFAULTS = {"fps": 12, "width": 960, "quality": 70,
            "watchdog_s": 6.0,          # cam frame older than this -> restart
            "adaptive": True,           # auto fps/width/quality under load
            "min_fps": 4, "min_width": 480, "min_quality": 40,
            "first_frame_wait_s": 8.0}  # server: wait for first frame, else 503

# Degradation ladder: (fps, width, quality) — each step cheaper to encode.
_LADDER = [(12, 960, 70), (8, 960, 65), (6, 720, 60),
           (5, 640, 55), (4, 480, 45)]


def _ladder_for(base_fps, base_width, base_quality):
    """Ladder that starts at the configured point and steps down to floors."""
    step = [(int(base_fps), int(base_width), int(base_quality))]
    for fps, w, q in _LADDER:
        cand = (min(fps, step[-1][0]), min(w, step[-1][1]), min(q, step[-1][2]))
        if cand != step[-1]:
            step.append(cand)
    floors = (max(1, int(min(base_fps, 4))), max(320, int(min(base_width, 480))),
              max(30, int(min(base_quality, 45))))
    last = (floors[0], floors[1], floors[2])
    if step[-1] != last:
        step.append(last)
    return step


class SendGate:
    """The video-stuck fix, distilled to a pure decision (unit-testable).

    The OLD endpoint emitted a frame only when a NEW seq appeared — before
    the first frame (or while the camera thread was lagging) it emitted
    NOTHING, so the browser <img> sat on an open multipart connection with
    zero bytes: a permanently frozen tile.

    SendGate: emit on a new seq, AND re-emit the latest frame whenever
    NOTHING has been sent for `resend_after` seconds — the browser always
    has bytes + the freshest image, and a stuck camera cannot produce a
    frozen tile. At most one duplicate JPEG per window, so a healthy
    12 fps stream is untouched (it never hits the resend branch).
    """

    def __init__(self, resend_after=1.0):
        self.resend_after = max(0.25, float(resend_after))
        self.last_seq = -1
        self.last_sent = 0.0

    def should_send(self, seq, now=None):
        now = time.time() if now is None else now
        if seq != self.last_seq or (now - self.last_sent) >= self.resend_after:
            self.last_seq = seq
            self.last_sent = now
            return True
        return False


class CamStream:
    """One camera's shared MJPEG bytes: encode once per tick, serve many.

    A single background thread owns the encode; ALL HTTP stream clients share
    the same bytes, so N browser tiles cost 1 encode per tick, not N.
    """

    def __init__(self, cam, fps=12, width=960, quality=70,
                 watchdog_s=6.0, adaptive=True, min_fps=4, min_width=480,
                 min_quality=40):
        self.cam = cam
        self.base_fps = min(30, max(1, int(fps or 12)))
        self.base_width = min(1920, max(320, int(width or 960)))
        self.base_quality = min(95, max(30, int(quality or 70)))
        self.fps = self.base_fps
        self.width = self.base_width
        self.quality = self.base_quality
        self.watchdog_s = max(2.0, float(watchdog_s or DEFAULTS["watchdog_s"]))
        self.adaptive = bool(adaptive)
        self.ladder = _ladder_for(self.fps, self.width, self.quality)
        self.min_fps = max(1, int(min_fps or 4))
        self.min_width = max(320, int(min_width or 480))
        self.min_quality = max(30, int(min_quality or 40))
        self._lock = threading.Lock()
        self._jpg = None
        self._seq = 0
        self._ts = 0.0
        self._stop = threading.Event()
        self._thread = None
        self.running = False
        # stats / adaptivity state
        self._enc_ms = 0.0           # EWMA of encode time
        self._restarts = 0
        self._last_restart = 0.0
        self._restart_cooldown_s = 10.0
        self._level = 0              # index into self.ladder (0 = full)
        self._good_since = None      # ts since which encode was light
        self._miss_frames = 0

    # ------------------------------------------------------------ lifecycle
    def start(self):
        if self.running:
            return
        self._stop.clear()
        name = getattr(self.cam, "name", "?")
        self._thread = threading.Thread(target=self._loop, name="stream-" + name,
                                        daemon=True)
        self._thread.start()
        self.running = True
        log.info("%s stream: %dfps %dw q%d (watchdog %.0fs adaptive=%s)",
                 name, self.fps, self.width, self.quality, self.watchdog_s,
                 self.adaptive)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.running = False

    # -------------------------------------------------------------- watch
    def _watchdog(self):
        """Restart a camera whose frames are stale. Bounded + cooled down."""
        cam = self.cam
        if not getattr(cam, "running", False):
            return  # the mission/broker restart policy handles dead cams
        _, ts, _ = cam.latest()
        if not ts:
            # camera just started; give it one full watchdog window
            return
        age = time.time() - ts
        if age < self.watchdog_s:
            return
        now = time.time()
        if now - self._last_restart < self._restart_cooldown_s:
            return
        self._last_restart = now
        self._restarts += 1
        log.warning("%s: camera frame %.1fs old (watchdog %.1fs) — "
                    "restarting camera thread #%d",
                    getattr(cam, "name", "?"), age, self.watchdog_s,
                    self._restarts)
        try:
            cam.stop()
        except Exception as e:
            log.debug("watchdog stop failed: %r", e)
        time.sleep(0.3)
        try:
            cam.start()
        except Exception as e:
            log.warning("%s: watchdog restart FAILED (%r) — will retry "
                        "after cooldown", getattr(cam, "name", "?"), e)

    # ------------------------------------------------------------- adapt
    def _adapt(self, enc_s, period):
        """Step the encode ladder up/down based on encoder load."""
        if not self.adaptive:
            return
        load = enc_s / period if period > 0 else 0.0
        now = time.time()
        if load > 0.60:
            self._good_since = None
            if self._level < len(self.ladder) - 1:
                self._level += 1
                self.fps, self.width, self.quality = self.ladder[self._level]
                log.info("%s: encoder overloaded (%.0fms/%.0fms tick) — "
                         "degraded to %dfps %dw q%d (level %d)",
                         getattr(self.cam, "name", "?"), enc_s * 1000,
                         period * 1000, self.fps, self.width, self.quality,
                         self._level)
        else:
            if self._good_since is None:
                self._good_since = now
            elif (now - self._good_since) > 8.0 and self._level > 0:
                self._level -= 1
                self.fps, self.width, self.quality = self.ladder[self._level]
                self._good_since = None
                log.info("%s: encoder headroom back — restored to %dfps %dw q%d "
                         "(level %d)", getattr(self.cam, "name", "?"),
                         self.fps, self.width, self.quality, self._level)

    # --------------------------------------------------------------- loop
    def _loop(self):
        while not self._stop.is_set():
            period = 1.0 / self.fps
            t0 = time.time()
            jpg = None
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
            enc_s = time.time() - t0
            if jpg:
                with self._lock:
                    self._jpg = jpg
                    self._seq += 1
                    self._ts = time.time()
                self._miss_frames = 0
            else:
                self._miss_frames += 1
                if self._miss_frames in (30, 120, 300):
                    log.warning("%s: %d consecutive empty encodes — camera "
                                "likely dead (watchdog %.1fs)",
                                getattr(self.cam, "name", "?"),
                                self._miss_frames, self.watchdog_s)
            # EWMA of encode time + adaptive step
            if self._enc_ms == 0.0:
                self._enc_ms = enc_s
            else:
                self._enc_ms = 0.7 * self._enc_ms + 0.3 * enc_s
            self._adapt(self._enc_ms, period)
            self._watchdog()
            dt = period - (time.time() - t0)
            self._stop.wait(max(0.0, dt))

    # ----------------------------------------------------------- consumers
    def latest(self):
        """(jpg_bytes_or_None, seq). seq rises once per encoded tick."""
        with self._lock:
            return self._jpg, self._seq

    def first_frame(self, timeout):
        """Block up to `timeout` s for the first encoded frame. True on
        success — used by the HTTP endpoint so the UI fallback engages
        instead of a socket that delivers zero bytes forever."""
        deadline = time.time() + max(0.5, float(timeout))
        while time.time() < deadline:
            jpg, _ = self.latest()
            if jpg:
                return True
            self._stop.wait(0.1)
        return False

    def status(self):
        with self._lock:
            jpg, seq, ts = self._jpg, self._seq, self._ts
        return {
            "fps": self.fps, "width": self.width, "quality": self.quality,
            "seq": seq, "bytes": len(jpg) if jpg else 0,
            "age_s": round(time.time() - ts, 2) if ts else None,
            "running": self.running,
            "encode_ms": round(self._enc_ms * 1000, 1),
            "adaptive_level": self._level,
            "adaptive": self.adaptive,
            "restarts": self._restarts,
            "watchdog_s": self.watchdog_s,
        }


class StreamManager:
    """Owns the per-camera CamStreams; tolerates missing/extra cameras."""

    def __init__(self, rig, cfg=None):
        cfg = cfg or {}
        try:
            fps = int(cfg.get("fps", DEFAULTS["fps"]))
            width = int(cfg.get("width", DEFAULTS["width"]))
            quality = int(cfg.get("quality", DEFAULTS["quality"]))
            watchdog_s = float(cfg.get("watchdog_s",
                                       DEFAULTS["watchdog_s"]))
            adaptive = bool(cfg.get("adaptive", DEFAULTS["adaptive"]))
            min_fps = int(cfg.get("min_fps", DEFAULTS["min_fps"]))
            min_width = int(cfg.get("min_width", DEFAULTS["min_width"]))
            min_quality = int(cfg.get("min_quality",
                                      DEFAULTS["min_quality"]))
        except Exception:
            (fps, width, quality, watchdog_s, adaptive,
             min_fps, min_width, min_quality) = (
                DEFAULTS["fps"], DEFAULTS["width"], DEFAULTS["quality"],
                DEFAULTS["watchdog_s"], True,
                DEFAULTS["min_fps"], DEFAULTS["min_width"],
                DEFAULTS["min_quality"])
        self.fps, self.width, self.quality = fps, width, quality
        self.watchdog_s, self.adaptive = watchdog_s, adaptive
        self.min_fps, self.min_width, self.min_quality = (
            min_fps, min_width, min_quality)
        self.first_frame_wait_s = float(
            cfg.get("first_frame_wait_s", DEFAULTS["first_frame_wait_s"]))
        cams = getattr(rig, "cams", None) or {}
        self.streams = {name: self._mk(name, cam)
                        for name, cam in cams.items()}

    def _mk(self, name, cam):
        return CamStream(cam, self.fps, self.width, self.quality,
                         watchdog_s=self.watchdog_s,
                         adaptive=self.adaptive, min_fps=self.min_fps,
                         min_width=self.min_width,
                         min_quality=self.min_quality)

    def sync(self, rig):
        """Adopt cameras discovered after construction (hotplug-safe).

        main.py builds the StreamManager BEFORE the cameras are detected
        (the HTTP server has to come up first), so `self.streams` starts
        empty. Idempotent: existing streams are kept, new cams get one,
        cams that vanished are stopped and dropped. Returns the current
        stream count.
        """
        cams = getattr(rig, "cams", None) or {}
        for name in [n for n in self.streams if n not in cams]:
            try:
                self.streams[name].stop()
            except Exception:
                pass
            self.streams.pop(name, None)
        for name, cam in cams.items():
            if name not in self.streams:
                self.streams[name] = self._mk(name, cam)
                log.info("stream adopted new camera %s", name)
        return len(self.streams)

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
