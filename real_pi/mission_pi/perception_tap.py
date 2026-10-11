"""Attach the perception pipeline to the running stack — WITHOUT editing it.

The constraint
--------------
`mission.py` is the flight-tested artifact. Its `_worker()` loop already does
capture -> (tiled) YOLO -> `qr_filter` -> decode -> consensus -> cue, and the
perception work in this change is explicitly *additive and observational*: it
must add tracking and a perception FSM without altering one line of the flight
path. Editing `_worker` to call a tracker would put a flight-tested loop back
under review for a change that is supposed to be observation-only.

The seam
--------
`Mission._worker()` imports its collaborators **inside the loop body**:

    def _worker(self, cam):
        from decoder import decode_frame
        from detector import detect_tiles
        ...
            boxes = detect_tiles(self.detector, frame, ...)

so `detect_tiles` and `decode_frame` are attribute lookups on the module at
every call. Replacing those two module attributes therefore intercepts exactly
the values the mission computed — the merged full-frame boxes and the payload —
and is the only place where (frame, boxes, payload) all exist together. This
module installs pass-through wrappers there. They observe and then return the
original object unchanged; the mission sees byte-identical results.

One update per frame, not per decode
------------------------------------
The mission may call `decode_frame` several times per frame (up to three
boxes, plus a periodic full-frame attempt). Feeding each of those to the
tracker as a separate observation would inflate its miss counters and make a
track look lost while the drone is staring straight at the panel. So
observations are **buffered per frame** and flushed once, when the next
frame's boxes arrive (or by a watchdog timer if the stream stops, since a
stalled camera is itself one of the recovery cases).

Failure isolation
-----------------
Every wrapper is wrapped again in try/except and every pipeline call is
best-effort: if perception raises, the original result is still returned and
the mission is unaffected. `perception.enabled: false` makes `install()` a
no-op, so the whole feature can be switched off in the field with a config
edit and a restart.
"""
import logging
import threading
import time

import detector as _detector
import decoder as _decoder
from perception import PerceptionPipeline, perception_config

log = logging.getLogger("perception_tap")

#: Depth counter for "we are inside detect_tiles". Tile-level `detect()` calls
#: must not be observed as if they were full frames (their coordinates are
#: tile-local, so treating them as frame-local would scatter the tracker).
_TLS = threading.local()


def _tiles_depth():
    return getattr(_TLS, "tiles_depth", 0)


class ObservationBuffer(object):
    """Collects one frame's boxes + payloads and flushes them as one update."""

    def __init__(self, pipeline, flush_timeout_s=0.5):
        self.pipeline = pipeline
        self.flush_timeout_s = float(flush_timeout_s)
        self._lock = threading.Lock()
        self._frame = None
        self._boxes = None
        self._payload = None
        self._ts = None
        self._camera = None
        self.flushes = 0
        self.errors = 0

    def note_boxes(self, frame, boxes, camera=None, ts=None):
        """A new frame's boxes arrived -> flush the previous frame first."""
        with self._lock:
            self._flush_locked()
            self._frame, self._boxes = frame, list(boxes or [])
            self._ts = time.time() if ts is None else float(ts)
            self._camera = camera

    def note_payload(self, payload):
        with self._lock:
            if payload and not self._payload:
                self._payload = payload

    def note_error(self, err, kind="detector"):
        try:
            self.pipeline.note_error(err, kind)
        except Exception:
            pass

    def pending(self):
        with self._lock:
            return self._boxes is not None or self._payload is not None

    def age_s(self, now=None):
        with self._lock:
            if self._ts is None:
                return 0.0
            return max(0.0, (time.time() if now is None else now) - self._ts)

    def flush(self, force=False):
        with self._lock:
            return self._flush_locked(force=force)

    def _flush_locked(self, force=False):
        if self._boxes is None and self._payload is None and not force:
            return None
        # If the buffer is older than the timeout it is the last frame we have
        # (camera stalled or the run ended) — flush it rather than hold it.
        frame, boxes, payload = self._frame, self._boxes, self._payload
        ts, camera = self._ts, self._camera
        self._frame, self._boxes, self._payload = None, None, None
        if boxes is None and payload is None:
            return None
        try:
            self.flushes += 1
            return self.pipeline.update(frame=frame, boxes=boxes or [],
                                        payload=payload, ts=ts, camera=camera,
                                        source="tap")
        except Exception as e:
            self.errors += 1
            log.debug("perception flush failed (mission unaffected): %r", e)
            return None


class PerceptionTap(object):
    """Detector decorator: identical behaviour, plus observation.

    `name` is forwarded **unchanged** on purpose — `Mission._worker` decides
    whether to run tiled inference from `self.detector.name != "classical"`,
    so a tap that renamed anything would silently change the inference path.
    """

    def __init__(self, inner, buffer):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "_buffer", buffer)

    # -- exact passthrough ---------------------------------------------
    @property
    def name(self):
        return getattr(self._inner, "name", "unknown")

    def __getattr__(self, item):
        # conf_thr, iou_thr, fallback_factory, using(), anything a preset or
        # a bench tool pokes at, reaches the real detector untouched.
        # Guard against recursion during unpickling / a half-built object.
        if item.startswith("_"):
            raise AttributeError(item)
        return getattr(self._inner, item)

    def set_thresholds(self, conf_thr=None, iou_thr=None):
        return self._inner.set_thresholds(conf_thr=conf_thr, iou_thr=iou_thr)

    def close(self):
        try:
            return self._inner.close()
        except Exception as e:
            log.debug("inner detector close failed: %r", e)

    def detect(self, frame_bgr):
        try:
            boxes = self._inner.detect(frame_bgr)
        except Exception as e:
            self._buffer.note_error(e, "detector")
            raise
        # Inside detect_tiles these coordinates are tile-local; the module
        # wrapper observes the merged full-frame result instead.
        if _tiles_depth() == 0:
            self._buffer.note_boxes(frame_bgr, boxes)
        return boxes


class _Installer(object):
    """Owns the monkey-patches so they can be cleanly reverted."""

    def __init__(self):
        self._orig = {}
        self._timer = None
        self.buffer = None
        self.pipeline = None

    def install(self, cfg, camera=None, own_decode=False, flush_timeout_s=0.5,
                timer=True):
        sec = perception_config(cfg)
        if not sec["enabled"]:
            log.info("[perception] disabled in config — tap not installed")
            return None
        self.pipeline = PerceptionPipeline(sec)
        self.buffer = ObservationBuffer(self.pipeline, flush_timeout_s)
        if own_decode:
            self.pipeline.cfg["decode_own"] = True

        if "detect_tiles" not in self._orig:
            self._orig["detect_tiles"] = _detector.detect_tiles
            _detector.detect_tiles = _wrap_detect_tiles(
                self._orig["detect_tiles"], self.buffer)
        if "decode_frame" not in self._orig:
            self._orig["decode_frame"] = _decoder.decode_frame
            _decoder.decode_frame = _wrap_decode_frame(
                self._orig["decode_frame"], self.buffer)
        if timer and self._timer is None:
            self._timer = threading.Thread(
                target=self._flush_loop, name="perception-flush", daemon=True)
            self._timer.start()
        log.info("[perception] tap installed (flush=%.1fs, decode_own=%s)",
                 flush_timeout_s, bool(own_decode))
        return self.buffer

    def wrap_detector(self, det):
        """Wrap a resolved detector so its non-tiled detect() is observed."""
        if det is None or self.buffer is None:
            return det
        if isinstance(det, PerceptionTap):
            return det
        return PerceptionTap(det, self.buffer)

    def uninstall(self):
        for mod, attr in ((_detector, "detect_tiles"), (_decoder, "decode_frame")):
            if attr in self._orig:
                setattr(mod, attr, self._orig.pop(attr))
        if self.buffer is not None:
            self.buffer.flush(force=True)
        log.info("[perception] tap removed")

    def _flush_loop(self):
        """Publish a frame whose successor never arrived (stalled camera)."""
        while True:
            time.sleep(0.25)
            try:
                b = self.buffer
                if b is not None and b.pending() and b.age_s() > b.flush_timeout_s:
                    b.flush(force=True)
            except Exception:
                pass


def _wrap_detect_tiles(orig, buffer):
    def wrapper(detector, frame_bgr, *a, **kw):
        prev = getattr(_TLS, "tiles_depth", 0)
        _TLS.tiles_depth = prev + 1
        try:
            boxes = orig(detector, frame_bgr, *a, **kw)
        except Exception as e:
            buffer.note_error(e, "detector")
            raise
        finally:
            _TLS.tiles_depth = prev
        try:
            buffer.note_boxes(frame_bgr, boxes)
        except Exception as e:                      # observation must not break
            log.debug("observe tiles failed: %r", e)
        return boxes
    wrapper.__name__ = getattr(orig, "__name__", "detect_tiles")
    return wrapper


def _wrap_decode_frame(orig, buffer):
    def wrapper(frame_bgr, *a, **kw):
        out = orig(frame_bgr, *a, **kw)
        try:
            if out:
                buffer.note_payload(out)
        except Exception as e:
            log.debug("observe payload failed: %r", e)
        return out
    wrapper.__name__ = getattr(orig, "__name__", "decode_frame")
    return wrapper


#: module-level installer (one tap per process)
INSTALLER = _Installer()


def install(cfg, **kw):
    """Install the tap. Safe to call twice; returns the ObservationBuffer."""
    try:
        return INSTALLER.install(cfg, **kw)
    except Exception as e:
        # Perception is an addition; it must never be the reason a mission
        # fails to start. Report loudly and carry on with detection intact.
        log.error("[perception] install failed — running WITHOUT the "
                  "perception tap: %r", e)
        INSTALLER.uninstall()
        return None


def uninstall():
    INSTALLER.uninstall()


def wrap_detector(det):
    return INSTALLER.wrap_detector(det)


def pipeline():
    return INSTALLER.pipeline


def snapshot():
    """Structured perception result, or a disabled marker."""
    p = INSTALLER.pipeline
    if p is None:
        return {"state": "DISABLED", "enabled": False,
                "perception_only": True, "drives_flight": False}
    return p.snapshot()
