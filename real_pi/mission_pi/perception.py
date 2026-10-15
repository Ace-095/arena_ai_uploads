"""Perception-level state machine + image-quality gate for the QR hunt.

What this is (and is not)
-------------------------
`mission.py` owns *flight*: SWEEP_GRID, TRACK, APPROACH, TRANSMIT. This module
owns **perception** only, and its whole output is a structured snapshot:

    SEARCHING -> CANDIDATE_DETECTED -> TARGET_TRACKING -> QR_DECODING
              -> PAYLOAD_VALIDATED -> RESULT_REPORTED

with recovery for target lost, decode failure, low image quality, low
detection confidence, camera errors, detector/Hailo errors, invalid payloads
and duplicates. Nothing here commands the vehicle, and no code path returns a
"found it" that isn't backed by a *validated payload* — a bounding box alone
can only ever reach `TARGET_TRACKING` (see `_step`).

Why a second state machine was needed
-------------------------------------
The flight FSM already has phases, so the temptation is to treat those as the
perception state. They are not interchangeable: the flight phase says where
the *drone* is in the mission, while the reliability questions are all
perception-local — "is the thing I am looking at the same object as last
frame?", "is this frame too blurred to be worth the decode CPU?", "have I seen
this payload enough times to believe it?", "did the target just disappear, or
did one frame fail?". Keeping that state separate is also what lets it be
observed without touching the flight path at all.

Decode economy
--------------
`QR_DECODING` is entered on a **cadence**, not every frame, and a decode is
skipped outright when the frame cannot support it (blur below floor, ROI too
small for the module budget). On a Pi 5 the decode is the expensive CPU step
after the 9-tile inference, so not spending it on a blurred frame — and not
spending it again on a payload already confirmed — is a direct FPS win.

Reuse: `consensus.ConsensusBuffer` supplies the multi-frame agreement
(required_streak + miss tolerance) rather than a second implementation of it.
"""
import logging
import math
import re
import threading
import time

import numpy as np

from consensus import ConsensusBuffer
from tracker import BoxTracker

try:
    import cv2
    _HAVE_CV2 = True
except Exception:                                   # pragma: no cover
    cv2 = None
    _HAVE_CV2 = False

log = logging.getLogger("perception")

#: Perception states, in the order of a successful hunt.
STATES = ("SEARCHING", "CANDIDATE_DETECTED", "TARGET_TRACKING", "QR_DECODING",
          "PAYLOAD_VALIDATED", "RESULT_REPORTED", "ERROR")

#: Defaults, all overridable from config.yaml `perception:`. Every value here
#: is a starting point for a bench calibration, not a measured constant.
DEFAULT_PERCEPTION = {
    "enabled": True,
    # --- quality gate -------------------------------------------------
    # Laplacian variance measured on the QR ROI (or a centre crop). The
    # absolute number depends on sensor, lens and scene contrast, so this is
    # a *relative* floor to be set from your own bench frames.
    "blur_min": 18.0,
    "luma_min": 25.0,          # mean grey below this = underexposed
    "luma_max": 235.0,         # above this = blown highlights
    "dark_pct_min": 8.0,       # 5th-percentile grey floor
    # Geometric motion-blur budget: exposure_s * image-plane speed. It is an
    # ESTIMATE from tracked velocity, not a measurement of the shutter.
    "exposure_s": 0.002,
    "max_motion_blur_px": 3.0,
    "require_quality_for_decode": True,
    # --- tracker ------------------------------------------------------
    "iou_thr": 0.25,
    "smooth": 0.5,
    "min_hits": 2,             # persistence before a candidate is trusted
    "max_misses": 8,
    "max_age_s": 2.0,
    "stale_s": 0.5,
    # --- decode cadence ------------------------------------------------
    "decode_every_n": 2,       # frames between decode attempts on a track
    "decode_own": False,       # tap feeds mission's decodes; standalone sets True
    "min_roi_side_px": 28,     # below this a decode attempt is wasted CPU
    "roi_margin": 0.35,
    "roi_upsample": 3.0,
    "perspective_fix": True,
    # --- payload trust -------------------------------------------------
    "required_streak": 2,
    "miss_tolerance": 1,
    "validate": True,
    "payload_min_len": 1,
    "payload_max_len": 128,
    "payload_pattern": "",     # optional regex the payload must match
    "duplicate_window_s": 30.0,
    # --- recovery ------------------------------------------------------
    "lost_grace_s": 1.5,       # how long a vanished target is still "yours"
    "error_recover_s": 5.0,    # detector errors: auto-return to SEARCHING
    "max_consecutive_errors": 5,
    "history": 60,
}


def _cfg_get(cfg, key, default=None):
    """`cfg` may be a full config (with `perception:`) or the section itself."""
    if not isinstance(cfg, dict):
        return DEFAULT_PERCEPTION.get(key, default)
    sec = cfg.get("perception")
    if isinstance(sec, dict):
        return sec.get(key, DEFAULT_PERCEPTION.get(key, default))
    return cfg.get(key, DEFAULT_PERCEPTION.get(key, default))


def perception_config(cfg):
    """Full perception settings dict from a mission config or section."""
    out = dict(DEFAULT_PERCEPTION)
    sec = None
    if isinstance(cfg, dict):
        sec = cfg.get("perception") if isinstance(cfg.get("perception"), dict) else cfg
    if isinstance(sec, dict):
        for k in DEFAULT_PERCEPTION:
            if k in sec:
                out[k] = sec[k]
    return out


class Quality(object):
    """One frame's usability verdict. Cheap, and never raises."""

    __slots__ = ("ok", "blur_var", "mean_luma", "p5_luma", "motion_blur_px",
                 "reason", "measured")

    def __init__(self, ok=True, blur_var=None, mean_luma=None, p5_luma=None,
                 motion_blur_px=None, reason="", measured=False):
        self.ok = ok
        self.blur_var = blur_var
        self.mean_luma = mean_luma
        self.p5_luma = p5_luma
        self.motion_blur_px = motion_blur_px
        self.reason = reason
        self.measured = measured

    def as_dict(self):
        return {"ok": self.ok,
                "blur_var": None if self.blur_var is None else round(self.blur_var, 1),
                "mean_luma": None if self.mean_luma is None else round(self.mean_luma, 1),
                "p5_luma": None if self.p5_luma is None else round(self.p5_luma, 1),
                "motion_blur_px": (None if self.motion_blur_px is None
                                   else round(self.motion_blur_px, 2)),
                "measured": self.measured, "reason": self.reason}


#: Exposure is judged on a small downscale of the scene, not the full frame.
#: mean/percentile survive this, and it turns a 3 MP histogram on the front
#: camera (~4608x2592) into a 77 k-pixel one — the Pi 5 cannot afford the
#: former at 10 Hz for a statistic that only needs to answer "is it dark?".
_EXPOSURE_PX = 320


def assess_quality(frame_bgr, cfg, roi=None, speed_px_s=None):
    """Blur / exposure verdict for one frame.

    Two measurements, deliberately taken from different patches, because the
    two defects live in different places:

    * **blur** is measured on the QR ROI when there is a target. Sharpness of
      the rest of the frame says nothing about whether the modules are
      resolvable, and the ROI is the cheap patch anyway.
    * **exposure** is measured on a downscaled centre crop of the *scene*.
      Measuring it on the ROI is a trap: a QR is ~50% black by construction,
      so a 5th-percentile luminance test on it reports "underexposed" on a
      perfectly exposed, perfectly readable code. (This bug was caught by the
      first end-to-end run; the ROI's p5 was 0 because the black modules are
      *supposed* to be 0.)

    Motion blur is estimated geometrically: tracked image-plane speed x the
    configured exposure. Reported as an estimate — the real shutter time is
    chosen by the ISP and is not read back here, which is exactly why
    `exposure_s` is a config value to be calibrated.
    """
    q = Quality()
    try:
        q.motion_blur_px = (None if speed_px_s is None
                            else abs(float(speed_px_s)) * float(cfg["exposure_s"]))
        if not _HAVE_CV2 or frame_bgr is None:
            q.reason = "no-cv2"
            return q
        h, w = frame_bgr.shape[:2]
        if h < 8 or w < 8:
            q.reason = "frame-too-small"
            return q

        # --- exposure: centre crop of the scene, downscaled -------------
        x1, y1 = int(w * 0.25), int(h * 0.25)
        scene = frame_bgr[y1:int(h * 0.75), x1:int(w * 0.75)]
        if scene is None or scene.size == 0:
            q.reason = "empty-scene"
            return q
        sg = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY) if scene.ndim == 3 else scene
        # Shrink only — never upscale. Upscaling a small frame turns a black
        # QR into grey and would quietly change what the percentile means.
        sh, sw = sg.shape[:2]
        scale = min(1.0, float(_EXPOSURE_PX) / max(1, sw))
        if scale < 1.0:
            sg = cv2.resize(sg, (max(1, int(sw * scale)), max(1, int(sh * scale))),
                            interpolation=cv2.INTER_AREA)
        q.mean_luma = float(sg.mean())
        hist = cv2.calcHist([sg], [0], None, [256], [0, 256]).ravel()
        total = float(hist.sum()) or 1.0
        acc, p5 = 0.0, 0.0
        for i, c in enumerate(hist):
            acc += c
            if acc / total >= 0.05:
                p5 = float(i)
                break
        q.p5_luma = p5

        # --- blur: the ROI when we have one, else the same centre crop ---
        blur_src = sg
        if roi is not None and int(roi[2]) > 8 and int(roi[3]) > 8:
            rx, ry, rw, rh = [int(v) for v in roi[:4]]
            rx1, ry1 = max(0, rx), max(0, ry)
            rx2, ry2 = min(w, rx + rw), min(h, ry + rh)
            patch = frame_bgr[ry1:ry2, rx1:rx2]
            if patch is not None and patch.size:
                blur_src = (cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                            if patch.ndim == 3 else patch)
        q.blur_var = float(cv2.Laplacian(blur_src, cv2.CV_64F).var())
        q.measured = True

        if (q.motion_blur_px is not None
                and q.motion_blur_px > float(cfg["max_motion_blur_px"])):
            q.ok = False
            q.reason = "motion-blur %.1fpx>%.1f" % (q.motion_blur_px,
                                                    cfg["max_motion_blur_px"])
        elif q.blur_var < float(cfg["blur_min"]):
            q.ok = False
            q.reason = "blur %.0f<%.0f" % (q.blur_var, cfg["blur_min"])
        elif q.mean_luma < float(cfg["luma_min"]) or q.p5_luma < float(cfg["dark_pct_min"]):
            q.ok = False
            q.reason = "dark mean=%.0f p5=%.0f" % (q.mean_luma, q.p5_luma)
        elif q.mean_luma > float(cfg["luma_max"]):
            q.ok = False
            q.reason = "blown mean=%.0f" % q.mean_luma
        else:
            q.reason = "ok"
    except Exception as e:
        # A quality check must never be the thing that kills detection.
        log.debug("quality assess failed: %r", e)
        q.reason = "quality-error"
    return q


def qr_roi(frame_bgr, box, margin=0.35, upsample=1.0, perspective_fix=True):
    """Crop the candidate QR, optionally perspective-correct and upscale it.

    Two separate wins, and it is worth being clear about which is which:

    * **perspective correction** changes the *geometry*. A panel seen at 30
      degrees of tilt is a trapezoid; `warpPerspective` through the detected
      quad re-rectangles it, which is what lets the finder patterns line up.
      This genuinely helps decoding and cannot be replaced by sharpening.
    * **upsampling** changes only the sampling grid. It makes a small QR
      *visible* to the detector stage, but it cannot restore detail the sensor
      never captured — `fov_report.py` is what tells you whether the modules
      are actually there. Reported separately (`upsampled`) so nobody reads
      the ROI size as extra optical resolution.

    Returns ``(roi_bgr, meta)``; `roi_bgr` is None when the crop is unusable.
    """
    meta = {"perspective": False, "upsampled": False, "quad": None}
    if not _HAVE_CV2 or frame_bgr is None or box is None:
        return None, meta
    try:
        h, w = frame_bgr.shape[:2]
        x, y, bw, bh = [int(v) for v in box[:4]]
        if bw <= 4 or bh <= 4:
            return None, meta
        mx, my = int(bw * margin), int(bh * margin)
        x1, y1 = max(0, x - mx), max(0, y - my)
        x2, y2 = min(w, x + bw + mx), min(h, y + bh + my)
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None, meta
        roi = frame_bgr[y1:y2, x1:x2].copy()
        if perspective_fix:
            try:
                det = cv2.QRCodeDetector()
                quad, _pts = det.detect(roi)
                if quad is not None and len(quad) == 4:
                    side = int(max(bw, bh) * (1.0 + 2 * margin))
                    side = max(16, min(side, 1024))
                    src = quad[0].astype("float32") if hasattr(quad[0], "astype") \
                        else quad.reshape(-1, 2).astype("float32")
                    dst = _square_pts(side)
                    m = cv2.getPerspectiveTransform(src, dst)
                    roi_w = cv2.warpPerspective(roi, m, (side, side),
                                                flags=cv2.INTER_LINEAR,
                                                borderMode=cv2.BORDER_REPLICATE)
                    if roi_w is not None and roi_w.size:
                        roi = roi_w
                        meta["perspective"] = True
                        meta["quad"] = [[float(a), float(b)] for a, b in src]
            except Exception:
                pass
        if upsample and upsample > 1.0:
            s = min(6.0, float(upsample))
            roi = cv2.resize(roi, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
            meta["upsampled"] = True
        meta["size"] = [int(roi.shape[1]), int(roi.shape[0])]
        return roi, meta
    except Exception as e:
        log.debug("qr_roi failed: %r", e)
        return None, meta


def _square_pts(side):
    return np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
                    dtype="float32")


class PayloadValidator(object):
    """Say whether a decoded string is a believable payload.

    A decode backend can return junk on a pathological crop. Rejecting it here
    — with a reason — is cheaper and safer than propagating it, and the reason
    is what a field log needs.
    """

    def __init__(self, cfg):
        sec = perception_config(cfg)
        self.enabled = bool(sec["validate"])
        self.min_len = int(sec["payload_min_len"])
        self.max_len = int(sec["payload_max_len"])
        self.pattern = None
        raw = str(sec.get("payload_pattern") or "").strip()
        if raw:
            try:
                self.pattern = re.compile(raw)
            except re.error as e:
                log.warning("perception.payload_pattern %r invalid (%s) — ignored",
                            raw, e)

    def check(self, payload):
        """-> (ok: bool, reason: str)"""
        if payload is None:
            return False, "none"
        if not self.enabled:
            return True, "validation-off"
        try:
            s = str(payload)
        except Exception:
            return False, "unprintable"
        if len(s) < self.min_len:
            return False, "too-short %d<%d" % (len(s), self.min_len)
        if len(s) > self.max_len:
            return False, "too-long %d>%d" % (len(s), self.max_len)
        if any(ord(c) < 32 or ord(c) == 127 for c in s):
            return False, "control-char"
        if not s.strip():
            return False, "blank"
        if self.pattern is not None and not self.pattern.search(s):
            return False, "pattern-mismatch"
        return True, "ok"


class PerceptionFSM(object):
    """The state machine itself. Thread-safe, bounded, never raises.

    Every transition is recorded with a timestamp and a reason so the field
    question "why did it never lock on?" has an answer in the log rather than
    in a guess.
    """

    def __init__(self, cfg=None):
        self.cfg = perception_config(cfg)
        self._lock = threading.Lock()
        self.state = "SEARCHING"
        self.detail = "init"
        self.since = time.time()
        self.history = []
        self.counts = {}
        self.recoveries = {}

    def _log(self, kind, reason=""):
        self.history.append({"ts": round(time.time(), 3), "state": self.state,
                             "event": kind, "reason": reason})
        if len(self.history) > int(self.cfg["history"]):
            del self.history[:-int(self.cfg["history"])]

    def _transition(self, state, detail, reason="", now=None):
        """`now` lets the caller keep the state clock on the SAME timeline as
        its frames. Mixing `time.time()` here with a caller-supplied `ts`
        makes `age_s` nonsense (a recovery timer that never fires, or fires
        instantly) — the kind of bug that only shows up in a replay."""
        if state != self.state:
            self.counts[state] = self.counts.get(state, 0) + 1
            self.state = state
            self.since = time.time() if now is None else float(now)
        self.detail = detail
        self._log(state, reason or detail)

    def note_recovery(self, reason):
        self.recoveries[reason] = self.recoveries.get(reason, 0) + 1
        self._log("recovery", reason)

    def age_s(self, now=None):
        return max(0.0, (time.time() if now is None else now) - self.since)

    def as_dict(self):
        return {"state": self.state, "detail": self.detail,
                "age_s": round(self.age_s(), 2),
                "counts": dict(self.counts),
                "recoveries": dict(self.recoveries),
                "history": list(self.history[-12:])}


class PerceptionPipeline(object):
    """Frame in, structured perception result out. Observation-only.

    Call `update()` once per frame with whatever the running stack already
    produced (frame, filtered boxes, decoded payload). Every argument is
    optional: a caller with only boxes still gets tracking, a caller with only
    a payload still gets validation. Nothing is raised at the caller, and
    nothing here can alter a box or a payload on its way back out.
    """

    def __init__(self, cfg=None, own_decode=None):
        self.cfg = perception_config(cfg)
        if own_decode is not None:
            self.cfg["decode_own"] = bool(own_decode)
        self.tracker = BoxTracker(
            iou_thr=self.cfg["iou_thr"], smooth=self.cfg["smooth"],
            min_hits=self.cfg["min_hits"], max_misses=self.cfg["max_misses"],
            max_age_s=self.cfg["max_age_s"], stale_s=self.cfg["stale_s"])
        self.fsm = PerceptionFSM(self.cfg)
        self.validator = PayloadValidator(self.cfg)
        self.consensus = ConsensusBuffer(
            required_consecutive=self.cfg["required_streak"],
            miss_tolerance=self.cfg["miss_tolerance"])
        self._lock = threading.Lock()
        self.enabled = bool(self.cfg["enabled"])
        self.quality = Quality(reason="not-measured")
        self.frames = 0
        self.decode_attempts = 0
        self.decodes_skipped = 0
        self.decode_failures = 0
        self.invalid_payloads = 0
        self.payloads_seen = 0
        self.errors = 0
        self._consecutive_errors = 0
        self._t_start = time.time()
        self._t_first_frame = None
        self._decode_frame_no = 0
        self._reported = {}          # payload -> first report ts
        self._validating = None      # payload currently in PAYLOAD_VALIDATED
        self._last_result = None
        self.last_roi_meta = {}
        self._t_first_valid = None

    # -- public ---------------------------------------------------------
    def note_error(self, err, kind="detector"):
        """Report a camera/detector/Hailo failure. Does not raise, by design."""
        with self._lock:
            self.errors += 1
            self._consecutive_errors += 1
            if self._consecutive_errors >= int(self.cfg["max_consecutive_errors"]):
                self.fsm._transition("ERROR", "%s: %s" % (kind, err),
                                     "consecutive-errors")
            else:
                self.fsm._log("error", "%s: %s" % (kind, err))
            log.warning("perception %s error (%d in a row): %s",
                        kind, self._consecutive_errors, err)

    def update(self, frame=None, boxes=None, payload=None, ts=None,
               camera=None, source="tap", decode_fn=None):
        """One perception cycle. Returns the structured result dict."""
        try:
            return self._update(frame, boxes, payload, ts, camera, source,
                                decode_fn)
        except Exception as e:
            self.errors += 1
            log.warning("perception update failed: %r", e)
            return self.snapshot()

    def snapshot(self):
        """Current structured result without advancing the pipeline."""
        with self._lock:
            if self._last_result is None:
                self._last_result = self._build()
            out = dict(self._last_result)
            out["ts"] = time.time()
            return out

    def reset(self):
        with self._lock:
            self.tracker.reset()
            self.consensus.reset()
            self.fsm = PerceptionFSM(self.cfg)
            self._consecutive_errors = 0
            self._reported = {}
            self._validating = None
            self._last_result = None
            self._t_first_valid = None

    # -- internals ------------------------------------------------------
    def _update(self, frame, boxes, payload, ts, camera, source, decode_fn):
        now = time.time() if ts is None else float(ts)
        t0 = time.time()
        with self._lock:
            self.frames += 1
            if self._t_first_frame is None:
                self._t_first_frame = now
            self._consecutive_errors = 0
            prev_state = self.fsm.state

            tracks = self.tracker.update(boxes if boxes is not None else [], now)
            best = self.tracker.best()

            # ---- quality gate (ROI-local when we have a target) ----------
            self.quality = assess_quality(
                frame, self.cfg,
                roi=(best.int_box if best is not None else None),
                speed_px_s=(best.speed_px_s if best is not None else None))

            # ---- our own decode attempt, on a cadence -------------------
            decode_ms = None
            if payload is None and self._should_own_decode(best, now):
                self._decode_frame_no += 1
                if self._decode_frame_no % max(1, int(self.cfg["decode_every_n"])) == 0:
                    td = time.time()
                    payload = self._decode_target(frame, best, decode_fn)
                    decode_ms = (time.time() - td) * 1000.0

            # ---- payload trust: validate, then consensus ----------------
            valid, reason = None, ""
            dup = False
            if payload is not None:
                self.payloads_seen += 1
                valid, reason = self.validator.check(payload)
                if not valid:
                    self.invalid_payloads += 1
                    self.fsm.note_recovery("invalid_payload")
                else:
                    if payload in self._reported:
                        dup = True
                        self.fsm.note_recovery("duplicate_payload")

            newly, cstate = self.consensus.update(payload if valid else None)
            if valid and self._t_first_valid is None:
                self._t_first_valid = now

            # ---- PAYLOAD_VALIDATED -> RESULT_REPORTED ------------------
            # The payload is SENT at the moment it is validated (inside
            # `_step`, so there is no added latency), and this move to the
            # reported state happens on the following frame. Keeping them as
            # two observable states is the point: "I trust this payload"
            # (N-frame agreement) and "I have published it" are different
            # claims, and a caller reading only the returned state must be
            # able to tell whether the report has already gone out.
            if self.fsm.state == "PAYLOAD_VALIDATED":
                self.fsm._transition("RESULT_REPORTED",
                                     "reported %r" % (self._validating,),
                                     now=now)

            # ---- the state machine -------------------------------------
            self._step(best, cstate, valid, reason, dup, now, prev_state)

            # ---- stall / error recovery --------------------------------
            # A detector/Hailo failure is not permanent: after the recovery
            # window the pipeline returns to SEARCHING rather than staying
            # latched in ERROR for the rest of the flight.
            if not self.tracker.tracks and self.fsm.state == "ERROR":
                if self.fsm.age_s(now) > float(self.cfg["error_recover_s"]):
                    self.fsm.note_recovery("detector_error_recovered")
                    self.fsm._transition("SEARCHING", "recovered", now=now)

            e2e_ms = (time.time() - t0) * 1000.0 + (decode_ms or 0.0)
            self._last_result = self._build(now=now, cstate=cstate, valid=valid,
                                            reason=reason, dup=dup,
                                            best=best, decode_ms=decode_ms,
                                            e2e_ms=e2e_ms, camera=camera,
                                            source=source)
            return dict(self._last_result)

    def _should_own_decode(self, best, now):
        if not self.cfg["decode_own"] or best is None or not best.confirmed:
            return False
        if self.quality.measured and not self.quality.ok and \
                bool(self.cfg["require_quality_for_decode"]):
            self.decodes_skipped += 1
            return False
        if best.side_px < float(self.cfg["min_roi_side_px"]):
            self.decodes_skipped += 1
            return False
        return True

    def _decode_target(self, frame, track, decode_fn):
        """Decode the tracked ROI. Never raises; None means 'no payload'."""
        self.decode_attempts += 1
        try:
            if frame is None or track is None:
                return None
            if decode_fn is not None:
                return decode_fn(frame, track.int_box)
            from decoder import decode_frame
            roi, meta = qr_roi(frame, track.int_box,
                               margin=float(self.cfg["roi_margin"]),
                               upsample=float(self.cfg["roi_upsample"]),
                               perspective_fix=bool(self.cfg["perspective_fix"]))
            self.last_roi_meta = meta
            if roi is not None:
                p = decode_frame(roi)
                if p:
                    return p
            self.decode_failures += 1
            return decode_frame(frame, track.int_box)
        except Exception as e:
            self.decode_failures += 1
            log.debug("decode_target failed: %r", e)
            return None

    def _step(self, best, cstate, valid, reason, dup, now, prev_state):
        """The transition table. Read it as the specification."""
        cfg = self.cfg
        confirmed = bool(cstate.get("confirmed"))
        tracked = best is not None and best.confirmed and not best.stale

        if self.fsm.state == "ERROR":
            # stay latched until the recovery timer in _update, unless a target
            # is genuinely back (then there is nothing to recover from)
            if tracked:
                self.fsm.note_recovery("track_after_error")
                self.fsm._transition("TARGET_TRACKING", "target after error",
                                     now=now)
            return

        if confirmed and valid:
            val = cstate["value"]
            # Already validated/reported THIS payload -> hold. Without this
            # guard RESULT_REPORTED bounces straight back to PAYLOAD_VALIDATED
            # every frame (the transition condition "not currently validating"
            # is true again the moment we leave), which would re-report a
            # duplicate forever. A genuinely different payload still starts a
            # fresh cycle, because its consensus value differs.
            if val is not None and val == self._validating and \
                    self.fsm.state in ("PAYLOAD_VALIDATED", "RESULT_REPORTED"):
                return
            if not dup:
                self._validating = val
                # Publish NOW — the streak has already been earned, and
                # delaying the send to earn a nicer-looking state sequence
                # would be trading real latency for log aesthetics.
                self._report(val, now)
                self.fsm._transition("PAYLOAD_VALIDATED",
                                     "%r streak=%d" % (val, cstate["streak"]),
                                     now=now)
            return

        if best is not None and not best.confirmed:
            self.fsm._transition("CANDIDATE_DETECTED",
                                 "hits=%d/%d" % (best.hits, self.tracker.min_hits),
                                 now=now)
            if not self.quality.ok and self.quality.measured:
                self.fsm.detail += " (quality: %s)" % self.quality.reason
            return

        if tracked:
            if self.fsm.state in ("PAYLOAD_VALIDATED", "RESULT_REPORTED") and dup:
                # Already reported this payload; keep tracking, do not re-report.
                return
            if self.fsm.state not in ("PAYLOAD_VALIDATED", "RESULT_REPORTED"):
                low_conf = (bool(cfg["require_quality_for_decode"])
                            and self.quality.measured and not self.quality.ok)
                if self.fsm.state == "QR_DECODING" and not low_conf:
                    # Stay in QR_DECODING across a failed attempt: the target
                    # is still there and the next frame may be sharp. Only the
                    # detail changes, so this does not spam the transition log.
                    self.fsm.detail = "no payload (%s)" % (reason or "not-readable")
                else:
                    self.fsm._transition("TARGET_TRACKING",
                                         "age=%.1fs conf=%.2f"
                                         % (best.age_s, best.conf), now=now)
            if self._decode_due(best):
                if self.fsm.state != "QR_DECODING":
                    self.fsm._transition("QR_DECODING",
                                         "roi=%dpx" % int(best.side_px),
                                         now=now)
            elif not self.quality.ok and self.quality.measured:
                self.fsm.detail = "quality: %s" % self.quality.reason
            return

        # ---- nothing usable this frame -----------------------------------
        # A target that vanished GRACEFULLY: a gap shorter than lost_grace_s
        # is a dropped frame or a bad frame, not a lost target, so the state
        # is held. Past the grace period the hunt is genuinely over, and that
        # is what the recovery counter records.
        grace = float(cfg["lost_grace_s"])
        fresh = [t for t in self.tracker.tracks if t.lost_for(now) <= grace]
        if fresh:
            self.fsm.detail = "no-confirm hits=%d gone=%.1fs" % (
                max(t.hits for t in fresh),
                min(t.lost_for(now) for t in fresh))
            return
        if self.fsm.state != "SEARCHING":
            reason_txt = ("decode_failed" if self.fsm.state == "QR_DECODING"
                          else "target_lost")
            self.fsm.note_recovery(reason_txt)
            self.fsm._transition("SEARCHING", "target gone (%s)" % reason_txt,
                                 now=now)

    def _decode_due(self, track):
        if track is None:
            return False
        if track.hits % max(1, int(self.cfg["decode_every_n"])) != 0:
            return False
        return True

    def _report(self, payload, now):
        """Mark a payload reported. Returns the payload only on the FIRST time."""
        if not payload:
            return None
        if payload in self._reported:
            return None
        self._reported[payload] = now
        # bound the set: drop entries older than the duplicate window
        win = float(self.cfg["duplicate_window_s"])
        for p in [p for p, t in self._reported.items() if now - t > win]:
            self._reported.pop(p, None)
        return payload

    def _build(self, now=None, cstate=None, valid=None, reason="", dup=False,
               best=None, decode_ms=None, e2e_ms=None, camera=None, source="tap"):
        now = time.time() if now is None else now
        cstate = cstate or self.consensus.status()
        b = best if best is not None else self.tracker.best()
        target = None
        if b is not None:
            target = {"track_id": b.id, "box": b.int_box, "conf": round(b.conf, 3),
                      "best_conf": round(b.best_conf, 3),
                      "age_s": round(b.age_s, 2),
                      "last_seen_s": round(b.lost_for(now), 2),
                      "stale": bool(b.stale), "confirmed": bool(b.confirmed),
                      "speed_px_s": round(b.speed_px_s, 1),
                      "side_px": int(b.side_px)}
        n_tracks = len(self.tracker.tracks)
        return {
            "ts": now,
            "state": self.fsm.state,
            "detail": self.fsm.detail,
            "state_age_s": round(self.fsm.age_s(now), 2),
            "payload": cstate["value"] if cstate.get("confirmed") else None,
            "payload_valid": bool(valid),
            "payload_confirmed": bool(cstate.get("confirmed")),
            "payload_validation": reason or "",
            "streak": cstate.get("streak", 0),
            "required_streak": cstate.get("required", 0),
            "duplicate": bool(dup),
            "duplicate_suppressed": bool(dup),
            "roi": self.last_roi_meta or {},
            "target": target,
            "quality": self.quality.as_dict(),
            "tracks": n_tracks,
            "camera": camera,
            "source": source,
            "timing": {"decode_ms": None if decode_ms is None else round(decode_ms, 1),
                       "e2e_ms": None if e2e_ms is None else round(e2e_ms, 1)},
            "stats": {"frames": self.frames,
                      "decode_attempts": self.decode_attempts,
                      "decodes_skipped": self.decodes_skipped,
                      "decode_failures": self.decode_failures,
                      "payloads_seen": self.payloads_seen,
                      "invalid_payloads": self.invalid_payloads,
                      "errors": self.errors,
                      # seconds from the first frame the pipeline ever saw to
                      # the first frame carrying a VALID payload (None = none
                      # yet). This is the Phase-8 "time to first valid QR
                      # payload" number, measured, not inferred.
                      "time_to_first_valid_s": (None if self._t_first_valid is None
                                                else round(self._t_first_valid
                                                           - (self._t_first_frame
                                                              or self._t_start), 3))},
            "fsm": self.fsm.as_dict(),
            # Phase 10: this is a PERCEPTION output. It is not evidence that
            # the airspace below is safe or that the vehicle may descend.
            "perception_only": True,
            "drives_flight": False,
        }
