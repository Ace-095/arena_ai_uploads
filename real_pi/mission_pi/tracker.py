"""Temporal tracking + bounding-box stabilisation for the QR hunt (numpy-free).

Why this file exists
--------------------
Every stage up to now was **per frame**: a YOLO box appeared, `qr_filter`
gated it, the consensus buffer counted payloads. Nothing in the stack knew
that two boxes in two consecutive frames were the *same physical QR panel*.
On a moving drone that is the whole reliability problem:

  * the bbox jitters by ±2-6 px frame to frame (drone vibration + AE/AWB
    hunting), so the ROI handed to the decoder keeps moving and
    `decode_frame` re-acquires a different (sometimes worse) crop each time;
  * one bad frame (motion blur, a glint, a shadow) produces a box, and a
    single spurious box was previously indistinguishable from a real one;
  * "the target disappeared" and "this one frame had no detection" looked
    identical, so a wobble reset any persistence logic.

This module answers only one question, per frame: *which box is which
object, and how long has it been there?* It is deliberately
observation-only — it never suppresses, moves or invents a box for the
mission's flight path (see `perception.py` for the state machine that uses
it). `mission.py` is not involved: the tap feeds us and we hand back state.

Design notes
------------
* **IoU association with a motion gate.** Pure IoU fails exactly when the
  drone moves fastest: at 2 m/s and 12 fps the panel moves ~15-25 px between
  frames, which collapses IoU for a 34 px box. Each track therefore predicts
  its next position from an estimated velocity, and a detection is accepted
  on `IoU >= iou_thr` **or** a predicted-centre distance inside a gate scaled
  by the box size.
* **EMA smoothing on the box, not on the image.** Frame averaging is
  explicitly *not* done anywhere (it would add blur to a moving target);
  smoothing the box coordinates removes jitter without touching pixels.
* **Fast scale adaptation.** A plain EMA on width/height lags badly during
  the approach stair (12 m -> 9 m -> 7 m is a 70% size change in seconds), so
  a jump past `snap_ratio` in area is taken at full weight instead of being
  smoothed. This is the "do not average in a way that adds blur" rule applied
  to geometry.
* **Persistence is explicit.** `hits`/`confirmed` mean a caller can require a
  target to be seen N frames before acting on it, and a single dropped frame
  only increments `misses` — it does not destroy the track.

All state is plain Python + math. No numpy import keeps this importable in
the stdlib-only test style this repo already uses (see tests/test_qr_filter.py).
"""
import logging
import math
import time

log = logging.getLogger("tracker")

#: How close (as a fraction of the box diagonal) a detection's centre must be
#: to a track's *predicted* centre to count as that track when IoU collapsed.
#:
#: 1.1 rather than something tighter because of the *first* fast frame: before
#: any velocity exists the prediction cannot help, and a step of one box
#: diagonal is exactly the case IoU misses. For a 34 px panel that is a 53 px
#: search radius in a 2028 px frame — two real panels closer than that would
#: be ambiguous to any centroid tracker, so nothing is given away.
DEFAULT_GATE = 1.1
#: Extra gate growth per second of travel at the track's own speed: the faster
#: the object moves, the less the constant-velocity prediction can be trusted.
GATE_SPEED_SLACK = 0.5


def _iou(a, b):
    """IoU of two (x, y, w, h) boxes. 0.0 on degenerate input, never raises."""
    try:
        ax1, ay1, aw, ah = a
        bx1, by1, bw, bh = b
        ax2, ay2 = ax1 + aw, ay1 + ah
        bx2, by2 = bx1 + bw, by1 + bh
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = ix2 - ix1, iy2 - iy1
        if iw <= 0 or ih <= 0:
            return 0.0
        inter = iw * ih
        union = aw * ah + bw * bh - inter
        return (inter / union) if union > 0 else 0.0
    except Exception:
        return 0.0


def _centre(box):
    x, y, w, h = box
    return (x + w / 2.0, y + h / 2.0)


def _diag(box):
    return math.hypot(box[2], box[3])


class Track:
    """One tracked object: smoothed geometry + how long it has existed.

    Geometry in FULL-FRAME pixels (the same coordinate space `BBox` uses), so
    a track's box can be handed straight to `decoder.decode_frame` as an ROI.
    """

    __slots__ = ("id", "x", "y", "w", "h", "conf", "source", "vx", "vy",
                 "hits", "misses", "age", "first_ts", "last_ts", "best_conf",
                 "confirmed", "stale", "_acc")

    def __init__(self, tid, box, conf, ts, source="detector"):
        self.id = int(tid)
        self.x, self.y, self.w, self.h = [float(v) for v in box[:4]]
        self.conf = float(conf)
        self.best_conf = float(conf)
        self.source = source
        self.vx = 0.0
        self.vy = 0.0
        self.hits = 1
        self.misses = 0
        self.age = 1
        self.first_ts = float(ts)
        self.last_ts = float(ts)
        self.confirmed = False
        self.stale = False
        self._acc = 1

    # -- geometry -------------------------------------------------------
    @property
    def box(self):
        """Smoothed (x, y, w, h) in full-frame pixels."""
        return (self.x, self.y, self.w, self.h)

    @property
    def int_box(self):
        """Integer (x, y, w, h) — what the decoder / overlay want."""
        return (int(round(self.x)), int(round(self.y)),
                int(round(self.w)), int(round(self.h)))

    @property
    def centre(self):
        return _centre(self.box)

    @property
    def side_px(self):
        """Shortest side — the QR-module budget that matters (see fov_report)."""
        return min(self.w, self.h)

    @property
    def speed_px_s(self):
        """Estimated image-plane speed. Drives the motion-blur estimate."""
        return math.hypot(self.vx, self.vy)

    @property
    def age_s(self):
        return max(0.0, self.last_ts - self.first_ts)

    def predict(self, dt):
        """Where this box should be `dt` seconds later at the current velocity."""
        return (self.x + self.vx * dt, self.y + self.vy * dt, self.w, self.h)

    def lost_for(self, now):
        return max(0.0, float(now) - self.last_ts)

    def __repr__(self):
        return ("Track#%d(%.0f,%.0f %dx%d conf=%.2f hits=%d miss=%d%s)"
                % (self.id, self.x, self.y, self.w, self.h, self.conf,
                   self.hits, self.misses, " STALE" if self.stale else ""))


class BoxTracker:
    """Frame-to-frame association + EMA smoothing over detector boxes.

    ``update(boxes, ts=None)`` returns the live tracks (confirmed first, then
    by confidence). It never raises into the caller: a malformed box is
    skipped, and a broken internal state degrades to "no tracks" rather than
    taking the detection worker down mid-flight.
    """

    def __init__(self, iou_thr=0.25, gate=DEFAULT_GATE, smooth=0.5,
                 min_hits=2, max_misses=8, max_age_s=2.0, stale_s=0.5,
                 snap_ratio=1.8, vel_smooth=0.5, max_tracks=16):
        self.iou_thr = float(iou_thr)
        self.gate = float(gate)
        self.smooth = min(1.0, max(0.0, float(smooth)))
        self.min_hits = max(1, int(min_hits))
        self.max_misses = max(1, int(max_misses))
        self.max_age_s = float(max_age_s)
        self.stale_s = float(stale_s)
        self.snap_ratio = max(1.0, float(snap_ratio))
        self.vel_smooth = min(1.0, max(0.0, float(vel_smooth)))
        self.max_tracks = max(1, int(max_tracks))
        self._tracks = []
        self._next_id = 1
        #: events raised by the last update(): dicts with kind + track id
        self.events = []
        #: tracks dropped (max_misses / max_age_s) by the last update()
        self.dropped = []
        self.stats = {"updates": 0, "detections": 0, "created": 0,
                      "confirmed": 0, "dropped": 0, "errors": 0}

    # -- public ---------------------------------------------------------
    @property
    def tracks(self):
        return list(self._tracks)

    def confirmed(self):
        """Tracks that have passed the persistence gate, strongest first."""
        return [t for t in self._sorted() if t.confirmed and not t.stale]

    def best(self):
        """Most confident live track (confirmed preferred), else None."""
        c = self.confirmed()
        if c:
            return c[0]
        live = self._sorted()
        return live[0] if live else None

    def reset(self):
        self._tracks = []
        self.events = []
        self.dropped = []

    def update(self, boxes, ts=None):
        """Associate `boxes` (objects with x/y/w/h/conf) and return live tracks."""
        now = time.time() if ts is None else float(ts)
        self.events = []
        self.dropped = []
        self.stats["updates"] += 1
        dets = []
        try:
            for b in (boxes or []):
                try:
                    box = (float(b.x), float(b.y), float(b.w), float(b.h))
                except Exception:
                    continue
                if box[2] <= 0 or box[3] <= 0:
                    continue
                dets.append((box, float(getattr(b, "conf", 0.0)),
                             str(getattr(b, "source", "detector"))))
        except Exception as e:
            self.stats["errors"] += 1
            log.debug("tracker: bad box list %r", e)
            return self._sorted()
        self.stats["detections"] += len(dets)
        try:
            self._associate(dets, now)
            self._expire(now)
        except Exception as e:
            # Never take the detection worker down: report and keep going.
            self.stats["errors"] += 1
            log.warning("tracker update failed (kept previous state): %r", e)
        return self._sorted()

    def stabilize(self, boxes, ts=None):
        """Convenience for callers that only want smoothed boxes (no ids).

        Returns ``[BBox-like]`` in the SAME order/style the tracker holds them.
        Used by the benchmark overlay so a viewer sees the stabilised box, not
        the raw per-frame jitter.
        """
        return [(t.x, t.y, t.w, t.h, t.conf, t.source) for t in self.update(boxes, ts)]

    # -- internals ------------------------------------------------------
    def _sorted(self):
        return sorted(self._tracks,
                      key=lambda t: (0 if t.confirmed else 1, -t.conf, t.id))

    def _associate(self, dets, now):
        pairs = []
        for ti, tr in enumerate(self._tracks):
            dt = max(0.0, now - tr.last_ts)
            pred = tr.predict(dt)
            pcx, pcy = _centre(pred)
            # Constant-velocity prediction + a gate that grows with the
            # velocity's own uncertainty (GATE_SPEED_SLACK * v * dt). A resting
            # target gets a tight gate; a fast one is allowed to miss its
            # prediction without spawning a duplicate track.
            gate_px = (self.gate * max(4.0, _diag(tr.box))
                       + GATE_SPEED_SLACK * tr.speed_px_s * dt)
            for di, (box, conf, _src) in enumerate(dets):
                iou = _iou(pred, box)
                cx, cy = _centre(box)
                dist = math.hypot(cx - pcx, cy - pcy)
                if iou < self.iou_thr and dist > gate_px:
                    continue
                # Rank on IoU, then on proximity: a far-but-centred detection
                # still beats a nearby one that barely overlaps.
                score = iou + (1.0 - min(1.0, dist / gate_px)) * 0.5
                pairs.append((score, iou, ti, di))
        pairs.sort(key=lambda p: -p[0])

        used_t, used_d = set(), set()
        for _score, _iou_v, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            self._absorb(self._tracks[ti], dets[di], now)

        for di, (box, conf, src) in enumerate(dets):
            if di in used_d:
                continue
            if len(self._tracks) >= self.max_tracks:
                break
            tr = Track(self._next_id, box, conf, now, src)
            self._next_id += 1
            self._tracks.append(tr)
            self.stats["created"] += 1
            self.events.append({"kind": "new", "track": tr.id,
                                "box": tr.int_box, "conf": round(tr.conf, 3)})
            # A new track must clear the persistence gate on THIS frame too:
            # with min_hits=1 (bench/replay configs) the very first sighting is
            # already confirmed, and skipping this made such a track wait for
            # a second frame it was never going to need.
            self._maybe_confirm(tr)

    def _absorb(self, tr, det, now):
        box, conf, src = det
        dt = max(1e-3, now - tr.last_ts)
        # velocity from the RAW centres (pre-smoothing) so it tracks real
        # motion instead of the lag of the filter itself
        ocx, ocy = _centre(tr.box)
        ncx, ncy = _centre(box)
        vx, vy = (ncx - ocx) / dt, (ncy - ocy) / dt
        vs = self.vel_smooth
        tr.vx = tr.vx * (1.0 - vs) + vx * vs
        tr.vy = tr.vy * (1.0 - vs) + vy * vs

        # Scale: snap instead of smoothing through a genuine size change
        # (approach stair 12 -> 9 -> 7 m). Plain EMA here would keep the ROI
        # a frame or two behind the target during exactly the descent where
        # decode matters most.
        old_area = max(1.0, tr.w * tr.h)
        new_area = max(1.0, box[2] * box[3])
        ratio = new_area / old_area
        if ratio > self.snap_ratio or ratio < (1.0 / self.snap_ratio):
            sa = 1.0
        else:
            sa = self.smooth
        s = self.smooth
        tr.x = tr.x * (1 - s) + box[0] * s
        tr.y = tr.y * (1 - s) + box[1] * s
        tr.w = tr.w * (1 - sa) + box[2] * sa
        tr.h = tr.h * (1 - sa) + box[3] * sa
        tr.conf = tr.conf * (1 - s) + conf * s
        tr.best_conf = max(tr.best_conf, conf)
        tr.source = src
        tr.hits += 1
        tr.misses = 0
        tr.age += 1
        tr.last_ts = now
        tr.stale = False
        self._maybe_confirm(tr)

    def _maybe_confirm(self, tr):
        """Apply the persistence gate and raise the event exactly once."""
        was = tr.confirmed
        tr.confirmed = tr.hits >= self.min_hits
        if tr.confirmed and not was:
            self.stats["confirmed"] += 1
            self.events.append({"kind": "confirmed", "track": tr.id,
                                "hits": tr.hits, "box": tr.int_box})

    def _expire(self, now):
        keep = []
        for tr in self._tracks:
            # A track whose last_ts is behind `now` was not matched this frame.
            # One update() == one frame, so this is exactly one miss.
            if now > tr.last_ts + 1e-9:
                tr.misses += 1
            tr.stale = tr.lost_for(now) > self.stale_s
            if tr.misses > self.max_misses or tr.lost_for(now) > self.max_age_s:
                self.stats["dropped"] += 1
                self.dropped.append(tr)
                self.events.append({"kind": "lost", "track": tr.id,
                                    "age_s": round(tr.age_s, 2),
                                    "misses": tr.misses})
                continue
            keep.append(tr)
        self._tracks = keep
