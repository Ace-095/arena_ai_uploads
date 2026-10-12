"""Blob-candidate fallback: hypothesis generator for the last-resort search.

Principle: blob detection GENERATES hypotheses; it never decides. A real
QR cue hands to TRACK, a decoder payload (+consensus) hands to TRANSMIT —
this module steers only and can never transmit (see Mission._blob_fallback).

Pipeline (mission side):
  1. During grid legs, _blob_observe() scores the bottom frame every ~2 s
     and ground-projects each QR-ish rectangle into _blob_obs.
  2. cluster_observations() groups sightings by ground position (a real
     panel reprojects to one stable spot; noise doesn't) and ranks the
     top-3 by QR-likeness.
  3. Each visit descends altitude stages; trend_ok() keeps only
     trajectories whose QR-structure score IMPROVES as we get closer.

find_candidates needs numpy + cv2 (same bench stack as decoder.py); the
clustering and trend helpers are stdlib-only so the policy stays testable
anywhere. Thresholds are starting guesses — tune from sim snapshots with
tools/blob_bench.py, not by staring at this file.

Candidate sources, and why there are two: the contour scan needs a clean blob
out of a global Otsu threshold, which a high-contrast QR panel destroys (its own
black modules drag the threshold down until the panel is no longer a connected
region — measured: 1 contour in the frame without a QR, 250 with one). OpenCV's
quirc-based QRCodeDetector is immune to that, because it scans for the finder
pattern's 1:1:3:1:1 run signature instead of looking for a blob. Both are used,
scored by one shared _measure() path, because they fail in different places
(see tests/test_blob_fallback.py for the measured numbers). Consequence for
anyone tuning the weights: the "runs" term does NOT separate a blank white
square from a QR panel — a blank square scores 0.62 — so do not read a high
score as evidence of QR structure.
"""
import logging
import math

log = logging.getLogger("blob")

try:
    import numpy as _np
    import cv2 as _cv2
    _HAVE_CV2 = True
except Exception:
    _np = None
    _cv2 = None
    _HAVE_CV2 = False


_QR_DETECTOR = None
_QR_DETECTOR_DEAD = False


def _qr_detector():
    """Cached cv2.QRCodeDetector (quirc), or None when unavailable.

    Measured on 12 synthetic cases (real QR panel, 2 textured grounds x 6 module
    scales) and 72 decoy cases (blank square, checkerboard, salt-pepper noise,
    stripes, dash "print", single concentric-square bullseye); see
    tests/test_blob_fallback.py, which pins every number quoted here.

      * it finds the QR in 5/12 cases, against 3/12 for the contour path alone
        - the two extra are 1.6 px/module on both grounds, a scale where the
        contour path finds nothing at all;
      * it produced 1 false positive in 72 decoy cases;
      * it decodes the payload from about 2.5 px/module up, which the contour
        path never does;
      * below 1 px/module it is blind, like everything else measured here.

    So it is used as a QR-SPECIFIC HYPOTHESIS SOURCE - it never decides
    anything: the caller still only ever gets ranked boxes and scores, exactly
    as before.
    """
    global _QR_DETECTOR, _QR_DETECTOR_DEAD
    if _QR_DETECTOR is None and not _QR_DETECTOR_DEAD:
        try:
            _QR_DETECTOR = _cv2.QRCodeDetector()
        except Exception as e:                 # old/odd OpenCV build
            _QR_DETECTOR_DEAD = True
            log.debug("QRCodeDetector unavailable: %r", e)
    return _QR_DETECTOR


def _qr_detector_boxes(gray, cfg):
    """[(quad, ...)] reported by the QR detector; [] when it is off/missing.

    cfg["qr_detector"] = False turns the extra cost off. Measured cost of
    detect() alone: 55.6 ms at 2028x1520, 20.2 ms at 1014x760, 5.4 ms at
    640x480; find_candidates is called about every blob_observe_s (~2 s), so
    this is a few percent of that budget, not a per-frame cost.
    """
    if not cfg.get("qr_detector", True):
        return []
    det = _qr_detector()
    if det is None:
        return []
    try:
        res = det.detect(gray)
    except Exception as e:
        log.debug("QRCodeDetector.detect failed: %r", e)
        return []
    # 4.x/5.x -> (ok, points); some builds -> (ok, points, straight)
    if not isinstance(res, tuple) or not res:
        return []
    ok = res[0]
    pts = res[1] if len(res) > 1 else None
    if not ok or pts is None or len(pts) == 0:
        return []
    try:
        quads = _np.asarray(pts, dtype=_np.float32).reshape(-1, 4, 2)
    except Exception:
        return []
    return [q.reshape(4, 1, 2) for q in quads]


def _quad_angle_score(quad):
    """1.0 when all four corners are ~90 deg (perspective-tolerant)."""
    try:
        pts = [tuple(map(float, p[0])) for p in quad]
        errs = []
        for i in range(4):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % 4]
            cx, cy = pts[(i + 2) % 4]
            v1 = (ax - bx, ay - by)
            v2 = (cx - bx, cy - by)
            n1 = math.hypot(*v1) or 1e-6
            n2 = math.hypot(*v2) or 1e-6
            cos_a = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            errs.append(abs(math.degrees(math.acos(cos_a)) - 90.0))
        return max(0.0, 1.0 - sum(errs) / (4.0 * 45.0))
    except Exception:
        return 0.0


def _local_contrast(gray, x, y, w, h):
    """Edge strength around the region (stddev of an expanded ROI)."""
    try:
        H, W = gray.shape[:2]
        x1, y1 = max(0, x - w // 2), max(0, y - h // 2)
        x2, y2 = min(W, x + w + w // 2), min(H, y + h + h // 2)
        roi = gray[y1:y2, x1:x2]
        if roi.size == 0:
            return 0.0
        _m, std = _cv2.meanStdDev(roi)
        return max(0.0, min(1.0, float(std[0][0]) / 60.0))
    except Exception:
        return 0.0


def _internal_runs(gray, quad, size=64):
    """B/W transitions inside the quad (warped square, middle rows/cols).

    A plain board shows ~0-4 runs; a QR v1 shows ~10+ (finder patterns +
    data modules). Normalized to 0..1, weight 0.35.

    MEASURED WEAKNESS, stated here because the obvious reading of a high score
    is wrong: this counts TRANSITIONS, so it does not separate a blank white
    square from a QR panel - a blank square only scores 0.62 because the other
    five terms still pay out, and a checkerboard earns runs for having many
    transitions and no QR structure at all. The ratio criterion that would
    separate them (a finder pattern's black/white runs are always 1:1:3:1:1) was
    prototyped and rejected: with a 50% tolerance it fires on noise, stripes and
    checkers too (66 false positives in 72 decoy cases), which is why the
    QR-specific work is delegated to OpenCV's detector instead. See
    tests/test_blob_fallback.py.
    """
    try:
        src = _np.float32([p[0] for p in quad])
        dst = _np.float32([[0, 0], [size, 0], [size, size], [0, size]])
        M = _cv2.getPerspectiveTransform(src, dst)
        warp = _cv2.warpPerspective(gray, M, (size, size))
        _t, bw = _cv2.threshold(warp, 0, 255,
                                _cv2.THRESH_BINARY + _cv2.THRESH_OTSU)
        runs = []
        for i in (size // 4, size // 2, 3 * size // 4):
            for line in (bw[i, :], bw[:, i]):
                runs.append(int(_np.sum(line[:-1] != line[1:])))
        return max(0.0, min(1.0, sum(runs) / (6.0 * 14.0)))
    except Exception:
        return 0.0


def _size_score(px, ctx):
    """1.0 when the implied ground size is panel-plausible (0.2..1.5 m)."""
    try:
        alt = float((ctx or {}).get("alt_m", 0))
        hfov = float((ctx or {}).get("hfov_deg", 0))
        img_w = float((ctx or {}).get("img_w", 0))
        if alt <= 0 or hfov <= 0 or img_w <= 0:
            return 0.5
        fw_m = 2.0 * alt * math.tan(math.radians(hfov) / 2.0)
        size_m = px * fw_m / img_w
        if 0.2 <= size_m <= 1.5:
            return 1.0
        if size_m < 0.05 or size_m > 5.0:
            return 0.0
        lo = (size_m - 0.05) / (0.2 - 0.05) if size_m < 0.2 else 1.0
        hi = (5.0 - size_m) / (5.0 - 1.5) if size_m > 1.5 else 1.0
        return max(0.0, min(lo, hi))
    except Exception:
        return 0.5


def _iou(a, b):
    ax2, ay2, bx2, by2 = a[0] + a[2], a[1] + a[3], b[0] + b[2], b[1] + b[3]
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def _scan(gray, bw, cfg, ctx, img_area):
    found = []
    min_area = float(cfg.get("min_area_px", 400))
    max_area = img_area * float(cfg.get("max_area_frac", 0.25))
    res = _cv2.findContours(bw, _cv2.RETR_EXTERNAL, _cv2.CHAIN_APPROX_SIMPLE)
    cnts = res[0] if len(res) == 2 else res[1]
    for c in cnts:
        area = _cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        peri = _cv2.arcLength(c, True)
        if peri <= 0:
            continue
        approx = _cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) != 4 or not _cv2.isContourConvex(approx):
            continue
        x, y, w, h = _cv2.boundingRect(approx)
        if w < 8 or h < 8:
            continue
        (_cx, _cy), (rw, rh), _ang = _cv2.minAreaRect(c)
        if rw < 1 or rh < 1:
            continue
        found.append(_measure(gray, approx, ctx, area=area, rw=rw, rh=rh))
    return found


def _measure(gray, quad, ctx, area=None, rw=None, rh=None):
    """(x, y, w, h, score, metrics) for one quad - the single scoring path.

    Shared by the contour scan and the QR-detector hypotheses so both are
    ranked on the same scale. Weights are unchanged from the original
    implementation: they are a heuristic, and the measured weakness (a BLANK
    white square scores 0.62 because runs=0 still earns rect+contrast+aspect+
    angles+size) is corrected by the `qr_detector` flag below rather than by
    re-tuning weights nobody has measured.
    """
    x, y, w, h = _cv2.boundingRect(quad)
    if area is None:
        area = _cv2.contourArea(quad)
    if rw is None or rh is None:
        (_c, (rw, rh), _a) = _cv2.minAreaRect(quad)
        rw, rh = max(rw, 1.0), max(rh, 1.0)
    rectangularity = area / max(1.0, rw * rh)
    aspect = min(rw, rh) / max(rw, rh)
    aspect_t = 1.0 - min(1.0, abs(aspect - 0.85) / 0.5)
    angles = _quad_angle_score(quad)
    contrast = _local_contrast(gray, x, y, w, h)
    runs = _internal_runs(gray, quad)
    size_s = _size_score(max(rw, rh), ctx)
    m = {"rect": round(rectangularity, 3), "aspect": round(aspect, 3),
         "angles": round(angles, 3), "contrast": round(contrast, 3),
         "runs": round(runs, 3), "size": round(size_s, 3),
         "qr_detector": 0.0}
    score = (0.35 * runs + 0.20 * rectangularity + 0.15 * contrast
             + 0.10 * aspect_t + 0.10 * angles + 0.10 * size_s)
    return (x, y, w, h, score, m)


def find_candidates(frame_bgr, cfg=None, ctx=None):
    """[(x, y, w, h, score01, metrics), ...] by score desc (deduped).

    ctx: {alt_m, hfov_deg, img_w, img_h} for the size-plausibility term.
    Scans both polarities (bright panel on dark ground AND dark object on
    bright ground) AND asks OpenCV's QR detector for QR-specific hypotheses,
    because the two paths fail on different things: the contour path needs a
    clean blob and finds nothing at 1.6 px/module, while the detector finds the
    QR there but is blind far below 1 px/module. Neither decides anything --
    this still only returns ranked boxes. Never raises; [] when cv2 is missing.
    """
    cfg = cfg or {}
    out = []
    if frame_bgr is None or not _HAVE_CV2 or getattr(frame_bgr, "size", 0) == 0:
        return out
    try:
        gray = frame_bgr if frame_bgr.ndim == 2 else _cv2.cvtColor(
            frame_bgr, _cv2.COLOR_BGR2GRAY)
        h_img, w_img = gray.shape[:2]
        blur = _cv2.GaussianBlur(gray, (5, 5), 0)
        _t, bw = _cv2.threshold(blur, 0, 255,
                                _cv2.THRESH_BINARY + _cv2.THRESH_OTSU)
        cands = _scan(gray, bw, cfg, ctx, w_img * h_img)
        cands += _scan(gray, 255 - bw, cfg, ctx, w_img * h_img)
        # QR-specific hypotheses, scored by the SAME _measure path so they are
        # ranked on one scale and cannot out-rank by construction. The
        # `qr_detector` flag records provenance only. A score floor was tried
        # and removed: it changed the score of 0 of 5 detector boxes measured
        # (they already score 0.956-0.972), so it was dead logic.
        for quad in _qr_detector_boxes(gray, cfg):
            try:
                bx, by, bwid, bhei, sc, m = _measure(gray, quad, ctx)
            except Exception:
                continue
            if bwid < 8 or bhei < 8:
                continue
            if bwid * bhei > w_img * h_img * float(cfg.get("max_area_frac", 0.25)):
                continue
            m["qr_detector"] = 1.0
            cands.append((bx, by, bwid, bhei, sc, m))
        cands.sort(key=lambda b: -b[4])
        for b in cands:
            keep = None
            for k in out:
                if _iou(b, k) >= 0.5:
                    keep = k
                    break
            if keep is None:
                out.append([b[0], b[1], b[2], b[3], b[4], b[5]])
                continue
            # same box seen twice: keep the higher score, but never lose the
            # detector flag or the low score of the geometry that found it
            if b[5].get("qr_detector"):
                keep[5]["qr_detector"] = 1.0
            if b[4] > keep[4]:
                keep[4] = b[4]
                for key in ("runs", "contrast"):
                    if b[5].get(key, 0) > keep[5].get(key, 0):
                        keep[5][key] = b[5][key]
    except Exception as e:
        log.debug("find_candidates failed: %r", e)
        return []
    return [tuple(b) for b in out[:int(cfg.get("max_cands", 10))]]


def cluster_observations(obs, cfg=None):
    """Group sightings by ground position -> ranked hypotheses.

    obs: [{e, n, score, ts}, ...] (metres east/north of home).
    Returns [{e, n, score, support}, ...] by score desc. A real panel
    reprojects to one stable spot; noise never forms a cluster — so the
    min_support gate IS the "moves inconsistently with the ground" abort.
    """
    cfg = cfg or {}
    rad = float(cfg.get("cluster_radius_m", 2.0))
    min_sup = int(cfg.get("min_support", 2))
    clusters = []
    for o in sorted(obs, key=lambda o: -o.get("score", 0)):
        placed = False
        for cl in clusters:
            if ((o["e"] - cl["e"]) ** 2 + (o["n"] - cl["n"]) ** 2) ** 0.5 <= rad:
                k = cl["support"]
                cl["e"] = (cl["e"] * k + o["e"]) / (k + 1)
                cl["n"] = (cl["n"] * k + o["n"]) / (k + 1)
                cl["score"] = max(cl["score"], o.get("score", 0))
                cl["support"] = k + 1
                placed = True
                break
        if not placed:
            clusters.append({"e": o["e"], "n": o["n"],
                             "score": o.get("score", 0), "support": 1})
    out = [cl for cl in clusters if cl["support"] >= min_sup]
    out.sort(key=lambda cl: -cl["score"])
    return out


def trend_ok(scores, cfg=None):
    """Positive QR-structure trajectory across descent stages.

    Latest must beat entry by trend_gain AND sit within trend_slack of the
    best so far: dips recover, flat/declining aborts. Needs >= 2 stages.
    """
    cfg = cfg or {}
    if len(scores) < 2:
        return True
    gain = float(cfg.get("trend_gain", 0.03))
    slack = float(cfg.get("trend_slack", 0.05))
    first, latest = scores[0], scores[-1]
    best = max(scores[:-1])
    return latest >= first + gain and latest >= best - slack
