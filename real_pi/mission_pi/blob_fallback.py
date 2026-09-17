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
    data modules). Normalized to 0..1. This is the strongest single term:
    it separates "white rectangle" from "QR-like structure".
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
        rectangularity = area / max(1.0, rw * rh)
        aspect = min(rw, rh) / max(rw, rh)
        aspect_t = 1.0 - min(1.0, abs(aspect - 0.85) / 0.5)
        angles = _quad_angle_score(approx)
        contrast = _local_contrast(gray, x, y, w, h)
        runs = _internal_runs(gray, approx)
        size_s = _size_score(max(rw, rh), ctx)
        m = {"rect": round(rectangularity, 3), "aspect": round(aspect, 3),
             "angles": round(angles, 3), "contrast": round(contrast, 3),
             "runs": round(runs, 3), "size": round(size_s, 3)}
        score = (0.35 * runs + 0.20 * rectangularity + 0.15 * contrast
                 + 0.10 * aspect_t + 0.10 * angles + 0.10 * size_s)
        found.append((x, y, w, h, score, m))
    return found


def find_candidates(frame_bgr, cfg=None, ctx=None):
    """[(x, y, w, h, score01, metrics), ...] by score desc (deduped).

    ctx: {alt_m, hfov_deg, img_w, img_h} for the size-plausibility term.
    Scans both polarities (bright panel on dark ground AND dark object on
    bright ground). Never raises; [] when cv2 is missing.
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
        cands.sort(key=lambda b: -b[4])
        for b in cands:
            if all(_iou(b, k) < 0.5 for k in out):
                out.append(b)
    except Exception as e:
        log.debug("find_candidates failed: %r", e)
        return []
    return out[:int(cfg.get("max_cands", 10))]


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
