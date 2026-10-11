#!/usr/bin/env python3
"""Perception benchmark: detection and decoding, measured SEPARATELY, scored.

    # 1) build a labelled corpus (synthetic, honest about being synthetic)
    python3 tools/perception_bench.py make --out /tmp/qrbench

    # 2) score the real pipeline against it
    python3 tools/perception_bench.py run --corpus /tmp/qrbench \
        --detector auto --json /tmp/qrbench/results.json --md /tmp/qrbench/results.md

    # 3) throughput on unlabelled footage (no ground truth -> no P/R claims)
    python3 tools/perception_bench.py run --video flight.mp4 --stage recorded

    # 4) live camera (needs the Pi) — bench-only, not flight evidence
    python3 tools/perception_bench.py live --seconds 30 --cam bottom

Why a benchmark tool instead of a paragraph of claims
-----------------------------------------------------
A QR hunt has two failure modes that look identical in a log and are fixed by
completely different work:

  * the **detector** never produced a box over the target (range/optics/FOV);
  * the detector boxed it fine and the **decoder** never recovered the payload
    (module sampling, blur, focus, perspective).

A single "success rate" number hides which one is happening, so this tool
scores them independently, per condition, and refuses to compute a quality
metric it has no ground truth for. Anything that needs hardware that is not
present is reported as `NOT YET MEASURED` with the reason — never as zero, and
never as a pass.

Stages are kept apart on purpose: `unit` (pipeline logic), `recorded`
(synthetic or captured video), `live` (a real camera on a bench), `flight`.
Passing one is not evidence for the next, and the report says so in every
header rather than in a footnote.
"""
import argparse
import glob
import json
import math
import os
import platform
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.dirname(os.path.abspath(__file__))
# Runnable both as `python3 tools/perception_bench.py` (tools/ is sys.path[0])
# and as `tools.perception_bench` from the test suite (mission_pi/ is).
for _p in (HERE, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

#: PROPOSED acceptance criteria. These are starting targets to argue with, not
#: measurements, and the report labels them that way on every row.
PROPOSED = (
    ("detection_recall_at_15m", ">=", 0.90,
     "a 15 m panel must be FOUND (this is not the same as read)"),
    ("detection_precision", ">=", 0.85,
     "the window-grill class of false box must stay filtered"),
    ("fp_per_frame", "<=", 0.10, "spurious boxes per frame"),
    ("decode_success_le9m", ">=", 0.90,
     "payload recovered once inside the approach stair"),
    ("pipeline_fps", ">=", 5.0, "on the host doing the work (Pi 5 target)"),
    ("time_to_first_payload_s", "<=", 10.0,
     "panel in view -> validated payload"),
)


def _vendor_gap():
    """Import cv2/numpy lazily so `--help` works on a bare host."""
    import cv2
    import numpy as np
    return cv2, np


# ==========================================================================
# corpus generation
# ==========================================================================
def _ground(np, h, w, rng):
    """Textured ground: not a flat colour (a flat frame is legitimately blurry,
    and using one would make the blur gate look broken)."""
    gy, gx = np.indices((h, w))
    base = 104 + (((gx // 7) + (gy // 5)) % 2) * 16
    base = base + ((gx * 13 + gy * 29) % 11) - 5
    out = np.clip(base, 0, 255).astype(np.uint8)
    return np.repeat(out[:, :, None], 3, axis=2)


def _qr_image(np, cv2, payload, border=4):
    import qrcode
    q = qrcode.make(payload, border=border).convert("L")
    return np.array(q)


def _paste(np, cv2, frame, patch, cx, cy, scale=1.0, angle=0.0, tilt=0.0):
    """Place the panel with optional rotation and a perspective (mount) tilt."""
    ph, pw = patch.shape[:2]
    if scale != 1.0:
        patch = cv2.resize(patch, (max(4, int(pw * scale)), max(4, int(ph * scale))),
                           interpolation=cv2.INTER_AREA)
        ph, pw = patch.shape[:2]
    if tilt:
        k = math.radians(tilt)
        src = np.float32([[0, 0], [pw, 0], [pw, ph], [0, ph]])
        # squeeze the far edge: what a tilted camera actually does
        inset = pw * 0.5 * (1.0 - math.cos(k))
        dst = np.float32([[inset, 0], [pw - inset, 0], [pw, ph], [0, ph]])
        m = cv2.getPerspectiveTransform(src, dst)
        patch = cv2.warpPerspective(patch, m, (pw, ph),
                                    borderMode=cv2.BORDER_REPLICATE)
    if angle:
        m = cv2.getRotationMatrix2D((pw / 2.0, ph / 2.0), angle, 1.0)
        cos, sin = abs(m[0, 0]), abs(m[0, 1])
        nw, nh = int(ph * sin + pw * cos), int(ph * cos + pw * sin)
        m[0, 2] += nw / 2.0 - pw / 2.0
        m[1, 2] += nh / 2.0 - ph / 2.0
        patch = cv2.warpAffine(patch, m, (nw, nh),
                               borderMode=cv2.BORDER_CONSTANT, borderValue=250)
        ph, pw = patch.shape[:2]
    x1, y1 = int(cx - pw / 2), int(cy - ph / 2)
    H, W = frame.shape[:2]
    if x1 < 0 or y1 < 0 or x1 + pw > W or y1 + ph > H:
        return None
    frame[y1:y1 + ph, x1:x1 + pw] = (patch[:, :, None] * np.ones(3, np.uint8))
    return (x1, y1, pw, ph)


def _motion_blur(np, cv2, frame, ksize, angle):
    if ksize < 3:
        return frame
    k = np.zeros((ksize, ksize), np.float32)
    k[ksize // 2, :] = 1.0 / ksize
    m = cv2.getRotationMatrix2D((ksize / 2.0, ksize / 2.0), angle, 1.0)
    k = cv2.warpAffine(k, m, (ksize, ksize))
    k /= (k.sum() or 1.0)
    return cv2.filter2D(frame, -1, k)


def make_corpus(out_dir, alts, payloads, seed=7, size=(2028, 1520)):
    """Write frames + labels.jsonl. Every condition is explicitly synthetic."""
    cv2, np = _vendor_gap()
    from fov_report import compute, _cfg

    cfg = _cfg(os.path.join(HERE, "config.yaml"))
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    rng = np.random.RandomState(seed)
    W, H = int(size[0]), int(size[1])
    labels = []
    n = 0

    conditions = [
        # (name, alt_m, extras)
        ("alt15_static", 15.0, {}),
        ("alt10_static", 10.0, {}),
        ("alt7_static", 7.0, {}),
        ("alt10_rot45", 10.0, {"angle": 45}),
        ("alt10_rot90", 10.0, {"angle": 90}),
        ("alt10_tilt30", 10.0, {"tilt": 30}),
        ("alt10_blur", 10.0, {"blur": 9, "blur_angle": 0}),
        ("alt10_blur_strong", 10.0, {"blur": 15, "blur_angle": 30}),
        ("alt10_dark", 10.0, {"gain": 0.28}),
        ("alt10_bright", 10.0, {"gain": 1.45}),
        ("alt10_noisy", 10.0, {"noise": 26}),
        ("alt10_occluded", 10.0, {"occlude": 0.22}),
        ("alt10_offcentre", 10.0, {"pos": (0.18, 0.72)}),
        ("alt10_none", 10.0, {"absent": True}),
    ]

    for cname, alt, ex in conditions:
        payload = payloads[n % len(payloads)]
        qr = _qr_image(np, cv2, payload)
        # Two LEAD-IN frames with no panel, then the panel. Without the lead-in
        # "time to first payload" is trivially 0.0 on every sequence and
        # measures nothing; with it, the metric answers the question the
        # mission actually asks: how long after the panel COULD be visible does
        # a validated payload arrive?
        for i in range(-2, 3):
            frame = _ground(np, H, W, rng)
            box = None
            if not ex.get("absent") and i >= 0:
                # the panel's on-frame size comes from the SAME geometry the
                # mission uses — not from a guessed pixel count
                row = compute(cfg, alt, 297.0, 25, 4, 0.0, None, 0.0, 0.0)
                target_px = max(8, int(round(row["qr_px"])))
                scale = target_px / float(qr.shape[0])
                px, py = ex.get("pos", (0.5, 0.5))
                cx = int(W * (0.5 + (px - 0.5) * 0.9)) if i else int(W * 0.5)
                cy = int(H * (0.5 + (py - 0.5) * 0.9)) if i else int(H * 0.5)
                jitter = 6 if i >= 1 else 0        # simulated vibration
                box = _paste(np, cv2, frame, qr, cx + jitter, cy - jitter,
                             scale=scale, angle=ex.get("angle", 0.0),
                             tilt=ex.get("tilt", 0.0))
                if box and ex.get("occlude"):
                    x1, y1, bw, bh = box
                    cw = max(2, int(bw * ex["occlude"]))
                    frame[y1:y1 + bh // 3, x1:x1 + cw] = 40
            if ex.get("blur"):
                frame = _motion_blur(np, cv2, frame, ex["blur"],
                                     ex.get("blur_angle", 0))
            if ex.get("gain"):
                frame = np.clip(frame.astype(np.float32) * ex["gain"], 0, 255) \
                    .astype(np.uint8)
            if ex.get("noise"):
                frame = np.clip(frame.astype(np.float32)
                                + rng.normal(0, ex["noise"], frame.shape),
                                0, 255).astype(np.uint8)
            name = "%s_%02d.jpg" % (cname, i + 2)
            cv2.imwrite(os.path.join(frames_dir, name), frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            labels.append({"frame": "frames/" + name, "condition": cname,
                           "seq": cname, "alt_m": alt,
                           "boxes": [] if box is None else [list(map(int, box))],
                           "payload": None if box is None else payload,
                           "synthetic": True,
                           "extras": {k: v for k, v in ex.items()
                                      if k in ("angle", "tilt", "blur", "gain",
                                               "noise", "occlude")}})
            n += 1

    # an enter/leave sequence for the recovery measurement
    for i in range(8):
        frame = _ground(np, H, W, rng)
        row = compute(cfg, 10.0, 297.0, 25, 4, 0.0, None, 0.0, 0.0)
        target_px = max(8, int(round(row["qr_px"])))
        box = None
        if 2 <= i <= 5:
            qr = _qr_image(np, cv2, "ENTER-EXIT-1")
            scale = target_px / float(qr.shape[0])
            cx = int(W * (0.30 + 0.12 * (i - 2)))
            box = _paste(np, cv2, frame, qr, cx, H // 2, scale=scale)
        name = "seq_enter_exit_%02d.jpg" % i
        cv2.imwrite(os.path.join(frames_dir, name), frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        labels.append({"frame": "frames/" + name, "condition": "enter_exit",
                       "seq": "enter_exit", "alt_m": 10.0,
                       "boxes": [] if box is None else [list(map(int, box))],
                       "payload": None if box is None else "ENTER-EXIT-1",
                       "synthetic": True, "extras": {}})

    with open(os.path.join(out_dir, "labels.jsonl"), "w") as fh:
        for row in labels:
            fh.write(json.dumps(row) + "\n")
    print("wrote %d frames + labels.jsonl to %s" % (len(labels), out_dir))
    print("NOTE: synthetic. Good for detecting regressions and measuring")
    print("      latency; NOT evidence about real optics, focus or print")
    print("      quality. Re-run with --corpus pointing at real captures.")
    return out_dir


# ==========================================================================
# scoring
# ==========================================================================
def _iou(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax1 + aw, bx1 + bw), min(ay1 + ah, by1 + bh)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def _coverage(pred, gt):
    """Fraction of the GROUND-TRUTH box that the prediction covers."""
    px1, py1, pw, ph = pred
    gx1, gy1, gw, gh = gt
    ix1, iy1 = max(px1, gx1), max(py1, gy1)
    ix2, iy2 = min(px1 + pw, gx1 + gw), min(py1 + ph, gy1 + gh)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    area = gw * gh
    return (iw * ih) / float(area) if area > 0 else 0.0


def match_boxes(pred, gt, iou_thr=0.5, cover_thr=0.0):
    """Greedy one-to-one matching -> (tp, fp, fn).

    Two acceptance rules, and the difference is not academic:

    * `iou_thr` (0.5) is the standard object-detection criterion, and it is
      what `detection_recall`/`precision` use so the numbers are comparable
      with anything else.
    * `cover_thr` accepts a box that CONTAINS the target even when IoU is below
      0.5 (a loose but correct box). A detector that boxes the whole A3 sheet
      around the QR scores IoU ~0.4 against the QR itself and would be counted
      as a miss AND a false positive — while the decoder, which only needs the
      code inside the ROI, succeeds. Scoring that as two failures would send
      someone to retrain a model that is working.

    Both are reported; neither is hidden.
    """
    pairs = []
    for i, p in enumerate(pred):
        for j, g in enumerate(gt):
            v = _iou(p, g)
            ok = v >= iou_thr
            if not ok and cover_thr and _coverage(p, g) >= cover_thr:
                ok = True
            if ok:
                pairs.append((v, i, j))
    pairs.sort(key=lambda x: -x[0])
    used_p, used_g = set(), set()
    for v, i, j in pairs:
        if i in used_p or j in used_g:
            continue
        used_p.add(i)
        used_g.add(j)
    return len(used_p), len(pred) - len(used_p), len(gt) - len(used_g)


def pct(values, q):
    if not values:
        return None
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round((len(s) - 1) * q))))
    return s[k]


def read_labels(corpus):
    path = os.path.join(corpus, "labels.jsonl")
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def system_probe():
    """Host vitals. Anything this host cannot answer is reported as such."""
    out = {"host": platform.node(), "platform": platform.platform(),
           "python": platform.python_version(), "cpu_pct": None,
           "rss_mb": None, "temp_c": None, "throttled": None,
           "throttled_note": "not a Raspberry Pi", "hailo": None,
           "hailo_note": "no Hailo device on this host"}
    try:
        import psutil
        out["cpu_pct"] = round(psutil.cpu_percent(interval=0.2), 1)
        out["rss_mb"] = round(psutil.Process().memory_info().rss / 1e6, 1)
    except Exception as e:
        out["cpu_pct"] = "NOT MEASURED (%s)" % type(e).__name__
    for p in ("/sys/class/thermal/thermal_zone0/temp",):
        try:
            with open(p) as fh:
                out["temp_c"] = round(int(fh.read().strip()) / 1000.0, 1)
        except Exception:
            pass
    try:
        r = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                           text=True, timeout=3)
        out["throttled"] = r.stdout.strip()
        out["throttled_note"] = "0x0 = clean"
    except Exception as e:
        out["throttled"] = "NOT MEASURED"
        out["throttled_note"] = "vcgencmd unavailable (%s)" % type(e).__name__
    try:
        r = subprocess.run(["hailortcli", "fw-control", "identify"],
                           capture_output=True, text=True, timeout=5)
        if r.returncode == 0:
            out["hailo"] = r.stdout.strip().splitlines()[:4]
            out["hailo_note"] = ("device present; per-model NPU utilisation is "
                                 "not exposed by HailoRT's CLI, so NPU load is "
                                 "NOT MEASURED (inference latency is measured "
                                 "directly instead)")
        else:
            out["hailo"] = "NOT PRESENT"
    except Exception:
        out["hailo"] = "NOT MEASURED (hailortcli unavailable)"
    return out


def run_corpus(corpus, detector_kind, hef, tiles, conf, stage, md_path, json_path):
    cv2, np = _vendor_gap()
    from detector import get_detector, detect_tiles
    from decoder import decode_frame
    from perception import PerceptionPipeline
    import yaml

    with open(os.path.join(HERE, "config.yaml")) as fh:
        cfg = yaml.safe_load(fh) or {}
    cfg.setdefault("perception", {})
    cfg["perception"].update({"enabled": True, "decode_own": True,
                              "min_hits": 1, "decode_every_n": 1,
                              "required_streak": 1, "blur_min": 0.0,
                              # the bench must MEASURE the decode, not skip it
                              "require_quality_for_decode": False})
    pipe = PerceptionPipeline(cfg, own_decode=True)

    try:
        det = get_detector({"kind": detector_kind, "hef_path": hef,
                            "conf_thr": conf, "prefer": ["onnx", "pt"]})
    except Exception as e:
        print("[bench] detector %r unavailable: %s" % (detector_kind, e))
        print("[bench] falling back to classical (no model) — detection "
              "numbers will be poor BY DESIGN and are labelled as such")
        det = get_detector({"kind": "classical", "conf_thr": conf})
    det_note = "detector=%s" % det.name
    if getattr(det, "conf_thr", None) is not None:
        det_note += " conf=%.2f" % det.conf_thr

    rows = read_labels(corpus)
    tp = fp = fn = 0
    ctp = cfp = cfn = 0            # coverage-based (operational) matching
    dec_ok = dec_tot = 0
    dec_after_detect_ok = dec_after_detect_tot = 0
    detect_ms, decode_ms, e2e_ms = [], [], []
    per_cond = {}
    seqs = {}
    t_start = 0.0
    prev_seq = None

    for idx, row in enumerate(rows):
        img = cv2.imread(os.path.join(corpus, row["frame"]))
        if img is None:
            continue
        if idx == 0:
            t_start = time.time()
        seq = row.get("seq", row["condition"])
        if prev_seq is not None and seq != prev_seq:
            # A sequence is a flight episode: consensus must not leak from the
            # previous episode, or a payload confirmed at 7 m would still be
            # "reported" on a frame at 15 m that decoded nothing.
            pipe.reset()
        prev_seq = seq
        gt = [tuple(b) for b in (row.get("boxes") or [])]

        t0 = time.time()
        try:
            boxes = detect_tiles(det, img) if tiles else det.detect(img)
        except Exception as e:
            print("[bench] detect failed on %s: %r" % (row["frame"], e))
            boxes = []
        d_ms = (time.time() - t0) * 1000.0
        detect_ms.append(d_ms)

        # --- detection scoring (independent of decoding) ---------------
        pred = [(b.x, b.y, b.w, b.h) for b in boxes]
        t = match_boxes(pred, gt)
        tp += t[0]
        fp += t[1]
        fn += t[2]
        tc = match_boxes(pred, gt, cover_thr=0.80)
        ctp += tc[0]
        cfp += tc[1]
        cfn += tc[2]
        c = per_cond.setdefault(row["condition"], {"tp": 0, "fp": 0, "fn": 0,
                                                   "dec_ok": 0, "dec_tot": 0,
                                                   "frames": 0})
        c["tp"] += t[0]
        c["fp"] += t[1]
        c["fn"] += t[2]
        c["frames"] += 1

        # --- decode scoring (the pipeline, exactly as the mission runs it)
        t1 = time.time()
        out = pipe.update(frame=img, boxes=boxes, ts=time.time())
        e_ms = (time.time() - t1) * 1000.0
        e2e_ms.append(e_ms)
        if out["timing"]["decode_ms"]:
            decode_ms.append(out["timing"]["decode_ms"])

        got = out["payload"]
        if row.get("payload"):
            dec_tot += 1
            c["dec_tot"] += 1
            if got == row["payload"]:
                dec_ok += 1
                c["dec_ok"] += 1
            if t[0] > 0:                    # detected AND labelled
                dec_after_detect_tot += 1
                if got == row["payload"]:
                    dec_after_detect_ok += 1
        elif got:
            # decoded a payload where the corpus says there is none
            fp += 1
            c["fp"] += 1

        s = seqs.setdefault(seq, {"frames": 0, "first_frame_ts": None,
                                  "first_payload_ts": None})
        s["frames"] += 1
        if s["first_frame_ts"] is None:
            # the moment the episode starts — i.e. when the panel could first
            # have been seen, which is what "time to payload" must measure
            s["first_frame_ts"] = time.time()
        if got and s["first_payload_ts"] is None and got == row.get("payload"):
            s["first_payload_ts"] = time.time()

    wall = max(1e-6, time.time() - t_start)
    n_frames = len(e2e_ms) or 1
    metrics = {
        "stage": stage,
        "frames": len(e2e_ms),
        "detection_recall": round(tp / float(tp + fn), 4) if (tp + fn) else None,
        "detection_precision": round(tp / float(tp + fp), 4) if (tp + fp) else None,
        "detection_recall_cov": (round(ctp / float(ctp + cfn), 4)
                                 if (ctp + cfn) else None),
        "detection_precision_cov": (round(ctp / float(ctp + cfp), 4)
                                    if (ctp + cfp) else None),
        "fp_per_frame": round(fp / float(n_frames), 4),
        "decode_success": round(dec_ok / float(dec_tot), 4) if dec_tot else None,
        "decode_after_detection": (round(dec_after_detect_ok /
                                         float(dec_after_detect_tot), 4)
                                   if dec_after_detect_tot else None),
        "detect_ms_mean": round(sum(detect_ms) / max(1, len(detect_ms)), 2),
        "detect_ms_p95": round(pct(detect_ms, 0.95) or 0, 2),
        "decode_ms_mean": (round(sum(decode_ms) / len(decode_ms), 2)
                           if decode_ms else "NOT MEASURED"),
        "pipeline_ms_mean": round(sum(e2e_ms) / n_frames, 2),
        "pipeline_ms_p95": round(pct(e2e_ms, 0.95) or 0, 2),
        "pipeline_fps": round(n_frames / wall, 2),
        "time_to_first_payload_s": None,
        "detector": det_note,
    }
    # Criteria that are defined per altitude/condition, derived from the same
    # per-frame records (never a separate estimate).
    def _recall_of(pred):
        tp_ = sum(c["tp"] for k, c in per_cond.items() if pred(k))
        fn_ = sum(c["fn"] for k, c in per_cond.items() if pred(k))
        return round(tp_ / float(tp_ + fn_), 4) if (tp_ + fn_) else None

    def _decode_of(pred):
        ok = sum(c["dec_ok"] for k, c in per_cond.items() if pred(k))
        tot = sum(c["dec_tot"] for k, c in per_cond.items() if pred(k))
        return round(ok / float(tot), 4) if tot else None

    metrics["detection_recall_at_15m"] = _recall_of(lambda k: "alt15" in k)
    metrics["detection_recall_at_10m"] = _recall_of(lambda k: "alt10" in k)
    metrics["decode_success_le9m"] = _decode_of(
        lambda k: any(t in k for t in ("alt7", "alt9", "alt5")))
    ttfp = [(v["first_payload_ts"] - v["first_frame_ts"])
            for v in seqs.values()
            if v["first_payload_ts"] and v["first_frame_ts"]]
    if ttfp:
        metrics["time_to_first_payload_s"] = round(min(ttfp), 2)
    elif dec_tot:
        metrics["time_to_first_payload_s"] = "NO PAYLOAD EVER VALIDATED"

    return metrics, per_cond, system_probe()


#: Criteria that only mean anything on the hardware that will fly. Scoring the
#: Pi's FPS budget on a laptop (or in CI) and calling it a FAIL would be a
#: false alarm; calling it a PASS would be worse.
HOST_DEPENDENT = {"pipeline_fps"}


def is_flight_host(sysinfo):
    """aarch64 + a Hailo device is the flight target; anything else is not."""
    arch = platform.machine().lower()
    arm = ("aarch64" in arch) or ("arm" in arch)
    hailo = str(sysinfo.get("hailo", ""))
    return arm and ("NOT" not in hailo.upper())


def verdict(metric, op, target):
    if metric is None:
        return "NOT MEASURED"
    if isinstance(metric, str):
        return "NOT MEASURED"
    ok = metric >= target if op == ">=" else metric <= target
    return "PASS" if ok else "FAIL"


def write_report(metrics, per_cond, sysinfo, md_path, stage, notes):
    lines = []
    A = lines.append
    A("# Perception benchmark — %s" % stage.upper())
    A("")
    A("**Stage: `%s`.** This is not flight evidence. %s"
      % (stage, STAGE_CAVEAT.get(stage, "")))
    A("")
    A("## Environment")
    A("")
    A("| item | value |")
    A("|---|---|")
    for k in ("host", "platform", "python", "cpu_pct", "rss_mb", "temp_c",
              "throttled", "hailo"):
        A("| %s | `%s` |" % (k, sysinfo.get(k)))
    A("| throttled note | %s |" % sysinfo.get("throttled_note"))
    A("| hailo note | %s |" % sysinfo.get("hailo_note"))
    A("| detector | %s |" % metrics.get("detector"))
    A("| flight host? | %s |" % ("yes" if is_flight_host(sysinfo) else
                                 "NO — latency/FPS below are THIS host's "
                                 "numbers, not the Pi 5's"))
    A("")
    A("## Measured")
    A("")
    A("| metric | value |")
    A("|---|---|")
    for k in ("frames", "detection_recall", "detection_recall_cov",
              "detection_precision", "detection_precision_cov", "fp_per_frame",
              "decode_success", "decode_after_detection", "detect_ms_mean",
              "detect_ms_p95", "decode_ms_mean", "pipeline_ms_mean",
              "pipeline_ms_p95", "pipeline_fps", "time_to_first_payload_s"):
        A("| %s | `%s` |" % (k, metrics.get(k)))
    A("")
    A("## Acceptance criteria (PROPOSED, not achieved claims)")
    A("")
    A("| criterion | target | measured | verdict | what it means |")
    A("|---|---|---|---|---|")
    for name, op, target, why in PROPOSED:
        val = metrics.get(name)
        v = verdict(val, op, target)
        if name in HOST_DEPENDENT and not is_flight_host(sysinfo):
            v = "NOT APPLICABLE (not the flight host)"
        A("| %s | %s %.2f | `%s` | **%s** | %s |"
          % (name, op, target, val, v, why))
    A("")
    A("## Per condition")
    A("")
    A("| condition | frames | tp | fp | fn | recall | decode ok/total |")
    A("|---|---|---|---|---|---|---|")
    for cname in sorted(per_cond):
        c = per_cond[cname]
        rec = (c["tp"] / float(c["tp"] + c["fn"])) if (c["tp"] + c["fn"]) else None
        A("| %s | %d | %d | %d | %d | %s | %d/%d |"
          % (cname, c["frames"], c["tp"], c["fp"], c["fn"],
             "n/a" if rec is None else "%.2f" % rec,
             c["dec_ok"], c["dec_tot"]))
    A("")
    A("## Notes")
    A("")
    for n in notes:
        A("* %s" % n)
    A("")
    text = "\n".join(lines)
    if md_path:
        with open(md_path, "w") as fh:
            fh.write(text)
    return text


STAGE_CAVEAT = {
    "unit": "Pipeline logic only; no optics, no camera, no flight.",
    "recorded": ("Synthetic or previously captured frames. Measured latency is "
                 "this host's; synthetic imagery makes NO claim about real "
                 "lens, focus or print quality."),
    "live": ("A real camera on a bench. Proves the capture path and the "
             "optics on a desk, NOT behaviour under flight motion, vibration "
             "or lighting."),
    "flight": "Captured in flight.",
}


def run_video(path, detector_kind, hef, tiles, conf, stage, json_path, md_path,
              max_frames=0):
    cv2, np = _vendor_gap()
    from detector import get_detector, detect_tiles
    cap = cv2.VideoCapture(path)
    try:
        det = get_detector({"kind": detector_kind, "hef_path": hef,
                            "conf_thr": conf, "prefer": ["onnx", "pt"]})
    except Exception as e:
        print("[bench] detector unavailable (%s) — classical" % e)
        det = get_detector({"kind": "classical"})
    detect_ms = []
    t0 = time.time()
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = time.time()
        boxes = detect_tiles(det, frame) if tiles else det.detect(frame)
        detect_ms.append((time.time() - t) * 1000.0)
        n += 1
        if max_frames and n >= max_frames:
            break
    cap.release()
    wall = max(1e-6, time.time() - t0)
    metrics = {
        "stage": stage, "frames": n,
        "detection_recall": None, "detection_precision": None,
        "fp_per_frame": None, "decode_success": None,
        "decode_after_detection": None,
        "detect_ms_mean": round(sum(detect_ms) / max(1, len(detect_ms)), 2),
        "detect_ms_p95": round(pct(detect_ms, 0.95) or 0, 2),
        "decode_ms_mean": "NOT MEASURED (unlabelled run)",
        "pipeline_ms_mean": round(sum(detect_ms) / max(1, len(detect_ms)), 2),
        "pipeline_ms_p95": round(pct(detect_ms, 0.95) or 0, 2),
        "pipeline_fps": round(n / wall, 2),
        "time_to_first_payload_s": "NOT MEASURED (unlabelled run)",
        "detector": "detector=%s" % det.name,
    }
    notes = ["Unlabelled footage: precision/recall/decoding CANNOT be computed "
             "without ground truth, so they are reported as NOT MEASURED "
             "rather than guessed. Use --corpus for scored runs."]
    text = write_report(metrics, {}, system_probe(), md_path, stage, notes)
    if json_path:
        with open(json_path, "w") as fh:
            json.dump({"metrics": metrics, "system": system_probe()}, fh, indent=2)
    return metrics, text


def run_live(cam_role, seconds, detector_kind, hef, conf, json_path, md_path):
    cv2, np = _vendor_gap()
    import yaml
    from cameras import CameraRig
    from detector import get_detector, detect_tiles

    with open(os.path.join(HERE, "config.yaml")) as fh:
        cfg = yaml.safe_load(fh) or {}
    rig = CameraRig(cfg)
    found = rig.detect()
    cam = rig.cams.get(cam_role) or (list(rig.cams.values()) or [None])[0]
    if cam is None:
        print("[bench] no camera found (rig.detect() -> %r)" % (found,))
        print("[bench] live stage: NOT YET MEASURED")
        return None
    cam.start()
    try:
        det = get_detector({"kind": detector_kind, "hef_path": hef,
                            "conf_thr": conf, "prefer": ["onnx", "pt"]})
    except Exception as e:
        print("[bench] detector unavailable (%s) — classical" % e)
        det = get_detector({"kind": "classical"})
    detect_ms, grab_ms, f0 = [], [], None
    deadline = time.time() + float(seconds)
    n = 0
    last = -1
    while time.time() < deadline:
        t = time.time()
        frame, ts, count = cam.latest()
        if frame is None or count == last:
            time.sleep(0.005)
            continue
        last = count
        grab_ms.append((time.time() - t) * 1000.0)
        t = time.time()
        boxes = detect_tiles(det, frame) if True else det.detect(frame)
        detect_ms.append((time.time() - t) * 1000.0)
        n += 1
        if f0 is None:
            f0 = time.time()
    cam.stop() if hasattr(cam, "stop") else None
    wall = max(1e-6, (time.time() - f0) if f0 else 1.0)
    metrics = {
        "stage": "live", "frames": n,
        "detection_recall": None, "detection_precision": None,
        "fp_per_frame": None, "decode_success": None,
        "decode_after_detection": None,
        "capture_wait_ms_mean": round(sum(grab_ms) / max(1, len(grab_ms)), 2),
        "detect_ms_mean": round(sum(detect_ms) / max(1, len(detect_ms)), 2),
        "detect_ms_p95": round(pct(detect_ms, 0.95) or 0, 2),
        "decode_ms_mean": "NOT MEASURED (no ground truth)",
        "pipeline_ms_mean": round(sum(detect_ms) / max(1, len(detect_ms)), 2),
        "pipeline_ms_p95": round(pct(detect_ms, 0.95) or 0, 2),
        "camera_fps": round(n / wall, 2),
        "pipeline_fps": round(n / wall, 2),
        "time_to_first_payload_s": "NOT MEASURED (no ground truth)",
        "detector": "detector=%s cam=%s" % (det.name, cam.name),
    }
    notes = ["Live bench run: FPS and latency are real camera numbers. "
             "Precision/recall/decoding need labelled frames — point the "
             "camera at a known QR and score the frames offline, or run "
             "`make` + `run` for a synthetic measurement.",
             "A desk result is NOT flight evidence: no vibration, no "
             "attitude change, no prop wash, no motion blur at speed."]
    text = write_report(metrics, {}, system_probe(), md_path, "live", notes)
    if json_path:
        with open(json_path, "w") as fh:
            json.dump({"metrics": metrics, "system": system_probe()}, fh, indent=2)
    return metrics, text


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    m = sub.add_parser("make", help="build a labelled synthetic corpus")
    m.add_argument("--out", required=True)
    m.add_argument("--alts", default="7,10,15")
    m.add_argument("--payloads", default="MISSION-QR-001,QR-TEST-001")
    m.add_argument("--size", default="2028x1520")

    r = sub.add_parser("run", help="score a corpus or a video")
    r.add_argument("--corpus", default="")
    r.add_argument("--video", default="")
    r.add_argument("--detector", default="auto")
    r.add_argument("--hef", default=os.path.join(HERE, "models/qr_yolov8n.hef"))
    r.add_argument("--tiles", action="store_true", default=True)
    r.add_argument("--no-tiles", dest="tiles", action="store_false")
    r.add_argument("--conf", type=float, default=0.35)
    r.add_argument("--stage", default="recorded")
    r.add_argument("--json", default="")
    r.add_argument("--md", default="")
    r.add_argument("--max-frames", type=int, default=0)

    lv = sub.add_parser("live", help="live camera throughput (needs hardware)")
    lv.add_argument("--cam", default="bottom")
    lv.add_argument("--seconds", type=float, default=20.0)
    lv.add_argument("--detector", default="auto")
    lv.add_argument("--hef", default=os.path.join(HERE, "models/qr_yolov8n.hef"))
    lv.add_argument("--conf", type=float, default=0.35)
    lv.add_argument("--json", default="")
    lv.add_argument("--md", default="")

    args = ap.parse_args()
    if args.cmd == "make":
        w, h = (int(x) for x in args.size.lower().split("x"))
        make_corpus(args.out, [float(a) for a in args.alts.split(",")],
                    [p for p in args.payloads.split(",") if p], size=(w, h))
        return 0

    if args.cmd == "run":
        if args.corpus:
            metrics, per_cond, sysinfo = run_corpus(
                args.corpus, args.detector, args.hef, args.tiles, args.conf,
                args.stage, args.md, args.json)
            notes = [
                "Scored against `labels.jsonl`. Detection (IoU>=0.5, greedy "
                "one-to-one) and decoding are measured INDEPENDENTLY: "
                "`decode_success` counts frames whose label has a payload, "
                "`decode_after_detection` counts only frames where the "
                "detector actually found the panel.",
                "Synthetic imagery cannot tell you about your lens, your "
                "focus or your print. It CAN tell you whether a change broke "
                "detection or decoding, and what the latency is.",
                "`time_to_first_payload_s` is measured from the first frame of "
                "a sequence that contains the panel to the first VALIDATED "
                "payload (required_streak satisfied), not to the first box.",
            ]
            text = write_report(metrics, per_cond, sysinfo, args.md,
                                args.stage, notes)
            if args.json:
                with open(args.json, "w") as fh:
                    json.dump({"metrics": metrics, "per_condition": per_cond,
                               "system": sysinfo, "stage": args.stage,
                               "proposed_criteria":
                                   [dict(zip(("name", "op", "target", "why"), p))
                                    for p in PROPOSED]}, fh, indent=2)
            print(text)
            return 0
        if args.video:
            metrics, text = run_video(args.video, args.detector, args.hef,
                                      args.tiles, args.conf, args.stage,
                                      args.json, args.md, args.max_frames)
            print(text)
            return 0
        print("run: pass --corpus DIR or --video FILE")
        return 2

    if args.cmd == "live":
        out = run_live(args.cam, args.seconds, args.detector, args.hef,
                       args.conf, args.json, args.md)
        if out is None:
            return 1
        print(out[1])
        return 0

    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
