#!/usr/bin/env python3
"""qr_angle_bench.py — live QR detection vs ANGLE, on the real camera.

Answers one question before you ever fly: **at which angles does this camera +
this panel actually detect and decode?** You set the panel (or the drone) to a
known angle, the tool scores the real frames, and it prints a table per angle.

It uses the mission's OWN pieces — detector.py (the `auto` ladder: Hailo ->
ONNX -> .pt -> classical), qr_filter.py and decoder.py — so what you measure is
what flies. Nothing here is a re-implementation.

HOW THE GROUND TRUTH WORKS (be honest about this — there are two kinds):

  * DECODE is self-verifying. A frame either produced a payload or it did not,
    and if you pass --payload we also check it MATCHED. That is hard evidence.
  * DETECTION is not. Without human annotation, "the panel was in view" is the
    OPERATOR'S ASSERTION, not a measurement. So the detection columns here are
    "did a box survive the filter while you held the panel at this angle", and
    they are labelled that way. For real precision/recall with IoU, save the
    frames (--save-frames) and score them later:
        python3 tools/perception_bench.py run --corpus <dir with labels.jsonl>

USAGE (on the Pi, from mission_pi/)

  # the main event: walk the angles, 15 frames each, panel at 10 m
  python3 tools/qr_angle_bench.py --source csi:bottom --dist 10 \
      --angles 0,15,30,45,60,75 --per-angle 15 --payload MISSION-QR-001

  # non-interactive (no Enter prompts) — for a scripted bench
  python3 tools/qr_angle_bench.py --source csi:bottom --dist 10 \
      --angles 0,30,45,60 --per-angle 20 --no-prompt --json /tmp/angles.json

  # with the AI HAT, and saving every frame for later annotation
  python3 tools/qr_angle_bench.py --source csi:bottom --hailo \
      --dist 10 --angles 0,45 --per-angle 10 --save-frames /tmp/angle_snaps

  # SCORE FRAMES YOU ALREADY CAPTURED, per angle (no camera needed):
  python3 tools/qr_angle_bench.py --frames "0:/tmp/bench/a0_*.jpg" \
      --frames "45:/tmp/bench/a45_*.jpg" --dist 10

  # quick sanity: is the camera even alive, and how many fps can it give?
  python3 tools/qr_angle_bench.py --source csi:bottom --dist 10 \
      --angles 0 --per-angle 30 --no-prompt

WHY ANGLE MATTERS: a QR is a 2D grid. Rotating the panel in the ground plane
(roll about the optical axis) is free for `cv2.QRCodeDetector` up to ~45 deg and
then it flips to the other diagonal. Tilting it (pitch, an off-nadir camera)
is NOT free: it foreshortens the modules anisotropically and is the failure the
synthetic corpus hit hardest (0/3 decode at 30 deg tilt, while detection stayed
1.00). This tool tells you YOUR numbers instead of the synthetic ones.
"""
import argparse
import glob
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def log(msg):
    print(msg)
    sys.stdout.flush()


# ==========================================================================
# scoring
# ==========================================================================
def score_frames(frames, det, qf, cfg, payload, tiles):
    """Run the mission's detector + filter + decoder over an iterable of frames.

    Returns a per-angle dict. Detection columns are counts of frames where a box
    survived the filter; decode columns are counts of frames that produced a
    payload (and a MATCHING one when `payload` is given).
    """
    from detector import detect_tiles
    from decoder import decode_frame

    n = 0
    raw_hits = 0
    filt_hits = 0
    decoded = 0
    matched = 0
    confs = []
    widths = []
    detect_ms = []
    decode_ms = []
    payloads = {}
    failures = []

    for frame in frames:
        if frame is None:
            continue
        n += 1
        t = time.time()
        try:
            boxes = detect_tiles(det, frame) if tiles else det.detect(frame)
        except Exception as e:                                # never die mid-bench
            failures.append("detect: %r" % (e,))
            boxes = []
        detect_ms.append((time.time() - t) * 1000.0)

        if boxes:
            raw_hits += 1
        kept = qf.filter_boxes(boxes) if boxes else []
        if kept:
            filt_hits += 1
            best = max(kept, key=lambda b: float(b.conf))
            confs.append(float(best.conf))
            widths.append(float(best.w))

        # Decode: try the biggest surviving box, else the whole frame. This
        # mirrors the mission, which decodes ROIs and periodically the full frame.
        t = time.time()
        cand = max(kept, key=lambda b: float(b.w) * float(b.h)) if kept else None
        payload_seen = None
        try:
            if cand is not None:
                payload_seen = decode_frame(frame, (cand.x, cand.y, cand.w, cand.h))
            if not payload_seen:
                payload_seen = decode_frame(frame)
        except Exception as e:
            failures.append("decode: %r" % (e,))
        decode_ms.append((time.time() - t) * 1000.0)

        if payload_seen:
            decoded += 1
            payloads[str(payload_seen)] = payloads.get(str(payload_seen), 0) + 1
            if payload and str(payload_seen).strip() == str(payload).strip():
                matched += 1

    return {
        "frames": n,
        "detected_any": raw_hits,
        "detected_filtered": filt_hits,
        "detected_any_rate": round(raw_hits / n, 4) if n else None,
        "detected_filtered_rate": round(filt_hits / n, 4) if n else None,
        "decoded": decoded,
        "decode_rate": round(decoded / n, 4) if n else None,
        "payload_matched": matched if payload else None,
        "payload_match_rate": round(matched / n, 4) if (n and payload) else None,
        "mean_conf": round(sum(confs) / len(confs), 4) if confs else None,
        "median_box_w_px": sorted(widths)[len(widths) // 2] if widths else None,
        "detect_ms_mean": round(sum(detect_ms) / len(detect_ms), 2) if detect_ms else None,
        "detect_ms_p95": _p95(detect_ms),
        "decode_ms_mean": round(sum(decode_ms) / len(decode_ms), 2) if decode_ms else None,
        "payloads_seen": payloads,
        "failures": failures[:5],
    }


def _p95(values):
    if not values:
        return None
    s = sorted(values)
    return round(s[min(len(s) - 1, int(len(s) * 0.95))], 2)


def evaluate(row, payload):
    """Turn one angle's counts into a verdict. Thresholds are explicitly OURS.

    Stated, not inherited: DETECT needs boxes in >=80% of frames, DECODE needs a
    payload in >=50%. A payload that MATCHED the expected text is a stronger
    signal than one that merely appeared, so a mismatch is called out.
    """
    n = row.get("frames") or 0
    if n == 0:
        return "NO FRAMES"
    dfr = row.get("detected_filtered_rate") or 0.0
    dr = row.get("decode_rate") or 0.0
    if payload and row.get("decoded") and not row.get("payload_matched"):
        return "DECODE != --payload"
    if dr >= 0.5 and dfr >= 0.8:
        return "DETECT + DECODE"
    if dfr >= 0.8:
        return "DETECT only"
    if dfr >= 0.2:
        return "weak detect"
    return "MISS"


# ==========================================================================
# sources
# ==========================================================================
def open_live_source(source, cfg_path):
    """(iterator-of-frames factory, description) for a live camera.

    `csi:bottom` / `csi:front` / `csi:<idx>` go through the mission's own
    CameraRig, so size, tuning file, focus and overlays are the mission's.
    Anything else (a number, a path, an rtsp:// url) goes to cv2.VideoCapture,
    which is how you bench a USB camera or a recorded clip.
    """
    import cv2
    import yaml

    if str(source).lower().startswith("csi"):
        from cameras import CameraRig
        with open(cfg_path) as fh:
            cfg = yaml.safe_load(fh) or {}
        role = str(source).split(":", 1)[1] if ":" in str(source) else ""
        rig = CameraRig(cfg)
        rig.detect()
        cam = None
        if role:
            cam = rig.cams.get(role)
            if cam is None and role.isdigit():
                cam = list(rig.cams.values())[int(role)]
        if cam is None:
            cam = rig.cams.get("bottom") or (list(rig.cams.values()) or [None])[0]
        if cam is None:
            log("[angle] no CSI camera found by CameraRig.detect()")
            return None, None
        cam.start()
        return _RigFrames(cam), rig

    src = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        log("[angle] could not open video source %r" % (source,))
        return None, None
    return _CapFrames(cap), cap


class _CapFrames(object):
    def __init__(self, cap):
        self.cap = cap

    def read(self, timeout=1.0):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


class _RigFrames(object):
    """Adapts a mission camera (latest() + a frame counter) to .read()."""

    def __init__(self, cam):
        self.cam = cam
        self.last = -1

    def read(self, timeout=1.0):
        import time as _t
        deadline = _t.time() + timeout
        while _t.time() < deadline:
            frame, ts, count = self.cam.latest()
            if frame is not None and count != self.last:
                self.last = count
                return frame
            _t.sleep(0.005)
        return None

    def close(self):
        try:
            self.cam.stop()
        except Exception:
            pass


# ==========================================================================
# reporting
# ==========================================================================
def print_table(rows, payload, dist, expected):
    log("")
    log("=" * 100)
    log("QR ANGLE BENCH  -  distance %s m   payload %s" % (dist, payload or "(any)"))
    if expected is not None:
        log("expected QR size at this distance: ~%d px across (%.2f px/module) - "
            "from tools/fov_report.py" % (expected["qr_px"], expected["px_per_module"]))
        log("NOTE: that expectation uses the VENDOR-QUOTED field of view, which is "
            "NOT MEASURED. If your measured box width disagrees, the FOV is wrong - "
            "re-measure it before trusting any range number.")
    log("=" * 100)
    log("%-8s %6s %8s %9s %8s %8s %8s %9s %-20s"
        % ("angle", "frames", "det_any", "det_filt", "decode", "match", "conf", "box_w", "verdict"))
    log("-" * 100)
    for r in rows:
        log("%-8s %6d %8s %9s %8s %8s %8s %9s %-20s" % (
            r["angle"] + " deg", r["frames"],
            _pct(r.get("detected_any_rate")),
            _pct(r.get("detected_filtered_rate")),
            _pct(r.get("decode_rate")),
            _pct(r.get("payload_match_rate")) if payload else "-",
            "%.2f" % r["mean_conf"] if r.get("mean_conf") is not None else "-",
            "%d" % r["median_box_w_px"] if r.get("median_box_w_px") else "-",
            r["verdict"]))
    log("-" * 100)
    for r in rows:
        seen = r.get("payloads_seen") or {}
        if seen:
            log("payloads seen at %s deg: %s" % (
                r["angle"], ", ".join("%s x%d" % (k, v) for k, v in sorted(seen.items()))))
        if r.get("failures"):
            log("  !! errors at %s deg: %s" % (r["angle"], "; ".join(r["failures"])))
    log("")
    log("det_any  = a raw detector box, before the qr_filter gate")
    log("det_filt = a box that PASSED qr_filter (this is what can steer the mission)")
    log("decode   = a payload came out of decoder.decode_frame")
    log("match    = that payload EQUALLED the expected text (hard evidence)")
    log("")
    log("GROUND TRUTH, STATED HONESTLY: the decode columns are self-verifying -")
    log("a frame either decoded the right payload or it did not. The detection")
    log("columns rest on YOUR assertion that the panel was in view at that angle,")
    log("which is not a measurement. For IoU-based precision/recall, keep the")
    log("frames (--save-frames) and score them with tools/perception_bench.py run.")
    log("")


def _pct(v):
    return "-" if v is None else "%.2f" % v


def write_json_md(rows, meta, json_path, md_path):
    if json_path:
        with open(json_path, "w") as fh:
            json.dump({"meta": meta, "angles": rows}, fh, indent=2)
        log("[angle] wrote %s" % json_path)
    if md_path:
        with open(md_path, "w") as fh:
            fh.write("# QR angle bench\n\n")
            for k, v in meta.items():
                fh.write("* %s: `%s`\n" % (k, v))
            fh.write("\n| angle (deg) | frames | det any | det filtered | decode | "
                     "match | mean conf | median box w (px) | verdict |\n")
            fh.write("|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")
            for r in rows:
                fh.write("| %s | %d | %s | %s | %s | %s | %s | %s | %s |\n" % (
                    r["angle"], r["frames"],
                    _pct(r.get("detected_any_rate")),
                    _pct(r.get("detected_filtered_rate")),
                    _pct(r.get("decode_rate")),
                    _pct(r.get("payload_match_rate")),
                    r["mean_conf"], r["median_box_w_px"], r["verdict"]))
            fh.write("\nDecode columns are self-verifying. Detection columns rest on "
                     "the operator's assertion that the panel was in view.\n")
        log("[angle] wrote %s" % md_path)


# ==========================================================================
# main
# ==========================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="csi:bottom",
                    help="csi:bottom | csi:front | csi:<idx> | <webcam idx> | <file/rtsp>")
    ap.add_argument("--frames", action="append", default=[], metavar="ANGLE:GLOB",
                    help="offline mode: score already-captured frames instead of a "
                         "live camera. Repeatable, e.g. --frames '45:/tmp/a45_*.jpg'")
    ap.add_argument("--angles", default="0,15,30,45,60,75",
                    help="comma list of panel angles in degrees (live mode)")
    ap.add_argument("--per-angle", type=int, default=15,
                    help="frames to capture per angle (live mode)")
    ap.add_argument("--dist", type=float, default=10.0, help="camera-to-panel distance in metres")
    ap.add_argument("--payload", default=None, help="expected QR text; enables the match column")
    ap.add_argument("--detector", default="auto",
                    choices=["auto", "hailo", "onnx", "pt", "yolo", "classical", "none"])
    ap.add_argument("--hef", default="models/qr_yolov8n.hef")
    ap.add_argument("--hailo", action="store_true", help="shorthand for --detector hailo")
    ap.add_argument("--conf", type=float, default=None, help="override detector conf_thr")
    ap.add_argument("--no-tiles", action="store_true",
                    help="disable 3x3 tiling (the mission tiles the bottom frame)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="do not wait for Enter between angles (scripted benches)")
    ap.add_argument("--settle", type=float, default=0.6,
                    help="seconds to discard after an angle change (motion settle)")
    ap.add_argument("--save-frames", default=None, help="directory to save scored frames")
    ap.add_argument("--config", default=os.path.join(_ROOT, "config.yaml"))
    ap.add_argument("--json", default=None)
    ap.add_argument("--md", default=None)
    args = ap.parse_args(argv)

    if args.hailo:
        args.detector = "hailo"

    try:
        from perception_bench import _vendor_gap, system_probe, pct   # reuse, no dupe
        _vendor_gap()
    except Exception as e:
        log("[angle] needs numpy + opencv (%s)" % e)
        return 2
    import yaml

    with open(args.config) as fh:
        cfg = yaml.safe_load(fh) or {}

    det_cfg = dict(cfg.get("detector", {}) or {})
    det_cfg["kind"] = args.detector
    det_cfg["hef_path"] = args.hef
    if args.conf is not None:
        det_cfg["conf_thr"] = args.conf
    from detector import get_detector
    try:
        det = get_detector(det_cfg)
    except Exception as e:
        log("[angle] no detector (%s)" % e)
        return 2

    from qr_filter import QrFilter, load_presets
    preset = det_cfg.get("preset", "day")
    presets = load_presets(cfg)
    p = presets.get(preset, presets.get("day", {}))
    qf = QrFilter.from_config(cfg)
    if p:
        qf.min_conf = float(p.get("conf_thr", qf.min_conf))
        qf = QrFilter(min_conf=qf.min_conf, min_side_px=qf.min_side_px,
                      aspect_min=qf.aspect_min, aspect_max=qf.aspect_max,
                      require_decode_for_cue=qf.require_decode_for_cue)
    tile_grid = det_cfg.get("tile_grid", [3, 3])
    tiles = (not args.no_tiles) and bool(det_cfg.get("tile_bottom", True))
    rows, tiles_sig = [], "%dx%d" % (tile_grid[0], tile_grid[1]) if tiles else "off"

    log("[angle] detector=%s  conf=%.2f  filter_min_conf=%.2f  tiles=%s  preset=%s"
        % (det.name, float(det_cfg.get("conf_thr", 0.25)), qf.min_conf, tiles_sig, preset))

    # expected geometry from the feasibility math (clearly marked as vendor-FOV based)
    expected = None
    try:
        from fov_report import compute
        cam = (cfg.get("cameras") or {}).get("bottom") or {}
        row = compute(cfg, args.dist, 297.0, 25, 4, 0.0,
                      tuple(cam.get("size") or [2028, 1520]), 0.0, 0.002)
        expected = {"qr_px": round(row["qr_px"]), "px_per_module": round(row["px_per_module"], 2)}
    except Exception:
        expected = None

    meta = {"stage": "live-camera angle bench (not flight evidence)",
            "detector": det.name, "distance_m": args.dist,
            "tiles": tiles_sig, "preset": preset,
            "filter_min_conf": qf.min_conf, "payload_expected": args.payload or "",
            "config": args.config}

    # ---------------- offline mode ----------------
    if args.frames:
        import cv2
        for spec in args.frames:
            if ":" not in spec:
                log("[angle] --frames needs ANGLE:GLOB, got %r" % spec)
                return 2
            ang, pattern = spec.split(":", 1)
            paths = sorted(glob.glob(pattern))
            if not paths:
                log("[angle] no files matched %r" % pattern)
                continue
            frames = [cv2.imread(pth) for pth in paths]
            r = score_frames(frames, det, qf, cfg, args.payload, tiles)
            r["angle"] = str(ang)
            r["verdict"] = evaluate(r, args.payload)
            r["source"] = pattern
            rows.append(r)
            log("[angle] %-6s %s -> %d frames" % (ang + " deg", pattern, r["frames"]))
        print_table(rows, args.payload, args.dist, expected)
        if args.json or args.md:
            write_json_md(rows, meta, args.json, args.md)
        # exit non-zero if nothing at 0 deg decoded - a smoke signal for scripts
        return 0 if any(r["frames"] for r in rows) else 1

    # ---------------- live mode ----------------
    idx, src = open_live_source(args.source, args.config)
    if idx is None:
        log("[angle] live source unavailable -> NOT YET MEASURED")
        log("[angle] passed through: no camera on this host. On the Pi try "
            "--source csi:bottom, then tools/list_cams.py to see the roles.")
        return 1
    import cv2
    angles = [a.strip() for a in str(args.angles).split(",") if a.strip()]
    snaps = 0
    try:
        for ang in angles:
            if not args.no_prompt:
                log("")
                log("*" * 72)
                log("*  SET THE PANEL / DRONE TO %s DEGREES  (angle in the GROUND PLANE)" % ang)
                log("*  then press Enter to capture %d frames" % args.per_angle)
                log("*  (typing 's' + Enter skips this angle)" % ())
                log("*" * 72)
                try:
                    ans = input().strip().lower()
                except EOFError:
                    ans = ""
                if ans == "s":
                    continue
            if args.settle > 0:
                time.sleep(args.settle)
            frames, saved = [], []
            for i in range(max(1, args.per_angle)):
                frame = idx.read(timeout=2.0)
                if frame is None:
                    continue
                frames.append(frame)
                if args.save_frames:
                    d = os.path.join(args.save_frames, "a%s" % ang.replace(".", "_"))
                    try:
                        os.makedirs(d, exist_ok=True)
                        fp = os.path.join(d, "f%03d.jpg" % i)
                        cv2.imwrite(fp, frame)
                        saved.append(fp)
                    except Exception as e:
                        log("[angle] could not save frame: %r" % (e,))
            snaps += len(saved)
            r = score_frames(frames, det, qf, cfg, args.payload, tiles)
            r["angle"] = str(ang)
            r["verdict"] = evaluate(r, args.payload)
            r["frames_saved"] = len(saved)
            rows.append(r)
            log("[angle] %s deg -> %d frames, %s" % (ang, r["frames"], r["verdict"]))
    finally:
        idx.close()

    print_table(rows, args.payload, args.dist, expected)
    if args.save_frames:
        log("[angle] saved %d frames under %s (annotate these for real IoU scoring)"
            % (snaps, args.save_frames))
    if args.json or args.md:
        write_json_md(rows, meta, args.json, args.md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
