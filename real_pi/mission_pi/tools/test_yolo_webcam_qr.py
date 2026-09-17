#!/usr/bin/env python3
"""QR webcam bench tool — the window-grill killer with live presets.

Watches a webcam (or a video file / RTSP URL), runs the SAME detector +
fake-QR filter + preset table the mission uses (detector.py, decoder.py,
qr_filter.py — no duplicated logic), and draws:

  * GREEN thick box + "QR 0.85 MISSION-QR-001"  — a box that PASSED the
    filter AND decoded to a real payload (the only boxes that matter)
  * GREEN thin box + "qr 0.52"                  — filter survivor (bbox cue)
  * ORANGE thin box + "fake 0.24 (aspect ...)"  — a REJECTED box (grill,
    speck, odd shape); shown only while fakes are not hidden (key f)

HUD (top-left): preset, conf, tile grid, require-decode, filter floor,
`raw:N filt:M` counts (raw detector boxes vs filter survivors) + fps.

Why: a diamond window grill read as "qr" at conf 0.16-0.24 and filled the
view with orange boxes. Default conf is now 0.35 + a geometry gate
(square aspect 0.4-2.5, min 15 px side). On the ground, use a
require-decode preset: only boxes that decode to a real payload count.

USAGE (from the mission_pi/ directory or the repo root):
  python3 tools/test_yolo_webcam_qr.py --source 0 \
      --model models/qr_yolov8n.onnx --onnx \
      --preset dark --require-decode --tiled

  # headless bench run (no window; prints one summary line per second)
  python3 tools/test_yolo_webcam_qr.py --source /tmp/demo.mp4 \
      --model models/qr_yolov8n.onnx --headless

LIVE KEYS (window mode):
  f  hide/show rejected (fake) boxes        v  toggle require-decode
  k  conf +0.05                              l  conf -0.05
  t  toggle 3x3 tiling                       p  next preset
  1-7 select preset by number                q / ESC quit

Ground recommendation (field report): preset `dark` (a.k.a.
dark_qr_boost) + --tiled for night; preset `kabaddi` at ground level.
Flight 15 m search: preset `day` (default).
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

try:
    import yaml
except Exception:
    yaml = None

PRESET_KEYS = "1234567"  # key -> preset list position


def load_cfg(path):
    if path and os.path.isfile(path) and yaml is not None:
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict):
                return data
        except Exception as e:
            print("[presets] could not read %s (%r) — using built-ins" % (path, e))
    return {}


def build_detector(args, conf_thr, prefer):
    """Use the repo's detector ladder so the tool == the mission."""
    from detector import get_detector
    cfg = {
        "kind": "yolo",
        "model_path": args.model or "",
        "conf_thr": conf_thr,
        "iou_thr": args.iou,
        "input_size": 640,
        "prefer": prefer,
    }
    try:
        return get_detector(cfg)
    except Exception as e:
        print("[detector] YOLO unavailable (%r) — falling back to classical" % e)
        from detector import ClassicalDetector
        return ClassicalDetector()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0",
                    help="camera index (0), device path (/dev/video0), "
                         "RTSP/HTTP URL, or video file (default: 0)")
    ap.add_argument("--model", default=None,
                    help="YOLO weights (models/qr_yolov8n.onnx or .pt); "
                         "default: auto-find in mission_pi/models/")
    ap.add_argument("--onnx", action="store_true",
                    help="prefer the .onnx weights (onnxruntime, no torch)")
    ap.add_argument("--pt", action="store_true",
                    help="prefer the .pt weights (ultralytics)")
    ap.add_argument("--classical", action="store_true",
                    help="no model — cv2/pyzbar detector (short range only)")
    ap.add_argument("--conf", type=float, default=None,
                    help="detector conf floor (default: the preset's conf)")
    ap.add_argument("--iou", type=float, default=0.45, help="NMS iou (default 0.45)")
    ap.add_argument("--preset", default="day",
                    help="qr_presets entry: day|far|dark|kabaddi|low|"
                         "aggressive|bench (aliases: dark_qr_boost, night, "
                         "ground, 15m). 'none' = plain --conf, no preset")
    ap.add_argument("--require-decode", action="store_true",
                    help="only a box that DECODED to a payload counts "
                         "(ground runs: only real QRs trigger)")
    ap.add_argument("--no-require-decode", dest="require_decode",
                    action="store_false")
    ap.set_defaults(require_decode=None)  # None = follow the preset
    ap.add_argument("--tiled", action="store_true",
                    help="force 3x3 tile inference (15 m answer)")
    ap.add_argument("--no-tile", action="store_true",
                    help="force full-frame inference (bench / close range)")
    ap.add_argument("--config", default=None,
                    help="config.yaml to read qr_presets/qr_filter from "
                         "(default: mission_pi/config.yaml next to this tool)")
    ap.add_argument("--headless", action="store_true",
                    help="no window — one summary line per second (bench CI)")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (0 = run until key/file end)")
    args = ap.parse_args()

    cfg = load_cfg(args.config or os.path.join(_ROOT, "config.yaml"))
    from qr_filter import (QrFilter, load_presets, normalize_preset_name,
                           preset_names)
    qf = QrFilter.from_config(cfg)
    presets = load_presets(cfg)
    names = preset_names(cfg)
    if not names:
        names = list(presets.keys())

    # ---- preset selection -------------------------------------------------
    active = normalize_preset_name(args.preset)
    if args.preset and args.preset.lower() != "none" and active not in presets:
        print("[preset] %r unknown — available: %s — starting on 'day'"
              % (args.preset, ", ".join(names)))
        active = "day"
    preset = presets.get(active)
    if args.require_decode is not None:  # CLI flag wins over the preset
        qf.require_decode_for_cue = args.require_decode
    elif preset:
        qf.require_decode_for_cue = preset["require_decode_for_cue"]
    conf = args.conf if args.conf is not None else (preset or {}).get(
        "conf_thr", qf.min_conf)
    tiled = preset is not None and (preset.get("tile_grid", [3, 3]) != [1, 1])
    if args.no_tile:
        tiled = False
    if args.tiled:
        tiled = True

    # ---- detector ----------------------------------------------------------
    if args.classical:
        from detector import ClassicalDetector
        det = ClassicalDetector()
    else:
        if args.pt:
            prefer = ["pt"]
        elif args.onnx or (args.model or "").lower().endswith(".onnx"):
            prefer = ["onnx"]
        else:
            prefer = ["onnx", "pt"]
        det = build_detector(args, conf, prefer)
    print("[detector] %s (conf floor %.2f)" % (det.name, conf))
    print("[filter]   min_conf=%.2f min_side=%dpx aspect=%.1f-%.1f "
          "require_decode_cue=%s"
          % (qf.min_conf, qf.min_side_px, qf.aspect_min, qf.aspect_max,
             qf.require_decode_for_cue))
    print("[preset]   %s (keys 1-%d: %s)"
          % (active or "none", len(names), ", ".join(names)))

    # ---- capture -----------------------------------------------------------
    import cv2
    from decoder import decode_frame
    from detector import detect_tiles
    src = args.source
    cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
    if not cap.isOpened():
        print("ERROR: cannot open source %r (need cv2 with V4L2/GStreamer, "
              "or a file/URL)" % src)
        return 2
    is_file = os.path.isfile(str(src))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    print("[camera] %dx%d source=%s tiled=%s" % (W, H, src, tiled))

    hide_fakes = False
    n_frames = 0
    n_raw = 0
    n_filt = 0
    t_prev = time.time()
    fps = 0.0
    t_last_line = 0.0

    def hud(frame, raw_n, filt_n, payload_any):
        lines = [
            "preset: %-10s conf %.2f  tile %s  req-decode: %s"
            % (active or "none", conf, "3x3" if tiled else "off",
               "ON " if qf.require_decode_for_cue else "off"),
            "filter: conf>=%.2f side>=%dpx aspect %.1f-%.1f | raw:%d filt:%d | %.1f fps"
            % (qf.min_conf, qf.min_side_px, qf.aspect_min, qf.aspect_max,
               raw_n, filt_n, fps),
            "keys: f fakes  v req-decode  k/l conf  t tile  p 1-7 preset  q quit",
        ]
        y = 26
        for ln in lines:
            cv2.putText(frame, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += 20
        if payload_any:
            cv2.putText(frame, "QR CONFIRMED: %s" % payload_any, (8, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, "QR CONFIRMED: %s" % payload_any, (8, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

    while True:
        ok, frame = cap.read()
        if not ok:
            if is_file:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop the file
                ok, frame = cap.read()
            if not ok or frame is None:
                print("[source] end of stream/file — exiting")
                break
        n_frames += 1
        now = time.time()
        dt = max(1e-3, now - t_prev)
        t_prev = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

        # ---- detect (tiled for 15 m) ----
        try:
            if tiled and det.name != "classical":
                raw = detect_tiles(det, frame, rows=3, cols=3,
                                   overlap=0.25, iou_thr=args.iou)
            else:
                raw = det.detect(frame)
        except Exception as e:
            print("[detect] %r" % e)
            raw = []
        n_raw += len(raw)
        # ---- fake-QR filter (the grill killer) ----
        kept = qf.filter_boxes(raw)
        n_filt += len(kept)
        kept_ids = set(id(b) for b in kept)
        rejected = [b for b in raw if id(b) not in kept_ids]

        # ---- decode survivors (best conf first) ----
        payload_any = None
        payload_box = None
        for b in sorted(kept, key=lambda b: -b.conf)[:3]:
            p = decode_frame(frame, (b.x, b.y, b.w, b.h))
            if p:
                payload_any = p
                payload_box = b
                break
        if qf.require_decode_for_cue and payload_box is None:
            kept = []  # strict: nothing counts until a box actually decodes

        # ---- draw ----
        for b in rejected:
            if hide_fakes:
                continue
            reason = qf.should_keep(b)[1]
            x, y, w, h = int(b.x), int(b.y), int(b.w), int(b.h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 1)
            cv2.putText(frame, "fake %.2f (%s)" % (b.conf, reason),
                        (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 140, 255), 1, cv2.LINE_AA)
        for b in kept:
            x, y, w, h = int(b.x), int(b.y), int(b.w), int(b.h)
            if b is payload_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.putText(frame, "QR %.2f %s" % (b.conf, payload_any),
                            (x, max(16, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, "QR %.2f %s" % (b.conf, payload_any),
                            (x, max(16, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 255, 0), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.putText(frame, "qr %.2f" % b.conf, (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1,
                            cv2.LINE_AA)
        hud(frame, len(raw), len(kept), payload_any)

        if args.headless:
            if now - t_last_line >= 1.0:
                t_last_line = now
                print("[bench] frame=%d raw=%d filt=%d payload=%r fps=%.1f"
                      % (n_frames, n_raw, n_filt, payload_any, fps))
        else:
            cv2.imshow("QR bench (f/v/k/l/t/p/1-7/q)", frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            elif k == ord("f"):
                hide_fakes = not hide_fakes
                print("[key] hide fakes: %s" % hide_fakes)
            elif k == ord("v"):
                qf.require_decode_for_cue = not qf.require_decode_for_cue
                print("[key] require_decode_for_cue: %s" % qf.require_decode_for_cue)
            elif k == ord("k"):
                conf = round(min(0.99, conf + 0.05), 2)
                det.set_thresholds(conf_thr=conf)
                print("[key] conf -> %.2f" % conf)
            elif k == ord("l"):
                conf = round(max(0.05, conf - 0.05), 2)
                det.set_thresholds(conf_thr=conf)
                print("[key] conf -> %.2f" % conf)
            elif k == ord("t"):
                tiled = not tiled
                print("[key] tiled: %s" % tiled)
            elif k == ord("p"):
                active = names[(names.index(active) + 1) % len(names)] \
                    if active in names else names[0]
                _apply_preset = presets.get(active)
                if _apply_preset:
                    conf = _apply_preset["conf_thr"]
                    det.set_thresholds(conf_thr=conf)
                    qf.require_decode_for_cue = _apply_preset["require_decode_for_cue"]
                    tiled = _apply_preset.get("tile_grid", [3, 3]) != [1, 1]
                print("[key] preset -> %s (conf %.2f, req-decode %s, tile %s)"
                      % (active, conf, qf.require_decode_for_cue, tiled))
            elif k in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5"),
                       ord("6"), ord("7")):
                idx = ord(k) - ord("1")
                if idx < len(names):
                    active = names[idx]
                    _apply_preset = presets.get(active)
                    if _apply_preset:
                        conf = _apply_preset["conf_thr"]
                        det.set_thresholds(conf_thr=conf)
                        qf.require_decode_for_cue = \
                            _apply_preset["require_decode_for_cue"]
                        tiled = _apply_preset.get("tile_grid", [3, 3]) != [1, 1]
                    print("[key] preset -> %s (conf %.2f, req-decode %s, tile %s)"
                          % (active, conf, qf.require_decode_for_cue, tiled))

        if args.max_frames and n_frames >= args.max_frames:
            print("[bench] max-frames reached (%d)" % n_frames)
            break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()
    print("[done] frames=%d raw_boxes=%d filter_kept=%d (rejected %d)"
          % (n_frames, n_raw, n_filt, n_raw - n_filt))
    return 0


if __name__ == "__main__":
    sys.exit(main())
