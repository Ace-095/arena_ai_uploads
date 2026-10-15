#!/usr/bin/env python3
"""Bench tool: run detector + decoder over images, print boxes/payloads/ms.

  python tools/qr_bench.py [--detector classical|onnx|pt|hailo|auto] [--hef ...] [--tiles] img_or_dir...

Use on the Pi to validate a new .hef (or the ONNX model on real frames) before
flight, or on x86 with --detector classical to sanity-check the decode stage.
--detector onnx loads models/qr_yolov8n.onnx through the mission's own
detector ladder, so the number you see here is the number the mission gets.
"""
import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--detector", default="classical",
                    choices=["classical", "onnx", "pt", "yolo", "hailo",
                             "auto", "none"],
                    help="onnx/yolo = CPU YOLO (ONNX; yolo also allows .pt), "
                         "pt = pin PyTorch, hailo = AI HAT (.hef), "
                         "classical = OpenCV QR only, auto = the mission "
                         "ladder (classical here by default: a bare image "
                         "bench has no model unless you ask for one)")
    ap.add_argument("--hef", default="models/qr_yolov8n.hef")
    ap.add_argument("--tiles", action="store_true")
    ap.add_argument("--conf", type=float, default=0.35)
    args = ap.parse_args()
    try:
        import cv2
    except Exception:
        sys.exit("need opencv-python for qr_bench")
    from detector import get_detector, detect_tiles
    from decoder import decode_frame
    # An explicit --detector hailo without the HAT/HEF must SAY SO and still
    # produce a bench number (the CPU ladder is what the mission falls back to
    # as well) — a traceback here tells the operator nothing.
    try:
        det = get_detector({"kind": args.detector, "hef_path": args.hef,
                            "conf_thr": args.conf})
    except Exception as e:
        if args.detector != "hailo":
            raise
        print("[detector] HAILO unavailable (%r)" % e)
        print("[detector] falling back to the CPU ladder — the mission does "
              "the same (detector.kind: auto). Check `hailortcli scan` and "
              "that %s exists (compile it on x86: training/compile_hef.md)."
              % args.hef)
        det = get_detector({"kind": "yolo", "conf_thr": args.conf,
                            "prefer": ["onnx", "pt"]})
    files = []
    for p in args.paths:
        files += sorted(glob.glob(os.path.join(p, "*"))) if os.path.isdir(p) else [p]
    files = [f for f in files if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    tot_ms, n = 0.0, 0
    if not files:
        print("no readable images in %s" % (args.paths,))
        return 2
    try:
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            t0 = time.time()
            boxes = detect_tiles(det, img) if args.tiles else det.detect(img)
            payload = None
            for b in boxes[:3]:
                payload = decode_frame(img, (b.x, b.y, b.w, b.h))
                if payload:
                    break
            if payload is None:
                payload = decode_frame(img)
            ms = (time.time() - t0) * 1000
            tot_ms += ms
            n += 1
            print("%s: %d box(es) %s payload=%r (%.0f ms)" % (
                os.path.basename(f), len(boxes),
                [(b.x, b.y, b.w, b.h) for b in boxes[:3]], payload, ms))
    except KeyboardInterrupt:
        # Ctrl-C mid-decode is normal on a Pi (zbar + 9 tiles is slow): report
        # what was measured instead of a stack trace through pyzbar.
        print("[bench] interrupted after %d frame(s)" % n)
    if not n:
        print("no readable images in %s" % (args.paths,))
        return 2
    print("avg %.1f ms/frame over %d frames (%s)" % (tot_ms / n, n, det.name))
    if args.tiles:
        print("[bench] tiled: that is 9 inferences per frame — --no-tile is "
              "~9x cheaper; the Hailo .hef is the real fix at 15 m")
    return 0


if __name__ == "__main__":
    main()
