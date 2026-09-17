#!/usr/bin/env python3
"""Bench tool: run detector + decoder over images, print boxes/payloads/ms.

  python tools/qr_bench.py [--detector classical|hailo] [--hef ...] [--tiles] img_or_dir...

Use on the Pi to validate a new .hef before flight, or on x86 with
--detector classical to sanity-check the decode stage.
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
    ap.add_argument("--detector", default="classical")
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
    det = get_detector({"kind": args.detector, "hef_path": args.hef,
                        "conf_thr": args.conf})
    files = []
    for p in args.paths:
        files += sorted(glob.glob(os.path.join(p, "*"))) if os.path.isdir(p) else [p]
    files = [f for f in files if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    tot_ms, n = 0.0, 0
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
    if n:
        print("avg %.1f ms/frame over %d frames (%s)" % (tot_ms / n, n, det.name))


if __name__ == "__main__":
    main()
