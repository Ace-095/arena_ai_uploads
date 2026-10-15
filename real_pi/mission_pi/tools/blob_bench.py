#!/usr/bin/env python3
"""Tune the blob fallback from a snapshot: score QR-ish rectangles.

  python tools/blob_bench.py snap.jpg --alt 15 --hfov 68
Prints candidates (box/score/metrics) + writes snap.blob.jpg annotated.
Snap a sim frame from http://127.0.0.1:8099/bottom.mjpg (or :8000 tiles)
and iterate here before touching blob_fallback weights/thresholds.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    ap = argparse.ArgumentParser(description="score blob candidates in an image")
    ap.add_argument("image")
    ap.add_argument("--alt", type=float, default=15.0)
    ap.add_argument("--hfov", type=float, default=68.0)
    ap.add_argument("--min-area", type=float, default=400)
    args = ap.parse_args()
    try:
        import cv2
    except Exception as e:
        print("need opencv (venv): %r" % (e,))
        return 2
    img = cv2.imread(args.image)
    if img is None:
        print("cannot read %r" % args.image)
        return 2
    from blob_fallback import find_candidates
    h, w = img.shape[:2]
    ctx = {"alt_m": args.alt, "hfov_deg": args.hfov, "img_w": w, "img_h": h}
    cands = find_candidates(img, {"min_area_px": args.min_area}, ctx)
    print("%s (%dx%d @ %.0fm): %d candidates" % (args.image, w, h, args.alt, len(cands)))
    for (x, y, ww, hh, sc, m) in cands:
        print("  box=(%d,%d,%d,%d) score=%.3f %s" % (x, y, ww, hh, sc, m))
        cv2.rectangle(img, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
        cv2.putText(img, "%.2f" % sc, (x, max(12, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    root, ext = os.path.splitext(args.image)
    out = root + ".blob" + (ext or ".jpg")
    cv2.imwrite(out, img)
    print("annotated: %s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
