#!/usr/bin/env python3
"""make_demo_frames.py — synthetic camera frames for bench/UI testing.

Writes a directory of JPEG/PNG frames (a QR panel on a ground-ish
background, moving + scaling a little frame to frame) that
`cameras.<role>.kind: file` plays back as a camera. Two uses:

  1. DEMO / link testing with no hardware and no simulator pixels:
         python3 tools/make_demo_frames.py --out demo_frames
         python3 main.py --config config.demo.yaml
     The mission detects + decodes the frames, so the whole chain
     (FC link -> trigger -> search -> decode -> consensus -> TRANSMIT ->
     STATUSTEXT to MP + `qr` event to the UI) runs end to end.

  2. Vision regression on the laptop: re-generate frames at a chosen pixel
     size and watch the detector/decoder cope (tools/qr_bench.py eats the
     same directory).

Needs: pip install qrcode pillow opencv-python   (or apt python3-opencv).
"""
import argparse
import math
import os
import sys

try:
    import numpy as np
    import cv2
except Exception as e:
    print("needs opencv + numpy:  pip install opencv-python numpy  (%r)" % (e,))
    sys.exit(2)

try:
    import qrcode
    from PIL import Image
except Exception as e:
    print("needs qrcode + pillow:  pip install qrcode pillow  (%r)" % (e,))
    sys.exit(2)


def qr_bgr(text, box_size=10, border=4):
    img = qrcode.make(str(text), box_size=box_size, border=border)
    arr = np.array(img.convert("L"))
    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)


def ground(w, h, seed=0):
    """Flat-ish textured ground so the frames are not a uniform block."""
    rng = np.random.default_rng(seed)
    base = np.full((h, w, 3), 96, np.uint8)
    noise = rng.integers(-14, 14, size=(h, w, 1)).astype(np.int16)
    img = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    img[:] = (40, 90, 50)                     # grassy green
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    for i in range(0, w, 64):                 # faint mowing stripes
        cv2.line(img, (i, 0), (i, h), (60, 110, 70), 6)
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description="synthetic QR camera frames")
    ap.add_argument("--text", default="MISSION-QR-001", help="QR payload")
    ap.add_argument("--out", default="demo_frames", help="output directory")
    ap.add_argument("--n", type=int, default=12, help="frames to write")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--px-min", type=int, default=140,
                    help="smallest QR side in pixels (a 'far' sighting)")
    ap.add_argument("--px-max", type=int, default=300,
                    help="largest QR side (the approach stair closing in)")
    ap.add_argument("--jpg", action="store_true", help="write .jpg not .png")
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    qr = qr_bgr(args.text)
    n = max(2, args.n)
    ext = ".jpg" if args.jpg else ".png"
    for i in range(n):
        f = i / float(n - 1)
        frame = ground(args.width, args.height, seed=i)
        px = int(args.px_min + (args.px_max - args.px_min) * f)
        tile = cv2.resize(qr, (px, px), interpolation=cv2.INTER_AREA)
        # wander a little so the cue is not pinned to one pixel
        cx = int(args.width * (0.5 + 0.06 * math.sin(i * 1.1)))
        cy = int(args.height * (0.5 + 0.05 * math.cos(i * 0.9)))
        x0 = max(0, min(args.width - px, cx - px // 2))
        y0 = max(0, min(args.height - px, cy - px // 2))
        frame[y0:y0 + px, x0:x0 + px] = tile
        # a second, decoy blob: proves the detector is not just "any square"
        cv2.rectangle(frame, (40, args.height - 120), (140, args.height - 40),
                      (200, 200, 200), -1)
        cv2.putText(frame, "frame %d/%d" % (i + 1, n), (20, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        path = os.path.join(args.out, "frame_%03d%s" % (i, ext))
        if args.jpg:
            cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        else:
            cv2.imwrite(path, frame)
    print("wrote %d frame(s) to %s/  (QR %r, %d..%d px, %dx%d)"
          % (n, args.out, args.text, args.px_min, args.px_max,
             args.width, args.height))
    print("play them back with:  cameras: { bottom: { kind: file, path: %s, "
          "fps: 4, size: [%d, %d] } }" % (args.out, args.width, args.height))
    return 0


if __name__ == "__main__":
    sys.exit(main())
