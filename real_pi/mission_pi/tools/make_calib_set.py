#!/usr/bin/env python3
"""Build a calibration set for the Hailo Dataflow Compiler from OUR frames.

The DFC runs on x86 only, but the images it calibrates on must look like what
the Pi actually feeds the network — otherwise the quantization ranges are
tuned for the wrong distribution and the .hef detects poorly (the classic
"compiles fine, finds nothing" failure).

So this tool:
  * collects frames from the Pi's own camera, a video, a directory, or a
    mix of all three;
  * drops near-black / blown-out / near-duplicate frames (a monotonous set is
    the #1 cause of "cannot find suitable quantization parameters");
  * LETTERBOXES every frame to the network input (640x640, plain uint8 RGB)
    exactly like `detector.py::_letterbox` does at runtime — so calibration
    and inference see the same thing;
  * writes `calib/NNN.jpg` plus a summary and the exact `hailo optimize`
    command to paste on the x86 box.

USAGE (on the Pi, from mission_pi/)
  # from the frames you already captured (fastest, honest):
  python3 tools/make_calib_set.py --from /tmp/bench snaps --out calib
  # straight off the camera, walking it around the room for ~1 minute:
  python3 tools/make_calib_set.py --camera csi:bottom --frames 200 --out calib
  # from a video file / a folder of training images (the dataset is ideal):
  python3 tools/make_calib_set.py --video /tmp/flight.mp4 --out calib
  python3 tools/make_calib_set.py --from /tmp/qr_dataset_med/images/train --out calib

Requirements on the Pi: numpy + cv2 (already there). No Hailo software needed
— the DFC consumes the folder on x86.
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir))

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def frame_stats(frame):
    """(mean, std) of a BGR frame as plain brightness numbers 0-255.

    Pure (takes any array-like with `.mean()`): a near-black frame calibrates
    nothing and usually means a lens cap or a dark room, so the tools print
    these numbers instead of leaving you to guess.
    """
    import numpy as np
    a = np.asarray(frame)
    if a.ndim == 3:
        a = a.mean(axis=2)
    return float(a.mean()), float(a.std())


def signature(frame, size=16):
    """Tiny grayscale fingerprint for near-duplicate detection."""
    import numpy as np
    a = np.asarray(frame)
    if a.ndim == 3:
        a = a.mean(axis=2)
    h, w = a.shape[:2]
    ys = (np.arange(size) * h // size).astype(int)
    xs = (np.arange(size) * w // size).astype(int)
    small = a[ys][:, xs].astype(np.float32)
    return small


def is_duplicate(sig, kept_sigs, thr=3.0):
    """True when `sig` is within `thr` mean-abs-difference of a kept one."""
    import numpy as np
    for k in kept_sigs:
        if float(np.abs(sig - k).mean()) < thr:
            return True
    return False


def collect_from_paths(paths):
    """Every image file under the given files/directories (sorted)."""
    out = []
    for p in paths:
        if os.path.isdir(p):
            for ext in IMG_EXT:
                out += sorted(glob.glob(os.path.join(p, "*" + ext)))
                out += sorted(glob.glob(os.path.join(p, "*" + ext.upper())))
        elif os.path.isfile(p):
            out.append(p)
    # de-dup while keeping order (a path can be reached twice)
    seen, uniq = set(), []
    for p in out:
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def summarize(stats, kept, candidates, out_dir, size, dropped):
    """Human summary + the command to run next. Pure: takes numbers only."""
    lines = []
    lines.append("[calib] %d candidates -> %d kept (%s) in %s" % (
        candidates, kept, ", ".join("%s %d" % (k, v)
                                    for k, v in sorted(dropped.items()))
        or "nothing dropped", out_dir))
    if stats:
        means = [m for m, _ in stats]
        stds = [s for _, s in stats]
        lines.append("[calib] brightness mean %.1f (%.1f..%.1f), contrast "
                     "%.1f (%.1f..%.1f) — a narrow spread means a monotonous "
                     "set, which is what breaks quantization"
                     % (sum(means) / len(means), min(means), max(means),
                        sum(stds) / len(stds), min(stds), max(stds)))
    if kept < 128:
        lines.append("[calib] WARNING: %d images < 128 — the DFC will still "
                     "run, but 'cannot find suitable quantization params' "
                     "becomes likely. Capture more frames (vary distance, "
                     "lighting and the QR's size in frame)." % kept)
    lines.append("[calib] on the x86 box (DFC venv):")
    lines.append("    hailo optimize --hw-arch <hailo8|hailo8l> --har best.har "
                 "\\")
    lines.append("        --calib-path calib/ --model-script yolov8n.alls \\")
    lines.append("        --output-har-path qr.har")
    lines.append("    (--hw-arch must match `hailortcli fw-control identify` "
                 "on the Pi; see training/compile_hef.md)")
    if size:
        lines.append("[calib] images are letterboxed to %dx%d uint8 RGB — the "
                     "same as detector.py feeds at runtime" % (size, size))
    return lines


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="paths", action="append", default=[],
                    help="image file or directory (repeatable)")
    ap.add_argument("--video", action="append", default=[],
                    help="video file to sample frames from (repeatable)")
    ap.add_argument("--camera", default="",
                    help="csi[:role|:slot|:index] — capture live")
    ap.add_argument("--frames", type=int, default=200,
                    help="max frames to take from --camera/--video (default 200)")
    ap.add_argument("--every", type=float, default=0.25,
                    help="seconds between captured frames (default 0.25)")
    ap.add_argument("--out", default="calib", help="output directory")
    ap.add_argument("--size", type=int, default=640,
                    help="network input size to letterbox to (default 640)")
    ap.add_argument("--native", action="store_true",
                    help="save frames at capture resolution instead of "
                         "letterboxing to --size")
    ap.add_argument("--min-mean", type=float, default=12.0,
                    help="drop frames darker than this (default 12)")
    ap.add_argument("--max-mean", type=float, default=243.0,
                    help="drop frames brighter than this (default 243)")
    ap.add_argument("--min-std", type=float, default=8.0,
                    help="drop flat frames (no texture) below this (default 8)")
    ap.add_argument("--dup-threshold", type=float, default=2.0,
                    help="near-duplicate removal threshold, 0 = keep all "
                         "(default 2.0 on a 16x16 signature)")
    ap.add_argument("--npz", action="store_true",
                    help="also write calib.npz with an (N,size,size,3) uint8 "
                         "'images' array for flows that want a numpy set")
    args = ap.parse_args()

    try:
        import cv2
        import numpy as np
    except Exception as e:
        print("need opencv + numpy on the Pi: %r" % e)
        return 2

    from detector import _letterbox   # the runtime's own letterbox

    sources = list(args.paths)
    files = collect_from_paths(sources)
    mode = "files"
    if not files and not args.video and not args.camera:
        print("nothing to read: give --from <dir>, --video <file> or "
              "--camera csi:bottom")
        return 2

    os.makedirs(args.out, exist_ok=True)
    dropped = {"dark": 0, "blown": 0, "flat": 0, "duplicate": 0,
               "unreadable": 0}
    kept_sigs, stats, n = [], [], 0
    n_cand = 0

    def take(frame, tag):
        nonlocal n, n_cand
        n_cand += 1
        if frame is None:
            dropped["unreadable"] += 1
            return False
        if n >= args.frames:
            return False
        mean, std = frame_stats(frame)
        stats.append((mean, std))
        if mean < args.min_mean:
            dropped["dark"] += 1
            return False
        if mean > args.max_mean:
            dropped["blown"] += 1
            return False
        if std < args.min_std:
            dropped["flat"] += 1
            return False
        if args.dup_threshold > 0:
            sig = signature(frame)
            if is_duplicate(sig, kept_sigs, args.dup_threshold):
                dropped["duplicate"] += 1
                return False
            kept_sigs.append(sig)
        out = frame if args.native else _letterbox(frame, args.size)[0]
        path = os.path.join(args.out, "%04d.jpg" % n)
        if not cv2.imwrite(path, out):
            print("[calib] could not write %s" % path)
            dropped["unreadable"] += 1
            return False
        n += 1
        return True

    for path in files:
        take(cv2.imread(path), path)

    for vid in args.video:
        cap = cv2.VideoCapture(vid)
        if not cap.isOpened():
            print("[calib] cannot open video %s" % vid)
            continue
        step = max(1, int(cap.get(cv2.CAP_PROP_FPS) * args.every))
        idx = 0
        while n < args.frames:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % step == 0:
                take(frame, vid)
            idx += 1
        cap.release()

    if args.camera and n < args.frames:
        try:
            from cameras import CameraRig
            import yaml
            cfg = {}
            conf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     os.pardir, "config.yaml")
            if os.path.isfile(conf_path):
                with open(conf_path) as f:
                    cfg = yaml.safe_load(f) or {}
            from tools.test_yolo_webcam_qr import CSISource
            import time
            src = CSISource(args.camera, cfg)
            while n < args.frames:
                ok, frame = src.read()
                if not ok or frame is None:
                    break
                take(frame, args.camera)
                time.sleep(max(0.0, args.every))
            src.release()
        except Exception as e:
            print("[calib] camera source failed (%r) — files/videos already "
                  "collected are kept" % e)

    if args.npz and n:
        arr = []
        for i in range(n):
            img = cv2.imread(os.path.join(args.out, "%04d.jpg" % i))
            if img is not None:
                arr.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if arr:
            np.savez_compressed(os.path.join(args.out, "calib.npz"),
                                images=np.stack(arr).astype(np.uint8))
            print("[calib] wrote %s/calib.npz %s" % (args.out,
                  np.stack(arr).shape))

    for line in summarize(stats, n, n_cand, args.out,
                          None if args.native else args.size, dropped):
        print(line)
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
