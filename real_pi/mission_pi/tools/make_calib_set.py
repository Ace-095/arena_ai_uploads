#!/usr/bin/env python3
"""Build the Hailo DFC calibration set from frames THIS mission actually sees.

Quantization is where a "it worked on my desktop" model quietly loses the small
far-away QR: the compiler computes activation ranges from the calibration
images, so a set of dark indoor frames will clip every bright ground scene. The
set must therefore be captured through the SAME camera/lens/pipeline as the
flight and carry the same framing the runtime feeds:

  * each image is letterboxed to the network input exactly like
    `detector.py` does (test: test_hef_and_focus.py), so calibration sees 640x640
    with the same black padding and scale as inference — a plain resize/squash
    would be a different distribution
  * near-black, blown-out and near-duplicate frames are dropped: 50 copies of
    one frame calibrate nothing (the set is judged by how much it varies)
  * the summary at the end says whether the set is diverse enough, in numbers,
    instead of letting a monotone set look fine

USAGE (on the Pi, from mission_pi/)
  # 1) collect from the real camera (stills, any tool) — e.g.
  python3 tools/cam_probe.py --snap-dir snaps
  #    and/or run the bench and keep its annotated frames' source frames:
  python3 tools/test_yolo_webcam_qr.py --source csi:bottom --no-tile \
      --save-frames /tmp/bench --max-frames 200
  # 2) build the set
  python3 tools/make_calib_set.py --from snaps /tmp/bench --out training/calib
  # 3) that prints the exact `hailo optimize --calib-path …` line to use.

Only cv2 + numpy are needed; no Hailo runtime (the DFC runs on x86).
"""
import argparse
import glob
import os
import sys

import numpy as np

try:
    import cv2
except Exception as e:                       # pragma: no cover
    print("cv2 is required (%r) — the Pi/laptop venv has it" % (e,))
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)

try:                                  # the single source of truth for input prep
    from detector import _letterbox as _runtime_letterbox
except Exception:                     # pragma: no cover - detector unimportable
    _runtime_letterbox = None

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

#: The DFC docs recommend 64-1024 images; fewer than this makes the activation
#: ranges noisy, and the zoo's own flows use ~1000. 128 is the floor where a
#: set stops being mostly luck for a 1-class model.
MIN_IMAGES = 128


# ------------------------------------------------------------------ pure bits --
def letterbox_to(frame, size=640):
    """The runtime's OWN letterbox (detector._letterbox), so calibration sees
    the exact tensor inference sees — same scale, same padding, same
    interpolation. Re-implementing it here would drift the moment the runtime
    changes, and the compiler would then compute ranges for a different input
    distribution. Only a minimal local copy is kept for the case where
    detector.py cannot be imported at all.
    """
    if _runtime_letterbox is not None:
        return _runtime_letterbox(frame, size)[0]
    h, w = frame.shape[:2]
    s = min(size / float(w), size / float(h))
    nw, nh = int(w * s), int(h * s)
    img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    canvas[(size - nh) // 2:(size - nh) // 2 + nh,
           (size - nw) // 2:(size - nw) // 2 + nw] = img
    return canvas


def frame_quality(frame):
    """(mean, std) of the grayscale frame — the two numbers that say whether
    this frame can teach the compiler anything."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(gray.mean()), float(gray.std())


def keepable(mean, std, min_mean=12.0, max_mean=245.0, min_std=10.0):
    """Dark/blown/flat frames are silently useless for calibration."""
    if mean < min_mean:
        return False, "too dark (mean %.0f)" % mean
    if mean > max_mean:
        return False, "blown out (mean %.0f)" % mean
    if std < min_std:
        return False, "flat/blank (std %.0f)" % std
    return True, ""


#: Two frames count as "the same scene" only when they also look equally
#: exposed. Brightness variety is not noise to be deduplicated away — it is
#: exactly what the compiler needs to see, since it drives the activation
#: ranges. Without this, a set captured at three exposures collapses to one.
BRIGHTNESS_TOL = 12.0
SIMILAR_COS = 0.86          # combined key cosine above this = same scene


def signature(frame, bins=32, n=8):
    """(key, mean): a cheap 'is this a new scene?' fingerprint.

    The key mixes the brightness DISTRIBUTION with a coarse layout thumbnail,
    both unit-normalised, so it is not fooled the way a mean-subtracted
    thumbnail alone is: two totally different noisy scenes share that, while
    their histograms do not. `mean` rides along because dedupe must not throw
    away exposure variety (see BRIGHTNESS_TOL).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [bins], [0, 256]).reshape(-1)
    hist = hist / (float(hist.sum()) or 1.0)
    thumb = cv2.cvtColor(cv2.resize(frame, (n, n),
                                    interpolation=cv2.INTER_AREA),
                         cv2.COLOR_BGR2GRAY).astype(np.float32)
    thumb -= thumb.mean()
    thumb /= (float(np.sqrt((thumb ** 2).sum())) or 1.0)
    key = np.concatenate([hist, thumb.reshape(-1)]).astype(np.float32)
    norm = float(np.sqrt((key ** 2).sum())) or 1.0
    return key / norm, float(gray.mean())


def too_similar(key, mean, kept, thresh=0.14, tol=BRIGHTNESS_TOL):
    """kept: [(key, mean), ...] from frames already accepted.

    Same scene AND same exposure -> a duplicate (drop it). Same scene but a
    different exposure -> keep: those frames teach the compiler different
    activation ranges.
    """
    for k, kmean in kept:
        if (float((key * k).sum()) > (1.0 - thresh)
                and abs(mean - kmean) < tol):
            return True
    return False


def summarize(stats, want):
    """stats: [(mean, std), ...] -> human lines about the set's diversity."""
    lines = []
    if not stats:
        return ["BAD  no frames kept — nothing to calibrate with"]
    means = np.array([s[0] for s in stats])
    stds = np.array([s[1] for s in stats])
    lines.append("kept %d frame(s); brightness mean %.0f-%.0f, detail (std) "
                 "%.0f-%.0f" % (len(stats), means.min(), means.max(),
                                stds.min(), stds.max()))
    if len(stats) < MIN_IMAGES:
        lines.append("WARN only %d frames — the DFC docs suggest 64-1024 and "
                     "the zoo flows use ~1000; below ~%d the activation ranges "
                     "get noisy. Capture more scenes, not more copies."
                     % (len(stats), MIN_IMAGES))
    if means.max() - means.min() < 25:
        lines.append("WARN every frame has nearly the same brightness — a set "
                     "from one time of day/location quantizes badly for the "
                     "others. Add ground shots at the distance you hunt from.")
    if stds.max() - stds.min() < 8:
        lines.append("WARN very little variation in scene detail — include "
                     "cluttered ground, not only sky/floor.")
    if not any("WARN" in ln for ln in lines):
        lines.append("OK   the set varies enough to be worth compiling with")
    return lines


# ------------------------------------------------------------------- plumbing --
def iter_paths(paths):
    out = []
    for p in paths:
        if os.path.isdir(p):
            for f in sorted(os.listdir(p)):
                if f.lower().endswith(IMG_EXT):
                    out.append(os.path.join(p, f))
        elif os.path.isfile(p) and p.lower().endswith(IMG_EXT):
            out.append(p)
    return out


def iter_video(path, stride=5):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print("[calib] cannot open video %s" % path)
        return
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        if i % max(1, stride) == 0:
            yield frame
        i += 1
    cap.release()


def collect(args):
    """-> (list of (frame, source, mean, std), skipped count)."""
    frames, skipped = [], 0
    sigs = []

    def consider(frame, src, label):
        nonlocal skipped
        # Quality and diversity are judged on the REAL frame: letterboxing
        # first would add the same black bars to every frame, pull every mean
        # down by the padding ratio and pile the histograms onto 0 — which made
        # genuinely different scenes score as duplicates and dusk frames as
        # "too dark". The padding is applied at SAVE time, so the written
        # images still match the runtime input exactly.
        mean, std = frame_quality(frame)
        ok, why = keepable(mean, std, args.min_mean, args.max_mean,
                           args.min_std)
        if not ok:
            if args.verbose:
                print("[calib] skip %s: %s" % (label, why))
            skipped += 1
            return
        sig = signature(frame)
        if too_similar(sig[0], sig[1], sigs, args.dedupe):
            skipped += 1
            if args.verbose:
                print("[calib] skip %s: near-duplicate of a kept frame"
                      % label)
            return
        sigs.append(sig)
        if args.letterbox:
            frame = letterbox_to(frame, args.letterbox)
        frames.append((frame, src, mean, std))

    for p in iter_paths(args.sources):
        if len(frames) >= args.want:
            break
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print("[calib] unreadable image %s" % p)
            continue
        consider(img, p, p)

    if args.video:
        n = 0
        for frame in iter_video(args.video, args.video_stride):
            if len(frames) >= args.want:
                break
            n += 1
            consider(frame, args.video, "%s#%d" % (args.video, n))

    return frames, skipped


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="sources", nargs="*", default=[],
                    help="directories/files of frames (jpg/png)")
    ap.add_argument("--video", default="", help="video file to sample frames from")
    ap.add_argument("--video-stride", type=int, default=5,
                    help="take every Nth video frame (default 5)")
    ap.add_argument("--out", default="training/calib",
                    help="output directory for the .jpg set (default "
                         "training/calib)")
    ap.add_argument("--want", type=int, default=256,
                    help="how many frames to keep (default 256)")
    ap.add_argument("--letterbox", type=int, default=640,
                    help="letterbox each frame to NxN like the runtime does "
                         "(0 = keep native resolution)")
    ap.add_argument("--min-mean", type=float, default=12.0)
    ap.add_argument("--max-mean", type=float, default=245.0)
    ap.add_argument("--min-std", type=float, default=10.0,
                    help="drop frames flatter than this (blank wall/ceiling)")
    ap.add_argument("--dedupe", type=float, default=0.14,
                    help="0 = keep everything, higher = drop more near-duplicates")
    ap.add_argument("--npz", action="store_true",
                    help="also write <out>.npz with an (N,H,W,3) uint8 array "
                         "for flows that want a numpy calibration set")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not args.sources and not args.video:
        ap.print_help()
        print("\ngive --from <dir> and/or --video <file>")
        return 2

    frames, skipped = collect(args)
    if not frames:
        print("[calib] nothing kept — are the paths right, and are the frames "
              "above --min-mean/--min-std?")
        return 3

    os.makedirs(args.out, exist_ok=True)
    for i, (frame, _src, _m, _s) in enumerate(frames, 1):
        cv2.imwrite(os.path.join(args.out, "calib_%04d.jpg" % i), frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 95])

    print("[calib] wrote %d frame(s) to %s (skipped %d)"
          % (len(frames), args.out, skipped))
    for ln in summarize([(m, s) for (_f, _s2, m, s) in frames], args.want):
        print("[calib] %s" % ln)

    if args.npz:
        arr = np.stack([f for (f, _s2, _m, _s) in frames]).astype(np.uint8)
        npz = os.path.splitext(args.out.rstrip("/"))[0] + ".npz"
        np.savez_compressed(npz, images=arr)
        print("[calib] wrote %s — images %s (uint8 RGB, N,H,W,3)"
              % (npz, list(arr.shape)))

    print("[calib] now compile with:" )
    print("  hailo optimize --hw-arch <hailo8|hailo8l> \\\n"
          "      --calib-path %s --model-script yolov8n.alls \\\n"
          "      -y -o qr_yolov8n_quantized.har qr_yolov8n.har" % args.out)
    print("[calib] (that model script is the Zoo's yolov8n.alls with the edits "
          "in training/compile_hef.md §2 — 1 class, and nms_postprocess "
          "pointing at training/nms_config.json so the HEF's decoders carry "
          "OUR names: check with tools/hef_info.py --nms-config)")
    print("[calib] (arch comes from `hailortcli fw-control identify` ON THE PI "
          "— hailo8 and hailo8l HEFs are not interchangeable)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
