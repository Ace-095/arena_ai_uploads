#!/usr/bin/env python3
"""Tests for tools/make_calib_set.py — the calibration set the Hailo DFC
quantizes against.

Why these matter: quantization ranges are computed FROM this set, so the two
ways it can quietly ruin a model are (a) the images are not what the runtime
feeds (different scaling/padding → the ranges describe a different input), and
(b) the set is monotonous or full of copies (narrow ranges → the small, far
QR's activations clip first, and it is the first thing to vanish at 15 m).
Both are checked here, plus the dedupe bug that ate exposure variety.

No camera and no Hailo runtime needed — pure cv2/numpy.
"""
import contextlib
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import cv2  # noqa: E402
import detector as D  # noqa: E402
import make_calib_set as M  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name,
                        extra if not cond else ""))
    if not cond:
        FAILS.append(name)


def _scene(mean, seed=0, size=(1520, 2028), rect=True):
    rng = np.random.default_rng(seed)
    img = rng.normal(mean, 40, (size[0], size[1], 3)).clip(0, 255).astype(np.uint8)
    if rect:
        cv2.rectangle(img, (800, 600), (1000, 800), (255, 255, 255), -1)
    return img


# ------------------------------------------------- 1. same input as runtime --
def test_letterbox_matches_the_runtime():
    """The calib images must be letterboxed EXACTLY like the runtime, or the
    compiler computes ranges for a distribution the network never sees in
    flight. The expectation is written out here independently (scale, integer
    truncation, INTER_LINEAR, centred black bars) so this test would catch the
    runtime changing under the tool."""
    for shape in [(1520, 2028, 3), (480, 640, 3), (1000, 1000, 3),
                  (200, 1500, 3)]:
        frame = _scene(120, size=shape[:2])
        got = M.letterbox_to(frame, 640)
        h, w = frame.shape[:2]
        s = 640.0 / max(w, h)
        nw, nh = int(w * s), int(h * s)
        exp = np.zeros((640, 640, 3), np.uint8)
        exp[(640 - nh) // 2:(640 - nh) // 2 + nh,
            (640 - nw) // 2:(640 - nw) // 2 + nw] = cv2.resize(
                frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        check("letterbox %s: shape + scale + interpolation + centring"
              % (list(shape),),
              got.shape == (640, 640, 3) and
              int(np.abs(got.astype(int) - exp.astype(int)).max()) == 0,
              (got.shape, int(np.abs(got.astype(int) - exp.astype(int)).max())))
        if nh < 640:
            check("the letterbox pads with pure black (%s)" % (list(shape),),
                  int(got[:max(1, (640 - nh) // 2 - 1)].max()) == 0)
    check("it really is the runtime function, not a copy",
          M._runtime_letterbox is D._letterbox)


# --------------------------------------------------- 2. frame screening -----
def test_quality_gate():
    dark = np.full((400, 600, 3), 3, np.uint8)
    blank = np.full((400, 600, 3), 128, np.uint8)
    blown = np.full((400, 600, 3), 254, np.uint8)
    good = _scene(128, size=(400, 600))
    check("a black frame is refused",
          M.keepable(*M.frame_quality(dark))[0] is False)
    check("a blank wall is refused (no detail to calibrate with)",
          M.keepable(*M.frame_quality(blank))[0] is False)
    check("a blown-out frame is refused",
          M.keepable(*M.frame_quality(blown))[0] is False)
    ok, why = M.keepable(*M.frame_quality(good))
    check("a normal scene is kept", ok, why)


def test_dedupe_keeps_exposure_variety():
    """The bug this test was written for: three differently-exposed shots of the
    same scene are all dropped as "duplicates", leaving a one-frame set. Same
    scene AND same exposure is a duplicate; same scene at a different exposure
    is exactly what the compiler needs."""
    kept = []
    scenes = [_scene(80, seed=1), _scene(140, seed=1), _scene(200, seed=1)]
    for f in scenes:
        key, mean = M.signature(f)
        if not M.too_similar(key, mean, kept):
            kept.append((key, mean))
    check("three exposures of one scene are all kept", len(kept) == 3, len(kept))

    # ...while true copies still go
    copy = _scene(140, seed=1)
    key, mean = M.signature(copy)
    check("an identical frame is dropped as a duplicate",
          M.too_similar(key, mean, kept) is True)

    # and a different scene at the same brightness is kept
    other = _scene(140, seed=99, rect=False)
    key2, mean2 = M.signature(other)
    check("a different scene at the same brightness is kept",
          M.too_similar(key2, mean2, kept) is False)


def test_summary_flags_a_weak_set():
    few = [(100.0, 30.0)] * 5
    text = "\n".join(M.summarize(few, 256))
    check("a tiny set is flagged with the recommended range",
          "WARN" in text and "64-1024" in text, text)
    monotone = [(100.0, 25.0 + (i % 11)) for i in range(200)]
    text2 = "\n".join(M.summarize(monotone, 256))
    check("a same-brightness set is flagged",
          "nearly the same brightness" in text2, text2)
    good = [(60.0 + i * 1.2, 20.0 + (i % 9)) for i in range(200)]
    text3 = "\n".join(M.summarize(good, 256))
    check("a varied set passes with a clear OK",
          "OK" in text3 and "WARN" not in text3, text3)
    check("an empty set is not a crash", M.summarize([], 256))


def test_cli_writes_a_usable_set():
    """End to end through main(): files in, numbered JPEGs + npz out, and the
    compile line that mentions the arch warning."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, "src")
        out = os.path.join(td, "calib")
        os.makedirs(src)
        for i in range(4):
            cv2.imwrite(os.path.join(src, "f%d.jpg" % i),
                        _scene(70 + i * 40, seed=i + 1))
        cv2.imwrite(os.path.join(src, "dark.jpg"), np.zeros((400, 600, 3),
                                                            np.uint8))
        argv = sys.argv[:]
        sys.argv = ["make_calib_set.py", "--from", src, "--out", out,
                    "--want", "10", "--npz"]
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                code = M.main()
        finally:
            sys.argv = argv
        printed = buf.getvalue()
        files = sorted(os.listdir(out)) if os.path.isdir(out) else []
        check("it exits 0 and writes one JPEG per kept frame",
              code == 0 and len(files) >= 3, (code, files))
        check("the written frames are the letterboxed 640x640 input",
              bool(files) and cv2.imread(os.path.join(out, files[0])).shape ==
              (640, 640, 3),
              cv2.imread(os.path.join(out, files[0])).shape if files else None)
        check("the npz is written when asked",
              os.path.isfile(os.path.join(td, "calib.npz")))
        # The tool's whole job is to hand the user something they can paste on
        # the x86 box, so assert on what it PRINTS, not on the source text.
        check("it prints the exact optimize line",
              "hailo optimize --hw-arch <hailo8|hailo8l>" in printed
              and "--calib-path %s" % out in printed
              and "--model-script yolov8n.alls" in printed
              and "-o qr_yolov8n_quantized.har qr_yolov8n.har" in printed,
              printed)
        check("the arch warning is printed (hailo8 vs hailo8l)",
              "hailo8l" in printed and "not interchangeable" in printed,
              printed)
        check("it says which model script edits are needed",
              "compile_hef.md" in printed and "nms_config.json" in printed,
              printed)


if __name__ == "__main__":
    test_letterbox_matches_the_runtime()
    test_quality_gate()
    test_dedupe_keeps_exposure_variety()
    test_summary_flags_a_weak_set()
    test_cli_writes_a_usable_set()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all calib-set tests passed")
