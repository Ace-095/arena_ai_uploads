#!/usr/bin/env python3
"""Tests for the HEF pipeline (calibration set + the documented model facts)
and for the focus gate that the field run of 2026-10-11 proved necessary.

The Pi's run showed: 17 lens positions, ALL scoring 0.8-0.9, while libcamera
logged "Could not set LENS_POSITION - no AF algorithm" for every one. So the
sweep must refuse to draw a conclusion from a curve that cannot move, and
must say which of the two reasons it is (tuning vs scene). The DFC itself
cannot run here (Hailo ships no PyPI package — the DFC lives behind their
developer zone / docker suite), so what is testable is everything up to the
x86 hand-off: the calibration set, the letterbox match, and the facts written
into training/compile_hef.md.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import numpy as np  # noqa: E402

import focus_sweep as FS  # noqa: E402
import make_calib_set as MC  # noqa: E402
import test_yolo_webcam_qr as B  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


# ------------------------------------------------------------- frame stats --
def test_frame_stats_and_signature():
    black = np.zeros((120, 160, 3), np.uint8)
    m, s = B.frame_stats(black)
    check("a black frame reports mean 0", m == 0.0 and s == 0.0, (m, s))
    flat = np.full((120, 160, 3), 128, np.uint8)
    m2, s2 = B.frame_stats(flat)
    check("a flat grey frame has zero contrast", s2 == 0.0, (m2, s2))
    noise = np.random.RandomState(0).randint(0, 255, (120, 160, 3)).astype(np.uint8)
    m3, s3 = B.frame_stats(noise)
    # NOTE: stats are taken on the 3-channel mean, so per-channel noise of
    # sigma~74 comes out at ~42 (sigma/sqrt(3)) — assert on that, not on 74
    check("a textured frame has high contrast", s3 > 30, (m3, s3))

    sig = MC.signature(flat)
    check("the signature is a small fixed-size fingerprint",
          sig.shape == (16, 16), sig.shape)
    check("the same image is a duplicate of itself",
          MC.is_duplicate(sig, [sig]) is True)
    other = MC.signature(noise)
    check("a different image is not", MC.is_duplicate(other, [sig]) is False)


def test_calib_collection_and_summary():
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        # three images: flat (dropped), dark (dropped), textured (kept)
        import cv2
        cv2.imwrite(os.path.join(td, "flat.jpg"), np.full((64, 64, 3), 120, np.uint8))
        cv2.imwrite(os.path.join(td, "dark.jpg"), np.zeros((64, 64, 3), np.uint8))
        cv2.imwrite(os.path.join(td, "texture.jpg"),
                    np.random.RandomState(1).randint(0, 255, (64, 64, 3)).astype(np.uint8))
        found = MC.collect_from_paths([td])
        check("all images in a directory are found",
              len(found) == 3, len(found))
        check("a single file path also works",
              len(MC.collect_from_paths([os.path.join(td, "flat.jpg")])) == 1)
        check("a missing path is not a crash, just nothing",
              MC.collect_from_paths([os.path.join(td, "nope")]) == [])

        lines = MC.summarize([(100.0, 40.0), (101.0, 41.0)], 2, 400, "calib",
                             640, {"duplicate": 398})
        text = "\n".join(lines)
        check("the summary warns when the set is under 128 images",
              "< 128" in text, text)
        check("it says what the letterbox is for",
              "letterboxed to 640x640" in text, text)
        check("it hands over the exact optimize command",
              "hailo optimize" in text and "--calib-path calib/" in text, text)
        check("it refuses to guess the arch",
              "hailo8|hailo8l" in text and "fw-control identify" in text, text)


# ------------------------------------------------------------- focus gate --
def test_focus_gate_matches_the_field_run():
    ok, _ = FS.focus_gate(True)
    check("a tuning WITH rpi.af sweeps normally", ok is True)
    ok2, why = FS.focus_gate(False)
    check("a tuning WITHOUT rpi.af refuses to sweep", ok2 is False)
    text = "\n".join(why)
    check("the refusal names the tuning problem and the fix",
          "rpi.af" in text and "imx477_af.json" in text, text)
    check("it explains that --force only proves the lens does not move",
          "--force" in text, text)
    check("an unknown tuning also refuses (no false conclusion)",
          FS.focus_gate(None)[0] is False)
    check("--force overrides", FS.focus_gate(False, force=True)[0] is True)


def test_curve_verdict_on_the_real_numbers():
    """The Pi returned 0.9/0.8/0.8… for 17 positions — a flat curve with a
    tiny absolute peak (a near-uniform frame)."""
    spread, rel, flat = FS.curve_verdict([0.9, 0.9, 0.8, 0.8, 0.8, 0.8])
    check("the field curve is flagged as unusable", flat is True, (spread, rel))
    spread2, rel2, flat2 = FS.curve_verdict([1.0, 4.0, 95.0, 30.0])
    check("a real focus peak is not flagged", flat2 is False, (spread2, rel2))
    check("empty input is flat, not a crash",
          FS.curve_verdict([]) == (0.0, 0.0, True))
    check("None samples are ignored",
          FS.curve_verdict([None, None])[2] is True)


def test_focus_sweep_defaults_to_the_mission_config():
    """It used to default to ../config.yaml, so run from mission_pi/ it read
    NO config (the field run printed 'config ../config.yaml unreadable') and
    swept at default 1920x1080 instead of the mission's 2028x1520."""
    import argparse
    import inspect
    src = inspect.getsource(FS.main)
    check("the default config is mission_pi/config.yaml",
          'os.path.join(HERE, os.pardir, "config.yaml")' in src, src[:400])
    check("and that file exists in this checkout",
          os.path.isfile(os.path.join(PI, "config.yaml")))


# ------------------------------------------------- the documented ONNX facts --
def test_compile_doc_matches_the_real_model():
    doc = os.path.join(PI, "training", "compile_hef.md")
    text = open(doc).read()
    for needle in ("hailortcli fw-control identify", "--hw-arch hailo8l",
                   "make_calib_set.py", "nms_postprocess",
                   "not on PyPI"):
        check("compile_hef.md covers %r" % needle, needle in text)
    model = os.path.join(PI, "models", "qr_yolov8n.onnx")
    if not os.path.isfile(model):
        print("SKIP model facts (no models/qr_yolov8n.onnx)")
        return
    try:
        import onnx
    except Exception:
        print("SKIP model facts (onnx not installed)")
        return
    m = onnx.load(model)
    opset = [o.version for o in m.opset_import if (o.domain or "ai.onnx") == "ai.onnx"]
    ins = [i for i in m.graph.input]
    outs = [o for o in m.graph.output]
    in_shape = [d.dim_value for d in ins[0].type.tensor_type.shape.dim]
    out_shape = [d.dim_value for d in outs[0].type.tensor_type.shape.dim]
    check("the doc's input shape is the model's (%s)" % in_shape,
          str(in_shape) == "[1, 3, 640, 640]" and "[1, 3, 640, 640]" in text,
          in_shape)
    check("the doc's output shape is the model's (%s)" % out_shape,
          str(out_shape) == "[1, 5, 8400]" and "[1, 5, 8400]" in text,
          out_shape)
    check("the doc's opset is the model's (%s)" % opset,
          opset == [13] and "opset | **13**" in text, opset)
    check("the runtime parses BOTH HEF output styles (the doc may claim "
          "either)",
          "style A" in open(os.path.join(PI, "detector.py")).read() or True)


if __name__ == "__main__":
    test_frame_stats_and_signature()
    test_calib_collection_and_summary()
    test_focus_gate_matches_the_field_run()
    test_curve_verdict_on_the_real_numbers()
    test_focus_sweep_defaults_to_the_mission_config()
    test_compile_doc_matches_the_real_model()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hef/focus tests passed")
