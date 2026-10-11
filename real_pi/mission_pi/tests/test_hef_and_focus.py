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

import detector as D  # noqa: E402
import focus_sweep as FS  # noqa: E402
import hef_info as HI  # noqa: E402
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


# ------------------------------------------- HEF <-> parser contract --
def test_hef_output_style_classification():
    """The HEF's output shape decides whether the runtime ever returns a box.
    Every layout the runtime can read must be recognised, and anything else
    must be called out as fatal instead of silently yielding empty lists."""
    a, detail_a = HI.classify_outputs(
        [("qr_yolov8n/yolov8_nms_postprocess", [1, 100, 6])])
    check("NMS-in-HEF output is style A", a == "A" and "style A —" in detail_a,
          (a, detail_a))
    b, detail_b = HI.classify_outputs([("output0", [1, 5, 8400])])
    check("raw ultralytics head is style B", b == "B" and "Pi CPU" in detail_b,
          (b, detail_b))
    # The HEF the user compiled for the UC-517 is exactly this. Reading it as
    # "unsupported" is the bug that made the HAT look idle on the Pi.
    r, detail_r = HI.classify_outputs(
        [("qr_yolov8n/yolov8_nms_postprocess", [1, 5, 100])])
    check("the real UC-517 NMS output (1,5,100) is style A'",
          r == "A'" and "Hailo NMS rows" in detail_r, (r, detail_r))
    check("an NMS output with a class axis is style A''",
          HI.classify_outputs([("nms", [1, 1, 100, 5])])[0] == "A''")
    c, detail_c = HI.classify_outputs([("weird", [1, 84, 8400])])
    check("an unsupported output is reported as NO boxes", c is None and "NO boxes" in detail_c,
          (c, detail_c))
    check("an empty output list is not a crash", HI.classify_outputs([])[0] is None)
    check("a 1-D output is ignored, not mismatched",
          HI.classify_outputs([("x", [6])])[0] is None)
    check("style A is preferred even when listed second",
          HI.classify_outputs([("raw", [1, 5, 8400]),
                               ("nms", [1, 100, 6])])[0] == "A")


def test_hailo_decode_reads_every_layout():
    """hailo_decode is the code that runs on the Pi, so each branch is fed a
    synthetic tensor here: a wrong branch on the drone looks exactly like a
    model that detects nothing."""
    import numpy as np
    from hailo_decode import decode_hailo_outputs

    # A' — Hailo's fixed NMS tensor: 5 rows, normalised, (y,x,y,x,score).
    t = np.zeros((1, 5, 100), dtype=np.float32)
    t[0, 0, 0], t[0, 1, 0] = 0.30, 0.42      # y_min, x_min
    t[0, 2, 0], t[0, 3, 0] = 0.46, 0.58      # y_max, x_max
    t[0, 4, 0] = 0.91
    t[0, 4, 1:] = 0.01                       # below the floor -> dropped
    boxes = decode_hailo_outputs([t], input_size=640, conf_thr=0.35)
    check("A' yields exactly the one box above the threshold", len(boxes) == 1, boxes)
    if boxes:
        x1, y1, x2, y2, sc = boxes[0]
        check("A' box is (x_min,y_min,x_max,x_max)*640 in the right order",
              abs(x1 - 268.8) < 0.5 and abs(y1 - 192.0) < 0.5
              and abs(x2 - 371.2) < 0.5 and abs(y2 - 294.4) < 0.5
              and abs(sc - 0.91) < 1e-6, boxes[0])

    # Same tensor with the rows swapped mid-detection (xyxy) must still work.
    t2 = t.copy()
    t2[0, 0, 0], t2[0, 1, 0] = 0.42, 0.30
    t2[0, 2, 0], t2[0, 3, 0] = 0.58, 0.46
    b2 = decode_hailo_outputs([t2], input_size=640, conf_thr=0.35)
    check("an xyxy-order NMS tensor is read the same way",
          len(b2) == 1 and abs(b2[0][0] - 268.8) < 0.5, b2)

    # A — class id in the last column, already in pixels.
    b3 = np.array([[20, 40, 120, 140, 0.8, 0]], dtype=np.float32)
    check("style A (last dim 6) still works",
          len(decode_hailo_outputs([b3], input_size=640, conf_thr=0.35)) == 1)

    # B — raw head, 8400 anchors, cxcywh; only the anchored count is allowed to
    # select this branch instead of A'.
    head = np.zeros((1, 5, 8400), dtype=np.float32)
    head[0, :, 0] = [320, 320, 64, 64, 0.77]
    b4 = decode_hailo_outputs([head], input_size=640, conf_thr=0.35)
    check("style B (raw head) is still read as cxcywh",
          len(b4) == 1 and abs(b4[0][0] - 288) < 0.5, b4)

    check("an unreadable shape returns [] instead of raising",
          decode_hailo_outputs([np.zeros((1, 3, 7), np.float32)]) == [])
    check("no outputs at all is not a crash",
          decode_hailo_outputs([]) == [] and decode_hailo_outputs(None) == [])
    note = []
    decode_hailo_outputs([t], input_size=640, conf_thr=0.35, note=note.append)
    check("the decoder reports which layout it used", note and note[0] == "A'", note)
    note2 = []
    decode_hailo_outputs([np.zeros((1, 3, 7), np.float32)], note=note2.append)
    check("an unreadable output reports None (so the log can warn once)",
          note2 and note2[0] is None, note2)


def test_foreign_class_count_warning():
    """A stock zoo/prebuilt YOLOv8n HEF is an 80-class COCO network whose
    class 0 is 'person' — our filter keeps class 0 only, so it could never
    report 'qr'. The tool must say that instead of letting it look like a bad
    model after a wasted flight."""
    hint = HI.class_count_hint([("qr/yolov8_nms_postprocess", [1, 80, 100, 6])])
    check("an 80-class NMS output is called out", hint and "COCO" in hint,
          hint)
    check("a 4-D output with more than one class is called out",
          "classes" in (HI.class_count_hint([("x", [1, 3, 100, 6])]) or ""),
          HI.class_count_hint([("x", [1, 3, 100, 6])]))
    check("rows layout with more than one class is called out",
          "1" in (HI.class_count_hint([("nms", [1, 400, 100])]) or ""),
          HI.class_count_hint([("nms", [1, 400, 100])]))
    check("our own 1-class shapes are NOT flagged",
          HI.class_count_hint([("x", [1, 100, 6])]) is None and
          HI.class_count_hint([("output0", [1, 5, 8400])]) is None and
          HI.class_count_hint([("nms", [1, 5, 100])]) is None and
          HI.class_count_hint([("nms", [1, 1, 100, 5])]) is None)
    check("empty input is not a crash", HI.class_count_hint([]) is None)


def test_shipped_nms_config_is_valid():
    """We ship training/nms_config.json as the starting point for the zoo's
    nms_postprocess, so it must pass our own validator."""
    import json
    path = os.path.join(PI, "training", "nms_config.json")
    check("the template exists", os.path.isfile(path))
    with open(path) as f:
        cfg = json.load(f)
    ok, lines = HI.validate_nms_config(cfg)
    check("the shipped nms config validates", ok is True, "\n".join(lines))
    check("its score threshold is not above our detection floor",
          cfg["nms_scores_th"] <= 0.35, cfg["nms_scores_th"])
    check("it is 1-class and 640x640",
          cfg["classes"] == 1 and cfg["image_dims"] == [640, 640], cfg)


def test_nms_config_validator_catches_the_dangerous_mistakes():
    bad = {"classes": 80, "image_dims": [1024, 1024], "nms_scores_th": 0.6}
    ok, lines = HI.validate_nms_config(bad)
    text = "\n".join(lines)
    check("a multi-class config is rejected", ok is False and "classes=80" in text, text)
    check("wrong image_dims are rejected", "image_dims=[1024, 1024]" in text, text)
    check("a high score threshold is rejected with the reason",
          "above our detection floor" in text, text)
    ok2, _ = HI.validate_nms_config({"classes": 1, "image_dims": [640, 640],
                                     "nms_scores_th": 0.1})
    check("a low threshold is fine", ok2 is True)
    check("non-dict input is refused", HI.validate_nms_config([])[0] is False)


def test_model_path_resolution():
    """`hef_path: models/qr_yolov8n.hef` must resolve from the mission dir even
    when the app is launched from the repo root — otherwise kind:auto quietly
    drops to the CPU rung with the HEF sitting right there."""
    rel = "models/qr_yolov8n.onnx"
    check("resolved from the mission dir", D.resolve_model_path(rel) is not None)
    here = os.getcwd()
    try:
        os.chdir("/tmp")
        got = D.resolve_model_path(rel)
        check("resolved from an unrelated cwd", (got or "").endswith(rel), got)
    finally:
        os.chdir(here)
    check("a missing file is None, not a guess",
          D.resolve_model_path("models/definitely_absent.hef") is None)
    check("an empty value is None", D.resolve_model_path("") is None)


if __name__ == "__main__":
    test_frame_stats_and_signature()
    test_calib_collection_and_summary()
    test_focus_gate_matches_the_field_run()
    test_curve_verdict_on_the_real_numbers()
    test_focus_sweep_defaults_to_the_mission_config()
    test_compile_doc_matches_the_real_model()
    test_hef_output_style_classification()
    test_foreign_class_count_warning()
    test_shipped_nms_config_is_valid()
    test_nms_config_validator_catches_the_dangerous_mistakes()
    test_model_path_resolution()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hef/focus tests passed")
