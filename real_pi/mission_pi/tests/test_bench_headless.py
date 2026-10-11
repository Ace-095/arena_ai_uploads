#!/usr/bin/env python3
"""Regression tests for the field failures of 2026-10-11 (Pi, over SSH):

1. `test_yolo_webcam_qr.py --source csi:bottom --onnx ...` died with
   `qt.qpa.xcb: could not connect to display` + SIGABRT. Qt aborts the whole
   process, so this can never be a try/except — the window decision must
   happen BEFORE the first imshow(). Auto-headless when there is no display.
2. `qr_bench.py --detector onnx` raised
   `unknown detector.kind: 'onnx'` — the bench names an operator types
   (`onnx`, `pt`) must be valid, and the error must list the real ones.
3. `list_cams.py` told the operator to "reseat the FPC cable" while one
   camera was detected, assigned and streaming, and exited 1 as if the rig
   were broken. A single-camera rig is a legitimate configuration.

No camera, no display, no network needed (the optional end-to-end bench test
skips itself when the model/onnxruntime are absent).
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import cameras as C  # noqa: E402
import detector as D  # noqa: E402
import list_cams as L  # noqa: E402
import test_yolo_webcam_qr as B  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


class FakeCam:
    def __init__(self, facing, model="imx477", idx=0, size=(2028, 1520)):
        self.facing, self.model = facing, model
        self.index, self.size = idx, size
        self.device_desc = "csi %s (idx %d)" % (model, idx)


# ------------------------------------------------- 1. window vs no display --
def test_headless_decision():
    none_env = {}
    desktop = {"DISPLAY": ":0"}
    wayland = {"WAYLAND_DISPLAY": "wayland-0"}

    show, why = B.decide_show(False, False, none_env)
    check("no display -> headless (Qt would abort the process)",
          show is False and "DISPLAY" in why, (show, why))
    show2, _ = B.decide_show(False, False, desktop)
    check("a normal desktop still opens the window", show2 is True)
    show3, _ = B.decide_show(False, False, wayland)
    check("Wayland counts as a display", show3 is True)
    show4, why4 = B.decide_show(True, False, desktop)
    check("--headless wins over a display", show4 is False)
    show5, why5 = B.decide_show(False, True, none_env)
    check("--show forces a window (e.g. X-forwarded)",
          show5 is True and "forced" in why5, (show5, why5))
    show6, _ = B.decide_show(True, True, desktop)
    check("--headless wins when both are given (safe default)",
          show6 is False)


# ------------------------------------------------------- 2. detector names --
def test_detector_kind_aliases():
    for k in ("onnx", "yolo_onnx", "onnxruntime", "pt", "pytorch", "torch",
              "yolo", "yolov8", "yolo_ultralytics"):
        check("%r is a CPU-YOLO kind" % k, k in D.YOLO_KINDS)
    check("the alias table pins the format",
          D.YOLO_FORMAT_ALIASES.get("onnx") == "onnx" and
          D.YOLO_FORMAT_ALIASES.get("pt") == "pt")
    try:
        D.get_detector({"kind": "nonsense"})
    except RuntimeError as e:
        msg = str(e)
        check("an unknown kind names the valid ones",
              "onnx" in msg and "hailo" in msg and "classical" in msg, msg)
    else:
        check("an unknown kind is rejected", False, "no RuntimeError")

    # kind=onnx must actually reach the ONNX rung when the model is present
    model = os.path.join(PI, "models", "qr_yolov8n.onnx")
    if os.path.isfile(model):
        try:
            det = D.get_detector({"kind": "onnx", "conf_thr": 0.35})
        except Exception as e:
            check("kind=onnx loads the ONNX model", False, repr(e))
        else:
            check("kind=onnx loads the ONNX model",
                  "onnx" in str(getattr(det, "name", "")).lower(),
                  getattr(det, "name", type(det).__name__))
    else:
        print("SKIP kind=onnx load (no models/qr_yolov8n.onnx)")


def test_qr_bench_accepts_the_documented_name():
    """`--detector onnx` is what the docs/tests tell an operator to type."""
    out = subprocess.run(
        [sys.executable, os.path.join(HERE, os.pardir, "tools", "qr_bench.py"),
         "--detector", "onnx", "/nonexistent/none.jpg"],
        capture_output=True, text=True, timeout=180)
    text = (out.stdout or "") + (out.stderr or "")
    check("--detector onnx parses (no 'invalid choice')",
          "invalid choice" not in text, text[-400:])
    check("and it fails only because the image is missing",
          "readable" in text or "no images" in text.lower(), text[-400:])
    bad = subprocess.run(
        [sys.executable, os.path.join(HERE, os.pardir, "tools", "qr_bench.py"),
         "--detector", "yolo_onnx_typo", "/nonexistent/none.jpg"],
        capture_output=True, text=True, timeout=180)
    btext = (bad.stdout or "") + (bad.stderr or "")
    check("a genuine typo is still refused, listing the valid names",
          "invalid choice" in btext and "onnx" in btext, btext[-400:])


# --------------------------------------------- 3. assignment report wording --
def test_single_camera_is_not_a_failure():
    cfg = {"cameras": {"single_cam_role": "bottom"}}
    lines, code = L.assignment_report({"cam2": FakeCam("bottom")}, cfg, True)
    text = "\n".join(lines)
    check("one working camera exits 0 (valid single-camera rig)", code == 0,
          code)
    check("it never tells you to reseat the cable of a detected camera",
          "reseat" not in text, text)
    check("the missing second role is reported as informational",
          "MISSING ROLE(S): front" in text and "not a failure" in text, text)
    check("and it is attributed to single_cam_role",
          "single_cam_role" in text, text)


def test_no_camera_keeps_the_physical_hint():
    cfg = {"cameras": {"single_cam_role": "bottom"}}
    lines, code = L.assignment_report({}, cfg, True)
    text = "\n".join(lines)
    check("no camera -> exit 1", code == 1, code)
    check("with boot config OK the hint is physical",
          "reseat the FPC cable" in text, text)
    lines2, _ = L.assignment_report({}, cfg, False)
    check("if the boot config is NOT ok, no cable advice (fix the config first)",
          "reseat" not in "\n".join(lines2), "\n".join(lines2))


def test_two_cameras_fill_both_roles():
    cfg = {"cameras": {}}
    lines, code = L.assignment_report(
        {"cam1": FakeCam("front", idx=1), "cam2": FakeCam("bottom", idx=0)},
        cfg, True)
    text = "\n".join(lines)
    check("two cameras -> exit 0 and no missing roles",
          code == 0 and "MISSING ROLE" not in text, text)


# ------------------------------------------- focus truth in camera status --
def test_status_reports_real_focus_capability():
    c = C.Camera("cam2", 0, (2028, 1520), 75.0, "bottom", autofocus=True,
                 tuning_file="imx477_af.json")
    st = c.status()
    check("status exposes autofocus + focus_controls",
          st.get("autofocus") is True and "focus_controls" in st, sorted(st))
    check("before an open the capability is unknown, not a lie",
          st.get("focus_ok") is None, st.get("focus_ok"))
    c.focus_controls = []
    st2 = c.status()
    check("no lens subdev -> focus_ok is False (config does nothing)",
          st2.get("focus_ok") is False and st2.get("focus_controls") == [],
          st2.get("focus_controls"))
    c.focus_controls = ["AfMode", "LensPosition"]
    check("a driven lens -> focus_ok True",
          c.status().get("focus_ok") is True)
    u = C.USBCamera("cam1", "/dev/video0", (640, 480), 66.0, "front")
    check("the CSI override does not leak into other camera kinds",
          "focus_controls" not in u.status(), sorted(u.status()))


def test_tuning_lookup_dedupes_and_looks_in_models():
    """The field log showed the SAME path twice in the warning (cwd == the
    mission dir on the Pi), which reads like a broken lookup, and models/ —
    the no-sudo place to drop a tuning file — was not searched at all."""
    import json
    import logging
    import tempfile

    class Capture(logging.Handler):
        def __init__(self):
            super().__init__()
            self.msgs = []

        def emit(self, record):
            self.msgs.append(record.getMessage())

    with tempfile.TemporaryDirectory() as td:
        cwd = os.getcwd()
        h = Capture()
        C.log.addHandler(h)
        old_level = C.log.level
        C.log.setLevel(logging.WARNING)
        try:
            os.chdir(td)
            c = C.Camera("cam2", 0, (2028, 1520), 75.0, "bottom",
                         tuning_file="no_such_tuning_xyz.json")
            c._load_tuning()
            # and a real file found by name in the working dir
            p = os.path.join(td, "mine.json")
            with open(p, "w") as f:
                json.dump({"version": 7}, f)
            t = C.Camera("cam2", 0, (2028, 1520), 75.0, "bottom",
                         tuning_file="mine.json")._load_tuning()
        finally:
            os.chdir(cwd)
            C.log.removeHandler(h)
            C.log.setLevel(old_level)

    text = "\n".join(h.msgs)
    check("a bare name is resolved to a file we can read",
          isinstance(t, dict) and t.get("version") == 7, repr(t))
    check("the warning names models/ as a searched location",
          os.path.join(PI, "models") in text, text)
    mission_path = os.path.join(PI, "no_such_tuning_xyz.json")
    check("no candidate is listed twice",
          text.count(mission_path) == 1, text)
    check("the model hint stays a warning, never an exception",
          "not found" in text, text)


if __name__ == "__main__":
    test_headless_decision()
    test_detector_kind_aliases()
    test_qr_bench_accepts_the_documented_name()
    test_single_camera_is_not_a_failure()
    test_no_camera_keeps_the_physical_hint()
    test_two_cameras_fill_both_roles()
    test_status_reports_real_focus_capability()
    test_tuning_lookup_dedupes_and_looks_in_models()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all bench-headless tests passed")
