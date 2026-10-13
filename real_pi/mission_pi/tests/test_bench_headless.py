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
    ok_probe = lambda cv, env: (True, "fake display works")          # noqa: E731
    bad_probe = lambda cv, env: (False, "fake: killed by signal 6")  # noqa: E731

    show, why = B.decide_show(False, False, none_env, probe=ok_probe)
    check("no display -> headless (Qt would abort the process)",
          show is False and "DISPLAY" in why, (show, why))
    show2, _ = B.decide_show(False, False, desktop, probe=ok_probe)
    check("a normal desktop still opens the window", show2 is True)
    show3, _ = B.decide_show(False, False, wayland, probe=ok_probe)
    check("Wayland counts as a display", show3 is True)
    show4, why4 = B.decide_show(True, False, desktop, probe=bad_probe)
    check("--headless wins over a display", show4 is False)
    show5, why5 = B.decide_show(False, True, none_env, probe=bad_probe)
    check("--show forces a window (e.g. X-forwarded)",
          show5 is True and "forced" in why5, (show5, why5))
    show6, _ = B.decide_show(True, True, desktop, probe=ok_probe)
    check("--headless wins when both are given (safe default)",
          show6 is False)
    # DISPLAY set but unusable is the real-world case that started this: an
    # `ssh -X`/`.bashrc` DISPLAY with no X cookie aborts the process, so a
    # display that cannot open a window must count as no display.
    show7, why7 = B.decide_show(False, False, desktop, probe=bad_probe)
    check("DISPLAY set but the probe fails -> headless, reason kept",
          show7 is False and "signal" in why7, (show7, why7))


class _HeadlessCV2:
    """opencv-python-headless: imshow/namedWindow simply do not exist."""


class _FakeCV2:
    """A cv2 whose window calls fail — the probe must survive that."""
    called = False

    @staticmethod
    def imshow(name, img):
        pass

    @staticmethod
    def namedWindow(name):
        _FakeCV2.called = True
        raise RuntimeError("boom")

    @staticmethod
    def destroyAllWindows():
        pass


class _WorkingCV2:
    """A cv2 whose window calls succeed (so the child exits 0)."""

    @staticmethod
    def imshow(name, img):
        pass

    @staticmethod
    def namedWindow(name):
        pass

    @staticmethod
    def destroyAllWindows():
        pass


def test_preset_keys_are_ints():
    """The field crash: pressing 1-7 raised
    `TypeError: ord() expected string of length 1, but int found`, because
    cv2.waitKey returns an INT and the digit branch called ord() on it."""
    check("1 selects the first preset", B.preset_index(ord("1"), 7) == 0)
    check("7 selects the seventh", B.preset_index(ord("7"), 7) == 6)
    check("a digit beyond the defined presets is refused, not a crash",
          B.preset_index(ord("7"), 3) is None)
    check("letters and out-of-range ints are refused",
          B.preset_index(ord("f"), 7) is None and
          B.preset_index(-1, 7) is None and
          B.preset_index(255, 7) is None)
    check("no ord() on an int anywhere in the key handling",
          "ord(k)" not in open(os.path.join(PI, "tools",
                                            "test_yolo_webcam_qr.py")).read())


def test_display_probe():
    """probe_display must never raise, must not need a real display, and must
    answer 'no' for every one of the ways a window can be impossible."""
    if not hasattr(os, "fork"):
        check("display probe skipped (no fork on this platform)", True)
        return

    ok, why = B.probe_display(cv_mod=_WorkingCV2(), env={})
    check("no DISPLAY -> False without touching the window API",
          ok is False and "headless" in why, (ok, why))
    ok3, why3 = B.probe_display(cv_mod=_HeadlessCV2(), env={"DISPLAY": ":0"})
    check("headless-only cv2 build (no imshow attr) -> False, blames cv2",
          ok3 is False and "GUI" in why3, (ok3, why3))
    ok4, why4 = B.probe_display(cv_mod=_FakeCV2(), env={"DISPLAY": ":0"})
    check("a window that cannot be created -> False (parent survives)",
          ok4 is False and "headless" in why4, (ok4, why4))
    check("the failing window call happened in a CHILD process",
          _FakeCV2.called is False)
    ok5, why5 = B.probe_display(cv_mod=_WorkingCV2(), env={"DISPLAY": ":0"})
    check("a working display probe -> True", ok5 is True, (ok5, why5))


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
    check("a driven lens with no known tuning stays unknown",
          c.status().get("focus_ok") is None)
    c.af_algorithm = True
    check("a driven lens + an AF algorithm -> focus_ok True",
          c.status().get("focus_ok") is True)
    u = C.USBCamera("cam1", "/dev/video0", (640, 480), 66.0, "front")
    check("the CSI override does not leak into other camera kinds",
          "focus_controls" not in u.status(), sorted(u.status()))


def test_af_algorithm_decides_autofocus_not_the_control_list():
    """The field trap of 2026-10-11: `camera_controls` listed AfMode,
    LensPosition + 6 more, and autofocus still could not work, because the
    stock imx477 tuning has NO rpi.af algorithm (verified against
    raspberrypi/libcamera: imx477.json has no rpi.af, imx708.json does).
    Reading the control list alone claims success; this must not."""
    import json
    import tempfile

    # the tuning-file shape: a LIST of single-key dicts (collapsing it to the
    # first entry makes every sensor look like rpi.black_level only)
    stock = {"algorithms": [{"rpi.black_level": {}}, {"rpi.agc": {}},
                            {"rpi.sharpen": {}}]}
    with_af = {"algorithms": [{"rpi.black_level": {}}, {"rpi.agc": {}},
                              {"rpi.af": {"lens": {"i2c": "0x0c"}}}]}
    check("all algorithms are listed, not just the first",
          C.tuning_algorithms(stock) == ["rpi.black_level", "rpi.agc",
                                        "rpi.sharpen"],
          C.tuning_algorithms(stock))
    check("a stock imx477-style tuning has NO autofocus algorithm",
          C.tuning_has_af(stock) is False)
    check("an Arducam-style tuning with rpi.af does",
          C.tuning_has_af(with_af) is True)
    check("a tuning with no algorithms at all is not a crash",
          C.tuning_has_af({}) is False and C.tuning_has_af(None) is False)

    with tempfile.TemporaryDirectory() as td:
        pisp = os.path.join(td, "pisp")
        os.makedirs(pisp)
        with open(os.path.join(pisp, "imx477.json"), "w") as f:
            json.dump(stock, f)
        real_dirs = C.IPA_TUNING_DIRS
        C.IPA_TUNING_DIRS = (pisp,)
        try:
            check("the default tuning for a sensor is found by name",
                  C.default_tuning_path("imx477") ==
                  os.path.join(pisp, "imx477.json"))
            check("an unknown sensor has no default tuning",
                  C.default_tuning_path("imx999") is None)

            c = C.Camera("cam2", 0, (2028, 1520), 75.0, "bottom",
                         autofocus=True, tuning_file="imx477_af.json")
            # exactly what the Pi reported, and exactly what it does NOT prove
            c.focus_controls = ["AfSpeed", "AfMode", "AfRange", "LensPosition",
                                "AfTrigger", "AfWindows", "AfMetering",
                                "AfPause"]
            c.model = "imx477"   # what the rig / camera_properties reports
            c._resolve_af_algorithm(None)       # no json installed -> default
            check("the stock default tuning is inspected, not assumed",
                  c.af_algorithm is False, c.af_algorithm)
            check("autofocus is reported IMPOSSIBLE while the control list "
                  "says otherwise", c.focus_possible() is False)
            st = c.status()
            check("status.focus_ok is False in the field state "
                  "(af_mode:auto would have hidden this)",
                  st.get("focus_ok") is False and st.get("af_algorithm") is False,
                  {k: st.get(k) for k in ("focus_ok", "af_algorithm",
                                          "focus_controls")})

            c._resolve_af_algorithm(with_af)    # Arducam's json dropped in
            check("with an AF tuning loaded, autofocus becomes possible",
                  c.focus_possible() is True and c.af_algorithm is True)

            # a sensor whose IPA cannot be read stays UNKNOWN, not a claim
            c2 = C.Camera("cam2", 0, (2028, 1520), 75.0, "bottom",
                          autofocus=True)
            c2.focus_controls = ["AfMode", "LensPosition"]
            c2._resolve_af_algorithm(None)
            c2.model = "imx999"
            c2._resolve_af_algorithm(None)
            check("an unreadable tuning reports unknown, never a false OK",
                  c2.focus_possible() is None, c2.status().get("focus_ok"))
        finally:
            C.IPA_TUNING_DIRS = real_dirs


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


def test_live_focus_keys_step_and_clamp():
    """[ ] step 0.05 dpt, - = step 0.25, and every step lands INSIDE the
    motor's range (0..12 dpt from Arducam's rpi.af tuning)."""
    check("fine step down", B.FOCUS_KEYS["["] == -0.05, repr(B.FOCUS_KEYS))
    check("fine step up", B.FOCUS_KEYS["]"] == 0.05, repr(B.FOCUS_KEYS))
    check("coarse step down", B.FOCUS_KEYS["-"] == -0.25, repr(B.FOCUS_KEYS))
    check("coarse step up", B.FOCUS_KEYS["="] == 0.25, repr(B.FOCUS_KEYS))
    check("a step lands", B.nudge_focus(1.0, 0.05) == 1.05,
          repr(B.nudge_focus(1.0, 0.05)))
    check("clamped at the far end", B.nudge_focus(0.02, -0.05) == 0.0,
          repr(B.nudge_focus(0.02, -0.05)))
    check("clamped at the near end", B.nudge_focus(11.98, 0.25) == 12.0,
          repr(B.nudge_focus(11.98, 0.25)))
    check("an unknown start begins at infinity focus",
          B.nudge_focus(None, 0.05) == 0.05, repr(B.nudge_focus(None, 0.05)))
    check("the HUD label carries distance",
          B.focus_text(0.067) == "lens 0.07 dpt (14.93 m)",
          B.focus_text(0.067))
    check("infinity is named, not divided",
          B.focus_text(0.0) == "lens 0.00 dpt (inf)", B.focus_text(0.0))
    check("unknown reads as a dash", B.focus_text(None) == "lens -",
          B.focus_text(None))


def test_preset_boost_reaches_the_camera():
    """Pressing 1-7 in the bench changed the detector's conf/tiling but NOT the
    camera, so the window kept the same exposure and look whatever preset was
    selected — the 2026-10-12 report ("presets change but the window stays in
    that preset's look"). The mission applies qr_presets.<name>.boost through
    Camera.apply_qr_profile(); the bench must use the SAME call."""

    class FakeCam:
        def __init__(self, raises=False):
            self.calls = []
            self.raises = raises

        def apply_qr_profile(self, name):
            self.calls.append(name)
            if self.raises:
                raise RuntimeError("profile not found")
            return {"exposure_us": 5000, "contrast": 1.8, "sharpness": 2.0}

    class FakeCap:
        def __init__(self, cam=None):
            if cam is not None:
                self.cam = cam

    cam = FakeCam()
    line = B.apply_preset_boost(FakeCap(cam),
                                {"boost": "qr_boost_day", "conf_thr": 0.35})
    check("a preset's boost profile is applied to the camera",
          cam.calls == ["qr_boost_day"], cam.calls)
    check("and the line says which profile and what it moved",
          line and "qr_boost_day" in line and "exposure_us=5000" in line
          and "contrast=1.8" in line, line)
    check("a USB/webcam source has no boost to apply",
          B.apply_preset_boost(FakeCap(), {"boost": "qr_boost_day"}) is None)
    check("a preset without a boost key is not an error",
          B.apply_preset_boost(FakeCap(FakeCam()), {"conf_thr": 0.3}) is None
          and B.apply_preset_boost(FakeCap(FakeCam()), None) is None)
    bad = B.apply_preset_boost(FakeCap(FakeCam(raises=True)),
                               {"boost": "nope"})
    check("a failing boost is reported, never raised",
          bad and "FAILED" in bad and "nope" in bad, bad)

    # The profiles the presets name must actually exist, or the ISP silently
    # keeps its old settings and only the HUD changes.
    import qr_camera_boost as QB
    cfg = {}
    try:
        import yaml
        cfg = yaml.safe_load(open(os.path.join(PI, "config.yaml"))) or {}
    except Exception as e:
        check("config.yaml readable for the preset check", False, e)
    presets = cfg.get("qr_presets") or {}
    missing = [n for n, p in presets.items()
               if (p or {}).get("boost")
               and p["boost"] not in QB.QR_BOOST_PROFILES]
    check("every preset's boost profile exists",
          bool(presets) and not missing, missing)
    check("the default preset (day) does apply a real profile",
          (presets.get("day") or {}).get("boost") in QB.QR_BOOST_PROFILES,
          presets.get("day"))


def test_focus_score_and_vision_hint():
    """The 2026-10-12 report was "camera looks blurry not sharp at all idk
    why" — with nothing in the output saying whether the fault was the light,
    the surface or the lens. Every stats line now carries `focus=`, so the
    reason has to be the RIGHT one: a black frame must not be called blurry,
    and a blurred-but-bright frame must be."""
    import numpy as np
    import cv2
    sharp = np.zeros((160, 220, 3), np.uint8)
    sharp[::4, :] = 255                       # checkerboard-ish: high Laplacian
    sharp[:, ::4] = 255
    soft = cv2.GaussianBlur(sharp, (15, 15), 6.0)
    s_sharp, s_soft = B.focus_score(sharp), B.focus_score(soft)
    check("a sharp frame scores above a blurred copy",
          s_sharp > s_soft > 0, (s_sharp, s_soft))
    import focus_sweep as FS
    check("the score is the sweep's own metric (they cannot drift)",
          abs(s_sharp - FS.sharpness(sharp, cv2, centre=0.5)) < 1e-6,
          (s_sharp, FS.sharpness(sharp, cv2, centre=0.5)))

    dark = B.vision_hint(4.0, 1.0, 3.0)
    check("a black frame is reported as black, not blurry",
          dark and "BLACK" in dark and "SOFT" not in dark, dark)
    flat = B.vision_hint(120.0, 4.0, 3.0)
    check("no contrast is reported before sharpness",
          flat and "CONTRAST" in flat and "SOFT" not in flat, flat)
    softmsg = B.vision_hint(120.0, 30.0, 4.0)
    check("contrast + no sharpness = the lens, said plainly",
          softmsg and "SOFT" in softmsg and "MECHANICAL" in softmsg
          and "focus=" in softmsg, softmsg)
    check("a healthy frame produces no hint at all",
          B.vision_hint(120.0, 40.0, 400.0) is None)
    check("unmeasurable stats are not guessed at",
          B.vision_hint(-1.0, -1.0, -1.0) is None)


def test_bench_stamps_which_code_it_is():
    """A log that cannot say which commit produced it is how a fixed bug gets
    debugged twice: the Pi ran the other branch's build and the pasted output
    still showed ord(k) and status 69."""
    import codeinfo as CI
    v = CI.code_version()
    check("the bench's checkout is identified by a commit hash",
          len(v.get("hash") or "") >= 7, v)
    line = "%s" % CI.describe(v)
    check("and it is printed as one readable line",
          v["hash"][:7] in line and "(" in line, line)
    check("a missing/broken git is reported, never raised",
          "unknown" in CI.describe({"hash": "", "error": "OSError: no git"}) and
          "no git" in CI.describe({}))
    check("uncommitted work is visible in the stamp",
          "uncommitted" in CI.describe({"hash": "abc1234", "branch": "b",
                                        "dirty": True}),
          CI.describe({"hash": "abc1234", "branch": "b", "dirty": True}))
    out = subprocess.run([sys.executable, os.path.join(PI, "codeinfo.py")],
                         capture_output=True, text=True).stdout
    del out          # codeinfo has no CLI; the import above is the test


if __name__ == "__main__":
    test_headless_decision()
    test_preset_keys_are_ints()
    test_live_focus_keys_step_and_clamp()
    test_preset_boost_reaches_the_camera()
    test_focus_score_and_vision_hint()
    test_bench_stamps_which_code_it_is()
    test_detector_kind_aliases()
    test_qr_bench_accepts_the_documented_name()
    test_single_camera_is_not_a_failure()
    test_no_camera_keeps_the_physical_hint()
    test_two_cameras_fill_both_roles()
    test_status_reports_real_focus_capability()
    test_af_algorithm_decides_autofocus_not_the_control_list()
    test_tuning_lookup_dedupes_and_looks_in_models()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all bench-headless tests passed")
