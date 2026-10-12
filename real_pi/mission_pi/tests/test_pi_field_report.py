"""Tests for tools/pi_field_report.py - the field survey the Pi runs itself.

This tool exists because the machine writing the code cannot reach a private
LAN address, so whatever it reports is taken at face value by someone standing
at the aircraft. That makes two things non-negotiable:

1. **A number it labels MEASURED must be right.** The FOV step is the one that
   matters: every range calculation in this repo is conditional on a
   VENDOR-QUOTED 75 deg, and this is the tool that would replace it. It is
   tested here by recovering a KNOWN field of view from a synthetic picture of
   a sheet - the first version of it was wrong twice (a flat frame measured
   "640 px" wide, and a landscape sheet was called vertical, skewing the answer
   -15 deg), so the accuracy is asserted rather than assumed.
2. **It must not actuate anything.** It runs on a flight vehicle. A structural
   test below fails if the source ever grows a call that writes config, sets a
   camera control, or sends MAVLink beyond the read-only probes.

Everything else here is graceful degradation: on a machine with no camera, no
Hailo and no FC (which is exactly where these tests DO run) every section must
return an explanation instead of raising.
"""
import importlib.util
import inspect
import signal
import json
import math
import os
import subprocess
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
_TOOL = os.path.join(_PI, "tools", "pi_field_report.py")
if _PI not in sys.path:
    sys.path.insert(0, _PI)

try:
    import numpy as np
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False
needs_cv2 = unittest.skipUnless(HAVE_CV2, "numpy + cv2 needed")


def _load():
    spec = importlib.util.spec_from_file_location("pi_field_report", _TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


F = _load()


def make_args(**kw):
    """A complete argument namespace for build()/sections."""
    class A(object):
        pass
    a = A()
    defaults = dict(config=os.path.join(_PI, "config.yaml"), out_dir="/tmp",
                    role="bottom", samples=2, infer_frames=2, dist_m=0.0,
                    panel_mm=0.0, panel_mm_h=0.0, qr_modules=0, no_capture=True,
                    no_decode=False, no_infer=False, with_fc=False,
                    fc_device=None, with_tests=False, sec="all", json=None)
    defaults.update(kw)
    for k, v in defaults.items():
        setattr(a, k, v)
    return a


def frame_with_sheet(fw, fh, sw, sh, ground=60, sheet=240, noise=4, seed=3):
    """A synthetic picture of a bright sheet on textured, darker ground."""
    rng = np.random.default_rng(seed)
    img = np.clip(ground + rng.normal(0, noise, (fh, fw)), 0, 255).astype(np.uint8)
    x0, y0 = (fw - sw) // 2, (fh - sh) // 2
    img[y0:y0 + sh, x0:x0 + sw] = sheet
    return img


class TestSheetDetection(unittest.TestCase):
    @needs_cv2
    def test_finds_the_sheet_and_its_size(self):
        for fw, fh, sw, sh in ((2028, 1520, 300, 212), (2028, 1520, 600, 424),
                               (640, 480, 100, 70)):
            with self.subTest(frame=(fw, fh), sheet=(sw, sh)):
                r = F._sheet_rect(frame_with_sheet(fw, fh, sw, sh))
                self.assertIsNotNone(r, "sheet not found in a clean synthetic frame")
                long_px, short_px, horiz = r
                self.assertAlmostEqual(long_px, sw, delta=3.0)
                self.assertAlmostEqual(short_px, sh, delta=3.0)
                self.assertTrue(horiz)

    @needs_cv2
    def test_orientation_comes_from_the_image_axis_not_the_rotated_rect(self):
        """The regression: minAreaRect's sides live in the rect's OWN frame, so
        treating `rw >= rh` as an image-axis fact reported a landscape sheet as
        vertical and skewed the recovered FOV by -15 deg."""
        r = F._sheet_rect(frame_with_sheet(2028, 1520, 212, 300))
        self.assertIsNotNone(r)
        self.assertFalse(r[2], "a portrait sheet must report the long side vertical")

    @needs_cv2
    def test_no_sheet_means_no_measurement(self):
        """A wrong FOV is worse than no FOV: these must all be refused."""
        cases = {
            "flat grey": np.full((480, 640), 128, np.uint8),
            "flat white": np.full((480, 640), 250, np.uint8),
            "black": np.zeros((480, 640), np.uint8),
            "textured ground only": frame_with_sheet(640, 480, 0, 0, ground=70, noise=25),
        }
        for name, img in cases.items():
            with self.subTest(case=name):
                self.assertIsNone(F._sheet_rect(img),
                                  "%s was accepted as a sheet" % name)

    @needs_cv2
    def test_a_bright_blob_at_the_edge_is_refused(self):
        """Distortion is worst off-centre, so this measures the CENTRE FOV."""
        img = np.full((480, 640), 50, np.uint8)
        img[10:110, 10:110] = 250
        self.assertIsNone(F._sheet_rect(img))

    @needs_cv2
    def test_a_sheet_clipped_by_the_frame_is_refused(self):
        img = np.full((480, 640), 50, np.uint8)
        img[0:200, 0:200] = 250
        self.assertIsNone(F._sheet_rect(img))

    @needs_cv2
    def test_a_sheet_filling_the_view_is_refused(self):
        """No contrast left: a FOV computed from it would be meaningless."""
        img = np.full((480, 640), 40, np.uint8)
        img[2:478, 2:638] = 250
        self.assertIsNone(F._sheet_rect(img))

    @needs_cv2
    def test_a_sheet_rotated_far_from_square_is_refused(self):
        img = np.full((1520, 2028), 60, np.uint8)
        box = cv2.boxPoints(((1014, 760), (600, 424), 40.0)).astype(int)
        cv2.fillPoly(img, [box], 240)
        self.assertIsNone(F._sheet_rect(img),
                          "a 40 deg rotated sheet cannot be attributed to an axis")


class _FakeCam(object):
    """Enough of camera.py's camera for the capture path."""

    def __init__(self, frames, size=(640, 480)):
        self._frames = list(frames)
        self.size = size
        self.hfov_deg = 75.0
        self.vfov_deg = 60.0
        self.kind = "file"
        self.index = -1
        self.model = "fake"
        self.facing = "bottom"
        self.running = False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def latest(self, *a, **k):
        import time
        time.sleep(0.001)
        if self._frames:
            return self._frames.pop(0), time.time(), 1
        return None, time.time(), 0


class TestFovAccuracy(unittest.TestCase):
    """Recover a KNOWN field of view. This is the claim the tool makes."""

    @needs_cv2
    def test_recovers_a_known_hfov(self):
        # if the true HFOV is T, a sheet of long side P at distance D projects to
        #   px = frame_span * P / (2*D*tan(T/2))
        for T, fw, fh, P, D in ((75.0, 2028, 1520, 0.420, 2.0),
                                (75.0, 2028, 1520, 0.420, 1.5),
                                (66.0, 4608, 2592, 0.420, 1.0),
                                (50.0, 2028, 1520, 0.420, 1.2)):
            with self.subTest(hfov_deg=T, dist_m=D):
                ground_w = 2 * D * math.tan(math.radians(T) / 2.0)
                sw = int(round(fw * P / ground_w))
                sh = int(round(sw * 0.707))
                r = F._sheet_rect(frame_with_sheet(fw, fh, sw, sh))
                self.assertIsNotNone(r, "sheet not found for a known geometry")
                long_px, _short, horiz = r
                span = fw if horiz else fh
                est = math.degrees(2 * math.atan(
                    ((P * span / long_px) / 2.0) / D))
                self.assertLess(abs(est - T), 1.0,
                                "recovered %.2f deg for a %.1f deg lens" % (est, T))


class TestSectionsDegradeGracefully(unittest.TestCase):
    """On a bare machine (no camera, no Hailo, no FC) nothing may raise."""

    def _args(self, **kw):
        class A(object):
            pass
        a = A()
        for k, v in dict(config=os.path.join(_PI, "config.yaml"), out_dir="/tmp",
                         role="bottom", samples=2, infer_frames=2, dist_m=0.0,
                         panel_mm=0.0, panel_mm_h=0.0, qr_modules=0,
                         no_capture=True, no_decode=False, no_infer=False,
                         with_fc=False, fc_device=None, with_tests=False,
                         sec="all", json=None).items():
            setattr(a, k, kw.get(k, v))
        return a

    def test_every_section_returns_a_dict(self):
        args = self._args()
        for name, fn in F.SECTIONS:
            with self.subTest(section=name):
                got = fn(args)
                self.assertIsInstance(got, dict,
                                      "%s did not return a report dict" % name)

    def test_the_decoder_section_never_imports_the_camera_when_it_has_frames(self):
        args = self._args(no_capture=True)
        got = F.sec_decode(args)
        self.assertFalse(got.get("attempted"))
        self.assertIn("why", got)

    def test_the_fc_section_is_off_unless_asked(self):
        got = F.sec_fc(self._args())
        self.assertFalse(got.get("connected"))
        self.assertIn("--with-fc", got.get("why", ""))

    def test_capture_of_a_fake_camera_is_timed_and_sized(self):
        frames = [frame_with_sheet(640, 480, 200, 140) for _ in range(3)]
        cam = _FakeCam(frames, size=(640, 480))
        got, ms = F._capture_into("bottom", cam, seconds=2.0, want=3)
        self.assertEqual(len(got), 3)
        self.assertEqual(len(ms), 3)
        self.assertTrue(all(isinstance(x, float) for x in ms))

    @needs_cv2
    def test_fov_uses_the_frames_already_captured(self):
        """Three sections used to open the camera; they must share instead."""
        F.FRAMES.by_role.clear()
        F.FRAMES.add("bottom", [frame_with_sheet(2028, 1520, 370, 262)])
        args = self._args(dist_m=1.5, panel_mm=420.0, panel_mm_h=297.0)
        got = F.sec_fov(args)
        self.assertTrue(got.get("measured"), got.get("error"))
        self.assertAlmostEqual(got["hfov_deg"], 75.0, delta=1.5)
        self.assertIn("aspect_check", got)
        F.FRAMES.by_role.clear()


class TestANativeCrashIsContained(unittest.TestCase):
    """The bug that killed the whole survey on the real Pi.

    HailoYolo8Detector() died with SIGBUS (exit -7) inside the detector section.
    A signal is not an exception, so try/except caught nothing, the survey aborted
    after three of ten sections, and NO report was written - the tool whose only
    job is to bring information back brought none. Anything that loads a native
    accelerator now runs in a child process.
    """

    def test_a_child_signal_is_reported_not_fatal(self):
        for sig, want in ((signal.SIGBUS, "SIGBUS"), (signal.SIGSEGV, "SIGSEGV"),
                          (signal.SIGKILL, "SIGKILL")):
            with self.subTest(signal=want):
                r = F._child("import os, signal; os.kill(os.getpid(), %d)" % sig)
                self.assertEqual(r["signal"], want)
                self.assertLess(r["rc"], 0)

    def test_output_before_a_crash_is_kept(self):
        r = F._child('import os, signal\n'
                     'print("i got this far", flush=True)\n'
                     'os.kill(os.getpid(), signal.SIGBUS)')
        self.assertIn("i got this far", r["stdout"])
        self.assertEqual(r["signal"], "SIGBUS")

    def test_a_child_that_exits_normally_has_no_signal(self):
        r = F._child('print("fine")')
        self.assertIsNone(r["signal"])
        self.assertEqual(r["rc"], 0)
        self.assertIn("fine", r["stdout"])

    def test_hailort_error_lines_are_extracted(self):
        err = ("[HailoRT] [warning] benign\n"
               "[HailoRT] [error] CHECK failed - max_desc_page_size given 16384 "
               "is bigger than hw max desc page size 4096\n"
               "noise here\n")
        got = F._hailort_errors(err)
        self.assertTrue(any("max_desc_page_size" in l for l in got))

    def test_the_survey_completes_even_when_a_section_crashes(self):
        """The regression, end to end: one section's child dies by SIGBUS and the
        report still contains every section."""
        real_child = F._child

        def crashing(code, timeout=120, cwd=None):
            if "PIJSON" in code:                 # the detector section
                return {"rc": -7, "signal": "SIGBUS", "stdout": "",
                        "stderr": "[HailoRT] [error] CHECK failed - boom",
                        "timed_out": False}
            return real_child(code, timeout=timeout, cwd=cwd)
        F._child = crashing
        try:
            rep = F.build(make_args(sec="all"))
        finally:
            F._child = real_child
        self.assertIn("detector", rep["sections"])
        self.assertTrue(rep["sections"]["detector"]["crashed"])
        self.assertEqual(rep["sections"]["detector"]["signal"], "SIGBUS")
        for name, _fn in F.SECTIONS:
            self.assertIn(name, rep["sections"],
                          "a crashed section lost the sections after it")
        text = F.summarise(rep)
        self.assertIn("CRASHED by SIGBUS", text)
        self.assertIn("CHECK failed - boom", text)


class TestHailoSteps(unittest.TestCase):
    def _with_child(self, results):
        """results: list of dicts, one per probe step, in order."""
        seq = list(results)

        def fake(code, timeout=120, cwd=None):
            return seq.pop(0) if seq else {"rc": 0, "signal": None,
                                           "stdout": "", "stderr": "",
                                           "timed_out": False}
        real, F._child = F._child, fake
        try:
            return F.sec_hailo_steps(make_args())
        finally:
            F._child = real

    @staticmethod
    def ok(out="fine"):
        return {"rc": 0, "signal": None, "stdout": out, "stderr": "",
                "timed_out": False}

    @staticmethod
    def crasher(err="[HailoRT] [error] CHECK failed - page size"):
        return {"rc": -7, "signal": "SIGBUS", "stdout": "", "stderr": err,
                "timed_out": False}

    @staticmethod
    def failing(err="ModuleNotFoundError: No module named 'hailo_platform'"):
        return {"rc": 1, "signal": None, "stdout": "", "stderr": err,
                "timed_out": False}

    def test_no_step_label_names_an_api_its_body_never_calls(self):
        """A LABEL IS A CLAIM ABOUT THE CODE THAT RUNS.

        Step 5 was labelled "hef.create_configure_params(StreamInterface.PCIe)"
        while its body called hailo_configure_params(), so a report reading
        "CRASHED at: 5. hef.create_configure_params" would have named a call
        that never ran - the exact trap that produced the SIGSEGV hunt. Every
        API a label names must actually appear in that step's program.
        """
        seen = []
        real, F._child = F._child, (
            lambda code, timeout=120, cwd=None: (
                seen.append(code),
                {"rc": 0, "signal": None, "stdout": "", "stderr": "",
                 "timed_out": False})[1])
        try:
            got = F.sec_hailo_steps(make_args())
        finally:
            F._child = real
        self.assertEqual(len(seen), 13, "one program per ladder step")
        steps = list(zip([e["step"] for e in got["steps"]], seen))
        apis = ("hef.create_configure_params", "hailo_configure_params",
                "create_from_hef", "vdev.configure", "create_params",
                "Device.scan", "InferVStreams", "activate", "vstream_infos",
                "VDevice", "HEF")
        for label, code in steps:
            for api in apis:
                if api in label:
                    self.assertIn(api, code,
                                  "%r is named in the label %r but is not in "
                                  "that step's program" % (api, label))

    def test_all_ok(self):
        got = self._with_child([self.ok()] * 13)
        self.assertEqual(got["verdict"], "ok - every step completed")
        self.assertIsNone(got["first_crash"])
        self.assertIsNone(got["first_failure"])

    def test_a_crash_names_the_step_and_keeps_probing(self):
        got = self._with_child([self.ok()] * 3 + [self.crasher()] + [self.ok()] * 9)
        # substring, not equality: the step labels carry an ordinal prefix and
        # pinning the exact string would break every time the ladder is reordered
        self.assertIn("open the device (hp.VDevice)", got["first_crash"])
        self.assertIn("CRASHED", got["verdict"])
        self.assertGreaterEqual(len(got["steps"]), 13,
                                "probing stopped at the crash instead of continuing")
        bad = [s for s in got["steps"] if s.get("signal")][0]
        self.assertTrue(any("page size" in l for l in bad["hailort_errors"]),
                        "HailoRT's own error text was not captured")

    def test_several_crashes_still_report_CRASHED_and_name_the_first(self):
        """The realistic shape on the real Pi.

        The ladder is cumulative - every step redoes the earlier calls - so ONE
        bad call makes several consecutive steps die. A verdict that only says
        CRASHED when exactly one step crashed would report a SIGSEGV as an
        ordinary failure precisely when it matters most. Found by mutating that
        condition: the mutation survived until this test existed.
        """
        # Choose the crashing step by its CODE, not by an index: positional
        # arithmetic broke the moment a step was inserted, and pinning labels
        # would break on every reorder. Steps 6-11 all contain this line, so this
        # is exactly the multi-crash shape.
        ok, crash = self.ok(), self.crasher()

        def by_code(code, timeout=120, cwd=None):
            return crash if "ng = vdev.configure(hef, cfg)" in code else ok
        real, F._child = F._child, by_code
        try:
            got = F.sec_hailo_steps(make_args())
        finally:
            F._child = real
        self.assertTrue(got["verdict"].startswith("CRASHED at:"), got["verdict"])
        self.assertIn("vdev.configure", got["first_crash"])
        self.assertGreater(len(got["steps"]), 5)

    def test_a_plain_failure_is_not_reported_as_a_crash(self):
        """The verdict bug: a machine with no hailo_platform reported 'all 5
        steps completed' while four had failed."""
        got = self._with_child([self.failing()] * 4 + [self.ok()] * 9)
        self.assertIn("import hailo_platform", got["verdict"])
        self.assertTrue(got["verdict"].startswith("FAILED at:"), got["verdict"])
        self.assertIsNone(got["first_crash"])
        self.assertIn("import hailo_platform", got["first_failure"])

    def test_a_missing_hef_is_called_out(self):
        got = self._with_child([self.ok()] * 13)
        self.assertIn("hef_path", got)
        if not (got["hef_path"] and os.path.exists(got["hef_path"])):
            self.assertIn("no HEF", got.get("note", ""))

    def test_a_timed_out_step_is_reported_as_a_failure_not_a_crash(self):
        """A wedged device is not the same as a segfaulting one, and the report
        should not conflate them."""
        got = self._with_child([{"rc": None, "signal": None, "stdout": "",
                                 "stderr": "", "timed_out": True}] * 13)
        self.assertIn("import hailo_platform", got["first_failure"])
        self.assertIsNone(got["first_crash"])
        self.assertIn("FAILED", got["verdict"])


class TestReportShape(unittest.TestCase):
    def _args(self, **kw):
        class A(object):
            pass
        a = A()
        for k, v in dict(config=os.path.join(_PI, "config.yaml"), out_dir="/tmp",
                         role="bottom", samples=1, infer_frames=1, dist_m=0.0,
                         panel_mm=0.0, panel_mm_h=0.0, qr_modules=0,
                         no_capture=True, no_decode=True, no_infer=True,
                         with_fc=False, fc_device=None, with_tests=False,
                         sec="all", json=None).items():
            setattr(a, k, kw.get(k, v))
        return a

    def test_summary_is_ascii_only(self):
        """It is read over ssh into terminals of unknown encoding."""
        rep = F.build(self._args(sec="system"))
        text = F.summarise(rep)
        self.assertTrue(text.isascii(),
                        "the summary printed non-ASCII: %r"
                        % [c for c in text if not c.isascii()][:5])

    def test_a_partial_report_does_not_print_empty_rows(self):
        rep = F.build(self._args(sec="fov"))
        text = F.summarise(rep)
        self.assertIn("FOV", text)
        self.assertNotIn("decode     :", text,
                         "a section that did not run was still summarised")
        self.assertNotIn("None", text)

    def test_the_report_is_json_serialisable(self):
        rep = F.build(self._args(sec="system"))
        blob = json.dumps(rep, sort_keys=True, default=str)
        self.assertIn("read_only", blob)
        self.assertTrue(json.loads(blob)["read_only"])

    def test_sec_limits_which_sections_run(self):
        rep = F.build(self._args(sec="deps"))
        self.assertEqual(list(rep["sections"].keys()), ["deps"])

    def test_all_sections_run_by_default(self):
        rep = F.build(self._args(sec="all"))
        for name, _fn in F.SECTIONS:
            self.assertIn(name, rep["sections"])


class TestItCannotActuateAnything(unittest.TestCase):
    """Structural read-only guarantee: this runs on a flight vehicle."""

    def test_source_contains_no_write_or_actuate_calls(self):
        src = open(_TOOL, encoding="utf-8").read()
        # strip comments/docstrings: the words legitimately appear in prose
        code = "\n".join(l.split("#")[0] for l in src.splitlines())
        for banned in ("yaml.safe_dump", "yaml.dump", "open(args.config, \"w\")",
                       "component_arm_disarm", "arm_disarm", ".arm(",
                       "set_mode", "upload_fence", "clear_fence", "nav_takeoff",
                       "set_vcm", "set_controls", "set_control", "pip install",
                       "hef_info.py --convert", "set_param"):
            self.assertNotIn(banned, code,
                             "pi_field_report must never actuate: found %r" % banned)

    def test_it_only_ever_opens_the_config_for_reading(self):
        src = open(_TOOL, encoding="utf-8").read()
        for line in src.splitlines():
            if "open(" in line and "config" in line:
                self.assertNotIn('"w"', line)
                self.assertNotIn("'w'", line)

    def test_the_read_only_flag_is_true_and_documented(self):
        self.assertIn("read_only", inspect.getsource(F.build))
        self.assertIn("READ-ONLY", F.__doc__)

    def test_no_fc_probe_runs_without_the_flag(self):
        """--with-fc is the only path to MAVLink, and it is opt-in."""
        src = inspect.getsource(F.sec_fc)
        self.assertIn("args.with_fc", src)
        self.assertTrue(src.index("args.with_fc") < src.index("fc_probe.py"),
                        "the fc_probe call must sit behind the flag")

    def test_it_never_runs_a_shell(self):
        src = open(_TOOL, encoding="utf-8").read()
        self.assertNotIn("shell=True", src)
        self.assertNotIn("os.system", src)


class TestCliEntryPoint(unittest.TestCase):
    def test_help_works(self):
        r = subprocess.run([sys.executable, _TOOL, "--help"],
                           capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("--dist-m", r.stdout)
        self.assertIn("--with-fc", r.stdout)

    def test_a_full_run_exits_zero_on_a_bare_machine(self):
        """No camera, no Hailo, no FC - the normal case for CI and laptops."""
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            r = subprocess.run([sys.executable, _TOOL, "--no-capture",
                                "--out-dir", td], capture_output=True, text=True,
                               timeout=300)
            self.assertEqual(r.returncode, 0, r.stderr[-2000:])
            self.assertIn("PI FIELD REPORT", r.stdout)
            files = [f for f in os.listdir(td) if f.endswith(".json")]
            self.assertEqual(len(files), 1)
            with open(os.path.join(td, files[0])) as fh:
                self.assertTrue(json.load(fh)["read_only"])

    def test_two_runs_do_not_overwrite_each_other(self):
        """A quick --sec run clobbered the full survey once."""
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            for sec in ("system", "deps"):
                subprocess.run([sys.executable, _TOOL, "--sec", sec,
                                "--out-dir", td], capture_output=True,
                               timeout=120)
            files = [f for f in os.listdir(td) if f.endswith(".json")]
            self.assertEqual(len(files), 2, "runs overwrote each other: %s" % files)

    def test_the_config_is_not_modified_by_a_run(self):
        cfg = os.path.join(_PI, "config.yaml")
        before = open(cfg, "rb").read()
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            subprocess.run([sys.executable, _TOOL, "--no-capture", "--out-dir", td],
                           capture_output=True, timeout=300)
        self.assertEqual(open(cfg, "rb").read(), before,
                         "the field report modified config.yaml")


if __name__ == "__main__":
    unittest.main(verbosity=2)
