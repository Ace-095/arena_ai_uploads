"""main.py bring-up reporting: the summary must not lie.

Run directly:   python3 tests/test_bringup_reporting.py

main.py had no tests at all, and this is the bug that proves it needed some:
`rig.detect()` fills `rig.cams` BEFORE `cam.start()` is attempted, so the old
`if rig.cams: say("cameras", "ok", ...)` reported

    ERROR: cam1 start failed: RuntimeError("cannot open video file ...")
    ERROR: cam2 start failed: RuntimeError("cannot open video file ...")
    INFO : bring-up cameras: ok (cam1=file/front, cam2=file/bottom)

The bring-up ladder exists so an operator tests one rung at a time. A line
claiming "ok" while every camera is dead sends them hunting a vision bug with
a black screen. This was observed on a REAL run, not invented.
"""
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import main as M                                                    # noqa: E402


class TestCameraSummary(unittest.TestCase):
    def test_all_started_is_ok(self):
        state, detail = M.camera_summary(["cam1", "cam2"], [])
        self.assertEqual(state, "ok")
        self.assertIn("cam1", detail)
        self.assertIn("cam2", detail)

    def test_every_camera_failed_is_failed_not_ok(self):
        """THE regression: assigned-but-not-started must never read as ok."""
        state, detail = M.camera_summary([], ["cam1", "cam2"],
                                         assigned=["cam1", "cam2"])
        self.assertEqual(state, "failed")
        self.assertIn("cam1", detail)
        self.assertIn("cam2", detail)

    def test_some_failed_is_degraded_and_names_the_failures(self):
        state, detail = M.camera_summary(["cam1"], ["cam2"])
        self.assertEqual(state, "degraded")
        self.assertIn("cam2", detail)
        self.assertIn("cam1", detail)

    def test_assigned_but_nothing_running_is_failed(self):
        """A rig that claims cameras it never started is the exact trap."""
        state, detail = M.camera_summary([], [], assigned=["cam1"])
        self.assertEqual(state, "failed")
        self.assertIn("cam1", detail)

    def test_nothing_at_all_is_failed(self):
        state, _ = M.camera_summary([], [])
        self.assertEqual(state, "failed")

    def test_only_a_success_reports_ok(self):
        """Exhaustive over the small input space: 'ok' iff started and no failures."""
        for started in ([], ["a"], ["a", "b"]):
            for failed in ([], ["a"], ["a", "b"]):
                for assigned in ([], ["a"], ["a", "b"]):
                    state, _ = M.camera_summary(started, failed, assigned)
                    expected_ok = bool(started) and not failed
                    self.assertEqual(
                        state == "ok", expected_ok,
                        "started=%r failed=%r assigned=%r -> %r"
                        % (started, failed, assigned, state))

    def test_state_is_always_one_of_three_words(self):
        for state in (M.camera_summary(["a"], [])[0],
                      M.camera_summary(["a"], ["b"])[0],
                      M.camera_summary([], ["a"])[0]):
            self.assertIn(state, ("ok", "degraded", "failed"))

    def test_inputs_are_not_mutated(self):
        started, failed = ["cam1"], ["cam2"]
        M.camera_summary(started, failed)
        self.assertEqual(started, ["cam1"])
        self.assertEqual(failed, ["cam2"])


class TestBringUpWiring(unittest.TestCase):
    """The helper must actually be wired into the camera bring-up path."""

    def test_bring_up_uses_the_helper(self):
        import inspect
        src = inspect.getsource(M.bring_up_thread)
        self.assertIn("camera_summary(", src,
                      "bring-up no longer calls camera_summary — the 'ok while "
                      "every camera failed' bug can come back")

    def test_bring_up_no_longer_reports_ok_on_a_bare_rig_cams_check(self):
        import inspect
        src = inspect.getsource(M.bring_up_thread)
        self.assertNotIn(
            'if rig.cams:\n                say("cameras", "ok"', src,
            "bring-up went back to reporting ok purely because rig.cams is "
            "non-empty (detect() populates it before start() is attempted)")

    def test_bring_up_tracks_failures(self):
        import inspect
        src = inspect.getsource(M.bring_up_thread)
        self.assertIn("failed_start", src,
                      "bring-up does not record which cameras failed to start")


if __name__ == "__main__":
    unittest.main(verbosity=2)
