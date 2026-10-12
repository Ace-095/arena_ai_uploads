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

    def test_some_failed_names_both_sides_and_stays_in_vocabulary(self):
        """Partial start is `failed` (see camera_summary's docstring): /health
        must not read ready when a camera never came up, and an out-of-
        vocabulary word would report stage=ready with ready=false."""
        from bringup import Bringup
        state, detail = M.camera_summary(["cam1"], ["cam2"])
        self.assertIn(state, Bringup.STATES)
        self.assertEqual(state, "failed")
        self.assertIn("cam2", detail)          # what died
        self.assertIn("cam1", detail)          # what survived

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

    def test_state_is_always_in_the_ledger_vocabulary(self):
        from bringup import Bringup
        for state in (M.camera_summary(["a"], [])[0],
                      M.camera_summary(["a"], ["b"])[0],
                      M.camera_summary([], ["a"])[0],
                      M.camera_summary([], [], ["a"])[0]):
            self.assertIn(state, Bringup.STATES)

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




# ===========================================================================
# The ledger must not be lied to (observed on a real run, not invented)
# ===========================================================================
class TestLedgerIsNotOverwritten(unittest.TestCase):
    """`bringup.mark()` is a plain setter BY DESIGN - the FC retry needs
    failed -> pending -> ok (main.py records the first failure, marks pending
    while retrying, then ok when a later attempt connects). So the invariant
    cannot live in mark(); it lives at the call site, and these tests pin it.
    """

    def setUp(self):
        from bringup import Bringup
        self.Bringup = Bringup

    def test_bring_up_thread_reports_a_dead_camera_honestly(self):
        """Runs the REAL bring_up_thread with two cameras that cannot start.

        Source-text matching was the first attempt and it was fragile: the
        assertion matched this file's own explanatory comment. This drives the
        function instead, and asserts on the LEDGER and the UI log - the two
        things an operator actually reads.
        """
        import threading
        from bringup import Bringup
        import main as M

        class _DeadCam(object):
            kind, index, model, facing = "file", -1, "file", "bottom"
            running = False

            def start(self):
                raise RuntimeError("cannot open video file '/tmp/missing'")

            def apply_qr_profile(self, profile):
                raise AssertionError("apply_qr_profile called on a dead camera")

        class _Rig(object):
            cams = {}

            def rescan_lock(self):
                return threading.RLock()

            def detect(self):
                self.cams = {"cam1": _DeadCam(), "cam2": _DeadCam()}
                return dict(self.cams)

        class _Streams(object):
            streams = {"cam1": object(), "cam2": object()}   # objects exist

            def sync(self, rig):
                pass

            def start_all(self):
                pass

            def describe(self):
                return "12fps 960w q70 x2"

        class _Mission(object):
            detector = None

        class _Args(object):
            no_cams = False
            no_fc = True          # stop after the camera+streams stages
            device = None
            baud = None
            fc_retry = None
            fc_timeout = None

        class _Hub(object):
            def __init__(self):
                self.logs = []

            def push(self, kind, payload=None):
                if kind == "log":
                    self.logs.append(payload.get("msg", ""))

        bu = Bringup()
        bu.stop = threading.Event()
        hub = _Hub()
        M.bring_up_thread({"detector": {"kind": "classical"}}, _Args(),
                          None, _Rig(), [None], _Streams(), _Mission(),
                          hub, bu)

        # THE regression: the summary said failed, then the QR-boost line used
        # to mark cameras ok and move /health's stage off the failure.
        self.assertEqual(bu.state.get("cameras"), "failed",
                         "the cameras ledger entry was overwritten back to ok: %r"
                         % (bu.state.get("cameras"),))
        self.assertEqual(bu.stage, "cameras",
                         "/health's stage no longer points at the failed cameras")
        self.assertFalse(bu.ok(),
                         "a box with no working camera must not read as ok")
        # and the operator is told, in the log, that the tiles are dead
        joined = " ".join(hub.logs)
        self.assertIn("FAILED to start", joined)
        self.assertIn("UI tiles will stay black", joined)
        self.assertNotIn("bring-up cameras: ok", joined,
                         "an unconditional cameras=ok reached the UI log")

    def test_one_live_one_dead_camera_keeps_the_failure_in_the_ledger(self):
        """THE scenario that actually reproduces the original overwrite.

        With BOTH cameras dead the QR-boost loop never reaches its `say()` (the
        started-guard skips them), so an all-dead rig cannot catch a regression
        here - a mutation restoring the bad `say()` passed this file until this
        test existed. One camera must be ALIVE so the loop runs and the bad line
        gets a chance to fire.
        """
        import threading
        from bringup import Bringup
        import main as M

        class _Cam(object):
            kind, index = "file", -1

            def __init__(self, name, facing, alive):
                self.name, self.facing, self.model = name, facing, "file"
                self.running = False
                self._alive = alive
                self.boosted = None

            def start(self):
                if not self._alive:
                    raise RuntimeError("cannot open video file '/tmp/missing'")
                self.running = True

            def apply_qr_profile(self, profile):
                self.boosted = profile

        class _Rig(object):
            cams = {}

            def rescan_lock(self):
                return threading.RLock()

            def detect(self):
                self.cams = {"cam1": _Cam("cam1", "front", True),
                             "cam2": _Cam("cam2", "bottom", False)}
                return dict(self.cams)

        class _Streams(object):
            streams = {"cam1": object()}

            def sync(self, rig):
                pass

            def start_all(self):
                pass

            def describe(self):
                return "12fps"

        class _Args(object):
            no_cams = False
            no_fc = True
            device = None
            baud = None
            fc_retry = None
            fc_timeout = None

        class _Hub(object):
            def __init__(self):
                self.logs = []

            def push(self, kind, payload=None):
                if kind == "log":
                    self.logs.append(payload.get("msg", ""))

        bu = Bringup()
        bu.stop = threading.Event()
        hub = _Hub()
        M.bring_up_thread({"detector": {"kind": "classical"}}, _Args(), None,
                          _Rig(), [None], _Streams(),
                          type("M2", (), {"detector": None})(), hub, bu)

        self.assertEqual(bu.state.get("cameras"), "failed",
                         "the QR-boost line for the LIVE camera marked the whole "
                         "cameras subsystem ok, erasing the dead one (state=%r)"
                         % (bu.state.get("cameras"),))
        self.assertEqual(bu.stage, "cameras")
        joined = " ".join(hub.logs)
        self.assertIn("only 1 of 2 cameras running", joined,
                      "the operator is not told that only one camera came up")
        self.assertNotIn("bring-up cameras: ok", joined)

    def test_dead_camera_never_gets_a_qr_profile_applied(self):
        """A profile write is a statement about a device; a camera that never
        started must not receive one."""
        # covered by the AssertionError in _DeadCam.apply_qr_profile used by
        # test_bring_up_thread_reports_a_dead_camera_honestly, which fails loudly
        # if the started-guard is ever removed
        import inspect
        src = inspect.getsource(M.bring_up_thread)
        self.assertIn("if cam_name not in started:", src,
                      "the QR-boost loop no longer skips cameras that failed to "
                      "start (apply_qr_profile would run against a dead device)")

    def test_a_failed_camera_state_survives_finish(self):
        bu = self.Bringup()
        for s in ("server", "detector", "streams", "fc"):
            bu.mark(s, "ok", "fine")
        state, detail = M.camera_summary([], ["cam1", "cam2"], ["cam1", "cam2"])
        bu.mark("cameras", state, detail)
        bu.finish()
        self.assertEqual(bu.state.get("cameras"), "failed")
        self.assertEqual(bu.stage, "cameras",
                         "/health's stage must point at the failed subsystem")
        self.assertFalse(bu.ok(), "a dead camera must not read as ready")

    def test_fc_retry_flow_still_works(self):
        """The flow a 'sticky failure' rule would have broken (verified)."""
        bu = self.Bringup()
        for s in ("server", "detector", "cameras", "streams"):
            bu.mark(s, "ok", "fine")
        bu.fail("fc", RuntimeError("no heartbeat"))
        bu.mark("fc", "pending", "retrying every 5s")
        self.assertEqual(bu.state.get("fc"), "pending",
                         "the retry must be able to leave `failed` behind")
        bu.mark("fc", "ok", "tcp:127.0.0.1:5762 (attempt 2)")
        bu.finish()
        self.assertEqual(bu.state.get("fc"), "ok",
                         "a successful retry must be able to report ok")
        self.assertTrue(bu.ok())


class TestStateVocabulary(unittest.TestCase):
    """`ok()` and `_stage_locked` only understand Bringup.STATES."""

    def test_camera_summary_only_returns_vocabulary_states(self):
        from bringup import Bringup
        for started in ([], ["cam1"], ["cam1", "cam2"]):
            for failed in ([], ["cam2"], ["cam1", "cam2"]):
                for assigned in ([], ["cam1"], ["cam1", "cam2"]):
                    state, _ = M.camera_summary(started, failed, assigned)
                    self.assertIn(
                        state, Bringup.STATES,
                        "camera_summary returned %r for started=%r failed=%r "
                        "assigned=%r - a state outside the vocabulary reports "
                        "stage=ready with ready=false" % (state, started,
                                                          failed, assigned))

    def test_vocabulary_is_the_documented_four(self):
        from bringup import Bringup
        self.assertEqual(set(Bringup.STATES),
                         {"pending", "ok", "failed", "skipped"})

    def test_unknown_state_warns_but_never_raises(self):
        """mark() must never take the box down (see fail()'s docstring), so a
        foreign state is RECORDED with a warning - the protection is that our
        own code cannot produce one (see the camera_summary vocabulary test)
        and that a developer sees the warning."""
        import logging
        from bringup import Bringup
        bu = Bringup()
        for s in ("server", "detector", "streams", "fc"):
            bu.mark(s, "ok", "fine")
        with self.assertLogs("bringup", level="WARNING") as cm:
            bu.mark("cameras", "degraded", "half up")
        self.assertTrue(any("unknown state" in m for m in cm.output),
                        "an out-of-vocabulary state was accepted silently: %r"
                        % (cm.output,))
        # and it really is the trap: stage says ready while ok() is False
        bu.finish()
        self.assertEqual(bu.stage, "ready")
        self.assertFalse(bu.ok())


class TestStreamsHonesty(unittest.TestCase):
    def test_streams_do_not_claim_ok_with_no_running_camera(self):
        """Covered behaviourally by
        TestLedgerIsNotOverwritten.test_bring_up_thread_reports_a_dead_camera_honestly
        (it asserts "UI tiles will stay black" reaches the log). This keeps a
        direct check on the state itself for the same scenario."""
        import threading
        from bringup import Bringup
        import main as M

        class _DeadCam(object):
            kind, index, model, facing = "file", -1, "file", "bottom"
            running = False

            def start(self):
                raise RuntimeError("nope")

        class _Rig(object):
            cams = {}

            def rescan_lock(self):
                return threading.RLock()

            def detect(self):
                self.cams = {"cam1": _DeadCam()}
                return dict(self.cams)

        class _Streams(object):
            streams = {"cam1": object()}

            def sync(self, rig):
                pass

            def start_all(self):
                pass

            def describe(self):
                return "12fps"

        class _Args(object):
            no_cams = False
            no_fc = True
            device = None
            baud = None
            fc_retry = None
            fc_timeout = None

        bu = Bringup()
        bu.stop = threading.Event()
        M.bring_up_thread({"detector": {"kind": "classical"}}, _Args(), None,
                          _Rig(), [None], _Streams(), type("M2", (), {"detector": None})(),
                          type("H", (), {"push": lambda self, *a, **k: None})(), bu)
        self.assertEqual(bu.state.get("streams"), "failed",
                         "streams claimed %r with no running camera"
                         % (bu.state.get("streams"),))


if __name__ == "__main__":
    unittest.main(verbosity=2)
