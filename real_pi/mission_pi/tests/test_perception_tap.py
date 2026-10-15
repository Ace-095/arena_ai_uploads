"""Perception tap tests — the "additive, observation-only" claim, verified.

The claim being tested is specific and falsifiable:

    installing the tap does not change a single value the mission computes.

So these tests do not check that tracking "works" (test_tracker.py and
test_perception_fsm.py do that). They check the plumbing:

  1. `detect_tiles`/`decode_frame` return the SAME objects after install as
     before, and `uninstall()` puts the originals back exactly.
  2. The detector's `name` is forwarded verbatim — `Mission._worker` picks its
     inference path from `name != "classical"`, so a renamed tap would
     silently change which model runs.
  3. Tile-level `detect()` calls are NOT observed as full frames (their
     coordinates are tile-local; treating them as frame-local would scatter
     the tracker across nine phantom targets).
  4. One perception update per frame, even when the mission decodes three
     boxes plus a full frame in that same frame.
  5. A detector exception still propagates unchanged to the mission, and is
     reported to the pipeline rather than swallowed.
  6. `perception.enabled: false` leaves the modules completely untouched.

A stub pipeline is injected so these tests need no cv2, no model and no
camera, and can run on any host.
"""
import os
import sys
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                     # noqa: E402

import decoder as D                    # noqa: E402
import detector as DT                  # noqa: E402
import perception_tap as PT            # noqa: E402
from detector import BBox, Detector    # noqa: E402


class StubPipeline(object):
    """Records every update/error; no images, no detection."""

    def __init__(self, cfg=None):
        self.cfg = {}
        self.updates = []
        self.errors = []
        self.notes = []

    def update(self, **kw):
        self.updates.append(kw)
        return {"state": "SEARCHING", "n": len(self.updates)}

    def note_error(self, err, kind="detector"):
        self.errors.append((kind, str(err)))

    def snapshot(self):
        return {"state": "SEARCHING", "updates": len(self.updates)}


class FakeDetector(Detector):
    name = "fake"

    def __init__(self, boxes=None, raises=None):
        self.boxes = boxes or []
        self.raises = raises
        self.calls = 0

    def detect(self, frame_bgr):
        self.calls += 1
        if self.raises:
            raise self.raises
        # two boxes, in TILE-LOCAL coordinates when called from detect_tiles
        return [BBox(10, 10, 20, 20, 0.9, "fake")]


class FakeClassical(FakeDetector):
    name = "classical"


def frame(w=64, h=48):
    """A real array: `detect_tiles` slices it, so a sentinel object will not
    do for anything that actually tiles."""
    gy, gx = np.indices((h, w))
    f = np.clip(110 + ((((gx // 4) + (gy // 4)) % 2) * 24 - 12), 0, 255)
    return np.repeat(f.astype(np.uint8)[:, :, None], 3, axis=2)


def cfg(enabled=True, **kw):
    c = {"enabled": enabled, "flush_timeout_s": 0.5}
    c.update(kw)
    return {"perception": c}


class TapTestCase(unittest.TestCase):
    """Installs/removes the tap around every test and injects the stub."""

    def setUp(self):
        self._orig_tiles = DT.detect_tiles
        self._orig_decode = D.decode_frame
        self.patcher = mock.patch.object(PT, "PerceptionPipeline", StubPipeline)
        self.patcher.start()
        PT.INSTALLER = PT._Installer()

    def tearDown(self):
        PT.uninstall()
        PT.INSTALLER = PT._Installer()
        self.patcher.stop()
        DT.detect_tiles = self._orig_tiles
        D.decode_frame = self._orig_decode

    def install(self, **kw):
        PT.install(cfg(**kw), timer=False)
        return PT.INSTALLER.pipeline


class TestPassthrough(TapTestCase):
    def test_detect_tiles_result_is_unchanged(self):
        det = FakeDetector()
        f = frame()
        before = self._orig_tiles(det, f, rows=3, cols=3)
        self.install()
        after = DT.detect_tiles(FakeDetector(), f, rows=3, cols=3)
        self.assertEqual(len(before), len(after))
        self.assertEqual([tuple(b) for b in before], [tuple(b) for b in after])

    def test_decode_frame_result_is_unchanged(self):
        sentinel = "PAYLOAD-XYZ"
        fake = mock.Mock(return_value=sentinel)
        D.decode_frame = fake                    # stands in for the original
        PT.install(cfg(), timer=False)           # ...which the tap now wraps
        self.assertEqual(D.decode_frame(None, (0, 0, 1, 1)), sentinel)
        self.assertTrue(fake.called)

    def test_uninstall_restores_the_originals_by_identity(self):
        self.install()
        self.assertIsNot(DT.detect_tiles, self._orig_tiles)
        self.assertIsNot(D.decode_frame, self._orig_decode)
        PT.uninstall()
        self.assertIs(DT.detect_tiles, self._orig_tiles)
        self.assertIs(D.decode_frame, self._orig_decode)

    def test_wrapper_keeps_the_function_name(self):
        self.install()
        self.assertEqual(DT.detect_tiles.__name__, "detect_tiles")
        self.assertEqual(D.decode_frame.__name__, "decode_frame")


class TestDetectorDecorator(TapTestCase):
    def setUp(self):
        super().setUp()
        self.buf = PT.install(cfg(), timer=False)

    def test_name_is_forwarded_verbatim(self):
        """Mission picks tiled-vs-full inference from this string."""
        self.assertEqual(PT.wrap_detector(FakeDetector()).name, "fake")
        self.assertEqual(PT.wrap_detector(FakeClassical()).name, "classical")

    def test_attributes_pass_through(self):
        inner = FakeDetector()
        inner.conf_thr = 0.42
        tap = PT.wrap_detector(inner)
        self.assertEqual(tap.conf_thr, 0.42)
        self.assertEqual(tap.calls, 0)

    def test_set_thresholds_reaches_the_real_detector(self):
        calls = []

        class Thresh(FakeDetector):
            def set_thresholds(self, conf_thr=None, iou_thr=None):
                calls.append((conf_thr, iou_thr))

        tap = PT.wrap_detector(Thresh())
        tap.set_thresholds(conf_thr=0.3, iou_thr=0.45)
        self.assertEqual(calls, [(0.3, 0.45)])

    def test_close_reaches_the_real_detector(self):
        closed = []

        class Closeable(FakeDetector):
            def close(self):
                closed.append(True)

        PT.wrap_detector(Closeable()).close()
        self.assertEqual(closed, [True])

    def test_double_wrapping_is_a_noop(self):
        tap = PT.wrap_detector(FakeDetector())
        self.assertIs(PT.wrap_detector(tap), tap)

    def test_full_frame_detect_is_observed(self):
        tap = PT.wrap_detector(FakeDetector())
        tap.detect(object())
        self.buf.flush(force=True)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 1)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates[0]["boxes"]), 1)

    def test_detector_exception_propagates_and_is_reported(self):
        boom = RuntimeError("HAILO_NETWORK_GROUP_NOT_ACTIVATED(69)")
        tap = PT.wrap_detector(FakeDetector(raises=boom))
        with self.assertRaises(RuntimeError):
            tap.detect(object())
        self.assertEqual(PT.INSTALLER.pipeline.errors[0][0], "detector")


class TestTileGuard(TapTestCase):
    def test_tile_level_detects_are_not_observed_as_frames(self):
        """Inside detect_tiles the tap must stay quiet; only the merged
        full-frame result is an observation."""
        self.install()
        tap = PT.wrap_detector(FakeDetector())
        DT.detect_tiles(tap, frame(), rows=3, cols=3)
        PT.INSTALLER.buffer.flush(force=True)
        updates = PT.INSTALLER.pipeline.updates
        self.assertEqual(len(updates), 1,
                         "9 tiles must collapse into ONE frame observation")
        self.assertEqual(updates[0]["source"], "tap")

    def test_detect_tiles_1x1_is_observed_once(self):
        self.install()
        tap = PT.wrap_detector(FakeDetector())
        DT.detect_tiles(tap, frame(), rows=1, cols=1)
        PT.INSTALLER.buffer.flush(force=True)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 1)

    def test_guard_is_restored_after_an_exception(self):
        self.install()
        tap = PT.wrap_detector(FakeDetector(raises=RuntimeError("x")))
        with self.assertRaises(RuntimeError):
            DT.detect_tiles(tap, frame(), rows=3, cols=3)
        self.assertEqual(PT._tiles_depth(), 0,
                         "a failed tiled run must not leave the guard stuck")


class TestOneUpdatePerFrame(TapTestCase):
    def test_three_decodes_in_one_frame_are_one_update(self):
        """`mission._worker` decodes up to 3 boxes plus a full-frame attempt
        per frame. Feeding each to the tracker would inflate its miss
        counters and make a locked target look lost."""
        self.install()
        tap = PT.wrap_detector(FakeDetector())
        f = frame()
        DT.detect_tiles(tap, f, rows=1, cols=1)
        D.decode_frame(f, (0, 0, 10, 10))         # box 1
        D.decode_frame(f, (0, 0, 10, 10))         # box 2
        D.decode_frame(f)                         # full-frame fallback
        PT.INSTALLER.buffer.flush(force=True)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 1)
        self.assertIsNone(PT.INSTALLER.pipeline.updates[0]["payload"])

    def test_frame_is_emitted_when_the_next_frame_arrives(self):
        self.install()
        tap = PT.wrap_detector(FakeDetector())
        f1, f2 = frame(), frame()
        DT.detect_tiles(tap, f1, rows=1, cols=1)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 0,
                         "frame 1 is still open")
        DT.detect_tiles(tap, f2, rows=1, cols=1)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 1,
                         "frame 2 flushes frame 1")
        self.assertIs(PT.INSTALLER.pipeline.updates[0]["frame"], f1)

    def test_payload_is_attached_to_the_frame_it_came_from(self):
        self.install()
        tap = PT.wrap_detector(FakeDetector())
        f1 = frame()
        DT.detect_tiles(tap, f1, rows=1, cols=1)
        D.decode_frame = mock.Mock(return_value="MISSION-QR-001")
        PT.INSTALLER.buffer.note_payload("MISSION-QR-001")
        DT.detect_tiles(tap, frame(), rows=1, cols=1)
        u = PT.INSTALLER.pipeline.updates[0]
        self.assertEqual(u["payload"], "MISSION-QR-001")
        self.assertIs(u["frame"], f1)

    def test_stalled_frame_is_flushed_by_the_watchdog(self):
        """A camera that dies mid-frame must not silence perception: the last
        buffered frame is published once it goes stale."""
        self.install(flush_timeout_s=0.05)
        tap = PT.wrap_detector(FakeDetector())
        DT.detect_tiles(tap, frame(), rows=1, cols=1)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 0)
        deadline = time.time() + 2.0
        PT.INSTALLER._timer = None
        import threading
        t = threading.Thread(target=PT.INSTALLER._flush_loop, daemon=True)
        t.start()
        while time.time() < deadline and not PT.INSTALLER.pipeline.updates:
            time.sleep(0.02)
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 1)

    def test_empty_buffer_flush_is_harmless(self):
        self.install()
        self.assertIsNone(PT.INSTALLER.buffer.flush(force=True))
        self.assertEqual(len(PT.INSTALLER.pipeline.updates), 0)


class TestDisabledAndFailureModes(TapTestCase):
    def test_disabled_leaves_the_modules_untouched(self):
        self.assertIsNone(PT.install(cfg(enabled=False), timer=False))
        self.assertIs(DT.detect_tiles, self._orig_tiles)
        self.assertIs(D.decode_frame, self._orig_decode)
        self.assertIsNone(PT.pipeline())

    def test_snapshot_reports_disabled(self):
        self.assertIsNone(PT.install(cfg(enabled=False), timer=False))
        snap = PT.snapshot()
        self.assertEqual(snap["state"], "DISABLED")
        self.assertFalse(snap["enabled"])
        self.assertFalse(snap["drives_flight"])

    def test_snapshot_forwards_the_pipeline(self):
        self.install()
        self.assertEqual(PT.snapshot()["state"], "SEARCHING")

    def test_install_failure_does_not_propagate(self):
        """The tap is an addition; it must never stop a mission from starting."""
        with mock.patch.object(PT, "PerceptionPipeline",
                               side_effect=RuntimeError("no numpy")):
            out = PT.install(cfg(), timer=False)
        self.assertIsNone(out, "install() degraded instead of raising")
        self.assertIs(DT.detect_tiles, self._orig_tiles,
                      "and the modules were left clean")

    def test_pipeline_exception_does_not_break_detection(self):
        self.install()
        PT.INSTALLER.pipeline.update = mock.Mock(side_effect=RuntimeError("x"))
        tap = PT.wrap_detector(FakeDetector())
        boxes = DT.detect_tiles(tap, frame(), rows=1, cols=1)
        self.assertEqual(len(boxes), 1, "detection still returned its boxes")
        # the pipeline runs on the flush, where the exception is swallowed
        self.assertIsNone(PT.INSTALLER.buffer.flush(force=True))
        self.assertEqual(PT.INSTALLER.buffer.errors, 1,
                         "a broken pipeline is counted, never propagated")

    def test_install_is_idempotent(self):
        self.install()
        first = DT.detect_tiles
        PT.install(cfg(), timer=False)      # second install
        self.assertIs(DT.detect_tiles, first,
                      "a double install must not stack wrappers")
        PT.uninstall()
        self.assertIs(DT.detect_tiles, self._orig_tiles)


class TestRealPipelineIntegration(unittest.TestCase):
    """End-to-end through the REAL pipeline, driven the way `_worker` does it.

    The stub above proves the plumbing is transparent; this proves the real
    pipeline produces a validated payload when fed frames the way the mission
    feeds them — including the in-loop `from detector import detect_tiles`
    import, which is the exact statement the tap depends on.
    """

    def setUp(self):
        import numpy as np
        self.np = np
        self._orig_tiles = DT.detect_tiles
        self._orig_decode = D.decode_frame
        PT.INSTALLER = PT._Installer()

    def tearDown(self):
        PT.uninstall()
        PT.INSTALLER = PT._Installer()
        DT.detect_tiles = self._orig_tiles
        D.decode_frame = self._orig_decode

    def _frame(self, w=320, h=240):
        """Textured ground view (a flat frame is legitimately 'blurry')."""
        gy, gx = self.np.indices((h, w))
        f = self.np.clip(110 + ((((gx // 6) + (gy // 6)) % 2) * 24 - 12),
                         0, 255).astype(self.np.uint8)
        return self.np.repeat(f[:, :, None], 3, axis=2)

    def test_mission_style_loop_moves_the_state_machine(self):
        # Stand in for the real decoder BEFORE installing, so the tap wraps it
        # — replacing it afterwards would remove the wrapper and nothing would
        # ever be observed (which is exactly the bug this ordering avoids).
        # a real decode does not succeed on the first frame it is pointed at
        D.decode_frame = mock.Mock(side_effect=[None, None,
                                                "MISSION-QR-001",
                                                "MISSION-QR-001"])
        PT.install({"perception": {"enabled": True, "decode_own": False,
                                   "blur_min": 1.0, "required_streak": 1,
                                   "min_hits": 1}}, timer=False)
        pipe = PT.INSTALLER.pipeline
        self.assertIsNotNone(pipe)

        class OneBox(Detector):
            name = "fake"                      # non-classical -> tiled path
            def detect(self, frame_bgr):
                return [BBox(120, 80, 64, 64, 0.7, "fake")]

        tap = PT.wrap_detector(OneBox())
        states = []
        for _ in range(4):
            # --- exactly the statements Mission._worker makes -------------
            from detector import detect_tiles          # in-loop import
            from decoder import decode_frame
            f = frame(320, 240)
            boxes = detect_tiles(tap, f, rows=1, cols=1)
            for b in boxes[:3]:
                decode_frame(f, (b.x, b.y, b.w, b.h))
            PT.INSTALLER.buffer.flush(force=True)      # publish this frame
            # --------------------------------------------------------------
            states.append(pipe.snapshot()["state"])
        snap = pipe.snapshot()
        self.assertEqual(snap["payload"], "MISSION-QR-001")
        self.assertIn(snap["state"], ("PAYLOAD_VALIDATED", "RESULT_REPORTED"))
        self.assertEqual(states[0], "TARGET_TRACKING")
        self.assertEqual(states[1], "QR_DECODING",
                         "no payload yet -> the decode state, not a fake success")
        self.assertEqual(states[2], "PAYLOAD_VALIDATED")
        self.assertEqual(states[3], "RESULT_REPORTED")
        self.assertFalse(pipe.snapshot()["drives_flight"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
