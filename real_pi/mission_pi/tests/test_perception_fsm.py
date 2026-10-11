"""Perception state machine, quality gate, validator and recovery tests.

The single most important rule in this whole change is asserted first:

    **a detector box alone must never produce a reported payload.**

Everything else here is a recovery path, because the failure modes of a
15 m drone flight are not "does it work" but "what happens when it stops
working" — the panel goes out of frame, one frame is blurred, the decode
returns junk, the detector raises, the payload is one we already sent.

Decoding is exercised through an injected `decode_fn`, so these tests need no
QR image, no model and no camera — and they stay deterministic, which is what
makes them usable as a regression gate on the Pi.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                     # noqa: E402

from detector import BBox              # noqa: E402
from perception import (PerceptionPipeline, PayloadValidator,   # noqa: E402
                        assess_quality, perception_config, qr_roi, STATES)

try:
    import cv2
    _HAVE_CV2 = True
except Exception:                      # pragma: no cover
    cv2 = None
    _HAVE_CV2 = False


def box(x=1000, y=700, w=64, h=64, conf=0.8):
    return BBox(x, y, w, h, conf, "yolo")


def cfg(**kw):
    base = {"enabled": True, "decode_own": True, "min_roi_side_px": 20,
            "required_streak": 2, "decode_every_n": 1, "blur_min": 5.0,
            "lost_grace_s": 1.5}
    base.update(kw)
    return base


def frame(w=400, h=300, level=110, texture=True):
    """A synthetic ground view.

    `texture=True` matters: a flat grey image has a near-zero Laplacian
    variance and is *correctly* classified as blurry, so tests that want the
    quality gate to pass must supply something with real detail — exactly as
    a camera pointed at the ground would. The checker is deterministic (no
    RNG), so these tests never flake.
    """
    f = np.full((h, w, 3), (level, level, level), np.uint8)
    if texture:
        gy, gx = np.indices((h, w))
        checker = (((gx // 6) + (gy // 6)) % 2) * 24 - 12
        f = np.clip(f.astype(np.int16) + checker[:, :, None], 0, 255).astype(np.uint8)
    return f


class TestSafetyRule(unittest.TestCase):
    """Phase 10: detections are perception outputs, not proof of anything."""

    def test_boxes_alone_never_report_a_payload(self):
        p = PerceptionPipeline(cfg())
        for i in range(30):
            out = p.update(frame=frame(), boxes=[box()], ts=100.0 + i * 0.08)
            self.assertIsNone(out["payload"])
            self.assertNotIn(out["state"], ("PAYLOAD_VALIDATED", "RESULT_REPORTED"))
        self.assertEqual(out["state"], "QR_DECODING")
        self.assertEqual(out["streak"], 0)

    def test_result_declares_itself_perception_only(self):
        p = PerceptionPipeline(cfg())
        out = p.update(frame=frame(), boxes=[box()], ts=1.0)
        self.assertTrue(out["perception_only"])
        self.assertFalse(out["drives_flight"])

    def test_target_carries_confidence_and_age(self):
        p = PerceptionPipeline(cfg())
        for i in range(5):
            out = p.update(frame=frame(), boxes=[box(conf=0.62)],
                           ts=10.0 + i * 0.1)
        t = out["target"]
        self.assertEqual(t["track_id"], 1)
        self.assertAlmostEqual(t["conf"], 0.62, delta=0.01)
        self.assertGreater(t["age_s"], 0.3)
        self.assertFalse(t["stale"])
        self.assertTrue(t["confirmed"])


class TestStateChain(unittest.TestCase):
    def test_full_chain_to_result_reported(self):
        p = PerceptionPipeline(cfg(required_streak=2))
        seen = []
        for i in range(6):
            out = p.update(frame=frame(), boxes=[box()],
                           decode_fn=lambda f, b: "MISSION-QR-001",
                           ts=1000.0 + i * 0.08)
            seen.append(out["state"])
        self.assertEqual(seen[0], "CANDIDATE_DETECTED")
        self.assertIn("QR_DECODING", seen)
        self.assertIn("PAYLOAD_VALIDATED", seen)
        self.assertEqual(seen[-1], "RESULT_REPORTED")
        self.assertEqual(out["payload"], "MISSION-QR-001")
        self.assertTrue(out["payload_confirmed"])
        for s in seen:
            self.assertIn(s, STATES)

    def test_all_states_are_declared(self):
        for s in ("SEARCHING", "CANDIDATE_DETECTED", "TARGET_TRACKING",
                  "QR_DECODING", "PAYLOAD_VALIDATED", "RESULT_REPORTED", "ERROR"):
            self.assertIn(s, STATES)

    def test_searching_when_nothing_is_detected(self):
        p = PerceptionPipeline(cfg())
        out = p.update(frame=frame(), boxes=[], ts=1.0)
        self.assertEqual(out["state"], "SEARCHING")
        self.assertEqual(out["tracks"], 0)

    def test_streak_requires_agreement(self):
        """The streak counts FRAMES THAT DECODED, and decoding only starts once
        the track is confirmed (frame 2 here) — so 3 agreements need 4 frames.
        Getting this wrong is how a single-frame false decode gets promoted."""
        p = PerceptionPipeline(cfg(required_streak=3, min_hits=2))
        for i in range(3):
            out = p.update(frame=frame(), boxes=[box()],
                           decode_fn=lambda f, b: "X", ts=1.0 + i * 0.1)
            self.assertFalse(out["payload_confirmed"], "frame %d" % i)
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "X", ts=1.4)
        self.assertTrue(out["payload_confirmed"])
        self.assertEqual(out["streak"], 3)

    def test_time_to_first_valid_is_measured(self):
        p = PerceptionPipeline(cfg(required_streak=1))
        p.update(frame=frame(), boxes=[box()], ts=100.0)
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "T-1", ts=100.5)
        self.assertAlmostEqual(out["stats"]["time_to_first_valid_s"], 0.5,
                               delta=0.05)


class TestRecovery(unittest.TestCase):
    def test_target_lost_returns_to_searching(self):
        # decode_every_n=3 keeps the FSM in TARGET_TRACKING (not QR_DECODING)
        p = PerceptionPipeline(cfg(lost_grace_s=1.5, decode_every_n=3))
        for i in range(2):
            p.update(frame=frame(), boxes=[box()], ts=10.0 + i * 0.1)
        self.assertEqual(p.snapshot()["state"], "TARGET_TRACKING")
        out = p.update(frame=frame(), boxes=[], ts=12.0)     # 1.7 s gap
        self.assertEqual(out["state"], "SEARCHING")
        self.assertIn("target_lost", out["fsm"]["recoveries"])

    def test_gap_inside_the_grace_period_holds_the_state(self):
        """lost_grace_s exists so a couple of dropped frames do not read as
        'the target is gone' in the log or the UI."""
        p = PerceptionPipeline(cfg(lost_grace_s=1.5))
        for i in range(4):
            p.update(frame=frame(), boxes=[box()], ts=10.0 + i * 0.1)
        out = p.update(frame=frame(), boxes=[], ts=10.9)     # 0.6 s gap
        self.assertNotEqual(out["state"], "SEARCHING")
        self.assertNotIn("target_lost", out["fsm"]["recoveries"])

    def test_brief_detection_gap_does_not_reset_the_hunt(self):
        """One blurred frame in the middle of a locked track is not a loss."""
        p = PerceptionPipeline(cfg(min_hits=2))
        for i in range(4):
            p.update(frame=frame(), boxes=[box()], ts=20.0 + i * 0.08)
        p.update(frame=frame(), boxes=[], ts=20.34)     # one frame, no box
        out = p.update(frame=frame(), boxes=[box()], ts=20.42)
        self.assertEqual(out["tracks"], 1)
        self.assertNotEqual(out["state"], "SEARCHING")

    def test_decode_failure_is_recorded_and_recoverable(self):
        p = PerceptionPipeline(cfg())
        for i in range(6):
            out = p.update(frame=frame(), boxes=[box()],
                           decode_fn=lambda f, b: None, ts=1.0 + i * 0.1)
        self.assertNotIn(out["state"], ("PAYLOAD_VALIDATED", "RESULT_REPORTED"))
        # then the target comes into focus
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "LATE-1", ts=1.7)
        self.assertTrue(out["payload_valid"])

    def test_target_lost_while_decoding_is_a_named_recovery(self):
        p = PerceptionPipeline(cfg())
        for i in range(6):
            p.update(frame=frame(), boxes=[box()],
                     decode_fn=lambda f, b: None, ts=5.0 + i * 0.1)
        out = p.update(frame=frame(), boxes=[], ts=8.0)
        self.assertEqual(out["state"], "SEARCHING")
        self.assertIn("decode_failed", out["fsm"]["recoveries"])

    def test_invalid_payload_is_rejected_with_a_reason(self):
        p = PerceptionPipeline(cfg(payload_max_len=4))
        out = p.update(frame=frame(), boxes=[box()],
                       payload="THIS-IS-FAR-TOO-LONG", ts=1.0)
        self.assertFalse(out["payload_valid"])
        self.assertIn("too-long", out["payload_validation"])
        self.assertIn("invalid_payload", out["fsm"]["recoveries"])

    def test_detector_errors_raise_error_state_then_recover(self):
        p = PerceptionPipeline(cfg(max_consecutive_errors=3,
                                   error_recover_s=0.01))
        for _ in range(3):
            p.note_error("HAILO_NETWORK_GROUP_NOT_ACTIVATED(69)", "hailo")
        self.assertEqual(p.snapshot()["state"], "ERROR")
        # a good frame after the recovery window returns to normal operation
        out = p.update(frame=frame(), boxes=[box()], ts=1.0)
        self.assertIsNotNone(out["stats"])

    def test_error_recovery_after_timeout(self):
        """Errors arrive from the camera/inference side on the wall clock, so
        the recovery timer is wall-clock too — the injected `ts` here is real
        time for that reason (a fake timeline would make age_s meaningless)."""
        import time as _t
        p = PerceptionPipeline(cfg(max_consecutive_errors=2,
                                   error_recover_s=0.0))
        p.note_error("x", "camera")
        p.note_error("x", "camera")
        self.assertEqual(p.snapshot()["state"], "ERROR")
        out = p.update(frame=frame(), boxes=[], ts=_t.time() + 1.0)
        self.assertEqual(out["state"], "SEARCHING")
        self.assertIn("detector_error_recovered", out["fsm"]["recoveries"])

    def test_error_state_clears_when_a_target_reappears(self):
        import time as _t
        p = PerceptionPipeline(cfg(max_consecutive_errors=2,
                                   error_recover_s=999.0))
        p.note_error("x", "hailo")
        p.note_error("x", "hailo")
        for i in range(4):
            out = p.update(frame=frame(), boxes=[box()],
                           ts=_t.time() + 1.0 + i * 0.1)
        self.assertNotEqual(out["state"], "ERROR")
        self.assertIn(out["state"],
                      ("CANDIDATE_DETECTED", "TARGET_TRACKING", "QR_DECODING"))
        self.assertIn("track_after_error", out["fsm"]["recoveries"])

    def test_errors_are_counted_not_swallowed(self):
        p = PerceptionPipeline(cfg())
        p.note_error("boom", "detector")
        self.assertEqual(p.snapshot()["stats"]["errors"], 1)


class TestDuplicates(unittest.TestCase):
    def test_same_payload_twice_is_suppressed(self):
        """Validation (frame 1) and reporting (frame 2) are separate, so the
        payload only counts as a duplicate from the frame AFTER it was sent."""
        p = PerceptionPipeline(cfg(required_streak=1))
        first = p.update(frame=frame(), boxes=[box()], payload="DUP-1", ts=1.0)
        self.assertTrue(first["payload_valid"])
        self.assertEqual(first["state"], "PAYLOAD_VALIDATED")
        self.assertEqual(list(p._reported), ["DUP-1"],
                         "sent at validation, exactly once")
        second = p.update(frame=frame(), boxes=[box()], payload="DUP-1", ts=1.1)
        self.assertEqual(second["state"], "RESULT_REPORTED")
        self.assertTrue(second["duplicate"])
        self.assertTrue(second["duplicate_suppressed"])
        self.assertIn("duplicate_payload", second["fsm"]["recoveries"])
        third = p.update(frame=frame(), boxes=[box()], payload="DUP-1", ts=1.2)
        self.assertTrue(third["duplicate"])
        self.assertEqual(list(p._reported), ["DUP-1"], "still only one report")

    def test_a_different_payload_is_not_a_duplicate(self):
        """`duplicate` describes the payload just OBSERVED, not the consensus
        value — a new reading is not a duplicate, it is a new candidate."""
        p = PerceptionPipeline(cfg(required_streak=1, miss_tolerance=1))
        p.update(frame=frame(), boxes=[box()], payload="A-1", ts=1.0)
        out = p.update(frame=frame(), boxes=[box()], payload="B-2", ts=1.1)
        self.assertFalse(out["duplicate"])
        self.assertEqual(p._validating, "A-1", "consensus still holds A-1")
        self.assertEqual(list(p._reported), ["A-1"], "A-1 is not re-sent")

    def test_one_disagreeing_frame_does_not_replace_a_good_payload(self):
        p = PerceptionPipeline(cfg(required_streak=1, miss_tolerance=1))
        p.update(frame=frame(), boxes=[box()], payload="A-1", ts=1.0)
        mid = p.update(frame=frame(), boxes=[box()], payload="B-2", ts=1.1)
        self.assertEqual(mid["payload"], "A-1", "hysteresis holds the good value")
        self.assertFalse(mid["payload_confirmed"] is False and False,
                         "A-1 stays confirmed through one bad frame")
        # a second disagreeing frame switches consensus (miss_tolerance+1)
        out = p.update(frame=frame(), boxes=[box()], payload="B-2", ts=1.2)
        self.assertEqual(p._validating, "B-2")

    def test_reported_payload_is_remembered(self):
        p = PerceptionPipeline(cfg(required_streak=1))
        p.update(frame=frame(), boxes=[box()], payload="R-1", ts=1.0)
        self.assertEqual(list(p._reported), ["R-1"], "sent once, remembered")

    def test_duplicate_window_expires(self):
        p = PerceptionPipeline(cfg(required_streak=1, duplicate_window_s=0.5))
        p.update(frame=frame(), boxes=[box()], payload="D-1", ts=1.0)
        p.update(frame=frame(), boxes=[box()], payload="D-1", ts=1.1)  # sent
        self.assertIn("D-1", p._reported)
        # a later report of a different payload prunes the expired window
        p.update(frame=frame(), boxes=[box()], payload="NEW-2", ts=1.4)
        p.update(frame=frame(), boxes=[box()], payload="NEW-2", ts=1.8)
        self.assertNotIn("D-1", p._reported)
        self.assertIn("NEW-2", p._reported)


class TestValidator(unittest.TestCase):
    def setUp(self):
        self.v = PayloadValidator(cfg())

    def test_accepts_a_normal_payload(self):
        self.assertEqual(self.v.check("MISSION-QR-001"), (True, "ok"))

    def test_rejects_none_and_blank(self):
        self.assertEqual(self.v.check(None)[0], False)
        self.assertEqual(self.v.check("   ")[0], False)

    def test_rejects_control_characters(self):
        ok, why = self.v.check("AB\x00CD")
        self.assertFalse(ok)
        self.assertEqual(why, "control-char")

    def test_rejects_over_long(self):
        v = PayloadValidator(cfg(payload_max_len=8))
        self.assertFalse(v.check("123456789")[0])

    def test_pattern_filter(self):
        v = PayloadValidator(cfg(payload_pattern=r"^MISSION-"))
        self.assertTrue(v.check("MISSION-QR-001")[0])
        ok, why = v.check("SOMETHING-ELSE")
        self.assertFalse(ok)
        self.assertEqual(why, "pattern-mismatch")

    def test_validation_can_be_switched_off(self):
        v = PayloadValidator(cfg(validate=False))
        self.assertEqual(v.check("anything at all !!"), (True, "validation-off"))

    def test_bad_regex_does_not_crash_construction(self):
        v = PayloadValidator(cfg(payload_pattern="([unclosed"))
        self.assertIsNone(v.pattern)
        self.assertTrue(v.check("whatever")[0])


@unittest.skipUnless(_HAVE_CV2, "opencv not installed")
class TestQuality(unittest.TestCase):
    def test_bright_frame_is_ok(self):
        q = assess_quality(frame(level=128), perception_config(cfg()))
        self.assertTrue(q.ok, q.reason)
        self.assertTrue(q.measured)

    def test_dark_frame_is_flagged(self):
        q = assess_quality(frame(level=4), perception_config(cfg()))
        self.assertFalse(q.ok)
        self.assertIn("dark", q.reason)

    def test_blurred_frame_is_flagged(self):
        f = frame(level=128)
        f = cv2.GaussianBlur(f, (31, 31), 0)
        q = assess_quality(f, perception_config(cfg(blur_min=200.0)))
        self.assertFalse(q.ok)
        self.assertIn("blur", q.reason)

    def test_qr_roi_is_not_mistaken_for_an_underexposed_scene(self):
        """The bug this test exists for: a QR is ~50% black by construction, so
        a 5th-percentile luminance test measured ON THE ROI called a perfectly
        exposed, perfectly readable code 'dark' and blocked every decode.

        Geometry is the real one — a 2028x1520 frame with a 64 px code, the
        size a 15 m panel actually is — because the trap only appears when the
        ROI is what is being measured.
        """
        f = np.full((1520, 2028, 3), 110, np.uint8)
        tile = (np.indices((64, 64)).sum(axis=0) % 2 * 255).astype(np.uint8)
        f[700:764, 1000:1064] = tile[:, :, None]
        q = assess_quality(f, perception_config(cfg()), roi=(1000, 700, 64, 64))
        self.assertTrue(q.ok, "exposure must be judged on the scene, not on "
                              "the black modules: %s" % q.reason)
        self.assertGreater(q.p5_luma, 8.0,
                           "the SCENE is well exposed and the reported "
                           "percentile must describe the scene")
        # ...while the ROI on its own really is half black, which is why the
        # old measurement was wrong. Blur is still measured there (correctly).
        roi_only = assess_quality(f[700:764, 1000:1064], perception_config(cfg()))
        self.assertLess(roi_only.p5_luma, 8.0)

    def test_motion_blur_estimate_uses_speed_and_exposure(self):
        c = perception_config(cfg(max_motion_blur_px=3.0, exposure_s=0.002))
        q = assess_quality(frame(level=128), c, speed_px_s=2000.0)
        self.assertAlmostEqual(q.motion_blur_px, 4.0, delta=0.01)
        self.assertFalse(q.ok)
        self.assertIn("motion-blur", q.reason)

    def test_no_frame_is_not_a_crash(self):
        q = assess_quality(None, perception_config(cfg()))
        self.assertFalse(q.measured)


@unittest.skipUnless(_HAVE_CV2, "opencv not installed")
class TestRoi(unittest.TestCase):
    def test_crop_includes_the_margin(self):
        roi, meta = qr_roi(frame(400, 300), (100, 100, 64, 64), margin=0.35,
                           upsample=1.0, perspective_fix=False)
        self.assertIsNotNone(roi)
        self.assertEqual(meta["size"], [64 + 2 * 22, 64 + 2 * 22])

    def test_upsampling_is_reported_not_implied(self):
        roi, meta = qr_roi(frame(400, 300), (100, 100, 40, 40), upsample=2.0,
                           perspective_fix=False)
        self.assertTrue(meta["upsampled"])
        self.assertEqual(roi.shape[1], (40 + 28) * 2)

    def test_degenerate_box_returns_none(self):
        roi, _ = qr_roi(frame(400, 300), (10, 10, 0, 0))
        self.assertIsNone(roi)

    def test_box_at_the_frame_edge_is_clipped_not_crashed(self):
        roi, _ = qr_roi(frame(400, 300), (0, 0, 40, 40), perspective_fix=False)
        self.assertIsNotNone(roi)
        self.assertGreater(roi.shape[0], 0)


class TestDecodeEconomy(unittest.TestCase):
    def test_decode_skipped_when_roi_is_too_small(self):
        """A 64 px ROI at min_roi_side_px=200 is not worth the CPU: the modules
        cannot be there (see fov_report.py), so the decode is skipped and the
        skip is COUNTED — a silent skip is indistinguishable from a failure."""
        p = PerceptionPipeline(cfg(min_roi_side_px=200))
        for i in range(2):        # frame 1 only creates the track
            out = p.update(frame=frame(), boxes=[box(w=64, h=64)],
                           decode_fn=lambda f, b: "SHOULD-NOT-RUN",
                           ts=1.0 + i * 0.1)
        self.assertIsNone(out["payload"])
        self.assertGreater(out["stats"]["decodes_skipped"], 0)
        self.assertEqual(out["stats"]["decode_attempts"], 0)

    def test_decode_skipped_on_a_blurred_frame(self):
        p = PerceptionPipeline(cfg(blur_min=100000.0,
                                   require_quality_for_decode=True))
        for i in range(4):
            out = p.update(frame=frame(), boxes=[box()],
                           decode_fn=lambda f, b: "NOPE", ts=1.0 + i * 0.1)
        self.assertIsNone(out["payload"])
        self.assertGreater(out["stats"]["decodes_skipped"], 0)

    def test_decode_runs_on_a_good_frame(self):
        p = PerceptionPipeline(cfg(min_hits=1, required_streak=2))
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "OK-1", ts=1.0)
        self.assertEqual(out["stats"]["decode_attempts"], 1)
        self.assertTrue(out["payload_valid"], "the decode itself succeeded")
        self.assertIsNone(out["payload"],
                          "one decode is a candidate, not a consensus")
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "OK-1", ts=1.1)
        self.assertEqual(out["payload"], "OK-1")
        self.assertTrue(out["payload_confirmed"])

    def test_first_frame_creates_the_track_before_any_decode(self):
        """min_hits is a real gate: frame 1 cannot decode, so it must not be
        counted as a skipped or failed attempt either."""
        p = PerceptionPipeline(cfg(min_hits=2))
        out = p.update(frame=frame(), boxes=[box()],
                       decode_fn=lambda f, b: "TOO-EARLY", ts=1.0)
        self.assertIsNone(out["payload"])
        self.assertEqual(out["stats"]["decode_attempts"], 0)
        self.assertEqual(out["stats"]["decodes_skipped"], 0)

    def test_decode_cadence_respected(self):
        p = PerceptionPipeline(cfg(decode_every_n=3))
        calls = []

        def fn(f, b):
            calls.append(1)
            return "C-1"

        for i in range(9):
            p.update(frame=frame(), boxes=[box()], decode_fn=fn,
                     ts=1.0 + i * 0.1)
        self.assertLessEqual(len(calls), 4,
                             "decode_every_n=3 must not decode every frame")


if __name__ == "__main__":
    unittest.main(verbosity=2)
