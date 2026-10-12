"""qr_angle_bench.py — angle-vs-detection scoring, verdicts and reporting.

Run directly:   python3 tests/test_qr_angle_bench.py
(`tools/run_tests.sh` runs every test file this way.)

These are the pure-logic tests: scoring maths, the verdict ladder and the
report writers. The live-camera capture path cannot be tested without a camera
and is exercised by `--frames` (offline mode) instead, which is exactly the mode
used here.
"""
import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_TOOLS = os.path.join(_ROOT, "tools")
for _p in (_ROOT, _TOOLS, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import qr_angle_bench as QAB                                       # noqa: E402


class FakeBox(object):
    """Duck-typed like detector.BBox (x, y, w, h, conf, source)."""

    def __init__(self, x, y, w, h, conf, source="test"):
        self.x, self.y, self.w, self.h = x, y, w, h
        self.conf, self.source = conf, source


class FakeDetector(object):
    def __init__(self, boxes=None, payload=None, raise_on=None):
        self.name = "fake"
        self._boxes = boxes or []
        self._payload = payload
        self._raise_on = raise_on
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        if self._raise_on == "detect":
            raise RuntimeError("detector exploded")
        return list(self._boxes)


class FakeFilter(object):
    def __init__(self, keep=True):
        self.keep = keep

    def filter_boxes(self, boxes):
        return list(boxes) if self.keep else []


class _Frames(object):
    """A stand-in for an image: score_frames only passes it through."""

    def __init__(self, n=1):
        self.n = n


class TestScoring(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        # frame count is what matters here; the fake detector ignores content
        self.frames = [np.zeros((4, 4, 3), np.uint8) for _ in range(4)]

    def _patch_decode(self, payload=None, seq=None):
        import decoder
        self._orig = decoder.decode_frame
        box = {"n": 0}

        def fake_decode(frame, bbox=None, margin=0.35):
            if seq is not None:
                i = box["n"]
                box["n"] += 1
                return seq[i] if i < len(seq) else None
            return payload
        decoder.decode_frame = fake_decode
        self.addCleanup(lambda: setattr(decoder, "decode_frame", self._orig))

    def _patch_tiles(self, det):
        import detector
        self._orig_t = detector.detect_tiles
        detector.detect_tiles = lambda d, f, *a, **k: d.detect(f)
        self.addCleanup(lambda: setattr(detector, "detect_tiles", self._orig_t))

    def test_all_frames_detected_and_decoded(self):
        det = FakeDetector([FakeBox(10, 10, 40, 40, 0.8)])
        self._patch_tiles(det)
        self._patch_decode("MISSION-QR-001")
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, "MISSION-QR-001", True)
        self.assertEqual(r["frames"], 4)
        self.assertEqual(r["detected_any"], 4)
        self.assertEqual(r["detected_filtered"], 4)
        self.assertEqual(r["decoded"], 4)
        self.assertEqual(r["payload_matched"], 4)
        self.assertEqual(r["decode_rate"], 1.0)
        self.assertEqual(r["median_box_w_px"], 40)
        self.assertAlmostEqual(r["mean_conf"], 0.8, places=4)

    def test_filter_rejection_is_counted_separately_from_raw(self):
        det = FakeDetector([FakeBox(10, 10, 40, 40, 0.8)])
        self._patch_tiles(det)
        self._patch_decode(None)
        r = QAB.score_frames(self.frames, det, FakeFilter(keep=False), {}, None, True)
        # raw detection still happened - only the filter gate rejected it
        self.assertEqual(r["detected_any"], 4)
        self.assertEqual(r["detected_filtered"], 0)
        self.assertEqual(r["decoded"], 0)
        self.assertIsNone(r["mean_conf"])
        self.assertIsNone(r["median_box_w_px"])

    def test_no_boxes_at_all_is_not_an_error(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        self._patch_decode("X")
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, None, True)
        self.assertEqual(r["detected_any"], 0)
        self.assertEqual(r["failures"], [])
        # full-frame decode fallback runs when no box survives
        self.assertEqual(r["decoded"], 4)

    def test_payload_counted_only_when_it_matches(self):
        det = FakeDetector([FakeBox(0, 0, 40, 40, 0.5)])
        self._patch_tiles(det)
        self._patch_decode("SOMETHING-ELSE")
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, "MISSION-QR-001", True)
        self.assertEqual(r["decoded"], 4)
        self.assertEqual(r["payload_matched"], 0)
        self.assertEqual(r["payload_match_rate"], 0.0)
        self.assertIn("SOMETHING-ELSE", r["payloads_seen"])

    def test_payload_comparison_ignores_surrounding_whitespace(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        self._patch_decode("  MISSION-QR-001\n")
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, "MISSION-QR-001", True)
        self.assertEqual(r["payload_matched"], 4)

    def test_payload_match_rate_absent_when_no_expected_payload(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        self._patch_decode("ANY")
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, None, True)
        self.assertIsNone(r["payload_matched"])
        self.assertIsNone(r["payload_match_rate"])

    def test_detector_exception_is_captured_not_raised(self):
        det = FakeDetector(raise_on="detect")
        self._patch_tiles(det)
        self._patch_decode(None)
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, None, True)
        self.assertEqual(r["frames"], 4)
        self.assertEqual(r["detected_any"], 0)
        self.assertTrue(r["failures"])
        self.assertIn("detector exploded", r["failures"][0])

    def test_empty_frame_list_gives_none_rates_not_a_crash(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        r = QAB.score_frames([], det, FakeFilter(), {}, None, True)
        self.assertEqual(r["frames"], 0)
        self.assertIsNone(r["decode_rate"])
        self.assertIsNone(r["detected_filtered_rate"])

    def test_none_frames_are_skipped(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        self._patch_decode(None)
        r = QAB.score_frames([None, None], det, FakeFilter(), {}, None, True)
        self.assertEqual(r["frames"], 0)

    def test_timing_is_recorded(self):
        det = FakeDetector([])
        self._patch_tiles(det)
        self._patch_decode(None)
        r = QAB.score_frames(self.frames, det, FakeFilter(), {}, None, True)
        self.assertIsNotNone(r["detect_ms_mean"])
        self.assertIsNotNone(r["decode_ms_mean"])
        self.assertIsNotNone(r["detect_ms_p95"])


class TestP95(unittest.TestCase):
    def test_p95_of_empty_is_none(self):
        self.assertIsNone(QAB._p95([]))

    def test_p95_picks_a_real_sample(self):
        vals = [float(i) for i in range(100)]
        self.assertEqual(QAB._p95(vals), 95.0)

    def test_p95_short_list_does_not_index_out_of_range(self):
        self.assertEqual(QAB._p95([1.0]), 1.0)
        self.assertEqual(QAB._p95([1.0, 2.0]), 2.0)


class TestVerdicts(unittest.TestCase):
    def _row(self, det=1.0, dec=1.0, frames=10, matched=None):
        return {"frames": frames, "detected_filtered_rate": det,
                "decode_rate": dec, "decoded": int(dec * frames),
                "payload_matched": matched}

    def test_detect_and_decode(self):
        self.assertEqual(QAB.evaluate(self._row(), None), "DETECT + DECODE")

    def test_detect_only_when_decoding_fails(self):
        self.assertEqual(QAB.evaluate(self._row(dec=0.0), None), "DETECT only")

    def test_weak_detect_band(self):
        self.assertEqual(QAB.evaluate(self._row(det=0.4, dec=0.0), None), "weak detect")

    def test_miss(self):
        self.assertEqual(QAB.evaluate(self._row(det=0.0, dec=0.0), None), "MISS")

    def test_no_frames(self):
        self.assertEqual(QAB.evaluate({"frames": 0}, None), "NO FRAMES")

    def test_wrong_payload_is_its_own_verdict(self):
        # decoded something, but not what we expected - a distinct failure
        v = QAB.evaluate(self._row(matched=0), "MISSION-QR-001")
        self.assertEqual(v, "DECODE != --payload")

    def test_matching_payload_keeps_the_normal_verdict(self):
        v = QAB.evaluate(self._row(matched=10), "MISSION-QR-001")
        self.assertEqual(v, "DETECT + DECODE")

    def test_exactly_at_the_thresholds_passes(self):
        # boundaries are inclusive - pin them so a refactor cannot flip them
        self.assertEqual(QAB.evaluate(self._row(det=0.8, dec=0.5), None), "DETECT + DECODE")

    def test_just_below_the_decode_threshold_is_detect_only(self):
        self.assertEqual(QAB.evaluate(self._row(det=0.8, dec=0.49), None), "DETECT only")


class TestPayloadDiagnostics(unittest.TestCase):
    def test_payloads_seen_counts_each_text(self):
        det = FakeDetector([])
        import numpy as np
        import detector
        import decoder
        orig_t, orig_d = detector.detect_tiles, decoder.decode_frame
        detector.detect_tiles = lambda d, f, *a, **k: []
        seq = ["A", "B", "A", None]
        it = iter(seq)
        decoder.decode_frame = lambda *a, **k: next(it, None)
        try:
            frames = [np.zeros((4, 4, 3), np.uint8) for _ in range(4)]
            r = QAB.score_frames(frames, det, FakeFilter(), {}, None, True)
        finally:
            detector.detect_tiles, decoder.decode_frame = orig_t, orig_d
        self.assertEqual(r["payloads_seen"], {"A": 2, "B": 1})


class TestReportWriters(unittest.TestCase):
    def _row(self):
        return {"angle": "0", "frames": 5, "detected_any_rate": 0.6,
                "detected_filtered_rate": 0.6, "decode_rate": 0.6,
                "payload_match_rate": 0.6, "mean_conf": 0.63,
                "median_box_w_px": 44, "verdict": "DETECT + DECODE",
                "payloads_seen": {"MISSION-QR-001": 3}, "failures": []}

    def test_json_and_md_are_written_and_parse(self):
        with tempfile.TemporaryDirectory() as d:
            jp, mp = os.path.join(d, "a.json"), os.path.join(d, "a.md")
            with redirect_stdout(io.StringIO()):
                QAB.write_json_md([self._row()], {"stage": "unit-test"}, jp, mp)
            with open(jp) as fh:
                data = json.load(fh)
            self.assertEqual(data["meta"]["stage"], "unit-test")
            self.assertEqual(data["angles"][0]["angle"], "0")
            self.assertEqual(data["angles"][0]["median_box_w_px"], 44)
            with open(mp) as fh:
                md = fh.read()
            self.assertIn("| 0 | 5 |", md)
            self.assertIn("DETECT + DECODE", md)
            self.assertIn("self-verifying", md)

    def test_table_prints_payload_diagnostics(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            QAB.print_table([self._row()], "MISSION-QR-001", 10.0, None)
        out = buf.getvalue()
        self.assertIn("payloads seen at 0 deg", out)
        self.assertIn("MISSION-QR-001 x3", out)

    def test_table_flags_errors(self):
        row = self._row()
        row["failures"] = ["detect: RuntimeError('boom')"]
        buf = io.StringIO()
        with redirect_stdout(buf):
            QAB.print_table([row], None, 10.0, None)
        self.assertIn("!!", buf.getvalue())

    def test_table_warns_that_the_fov_is_not_measured(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            QAB.print_table([self._row()], None, 10.0,
                            {"qr_px": 39, "px_per_module": 1.19})
        out = buf.getvalue()
        self.assertIn("VENDOR-QUOTED", out)
        self.assertIn("NOT MEASURED", out)


class TestArgSurface(unittest.TestCase):
    def test_help_exits_zero_without_a_camera(self):
        """--help must work on a bare host with no cv2/numpy/picamera2."""
        argv, out = sys.argv, io.StringIO()
        sys.argv = ["qr_angle_bench.py", "--help"]
        try:
            with redirect_stdout(out):
                with self.assertRaises(SystemExit) as cm:
                    QAB.main()
            self.assertEqual(cm.exception.code, 0)
        finally:
            sys.argv = argv
        self.assertIn("--angles", out.getvalue())
        self.assertIn("--per-angle", out.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
