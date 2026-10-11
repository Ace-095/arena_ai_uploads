"""Benchmark scoring tests — the measurement is code, so it gets tested too.

A benchmark that silently mis-scores is worse than no benchmark: it turns a
working detector into a "regression" and sends someone to retrain a model that
was fine. The two rules that matter here:

  * **strict IoU>=0.5** — standard, comparable with anything else;
  * **coverage >= 80% of the target** — the operational rule. A detector that
    boxes the whole panel AROUND the code scores IoU ~0.4 and would be counted
    as a miss AND a false positive, while the decoder (which only needs the
    code inside the ROI) succeeds. Both are reported; neither is hidden.

Latency/throughput numbers are host-dependent, so nothing here asserts a
performance figure — it asserts the ARITHMETIC that turns raw frames into
those figures, which is the part that must not drift.
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.perception_bench import (match_boxes, _iou, _coverage, pct,   # noqa: E402
                                    verdict, is_flight_host, PROPOSED,
                                    HOST_DEPENDENT, make_corpus, read_labels)

try:
    import cv2
    import numpy as np
    _HAVE_CV2 = True
except Exception:                       # pragma: no cover
    _HAVE_CV2 = False


class TestMatching(unittest.TestCase):
    def test_perfect_match(self):
        self.assertEqual(match_boxes([(0, 0, 10, 10)], [(0, 0, 10, 10)]),
                         (1, 0, 0))

    def test_offset_box_is_a_miss_and_a_false_positive(self):
        self.assertEqual(match_boxes([(500, 500, 10, 10)], [(0, 0, 10, 10)]),
                         (0, 1, 1))

    def test_no_prediction_is_a_pure_miss(self):
        self.assertEqual(match_boxes([], [(0, 0, 10, 10)]), (0, 0, 1))

    def test_no_ground_truth_but_a_box_is_a_false_positive(self):
        self.assertEqual(match_boxes([(0, 0, 10, 10)], []), (0, 1, 0))

    def test_matching_is_one_to_one(self):
        gt = [(0, 0, 10, 10)]
        pred = [(0, 0, 10, 10), (1, 1, 10, 10)]     # both overlap the same gt
        self.assertEqual(match_boxes(pred, gt), (1, 1, 0))

    def test_loose_but_correct_box_passes_the_coverage_rule(self):
        """The exact field case: a 39 px code inside a 59 px box (IoU 0.43)."""
        gt = [(1000, 734, 39, 39)]
        pred = [(988, 726, 61, 61)]
        self.assertEqual(match_boxes(pred, gt), (0, 1, 1),
                         "strict IoU calls this a miss + false positive")
        self.assertEqual(match_boxes(pred, gt, cover_thr=0.80), (1, 0, 0),
                         "the code IS inside the box — the decoder will find it")

    def test_coverage_rule_still_rejects_a_genuinely_missed_target(self):
        gt = [(1000, 734, 39, 39)]
        self.assertEqual(match_boxes([(1200, 900, 39, 39)], gt, cover_thr=0.80),
                         (0, 1, 1))

    def test_a_box_covering_only_part_of_the_target_is_not_a_hit(self):
        gt = [(0, 0, 100, 100)]
        # 40% of the area covered, IoU 0.4 -> fails both rules
        self.assertEqual(match_boxes([(0, 0, 40, 100)], gt, cover_thr=0.80),
                         (0, 1, 1))

    def test_iou_and_coverage_helpers(self):
        self.assertAlmostEqual(_iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0)
        self.assertAlmostEqual(_coverage((0, 0, 20, 20), (0, 0, 10, 10)), 1.0)
        self.assertAlmostEqual(_coverage((5, 0, 10, 10), (0, 0, 10, 10)), 0.5)


class TestPercentile(unittest.TestCase):
    def test_p95_of_a_sorted_run(self):
        self.assertEqual(pct(list(range(1, 101)), 0.95), 95)

    def test_empty_is_none_not_zero(self):
        self.assertIsNone(pct([], 0.95))

    def test_single_value(self):
        self.assertEqual(pct([7], 0.5), 7)


class TestVerdicts(unittest.TestCase):
    def test_pass_and_fail(self):
        self.assertEqual(verdict(0.95, ">=", 0.90), "PASS")
        self.assertEqual(verdict(0.85, ">=", 0.90), "FAIL")
        self.assertEqual(verdict(0.05, "<=", 0.10), "PASS")

    def test_unmeasured_stays_unmeasured(self):
        """Never report an absent measurement as 0 — that reads as a failure
        and hides the difference between 'bad' and 'not run'."""
        self.assertEqual(verdict(None, ">=", 0.90), "NOT MEASURED")
        self.assertEqual(verdict("NO PAYLOAD EVER VALIDATED", ">=", 0.90),
                         "NOT MEASURED")

    def test_every_proposed_criterion_is_well_formed(self):
        for name, op, target, why in PROPOSED:
            self.assertIn(op, (">=", "<="))
            self.assertIsInstance(target, float)
            self.assertTrue(why)


class TestFlightHost(unittest.TestCase):
    def test_a_laptop_is_not_the_flight_host(self):
        self.assertFalse(is_flight_host({"hailo": "NOT MEASURED",
                                         "platform": "Linux x86_64"}))

    def test_arm_with_a_hailo_device_is(self):
        import platform as _p
        if "arm" in _p.machine().lower() or "aarch64" in _p.machine().lower():
            self.assertTrue(is_flight_host({"hailo": "Hailo-8 ..."}))
        else:
            # on x86 the arch test dominates regardless of the hailo string
            self.assertFalse(is_flight_host({"hailo": "Hailo-8 ..."}))

    def test_fps_is_declared_host_dependent(self):
        self.assertIn("pipeline_fps", HOST_DEPENDENT,
                      "scoring the Pi's FPS budget on a laptop would be a "
                      "false alarm; it must be flagged, not silently scored")


@unittest.skipUnless(_HAVE_CV2, "opencv/numpy not installed")
class TestCorpusRoundTrip(unittest.TestCase):
    def test_make_then_read(self):
        with tempfile.TemporaryDirectory() as d:
            make_corpus(d, [10.0], ["MISSION-QR-001"], size=(640, 480))
            rows = read_labels(d)
            self.assertTrue(rows)
            self.assertTrue(all(r["synthetic"] for r in rows),
                            "the corpus must declare itself synthetic")
            self.assertTrue(any(r["payload"] for r in rows),
                            "at least one frame must carry a labelled payload")
            self.assertTrue(any(not r["boxes"] for r in rows),
                            "the lead-in frames must have no ground truth, "
                            "otherwise time-to-payload is trivially zero")
            for r in rows:
                self.assertTrue(os.path.isfile(os.path.join(d, r["frame"])))

    def test_geometry_drives_the_panel_size(self):
        """The panel's pixel size must come from the configured FOV, not from
        a number someone typed — otherwise the corpus silently stops
        representing the real optics."""
        with tempfile.TemporaryDirectory() as d:
            make_corpus(d, [15.0], ["MISSION-QR-001"], size=(2028, 1520))
            rows = [r for r in read_labels(d) if r["boxes"]]
            self.assertTrue(rows)
            side = rows[0]["boxes"][0][2]
            self.assertGreater(side, 15)
            self.assertLess(side, 60,
                            "a 297 mm symbol at 15 m through the configured "
                            "75 deg lens is tens of pixels, not hundreds")


if __name__ == "__main__":
    unittest.main(verbosity=2)
