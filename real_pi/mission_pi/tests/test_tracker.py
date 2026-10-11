"""Temporal tracker tests (stdlib + math only, no numpy/cv2 needed).

What is actually being pinned here, and why each one would cost a flight if it
regressed:

  1. **Association survives fast motion.** A 34 px QR at 2 m/s and 12 fps
     moves ~15 px between frames; plain IoU drops it and the mission sees a
     brand-new object every frame. The motion gate must keep the id.
  2. **Smoothing removes jitter without lagging through a size change.** The
     approach stair (12 -> 9 -> 7 m) triples the box area in seconds.
  3. **Persistence is real.** `min_hits` must gate `confirmed`, and one
     dropped frame must NOT destroy a track.
  4. **Expiry is bounded.** A target that is gone is gone — but only after
     `max_misses`/`max_age_s`, never on the first miss.
  5. **Malformed input never raises.** This runs inside the detection worker.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detector import BBox            # noqa: E402
from tracker import BoxTracker, Track, _iou   # noqa: E402


def b(x, y, w=34, h=34, conf=0.5, src="yolo"):
    return BBox(x, y, w, h, conf, src)


class TestIou(unittest.TestCase):
    def test_identical(self):
        self.assertAlmostEqual(_iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0, places=6)

    def test_disjoint(self):
        self.assertEqual(_iou((0, 0, 10, 10), (50, 50, 10, 10)), 0.0)

    def test_degenerate_is_zero_not_a_crash(self):
        self.assertEqual(_iou((0, 0, 0, 0), (0, 0, 10, 10)), 0.0)


class TestAssociation(unittest.TestCase):
    def test_track_keeps_its_id_across_jitter(self):
        tk = BoxTracker()
        ids = set()
        for i in range(10):
            j = (i % 3) - 1          # +-1 px jitter, the real bench behaviour
            tr = tk.update([b(500 + j, 400 - j)], ts=100.0 + i / 12.0)
            ids.update(t.id for t in tr)
        self.assertEqual(ids, {1}, "jitter must not spawn a new track per frame")
        self.assertEqual(len(tk.tracks), 1)

    def test_fast_motion_keeps_the_id_where_iou_would_fail(self):
        """2 m/s at 15 m / 12 fps: ~20 px between frames, IoU ~0.2 -> below the
        default 0.25 IoU floor. The motion gate must still associate."""
        tk = BoxTracker(iou_thr=0.25)
        first = tk.update([b(400, 400)], ts=10.0)
        self.assertEqual(first[0].hits, 1)
        # second frame 20 px to the right: IoU = 14*34 / (34*34*2-14*34) ~ 0.26
        # -> take a harder case, 26 px, IoU ~ 0.11
        tr = tk.update([b(426, 400)], ts=10.0 + 1 / 12.0)
        self.assertEqual(len(tr), 1, "a fast target must not double-count")
        self.assertEqual(tr[0].hits, 2)
        self.assertLess(_iou((400, 400, 34, 34), (426, 400, 34, 34)), 0.25)

    def test_two_targets_stay_separate(self):
        tk = BoxTracker()
        tk.update([b(100, 100), b(900, 800)], ts=1.0)
        tk.update([b(102, 101), b(898, 801)], ts=1.1)
        self.assertEqual(len(tk.tracks), 2)
        self.assertEqual(sorted(t.id for t in tk.tracks), [1, 2])

    def test_crossing_targets_do_not_swap(self):
        tk = BoxTracker()
        tk.update([b(100, 100), b(500, 100)], ts=1.0)
        # 50 px toward each other in 0.1 s (500 px/s image motion) and still
        # 300 px apart -> no ambiguity, and no duplicate tracks either.
        tr = tk.update([b(150, 100), b(450, 100)], ts=1.1)
        self.assertEqual(len(tr), 2, "a 50 px step must associate, not spawn")
        self.assertEqual(sorted(t.id for t in tr), [1, 2],
                         "the two ids must survive the approach")
        by_id = {t.id: t for t in tr}
        # the box is SMOOTHED, so it sits between the old and new detection —
        # that is the intended behaviour, not a lag to be asserted away
        self.assertTrue(100 < by_id[1].x < 150)
        self.assertTrue(450 < by_id[2].x < 500)

    def test_gate_limit_is_documented_not_unbounded(self):
        """A step larger than the gate DOES start a new track — that is the
        honest limit of centroid association, and worth pinning so nobody
        believes the tracker is magic. 34 px box, default gate 1.1 -> ~53 px."""
        tk = BoxTracker()
        tk.update([b(400, 400)], ts=1.0)
        tr = tk.update([b(400 + 300, 400)], ts=1.1)     # 300 px teleport
        self.assertEqual(len(tr), 2,
                         "beyond the gate a new track is the correct answer")

    def test_velocity_estimate(self):
        tk = BoxTracker(vel_smooth=1.0)
        tk.update([b(100, 100)], ts=0.0)
        tr = tk.update([b(130, 100)], ts=0.1)     # 30 px in 0.1 s
        self.assertAlmostEqual(tr[0].vx, 300.0, delta=40.0)
        self.assertGreater(tr[0].speed_px_s, 200.0)


class TestSmoothing(unittest.TestCase):
    def test_jitter_is_reduced(self):
        tk = BoxTracker(smooth=0.5, min_hits=1)
        xs = []
        for i in range(9):
            j = 3 if i % 2 else -3                  # +-3 px sawtooth
            tr = tk.update([b(500 + j, 400)], ts=100.0 + i / 12.0)
            xs.append(tr[0].x)
        spread_smoothed = max(xs[2:]) - min(xs[2:])
        self.assertLess(spread_smoothed, 3.0,
                        "EMA must damp a +-3 px sawtooth on the box centre")

    def test_scale_change_snaps_instead_of_lagging(self):
        """Approach stair: 34 px at 12 m -> 60 px at 7 m. A plain EMA keeps the
        ROI a frame behind exactly when decode matters."""
        tk = BoxTracker(smooth=0.5, snap_ratio=1.8, min_hits=1)
        tk.update([b(500, 400, 34, 34)], ts=1.0)
        tr = tk.update([b(490, 392, 60, 60)], ts=1.1)
        # area ratio = (60/34)^2 = 3.1 > 1.8 -> width taken at full weight
        self.assertAlmostEqual(tr[0].w, 60.0, delta=1.0)

    def test_small_scale_noise_is_still_smoothed(self):
        tk = BoxTracker(smooth=0.5, snap_ratio=1.8, min_hits=1)
        tk.update([b(500, 400, 34, 34)], ts=1.0)
        tr = tk.update([b(500, 400, 36, 36)], ts=1.1)   # ratio 1.12 < 1.8
        self.assertLess(tr[0].w, 36.0)
        self.assertGreater(tr[0].w, 34.0)


class TestPersistence(unittest.TestCase):
    def test_not_confirmed_before_min_hits(self):
        tk = BoxTracker(min_hits=3)
        self.assertFalse(tk.update([b(10, 10)], ts=1.0)[0].confirmed)
        self.assertFalse(tk.update([b(10, 10)], ts=1.1)[0].confirmed)
        third = tk.update([b(10, 10)], ts=1.2)[0]
        self.assertTrue(third.confirmed)
        self.assertEqual(tk.stats["confirmed"], 1)

    def test_one_dropped_frame_does_not_lose_the_track(self):
        tk = BoxTracker(min_hits=2, max_misses=8)
        tk.update([b(10, 10)], ts=1.0)
        tk.update([b(10, 10)], ts=1.1)
        tk.update([], ts=1.2)                       # the blurry frame
        self.assertEqual(len(tk.tracks), 1)
        self.assertEqual(tk.tracks[0].misses, 1)
        # ...and it recovers its streak on the next good frame
        tr = tk.update([b(10, 10)], ts=1.3)
        self.assertEqual(tr[0].misses, 0)
        self.assertTrue(tr[0].confirmed)

    def test_stale_flag_after_stale_s(self):
        tk = BoxTracker(stale_s=0.4)
        tk.update([b(10, 10)], ts=1.0)
        tk.update([], ts=1.2)
        self.assertFalse(tk.tracks[0].stale)
        tk.update([], ts=1.6)
        self.assertTrue(tk.tracks[0].stale)
        self.assertEqual(tk.confirmed(), [],
                         "a stale track must not be reported as a live confirm")

    def test_expiry_after_max_misses(self):
        tk = BoxTracker(max_misses=2, max_age_s=100.0)
        tk.update([b(10, 10)], ts=1.0)
        tk.update([], ts=1.1)
        tk.update([], ts=1.2)
        tk.update([], ts=1.3)                       # misses = 3 > 2
        self.assertEqual(tk.tracks, [])
        self.assertEqual(tk.stats["dropped"], 1)
        self.assertEqual([e["kind"] for e in tk.events], ["lost"])

    def test_expiry_after_max_age_s(self):
        tk = BoxTracker(max_misses=99, max_age_s=2.0)
        tk.update([b(10, 10)], ts=1.0)
        tk.update([], ts=10.0)
        self.assertEqual(tk.tracks, [], "a 9 s old target is not trackable")

    def test_lost_event_carries_age(self):
        tk = BoxTracker(max_misses=1, max_age_s=100.0)
        tk.update([b(10, 10)], ts=1.0)
        tk.update([b(10, 10)], ts=1.1)
        tk.update([], ts=1.2)
        tk.update([], ts=1.3)
        lost = [e for e in tk.events if e["kind"] == "lost"]
        self.assertEqual(len(lost), 1)
        self.assertIn("age_s", lost[0])


class TestRobustness(unittest.TestCase):
    def test_never_raises_on_junk(self):
        tk = BoxTracker()
        for junk in ([None], ["nope"], [b(0, 0, 0, 0)], None, [object()],
                     [b(-5, -5, 34, 34)]):
            tk.update(junk, ts=1.0)           # must not raise

    def test_all_boxes_broken_is_not_fatal(self):
        tk = BoxTracker()
        tk.update([None, "x"], ts=1.0)
        self.assertEqual(tk.tracks, [])
        self.assertEqual(tk.stats["updates"], 1)

    def test_max_tracks_is_bounded(self):
        tk = BoxTracker(max_tracks=3)
        tk.update([b(10 * i, 10) for i in range(10)], ts=1.0)
        self.assertEqual(len(tk.tracks), 3)

    def test_reset_clears_state(self):
        tk = BoxTracker()
        tk.update([b(10, 10)], ts=1.0)
        tk.reset()
        self.assertEqual(tk.tracks, [])

    def test_stabilize_shape(self):
        tk = BoxTracker()
        out = tk.stabilize([b(10, 10)], ts=1.0)
        self.assertEqual(len(out), 1)
        self.assertEqual(len(out[0]), 6)      # x, y, w, h, conf, source

    def test_track_helpers(self):
        t = Track(1, (100, 200, 34, 34), 0.6, 5.0)
        self.assertEqual(t.int_box, (100, 200, 34, 34))
        self.assertEqual(t.side_px, 34)
        self.assertEqual(t.age_s, 0.0)
        self.assertIn("Track#1", repr(t))


if __name__ == "__main__":
    unittest.main(verbosity=2)
