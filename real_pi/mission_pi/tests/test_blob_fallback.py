"""Tests for blob_fallback - the last-resort QR hypothesis generator.

This module had NO test file, and the gap mattered: measuring it showed the
contour-only path finds a real QR panel in 3 of 12 synthetic cases (and none at
all at 1.6 px/module, the regime the fallback exists for) while naming a BLANK
white square as a candidate. See TestDiscrimination for the numbers.

What is deliberately NOT here:
  * A hand-rolled 1:1:3:1:1 finder-pattern scanner. It was prototyped, measured
    (11/12 recall but 66/72 decoy false positives - it fired on blank squares,
    checkers, noise and stripes alike) and REJECTED. ZXing's real implementation
    is ~700 lines of cross-checking; a 60-line imitation is worse than useless.
  * Any re-weighting of the existing score. Nobody has measured the weights.
  * A score floor for detector boxes: measured to change 0 of 5 scores - dead.
Instead the QR-specific work is delegated to OpenCV's quirc-based
QRCodeDetector, which is the established solution and measured strictly better.

All panels here are SYNTHETIC (rendered on textured ground). They are not
flight evidence and not camera evidence.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import blob_fallback as bf

CV2 = bf._HAVE_CV2
needs_cv2 = unittest.skipUnless(CV2, "opencv not installed")

try:
    import cv2
    import qrcode
    HAVE_RENDER = True
except Exception:
    HAVE_RENDER = False
needs_render = unittest.skipUnless(CV2 and HAVE_RENDER, "cv2 + qrcode needed")

H, W = 760, 1014


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def ground_soil(h=H, w=W, seed=11):
    rng = np.random.default_rng(seed)
    small = rng.integers(60, 110, size=(h // 16 + 1, w // 16 + 1), dtype=np.uint8)
    g = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC).astype(np.float32)
    g += rng.normal(0, 6, size=(h, w)).astype(np.float32)
    return np.clip(g, 0, 255).astype(np.uint8)


def ground_grass(h=H, w=W, seed=11):
    rng = np.random.default_rng(seed)
    small = rng.integers(90, 150, size=(h // 12 + 1, w // 12 + 1), dtype=np.uint8)
    g = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC).astype(np.float32)
    g += rng.normal(0, 9, size=(h, w)).astype(np.float32)
    g -= 18 * np.abs(np.sin(np.arange(w)[None, :] / 3.0))
    return np.clip(g, 0, 255).astype(np.uint8)


def qr_panel(version=4, payload="MISSION-QR-001"):
    """A REAL QR: dark modules are BLACK. (qrcode marks dark with True, so the
    first version of this rig rendered an INVERTED panel - a white-on-black
    pattern that is not a QR. Always assert the dark fraction.)"""
    q = qrcode.QRCode(version=version,
                      error_correction=qrcode.constants.ERROR_CORRECT_M,
                      box_size=1, border=4)
    q.add_data(payload)
    q.make()
    m = np.array(q.get_matrix(), dtype=np.uint8)
    return (1 - m) * 255, m.shape[0]


def decoy(kind, n, rng):
    if kind == "plain":                       # blank white square
        return np.full((n, n), 255, np.uint8)
    if kind == "checker":
        return ((np.indices((n, n)).sum(0) // max(1, n // 8) % 2) * 255).astype(np.uint8)
    if kind == "noise":
        return (rng.integers(0, 2, size=(n, n), dtype=np.uint8) * 255).astype(np.uint8)
    if kind == "stripes":
        s = np.zeros((n, n), np.uint8)
        s[::2] = 255
        return s
    if kind == "textish":
        s = np.full((n, n), 255, np.uint8)
        for _ in range(2 * n):
            r = int(rng.integers(1, n - 1))
            c = int(rng.integers(0, n // 2))
            s[r, c:c + int(rng.integers(1, n // 3))] = 0
        return s
    if kind == "bullseye":                    # ONE finder-pattern shape, no QR
        s = np.zeros((n, n), np.uint8)
        for i in range(0, n, 2):
            s[i:i + 1, :] = 255
            s[:, i:i + 1] = 255
        return s
    raise KeyError(kind)


DECOYS = ("plain", "checker", "noise", "stripes", "textish", "bullseye")


def place(panel, g, size):
    p = cv2.resize(panel, (size, size), interpolation=cv2.INTER_AREA)
    out = g.copy()
    y0, x0 = H // 2 - size // 2, W // 2 - size // 2
    out[y0:y0 + size, x0:x0 + size] = p
    return out, (x0, y0, size, size)


def hits(cands, truth, frac=0.4):
    """True when some candidate covers >=40% of the truth box.

    Deliberately NOT IoU: every recall/false-positive number quoted in this
    file was measured with coverage-over-truth, and an earlier version of this
    helper used IoU (stricter) which made the pinned numbers disagree with the
    measurement they came from.
    """
    x0, y0, w0, h0 = truth
    for c in cands:
        ix = max(0, min(c[0] + c[2], x0 + w0) - max(c[0], x0))
        iy = max(0, min(c[1] + c[3], y0 + h0) - max(c[1], y0))
        if ix * iy > frac * w0 * h0:
            return True
    return False


def ctx_of(img):
    return {"alt_m": 10.0, "hfov_deg": 75.0, "img_w": img.shape[1],
            "img_h": img.shape[0]}


def frames_for(ppm, ground_fn=ground_soil, seed=11):
    """{name: (frame, truth)} for the QR and every decoy at one scale."""
    rng = np.random.default_rng(seed)
    g = ground_fn(seed=seed)
    qr, nmod = qr_panel(4)
    size = int(round(nmod * ppm))
    out = {"REAL_QR": place(qr, g, size)}
    for d in DECOYS:
        out[d] = place(decoy(d, nmod, rng), g, size)
    return out, size


# --------------------------------------------------------------------------
# the pure-python helpers (no cv2 needed) - what the search policy relies on
# --------------------------------------------------------------------------
class TestIou(unittest.TestCase):
    def test_identical_boxes(self):
        self.assertAlmostEqual(bf._iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0)

    def test_disjoint_is_zero(self):
        self.assertEqual(bf._iou((0, 0, 10, 10), (50, 50, 10, 10)), 0.0)

    def test_half_overlap(self):
        # 5x10 intersection, 150 union
        self.assertAlmostEqual(bf._iou((0, 0, 10, 10), (5, 0, 10, 10)), 5 * 10 / 150.0)

    def test_zero_area_never_divides_by_zero(self):
        self.assertEqual(bf._iou((0, 0, 0, 0), (0, 0, 0, 0)), 0.0)

    def test_is_symmetric(self):
        a, b = (3, 4, 20, 8), (10, 6, 20, 8)
        self.assertAlmostEqual(bf._iou(a, b), bf._iou(b, a))

    def test_dedup_threshold_is_the_one_find_candidates_uses(self):
        """find_candidates drops a box when IoU >= 0.5 with a kept one."""
        self.assertGreaterEqual(bf._iou((0, 0, 10, 10), (2, 0, 10, 10)), 0.5)
        self.assertLess(bf._iou((0, 0, 10, 10), (6, 0, 10, 10)), 0.5)


class TestClusterObservations(unittest.TestCase):
    """The min_support gate IS the 'moves inconsistently with the ground'
    abort, so it must not be possible to pass it with scattered noise."""

    def test_stable_sightings_form_one_cluster(self):
        obs = [{"e": 10.0, "n": 5.0, "score": 0.5, "ts": 1},
               {"e": 10.4, "n": 5.3, "score": 0.6, "ts": 2},
               {"e": 9.8, "n": 4.9, "score": 0.7, "ts": 3}]
        cl = bf.cluster_observations(obs, {"cluster_radius_m": 2.0, "min_support": 2})
        self.assertEqual(len(cl), 1)
        self.assertEqual(cl[0]["support"], 3)
        self.assertAlmostEqual(cl[0]["score"], 0.7)      # best member wins

    def test_scattered_noise_never_reaches_min_support(self):
        rng = np.random.default_rng(4)
        obs = [{"e": float(rng.uniform(-50, 50)), "n": float(rng.uniform(-50, 50)),
                "score": 0.5, "ts": i} for i in range(12)]
        cl = bf.cluster_observations(obs, {"cluster_radius_m": 2.0, "min_support": 2})
        self.assertEqual(cl, [], "scattered sightings formed a cluster")

    def test_single_observation_is_below_support(self):
        self.assertEqual(bf.cluster_observations(
            [{"e": 0, "n": 0, "score": 1.0, "ts": 0}], {"min_support": 2}), [])

    def test_min_support_one_keeps_a_lone_sighting(self):
        cl = bf.cluster_observations([{"e": 0, "n": 0, "score": 1.0, "ts": 0}],
                                     {"min_support": 1})
        self.assertEqual(len(cl), 1)

    def test_two_spots_stay_separate(self):
        obs = [{"e": 0, "n": 0, "score": 0.5, "ts": 0},
               {"e": 0.5, "n": 0.5, "score": 0.5, "ts": 1},
               {"e": 30, "n": 30, "score": 0.9, "ts": 2},
               {"e": 30.4, "n": 29.8, "score": 0.4, "ts": 3}]
        cl = bf.cluster_observations(obs, {"cluster_radius_m": 2.0, "min_support": 2})
        self.assertEqual(len(cl), 2)
        self.assertAlmostEqual(cl[0]["score"], 0.9)      # sorted by score

    def test_centroid_is_updated(self):
        obs = [{"e": 0.0, "n": 0.0, "score": 1, "ts": 0},
               {"e": 2.0, "n": 0.0, "score": 1, "ts": 1}]
        cl = bf.cluster_observations(obs, {"cluster_radius_m": 3.0, "min_support": 2})
        self.assertAlmostEqual(cl[0]["e"], 1.0)

    def test_empty_input(self):
        self.assertEqual(bf.cluster_observations([], {}), [])


class TestTrendOk(unittest.TestCase):
    """Keeps only trajectories that IMPROVE with proximity."""

    def test_single_stage_cannot_judge(self):
        self.assertTrue(bf.trend_ok([0.2]))

    def test_empty_cannot_judge(self):
        self.assertTrue(bf.trend_ok([]))

    def test_improving_trajectory_passes(self):
        self.assertTrue(bf.trend_ok([0.30, 0.45, 0.60]))

    def test_flat_trajectory_fails(self):
        self.assertFalse(bf.trend_ok([0.50, 0.50, 0.50]))

    def test_declining_trajectory_fails(self):
        self.assertFalse(bf.trend_ok([0.60, 0.50, 0.40]))

    def test_a_dip_that_recovers_passes(self):
        """Latest must beat entry AND sit within slack of the best so far."""
        # latest 0.67 >= entry 0.30 + 0.03, and >= best 0.70 - 0.05
        self.assertTrue(bf.trend_ok([0.30, 0.70, 0.50, 0.67],
                                    {"trend_gain": 0.03, "trend_slack": 0.05}))

    def test_a_dip_that_does_not_recover_fails(self):
        self.assertFalse(bf.trend_ok([0.30, 0.90, 0.40],
                                     {"trend_gain": 0.03, "trend_slack": 0.05}))

    def test_gain_threshold_is_configurable(self):
        s = [0.50, 0.52]
        self.assertFalse(bf.trend_ok(s, {"trend_gain": 0.05}))
        self.assertTrue(bf.trend_ok(s, {"trend_gain": 0.01}))


class TestSizeScore(unittest.TestCase):
    """Ground-size plausibility: the one term with real geometry behind it."""

    def test_panel_plausible_size_scores_one(self):
        # 0.65 m panel, 10 m alt, 75 deg over 2028 px -> ~0.94 m? use a direct calc
        ctx = {"alt_m": 10.0, "hfov_deg": 75.0, "img_w": 2028.0}
        fw = 2 * 10.0 * np.tan(np.radians(75.0) / 2.0)
        px_for_0_65m = 0.65 * 2028.0 / fw
        self.assertEqual(bf._size_score(px_for_0_65m, ctx), 1.0)

    def test_tiny_blob_scores_zero(self):
        self.assertEqual(bf._size_score(1.0, {"alt_m": 10.0, "hfov_deg": 75.0,
                                              "img_w": 2028.0}), 0.0)

    def test_absurdly_large_scores_zero(self):
        self.assertEqual(bf._size_score(5000.0, {"alt_m": 10.0, "hfov_deg": 75.0,
                                                 "img_w": 2028.0}), 0.0)

    def test_missing_context_is_neutral_not_a_veto(self):
        """0.5, not 0.0: without geometry we must not claim the box is wrong."""
        for ctx in ({}, None, {"alt_m": 0}, {"alt_m": 10, "hfov_deg": 0},
                    {"alt_m": 10, "hfov_deg": 75, "img_w": 0}):
            self.assertEqual(bf._size_score(50.0, ctx), 0.5)

    def test_higher_altitude_accepts_a_smaller_box(self):
        """The same box implies a bigger ground object when further away."""
        near = bf._size_score(12.0, {"alt_m": 3.0, "hfov_deg": 75.0, "img_w": 2028.0})
        far = bf._size_score(12.0, {"alt_m": 15.0, "hfov_deg": 75.0, "img_w": 2028.0})
        self.assertLessEqual(near, far)


class TestQuadAngleScore(unittest.TestCase):
    def test_axis_aligned_square_is_one(self):
        q = np.array([[[0, 0]], [[10, 0]], [[10, 10]], [[0, 10]]], dtype=np.int32)
        self.assertAlmostEqual(bf._quad_angle_score(q), 1.0, places=5)

    def test_rotated_square_still_scores_high(self):
        q = np.array([[[0, 5]], [[5, 0]], [[10, 5]], [[5, 10]]], dtype=np.int32)
        self.assertGreater(bf._quad_angle_score(q), 0.95)

    def test_a_sliver_still_has_right_angles_and_scores_one(self):
        """Not a bug: the angle term measures CORNERS, so a 100x1 rectangle is
        genuinely rectangular. Slenderness is the aspect term's job, which is
        why both terms exist. (An earlier expectation here was simply wrong.)"""
        q = np.array([[[0, 0]], [[100, 0]], [[100, 1]], [[0, 1]]], dtype=np.int32)
        self.assertAlmostEqual(bf._quad_angle_score(q), 1.0, places=5)

    def test_a_sheared_quad_scores_low(self):
        """This is what the term is for: a parallelogram with no right angles."""
        q = np.array([[[0, 0]], [[100, 0]], [[150, 20]], [[50, 20]]], dtype=np.int32)
        self.assertLess(bf._quad_angle_score(q), 0.8)

    def test_garbage_never_raises(self):
        self.assertEqual(bf._quad_angle_score([(0, 0), (1, 1)]), 0.0)


# --------------------------------------------------------------------------
# the detector wrapper
# --------------------------------------------------------------------------
class TestQrDetectorWrapper(unittest.TestCase):
    """The wrapper must be a faithful pass-through: it may only add boxes."""

    def test_kill_switch_returns_nothing(self):
        self.assertEqual(bf._qr_detector_boxes(np.zeros((50, 50), np.uint8),
                                               {"qr_detector": False}), [])

    def test_garbage_frame_never_raises(self):
        for bad in (np.zeros((0, 0), np.uint8), np.zeros((5, 5), np.uint8)):
            self.assertEqual(bf._qr_detector_boxes(bad, {}), [])

    def test_detector_is_cached_not_rebuilt_per_call(self):
        if not CV2:
            self.skipTest("no cv2")
        self.assertIs(bf._qr_detector(), bf._qr_detector())

    def test_shapes_are_4x1x2_quads(self):
        """Whatever the OpenCV return shape, _measure must be able to eat it."""
        if not CV2:
            self.skipTest("no cv2")
        for q in bf._qr_detector_boxes(np.zeros((64, 64), np.uint8), {}):
            self.assertEqual(q.shape, (4, 1, 2))

    def test_a_fake_detector_shape_is_handled(self):
        """OpenCV has returned both (ok, pts) and (ok, pts, straight)."""
        quad = np.array([[[0, 0]], [[9, 0]], [[9, 9]], [[0, 9]]], np.float32)

        class _Two(object):
            def detect(self, _g):
                return True, quad[None, ...]

        class _Three(object):
            def detect(self, _g):
                return True, quad[None, ...], None

        class _Nothing(object):
            def detect(self, _g):
                return False, None

        real = bf._QR_DETECTOR
        try:
            for fake, want in ((_Two(), 1), (_Three(), 1), (_Nothing(), 0)):
                bf._QR_DETECTOR = fake
                got = bf._qr_detector_boxes(np.zeros((64, 64), np.uint8), {})
                self.assertEqual(len(got), want, "shape handling broke for %r"
                                 % (type(fake).__name__,))
                if want:
                    self.assertEqual(got[0].shape, (4, 1, 2))
        finally:
            bf._QR_DETECTOR = real

    def test_a_throwing_detector_never_takes_the_pipeline_down(self):
        class _Boom(object):
            def detect(self, _g):
                raise RuntimeError("nope")

        real = bf._QR_DETECTOR
        try:
            bf._QR_DETECTOR = _Boom()
            self.assertEqual(bf._qr_detector_boxes(np.zeros((64, 64), np.uint8), {}), [])
        finally:
            bf._QR_DETECTOR = real


# --------------------------------------------------------------------------
# interface guarantees the mission depends on
# --------------------------------------------------------------------------
class TestFindCandidatesContract(unittest.TestCase):
    def test_missing_cv2_returns_empty_not_raise(self):
        if CV2:
            self.skipTest("cv2 is installed - the no-cv2 path cannot run here")
        self.assertEqual(bf.find_candidates(np.zeros((10, 10), np.uint8), {}, {}), [])

    @needs_cv2
    def test_none_and_empty_frames_return_empty(self):
        for bad in (None, np.zeros((0, 0, 3), np.uint8)):
            self.assertEqual(bf.find_candidates(bad, {}, {}), [])

    @needs_cv2
    def test_flat_frame_yields_nothing(self):
        """Flat grey has no structure; it must not invent candidates."""
        self.assertEqual(bf.find_candidates(np.full((200, 200), 128, np.uint8), {}, {}), [])

    @needs_cv2
    def test_tuples_of_six_with_a_metrics_dict(self):
        img, _ = place(decoy("plain", 41, np.random.default_rng(1)),
                       ground_soil(seed=1), 100)
        for c in bf.find_candidates(img, {}, ctx_of(img)):
            self.assertEqual(len(c), 6)
            self.assertIsInstance(c[5], dict)
            for key in ("rect", "aspect", "angles", "contrast", "runs", "size",
                        "qr_detector"):
                self.assertIn(key, c[5])

    @needs_cv2
    def test_scores_are_bounded_and_sorted(self):
        img, _ = place(decoy("checker", 41, np.random.default_rng(1)),
                       ground_soil(seed=1), 160)
        cands = bf.find_candidates(img, {}, ctx_of(img))
        scores = [c[4] for c in cands]
        self.assertEqual(scores, sorted(scores, reverse=True))
        for s in scores:
            self.assertGreaterEqual(s, 0.0)
            self.assertLessEqual(s, 1.0)

    @needs_cv2
    def test_max_cands_is_respected(self):
        img = ground_soil(seed=2).copy()
        for i in range(6):
            x = 40 + i * 150
            img[100:200, x:x + 90] = 255
        full = bf.find_candidates(img, {}, ctx_of(img))
        self.assertGreater(len(full), 2, "test frame produced too few boxes")
        self.assertLessEqual(len(bf.find_candidates(img, {"max_cands": 2}, ctx_of(img))), 2)

    @needs_cv2
    def test_no_duplicate_boxes(self):
        img, _ = place(decoy("checker", 41, np.random.default_rng(1)),
                       ground_soil(seed=1), 160)
        cands = bf.find_candidates(img, {}, ctx_of(img))
        for i, a in enumerate(cands):
            for b in cands[i + 1:]:
                self.assertLess(bf._iou(a[:4], b[:4]), 0.5,
                                "two kept boxes overlap: %s %s" % (a[:4], b[:4]))

    @needs_cv2
    def test_the_detector_can_be_switched_off(self):
        img, _ = place(qr_panel(4)[0], ground_soil(seed=11), 66)
        off = bf.find_candidates(img, {"qr_detector": False}, ctx_of(img))
        on = bf.find_candidates(img, {}, ctx_of(img))
        self.assertFalse(any(c[5]["qr_detector"] for c in off),
                         "a detector box appeared with the detector switched off")
        self.assertNotIn(True, [c[5]["qr_detector"] for c in off])


# --------------------------------------------------------------------------
# what the measurements actually showed
# --------------------------------------------------------------------------
@needs_render
class TestDiscrimination(unittest.TestCase):
    """MEASURED on synthetic panels. Not flight, not camera evidence.

    Before this change the contour-only path found the QR in 3 of 12 cases and
    produced 12 decoy false positives out of 72; a BLANK white square is named
    as a candidate from 1.6 px/module up. The numbers below are the parts a
    regression would move, so they are asserted rather than described.
    """

    def test_real_qr_dark_fraction_proves_the_panel_is_not_inverted(self):
        """The rig's own guard: qrcode's matrix marks DARK with True, and
        forgetting that renders a white-on-black pattern that is not a QR."""
        qr, nmod = qr_panel(4)
        self.assertEqual(qr.shape, (nmod, nmod))
        self.assertLess(float((qr == 0).mean()), 0.5,
                        "the panel is mostly black - it is inverted")
        self.assertGreater(float((qr == 0).mean()), 0.1)

    def test_the_module_size_sweep_matches_the_geometry(self):
        """Guards the rig: 41 modules at 1.6 px/module must be a 66 px panel."""
        _frames, size = frames_for(1.6)
        self.assertEqual(size, 66)

    def test_contour_path_alone_is_called_out_by_these_panels(self):
        """Documents the weakness rather than hiding it: with the detector OFF
        the QR is missed at 1.6 px/module on BOTH grounds, while a BLANK white
        square is a candidate. If a future change fixes the contour path, this
        test tells us by failing - which is the point.

        Scope note: this is asserted for 1.6 px/module only, because that is
        what was measured. At 2.5 px/module the outcome depends on the ground -
        found on soil, missed on grass - so the weaker blanket claim an earlier
        version of this test made was wrong.
        """
        for ground in (ground_soil, ground_grass):
            frames, _size = frames_for(1.6, ground)
            img, truth = frames["REAL_QR"]
            off = bf.find_candidates(img, {"qr_detector": False}, ctx_of(img))
            self.assertFalse(hits(off, truth),
                             "the contour path now finds the QR at 1.6 px/module "
                             "on %s - the detector's value must be re-measured"
                             % ground.__name__)
        # The sharpest statement of the flaw: same ground, same scale, same
        # panel size - the contour path misses the REAL QR and accepts a BLANK
        # square. Measured on soil at 1.6; on grass the blank square is not a
        # candidate until 2.5, so that scale is where it is asserted.
        for ground, ppm in ((ground_soil, 1.6), (ground_grass, 2.5)):
            frames, _size = frames_for(ppm, ground)
            blind, blank = frames["plain"]
            off_blank = bf.find_candidates(blind, {"qr_detector": False}, ctx_of(blind))
            self.assertTrue(hits(off_blank, blank),
                            "a blank white square is no longer a candidate on %s "
                            "at %.1f px/module - re-measure the false-positive "
                            "figure" % (ground.__name__, ppm))

    def test_the_qr_detector_finds_the_qr_where_the_contour_path_cannot(self):
        """THE reason this change exists: 1.6 px/module."""
        frames, size = frames_for(1.6)
        img, truth = frames["REAL_QR"]
        off = bf.find_candidates(img, {"qr_detector": False}, ctx_of(img))
        on = bf.find_candidates(img, {}, ctx_of(img))
        self.assertFalse(hits(off, truth))
        self.assertTrue(hits(on, truth),
                        "the QR detector did not rescue the %.1f px/module case" % 1.6)
        flagged = [c for c in on if c[5]["qr_detector"]]
        self.assertTrue(flagged, "no candidate was flagged as detector-sourced")

    def test_detector_boxes_score_on_the_same_scale(self):
        """No floor, no bonus: a detector box's score is its measured score.
        Measured 0.956-0.972 over five panels, so the floor once tried there
        changed nothing and was removed."""
        frames, _size = frames_for(2.5)
        img, truth = frames["REAL_QR"]
        flagged = [c for c in bf.find_candidates(img, {}, ctx_of(img))
                   if c[5]["qr_detector"]]
        self.assertTrue(flagged)
        for c in flagged:
            self.assertLessEqual(c[4], 1.0)
            self.assertGreater(c[4], 0.9,
                               "detector box scored %.3f - re-measure the claim "
                               "that detector boxes already score high" % c[4])

    def test_the_detector_adds_no_false_positives_on_these_decoys(self):
        """1 false positive in 72 decoy cases was measured across the full
        sweep; this pins the two decoy types that the CONTOUR path already
        mis-fires on, to show the detector is not adding to them."""
        frames, _size = frames_for(2.5)
        for name in ("noise", "stripes", "textish"):
            img, truth = frames[name]
            on = bf.find_candidates(img, {}, ctx_of(img))
            flagged = [c for c in on if c[5]["qr_detector"]]
            self.assertEqual(flagged, [],
                             "the QR detector fired on the %s decoy" % name)

    def test_a_blank_square_is_never_detector_flagged(self):
        frames, _size = frames_for(2.5)
        img, _truth = frames["plain"]
        on = bf.find_candidates(img, {}, ctx_of(img))
        self.assertEqual([c for c in on if c[5]["qr_detector"]], [],
                         "a blank white square was reported as a QR hypothesis")

    def test_recall_is_at_least_what_was_measured(self):
        """3/12 contour-only -> 5/12 with the detector, over both grounds and
        six scales. Asserted as a floor so a regression trips it."""
        found_on = found_off = 0
        cases = 0
        for ground in (ground_soil, ground_grass):
            for ppm in (0.8, 1.0, 1.2, 1.6, 2.5, 4.0):
                frames, _size = frames_for(ppm, ground)
                img, truth = frames["REAL_QR"]
                cases += 1
                if hits(bf.find_candidates(img, {}, ctx_of(img)), truth):
                    found_on += 1
                if hits(bf.find_candidates(img, {"qr_detector": False}, ctx_of(img)), truth):
                    found_off += 1
        self.assertGreaterEqual(found_on, 5, "recall dropped: %d/%d" % (found_on, cases))
        self.assertGreater(found_on, found_off, "the detector added no recall")
        self.assertLessEqual(found_off, 3, "contour-only recall changed: %d" % found_off)

    def test_below_one_pixel_per_module_nothing_can_see_it(self):
        """0.8 and 1.0 px/module: the finder pattern is under 7 px across, and
        BOTH paths miss. This is the measured floor for the fallback, and it is
        why the 15 m range figure is labelled detect-only."""
        for ppm in (0.8, 1.0):
            for ground in (ground_soil, ground_grass):
                frames, _size = frames_for(ppm, ground)
                img, truth = frames["REAL_QR"]
                cands = bf.find_candidates(img, {}, ctx_of(img))
                self.assertFalse(hits(cands, truth),
                                 "%.1f px/module was detected - the documented "
                                 "resolution floor needs re-measuring" % ppm)


# --------------------------------------------------------------------------
# mission-side integration seams (no flight logic touched)
# --------------------------------------------------------------------------
class TestMissionSeam(unittest.TestCase):
    def test_mission_unpacks_six_values(self):
        """mission.py: `for (x, y, w, h, score, _m) in find_candidates(...)`"""
        import inspect
        import mission
        src = inspect.getsource(mission.Mission._blob_observe)
        self.assertIn("for (x, y, w, h, score, _m) in find_candidates(", src)

    def test_mission_still_only_uses_boxes_and_scores(self):
        """The module's contract: it steers, it never decides or transmits."""
        import inspect
        import mission
        for name in ("_blob_observe", "_blob_stage_score"):
            src = inspect.getsource(getattr(mission.Mission, name))
            for forbidden in ("decode_frame", "emit(", "send("):
                self.assertNotIn(forbidden, src,
                                 "%s reached into %r - blob_fallback must only "
                                 "generate hypotheses" % (name, forbidden))

    def test_the_module_never_decides_anything(self):
        """No API here may return a payload: that is the decoder's job.

        Checked as CALLS, not substrings - the module's docstring legitimately
        talks about "a decoder payload", and an earlier version of this test
        failed on its own documentation.
        """
        import inspect
        src = inspect.getsource(bf)
        for call in ("detectAndDecode", ".decode(", "decode_frame",
                     "QRCodeDetector().decode"):
            self.assertNotIn(call, src,
                             "blob_fallback must not decode (found %r)" % call)


if __name__ == "__main__":
    unittest.main(verbosity=2)
