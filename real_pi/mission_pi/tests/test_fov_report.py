"""fov_report.py — the geometry report, including the front-camera role.

Run directly:   python3 tests/test_fov_report.py

Everything here is pure arithmetic over a config dict (`compute()` does no I/O),
so these run anywhere. The numbers being pinned are the ones the field decisions
rest on: px/module per altitude, which camera a row is about, and what the
detector NETWORK actually receives after letterboxing.
"""
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_TOOLS = os.path.join(_ROOT, "tools")
for _p in (_ROOT, _TOOLS, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fov_report as FR                                              # noqa: E402


#: The two real roles from mission_pi/config.yaml (vendor FOVs).
CFG = {
    "detector": {"input_size": 640, "tile_bottom": True, "tile_front": False,
                 "tile_grid": [3, 3]},
    "cameras": {
        "front": {"kind": "auto", "match_name": "imx708", "size": [4608, 2592],
                  "hfov_deg": 66.0, "vfov_deg": 41.0},
        "bottom": {"kind": "auto", "match_name": "imx477", "size": [2028, 1520],
                   "hfov_deg": 75.0, "vfov_deg": 60.0},
    },
}


def row(alt, role="bottom", **kw):
    kw.setdefault("speed_ms", 2.0)
    kw.setdefault("exposure_s", 0.002)
    return FR.compute(CFG, alt, 297.0, 25, 4, 0.0, None, cam_role=role, **kw)


class TestRoleSelection(unittest.TestCase):
    def test_bottom_is_the_default_role(self):
        """Without cam_role, compute() must keep answering for the bottom cam."""
        default = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        explicit = row(15.0, "bottom")
        self.assertEqual(default["px_per_module"], explicit["px_per_module"])
        self.assertEqual(default["qr_px"], explicit["qr_px"])

    def test_front_uses_the_front_sensor_size_and_fov(self):
        b, f = row(15.0, "bottom"), row(15.0, "front")
        # 4608 px at 66 deg beats 2028 px at 75 deg - more px/module
        self.assertGreater(f["px_per_module"], b["px_per_module"])

    def test_unknown_role_falls_back_to_bottom(self):
        weird = row(15.0, "rearcamera")
        self.assertEqual(weird["px_per_module"], row(15.0, "bottom")["px_per_module"])

    def test_pinned_px_per_module_values(self):
        """Pin the exact numbers quoted in the docs (vendor FOV, not measured)."""
        self.assertEqual(row(10.0, "bottom")["px_per_module"], 1.19)
        self.assertEqual(row(15.0, "bottom")["px_per_module"], 0.79)
        self.assertEqual(row(15.0, "front")["px_per_module"], 2.13)

    def test_defaults_when_a_role_is_missing_from_config(self):
        """A broken/absent config must still not silently use the other sensor.

        Footprint depends only on ALTITUDE and HFOV, not sensor width - both
        roles fall back to the 75 deg default hfov here. What must differ is the
        resolution-driven half of the math: px_per_m is img_w / footprint.
        """
        f = FR.compute({"cameras": {}}, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002,
                       cam_role="front")
        b = FR.compute({"cameras": {}}, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        self.assertEqual(f["footprint_m"][0], b["footprint_m"][0])   # same hfov default
        self.assertAlmostEqual(f["px_per_m"] / b["px_per_m"], 4608.0 / 2028.0,
                               places=2)
        self.assertGreater(f["px_per_module"], b["px_per_module"])


class TestNetworkInput(unittest.TestCase):
    """The frame-pixel verdict is optimistic; these are the deciding numbers."""

    def test_tiling_is_what_makes_bottom_work_at_15m(self):
        r = row(15.0, "bottom")
        self.assertFalse(r["detectable_at_input"])       # 8.3 px - the network
        self.assertTrue(r["detectable_at_input_tiled"])  # 24.8 px - with 3x3

    def test_tiling_is_needed_for_front_too_despite_12mp(self):
        """A 4608 px frame is shrunk 7.2x, so MORE pixels did not save it."""
        r = row(15.0, "front")
        self.assertFalse(r["detectable_at_input"])
        self.assertTrue(r["detectable_at_input_tiled"])

    def test_input_math_is_a_letterbox_shrink(self):
        r = row(15.0, "front")
        # 70.2 px * 640/4608 = 9.75
        self.assertAlmostEqual(r["net_input_px"], round(r["qr_px"] * 640.0 / 4608, 1),
                               places=1)

    def test_a_frame_narrower_than_the_input_is_not_upscaled(self):
        """min(1.0, ...) - a small frame must not be scored as magnified."""
        cfg = {"cameras": {"bottom": {"size": [320, 240], "hfov_deg": 75.0,
                                      "vfov_deg": 60.0}}}
        r = FR.compute(cfg, 5.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        self.assertAlmostEqual(r["net_input_px"], r["qr_px"], places=1)

    def test_tile_columns_change_the_tiled_number(self):
        one = row(15.0, "bottom", net_tile_cols=1)
        three = row(15.0, "bottom", net_tile_cols=3)
        self.assertLess(one["net_input_px_tiled"], three["net_input_px_tiled"])


class TestVerdictThresholds(unittest.TestCase):
    def test_decode_floor_matches_the_repos_own_bench(self):
        # the corpus decoded a 29-ish module symbol at ~1.3 px/module
        self.assertEqual(FR.PX_PER_MODULE_EMPIRICAL, 1.3)

    def test_decode_target_is_the_doc_heuristic(self):
        self.assertEqual(FR.PX_PER_MODULE_TARGET, 3.5)

    def test_no_decode_verdict_out_at_15m_on_the_bottom_cam(self):
        r = row(15.0, "bottom")
        self.assertFalse(r["decodable_empirical"])
        self.assertFalse(r["decodable_at_target"])

    def test_bottom_cam_clears_the_empirical_floor_at_9m(self):
        self.assertTrue(row(9.0, "bottom")["decodable_empirical"])

    def test_bigger_qr_raises_px_per_module(self):
        small = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        big = FR.compute(CFG, 15.0, 420.0, 25, 4, 0.0, None, 2.0, 0.002)
        self.assertGreater(big["px_per_module"], small["px_per_module"])

    def test_shorter_payload_raises_px_per_module(self):
        v2 = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        v1 = FR.compute(CFG, 15.0, 297.0, 21, 4, 0.0, None, 2.0, 0.002)
        self.assertGreater(v1["px_per_module"], v2["px_per_module"])

    def test_tilt_foreshortens_and_costs_px_per_module(self):
        flat = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 2.0, 0.002)
        tilted = FR.compute(CFG, 15.0, 297.0, 25, 4, 30.0, None, 2.0, 0.002)
        self.assertLess(tilted["px_per_module"], flat["px_per_module"])

    def test_motion_blur_scales_with_speed(self):
        slow = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 1.0, 0.002)
        fast = FR.compute(CFG, 15.0, 297.0, 25, 4, 0.0, None, 3.0, 0.002)
        self.assertLess(slow["motion_blur_px"], fast["motion_blur_px"])


class TestCli(unittest.TestCase):
    def test_cam_choices_are_bottom_and_front(self):
        import inspect
        src = inspect.getsource(FR.main)
        self.assertIn('"--cam"', src)
        self.assertIn('choices=["bottom", "front"]', src)

    def test_net_input_constant_matches_the_detector_default(self):
        # detector.py default input_size is 640; if that moves, this fails loudly
        import detector
        import inspect as _i
        sig = _i.signature(detector.HailoYolo8Detector.__init__)
        self.assertEqual(sig.parameters["input_size"].default, FR.NET_INPUT_PX)


if __name__ == "__main__":
    unittest.main(verbosity=2)
