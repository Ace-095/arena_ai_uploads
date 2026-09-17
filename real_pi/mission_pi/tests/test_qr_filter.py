"""Fake-QR filter + preset tests (stdlib only, no numpy/cv2 needed).

Covers the window-grill regression:
  * grill blobs (conf 0.16/0.24, tall diamond aspect) are REJECTED by the
    filter — before they can draw an overlay or steer the drone;
  * a real 15 m QR (34x34 px, conf ~0.52, square) SURVIVES;
  * require_decode_for_cue mode admits only decoded boxes to the cue path;
  * presets: names/aliases, config normalization, live application onto a
    mission-shaped object (detector thresholds, tile grid, cue strictness).
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detector import BBox          # noqa: E402
from qr_filter import (QrFilter, DEFAULT_FILTER, DEFAULT_PRESETS,  # noqa: E402
                       load_presets, preset_names, apply_preset,
                       normalize_preset_name)


def box(x=100, y=100, w=34, h=34, conf=0.52, src="yolo"):
    return BBox(x, y, w, h, conf, src)


class TestGrillFilter(unittest.TestCase):
    def setUp(self):
        # config.yaml defaults (also the built-in defaults)
        self.qf = QrFilter.from_config({"qr_filter": dict(DEFAULT_FILTER)})

    def test_real_15m_qr_survives(self):
        # 34 px square at 15 m, measured conf 0.52 on the bench
        b = box(w=34, h=34, conf=0.52)
        self.assertTrue(self.qf.should_keep(b)[0])

    def test_grill_low_conf_rejected(self):
        for conf in (0.16, 0.24):
            b = box(w=60, h=60, conf=conf)
            kept, reason = self.qf.should_keep(b)
            self.assertFalse(kept, "conf %.2f must be filtered" % conf)
            self.assertIn("conf", reason)

    def test_grill_tall_diamond_aspect_rejected(self):
        # diamond lattice shard: tall/narrow, conf above floor
        b = box(w=20, h=120, conf=0.5)
        kept, reason = self.qf.should_keep(b)
        self.assertFalse(kept)
        self.assertIn("aspect", reason)

    def test_wide_strip_rejected(self):
        b = box(w=200, h=20, conf=0.5)   # aspect 10
        self.assertFalse(self.qf.should_keep(b)[0])

    def test_square_band_edges(self):
        # aspect 0.4 and 2.5 are INCLUSIVE bounds
        self.assertTrue(self.qf.should_keep(box(w=40, h=100, conf=0.5))[0])   # 0.4
        self.assertTrue(self.qf.should_keep(box(w=100, h=40, conf=0.5))[0])   # 2.5
        self.assertFalse(self.qf.should_keep(box(w=39, h=100, conf=0.5))[0])  # 0.39
        self.assertFalse(self.qf.should_keep(box(w=101, h=40, conf=0.5))[0])  # 2.52

    def test_tiny_speck_rejected(self):
        self.assertFalse(self.qf.should_keep(box(w=10, h=10, conf=0.9))[0])
        self.assertFalse(self.qf.should_keep(box(w=14, h=40, conf=0.9))[0])

    def test_filter_boxes_keeps_only_survivors(self):
        boxes = [
            box(w=34, h=34, conf=0.52),      # keep
            box(w=34, h=34, conf=0.2),       # reject conf
            box(w=20, h=120, conf=0.5),      # reject aspect
            box(w=8, h=8, conf=0.8),         # reject size
            box(w=60, h=25, conf=0.35),      # keep (aspect 2.4, conf at floor)
        ]
        kept = self.qf.filter_boxes(boxes)
        self.assertEqual(len(kept), 2)
        self.assertAlmostEqual(kept[0].conf, 0.52)
        self.assertAlmostEqual(kept[1].conf, 0.35)

    def test_bad_box_never_kept(self):
        class Bad:
            w = h = conf = object()
        self.assertFalse(self.qf.should_keep(Bad())[0])


class TestRequireDecodeCue(unittest.TestCase):
    """The cue-path gate: in strict mode only DECODED boxes may steer."""

    def test_strict_mode_flag(self):
        qf = QrFilter.from_config({"qr_filter": {"require_decode_for_cue": True}})
        self.assertTrue(qf.require_decode_for_cue)
        self.assertFalse(QrFilter().require_decode_for_cue)  # default off

    def test_default_mode_allows_bbox_cue(self):
        # default (flight) preset: a surviving bbox raises a cue pre-decode
        qf = QrFilter.from_config({})
        b = box(w=34, h=34, conf=0.52)
        self.assertTrue(qf.should_keep(b)[0])
        self.assertFalse(qf.require_decode_for_cue)


class _StubDetector:
    """Duck-typed stand-in for the YOLO detector (has set_thresholds)."""
    name = "stub"
    def __init__(self):
        self.conf_thr = 0.25
        self.iou_thr = 0.45
    def set_thresholds(self, conf_thr=None, iou_thr=None):
        if conf_thr is not None:
            self.conf_thr = float(conf_thr)
        if iou_thr is not None:
            self.iou_thr = float(iou_thr)


class _StubCam:
    facing = "bottom"
    name = "cam2"
    def __init__(self):
        self.boosts = []
    def apply_qr_profile(self, p):
        self.boosts.append(p)


class _StubRig:
    def __init__(self):
        self.cams = {"cam2": _StubCam()}


class _StubHub:
    def __init__(self):
        self.events = []
    def push(self, chan, msg):
        self.events.append((chan, msg))


class _StubMission:
    """mission-shaped object with just the attributes apply_preset touches."""
    def __init__(self, cfg=None):
        self.cfg = cfg or {}
        self.detector = _StubDetector()
        self.rig = _StubRig()
        self.hub = _StubHub()
        self.qf = QrFilter.from_config(self.cfg)
        self.tile_rows = 3
        self.tile_cols = 3
        self.full_decode_every = 5
        self.preset_name = "day"
        self.logs = []
    def _log(self, level, msg):
        self.logs.append((level, msg))


class TestPresets(unittest.TestCase):
    def test_builtin_table_shape(self):
        names = list(DEFAULT_PRESETS.keys())
        self.assertEqual(len(names), 7)          # the 1-7 key order
        self.assertEqual(names[:3], ["day", "far", "dark"])
        for p in DEFAULT_PRESETS.values():
            self.assertIn("conf_thr", p)
            self.assertIn("require_decode_for_cue", p)
            self.assertIn("boost", p)
        self.assertTrue(DEFAULT_PRESETS["dark"]["require_decode_for_cue"])
        self.assertFalse(DEFAULT_PRESETS["day"]["require_decode_for_cue"])

    def test_aliases(self):
        self.assertEqual(normalize_preset_name("dark_qr_boost"), "dark")
        self.assertEqual(normalize_preset_name("night"), "dark")
        self.assertEqual(normalize_preset_name("ground"), "kabaddi")
        self.assertEqual(normalize_preset_name("15m"), "far")
        self.assertEqual(normalize_preset_name("Day"), "day")
        self.assertEqual(normalize_preset_name("bogus"), "bogus")

    def test_load_presets_from_config(self):
        cfg = {"qr_presets": {
            "dark": {"conf_thr": 0.28, "tile_grid": [3, 3],
                     "require_decode_for_cue": True, "boost": "qr_boost_lowlight"},
            "half": {"conf_thr": 0.3},   # missing keys -> defaults
        }}
        ps = load_presets(cfg)
        self.assertEqual(sorted(ps), ["dark", "half"])
        self.assertAlmostEqual(ps["dark"]["conf_thr"], 0.28)
        self.assertEqual(ps["half"]["iou_thr"], 0.45)        # default
        self.assertEqual(ps["half"]["tile_grid"], [3, 3])    # default

    def test_load_presets_empty_config_falls_back(self):
        self.assertEqual(list(load_presets({})), list(DEFAULT_PRESETS))
        self.assertEqual(list(load_presets(None)), list(DEFAULT_PRESETS))

    def test_preset_names_order_stable(self):
        cfg = {"qr_presets": {"day": {}, "kabaddi": {}}}
        self.assertEqual(preset_names(cfg), ["day", "kabaddi"])

    def test_apply_preset_muts_mission(self):
        m = _StubMission()
        applied = apply_preset(m, "dark")
        self.assertIsNotNone(applied)
        self.assertEqual(m.preset_name, "dark")
        self.assertAlmostEqual(m.detector.conf_thr, 0.30)
        self.assertTrue(m.qf.require_decode_for_cue)
        self.assertEqual((m.tile_rows, m.tile_cols), (3, 3))
        self.assertEqual(m.full_decode_every, 2)
        self.assertEqual(m.rig.cams["cam2"].boosts, ["qr_boost_lowlight"])

    def test_apply_preset_alias(self):
        m = _StubMission()
        applied = apply_preset(m, "dark_qr_boost")
        self.assertIsNotNone(applied)
        self.assertEqual(m.preset_name, "dark")

    def test_apply_preset_kabaddi_full_frame(self):
        m = _StubMission()
        apply_preset(m, "kabaddi")
        self.assertEqual((m.tile_rows, m.tile_cols), (1, 1))
        self.assertEqual(m.full_decode_every, 1)
        self.assertTrue(m.qf.require_decode_for_cue)

    def test_apply_preset_day_restores_flight_cues(self):
        m = _StubMission()
        apply_preset(m, "dark")
        apply_preset(m, "day")
        self.assertFalse(m.qf.require_decode_for_cue)
        self.assertAlmostEqual(m.detector.conf_thr, 0.35)
        self.assertEqual(m.full_decode_every, 3)

    def test_apply_preset_unknown_returns_none(self):
        m = _StubMission()
        self.assertIsNone(apply_preset(m, "nope"))
        self.assertEqual(m.preset_name, "day")  # untouched

    def test_apply_preset_no_rig_no_crash(self):
        m = _StubMission()
        m.rig = None
        m.detector = None
        self.assertIsNotNone(apply_preset(m, "far"))  # best-effort, no raise


class TestConfigFile(unittest.TestCase):
    """config.yaml must parse and carry the new blocks (real file check)."""

    def test_config_yaml_has_filter_and_presets(self):
        try:
            import yaml
        except Exception:
            self.skipTest("pyyaml not in this interpreter")
        path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "config.yaml")
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.assertIn("qr_filter", cfg)
        self.assertIn("qr_presets", cfg)
        self.assertEqual(cfg["qr_filter"]["min_conf"], 0.35)
        self.assertEqual(cfg["detector"]["preset"], "day")
        self.assertIn("dark", cfg["qr_presets"])
        self.assertTrue(cfg["qr_presets"]["dark"]["require_decode_for_cue"])
        # presets in the file must load without crashing the normalizer
        ps = load_presets(cfg)
        self.assertGreaterEqual(len(ps), 7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
