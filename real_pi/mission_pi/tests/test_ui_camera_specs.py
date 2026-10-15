"""The UI's camera numbers must be the rig that actually flies.

WHY THIS FILE EXISTS
--------------------
`mission-ui/js/geo.js` and `js/map.js` each carry a hardcoded CAM_SPECS table,
and `optimalSpacing()` feeds the lawnmower rows the UI generates and the
coverage rectangles it draws. Both files still described the OLD Waveshare lens
(100 deg HFOV, 4056x3040) long after the mounted camera became an Arducam UC-517
(75 deg HFOV, 2028x1520), and nothing in either test suite compared them.

Measured consequence of that drift, at the 30% overlap the UI defaults to:

  altitude   planned spacing   75 deg lens can cover    error
      10 m          10.74 m                   8.08 m    +32.9%
      15 m          16.11 m                  12.12 m    +32.9%

i.e. every planned pass was 33% too far from the last one: the search grid the
UI produced could not see the ground between rows. That is a perception gap
created by a stale constant, not by anything in the flight code.

These tests are dependency-free (no node, no jsdom) so they run in the normal
`tools/run_tests.sh` sweep on any machine, next to the code they guard. If the
lens is ever changed, change mission_pi/config.yaml and these fail until the UI
agrees.
"""
import os
import re
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_PI)
_UI = os.path.join(_ROOT, "mission-ui")

if _PI not in sys.path:
    sys.path.insert(0, _PI)

try:
    import yaml
except Exception:                       # pragma: no cover - env-dependent
    yaml = None


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _bottom_spec(js_source):
    """The `bottom: { ... }` block's numbers, as the browser would read them."""
    start = js_source.index("bottom: {")
    block = js_source[start:js_source.index("}", start)]
    hfov = float(re.search(r"hfov_deg:\s*([0-9.]+)", block).group(1))
    vfov = float(re.search(r"vfov_deg:\s*([0-9.]+)", block).group(1))
    res = re.search(r"resolution:\s*\[(\d+),\s*(\d+)\]", block)
    return {"hfov": hfov, "vfov": vfov,
            "res": (int(res.group(1)), int(res.group(2)))}


def _front_spec(js_source):
    start = js_source.index("front: {")
    block = js_source[start:js_source.index("}", start)]
    return {"hfov": float(re.search(r"hfov_deg:\s*([0-9.]+)", block).group(1))}


class CameraSpecDriftTest(unittest.TestCase):
    """Values here are CONFIGURED (config.yaml), and the FOV is vendor-quoted."""

    @classmethod
    def setUpClass(cls):
        if yaml is None:
            raise unittest.SkipTest("pyyaml not installed")
        if not os.path.isdir(_UI):
            raise unittest.SkipTest("mission-ui/ is not present in this checkout")
        cls.cfg = yaml.safe_load(_read(os.path.join(_PI, "config.yaml"))) or {}
        cls.cams = cls.cfg.get("cameras") or {}

    def test_config_has_the_two_roles_these_tests_compare(self):
        for role in ("front", "bottom"):
            self.assertIn(role, self.cams, "config.yaml lost cameras.%s" % role)
            self.assertIsNotNone(self.cams[role].get("hfov_deg"),
                                 "cameras.%s has no hfov_deg to compare" % role)

    def test_geo_js_matches_the_flight_config(self):
        spec = _bottom_spec(_read(os.path.join(_UI, "js", "geo.js")))
        want = self.cams["bottom"]
        self.assertEqual(spec["hfov"], float(want["hfov_deg"]),
                         "geo.js bottom hfov_deg != config.yaml cameras.bottom")
        if want.get("vfov_deg"):
            self.assertEqual(spec["vfov"], float(want["vfov_deg"]),
                             "geo.js bottom vfov_deg != config.yaml")
        self.assertEqual(list(spec["res"]), [int(x) for x in want["size"]],
                         "geo.js bottom resolution != config.yaml capture size")
        self.assertEqual(_front_spec(_read(os.path.join(_UI, "js", "geo.js")))["hfov"],
                         float(self.cams["front"]["hfov_deg"]),
                         "geo.js front hfov_deg != config.yaml")

    def test_map_js_matches_the_flight_config(self):
        """map.js is the one that actually spaces the rows, so it matters most."""
        spec = _bottom_spec(_read(os.path.join(_UI, "js", "map.js")))
        want = self.cams["bottom"]
        self.assertEqual(spec["hfov"], float(want["hfov_deg"]),
                         "map.js bottom hfov_deg != config.yaml cameras.bottom - "
                         "the coverage grid would be planned for the wrong lens")
        if want.get("vfov_deg"):
            self.assertEqual(spec["vfov"], float(want["vfov_deg"]))
        self.assertEqual(list(spec["res"]), [int(x) for x in want["size"]])
        self.assertEqual(_front_spec(_read(os.path.join(_UI, "js", "map.js")))["hfov"],
                         float(self.cams["front"]["hfov_deg"]))

    def test_the_two_js_files_agree_with_each_other(self):
        a = _bottom_spec(_read(os.path.join(_UI, "js", "geo.js")))
        b = _bottom_spec(_read(os.path.join(_UI, "js", "map.js")))
        self.assertEqual(a, b, "geo.js and map.js disagree about the camera")

    def test_the_old_waveshare_spec_is_gone(self):
        """The specific stale values, so a copy-paste cannot bring them back."""
        for name in ("js/geo.js", "js/map.js"):
            src = _read(os.path.join(_UI, name))
            start = src.index("bottom: {")
            block = src[start:src.index("}", start)]
            self.assertNotIn("100.0", block, "%s still has the 100 deg HFOV" % name)
            self.assertNotIn("Waveshare", block, "%s still names the old lens" % name)

    def test_the_planning_numbers_are_what_this_change_was_measured_on(self):
        """Recompute spacing the way optimalSpacing() does, and pin the figure.

        30% overlap, nadir, footprint_min * (1 - overlap). This is the number
        quoted in the docstrings above; if the formula in the JS ever changes,
        this test is the one that should be revisited deliberately.
        """
        import math
        spec = _bottom_spec(_read(os.path.join(_UI, "js", "map.js")))
        alt, overlap = 10.0, 0.3
        w = 2 * alt * math.tan(math.radians(spec["hfov"]) / 2.0)
        h = 2 * alt * math.tan(math.radians(spec["vfov"]) / 2.0)
        spacing = max(1.0, min(w, h) * (1.0 - overlap))
        self.assertAlmostEqual(w, 15.3, delta=0.2, msg="10 m footprint width")
        self.assertAlmostEqual(h, 11.5, delta=0.2, msg="10 m footprint height")
        self.assertAlmostEqual(spacing, 8.08, delta=0.05, msg="10 m row spacing")

    def test_the_fov_is_labelled_unverified_in_the_ui_too(self):
        """The repo's rule: vendor-quoted FOV must say so wherever it is used."""
        for name in ("js/geo.js", "js/map.js"):
            src = _read(os.path.join(_UI, name))
            start = src.index("bottom: {")
            block = src[start:src.index("}", start)]
            self.assertIn("VENDOR-QUOTED", block,
                          "%s uses a vendor FOV without saying it is unmeasured" % name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
