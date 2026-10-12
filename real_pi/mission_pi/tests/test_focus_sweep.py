#!/usr/bin/env python3
"""Tests for the focus-sweep tool's pure parts + the manual-focus config path.

The sweep itself needs a VCM lens on a Pi; what must be provably right without
hardware is the position list (a bad sweep aims at the wrong distance), the
distance read-out (dioptres are easy to get wrong by 1000x), the best-pick,
the config snippet the operator pastes, and the precedence between
`autofocus` and `lens_position`.

No picamera2, no cv2, no camera.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import cameras as C  # noqa: E402
import focus_sweep as FS  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


REAL_CAMERA = C.Camera


# ----------------------------------------------------------- position lists --
def test_position_list_range_and_list_forms():
    r = FS.parse_positions("0:1:0.25")
    check("range form 0:1:0.25", r == [0, 0.25, 0.5, 0.75, 1.0], repr(r))
    check("range form includes the stop when it lands on it",
          FS.parse_positions("0:0.5:0.5") == [0, 0.5], repr(FS.parse_positions("0:0.5:0.5")))
    check("list form", FS.parse_positions("0,0.067,0.2") == [0, 0.067, 0.2])
    check("single value", FS.parse_positions("0.1") == [0.1])
    check("spaces in a list are fine", FS.parse_positions("0, 0.1 ") == [0, 0.1])


def test_position_list_rejects_nonsense():
    for bad in ("", "0:1", "0:1:0", "0:1:-1", "a,b"):
        try:
            out = FS.parse_positions(bad)
        except ValueError:
            check("rejected: %r" % bad, True)
            continue
        check("rejected: %r" % bad, False, "returned %r" % (out,))


def test_positions_are_capped():
    r = FS.parse_positions("0:1000:0.01")
    check("a huge range cannot hang the tool", len(r) <= 1000, len(r))


# ------------------------------------------------------------- distance unit --
def test_distance_units_are_dioptres():
    check("0 is infinity", FS.describe_distance(0) == "infinity")
    check("1 dioptre is 1 m", FS.describe_distance(1) == "1.00 m")
    check("0.067 is ~15 m (the mission altitude)",
          FS.describe_distance(0.067).startswith("14.9"),
          FS.describe_distance(0.067))
    check("0.5 is 2 m", FS.describe_distance(0.5) == "2.00 m")
    check("tiny values are not '1000 km'",
          FS.describe_distance(1e-7) == "infinity")
    check("junk is not a crash", FS.describe_distance("x") == "?")


def test_best_pick_ignores_failed_samples():
    samples = [(0, 12.0), (0.1, None), (0.2, 31.0), (0.3, 5.0)]
    check("the highest score wins", FS.pick_best(samples) == (0.2, 31.0),
          repr(FS.pick_best(samples)))
    check("all-failed -> (None, None)", FS.pick_best([(0, None)]) == (None, None))
    check("empty -> (None, None)", FS.pick_best([]) == (None, None))


def test_config_snippet_is_pasteable():
    snip = FS.config_snippet("bottom", 0.067, 245.7, "imx477")
    check("names the role", "cameras.bottom:" in snip, snip)
    check("turns autofocus off", "autofocus: false" in snip, snip)
    check("sets the measured position", "lens_position: 0.067" in snip, snip)
    check("records the score so it can be compared later",
          "246" in snip and "imx477" in snip, snip)   # 245.7 -> "%.0f"
    check("a missing measurement is obvious, not zero",
          "lens_position: <n>" in FS.config_snippet("bottom", None), snip)


# ------------------------------------------------- manual-focus config path --
def test_lens_position_precedence_and_parsing():
    a = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                    autofocus=True, lens_position=0.067)
    check("autofocus wins over a pinned position (no silent lie)",
          a.wants_lens_position() is None)
    b = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                    autofocus=False, lens_position=0.067)
    check("manual focus position is used when AF is off",
          b.wants_lens_position() == 0.067, b.wants_lens_position())
    c = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom", autofocus=False)
    check("unset position -> leave the lens alone",
          c.wants_lens_position() is None)
    d = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom", autofocus=False,
                    lens_position="0.05")
    check("a numeric string is accepted", d.wants_lens_position() == 0.05)
    e = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom", autofocus=False,
                    lens_position="five")
    check("junk degrades to None, never a crash", e.wants_lens_position() is None)
    f = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom", autofocus=False,
                    lens_position=0)
    check("position 0 (infinity) is a real value, not 'unset'",
          f.wants_lens_position() == 0)


def test_config_key_reaches_the_camera_object():
    """`cameras.bottom.lens_position` must arrive at the CSI object even when
    the role was assigned by sensor match."""
    class FakeCSI(REAL_CAMERA):
        def __init__(self, name, index, size, hfov, facing, rot=0, q=80,
                     autofocus=None, tuning_file=None, lens_position=None):
            super().__init__(name, index, size, hfov, facing, rot, q,
                             autofocus=autofocus, tuning_file=tuning_file,
                             lens_position=lens_position)

    real = C.Camera
    real_list = C.list_cameras
    C.Camera = FakeCSI
    C.list_cameras = lambda: [(0, "imx477", {})]
    try:
        rig = C.CameraRig({"cameras": {"single_cam_role": "bottom",
                                       "bottom": {"kind": "auto",
                                                  "match_name": "imx477",
                                                  "autofocus": False,
                                                  "lens_position": 0.067}}})
        a = rig.detect()
    finally:
        C.Camera = real
        C.list_cameras = real_list
    only = next(iter(a.values()))
    check("lens_position reached the object", only.lens_position == 0.067,
          repr(getattr(only, "lens_position", None)))
    check("the role is still bottom", only.facing == "bottom", only.facing)


def test_tuning_file_can_be_a_repo_path(tmp_path="/tmp"):
    """A tuning file we ship ourselves must load by PATH (no sudo, survives
    OS updates) — and a missing one must degrade, never raise."""
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "my_af.json")
        with open(p, "w") as f:
            json.dump({"version": 1, "algorithms": []}, f)
        c = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                        autofocus=False, tuning_file=p)
        t = c._load_tuning()
        check("an absolute path is loaded as JSON",
              isinstance(t, dict) and t.get("version") == 1, repr(t))
        c2 = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                         autofocus=False, tuning_file="/nope/missing.json")
        check("a missing path degrades to None (no raise)",
              c2._load_tuning() is None)
        # a name that exists NOWHERE - not "imx477_af.json", because the repo
        # now SHIPS that file and a fixture that leans on its absence breaks the
        # moment the missing file arrives (it did)
        c3 = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                         autofocus=False,
                         tuning_file="no_such_tuning_xyz.json")
        check("a bare name still goes through Picamera2.load_tuning_file "
              "and degrades here", c3._load_tuning() is None)
        # and the file the repo DOES ship now resolves for a bare name - the
        # whole point of shipping it: the UC-517's AF needs no sudo
        c4 = REAL_CAMERA("cam2", 0, (2028, 1520), 75.0, "bottom",
                         autofocus=False, tuning_file="imx477_af.json")
        t4 = c4._load_tuning()
        algos = [list(a)[0] for a in ((t4 or {}).get("algorithms") or []) if a]
        check("the shipped imx477_af.json resolves for a bare name and "
              "carries rpi.af", isinstance(t4, dict) and
              any(a.endswith(".af") for a in algos),
              "algos=%r" % (algos[:5],))


if __name__ == "__main__":
    test_position_list_range_and_list_forms()
    test_position_list_rejects_nonsense()
    test_positions_are_capped()
    test_distance_units_are_dioptres()
    test_best_pick_ignores_failed_samples()
    test_config_snippet_is_pasteable()
    test_lens_position_precedence_and_parsing()
    test_config_key_reaches_the_camera_object()
    test_tuning_file_can_be_a_repo_path()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all focus-sweep tests passed")
