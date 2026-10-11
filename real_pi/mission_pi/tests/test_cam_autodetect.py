#!/usr/bin/env python3
"""Stdlib tests for the real_pi camera auto-detect (cameras.CameraRig).

No real cameras, no cv2: the hardware enumerators (list_cameras for CSI,
v4l2_capture_devices for USB) are monkeypatched and the camera CLASSES are
replaced with fakes that record start/stop.

Locked down:
  * CSI sensor -> role (imx708 front, imx477 bottom), role match_name
  * USB match_vidpid / match_name assignment
  * first-free-USB-node fallback (deterministic)
  * single camera -> single_cam_role
  * rescan(): unchanged assignments keep their RUNNING object (a live feed
    is never interrupted); newly appeared cameras are adopted; vanished
    ones are stopped
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))
import cameras as C  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


# ----------------------------------------------------------------- fakes --
class FakeCSI(C._BaseCamera):
    kind = "rpi"

    def __init__(self, name, index, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80, autofocus=None,
                 tuning_file=None):
        super().__init__(name, index, size, hfov_deg, facing, rotation_deg,
                         jpeg_quality)
        self.kind = "rpi"
        # recorded so the tests can prove config -> object wiring
        self.autofocus = None if autofocus is None else bool(autofocus)
        self.tuning_file = tuning_file
        self._open_calls = 0

    def _open(self):
        self._open_calls += 1

    def _grab(self):
        return None


class FakeUSB(C._BaseCamera):
    kind = "usb"

    def __init__(self, name, source, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80, backend=None, fps=30,
                 controls=None, fourcc=None):
        super().__init__(name, -1, size, hfov_deg, facing, rotation_deg,
                         jpeg_quality)
        self.kind = "usb"
        self.source = source
        self.index = source
        self._open_calls = 0

    def _open(self):
        self._open_calls += 1

    def _grab(self):
        return None


class HW:
    """Swappable hardware picture."""

    def __init__(self):
        self.csi = []        # [(index, model, info)]
        self.usb = []        # [dict node/name/vidpid/usb_desc/is_capture]

    def patch(self):
        C.list_cameras = lambda: list(self.csi)
        C.v4l2_capture_devices = lambda: list(self.usb)

    def unpatch(self, real_list_cameras, real_v4l2):
        C.list_cameras = real_list_cameras
        C.v4l2_capture_devices = real_v4l2


REAL_LIST_CAMERAS = C.list_cameras
REAL_V4L2 = C.v4l2_capture_devices
# make_rig() swaps C.Camera for FakeCSI and never restores it, so capture
# the real class up-front for tests that need its own methods.
REAL_CAMERA = C.Camera

CFG = {"cameras": {"auto_detect": True, "single_cam_role": "bottom",
                   "front": {"kind": "auto", "match_name": "imx708",
                             "size": [1920, 1080], "hfov_deg": 66.0},
                   "bottom": {"kind": "auto", "match_name": "imx477",
                              "size": [2028, 1520], "hfov_deg": 100.0}}}


def make_rig(cfg):
    C.Camera = FakeCSI
    C.USBCamera = FakeUSB
    return C.CameraRig(cfg)


def usb(node, vidpid, desc=""):
    n = int(node.split("video")[1])
    return {"node": node, "name": "usb%d" % n, "vidpid": vidpid,
            "usb_desc": desc, "is_capture": True}


# ------------------------------------------------------------------- tests --
def test_csi_sensor_roles():
    hw = HW()
    hw.csi = [(0, "imx708", {"Model": "imx708"}),
              (1, "imx477", {"Model": "imx477"})]
    hw.patch()
    rig = make_rig(CFG)
    a = rig.detect()
    rig2_cams = a
    check("csi: both cameras assigned", set(a) == {"cam1", "cam2"},
          repr({k: (c.kind, c.model) for k, c in a.items()}))
    check("csi: imx708 -> cam1 front",
          a["cam1"].model == "imx708" and a["cam1"].facing == "front",
          repr((a["cam1"].model, a["cam1"].facing)))
    check("csi: imx477 -> cam2 bottom",
          a["cam2"].model == "imx477" and a["cam2"].facing == "bottom")
    check("csi: device_desc carries the sensor",
          "csi" in a["cam1"].device_desc, a["cam1"].device_desc)
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_usb_vidpid_match_and_fallback():
    hw = HW()
    hw.csi = [(0, "imx477", {})]
    hw.usb = [usb("/dev/video1", "046d:0825", "Logitech C920"),
              usb("/dev/video3", "174c:55e8", "Gembird")]
    hw.patch()
    cfg = {"cameras": {"auto_detect": True,
                       "front": {"kind": "auto", "match_vidpid": "174c:55e8",
                                 "match_name": "imx708",
                                 "size": [1920, 1080]},
                       "bottom": {"kind": "auto", "match_name": "imx477",
                                  "size": [2028, 1520]}}}
    rig = make_rig(cfg)
    a = rig.detect()
    check("usb: front picks the match_vidpid node even if it sorts last",
          a["cam1"].source == "/dev/video3",
          repr({k: getattr(c, "source", c.index) for k, c in a.items()}))
    check("usb: bottom stays the CSI imx477", a["cam2"].model == "imx477")
    check("usb: device_desc has node + vidpid",
          "/dev/video3" in a["cam1"].device_desc
          and "174c:55e8" in a["cam1"].device_desc,
          a["cam1"].device_desc)

    # now front match falls back to name substring
    hw2 = HW()
    hw2.usb = [usb("/dev/video1", "046d:0825", "Logitech C920"),
               usb("/dev/video3", "174c:55e8", "Gembird")]
    hw2.patch()
    cfg2 = {"cameras": {"auto_detect": True,
                        "front": {"kind": "auto", "match_name": "logitech",
                                  "size": [1920, 1080]},
                        "bottom": {"kind": "auto", "size": [2028, 1520]}}}
    a2 = make_rig(cfg2).detect()
    check("usb: front match_name substring wins (logitech)",
          a2["cam1"].source == "/dev/video1",
          repr({k: c.source for k, c in a2.items()}))
    check("usb: bottom takes first remaining free node",
          a2["cam2"].source == "/dev/video3",
          repr({k: c.source for k, c in a2.items()}))
    hw2.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_single_camera_role():
    hw = HW()
    hw.csi = [(0, "imx477", {})]
    hw.patch()
    rig = make_rig(CFG)
    a = rig.detect()
    check("single: one camera assigned", len(a) == 1, repr(a))
    only = next(iter(a.values()))
    check("single: treated as bottom (single_cam_role)",
          only.facing == "bottom", only.facing)
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_single_usb_camera_also_takes_the_configured_role():
    """Auto-detect is meant to work with WHATEVER is plugged in: one USB
    camera and no CSI module must still give the mission a downward camera,
    not silently leave 'bottom' empty and hand the only camera to 'front'.
    """
    hw = HW()
    hw.usb = [usb("/dev/video0", "046d:0825", "Some USB cam")]
    hw.patch()
    a = make_rig(CFG).detect()
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)
    check("single usb: one camera assigned", len(a) == 1, repr(a))
    only = next(iter(a.values()))
    check("single usb: auto-picked camera is re-roled to bottom",
          only.facing == "bottom", only.facing)
    check("single usb: it was marked as auto-picked",
          getattr(only, "_auto_picked", False) is True, repr(only.__dict__))


def test_declared_usb_role_is_not_re_roled():
    """If config names the device for a role, that is a decision — one camera
    that the operator deliberately gave the front role must keep it."""
    hw = HW()
    hw.usb = [usb("/dev/video0", "046d:0825", "Some USB cam")]
    hw.patch()
    cfg = {"cameras": {"auto_detect": True, "single_cam_role": "bottom",
                       "front": {"kind": "usb", "device": "/dev/video0",
                                 "match_vidpid": "046d:0825",
                                 "hfov_deg": 66.0},
                       "bottom": {"kind": "auto", "match_name": "imx477"}}}
    a = make_rig(cfg).detect()
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)
    check("declared: one camera assigned", len(a) == 1, repr(a))
    check("declared: the named front role is kept",
          a["cam1"].facing == "front", a["cam1"].facing)


def test_rescan_keeps_a_live_camera_it_cannot_see():
    """Auto-detect runs every 30 s on real hardware: one failed probe
    (libcamera busy, an I2C/USB hiccup) must not black out a camera that is
    demonstrably still streaming. It IS dropped once it really stops.
    """
    hw = HW()
    hw.csi = [(0, "imx477", {})]
    hw.patch()
    rig = make_rig(CFG)
    a = rig.detect()
    key = next(iter(a))
    cam = a[key]
    cam.latest = lambda: (None, time.time(), 7)      # still delivering frames
    hw.csi = []                                      # detection now sees nothing
    changed = rig.rescan()
    check("rescan+live: streaming camera kept", key in rig.cams,
          repr(list(rig.cams)))
    check("rescan+live: kept WITHOUT being reported as a change",
          key not in changed, repr(changed))
    # ...and when the frames really stop, the next scan drops it
    cam.latest = lambda: (None, time.time() - 60.0, 7)
    changed = rig.rescan()
    check("rescan+live: stale camera is dropped", key not in rig.cams,
          repr(list(rig.cams)))
    check("rescan+live: the drop IS reported", key in changed, repr(changed))
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)

def test_rescan_keeps_unchanged_and_adopts_new():
    hw = HW()
    hw.csi = [(0, "imx477", {})]
    hw.usb = [usb("/dev/video1", "046d:0825", "Logitech C920")]
    hw.patch()
    rig = make_rig(CFG)
    a = rig.detect()
    rig.cams = dict(a)
    for c in rig.cams.values():
        c.start()
    cam1_before = rig.cams["cam1"]
    cam2_before = rig.cams["cam2"]

    # 1) no hardware change -> nothing changed, same running objects
    changed = rig.rescan()
    check("rescan: no-op keeps running objects", changed == []
          and rig.cams["cam1"] is cam1_before
          and rig.cams["cam2"] is cam2_before, repr(changed))
    check("rescan: no-op does not restart cameras",
          cam1_before._open_calls == 1 and cam2_before._open_calls == 1)

    # 2) a camera that was MISSING appears -> adopted + started
    #    (fresh rig with NO usb at all: cam1 unfilled)
    hw3 = HW()
    hw3.csi = [(0, "imx477", {})]
    hw3.patch()
    rig3 = make_rig(CFG)
    a3 = rig3.detect()
    rig3.cams = dict(a3)
    for c in rig3.cams.values():
        c.start()
    check("rescan-prep: cam1 unfilled with no USB", "cam1" not in rig3.cams,
          repr(sorted(rig3.cams)))
    hw3.usb = [usb("/dev/video5", "1d57:1391", "Gembird Webcam")]
    changed3 = rig3.rescan()
    check("rescan: new USB camera adopted", "cam1" in changed3
          and "cam1" in rig3.cams, repr(changed3))
    check("rescan: adopted camera is started",
          rig3.cams["cam1"].running is True
          and rig3.cams["cam1"]._open_calls == 1)
    check("rescan: untouched camera keeps its object",
          rig3.cams["cam2"] is a3["cam2"])
    hw3.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_rescan_drops_vanished():
    hw = HW()
    hw.csi = [(0, "imx708", {}), (1, "imx477", {})]
    hw.patch()
    rig = make_rig(CFG)
    rig.cams = dict(rig.detect())
    for c in rig.cams.values():
        c.start()
    cam1 = rig.cams["cam1"]
    hw.csi = [(1, "imx477", {})]   # imx708 vanishes
    changed = rig.rescan()
    check("rescan: vanished camera stopped + dropped",
          "cam1" in changed and cam1.running is False
          and "cam1" not in rig.cams, repr(changed))
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_explicit_kind_wins():
    hw = HW()
    hw.csi = [(0, "imx708", {})]
    hw.usb = [usb("/dev/video1", "046d:0825", "Logitech")]
    hw.patch()
    cfg = {"cameras": {"auto_detect": True,
                       "front": {"kind": "usb", "device": "/dev/video1",
                                 "size": [1920, 1080]},
                       "bottom": {"kind": "auto", "match_name": "imx708",
                                  "size": [2028, 1520]}}}
    a = make_rig(cfg).detect()
    check("explicit: kind usb with device wins for front",
          a["cam1"].source == "/dev/video1")
    check("explicit: bottom still auto from CSI", a["cam2"].model == "imx708")
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_csi_index_pin_beats_identical_sensors():
    """The Arducam UC-517 and a Waveshare module BOTH report "imx477".

    match_name cannot tell them apart, so without a pin the wrong physical
    camera can become "bottom" — and the bottom lens is what the whole 15 m
    QR model (FOV, footprint, focus) is built on.
    """
    hw = HW()
    # two IMX477s: an Arducam UC-517 and a Waveshare IR-CUT module
    hw.csi = [(0, "imx477", {}), (1, "imx477", {})]
    hw.patch()
    cfg = {"cameras": {"auto_detect": True,
                       "front": {"kind": "auto", "match_name": "imx708",
                                 "size": [1920, 1080], "hfov_deg": 66.0},
                       "bottom": {"kind": "auto", "match_name": "imx477",
                                  "csi_index": 1, "size": [2028, 1520],
                                  "hfov_deg": 75.0}}}
    a = make_rig(cfg).detect()
    check("csi_index: bottom pinned to index 1 (not 'first imx477 found')",
          a.get("cam2") is not None and a["cam2"].index == 1,
          repr({k: (c.index, c.model) for k, c in a.items()}))
    check("csi_index: bottom keeps its own lens numbers",
          a["cam2"].hfov_deg == 75.0)
    check("csi_index: the other camera takes the free slot",
          a.get("cam1") is not None and a["cam1"].index == 0,
          repr({k: (c.index, c.model) for k, c in a.items()}))

    # pinning is optional and never fatal: a bad/absent index falls back to
    # the sensor match instead of leaving the role empty
    cfg["cameras"]["bottom"]["csi_index"] = 9
    b = make_rig(cfg).detect()
    check("csi_index: impossible index falls back to sensor match",
          b.get("cam2") is not None and b["cam2"].model == "imx477",
          repr({k: (c.index, c.model) for k, c in b.items()}))
    cfg["cameras"]["bottom"]["csi_index"] = "not-a-number"
    c = make_rig(cfg).detect()
    check("csi_index: junk value falls back to sensor match",
          c.get("cam2") is not None and c["cam2"].model == "imx477")
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)


def test_csi_autofocus_and_tuning_are_config_driven():
    """A motorised-focus IMX477 (UC-517) must be able to autofocus from
    config, and the fixed-focus Waveshare must NOT be told to."""
    hw = HW()
    hw.csi = [(0, "imx708", {}), (1, "imx477", {})]
    hw.patch()
    cfg = {"cameras": {"auto_detect": True,
                       "front": {"kind": "auto", "match_name": "imx708",
                                 "size": [1920, 1080], "hfov_deg": 66.0,
                                 "autofocus": True},
                       "bottom": {"kind": "auto", "match_name": "imx477",
                                  "size": [2028, 1520], "hfov_deg": 75.0,
                                  "autofocus": True,
                                  "tuning_file": "imx477_af.json"}}}
    a = make_rig(cfg).detect()
    check("autofocus: forwarded to the CSI object",
          a["cam2"].autofocus is True)
    check("tuning_file: forwarded to the CSI object",
          a["cam2"].tuning_file == "imx477_af.json")
    check("tuning_file: not invented for roles that did not ask",
          getattr(a["cam1"], "tuning_file", None) is None)
    hw.unpatch(REAL_LIST_CAMERAS, REAL_V4L2)

    # the sensor-class rule itself (no hardware needed)
    d = REAL_CAMERA("cam2", 1, (2028, 1520), 75.0, "bottom")
    check("autofocus: unset + non-imx708 -> off (old behaviour kept)",
          d.wants_autofocus() is False)
    d.model = "imx708"
    check("autofocus: unset + imx708 -> on (old behaviour kept)",
          d.wants_autofocus() is True)
    d.autofocus = False
    check("autofocus: explicit false beats the sensor default",
          d.wants_autofocus() is False)
    e = REAL_CAMERA("cam2", 1, (2028, 1520), 75.0, "bottom", autofocus=True)
    e.model = "imx477"
    check("autofocus: explicit true enables it on an imx477 (UC-517)",
          e.wants_autofocus() is True)
    # a tuning file that does not exist must degrade, never raise
    e.tuning_file = "definitely_not_a_tuning_file.json"
    check("tuning_file: missing file degrades to None, no crash",
          e._load_tuning() is None)


if __name__ == "__main__":
    test_csi_sensor_roles()
    test_usb_vidpid_match_and_fallback()
    test_single_camera_role()
    test_single_usb_camera_also_takes_the_configured_role()
    test_declared_usb_role_is_not_re_roled()
    test_rescan_keeps_a_live_camera_it_cannot_see()
    test_rescan_keeps_unchanged_and_adopts_new()
    test_rescan_drops_vanished()
    test_explicit_kind_wins()
    test_csi_index_pin_beats_identical_sensors()
    test_csi_autofocus_and_tuning_are_config_driven()
    print("-" * 60)
    if FAILS:
        print("FAILED %d/%d: %s" % (len(FAILS), 11, ", ".join(FAILS)))
        sys.exit(1)
    print("all cam-autodetect tests passed")
