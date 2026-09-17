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
                 rotation_deg=0, jpeg_quality=80):
        super().__init__(name, index, size, hfov_deg, facing, rotation_deg,
                         jpeg_quality)
        self.kind = "rpi"
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


if __name__ == "__main__":
    test_csi_sensor_roles()
    test_usb_vidpid_match_and_fallback()
    test_single_camera_role()
    test_rescan_keeps_unchanged_and_adopts_new()
    test_rescan_drops_vanished()
    test_explicit_kind_wins()
    print("-" * 60)
    if FAILS:
        print("FAILED %d/%d: %s" % (len(FAILS), 6, ", ".join(FAILS)))
        sys.exit(1)
    print("all cam-autodetect tests passed")
