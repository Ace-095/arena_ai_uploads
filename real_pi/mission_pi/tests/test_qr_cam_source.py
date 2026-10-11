#!/usr/bin/env python3
"""Stdlib tests for the QR bench tool's camera + detector plumbing.

The Pi's CSI cameras are libcamera devices OpenCV cannot open, and the Hailo
HAT is not present on a dev machine. Both are therefore faked here so the
FAILURE and SELECTION paths are provable without hardware:

  * `--source csi:bottom` picks the downward camera out of the rig
  * `--source csi:1` picks by libcamera index; unknown specs fall back with a
    message instead of dying
  * read() waits for the camera thread's first frame and release() stops it
  * `--hailo` with a missing HAT/HEF degrades to the CPU ladder (a bench run
    must never turn into a traceback because the HAT is absent)

No cv2, no picamera2, no hailo_platform required.
"""
import os
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import cameras as C  # noqa: E402
import test_yolo_webcam_qr as T  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


class FakeCam:
    """A camera object with the attributes CSISource actually touches."""

    def __init__(self, name, index, facing, model, size=(2028, 1520)):
        self.name = name
        self.index = index
        self.facing = facing
        self.model = model
        self.size = size
        self.autofocus = True
        self.started = 0
        self.stopped = 0
        self.frames = 0

    def start(self):
        self.started += 1

    def stop(self):
        self.stopped += 1

    def latest(self):
        # a real frame first, then None (to exercise the wait loop once)
        self.frames += 1
        if self.frames == 1:
            return None, None, 0
        return ("FRAME-bgr", time.time(), self.frames)


class FakeRig:
    """Stands in for cameras.CameraRig: detect() = the hardware picture."""

    def __init__(self, cfg):
        self.cfg = cfg

    def detect(self):
        return {
            "cam1": FakeCam("cam1", 0, "front", "imx708", (4608, 2592)),
            "cam2": FakeCam("cam2", 1, "bottom", "imx477", (2028, 1520)),
        }


class EmptyRig(FakeRig):
    def detect(self):
        return {}


REAL_RIG = C.CameraRig


def with_rig(cls, fn):
    C.CameraRig = cls
    try:
        return fn()
    finally:
        C.CameraRig = REAL_RIG


# --------------------------------------------------------------- CSI source --
def test_csi_source_picks_the_bottom_camera():
    def run():
        src = T.CSISource("csi:bottom", {})
        ok, frame = src.read()
        return src, ok, frame
    src, ok, frame = with_rig(FakeRig, run)
    check("csi:bottom -> the downward camera", src.cam.facing == "bottom",
          src.cam.facing)
    check("csi:bottom -> cam2 (slot), not cam1", src.cam.name == "cam2",
          src.cam.name)
    check("csi:bottom -> the camera was started", src.cam.started == 1,
          repr(src.cam.started))
    check("csi:read waits for the first real frame then returns it",
          ok is True and frame == "FRAME-bgr", repr((ok, frame)))
    src.release()
    check("csi:release stops the camera", src.cam.stopped == 1,
          repr(src.cam.stopped))


def test_csi_source_by_index_and_role():
    def by_index():
        return T.CSISource("csi:0", {})
    src = with_rig(FakeRig, by_index)
    check("csi:<index> selects by libcamera index", src.cam.name == "cam1",
          src.cam.name)

    def by_role():
        return T.CSISource("csi:front", {})
    src2 = with_rig(FakeRig, by_role)
    check("csi:front selects the front role", src2.cam.facing == "front",
          src2.cam.facing)

    def bare():
        return T.CSISource("csi", {})
    src3 = with_rig(FakeRig, bare)
    check("bare 'csi' defaults to the downward camera",
          src3.cam.facing == "bottom", src3.cam.facing)

    def by_slot():
        return T.CSISource("csi:cam1", {})
    src4 = with_rig(FakeRig, by_slot)
    check("csi:<slot> selects the rig slot", src4.cam.name == "cam1",
          src4.cam.name)


def test_csi_source_reports_what_it_picked_and_degrades():
    def unknown():
        return T.CSISource("csi:video9", {})       # nothing matches
    src = with_rig(FakeRig, unknown)
    check("unknown spec still yields a working camera (best guess)",
          src.cam.facing == "bottom", src.cam.facing)

    def no_hardware():
        try:
            return T.CSISource("csi:bottom", {})
        except Exception as e:
            return e
    err = with_rig(EmptyRig, no_hardware)
    check("no camera -> a clear error, not a crash",
          isinstance(err, RuntimeError) and "no CSI camera" in str(err),
          repr(err))


def test_csi_source_geometry_comes_from_config_not_opencv():
    def run():
        return T.CSISource("csi:bottom", {})
    src = with_rig(FakeRig, run)
    import cv2
    check("get(width) reports the config size",
          src.get(cv2.CAP_PROP_FRAME_WIDTH) == 2028.0,
          src.get(cv2.CAP_PROP_FRAME_WIDTH))
    check("get(height) reports the config size",
          src.get(cv2.CAP_PROP_FRAME_HEIGHT) == 1520.0,
          src.get(cv2.CAP_PROP_FRAME_HEIGHT))
    check("set() is a no-op (size/fps live in config.yaml)", src.set(3, 640) is False)
    check("isOpened() is true for a started CSI camera", src.isOpened() is True)


# ------------------------------------------------------------------ detector --
def test_hailo_flag_degrades_without_the_hat():
    args = SimpleNamespace(model=None, iou=0.45, hef="/nonexistent/nope.hef")
    det = T.build_detector(args, 0.35, None, kind="hailo")
    check("--hailo without a HAT/HEF falls back to a real detector",
          det is not None and hasattr(det, "detect"), repr(det))
    check("the fallback is NOT labelled hailo", det.name != "hailo", det.name)


def test_cli_exposes_the_pi_and_hailo_paths():
    import subprocess
    out = subprocess.run([sys.executable,
                          os.path.join(PI, "tools", "test_yolo_webcam_qr.py"),
                          "--help"], capture_output=True, text=True, timeout=60)
    helptext = out.stdout
    check("--hailo is documented in --help", "--hailo" in helptext)
    check("--hef is documented in --help", "--hef" in helptext)
    check("the CSI source is documented in --help", "csi:bottom" in helptext)
    check("--help exits 0", out.returncode == 0, repr(out.returncode))


if __name__ == "__main__":
    test_csi_source_picks_the_bottom_camera()
    test_csi_source_by_index_and_role()
    test_csi_source_reports_what_it_picked_and_degrades()
    test_csi_source_geometry_comes_from_config_not_opencv()
    test_hailo_flag_degrades_without_the_hat()
    test_cli_exposes_the_pi_and_hailo_paths()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all qr-cam-source tests passed")
