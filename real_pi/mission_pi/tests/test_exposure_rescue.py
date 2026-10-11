#!/usr/bin/env python3
"""Tests for the night/blind-camera fix (field report 2026-10-13, ~1 AM):

    "presets were changing but it was full on dark"

The presets were applied correctly — they set the camera to manual daylight
exposures ({AeEnable: False, ExposureTime: 5000, AnalogueGain: 1.26}), and
libcamera's AeEnable=false pins BOTH exposure and gain, so in a dark room the
picture was black (frame_mean=4) with no way to recover. A night profile must
therefore let the AE expose, with a FrameDurationLimits budget long enough to
collect light, and a manual profile must be rescued automatically when the
scene is too dark (or blown) for it.

No camera needed: the decisions are pure functions and the libcamera mapping is
checked against a stub module.
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)

import qr_camera_boost as QB  # noqa: E402
import cameras as C  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


# ------------------------------------------------- contract of the profiles --
def test_profiles_stay_compatible():
    """The UI/preset tests read these keys straight off the profile, so every
    profile must keep them even when AE owns exposure (a KeyError there would
    break the preset path, not just the picture)."""
    required = ("exposure_us", "gain_db", "brightness", "contrast",
                "saturation", "sharpness")
    for name, prof in sorted(QB.QR_BOOST_PROFILES.items()):
        missing = [k for k in required if k not in prof]
        check("profile %s keeps the tuning keys" % name, not missing, missing)

    day = QB.get_profile("qr_boost_day")
    check("the day profile is still manual (5 ms avoids daylight motion blur)",
          day.get("adaptive") is False, day.get("adaptive"))
    for name in ("qr_boost_lowlight", "qr_boost_night", "qr_boost_auto"):
        prof = QB.get_profile(name)
        check("%s uses AUTO EXPOSURE (a manual profile goes black at night)"
              % name, prof.get("adaptive") is True, prof.get("adaptive"))
    for name in ("qr_boost_lowlight", "qr_boost_night"):
        prof = QB.get_profile(name)
        check("%s gives the AE a long frame-duration budget" % name,
              int(prof.get("frame_duration_us") or 0) >= 100000,
              prof.get("frame_duration_us"))
        check("%s asks for the long-exposure mode and good NR" % name,
              prof.get("ae_exposure_mode") == "long"
              and prof.get("noise_reduction") == "highquality", prof)


# ------------------------------------------------------- the rescue decision --
def test_dark_rescue_decision():
    day = QB.get_profile("qr_boost_day")
    over, why = QB.dark_rescue(day, 4.0)
    check("a manual profile on a dark frame is rescued",
          bool(over) and over.get("adaptive") is True, (over, why))
    check("the rescue hands the AE a long frame-duration budget",
          int(over.get("frame_duration_us") or 0) >= 100000
          and over.get("ae_exposure_mode") == "long", over)
    check("the rescue says WHY, with the numbers", why and "mean 4" in why
          and "manual" in why, why)
    check("a bright frame is left alone", QB.dark_rescue(day, 120.0) == (None,
                                                                       None))
    check("a blown-out frame is rescued too (contrast dies up there)",
          QB.dark_rescue(day, 250.0)[0] is not None)
    check("an AUTO profile is never fought with",
          QB.dark_rescue(QB.get_profile("qr_boost_lowlight"), 4.0)
          == (None, None))
    check("unmeasurable/unknown stats change nothing",
          QB.dark_rescue(day, None) == (None, None)
          and QB.dark_rescue(day, -1.0) == (None, None)
          and QB.dark_rescue({}, 4.0) == (None, None)
          and QB.dark_rescue(None, 4.0) == (None, None))
    check("just below/above the thresholds behaves",
          QB.dark_rescue(day, QB.DARK_MEAN + 1)[0] is None
          and QB.dark_rescue(day, QB.DARK_MEAN - 1)[0] is not None)
    check("the restore threshold is above the dark threshold (no flapping)",
          QB.RESTORE_MEAN > QB.DARK_MEAN and not QB.recovered(30.0)
          and QB.recovered(60.0))


# ------------------------------------------------- the libcamera key mapping --
def test_new_tuning_keys_reach_libcamera():
    """frame_duration_us / ae_exposure_mode / noise_reduction must become real
    libcamera controls, or a night profile is only a comment."""
    fake_lc = types.ModuleType("libcamera")

    class Enums(object):
        Normal = "AE_NORMAL"
        Long = "AE_LONG"
        Short = "AE_SHORT"
        Custom = "AE_CUSTOM"

    class NR(object):
        Off = "NR_OFF"
        Fast = "NR_FAST"
        HighQuality = "NR_HQ"

    ctrl = types.SimpleNamespace(AeExposureModeEnum=Enums,
                                 NoiseReductionModeEnum=NR)
    fake_lc.controls = ctrl
    saved = sys.modules.get("libcamera")
    sys.modules["libcamera"] = fake_lc

    class FakePicam(object):
        def __init__(self):
            self.ctrls = {}

        def set_controls(self, ctrls):
            self.ctrls.update(ctrls)

    had = getattr(C, "_HAVE_PICAM", False)
    try:
        C._HAVE_PICAM = True
        cam = C.Camera("cam2", 0, (640, 480), 60.0, "bottom")
        cam._picam = FakePicam()
        prof = QB.get_profile("qr_boost_lowlight")
        cam.apply_tuning({k: prof[k] for k in
                          ("adaptive", "frame_duration_us", "ae_exposure_mode",
                           "noise_reduction", "brightness", "contrast",
                           "saturation", "sharpness")})
        ctrls = cam._picam.ctrls
        check("FrameDurationLimits is set (the AE's exposure ceiling)",
              ctrls.get("FrameDurationLimits") == (100000, 100000), ctrls)
        check("AeExposureMode=Long is set", ctrls.get("AeExposureMode") == "AE_LONG",
              ctrls)
        check("NoiseReductionMode=HighQuality is set",
              ctrls.get("NoiseReductionMode") == "NR_HQ", ctrls)
        check("AE is ON and no manual exposure is forced on top of it",
              ctrls.get("AeEnable") is True and "ExposureTime" not in ctrls
              and "AnalogueGain" not in ctrls, ctrls)

        # a MANUAL day profile must still pin exposure + gain
        cam._picam.ctrls = {}
        day = QB.get_profile("qr_boost_day")
        cam.apply_tuning({k: day[k] for k in
                          ("adaptive", "exposure_us", "gain_db", "contrast")})
        ctrls = cam._picam.ctrls
        check("the manual day profile still sets ExposureTime + AnalogueGain",
              ctrls.get("AeEnable") is False
              and ctrls.get("ExposureTime") == 5000
              and abs(ctrls.get("AnalogueGain", 0) - 1.2589) < 1e-3, ctrls)
    finally:
        C._HAVE_PICAM = had
        if saved is None:
            sys.modules.pop("libcamera", None)
        else:
            sys.modules["libcamera"] = saved


# ------------------------------------------------------ the rescue in a camera --
def test_camera_rescues_itself_and_comes_back(stub_libcamera=True):
    """End to end through the camera object: dark -> AE takes over (once, with
    a cooldown), light -> the operator's profile comes back by itself."""
    had = getattr(C, "_HAVE_PICAM", False)

    class FakePicam(object):
        def __init__(self):
            self.ctrls = {}
            self.history = []

        def set_controls(self, ctrls):
            self.ctrls.update(ctrls)
            self.history.append(dict(ctrls))

    try:
        C._HAVE_PICAM = True
        cam = C.Camera("cam2", 0, (640, 480), 60.0, "bottom")
        cam._picam = FakePicam()
        cam.apply_qr_profile("qr_boost_day")          # the daylight preset
        check("with the day profile the camera is MANUAL",
              cam.tuning["adaptive"] is False, cam.tuning["adaptive"])

        line = cam.rescue_dark_frame(4.0, now=1000.0)
        check("a dark frame triggers the rescue, and says so",
              line and "AUTO EXPOSURE restored" in line and "mean 4" in line,
              line)
        check("...and the camera really is on auto now",
              cam.tuning["adaptive"] is True
              and cam._picam.ctrls.get("AeEnable") is True,
              cam.tuning["adaptive"])
        check("it is not repeated on the next dark frame (cooldown)",
              cam.rescue_dark_frame(4.0, now=1001.0) is None)
        check("once the light is back the operator's profile returns",
              "re-applying" in (cam.rescue_dark_frame(90.0, now=1010.0) or "")
              and cam.tuning["adaptive"] is False,
              cam.tuning["adaptive"])
        check("a fresh darkness rescues again after the cooldown",
              (cam.rescue_dark_frame(5.0, now=1020.0) or "").startswith(
                  "the scene is dark"))

        # An already-auto camera must never be 'rescued' — that would fight the
        # AE (the 2026-10-13 preset bug in reverse).
        cam2 = C.Camera("cam2", 0, (640, 480), 60.0, "bottom")
        cam2._picam = FakePicam()
        cam2.apply_qr_profile("qr_boost_lowlight")
        check("an auto profile is left alone at any brightness",
              cam2.rescue_dark_frame(4.0, now=2000.0) is None
              and cam2.tuning["adaptive"] is True)
    finally:
        C._HAVE_PICAM = had


def test_detector_input_is_configurable():
    """`qr_boost.detect_raw` picks which frame the DETECTOR gets — the raw/gamma
    frame the model was trained on (the default) or the software-boosted one
    (CLAHE, tuned for the eye). The boost is computed lazily on demand, so it
    only runs when a caller actually asks for the boosted frame."""
    import threading
    real = QB.enhance_qr_frame
    QB.enhance_qr_frame = lambda f, mode=None: "CLAHE(%s)" % f

    class Probe(C.Camera):
        def __init__(self, detect_raw):
            self.name = "cam2"
            self.qr_boost = {"detect_raw": detect_raw,
                             "qr_software_enhance": True,
                             "qr_enhance_mode": "qr_boost"}
            self._lock = threading.Lock()
            self._frame_raw = "RAW"
            self._frame_ts = 1.0
            self._frames = 7
            self._enh_cache = None

    try:
        boosted = Probe(False)
        raw = Probe(True)
        check("detect_raw false hands the detector the boosted frame",
              boosted.latest(detect=True)[0] == "CLAHE(RAW)")
        check("detect_raw true hands the detector the raw frame",
              raw.latest(detect=True)[0] == "RAW")
        check("raw=True still means raw, whatever detect_raw says",
              raw.latest(raw=True)[0] == "RAW"
              and boosted.latest(raw=True)[0] == "RAW")
        check("plain latest() (streaming) is the boosted frame",
              boosted.latest()[0] == "CLAHE(RAW)" and raw.latest()[0] == "CLAHE(RAW)")
        check("ts + frame count ride along",
              boosted.latest(detect=True)[1:] == (1.0, 7))
    finally:
        QB.enhance_qr_frame = real


def test_manual_preset_in_the_dark_stays_on_auto():
    """2026-10-13 bug: pressing day/far/aggressive while the room was dark
    re-applied the MANUAL profile and the frame went black again (mean 4), and
    because the rescue was still flagged on, nothing ever restored it. A manual
    preset chosen in the dark must keep auto exposure until the light is back,
    then the chosen preset must apply by itself."""
    import time as _time
    had = getattr(C, "_HAVE_PICAM", False)

    class FakePicam(object):
        def __init__(self):
            self.ctrls = {}

        def set_controls(self, ctrls):
            self.ctrls.update(ctrls)

    try:
        C._HAVE_PICAM = True
        cam = C.Camera("cam2", 0, (640, 480), 60.0, "bottom")
        cam._picam = FakePicam()
        cam.frame_mean = 4.0                      # the room is dark right now
        for preset in ("qr_boost_day", "qr_boost_far", "qr_boost_aggressive"):
            if preset not in QB.QR_BOOST_PROFILES:
                continue
            cam.apply_qr_profile(preset)
            check("%s chosen in the dark keeps AUTO exposure" % preset,
                  cam.tuning["adaptive"] is True
                  and cam._picam.ctrls.get("AeEnable") is True,
                  cam.tuning.get("adaptive"))
            check("%s is remembered as the operator's choice" % preset,
                  cam.qr_boost.get("qr_boost_mode") == preset)
            cam.frame_mean = 4.0
        cam.apply_qr_profile("qr_boost_day")
        cam.frame_mean = 4.0
        check("a dark frame during the rescue does not undo it",
              cam.rescue_dark_frame(4.0, now=_time.time() + 60) is None
              and cam.tuning["adaptive"] is True)
        line = cam.rescue_dark_frame(90.0, now=_time.time() + 120)
        check("light back -> the chosen manual preset is applied again",
              line and "re-applying" in line and "qr_boost_day" in line
              and cam.tuning["adaptive"] is False, line)
        cam.frame_mean = 90.0
        cam.apply_qr_profile("qr_boost_day")
        check("a manual preset in a bright room is applied as-is",
              cam.tuning["adaptive"] is False and cam._rescue_on is False)
    finally:
        C._HAVE_PICAM = had


if __name__ == "__main__":
    test_profiles_stay_compatible()
    test_dark_rescue_decision()
    test_new_tuning_keys_reach_libcamera()
    test_camera_rescues_itself_and_comes_back()
    test_detector_input_is_configurable()
    test_manual_preset_in_the_dark_stays_on_auto()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all exposure-rescue tests passed")
