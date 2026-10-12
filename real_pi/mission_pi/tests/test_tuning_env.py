"""The tuning file must reach libcamera BEFORE the first camera manager.

Measured on the UC-517 (2026-10-13): focus_sweep drove LensPosition against the
stock imx477.json - "Could not set LENS_POSITION - no AF algorithm", seven
identical scores - while the repo shipped a working imx477_af.json and every
gate said the AF algorithm was present. libcamera picks each camera's tuning
when the camera is registered (first CameraManager start); picamera2's
`tuning=` dict acts at Picamera2 CONSTRUCTION, which is afterwards. The fix is
LIBCAMERA_IPA_CONFIG_PATH with per-sensor files, installed before any
enumeration - and the per-sensor naming is what keeps a second camera from
being handed the first one's tuning.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path[:0] = [PI, os.path.join(PI, "tools")]

import cameras  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


def _af_tuning(path):
    with open(path, "w") as f:
        json.dump({"version": 1,
                   "algorithms": [{"rpi.black_level": {}},
                                  {"rpi.af": {"ranges": {}}}]}, f)
    return path


def _save_env():
    return (os.environ.get("LIBCAMERA_IPA_CONFIG_PATH"),
            os.environ.get("LIBCAMERA_RPI_TUNING_FILE"))


def _restore_env(saved):
    old_path, old_rpi = saved
    for key, val in (("LIBCAMERA_IPA_CONFIG_PATH", old_path),
                     ("LIBCAMERA_RPI_TUNING_FILE", old_rpi)):
        if val is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = val


def _env_root():
    return [x for x in
            os.environ.get("LIBCAMERA_IPA_CONFIG_PATH", "").split(os.pathsep) if x]


def test_install_materialises_the_sensor_named_file():
    saved = _save_env()
    try:
        with tempfile.TemporaryDirectory() as td:
            tuning = _af_tuning(os.path.join(td, "imx477_af.json"))
            cache = os.path.join(td, "cache")
            installed, notes = cameras.install_tuning_env(
                cfg={"cameras": {"bottom": {"match_name": "imx477",
                                            "tuning_file": tuning}}},
                cache_root=cache, ipa_name="rpi/pisp")
            dest = os.path.join(cache, "rpi", "pisp", "imx477.json")
            check("one file installed", len(installed) == 1, repr(installed))
            check("under the SENSOR name libcamera looks up",
                  os.path.isfile(dest), repr(installed))
            check("and it is the AF tuning",
                  cameras.tuning_has_af(json.load(open(dest))))
            check("the search path starts with the cache root",
                  _env_root() and _env_root()[0] == cache, repr(_env_root()))
            check("nothing to warn about", not notes, repr(notes))
    finally:
        _restore_env(saved)


def test_only_named_sensors_get_an_override():
    """A second camera must keep its STOCK tuning - the global
    LIBCAMERA_RPI_TUNING_FILE override cannot promise that."""
    saved = _save_env()
    try:
        with tempfile.TemporaryDirectory() as td:
            tuning = _af_tuning(os.path.join(td, "imx477_af.json"))
            cache = os.path.join(td, "cache")
            cameras.install_tuning_env(
                cfg={"cameras": {"front": {"match_name": "imx708"},
                                 "bottom": {"match_name": "imx477",
                                            "tuning_file": tuning}}},
                cache_root=cache, ipa_name="rpi/pisp")
            check("the overridden sensor is there",
                  os.path.isfile(os.path.join(cache, "rpi", "pisp",
                                              "imx477.json")))
            check("the OTHER sensor is NOT overridden",
                  not os.path.exists(os.path.join(cache, "rpi", "pisp",
                                                  "imx708.json")))
    finally:
        _restore_env(saved)


def test_the_env_is_set_before_any_enumeration():
    """THE measured failure: the camera manager started before the tuning
    override existed, and every camera locked in the stock tuning."""
    saved = _save_env()
    old_home = os.environ.get("HOME")
    try:
        with tempfile.TemporaryDirectory() as td:
            tuning = _af_tuning(os.path.join(td, "imx477_af.json"))
            os.environ["HOME"] = td
            seen = {}

            class FakePicam:
                @staticmethod
                def global_camera_info():
                    seen["env"] = list(_env_root())
                    return []

            old_picam, old_have = cameras.Picamera2, cameras._HAVE_PICAM
            cameras.Picamera2, cameras._HAVE_PICAM = FakePicam, True
            try:
                cameras.CameraRig({"cameras": {
                    "bottom": {"kind": "auto", "match_name": "imx477",
                               "tuning_file": tuning}}}).detect()
            finally:
                cameras.Picamera2, cameras._HAVE_PICAM = old_picam, old_have
            check("the enumerator ran", "env" in seen, repr(seen))
            # precise: the override installed BY THIS detect() - derived from
            # the patched HOME - was already first on the path when the
            # enumeration ran
            check("the override was already on the path when it ran",
                  bool(seen.get("env")) and seen["env"][0].startswith(td),
                  repr(seen))
    finally:
        if old_home is not None:
            os.environ["HOME"] = old_home
        _restore_env(saved)


def test_open_does_not_use_the_global_override_when_the_path_is_active():
    """picamera2 turns tuning= into LIBCAMERA_RPI_TUNING_FILE - one file for
    EVERY camera in the process. When the per-sensor path is active it must
    not be used."""
    saved = _save_env()
    try:
        with tempfile.TemporaryDirectory() as td:
            tuning = _af_tuning(os.path.join(td, "imx477_af.json"))
            calls = []

            class FakePicam:
                def __init__(self, index=0, **kw):
                    calls.append(kw)
                    self.camera_properties = {"Model": "imx477"}

                def close(self):
                    pass

            old_picam, old_have = cameras.Picamera2, cameras._HAVE_PICAM
            cameras.Picamera2, cameras._HAVE_PICAM = FakePicam, True
            try:
                cam = cameras.Camera("cam2", 0, (2028, 1520), 75.0, "bottom",
                                     tuning_file=tuning)
                cam.model = "imx477"
                try:
                    cam._open()
                except Exception:
                    pass  # the fake only records constructor kwargs
            finally:
                cameras.Picamera2, cameras._HAVE_PICAM = old_picam, old_have
            check("the camera opened", len(calls) == 1, repr(calls))
            check("without the global tuning override",
                  calls and calls[0].get("tuning") is None, repr(calls))
    finally:
        _restore_env(saved)


def test_a_missing_tuning_file_is_a_note_not_a_crash():
    saved = _save_env()
    try:
        with tempfile.TemporaryDirectory() as td:
            installed, notes = cameras.install_tuning_env(
                cfg={"cameras": {"bottom": {"match_name": "imx477",
                                            "tuning_file": "no_such.json"}}},
                cache_root=os.path.join(td, "cache"), ipa_name="rpi/pisp")
            check("nothing installed", installed == [], repr(installed))
            check("and it says so", bool(notes), repr(notes))
            check("junk cfg never raises",
                  cameras.install_tuning_env(cfg={"cameras": {"bottom": None,
                                                              "front": "junk"}},
                                             cache_root=td)[0] == [])
    finally:
        _restore_env(saved)


if __name__ == "__main__":
    test_install_materialises_the_sensor_named_file()
    test_only_named_sensors_get_an_override()
    test_the_env_is_set_before_any_enumeration()
    test_open_does_not_use_the_global_override_when_the_path_is_active()
    test_a_missing_tuning_file_is_a_note_not_a_crash()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all tuning-env tests passed")
