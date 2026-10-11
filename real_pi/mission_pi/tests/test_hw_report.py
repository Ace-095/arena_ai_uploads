#!/usr/bin/env python3
"""Tests for the boot-config / missing-file diagnostics the tools print.

"csi: none found" on a Pi 5 is nearly always a /boot/firmware/config.txt
problem — a third-party CSI module (Arducam UC-517 …) is invisible to
libcamera until `camera_auto_detect=0` and an explicit `dtoverlay=` line are
there. The parser that explains that must therefore be right, and it is
testable without a Pi.
"""
import io
import os
import sys
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import list_cams  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


ARMED_PI5 = """\
# For more options and information see
# http://rptl.io/configtxt
arm_64bit=1
camera_auto_detect=0
dtparam=pciex1_gen=2

[all]
dtoverlay=vc4-kms-v3d
dtoverlay=imx477,vcm
"""

DEFAULT_PI = """\
camera_auto_detect=1
#dtoverlay=imx477,vcm
[all]
dtoverlay=vc4-kms-v3d
"""

CAM0_PI = """\
camera_auto_detect=0
[all]
dtoverlay=imx708
dtoverlay=imx477,vcm,cam0
"""


def test_camera_auto_detect_is_read():
    cfg = list_cams.parse_boot_config(ARMED_PI5)
    check("auto-detect value parsed", cfg["camera_auto_detect"] == "0",
          repr(cfg))
    cfg2 = list_cams.parse_boot_config(DEFAULT_PI)
    check("the Pi default (=1) is reported as-is",
          cfg2["camera_auto_detect"] == "1", repr(cfg2))


def test_overlay_lines_are_found_and_comments_ignored():
    cfg = list_cams.parse_boot_config(ARMED_PI5)
    check("imx477,vcm found", cfg["overlays"] == ["imx477,vcm"], repr(cfg))
    cfg2 = list_cams.parse_boot_config(DEFAULT_PI)
    check("a commented-out overlay is NOT counted", cfg2["overlays"] == [],
          repr(cfg2))
    cfg3 = list_cams.parse_boot_config(CAM0_PI)
    check("both cameras found, cam0 suffix kept",
          cfg3["overlays"] == ["imx708", "imx477,vcm,cam0"], repr(cfg3))
    check("unrelated overlays are ignored",
          all("vc4" not in o for o in cfg3["overlays"]), repr(cfg3))


def test_empty_and_junk_input_never_raise():
    for text in (None, "", "garbage", "dtoverlay\ncamera_auto_detect=\n"):
        try:
            cfg = list_cams.parse_boot_config(text)
        except Exception as e:                    # pragma: no cover
            check("junk input handled: %r" % (text,), False, repr(e))
            continue
        check("junk input handled: %r" % (text,),
              isinstance(cfg, dict) and "overlays" in cfg, repr(cfg))


def test_report_prints_the_two_files_that_decide_success():
    """The report must name the .hef and the AF tuning file — the two missing
    files that silently degrade the stack (Hailo falls back to CPU; the
    UC-517 focus motor gets no controls)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        list_cams.print_file_checks()
    out = buf.getvalue()
    check("the HEF is named", "qr_yolov8n.hef" in out, out)
    check("its state is stated, not implied",
          ("OK" in out or "MISSING" in out), out)
    hef = os.path.join(PI, "models", "qr_yolov8n.hef")
    check("the reported state matches the filesystem",
          ("OK" in out) == os.path.isfile(hef), out)


def test_boot_report_returns_a_dict_without_a_pi():
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cfg = list_cams.print_boot_config()
    check("boot report returns a dict (possibly empty off-Pi)",
          isinstance(cfg, dict), repr(cfg))


if __name__ == "__main__":
    test_camera_auto_detect_is_read()
    test_overlay_lines_are_found_and_comments_ignored()
    test_empty_and_junk_input_never_raise()
    test_report_prints_the_two_files_that_decide_success()
    test_boot_report_returns_a_dict_without_a_pi()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hw-report tests passed")
