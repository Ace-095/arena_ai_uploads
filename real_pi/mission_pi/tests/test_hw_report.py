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


USER_FILE = """\
# For more options and information see
dtparam=i2c_arm=off
camera_auto_detect=1
dtoverlay=vc4-kms-v3d
arm_64bit=1
[cm4]
otg_mode=1
[cm5]
dtoverlay=dwc2,dr_mode=host
[pi5]
dtoverlay=nospi10

[all],,
"""


def test_user_config_is_diagnosed_correctly():
    """The exact config.txt from the field run: auto-detect still on, no
    camera overlay, and a malformed [all],, header at the end."""
    cfg = list_cams.parse_boot_config(USER_FILE)
    check("field file: auto-detect read as 1", cfg["camera_auto_detect"] == "1",
          repr(cfg["camera_auto_detect"]))
    check("field file: no camera overlay found", cfg["overlays"] == [],
          repr(cfg["overlays"]))
    check("field file: the malformed header is caught",
          cfg["malformed_sections"] == ["[all],,"],
          repr(cfg["malformed_sections"]))
    check("field file: real sections are listed",
          {"cm4", "cm5", "pi5"} <= set(cfg["sections"]), repr(cfg["sections"]))
    ok, lines = list_cams.boot_verdict(cfg, board="pi5")
    text = "\n".join(lines)
    check("field file: verdict fails", ok is False, text)
    check("field file: verdict names the malformed header",
          "malformed section header" in text, text)
    check("field file: verdict asks for camera_auto_detect=0",
          "set it to 0" in text, text)


def test_overlay_under_the_wrong_section_is_caught():
    """A correct-looking dtoverlay line in the wrong section never applies."""
    cfg = list_cams.parse_boot_config(
        "camera_auto_detect=0\n[cm4]\ndtoverlay=imx477,vcm\n")
    ok, lines = list_cams.boot_verdict(cfg, board="pi5", dtbo_path="/nonexistent")
    text = "\n".join(lines)
    check("wrong section -> verdict fails", ok is False, text)
    check("wrong section -> says it is skipped", "skipped" in text, text)

    cfg2 = list_cams.parse_boot_config(
        "camera_auto_detect=0\n[pi5]\ndtoverlay=imx477,vcm\n")
    ok2, lines2 = list_cams.boot_verdict(cfg2, board="pi5",
                                         dtbo_path="/nonexistent/x.dtbo")
    text2 = "\n".join(lines2)
    check("matching model section is accepted (only the dtbo may fail)",
          "sits under [pi5]" not in text2 and "[pi5] — this board" in text2,
          text2)

    cfg3 = list_cams.parse_boot_config(
        "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
    ok3, lines3 = list_cams.boot_verdict(cfg3, board="pi4",
                                         dtbo_path="/nonexistent/x.dtbo")
    check("[all] applies to every board",
          all("sits under" not in l for l in lines3), "\n".join(lines3))


def test_board_section_maps_models():
    check("Pi 5 -> pi5",
          list_cams.board_section("Raspberry Pi 5 Model B Rev 1.0") == "pi5")
    check("CM4 -> cm4",
          list_cams.board_section("Raspberry Pi Compute Module 4 Rev 1.0") == "cm4")
    check("Zero 2 W -> zero2w",
          list_cams.board_section("Raspberry Pi Zero 2 W Rev 1.0") == "zero2w")
    check("unknown model -> None (never a guess)",
          list_cams.board_section("Some Other SBC") is None)


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
    hef_line = [ln for ln in out.splitlines() if "qr_yolov8n.hef" in ln]
    check("the reported state matches the filesystem",
          len(hef_line) == 1 and
          (("OK" in hef_line[0]) == os.path.isfile(hef)), out)


def test_file_checks_report_the_af_algorithm_not_just_the_filename():
    """The decisive autofocus question is whether the tuning libcamera will
    load carries `rpi.af` — the camera advertises AfMode/LensPosition either
    way, so a filename check alone let a dead autofocus look healthy."""
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        pisp = os.path.join(td, "pisp")
        vc4 = os.path.join(td, "vc4")
        os.makedirs(pisp)
        os.makedirs(vc4)
        with open("/tmp/imx477.json", "w") as f:   # stock: no rpi.af
            json.dump({"algorithms": [{"rpi.black_level": {}},
                                      {"rpi.agc": {}}]}, f)
        with open(os.path.join(pisp, "imx477.json"), "w") as f:
            json.dump({"algorithms": [{"rpi.black_level": {}},
                                      {"rpi.af": {"lens": {}}}]}, f)
        os.unlink("/tmp/imx477.json")
        real = list_cams.IPA_TUNING_DIRS
        list_cams.IPA_TUNING_DIRS = (pisp, vc4)
        try:
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                list_cams.print_file_checks()
            out = buf.getvalue()
        finally:
            list_cams.IPA_TUNING_DIRS = real
        check("an installed AF tuning is reported as autofocus AVAILABLE",
              "imx477.json contains rpi.af" in out, out)
        check("the missing vendor json is still called out",
              "MISSING" in out, out)
        check("an absent sensor tuning is simply not mentioned",
              "vc4/imx477.json" not in out, out)


def test_the_repo_local_af_tuning_is_reported_where_the_stack_reads_it():
    """A shipped tuning file is the no-sudo fix for the UC-517's dead autofocus
    - but only if this report can SEE it. The check looked in tools/ and
    tools/models/ while cameras._load_tuning reads mission_pi/ and
    mission_pi/models/, so a perfectly good copy in the right place printed
    nothing here."""
    cand = os.path.join(PI, "models", "imx477_af.json")
    check("the shipped AF tuning is present at %s" % cand,
          os.path.isfile(cand))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        list_cams.print_file_checks()
    out = buf.getvalue()
    check("the report names it as the copy the stack will use",
          ("repo-local %s" % cand) in out, out)
    check("and claims no tools/-relative path the stack never reads",
          os.path.join("tools", "models", "imx477_af.json") not in out, out)


def _vcm_dtbo(tmpdir, with_vcm=True):
    """A fake dtbo: parameter names are plain strings in the blob."""
    p = os.path.join(tmpdir, "imx477.dtbo")
    body = (b"\xd0\x0d\xfe\xed" +
            (b"\x00vcm\x00rotation\x00cam0\x00" if with_vcm
             else b"\x00rotation\x00orientation\x00cam0\x00link-frequency\x00"))
    with open(p, "wb") as f:
        f.write(body)
    return p


def test_verdict_catches_the_real_field_failure():
    """The exact state of the user's Pi: camera_auto_detect=1 and no camera
    overlay at all. The verdict must name both, and fail."""
    cfg = list_cams.parse_boot_config(
        "camera_auto_detect=1\n[all]\ndtoverlay=vc4-kms-v3d\n")
    ok, lines = list_cams.boot_verdict(cfg)
    text = "\n".join(lines)
    check("verdict fails when auto-detect is on", ok is False, text)
    check("verdict says to set camera_auto_detect=0",
          "camera_auto_detect=1" in text and "set it to 0" in text, text)
    check("verdict gives the exact overlay line to add",
          "dtoverlay=imx477,vcm" in text, text)


def test_dtoverlay_h_output_is_parsed():
    """`dtoverlay -h imx477` is the authoritative vcm check (bare `strings`
    hides 3-character param names). Its output shape is stable; parse it."""
    arducam = """\
Name:   imx477
Info:   Overlay for the Sony IMX477 camera on the Raspberry Pi
Usage:  dtoverlay=imx477,<param>=<val>
Params: rotation                  Mounting rotation of the camera sensor

        orientation               Orientation of the camera sensor

        cam0                      Select the camera port

        vcm                       Enable the VCM (voice coil motor) driver
"""
    stock = """\
Name:   imx477
Info:   Overlay for the Sony IMX477 camera on the Raspberry Pi
Usage:  dtoverlay=imx477,<param>=<val>
Params: rotation                  Mounting rotation of the camera sensor

        orientation               Orientation of the camera sensor

        cam0                      Select the camera port
"""
    p_a = list_cams.parse_dtoverlay_params(arducam)
    p_s = list_cams.parse_dtoverlay_params(stock)
    check("Arducam's params include vcm", "vcm" in p_a, repr(p_a))
    check("other params are parsed too", {"rotation", "cam0"} <= p_a, repr(p_a))
    check("the stock overlay has NO vcm", "vcm" not in p_s, repr(p_s))
    check("headings are not mistaken for params",
          not any(x.startswith(("Name", "Info", "Usage")) for x in p_a), repr(p_a))
    check("empty output -> empty set",
          list_cams.parse_dtoverlay_params("") == set())
    check("garbage -> empty set",
          list_cams.parse_dtoverlay_params("no such overlay") == set())


def test_verdict_advice_does_not_use_the_three_letter_trap():
    """Regression: the advice told people to run `strings <dtbo> | grep vcm`,
    which prints NOTHING for a 3-character param name (GNU strings needs 4+
    characters by default) — a false 'still stock' verdict."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        stock = _vcm_dtbo(td, with_vcm=False)
        cfg = list_cams.parse_boot_config(
            "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
        _ok, lines = list_cams.boot_verdict(cfg, dtbo_path=stock)
        text = "\n".join(lines)
        check("advice recommends dtoverlay -h",
              "dtoverlay -h imx477" in text, text)
        check("advice avoids the bare strings|grep trap",
              "strings " not in text, text)


def test_verdict_uses_dtoverlay_h_when_available():
    """When `dtoverlay -h` can be read, it decides — even if a stale/stock
    file sits at the path."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        stock = _vcm_dtbo(td, with_vcm=False)
        cfg = list_cams.parse_boot_config(
            "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
        real = list_cams.dtoverlay_h_params
        list_cams.dtoverlay_h_params = lambda name: {"rotation", "cam0", "vcm"}
        try:
            ok, lines = list_cams.boot_verdict(cfg, dtbo_path=stock)
        finally:
            list_cams.dtoverlay_h_params = real
        text = "\n".join(lines)
        check("dtoverlay -h result wins over the file scan", ok is True, text)
        check("the method is named in the OK line",
              "dtoverlay -h imx477" in text, text)
        check("and it says the motor can be driven",
              "focus motor can be driven" in text, text)


def test_verdict_detects_the_stock_overlay_without_vcm():
    """A correct config.txt + the STOCK dtbo is still broken: ,vcm cannot
    apply. This is what a half-finished Arducam install looks like."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        stock = _vcm_dtbo(td, with_vcm=False)
        cfg = list_cams.parse_boot_config(
            "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
        real = list_cams.dtoverlay_h_params
        list_cams.dtoverlay_h_params = lambda name: None
        try:
            ok, lines = list_cams.boot_verdict(cfg, dtbo_path=stock)
            good = _vcm_dtbo(td, with_vcm=True)
            ok2, lines2 = list_cams.boot_verdict(cfg, dtbo_path=good)
        finally:
            list_cams.dtoverlay_h_params = real
        text = "\n".join(lines)
        check("verdict fails with the stock dtbo + ,vcm", ok is False, text)
        check("verdict names it as the STOCK overlay",
              "STOCK" in text and "no vcm" in text, text)
        text2 = "\n".join(lines2)
        check("verdict passes with Arducam's dtbo", ok2 is True, text2)
        check("the file scan is named, since dtoverlay -h could not answer",
              "vcm" in text2 and "stock" in text2.lower(), text2)


def test_verdict_when_dtoverlay_h_and_the_file_disagree():
    """The user's actual Pi, 2026-10-11: the camera streams, the dtbo file
    references vcm, but `dtoverlay -h imx477` lists only the stock parameter
    names. The verdict must NOT declare the camera broken (it demonstrably
    works) and must not claim the motor is proven — it must say to verify."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        good = _vcm_dtbo(td, with_vcm=True)
        cfg = list_cams.parse_boot_config(
            "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
        real = list_cams.dtoverlay_h_params
        list_cams.dtoverlay_h_params = lambda name: {
            "rotation", "orientation", "media-controller", "cam0",
            "always-on", "sync-source", "sync-sink", "link-frequency"}
        try:
            ok, lines = list_cams.boot_verdict(cfg, dtbo_path=good)
        finally:
            list_cams.dtoverlay_h_params = real
        text = "\n".join(lines)
        check("an unexplained-but-working overlay is not a failure",
              ok is True, text)
        check("the disagreement is stated plainly",
              "does NOT advertise" in text, text)
        check("the stock-obsolete line is NOT printed as BAD",
              "BAD" not in text, text)
        check("it tells the operator to verify the lens, not to reinstall",
              "VERIFY" in text, text)


def test_verdict_handles_missing_dtbo_and_other_cameras():
    cfg = list_cams.parse_boot_config(
        "camera_auto_detect=0\n[all]\ndtoverlay=imx477,vcm\n")
    ok, lines = list_cams.boot_verdict(cfg, dtbo_path="/nonexistent/imx477.dtbo")
    text = "\n".join(lines)
    check("verdict fails when the dtbo is missing", ok is False, text)
    check("verdict names the missing file", "dtbo" in text, text)
    check("verdict reminds about a second camera",
          "imx708" in text, text)


def test_vcm_scan_is_honest_about_unreadable_files():
    check("unreadable dtbo -> None, never a guess",
          list_cams.dtbo_supports_vcm("/nonexistent/x.dtbo") is None)
    check("a directory is not a dtbo",
          list_cams.dtbo_supports_vcm("/tmp") is None)


def test_boot_report_returns_a_dict_without_a_pi():
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cfg = list_cams.print_boot_config()
    check("boot report returns a dict (possibly empty off-Pi)",
          isinstance(cfg, dict), repr(cfg))


if __name__ == "__main__":
    test_camera_auto_detect_is_read()
    test_overlay_lines_are_found_and_comments_ignored()
    test_user_config_is_diagnosed_correctly()
    test_overlay_under_the_wrong_section_is_caught()
    test_board_section_maps_models()
    test_empty_and_junk_input_never_raise()
    test_report_prints_the_two_files_that_decide_success()
    test_verdict_catches_the_real_field_failure()
    test_dtoverlay_h_output_is_parsed()
    test_verdict_advice_does_not_use_the_three_letter_trap()
    test_verdict_uses_dtoverlay_h_when_available()
    test_verdict_detects_the_stock_overlay_without_vcm()
    test_verdict_when_dtoverlay_h_and_the_file_disagree()
    test_file_checks_report_the_af_algorithm_not_just_the_filename()
    test_the_repo_local_af_tuning_is_reported_where_the_stack_reads_it()
    test_verdict_handles_missing_dtbo_and_other_cameras()
    test_vcm_scan_is_honest_about_unreadable_files()
    test_boot_report_returns_a_dict_without_a_pi()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hw-report tests passed")
