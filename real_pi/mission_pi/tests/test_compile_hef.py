"""The HEF is a build artifact - and the tool that derives it must not lie.

The NPU runs HEF only (established: Ultralytics docs, Hailo community, Hailo's
own ONNX-Runtime EP all require DFC compilation first), so ONNX stays the
source of truth and these tests pin the one command that bridges them.
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path[:0] = [PI, os.path.join(PI, "tools")]

import compile_hef as CH  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name,
                        extra if not cond else ""))
    if not cond:
        FAILS.append(name)


def test_the_three_stages_are_the_documented_ones():
    cmds = CH.stage_commands("m.onnx", "calib/", "m.har", "m.hef",
                             hw_arch="hailo8l", alls="y.alls")
    check("parse, optimize, compile in order",
          [c[1] for c in cmds] == ["parse", "optimize", "compile"],
          repr([c[1] for c in cmds]))
    check("the HAT arch is carried through every stage",
          all("hailo8l" in c for c in cmds), repr(cmds))
    check("calibration reaches the optimize stage",
          "--calib-path" in cmds[1] and "calib/" in cmds[1], repr(cmds[1]))
    check("the model script is optional but passed when given",
          "--model-script" in cmds[1] and "y.alls" in cmds[1], repr(cmds[1]))
    check("optimize reads the parsed har and writes the quantized one",
          "m.har" in cmds[1] and "m_quantized.har" in cmds[1], repr(cmds[1]))
    check("compile reads the QUANTIZED har",
          "m_quantized.har" in cmds[2], repr(cmds[2]))


def test_prefflight_names_what_is_missing():
    with tempfile.TemporaryDirectory() as td:
        ok, lines = CH.preflight(os.path.join(td, "nope.onnx"),
                                 os.path.join(td, "nocalib"))
        check("missing onnx and calib both reported",
              not ok and any("MISSING onnx" in l for l in lines)
              and any("MISSING calibration dir" in l for l in lines),
              repr(lines))
        onnx = os.path.join(td, "m.onnx")
        open(onnx, "w").write("x")
        calib = os.path.join(td, "calib")
        os.makedirs(calib)
        ok, lines = CH.preflight(onnx, calib)
        check("an EMPTY calib dir is called out",
              not ok and any("EMPTY" in l for l in lines), repr(lines))
        open(os.path.join(calib, "a.jpg"), "w").write("x")
        ok, lines = CH.preflight(onnx, calib)
        check("with files present only the compiler note remains",
              ok and all("compiler" in l for l in lines), repr(lines))


def test_an_existing_hef_is_never_silently_overwritten():
    """A working HEF is flight-critical: the tool refuses, and says why."""
    import subprocess
    with tempfile.TemporaryDirectory() as td:
        onnx = os.path.join(td, "m.onnx")
        open(onnx, "w").write("x")
        calib = os.path.join(td, "calib")
        os.makedirs(calib)
        open(os.path.join(calib, "a.jpg"), "w").write("x")
        out = os.path.join(td, "m.hef")
        open(out, "w").write("PRECIOUS")
        r = subprocess.run([sys.executable,
                            os.path.join(PI, "tools", "compile_hef.py"),
                            "--onnx", onnx, "--calib", calib, "--out", out],
                           capture_output=True, text=True)
        check("the recipe path refuses to overwrite",
              r.returncode == 1 and "refusing to overwrite" in r.stdout,
              r.stdout + r.stderr)
        check("and the existing file is untouched",
              open(out).read() == "PRECIOUS")
        r2 = subprocess.run([sys.executable,
                             os.path.join(PI, "tools", "compile_hef.py"),
                             "--validate", os.path.join(td, "nope.hef")],
                            capture_output=True, text=True)
        check("validating a missing file is a clean failure, not a traceback",
              r2.returncode == 1 and "Traceback" not in r2.stderr,
              r2.stderr)


def test_validate_says_what_the_ladder_will_ask_for():
    ok, lines = CH.validate_hef(os.path.join(PI, "models", "qr_yolov8n.hef"))
    if not ok and any("not importable" in l for l in lines):
        # not line-start "SKIP": run_tests.sh counts a whole self-checking file
        # as skipped on that word, and the other checks here DID run
        print("note: validate not exercised on this host (no hailo_platform): "
              "%s" % lines[0])
        return
    check("the shipped HEF opens and names its tensors", ok, repr(lines))
    check("it names an output the ladder can read",
          ok and any("outputs" in l for l in lines), repr(lines))


if __name__ == "__main__":
    test_the_three_stages_are_the_documented_ones()
    test_prefflight_names_what_is_missing()
    test_an_existing_hef_is_never_silently_overwritten()
    test_validate_says_what_the_ladder_will_ask_for()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all compile-hef tests passed")
