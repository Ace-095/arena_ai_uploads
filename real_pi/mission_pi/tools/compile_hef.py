#!/usr/bin/env python3
"""One command from ONNX to a validated HEF - or the exact recipe to run elsewhere.

The Hailo NPU executes ONE format: HEF. It is the accelerator's machine code -
there is no runtime path from ONNX/PT to the HAT (Ultralytics: "HailoRT cannot
execute ONNX without first compiling it to HEF"; Hailo's own ONNX-Runtime
execution provider also requires DFC compilation first). So the honest shape of
"run YOLO on the AI HAT" is: ONNX stays the source of truth, and HEF is a
DERIVED build artifact. This tool makes deriving it one command instead of a
doc page, and - more importantly - makes the result CHECKABLE on the Pi, where
the compiler is not installed but the runtime is.

The stage commands are the ones training/compile_hef.md documents (hailo parse
-> optimize -> compile with the model zoo's yolov8n yaml and OUR calibration
set). Nothing here re-implements the compiler; it only assembles, runs and
verifies its invocations.

USAGE
  # where the Hailo compiler exists (x86_64 Linux, or Hailo's docker image):
  python3 tools/compile_hef.py --onnx models/qr_yolov8n.onnx \
      --calib training/calib --run

  # anywhere: print the exact commands (copy-paste / docker block)
  python3 tools/compile_hef.py --onnx models/qr_yolov8n.onnx --calib training/calib

  # on the Pi (no compiler, but HailoRT): prove a HEF is one the ladder can eat
  python3 tools/compile_hef.py --validate models/qr_yolov8n.hef
"""
import argparse
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
if PI not in sys.path:
    sys.path.insert(0, PI)

#: the zoo network yaml the docs compile with (classes edited to 1 there)
ZOO_YAML = ("$HAILO_MODEL_ZOO/hailo_model_zoo/cfg/networks/"
            "yolov8n.yaml")


def stage_commands(onnx, calib, har, out, hw_arch="hailo8l",
                   yaml_path=ZOO_YAML, alls=None):
    """The parse -> optimize -> compile argv triples. Pure, so tests own it."""
    cmds = [
        ["hailo", "parse", "--hw-arch", hw_arch, "--ckpt", onnx,
         "--yaml", yaml_path, "--har", har],
        ["hailo", "optimize", "--hw-arch", hw_arch, "--har", har,
         "--calib-path", calib]
        + (["--model-script", alls] if alls else [])
        + ["--output-har-path", har.replace(".har", "_quantized.har")],
        ["hailo", "compile", "--hw-arch", hw_arch,
         "--har", har.replace(".har", "_quantized.har"), "--hef", out],
    ]
    return cmds


def preflight(onnx, calib):
    """(ok, lines): what must exist before any stage can run."""
    lines = []
    ok = True
    if not os.path.isfile(onnx):
        lines.append("MISSING onnx: %s - export it first (training/README.md)"
                     % onnx)
        ok = False
    if not os.path.isdir(calib):
        lines.append("MISSING calibration dir: %s - build it with "
                     "training/make_qr_dataset.py (a few dozen frames is "
                     "enough; quantisation is measured against THESE images)"
                     % calib)
        ok = False
    elif not os.listdir(calib):
        lines.append("EMPTY calibration dir: %s" % calib)
        ok = False
    if shutil.which("hailo") is None:
        lines.append("no `hailo` compiler on PATH - this machine can only "
                     "PRINT the recipe or VALIDATE a HEF built elsewhere")
    return ok, lines


def validate_hef(path):
    """(ok, lines): on a machine WITH HailoRT, prove the HEF is loadable and
    name the tensors the detector ladder will ask for. Never raises."""
    lines = []
    if not os.path.isfile(path):
        return False, ["no such file: %s" % path]
    try:
        from hailo_platform import HEF
    except Exception as e:
        return False, ["hailo_platform not importable here (%r) - validate on "
                       "the Pi" % e]
    try:
        hef = HEF(path)
        ins = [(i.name, tuple(i.shape)) for i in hef.get_input_vstream_infos()]
        outs = [(o.name, tuple(o.shape)) for o in hef.get_output_vstream_infos()]
    except Exception as e:
        return False, ["HEF would not open: %r" % e]
    lines.append("inputs : %s" % ", ".join("%s%s" % (n, s) for n, s in ins))
    lines.append("outputs: %s" % ", ".join("%s%s" % (n, s) for n, s in outs))
    names = [n for n, _ in outs]
    nms = any("nms" in n.lower() for n in names)
    lines.append("NMS in the HEF: %s (%s)"
                 % ("yes - preferred" if nms else "no - the runtime will parse "
                    "the raw head and NMS runs on the CPU",
                    ", ".join(names)))
    return True, lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--onnx", default=os.path.join(PI, "models",
                                                    "qr_yolov8n.onnx"))
    ap.add_argument("--out", default=None,
                    help="HEF to write (default: the onnx path with .hef)")
    ap.add_argument("--calib", default=os.path.join(PI, "training", "calib"))
    ap.add_argument("--hw-arch", default="hailo8l",
                    help="hailo8l for the Pi AI HAT, hailo8 for the older HAT")
    ap.add_argument("--alls", default=None,
                    help="model-script (the docs' yolov8n.alls) if you have it")
    ap.add_argument("--run", action="store_true",
                    help="actually execute the stages (needs the compiler)")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing HEF")
    ap.add_argument("--validate", metavar="HEF",
                    help="only validate an existing HEF (works on the Pi)")
    args = ap.parse_args()
    out = args.out or os.path.splitext(args.onnx)[0] + ".hef"

    if args.validate:
        ok, lines = validate_hef(args.validate)
        for ln in lines:
            print("[hef] %s" % ln)
        return 0 if ok else 1

    ok, lines = preflight(args.onnx, args.calib)
    for ln in lines:
        print("[hef] %s" % ln)
    if os.path.isfile(out) and not args.force:
        print("[hef] %s EXISTS - refusing to overwrite without --force (a "
              "working HEF is flight-critical; validate first)" % out)
        return 1
    cmds = stage_commands(args.onnx, args.calib,
                          os.path.splitext(out)[0] + ".har", out,
                          hw_arch=args.hw_arch, alls=args.alls)
    have_compiler = shutil.which("hailo") is not None
    if args.run and have_compiler:
        import subprocess
        tmp = out + ".part"
        for c in cmds:
            if c[1] == "compile":
                c = [tmp if a == out else a for a in c]   # never half-write out
            print("[hef] $ %s" % " ".join(c))
            r = subprocess.run(c)
            if r.returncode != 0:
                print("[hef] stage failed (%d): %s" % (r.returncode, c[1]))
                return r.returncode
        if os.path.isfile(tmp):
            os.replace(tmp, out)
        vok, vlines = validate_hef(out)
        for ln in vlines:
            print("[hef] %s" % ln)
        return 0 if vok else 1
    print("[hef] recipe (run where the Hailo compiler is installed - an x86_64")
    print("[hef] Linux box or Hailo's software-suite docker image; the Pi can")
    print("[hef] only --validate):")
    for c in cmds:
        print("    %s \\" % " ".join(c[:-1]))
        print("        %s" % c[-1])
    print("[hef] then copy %s to the Pi and:" % os.path.basename(out))
    print("    python3 tools/compile_hef.py --validate %s" % out)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
