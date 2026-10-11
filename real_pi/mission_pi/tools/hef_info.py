#!/usr/bin/env python3
"""Inspect a .hef and say whether THIS Pi can use it.

Answers the three questions that otherwise surface as a mysterious failure in
the middle of a mission:

  1. does the HEF load at all on the installed HailoRT (version/arch match)?
  2. what does it expect (input tensor name/shape) and what does it emit?
  3. will our runtime parser understand those outputs?

The last one is decided by `detector.py::HailoYolo8Detector._parse`:
  * style A — NMS done inside the HEF: a tensor whose last dim is 6
    (x1,y1,x2,y2,score,class)   -> preferred (NMS on the Hailo, free on the Pi)
  * style B — raw ultralytics head (1,5,8400) cxcywh+conf -> also parsed, but
    NMS then runs on the Pi's CPU.
If neither matches, every inference silently returns no boxes — which looks
like "the model is bad" and is really "the outputs are in a shape nobody
reads". This tool says so up front.

USAGE (on the Pi, from mission_pi/)
  python3 tools/hef_info.py                                  # models/qr_yolov8n.hef
  python3 tools/hef_info.py --hef /path/to/other.hef
  python3 tools/hef_info.py --nms-config training/nms_config.json   # check a config
  python3 tools/hef_info.py --list-device                    # what the HAT says it is

Needs the Hailo runtime (`hailo_platform`) for the HEF/device parts; the
--nms-config check is pure Python and runs anywhere.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir))

#: What our parser needs, kept next to the classifier so the two cannot drift.
STYLE_A_LAST_DIM = 6
STYLE_B_HEAD = (1, 5)
STYLE_B_MIN_ANCHORS = 1000


def classify_outputs(shapes):
    """[(name, shape), ...] -> (style, detail) with style in "A"/"B"/None.

    Pure: this is the contract between the HEF we compile and the runtime
    parser, so it is tested without any Hailo hardware.
    """
    norm = []
    for name, shape in shapes or []:
        shp = tuple(int(d) for d in (shape or ()) if d is not None)
        if shp:
            norm.append((name, shp))
    # Two passes, A before B — exactly like detector.py::_parse, which scans
    # every output for the NMS tensor before falling back to the raw head. A
    # single pass would make the verdict depend on tensor order, and the
    # verdict must describe what the RUNTIME will do, not what looks tidy.
    for name, shp in norm:
        if shp[-1] == STYLE_A_LAST_DIM and len(shp) >= 2:
            return "A", ("%s %s — NMS-in-HEF (x1,y1,x2,y2,score,class); the "
                         "Hailo does the NMS" % (name, list(shp)))
    for name, shp in norm:
        if (len(shp) == 3 and shp[0] == 1 and shp[1] == STYLE_B_HEAD[1]
                and shp[2] >= STYLE_B_MIN_ANCHORS):
            return "B", ("%s %s — raw ultralytics head; parsed, but NMS then "
                         "runs on the Pi CPU" % (name, list(shp)))
    return None, ("no output looks like either supported shape — the runtime "
                  "would return NO boxes for every frame. Compile with the "
                  "zoo's yolov8n .alls (nms_postprocess), or export the raw "
                  "(1,5,8400) head.")


def validate_nms_config(cfg):
    """Check a Hailo nms_config.json against THIS project. -> (ok, lines).

    Pure so the template we ship is verified in CI rather than trusted. The
    thresholds matter for the 15 m job: the HEF's score threshold is a HARD
    floor applied before our own filtering ever sees a box, so setting it high
    deletes the small far-away QR we are specifically hunting.
    """
    lines, ok = [], True
    if not isinstance(cfg, dict):
        return False, ["not a JSON object"]
    classes = cfg.get("classes")
    dims = cfg.get("image_dims")
    scores = cfg.get("nms_scores_th")
    iou = cfg.get("nms_iou_th")

    if classes == 1:
        lines.append("OK   classes=1 (our model has exactly one class, 'qr')")
    else:
        ok = False
        lines.append("BAD  classes=%r — our checkpoint is 1-class; a mismatch "
                     "here produces boxes with the wrong/no class id and the "
                     "runtime filter (class 0 only) drops them" % (classes,))
    if dims == [640, 640]:
        lines.append("OK   image_dims=[640,640] matches the network input")
    else:
        ok = False
        lines.append("BAD  image_dims=%r — must be [640,640] to match the "
                     "ONNX input and detector.input_size" % (dims,))
    if isinstance(scores, (int, float)):
        if scores > 0.35:
            ok = False
            lines.append("BAD  nms_scores_th=%s is above our detection floor — "
                         "the HEF would delete weak boxes before our code sees "
                         "them. Use 0.25 (or lower; tune with tools/qr_bench.py)"
                         % scores)
        else:
            lines.append("OK   nms_scores_th=%s (our conf floor is 0.25-0.35, "
                         "so the HEF is not the bottleneck)" % scores)
    else:
        ok = False
        lines.append("BAD  nms_scores_th missing")
    if isinstance(iou, (int, float)):
        ok = ok and True
        lines.append("OK   nms_iou_th=%s" % iou)
    else:
        lines.append("note nms_iou_th missing — the zoo default is 0.7")
    if cfg.get("bbox_decoders"):
        lines.append("OK   bbox_decoders present (%d)"
                     % len(cfg.get("bbox_decoders") or []))
    else:
        lines.append("note no bbox_decoders — the zoo's yolov8n .alls supplies "
                     "them; that is fine if you use the zoo script")
    return ok, lines


def _shape_of(info):
    try:
        shp = list(getattr(info, "shape", []) or [])
    except Exception:
        shp = []
    return [int(d) if d is not None else None for d in shp]


def inspect_hef(path, input_size=640):
    """Load the HEF through the Hailo runtime and report what it is."""
    print("[hef] file: %s (%d bytes)" % (path, os.path.getsize(path)
                                         if os.path.isfile(path) else -1))
    try:
        from hailo_platform import HEF
    except Exception as e:
        print("BAD  the Hailo runtime is not importable here (%r)." % e)
        print("     On the Pi: sudo apt install hailo-all (or hailo-h10-all), "
              "then re-run. Off-Pi this tool can still do --nms-config.")
        return 2
    try:
        hef = HEF(path)
    except Exception as e:
        print("BAD  the HEF did not load: %r" % e)
        print("     Most likely the architecture: this HEF was compiled for a "
              "different hat (hailo8 vs hailo8l) or by a newer Dataflow "
              "Compiler than the installed HailoRT. Check both:")
        print("       hailortcli fw-control identify   # Device Architecture")
        print("       hailortcli --version")
        return 2
    arch = None
    for attr in ("get_arch", "arch"):
        try:
            v = getattr(hef, attr)
            arch = v() if callable(v) else v
            if arch:
                break
        except Exception:
            arch = None
    print("[hef] architecture: %s" % (arch or "unknown (HailoRT did not say)"))

    ins = []
    try:
        ins = [(i.name, _shape_of(i)) for i in hef.get_input_vstream_infos()]
    except Exception as e:
        print("note could not list input vstreams: %r" % e)
    outs = []
    try:
        outs = [(o.name, _shape_of(o)) for o in hef.get_output_vstream_infos()]
    except Exception as e:
        print("note could not list output vstreams: %r" % e)

    print("[hef] inputs : %s" % (ins or "?"))
    print("[hef] outputs: %s" % (outs or "?"))
    ok = True
    if ins:
        shp = ins[0][1]
        if len(shp) >= 3 and shp[-1] == 3 and shp[-2] == input_size:
            print("OK   input is %dx%d RGB — matches detector.input_size"
                  % (input_size, input_size))
        elif len(shp) >= 3 and shp[1] == 3 and shp[2] == input_size:
            print("OK   input is 3x%dx%d NCHW — the runtime feeds NHWC uint8; "
                  "HailoRT converts, no action needed" % (input_size, input_size))
        else:
            print("note input shape %s is not %dx%d — check "
                  "config.yaml detector.input_size" % (shp, input_size,
                                                       input_size))
    style, detail = classify_outputs(outs)
    if style:
        print("OK   output style %s: %s" % (style, detail))
    else:
        ok = False
        print("BAD  %s" % detail)
    print("[hef] runtime verdict: %s" % ("USABLE with detector.py"
                                         if ok else "NOT USABLE as-is"))
    return 0 if ok else 3


def list_device():
    """What the HAT says it is — the value `--hw-arch` must match."""
    try:
        from hailo_platform import Device
    except Exception as e:
        print("BAD  hailo_platform not importable (%r) — on the Pi: "
              "sudo apt install hailo-all" % e)
        return 2
    try:
        devs = Device.scan()
    except Exception as e:
        print("BAD  device scan failed (%r) — check `hailortcli scan`, the "
              "PCIe overlay (dtparam=pciex1_gen=2) and rpi-eeprom" % e)
        return 2
    if not devs:
        print("BAD  no Hailo device found — `hailortcli scan` for details")
        return 2
    for d in devs:
        try:
            with Device(d) as dev:
                info = dev.get_arch() if hasattr(dev, "get_arch") else None
                print("OK   device %s architecture=%s" % (d, info))
        except Exception as e:
            print("note device %s present but not queryable: %r" % (d, e))
    print("[hef] compile with --hw-arch matching the architecture above "
          "(hailo8 / hailo8l) — see training/compile_hef.md")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hef", default=os.path.join(HERE, os.pardir, "models",
                                                 "qr_yolov8n.hef"))
    ap.add_argument("--input-size", type=int, default=640)
    ap.add_argument("--nms-config", default="",
                    help="validate a Hailo nms_config.json instead")
    ap.add_argument("--list-device", action="store_true",
                    help="print what the HAT reports (for --hw-arch)")
    args = ap.parse_args()

    if args.nms_config:
        try:
            with open(args.nms_config) as f:
                cfg = json.load(f)
        except Exception as e:
            print("BAD  cannot read %s: %r" % (args.nms_config, e))
            return 2
        ok, lines = validate_nms_config(cfg)
        for ln in lines:
            print(ln)
        print("[nms] %s" % ("config is consistent with this project"
                            if ok else "config would hurt detection — fix the "
                                       "BAD lines"))
        return 0 if ok else 3
    if args.list_device:
        return list_device()
    if not os.path.isfile(args.hef):
        print("BAD  no HEF at %s" % args.hef)
        print("     Put it at models/qr_yolov8n.hef (see training/compile_hef.md) "
              "or pass --hef PATH.")
        return 2
    return inspect_hef(args.hef, args.input_size)


if __name__ == "__main__":
    sys.exit(main())
