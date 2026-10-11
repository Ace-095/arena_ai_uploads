#!/usr/bin/env python3
"""Inspect a .hef and say whether THIS Pi can use it.

Answers the three questions that otherwise surface as a mysterious failure in
the middle of a mission:

  1. does the HEF load at all on the installed HailoRT (version/arch match)?
  2. what does it expect (input tensor name/shape) and what does it emit?
  3. will our runtime parser understand those outputs?

The last one is decided by `hailo_decode.py`, the same code the runtime uses:
  * style A    — NMS done inside the HEF, class id in the last column:
    (..., 6) = x1,y1,x2,y2,score,class
  * style A''  — same, with an explicit class axis: (b, c, N, 5)
  * style B    — raw ultralytics head (1,5,8400) cxcywh+conf: parsed, but NMS
    then runs on the Pi CPU
  * style A'   — Hailo's fixed NMS tensor, FEATURES-MAJOR rows: (b, 5*C, N) of
    (y_min, x_min, y_max, x_max, score) normalised. This is what our own
    QR HEF returns as (1, 5, 100) = 1 class x 100 boxes x 5 values.
If none matches, every inference silently returns no boxes — which looks
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
sys.path.insert(0, os.path.join(HERE, os.pardir, "mission_pi"))

try:
    import hailo_decode as HD          # the parser the runtime uses
except Exception as _e:                # pragma: no cover - needs numpy
    HD = None
    HD_ERR = _e
else:
    HD_ERR = None

#: Kept for callers/tests that import these names; the rules themselves live in
#: hailo_decode so the inspector and the runtime cannot drift apart.
STYLE_A_LAST_DIM = 6
STYLE_B_HEAD = (1, 5)
STYLE_B_MIN_ANCHORS = 1000


def classify_outputs(shapes):
    """[(name, shape), ...] -> (style, detail), style in the decoder's names.

    Delegates the shape rules to hailo_decode (the code the Pi actually runs),
    then reports the branch the RUNTIME would choose: the first style in
    DECODE_ORDER present in the outputs, so the verdict never depends on the
    order the HEF happens to list its tensors in.
    """
    if HD is None:
        return None, ("hailo_decode.py is not importable (%r) — install numpy "
                      "to get a verdict" % (HD_ERR,))
    found = []
    details = {}
    for name, shape in shapes or []:
        st = HD.style_of_shape(shape)
        if st and st not in found:
            found.append(st)
            details[st] = HD.describe_shape(name, shape)
    for st in HD.DECODE_ORDER:
        if st in found:
            return st, details[st]
    return None, ("no output matches any layout the runtime reads (A, A'', B, "
                  "A', A'-flat) — every frame would come back with NO boxes. "
                  "Compile with the zoo's yolov8n .alls (nms_postprocess) or "
                  "export the raw (1,5,8400) head.")


#: Class counts that mean "this is somebody else's network": the stock Hailo
#: zoo / Ultralytics YOLOv8n ships 80 COCO classes, where class 0 is 'person'.
#: Our runtime filter keeps class 0 only, so a COCO HEF can never report 'qr' —
#: it would look like a model that detects nothing (or everything but QR).
FOREIGN_CLASS_COUNTS = (80, 84)
#: Values per box in a Hailo NMS tensor (rows layout, style A').
NMS_ROWS = 5


def class_count_hint(shapes):
    """A warning string when the HEF looks multi-class/foreign, else None.

    Pure: shape 80 or 84 in an NMS tensor, or an (1, C, N, 6) tensor with C > 1.
    """
    for name, shape in shapes or []:
        shp = [int(d) for d in (shape or ()) if d is not None]
        if not shp:
            continue
        for d in shp[:-1]:
            if d in FOREIGN_CLASS_COUNTS:
                return ("%s has a dimension of %d — that is the COCO class "
                        "count (80) or its +4 regression form (84). This HEF "
                        "is very likely a stock YOLOv8n/COCO network, not our "
                        "1-class 'qr' model: our filter keeps class 0 only, so "
                        "it would never report a QR. Compile from "
                        "models/qr_yolov8n.onnx (training/compile_hef.md)."
                        % (name, d))
        if len(shp) == 4 and shp[-1] == STYLE_A_LAST_DIM and shp[1] > 1:
            return ("%s suggests %d classes — our model has exactly 1 ('qr'); "
                    "the runtime keeps class 0 only."
                    % (name, shp[1]))
        # (1, 5*C, N) rows layout: 5 values per box x C classes. Gate on the
        # decoder's own verdict — [1, 100, 6] also divides by 5 but is style A
        # (one class, class id in the last column) and must never be flagged.
        if (len(shp) == 3 and HD is not None and HD.style_of_shape(shp) == "A'"
                and shp[1] // NMS_ROWS > 1):
            return ("%s packs %d classes x %d boxes — our model has exactly 1 "
                    "('qr'); the runtime would read only the first class's "
                    "rows." % (name, shp[1] // NMS_ROWS, shp[2]))
    return None


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
    foreign = class_count_hint(outs)
    if foreign:
        print("WARN %s" % foreign)
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
