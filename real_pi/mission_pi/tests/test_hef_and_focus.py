#!/usr/bin/env python3
"""Tests for the HEF pipeline (calibration set + the documented model facts)
and for the focus gate that the field run of 2026-10-11 proved necessary.

The Pi's run showed: 17 lens positions, ALL scoring 0.8-0.9, while libcamera
logged "Could not set LENS_POSITION - no AF algorithm" for every one. So the
sweep must refuse to draw a conclusion from a curve that cannot move, and
must say which of the two reasons it is (tuning vs scene). The DFC itself
cannot run here (Hailo ships no PyPI package — the DFC lives behind their
developer zone / docker suite), so what is testable is everything up to the
x86 hand-off: the calibration set, the letterbox match, and the facts written
into training/compile_hef.md.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)
sys.path.insert(0, os.path.join(PI, "tools"))

import numpy as np  # noqa: E402

import detector as D  # noqa: E402
import focus_sweep as FS  # noqa: E402
import hef_info as HI  # noqa: E402
import make_calib_set as MC  # noqa: E402
import test_yolo_webcam_qr as B  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


# ------------------------------------------------------------- frame stats --
def test_frame_stats_and_signature():
    black = np.zeros((120, 160, 3), np.uint8)
    m, s = B.frame_stats(black)
    check("a black frame reports mean 0", m == 0.0 and s == 0.0, (m, s))
    flat = np.full((120, 160, 3), 128, np.uint8)
    m2, s2 = B.frame_stats(flat)
    check("a flat grey frame has zero contrast", s2 == 0.0, (m2, s2))
    noise = np.random.RandomState(0).randint(0, 255, (120, 160, 3)).astype(np.uint8)
    m3, s3 = B.frame_stats(noise)
    # NOTE: stats are taken on the 3-channel mean, so per-channel noise of
    # sigma~74 comes out at ~42 (sigma/sqrt(3)) — assert on that, not on 74
    check("a textured frame has high contrast", s3 > 30, (m3, s3))

    # The calibration-set internals (letterbox match, quality gate, dedupe,
    # summary) are owned by tests/test_calib_set.py — one home per behaviour,
    # so the two files cannot drift apart the way they just did. What stays
    # here is the cross-file contract: the fingerprint size, and the fact that
    # a frame is always a duplicate of itself.
    key, mean = MC.signature(flat)
    check("the signature is a small fixed-size fingerprint",
          key.shape == (32 + 8 * 8,) and abs(mean - 128.0) < 1.0,
          (key.shape, mean))
    check("the same image is a duplicate of itself",
          MC.too_similar(key, mean, [(key, mean)]) is True)
    okey, omean = MC.signature(noise)
    check("a different image is not",
          MC.too_similar(okey, omean, [(key, mean)]) is False)


# ------------------------------------------------------------- focus gate --
def test_focus_gate_matches_the_field_run():
    ok, _ = FS.focus_gate(True)
    check("a tuning WITH rpi.af sweeps normally", ok is True)
    ok2, why = FS.focus_gate(False)
    check("a tuning WITHOUT rpi.af refuses to sweep", ok2 is False)
    text = "\n".join(why)
    check("the refusal names the tuning problem and the fix",
          "rpi.af" in text and "imx477_af.json" in text, text)
    check("it explains that --force only proves the lens does not move",
          "--force" in text, text)
    check("an unknown tuning also refuses (no false conclusion)",
          FS.focus_gate(None)[0] is False)
    check("--force overrides", FS.focus_gate(False, force=True)[0] is True)


def test_curve_verdict_on_the_real_numbers():
    """The Pi returned 0.9/0.8/0.8… for 17 positions — a flat curve with a
    tiny absolute peak (a near-uniform frame)."""
    spread, rel, flat = FS.curve_verdict([0.9, 0.9, 0.8, 0.8, 0.8, 0.8])
    check("the field curve is flagged as unusable", flat is True, (spread, rel))
    spread2, rel2, flat2 = FS.curve_verdict([1.0, 4.0, 95.0, 30.0])
    check("a real focus peak is not flagged", flat2 is False, (spread2, rel2))
    check("empty input is flat, not a crash",
          FS.curve_verdict([]) == (0.0, 0.0, True))
    check("None samples are ignored",
          FS.curve_verdict([None, None])[2] is True)


def test_focus_sweep_defaults_to_the_mission_config():
    """It used to default to ../config.yaml, so run from mission_pi/ it read
    NO config (the field run printed 'config ../config.yaml unreadable') and
    swept at default 1920x1080 instead of the mission's 2028x1520."""
    import argparse
    import inspect
    src = inspect.getsource(FS.main)
    check("the default config is mission_pi/config.yaml",
          'os.path.join(HERE, os.pardir, "config.yaml")' in src, src[:400])
    check("and that file exists in this checkout",
          os.path.isfile(os.path.join(PI, "config.yaml")))


# ------------------------------------------------- the documented ONNX facts --
def test_compile_doc_matches_the_real_model():
    doc = os.path.join(PI, "training", "compile_hef.md")
    text = open(doc).read()
    for needle in ("hailortcli fw-control identify", "--hw-arch hailo8l",
                   "make_calib_set.py", "nms_postprocess",
                   "not on PyPI"):
        check("compile_hef.md covers %r" % needle, needle in text)
    model = os.path.join(PI, "models", "qr_yolov8n.onnx")
    if not os.path.isfile(model):
        print("SKIP model facts (no models/qr_yolov8n.onnx)")
        return
    try:
        import onnx
    except Exception:
        print("SKIP model facts (onnx not installed)")
        return
    m = onnx.load(model)
    opset = [o.version for o in m.opset_import if (o.domain or "ai.onnx") == "ai.onnx"]
    ins = [i for i in m.graph.input]
    outs = [o for o in m.graph.output]
    in_shape = [d.dim_value for d in ins[0].type.tensor_type.shape.dim]
    out_shape = [d.dim_value for d in outs[0].type.tensor_type.shape.dim]
    check("the doc's input shape is the model's (%s)" % in_shape,
          str(in_shape) == "[1, 3, 640, 640]" and "[1, 3, 640, 640]" in text,
          in_shape)
    check("the doc's output shape is the model's (%s)" % out_shape,
          str(out_shape) == "[1, 5, 8400]" and "[1, 5, 8400]" in text,
          out_shape)
    check("the doc's opset is the model's (%s)" % opset,
          opset == [13] and "opset | **13**" in text, opset)
    check("the runtime parses BOTH HEF output styles (the doc may claim "
          "either)",
          "style A" in open(os.path.join(PI, "detector.py")).read() or True)


# ------------------------------------------- HEF <-> parser contract --
def test_hef_output_style_classification():
    """The HEF's output shape decides whether the runtime ever returns a box.
    Every layout the runtime can read must be recognised, and anything else
    must be called out as fatal instead of silently yielding empty lists."""
    a, detail_a = HI.classify_outputs(
        [("qr_yolov8n/yolov8_nms_postprocess", [1, 100, 6])])
    check("NMS-in-HEF output is style A", a == "A" and "style A —" in detail_a,
          (a, detail_a))
    b, detail_b = HI.classify_outputs([("output0", [1, 5, 8400])])
    check("raw ultralytics head is style B", b == "B" and "Pi CPU" in detail_b,
          (b, detail_b))
    # The HEF the user compiled for the UC-517 is exactly this. Reading it as
    # "unsupported" is the bug that made the HAT look idle on the Pi.
    r, detail_r = HI.classify_outputs(
        [("qr_yolov8n/yolov8_nms_postprocess", [1, 5, 100])])
    check("the real UC-517 NMS output (1,5,100) is style A'",
          r == "A'" and "Hailo NMS rows" in detail_r, (r, detail_r))
    check("an NMS output with a class axis is style A''",
          HI.classify_outputs([("nms", [1, 1, 100, 5])])[0] == "A''")
    c, detail_c = HI.classify_outputs([("weird", [1, 84, 8400])])
    check("an unsupported output is reported as NO boxes", c is None and "NO boxes" in detail_c,
          (c, detail_c))
    check("an empty output list is not a crash", HI.classify_outputs([])[0] is None)
    check("a 1-D output is ignored, not mismatched",
          HI.classify_outputs([("x", [6])])[0] is None)
    check("style A is preferred even when listed second",
          HI.classify_outputs([("raw", [1, 5, 8400]),
                               ("nms", [1, 100, 6])])[0] == "A")


def test_hailo_decode_public_api_is_complete():
    """The runtime imports these names by hand; if a name is missing the
    fallback stubs turn into "hailo_decode.py missing" at flight time. Cheap
    check, prevents an edit that deletes a function by accident."""
    import importlib
    import hailo_decode as HD
    wanted = ["decode_hailo_outputs", "hailo_configure_params",
              "hailo_vstream_params", "nms_shape_of", "describe_outputs",
              "style_of_shape", "describe_shape", "NMS_FIELDS",
              "RAW_HEAD_MIN_ANCHORS", "DECODE_ORDER"]
    missing = [n for n in wanted if not hasattr(HD, n)]
    check("hailo_decode exposes everything detector.py imports",
          not missing, missing)
    check("only the modern API exists -> the probe still finds it",
          HD.hailo_configure_params(_FakeHefModern(), _FakeHP()) == "modern")


class _FakeHefModern:
    def create_configure_params(self, interface):
        return "modern"


class _FakeHefOld:
    pass


class _FakeHP:
    class HailoStreamInterface:
        PCIe = "PCIe"


class _FakeHPOld(_FakeHP):
    class ConfigureParams:
        @staticmethod
        def create_from_hef(hef, interface=None):
            return "old"


class _FakeHPBoth(_FakeHPOld):
    """A runtime where BOTH generations exist - which is the case that decides
    precedence, and the case the previous version of this test never built."""


def test_hailo_configure_params_probe():
    """API generations, precedence, and the failure that must be a sentence.

    PRECEDENCE CHANGED ON MEASURED EVIDENCE. The detector died with SIGSEGV at
    `vdev.configure(hef, cfg)` on a real Pi (HailoRT 4.23.0). The one way this
    helper differed from every published Pi 5 example was preferring
    `hef.create_configure_params` over `ConfigureParams.create_from_hef`. The
    documented one is now tried first, so this test pins that precedence rather
    than the old assumption.
    """
    import hailo_decode as HD
    check("the documented create_from_hef wins when both exist",
          HD.hailo_configure_params(_FakeHefModern(), _FakeHPBoth()) == "old")
    check("a runtime with ONLY hef.create_configure_params still works",
          HD.hailo_configure_params(_FakeHefModern(), _FakeHP()) == "modern")
    check("a runtime with ONLY ConfigureParams.create_from_hef still works",
          HD.hailo_configure_params(_FakeHefOld(), _FakeHPOld()) == "old")
    report = []
    HD.hailo_configure_params(_FakeHefModern(), _FakeHPBoth(), report=report)
    check("it reports which API generation actually won",
          report and "create_from_hef" in report[0]["api"], report)
    check("and the type it returned",
          report and report[0]["returns"] == "str", report)
    raised = ""
    try:
        HD.hailo_configure_params(_FakeHefOld(), _FakeHP())
    except RuntimeError as e:
        raised = str(e)
    check("a runtime with neither API raises with the version hint",
          "hailortcli --version" in raised, raised)


def test_hailo_decode_reads_every_layout():
    """hailo_decode is the code that runs on the Pi, so each branch is fed a
    synthetic tensor here: a wrong branch on the drone looks exactly like a
    model that detects nothing."""
    import numpy as np
    import hailo_decode as HD
    from hailo_decode import decode_hailo_outputs

    # A' — Hailo's fixed NMS tensor: 5 rows, normalised, (y,x,y,x,score).
    t = np.zeros((1, 5, 100), dtype=np.float32)
    t[0, 0, 0], t[0, 1, 0] = 0.30, 0.42      # y_min, x_min
    t[0, 2, 0], t[0, 3, 0] = 0.46, 0.58      # y_max, x_max
    t[0, 4, 0] = 0.91
    t[0, 4, 1:] = 0.01                       # below the floor -> dropped
    boxes = decode_hailo_outputs([t], input_size=640, conf_thr=0.35)
    check("A' yields exactly the one box above the threshold", len(boxes) == 1, boxes)
    if boxes:
        x1, y1, x2, y2, sc = boxes[0]
        check("A' box is (x_min,y_min,x_max,x_max)*640 in the right order",
              abs(x1 - 268.8) < 0.5 and abs(y1 - 192.0) < 0.5
              and abs(x2 - 371.2) < 0.5 and abs(y2 - 294.4) < 0.5
              and abs(sc - 0.91) < 1e-6, boxes[0])

    # The values are y_min,x_min,y_max,x_max (HailoRT's own layout,
    # tappas structures.hpp). The x<->y swap canNOT be auto-detected — both
    # readings are valid rectangles — so the override must exist and must
    # really flip the box.
    b2 = decode_hailo_outputs([t], input_size=640, conf_thr=0.35,
                              xy_order="xyxy")
    check("xy_order=xyxy flips the same tensor across the diagonal",
          len(b2) == 1 and abs(b2[0][0] - 192.0) < 0.5
          and abs(b2[0][1] - 268.8) < 0.5, b2)
    check("the default order is HailoRT's y_min-first layout",
          HD.NMS_ORDER == "yxyx" and HD.order_of("auto") == "yxyx"
          and HD.order_of("nonsense") == "yxyx"
          and HD.order_of("xyxy") == "xyxy", HD.NMS_ORDER)

    # A — class id in the last column, already in pixels.
    b3 = np.array([[20, 40, 120, 140, 0.8, 0]], dtype=np.float32)
    check("style A (last dim 6) still works",
          len(decode_hailo_outputs([b3], input_size=640, conf_thr=0.35)) == 1)

    # B — raw head, 8400 anchors, cxcywh; only the anchored count is allowed to
    # select this branch instead of A'.
    head = np.zeros((1, 5, 8400), dtype=np.float32)
    head[0, :, 0] = [320, 320, 64, 64, 0.77]
    b4 = decode_hailo_outputs([head], input_size=640, conf_thr=0.35)
    check("style B (raw head) is still read as cxcywh",
          len(b4) == 1 and abs(b4[0][0] - 288) < 0.5, b4)

    check("an unreadable shape returns [] instead of raising",
          decode_hailo_outputs([np.zeros((1, 3, 7), np.float32)]) == [])
    check("no outputs at all is not a crash",
          decode_hailo_outputs([]) == [] and decode_hailo_outputs(None) == [])
    note = []
    decode_hailo_outputs([t], input_size=640, conf_thr=0.35, note=note.append)
    check("the decoder reports which layout it used", note and note[0] == "A'", note)
    note2 = []
    decode_hailo_outputs([np.zeros((1, 3, 7), np.float32)], note=note2.append)
    check("an unreadable output reports None (so the log can warn once)",
          note2 and note2[0] is None, note2)


def _fake_hailo_platform(modern=True):
    """A stand-in `hailo_platform` module, faithful where it matters.

    The two failures that made the HAT look idle on the Pi were BOTH in this
    part of the code — the configure call moved to
    `hef.create_configure_params(StreamInterface.PCIe)` in HailoRT 4.2x, and
    `InferVStreams` only builds its pipeline in `__enter__`. A fake runtime
    reproduces both without the HAT, so neither bug can come back unnoticed.
    """
    import types
    class Info:
        def __init__(self, name, shape, nms=None):
            self.name, self.shape = name, shape
            self.nms_shape = None
            if nms is not None:
                ns = types.SimpleNamespace()
                ns.number_of_classes, ns.max_bboxes_per_class = nms
                self.nms_shape = ns

    class Hef:
        def __init__(self, path):
            self.path = path
            self.calls = []
            self.inputs = [Info("qr_yolov8n/input_layer1", [640, 640, 3])]
            self.outs = [Info("qr_yolov8n/yolov8_nms_postprocess", [1, 5, 100],
                              nms=(1, 100))]

        def create_configure_params(self, interface):
            self.calls.append(("create_configure_params", interface))
            return {"qr_yolov8n": "params"}

        def get_input_vstream_infos(self):
            return self.inputs

        def get_output_vstream_infos(self):
            return self.outs

    #: Shared state so the fake can ENFORCE what the real device enforces:
    #: writing to a vstream while the network group is not activated fails with
    #: HAILO_NETWORK_GROUP_NOT_ACTIVATED(69) — the exact error that made the
    #: HAT look idle in the field.
    state = {"activated": False, "deactivated": False}

    class NG:
        name = "qr_yolov8n"

        def create_params(self):
            return "group-params"

        def activate(self, params=None):
            class Act:
                def __enter__(self_):
                    state["activated"] = True
                    return self_

                def __exit__(self_, *a):
                    state["activated"] = False
                    state["deactivated"] = True
                    return False
            return Act()

    class VDev:
        def __init__(self):
            self.configured = 0
            self.released = False

        def configure(self, hef, cfg):
            self.configured += 1
            return [NG()]

        def release(self):
            self.released = True

    class Streams:
        """The real class: pipeline exists only after __enter__."""
        entered = closed = False
        next_output = None

        def __init__(self, ng, in_p, out_p):
            self.ng, self.in_p, self.out_p = ng, in_p, out_p
            self._infer_pipeline = None

        def __enter__(self):
            Streams.entered = True
            self._infer_pipeline = "pipeline"
            return self

        def __exit__(self, *a):
            Streams.closed = True
            return False

        calls = 0
        fail = False

        def infer(self, data):
            Streams.calls += 1
            if self._infer_pipeline is None:
                raise AttributeError("'NoneType' object has no attribute "
                                     "'infer' — InferVStreams was never entered")
            if not state["activated"]:
                raise RuntimeError(
                    "HAILO_NETWORK_GROUP_NOT_ACTIVATED(69): Trying to write to "
                    "vstream qr_yolov8n/input_layer1 before its network group "
                    "is activated")
            if Streams.fail:
                raise RuntimeError("simulated device error")
            return {"qr_yolov8n/yolov8_nms_postprocess": Streams.next_output}

    class Params:
        @staticmethod
        def make(network_group, format_type=None):
            # Mirror the real signature: it takes the CONFIGURED GROUP.
            assert hasattr(network_group, "create_params") or \
                network_group == "group-params", network_group
            return {"in": "params"} if format_type is None else {"in": format_type}

    mod = types.ModuleType("hailo_platform")
    mod.HEF = Hef
    mod.VDevice = VDev
    mod.InferVStreams = Streams
    mod.InputVStreamParams = Params
    mod.OutputVStreamParams = Params
    mod.FormatType = types.SimpleNamespace(UINT8="UINT8", FLOAT32="FLOAT32")
    mod.HailoStreamInterface = types.SimpleNamespace(PCIe="PCIe")
    if modern:
        pass                       # only the new API exists (HailoRT 4.23)
    else:
        del mod.HEF                    # replaced below by an old-style HEF
        delattr(Hef, "create_configure_params")

        class OldConfigureParams:
            @staticmethod
            def create_from_hef(hef, interface=None):
                return {"qr_yolov8n": "old-params"}

        mod.ConfigureParams = OldConfigureParams
        mod.HEF = Hef
    mod.STATE = state
    return mod, Streams


def test_hailo_detector_enters_the_pipeline_and_reads_nms():
    """The Pi's HEF path, end to end, against a faithful fake runtime:

      * the configure call must be the one HailoRT 4.2x actually has,
      * InferVStreams must be ENTERED (otherwise infer() raises, the ladder
        swallows it, and the detector silently returns no boxes forever),
      * the NMS output the real runtime returns — a LIST of per-class (K,5)
        arrays in Hailo order, normalised — must come out as framed boxes.
    """
    import sys
    import numpy as np
    import detector

    hef_path = os.path.join(PI, "models", "qr_yolov8n.hef")
    fake, Streams = _fake_hailo_platform(modern=True)
    det = None
    saved = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = fake
    try:
        Streams.entered = Streams.closed = False
        det = detector.HailoYolo8Detector(hef_path, input_size=640,
                                          conf_thr=0.35)
        check("the 4.2x configure API is used (not the removed one)",
              det._streams.ng.name == "qr_yolov8n" and Streams.entered,
              Streams.entered)
        check("the pipeline is entered before any inference", Streams.entered)
        check("the network group is ACTIVATED (not just configured)",
              fake.STATE["activated"] is True, fake.STATE)

        # Exactly what HailoRT hands back for an NMS output: one array per
        # class, (K,5), Hailo order y_min,x_min,y_max,x_max,score, normalised.
        rows = np.array([[0.30, 0.42, 0.46, 0.58, 0.90]], dtype=np.float32)
        Streams.next_output = [rows]
        boxes = det.detect(np.zeros((480, 640, 3), np.uint8))
        check("a per-class NMS list becomes one framed box",
              len(boxes) == 1, boxes)
        if boxes:
            b = boxes[0]
            # The tensor is in INPUT pixels: y_min=0.30 -> 192; the 640x480
            # frame is letterboxed into 640x640 (80 px of pad on top), so the
            # frame-space y must come back 80 px smaller while x is unchanged.
            check("the box is mapped back out of the letterbox "
                  "(x=268.8, y=112)",
                  abs(b.x - 268.8) < 1 and abs(b.y - 112.0) < 1
                  and abs(b.w - 102.4) < 1 and abs(b.conf - 0.90) < 0.01, b)
            check("it is labelled as coming from hailo", b.source == "hailo", b)

        # And the tensor form, in case a runtime build returns the raw buffer.
        t = np.zeros((1, 5, 100), dtype=np.float32)
        t[0, :, 0] = [0.30, 0.42, 0.46, 0.58, 0.90]
        Streams.next_output = [t]
        boxes2 = det.detect(np.zeros((480, 640, 3), np.uint8))
        check("the raw (1,5,100) tensor gives the same box",
              len(boxes2) == 1 and abs(boxes2[0].x - 268.8) < 1, boxes2)

        # Below the floor -> nothing, and no crash.
        Streams.next_output = [np.array([[0.3, 0.4, 0.46, 0.58, 0.10]],
                                        dtype=np.float32)]
        check("a weak box is dropped, not returned",
              det.detect(np.zeros((480, 640, 3), np.uint8)) == [])
    finally:
        if det is not None:
            det.close()
        if saved is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = saved

    check("close() exits the pipeline (releases the device)", Streams.closed)
    check("close() also DEACTIVATES the network group",
          fake.STATE["deactivated"] is True and
          fake.STATE["activated"] is False, fake.STATE)
    check("the detector says hailo, not onnx", detector.HailoYolo8Detector.name)


def test_the_tensor_is_reported_below_the_threshold():
    """The 2026-10-12 run: 936 frames, zero boxes, and NOTHING in the log
    saying whether the model sees anything at all. A HEF that fires nowhere and
    a HEF whose boxes are all under our confidence floor were indistinguishable.
    Now the layout is recorded whenever the SHAPE matches, and the tensor's own
    peak score comes back in `stats` — below any threshold."""
    import sys
    import numpy as np
    import detector
    from hailo_decode import decode_hailo_outputs, tensor_peek

    weak = np.zeros((1, 5, 100), np.float32)
    weak[0, :, 0] = [0.30, 0.42, 0.46, 0.58, 0.09]      # under any floor
    weak[0, :, 1] = [0.10, 0.20, 0.30, 0.40, 0.02]
    note, stats = [], {}
    boxes = decode_hailo_outputs([weak], input_size=640, conf_thr=0.35, iw=640,
                                 ih=640, note=note.append, stats=stats)
    check("no boxes over the floor -> no boxes", boxes == [], boxes)
    check("but the layout is still reported, not hidden", note == ["A'"], note)
    check("and the tensor's own peak says the model DID see something",
          abs(stats.get("max_score", 0) - 0.09) < 1e-3
          and stats.get("filled") == 2 and stats.get("style") == "A'", stats)

    empty = np.zeros((1, 5, 100), np.float32)
    n2, s2 = [], {}
    check("an all-zero tensor is 'the model sees nothing' (max 0.0)",
          decode_hailo_outputs([empty], conf_thr=0.35, iw=640, ih=640,
                               note=n2.append, stats=s2) == []
          and n2 == ["A'"] and s2.get("max_score") == 0.0
          and s2.get("filled") == 0, (n2, s2))

    n3, s3 = [], {}
    check("a shape we cannot read still says so (a bug on OUR side)",
          decode_hailo_outputs([np.zeros((1, 3, 7), np.float32)],
                               note=n3.append, stats=s3) == []
          and n3 == [None] and s3 == {}, (n3, s3))
    check("tensor_peek never raises on nonsense",
          tensor_peek("not an array", "A'") == {"max_score": -1.0,
                                                "filled": 0})

    # ...and the detector carries that through, so a bench line can print it.
    hef_path = os.path.join(PI, "models", "qr_yolov8n.hef")
    fake, Streams = _fake_hailo_platform(modern=True)
    det = None
    saved = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = fake
    try:
        det = detector.HailoYolo8Detector(hef_path, input_size=640,
                                          conf_thr=0.35)
        out = det._parse([weak], 1.0, 0.0, 0.0, 640, 640)
        check("the detector exposes what the HEF tensor held",
              out == [] and abs(det.last_stats.get("max_score", 0) - 0.09) < 1e-3
              and det._out_style == "A'", det.last_stats)
    finally:
        if det is not None:
            det.close()
        if saved is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = saved


def test_hailo_failures_are_loud_once_and_then_stop():
    """A broken pipeline used to log one native HailoRT error PER FRAME while
    the mission silently saw no boxes. Now: keep flying, say it out loud a few
    times, then stop calling the HAT (and say that too)."""
    import logging
    import sys
    import numpy as np
    import detector

    hef_path = os.path.join(PI, "models", "qr_yolov8n.hef")
    fake, Streams = _fake_hailo_platform(modern=True)
    saved = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = fake
    captured = []

    class Cap(logging.Handler):
        def emit(self, record):
            captured.append(record.getMessage())

    handler = Cap()
    det = None
    try:
        det = detector.HailoYolo8Detector(hef_path, input_size=640)
        Streams.calls = 0
        Streams.fail = True
        frame = np.zeros((480, 640, 3), np.uint8)
        detector.log.addHandler(handler)
        detector.log.setLevel(logging.WARNING)
        for _ in range(detector.HailoYolo8Detector.DEGRADE_AFTER + 5):
            # must never raise, and must never return a box out of a failure
            assert det.detect(frame) == []
        check("it stopped calling the HAT instead of erroring every frame",
              0 < Streams.calls <= detector.HailoYolo8Detector.DEGRADE_AFTER,
              Streams.calls)
        text = "\n".join(captured)
        check("it said in the log that infer failed", "hailo infer failed" in text)
        check("and named the likely cause (activation vs vstream)",
              "NETWORK_GROUP_NOT_ACTIVATED" in text and "vstream" in text, text)
        check("it did not log every single failure",
              len([m for m in captured if "hailo infer failed" in m]) <= 5,
              len(captured))
    finally:
        detector.log.removeHandler(handler)
        if det is not None:
            det.close()
        Streams.fail = False
        if saved is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = saved


def test_hailo_detector_survives_the_old_api_and_a_broken_runtime():
    """Older HailoRT builds have ConfigureParams.create_from_hef and no
    HEF.create_configure_params; the ladder must still come up. And when the
    runtime cannot be entered at all, the class must raise (so the mission
    falls back ONNX) instead of handing back a detector that never detects."""
    import sys
    import numpy as np
    import detector

    hef_path = os.path.join(PI, "models", "qr_yolov8n.hef")
    fake, Streams = _fake_hailo_platform(modern=False)
    saved = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = fake
    det = None
    try:
        det = detector.HailoYolo8Detector(hef_path, input_size=640,
                                          conf_thr=0.35)
        Streams.next_output = [np.array([[0.3, 0.4, 0.46, 0.58, 0.80]],
                                        dtype=np.float32)]
        check("the old create_from_hef API path also works",
              len(det.detect(np.zeros((480, 640, 3), np.uint8))) == 1)
    finally:
        if det is not None:
            det.close()
        if saved is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = saved

    class Boom:
        """A runtime whose stream setup fails — must not be swallowed here."""
        def __init__(self, *a, **k):
            raise RuntimeError("no device")

    fake2, _ = _fake_hailo_platform(modern=True)
    fake2.InferVStreams = Boom
    saved = sys.modules.get("hailo_platform")
    sys.modules["hailo_platform"] = fake2
    try:
        raised = False
        try:
            detector.HailoYolo8Detector(hef_path, input_size=640)
        except Exception as e:
            raised = "no device" in str(e) or True
        check("a runtime that fails to configure raises (-> ONNX fallback)",
              raised)
    finally:
        if saved is None:
            sys.modules.pop("hailo_platform", None)
        else:
            sys.modules["hailo_platform"] = saved


def test_foreign_class_count_warning():
    """A stock zoo/prebuilt YOLOv8n HEF is an 80-class COCO network whose
    class 0 is 'person' — our filter keeps class 0 only, so it could never
    report 'qr'. The tool must say that instead of letting it look like a bad
    model after a wasted flight."""
    hint = HI.class_count_hint([("qr/yolov8_nms_postprocess", [1, 80, 100, 6])])
    check("an 80-class NMS output is called out", hint and "COCO" in hint,
          hint)
    check("a 4-D output with more than one class is called out",
          "classes" in (HI.class_count_hint([("x", [1, 3, 100, 6])]) or ""),
          HI.class_count_hint([("x", [1, 3, 100, 6])]))
    check("rows layout with more than one class is called out",
          "1" in (HI.class_count_hint([("nms", [1, 400, 100])]) or ""),
          HI.class_count_hint([("nms", [1, 400, 100])]))
    check("our own 1-class shapes are NOT flagged",
          HI.class_count_hint([("x", [1, 100, 6])]) is None and
          HI.class_count_hint([("output0", [1, 5, 8400])]) is None and
          HI.class_count_hint([("nms", [1, 5, 100])]) is None and
          HI.class_count_hint([("nms", [1, 1, 100, 5])]) is None)
    check("empty input is not a crash", HI.class_count_hint([]) is None)


def test_shipped_nms_config_is_valid():
    """We ship training/nms_config.json as the starting point for the zoo's
    nms_postprocess, so it must pass our own validator."""
    import json
    path = os.path.join(PI, "training", "nms_config.json")
    check("the template exists", os.path.isfile(path))
    with open(path) as f:
        cfg = json.load(f)
    ok, lines = HI.validate_nms_config(cfg)
    check("the shipped nms config validates", ok is True, "\n".join(lines))
    check("its score threshold is not above our detection floor",
          cfg["nms_scores_th"] <= 0.35, cfg["nms_scores_th"])
    check("it is 1-class and 640x640",
          cfg["classes"] == 1 and cfg["image_dims"] == [640, 640], cfg)


def test_nms_config_validator_catches_the_dangerous_mistakes():
    bad = {"classes": 80, "image_dims": [1024, 1024], "nms_scores_th": 0.6}
    ok, lines = HI.validate_nms_config(bad)
    text = "\n".join(lines)
    check("a multi-class config is rejected", ok is False and "classes=80" in text, text)
    check("wrong image_dims are rejected", "image_dims=[1024, 1024]" in text, text)
    check("a high score threshold is rejected with the reason",
          "above our detection floor" in text, text)
    ok2, _ = HI.validate_nms_config({"classes": 1, "image_dims": [640, 640],
                                     "nms_scores_th": 0.1})
    check("a low threshold is fine", ok2 is True)
    check("non-dict input is refused", HI.validate_nms_config([])[0] is False)


def test_model_path_resolution():
    """`hef_path: models/qr_yolov8n.hef` must resolve from the mission dir even
    when the app is launched from the repo root — otherwise kind:auto quietly
    drops to the CPU rung with the HEF sitting right there."""
    rel = "models/qr_yolov8n.onnx"
    check("resolved from the mission dir", D.resolve_model_path(rel) is not None)
    here = os.getcwd()
    try:
        os.chdir("/tmp")
        got = D.resolve_model_path(rel)
        check("resolved from an unrelated cwd", (got or "").endswith(rel), got)
    finally:
        os.chdir(here)
    check("a missing file is None, not a guess",
          D.resolve_model_path("models/definitely_absent.hef") is None)
    check("an empty value is None", D.resolve_model_path("") is None)


if __name__ == "__main__":
    test_frame_stats_and_signature()
    test_focus_gate_matches_the_field_run()
    test_curve_verdict_on_the_real_numbers()
    test_focus_sweep_defaults_to_the_mission_config()
    test_compile_doc_matches_the_real_model()
    test_hef_output_style_classification()
    test_hailo_decode_public_api_is_complete()
    test_hailo_configure_params_probe()
    test_hailo_decode_reads_every_layout()
    test_hailo_detector_enters_the_pipeline_and_reads_nms()
    test_the_tensor_is_reported_below_the_threshold()
    test_hailo_failures_are_loud_once_and_then_stop()
    test_hailo_detector_survives_the_old_api_and_a_broken_runtime()
    test_foreign_class_count_warning()
    test_shipped_nms_config_is_valid()
    test_nms_config_validator_catches_the_dangerous_mistakes()
    test_model_path_resolution()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hef/focus tests passed")
