#!/usr/bin/env python3
"""Tests for the zero-box field run (2026-10-13, 1,378 frames, raw_boxes=0).

Three things are pinned here, each one a way the drone could be blind while
the logs look fine:

  1. The NMS result arrives as nested / ragged per-class lists. The decoder
     must read every array at any depth, and an EMPTY (0,5) class is a valid
     "nothing here", not a parse error.
  2. When the HAT gives up (DEGRADE_AFTER consecutive errors), the CPU model
     takes over instead of returning [] forever.
  3. The camera does its software QR boost on demand, once per frame, and the
     detector can take the raw frame (the default).

No hardware: the HAT is replaced by a detector object built without __init__.
"""
import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)

import numpy as np  # noqa: E402

import cameras as C  # noqa: E402
import detector as D  # noqa: E402
import hailo_decode as HD  # noqa: E402
import qr_camera_boost as QB  # noqa: E402

FAILS = []


def check(name, cond, extra=""):
    print("%s %s %s" % ("PASS" if cond else "FAIL", name, extra if not cond else ""))
    if not cond:
        FAILS.append(name)


# ------------------------------------------------ 1. nested / ragged NMS ----
def test_ragged_per_class_list_decodes():
    """Per-class lists of different lengths: the old code called np.asarray on
    the whole nest and raised, which the detector counted as a failure."""
    box = np.array([[0.30, 0.42, 0.46, 0.58, 0.91]], np.float32)  # y1,x1,y2,x2,s
    ragged = [box, np.zeros((0, 5), np.float32)]
    raised = ""
    boxes = []
    try:
        boxes = HD.decode_hailo_outputs([ragged], input_size=640, conf_thr=0.35)
    except Exception as e:  # pragma: no cover - the failure we are guarding
        raised = repr(e)
    check("ragged per-class list does not raise", not raised, raised)
    check("ragged list yields the one real box", len(boxes) == 1, boxes)


def test_batch_wrapped_nest_decodes():
    """[[ per-class ]] — one extra batch level, as some runtimes return it."""
    box = np.array([[0.30, 0.42, 0.46, 0.58, 0.91]], np.float32)
    boxes = HD.decode_hailo_outputs([[[box]]], input_size=640, conf_thr=0.35)
    check("batch-wrapped nest yields the box", len(boxes) == 1, boxes)


def test_empty_class_is_a_valid_answer():
    """A (0,5) class row is 'nothing detected' and must come back as [] with
    the layout still recognised — not as an unreadable output."""
    note = []
    boxes = HD.decode_hailo_outputs([[np.zeros((0, 5), np.float32)]],
                                    input_size=640, conf_thr=0.35,
                                    note=note.append)
    check("empty (0,5) gives no boxes", boxes == [], boxes)
    peek = HD.tensor_peek(np.zeros((0, 5), np.float32), "A'")
    check("tensor_peek on an empty tensor reports max_score 0.0 (not -1)",
          peek["max_score"] == 0.0 and peek["filled"] == 0, peek)


def test_describe_result_names_the_structure():
    """The one-line log must show nesting and shape, so a new layout can be
    fixed from a single pasted line."""
    text = HD.describe_result([[np.zeros((0, 5), np.float32)]])
    check("describe_result shows the nesting and the empty shape",
          "list[1]" in text and "(0, 5)" in text, text)
    check("describe_result survives an odd object",
          isinstance(HD.describe_result(object()), str))


# ----------------------------------------------- 2. fallback after degrade --
class _Blind(D.HailoYolo8Detector):
    """A HAT object whose inference always fails — no hardware needed."""

    def __init__(self):
        self.name = "hailo-test"
        self._lock = threading.Lock()
        self._fails = 0
        self.last_stats = {}
        self._seen_struct = True
        self.fallback_factory = None
        self._fallback = None
        self._fallback_tried = False
        self.calls = 0

    def _detect_locked(self, frame_bgr):
        self.calls += 1
        raise RuntimeError("simulated HAT error")


class _CpuStub(object):
    name = "cpu-stub"

    def __init__(self):
        self.seen = 0

    def detect(self, frame):
        self.seen += 1
        return ["cpu-box"]


def test_fallback_takes_over_after_degrade():
    """After DEGRADE_AFTER failures the HAT is no longer called, and the CPU
    detector answers every frame; `using` says so for the bench."""
    d = _Blind()
    cpu = _CpuStub()
    made = []

    def factory():
        made.append(cpu)
        return cpu

    d.fallback_factory = factory
    frame = np.zeros((8, 8, 3), np.uint8)
    out = []
    for _ in range(D.HailoYolo8Detector.DEGRADE_AFTER + 1):
        out.append(d.detect(frame))
    check("the first 20 frames return [] while the HAT is still tried",
          all(o == [] for o in out[:20]), out[:20])
    check("the 21st frame is answered by the CPU fallback",
          out[20] == ["cpu-box"], out[20])
    check("the HAT is not called after degrade (no per-frame retries)",
          d.calls == D.HailoYolo8Detector.DEGRADE_AFTER, d.calls)
    for _ in range(5):
        d.detect(frame)
    check("the fallback is built once, not per frame", len(made) == 1, len(made))
    check("the fallback is used for every frame after degrade",
          cpu.seen == 6, cpu.seen)
    check("using reports the fallback by name",
          d.using == "fallback:cpu-stub", d.using)


def test_no_fallback_means_no_boxes_not_a_crash():
    """With no CPU fallback configured the old behaviour remains: [] and a
    loud note, never an exception into the mission loop."""
    d = _Blind()
    frame = np.zeros((8, 8, 3), np.uint8)
    out = [d.detect(frame) for _ in range(25)]
    check("no fallback -> [] forever, no exception",
          all(o == [] for o in out))
    check("no fallback -> using is still 'hailo'", d.using == "hailo", d.using)


def test_broken_fallback_factory_is_reported_once():
    """If the CPU model itself cannot load, say so once and keep going."""
    d = _Blind()

    def broken():
        raise RuntimeError("no onnx runtime")

    d.fallback_factory = broken
    frame = np.zeros((8, 8, 3), np.uint8)
    for _ in range(30):
        assert d.detect(frame) == []
    check("a broken fallback is tried once only", d._fallback_tried is True)


# --------------------------------------- 3. lazy enhance, once per frame -----
class _CamProbe(C.Camera):
    def __init__(self, detect_raw=True):
        self.name = "cam-probe"
        self.qr_boost = {"detect_raw": detect_raw,
                         "qr_software_enhance": True,
                         "qr_enhance_mode": "qr_boost"}
        self._lock = threading.Lock()
        self._frame_raw = "RAW-1"
        self._frame_ts = 1.0
        self._frames = 1
        self._enh_cache = None


def test_enhance_runs_lazily_once_per_frame():
    """The boost is computed when a caller asks for the boosted frame, once per
    frame count, and never for a raw request."""
    calls = []
    real = QB.enhance_qr_frame
    QB.enhance_qr_frame = lambda f, mode=None: (calls.append(f) or ("E:" + f))
    try:
        cam = _CamProbe(detect_raw=False)
        check("latest(raw=True) never boosts",
              cam.latest(raw=True)[0] == "RAW-1" and calls == [], calls)
        a = cam.latest()[0]
        b = cam.latest(detect=True)[0]
        check("two asks for the same frame boost it once",
              a == "E:RAW-1" and b == "E:RAW-1" and len(calls) == 1, calls)
        cam._frame_raw, cam._frames = "RAW-2", 2
        check("a new frame count is boosted again",
              cam.latest()[0] == "E:RAW-2" and len(calls) == 2, calls)
    finally:
        QB.enhance_qr_frame = real


def test_detect_raw_default_skips_the_boost():
    """Default detect_raw=True: the detector never pays for the boost."""
    calls = []
    real = QB.enhance_qr_frame
    QB.enhance_qr_frame = lambda f, mode=None: (calls.append(f) or "E")
    try:
        cam = _CamProbe(detect_raw=True)
        check("detect=True with detect_raw returns the raw frame",
              cam.latest(detect=True)[0] == "RAW-1" and calls == [], calls)
        check("the shipped default is detect_raw=True",
              C.Camera.QR_BOOST_DEFAULTS.get("detect_raw") is True
              if hasattr(C.Camera, "QR_BOOST_DEFAULTS") else
              C._BaseCamera.QR_BOOST_DEFAULTS.get("detect_raw") is True)
    finally:
        QB.enhance_qr_frame = real


if __name__ == "__main__":
    test_ragged_per_class_list_decodes()
    test_batch_wrapped_nest_decodes()
    test_empty_class_is_a_valid_answer()
    test_describe_result_names_the_structure()
    test_fallback_takes_over_after_degrade()
    test_no_fallback_means_no_boxes_not_a_crash()
    test_broken_fallback_factory_is_reported_once()
    test_enhance_runs_lazily_once_per_frame()
    test_detect_raw_default_skips_the_boost()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    print("all hailo-nesting / fallback / lazy-enhance tests passed")
