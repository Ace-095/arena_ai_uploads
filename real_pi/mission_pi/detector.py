"""Stage-1 detectors: find QR *candidate boxes* (decode happens in decoder.py).

Kinds (config `detector.kind`):
  auto       Hailo YOLO if a .hef exists and hailo_platform imports, else classical.
  hailo      YOLOv8n (fine-tuned for QR) on the AI HAT via HailoRT.
  yolo       YOLOv8n .pt or .onnx on laptop / x86 — uses ultralytics or onnxruntime
             (for SITL+Gazebo real test, replaces classical for 10m/15m A3)
             Tries: models/qr_yolov8n.pt -> models/qr_yolov8n.onnx -> detectors/yolo_ultralytics -> detectors/yolo_onnx
  classical  No model: cv2 multi-detect + pyzbar boxes. Works day one at
             short range; weak past ~8 m (see README detection math).
  custom     YOUR future model: `detector.custom_module` must define a
             `Detector` subclass (see models/README.md contract).
             Use custom_module: detectors.yolo_ultralytics (needs ultralytics)
             or detectors.yolo_onnx (needs onnxruntime) for laptop YOLO
  none       Disable stage 1 (decode-only pipeline).

The 15 m problem: with the 113-deg bottom lens an A3 QR is ~34 px in
the frame, ~5 px after a naive 640 resize — hopeless. The mission
therefore runs the YOLO stage on 3x3 TILES of the bottom frame (see
detect_tiles), where the QR is ~15 px on the network input: detectable
for a fine-tuned 1-class YOLOv8n.

YOLO on laptop: for SITL+Gazebo real test, use kind: yolo or custom yolo_ultralytics
instead of classical — A3 at 10m/15m will actually be detected.
"""
import importlib
import logging
import os
import threading
from collections import namedtuple

log = logging.getLogger("detector")

BBox = namedtuple("BBox", ["x", "y", "w", "h", "conf", "source"])

try:
    import numpy as _np
    _HAVE_NP = True
except Exception:
    _np = None
    _HAVE_NP = False

try:
    import cv2 as _cv2
    _HAVE_CV2 = True
except Exception:
    _cv2 = None
    _HAVE_CV2 = False


class Detector:
    """Detector contract (subclass this for custom models)."""

    name = "base"

    def detect(self, frame_bgr):
        """Return [BBox, ...] in FULL-FRAME pixel coords. Never raises."""
        raise NotImplementedError

    def set_thresholds(self, conf_thr=None, iou_thr=None):
        """Live re-tuning (QR presets / webcam tool k-l keys). Detectors
        that have no thresholds (classical) simply no-op."""
        if conf_thr is not None and hasattr(self, "conf_thr"):
            self.conf_thr = float(conf_thr)
        if iou_thr is not None and hasattr(self, "iou_thr"):
            self.iou_thr = float(iou_thr)

    def close(self):
        pass


# --------------------------------------------------------------------------
# Classical (no-model) detector — day-one fallback
# --------------------------------------------------------------------------
class ClassicalDetector(Detector):
    name = "classical"

    def detect(self, frame_bgr):
        try:
            from decoder import classical_boxes
        except Exception:
            import sys
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from decoder import classical_boxes
        try:
            return [BBox(x, y, w, h, c, "classical")
                    for (x, y, w, h, c) in classical_boxes(frame_bgr)]
        except Exception as e:
            log.debug("classical detect failed: %r", e)
            return []


# --------------------------------------------------------------------------
# Hailo YOLOv8 detector
# --------------------------------------------------------------------------
def _letterbox(frame, size=640):
    h, w = frame.shape[:2]
    s = min(size / w, size / h)
    nw, nh = int(w * s), int(h * s)
    img = _cv2.resize(frame, (nw, nh), interpolation=_cv2.INTER_LINEAR)
    canvas = _np.zeros((size, size, 3), dtype=_np.uint8)
    canvas[(size - nh) // 2:(size - nh) // 2 + nh,
           (size - nw) // 2:(size - nw) // 2 + nw] = img
    return canvas, s, ((size - nw) // 2, (size - nh) // 2)


def _host_nms(boxes, iou_thr=0.5):
    """boxes: [(x1,y1,x2,y2,conf), ...] -> kept subset."""
    boxes = sorted(boxes, key=lambda b: -b[4])
    keep = []
    for b in boxes:
        ok = True
        for k in keep:
            ix1, iy1 = max(b[0], k[0]), max(b[1], k[1])
            ix2, iy2 = min(b[2], k[2]), min(b[3], k[3])
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = (b[2] - b[0]) * (b[3] - b[1]) + (k[2] - k[0]) * (k[3] - k[1]) - inter
            if union > 0 and inter / union >= iou_thr:
                ok = False
                break
        if ok:
            keep.append(b)
    return keep


class HailoYolo8Detector(Detector):
    """YOLOv8n (1 class: qr) on the Hailo AI HAT.

    Handles both HEF output styles:
      * NMS-in-HEF (Hailo zoo retrain flow): a tensor with last dim 6
        [x1,y1,x2,y2,score,class] (relative or input-pixel coords).
      * raw ultralytics-style head: (1, 5, 8400) cxcywh+conf in input px.
    """
    name = "hailo"

    def __init__(self, hef_path, input_size=640, conf_thr=0.35, iou_thr=0.5):
        if not _HAVE_NP or not _HAVE_CV2:
            raise RuntimeError("hailo detector needs numpy + cv2")
        try:
            from hailo_platform import (HEF, VDevice, ConfigureParams,
                                        InputVStreamParams, OutputVStreamParams,
                                        InferVStreams, FormatType)
        except Exception as e:
            raise RuntimeError("hailo_platform import failed (%r) — "
                               "install hailo-all / hailort on the Pi" % (e,))
        if not os.path.isfile(hef_path):
            raise RuntimeError("HEF not found: %s" % hef_path)
        self.hef_path = hef_path
        self.input_size = int(input_size)
        self.conf_thr = float(conf_thr)
        self.iou_thr = float(iou_thr)
        self._lock = threading.Lock()
        hef = HEF(hef_path)
        self._vdev = VDevice()
        cfg = ConfigureParams.create_from_hef(hef, interface="PCIe")
        self._ng = self._vdev.configure(hef, cfg)[0]
        self._ng_params = self._ng.create_params()
        in_p = InputVStreamParams.make(self._ng_params, format_type=FormatType.UINT8)
        out_p = OutputVStreamParams.make(self._ng_params, format_type=FormatType.FLOAT32)
        self._streams = InferVStreams(self._ng, in_p, out_p)
        infos = hef.get_input_vstream_infos()
        self._in_name = infos[0].name if infos else None
        log.info("hailo ready: %s (in=%s size=%d)", hef_path, self._in_name, self.input_size)

    def detect(self, frame_bgr):
        try:
            with self._lock:
                return self._detect_locked(frame_bgr)
        except Exception as e:
            log.debug("hailo detect failed: %r", e)
            return []

    def _detect_locked(self, frame_bgr):
        ih, iw = frame_bgr.shape[:2]
        net_in, scale, (px, py) = _letterbox(frame_bgr, self.input_size)
        net_in = _cv2.cvtColor(net_in, _cv2.COLOR_BGR2RGB)
        res = self._streams.infer({self._in_name: _np.expand_dims(net_in, 0)}
                                  if self._in_name else net_in)
        outs = list(res.values()) if isinstance(res, dict) else [res]
        boxes = self._parse(outs, scale, px, py, iw, ih)
        return [BBox(x1, y1, x2 - x1, y2 - y1, c, "hailo")
                for (x1, y1, x2, y2, c) in boxes]

    def _parse(self, outs, scale, px, py, iw, ih):
        S = self.input_size
        # style A: NMS tensor (..., 6)
        for o in outs:
            a = _np.asarray(o)
            if a.shape[-1] == 6 and a.size >= 6:
                det = a.reshape(-1, 6)
                out = []
                for x1, y1, x2, y2, sc, cl in det:
                    if sc < self.conf_thr or int(cl) != 0:
                        continue
                    if max(x1, y1, x2, y2) <= 1.5:  # relative -> input px
                        x1, y1, x2, y2 = x1 * S, y1 * S, x2 * S, y2 * S
                    X1 = max(0, (x1 - px) / scale)
                    Y1 = max(0, (y1 - py) / scale)
                    X2 = min(iw, (x2 - px) / scale)
                    Y2 = min(ih, (y2 - py) / scale)
                    if X2 > X1 and Y2 > Y1:
                        out.append((X1, Y1, X2, Y2, float(sc)))
                return _host_nms(out, self.iou_thr)
        # style B: raw head (1, 5, 8400) cxcywh + conf, input px
        for o in outs:
            a = _np.asarray(o)
            if a.ndim == 3 and a.shape[1] == 5 and a.shape[2] > 1000:
                det = a[0].transpose(1, 0)
                out = []
                for cx, cy, w, h, sc in det:
                    if sc < self.conf_thr:
                        continue
                    x1, y1 = cx - w / 2 - px, cy - h / 2 - py
                    x2, y2 = cx + w / 2 - px, cy + h / 2 - py
                    X1, Y1 = max(0, x1 / scale), max(0, y1 / scale)
                    X2, Y2 = min(iw, x2 / scale), min(ih, y2 / scale)
                    if X2 > X1 and Y2 > Y1:
                        out.append((X1, Y1, X2, Y2, float(sc)))
                return _host_nms(out, self.iou_thr)
        log.debug("hailo: unrecognized output shapes %s", [getattr(o, "shape", "?") for o in outs])
        return []

    def close(self):
        try:
            self._streams.close()
        except Exception:
            pass
        try:
            self._vdev.release()
        except Exception:
            pass


# --------------------------------------------------------------------------
# Tiled inference (the 15 m answer) + factory
# --------------------------------------------------------------------------
def detect_tiles(detector, frame_bgr, rows=3, cols=3, overlap=0.25, iou_thr=0.5):
    """Run `detector` on rows×cols overlapping tiles, merge to frame coords."""
    h, w = frame_bgr.shape[:2]
    if rows <= 1 and cols <= 1:
        return detector.detect(frame_bgr)
    boxes = []
    for r in range(rows):
        for c in range(cols):
            x1 = int(max(0, c * w / cols - (w * overlap / cols if c else 0)))
            y1 = int(max(0, r * h / rows - (h * overlap / rows if r else 0)))
            x2 = int(min(w, (c + 1) * w / cols + (w * overlap / cols if c < cols - 1 else 0)))
            y2 = int(min(h, (r + 1) * h / rows + (h * overlap / rows if r < rows - 1 else 0)))
            tile = frame_bgr[y1:y2, x1:x2]
            if tile.size == 0:
                continue
            for b in detector.detect(tile):
                boxes.append((b.x + x1, b.y + y1, b.x + x1 + b.w, b.y + y1 + b.h, b.conf, b.source))
    merged = _host_nms([(x1, y1, x2, y2, c) for (x1, y1, x2, y2, c, s) in boxes], iou_thr)
    # reattach source of best contributor (approx: first match)
    out = []
    for (x1, y1, x2, y2, c) in merged:
        src = next((s for (X1, Y1, X2, Y2, C, s) in boxes
                    if abs(X1 - x1) < 2 and abs(Y1 - y1) < 2), "tiles")
        out.append(BBox(int(x1), int(y1), int(x2 - x1), int(y2 - y1), c, src))
    return out


def load_custom(module_path):
    """Custom-model placeholder loader. `module_path` (e.g. "myqr.det")
    must define `Detector` (a Detector subclass) or `create_detector(cfg)`
    returning one."""
    mod = importlib.import_module(module_path)
    if hasattr(mod, "create_detector"):
        return mod.create_detector()
    if hasattr(mod, "Detector") and isinstance(mod.Detector, type):
        return mod.Detector()
    raise RuntimeError("custom module %r must define Detector or create_detector()" % module_path)


def _try_yolo_laptop(cfg):
    """Try to load YOLO for laptop/SITL — pt via ultralytics, then onnx, then custom modules"""
    # 1) explicit model_path in cfg
    mp = cfg.get("model_path") or cfg.get("hef_path") or ""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if mp:
        candidates.append(mp)
        candidates.append(os.path.join(base_dir, mp))
    candidates.extend([
        os.path.join(base_dir, "models/qr_yolov8n.pt"),
        os.path.join(base_dir, "models/qr_yolov8n.onnx"),
        "models/qr_yolov8n.pt",
        "models/qr_yolov8n.onnx",
        "/tmp/runs/qr_yolov8n_small/weights/best.pt",
        "/tmp/runs/qr_yolov8n_small/weights/best.onnx",
    ])
    # real_pi: format order is config-driven (detector.prefer, default
    # ["onnx", "pt"] for the Pi 5: onnxruntime is far lighter than torch
    # on ARM — no 500MB torch RAM hit — and usually faster there; .pt via
    # ultralytics stays as the fallback). Order matters for bench/SITL too.
    try:
        prefer = [str(x).lower() for x in
                  (cfg.get("prefer") or ["onnx", "pt"])]
    except Exception:
        prefer = ["onnx", "pt"]
    for fmt in prefer:
        suffix = ".pt" if fmt.startswith("p") else ".onnx"
        for p in candidates:
            if not (p.endswith(suffix) and os.path.isfile(p)):
                continue
            try:
                if suffix == ".pt":
                    from detectors.yolo_ultralytics import YoloUltralyticsDetector
                    return YoloUltralyticsDetector(p, cfg.get("conf_thr", 0.25), cfg.get("iou_thr", 0.45), cfg.get("input_size", 640))
                from detectors.yolo_onnx import YoloOnnxDetector
                return YoloOnnxDetector(p, cfg.get("conf_thr", 0.25), cfg.get("iou_thr", 0.45), cfg.get("input_size", 640))
            except Exception as e:
                log.debug("yolo %s %s failed: %s", fmt, p, e)
    # Try importing detectors modules directly (they will search themselves)
    try:
        from detectors.yolo_ultralytics import YoloUltralyticsDetector
        return YoloUltralyticsDetector(cfg.get("model_path", "models/qr_yolov8n.pt"), cfg.get("conf_thr", 0.25), cfg.get("iou_thr", 0.45), cfg.get("input_size", 640))
    except Exception as e:
        log.debug(f"yolo_ultralytics auto failed: {e}")
    try:
        from detectors.yolo_onnx import YoloOnnxDetector
        return YoloOnnxDetector(cfg.get("model_path", "models/qr_yolov8n.onnx"), cfg.get("conf_thr", 0.25), cfg.get("iou_thr", 0.45), cfg.get("input_size", 640))
    except Exception as e:
        log.debug(f"yolo_onnx auto failed: {e}")
    return None

def get_detector(cfg):
    """cfg: dict with kind/hef_path/input_size/conf_thr/iou_thr/custom_module."""
    kind = str(cfg.get("kind", "auto")).lower()
    if kind == "auto":
        # real_pi ladder: Hailo AI HAT (.hef) -> ONNX -> .pt -> classical.
        # Every rung falls through to the next, so the Pi never ends up
        # with NO detector; main.py records the chosen rung in bring-up.
        hef = cfg.get("hef_path", "")
        if hef and os.path.isfile(hef):
            try:
                d = HailoYolo8Detector(hef, cfg.get("input_size", 640),
                                       cfg.get("conf_thr", 0.35),
                                       cfg.get("iou_thr", 0.5))
                log.info("detector auto: HAILO %s", d.name)
                return d
            except Exception as e:
                log.warning("hailo unavailable (%r) — CPU YOLO fallback", e)
        else:
            log.info("detector auto: no HEF at %r — CPU YOLO fallback", hef)
        y = _try_yolo_laptop(cfg)
        if y:
            log.info("detector auto: CPU YOLO %s", y.name)
            return y
        log.warning("detector auto: YOLO unavailable — classical fallback "
                    "(weak past ~8 m; put a model in models/ + deps)")
        return ClassicalDetector()
    if kind == "hailo":
        return HailoYolo8Detector(cfg["hef_path"], cfg.get("input_size", 640),
                                  cfg.get("conf_thr", 0.35), cfg.get("iou_thr", 0.5))
    if kind in ("yolo", "yolov8", "yolo_ultralytics", "yolo_onnx"):
        y = _try_yolo_laptop(cfg)
        if y:
            return y
        raise RuntimeError("yolo kind requested but no model found — put models/qr_yolov8n.pt or .onnx in place and pip install ultralytics or onnxruntime")
    if kind == "classical":
        return ClassicalDetector()
    if kind == "custom":
        return load_custom(cfg["custom_module"])
    if kind == "none":
        return ClassicalDetector()  # decode-only still tries classical boxes
    raise RuntimeError("unknown detector.kind: %r (use auto/hailo/yolo/classical/custom/none)" % kind)
