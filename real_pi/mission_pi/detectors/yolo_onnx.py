"""
YOLO ONNX detector for laptop / SITL+Gazebo — no torch needed, only onnxruntime
Uses qr_yolov8n.onnx (11.7MB) — 1 class QR
Lighter than ultralytics .pt, good for Pop!_OS laptop without GPU

Config:
  detector:
    kind: custom
    custom_module: detectors.yolo_onnx
    model_path: models/qr_yolov8n.onnx
    conf_thr: 0.25
    iou_thr: 0.45
    input_size: 640
"""
import os
import logging
import threading

log = logging.getLogger("detector")

try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

try:
    import numpy as np
    _HAVE_NP = True
except Exception:
    np = None
    _HAVE_NP = False

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detector import Detector, BBox, _host_nms

def _letterbox(frame, size=640):
    h,w = frame.shape[:2]
    s = min(size/w, size/h)
    nw, nh = int(w*s), int(h*s)
    img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    canvas[(size-nh)//2:(size-nh)//2+nh, (size-nw)//2:(size-nw)//2+nw] = img
    return canvas, s, ((size-nw)//2, (size-nh)//2)

class YoloOnnxDetector(Detector):
    name = "yolo_onnx"

    def __init__(self, model_path="models/qr_yolov8n.onnx", conf_thr=0.25, iou_thr=0.45, input_size=640):
        if not _HAVE_CV2 or not _HAVE_NP:
            raise RuntimeError("yolo_onnx needs cv2 + numpy")
        try:
            import onnxruntime as ort
        except Exception as e:
            raise RuntimeError(f"onnxruntime not installed: pip install onnxruntime ({e})")

        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            model_path,
            os.path.join(base_dir, model_path),
            os.path.join(base_dir, "models/qr_yolov8n.onnx"),
            "/tmp/runs/qr_yolov8n_small/weights/best.onnx",
        ]
        found = None
        for p in candidates:
            if os.path.isfile(p):
                found = p
                break
        if not found:
            raise RuntimeError(f"ONNX model not found, tried: {candidates}")

        self.model_path = found
        self.conf_thr = float(conf_thr)
        self.iou_thr = float(iou_thr)
        self.input_size = int(input_size)
        self._lock = threading.Lock()

        log.info(f"Loading YOLO ONNX model: {found}")
        self.session = ort.InferenceSession(found, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        log.info(f"YOLO ONNX ready: {found} input {self.input_name} size {input_size}")

    def detect(self, frame_bgr):
        try:
            with self._lock:
                return self._detect_locked(frame_bgr)
        except Exception as e:
            log.debug(f"yolo_onnx detect failed: {e}")
            return []

    def _detect_locked(self, frame_bgr):
        ih,iw = frame_bgr.shape[:2]
        # letterbox + BGR->RGB + normalize 0-1 + transpose to NCHW
        net_in, scale, (px, py) = _letterbox(frame_bgr, self.input_size)
        net_in = cv2.cvtColor(net_in, cv2.COLOR_BGR2RGB)
        net_in = net_in.astype(np.float32) / 255.0
        net_in = np.transpose(net_in, (2,0,1))
        net_in = np.expand_dims(net_in, 0)

        outs = self.session.run(None, {self.input_name: net_in})
        # YOLOv8 ONNX output: (1, 5, 8400) or (1, 84, 8400) etc — 1 class = 5 = cxcywh+conf
        # Our model is 1 class, so shape (1,5,8400): cx,cy,w,h,conf
        boxes = self._parse(outs, scale, px, py, iw, ih)
        return [BBox(x1, y1, x2-x1, y2-y1, c, "yolo_onnx") for (x1,y1,x2,y2,c) in boxes]

    def _parse(self, outs, scale, px, py, iw, ih):
        S = self.input_size
        for o in outs:
            a = np.asarray(o)
            # (1,5,8400) -> (5,8400) -> transpose to (8400,5)
            if a.ndim == 3 and a.shape[1] in (5, 84, 85) and a.shape[2] > 1000:
                det = a[0]
                if det.shape[0] == 5:  # 1 class
                    det = det.transpose(1,0)  # (8400,5)
                    out = []
                    for cx,cy,w,h,sc in det:
                        if sc < self.conf_thr:
                            continue
                        x1 = (cx - w/2 - px) / scale
                        y1 = (cy - h/2 - py) / scale
                        x2 = (cx + w/2 - px) / scale
                        y2 = (cy + h/2 - py) / scale
                        X1 = max(0, min(iw, x1))
                        Y1 = max(0, min(ih, y1))
                        X2 = max(0, min(iw, x2))
                        Y2 = max(0, min(ih, y2))
                        if X2 > X1 and Y2 > Y1:
                            out.append((X1,Y1,X2,Y2,float(sc)))
                    return _host_nms(out, self.iou_thr)
                elif det.shape[0] >= 84:  # COCO style 84 = 4+80, but we have 1 class -> 5
                    # (84,8400) -> first 4 cxcywh, rest class scores
                    det = det.transpose(1,0)
                    out = []
                    for row in det:
                        cx,cy,w,h = row[:4]
                        # class scores
                        scores = row[4:]
                        cls = int(np.argmax(scores))
                        sc = float(scores[cls])
                        if cls != 0 or sc < self.conf_thr:
                            continue
                        x1 = (cx - w/2 - px) / scale
                        y1 = (cy - h/2 - py) / scale
                        x2 = (cx + w/2 - px) / scale
                        y2 = (cy + h/2 - py) / scale
                        X1 = max(0, min(iw, x1))
                        Y1 = max(0, min(ih, y1))
                        X2 = max(0, min(iw, x2))
                        Y2 = max(0, min(ih, y2))
                        if X2 > X1 and Y2 > Y1:
                            out.append((X1,Y1,X2,Y2,sc))
                    return _host_nms(out, self.iou_thr)
        log.debug(f"yolo_onnx: unrecognized output shapes {[getattr(o,'shape','?') for o in outs]}")
        return []

    def close(self):
        try:
            del self.session
        except Exception:
            pass

def create_detector(cfg=None):
    cfg = cfg or {}
    return YoloOnnxDetector(
        model_path=cfg.get("model_path", cfg.get("onnx_path", "models/qr_yolov8n.onnx")),
        conf_thr=cfg.get("conf_thr", 0.25),
        iou_thr=cfg.get("iou_thr", 0.45),
        input_size=cfg.get("input_size", 640)
    )

Detector = YoloOnnxDetector
