"""
YOLO Ultralytics detector for laptop / SITL+Gazebo
Uses ultralytics YOLOv8 .pt model (qr_yolov8n.pt) — 1 class QR
Works on Pop!_OS laptop without Hailo, for real test with SITL+Gazebo

Config:
  detector:
    kind: custom
    custom_module: detectors.yolo_ultralytics
    model_path: models/qr_yolov8n.pt  # or best.pt
    conf_thr: 0.25
    iou_thr: 0.45
    input_size: 640

Or use kind: yolo (auto) in detector.py — will try ultralytics first

This is the YOLO you asked to run on laptop to see results.
For Gazebo SITL: use this instead of classical so A3 at 10m/15m actually detected.

Performance: ~30-50ms per 640 inference on CPU, ~100ms per tile (3x3 = 9 tiles = ~900ms per frame)
On GPU: ~10ms per tile
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

# Import base Detector and BBox from detector.py
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detector import Detector, BBox, _letterbox, _host_nms

class YoloUltralyticsDetector(Detector):
    name = "yolo_ultralytics"

    def __init__(self, model_path="models/qr_yolov8n.pt", conf_thr=0.25, iou_thr=0.45, input_size=640):
        if not _HAVE_CV2 or not _HAVE_NP:
            raise RuntimeError("yolo_ultralytics needs cv2 + numpy")
        try:
            from ultralytics import YOLO
        except Exception as e:
            raise RuntimeError(f"ultralytics not installed: pip install ultralytics ({e})")

        # Resolve path
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            model_path,
            os.path.join(base_dir, model_path),
            os.path.join(base_dir, "models/qr_yolov8n.pt"),
            os.path.join(base_dir, "models/qr_yolov8n.onnx"),
            "/tmp/runs/qr_yolov8n_small/weights/best.pt",
        ]
        found = None
        for p in candidates:
            if os.path.isfile(p):
                found = p
                break
        if not found:
            raise RuntimeError(f"YOLO model not found, tried: {candidates}")

        self.model_path = found
        self.conf_thr = float(conf_thr)
        self.iou_thr = float(iou_thr)
        self.input_size = int(input_size)
        self._lock = threading.Lock()

        log.info(f"Loading YOLO ultralytics model: {found} conf {conf_thr} iou {iou_thr}")
        self.model = YOLO(found)
        # Warmup
        try:
            dummy = np.zeros((640,640,3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False, conf=self.conf_thr, iou=self.iou_thr)
        except Exception as e:
            log.debug(f"YOLO warmup failed: {e}")

        log.info(f"YOLO ultralytics ready: {found} size={input_size}")

    def detect(self, frame_bgr):
        """Return [BBox] in full-frame coords"""
        try:
            with self._lock:
                return self._detect_locked(frame_bgr)
        except Exception as e:
            log.debug(f"yolo_ultralytics detect failed: {e}")
            return []

    def _detect_locked(self, frame_bgr):
        h,w = frame_bgr.shape[:2]
        # ultralytics handles letterbox internally, but we can pass directly
        results = self.model.predict(frame_bgr, verbose=False, conf=self.conf_thr, iou=self.iou_thr, imgsz=self.input_size)
        out = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for box in boxes:
                # box.xyxy, box.conf, box.cls
                try:
                    x1,y1,x2,y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0]) if hasattr(box, 'cls') else 0
                    # Only class 0 = qr
                    if cls != 0:
                        continue
                    if conf < self.conf_thr:
                        continue
                    # Clip to frame
                    x1 = max(0, min(w, x1))
                    y1 = max(0, min(h, y1))
                    x2 = max(0, min(w, x2))
                    y2 = max(0, min(h, y2))
                    if x2 > x1 and y2 > y1:
                        out.append(BBox(int(x1), int(y1), int(x2-x1), int(y2-y1), conf, "yolo"))
                except Exception as e:
                    log.debug(f"box parse failed: {e}")
                    continue
        return out

    def close(self):
        try:
            del self.model
        except Exception:
            pass

# Factory for detector.py custom loader
def create_detector(cfg=None):
    cfg = cfg or {}
    return YoloUltralyticsDetector(
        model_path=cfg.get("model_path", cfg.get("hef_path", "models/qr_yolov8n.pt")),
        conf_thr=cfg.get("conf_thr", 0.25),
        iou_thr=cfg.get("iou_thr", 0.45),
        input_size=cfg.get("input_size", 640)
    )

# Alias for custom_module loader expects Detector class
Detector = YoloUltralyticsDetector
