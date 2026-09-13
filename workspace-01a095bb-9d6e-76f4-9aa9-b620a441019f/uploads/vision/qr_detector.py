"""High-Performance Aerial QR Code Detector using ONNX Runtime (YOLO) with pyzbar fallback."""

import logging
import cv2
import numpy as np
from collections import deque
from typing import Tuple, Optional, List, Any

logger = logging.getLogger(__name__)


class QRDetector:
    """Detect and locate QR codes in video frames using YOLO ONNX + pyzbar fallback.

    CHANGE 4: in addition to the boolean ``detect()`` API (kept for backward
    compatibility), this detector now computes a per-frame *confidence* score in
    [0, 1] and tracks temporal consistency across recent detections. The score
    fuses: (a) decoding success, (b) target pixel size vs the minimum gate,
    (c) detection-quality margins, and (d) agreement of the current detection's
    pixel location with the recent track (rejects single-frame hallucinations).
    The FSM (INITIAL_SCAN / SEARCH_SQUARE) requires several consecutive
    confident frames before committing to ALIGNMENT, which dramatically reduces
    false positives and stabilises the hand-off.
    """

    def __init__(
        self,
        min_area: int = 1600,
        min_width_px: int = 80,
        qr_size_cm: float = 21.0,
        fov_horizontal_deg: float = 66.0,
        use_onnx: bool = True,
        model_path: str = "models/qr_yolo.onnx",
        input_size: int = 320,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
    ):
        self.min_area = min_area
        self.min_width_px = min_width_px
        self.qr_size_cm = qr_size_cm
        self.fov_horizontal_deg = fov_horizontal_deg
        self.use_onnx = use_onnx
        self.model_path = model_path
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        # Adaptive lighting history
        self.brightness_history = deque(maxlen=20)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Detection history for confidence filtering
        self.detection_history = deque(maxlen=5)

        # Track last successfully matched coordinates
        self.last_center = None
        self.last_bbox = None

        # CHANGE 4: confidence / temporal tracking state
        # Rolling window of recent detection centers (px) for consistency check.
        self._track_window = deque(maxlen=5)
        # Most recent confidence + track metadata (exposed for diagnostics).
        self.last_confidence: float = 0.0
        self._last_track_jitter_px: float = float("inf")

        # ONNX Runtime session
        self._session = None
        self._input_name = None
        self._output_names = None
        self._initialized = False

        if self.use_onnx:
            self._init_onnx()

    def _init_onnx(self):
        """Initialize ONNX Runtime session for YOLO inference."""
        try:
            import onnxruntime as ort

            providers = ["CPUExecutionProvider"]
            # Try to use AI HAT (Hailo/V4L2) if available
            try:
                import onnxruntime_extensions as ort_ext

                providers.insert(0, "VitisAIExecutionProvider")
            except ImportError:
                pass

            self._session = ort.InferenceSession(self.model_path, providers=providers)
            self._input_name = self._session.get_inputs()[0].name
            self._output_names = [o.name for o in self._session.get_outputs()]
            self._initialized = True
            logger.info(f"ONNX QR detector initialized: {self.model_path} (providers: {providers})")
        except Exception as e:
            logger.warning(f"ONNX init failed ({e}), falling back to pyzbar")
            self.use_onnx = False
            self._initialized = False

    def _track_consistency(self, center) -> float:
        """Return a [0,1] consistency score for a new detection center.

        Compares the candidate center against the recent track of confirmed
        detections. High agreement (small mean distance) → high score. A cold
        track (no history) returns 0.5 (neutral) so a first valid detection is
        not penalised; a wildly jumping detection scores low.
        """
        if center is None:
            return 0.0
        if len(self._track_window) == 0:
            return 0.5
        dists = [
            float(np.hypot(center[0] - c[0], center[1] - c[1])) for c in self._track_window
        ]
        mean_jitter = sum(dists) / len(dists)
        self._last_track_jitter_px = mean_jitter
        # Decay: full credit at 30 px mean offset, none at 150 px.
        score = max(0.0, 1.0 - (mean_jitter - 30.0) / 120.0)
        return min(1.0, score)

    def _confidence(self, decoded_ok: bool, bbox_width_px: int, center) -> float:
        """Fuse sub-scores into a single [0,1] detection confidence (CHANGE 4)."""
        # Size margin: how far above the minimum gate is the detection?
        size_ratio = bbox_width_px / max(1, self.min_width_px)
        size_score = min(1.0, size_ratio / 2.0)  # 1.0 once 2x the gate
        # Decode readiness: a QR pyzbar actually decoded is strong evidence.
        decode_score = 1.0 if decoded_ok else 0.45
        # Temporal agreement with the recent track.
        track_score = self._track_consistency(center)

        confidence = (0.5 * decode_score) + (0.3 * size_score) + (0.2 * track_score)
        return float(min(1.0, max(0.0, confidence)))

    def _preprocess(self, frame: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """Letterbox resize + normalize for YOLO input. Returns (input_tensor, scale, padding)."""
        h, w = frame.shape[:2]
        scale = self.input_size / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)

        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Letterbox to square
        top = (self.input_size - new_h) // 2
        bottom = self.input_size - new_h - top
        left = (self.input_size - new_w) // 2
        right = self.input_size - new_w - left

        padded = cv2.copyMakeBorder(
            resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )

        # Normalize + CHW
        input_tensor = padded.astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1))  # HWC -> CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch dim

        return input_tensor, scale, (left, top)

    def _postprocess(
        self, outputs: np.ndarray, scale: float, pad: Tuple[int, int], orig_shape: Tuple[int, int]
    ) -> List[Tuple[float, float, float, float, float]]:
        """Parse YOLO output: NMS, scale back to original frame. Returns list of (cx, cy, w, h, conf)."""
        # YOLOv8/v11 output: (1, 84, 8400) or similar - [x, y, w, h, conf, class...]
        # Assuming single-class QR model: [x_center, y_center, width, height, confidence]
        preds = outputs[0]  # (84, 8400) or (8400, 84)
        if preds.shape[0] > preds.shape[1]:
            preds = preds.T  # Ensure (N, 84)

        boxes = []
        for pred in preds:
            conf = pred[4]
            if conf < self.conf_threshold:
                continue

            # xywh (normalized to input_size)
            cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]

            # Convert to pixel coords in padded input
            cx_px = cx * self.input_size
            cy_px = cy * self.input_size
            w_px = w * self.input_size
            h_px = h * self.input_size

            # Remove padding
            cx_px -= pad[0]
            cy_px -= pad[1]

            # Scale back to original frame
            cx_px /= scale
            cy_px /= scale
            w_px /= scale
            h_px /= scale

            # Clip to frame bounds
            fw, fh = orig_shape[1], orig_shape[0]
            cx_px = np.clip(cx_px, 0, fw - 1)
            cy_px = np.clip(cy_px, 0, fh - 1)
            w_px = min(w_px, fw - cx_px)
            h_px = min(h_px, fh - cy_px)

            if w_px >= self.min_width_px and h_px >= self.min_width_px:
                boxes.append((cx_px, cy_px, w_px, h_px, float(conf)))

        # NMS
        if not boxes:
            return []

        boxes_np = np.array(boxes)
        # Sort by confidence
        order = boxes_np[:, 4].argsort()[::-1]
        keep = []
        while len(order) > 0:
            i = order[0]
            keep.append(i)
            if len(order) == 1:
                break
            # IoU with remaining
            xx1 = np.maximum(boxes_np[i, 0] - boxes_np[i, 2] / 2, boxes_np[order[1:], 0] - boxes_np[order[1:], 2] / 2)
            yy1 = np.maximum(boxes_np[i, 1] - boxes_np[i, 3] / 2, boxes_np[order[1:], 1] - boxes_np[order[1:], 3] / 2)
            xx2 = np.minimum(boxes_np[i, 0] + boxes_np[i, 2] / 2, boxes_np[order[1:], 0] + boxes_np[order[1:], 2] / 2)
            yy2 = np.minimum(boxes_np[i, 1] + boxes_np[i, 3] / 2, boxes_np[order[1:], 1] + boxes_np[order[1:], 3] / 2)

            w_int = np.maximum(0, xx2 - xx1)
            h_int = np.maximum(0, yy2 - yy1)
            inter = w_int * h_int
            area_i = boxes_np[i, 2] * boxes_np[i, 3]
            area_o = boxes_np[order[1:], 2] * boxes_np[order[1:], 3]
            iou = inter / (area_i + area_o - inter + 1e-6)

            order = order[1:][iou <= self.iou_threshold]

        return [tuple(boxes_np[i]) for i in keep]

    def _detect_onnx(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]], float]:
        """Run YOLO ONNX inference. Returns (found, bbox, center, raw_confidence)."""
        if not self._initialized:
            return False, None, None, 0.0

        h, w = frame.shape[:2]
        input_tensor, scale, pad = self._preprocess(frame)

        try:
            outputs = self._session.run(self._output_names, {self._input_name: input_tensor})
            detections = self._postprocess(outputs[0], scale, pad, (h, w))

            if not detections:
                return False, None, None, 0.0

            # Take highest confidence detection
            cx, cy, bw, bh, conf = max(detections, key=lambda x: x[4])

            # Convert to bbox format (4, 2) polygon
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            bbox = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32)
            center = (int(cx), int(cy))

            logger.debug(f"ONNX QR detect: {bw:.0f}x{bh:.0f}px @ ({cx:.0f},{cy:.0f}) conf={conf:.2f}")
            return True, bbox, center, conf

        except Exception as e:
            logger.debug(f"ONNX inference error: {e}")
            return False, None, None, 0.0

    def _detect_pyzbar(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]], float]:
        """Fallback to original pyzbar multi-stage detection. Returns (found, bbox, center, raw_confidence)."""
        from pyzbar import pyzbar

        h, w = frame.shape[:2]
        gray = self._adapt_preprocessing(frame)

        # Stage 1: Multi-scale direct scan
        for scale_factor in [1.0, 0.75, 0.5]:
            scaled_gray = gray
            if scale_factor != 1.0:
                scaled_gray = cv2.resize(
                    gray, (int(w * scale_factor), int(h * scale_factor)), interpolation=cv2.INTER_AREA
                )

            barcodes = pyzbar.decode(scaled_gray)
            if barcodes:
                for barcode in barcodes:
                    found, bbox, center = self._process_barcode_result(barcode, scale_factor)
                    if found:
                        return True, bbox, center, 1.0

        # Stage 2: ROI candidates
        candidate_rois = self._find_candidate_regions(gray)
        for rx, ry, rw, rh in candidate_rois:
            roi = gray[ry : ry + rh, rx : rx + rw]
            if roi.size == 0:
                continue

            zoom_factor = 800.0 / max(rw, rh)
            if zoom_factor > 1.0:
                zoomed_roi = cv2.resize(
                    roi, (int(rw * zoom_factor), int(rh * zoom_factor)), interpolation=cv2.INTER_CUBIC
                )
            else:
                zoomed_roi = roi
                zoom_factor = 1.0

            blur = cv2.GaussianBlur(zoomed_roi, (0, 0), 1.0)
            sharpened_roi = cv2.addWeighted(zoomed_roi, 2.0, blur, -1.0, 0)

            barcodes = pyzbar.decode(sharpened_roi)
            if barcodes:
                for barcode in barcodes:
                    bx = rx + int(barcode.rect.left / zoom_factor)
                    by = ry + int(barcode.rect.top / zoom_factor)
                    bw = int(barcode.rect.width / zoom_factor)
                    bh = int(barcode.rect.height / zoom_factor)

                    if bw >= self.min_width_px:
                        cx = bx + bw // 2
                        cy = by + bh // 2
                        bbox_pts = np.array(
                            [[bx, by], [bx + bw, by], [bx + bw, by + bh], [bx, by + bh]], dtype=np.int32
                        )
                        return True, bbox_pts, (cx, cy), 0.8

        # Stage 3: Adaptive threshold fallbacks
        for thresh_method in [cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.ADAPTIVE_THRESH_MEAN_C]:
            thresh = cv2.adaptiveThreshold(gray, 255, thresh_method, cv2.THRESH_BINARY, 21, 10)
            barcodes = pyzbar.decode(thresh)
            if barcodes:
                for barcode in barcodes:
                    found, bbox, center = self._process_barcode_result(barcode, 1.0)
                    if found:
                        return True, bbox, center, 0.7

        return False, None, None, 0.0

    def detect(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]]]:
        """
        Scan frame for QR codes using ONNX YOLO (primary) + pyzbar (fallback).

        Args:
            frame: Input image frame (BGR format)

        Returns:
            found: Boolean indicating successful detection
            bbox: (4, 2) NumPy array of bounding box points or None
            center: (cx, cy) center coordinate tuple in pixels or None
        """
        if frame is None or frame.size == 0:
            self._miss()
            return False, None, None

        # Try ONNX first
        if self.use_onnx and self._initialized:
            found, bbox, center, raw_conf = self._detect_onnx(frame)
        else:
            found, bbox, center, raw_conf = False, None, None, 0.0

        # Fallback to pyzbar
        if not found:
            found, bbox, center, raw_conf = self._detect_pyzbar(frame)

        if found:
            bbox_width_px = int(np.max(bbox[:, 0]) - np.min(bbox[:, 0]))
            # Use raw ONNX confidence if available, else decoded_ok proxy
            decoded_ok = raw_conf > 0.5
            return self._accept(decoded_ok, bbox, center, bbox_width_px=bbox_width_px)

        self._miss()
        return False, None, None

    def _accept(self, decoded_ok: bool, bbox: np.ndarray, center, bbox_width_px: int = 0) -> tuple:
        """Record a successful detection, update confidence + track, and return it."""
        if bbox_width_px <= 0 and bbox is not None:
            xs = bbox[:, 0]
            bbox_width_px = int(xs.max() - xs.min())
        self.detection_history.append(True)
        self._track_window.append((int(center[0]), int(center[1])))
        self.last_center = center
        self.last_bbox = bbox
        self.last_confidence = self._confidence(decoded_ok, bbox_width_px, center)
        logger.debug(
            f"QR accept: conf={self.last_confidence:.2f} w={bbox_width_px}px jitter={self._last_track_jitter_px:.0f}px decoded={decoded_ok}"
        )
        return True, bbox, center

    def _miss(self):
        """Record a no-detection frame (decays the temporal track)."""
        self.detection_history.append(False)
        self.last_confidence = 0.0

    def estimate_distance(self, pixel_width: int, total_width_px: int) -> float:
        """Calculate distance from camera to QR code based on focal lengths."""
        fov_horizontal_rad = np.radians(self.fov_horizontal_deg)
        focal_length_px = (total_width_px / 2.0) / np.tan(fov_horizontal_rad / 2.0)
        real_width_m = self.qr_size_cm / 100.0
        distance = (real_width_m * focal_length_px) / pixel_width
        return float(distance)

    def _adapt_preprocessing(self, frame: np.ndarray) -> np.ndarray:
        """Adapt frame preprocessing dynamically based on scene brightness."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)
        self.brightness_history.append(avg_brightness)

        if len(self.brightness_history) >= 10:
            mean_brightness = np.mean(self.brightness_history)
            brightness_std = float(np.std(self.brightness_history))
            if mean_brightness > 180:
                enhanced = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
                enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
            elif mean_brightness < 80:
                enhanced = cv2.fastNlMeansDenoising(gray, None, h=7, templateWindowSize=7, searchWindowSize=21)
                enhanced = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8)).apply(enhanced)
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                enhanced = cv2.filter2D(enhanced, -1, kernel)
            elif brightness_std > 40:
                enhanced = cv2.createCLAHE(clipLimit=2.8, tileGridSize=(8, 8)).apply(gray)
            else:
                enhanced = self.clahe.apply(gray)
        else:
            enhanced = self.clahe.apply(gray)

        return enhanced

    def _find_candidate_regions(self, gray: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Identify candidate rectangular high-contrast contours (potential QR blocks)."""
        h, w = gray.shape
        edges1 = cv2.Canny(gray, 50, 150)
        edges2 = cv2.Canny(gray, 30, 100)
        edges = cv2.bitwise_or(edges1, edges2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        dilated = cv2.dilate(closed, kernel, iterations=1)

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000 or area > (w * h * 0.7):
                continue

            rx, ry, rw, rh = cv2.boundingRect(cnt)
            aspect_ratio = float(rw) / rh if rh > 0 else 0

            if 0.5 < aspect_ratio < 2.0:
                roi = gray[ry : ry + rh, rx : rx + rw]
                if roi.size > 0 and np.std(roi) > 40:
                    pad = 20
                    bx = max(0, rx - pad)
                    by = max(0, ry - pad)
                    bw = min(w - bx, rw + 2 * pad)
                    bh = min(h - by, rh + 2 * pad)
                    candidates.append((bx, by, bw, bh))

        return candidates[:5]

    def _process_barcode_result(self, barcode: Any, scale: float) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]]]:
        """Convert a pyzbar barcode result to standardized bounding box and center offsets."""
        rx = int(barcode.rect.left / scale)
        ry = int(barcode.rect.top / scale)
        rw = int(barcode.rect.width / scale)
        rh = int(barcode.rect.height / scale)

        if rw < self.min_width_px:
            logger.warning(f"Detected QR width {rw}px is below minimum width safety gate ({self.min_width_px}px). Drone too high.")
            return False, None, None

        if rw * rh < self.min_area:
            return False, None, None

        cx = rx + rw // 2
        cy = ry + rh // 2

        bbox = np.array(
            [[rx, ry], [rx + rw, ry], [rx + rw, ry + rh], [rx, ry + rh]], dtype=np.int32
        )

        return True, bbox, (cx, cy)