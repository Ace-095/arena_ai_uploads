"""
QR / Intelligence Cache Detection & Decode Pipeline (Phase 7).

Stage 1: detect_cache(frame) -> Optional[BBox]
    Pluggable detector strategy interface.
    PlaceholderColorShapeDetector: heuristic/contour target locator for synthetic/sim.
    HailoCacheDetector: NEED TEST stub for Hailo AI HAT hardware accelerator.

Stage 2: decode_qr(frame, bbox) -> Optional[str]
    Decodes QR code from ROI or full frame using OpenCV cv2.QRCodeDetector / pyzbar.

ConsensusBuffer:
    Requires N (default 3-5) consecutive matching decodes before confirming payload.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from config import config
from models import BBox, QRDetectionResult
from telemetry import push_event

log = logging.getLogger("qr_pipeline")


# ---------------------------------------------------------------------------
# Stage 1: Pluggable Cache Detector Strategy Interface & Implementations
# ---------------------------------------------------------------------------

class CacheDetector(ABC):
    """Abstract interface for Stage 1 target location."""

    @abstractmethod
    def detect_cache(self, frame: np.ndarray) -> Optional[BBox]:
        """Detect candidate Intelligence Cache bounding box in BGR frame."""
        ...


class PlaceholderColorShapeDetector(CacheDetector):
    """
    Heuristic/contour detector targeting synthetic and sim QR targets.
    Locates high-contrast square or dark/light bounding regions.
    """

    def __init__(self, min_area_ratio: float = 0.001, max_area_ratio: float = 0.8):
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio

    def detect_cache(self, frame: np.ndarray) -> Optional[BBox]:
        if frame is None or frame.size == 0:
            return None

        h_img, w_img = frame.shape[:2]
        total_area = float(h_img * w_img)

        # Convert to greyscale & threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Otsu thresholding or adaptive thresholding to isolate target
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        best_bbox = None
        best_score = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            area_ratio = area / total_area

            if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / float(h) if h > 0 else 0.0

            # Target is roughly square (aspect ratio ~ 1.0)
            if 0.6 <= aspect_ratio <= 1.6:
                # Squareness score
                score = 1.0 - abs(1.0 - aspect_ratio)
                if score > best_score:
                    best_score = score
                    best_bbox = BBox(x=int(x), y=int(y), w=int(w), h=int(h), confidence=round(score, 2))

        # No fallback: if no contour matched the criteria, return None.
        # A prior version fabricated a fake center-50% bbox here with
        # confidence=0.5 whenever no real contour was found — confirmed by
        # testing that this made detect_cache() return "something detected"
        # even on a completely blank frame with no target in it at all,
        # which defeats the Optional[BBox] contract every caller relies on
        # to mean "nothing found." decode_qr() already handles bbox=None
        # correctly via its full-frame fallback path, so there's no reason
        # to fabricate one here.
        return best_bbox


class HailoCacheDetector(CacheDetector):
    """
    NEED TEST: Hailo AI HAT integration for detect_cache.
    Exact SDK depends on specific Hailo variant / HailoRT version on real hardware.
    Implemented as a pluggable strategy; hardware acceleration bring-up in Phase 9.
    """

    def __init__(self, hef_path: str = "cache_detector.hef"):
        self.hef_path = hef_path

    def detect_cache(self, frame: np.ndarray) -> Optional[BBox]:
        raise NotImplementedError(
            f"HailoCacheDetector for '{self.hef_path}' requires HailoRT SDK and "
            "hardware bring-up in Phase 9. Use PlaceholderColorShapeDetector for sim/synthetic."
        )


def get_cache_detector(strategy_name: Optional[str] = None) -> CacheDetector:
    strat = (strategy_name or config.CACHE_DETECTOR_STRATEGY).lower().strip()
    if strat == "hailo":
        return HailoCacheDetector()
    return PlaceholderColorShapeDetector()


# ---------------------------------------------------------------------------
# Stage 2: QR Decode Implementation
# ---------------------------------------------------------------------------

_cv2_qr_detector = cv2.QRCodeDetector()


def decode_qr(frame: np.ndarray, bbox: Optional[BBox] = None) -> Optional[str]:
    """
    Decode QR payload from BGR frame.
    If bbox is provided, attempts decoding on ROI first before full frame.
    Uses cv2.QRCodeDetector with fallback to pyzbar if available.
    """
    if frame is None or frame.size == 0:
        return None

    # 1. Try decoding ROI if bbox provided
    if bbox is not None and bbox.w > 10 and bbox.h > 10:
        h_img, w_img = frame.shape[:2]
        x1 = max(0, bbox.x)
        y1 = max(0, bbox.y)
        x2 = min(w_img, bbox.x + bbox.w)
        y2 = min(h_img, bbox.y + bbox.h)

        if x2 > x1 and y2 > y1:
            roi = frame[y1:y2, x1:x2]
            payload, _, _ = _cv2_qr_detector.detectAndDecode(roi)
            if payload:
                return payload.strip()

    # 2. Try full frame decoding with cv2.QRCodeDetector
    payload, _, _ = _cv2_qr_detector.detectAndDecode(frame)
    if payload:
        return payload.strip()

    # 3. Fallback: pyzbar if installed
    try:
        from pyzbar import pyzbar
        decoded_objs = pyzbar.decode(frame)
        for obj in decoded_objs:
            if obj.data:
                return obj.data.decode("utf-8").strip()
    except ImportError:
        pass

    return None


# ---------------------------------------------------------------------------
# Consensus Buffer (N-Frame Streak Verification)
# ---------------------------------------------------------------------------

class ConsensusBuffer:
    """
    Requires the same decoded string value for N consecutive frames before
    accepting it. Prevents single-frame false decodes from triggering
    detection callbacks.

    miss_tolerance: how many consecutive "bad" frames (no decode, or a
    decode that disagrees with the value currently being tracked) are
    absorbed without discarding the in-progress streak. Default 1.

    This exists because the original implementation reset the streak to
    zero on the very first bad frame — confirmed by test: feeding it
    good, good, BLANK, good, good never reached consensus, because the
    single blank frame in the middle wiped out the streak of 2 that had
    already built up, and the two "good" frames after it could only get
    it back up to streak=2 again by the end. A single motion-blurred
    frame during an otherwise-good approach shouldn't throw away that
    progress — that's the whole point of using a streak requirement
    instead of a single-frame decode in the first place.
    """

    def __init__(self, required_consecutive: int = 3, miss_tolerance: int = 1):
        self.required_consecutive = max(1, required_consecutive)
        self.miss_tolerance = max(0, miss_tolerance)
        self._current_value: Optional[str] = None
        self._current_streak: int = 0
        self._miss_streak: int = 0
        self._last_bbox: Optional[BBox] = None
        self._confirmed: bool = False
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._current_value = None
            self._current_streak = 0
            self._miss_streak = 0
            self._last_bbox = None
            self._confirmed = False

    def update(self, decoded_value: Optional[str], bbox: Optional[BBox] = None) -> Tuple[bool, QRDetectionResult]:
        """
        Update consensus state with new frame decode result.
        Returns (is_newly_confirmed, QRDetectionResult).
        """
        with self._lock:
            has_value = decoded_value is not None and len(decoded_value) > 0

            if has_value and decoded_value == self._current_value:
                # Matches what we're already tracking — build the streak.
                self._current_streak += 1
                self._miss_streak = 0
                if bbox is not None:
                    self._last_bbox = bbox

            elif has_value and self._current_value is None:
                # First real decode we've seen at all — start tracking it.
                self._current_value = decoded_value
                self._current_streak = 1
                self._miss_streak = 0
                self._confirmed = False
                if bbox is not None:
                    self._last_bbox = bbox

            else:
                # Either no decode this frame, or a decode that disagrees
                # with the value we're tracking (a possible misread).
                # Don't touch the existing streak/value yet — only give up
                # on it once miss_streak exceeds the configured tolerance.
                self._miss_streak += 1
                if self._miss_streak > self.miss_tolerance:
                    if has_value:
                        # Enough consecutive disagreement to treat this as
                        # a genuine change of target, not noise — switch.
                        self._current_value = decoded_value
                        self._current_streak = 1
                        if bbox is not None:
                            self._last_bbox = bbox
                    else:
                        # Sustained no-signal — give up on the streak.
                        self._current_value = None
                        self._current_streak = 0
                    self._miss_streak = 0
                    self._confirmed = False

            newly_confirmed = False
            if self._current_streak >= self.required_consecutive and not self._confirmed:
                self._confirmed = True
                newly_confirmed = True
                log.info("Consensus REACHED for payload '%s' (%d consecutive frames)!",
                         self._current_value, self._current_streak)

            res = QRDetectionResult(
                bbox=self._last_bbox,
                payload=self._current_value if self._confirmed else None,
                confirmed=self._confirmed,
                streak=self._current_streak,
                required_streak=self.required_consecutive,
                timestamp=time.time(),
            )

            return newly_confirmed, res

    def get_status(self) -> QRDetectionResult:
        with self._lock:
            return QRDetectionResult(
                bbox=self._last_bbox,
                payload=self._current_value if self._confirmed else None,
                confirmed=self._confirmed,
                streak=self._current_streak,
                required_streak=self.required_consecutive,
                timestamp=time.time(),
            )


# ---------------------------------------------------------------------------
# QR Pipeline Worker / Runner
# ---------------------------------------------------------------------------

class QRPipelineRunner:
    """
    Background pipeline runner that consumes frames from a CameraSource,
    runs detect_cache -> decode_qr -> ConsensusBuffer, and notifies
    search_controller on consensus.
    """

    def __init__(self, detector: Optional[CacheDetector] = None, required_streak: int = 3):
        self.detector = detector or get_cache_detector()
        self.consensus = ConsensusBuffer(required_consecutive=required_streak)
        self._source = None
        self._search_controller = None
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def set_source(self, source) -> None:
        """
        Swap the active camera source.

        Calls start() on the new source — CameraSource.get_frame() is
        documented to require start() first, and this call was
        previously missing entirely here. _run_loop's get_frame() call
        was therefore always raising RuntimeError, silently swallowed by
        its broad except at DEBUG level — confirmed by direct
        reproduction: consensus streak stayed at 0 forever, never
        reaching even a single successful decode.

        Deliberately does NOT stop the previous source — CameraManager
        independently owns start/stop lifecycle for these same source
        objects (for WebRTC streaming), and stopping one here could pull
        it out from under CameraManager while it's still in use.
        start() is required to be idempotent by the CameraSource
        contract, so calling it again here even if CameraManager already
        started this same source is safe.
        """
        self._source = source
        if source is not None:
            source.start()

    def set_search_controller(self, controller) -> None:
        self._search_controller = controller

    def start(self) -> None:
        with self._lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                return
            self._stop_event.clear()
            self._worker_thread = threading.Thread(
                target=self._run_loop, name="qr-pipeline-runner", daemon=True
            )
            self._worker_thread.start()
            log.info("QRPipelineRunner started.")

    def stop(self) -> None:
        self._stop_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
            self._worker_thread = None
            log.info("QRPipelineRunner stopped.")

    def process_single_frame(self, frame: np.ndarray) -> Tuple[bool, QRDetectionResult]:
        """Process a single frame synchronously (for testing / manual API call)."""
        bbox = self.detector.detect_cache(frame)
        payload = decode_qr(frame, bbox)
        newly_confirmed, result = self.consensus.update(payload, bbox)

        if newly_confirmed:
            bbox_dict = bbox.model_dump() if bbox is not None else {"x": 0, "y": 0, "w": 0, "h": 0}
            if self._search_controller is not None:
                self._search_controller.on_detection(bbox_dict, 0.95)
            push_event("qr_decoded", result.model_dump())

        return newly_confirmed, result

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._source is None:
                time.sleep(0.2)
                continue

            try:
                frame = self._source.get_frame()
                if frame is not None:
                    self.process_single_frame(frame)
            except Exception as exc:
                log.debug("QRPipelineRunner frame error: %s", exc)
                time.sleep(0.1)


# Global instance
qr_pipeline_runner = QRPipelineRunner()
