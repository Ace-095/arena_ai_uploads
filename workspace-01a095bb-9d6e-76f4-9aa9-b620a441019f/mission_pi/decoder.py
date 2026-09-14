"""QR decode stage: full frame -> ROI crop/upscale -> tiles.

Strategy (in order, first payload wins):
  1. If a detector bbox exists: crop with margin, upscale so the QR is
     >= ~300 px, try decode variants (plain / CLAHE / mild blur).
  2. Full frame at native resolution (catches close/large QRs cheaply).
  3. 2x2 overlapping tiles upscaled (catches small/far QRs).

Decoders: pyzbar first (most robust), cv2.QRCodeDetector second.
Everything degrades gracefully if a backend is missing.
"""
import logging

log = logging.getLogger("decoder")

try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except Exception:
    pyzbar = None
    _HAVE_PYZBAR = False

_cv2det = None


def _cv2_detector():
    global _cv2det
    if _cv2det is None and _HAVE_CV2:
        _cv2det = cv2.QRCodeDetector()
    return _cv2det


def _to_gray(frame):
    if frame is None or getattr(frame, "size", 0) == 0:
        return None
    if frame.ndim == 2:
        return frame
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _try_backends(gray):
    """Run both decoders on one grayscale image. Returns payload or None."""
    if gray is None:
        return None
    if _HAVE_PYZBAR:
        try:
            for obj in pyzbar.decode(gray):
                if obj.data:
                    return obj.data.decode("utf-8", "replace").strip() or None
        except Exception as e:
            log.debug("pyzbar failed: %r", e)
    det = _cv2_detector()
    if det is not None:
        try:
            payload, _, _ = det.detectAndDecode(gray)
            if payload:
                return payload.strip()
        except Exception as e:
            log.debug("cv2 qr decode failed: %r", e)
    return None


def _variants(gray):
    """Yield preprocessed variants of a grayscale ROI."""
    yield gray
    if _HAVE_CV2:
        try:
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            yield clahe.apply(gray)
        except Exception:
            pass
        try:
            yield cv2.GaussianBlur(gray, (3, 3), 0)
        except Exception:
            pass


def _upscale(gray, min_side=360):
    h, w = gray.shape[:2]
    s = max(w, h)
    if s >= min_side:
        return gray
    k = min_side / max(1, s)
    k = min(k, 4.0)
    return cv2.resize(gray, (int(w * k), int(h * k)), interpolation=cv2.INTER_CUBIC)


def decode_frame(frame_bgr, bbox=None, margin=0.35):
    """Decode a QR payload from a BGR frame.

    bbox: optional (x, y, w, h) candidate from the detector.
    Returns payload string or None. Never raises on bad input.
    """
    if not _HAVE_CV2 or frame_bgr is None:
        return None
    try:
        h_img, w_img = frame_bgr.shape[:2]
        # 1) ROI path
        if bbox is not None:
            x, y, w, h = [int(v) for v in bbox[:4]]
            if w > 8 and h > 8:
                mx, my = int(w * margin), int(h * margin)
                x1, y1 = max(0, x - mx), max(0, y - my)
                x2, y2 = min(w_img, x + w + mx), min(h_img, y + h + my)
                if x2 > x1 and y2 > y1:
                    roi = _to_gray(frame_bgr[y1:y2, x1:x2])
                    if roi is not None:
                        roi = _upscale(roi)
                        for v in _variants(roi):
                            out = _try_backends(v)
                            if out:
                                return out
        # 2) full frame
        gray = _to_gray(frame_bgr)
        for v in _variants(gray):
            out = _try_backends(v)
            if out:
                return out
        # 3) 2x2 overlapping tiles (small/far QRs)
        if max(w_img, h_img) > 900:
            for (x1, y1, x2, y2) in _tiles(w_img, h_img):
                tile = gray[y1:y2, x1:x2]
                if tile.size == 0:
                    continue
                tile = _upscale(tile, min_side=700)
                for v in _variants(tile):
                    out = _try_backends(v)
                    if out:
                        return out
    except Exception as e:
        log.debug("decode_frame error: %r", e)
    return None


def _tiles(w, h, n=2, overlap=0.2):
    out = []
    for iy in range(n):
        for ix in range(n):
            x1 = int(ix * w / n - (w * overlap / n if ix else 0))
            y1 = int(iy * h / n - (h * overlap / n if iy else 0))
            x2 = int((ix + 1) * w / n + (w * overlap / n if ix < n - 1 else 0))
            y2 = int((iy + 1) * h / n + (h * overlap / n if iy < n - 1 else 0))
            out.append((max(0, x1), max(0, y1), min(w, x2), min(h, y2)))
    return out


def classical_boxes(frame_bgr):
    """Day-one detector boxes without any trained model.

    Merges cv2 multi-detect + pyzbar rects, deduped by IoU.
    Returns [(x, y, w, h, conf), ...]. Weak past ~8 m — the YOLO stage
    is what carries 15 m detection (see detector.py / README).
    """
    boxes = []
    if not _HAVE_CV2 or frame_bgr is None:
        return boxes
    try:
        gray = _to_gray(frame_bgr)
        det = _cv2_detector()
        if det is not None:
            ok, points = det.detectMulti(gray)
            if ok and points is not None:
                for quad in points:
                    xs = [p[0] for p in quad]
                    ys = [p[1] for p in quad]
                    x1, y1, x2, y2 = int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))
                    boxes.append((x1, y1, x2 - x1, y2 - y1, 0.6))
        if _HAVE_PYZBAR:
            for obj in pyzbar.decode(gray):
                r = obj.rect
                boxes.append((r.left, r.top, r.width, r.height, 0.9))
    except Exception as e:
        log.debug("classical_boxes error: %r", e)
    return _nms(boxes, 0.4)


def _iou(a, b):
    ax2, ay2, bx2, by2 = a[0] + a[2], a[1] + a[3], b[0] + b[2], b[1] + b[3]
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def _nms(boxes, iou_thr):
    boxes = sorted(boxes, key=lambda b: -b[4])
    keep = []
    for b in boxes:
        if all(_iou(b, k) < iou_thr for k in keep):
            keep.append(b)
    return keep
