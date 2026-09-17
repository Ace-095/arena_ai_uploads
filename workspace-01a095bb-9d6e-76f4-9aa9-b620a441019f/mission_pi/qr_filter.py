"""
qr_filter.py — Fake QR filtering for ground mission

Your screenshot: window grill detected as QR at 0.16 / 0.24 conf — useless for ground.
This module kills fakes so Pi only steers toward real QR.

Used by:
  - mission_pi/mission.py (cues + approach)
  - mission_pi/tools/test_yolo_webcam_qr.py (laptop preview)
  - detector wrapper (optional)

Filters:
  1. Geometry: QR is square-ish, not tall grill. Aspect 0.4-2.5, min 15px, max 90% frame
  2. Confidence: low-conf <0.35 without decode = likely fake (your 0.16/0.24)
  3. Decode verification: if payload decodes, it's REAL — always keep (green)
  4. Optional: require_decode — ONLY keep decoded boxes (zero fakes, recommended for ground)

Presets for dark (integrated):
  daylight, dark, night, dark_qr_boost, qr_boost_day, qr_boost_night, gazebo_dark
  — same as camera_tune.py / test_yolo_webcam_qr.py for easy control
"""

import logging
log = logging.getLogger("qr_filter")

# Dark presets — same as camera_tune.py for easy workflow
# V4L2 values are for laptop webcam, Pi values are for rpi tuning
PRESETS = {
    "daylight": {
        "description": "Normal daylight — auto exposure, default",
        "detector": {"conf_thr": 0.35, "require_decode": False, "hide_fake": True},
        "v4l2": {"brightness": 0, "contrast": 45, "saturation": 63, "gain": 0, "exposure": 50, "sharpness": 100},
        "sw": {"brightness": 1.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0},
        "pi": {"exposure_us": 8333, "gain_db": 6.0, "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0, "adaptive": True},
    },
    "dark": {
        "description": "Pitch dark — high exposure 30ms, high gain 20dB, brightness 80 — for dark you saw",
        "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True},
        "v4l2": {"brightness": 64, "contrast": 75, "saturation": 70, "gain": 20, "exposure": 300, "sharpness": 100},
        "sw": {"brightness": 1.3, "contrast": 1.5, "saturation": 1.2, "sharpness": 1.3},
        "pi": {"exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.5, "saturation": 1.2, "sharpness": 1.3, "adaptive": False},
    },
    "night": {
        "description": "Night — max exposure 50ms, max gain 24dB, brightness 100",
        "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True},
        "v4l2": {"brightness": 64, "contrast": 80, "saturation": 63, "gain": 40, "exposure": 400, "sharpness": 100},
        "sw": {"brightness": 1.5, "contrast": 1.8, "saturation": 1.0, "sharpness": 1.5},
        "pi": {"exposure_us": 50000, "gain_db": 24.0, "brightness": 100.0, "contrast": 1.8, "saturation": 1.0, "sharpness": 1.5, "adaptive": False},
    },
    "dark_qr_boost": {
        "description": "Dark + QR boost — BEST for QR in pitch dark, QR pops vs ground",
        "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True},
        "v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 20, "exposure": 300, "sharpness": 100},
        "sw": {"brightness": 1.2, "contrast": 2.2, "saturation": 0.6, "sharpness": 2.5},
        "pi": {"exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0, "adaptive": False, "qr_boost_enabled": True, "qr_boost_mode": "bottom_qr_boost", "qr_enhance_mode": "ground_suppress"},
    },
    "qr_boost_day": {
        "description": "Daylight QR boost — contrast high, sat low, sharp high, QR pops",
        "detector": {"conf_thr": 0.35, "require_decode": True, "hide_fake": True},
        "v4l2": {"brightness": 0, "contrast": 75, "saturation": 30, "gain": 0, "exposure": 50, "sharpness": 100},
        "sw": {"brightness": 1.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0},
        "pi": {"exposure_us": 5000, "gain_db": 2.0, "brightness": -10.0, "contrast": 1.8, "saturation": 0.7, "sharpness": 2.0, "adaptive": False, "qr_boost_enabled": True},
    },
    "qr_boost_night": {
        "description": "Night QR boost — dark settings + QR boost, best for night QR",
        "detector": {"conf_thr": 0.30, "require_decode": True, "hide_fake": True},
        "v4l2": {"brightness": 64, "contrast": 90, "saturation": 30, "gain": 30, "exposure": 350, "sharpness": 100},
        "sw": {"brightness": 1.3, "contrast": 2.0, "saturation": 0.6, "sharpness": 2.5},
        "pi": {"exposure_us": 30000, "gain_db": 20.0, "brightness": 80.0, "contrast": 1.8, "saturation": 0.6, "sharpness": 2.0, "adaptive": False, "qr_boost_enabled": True},
    },
    "gazebo_dark": {
        "description": "Gazebo dark — for gz_cam_bridge.py --qr-boost --contrast 2.2 --brightness 1.3",
        "detector": {"conf_thr": 0.25, "require_decode": False, "hide_fake": True},
        "v4l2": {"brightness": 64, "contrast": 85, "saturation": 50, "gain": 20, "exposure": 300, "sharpness": 100},
        "sw": {"brightness": 1.3, "contrast": 2.2, "saturation": 0.8, "sharpness": 2.5},
        "pi": {"contrast": 2.2, "saturation": 0.8, "sharpness": 2.5, "brightness": 1.3, "qr_boost": True},
    },
}

def get_preset(name):
    return PRESETS.get(name, PRESETS["daylight"])

def list_presets():
    return list(PRESETS.keys())

def is_fake_box(box, frame_shape=None, min_size=15, max_aspect=2.5, min_aspect=0.4):
    """Heuristic to filter obvious non-QR: extreme aspect, too small, too large"""
    try:
        x, y, w, h = box.x, box.y, box.w, box.h
    except AttributeError:
        # tuple (x,y,w,h,conf) or (x,y,w,h)
        x, y, w, h = box[0], box[1], box[2], box[3]
    if w < min_size or h < min_size:
        return True, f"too small {w}x{h} < {min_size}"
    if frame_shape is not None:
        try:
            fh, fw = frame_shape[:2]
            if w > fw * 0.9 or h > fh * 0.9:
                return True, f"too large {w}x{h} vs frame {fw}x{fh}"
        except Exception:
            pass
    aspect = w / (h + 1e-6)
    if aspect < min_aspect or aspect > max_aspect:
        return True, f"aspect {aspect:.2f} not in {min_aspect}-{max_aspect} (tall grill)"
    return False, ""

def filter_boxes(boxes, payloads=None, frame_shape=None, require_decode=False, hide_fake=True, min_conf=0.0, min_size=15):
    """
    Filter YOLO boxes to remove fakes — returns filtered (boxes, payloads)
    - require_decode: ONLY keep boxes that decoded to payload (kills ALL fakes, best for ground)
    - hide_fake: hide low-conf <0.35 non-decoded + extreme aspect (your 0.16/0.24 grill)
    """
    if payloads is None:
        payloads = [None] * len(boxes)
    out_boxes = []
    out_payloads = []
    for b, p in zip(boxes, payloads):
        try:
            conf = b.conf
        except AttributeError:
            conf = b[4] if len(b) > 4 else 1.0
        if conf < min_conf:
            continue
        is_fake, reason = is_fake_box(b, frame_shape, min_size=min_size)
        if hide_fake and is_fake:
            log.debug(f"filter fake box {b} reason {reason}")
            continue
        if require_decode and p is None:
            continue
        if hide_fake and p is None and conf < 0.35:
            # your 0.16/0.24 orange boxes — hide
            continue
        out_boxes.append(b)
        out_payloads.append(p)
    return out_boxes, out_payloads

def filter_cue_box(box, frame_shape=None, conf_thr=0.35):
    """Should this box trigger a flight cue? Avoid steering to fakes."""
    try:
        conf = box.conf
    except AttributeError:
        conf = box[4] if len(box) > 4 else 0.0
    if conf < conf_thr:
        return False, f"conf {conf:.2f} < {conf_thr}"
    is_fake, reason = is_fake_box(box, frame_shape)
    if is_fake:
        return False, reason
    return True, ""
