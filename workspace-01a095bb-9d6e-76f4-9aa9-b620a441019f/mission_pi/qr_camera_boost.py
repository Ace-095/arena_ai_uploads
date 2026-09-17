"""
qr_camera_boost.py — Make QR pop more than ground via camera ISP + software filters

For arena/test1: QR is A3 297×420 mm on ground (grass/dirt). Ground is green/brown low-contrast,
QR is black/white high-contrast. Goal: suppress ground, boost QR.

Hardware ISP tuning (Pi Cam3 + IMX477) + software enhancement (CLAHE, unsharp, contrast, desat).

Profiles:
  - normal: default libcamera auto
  - qr_boost_day: high contrast, high sharpness, low saturation (ground desat), slightly dark, manual exposure
  - qr_boost_lowlight: higher gain, slightly brighter
  - qr_boost_ground_suppress: desaturate green/brown, boost B/W
"""

import logging
log = logging.getLogger("qr_boost")

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


# --- ISP tuning profiles for QR visibility ---
# Values are in contract tuning units (same as UI sliders):
# exposure_us µs, gain_db dB, brightness -100..100, contrast 0..2, saturation 0..2, sharpness 0..2
# These make QR black/white pop vs ground green/brown.

QR_BOOST_PROFILES = {
    # Normal — libcamera auto, for reference
    "normal": {
        "adaptive": True,
        "exposure_us": 8333,
        "gain_db": 6.0,
        "brightness": 0.0,
        "contrast": 1.0,
        "saturation": 1.0,
        "sharpness": 1.0,
        "af_mode": "continuous",
        "description": "Auto — default"
    },
    # Daytime QR boost — main profile for A3 on ground at 10/15 m
    "qr_boost_day": {
        "adaptive": False,
        "exposure_us": 5000,
        "gain_db": 2.0,
        "brightness": -10.0,
        "contrast": 1.8,
        "saturation": 0.7,
        "sharpness": 2.0,
        "af_mode": "continuous",
        "description": "QR boost day — high contrast 1.8, sharpness 2.0, sat 0.7, dark -10, exp 5ms"
    },
    "qr_boost_aggressive": {
        "adaptive": False,
        "exposure_us": 4000,
        "gain_db": 1.0,
        "brightness": -15.0,
        "contrast": 2.0,
        "saturation": 0.5,
        "sharpness": 2.2,
        "af_mode": "continuous",
        "description": "Aggressive — contrast 2.0, sat 0.5, sharp 2.2, very dark — ground suppressed"
    },
    "qr_boost_lowlight": {
        "adaptive": False,
        "exposure_us": 8000,
        "gain_db": 6.0,
        "brightness": 5.0,
        "contrast": 1.6,
        "saturation": 0.8,
        "sharpness": 1.8,
        "af_mode": "continuous",
        "description": "Low light — brighter, higher gain, contrast 1.6"
    },
    "bottom_qr_boost": {
        "adaptive": False,
        "exposure_us": 5000,
        "gain_db": 2.0,
        "brightness": -12.0,
        "contrast": 1.9,
        "saturation": 0.6,
        "sharpness": 2.0,
        "af_mode": "manual",
        "description": "Bottom IMX477 B — contrast 1.9 sat 0.6 sharp 2.0 — ground suppressed, QR pops"
    },
    "front_qr_boost": {
        "adaptive": False,
        "exposure_us": 6000,
        "gain_db": 3.0,
        "brightness": -5.0,
        "contrast": 1.6,
        "saturation": 0.8,
        "sharpness": 1.8,
        "af_mode": "continuous",
        "description": "Front Pi Cam3 — AF cont, contrast 1.6 sat 0.8 — QR visible while flying"
    },
    # --- DARK PRESETS — pitch dark you saw, integrated with camera_tune.py ---
    "daylight": {
        "adaptive": True,
        "exposure_us": 8333,
        "gain_db": 6.0,
        "brightness": 0.0,
        "contrast": 1.0,
        "saturation": 1.0,
        "sharpness": 1.0,
        "af_mode": "continuous",
        "description": "Daylight — normal auto, default"
    },
    "dark": {
        "adaptive": False,
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.5,
        "saturation": 1.2,
        "sharpness": 1.3,
        "af_mode": "continuous",
        "description": "Pitch dark — high exposure 30ms, high gain 20dB, brightness 80 — for dark you saw"
    },
    "night": {
        "adaptive": False,
        "exposure_us": 50000,
        "gain_db": 24.0,
        "brightness": 100.0,
        "contrast": 1.8,
        "saturation": 1.0,
        "sharpness": 1.5,
        "af_mode": "continuous",
        "description": "Night — max exposure 50ms, max gain 24dB, brightness 100"
    },
    "dark_qr_boost": {
        "adaptive": False,
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.8,
        "saturation": 0.6,
        "sharpness": 2.0,
        "af_mode": "continuous",
        "description": "Dark + QR boost — BEST for QR in pitch dark, QR pops vs ground"
    },
    "qr_boost_night": {
        "adaptive": False,
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 1.8,
        "saturation": 0.6,
        "sharpness": 2.0,
        "af_mode": "continuous",
        "description": "Night QR boost — dark settings + QR boost, best for night QR"
    },
    "gazebo_dark": {
        "adaptive": False,
        "exposure_us": 30000,
        "gain_db": 20.0,
        "brightness": 80.0,
        "contrast": 2.2,
        "saturation": 0.8,
        "sharpness": 2.5,
        "af_mode": "continuous",
        "description": "Gazebo dark — for gz_cam_bridge --qr-boost --contrast 2.2 --brightness 1.3 --saturation 0.8 --sharpness 2.5"
    },
}

def get_profile(name):
    return QR_BOOST_PROFILES.get(name, QR_BOOST_PROFILES["normal"])

def list_profiles():
    return list(QR_BOOST_PROFILES.keys())


# --- Software enhancement — make QR pop more than ground after capture ---
# This runs on CPU after frame grab, before detection, so detector sees enhanced frame.
# Keeps original for stream, but detection gets boosted version.

def enhance_qr_frame(frame_bgr, mode="qr_boost"):
    """
    Enhance BGR frame so QR pops more than ground.
    mode:
      - qr_boost: CLAHE + unsharp + contrast boost + green suppression
      - ground_suppress: desaturate green/brown, keep B/W
      - none: return original
    Returns enhanced BGR frame (new array).
    """
    if not _HAVE_CV2 or frame_bgr is None or mode == "none":
        return frame_bgr
    try:
        if mode == "qr_boost":
            return _enhance_qr_boost(frame_bgr)
        elif mode == "ground_suppress":
            return _enhance_ground_suppress(frame_bgr)
        elif mode == "adaptive":
            return _enhance_adaptive(frame_bgr)
        else:
            return frame_bgr
    except Exception as e:
        log.debug("enhance_qr_frame failed: %r", e)
        return frame_bgr


def _enhance_qr_boost(frame_bgr):
    """Main QR boost: CLAHE on L channel + unsharp mask + contrast + slight desat green."""
    # Convert to LAB for CLAHE on L
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l2 = clahe.apply(l)
    lab2 = cv2.merge((l2, a, b))
    enhanced = cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)

    # Unsharp mask — makes QR edges crisp at 10/15 m
    # blurred = GaussianBlur, then enhanced = 1.5*original - 0.5*blurred
    blurred = cv2.GaussianBlur(enhanced, (0, 0), 3)
    enhanced = cv2.addWeighted(enhanced, 1.5, blurred, -0.5, 0)

    # Contrast boost + slight desaturation of green
    # Convert to float, boost contrast around mean
    # For ground suppression: reduce green channel slightly if green dominates (grass)
    # Simple: convert to HSV, reduce S where H is green (35-85), keep B/W (low S) untouched
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    # Green mask: H 35-85, S > 50, V > 50 — grass
    green_mask = cv2.inRange(hsv, (35, 50, 50), (85, 255, 255))
    # Reduce saturation of green areas by 40%
    s = s.astype(np.float32)
    s_green = s.copy()
    # Where green mask, reduce S
    s = np.where(green_mask > 0, s * 0.6, s)
    s = np.clip(s, 0, 255).astype(np.uint8)
    # Boost V contrast slightly
    v = cv2.convertScaleAbs(v, alpha=1.2, beta=-10)  # alpha contrast, beta brightness
    hsv2 = cv2.merge((h, s, v))
    enhanced = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

    return enhanced


def _enhance_ground_suppress(frame_bgr):
    """Strong ground suppression: desaturate green/brown, keep B/W QR."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    # Masks for ground colors
    # Green 35-85, Brown 10-30 (low S high V for dirt)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    lower_brown = np.array([10, 30, 30])
    upper_brown = np.array([30, 200, 200])

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
    mask_ground = cv2.bitwise_or(mask_green, mask_brown)

    # Desaturate ground: S * 0.4
    s = s.astype(np.float32)
    s = np.where(mask_ground > 0, s * 0.4, s)
    s = np.clip(s, 0, 255).astype(np.uint8)

    # Slightly darken ground V
    v = v.astype(np.float32)
    v = np.where(mask_ground > 0, v * 0.9, v)
    v = np.clip(v, 0, 255).astype(np.uint8)

    hsv2 = cv2.merge((h, s, v))
    result = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

    # CLAHE for QR contrast
    lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    return result


def _enhance_adaptive(frame_bgr):
    """Adaptive: if frame bright (day), use qr_boost, if dark, use lowlight boost."""
    # Estimate brightness via mean V in HSV
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mean_v = np.mean(hsv[:, :, 2])
    if mean_v > 180:  # bright day, ground overexposed
        return _enhance_qr_boost(frame_bgr)
    elif mean_v < 80:  # low light
        # Brighter + CLAHE
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        # Boost brightness
        l = cv2.convertScaleAbs(l, alpha=1.1, beta=15)
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    else:
        return _enhance_qr_boost(frame_bgr)


# --- Detection-time helper: enhance ROI for decode ---
def enhance_roi_for_decode(roi_gray):
    """Enhance grayscale ROI for pyzbar/cv2 decode — extra sharp + contrast."""
    if not _HAVE_CV2 or roi_gray is None:
        return roi_gray
    try:
        # CLAHE
        clahe = cv2.createCLAHE(clipLimit=3.5, tileGridSize=(8, 8))
        enhanced = clahe.apply(roi_gray)
        # Unsharp
        blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
        enhanced = cv2.addWeighted(enhanced, 1.6, blurred, -0.6, 0)
        # Adaptive threshold variant for QR
        # Keep both original and thresholded — decoder tries variants anyway
        return enhanced
    except Exception:
        return roi_gray
