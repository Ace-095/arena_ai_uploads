"""
qr_camera_boost.py — Make QR pop more than ground via camera ISP + software filters

For arena/test1: QR is A3 297×420 mm on ground (grass/dirt). Ground is green/brown low-contrast,
QR is black/white high-contrast. Goal: suppress ground, boost QR.

Hardware ISP tuning (Pi Cam3 + IMX477) + software enhancement (CLAHE, unsharp, contrast, desat).

Profiles:
  - normal: default libcamera auto
  - qr_boost_day: high contrast, high sharpness, low saturation (ground desat), slightly dark, manual exposure
  - qr_boost_lowlight: AUTO EXPOSURE with a long frame-duration budget (up to
    100 ms) + high-quality NR — the night-ground profile. Manual exposure is
    deliberately NOT used at night: AeEnable=false in a dark room = black.
  - qr_boost_night: as above but 200 ms / 5 fps (very dark ground, static scene)
  - qr_boost_auto: AE always; cannot go black whatever the light is

  Tuning keys beyond exposure/gain: frame_duration_us (=> FrameDurationLimits,
  the ceiling the AE may expose to), ae_exposure_mode (normal|long|short),
  noise_reduction (off|fast|highquality), exposure_value (EV bias).
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

#: Frame mean (0..255) below which the picture is too dark to see, let alone
#: decode, and a MANUAL exposure profile has to be overridden. Field report
#: 2026-10-13: pressing presets set the camera to {AeEnable: False,
#: ExposureTime: 5000, AnalogueGain: 1.26} — a DAYLIGHT-GROUND setting — in a
#: night room, and the window went black (frame_mean=4). libcamera's AeEnable
#: false means manual exposure AND manual gain, so a manual profile in the dark
#: cannot recover by itself: something has to hand exposure back to the AE.
DARK_MEAN = 32.0
#: Above this the frame is blown out, which destroys QR contrast just as badly.
BLOWN_MEAN = 230.0
#: Once the scene is this bright again, a rescued camera may go back to the
#: profile the operator asked for (hysteresis: 32 -> 60, so it cannot flap).
RESTORE_MEAN = 60.0

#: What "give exposure back to the camera" means concretely: AE ON, a long
#: FrameDurationLimits budget (AE clips its own exposure to that), the long
#: exposure profile so AE trades time for gain (less noise), high-quality noise
#: reduction, and a small brightness lift.
RESCUE_TUNING = {
    "adaptive": True,
    "frame_duration_us": 100000,     # 10 fps: AE may expose up to 100 ms
    "ae_exposure_mode": "long",
    "noise_reduction": "highquality",
    "brightness": 8.0,
    "contrast": 1.5,
    "saturation": 0.8,
    "sharpness": 1.9,
}


def dark_rescue(tuning, mean, std=None):
    """Manual profile + a dark (or blown) frame -> (overrides, reason)|(None,None).

    Pure, so the decision is testable without a camera. Only MANUAL profiles are
    rescued — an adaptive one is already exposing the scene correctly and must
    not be fought with. The rescue never guesses a new manual exposure (that is
    what produced the black picture); it hands exposure back to the AE and gives
    it a long frame-duration budget to work with.
    """
    if not tuning or mean is None or mean < 0:
        return None, None
    if tuning.get("adaptive", True):
        return None, None
    if mean < DARK_MEAN:
        why = ("the scene is dark (frame mean %.0f < %.0f) and this profile "
               "forces manual exposure/gain %sms/%sx"
               % (mean, DARK_MEAN, tuning.get("exposure_us"),
                  tuning.get("gain_db")))
    elif mean > BLOWN_MEAN:
        why = ("the scene is blown out (frame mean %.0f > %.0f) and this "
               "profile forces manual exposure %sms"
               % (mean, BLOWN_MEAN, tuning.get("exposure_us")))
    else:
        return None, None
    return dict(RESCUE_TUNING), why


def recovered(mean):
    """Is the scene bright enough to hand the operator's profile back?"""
    return bool(mean is not None and mean >= RESTORE_MEAN)


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
    # High contrast + high sharpness makes QR edges crisp, low saturation suppresses green grass
    # Slightly dark brightness avoids washout, manual exposure 5000-6000 us reduces motion blur
    "qr_boost_day": {
        "adaptive": False,
        "exposure_us": 5000,      # manual 5 ms — less blur at 2 m/s, avoids overexposure of white ground
        "gain_db": 2.0,           # low gain — less noise, QR still high contrast
        "brightness": -10.0,      # slightly darker — white ground not blown, black QR stays black
        "contrast": 1.8,          # HIGH — makes black/white QR pop vs mid-tone ground
        "saturation": 0.7,        # LOW — desaturate green/brown ground, QR B/W unaffected
        "sharpness": 2.0,         # HIGH — QR finder patterns sharper at distance
        "af_mode": "continuous",  # for front cam; bottom fixed focus ignores this
        "description": "QR boost day — high contrast 1.8, sharpness 2.0, sat 0.7, dark -10, exp 5ms"
    },
    # Aggressive QR boost — for difficult ground (bright grass, shadows)
    "qr_boost_aggressive": {
        "adaptive": False,
        "exposure_us": 4000,
        "gain_db": 1.0,
        "brightness": -15.0,
        "contrast": 2.0,          # max contrast
        "saturation": 0.5,        # strong desat — ground almost gray, QR still B/W
        "sharpness": 2.2,
        "af_mode": "continuous",
        "description": "Aggressive — contrast 2.0, sat 0.5, sharp 2.2, very dark — ground suppressed"
    },
    # Low light / overcast / night ground. AUTO EXPOSURE ON: at 8 ms manual it
    # produced a black window indoors at night (frame_mean=4), because
    # AeEnable=false is manual exposure AND manual gain. The frame-duration
    # budget is what actually lets the sensor collect light (AE clips its own
    # exposure to FrameDurationLimits), and the "long" exposure profile makes AE
    # trade time for gain, which is less noisy for a static QR on the ground.
    "qr_boost_lowlight": {
        "adaptive": True,
        "frame_duration_us": 100000,   # up to 100 ms of exposure (10 fps)
        "ae_exposure_mode": "long",
        "noise_reduction": "highquality",
        "exposure_us": 8000,           # what a MANUAL fallback would use
        "gain_db": 6.0,
        "brightness": 8.0,
        "contrast": 1.5,
        "saturation": 0.8,
        "sharpness": 1.9,
        "af_mode": "continuous",
        "description": "Night/low light — AE on, up to 100 ms exposure, high-"
                       "quality NR, contrast 1.5 (was 8 ms manual = black)"
    },
    # Very dark ground (no street light): 5 fps, only for a stationary scene.
    "qr_boost_night": {
        "adaptive": True,
        "frame_duration_us": 200000,   # up to 200 ms of exposure (5 fps)
        "ae_exposure_mode": "long",
        "noise_reduction": "highquality",
        "exposure_us": 30000,
        "gain_db": 12.0,
        "brightness": 10.0,
        "contrast": 1.4,
        "saturation": 0.8,
        "sharpness": 1.8,
        "af_mode": "continuous",
        "description": "Very dark ground — AE on, up to 200 ms exposure "
                       "(5 fps, static scene only), max NR"
    },
    # General purpose: let the camera expose the scene itself, boost the ISP.
    # Safest profile indoors — it cannot go black, whatever the light is.
    "qr_boost_auto": {
        "adaptive": True,
        "frame_duration_us": 66666,    # 15 fps, up to 66 ms of exposure
        "ae_exposure_mode": "normal",
        "noise_reduction": "fast",
        "exposure_us": 8000,
        "gain_db": 6.0,
        "brightness": 0.0,
        "contrast": 1.6,
        "saturation": 0.85,
        "sharpness": 2.0,
        "af_mode": "continuous",
        "description": "Auto exposure + ISP sharpening — cannot be blinded by "
                       "the light level"
    },
    # Bottom cam specific — fixed focus, infinity, ground suppress
    "bottom_qr_boost": {
        "adaptive": False,
        "exposure_us": 5000,
        "gain_db": 2.0,
        "brightness": -12.0,
        "contrast": 1.9,
        "saturation": 0.6,        # desaturate green grass
        "sharpness": 2.0,
        "af_mode": "manual",      # bottom fixed focus — manual
        "description": "Bottom IMX477 B — contrast 1.9 sat 0.6 sharp 2.0 — ground suppressed, QR pops"
    },
    # Front cam specific — AF continuous, slightly less aggressive
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
