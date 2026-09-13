"""Visual servoing PID controller for precision aerial alignment over a QR code.

CHANGE 1: EMA filtering on detection center, adaptive PID gains, velocity
smoothing, and oscillation detection for stable, overshoot-free convergence.
"""

import logging
import math
import time
import numpy as np
from collections import deque
from utils.filters import PIDController
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


class AlignmentController:
    """Translate target pixel deviations into precise vehicle guided velocity vectors."""

    def __init__(self, config: dict):
        cfg = config['alignment']
        # QR (mission) target tuning
        self.qr_deadzone_px = cfg['center_deadzone_px']
        self.max_vel = cfg['max_velocity']
        self.qr_stable_required = cfg['stable_frames_required']

        # PID constants
        self.kp = cfg.get('pid_kp', 0.5)
        self.ki = cfg.get('pid_ki', 0.08)
        self.kd = cfg.get('pid_kd', 0.2)

        # CHANGE 1: adaptive gain scaling. When the pixel error is near the
        # deadzone (within 2×) we soften gains to avoid overshoot; far from the
        # deadzone we use full gains for fast convergence.
        self.gain_scale_near = cfg.get('gain_scale_near', 0.6)
        self.gain_scale_far = cfg.get('gain_scale_far', 1.0)

        # CHANGE 1: EMA smoothing constants.
        self.center_ema_alpha = cfg.get('center_ema_alpha', 0.35)
        self.vel_ema_alpha = cfg.get('vel_ema_alpha', 0.45)

        # Axis inversion flags — flip to true if the vehicle drifts away from
        # the target instead of converging. Applied as -1 multiplier to the
        # metric errors before PID update. Correct value depends on camera
        # mounting orientation (image-top = drone-??). See alignment.yaml.
        self.invert_x = cfg.get('invert_x', False)
        self.invert_y = cfg.get('invert_y', False)

        # Camera physical properties
        self.image_width = config['camera']['width']
        self.image_height = config['camera']['height']

        # QR sizing config
        self.qr_size_cm = config['vision']['qr_size_cm']

        # Platform target tuning (tighter defaults)
        p_cfg = config.get('platform', {})
        self.platform_deadzone_px = p_cfg.get('center_deadzone_px', 20)
        self.platform_stable_required = p_cfg.get('stable_frames_required', 30)
        self.platform_size_cm = p_cfg.get('marker_size_cm', 30.0)

        # Optics
        self.fov_horizontal_rad = np.radians(config['camera'].get('fov_horizontal_deg', 66.0))

        # PID instances
        self.pid_x = PIDController(self.kp, self.ki, self.kd, self.max_vel)
        self.pid_y = PIDController(self.kp, self.ki, self.kd, self.max_vel)

        self.stable_counter = 0

        # CHANGE 1: EMA-filtered detection center (smoothes pixel jitter).
        self._smoothed_cx: Optional[float] = None
        self._smoothed_cy: Optional[float] = None

        # CHANGE 1: EMA-filtered velocity output (smoothes command steps).
        self._smoothed_vx: float = 0.0
        self._smoothed_vy: float = 0.0

        # CHANGE 1: rolling pixel-error history for oscillation detection.
        self._error_history = deque(maxlen=10)

    def reset(self):
        """Reset the PID controller memory and stable counter."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.stable_counter = 0
        self._smoothed_cx = None
        self._smoothed_cy = None
        self._smoothed_vx = 0.0
        self._smoothed_vy = 0.0
        self._error_history.clear()
        logger.info("Alignment PID controller state reset.")

    def _ema_center(self, cx: float, cy: float) -> Tuple[float, float]:
        """Exponential moving average on the detected center to kill jitter.

        First detection seeds the filter directly; later frames blend the new
        observation in with weight ``center_ema_alpha``. Small alpha = heavy
        smoothing (slower response); 1.0 = raw signal (no smoothing).
        """
        if self._smoothed_cx is None:
            self._smoothed_cx = float(cx)
            self._smoothed_cy = float(cy)
        else:
            a = self.center_ema_alpha
            self._smoothed_cx = a * cx + (1.0 - a) * self._smoothed_cx
            self._smoothed_cy = a * cy + (1.0 - a) * self._smoothed_cy
        return self._smoothed_cx, self._smoothed_cy

    def _ema_velocity(self, vx: float, vy: float) -> Tuple[float, float]:
        """Exponential moving average on commanded velocity to prevent steps."""
        a = self.vel_ema_alpha
        self._smoothed_vx = a * vx + (1.0 - a) * self._smoothed_vx
        self._smoothed_vy = a * vy + (1.0 - a) * self._smoothed_vy
        return self._smoothed_vx, self._smoothed_vy

    def _adaptive_gain_scale(self, err_px: float, deadzone: float) -> float:
        """Gain multiplier that softens corrections near the deadzone.

        Within 2× deadzone the multiplier blends linearly from ``gain_scale_near``
        (soft, precise) up to ``gain_scale_far`` (full) at 2× deadzone. Beyond that
        full gain applies for rapid convergence.
        """
        threshold = max(1.0, deadzone * 2.0)
        t = min(1.0, abs(err_px) / threshold)
        return self.gain_scale_near + t * (self.gain_scale_far - self.gain_scale_near)

    def _detect_oscillation(self) -> bool:
        """Detect hunting: error sign flips ≥3 times in the recent window.

        When the drone oscillates around the target (sign-flips), the velocity
        output is damped so the EMA + adaptive gains can settle it.
        """
        if len(self._error_history) < 5:
            return False
        flips = 0
        prev = None
        for e in self._error_history:
            s = 1 if e > 0 else (-1 if e < 0 else 0)
            if s != 0:
                if prev is not None and prev != s:
                    flips += 1
                prev = s
        return flips >= 3

    def compute(self, center: Tuple[int, int], pixel_width: Optional[int] = None,
                altitude_m: float = 5.0, frame_size: Optional[Tuple[int, int]] = None,
                mode: str = 'qr', confidence: float = 1.0) -> Tuple[float, float, bool]:
        """
        Compute horizontal velocity command (vx, vy) to center the drone over the target.

        CHANGE 1: applies EMA on center + velocity, adaptive gain scaling, and
        oscillation damping.  ``confidence`` scales the deadzone — low-confidence
        detections use a wider gate so noisy positions do not cause corrections.

        Args:
            center: (cx, cy) pixel coordinates of the detected target
            pixel_width: Bounding width of the target in pixels
            altitude_m: Telemetry height above ground in meters
            frame_size: Optional (width, height) tuple of the actual camera frame
            mode: Detection mode ('qr' or 'platform')
            confidence: [0-1] detection confidence for adaptive deadband

        Returns:
            vx: Forward velocity command (NED local frame X, m/s)
            vy: Right velocity command (NED local frame Y, m/s)
            aligned: True if centering errors are stable within the deadzone
        """
        cx, cy = center
        current_time = time.time()

        # Select context-specific tuning
        if mode == 'platform':
            deadzone = self.platform_deadzone_px
            stable_req = self.platform_stable_required
            marker_size_cm = self.platform_size_cm
        else:
            deadzone = self.qr_deadzone_px
            stable_req = self.qr_stable_required
            marker_size_cm = self.qr_size_cm

        # CHANGE 1: scale deadzone inversely with confidence — low confidence
        # widens the gate so jittery detections don't drive corrections.
        effective_deadzone = deadzone * (1.0 + (1.0 - max(0.0, min(1.0, confidence))) * 0.5)

        # Dynamically support actual frame dimensions if passed
        width = frame_size[0] if frame_size is not None else self.image_width
        height = frame_size[1] if frame_size is not None else self.image_height

        img_cx = width / 2.0
        img_cy = height / 2.0

        # ── CHANGE 1: EMA-smoothed center (kills single-frame pixel jitter) ─
        scx, scy = self._ema_center(cx, cy)

        err_x_px = scx - img_cx
        err_y_px = scy - img_cy

        if abs(err_x_px) < effective_deadzone:
            err_x_px = 0.0
        if abs(err_y_px) < effective_deadzone:
            err_y_px = 0.0

        # Track error for oscillation detection (x-axis sign flips)
        self._error_history.append(err_x_px)

        # Estimate distance from target for metric scale conversions
        if pixel_width is not None and pixel_width > 0:
            focal_length_px = (width / 2.0) / np.tan(self.fov_horizontal_rad / 2.0)
            real_width_m = marker_size_cm / 100.0
            distance_m = (real_width_m * focal_length_px) / pixel_width
        else:
            distance_m = altitude_m

        px_to_m_ratio = (2.0 * distance_m * np.tan(self.fov_horizontal_rad / 2.0)) / width

        error_x_m = err_x_px * px_to_m_ratio
        error_y_m = err_y_px * px_to_m_ratio

        if self.invert_x:
            error_x_m = -error_x_m
        if self.invert_y:
            error_y_m = -error_y_m

        # ── CHANGE 1: adaptive gain scaling ──────────────────────────────────
        gain_scale = self._adaptive_gain_scale(err_x_px, effective_deadzone)

        # Temporarily scale PID gains (save/restore to avoid mutating config)
        saved = [(p.kp, p.ki, p.kd) for p in (self.pid_x, self.pid_y)]
        for pid in (self.pid_x, self.pid_y):
            pid.kp *= gain_scale
            pid.ki *= gain_scale
            pid.kd *= gain_scale

        vy = self.pid_x.update(error_x_m, current_time)
        vx = self.pid_y.update(error_y_m, current_time)

        # Restore original gains
        for pid, (kp, ki, kd) in zip((self.pid_x, self.pid_y), saved):
            pid.kp, pid.ki, pid.kd = kp, ki, kd

        # ── CHANGE 1: velocity EMA smoothing ────────────────────────────────
        vx, vy = self._ema_velocity(vx, vy)

        # ── CHANGE 1: oscillation damping ───────────────────────────────────
        if self._detect_oscillation():
            vx *= 0.5
            vy *= 0.5
            logger.debug("Oscillation detected — halving velocity commands")

        # Check alignment stability
        if err_x_px == 0.0 and err_y_px == 0.0:
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        aligned = self.stable_counter >= stable_req

        if int(current_time) % 5 == 0:
            logger.debug(
                f"Aligning [{mode}]: px_err=({err_x_px:.0f},{err_y_px:.0f}) "
                f"m_err=({error_x_m:.2f},{error_y_m:.2f}) "
                f"cmd_vel=({vx:.2f},{vy:.2f}) "
                f"gain={gain_scale:.2f} invert=({self.invert_x},{self.invert_y}) "
                f"stable={self.stable_counter}/{stable_req}"
            )

        return float(vx), float(vy), aligned
