"""Camera auto-detect + capture threads.

Backends (per-camera `kind` in config):
  rpi   picamera2 / CSI stack (Pi 5). Roles by SENSOR: imx708 -> cam1 front,
        imx477 -> cam2 bottom; unknown sensors fill by index order.
  usb   any OpenCV-readable source: V4L2 index (laptop webcam), /dev/videoN,
        http/mjpeg/rtsp URL, or a GStreamer pipeline (Gazebo RTP stream).
  url   alias of usb (makes stream configs self-documenting).
  file  bench playback: a video file or a directory of images, looped.
  mirror  display-only alias of another camera (no device of its own):
          bench use is cam1 mirroring the single laptop webcam so the
          UI's CAM1 ("Pi Cam 3") tile lights up without a second
          VideoCapture on the same device (fails on Windows, races V4L2).

Each camera runs a capture thread holding the latest BGR frame + an
annotated JPEG for the UI poll endpoint. Mirror cams are skipped by
detection workers (display_only) and forward control tuning to their
source. `usb` cameras accept an optional `controls:` map of V4L2/DShow
properties (brightness/contrast/saturation/hue/gain/exposure/...) —
best-effort per driver, with set-vs-actual logged.
"""
import glob
import logging
import os
import threading
import time

log = logging.getLogger("cameras")

#: Where libcamera keeps per-sensor tuning files (Pi 5 = pisp, older = vc4).
IPA_TUNING_DIRS = ("/usr/share/libcamera/ipa/rpi/pisp",
                   "/usr/share/libcamera/ipa/rpi/vc4")


def tuning_algorithms(tuning):
    """Algorithm names in a libcamera tuning file, e.g. ['rpi.agc', 'rpi.af'].

    The file shape is `"algorithms": [{"rpi.agc": {...}}, ...]` — a LIST of
    single-key dicts, which is worth stating because collapsing it to the
    first entry makes every sensor look as if it only had `rpi.black_level`.
    """
    out = []
    for algo in (tuning or {}).get("algorithms") or []:
        if isinstance(algo, dict):
            out.extend(str(k) for k in algo)
        elif algo:
            out.append(str(algo))
    return out


def tuning_has_af(tuning):
    """Does this tuning file carry the AUTOFOCUS algorithm (rpi.af)?

    This — not the camera's control list — decides whether autofocus can work.
    Verified against raspberrypi/libcamera: the stock `imx477.json` has no
    `rpi.af`, while `imx708.json` (Pi Cam 3) does. libcamera still ADVERTISES
    AfMode/LensPosition for IMX477, so `camera_controls` claims autofocus is
    possible and then only logs "Could not set AF_MODE - no AF algorithm".
    """
    return any(name.split(".")[-1] == "af" for name in tuning_algorithms(tuning))


def default_tuning_path(sensor, dirs=None):
    """The `<sensor>.json` libcamera would load when we pass no tuning file.

    `dirs` is resolved at CALL time so a test (or a different install layout)
    can patch IPA_TUNING_DIRS — a bound default would freeze the module value.
    """
    if not sensor:
        return None
    for d in (IPA_TUNING_DIRS if dirs is None else dirs):
        p = os.path.join(d, "%s.json" % sensor)
        if os.path.isfile(p):
            return p
    return None

try:
    from picamera2 import Picamera2
    _HAVE_PICAM = True
except Exception as e:
    Picamera2 = None
    _HAVE_PICAM = False
    _PICAM_ERR = e

try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False


def list_cameras():
    """[(index, model, full_info_dict), ...] — [] when picamera2 is missing."""
    if not _HAVE_PICAM:
        log.warning("picamera2 unavailable (%r)", _PICAM_ERR)
        return []
    try:
        infos = Picamera2.global_camera_info()
    except Exception as e:
        log.warning("global_camera_info failed: %r", e)
        return []
    out = []
    for i, info in enumerate(infos):
        out.append((i, str(info.get("Model", "?")).lower(), dict(info)))
    return out


def list_video_devices():
    """V4L2 device nodes present on this machine (laptop webcams)."""
    return sorted(glob.glob("/dev/video*"))


def v4l2_capture_devices():
    """USB V4L2 capture devices on this Linux box (real_pi auto-detect).

    Returns [{node: '/dev/videoN', name, vidpid: 'vvvv:pppp', usb_desc,
    is_capture}] for devices that actually hang off a USB parent — CSI
    (libcamera) nodes have NO usb parent, so they are excluded here and
    stay the territory of list_cameras() (picamera2). Linux only; []
    anywhere else.
    """
    import re
    out = []
    base = "/sys/class/video4linux"
    for entry in sorted(glob.glob(base + "/video*"),
                        key=lambda p: int(re.search(r"video(\d+)$", p).group(1))
                        if re.search(r"video(\d+)$", p) else 999):
        try:
            with open(entry + "/name") as f:
                name = f.read().strip()
        except Exception:
            continue
        vidpid = None
        usb_desc = ""
        p = entry + "/device"
        for _ in range(10):
            try:
                if os.path.islink(p):
                    p = os.path.realpath(p)
            except Exception:
                break
            m = re.search(r"(.*usb)usb([0-9]+)$", p)
            if m:
                usb_dir = m.group(1) + "usb" + m.group(2)
                try:
                    with open(usb_dir + "/idVendor") as f:
                        v = f.read().strip()
                    with open(usb_dir + "/idProduct") as f:
                        pr = f.read().strip()
                    vidpid = v.lower() + ":" + pr.lower()
                except Exception:
                    vidpid = None
                try:
                    with open(usb_dir + "/manufacturer") as f:
                        man = f.read().strip()
                    with open(usb_dir + "/product") as f:
                        prod = f.read().strip()
                    usb_desc = (man + " " + prod).strip()
                except Exception:
                    usb_desc = ""
                break
            parent = os.path.dirname(p)
            if parent == p:
                break
            p = parent
        if vidpid is None:
            continue  # CSI / non-USB — not an auto USB slot
        try:
            with open(entry + "/video_index") as f:
                vidx = int(f.read().strip())
        except Exception:
            vidx = -1
        # USB cams expose capture + metadata nodes; capture is index 0.
        is_capture = (vidx == 0) and ("metadata" not in name.lower())
        out.append({"node": "/dev/" + os.path.basename(entry),
                    "name": name, "vidpid": vidpid,
                    "usb_desc": usb_desc, "is_capture": is_capture})
    return out


class _BaseCamera:
    kind = "base"

    #: Contract tuning shape (api-contract.md Pi REST): the UI sliders speak
    #: these units. Defaults mirror the UI slider defaults so a fresh seed is
    #: a no-op round-trip.
    #: `frame_duration_us` is the ceiling the auto-exposure may expose to
    #: (FrameDurationLimits): with the default 33333 the AE can never expose
    #: longer than 30 fps allows, which is what makes night scenes black no
    #: matter what the gain does. The other three are ISP/AE hints the night
    #: profiles set (long exposure mode, high-quality noise reduction, EV bias).
    TUNING_DEFAULTS = {"exposure_us": 8333, "gain_db": 6.0, "af_mode": "auto",
                       "adaptive": True, "brightness": 0.0, "contrast": 1.0,
                       "saturation": 1.0, "sharpness": 1.0,
                       "frame_duration_us": 33333, "ae_exposure_mode": "normal",
                       "noise_reduction": "fast", "exposure_value": 0.0}

    #: Seconds between exposure-rescue actions (see rescue_dark_frame).
    RESCUE_COOLDOWN = 5.0

    #: QR boost defaults — makes QR pop more than ground
    QR_BOOST_DEFAULTS = {"qr_boost_enabled": False, "qr_boost_mode": "qr_boost_day",
                         "qr_software_enhance": True, "qr_enhance_mode": "qr_boost",
                         "detect_raw": True}

    def __init__(self, name, index, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80):
        self.name = name            # cam1 | cam2
        self.index = index          # backend handle (picam idx / V4L2 idx / -1)
        self.size = tuple(size)     # (w, h)
        self.hfov_deg = float(hfov_deg)
        self.facing = facing        # front | bottom
        self.rotation_deg = int(rotation_deg)
        self.jpeg_quality = int(jpeg_quality)
        self.model = "?"
        self.device_desc = ""       # e.g. "csi imx708" / "v4l2 /dev/video1 (174c:55e8)"
        self.display_only = False  # True for mirrors: UI tile only, no detector worker
        self.tuning = dict(self.TUNING_DEFAULTS)
        self.qr_boost = dict(self.QR_BOOST_DEFAULTS)
        self._lock = threading.Lock()
        self._frame = None
        self._frame_raw = None      # raw frame before software enhance
        self._enh_cache = None      # (frame_count, boosted) — see _enhanced_for
        self._frame_ts = 0.0
        self._frames = 0
        self._overlay = []          # [(x,y,w,h,label), ...] drawn on stream
        self._stop = threading.Event()
        self._thread = None
        self.running = False
        # real_pi: no-camera-crash watchdog — if no frame arrives for
        # reopen_after_s the capture is restarted IN the capture thread
        # (close -> open). Bounded by a cooldown; reported in status().
        self.reopen_after_s = 10.0
        self.reopen_cooldown_s = 15.0
        # exposure rescue state (see rescue_dark_frame)
        self.frame_mean = -1.0        # last measured brightness (subsampled)
        self._rescue_on = False       # AE currently owns exposure after a rescue
        self._rescue_last = 0.0
        self.restarts = 0
        self._fail_since = None
        self._last_reopen = 0.0

    # -- lifecycle (backends implement _open/_grab/_close) ------------------
    def _open(self):
        raise NotImplementedError

    def _grab(self):
        """Return a fresh BGR numpy frame, or None when none is ready."""
        raise NotImplementedError

    def _close(self):
        pass

    def start(self):
        self._open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="cam-" + self.name,
                                        daemon=True)
        self._thread.start()
        self.running = True
        log.info("%s started: kind=%s size=%s facing=%s",
                 self.name, self.kind, self.size, self.facing)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self._close()
        except Exception:
            pass
        self.running = False

    def _apply_rotation(self, img):
        r = self.rotation_deg % 360
        if r == 0 or not _HAVE_CV2:
            return img
        try:
            if r == 90:
                return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            if r == 180:
                return cv2.rotate(img, cv2.ROTATE_180)
            if r == 270:
                return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        except Exception:
            pass
        return img

    def _loop(self):
        # The software QR boost is computed on demand (see latest), not here.

        while not self._stop.is_set():
            try:
                arr = self._grab()
            except Exception:
                arr = None
            if arr is None:
                # real_pi: stale-frame watchdog. A camera whose grab keeps
                # failing (unplugged, libcamera hiccup, CSI bus error) must
                # not hold the mission's video hostage — restart the
                # capture in-thread after reopen_after_s, with cooldown so
                # a truly-dead device cannot trigger a restart storm.
                now = time.time()
                if self._fail_since is None:
                    self._fail_since = now
                elif (now - self._fail_since > self.reopen_after_s and
                      now - self._last_reopen > self.reopen_cooldown_s):
                    self._last_reopen = now
                    self._fail_since = now
                    self.restarts += 1
                    log.warning("%s: no frames for %.0fs — restarting "
                                "capture (restart #%d)", self.name,
                                now - self._fail_since, self.restarts)
                    try:
                        self._close()
                        time.sleep(0.3)
                        self._open()
                        log.info("%s: capture restarted after %.0fs dark",
                                 self.name, now - self._fail_since)
                    except Exception as e:
                        log.warning("%s: capture restart failed: %r — "
                                    "retrying after cooldown", self.name, e)
                time.sleep(0.1)
                continue
            self._fail_since = None
            try:
                arr = self._apply_rotation(arr)
                h, w = arr.shape[:2]
                if (w, h) != tuple(self.size) and _HAVE_CV2:
                    arr = cv2.resize(arr, tuple(self.size))
            except Exception:
                pass

            # Only the RAW frame is stored here. The software QR boost (CLAHE +
            # unsharp: 100+ ms per 2028x1520 frame on a Pi 5) is computed on
            # demand in latest(), so a consumer that does not want it (the
            # detector with detect_raw, a bench with no window) costs nothing.
            raw = arr
            with self._lock:
                self._frame_raw = raw
                self._frame_ts = time.time()
                self._frames += 1

            # Brightness of every frame is cheap on a subsample, and it is what
            # keeps a manual (daylight) profile from leaving the camera blind at
            # night — see rescue_dark_frame().
            try:
                self.frame_mean = float(raw[::12, ::12].mean())
                line = self.rescue_dark_frame(self.frame_mean)
                if line:
                    log.warning("%s: %s", self.name, line)
            except Exception as e:                 # never break the capture
                log.debug("%s: brightness/rescue failed: %r", self.name, e)

    # -- consumers ------------------------------------------------------
    def latest(self, raw=False, detect=False):
        """(frame_bgr_or_None, ts, count). Frame is a shared reference —
        consumers must not modify it in place.
        raw=True    -> the frame before the software QR boost (debug/stream).
        detect=True -> whichever frame the DETECTOR should see: config
                       `qr_boost.detect_raw` decides between the raw frame
                       (what the HEF was calibrated and trained on — the
                       default) and the boosted one.
        The boost is computed once per frame, on the first caller that asks
        for it, outside the lock — never on the capture thread."""
        with self._lock:
            raw_f, ts, n = self._frame_raw, self._frame_ts, self._frames
        if raw or raw_f is None or (detect and self.qr_boost.get("detect_raw")):
            return raw_f, ts, n
        return self._enhanced_for(raw_f, n), ts, n

    def _enhanced_for(self, raw_f, n):
        with self._lock:
            cache = self._enh_cache
        if cache is not None and cache[0] == n:
            return cache[1]
        enh = raw_f
        try:
            if self.qr_boost.get("qr_software_enhance"):
                mode = self.qr_boost.get("qr_enhance_mode", "qr_boost")
                if mode and mode != "none":
                    from qr_camera_boost import enhance_qr_frame
                    enh = enhance_qr_frame(raw_f, mode=mode)
        except Exception as e:
            log.debug("%s qr enhance failed: %r", self.name, e)
            enh = raw_f
        with self._lock:
            self._enh_cache = (n, enh)
        return enh

    def set_overlay(self, boxes):
        """boxes: [(x,y,w,h,label), ...] drawn onto the streamed JPEG."""
        with self._lock:
            self._overlay = list(boxes or [])

    # -- contract tuning (UI sliders) ------------------------------------
    def apply_tuning(self, tuning):
        """Store contract tuning; backends override to drive hardware.
        Accepts the mock's `gain` alias for `gain_db`. Returns get_tuning()."""
        for k in self.TUNING_DEFAULTS:
            if tuning is not None and k in tuning:
                self.tuning[k] = tuning[k]
        if tuning is not None and "gain" in tuning and "gain_db" not in tuning:
            self.tuning["gain_db"] = tuning["gain"]
        # QR boost tuning
        for k in self.QR_BOOST_DEFAULTS:
            if tuning is not None and k in tuning:
                self.qr_boost[k] = tuning[k]
        return self.get_tuning()

    def get_tuning(self):
        """Contract-shaped tuning incl. the mock's `gain` alias + QR boost."""
        t = dict(self.tuning)
        t["gain"] = t.get("gain_db")
        # include QR boost for UI
        for k, v in self.qr_boost.items():
            t[k] = v
        return t

    def rescue_dark_frame(self, mean, std=None, now=None):
        """A MANUAL profile + a dark/blown frame -> hand exposure back to the AE.

        Returns the line to show, or None. This is the fix for the 2026-10-13
        field report: the presets impose manual daylight exposures (5 ms /
        1.26x), which in a night room produced frame_mean=4 — a black window and
        no detections, with nothing able to recover because AeEnable=false also
        pins the GAIN. The rescue turns AE back on with a long frame-duration
        budget (the only way the sensor can collect more light), and restores
        the operator's profile by itself once the scene is bright again.
        """
        import time as _time
        now = _time.time() if now is None else now
        try:
            from qr_camera_boost import dark_rescue, recovered
        except Exception:                      # module missing: never break capture
            return None
        slow = (now - self._rescue_last) < self.RESCUE_COOLDOWN
        if self._rescue_on:
            if slow or not recovered(mean):
                return None
            self._rescue_on = False
            self._rescue_last = now
            want = self.qr_boost.get("qr_boost_mode")
            if want:
                self.apply_qr_profile(want)
            return ("light is back (mean %.0f) — re-applying the '%s' "
                    "camera profile" % (mean, want or "previous"))
        over, why = dark_rescue(self.tuning, mean, std)
        if not over or slow:
            return None
        self._rescue_last = now
        self._rescue_on = True
        self.apply_tuning(over)
        return ("%s — AUTO EXPOSURE restored (up to %.0f ms, long-exposure "
                "mode, high-quality NR). It stays on until the scene is "
                "brighter (mean >= 60), then the profile comes back by itself."
                % (why, float(over.get("frame_duration_us", 0)) / 1000.0))

    def apply_qr_profile(self, profile_name):
        """Apply a QR boost ISP profile (see qr_camera_boost.QR_BOOST_PROFILES)."""
        try:
            from qr_camera_boost import get_profile
            prof = get_profile(profile_name)
            if not prof:
                log.warning("%s unknown QR profile %r", self.name, profile_name)
                return self.get_tuning()
            # Map profile to contract tuning
            tuning = {k: v for k, v in prof.items() if k in self.TUNING_DEFAULTS}
            # QR boost meta
            self.qr_boost["qr_boost_enabled"] = profile_name != "normal"
            self.qr_boost["qr_boost_mode"] = profile_name
            # Apply ISP tuning via backend
            return self.apply_tuning(tuning)
        except Exception as e:
            log.warning("%s apply_qr_profile %r failed: %r", self.name, profile_name, e)
            return self.get_tuning()

    def jpeg(self, max_width=960, quality=None, raw=False):
        """Annotated JPEG bytes for /api/camera/frame/<cam> (or None).
        raw=False = enhanced frame (QR pops), raw=True = raw frame."""
        frame, ts, _ = self.latest(raw=raw)
        if frame is None or not _HAVE_CV2:
            return None
        try:
            img = frame.copy()  # don't modify shared frame
            h, w = img.shape[:2]
            if w > max_width:
                s = max_width / w
                img = cv2.resize(img, (max_width, int(h * s)))
                k = s
            else:
                k = 1.0
            with self._lock:
                ov = list(self._overlay)
            for (x, y, ww, hh, label) in ov:
                p1 = (int(x * k), int(y * k))
                p2 = (int((x + ww) * k), int((y + hh) * k))
                cv2.rectangle(img, p1, p2, (0, 255, 0), 2)
                if label:
                    cv2.putText(img, str(label)[:24], (p1[0], max(12, p1[1] - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            q = int(quality) if quality else self.jpeg_quality
            ok, buf = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, q])
            return bytes(buf) if ok else None
        except Exception as e:
            log.debug("%s jpeg failed: %r", self.name, e)
            return None

    def status(self):
        _, ts, n = self.latest()
        return {"name": self.name, "kind": self.kind, "index": self.index,
                "model": self.model, "device_desc": self.device_desc,
                "size": list(self.size),
                "facing": self.facing, "hfov_deg": self.hfov_deg,
                "running": self.running, "frames": n,
                "age_s": round(time.time() - ts, 2) if ts else None,
                "restarts": self.restarts,
                "qr_boost": dict(self.qr_boost),
                "tuning": dict(self.tuning)}


class Camera(_BaseCamera):
    """Pi CSI camera via picamera2 (kind: rpi)."""
    kind = "rpi"

    def __init__(self, name, index, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80, autofocus=None,
                 tuning_file=None, lens_position=None):
        super().__init__(name, index, size, hfov_deg, facing,
                         rotation_deg, jpeg_quality)
        self._picam = None
        # None = "let the sensor decide" (imx708 autofocuses by default, every
        # other sensor does not). True/False = explicit config decision.
        # Needed because a motorised-focus third-party module (Arducam UC-517 =
        # IMX477 + VCM lens) IS autofocus, while a Waveshare IMX477 IR-CUT is
        # NOT — same sensor string, opposite answer.
        self.autofocus = None if autofocus is None else bool(autofocus)
        # Arducam's motorised-focus cameras need THEIR tuning file for AfMode /
        # LensPosition to exist at all on Pi 5 (pisp IPA):
        #   /usr/share/libcamera/ipa/rpi/pisp/imx477_af.json
        # Without it the camera opens fine but has no focus controls.
        self.tuning_file = str(tuning_file).strip() if tuning_file else None
        # Filled at open: the focus-related control names the camera exposes
        # ([] = none at all). None = not opened yet / unknown.
        self.focus_controls = None
        # Does the LOADED TUNING carry the rpi.af algorithm? True/False, or
        # None when it could not be determined. This is the real gate for
        # autofocus: the control list is advertised even when the algorithm is
        # absent, and libcamera merely LOGS "Could not set AF_MODE - no AF
        # algorithm" instead of raising.
        self.af_algorithm = None
        #: Which tuning file was actually used (path or name) — for the log.
        self.tuning_source = self.tuning_file
        # Pinned manual focus, in dioptres (libcamera's LensPosition unit:
        # 0 = infinity, 1 = 1 m, 0.067 = 15 m). A downward camera looking at
        # the ground from a fixed altitude does not want continuous autofocus
        # hunting over low-texture ground — one measured position is better
        # and reproducible. Ignored while autofocus is requested (logged).
        try:
            self.lens_position = (None if lens_position in (None, "")
                                  else float(lens_position))
        except (TypeError, ValueError):
            log.warning("%s lens_position %r is not a number — ignoring",
                        name, lens_position)
            self.lens_position = None

    def wants_autofocus(self):
        """Explicit config wins; otherwise only the Pi Cam 3 (imx708)."""
        if self.autofocus is not None:
            return self.autofocus
        return "imx708" in (self.model or "")

    def wants_lens_position(self):
        """The LensPosition to apply at open, or None to leave the lens as-is.

        Autofocus wins when both are configured: the position would be
        overwritten by the AF loop, so applying it would be a lie. `_open()`
        logs the conflict once.
        """
        if self.lens_position is None:
            return None
        if self.wants_autofocus():
            return None
        return self.lens_position

    def status(self):
        """Base status + what the OPERATOR needs to judge focus.

        `af_mode` in `tuning` is what config ASKS for; `focus_controls` is
        what the camera can actually DO. When it is `[]`, autofocus is
        configured against a lens no software can move, so the UI/bring-up
        log must say that instead of showing a reassuring `af_mode: auto`.
        """
        st = super().status()
        st["autofocus"] = self.wants_autofocus()
        st["lens_position"] = self.lens_position
        st["focus_controls"] = (None if self.focus_controls is None
                                else list(self.focus_controls))
        st["af_algorithm"] = self.af_algorithm
        st["tuning_source"] = self.tuning_source
        # False as soon as EITHER the controls or the tuning's rpi.af is
        # missing — the control list alone would report a comforting True.
        st["focus_ok"] = self.focus_possible()
        return st

    def _load_tuning(self):
        """Tuning file as a dict, or None. Never raises.

        `tuning_file` may be the NAME of a file the libcamera IPA dir already
        has (imx477_af.json) OR a PATH to one we ship ourselves — an AF tuning
        that lives in the repo needs no sudo and survives OS updates.
        """
        if not self.tuning_file:
            return None
        import json
        spec = self.tuning_file
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = []
        if os.path.isabs(spec):
            candidates.append(spec)
        else:
            candidates.append(os.path.join(here, spec))
            candidates.append(os.path.join(os.getcwd(), spec))
            # models/ is where the repo keeps its weights, so it is the
            # obvious no-sudo drop-in for a tuning file we ship ourselves.
            candidates.append(os.path.join(here, "models", spec))
        # de-dup: cwd == the repo dir on the Pi, so the same path otherwise
        # shows up twice in the warning and looks like a broken lookup
        seen, uniq = set(), []
        for p in candidates:
            if p not in seen:
                seen.add(p)
                uniq.append(p)
        candidates = uniq
        # 1) A file we can read ourselves: absolute, repo-relative, in the
        #    working dir, or shipped in models/. A copy in the repo needs no
        #    sudo and survives OS updates, so it wins over the system one.
        for path in candidates:
            if os.path.isfile(path):
                try:
                    with open(path) as f:
                        tuning = json.load(f)
                    log.info("%s using tuning file %s", self.name, path)
                    return tuning
                except Exception as e:
                    log.warning("%s tuning file %s unreadable (%r) — using the "
                                "default tuning; autofocus controls may be "
                                "absent", self.name, path, e)
                    return None
        # 2) A bare NAME is resolved by libcamera in its own IPA dir
        #    (/usr/share/libcamera/ipa/rpi/pisp/...) — which is exactly where
        #    Arducam's imx477_af.json lands when it is installed system-wide.
        #    An explicit path that does not exist is an operator error instead,
        #    so only names get this second chance.
        if "/" not in spec:
            try:
                tuning = Picamera2.load_tuning_file(spec)
                log.info("%s using tuning file %s (libcamera IPA dir)",
                         self.name, spec)
                return tuning
            except Exception as e:
                log.warning("%s tuning file %r not found — looked in %s and in "
                            "libcamera's IPA dir (%r). Using the default "
                            "tuning; autofocus controls may be absent",
                            self.name, spec, ", ".join(candidates), e)
                return None
        log.warning("%s tuning file %r not found (looked in %s) — using the "
                    "default tuning; autofocus controls may be absent",
                    self.name, spec, ", ".join(candidates))
        return None

    def focus_possible(self):
        """True / False / None (unknown) — CAN this camera autofocus at all?

        False is provable from two independent facts, either of which is
        enough: no focus controls on the camera, or no `rpi.af` algorithm in
        the tuning file that was loaded.
        """
        if self.focus_controls == [] or self.af_algorithm is False:
            return False
        if self.focus_controls and self.af_algorithm is True:
            return True
        return None

    def _resolve_af_algorithm(self, tuning):
        """Set self.af_algorithm / self.tuning_source from the tuning in use."""
        import json
        if not self.model and self._picam is not None:
            try:
                props = getattr(self._picam, "camera_properties", None) or {}
                if props.get("Model"):
                    self.model = str(props["Model"])
            except Exception:
                pass
        info, source = tuning, self.tuning_file
        if info is None:
            # We pass nothing, so libcamera uses <sensor>.json from its own
            # IPA dir — read exactly that file to answer the question.
            source = default_tuning_path(self.model)
            if source:
                try:
                    with open(source) as f:
                        info = json.load(f)
                except Exception as e:
                    log.debug("%s: could not read %s: %r", self.name, source, e)
                    info = None
        self.af_algorithm = (None if info is None else tuning_has_af(info))
        self.tuning_source = source

    def _open(self):
        if not _HAVE_PICAM:
            raise RuntimeError("picamera2 not installed: %r" % (_PICAM_ERR,))
        tuning = self._load_tuning()
        self._picam = (Picamera2(self.index, tuning=tuning)
                       if tuning is not None else Picamera2(self.index))
        # The SENSOR NAME decides which default tuning libcamera picks, and
        # the camera knows it even when config never said: read it from the
        # opened device before judging the tuning (rig-supplied model is kept
        # when present).
        if not self.model:
            try:
                props = getattr(self._picam, "camera_properties", None) or {}
                if props.get("Model"):
                    self.model = str(props["Model"])
            except Exception as e:
                log.debug("%s: no camera_properties: %r", self.name, e)
        try:
            self._resolve_af_algorithm(tuning)
        except Exception as e:
            log.debug("%s: AF-algorithm check failed: %r", self.name, e)
        cfg = self._picam.create_video_configuration(
            main={"size": tuple(self.size), "format": "RGB888"})
        self._picam.configure(cfg)
        # Which focus controls exist at all? Ask before trying to set them:
        #   AfMode/LensPosition present -> the VCM/lens is really driven
        #   []                          -> no AF algorithm (stock overlay or a
        #                                  module with no motor); libcamera
        #                                  only WARNs, it does not raise, so
        #                                  without this check the config
        #                                  silently does nothing.
        try:
            names = sorted(k for k in (self._picam.camera_controls or {})
                           if any(s in k.lower()
                                  for s in ("af", "lens", "focus")))
            self.focus_controls = names
            if names:
                log.info("%s focus controls: %s (AF algorithm in %s: %s)",
                         self.name, ", ".join(names),
                         self.tuning_source or self.model,
                         self.af_algorithm)
            if self.wants_autofocus() and self.focus_possible() is not True:
                how = []
                if not names:
                    how.append("the camera exposes no AfMode/LensPosition "
                               "controls at all")
                if self.af_algorithm is False:
                    how.append("the tuning in use (%s) has no rpi.af algorithm"
                               % (self.tuning_source or self.model))
                elif self.af_algorithm is None:
                    how.append("the tuning in use could not be inspected, so "
                               "rpi.af presence is unknown")
                log.warning("%s: autofocus is configured but CANNOT work — %s. "
                            "libcamera only warns ('Could not set AF_MODE - no "
                            "AF algorithm'); nothing here raises. Fix: load a "
                            "tuning file that HAS an AF algorithm (Arducam's "
                            "imx477_af.json for the UC-517 — drop it next to "
                            "config.yaml, in models/, or in the IPA dir), or "
                            "set cameras.%s.autofocus: false and rely on the "
                            "fixed lens (focused at ~infinity it is sharp from "
                            "~0.9 m to infinity, so 15 m is covered)",
                            self.name, "; ".join(how), self.facing or "?")
        except Exception as e:
            log.debug("%s: could not read camera_controls: %r", self.name, e)

        controls = {"AeEnable": True}
        if self.wants_autofocus():
            if self.lens_position is not None:
                log.warning("%s has BOTH autofocus and lens_position=%s — "
                            "autofocus wins; drop one of them in config.yaml",
                            self.name, self.lens_position)
            try:
                from libcamera import controls as CTRLS
                controls["AfMode"] = CTRLS.AfModeEnum.Continuous
            except Exception as e:
                log.debug("%s: autofocus requested but unavailable: %r",
                          self.name, e)
        else:
            pos = self.wants_lens_position()
            if pos is not None:
                controls["LensPosition"] = float(pos)
        try:
            self._picam.set_controls(controls)
        except Exception as e:
            # Requesting autofocus on a camera whose lens the kernel does not
            # drive is a real configuration/hardware mismatch — it must be
            # visible, not a debug line. Everything else stays quiet.
            if "AfMode" in controls:
                log.warning("%s: autofocus requested but the camera rejected "
                            "it (%r) — the VCM/lens is not available. Use "
                            "cameras.%s.lens_position or autofocus: false. "
                            "Check the overlay (`dtoverlay -h imx477`) and "
                            "run tools/focus_sweep.py.",
                            self.name, e, self.facing or "?")
            elif "LensPosition" in controls:
                log.warning("%s: lens_position=%s rejected (%r) — no manual "
                            "focus control on this camera",
                            self.name, controls.get("LensPosition"), e)
            else:
                log.debug("%s controls failed: %r", self.name, e)
        self._picam.start()
        time.sleep(0.8)  # let AE/AWB settle

    def _grab(self):
        try:
            arr = self._picam.capture_array("main")
        except Exception:
            time.sleep(0.1)
            return None
        if _HAVE_CV2:
            try:
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            except Exception:
                pass
        return arr

    def _close(self):
        try:
            self._picam and self._picam.stop()
        except Exception:
            pass
        self._picam = None

    def apply_tuning(self, tuning):
        """Contract tuning -> libcamera controls (the contract was designed
        for these units: ExposureTime µs, 0..2 scalers, -100..100 brightness)."""
        t = super().apply_tuning(tuning)
        if not _HAVE_PICAM or self._picam is None:
            return t
        ctrls = {}
        if "exposure_us" in (tuning or {}) or "adaptive" in (tuning or {}):
            ctrls["AeEnable"] = bool(t["adaptive"])
            if not t["adaptive"]:
                ctrls["ExposureTime"] = int(max(100, min(1000000, float(t["exposure_us"]))))
        if "gain_db" in (tuning or {}) or "gain" in (tuning or {}) or "adaptive" in (tuning or {}):
            if not t["adaptive"]:
                ctrls["AnalogueGain"] = 10.0 ** (float(t["gain_db"]) / 20.0)
        if tuning is not None and "brightness" in tuning:
            ctrls["Brightness"] = max(-1.0, min(1.0, float(t["brightness"]) / 100.0))
        for k in ("contrast", "saturation", "sharpness"):
            if tuning is not None and k in tuning:
                ctrls[k.capitalize()] = max(0.0, float(t[k]))
        if tuning is not None and "af_mode" in tuning and self.wants_autofocus():
            try:
                from libcamera import controls as LC
                ctrls["AfMode"] = {"manual": LC.AfModeEnum.Manual,
                                   "continuous": LC.AfModeEnum.Continuous}.get(
                                       t["af_mode"], LC.AfModeEnum.Auto)
            except Exception:
                pass
        if tuning is not None and "frame_duration_us" in tuning:
            # The ceiling the AE may expose to. Without this, a night profile
            # cannot collect more light than 1/30 s no matter the gain.
            v = int(max(1000, min(1000000, float(t["frame_duration_us"]))))
            ctrls["FrameDurationLimits"] = (v, v)
        if tuning is not None and "ae_exposure_mode" in tuning:
            try:
                from libcamera import controls as LC
                ctrls["AeExposureMode"] = {
                    "short": LC.AeExposureModeEnum.Short,
                    "long": LC.AeExposureModeEnum.Long,
                    "custom": LC.AeExposureModeEnum.Custom,
                }.get(str(t["ae_exposure_mode"]).lower(),
                      LC.AeExposureModeEnum.Normal)
            except Exception:
                pass
        if tuning is not None and "noise_reduction" in tuning:
            try:
                from libcamera import controls as LC
                ctrls["NoiseReductionMode"] = {
                    "off": LC.NoiseReductionModeEnum.Off,
                    "fast": LC.NoiseReductionModeEnum.Fast,
                    "highquality": LC.NoiseReductionModeEnum.HighQuality,
                }.get(str(t["noise_reduction"]).lower())
            except Exception:
                pass
        if tuning is not None and "exposure_value" in tuning:
            ctrls["ExposureValue"] = float(t["exposure_value"])
        if ctrls:
            try:
                self._picam.set_controls(ctrls)
                log.info("%s tuning: %s", self.name, ctrls)
            except Exception as e:
                log.warning("%s tuning %s failed: %r", self.name, ctrls, e)
        return t


class USBCamera(_BaseCamera):
    """OpenCV source: V4L2 index, /dev/videoN, URL, or GStreamer pipeline."""
    kind = "usb"

    #: Friendly control name -> cv2 CAP_PROP attribute. Ranges are
    #: DRIVER-specific (V4L2 vs DShow vs MSMF all differ) — query with
    #: get_controls() before baking numbers into config.
    CONTROL_PROPS = {
        "brightness": "CAP_PROP_BRIGHTNESS",
        "contrast": "CAP_PROP_CONTRAST",
        "saturation": "CAP_PROP_SATURATION",
        "hue": "CAP_PROP_HUE",
        "gain": "CAP_PROP_GAIN",
        "exposure": "CAP_PROP_EXPOSURE",
        "autoexposure": "CAP_PROP_AUTO_EXPOSURE",
        "auto_exposure": "CAP_PROP_AUTO_EXPOSURE",
        "autofocus": "CAP_PROP_AUTOFOCUS",
        "auto_focus": "CAP_PROP_AUTOFOCUS",
        "focus": "CAP_PROP_FOCUS",
        "sharpness": "CAP_PROP_SHARPNESS",
        "autowb": "CAP_PROP_AUTO_WB",
        "auto_wb": "CAP_PROP_AUTO_WB",
        "zoom": "CAP_PROP_ZOOM",
    }

    def __init__(self, name, source, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80, backend=None, fps=30,
                 controls=None, fourcc=None):
        super().__init__(name, source if isinstance(source, int) else -1,
                         size, hfov_deg, facing, rotation_deg, jpeg_quality)
        self.source = source
        self.backend = (backend or "").lower() or None
        self.fps = float(fps or 30)
        self.controls = dict(controls or {})
        self.fourcc = (fourcc or "").upper() or None
        self._cap = None
        self._fail = 0  # self-checked fix: init fail counter for auto-reconnect
        self._anchors = {}  # driver currents at first tuning: the 1.0/0 point

    def _api(self):
        if self.backend == "gstreamer" or (
                self.backend is None and isinstance(self.source, str)
                and "!" in self.source):
            return getattr(cv2, "CAP_GSTREAMER", cv2.CAP_ANY)
        return {"v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
                "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
                "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
                "ffmpeg": getattr(cv2, "CAP_FFMPEG", cv2.CAP_ANY),
                "mjpeg": getattr(cv2, "CAP_FFMPEG", cv2.CAP_ANY),  # http mjpeg (Gazebo bridge)
                "any": cv2.CAP_ANY}.get(self.backend or "any", cv2.CAP_ANY)

    def _open(self):
        if not _HAVE_CV2:
            raise RuntimeError("opencv (cv2) not installed — needed for usb/url/file cameras")
        # Self-checked fix: URL MJPEG (Gazebo bridge) needs robust open
        # Old code failed immediately if first read failed (bridge not ready yet)
        # New: try multiple APIs, retry read, allow URL without immediate frame
        apis_to_try = []
        primary_api = self._api()
        apis_to_try.append(primary_api)
        if primary_api != cv2.CAP_ANY:
            apis_to_try.append(cv2.CAP_ANY)
            apis_to_try.append(getattr(cv2, "CAP_FFMPEG", cv2.CAP_ANY))
        tried = []
        last_err = None
        for api in apis_to_try:
            if api in tried:
                continue
            tried.append(api)
            try:
                cap = cv2.VideoCapture(self.source, api) if api != cv2.CAP_ANY else cv2.VideoCapture(self.source)
                if not cap.isOpened():
                    cap.release()
                    continue
                try:
                    if self.fourcc:
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4]))
                    # For URL, don't force size/fps — let bridge decide
                    if not (isinstance(self.source, str) and self.source.startswith("http")):
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                except Exception:
                    pass
                # Try read with retries for URL
                ok, frame = False, None
                for _ in range(5):
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        break
                    time.sleep(0.5)
                # For URL, allow open even if first read fails — _grab will retry + auto-reconnect
                if isinstance(self.source, str) and self.source.startswith("http"):
                    self._cap = cap
                    self._fail = 0
                    log.info("%s opened URL %r (backend %s) — will retry frames in _grab", self.name, self.source, api)
                    return
                if ok and frame is not None:
                    self._cap = cap
                    self._fail = 0
                    return
                cap.release()
            except Exception as e:
                last_err = e
                continue
        raise RuntimeError("cannot open video source %r (backend=%s) tried %s last_err=%r" % (
            self.source, self.backend or "any", tried, last_err))
        try:
            aw = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            ah = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            afps = cap.get(cv2.CAP_PROP_FPS)
            log.info("%s actual: %.0fx%.0f @ %.0f fps (want %s @ %s)",
                     self.name, aw, ah, afps, self.size, self.fps)
        except Exception:
            pass
        if self.controls:
            self.apply_controls(self.controls, loud=True)

    def apply_controls(self, controls, loud=False):
        """Best-effort property set. Returns {name: actual-or-None}."""
        out = {}
        if not _HAVE_CV2 or self._cap is None:
            return out
        say = log.info if loud else log.debug
        for name, val in (controls or {}).items():
            key = str(name).lower()
            attr = self.CONTROL_PROPS.get(key)
            prop = getattr(cv2, attr, None) if attr else None
            if prop is None:
                say("%s control %s: unsupported (want one of %s)",
                    self.name, name, sorted(self.CONTROL_PROPS))
                out[name] = None
                continue
            try:
                ok = self._cap.set(prop, float(val))
                actual = self._cap.get(prop)
            except Exception as e:
                say("%s control %s=%s failed: %r", self.name, name, val, e)
                out[name] = None
                continue
            self.controls[key] = val
            say("%s control %s=%s (set_ok=%s actual=%s)",
                self.name, name, val, ok, actual)
            out[name] = actual
        return out

    def get_controls(self):
        """Read back every supported property (driver ranges included)."""
        out = {}
        if not _HAVE_CV2 or self._cap is None:
            return out
        seen = set()
        for name, attr in self.CONTROL_PROPS.items():
            if attr in seen:
                continue
            seen.add(attr)
            prop = getattr(cv2, attr, None)
            if prop is None:
                continue
            try:
                out[name] = self._cap.get(prop)
            except Exception:
                pass
        return out

    def status(self):
        st = super().status()
        st["controls"] = dict(self.controls)
        return st

    def _driver_family(self):
        """'dshow' (log exposure, 0/1 auto flag, -100..100 brightness) or
        'v4l2' (100us-unit exposure, 1=manual/3=auto). Best-effort guess for
        backend:any from the OS — logged, and every set reports actuals."""
        b = (self.backend or "any").lower()
        if b in ("dshow", "msmf"):
            return "dshow"
        if b in ("v4l2",):
            return "v4l2"
        import sys
        return "dshow" if sys.platform == "win32" else "v4l2"

    def apply_tuning(self, tuning):
        """Contract tuning -> V4L2/DShow props (bench mapping).

        Driver ranges differ per vendor, so the 0..2 scalers work RELATIVE
        to the driver's own current value (snapshotted as anchors on the
        first tuning: 1.0 == 'leave the driver default alone'). Brightness
        is native -100..100 on DShow, relative on V4L2. Exposure is
        absolute per driver family. adaptive=False flips the driver toward
        manual exposure so the exposure/gain sliders bite; adaptive=True
        (the UI default) hands control back to the driver.
        """
        t = super().apply_tuning(tuning)
        if not _HAVE_CV2 or self._cap is None or not tuning:
            return t
        if not self._anchors:
            cur = self.get_controls()
            for k in ("brightness", "contrast", "saturation", "sharpness",
                      "gain", "exposure"):
                if cur.get(k) not in (None,):
                    self._anchors[k] = cur[k]
            log.info("%s tuning anchors (%s driver): %s", self.name,
                     self._driver_family(), self._anchors)
        fam = self._driver_family()

        def anchor(k):
            v = self._anchors.get(k)
            return v if isinstance(v, (int, float)) else None

        drv = {}
        for k in ("contrast", "saturation", "sharpness"):
            if k in tuning:
                a = anchor(k)
                if a:
                    drv[k] = max(0.0, a * float(t[k]))
        if "brightness" in tuning:
            if fam == "dshow":
                drv["brightness"] = max(-100.0, min(100.0, float(t["brightness"])))
            else:
                a = anchor("brightness")
                if a:
                    drv["brightness"] = max(0.0, a + (float(t["brightness"]) / 100.0) * a)
        if "gain_db" in tuning or "gain" in tuning:
            a = anchor("gain")
            if a:  # UI default 6 dB == keep the driver current
                drv["gain"] = max(0.0, a * max(0.05, float(t["gain_db"]) / 6.0))
        if "adaptive" in tuning or "exposure_us" in tuning:
            manual = not bool(t["adaptive"])
            drv["autoexposure"] = (0 if manual else 1) if fam == "dshow" else (1 if manual else 3)
            if manual and "exposure_us" in tuning:
                us = max(100.0, float(t["exposure_us"]))
                if fam == "dshow":
                    import math
                    drv["exposure"] = round(math.log2(us / 1e6))
                else:
                    drv["exposure"] = round(us / 100.0)
        if "af_mode" in tuning:
            drv["autofocus"] = 0 if t["af_mode"] == "manual" else 1
        if drv:
            applied = self.apply_controls(drv)
            log.info("%s tuning applied: %s", self.name, applied)
        return t

    def _grab(self):
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if ok and frame is not None:
            self._fail = 0
            return frame
        # MJPEG URL can drop (bridge restart, network hiccup) — auto-reconnect
        self._fail = getattr(self, '_fail', 0) + 1
        if self._fail % 10 == 0:
            log.warning("%s no frames for %d reads (source %r) — trying reopen", self.name, self._fail, self.source)
        if self._fail >= 10:
            try:
                self._close()
                time.sleep(0.5)
                self._open()
                log.info("%s reopened after %d fails", self.name, self._fail)
                self._fail = 0
                ok, frame = self._cap.read() if self._cap else (False, None)
                return frame if ok and frame is not None else None
            except Exception as e:
                log.debug("%s reopen failed: %r", self.name, e)
                time.sleep(1.0)
        return None

    def _close(self):
        try:
            self._cap and self._cap.release()
        except Exception:
            pass
        self._cap = None


class FileCamera(_BaseCamera):
    """Bench playback: video file or directory of images, looped."""
    kind = "file"
    IMG_EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".webp")

    def __init__(self, name, path, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80, fps=5):
        super().__init__(name, -1, size, hfov_deg, facing,
                         rotation_deg, jpeg_quality)
        self.path = path
        self.fps = float(fps or 5)
        self._cap = None
        self._images = []
        self._img_idx = 0
        self._img_next = 0.0

    def _open(self):
        if not _HAVE_CV2:
            raise RuntimeError("opencv (cv2) not installed — needed for usb/url/file cameras")
        if not self.path:
            raise RuntimeError("file camera needs cameras.<role>.path")
        if os.path.isdir(self.path):
            self._images = sorted(p for p in
                                  (os.path.join(self.path, f)
                                   for f in os.listdir(self.path))
                                  if p.lower().endswith(self.IMG_EXTS))
            if not self._images:
                raise RuntimeError("no images in %r" % self.path)
            probe = cv2.imread(self._images[0])
            if probe is None:
                raise RuntimeError("cannot read %r" % self._images[0])
        else:
            cap = cv2.VideoCapture(self.path)
            if not cap.isOpened():
                raise RuntimeError("cannot open video file %r" % self.path)
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                raise RuntimeError("no frames in %r" % self.path)
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self._cap = cap

    def _grab(self):
        if self._images:
            now = time.time()
            if now < self._img_next:
                return None  # hold current frame until pace interval
            self._img_next = now + 1.0 / max(0.5, self.fps)
            p = self._images[self._img_idx % len(self._images)]
            self._img_idx += 1
            return cv2.imread(p)
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:  # loop the clip
            try:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._cap.read()
            except Exception:
                return None
        return frame if ok and frame is not None else None

    def _close(self):
        try:
            self._cap and self._cap.release()
        except Exception:
            pass
        self._cap = None


class MirrorCamera(_BaseCamera):
    """Display-only alias of another camera (kind: mirror).

    No device of its own — each grab returns the source's latest frame
    (resized when our size differs). Bench use: the single laptop webcam
    (cam2/bottom, the search eye) ALSO feeds the UI's CAM1 ("Pi Cam 3")
    tile without opening the device twice. Detection workers skip
    mirrors (Mission checks display_only); the source's detection
    overlay is copied onto our JPEG so both tiles show the boxes.
    Control tuning forwards to the real device.
    """
    kind = "mirror"

    def __init__(self, name, source_cam, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80):
        super().__init__(name, source_cam.index if source_cam else -1,
                         size, hfov_deg, facing, rotation_deg, jpeg_quality)
        self.display_only = True
        self._src = source_cam
        self.model = "mirror:%s" % (source_cam.name if source_cam else "?")

    def _open(self):
        if self._src is None:
            raise RuntimeError("mirror %s has no source camera" % self.name)

    def _grab(self):
        if self._src is None:
            return None
        frame, _, _ = self._src.latest()
        return frame

    def jpeg(self, max_width=960):
        try:
            with self._src._lock:
                ov = list(self._src._overlay)
            with self._lock:
                self._overlay = ov
        except Exception:
            pass
        return super().jpeg(max_width)

    def apply_controls(self, controls, loud=False):
        fn = getattr(self._src, "apply_controls", None)
        return fn(controls, loud=loud) if fn else {}

    def get_controls(self):
        fn = getattr(self._src, "get_controls", None)
        return fn() if fn else {}

    def apply_tuning(self, tuning):
        return self._src.apply_tuning(tuning) if self._src else self.get_tuning()

    def get_tuning(self):
        return self._src.get_tuning() if self._src else super().get_tuning()

    def status(self):
        st = super().status()
        st["mirrors"] = self._src.name if self._src else None
        st["display_only"] = True
        return st


class CameraRig:
    """Owns cam1 (front) + cam2 (bottom); tolerates 0/1/2 cameras.

    real_pi auto-detect order for each ROLE (front/bottom):
      1. explicit config kind (usb/url/file/mirror) — always wins;
      2. CSI camera whose sensor matches `match_name` (e.g. imx477),
         else the role's default sensor (imx708->front, imx477->bottom),
         else by index order;
      3. USB V4L2 camera whose VID:PID matches `match_vidpid` or whose
         product name matches `match_name` (real_pi/tools/list_cams.py);
      4. the first remaining free USB capture node (deterministic order).
    `auto_detect: false` disables 2/4 (explicit + CSI-by-config only).
    rescan() re-runs all of this at runtime (hot-plug) without restart.
    """

    #: default sensor -> role when nothing else matches
    DEFAULT_SENSOR_ROLE = {"imx708": "front", "imx477": "bottom"}

    def __init__(self, cfg):
        self.cfg = cfg
        self.cams = {}  # name -> camera
        self.rescan_count = 0
        # A camera that detect() no longer reports is only DROPPED once it has
        # actually stopped delivering frames for this long. One failed probe
        # (libcamera busy, a USB glitch) must not black out a live mission feed
        # until the next rescan cycle; a genuinely unplugged camera goes stale
        # within a frame or two and is dropped on the following rescan.
        cam_cfg = (cfg.get("cameras", {}) or {})
        try:
            self.rescan_keep_grace_s = float(
                cam_cfg.get("rescan_keep_grace_s", 5.0))
        except (TypeError, ValueError):
            self.rescan_keep_grace_s = 5.0
        self._lock = threading.RLock()  # rescan vs bring-up mutual exclusion

    def rescan_lock(self):
        return self._lock

    @staticmethod
    def _mk_explicit(name, ccfg, facing, kind):
        size = ccfg.get("size", [1280, 720])
        rot = ccfg.get("rotation_deg", 0)
        q = ccfg.get("jpeg_quality", 80)
        hfov = ccfg.get("hfov_deg", 68.0)
        if kind == "file":
            c = FileCamera(name, ccfg.get("path", ""), size, hfov, facing,
                           rot, q, fps=ccfg.get("fps", 5))
            c.model = "file"
            log.info("assigned %s -> file %r (%s)", name, ccfg.get("path"), facing)
        else:
            src = ccfg.get("device", 0)
            if isinstance(src, str) and src.isdigit():
                src = int(src)
            c = USBCamera(name, src, size, hfov, facing, rot, q,
                          backend=ccfg.get("backend"), fps=ccfg.get("fps", 30),
                          controls=ccfg.get("controls"),
                          fourcc=ccfg.get("fourcc"))
            c.model = ("usb" if isinstance(src, int) or str(src).startswith("/dev/")
                       else "url")
            c.device_desc = ("v4l2 %r" % src) if isinstance(src, int) else str(src)
            log.info("assigned %s -> %s %r (%s)", name, c.model, src, facing)
        return c

    def detect(self):
        cam_cfg = self.cfg.get("cameras", {}) or {}
        front_cfg = cam_cfg.get("front", {}) or {}
        bottom_cfg = cam_cfg.get("bottom", {}) or {}
        assigned = {}

        # 1) explicit usb/url/file kinds — no probing needed
        for name, ccfg, facing in (("cam1", front_cfg, "front"),
                                   ("cam2", bottom_cfg, "bottom")):
            kind = str(ccfg.get("kind", "") or "").lower()
            if kind in ("usb", "url", "file"):
                try:
                    assigned[name] = self._mk_explicit(name, ccfg, facing, kind)
                except Exception as e:
                    log.error("%s (%s) build failed: %r", name, kind, e)

        # 2) CSI slots via libcamera/picamera2 probe.
        #    kind "rpi" (explicit CSI) and "auto" (real_pi: CSI first, USB
        #    next) both try the CSI probe; legacy empty config probes both.
        auto_detect = bool(cam_cfg.get("auto_detect", True))
        slots = []
        for key, name, ccfg, facing in (("front", "cam1", front_cfg, "front"),
                                        ("bottom", "cam2", bottom_cfg, "bottom")):
            if name in assigned:
                continue
            if (not cam_cfg) or (key in cam_cfg):
                kind = str(ccfg.get("kind", "rpi") or "rpi").lower()
                if kind in ("rpi", "auto"):
                    slots.append((name, ccfg, facing))
        if slots:
            found = list_cameras()
            if not found:
                log.warning("no CSI cameras detected")
            else:
                def _mk(name, idx, model, ccfg, facing):
                    c = Camera(name, idx, ccfg.get("size", [1920, 1080]),
                               ccfg.get("hfov_deg", 66.0), facing,
                               ccfg.get("rotation_deg", 0),
                               ccfg.get("jpeg_quality", 80),
                               autofocus=ccfg.get("autofocus"),
                               tuning_file=ccfg.get("tuning_file"),
                               lens_position=ccfg.get("lens_position"))
                    c.model = model
                    c.device_desc = "csi %s (idx %d)" % (model, idx)
                    return c

                used = set()
                # pass 0: explicit CSI index pin (cameras.<role>.csi_index).
                # Required when two cameras report the SAME sensor — e.g. an
                # Arducam UC-517 and a Waveshare module are both "imx477", so
                # match_name cannot tell them apart, and which one becomes
                # "bottom" decides the whole 15 m camera model (lens, FOV,
                # focus). Pinning the libcamera index makes roles certain.
                for name, ccfg, facing in slots:
                    if name in assigned:
                        continue
                    want_idx = ccfg.get("csi_index")
                    if want_idx in (None, "", "auto"):
                        continue
                    try:
                        want_idx = int(want_idx)
                    except (TypeError, ValueError):
                        log.warning("%s csi_index %r is not an integer — "
                                    "ignoring it", name, ccfg.get("csi_index"))
                        continue
                    hit = next(((i, m) for i, m, _ in found if i == want_idx), None)
                    if hit is None or want_idx in used:
                        log.warning("%s csi_index %d not available (found: %s)%s",
                                    name, want_idx,
                                    [i for i, _m, _ in found],
                                    " — already taken" if want_idx in used else "")
                        continue
                    i, m = hit
                    assigned[name] = _mk(name, i, m, ccfg, facing)
                    used.add(i)
                    log.info("%s pinned to CSI index %d (%s) by csi_index",
                             name, i, m)

                # pass 1: explicit role match_name (e.g. imx477 -> bottom)
                for name, ccfg, facing in slots:
                    if name in assigned:
                        continue
                    want_name = str(ccfg.get("match_name") or "").lower()
                    if not want_name:
                        continue
                    for i, m, _info in found:
                        if i in used or want_name not in m.lower():
                            continue
                        assigned[name] = _mk(name, i, m, ccfg, facing)
                        used.add(i)
                        break
                # pass 2: default sensor->role (imx708 front, imx477 bottom)
                role_of_name = {n: f for n, _c, f in slots}
                for i, m, _info in found:
                    if i in used or not m:
                        continue
                    sensor = m.lower()
                    for s, role in self.DEFAULT_SENSOR_ROLE.items():
                        if s in sensor:
                            rname = "cam1" if role == "front" else "cam2"
                            if rname in role_of_name and rname not in assigned:
                                ccfg, facing = next(
                                    (c, f) for n, c, f in slots if n == rname)
                                assigned[rname] = _mk(rname, i, m, ccfg, facing)
                                used.add(i)
                            break
                # pass 3: unknown sensors fill remaining slots by index order
                for name, ccfg, facing in slots:
                    if name not in assigned:
                        for i, m, _info in found:
                            if i not in used:
                                assigned[name] = _mk(name, i, m, ccfg, facing)
                                used.add(i)
                                break
        # 2b) USB auto-detect for roles still unassigned (real_pi)
        #     match_vidpid / match_name per role, else first free capture
        #     node. CSI nodes are excluded (no USB parent in sysfs).
        #     A node taken by one role is NOT offered to the other — one
        #     camera per role, always.
        if auto_detect:
            taken_nodes = set()
            # A camera an explicit role already claimed (kind: usb with
            # device: /dev/videoN) must NOT be offered to the other role, or
            # one physical camera ends up serving two roles at once.
            for c in assigned.values():
                if getattr(c, "kind", "") == "usb":
                    src = getattr(c, "source", None)
                    if isinstance(src, str) and src.startswith("/dev/"):
                        taken_nodes.add(src)
            for name, ccfg, facing in (("cam1", front_cfg, "front"),
                                       ("cam2", bottom_cfg, "bottom")):
                if name in assigned:
                    continue
                kind = str(ccfg.get("kind", "auto") or "auto").lower()
                if kind not in ("auto",):
                    continue
                dev = ccfg.get("device")
                if dev not in (None, "", "auto"):
                    continue  # explicit device belongs to the usb-kind path
                node = None
                desc = ""
                try:
                    usb = [d for d in v4l2_capture_devices()
                           if d["is_capture"]
                           and d["node"] not in taken_nodes]
                except Exception as e:
                    log.debug("v4l2 enumerate failed: %r", e)
                    usb = []
                match_vid = str(ccfg.get("match_vidpid") or "").lower()
                match_name = str(ccfg.get("match_name") or "").lower()
                declared = False
                if match_vid or match_name:
                    for d in usb:
                        if (match_vid and d["vidpid"] == match_vid) or \
                                (match_name and
                                 (match_name in d["usb_desc"].lower() or
                                  match_name in d["name"].lower())):
                            node, desc = d["node"], d
                            declared = True
                            break
                if node is None:
                    for d in usb:
                        node, desc = d["node"], d
                        break
                if node is None:
                    continue
                taken_nodes.add(node)
                try:
                    size = ccfg.get("size", [1280, 720])
                    c = USBCamera(name, node, size,
                                  ccfg.get("hfov_deg", 68.0), facing,
                                  ccfg.get("rotation_deg", 0),
                                  ccfg.get("jpeg_quality", 80),
                                  backend=ccfg.get("backend"),
                                  fps=ccfg.get("fps", 30),
                                  controls=ccfg.get("controls"),
                                  fourcc=ccfg.get("fourcc"))
                    c.model = (desc.get("usb_desc") or
                               "v4l2 %s (%s)" % (node, desc.get("vidpid")))
                    c.device_desc = "v4l2 %s (%s)" % (node,
                                                      desc.get("vidpid"))
                    # found by scanning, not named in config -> this role was a
                    # guess, so the single-camera rule may re-role it
                    c._auto_picked = not declared
                    assigned[name] = c
                    log.info("%s auto-detected USB cam %s (%s)", name,
                             node, c.model)
                except Exception as e:
                    log.error("%s USB auto-detect build failed: %r", name, e)

        # 3) mirrors (display-only aliases; source must be assigned first)
        for name, ccfg, facing in (("cam1", front_cfg, "front"),
                                   ("cam2", bottom_cfg, "bottom")):
            if name in assigned:
                continue
            kind = str(ccfg.get("kind", "") or "").lower()
            if kind != "mirror":
                continue
            src = ccfg.get("source", "cam2" if name == "cam1" else "cam1")
            src_cam = assigned.get(src)
            if src_cam is None or getattr(src_cam, "display_only", False):
                log.error("%s mirror source %r unavailable — skipping", name, src)
                continue
            c = MirrorCamera(name, src_cam,
                             ccfg.get("size", src_cam.size),
                             ccfg.get("hfov_deg", src_cam.hfov_deg),
                             facing, ccfg.get("rotation_deg", 0),
                             ccfg.get("jpeg_quality", 80))
            assigned[name] = c
            log.info("assigned %s -> mirror of %s (%s)", name, src, facing)

        # ONE camera of any kind: its role comes from config, because which
        # single camera the operator plugged in says nothing about whether it
        # points down — and on this mission the downward role is the one that
        # finds the QR. An EXPLICITLY configured role (kind/device written in
        # config) is a deliberate decision and is left alone.
        if len(assigned) == 1:
            only = next(iter(assigned.values()))
            if only.kind == "rpi" or getattr(only, "_auto_picked", False):
                role = str(cam_cfg.get("single_cam_role", "bottom")).lower()
                if role not in ("front", "bottom"):
                    role = "bottom"
                only.facing = role
                log.warning("single camera (%s) — treating as %s", only.model, role)
        self.cams = assigned
        for c in assigned.values():
            log.info("assigned %s -> idx %s (%s, %s)", c.name, c.index, c.model, c.facing)
        return assigned

    def start_all(self):
        for c in self.cams.values():
            try:
                c.start()
            except Exception as e:
                log.error("%s start failed: %r", c.name, e)

    def rescan(self):
        """Hot-plug rescan: re-run detect(), adopt new cameras, drop dead
        ones. Returns the sorted list of CHANGED camera names (added,
        removed or re-assigned). Safe from the auto-rescan timer thread or
        POST /api/cameras/rescan; the caller syncs the MJPEG streams.

        UNCHANGED assignments keep their RUNNING camera object — a live
        feed is never interrupted by a no-op rescan. Only genuinely new
        or re-assigned roles are started/restarted, under the rig lock so
        this cannot race the bring-up thread's detect()+start_all().
        """
        with self._lock:
            self.rescan_count += 1
            # NOTE: detect() replaces self.cams as a side effect, so the
            # OLD (possibly running) objects must be snapshotted first.
            old_cams = {n: c for n, c in self.cams.items()}
            try:
                new = self.detect()
            except Exception as e:
                log.warning("rescan #%d detect failed: %r",
                            self.rescan_count, e)
                self.cams = old_cams      # keep what was running
                return []
            changed = set()
            # unchanged assignments keep their RUNNING object — a live
            # feed must never be interrupted by a no-op rescan
            for n, old in old_cams.items():
                c = new.get(n)
                if c is not None and \
                        (old.kind, old.index, old.model) == \
                        (c.kind, c.index, c.model):
                    new[n] = old
            # roles detect() failed to see: keep them ONLY while they are
            # demonstrably still streaming (see __init__). A camera that is
            # really gone stops producing frames, so it is dropped next scan.
            for n, old in old_cams.items():
                if n in new:
                    continue
                try:
                    _frame, ts, cnt = old.latest()
                    age = None if not ts else time.time() - ts
                except Exception:
                    age, cnt = None, 0
                if cnt and age is not None and age < self.rescan_keep_grace_s:
                    new[n] = old
                    log.warning("rescan #%d: %s was not auto-detected but is "
                                "still streaming (frame %.1fs old) — kept; "
                                "it will be dropped if it really is gone",
                                self.rescan_count, n, age)
            self.cams = new
            # 1) roles that vanished — stop them
            for n in old_cams:
                if n not in new:
                    try:
                        old_cams[n].stop()
                    except Exception:
                        pass
                    changed.add(n)
                    log.info("rescan #%d: camera %s gone", self.rescan_count,
                             n)
            # 2) new roles — start; re-assigned roles — restart on new dev
            for n, c in new.items():
                old = old_cams.get(n)
                if old is None:
                    try:
                        c.start()
                        changed.add(n)
                        log.info("rescan #%d: adopted new camera %s (%s)",
                                 self.rescan_count, n,
                                 c.device_desc or c.model)
                    except Exception as e:
                        log.error("rescan start %s failed: %r", n, e)
                elif c is not old:
                    try:
                        old.stop()
                        c.start()
                        changed.add(n)
                        log.info("rescan #%d: camera %s re-assigned (%s -> %s)",
                                 self.rescan_count, n,
                                 old.device_desc or old.model,
                                 c.device_desc or c.model)
                    except Exception as e:
                        log.error("rescan restart %s failed: %r", n, e)
            return sorted(changed)

    def stop_all(self):
        for c in self.cams.values():
            try:
                c.stop()
            except Exception:
                pass

    def get(self, name):
        return self.cams.get(name)

    def status(self):
        return {n: c.status() for n, c in self.cams.items()}
