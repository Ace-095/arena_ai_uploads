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


class _BaseCamera:
    kind = "base"

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
        self.display_only = False  # True for mirrors: UI tile only, no detector worker
        self._lock = threading.Lock()
        self._frame = None
        self._frame_ts = 0.0
        self._frames = 0
        self._overlay = []          # [(x,y,w,h,label), ...] drawn on stream
        self._stop = threading.Event()
        self._thread = None
        self.running = False

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
        while not self._stop.is_set():
            try:
                arr = self._grab()
            except Exception:
                arr = None
            if arr is None:
                time.sleep(0.1)
                continue
            try:
                arr = self._apply_rotation(arr)
                h, w = arr.shape[:2]
                if (w, h) != tuple(self.size) and _HAVE_CV2:
                    arr = cv2.resize(arr, tuple(self.size))
            except Exception:
                pass
            with self._lock:
                self._frame = arr
                self._frame_ts = time.time()
                self._frames += 1

    # -- consumers ------------------------------------------------------
    def latest(self):
        """(frame_bgr_or_None, ts, count). Frame is a shared reference —
        consumers must not modify it in place."""
        with self._lock:
            return self._frame, self._frame_ts, self._frames

    def set_overlay(self, boxes):
        """boxes: [(x,y,w,h,label), ...] drawn onto the streamed JPEG."""
        with self._lock:
            self._overlay = list(boxes or [])

    def jpeg(self, max_width=960):
        """Annotated JPEG bytes for /api/camera/frame/<cam> (or None)."""
        frame, ts, _ = self.latest()
        if frame is None or not _HAVE_CV2:
            return None
        try:
            img = frame
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
            ok, buf = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            return bytes(buf) if ok else None
        except Exception as e:
            log.debug("%s jpeg failed: %r", self.name, e)
            return None

    def status(self):
        _, ts, n = self.latest()
        return {"name": self.name, "kind": self.kind, "index": self.index,
                "model": self.model, "size": list(self.size),
                "facing": self.facing, "hfov_deg": self.hfov_deg,
                "running": self.running, "frames": n,
                "age_s": round(time.time() - ts, 2) if ts else None}


class Camera(_BaseCamera):
    """Pi CSI camera via picamera2 (kind: rpi)."""
    kind = "rpi"

    def __init__(self, name, index, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80):
        super().__init__(name, index, size, hfov_deg, facing,
                         rotation_deg, jpeg_quality)
        self._picam = None

    def _open(self):
        if not _HAVE_PICAM:
            raise RuntimeError("picamera2 not installed: %r" % (_PICAM_ERR,))
        self._picam = Picamera2(self.index)
        cfg = self._picam.create_video_configuration(
            main={"size": tuple(self.size), "format": "RGB888"})
        self._picam.configure(cfg)
        controls = {"AeEnable": True}
        if "imx708" in self.model:
            try:
                from libcamera import controls as CTRLS
                controls["AfMode"] = CTRLS.AfModeEnum.Continuous
            except Exception:
                pass
        try:
            self._picam.set_controls(controls)
        except Exception as e:
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

    def _api(self):
        if self.backend == "gstreamer" or (
                self.backend is None and isinstance(self.source, str)
                and "!" in self.source):
            return getattr(cv2, "CAP_GSTREAMER", cv2.CAP_ANY)
        return {"v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
                "dshow": getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY),
                "msmf": getattr(cv2, "CAP_MSMF", cv2.CAP_ANY),
                "ffmpeg": getattr(cv2, "CAP_FFMPEG", cv2.CAP_ANY),
                "any": cv2.CAP_ANY}.get(self.backend or "any", cv2.CAP_ANY)

    def _open(self):
        if not _HAVE_CV2:
            raise RuntimeError("opencv (cv2) not installed — needed for usb/url/file cameras")
        api = self._api()
        cap = cv2.VideoCapture(self.source, api) if api != cv2.CAP_ANY \
            else cv2.VideoCapture(self.source)
        if not cap.isOpened():
            raise RuntimeError("cannot open video source %r (backend=%s)" % (
                self.source, self.backend or "any"))
        try:
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*self.fourcc[:4]))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception:
            pass
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            raise RuntimeError("no frames from video source %r" % (self.source,))
        self._cap = cap
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

    def _grab(self):
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        return frame if ok and frame is not None else None

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

    def status(self):
        st = super().status()
        st["mirrors"] = self._src.name if self._src else None
        st["display_only"] = True
        return st


class CameraRig:
    """Owns cam1 (front) + cam2 (bottom); tolerates 0/1/2 cameras."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.cams = {}  # name -> camera

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

        # 2) rpi slots via CSI probe (legacy: empty config probes both)
        slots = []
        for key, name, ccfg, facing in (("front", "cam1", front_cfg, "front"),
                                        ("bottom", "cam2", bottom_cfg, "bottom")):
            if name in assigned:
                continue
            if (not cam_cfg) or (key in cam_cfg):
                kind = str(ccfg.get("kind", "rpi") or "rpi").lower()
                if kind == "rpi":
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
                               ccfg.get("jpeg_quality", 80))
                    c.model = model
                    return c

                want = {n for n, _c, _f in slots}
                idx708 = [i for i, m, _ in found if "imx708" in m]
                idx477 = [i for i, m, _ in found if "imx477" in m]
                if "cam1" in want and idx708:
                    assigned["cam1"] = _mk("cam1", idx708[0], "imx708", front_cfg, "front")
                if "cam2" in want and idx477:
                    assigned["cam2"] = _mk("cam2", idx477[0], "imx477", bottom_cfg, "bottom")
                # unknown sensors: fill remaining slots by index order
                used = {c.index for c in assigned.values() if c.kind == "rpi"}
                rest = [(i, m) for i, m, _ in found if i not in used]
                for name, ccfg, facing in slots:
                    if name not in assigned and rest:
                        i, m = rest.pop(0)
                        assigned[name] = _mk(name, i, m, ccfg, facing)

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

        # single rpi camera: role comes from config (explicit kinds keep theirs)
        if len(assigned) == 1:
            only = next(iter(assigned.values()))
            if only.kind == "rpi":
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
