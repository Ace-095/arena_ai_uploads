"""Camera auto-detect + capture threads (picamera2 / rpi camera stack).

Roles are assigned by SENSOR, not port index, so wiring order can't fool us:
  imx708 (Pi Camera Module 3) -> cam1 "front"
  imx477 (HQ Camera)          -> cam2 "bottom"
Unknown sensors fall back to index order. One camera? It still works —
single_cam_role (default: bottom geometry) decides how its frames are used.

Each camera runs a capture thread holding the latest BGR frame + an
annotated JPEG for the UI poll endpoint. OpenCV is only used for JPEG
encode + overlay drawing.
"""
import logging
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


class Camera:
    def __init__(self, name, index, size, hfov_deg, facing,
                 rotation_deg=0, jpeg_quality=80):
        self.name = name            # cam1 | cam2
        self.index = index          # picamera2 index
        self.size = tuple(size)     # (w, h)
        self.hfov_deg = float(hfov_deg)
        self.facing = facing        # front | bottom
        self.rotation_deg = int(rotation_deg)
        self.jpeg_quality = int(jpeg_quality)
        self.model = "?"
        self._picam = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_ts = 0.0
        self._frames = 0
        self._overlay = []          # [(x,y,w,h,label), ...] drawn on stream
        self._stop = threading.Event()
        self._thread = None
        self.running = False

    # -- lifecycle ------------------------------------------------------
    def start(self):
        if not _HAVE_PICAM:
            raise RuntimeError("picamera2 not installed: %r" % (_PICAM_ERR,))
        self._picam = Picamera2(self.index)
        cfg = self._picam.create_video_configuration(
            main={"size": self.size, "format": "RGB888"})
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
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="cam-" + self.name,
                                        daemon=True)
        self._thread.start()
        self.running = True
        log.info("%s started: idx=%d model=%s size=%s facing=%s",
                 self.name, self.index, self.model, self.size, self.facing)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self._picam and self._picam.stop()
        except Exception:
            pass
        self.running = False

    def _loop(self):
        while not self._stop.is_set():
            try:
                arr = self._picam.capture_array("main")
            except Exception:
                time.sleep(0.1)
                continue
            if _HAVE_CV2:
                try:
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
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
        return {"name": self.name, "index": self.index, "model": self.model,
                "size": list(self.size), "facing": self.facing,
                "hfov_deg": self.hfov_deg, "running": self.running,
                "frames": n, "age_s": round(time.time() - ts, 2) if ts else None}


class CameraRig:
    """Owns cam1 (front) + cam2 (bottom); tolerates 0/1/2 cameras."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.cams = {}  # name -> Camera

    def detect(self):
        found = list_cameras()
        if not found:
            log.warning("no CSI cameras detected")
            return {}
        by_model = {}
        for idx, model, _info in found:
            by_model.setdefault(model, []).append(idx)
        cam_cfg = self.cfg.get("cameras", {})
        front_cfg = cam_cfg.get("front", {})
        bottom_cfg = cam_cfg.get("bottom", {})

        def _mk(name, idx, model, ccfg, facing):
            c = Camera(name, idx, ccfg.get("size", [1920, 1080]),
                       ccfg.get("hfov_deg", 66.0), facing,
                       ccfg.get("rotation_deg", 0),
                       ccfg.get("jpeg_quality", 80))
            c.model = model
            return c

        assigned = {}
        idx708 = [i for i, m, _ in found if "imx708" in m]
        idx477 = [i for i, m, _ in found if "imx477" in m]
        if idx708:
            assigned["cam1"] = _mk("cam1", idx708[0], "imx708", front_cfg, "front")
        if idx477:
            assigned["cam2"] = _mk("cam2", idx477[0], "imx477", bottom_cfg, "bottom")
        # unknown sensors: fill remaining slots by index order
        used = {c.index for c in assigned.values()}
        rest = [(i, m) for i, m, _ in found if i not in used]
        for name, ccfg, facing in (("cam1", front_cfg, "front"),
                                   ("cam2", bottom_cfg, "bottom")):
            if name not in assigned and rest:
                i, m = rest.pop(0)
                assigned[name] = _mk(name, i, m, ccfg, facing)
        # single camera: role comes from config
        if len(assigned) == 1:
            only = next(iter(assigned.values()))
            role = str(cam_cfg.get("single_cam_role", "bottom")).lower()
            if role not in ("front", "bottom"):
                role = "bottom"
            only.facing = role
            log.warning("single camera (%s) — treating as %s", only.model, role)
        self.cams = assigned
        for c in assigned.values():
            log.info("assigned %s -> idx %d (%s, %s)", c.name, c.index, c.model, c.facing)
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
