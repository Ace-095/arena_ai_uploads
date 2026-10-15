#!/usr/bin/env python3
"""
Gazebo Harmonic camera bridge: gz-transport image topics -> MJPEG over HTTP + QR boost (Kabaddi).

Why this exists: mission_pi drinks plain OpenCV sources (V4L2 index, file,
http/mjpeg...). Gazebo cameras publish gz.msgs.Image on gz-transport topics,
which OpenCV cannot open directly. This script subscribes (2 cameras) and
re-serves each as an MJPEG stream that mission_pi's `url` backend opens like
any IP camera — and that a human can check in a browser.

  Terminal C (SYSTEM python3 — the gz bindings are apt packages, and this
  script must NOT run in the mission_pi venv):
    sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil python3-opencv
    python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress
    # check:  http://127.0.0.1:8099/  (both cameras + health, in a browser)
    # raw:    http://127.0.0.1:8099/front_raw.mjpg , /bottom_raw.mjpg

  mission_pi side (config.gazebo.yaml):
    front:  { kind: url, backend: mjpeg, device: "http://127.0.0.1:8099/front.mjpg", ... }
    bottom: { kind: url, backend: mjpeg, device: "http://127.0.0.1:8099/bottom.mjpg", ... }

QR boost (Kabaddi setting): Makes QR pop more than ground via PIL contrast+sharpness+desat + optional cv2 HSV ground suppression + CLAHE.
Same logic as mission_pi/qr_camera_boost.py but using PIL so Gazebo sim also shows QR enhanced vs ground.
For SITL: use --qr-boost to enable, --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 for bottom cam.
"""
import argparse
import io
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger("gz_cam_bridge")

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image, PixelFormatType
    _HAVE_GZ = True
except Exception as e:
    Node = Image = PixelFormatType = None
    _HAVE_GZ = False
    _GZ_ERR = e

try:
    from PIL import Image as PILImage, ImageEnhance
    _HAVE_PIL = True
except Exception as e:
    PILImage = None
    ImageEnhance = None
    _HAVE_PIL = False
    _PIL_ERR = e

# Optional cv2 for advanced ground suppression (if available in system python)
try:
    import cv2
    import numpy as np
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    np = None
    _HAVE_CV2 = False


def _fmt_map():
    """{numeric pixel_format_type: NAME} from the live protobuf enum."""
    out = {}
    try:
        for v in PixelFormatType.DESCRIPTOR.values:
            out[int(v.number)] = str(v.name)
    except Exception:
        pass
    return out


class CamState:
    def __init__(self, name, topic, jpeg_quality=80, qr_boost=False,
                 contrast=1.0, saturation=1.0, sharpness=1.0, brightness=1.0,
                 enhance_mode="qr_boost"):
        self.name = name
        self.topic = topic
        self.jpeg_quality = int(jpeg_quality)
        self.qr_boost = bool(qr_boost)
        self.contrast = float(contrast)
        self.saturation = float(saturation)
        self.sharpness = float(sharpness)
        self.brightness = float(brightness)
        self.enhance_mode = enhance_mode
        self._lock = threading.Lock()
        self._jpg = None
        self._jpg_raw = None
        self._ts = 0.0
        self._seq = 0
        self._frames = 0
        self._w = 0
        self._h = 0
        self._fmt = "?"
        self._warned_stride = False
        self._warned_fmt = False

    def _apply_qr_boost_pil(self, pil_img):
        """Apply QR boost via PIL ImageEnhance — makes QR pop vs ground (Kabaddi)."""
        if not self.qr_boost or not _HAVE_PIL:
            return pil_img
        try:
            # Contrast HIGH — QR B/W pop vs mid-tone ground
            if self.contrast != 1.0:
                pil_img = ImageEnhance.Contrast(pil_img).enhance(self.contrast)
            # Saturation LOW — desaturate green/brown ground to gray, QR B/W stays
            if self.saturation != 1.0:
                pil_img = ImageEnhance.Color(pil_img).enhance(self.saturation)
            # Sharpness HIGH — finder patterns crisp at 10/15 m
            if self.sharpness != 1.0:
                pil_img = ImageEnhance.Sharpness(pil_img).enhance(self.sharpness)
            # Brightness — slightly dark avoids washout
            if self.brightness != 1.0:
                pil_img = ImageEnhance.Brightness(pil_img).enhance(self.brightness)

            # Advanced ground suppression via cv2 if available
            if _HAVE_CV2 and self.enhance_mode in ("ground_suppress", "qr_boost"):
                cv_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
                if self.enhance_mode == "ground_suppress":
                    lower_green = np.array([35, 40, 40])
                    upper_green = np.array([85, 255, 255])
                    lower_brown = np.array([10, 30, 30])
                    upper_brown = np.array([30, 200, 200])
                    mask_green = cv2.inRange(hsv, lower_green, upper_green)
                    mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
                    mask_ground = cv2.bitwise_or(mask_green, mask_brown)
                    h, s, v = cv2.split(hsv)
                    s = s.astype(np.float32)
                    s = np.where(mask_ground > 0, s * 0.4, s)
                    s = np.clip(s, 0, 255).astype(np.uint8)
                    v = v.astype(np.float32)
                    v = np.where(mask_ground > 0, v * 0.9, v)
                    v = np.clip(v, 0, 255).astype(np.uint8)
                    hsv2 = cv2.merge((h, s, v))
                    cv_img = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)
                    pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
                elif self.enhance_mode == "qr_boost":
                    lower_green = np.array([35, 50, 50])
                    upper_green = np.array([85, 255, 255])
                    mask_green = cv2.inRange(hsv, lower_green, upper_green)
                    h, s, v = cv2.split(hsv)
                    s = s.astype(np.float32)
                    s = np.where(mask_green > 0, s * 0.6, s)
                    s = np.clip(s, 0, 255).astype(np.uint8)
                    hsv2 = cv2.merge((h, s, v))
                    cv_img = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)
                    lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
                    l, a, b = cv2.split(lab)
                    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                    l = clahe.apply(l)
                    lab = cv2.merge((l, a, b))
                    cv_img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
                    pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        except Exception as e:
            log.debug("%s QR boost PIL failed: %r", self.name, e)
        return pil_img

    def on_image(self, msg, fmt_names):
        try:
            w, h = int(msg.width), int(msg.height)
            fmt = fmt_names.get(int(msg.pixel_format_type),
                                "FMT_%d" % int(msg.pixel_format_type))
            data = bytes(msg.data)
            step = int(msg.step or 0)
        except Exception as e:
            log.debug("%s bad msg: %r", self.name, e)
            return
        try:
            rgb = self._to_rgb(data, w, h, fmt, step)
        except Exception as e:
            if not self._warned_fmt:
                self._warned_fmt = True
                log.warning("%s convert failed (fmt=%s %dx%d step=%d): %r",
                            self.name, fmt, w, h, step, e)
            return
        try:
            buf_raw = io.BytesIO()
            rgb.save(buf_raw, "JPEG", quality=self.jpeg_quality)
            jpg_raw = buf_raw.getvalue()

            rgb_boosted = self._apply_qr_boost_pil(rgb)

            buf = io.BytesIO()
            rgb_boosted.save(buf, "JPEG", quality=self.jpeg_quality)
            jpg = buf.getvalue()
        except Exception as e:
            log.debug("%s jpeg failed: %r", self.name, e)
            return
        with self._lock:
            self._jpg, self._jpg_raw, self._ts, self._w, self._h, self._fmt = \
                jpg, jpg_raw, time.time(), w, h, fmt
            self._seq += 1
            self._frames += 1

    def _to_rgb(self, data, w, h, fmt, step):
        nch = 1 if fmt.startswith("L_INT") or fmt.startswith("L_") else 3
        want = w * nch
        if step and step != want:
            if not self._warned_stride:
                self._warned_stride = True
                log.warning("%s stride %d != width %d — restriding rows",
                            self.name, step)
            data = b"".join(data[i * step:i * step + want]
                            for i in range(h))
        mode = "L" if nch == 1 else "RGB"
        img = PILImage.frombytes(mode, (w, h), data)
        if fmt.startswith("BGR"):
            ch = img.split()
            img = PILImage.merge("RGB", (ch[2], ch[1], ch[0]))
        return img.convert("RGB")

    def latest(self, raw=False):
        with self._lock:
            if raw:
                return self._jpg_raw, self._ts, self._seq
            return self._jpg, self._ts, self._seq

    def status(self):
        with self._lock:
            return {"topic": self.topic, "frames": self._frames,
                    "age_s": round(time.time() - self._ts, 2) if self._ts else None,
                    "size": [self._w, self._h], "format": self._fmt,
                    "qr_boost": self.qr_boost,
                    "contrast": self.contrast,
                    "saturation": self.saturation,
                    "sharpness": self.sharpness,
                    "brightness": self.brightness,
                    "enhance_mode": self.enhance_mode}


class Handler(BaseHTTPRequestHandler):
    server_version = "gz_cam_bridge/1"
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send_mjpeg(self, cam, raw=False):
        try:
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
        except (BrokenPipeError, ConnectionResetError):
            return
        last = -1
        while True:
            jpg, _, seq = cam.latest(raw=raw)
            if jpg is not None and seq != last:
                last = seq
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                time.sleep(0.05)

    def do_GET(self):
        cams = self.server.cams
        if self.path == "/front.mjpg" and "front" in cams:
            return self._send_mjpeg(cams["front"], raw=False)
        if self.path == "/bottom.mjpg" and "bottom" in cams:
            return self._send_mjpeg(cams["bottom"], raw=False)
        if self.path == "/front_raw.mjpg" and "front" in cams:
            return self._send_mjpeg(cams["front"], raw=True)
        if self.path == "/bottom_raw.mjpg" and "bottom" in cams:
            return self._send_mjpeg(cams["bottom"], raw=True)
        if self.path == "/health":
            body = json.dumps(
                {n: c.status() for n, c in cams.items()}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ("/", "/index.html"):
            body = ("""<html><head><title>gz_cam_bridge + QR boost</title></head><body>
<h2>gz_cam_bridge — arena/test1 + QR boost (Kabaddi)</h2>
<table><tr>
<td><h3>front (cam1) boosted</h3><img src="/front.mjpg" width="640"><br><small><a href="/front_raw.mjpg">raw</a></small></td>
<td><h3>bottom (cam2) boosted</h3><img src="/bottom.mjpg" width="640"><br><small><a href="/bottom_raw.mjpg">raw</a></small></td>
</tr></table>
<p><a href="/health">/health</a> — frames + QR boost settings (JSON).</p>
<p>QR boost: contrast HIGH makes QR B/W pop vs ground, sat LOW desaturates green/brown grass to gray, sharp HIGH makes finder crisp at 15 m.</p>
<p>Front should show horizon/runway, bottom ground. If dark, check <code>gz topic -l | grep -i image</code>.</p>
<p>Args: --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress|qr_boost</p>
</body></html>""").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404, "want /front.mjpg | /bottom.mjpg | /front_raw.mjpg | /bottom_raw.mjpg | /health | /")

def main():
    ap = argparse.ArgumentParser(description="Gazebo cam bridge + QR boost (Kabaddi) for SITL")
    ap.add_argument("--front-topic", default="/iris/front/image")
    ap.add_argument("--bottom-topic", default="/iris/bottom/image")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--bind", default="0.0.0.0", help="Bind 0.0.0.0 for 2-laptop Pop!_OS+Windows, 127.0.0.1 for single")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--stats-every", type=float, default=10.0)
    ap.add_argument("--verbose", action="store_true")
    # QR boost (Kabaddi) args for SITL
    ap.add_argument("--qr-boost", action="store_true", help="Enable QR boost — makes QR pop vs ground (contrast+sharpness+desat)")
    ap.add_argument("--contrast", type=float, default=1.8, help="Contrast HIGH 1.8-2.0 makes QR B/W pop vs ground")
    ap.add_argument("--saturation", type=float, default=0.6, help="Saturation LOW 0.5-0.7 desaturates green/brown grass to gray")
    ap.add_argument("--sharpness", type=float, default=2.0, help="Sharpness HIGH 2.0 makes finder crisp at 10/15m")
    ap.add_argument("--brightness", type=float, default=0.9, help="Brightness 0.9 slightly dark avoids washout")
    ap.add_argument("--enhance-mode", default="ground_suppress", choices=["qr_boost", "ground_suppress", "none"],
                    help="Software enhance mode: qr_boost (CLAHE+green suppress) or ground_suppress (strong desat)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")
    if not _HAVE_GZ:
        print("need Gazebo python bindings: sudo apt install python3-gz-transport13 python3-gz-msgs10")
        print("binding import error: %r" % (_GZ_ERR,))
        print("NOTE: run with SYSTEM python3, not the mission_pi venv.")
        return 2
    if not _HAVE_PIL:
        print("need: sudo apt install python3-pil   (%r)" % (_PIL_ERR,))
        return 2

    fmt_names = _fmt_map()
    log.info("pixel formats known: %s",
             ", ".join("%s=%d" % (n, i) for i, n in sorted(fmt_names.items())))
    if args.qr_boost:
        log.info("QR boost ENABLED (Kabaddi): contrast=%.1f sat=%.1f sharp=%.1f bright=%.1f mode=%s — QR pops vs ground",
                 args.contrast, args.saturation, args.sharpness, args.brightness, args.enhance_mode)
    else:
        log.info("QR boost disabled — use --qr-boost for QR pop vs ground (contrast 1.8 sat 0.6 sharp 2.0)")

    front = CamState("front", args.front_topic, args.jpeg_quality,
                     qr_boost=args.qr_boost, contrast=args.contrast, saturation=args.saturation,
                     sharpness=args.sharpness, brightness=args.brightness, enhance_mode=args.enhance_mode)
    bottom = CamState("bottom", args.bottom_topic, args.jpeg_quality,
                      qr_boost=args.qr_boost, contrast=args.contrast, saturation=args.saturation,
                      sharpness=args.sharpness, brightness=args.brightness, enhance_mode=args.enhance_mode)
    node = Node()
    ok_f = node.subscribe(Image, args.front_topic,
                          lambda m: front.on_image(m, fmt_names))
    ok_b = node.subscribe(Image, args.bottom_topic,
                          lambda m: bottom.on_image(m, fmt_names))
    print("subscribe %s -> %s" % (args.front_topic, "OK" if ok_f else "FAIL"))
    print("subscribe %s -> %s" % (args.bottom_topic, "OK" if ok_b else "FAIL"))
    if not (ok_f and ok_b):
        print("HINT: subscribe FAIL usually means the topic does not exist "
              "YET — start gz sim first, then this bridge. Live topics: "
              "`gz topic -l | grep -i image`")
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    srv.cams = {"front": front, "bottom": bottom}
    threading.Thread(target=srv.serve_forever, name="http", daemon=True).start()
    print("MJPEG up:  http://%s:%d/  (front.mjpg, bottom.mjpg, front_raw.mjpg, bottom_raw.mjpg, health)" % (args.bind, args.port))
    print("QR boost: %s (contrast %.1f sat %.1f sharp %.1f bright %.1f mode %s)" %
          ("ON" if args.qr_boost else "OFF", args.contrast, args.saturation, args.sharpness, args.brightness, args.enhance_mode))
    try:
        while True:
            time.sleep(args.stats_every)
            for c in (front, bottom):
                st = c.status()
                log.info("%s: %d frames age=%ss %s %s qr_boost=%s",
                         c.name, st["frames"], st["age_s"], st["size"], st["format"], st["qr_boost"])
    except KeyboardInterrupt:
        pass
    srv.shutdown()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
