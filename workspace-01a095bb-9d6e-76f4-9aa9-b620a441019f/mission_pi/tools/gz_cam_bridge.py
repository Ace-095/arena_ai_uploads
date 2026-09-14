#!/usr/bin/env python3
"""Gazebo Harmonic camera bridge: gz-transport image topics -> MJPEG over HTTP.

Why this exists: mission_pi drinks plain OpenCV sources (V4L2 index, file,
http/mjpeg...). Gazebo cameras publish gz.msgs.Image on gz-transport topics,
which OpenCV cannot open directly. This script subscribes (2 cameras) and
re-serves each as an MJPEG stream that mission_pi's `url` backend opens like
any IP camera — and that a human can check in a browser.

  Terminal C (SYSTEM python3 — the gz bindings are apt packages, and this
  script must NOT run in the mission_pi venv):
    sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil
    python3 tools/gz_cam_bridge.py --port 8099
    # check:  http://127.0.0.1:8099/  (both cameras + health, in a browser)

  mission_pi side (config.gazebo.yaml):
    front:  { kind: url, backend: mjpeg, device: "http://127.0.0.1:8099/front.mjpg", ... }
    bottom: { kind: url, backend: mjpeg, device: "http://127.0.0.1:8099/bottom.mjpg", ... }

Needs no ROS, no GStreamer, no numpy: just the gz python bindings + PIL.
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
    from PIL import Image as PILImage
    _HAVE_PIL = True
except Exception as e:
    PILImage = None
    _HAVE_PIL = False
    _PIL_ERR = e


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
    def __init__(self, name, topic, jpeg_quality=80):
        self.name = name
        self.topic = topic
        self.jpeg_quality = int(jpeg_quality)
        self._lock = threading.Lock()
        self._jpg = None
        self._ts = 0.0
        self._seq = 0
        self._frames = 0
        self._w = 0
        self._h = 0
        self._fmt = "?"
        self._warned_stride = False
        self._warned_fmt = False

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
            buf = io.BytesIO()
            rgb.save(buf, "JPEG", quality=self.jpeg_quality)
            jpg = buf.getvalue()
        except Exception as e:
            log.debug("%s jpeg failed: %r", self.name, e)
            return
        with self._lock:
            self._jpg, self._ts, self._w, self._h, self._fmt = \
                jpg, time.time(), w, h, fmt
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

    def latest(self):
        with self._lock:
            return self._jpg, self._ts, self._seq

    def status(self):
        with self._lock:
            return {"topic": self.topic, "frames": self._frames,
                    "age_s": round(time.time() - self._ts, 2) if self._ts else None,
                    "size": [self._w, self._h], "format": self._fmt}


class Handler(BaseHTTPRequestHandler):
    server_version = "gz_cam_bridge/1"
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass  # stats thread covers logging; per-request logs are spam

    def _send_mjpeg(self, cam):
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
            jpg, _, seq = cam.latest()
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
            return self._send_mjpeg(cams["front"])
        if self.path == "/bottom.mjpg" and "bottom" in cams:
            return self._send_mjpeg(cams["bottom"])
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
            body = ("""<html><head><title>gz_cam_bridge</title></head><body>
<h2>gz_cam_bridge</h2>
<table><tr><td><h3>front (cam1)</h3><img src="/front.mjpg" width="640"></td>
<td><h3>bottom (cam2)</h3><img src="/bottom.mjpg" width="640"></td></tr></table>
<p><a href="/health">/health</a> — frame counters + ages (JSON).</p>
<p>Front should show the horizon/runway, bottom the ground. If a pane is
dark, that gz topic has no publisher — check <code>gz topic -l</code>.</p>
</body></html>""").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404, "want /front.mjpg | /bottom.mjpg | /health | /")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--front-topic", default="/iris/front/image")
    ap.add_argument("--bottom-topic", default="/iris/bottom/image")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--stats-every", type=float, default=10.0)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")
    if not _HAVE_GZ:
        print("need Gazebo python bindings: "
              "sudo apt install python3-gz-transport13 python3-gz-msgs10")
        print("binding import error: %r" % (_GZ_ERR,))
        print("NOTE: run with SYSTEM python3, not the mission_pi venv.")
        return 2
    if not _HAVE_PIL:
        print("need: sudo apt install python3-pil   (%r)" % (_PIL_ERR,))
        return 2

    fmt_names = _fmt_map()
    log.info("pixel formats known: %s",
             ", ".join("%s=%d" % (n, i) for i, n in sorted(fmt_names.items())))

    front = CamState("front", args.front_topic, args.jpeg_quality)
    bottom = CamState("bottom", args.bottom_topic, args.jpeg_quality)
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
    print("MJPEG up:  http://%s:%d/  (front.mjpg, bottom.mjpg, health)"
          % (args.bind, args.port))
    try:
        while True:
            time.sleep(args.stats_every)
            for c in (front, bottom):
                st = c.status()
                log.info("%s: %d frames age=%ss %s %s", c.name, st["frames"],
                         st["age_s"], st["size"], st["format"])
    except KeyboardInterrupt:
        pass
    srv.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
