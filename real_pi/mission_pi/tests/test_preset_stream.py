#!/usr/bin/env python3
"""Integration test (the user's ask): when the UI switches the QR preset,

  1. THE CAMERA SETTINGS ACTUALLY CHANGE — the preset's ISP boost profile
     must land on the bottom camera (exposure/gain/contrast/saturation/
     sharpness/AF from qr_camera_boost.QR_BOOST_PROFILES, qr_boost flags),
     via the EXACT UI sequence: GET /api/qr/presets -> POST /api/qr/preset;
  2. THE STREAM STAYS GOOD — no "always stuck" regression: before and after
     each switch the MJPEG stream must keep delivering frames (frame count
     in a window, max inter-frame gap) and a fresh connection must get its
     FIRST frame quickly (no delay).

Everything here is production code paths: FileCamera (inherits the real
apply_qr_profile -> apply_tuning chain), the real StreamManager (SendGate /
watchdog / first-frame 503), the real server endpoints, the real preset
table from config.yaml. Self-skips without fastapi/uvicorn/cv2.
"""
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.join(HERE, "..")
sys.path.insert(0, os.path.abspath(PI))

try:
    import fastapi  # noqa: F401
    import uvicorn
    import cv2  # noqa: F401
except Exception:
    print("SKIP test_preset_stream (fastapi/uvicorn/cv2 not installed)")
    sys.exit(0)

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def http_json(url, body=None, timeout=6.0):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"} if data else {},
        method="POST" if data else "GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or "{}")


def read_stream(port, cam, seconds):
    """Read the MJPEG endpoint over a raw socket for `seconds` (one pass).

    Returns (n_frames, t_first, max_gap_s): t_first = connect -> first
    frame bytes (the 'no delay' number); max_gap = worst pause between
    consecutive frames (the 'no stuck' number). Frame = JPEG SOI marker.
    """
    s = socket.create_connection(("127.0.0.1", port), timeout=5.0)
    s.settimeout(1.0)
    s.sendall(("GET /api/camera/stream/%s HTTP/1.1\r\nHost: 127.0.0.1:%d\r\n"
               "Connection: close\r\n\r\n" % (cam, port)).encode())
    t0 = time.time()
    times = []
    end = t0 + seconds
    try:
        while time.time() < end:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            now = time.time()
            i = 0
            while True:
                j = chunk.find(b"\xff\xd8\xff", i)
                if j < 0:
                    break
                times.append(now)
                i = j + 3
    finally:
        try:
            s.close()
        except Exception:
            pass
    gaps = [b - a for a, b in zip(times, times[1:])]
    return len(times), (times[0] - t0 if times else None), (max(gaps) if gaps else 0.0)


def make_frames_dir():
    """1280x720 frames with a QR + some contrast content (production
    make_demo_frames if present, else plain synthetic frames)."""
    out = "/tmp/preset_stream_frames"
    os.makedirs(out, exist_ok=True)
    try:
        import qrcode
        import numpy as np
        qr = qrcode.make("MISSION-QR-001")
        q = np.array(qr.convert("L"))
        q = cv2.resize(q, (240, 240), interpolation=cv2.INTER_NEAREST)
        base = np.full((720, 1280, 3), 120, dtype=np.uint8)
        base[:, :, 0] = 140  # brownish ground
        base[200:440, 520:760] = q[:, :, None] * np.array([[1]])
        for i in range(8):
            cv2.imwrite(os.path.join(out, "f%02d.png" % i), base)
    except Exception as e:
        print("WARN demo frames failed (%r) — plain frames" % e)
        for i in range(8):
            img = np.full((720, 1280, 3), 80 + i * 10, dtype=np.uint8)
            cv2.rectangle(img, (500, 200), (780, 480), (255, 255, 255), -1)
            cv2.imwrite(os.path.join(out, "f%02d.png" % i), img)
    return out


def main():
    import yaml
    import numpy as np  # noqa: F401  (frame synthesis above)
    from cameras import CameraRig, FileCamera
    from detector import get_detector
    from mission import Mission
    from qr_camera_boost import get_profile
    from server import create_app
    from streamm1 import StreamManager

    frames = make_frames_dir()
    with open(os.path.join(PI, "config.yaml")) as f:
        cfg = yaml.safe_load(f)
    cfg["stream"].update({"fps": 10, "width": 640, "first_frame_wait_s": 5.0})
    cfg.setdefault("cameras", {})["auto_rescan_s"] = 0

    # ---- production objects ---------------------------------------------
    cam1 = FileCamera("cam1", frames, (1280, 720), 66.0, "front", fps=10)
    cam2 = FileCamera("cam2", frames, (1280, 720), 100.0, "bottom", fps=10)
    rig = CameraRig(cfg)
    rig.cams = {"cam1": cam1, "cam2": cam2}
    det = get_detector({"kind": "classical"})

    class _FC:
        def link_ok(self):
            return False

        def get_position(self, timeout=2.0):
            return {"lat": 0.0, "lon": 0.0, "alt_rel": 15.0}

    class _Hub:
        def push(self, c, m):
            pass

    mission = Mission(_FC(), rig, det, _Hub(), cfg)
    streams = StreamManager(rig, cfg.get("stream", {}))
    app = create_app(rig, mission, _FC(), streams)

    rig.start_all()
    streams.sync(rig)
    streams.start_all()
    time.sleep(0.8)  # let the first frames land

    port = free_port()
    cfg_srv = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error")
    srv = uvicorn.Server(cfg_srv)
    srv.install_signal_handlers = lambda: None
    th = threading.Thread(target=srv.run, daemon=True)
    th.start()
    t0 = time.time()
    while time.time() - t0 < 10:
        try:
            urllib.request.urlopen("http://127.0.0.1:%d/health" % port, timeout=1)
            break
        except Exception:
            time.sleep(0.1)
    else:
        print("FAIL server did not come up")
        sys.exit(1)
    base = "http://127.0.0.1:%d" % port
    try:
        # ================================================================
        # BASELINE STREAM (before any switch)
        # ================================================================
        n0, t_first0, gap0 = read_stream(port, "cam2", 4.0)
        check("baseline: frames flowing", n0 >= 12, "n=%d" % n0)
        check("baseline: first frame fast (no stuck)",
              t_first0 is not None and t_first0 < 1.5, "t_first=%s" % t_first0)
        check("baseline: no long gaps", gap0 < 1.0, "max_gap=%.2fs" % gap0)

        # ================================================================
        # THE UI SEQUENCE: GET presets -> POST preset
        # ================================================================
        st, d = http_json(base + "/api/qr/presets")
        check("ui step 1: GET /api/qr/presets", st == 200 and "dark" in d.get("names", []))
        # NOTE: Mission.__init__ already applied the startup preset
        # (detector.preset: day -> boost qr_boost_day), so the baseline IS
        # the day profile on the bottom cam — that is the correct
        # production behavior (config preset lands on camera at boot).
        prof = get_profile("qr_boost_day")
        check("baseline camera settings = startup preset (day) profile",
              cam2.tuning["exposure_us"] == prof["exposure_us"]
              and cam2.tuning["contrast"] == prof["contrast"]
              and cam2.tuning["saturation"] == prof["saturation"]
              and cam2.qr_boost["qr_boost_mode"] == "qr_boost_day",
              str(cam2.tuning))

        st, d = http_json(base + "/api/qr/preset", {"preset": "dark"})
        check("ui step 2: POST /api/qr/preset dark", st == 200 and d.get("ok"))
        prof = get_profile("qr_boost_lowlight")
        check("cam settings CHANGED to dark preset profile",
              cam2.tuning["exposure_us"] == prof["exposure_us"]
              and cam2.tuning["contrast"] == prof["contrast"]
              and cam2.tuning["saturation"] == prof["saturation"]
              and cam2.tuning["sharpness"] == prof["sharpness"]
              and cam2.tuning["brightness"] == prof["brightness"],
              str(cam2.tuning))
        check("cam qr_boost flags updated",
              cam2.qr_boost["qr_boost_mode"] == "qr_boost_lowlight"
              and cam2.qr_boost["qr_boost_enabled"] is True,
              str(cam2.qr_boost))
        check("mission follows the preset",
              mission.preset_name == "dark"
              and mission.qf.require_decode_for_cue is True)
        st, d = http_json(base + "/api/fsm/status")
        check("status reflects new preset + strict cues",
              st == 200 and d.get("qr_preset") == "dark"
              and d.get("qr_filter", {}).get("require_decode_for_cue") is True)

        # ================================================================
        # STREAM AFTER SWITCH — must still be good (no delay, no stuck)
        # ================================================================
        n1, t_first1, gap1 = read_stream(port, "cam2", 4.0)
        print("  metrics: baseline n=%d t_first=%.2fs gap=%.2fs | "
              "after-dark n=%d t_first=%s gap=%.2fs"
              % (n0, t_first0 or -1, gap0, n1,
                 ("%.2fs" % t_first1) if t_first1 is not None else "?", gap1))
        check("after switch: frames still flowing", n1 >= 12, "n=%d" % n1)
        check("after switch: first frame fast (no delay from switch)",
              t_first1 is not None and t_first1 < 1.5, "t_first=%s" % t_first1)
        check("after switch: no long gaps (not stuck)", gap1 < 1.0,
              "max_gap=%.2fs" % gap1)

        # second switch — a DIFFERENT profile must land (proves per-preset)
        st, d = http_json(base + "/api/qr/preset", {"preset": "aggressive"})
        check("switch to aggressive", st == 200)
        prof = get_profile("qr_boost_aggressive")
        check("cam settings CHANGED to aggressive profile",
              cam2.tuning["exposure_us"] == prof["exposure_us"]
              and cam2.tuning["contrast"] == prof["contrast"]
              and cam2.tuning["saturation"] == prof["saturation"],
              str(cam2.tuning))
        n2, t_first2, gap2 = read_stream(port, "cam2", 3.0)
        check("after 2nd switch: stream healthy",
              n2 >= 9 and t_first2 is not None and t_first2 < 1.5 and gap2 < 1.0,
              "n=%d t_first=%s gap=%.2f" % (n2, t_first2, gap2))

        # back to day — original values must come back
        st, d = http_json(base + "/api/qr/preset", {"preset": "day"})
        prof = get_profile("qr_boost_day")
        check("back to day: settings restored to day profile",
              st == 200 and cam2.tuning["exposure_us"] == prof["exposure_us"]
              and cam2.tuning["contrast"] == prof["contrast"])
    finally:
        srv.should_exit = True
        th.join(timeout=5)
        streams.stop_all()
        rig.stop_all()

    if FAILS:
        print("%d FAILURE(S): %s" % (len(FAILS), FAILS))
        sys.exit(1)
    print("test_preset_stream OK")


if __name__ == "__main__":
    main()
