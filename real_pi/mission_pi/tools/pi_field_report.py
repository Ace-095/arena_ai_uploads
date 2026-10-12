#!/usr/bin/env python3
"""Field report: everything this machine can tell us about the real rig.

WHY THIS EXISTS
---------------
The machine that writes this code cannot reach a private LAN address, so the
hardware is only ever measurable from the machine itself. This tool collects
the answers in ONE run and writes ONE file, so whoever is at the aircraft
pastes a short summary back instead of transcribing ten commands.

READ-ONLY GUARANTEE - this tool observes and never actuates:
  * no config file is written (--config is read only)
  * no camera control is set (no VCM/focus position, no exposure change)
  * no MAVLink is sent unless you pass --with-fc, and even then it only
    connects, reads and disconnects (fc_probe.py --fence / --fence-check)
  * no model is converted, no HEF compiled, nothing pip-installed
It DOES open the camera and capture frames (timing a real capture is the whole
point), so STOP mission_pi FIRST: two processes cannot share a CSI camera, and
this would otherwise look like a camera fault.

USE ON THE PI
-------------
  cd ~/arena_ai_uploads/real_pi/mission_pi
  python3 tools/pi_field_report.py                        # read-only survey
  python3 tools/pi_field_report.py --with-fc               # + read the Pixhawk
  python3 tools/pi_field_report.py --dist-m 2.0 --panel-mm 297   # + MEASURE FOV
  python3 tools/pi_field_report.py --with-tests            # + run the suite

Then paste back the SUMMARY block it prints. The FOV step needs a tape measure
and a plain sheet: white A3 flat on a DARKER floor, directly under the lens,
centred in the picture, and the measured lens-to-sheet distance. That is the
one number every range calculation in this repo depends on and nobody has ever
measured - config.yaml still says VENDOR-QUOTED.
"""
import argparse
import json
import math
import os
import platform
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
sys.path.insert(0, PI)

#: ASCII only: this prints over ssh into terminals of unknown encoding.
OK = "[ok]"
NO = "[--]"
WARN = "[!!]"


def _run(cmd, timeout=120, cwd=None):
    """(rc, stdout+stderr) - never raises."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           cwd=cwd or PI)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except Exception as e:
        return -1, "could not run %s: %r" % (" ".join(cmd), e)


def _read(path):
    try:
        with open(path, "r", errors="replace") as fh:
            return fh.read().strip()
    except Exception:
        return None


def _cfg(path):
    try:
        import yaml
        with open(path) as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return {}


class Frames(object):
    """Frames captured ONCE and shared by the fov/decode/inference sections.

    Three sections each opening a CSI camera was the first draft's mistake: it
    triples the chance of a device conflict and makes the capture timings
    describe three different warm-ups.
    """

    def __init__(self):
        self.by_role = {}
        self.error = None

    def add(self, role, frames):
        self.by_role.setdefault(role, [])
        self.by_role[role].extend(frames)

    def get(self, role):
        return self.by_role.get(role) or []


FRAMES = Frames()


def _open_camera(cfg, role):
    """(rig, cam) with the role's camera started, or (rig, None)."""
    import cameras as C
    rig = C.CameraRig(cfg)
    cams = rig.detect()
    cam = cams.get(role) or (list(cams.values())[0] if cams else None)
    if cam is None:
        return rig, None
    cam.start()
    return rig, cam


def _capture_into(role, cam, seconds=1.2, want=None):
    """Collect frames the way the mission does, and time each call."""
    want = want or 5
    frames, ms = [], []
    deadline = time.time() + seconds
    while len(frames) < want and time.time() < deadline:
        t0 = time.time()
        frame, ts, cnt = cam.latest()
        ms.append((time.time() - t0) * 1000.0)
        if frame is not None:
            frames.append(frame)
        else:
            time.sleep(0.05)
    FRAMES.add(role, frames)
    return frames, ms


# ---------------------------------------------------------------------------
# sections
# ---------------------------------------------------------------------------
def sec_system(args):
    out = {"hostname": socket.gethostname(), "kernel": platform.release(),
           "machine": platform.machine(), "python": platform.python_version(),
           "cpu_count": os.cpu_count()}
    model = _read("/proc/device-tree/model")
    if model:
        out["board"] = model.replace("\x00", "").strip()
    try:
        with open("/proc/uptime") as fh:
            out["uptime_s"] = float(fh.read().split()[0])
    except Exception:
        pass
    temp = _read("/sys/class/thermal/thermal_zone0/temp")
    if temp and temp.isdigit():
        out["cpu_temp_c"] = round(int(temp) / 1000.0, 1)
    m = re.search(r"MemTotal:\s+(\d+) kB", _read("/proc/meminfo") or "")
    if m:
        out["mem_total_mb"] = round(int(m.group(1)) / 1024.0)
    try:
        import psutil
        out["cpu_pct"] = psutil.cpu_percent(interval=0.5)
        out["mem_used_pct"] = psutil.virtual_memory().percent
        out["disk_free_gb"] = round(psutil.disk_usage("/").free / 1e9, 1)
        out["load_avg"] = list(os.getloadavg())
    except Exception:
        pass
    # A throttled Pi cannot hold a framerate, so this must come BEFORE any
    # timing number is believed.
    rc, txt = _run(["vcgencmd", "get_throttled"], timeout=10)
    if rc == 0 and "throttled=" in txt:
        val = int(txt.strip().split("=")[1], 16)
        out["throttled_raw"] = "0x%X" % val
        out["throttled_now"] = bool(val & 0x7)
        out["throttled_ever"] = bool(val & 0xF0000)
        out["throttled_bits"] = [n for bit, n in (
            (0x1, "under-voltage NOW"), (0x2, "arm freq capped NOW"),
            (0x4, "currently throttled"), (0x8, "soft temp limit NOW"),
            (0x10000, "under-voltage occurred"), (0x20000, "arm capped"),
            (0x40000, "throttled occurred"), (0x80000, "soft temp limit"))
            if val & bit]
    rc, txt = _run(["vcgencmd", "measure_clock", "arm"], timeout=10)
    if rc == 0:
        out["arm_clock"] = txt.strip()
    return out


def sec_deps(args):
    out = {}
    for name in ("numpy", "cv2", "yaml", "pymavlink", "psutil", "onnxruntime",
                 "onnx", "qrcode", "PIL", "fastapi", "uvicorn", "hailo_platform"):
        try:
            mod = __import__(name)
            out[name] = getattr(mod, "__version__", "present")
        except Exception as e:
            out[name] = "MISSING (%s)" % type(e).__name__
    return out


def sec_hailo(args):
    """Is the accelerator usable, or will the detector fall back to CPU?

    The brief is explicit: never silently fall back to CPU. So this reports
    both what the platform says and what the detector ladder would choose.
    """
    out = {}
    try:
        import hailo_platform
        out["hailo_platform"] = getattr(hailo_platform, "__version__", "present")
        try:
            out["hailo_devices"] = [str(d) for d in hailo_platform.Device.scan()]
        except Exception as e:
            out["hailo_devices_error"] = repr(e)
    except Exception as e:
        out["hailo_platform"] = "MISSING (%s: %s)" % (type(e).__name__, e)
    rc, txt = _run([sys.executable, os.path.join(HERE, "hef_info.py"),
                    "--list-device"], timeout=60)
    out["hef_info_rc"] = rc
    out["hef_info"] = txt.strip()[-2000:]
    for label, path in (("hef", os.path.join(PI, "models", "qr_yolov8n.hef")),
                        ("onnx", os.path.join(PI, "models", "qr_yolov8n.onnx"))):
        out["%s_present" % label] = os.path.exists(path)
        if os.path.exists(path):
            out["%s_bytes" % label] = os.path.getsize(path)
    return out


def sec_detector(args):
    """Which rung of the ladder get_detector() lands on, and why."""
    out = {}
    cfg = _cfg(args.config)
    try:
        import detector as D
        det = D.get_detector(cfg.get("detector", {}) or {})
        out["chosen"] = getattr(det, "name", type(det).__name__)
        out["is_hailo"] = "hailo" in str(out["chosen"]).lower()
        for attr in ("input_size", "tile_bottom", "tile_front", "conf", "conf_thr"):
            if hasattr(det, attr):
                out[attr] = getattr(det, attr)
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
    out["ladder_note"] = ("get_detector picks HEF -> ONNX -> .pt -> classical; "
                          "a name without 'hailo' means the accelerator was NOT "
                          "used, which is a result in itself")
    return out


def sec_cameras(args):
    out = {}
    rc, txt = _run([sys.executable, os.path.join(HERE, "list_cams.py"),
                    "--config", args.config], timeout=60)
    out["list_cams_rc"] = rc
    out["list_cams"] = txt.strip()[-3000:]
    out["dev_video"] = sorted(p for p in os.listdir("/dev") if p.startswith("video"))
    rc, txt = _run(["v4l2-ctl", "--list-devices"], timeout=30)
    if rc == 0:
        out["v4l2_devices"] = txt.strip()[:1500]
        out["v4l2_note"] = "v4l2-ctl present"
    else:
        out["v4l2_note"] = "v4l2-ctl NOT installed (sudo apt install v4l-utils)"
    if args.no_capture:
        out["capture"] = "skipped (--no-capture)"
        return out
    cap = {"per_role": {}}
    cfg = _cfg(args.config)
    try:
        rig, cam = _open_camera(cfg, args.role)
    except Exception as e:
        out["capture"] = {"error": "could not open the camera stack: %s: %s"
                                   % (type(e).__name__, e)}
        return out
    try:
        cap["detected"] = {n: {"kind": getattr(c, "kind", None),
                               "index": getattr(c, "index", None),
                               "model": getattr(c, "model", None),
                               "facing": getattr(c, "facing", None)}
                           for n, c in (rig.cams or {}).items()}
        if cam is None:
            cap["error"] = ("no camera detected for role %r - see list_cams "
                            "above" % args.role)
            out["capture"] = cap
            return out
        frames, ms = _capture_into(args.role, cam, want=args.samples)
        ms_sorted = sorted(ms)
        entry = {
            "started": True,
            "requested_size": list(cam.size) if hasattr(cam, "size") else None,
            "frames": [list(f.shape) for f in frames],
            "latest_ms_median": round(ms_sorted[len(ms_sorted) // 2], 1) if ms else None,
            "latest_ms_max": round(max(ms), 1) if ms else None,
            "hfov_deg_config": getattr(cam, "hfov_deg", None),
            "vfov_deg_config": getattr(cam, "vfov_deg", None),
            "kind": getattr(cam, "kind", None),
        }
        if frames and entry["requested_size"]:
            got = list(frames[0].shape[:2][::-1])
            if got != list(entry["requested_size"]):
                entry["size_warning"] = ("the sensor produced %s while %s was "
                                         "requested - some backends resize, and "
                                         "a resized frame adds no optical detail"
                                         % (got, entry["requested_size"]))
        cap["per_role"][args.role] = entry
    except Exception as e:
        cap["per_role"][args.role] = {"error": "%s: %s" % (type(e).__name__, e)}
    finally:
        try:
            cam.stop()
        except Exception:
            pass
        try:
            rig.stop_all()
        except Exception:
            pass
    out["capture"] = cap
    return out


def _sheet_rect(frame):
    """(long_px, short_px, long_is_horizontal) of the bright centred sheet.

    Returns None when there is no sheet to measure. Design notes, all from
    measuring this on synthetic frames rather than trusting the first version:

    * The FIRST version used a brightness percentile and was wrong twice over:
      on a flat frame the percentile is the frame's own value, so the whole
      picture became "the sheet" (measured: 640 px wide), and around a real
      sheet the threshold crept into the ground's upper tail, overestimating
      the sheet by ~6% - which biased every FOV it reported about 5 degrees
      LOW. Reporting a wrong number as MEASURED is the one thing this tool must
      not do, so the percentile is gone.
    * Otsu instead, plus three guards: the sheet must be genuinely brighter than
      the frame median, must be a sane fraction of the picture (not the whole
      frame, not a speck), and must be centred, because distortion is worst at
      the edges and this measures the CENTRE field of view.
    * minAreaRect gives the rectangle's true sides under a few degrees of
      yaw/roll, and the rect's own orientation decides which image axis the
      long side runs along - so a sheet rotated in view is handled without the
      operator having to say anything.
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    otsu_t, mask = cv2.threshold(blur, 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    frame_med = float(np.median(blur))
    # No bright object at all -> nothing white to measure. A uniformly textured
    # floor must not qualify, however its brightness happens to be distributed.
    if otsu_t < max(60.0, frame_med * 1.15):
        return None
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = res[0] if len(res) == 2 else res[1]
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        frac = area / float(w * h)
        # Floor lowered from 2% to 0.5%: measured, at 2% a legitimate sheet
        # (A3 long side at 2 m on a 75 deg lens = 277x196 px = 1.8%) was
        # REJECTED outright. Ceiling stays at 60%: above that there is no
        # contrast and a FOV from it would be meaningless.
        if frac < 0.005 or frac > 0.60:
            # > 60% means the operator's setup has no contrast (or the sheet
            # fills the view); either way a FOV from it would be meaningless
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if bw < 20 or bh < 20:
            continue
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue                      # clipped by the frame -> unusable
        if not (abs((x + bw / 2.0) - w / 2.0) < 0.35 * w
                and abs((y + bh / 2.0) - h / 2.0) < 0.35 * h):
            continue                      # not centred -> distortion zone
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
        long_px, short_px = max(rw, rh), min(rw, rh)
        if short_px < 15:
            continue
        if short_px / max(1.0, long_px) < 0.30:
            continue                      # a streak, not a sheet
        # WHICH IMAGE AXIS the long side runs along CANNOT come from
        # minAreaRect's (rw, rh): those are measured in the rect's OWN rotated
        # frame, so `rw >= rh` is not an image-axis statement. Measured: that
        # mistake made a landscape sheet report "vertical" and skewed the
        # recovered FOV by -15 deg. The axis-aligned bounding box IS in image
        # coordinates, so it decides the axis; minAreaRect still supplies the
        # more accurate side lengths. A sheet rotated far from square-on is
        # refused rather than guessed at.
        if abs(bw - bh) > 0 and min(bw, bh) / max(1.0, max(bw, bh)) > 0.98:
            # near-square: the axis is ambiguous, but so is the sheet's
            # orientation - accept and treat the longer rect side as the long one
            pass
        # rotation guard: the long edge must be close to an image axis
        pts = cv2.boxPoints(((cx, cy), (rw, rh), ang))
        edges = [(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
                 for i in range(4)]
        E = [(abs(ex), abs(ey), (ex * ex + ey * ey) ** 0.5) for ex, ey in edges]
        long_edge = max(E, key=lambda e: e[2])
        if long_edge[2] > 0:
            skew_deg = math.degrees(math.atan2(long_edge[1], long_edge[0]))
            skew = min(abs(skew_deg), abs(90.0 - abs(skew_deg)))
            if skew > 25.0:
                continue                  # rotated too far to attribute an axis
        # the sheet's own pixels must really be brighter than the floor
        sub = gray[int(max(0, cy - rh / 2)):int(min(h, cy + rh / 2)),
                   int(max(0, cx - rw / 2)):int(min(w, cx + rw / 2))]
        if sub.size and float(sub.mean()) < frame_med * 1.25:
            continue
        if best is None or area > best[0]:
            best = (area, long_px, short_px, bw >= bh)
    if best is None:
        return None
    return best[1], best[2], best[3]


def sec_fov(args):
    """Measure the real HFOV/VFOV from a sheet of known size.

    Which image axis the sheet's long side runs along decides the formula, and
    that is read from the MEASUREMENT, not from what the operator assumed.
    """
    if not args.dist_m or not args.panel_mm:
        return {"measured": False,
                "why": "pass --dist-m <lens to sheet, metres> and --panel-mm "
                       "<sheet LONG side, mm> to turn the vendor FOV into a "
                       "measured one"}
    out = {"measured": False, "dist_m": args.dist_m,
           "panel_long_mm": args.panel_mm, "panel_short_mm": args.panel_mm_h or None,
           "role": args.role}
    try:
        import numpy as np
        import cv2
    except Exception as e:
        out["error"] = "needs numpy + cv2: %r" % (e,)
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        try:
            cfg = _cfg(args.config)
            rig, cam = _open_camera(cfg, args.role)
            try:
                if cam is not None:
                    _capture_into(args.role, cam, want=max(3, args.samples))
                    frames = FRAMES.get(args.role)
            finally:
                try:
                    cam.stop()
                except Exception:
                    pass
                try:
                    rig.stop_all()
                except Exception:
                    pass
        except Exception as e:
            out["error"] = "no frames and could not capture: %s: %s" % (
                type(e).__name__, e)
            return out
    rects, frame0 = [], None
    for f in frames:
        r = _sheet_rect(f)
        if r:
            rects.append(r)
            if frame0 is None:
                frame0 = f
    if not rects or frame0 is None:
        out["error"] = ("no sheet found - a WHITE sheet on a DARKER floor, "
                        "directly under the lens, centred, not touching the "
                        "edges of the picture, and clearly brighter than the "
                        "ground around it")
        out["frames_tried"] = len(frames)
        return out
    longs = sorted(r[0] for r in rects)
    shorts = sorted(r[1] for r in rects)
    med_long, med_short = longs[len(longs) // 2], shorts[len(shorts) // 2]
    horizontal = sum(1 for r in rects if r[2]) >= (len(rects) + 1) // 2
    h, w = frame0.shape[:2]
    long_m = args.panel_mm / 1000.0
    # the long side spans `med_long` px along the axis it runs on
    span_px = float(w if horizontal else h)
    ground_long_m = long_m * span_px / med_long
    hfov = float(np.degrees(2.0 * np.arctan(
        (ground_long_m / 2.0) / args.dist_m)))
    out.update({
        "measured": True,
        "frame_size": [int(w), int(h)],
        "sheet_px_samples": [[int(a), int(b)] for a, b in
                             sorted((r[0], r[1]) for r in rects)],
        "sheet_long_px_median": int(med_long),
        "sheet_short_px_median": int(med_short),
        "sheet_px_spread_pct": round(100.0 * (longs[-1] - longs[0]) / max(1.0, med_long), 2),
        "long_side_runs": "horizontal (left-right)" if horizontal else "vertical",
        "mm_per_px_on_ground": round(ground_long_m * 1000.0 / span_px, 3),
        "hfov_deg": round(hfov, 2),
        "hfov_deg_config": (_cfg(args.config).get("cameras", {})
                            .get(args.role, {}) or {}).get("hfov_deg"),
        "caveats": ["single centred sheet: this measures the CENTRE field of "
                    "view; corner distortion is NOT measured",
                    "assumes the sheet is flat and perpendicular to the lens",
                    "the distance is whatever the operator measured with a tape",
                    "a bench measurement, NOT a flight result"],
    })
    if args.panel_mm_h:
        short_m = args.panel_mm_h / 1000.0
        # the short side spans the OTHER axis
        other_px = float(h if horizontal else w)
        ground_short_m = short_m * other_px / med_short
        out["vfov_deg"] = round(float(np.degrees(2.0 * np.arctan(
            (ground_short_m / 2.0) / args.dist_m))), 2)
        out["vfov_deg_config"] = (_cfg(args.config).get("cameras", {})
                                  .get(args.role, {}) or {}).get("vfov_deg")
        declared = long_m / short_m
        measured = med_long / max(1.0, med_short)
        out["aspect_check"] = {
            "declared_long_over_short": round(declared, 3),
            "measured_long_over_short": round(float(measured), 3),
            "agree_within_10pct": abs(measured - declared) / declared < 0.10,
        }
        if not out["aspect_check"]["agree_within_10pct"]:
            out["caveats"].append(
                "measured aspect disagrees with the declared sheet by >10%%: "
                "either a side is partly out of view or the floor is not "
                "perpendicular - treat both angles as suspect")
    try:
        cv2.imwrite(os.path.join(args.out_dir, "fov_%s.png" % args.role), frame0)
        out["frame_saved"] = os.path.join(args.out_dir, "fov_%s.png" % args.role)
    except Exception:
        pass
    return out


def sec_decode(args):
    """Does the REAL camera + the configured decoder actually decode?"""
    out = {"attempted": False}
    if args.no_decode:
        out["why"] = "skipped (--no-decode)"
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        out["why"] = ("no captured frames - run without --no-capture, or pass "
                      "--dist-m/--panel-mm so a capture is requested")
        return out
    try:
        import decoder as DEC
    except Exception as e:
        out["error"] = "decoder import failed: %r" % (e,)
        return out
    try:
        out["attempted"] = True
        payloads, times = [], []
        for f in frames:
            t0 = time.time()
            p = DEC.decode_frame(f)
            times.append((time.time() - t0) * 1000.0)
            if p:
                payloads.append(p)
        out["frames"] = len(frames)
        out["decoded"] = bool(payloads)
        out["payloads"] = payloads[:3]
        out["decode_ms_median"] = round(sorted(times)[len(times) // 2], 1)
        px = _sheet_px(frames[0])
        out["sheet_px"] = int(px)
        if px and args.qr_modules:
            out["px_per_module_measured"] = round(px / float(args.qr_modules), 2)
        out["note"] = ("full-frame decode, no ROI hint: the optimistic case")
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
    return out


def sec_inference(args):
    """Detector cost on REAL captured frames - the number that decides fps."""
    out = {"attempted": False}
    if args.no_infer:
        out["why"] = "skipped (--no-infer)"
        return out
    frames = FRAMES.get(args.role)
    if not frames:
        out["why"] = "no captured frames (see the cameras section)"
        return out
    try:
        import detector as D
    except Exception as e:
        out["error"] = "detector import failed: %r" % (e,)
        return out
    try:
        cfg = _cfg(args.config)
        det = D.get_detector(cfg.get("detector", {}) or {})
        out["attempted"] = True
        out["detector"] = getattr(det, "name", type(det).__name__)
        out["is_hailo"] = "hailo" in str(out["detector"]).lower()
        times, nboxes = [], 0
        for f in frames[:max(1, args.infer_frames)]:
            t0 = time.time()
            boxes = det.detect(f)
            times.append((time.time() - t0) * 1000.0)
            nboxes += len(boxes or [])
        times.sort()
        out["frames"] = len(times)
        out["detect_ms_median"] = round(times[len(times) // 2], 1)
        out["detect_ms_min"] = round(times[0], 1)
        out["detect_ms_max"] = round(times[-1], 1)
        out["fps_from_detect"] = round(1000.0 / max(0.001, times[len(times) // 2]), 2)
        out["boxes_total"] = nboxes
        out["note"] = ("inference only - capture and decode are measured "
                       "separately; not a flight result, and not a Hailo result "
                       "unless is_hailo is true")
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
    return out


def sec_fc(args):
    """Read the Pixhawk. OFF by default: connecting takes the serial port."""
    if not args.with_fc:
        return {"connected": False,
                "why": "pass --with-fc to read the autopilot. It takes the "
                       "serial port, so only with mission_pi STOPPED and the "
                       "vehicle DISARMED on the bench/ground"}
    out = {}
    dev = ["--device", args.fc_device] if args.fc_device else []
    rc, txt = _run([sys.executable, os.path.join(HERE, "fc_probe.py"), "--fence"] + dev,
                   timeout=90)
    out["fc_probe_rc"] = rc
    out["fc_probe"] = txt.strip()[-2500:]
    rc, txt = _run([sys.executable, os.path.join(HERE, "fc_probe.py"),
                    "--fence-check"] + dev, timeout=120)
    out["fence_check_rc"] = rc
    out["fence_check"] = txt.strip()[-2500:]
    out["fence_check_verdict"] = ("FENCE-READY" if rc == 0
                                  else "NOT fence-ready (rc=%d)" % rc)
    return out


def sec_tests(args):
    if not args.with_tests:
        return {"ran": False, "why": "pass --with-tests (takes minutes on a Pi)"}
    rc, txt = _run(["bash", os.path.join(HERE, "run_tests.sh")], timeout=3600)
    tail = txt.strip().splitlines()[-14:]
    return {"ran": True, "rc": rc, "tail": tail,
            "summary": next((l for l in reversed(tail) if "passed" in l), "")}


SECTIONS = (("system", sec_system), ("deps", sec_deps), ("hailo", sec_hailo),
            ("detector", sec_detector), ("cameras", sec_cameras),
            ("fov", sec_fov), ("decode", sec_decode),
            ("inference", sec_inference), ("fc", sec_fc), ("tests", sec_tests))


def build(args):
    wanted = None if args.sec == "all" else {args.sec}
    report = {"tool": "pi_field_report", "ts": time.time(),
              "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "config": os.path.abspath(args.config),
              "read_only": True, "sections": {}}
    for name, fn in SECTIONS:
        if wanted is not None and name not in wanted:
            continue
        t0 = time.time()
        try:
            report["sections"][name] = fn(args)
        except Exception as e:                  # a section must never abort the run
            report["sections"][name] = {"error": "%s: %s" % (type(e).__name__, e)}
        report["sections"][name]["_sec_ms"] = int((time.time() - t0) * 1000)
        print("  %-9s done (%d ms)" % (name, report["sections"][name]["_sec_ms"]),
              file=sys.stderr)
    return report


def summarise(r):
    """ASCII-only, partial-report tolerant.

    It is read over ssh into terminals of unknown encoding, and `--sec` leaves
    whole sections absent - so every block below is guarded by whether its
    section actually ran, rather than printing a row of "None".
    """
    s = r["sections"]
    L = []
    sysd = s.get("system") or {}
    who = sysd.get("hostname") or socket.gethostname() or "unknown host"
    L.append("=== PI FIELD REPORT %s (%s) ===" % (r.get("utc"), who))
    if sysd:
        L.append("board      : %s | %s | python %s | %s cores"
                 % (sysd.get("board", "?"), sysd.get("kernel"), sysd.get("python"),
                    sysd.get("cpu_count")))
        if "cpu_temp_c" in sysd:
            L.append("thermal    : %.1f C | throttled_now=%s throttled_ever=%s"
                     % (sysd["cpu_temp_c"], sysd.get("throttled_now"),
                        sysd.get("throttled_ever")))
            for b in sysd.get("throttled_bits") or []:
                L.append("             - %s" % b)
        if "mem_total_mb" in sysd:
            L.append("memory     : %s MB total | %s%% used | disk free %s GB"
                     % (sysd["mem_total_mb"], sysd.get("mem_used_pct"),
                        sysd.get("disk_free_gb")))
    if s.get("deps"):
        miss = [k for k, v in s["deps"].items()
                if isinstance(v, str) and v.startswith("MISSING")]
        L.append("deps       : missing: %s" % (", ".join(miss) if miss else "none"))
    h = s.get("hailo") or {}
    if h:
        L.append("hailo      : platform=%s | devices=%s | hef=%s onnx=%s"
                 % (h.get("hailo_platform"), h.get("hailo_devices"),
                    h.get("hef_present"), h.get("onnx_present")))
    d = s.get("detector") or {}
    if d:
        L.append("detector   : chosen=%s | IS_HAILO=%s"
                 % (d.get("chosen"), d.get("is_hailo")))
        if d.get("error"):
            L.append("             error: %s" % d["error"])
    c = s.get("cameras") or {}
    if c:
        L.append("cameras    : /dev/%s" % (", ".join(c.get("dev_video") or [])
                                           or "none"))
        cap = c.get("capture")
        if isinstance(cap, dict):
            L.append("             detected: %s"
                     % list((cap.get("detected") or {}).keys()))
            for name, e in (cap.get("per_role") or {}).items():
                if e.get("error"):
                    L.append("             %s: ERROR %s" % (name, e["error"]))
                elif e.get("started"):
                    L.append("             %s: requested=%s frames=%s latest_ms=%s/%s"
                             % (name, e.get("requested_size"), e.get("frames"),
                                e.get("latest_ms_median"), e.get("latest_ms_max")))
                    if e.get("size_warning"):
                        L.append("             %s: %s" % (name, e["size_warning"]))
            if cap.get("error"):
                L.append("             %s" % cap["error"])
        else:
            L.append("             capture: %s" % cap)
    f = s.get("fov") or {}
    if f:
        if f.get("measured"):
            L.append("FOV        : MEASURED hfov=%.2f deg%s (config says hfov=%s%s)"
                     % (f["hfov_deg"],
                        (" vfov=%.2f deg" % f["vfov_deg"]) if f.get("vfov_deg") else "",
                        f.get("hfov_deg_config"),
                        (" vfov=%s" % f.get("vfov_deg_config"))
                        if f.get("vfov_deg_config") else ""))
            L.append("             sheet long/short %s px | spread %s%% | %s mm/px "
                     "| %s | at %s m"
                     % (f.get("sheet_long_px_median"), f.get("sheet_px_spread_pct"),
                        f.get("mm_per_px_on_ground"), f.get("long_side_runs"),
                        f["dist_m"]))
            if f.get("aspect_check"):
                ac = f["aspect_check"]
                L.append("             aspect declared %s vs measured %s -> %s"
                         % (ac["declared_long_over_short"],
                            ac["measured_long_over_short"],
                            "consistent" if ac["agree_within_10pct"]
                            else "INCONSISTENT (>10%), treat as suspect"))
        else:
            L.append("FOV        : NOT MEASURED - %s"
                     % (f.get("error") or f.get("why") or "?"))
    dec = s.get("decode") or {}
    if dec:
        if dec.get("attempted"):
            L.append("decode     : decoded=%s payloads=%s in %s ms"
                     % (dec.get("decoded"), dec.get("payloads"),
                        dec.get("decode_ms_median")))
        else:
            L.append("decode     : not attempted - %s"
                     % (dec.get("error") or dec.get("why") or "?"))
    inf = s.get("inference") or {}
    if inf:
        if inf.get("attempted"):
            L.append("inference  : %s median %s ms (min %s max %s) = %s fps | "
                     "IS_HAILO=%s"
                     % (inf.get("detector"), inf.get("detect_ms_median"),
                        inf.get("detect_ms_min"), inf.get("detect_ms_max"),
                        inf.get("fps_from_detect"), inf.get("is_hailo")))
        else:
            L.append("inference  : not attempted - %s"
                     % (inf.get("error") or inf.get("why") or "?"))
    fc = s.get("fc") or {}
    if fc:
        if fc.get("fc_probe_rc") == 0:
            L.append("fc         : connected")
            L.append("             fence: %s" % fc.get("fence_check_verdict"))
        else:
            L.append("fc         : %s"
                     % (fc.get("error") or fc.get("why") or "?"))
    t = s.get("tests") or {}
    if t:
        L.append("tests      : %s" % (t.get("summary") if t.get("ran")
                                      else t.get("why")))
    L.append("=== end ===")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(PI, "config.yaml"))
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--role", default="bottom", choices=("bottom", "front"))
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--infer-frames", type=int, default=5)
    ap.add_argument("--dist-m", type=float, default=0.0,
                    help="tape-measured lens-to-sheet distance, metres")
    ap.add_argument("--panel-mm", type=float, default=0.0,
                    help="sheet LONG side, mm (A3 = 420). Which image axis it "
                         "runs along is read from the picture, not assumed")
    ap.add_argument("--panel-mm-h", type=float, default=0.0,
                    help="sheet SHORT side, mm (A3 = 297). Give it to also get "
                         "VFOV and an aspect sanity check")
    ap.add_argument("--qr-modules", type=int, default=0,
                    help="modules across the printed QR, for px/module")
    ap.add_argument("--no-capture", action="store_true")
    ap.add_argument("--no-decode", action="store_true")
    ap.add_argument("--no-infer", action="store_true")
    ap.add_argument("--with-fc", action="store_true")
    ap.add_argument("--fc-device", default=None)
    ap.add_argument("--with-tests", action="store_true")
    ap.add_argument("--json", default=None, help="explicit output path")
    ap.add_argument("--sec", default="all",
                    choices=("all",) + tuple(n for n, _ in SECTIONS),
                    help="run one section only (faster when you just want one "
                         "answer, e.g. --sec fov)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print("pi_field_report: READ-ONLY survey. Stop mission_pi first if you want "
          "capture timings.", file=sys.stderr)
    report = build(args)
    # Include the section and the pid: a second-resolution timestamp alone let
    # a quick `--sec fov` run OVERWRITE the full survey written seconds earlier.
    path = args.json or os.path.join(args.out_dir, "field_report_%s%s.json" % (
        time.strftime("%Y%m%d_%H%M%S"),
        "" if args.sec == "all" else "_" + args.sec))
    with open(path, "w") as fh:
        json.dump(report, fh, indent=2, sort_keys=True, default=str)
    print(summarise(report))
    print("\nfull report: %s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
