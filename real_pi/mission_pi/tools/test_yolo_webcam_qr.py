#!/usr/bin/env python3
"""QR webcam bench tool — the window-grill killer with live presets.

Watches a webcam (or a video file / RTSP URL), runs the SAME detector +
fake-QR filter + preset table the mission uses (detector.py, decoder.py,
qr_filter.py — no duplicated logic), and draws:

  * GREEN thick box + "QR 0.85 MISSION-QR-001"  — a box that PASSED the
    filter AND decoded to a real payload (the only boxes that matter)
  * GREEN thin box + "qr 0.52"                  — filter survivor (bbox cue)
  * ORANGE thin box + "fake 0.24 (aspect ...)"  — a REJECTED box (grill,
    speck, odd shape); shown only while fakes are not hidden (key f)

HUD (top-left): preset, conf, tile grid, require-decode, filter floor,
`raw:N filt:M` counts (raw detector boxes vs filter survivors) + fps.

Why: a diamond window grill read as "qr" at conf 0.16-0.24 and filled the
view with orange boxes. Default conf is now 0.35 + a geometry gate
(square aspect 0.4-2.5, min 15 px side). On the ground, use a
require-decode preset: only boxes that decode to a real payload count.

USAGE (from the mission_pi/ directory or the repo root):
  python3 tools/test_yolo_webcam_qr.py --source 0 \
      --model models/qr_yolov8n.onnx --onnx \
      --preset dark --require-decode --tiled

  # ON THE PI, with the real CSI camera (e.g. the Arducam UC-517) and the
  # Hailo AI HAT — no webcam, no server:
  python3 tools/test_yolo_webcam_qr.py --source csi:bottom --hailo \
      --hef models/qr_yolov8n.hef --preset day --require-decode --tiled

  # headless bench run (no window; prints one summary line per second)
  python3 tools/test_yolo_webcam_qr.py --source /tmp/demo.mp4 \
      --model models/qr_yolov8n.onnx --headless
  python3 tools/test_yolo_webcam_qr.py --source csi --hailo --headless

  # NO DISPLAY (over SSH, e.g. on the Pi)? It runs headless by itself instead
  # of letting Qt abort the process, and --save-frames gives you JPEGs to
  # scp back and look at:
  python3 tools/test_yolo_webcam_qr.py --source csi:bottom --onnx \
      --preset day --require-decode --tiled --save-frames /tmp/bench

LIVE KEYS (window mode):
  f  hide/show rejected (fake) boxes        v  toggle require-decode
  k  conf +0.05                              l  conf -0.05
  t  toggle 3x3 tiling                       p  next preset
  1-8 select preset by number                q / ESC quit

Ground recommendation (field report): preset `dark` (a.k.a.
dark_qr_boost) + --tiled for night; preset `kabaddi` at ground level.
Flight 15 m search: preset `day` (default).
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

try:
    import yaml
except Exception:
    yaml = None

PRESET_KEYS = "12345678"  # key -> preset list position


#: Which tuning keys a boost profile is expected to move. Only used to PRINT
#: what changed — the application itself goes through the mission's own
#: Camera.apply_qr_profile(), so the bench and the aircraft cannot diverge.
BOOST_KEYS = ("exposure_us", "gain_db", "brightness", "contrast", "saturation",
              "sharpness")


def apply_preset_boost(cap, preset):
    """Push qr_presets.<name>.boost onto the CSI camera -> a log line or None.

    Until 2026-10-12 the bench applied a preset by changing conf/tiling only:
    pressing 1-7 moved the detector but NOT the camera, so the window kept the
    same exposure/contrast look whatever preset was selected — exactly what the
    operator reported ("presets change but the window stays in that preset's
    look"). The mission has always applied the boost through
    Camera.apply_qr_profile(); the bench now uses the same call.
    """
    name = (preset or {}).get("boost")
    cam = getattr(cap, "cam", None)          # only CSISource has a Camera
    fn = getattr(cam, "apply_qr_profile", None)
    if not name or not callable(fn):
        return None
    try:
        t = fn(name) or {}
    except Exception as e:
        return ("[boost] %s FAILED: %r — the camera keeps its previous ISP "
                "settings" % (name, e))
    shown = ["%s=%s" % (k, t[k]) for k in BOOST_KEYS if k in t]
    return "[boost] %s -> %s" % (name, ", ".join(shown) or "no change reported")


def _view(frame, max_w):
    """A window-sized copy of the frame. The Pi's CSI frame is 2028x1520; pushing
    that full size through the VNC window is what made the preview stutter."""
    try:
        h, w = frame.shape[:2]
        if max_w and w > max_w:
            import cv2
            return cv2.resize(frame, (int(max_w), int(h * max_w / float(w))),
                              interpolation=cv2.INTER_LINEAR)
    except Exception:
        pass
    return frame


def preset_index(key, n_presets):
    """Which preset does this keypress select? -> index or None.

    `cv2.waitKey` returns an INT; the field crash was
    `TypeError: ord() expected string of length 1, but int found` — the digit
    branch turned the int key back into a character. Keep the arithmetic here,
    pure, so it is tested instead of discovered by pressing a key.
    """
    try:
        idx = int(key) - ord("1")
    except (TypeError, ValueError):
        return None
    return idx if 0 <= idx < int(n_presets) else None


def probe_display(cv_mod=None, env=None):
    """Can a window REALLY open here? -> (ok, reason).

    `DISPLAY` being set is not enough. On a Pi over SSH, `DISPLAY=:0` (often
    exported from .bashrc, or left over from an `ssh -X`) points at the
    desktop's X server while the session has no X cookie — and then Qt/GTK
    ABORT the whole process (SIGABRT, not a Python exception), which no
    try/except can survive. The only safe way to ask is in a CHILD process:
    if the child dies, the bench still runs.

    Also catches an OpenCV built without GUI support (opencv-python-headless),
    where imshow simply does not exist.
    """
    env = os.environ if env is None else env
    if cv_mod is None:
        try:
            import cv2 as cv_mod          # imported lazily elsewhere too
        except Exception as e:
            return False, "cv2 is not importable (%r) — cannot open a window" % (e,)
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
        return False, ("no DISPLAY/WAYLAND_DISPLAY — this is a headless shell "
                       "(SSH): use --save-frames to see annotated JPEGs, or "
                       "--show from a VNC/desktop terminal")
    if not hasattr(cv_mod, "imshow") or not hasattr(cv_mod, "namedWindow"):
        return False, ("this cv2 has no GUI support (opencv-python-headless?) "
                       "— no window is possible in this Python; use "
                       "--save-frames")
    if not hasattr(os, "fork"):                     # Windows: env check only
        return True, "display env present (cannot probe on this platform)"
    try:
        pid = os.fork()
    except OSError:
        return True, "display env present (fork unavailable to probe)"
    if pid == 0:                                    # child: try, then die
        code = 0
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, 1)
            os.dup2(devnull, 2)
            cv_mod.namedWindow("__display_probe__")
            cv_mod.destroyAllWindows()
        except Exception:
            code = 2
        os._exit(code)
    _, status = os.waitpid(pid, 0)
    if os.WIFSIGNALED(status):
        return False, ("DISPLAY is set (%s) but no window can be created here "
                       "— the attempt was killed by signal %d (no X cookie / "
                       "no X forwarding). Running headless; use --save-frames, "
                       "or run this from the VNC desktop's terminal"
                       % (os.environ.get("DISPLAY") or os.environ.get(
                           "WAYLAND_DISPLAY"), os.WTERMSIG(status)))
    if os.WEXITSTATUS(status) != 0:
        return False, ("DISPLAY is set but OpenCV could not open a window "
                       "there — running headless (use --save-frames)")
    return True, "display probe succeeded — window will open"


def decide_show(headless, show, env=None, probe=None):
    """Open a window? -> (show, reason).

    Qt ABORTS the whole process (SIGABRT, not a Python exception) when it
    cannot init a platform plugin, and over SSH there is no display — so the
    decision has to be made BEFORE the first imshow(), never as a try/except
    around it. Explicit flags always win; otherwise a display decides, and a
    display that cannot actually open a window counts as no display.

    Pure apart from `probe` (injected in tests) so it can be tested with no
    display, no camera and no Qt.
    """
    env = os.environ if env is None else env
    if headless:
        return False, "headless requested"
    if show:
        return True, "window forced with --show"
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
        return False, ("no DISPLAY/WAYLAND_DISPLAY (SSH?) — running headless; "
                       "use --save-frames to see annotated JPEGs, or --show "
                       "from a VNC/desktop terminal")
    # A display is announced — but announcing is not the same as working, and
    # a broken one aborts the process, so ask for real (in a child) first.
    probe = probe_display if probe is None else probe
    ok, why = probe(None, env)
    return bool(ok), why


def load_cfg(path):
    if path and os.path.isfile(path) and yaml is not None:
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            if isinstance(data, dict):
                return data
        except Exception as e:
            print("[presets] could not read %s (%r) — using built-ins" % (path, e))
    return {}


def build_detector(args, conf_thr, prefer, kind="yolo"):
    """Use the repo's detector ladder so the tool == the mission.

    kind="hailo" uses the AI HAT (.hef). A missing HAT/HEF degrades to the
    same CPU ladder the mission would use — an explicit --hailo must never
    turn a bench run into a traceback.
    """
    from detector import get_detector
    base = {
        "model_path": args.model or "",
        "conf_thr": conf_thr,
        "iou_thr": args.iou,
        "input_size": 640,
        "prefer": prefer,
        # How to read a Hailo NMS box (see config.yaml detector.nms_order).
        "nms_order": getattr(args, "nms_order", "yxyx"),
    }
    if kind == "hailo":
        try:
            return get_detector(dict(base, kind="hailo",
                                     hef_path=getattr(args, "hef", "")))
        except Exception as e:
            print("[detector] HAILO unavailable (%r) — using the CPU ladder "
                  "instead (hailortcli scan / the .hef path)" % e)
    cfg = dict(base, kind="yolo")
    try:
        return get_detector(cfg)
    except Exception as e:
        print("[detector] YOLO unavailable (%r) — falling back to classical" % e)
        from detector import ClassicalDetector
        return ClassicalDetector()


#: Laplacian variance below which a frame that HAS contrast is called soft.
#: A floor for "something is wrong", not a target: this metric also scales
#: with resolution, contrast and sensor noise, so it is only meaningful next
#: to itself (turn the lens ring and it must rise) or another camera of the
#: same model. The 2026-10-12 field report was "camera looks blurry, idk why"
#: with nothing in the output saying whether that was the lens, the light or
#: the surface — hence a number in every stats line.
MIN_FOCUS = 25.0


def focus_score(frame, cv2mod=None):
    """Sharpness of the centre 50% — -1.0 when it cannot be measured.

    Same metric as `tools/focus_sweep.py` prints (the sweep's own function is
    called, so the two can never drift), which is what lets a value seen live
    be compared against a sweep table.
    """
    try:
        import focus_sweep as FS
        if cv2mod is None:
            import cv2 as cv2mod
        return FS.sharpness(frame, cv2mod, centre=0.5)
    except Exception:
        pass
    try:                                   # sweep module unavailable: same maths
        import cv2
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = g.shape[:2]
        y0, x0 = h // 4, w // 4
        return float(cv2.Laplacian(g[y0:h - y0, x0:w - x0], cv2.CV_64F).var())
    except Exception:
        return -1.0


def vision_hint(mean, std, focus):
    """Why the camera can see nothing? -> one line, or None. Pure.

    Ordered by what has to be fixed FIRST: a black frame has no contrast and
    no sharpness either, so calling a lens cap "blurry" would send the
    operator to adjust a lens that is fine. Returns None when the numbers are
    unavailable (-1), because guessing is how time gets wasted.
    """
    if mean is None or mean < 0:
        return None
    if mean < 12:
        return ("[hint] the frame is nearly BLACK (mean %.0f) — lens cap on, "
                "no light, or exposure/gain wrong. Nothing can be detected in "
                "a black frame." % mean)
    if std < 8:
        return ("[hint] the frame has almost no CONTRAST (std %.0f) — it is a "
                "blank surface (wall, ceiling, out-of-focus distance). Point "
                "the camera at the printed QR panel." % std)
    if 0 <= focus < MIN_FOCUS:
        return ("[hint] the frame has contrast (std %.0f) but is SOFT "
                "(focus=%.0f, floor %.0f): the image is out of focus. On the "
                "UC-517 focus is MECHANICAL — loosen the ring on the lens "
                "barrel and turn it while watching focus= rise, at the "
                "distance you care about, then re-tighten. --save-frames "
                "/tmp/bench keeps the frames to compare afterwards."
                % (std, focus, MIN_FOCUS))
    return None


def frame_stats(frame):
    """(mean, std) brightness/contrast of a BGR frame — pure, numpy only.

    Added after the field run where raw=0 for 15 frames and nothing in the
    output said whether the camera was looking at a wall, sitting in the dark,
    or simply not in front of the QR. Numbers settle that in one line.
    """
    try:
        import numpy as np
        a = np.asarray(frame)
        if a.ndim == 3:
            a = a.mean(axis=2)
        return float(a.mean()), float(a.std())
    except Exception:
        return -1.0, -1.0


class CSISource:
    """The Pi's OWN CSI camera, behind the tiny cv2.VideoCapture interface.

    The UC-517 / any Pi CSI camera is a libcamera device that OpenCV cannot
    open, so `--source 0` only ever finds USB webcams. This source runs the
    mission's own camera code (same auto-detect, same tuning file, same
    autofocus gate), which is what makes a bench number mean something for
    the real aircraft.

        --source csi             best guess: the bottom / downward camera
        --source csi:bottom      by role  (front | bottom)
        --source csi:cam2        by rig slot
        --source csi:1           by libcamera index (tools/list_cams.py)
    """

    def __init__(self, spec, cfg=None):
        from cameras import CameraRig
        want = str(spec).split(":", 1)[1].strip().lower() if ":" in str(spec) else ""
        self._rig = CameraRig(cfg or {})
        cams = self._rig.detect() or {}
        if not cams:
            raise RuntimeError("no CSI camera detected — check the ribbon, "
                               "the boot overlay (dtoverlay=imx477,vcm) and "
                               "run tools/list_cams.py")
        pick = None
        if want.isdigit():
            pick = next((c for c in cams.values()
                         if str(getattr(c, "index", "")) == want), None)
        elif want in ("front", "bottom"):
            pick = next((c for c in cams.values()
                         if str(getattr(c, "facing", "")) == want), None)
        elif want in cams:
            pick = cams[want]
        if pick is None:
            if want:
                print("[camera] %r matched nothing in %s — using the best guess"
                      % (want, sorted(cams)))
            pick = next((c for c in cams.values()
                         if str(getattr(c, "facing", "")) == "bottom"), None) \
                or next(iter(cams.values()))
        self.cam = pick
        self._size = tuple(getattr(pick, "size", (0, 0)) or (0, 0))
        pick.start()
        print("[camera] CSI %s (%s) model=%s size=%s autofocus=%s"
              % (pick.name, pick.facing, pick.model, self._size,
                 getattr(pick, "autofocus", None)))
        _qb = getattr(pick, "qr_boost", None) or {}
        print("[detector-input] %s  (config.yaml qr_boost.detect_raw)"
              % ("RAW / gamma-corrected" if _qb.get("detect_raw")
                 else "software-boosted (CLAHE+unsharp)"))
        print("[exposure] dark frames are rescued automatically: a manual "
              "profile hands exposure back to the AE (%s)"
              % ("arms after %.0f s" % getattr(pick, "RESCUE_COOLDOWN", 5.0)))

    # -- cv2.VideoCapture lookalikes -------------------------------------
    def isOpened(self):
        return True

    def get(self, prop):
        try:
            import cv2
            if prop == cv2.CAP_PROP_FRAME_WIDTH:
                return float(self._size[0] or 0)
            if prop == cv2.CAP_PROP_FRAME_HEIGHT:
                return float(self._size[1] or 0)
        except Exception:
            pass
        return 0.0

    def set(self, *a, **k):
        return False                      # size/fps come from config.yaml

    def read(self):
        """(ok, BGR frame) — a NEW frame, waiting up to ~2 s for one.

        The camera thread keeps the latest frame. Returning it again when no
        new one has arrived made the bench detect (and draw) the same picture
        several times per camera frame — wasted CPU that showed up as lag."""
        deadline = time.time() + 2.0
        last = getattr(self, "_last_n", -1)
        while time.time() < deadline:
            try:
                frame, _ts, n = self.cam.latest(detect=True)
            except Exception:
                frame, n = None, None
            if frame is not None and n != last:
                self._last_n = n
                return True, frame
            time.sleep(0.01)
        return False, None

    def release(self):
        try:
            self.cam.stop()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0",
                    help="camera index (0), device path (/dev/video0), "
                         "RTSP/HTTP URL, or video file (default: 0)")
    ap.add_argument("--model", default=None,
                    help="YOLO weights (models/qr_yolov8n.onnx or .pt); "
                         "default: auto-find in mission_pi/models/")
    ap.add_argument("--onnx", action="store_true",
                    help="prefer the .onnx weights (onnxruntime, no torch)")
    ap.add_argument("--pt", action="store_true",
                    help="prefer the .pt weights (ultralytics)")
    ap.add_argument("--classical", action="store_true",
                    help="no model — cv2/pyzbar detector (short range only)")
    ap.add_argument("--hailo", action="store_true",
                    help="detect with the Hailo AI HAT (needs hailo_platform "
                         "+ the .hef); falls back to the CPU ladder if "
                         "either is missing")
    ap.add_argument("--hef", default="models/qr_yolov8n.hef",
                    help="Hailo .hef model for --hailo "
                         "(default: models/qr_yolov8n.hef)")
    ap.add_argument("--conf", type=float, default=None,
                    help="detector conf floor (default: the preset's conf)")
    ap.add_argument("--iou", type=float, default=0.45, help="NMS iou (default 0.45)")
    ap.add_argument("--preset", default="day",
                    help="qr_presets entry: day|far|dark|kabaddi|low|auto|"
                         "aggressive|bench (aliases: dark_qr_boost, night, "
                         "ground, 15m). 'none' = plain --conf, no preset")
    ap.add_argument("--require-decode", action="store_true",
                    help="only a box that DECODED to a payload counts "
                         "(ground runs: only real QRs trigger)")
    ap.add_argument("--no-require-decode", dest="require_decode",
                    action="store_false")
    ap.set_defaults(require_decode=None)  # None = follow the preset
    ap.add_argument("--tiled", action="store_true",
                    help="force 3x3 tile inference (15 m answer)")
    ap.add_argument("--no-tile", action="store_true",
                    help="force full-frame inference (bench / close range)")
    ap.add_argument("--config", default=None,
                    help="config.yaml to read qr_presets/qr_filter from "
                         "(default: mission_pi/config.yaml next to this tool)")
    ap.add_argument("--headless", action="store_true",
                    help="no window — one summary line per second (bench CI)")
    ap.add_argument("--show", action="store_true",
                    help="force the window even without DISPLAY "
                         "(default: window only when a display exists)")
    ap.add_argument("--view-width", type=int, default=1024,
                    help="window width in px (the frame is shrunk for the "
                         "window only; detection always sees full resolution). "
                         "0 = full size. Smaller = smoother over VNC.")
    ap.add_argument("--save-frames", default="",
                    help="dir to write annotated frames into (default off) — "
                         "the way to SEE a headless run; one JPEG/second")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (0 = run until key/file end)")
    ap.add_argument("--nms-order", dest="nms_order", default="yxyx",
                    choices=["yxyx", "xyxy"],
                    help="order of the 5 values in a Hailo NMS box "
                         "(default yxyx = HailoRT's own layout: y_min, x_min, "
                         "y_max, x_max, score; use xyxy only if boxes come out "
                         "mirrored across the frame diagonal)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="DEBUG logging (per-frame detector detail)")
    args = ap.parse_args()

    # Without this the detector's own lines ("hailo ready: … outs=…",
    # "hailo output layout: A'") never reach the operator — only warnings
    # survive the default root logger, and which rung/layout is in use is
    # exactly what a field log has to say.
    import logging
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="[%(name)s] %(message)s")

    # A window needs a display; Qt aborts the process, not an exception, when
    # there is none — so decide here (and say which branch was taken).
    _show, why = decide_show(args.headless, args.show)
    print("[bench] window %s — %s" % ("ON" if _show else "OFF (headless)", why))
    # Which code is this? A pasted log that cannot answer that cost a whole
    # round trip on 2026-10-12 (the Pi ran the OTHER branch's build, so it
    # still crashed on ord() and still reported status 69).
    from codeinfo import code_version, describe as _describe_code
    print("[bench] code %s" % _describe_code(code_version()))
    if args.save_frames:
        os.makedirs(args.save_frames, exist_ok=True)
        print("[bench] annotated frames -> %s (1/s)" % args.save_frames)

    cfg = load_cfg(args.config or os.path.join(_ROOT, "config.yaml"))
    from qr_filter import (QrFilter, load_presets, normalize_preset_name,
                           preset_names)
    qf = QrFilter.from_config(cfg)
    presets = load_presets(cfg)
    names = preset_names(cfg)
    if not names:
        names = list(presets.keys())

    # ---- preset selection -------------------------------------------------
    active = normalize_preset_name(args.preset)
    if args.preset and args.preset.lower() != "none" and active not in presets:
        print("[preset] %r unknown — available: %s — starting on 'day'"
              % (args.preset, ", ".join(names)))
        active = "day"
    preset = presets.get(active)
    if args.require_decode is not None:  # CLI flag wins over the preset
        qf.require_decode_for_cue = args.require_decode
    elif preset:
        qf.require_decode_for_cue = preset["require_decode_for_cue"]
    conf = args.conf if args.conf is not None else (preset or {}).get(
        "conf_thr", qf.min_conf)
    tiled = preset is not None and (preset.get("tile_grid", [3, 3]) != [1, 1])
    if args.no_tile:
        tiled = False
    if args.tiled:
        tiled = True

    # ---- detector ----------------------------------------------------------
    if args.classical:
        from detector import ClassicalDetector
        det = ClassicalDetector()
    elif args.hailo:
        det = build_detector(args, conf, None, kind="hailo")
    else:
        if args.pt:
            prefer = ["pt"]
        elif args.onnx or (args.model or "").lower().endswith(".onnx"):
            prefer = ["onnx"]
        else:
            prefer = ["onnx", "pt"]
        det = build_detector(args, conf, prefer)
    print("[detector] %s (conf floor %.2f)" % (det.name, conf))
    print("[filter]   min_conf=%.2f min_side=%dpx aspect=%.1f-%.1f "
          "require_decode_cue=%s"
          % (qf.min_conf, qf.min_side_px, qf.aspect_min, qf.aspect_max,
             qf.require_decode_for_cue))
    print("[preset]   %s (keys 1-%d: %s)"
          % (active or "none", len(names), ", ".join(names)))

    # ---- capture -----------------------------------------------------------
    import cv2
    from decoder import decode_frame
    from detector import detect_tiles
    src = args.source
    if str(src).lower().startswith("csi"):
        # the Pi's own CSI camera (UC-517 …), not an OpenCV device
        try:
            cap = CSISource(src, cfg)
        except Exception as e:
            print("ERROR: CSI source unavailable: %r" % e)
            return 2
    else:
        cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
        if not cap.isOpened():
            print("ERROR: cannot open source %r (need cv2 with V4L2/GStreamer, "
                  "or a file/URL). For the Pi's CSI camera use "
                  "--source csi:bottom" % src)
            return 2
    is_file = os.path.isfile(str(src))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    print("[camera] %dx%d source=%s tiled=%s" % (W, H, src, tiled))
    _boost_line = apply_preset_boost(cap, preset)
    if _boost_line:
        print(_boost_line)
    elif str(src).lower().startswith("csi") and (preset or {}).get("boost"):
        print("[boost] %s NOT applied (this camera has no apply_qr_profile)"
              % preset["boost"])

    hide_fakes = False
    n_frames = 0
    n_raw = 0
    n_filt = 0
    t_prev = time.time()
    fps = 0.0
    t_last_line = 0.0
    last_focus = [-1.0]    # sharpness, refreshed once a second (see below)
    t_run_start = time.time()
    n_line_prev = 0
    t_line_prev = t_run_start
    hinted_empty = False
    hinted_vision = False
    t_true_start = time.time()
    hint = ("[hint] no detector boxes at all after %d frames (%.0fs). Check, in "
            "this order: (1) a QR is actually IN VIEW and big enough (>= ~25px "
            "in the frame); (2) look at the saved JPEG (--save-frames) — the "
            "camera may be pointed at a wall or dark; (3) rerun with "
            "--conf 0.10 and read maxconf= in the line above; (4) --no-tile is "
            "~9x faster (one inference) at the cost of small-QR reach at 15 m")

    def hud(frame, raw_n, filt_n, payload_any, sharp=-1.0):
        lines = [
            "preset: %-10s conf %.2f  tile %s  req-decode: %s"
            % (active or "none", conf, "3x3" if tiled else "off",
               "ON " if qf.require_decode_for_cue else "off"),
            "filter: conf>=%.2f side>=%dpx aspect %.1f-%.1f | raw:%d filt:%d "
            "| %.1f fps | focus %.0f"
            % (qf.min_conf, qf.min_side_px, qf.aspect_min, qf.aspect_max,
               raw_n, filt_n, fps, sharp),
            "keys: f fakes  v req-decode  k/l conf  t tile  p 1-8 preset  q quit",
        ]
        y = 26
        for ln in lines:
            cv2.putText(frame, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += 20
        if payload_any:
            cv2.putText(frame, "QR CONFIRMED: %s" % payload_any, (8, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, "QR CONFIRMED: %s" % payload_any, (8, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

    try:
      while True:
        ok, frame = cap.read()
        if not ok:
            if is_file:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop the file
                ok, frame = cap.read()
            if not ok or frame is None:
                print("[source] end of stream/file — exiting")
                break
        n_frames += 1
        now = time.time()
        dt = max(1e-3, now - t_prev)
        t_prev = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

        # ---- detect (tiled for 15 m) ----
        try:
            if tiled and det.name != "classical":
                raw = detect_tiles(det, frame, rows=3, cols=3,
                                   overlap=0.25, iou_thr=args.iou)
            else:
                raw = det.detect(frame)
        except Exception as e:
            print("[detect] %r" % e)
            raw = []
        n_raw += len(raw)
        # ---- fake-QR filter (the grill killer) ----
        kept = qf.filter_boxes(raw)
        n_filt += len(kept)
        kept_ids = set(id(b) for b in kept)
        rejected = [b for b in raw if id(b) not in kept_ids]

        # ---- decode survivors (best conf first) ----
        payload_any = None
        payload_box = None
        for b in sorted(kept, key=lambda b: -b.conf)[:3]:
            p = decode_frame(frame, (b.x, b.y, b.w, b.h))
            if p:
                payload_any = p
                payload_box = b
                break
        if qf.require_decode_for_cue and payload_box is None:
            kept = []  # strict: nothing counts until a box actually decodes

        # ---- draw ----
        for b in rejected:
            if hide_fakes:
                continue
            reason = qf.should_keep(b)[1]
            x, y, w, h = int(b.x), int(b.y), int(b.w), int(b.h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 1)
            cv2.putText(frame, "fake %.2f (%s)" % (b.conf, reason),
                        (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 140, 255), 1, cv2.LINE_AA)
        for b in kept:
            x, y, w, h = int(b.x), int(b.y), int(b.w), int(b.h)
            if b is payload_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.putText(frame, "QR %.2f %s" % (b.conf, payload_any),
                            (x, max(16, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, "QR %.2f %s" % (b.conf, payload_any),
                            (x, max(16, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 255, 0), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.putText(frame, "qr %.2f" % b.conf, (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1,
                            cv2.LINE_AA)
        fsharp = last_focus[0]      # refreshed once a second, not per frame
        hud(frame, len(raw), len(kept), payload_any, fsharp)

        maxconf = max([b.conf for b in raw], default=0.0)
        # ONE line per second in BOTH modes. In window mode this is the record of
        # what the run actually saw: `nms_max=` is the HEF tensor's own best
        # score BELOW our confidence floor, the only way to tell "the model
        # detects nothing" from "everything is under the floor" — the
        # 2026-10-12 run printed raw_boxes=0 for 936 frames with no explanation.
        # `focus=` is what the lens ring gets adjusted against.
        if now - t_last_line >= 1.0:
            t_last_line = now
            # windowed fps: a cumulative average keeps DROPPING on screen
            # and looks like the tool is dying instead of steady at 1 fps
            win_dt = max(1e-3, now - t_line_prev)
            win_fps = (n_frames - n_line_prev) / win_dt
            n_line_prev, t_line_prev = n_frames, now
            fmean, fstd = frame_stats(frame)
            last_focus[0] = focus_score(frame)
            fsharp = last_focus[0]
            st = getattr(det, "last_stats", None) or {}
            peak = st.get("max_score", -1.0)
            using = getattr(det, "using", det.name)
            extra = "det=%s " % using + (
                "nms_max=%.2f filled=%d "
                % (peak, st.get("filled", 0))
                if isinstance(peak, (int, float)) and peak >= 0 else "nms=n/a ")
            print("[bench] frame=%d raw=%d filt=%d maxconf=%.2f %s"
                  "payload=%r frame_mean=%.0f frame_std=%.0f focus=%.0f "
                  "fps=%.1f (%.0f ms/frame)"
                  % (n_frames, n_raw, n_filt, maxconf, extra, payload_any,
                     fmean, fstd, fsharp, win_fps,
                     1000.0 / max(1e-3, win_fps)))
            if not hinted_vision:
                msg = vision_hint(fmean, fstd, fsharp)
                if msg:
                    hinted_vision = True
                    print(msg)
            if (not hinted_empty and n_raw == 0 and n_frames >= 10
                    and (now - t_true_start) >= 10.0):
                hinted_empty = True
                print(hint % (n_frames, now - t_true_start))
            if args.save_frames:
                path = os.path.join(args.save_frames, "bench_%05d.jpg"
                                    % n_frames)
                if cv2.imwrite(path, frame):
                    print("[bench] wrote %s" % path)

        if _show:
            cv2.imshow("QR bench (f/v/k/l/t/p/1-8/q)", _view(frame, args.view_width))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            elif k == ord("f"):
                hide_fakes = not hide_fakes
                print("[key] hide fakes: %s" % hide_fakes)
            elif k == ord("v"):
                qf.require_decode_for_cue = not qf.require_decode_for_cue
                print("[key] require_decode_for_cue: %s" % qf.require_decode_for_cue)
            elif k == ord("k"):
                conf = round(min(0.99, conf + 0.05), 2)
                det.set_thresholds(conf_thr=conf)
                print("[key] conf -> %.2f" % conf)
            elif k == ord("l"):
                conf = round(max(0.05, conf - 0.05), 2)
                det.set_thresholds(conf_thr=conf)
                print("[key] conf -> %.2f" % conf)
            elif k == ord("t"):
                tiled = not tiled
                print("[key] tiled: %s" % tiled)
            elif k == ord("p"):
                active = names[(names.index(active) + 1) % len(names)] \
                    if active in names else names[0]
                _apply_preset = presets.get(active)
                if _apply_preset:
                    conf = _apply_preset["conf_thr"]
                    det.set_thresholds(conf_thr=conf)
                    qf.require_decode_for_cue = _apply_preset["require_decode_for_cue"]
                    tiled = _apply_preset.get("tile_grid", [3, 3]) != [1, 1]
                print("[key] preset -> %s (conf %.2f, req-decode %s, tile %s)"
                      % (active, conf, qf.require_decode_for_cue, tiled))
                line = apply_preset_boost(cap, _apply_preset)
                if line:
                    print(line)
            elif preset_index(k, len(names) or 1) is not None:
                idx = preset_index(k, len(names))
                if idx is not None:
                    active = names[idx]
                    _apply_preset = presets.get(active)
                    if _apply_preset:
                        conf = _apply_preset["conf_thr"]
                        det.set_thresholds(conf_thr=conf)
                        qf.require_decode_for_cue = \
                            _apply_preset["require_decode_for_cue"]
                        tiled = _apply_preset.get("tile_grid", [3, 3]) != [1, 1]
                    print("[key] preset -> %s (conf %.2f, req-decode %s, tile %s)"
                          % (active, conf, qf.require_decode_for_cue, tiled))
                    line = apply_preset_boost(cap, _apply_preset)
                    if line:
                        print(line)

        if args.max_frames and n_frames >= args.max_frames:
            print("[bench] max-frames reached (%d)" % n_frames)
            break
    except KeyboardInterrupt:
        # Ctrl-C on a slow CPU run should print a summary, not 15 lines of
        # traceback through onnxruntime/pyzbar.
        print("[bench] interrupted at frame %d" % n_frames)
        if not det.name == "classical" and tiled and n_frames >= 3:
            print("[bench] if that felt stuck: tiled inference is 9 passes per "
                  "frame — ONNX on the Pi 5 CPU measures ~1 fps at 2028x1520. "
                  "Use --no-tile for a fast full-frame pass, or run the Hailo "
                  ".hef for 15 m real-time (training/compile_hef.md)")

    cap.release()
    if _show:
        cv2.destroyAllWindows()
    print("[done] frames=%d raw_boxes=%d filter_kept=%d (rejected %d)"
          % (n_frames, n_raw, n_filt, n_raw - n_filt))
    return 0


if __name__ == "__main__":
    sys.exit(main())
