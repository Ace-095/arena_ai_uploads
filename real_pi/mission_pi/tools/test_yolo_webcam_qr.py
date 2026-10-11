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
  1-7 select preset by number                q / ESC quit

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

PRESET_KEYS = "1234567"  # key -> preset list position


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
        """(ok, BGR frame) — waits up to ~2 s for the camera thread."""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            try:
                frame, _ts, _n = self.cam.latest()
            except Exception:
                frame = None
            if frame is not None:
                return True, frame
            time.sleep(0.02)
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
                    help="qr_presets entry: day|far|dark|kabaddi|low|"
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
    ap.add_argument("--save-frames", default="",
                    help="dir to write annotated frames into (default off) — "
                         "the way to SEE a headless run; one JPEG/second")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (0 = run until key/file end)")
    args = ap.parse_args()

    # A window needs a display; Qt aborts the process, not an exception, when
    # there is none — so decide here (and say which branch was taken).
    _show, why = decide_show(args.headless, args.show)
    print("[bench] window %s — %s" % ("ON" if _show else "OFF (headless)", why))
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

    hide_fakes = False
    n_frames = 0
    n_raw = 0
    n_filt = 0
    t_prev = time.time()
    fps = 0.0
    t_last_line = 0.0
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

    def hud(frame, raw_n, filt_n, payload_any):
        lines = [
            "preset: %-10s conf %.2f  tile %s  req-decode: %s"
            % (active or "none", conf, "3x3" if tiled else "off",
               "ON " if qf.require_decode_for_cue else "off"),
            "filter: conf>=%.2f side>=%dpx aspect %.1f-%.1f | raw:%d filt:%d | %.1f fps"
            % (qf.min_conf, qf.min_side_px, qf.aspect_min, qf.aspect_max,
               raw_n, filt_n, fps),
            "keys: f fakes  v req-decode  k/l conf  t tile  p 1-7 preset  q quit",
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
        hud(frame, len(raw), len(kept), payload_any)

        maxconf = max([b.conf for b in raw], default=0.0)
        if not _show:
            if now - t_last_line >= 1.0:
                t_last_line = now
                # windowed fps: a cumulative average keeps DROPPING on screen
                # and looks like the tool is dying instead of steady at 1 fps
                win_dt = max(1e-3, now - t_line_prev)
                win_fps = (n_frames - n_line_prev) / win_dt
                n_line_prev, t_line_prev = n_frames, now
                fmean, fstd = frame_stats(frame)
                print("[bench] frame=%d raw=%d filt=%d maxconf=%.2f "
                      "payload=%r frame_mean=%.0f frame_std=%.0f fps=%.1f "
                      "(%.0f ms/frame)"
                      % (n_frames, n_raw, n_filt, maxconf, payload_any,
                         fmean, fstd, win_fps, 1000.0 / max(1e-3, win_fps)))
                if fmean >= 0 and not hinted_vision:
                    if fmean < 12:
                        hinted_vision = True
                        print("[hint] the frame is nearly BLACK (mean %.0f) — "
                              "lens cap on, no light, or exposure/gain wrong. "
                              "Nothing can be detected in a black frame."
                              % fmean)
                    elif fstd < 8:
                        hinted_vision = True
                        print("[hint] the frame has almost no CONTRAST "
                              "(std %.0f) — it is a blank surface (wall, "
                              "ceiling, out-of-focus distance). Point the "
                              "camera at the printed QR panel." % fstd)
                if (not hinted_empty and n_raw == 0 and n_frames >= 10
                        and (now - t_true_start) >= 10.0):
                    hinted_empty = True
                    print(hint % (n_frames, now - t_true_start))
                if args.save_frames:
                    path = os.path.join(args.save_frames, "bench_%05d.jpg"
                                        % n_frames)
                    if cv2.imwrite(path, frame):
                        print("[bench] wrote %s" % path)
        else:
            cv2.imshow("QR bench (f/v/k/l/t/p/1-7/q)", frame)
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
            elif k in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5"),
                       ord("6"), ord("7")):
                idx = ord(k) - ord("1")
                if idx < len(names):
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
