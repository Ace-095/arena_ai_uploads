#!/usr/bin/env python3
"""Find the lens position that makes the ground sharp — then pin it.

A downward camera at a fixed altitude does NOT want continuous autofocus
hunting over low-texture ground: it wants ONE measured lens position, set at
startup. This tool sweeps the VCM through a list of positions, scores each
frame (variance of the Laplacian — the standard focus metric), prints a table
and tells you the exact `config.yaml` lines to paste.

Lens position is libcamera's **dioptres**: 0 = infinity, 1 = 1 m, 0.067 = 15 m.
The tool prints the equivalent distance, so you can aim the sweep at the
mission altitude instead of guessing.

USAGE (on the Pi, from mission_pi/)
  # wide first pass: infinity -> 50 cm (16 steps)
  python3 tools/focus_sweep.py --source csi:bottom --positions 0:2:0.125

  # fine pass around whatever won the first sweep
  python3 tools/focus_sweep.py --source csi:bottom --positions 0.02,0.04,0.06,0.08,0.10

  # explicit list, custom output dir, allow the motor to settle longer
  python3 tools/focus_sweep.py --positions 0,0.067,0.2 --settle 1.2 --snap-dir snaps

What it does NOT prove: that 15 m real ground is sharp — put the printed QR
panel on the ground at the mission altitude and run it; that is the honest
test.

Requires picamera2 (Pi) + opencv. Never touches the FC.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir))

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_positions(spec):
    """'0:2:0.125' (start:stop:step) or '0,0.05,0.2' or '0.1' -> [floats].

    Pure so it can be tested off-Pi: the sweep is only as good as this list.
    """
    s = str(spec or "").strip()
    if not s:
        raise ValueError("empty positions spec")
    if ":" in s:
        parts = s.split(":")
        if len(parts) != 3:
            raise ValueError("range form is start:stop:step, got %r" % spec)
        start, stop, step = (float(p) for p in parts)
        if step <= 0:
            raise ValueError("step must be > 0, got %r" % step)
        out, v, n = [], start, 0
        while v <= stop + 1e-9 and n < 1000:
            out.append(round(v, 6))
            v = start + step * (n + 1)
            n += 1
        return out
    return [round(float(p), 6) for p in s.split(",") if p.strip() != ""]


def describe_distance(pos):
    """Dioptres -> human distance. 0 (or ~0) is infinity."""
    try:
        p = float(pos)
    except (TypeError, ValueError):
        return "?"
    if p <= 1e-6:
        return "infinity"
    d = 1.0 / p
    return "%.2f m" % d if d < 1000 else "%.1f km" % (d / 1000.0)


def pick_best(samples):
    """samples: [(pos, score)] -> (best_pos, best_score) or (None, None)."""
    best = None
    for pos, score in samples or []:
        if score is None:
            continue
        if best is None or score > best[1]:
            best = (pos, score)
    return best if best else (None, None)


def focus_gate(af_algorithm, force=False):
    """May this sweep produce a meaningful curve? -> (proceed, lines).

    LensPosition is gated by the SAME `rpi.af` algorithm as autofocus —
    field-proven on the UC-517 (2026-10-11): libcamera logged
    "Could not set LENS_POSITION - no AF algorithm" for every position and
    all 17 scored the same, which reads like a broken lens when it is really a
    missing tuning file. Pure, so the decision is testable without hardware.
    """
    if af_algorithm is True or force:
        return True, []
    return False, [
        "[focus] REFUSING to sweep: the tuning in use has no rpi.af AF "
        "algorithm, so LensPosition cannot move the lens — every position "
        "would score the same and the result would be meaningless.",
        "        This is a TUNING problem, not necessarily a hardware one: "
        "install Arducam's imx477_af.json (next to config.yaml, in models/, "
        "or in /usr/share/libcamera/ipa/rpi/pisp/) and re-run.",
        "        --force sweeps anyway; it will only prove the lens does not "
        "move."]


def curve_verdict(scores):
    """(spread, relative, flat?) for a list of sharpness scores.

    A flat curve means either a lens that cannot move or a scene with nothing
    to focus on; a tiny absolute peak means the frame is nearly uniform (blank
    wall / covered lens). Both must be said out loud.
    """
    vals = [s for s in scores if s is not None]
    if not vals:
        return 0.0, 0.0, True
    spread = max(vals) - min(vals)
    peak = max(vals)
    rel = spread / peak if peak > 0 else 0.0
    return spread, rel, bool(rel < 0.05 or peak < 3.0)


def config_snippet(role, pos, score=None, model=""):
    """The exact lines to paste into config.yaml."""
    head = ("cameras.%s:" % role) if role else "cameras.<role>:"
    lines = [head,
             "  autofocus: false          # measured, not hunting",
             "  lens_position: %s" % (("%g" % pos) if pos is not None
                                     else "<n>")]
    if score is not None:
        lines.append("  # focus_sweep score %.0f%s" % (
            score, (" (%s)" % model) if model else ""))
    return "\n".join(lines)


def sharpness(frame_bgr, cv2mod, centre=0.5):
    """Variance of the Laplacian over the central crop — higher = sharper."""
    h, w = frame_bgr.shape[:2]
    ch, cw = int(h * centre), int(w * centre)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    crop = frame_bgr[y0:y0 + ch, x0:x0 + cw]
    gray = cv2mod.cvtColor(crop, cv2mod.COLOR_BGR2GRAY)
    return float(cv2mod.Laplacian(gray, cv2mod.CV_64F).var())


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="csi:bottom",
                    help="csi:<role|slot|index> (default csi:bottom) — the "
                         "camera whose focus you are setting")
    ap.add_argument("--positions", default="0:2:0.125",
                    help="start:stop:step (dioptres) or a comma list "
                         "(0 = infinity, 0.067 = 15 m). Default 0:2:0.125")
    ap.add_argument("--config",
                    default=os.path.join(HERE, os.pardir, "config.yaml"),
                    help="config.yaml the camera settings come from "
                         "(default: mission_pi/config.yaml)")
    ap.add_argument("--settle", type=float, default=0.7,
                    help="seconds to wait after moving the lens (default 0.7)")
    ap.add_argument("--snap-dir", default="snaps",
                    help="where to write focus_<pos>.jpg and focus_best.jpg")
    ap.add_argument("--centre", type=float, default=0.5,
                    help="fraction of the frame scored, centred (default 0.5)")
    ap.add_argument("--force", action="store_true",
                    help="sweep even when the tuning has no rpi.af algorithm "
                         "(the numbers will be flat — this proves the lens "
                         "does not move)")
    args = ap.parse_args()

    try:
        import yaml
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
    except Exception as e:
        print("config %s unreadable (%r) — defaults will be used"
              % (args.config, e))
        cfg = {}

    try:
        positions = parse_positions(args.positions)
    except ValueError as e:
        print("bad --positions: %s" % e)
        return 2
    if not positions:
        print("no positions to sweep")
        return 2

    try:
        import cv2
    except Exception:
        print("need opencv on the Pi: sudo apt install python3-opencv")
        return 2
    try:
        from picamera2 import Picamera2
    except Exception as e:
        print("need picamera2: %r" % e)
        return 2

    # Which camera, and with which tuning/size — the mission's own config.
    from cameras import CameraRig, Camera as PiCamera
    rig = CameraRig(cfg)
    cams = rig.detect() or {}
    if not cams:
        print("no CSI camera detected — run tools/list_cams.py first")
        return 1
    want = str(args.source).split(":", 1)[1].strip().lower() \
        if ":" in str(args.source) else ""
    cam = None
    if want.isdigit():
        cam = next((c for c in cams.values()
                    if str(getattr(c, "index", "")) == want), None)
    elif want in ("front", "bottom"):
        cam = next((c for c in cams.values()
                    if str(getattr(c, "facing", "")) == want), None)
    elif want in cams:
        cam = cams[want]
    if cam is None:
        cam = next((c for c in cams.values()
                    if str(getattr(c, "facing", "")) == "bottom"), None) \
            or next(iter(cams.values()))
    print("[camera] %s (%s) model=%s index=%s size=%s"
          % (cam.name, cam.facing, cam.model, cam.index, cam.size))

    tuning = None
    if isinstance(cam, PiCamera):
        tuning = cam._load_tuning()
    picam = (Picamera2(cam.index, tuning=tuning) if tuning is not None
             else Picamera2(cam.index))
    conf = {"size": tuple(cam.size), "format": "RGB888"}
    picam.configure(picam.create_video_configuration(main=conf))
    picam.start()
    time.sleep(1.0)

    try:
        from libcamera import controls as LC
    except Exception as e:
        print("libcamera controls unavailable: %r" % e)
        return 2
    # Does the tuning carry the AF algorithm? Without it, LensPosition is a
    # NO-OP — libcamera logs "Could not set LENS_POSITION - no AF algorithm"
    # and every position returns the same image, which looks like a lens that
    # cannot focus instead of a tuning file that cannot focus it.
    if isinstance(cam, PiCamera):
        try:
            cam._resolve_af_algorithm(tuning)
        except Exception:
            pass
    proceed, why = focus_gate(getattr(cam, "af_algorithm", None), args.force)
    if not proceed:
        for ln in why:
            print(ln)
        picam.close()
        return 3
    try:
        picam.set_controls({"AfMode": LC.AfModeEnum.Manual})
    except Exception as e:
        print("this camera rejected the manual-focus control (AfMode: %r)."
              % e)
        return 3

    os.makedirs(args.snap_dir, exist_ok=True)
    print("[sweep] %d positions, settle %.2fs, scoring the central %.0f%%"
          % (len(positions), args.settle, args.centre * 100))
    print("%-10s %-10s %-10s %s"
          % ("position", "distance", "sharpness", "frame"))
    samples = []
    for pos in positions:
        try:
            picam.set_controls({"LensPosition": float(pos)})
        except Exception as e:
            print("  %-8s set failed: %r" % (pos, e))
            samples.append((pos, None))
            continue
        time.sleep(max(0.0, args.settle))
        frame = picam.capture_array("main")
        if frame is None:
            samples.append((pos, None))
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        score = sharpness(frame, cv2, args.centre)
        samples.append((pos, score))
        mean = float(frame.mean())
        cv2.imwrite(os.path.join(args.snap_dir, "focus_%g.jpg" % pos), frame)
        print("%-10g %-10s %-10.1f brightness %.0f"
              % (pos, describe_distance(pos), score, mean))

    best_pos, best_score = pick_best(samples)
    print("-" * 46)
    if best_pos is None:
        print("no usable frames — is something covering the lens?")
        picam.close()
        return 1
    scores = [s for _, s in samples if s is not None]
    spread, rel, flat = curve_verdict(scores)
    print("BEST: position %g (%s), sharpness %.1f"
          % (best_pos, describe_distance(best_pos), best_score))
    if flat:
        print("")
        print("READ THIS BEFORE TRUSTING THOSE NUMBERS: the curve is FLAT "
              "(spread %.1f%%, peak %.1f). That happens for one of two "
              "reasons:" % (rel * 100, max(scores)))
        print("  * the lens did not move at all — the tuning has no rpi.af "
              "algorithm (libcamera warns 'Could not set LENS_POSITION - no AF "
              "algorithm'), or there is no VCM on the module; or")
        print("  * there was nothing to focus on — point the camera at the "
              "printed QR panel / high-contrast target at 1-2 m first.")
        print("Peak sharpness under ~3.0 means the frame is nearly uniform: a "
              "blank wall or a covered lens scores exactly like this.")
    best_file = os.path.join(args.snap_dir, "focus_%g.jpg" % best_pos)
    if os.path.isfile(best_file):
        print("best frame: %s  (compare it by eye too)" % best_file)
    print("")
    print("Paste into %s:" % args.config)
    print(config_snippet(getattr(cam, "facing", None), best_pos, best_score,
                         cam.model))
    print("")
    print("Then verify at altitude, not on the bench: put the printed QR panel "
          "on the ground at 15 m and re-check with qr_bench / the UI.")
    picam.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
