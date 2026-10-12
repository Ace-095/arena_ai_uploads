#!/usr/bin/env python3
"""A3 QR at 10-15 m: what the geometry actually allows, from the real config.

    python3 tools/fov_report.py                       # config.yaml, A3 at 10/15 m
    python3 tools/fov_report.py --qr-mm 200 --modules 25
    python3 tools/fov_report.py --exposure-ms 1,2,4,8 --speed 1,2,3
    python3 tools/fov_report.py --json out.json

Why this exists
---------------
"Can we read an A3 QR from 15 m?" is answered far too often with a confidence
number from a bench image. The physics is arithmetic and it is cheap to do
properly, so this tool does it from the values in `config.yaml` — and it labels
every input as MEASURED, CONFIGURED or VENDOR-QUOTED, because the difference
between those three is exactly what makes a range claim trustworthy or not.

The two questions are kept SEPARATE, because they fail independently:

  * **detection** needs the panel to be a distinguishable blob (a few tens of
    pixels is enough for a YOLO trained on the target);
  * **decoding** needs the QR's *modules* to be sampled — a 15 m panel that is
    34 px wide is detectable and contains ~1.2 px per module, which no barcode
    reader will recover. A box is not a payload.

Both are reported per altitude and per exposure, with the number that is
missing (the true lens FOV) called out rather than invented.

Nothing here talks to hardware, so it runs anywhere and is safe in CI.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml                       # noqa: E402

try:
    from geo import footprint_m
except Exception:                 # pragma: no cover
    footprint_m = None

#: A3, the board the mission hunts for.
A3_W_MM, A3_H_MM = 297.0, 420.0

#: The repo's own starting heuristic (docs/qr-resolution-and-command-tests.md):
#: "aim for roughly 3-4 captured pixels per module as a starting margin, then
#: validate on actual target material". NOT a guaranteed decode threshold.
PX_PER_MODULE_TARGET = 3.5

#: What the repo's OWN bench numbers actually demonstrate. models/README.md
#: reports a 29-module symbol (21 + 2x4 quiet zone) decoding at 38 px
#: equivalent (38/29 = 1.31 px/module) and "at the limit" at 34 px
#: (34/29 = 1.17). So the demonstrated floor is ~1.2-1.3 px/module with that
#: lens — far below the 3.5 px heuristic. Both are reported, because quoting
#: only the heuristic would make 10 m look impossible when it demonstrably is
#: not, and quoting only the empirical floor would hide how little margin
#: there is (a 1.2 px/module decode has no tolerance for blur or noise).
PX_PER_MODULE_EMPIRICAL = 1.3
#: Module count that empirical figure was derived with.
EMPIRICAL_MODULES = 29

#: Below this the panel is not a useful blob for a 640-input YOLO even with
#: 3x3 tiling of a 2028 px frame (a tile is ~256 px -> 640/256 = 2.5x).
MIN_DETECT_PX = 12

#: What the detector network actually receives (detector.input_size in config).
NET_INPUT_PX = 640


def _cfg(path):
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def _num(v, default):
    try:
        return float(v)
    except Exception:
        return float(default)


def compute(cfg, alt_m, qr_mm, modules, quiet, tilt_deg, sensor_px,
            speed_ms, exposure_s, cam_role="bottom",
            net_tile_cols=3, net_input_px=NET_INPUT_PX):
    """One geometry row. Pure arithmetic, no I/O, so it is unit-testable.

    cam_role: which camera config to read (bottom | front). For the FORWARD
    camera `alt_m` is the DISTANCE AHEAD to a panel that faces the camera
    square-on: the same 2*d*tan(fov/2) footprint applies, and there is no
    foreshortening because the panel is perpendicular to the optical axis.
    It is NOT a ground footprint — do not read a front row as "altitude".
    """
    cam_role = cam_role if cam_role in ("bottom", "front") else "bottom"
    cam = ((cfg.get("cameras") or {}).get(cam_role) or {})
    size = cam.get("size") or ([4608, 2592] if cam_role == "front" else [2028, 1520])
    img_w, img_h = int(size[0]), int(size[1])
    hfov = _num(cam.get("hfov_deg"), 75.0)
    vfov = cam.get("vfov_deg")

    if footprint_m is not None:
        foot_w, foot_h = footprint_m(alt_m, hfov, img_w, img_h,
                                     vfov_deg=_num(vfov, 60.0) if vfov else None)
    else:                                   # pragma: no cover
        foot_w = 2.0 * alt_m * math.tan(math.radians(hfov) / 2.0)
        foot_h = foot_w * img_h / float(img_w)

    # A tilted camera widens the footprint (1/cos) and foreshortens the target
    # (cos) — the two effects do not cancel.
    tilt = math.radians(max(0.0, min(85.0, tilt_deg)))
    foot_w_t = foot_w / max(0.2, math.cos(tilt))
    m_per_px = foot_w_t / float(img_w)
    px_per_m = 1.0 / m_per_px if m_per_px > 0 else 0.0

    # The QR symbol sees the tilt as a foreshortening along one axis.
    eff_mm = qr_mm * math.cos(tilt)
    qr_px = eff_mm / 1000.0 * px_per_m

    total_modules = modules + 2 * quiet
    px_per_module = qr_px / float(total_modules) if total_modules else 0.0

    blur_px = speed_ms * exposure_s * px_per_m
    # The detector does NOT see the frame at capture resolution: YOLO letterboxes
    # to `input_size` (640). A 4608 px front frame is therefore shrunk 7.2x
    # BEFORE the network runs, so a 70 px QR arrives as ~10 px. `detectable`
    # above only asks "is the QR a big enough blob in the frame"; this asks the
    # question that actually decides detection. Tiling (detector.tile_bottom /
    # tile_front) recovers it by feeding one tile at a time.
    tiles = max(1, int(net_tile_cols) or 1)
    net_px_full = qr_px * min(1.0, float(NET_INPUT_PX) / max(1.0, img_w))
    tile_w = img_w / float(tiles)
    net_px_tiled = qr_px * min(1.0, float(NET_INPUT_PX) / max(1.0, tile_w))
    row = {
        "alt_m": round(alt_m, 2),
        "footprint_m": [round(foot_w_t, 2), round(foot_h, 2)],
        "gsd_mm_per_px": round(m_per_px * 1000.0, 2),
        "px_per_m": round(px_per_m, 1),
        "qr_px": round(qr_px, 1),
        "px_per_module": round(px_per_module, 2),
        "motion_blur_px": round(blur_px, 2),
        "detectable": qr_px >= MIN_DETECT_PX,
        "net_input_px": round(net_px_full, 1),
        "net_input_px_tiled": round(net_px_tiled, 1),
        "detectable_at_input": net_px_full >= MIN_DETECT_PX,
        "detectable_at_input_tiled": net_px_tiled >= MIN_DETECT_PX,
        "decodable_at_target": px_per_module >= PX_PER_MODULE_TARGET,
        "decodable_empirical": px_per_module >= PX_PER_MODULE_EMPIRICAL,
        "blur_ok": blur_px <= 3.0,
    }
    return row


#: Levers, in the order a field team can actually pull them. Each is a pure
#: multiplier on px/module, so the table shows the SIZE of the win without any
#: claim about optics quality.
LEVERS = (
    ("as configured today", {}),
    ("QR fills the A3 LONG side (420 mm)", {"qr_mm": A3_H_MM}),
    ("+ shorter payload (v1, 21 modules)", {"qr_mm": A3_H_MM, "modules": 21}),
    ("+ narrower lens (50 deg HFOV, not 75)", {"qr_mm": A3_H_MM, "modules": 21,
                                               "hfov_deg": 50.0}),
    ("+ 30 deg mount tilt (foreshortened)", {"qr_mm": A3_H_MM, "modules": 21,
                                             "hfov_deg": 50.0, "tilt_deg": 30.0}),
)


def _levers(cfg, args, alt_default):
    """px/module for each realistic change, at the altitudes that matter."""
    print("=" * 78)
    print("LEVERS — px/module at the BOARD, from the configured geometry")
    print("=" * 78)
    print("  Each row is a change you can make before flying. The ordering is")
    print("  by cost to the field team, cheapest first.")
    print()
    alts = [float(x) for x in str(args.alts).split(",") if x.strip()] or [alt_default]
    head = "  %-42s" % "change" + "".join("%9s" % ("%.0fm" % a) for a in alts)
    print(head)
    print("  " + "-" * (len(head) - 2))
    for label, over in LEVERS:
        base = {"qr_mm": args.qr_mm, "modules": args.modules,
                "quiet": args.quiet, "tilt_deg": args.tilt}
        base.update(over)
        cells = []
        for a in alts:
            local = dict(cfg)
            if "hfov_deg" in base:
                local = json.loads(json.dumps(cfg))
                local.setdefault("cameras", {}).setdefault("bottom", {})
                local["cameras"]["bottom"]["hfov_deg"] = base["hfov_deg"]
                local["cameras"]["bottom"]["vfov_deg"] = base["hfov_deg"] * 0.8
            r = compute(local, a, base["qr_mm"], base["modules"], base["quiet"],
                        base["tilt_deg"], None, 0.0, 0.0)
            cells.append("%8.2f%s" % (r["px_per_module"],
                                      "*" if r["decodable_empirical"] else " "))
        print("  %-42s%s" % (label, "".join(cells)))
    print()
    print("  `*` = at or above the ~%.1f px/module floor this repo has actually"
          % PX_PER_MODULE_EMPIRICAL)
    print("        demonstrated decoding at. Blank means detection only.")
    print("  NOTE: a narrower lens WINS on pixels per metre but loses ground")
    print("        footprint — the search grid spacing re-computes and the")
    print("        mission takes longer. That trade is a mission decision, not a")
    print("        perception one, so it is reported and not decided here.")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--config", default=os.path.join(here, "config.yaml"))
    ap.add_argument("--alts", default="10,12,15", help="altitudes, metres")
    ap.add_argument("--qr-mm", type=float, default=A3_W_MM,
                    help="QR symbol size in mm INCLUDING the quiet zone "
                         "(default = the A3 short side, %.0f)" % A3_W_MM)
    ap.add_argument("--modules", type=int, default=25,
                    help="QR modules across, excluding quiet zone "
                         "(25 = version 2, 21 = version 1)")
    ap.add_argument("--quiet", type=int, default=4, help="quiet zone modules")
    ap.add_argument("--cam", default="bottom", choices=["bottom", "front"],
                    help="which camera role to compute for. 'front' answers "
                         "'how far AHEAD can the forward camera read a panel that "
                         "faces it' - there is no foreshortening in that case, and "
                         "the distance is NOT an altitude")
    ap.add_argument("--tilt", type=float, default=0.0,
                    help="camera tilt from nadir, degrees")
    ap.add_argument("--speed", default="2.0", help="ground speed m/s (list ok)")
    ap.add_argument("--exposure-ms", default="2.0", help="shutter ms (list ok)")
    ap.add_argument("--json", default="", help="also write the rows here")
    args = ap.parse_args()

    cfg = _cfg(args.config)
    alts = [float(x) for x in str(args.alts).split(",") if x.strip()]
    speeds = [float(x) for x in str(args.speed).split(",") if x.strip()]
    exps = [float(x) / 1000.0 for x in str(args.exposure_ms).split(",") if x.strip()]

    cam = ((cfg.get("cameras") or {}).get(args.cam) or {})
    size = cam.get("size") or ([4608, 2592] if args.cam == "front" else [2028, 1520])

    print("=" * 78)
    print("A3 QR geometry report [%s camera] — from %s" % (args.cam, args.config))
    print("=" * 78)
    print()
    print("INPUTS")
    print("-" * 78)
    print("  %-28s %-22s %s" % ("value", "used", "provenance"))
    print("  %-28s %-22s %s" % ("camera role", args.cam,
                                "CONFIGURED (cameras.%s)" % args.cam))
    print("  %-28s %-22s %s" % ("%s capture size" % args.cam,
                                "%dx%d" % (int(size[0]), int(size[1])),
                                "CONFIGURED (real sensor mode)"))
    print("  %-28s %-22s %s" % ("horizontal FOV", "%.1f deg" % _num(cam.get("hfov_deg"), 75.0),
                                "VENDOR-QUOTED — measure before trusting"))
    print("  %-28s %-22s %s" % ("vertical FOV", "%.1f deg" % _num(cam.get("vfov_deg"), 60.0),
                                "VENDOR-QUOTED — measure before trusting"))
    print("  %-28s %-22s %s" % ("QR symbol", "%.0f mm" % args.qr_mm,
                                "ASSUMED from A3 short side (297 mm)"))
    print("  %-28s %-22s %s" % ("QR modules",
                                "%d + %d quiet" % (args.modules, 2 * args.quiet),
                                "ASSUMED from payload length"))
    print("  %-28s %-22s %s" % ("camera tilt", "%.1f deg" % args.tilt,
                                "ASSUMED (nadir)" if not args.tilt else "ASSUMED"))
    print("  %-28s %-22s %s" % ("decode target",
                                "%.1f px/module" % PX_PER_MODULE_TARGET,
                                "repo heuristic, NOT a measured threshold"))
    print()

    rows = []
    for speed in speeds:
        for exp in exps:
            print("=" * 78)
            print("speed %.1f m/s   exposure %.1f ms" % (speed, exp * 1000.0))
            print("=" * 78)
            print("  %-6s %-12s %-9s %-8s %-8s %-8s %s"
                  % ("alt", "footprint m", "mm/px", "QR px", "px/mod",
                     "blur px", "verdict"))
            for alt in alts:
                r = compute(cfg, alt, args.qr_mm, args.modules, args.quiet,
                            args.tilt, None, speed, exp, cam_role=args.cam)
                r.update({"speed_ms": speed, "exposure_ms": exp * 1000.0})
                rows.append(r)
                verdict = []
                verdict.append("DETECT" if r["detectable"] else "no-detect")
                if r["decodable_at_target"]:
                    verdict.append("DECODE-ok")
                elif r["decodable_empirical"]:
                    verdict.append("DECODE-thin-margin")
                else:
                    verdict.append("no-decode")
                if not r["blur_ok"]:
                    verdict.append("BLURRY")
                print("  %-6.1f %-12s %-9.1f %-8.1f %-8.2f %-8.2f %s"
                      % (alt, "%.1fx%.1f" % tuple(r["footprint_m"]),
                         r["gsd_mm_per_px"], r["qr_px"], r["px_per_module"],
                         r["motion_blur_px"], " ".join(verdict)))
            print()
            print("  What the NETWORK actually receives (input_size=%d):" % NET_INPUT_PX)
            print("  %-6s %-14s %-14s %s" % ("alt", "full frame", "3x3 tiled", "note"))
            for r in rows[-len(alts):]:
                note = ""
                if not r["detectable_at_input"] and r["detectable_at_input_tiled"]:
                    note = "needs TILING (detector.tile_%s: true)" % args.cam
                elif not r["detectable_at_input_tiled"]:
                    note = "too small even tiled"
                print("  %-6.1f %-14s %-14s %s"
                      % (r["alt_m"], "%.1f px" % r["net_input_px"],
                         "%.1f px" % r["net_input_px_tiled"], note))
            print()
            print("  'detectable' above counts pixels IN THE FRAME. A %d px frame is"
                  % int(size[0]))
            print("  letterboxed to %d px before YOLO runs, so the frame-pixel"
                  % NET_INPUT_PX)
            print("  verdict can be optimistic — this table is the one that decides.")
            print()

    _levers(cfg, args, alts[0] if alts else 10.0)

    print("=" * 78)
    print("HOW TO READ THIS")
    print("=" * 78)
    print("""
  * `QR px` is the panel's size in the CAPTURED frame. Detection only needs a
    blob, so DETECT is the easy half.
  * `px/mod` is what the barcode reader gets. Two thresholds are reported and
    they disagree by design:
      - DECODE-ok            >= %.1f px/module — the repo's documented margin.
      - DECODE-thin-margin   >= %.1f px/module — what this repo's own bench
                             table actually DEMONSTRATED decoding (a %d-module
                             symbol read at 38 px). It works, with almost no
                             tolerance for blur, noise or tilt.
      - no-decode            below that: no amount of upscaling invents modules
                             the sensor never sampled.
    That is why the tiled 3x3 path exists (it raises the EFFECTIVE px/module at
    the NETWORK input, not at capture) and why the mission descends the
    approach stair for the final decode.
  * `blur px` is speed x exposure x px/m: an ESTIMATE. The real shutter time is
    chosen by the ISP and is not read back here. Keep it under ~3 px.
  * FOV is vendor-quoted in config.yaml. Measure it (§ tools/cam_probe.py and a
    tape measure at 10 m) and re-run this tool — every number below moves with
    it. Until then treat the whole table as CONDITIONAL.
""" % (PX_PER_MODULE_TARGET, PX_PER_MODULE_EMPIRICAL, EMPIRICAL_MODULES))

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"inputs": {"qr_mm": args.qr_mm, "modules": args.modules,
                                  "quiet": args.quiet, "tilt_deg": args.tilt,
                                  "config": args.config},
                       "rows": rows}, fh, indent=2)
        print("wrote %s (%d rows)" % (args.json, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
