#!/usr/bin/env python3
"""Bench tool: list CSI cameras, grab one frame each, save test JPGs.

  python tools/cam_probe.py [--snap-dir snaps]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from cameras import CameraRig  # noqa


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snap-dir", default="snaps")
    ap.add_argument("--config", default="config.yaml",
                    help="config.yaml to probe WITH (default: config.yaml) — "
                         "snapshots then use the mission's real size, tuning "
                         "file and focus settings instead of defaults")
    ap.add_argument("--defaults", action="store_true",
                    help="ignore config.yaml and probe with built-in defaults")
    args = ap.parse_args()

    cfg = {}
    if not args.defaults:
        if os.path.isfile(args.config):
            try:
                import yaml
                with open(args.config) as f:
                    cfg = yaml.safe_load(f) or {}
                print("config: %s" % args.config)
            except Exception as e:
                print("config %s unreadable (%r) — using defaults"
                      % (args.config, e))
        else:
            print("config %s not found — using defaults" % args.config)
    rig = CameraRig(cfg)
    rig.detect()
    if not rig.cams:
        print("no cameras found (picamera2 + CSI ribbons seated?)")
        return 1
    rig.start_all()
    time.sleep(2.0)
    os.makedirs(args.snap_dir, exist_ok=True)
    for name, cam in rig.cams.items():
        frame, ts, n = cam.latest()
        print("%s: %s frames=%d age=%s" % (
            name, cam.status(), n,
            ("%.1fs" % (time.time() - ts)) if ts else "never"))
        jpg = cam.jpeg()
        if jpg:
            p = os.path.join(args.snap_dir, name + ".jpg")
            open(p, "wb").write(jpg)
            print("  wrote %s (%d bytes)" % (p, len(jpg)))
    rig.stop_all()
    return 0


if __name__ == "__main__":
    sys.exit(main())
