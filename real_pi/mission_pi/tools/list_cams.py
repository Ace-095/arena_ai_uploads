#!/usr/bin/env python3
"""real_pi camera auto-detect inspector.

Shows EXACTLY what this machine sees and how config.yaml would assign the
roles — the 30-second answer to "why is cam2 red?":

    python3 tools/list_cams.py                  # system + default config
    python3 tools/list_cams.py --config config.yaml

Exit 0 when every configured role gets a camera, 1 otherwise.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()

    cfg = {}
    if os.path.isfile(args.config):
        try:
            import yaml
            with open(args.config) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            print("config %s unreadable: %r" % (args.config, e))
    else:
        print("config %s not found — showing raw hardware only" % args.config)

    print("== hardware ==")
    from cameras import (list_cameras, list_video_devices,
                         v4l2_capture_devices, _HAVE_PICAM)
    print("[csi] picamera2: %s" % ("OK" if _HAVE_PICAM else
                                    "MISSING (apt install python3-picamera2)"))
    csi = list_cameras()
    if csi:
        for i, model, info in csi:
            print("  csi idx %d: %s  (sensor=%s)" % (
                i, model, info.get("Sensor", "?")))
    else:
        print("  csi: none found")
    usb = v4l2_capture_devices()
    print("[usb] V4L2 capture nodes:")
    if usb:
        for d in usb:
            tag = "CAPTURE" if d["is_capture"] else "skip"
            print("  %-12s %s  vid:pid=%s  %s" % (
                d["node"], tag, d["vidpid"], d["usb_desc"] or d["name"]))
    else:
        print("  none (all /dev/video* nodes: %s)"
              % (list_video_devices() or "no /dev/video* at all"))

    print("== assignment (per %s) ==" % args.config)
    try:
        from cameras import CameraRig
        rig = CameraRig(cfg)
        assigned = rig.detect()
        if not assigned:
            print("  NO CAMERAS ASSIGNED — check hardware + config")
            return 1
        for name, c in sorted(assigned.items()):
            print("  %s (%s): %s  device=%s  size=%s" % (
                name, c.facing, c.model, c.device_desc or c.index,
                c.size))
        single = (cfg.get("cameras", {}) or {}).get("single_cam_role", "bottom")
        if len(assigned) == 1:
            print("  note: single camera -> treated as %r (cameras."
                  "single_cam_role)" % single)
        missing = {"front", "bottom"} - \
            {c.facing for c in assigned.values()}
        if missing:
            print("  MISSING ROLE(S): %s" % ", ".join(sorted(missing)))
            print("  fix: plug the camera, or set cameras.%s.kind: usb "
                  "with device: /dev/videoN, or loosen match_name"
                  % sorted(missing)[0])
            return 1
        return 0
    except Exception as e:
        print("  assignment failed: %r" % e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
