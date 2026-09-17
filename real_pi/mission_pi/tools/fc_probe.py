#!/usr/bin/env python3
"""Bench tool: find the FC over USB, print heartbeat/mode/position/home.

  python tools/fc_probe.py [--device /dev/ttyACM0] [--fence] [--plan]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fc_link import FCLink  # noqa


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--fence", action="store_true")
    ap.add_argument("--plan", action="store_true")
    args = ap.parse_args()
    fc = FCLink()
    print("device:", fc.connect(device=args.device))
    import time
    time.sleep(1.0)
    print("mode:", fc.mode, "| armed:", fc.armed, "| link:", fc.link_ok())
    try:
        print("pos:", fc.get_position())
    except Exception as e:
        print("pos: %s" % e)
    try:
        print("home:", fc.get_home())
    except Exception as e:
        print("home: %s" % e)
    print("batt:", fc.get_battery())
    if args.fence:
        try:
            f = fc.read_fence()
            print("fence: %d verts" % len(f))
            for v in f:
                print("  %.7f %.7f" % v)
        except Exception as e:
            print("fence: %s" % e)
    if args.plan:
        try:
            for it in fc.read_plan():
                print("  #%d cmd=%d lat=%.6f lon=%.6f alt=%.1f" % (
                    it["seq"], it["command"], it["lat"], it["lon"], it["alt"]))
        except Exception as e:
            print("plan: %s" % e)
    fc.close()


if __name__ == "__main__":
    main()
