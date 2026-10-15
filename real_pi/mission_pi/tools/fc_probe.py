#!/usr/bin/env python3
"""Bench tool: find the FC over USB, print heartbeat/mode/position/home.

  python tools/fc_probe.py [--device /dev/ttyACM0] [--fence] [--plan]
  python tools/fc_probe.py --fence-check     # is this FC fence-ready for the UI?

--fence-check answers the question that matters before a mission: does THIS
autopilot support the polygon fence the UI uploads? It prints the firmware
version, whether MAVLink 2 is enabled on the USB port (the fence extension
mission type needs it), the FENCE_* parameters and the fence the FC currently
holds. A Pixhawk 2.4.8 clone with old firmware fails here, not in the air.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fc_link import FCLink  # noqa

FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX",
                "FENCE_MARGIN", "SERIAL0_PROTOCOL", "SERIAL1_PROTOCOL")


def fence_check(fc):
    """Report fence readiness honestly; never claims more than it read."""
    ok = True
    try:
        ver = fc.get_autopilot_version()
    except Exception as e:
        ver = None
        print("firmware    : UNKNOWN (%s)" % e)
    if ver:
        print("firmware    : %s" % ver)
    else:
        print("firmware    : not reported by the FC")

    # The capability bits are the FC's own statement of what it implements.
    # A 2.4.8 whose firmware predates mission fences answers fine here and
    # still cannot store one — that must fail the check, not pass it.
    try:
        m2_bit, fence_bit, cap_text = fc.fence_capability()
        print("capability  : %s" % cap_text)
        if fence_bit is False:
            print("  -> firmware does NOT implement the fence mission type: "
                  "update to ArduCopter >= 4.0 (4.3+ recommended). On a "
                  "Pixhawk 2.4.8 check the board is 2 MB flash; 1 MB boards "
                  "only run old Pixhawk1-1M builds.")
            ok = False
        elif fence_bit is None:
            # A real ArduPilot always answers this. Silence on the bench means
            # old/custom firmware or a half-broken link — either way the fence
            # question is unanswered, so the honest verdict is NOT ready.
            print("  -> the FC did not answer AUTOPILOT_VERSION at all, so "
                  "fence support cannot be confirmed: check the firmware "
                  "version in Mission Planner and re-run this probe")
            ok = False
        if m2_bit and not fc.mavlink2:
            print("  -> the firmware supports MAVLink 2 but the frames on this "
                  "link are v1: set SERIALx_PROTOCOL=2 for the port in use")
            ok = False
    except Exception as e:
        print("capability  : read FAILED (%s)" % e)

    print("mavlink     : %s" % ("MAVLink 2 (needed for the fence mission type)"
                                if fc.mavlink2 else
                                "MAVLink 1 — the FENCE mission type will NOT work; "
                                "set SERIAL0_PROTOCOL=2 and reboot"))
    if not fc.mavlink2:
        ok = False

    print("params      :")
    for name in FENCE_PARAMS:
        try:
            v = fc.get_param(name, timeout=2.0)
            print("  %-16s %s" % (name, v))
        except Exception as e:
            print("  %-16s ?  (%s)" % (name, e))

    try:
        en = fc.get_param("FENCE_ENABLE", timeout=2.0)
        typ = fc.get_param("FENCE_TYPE", timeout=2.0)
        if not en:
            print("  -> FENCE_ENABLE=0: the fence is NOT armed to act on a "
                  "breach (storing vertices is not enforcement)")
            ok = False
        if typ is not None and not (int(typ) & 4):
            print("  -> FENCE_TYPE=%s has no polygon bit (4): set FENCE_TYPE to "
                  "include 4 (e.g. 4 or 7)" % typ)
            ok = False
    except Exception:
        print("  -> could not read FENCE_ENABLE / FENCE_TYPE")

    try:
        f = fc.read_fence(timeout=5.0)
        print("fence       : %d vertices stored on the FC" % len(f))
        for lat, lon in f:
            print("  %.7f %.7f" % (lat, lon))
    except Exception as e:
        print("fence       : read FAILED (%s)" % e)
        ok = False

    print("verdict     : %s" % ("FENCE-READY (write test: UI 'UPLOAD FENCE → FC')"
                                if ok else
                                "NOT fence-ready yet — fix the lines marked above"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--fence", action="store_true")
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--fence-check", action="store_true",
                    help="firmware + MAVLink2 + FENCE_* + stored fence, i.e. "
                         "will the UI be able to set a geofence on this FC?")
    args = ap.parse_args()
    fc = FCLink()
    print("device:", fc.connect(device=args.device))
    import time
    time.sleep(1.0)
    print("mode:", fc.mode, "| armed:", fc.armed, "| link:", fc.link_ok())
    if args.fence_check:
        rc = fence_check(fc)
        fc.close()
        sys.exit(rc)
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
