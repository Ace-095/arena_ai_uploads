#!/usr/bin/env python3
"""Param-path decider: does THIS link answer PARAM_REQUEST_READ at all?

  python tools/param_probe.py                                   # like FCLink (sysid 51, offboard HB)
  python tools/param_probe.py --sysid 252 --hb gcs              # GCS-identity hypothesis
  python tools/param_probe.py --params WPNAV_SPEED,FENCE_ENABLE

Run on the machine with pymavlink + the FC link (Linux SITL side).
Prints a heartbeat census (proves the wire is alive) then one line per
answer (or the honest timeout). If --sysid 252 --hb gcs answers while
the default doesn't, the FC gates params on GCS identity — say so and
we'll change FCLink's heartbeat, not the plumbing.
"""
import argparse
import sys
import threading
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="tcp:127.0.0.1:5762")
    ap.add_argument("--sysid", type=int, default=51)
    ap.add_argument("--hb", default="offboard", choices=["offboard", "gcs", "none"])
    ap.add_argument("--params", default="WP_SPD,WPNAV_SPEED,FENCE_ENABLE")
    ap.add_argument("--timeout", type=float, default=6.0)
    a = ap.parse_args()

    try:
        conn = mavutil.mavlink_connection(
            a.device, source_system=a.sysid,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    except OSError as e:
        sys.exit("SITL not listening on %s (%s) -- start Terminal B "
                 "(sim_vehicle) first, then re-run" % (a.device, e))
    hb_type = {"offboard": mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
               "gcs": mavutil.mavlink.MAV_TYPE_GCS}.get(a.hb)

    seen = []
    census = {}
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                m = conn.recv_match(blocking=True, timeout=0.3)
            except Exception:
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == "HEARTBEAT":
                try:
                    key = (m.get_srcSystem(), m.type)
                except Exception:
                    key = (-1, -1)
                census[key] = census.get(key, 0) + 1
            elif t in ("PARAM_VALUE", "PARAM_EXT_VALUE", "STATUSTEXT"):
                try:
                    if t == "STATUSTEXT":
                        s = bytes(m.text).decode("utf-8", "ignore").strip("\x00 ")
                    else:
                        pid = m.param_id
                        if isinstance(pid, bytes):
                            pid = pid.decode("utf-8", "ignore")
                        s = "%s=%s (type %s)" % (
                            str(pid).split("\x00")[0], m.param_value, m.param_type)
                except Exception as e:
                    s = "<unparseable: %r>" % e
                seen.append((time.time(), t, m.get_srcSystem(), s))

    rt = threading.Thread(target=reader, daemon=True)
    rt.start()
    try:
        conn.wait_heartbeat(timeout=5.0)
    except Exception:
        pass
    time.sleep(1.0)
    print("wire census (2 s): %s" % (census or "NOTHING HEARD — wrong device/link down"))
    if not census:
        return 2
    ts = conn.target_system or 1

    for name in [p.strip().upper()[:16] for p in a.params.split(",") if p.strip()]:
        if hb_type is not None:
            conn.mav.heartbeat_send(
                hb_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
        conn.mav.param_request_read_send(ts, 1, name.encode("utf-8"), -1)
        t0 = time.time()
        got = None
        while time.time() - t0 < a.timeout:
            for (_tt, _t, _s, s) in list(seen):
                if s.startswith(name + "="):
                    got = s
                    break
            if got:
                break
            time.sleep(0.05)
        print("%-14s <- %s" % (name, got if got else "TIMEOUT (no PARAM_VALUE in %.0fs)" % a.timeout))
    stop.set()
    rt.join(timeout=1.0)
    try:
        conn.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
