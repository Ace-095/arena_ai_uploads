#!/usr/bin/env python3
"""Regression test: vehicle latch must ignore companion/GCS heartbeats.

Real failure (2026-09-14 bench): mission_pi's FC link heartbeats as
sysid 51 / type 18 (ONBOARD_CONTROLLER) on the same MP mirror. The old
latch (`type != GCS`) flapped veh_sysid 1 <-> 51, so param reads (and
fence/mode/plan — all addressed to veh_sysid, with replies filtered to
it) timed out. Stdlib only.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bridge"))

from mav_client import MavClient


class StubHub:
    def __init__(self):
        self.events = []

    def emit_threadsafe(self, event, data):
        self.events.append((event, data))


def hb(typ, custom_mode=0, base_mode=0, status=4):
    return {"type": typ, "custom_mode": custom_mode, "base_mode": base_mode,
            "system_status": status}


def main():
    hub = StubHub()
    mav = MavClient(hub, port=14551)  # constructed only: no threads/sockets

    mav._dispatch("HEARTBEAT", 1, hb(2))          # SITL quad
    assert mav.veh_sysid == 1 and mav.veh_type == 2, (mav.veh_sysid, mav.veh_type)
    assert mav.mode == "STABILIZE", mav.mode

    mav._dispatch("HEARTBEAT", 51, hb(18))        # mission_pi companion
    assert mav.veh_sysid == 1, mav.veh_sysid      # must NOT steal the latch
    assert mav.mode == "STABILIZE", mav.mode      # ...nor the mode display
    assert any(e == "log" and "sysid=51" in d.get("msg", "")
               for e, d in hub.events), hub.events

    mav._dispatch("HEARTBEAT", 255, hb(6))        # a GCS on the mirror
    assert mav.veh_sysid == 1, mav.veh_sysid

    # interleave storm: vehicle still wins, latch never flaps
    for _ in range(5):
        mav._dispatch("HEARTBEAT", 51, hb(18))
        mav._dispatch("HEARTBEAT", 1, hb(2))
    assert mav.veh_sysid == 1, mav.veh_sysid

    # ...but a genuine second VEHICLE can still take over (vehicle swap)
    mav._dispatch("HEARTBEAT", 2, hb(2))
    assert mav.veh_sysid == 2, mav.veh_sysid

    # ignore-note logged exactly once per (sysid, type)
    notes = [d for e, d in hub.events
             if e == "log" and "ignoring heartbeat" in d.get("msg", "")]
    assert len(notes) == 2, notes  # (51,18) + (255,6)

    print("VEHLATCH-TEST PASS")


if __name__ == "__main__":
    main()
