#!/usr/bin/env python3
"""Stdlib tests for the external-mode guard (_guided_ok/_mode_tripped)."""
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import mission as _mission_mod  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


Mission = getattr(_mission_mod, "Mission", None)
check("Mission class found", Mission is not None)


class FakeHub:
    def __init__(self):
        self.msgs = []

    def push(self, channel, data):
        self.msgs.append((channel, data))
        return 1


class FakeFC:
    def __init__(self, mode="GUIDED"):
        self.mode = mode


class FakeFCParams(FakeFC):
    def __init__(self, params):
        super().__init__("GUIDED")
        self.params = dict(params)

    def get_param(self, name, timeout=4.0):
        if name not in self.params:
            raise Exception("get_param(%s) timeout" % name)
        return self.params[name]


if Mission is not None:
    m = Mission.__new__(Mission)
    m.fc = FakeFC("GUIDED")
    m.hub = FakeHub()
    m._lock = threading.Lock()
    m._t_phase = 0.0
    m.phase, m.detail = "TEST", ""
    m._speed_par = None
    m.payload = None

    check("GUIDED ok", m._guided_ok() is True)
    m.fc.mode = "RTL"
    check("1 stale read tolerated", m._guided_ok() is True)
    check("2 stale reads tolerated", m._guided_ok() is True)
    check("3rd consecutive trips", m._guided_ok() is False)
    m.fc.mode = "GUIDED"
    check("recovery resets counter", m._guided_ok() is True)
    m.fc.mode = "STABILIZE"
    m._guided_ok()
    check("counter survives across calls", getattr(m, "_offmode_n", -1) == 1)

    m.fc.mode = "RTL"
    r = m._mode_tripped()
    check("trip in RTL returns False + DONE",
          r is False and m.phase == "DONE", repr((r, m.phase)))
    check("trip logged loudly",
          any("left GUIDED" in (d.get("msg", "") + d.get("detail", ""))
              for c, d in m.hub.msgs), repr(m.hub.msgs)[:200])

    def snap_mission(params):
        mm = Mission.__new__(Mission)
        mm.fc = FakeFCParams(params)
        mm.hub = FakeHub()
        mm._lock = threading.Lock()
        mm._t_phase = 0.0
        mm.phase, mm.detail = "TEST", ""
        mm._speed_par = None
        return mm

    m47 = snap_mission({"WP_SPD": 6.0})
    m47._snapshot_speed()
    check("4.7+: WP_SPD preferred (scale 1)",
          m47._speed_par == ("WP_SPD", 1.0, 6.0), repr(m47._speed_par))

    mold = snap_mission({"WPNAV_SPEED": 600})
    mold._snapshot_speed()
    check("older: WPNAV_SPEED fallback (scale 100)",
          mold._speed_par == ("WPNAV_SPEED", 100.0, 600.0), repr(mold._speed_par))

    mboth = snap_mission({"WP_SPD": 5.5, "WPNAV_SPEED": 600})
    mboth._snapshot_speed()
    check("both present: WP_SPD wins",
          mboth._speed_par[0] == "WP_SPD", repr(mboth._speed_par))

    mnone = snap_mission({})
    mnone._snapshot_speed()
    check("neither: None + loud warn",
          mnone._speed_par is None
          and any("speed read failed" in d.get("msg", "")
                  for c, d in mnone.hub.msgs))

print("FAILS:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
