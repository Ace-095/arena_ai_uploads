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


if Mission is not None:
    m = Mission.__new__(Mission)
    m.fc = FakeFC("GUIDED")
    m.hub = FakeHub()
    m._lock = threading.Lock()
    m._t_phase = 0.0
    m.phase, m.detail = "TEST", ""
    m._wpnav_orig = None
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

print("FAILS:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
