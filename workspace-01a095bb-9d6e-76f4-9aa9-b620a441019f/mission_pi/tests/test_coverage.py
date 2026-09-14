#!/usr/bin/env python3
"""Stdlib tests for coverage heatmap: geo.cover_cells + Mission._cover_tick."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import geo  # noqa
import mission as _mission_mod  # noqa (top imports are stdlib+geo+consensus)

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


olat, olon = 15.35, 75.12


def at(e, n):
    lat, lon = geo.enu_to_latlon(e, n, olat, olon)
    return {"lat": lat, "lon": lon}


# --- geo.cover_cells: deterministic around a cell centre (2.5, 2.5) ---
cplat, cplon = geo.enu_to_latlon(2.5, 2.5, olat, olon)
c0 = geo.cover_cells(cplat, cplon, (olat, olon), 5.0, 0.0)
check("r=0 -> home cell only", c0 == {(0, 0)}, repr(c0))
c75 = geo.cover_cells(cplat, cplon, (olat, olon), 5.0, 7.5)
check("r=7.5 -> 3x3", len(c75) == 9 and (0, 0) in c75, repr(sorted(c75)))
c11 = geo.cover_cells(cplat, cplon, (olat, olon), 5.0, 11.0)
check("r=11 -> 13, superset", len(c11) == 13 and c75 < c11, repr(sorted(c11)))
far = geo.cover_cells(*at(500, 500).values(), (olat, olon), 5.0, 5.0)
check("far point disjoint", far.isdisjoint(c75), repr(sorted(far)))

# --- Mission._cover_tick via bare instance (no FC/SITL needed) ---
Mission = getattr(_mission_mod, "Mission", None) \
    or getattr(_mission_mod, "MissionRunner", None) \
    or getattr(_mission_mod, "MissionController", None)
check("Mission class found", Mission is not None)


class FakeHub:
    def __init__(self):
        self.msgs = []

    def push(self, channel, data):
        self.msgs.append((channel, data))
        return 1


class FakeCam:
    hfov_deg = 66.0
    size = (2560, 1440)


if Mission is not None:
    m = Mission.__new__(Mission)
    m.home = (olat, olon)
    m.cov_cell_m = 5.0
    m.cov_every_s = 3600.0  # throttle: only forced pushes go out
    m._cov, m._cov_hot, m._cov_last_push = set(), set(), 0.0
    m.hub = FakeHub()
    cam = FakeCam()

    # footprint @15m: min(19.5, 11.0)/2 = 5.49 -> plus-shape = 5 cells
    m._cover_tick(cam, at(2.5, 2.5), 15.0)
    check("tick1 pushes once", len(m.hub.msgs) == 1)
    ch, d = m.hub.msgs[0]
    got = set(map(tuple, d["cells"]))
    check("tick1 plus-shape x5",
          ch == "coverage" and d["total"] == 5 and len(got) == 5
          and d["origin"] == {"lat": olat, "lon": olon}
          and d["cell_m"] == 5.0, repr(d)[:200])
    check("payload JSON-serializable",
          json.loads(json.dumps(d))["total"] == 5)
    first = set(map(tuple, d["hot"]))
    check("hot == fresh cells", first == got, repr(d["hot"]))

    m._cover_tick(cam, at(2.5, 2.5), 15.0)
    check("tick2 throttled (no push)", len(m.hub.msgs) == 1)

    m._cover_tick(cam, at(32.5, 2.5), 15.0)  # 30 m east: disjoint plus
    check("tick3 accumulates silently",
          len(m.hub.msgs) == 1 and len(m._cov) == 10, repr(sorted(m._cov)))

    m._cov_last_push = 0.0  # force cadence
    before = set(m._cov)
    m._cover_tick(cam, at(62.5, 2.5), 15.0)  # third spot: fresh again
    check("forced push carries total=15", len(m.hub.msgs) == 2
          and m.hub.msgs[1][1]["total"] == 15)
    hot2 = set(map(tuple, m.hub.msgs[1][1]["hot"]))
    check("hot2 == 10 cells since last push (ticks 3+4)",
          len(hot2) == 10 and hot2.isdisjoint(first)
          and hot2 | first == set(m._cov), repr(sorted(hot2)))

print("FAILS:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
