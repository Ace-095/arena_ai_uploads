#!/usr/bin/env python3
"""Stdlib tests for the supervised FC link (fc_link.FCLink watchdog).

The bug this locks down (found 2026-09-14 on the bench): when the FC
connection died — SITL restarted, USB replugged, laptop slept — mission_pi
kept a dead socket forever. pymavlink spun printing "EOF on TCP socket",
`link_ok()` stayed False, the mission sat in WAIT_TRIGGER until its timeout,
and nothing ever reconnected. Now a watchdog notices a stale vehicle
heartbeat, tears the link down and re-acquires it, reporting each step.

No pymavlink needed: `find_fc` is monkeypatched and the connection object is
a duck-typed stub.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import fc_link  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


class FakeConn:
    """What FCLink needs from a mavlink connection: close() + target ids."""

    def __init__(self, sysid=1):
        self.target_system = sysid
        self.target_component = 1
        self.closed = False
        self.mode_mapping = lambda: {"GUIDED": 4, "AUTO": 3, "STABILIZE": 0}

    def close(self):
        self.closed = True

    def mav(self):  # pragma: no cover - never called (threads are not started)
        raise AssertionError("watchdog test must not touch the wire")


def make_link(connected=True, hb_age=0.0):
    lk = fc_link.FCLink()
    lk.conn = FakeConn() if connected else None
    lk.device = "tcp:127.0.0.1:5762"
    lk.target_system = 1
    lk.last_hb = time.time() - hb_age
    return lk


# --- link_ok / link_state --------------------------------------------------
lk = make_link(hb_age=0.2)
check("link_ok with a fresh heartbeat", lk.link_ok(max_hb_age=3.0))
st = lk.link_state()
check("link_state reports the device", st["device"] == "tcp:127.0.0.1:5762", st)
check("link_state not down", st["down"] is False and st["reconnects"] == 0)
check("link_state hb_age sane", 0.0 <= st["hb_age_s"] < 3.0, st)

dead = make_link(hb_age=30.0)
check("link_ok false on a stale heartbeat", not dead.link_ok(max_hb_age=3.0))
check("link_state hb_age shows the stall", dead.link_state()["hb_age_s"] >= 29.0,
      dead.link_state())

# --- watchdog: loss -> failed retries -> recovery --------------------------
events = []
lk = make_link(hb_age=0.1)
lk._on_event = lambda kind, detail: events.append((kind, detail))

attempts = {"n": 0}
orig_find = fc_link.find_fc


def flaky_find_fc(bauds=None, hb_timeout=3.0, device=None):
    """First two reconnect attempts fail (SITL still down), third succeeds."""
    attempts["n"] += 1
    if attempts["n"] < 3:
        raise RuntimeError("no FC heartbeat; tried: %s@None: [Errno 111] "
                           "Connection refused" % device)
    return FakeConn(sysid=1), device


fc_link.find_fc = flaky_find_fc
try:
    # start_watchdog clamps to dead_s>=3s / retry_s>=1s, so size the waits
    # against the clamped values rather than hoping for fast ones.
    lk.start_watchdog(dead_s=3.0, retry_s=1.0)
    check("clamps are applied (dead_s>=3, retry_s>=1)",
          lk._wd_dead_s >= 3.0 and lk._wd_retry_s >= 1.0,
          (lk._wd_dead_s, lk._wd_retry_s))
    check("watchdog thread running", lk.link_state()["watchdog"] is True)
    # go quiet: no vehicle heartbeats any more
    lk.last_hb = time.time() - 60.0
    t_end = time.time() + 12.0
    while time.time() < t_end and lk.reconnects == 0:
        time.sleep(0.05)
    kinds = [k for k, _d in events]
    check("link_lost reported", "link_lost" in kinds, kinds)
    check("link_retry reported while the FC is gone", "link_retry" in kinds, kinds)
    check("link_restored after a successful reconnect", "link_restored" in kinds,
          kinds)
    check("reconnect counted", lk.reconnects == 1, lk.reconnects)
    check("link is ok again", lk.link_ok(max_hb_age=3.0))
    check("link_down cleared", lk.link_down is False)
    check("fresh heartbeat after reconnect",
          (time.time() - lk.last_hb) < 3.0, lk.last_hb)
    check("device kept across the reconnect", lk.device == "tcp:127.0.0.1:5762")
    check("find_fc was retried (>=3 attempts)", attempts["n"] >= 3, attempts)
    lk.stop_watchdog()
    check("watchdog stopped", lk.link_state()["watchdog"] is False)
    # stopping must be idempotent and must not resurrect the thread
    lk.stop_watchdog()
    check("stop_watchdog idempotent", lk._watchdog is None)
finally:
    fc_link.find_fc = orig_find
    lk.stop_watchdog()

# --- a never-returning FC must not kill the watchdog -----------------------
events2 = []
lk2 = make_link(hb_age=0.1)
lk2._on_event = lambda kind, detail: events2.append((kind, detail))


def dead_find_fc(bauds=None, hb_timeout=3.0, device=None):
    raise RuntimeError("no FC heartbeat; tried: %s@None: Connection refused"
                       % device)


fc_link.find_fc = dead_find_fc
try:
    lk2.start_watchdog(dead_s=3.0, retry_s=1.0)
    lk2.last_hb = time.time() - 30.0
    # one link_lost, then a retry per polling interval — give it 3 intervals
    t_end = time.time() + 8.0
    while time.time() < t_end and [k for k, _ in events2].count("link_retry") < 2:
        time.sleep(0.1)
    kinds2 = [k for k, _d in events2]
    check("permanent loss: link_lost + repeated link_retry",
          "link_lost" in kinds2 and kinds2.count("link_retry") >= 2, kinds2)
    check("permanent loss: no reconnect counted", lk2.reconnects == 0)
    check("permanent loss: link_down stays true", lk2.link_down is True)
    check("permanent loss: conn torn down (None) while retrying",
          lk2.conn is None or lk2.conn.closed, lk2.conn)
    lk2.stop_watchdog()
finally:
    fc_link.find_fc = orig_find
    lk2.stop_watchdog()

# --- reconnect() is callable directly (tools/bench use) --------------------
lk3 = make_link(hb_age=0.1)
calls = {"n": 0}


def good_find_fc(bauds=None, hb_timeout=3.0, device=None):
    calls["n"] += 1
    return FakeConn(sysid=7), device


fc_link.find_fc = good_find_fc
try:
    old_conn = lk3.conn
    dev = lk3.reconnect()
    check("reconnect returns the device", dev == "tcp:127.0.0.1:5762", dev)
    check("reconnect closed the old connection", old_conn.closed is True)
    check("reconnect adopted the new sysid", lk3.target_system == 7,
          lk3.target_system)
    check("reconnect counted", lk3.reconnects == 1)
finally:
    fc_link.find_fc = orig_find

print("FAILS: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
