#!/usr/bin/env python3
"""Stdlib tests for store_forward.py + qr_relay proxies (run anywhere)."""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import store_forward as sf  # noqa
from qr_relay import publish_once, relay_qr  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


class FakeFC:
    def __init__(self, up=True):
        self.up = up
        self.sent = []

    def link_ok(self):
        return self.up

    def send_statustext(self, text, severity=6):
        if not self.up:
            raise IOError("link down")
        self.sent.append(text)


class FakeHub:
    def __init__(self, n=0):
        self.n_clients = n
        self.events = []

    def push(self, channel, data):
        if self.n_clients <= 0:
            return 0
        self.events.append((channel, data))
        return self.n_clients


tmpd = tempfile.mkdtemp()

# 1. buffer + persist + reload
st = sf.ResultStore(os.path.join(tmpd, "pending.jsonl"))
check("fresh store empty", st.pending() == [])
i1 = st.buffer("MISSION-QR-001", (15.1, 75.1, 12.0))
check("buffer returns id 1", i1 == 1)
pend = sf.ResultStore(os.path.join(tmpd, "pending.jsonl")).pending()
check("survives reload with gps", len(pend) == 1
      and pend[0]["code"] == "MISSION-QR-001" and pend[0]["lat"] == 15.1
      and pend[0]["alt"] == 12.0, repr(pend))

# 2. publish_once route proxies
check("both down -> (F,F)",
      publish_once("QR1", FakeFC(up=False), FakeHub(n=0)) == (False, False))
fc = FakeFC(up=True)
check("fc up only -> (T,F)",
      publish_once("QR1", fc, FakeHub(n=0)) == (True, False))
check("statustext text shape", fc.sent[-1].startswith("QR:"))
hub = FakeHub(n=1)
check("both up -> (T,T)", publish_once("QR1", fc, hub) == (True, True))
check("ws event carries payload+gpts",
      hub.events[-1][1]["payload"] == "QR1", repr(hub.events[-1]))
check("empty payload -> (F,F)",
      publish_once("  ", fc, hub) == (False, False))

# 3. flusher: down keeps, recovery drops
fc2, hub2 = FakeFC(up=False), FakeHub(n=0)
fwd = sf.StoreForward(fc2, hub2, path=os.path.join(tmpd, "p2.jsonl"), poll_s=60)
fwd.store.buffer("AAA", None)
check("flush while down keeps", fwd.flush_once() == (0, 1))
check("attempt noted", fwd.store.pending()[0]["attempts"] == 0)  # no link: no attempt
fc2.up = True
f, k = fwd.flush_once()
check("flush on recovery drops", (f, k) == (1, 0) and fwd.store.pending() == [])

# 4. relay_qr buffers only on total failure
st3 = sf.ResultStore(os.path.join(tmpd, "p3.jsonl"))
t = relay_qr("BBB", FakeFC(up=False), FakeHub(n=0),
             interval_s=999, window_s=0.01, store=st3, gps=(1, 2, 3))
t.join(timeout=5)
p3 = st3.pending()
check("relay buffers when both fail",
      len(p3) == 1 and p3[0]["code"] == "BBB" and p3[0]["lat"] == 1, repr(p3))
st4 = sf.ResultStore(os.path.join(tmpd, "p4.jsonl"))
t = relay_qr("CCC", FakeFC(up=True), FakeHub(n=0),
             interval_s=999, window_s=0.01, store=st4)
t.join(timeout=5)
check("relay skips store when a route ok", st4.pending() == [])

print("FAILS:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
