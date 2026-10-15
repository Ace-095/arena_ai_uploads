#!/usr/bin/env python3
"""Stdlib tests for Mission.reset() — re-running the hunt in one process.

Why it matters: tools/qr_bench.py re-runs the mission repeatedly, and main.py
restarts the FSM after a no-link FAILSAFE. If per-run state (payload, plan,
fence, coverage cells, consensus streak, the _stop/_abort events) survived, the
second run would either refuse to start or — worse — report the PREVIOUS run's
payload instantly, which in competition looks like a decode that never happened.

No hardware: fc/rig/detector are dummies and nothing is flown.
"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mission import Mission  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


class DummyFC:
    mode = "AUTO"
    armed = True
    device = "tcp:127.0.0.1:5762"

    def link_ok(self, max_hb_age=3.0):
        return False


class DummyRig:
    cams = {}

    def get(self, name):
        return None


class DummyHub:
    def __init__(self):
        self.sent = []

    def push(self, channel, data):
        self.sent.append((channel, data))


TMP = tempfile.mkdtemp(prefix="mission_reset_")
CFG = {
    "mission_id": "reset-test",
    "mission": {"sweep_alt_m": 12.0, "link_timeout_s": 1.0},
    "relay": {"store_path": os.path.join(TMP, "pending.jsonl"),
              "flush_poll_s": 0.2, "store_forward": False},
    "decode": {"required_streak": 3, "miss_tolerance": 1},
}

hub = DummyHub()
m = Mission(DummyFC(), DummyRig(), None, hub, CFG)
check("constructs without hardware", m.phase == "BOOT", m.phase)
check("store-forward is off in the test cfg", m.store_fwd.enabled is False)

# ---- dirty the run state the way a completed hunt would ------------------
m.payload = "MISSION-QR-001"
m.phase, m.detail = "DONE", "payload delivered"
m.home = (47.3977, 8.5456, 488.0)
m.plan = [{"cmd": 16, "lat": 47.4, "lon": 8.5, "alt": 15.0}] * 6
m.fence = [{"cmd": 5001, "lat": 47.41, "lon": 8.55}]
m.sprayer_seqs = [[3, 4]]
m.stats = {"frames": 1234, "cues": 9, "decodes": 3}
m._cue = ("bottom", 100, 100, 40, 40, time.time())
m._cue_hits = 4
m._advancing = True
m._search_t0 = time.time() - 90.0
m._offmode_n = 7
m._cov = {(0, 0), (1, 0), (2, 3)}
m._cov_hot = {(2, 3)}
m._blob_obs = [{"lat": 47.4, "lon": 8.5, "w": 0.4}] * 5
m._stop.set()
m._abort.set()
m._takeover.set()
m._set_phase("DONE", "payload delivered")
for i in range(5):                                   # a confirmed streak
    m._consensus.push("MISSION-QR-001") if hasattr(m._consensus, "push") else None
check("state is dirty before reset", m.payload and m.plan and m._cov
      and m._stop.is_set(), (m.payload, len(m.plan), len(m._cov)))

# ---- reset ---------------------------------------------------------------
t_before = m._t_phase
time.sleep(0.02)
m.reset()

check("phase back to BOOT", m.phase == "BOOT", m.phase)
check("detail says reset", m.detail == "reset", m.detail)
check("payload cleared (no stale answer)", m.payload is None, m.payload)
check("home cleared", m.home is None, m.home)
check("plan cleared", m.plan == [], m.plan)
check("fence cleared", m.fence == [], m.fence)
check("sprayer sequences cleared", m.sprayer_seqs == [], m.sprayer_seqs)
check("stats zeroed", m.stats == {"frames": 0, "cues": 0, "decodes": 0}, m.stats)
check("cue cleared", m._cue is None and m._cue_hits == 0, m._cue)
check("advancing flag cleared", m._advancing is False)
check("search clock cleared", m._search_t0 is None, m._search_t0)
check("off-mode counter cleared", m._offmode_n == 0, m._offmode_n)
check("coverage cleared", not m._cov and not m._cov_hot, (m._cov, m._cov_hot))
check("blob observations cleared", m._blob_obs == [], m._blob_obs)
check("phase timer restarted", m._t_phase > t_before, (t_before, m._t_phase))
check("_stop cleared so run() can start again", m._stop.is_set() is False)
check("_abort cleared", m._abort.is_set() is False)
check("_takeover cleared", m._takeover.is_set() is False)
check("worker list cleared", m._workers == [], m._workers)

# consensus must not carry a streak into the next run
cons = m._consensus
streak = None
for attr in ("streak", "count", "_streak", "hits", "_hits"):
    if hasattr(cons, attr):
        streak = getattr(cons, attr)
        break
check("consensus streak reset", not streak,
      "%s=%r" % (attr if streak is not None else "?", streak))

# ---- owned objects survive (reset is not a re-construction) --------------
check("fc kept", m.fc is not None and m.fc.device == "tcp:127.0.0.1:5762")
check("rig kept", m.rig is not None)
check("hub kept", m.hub is hub)
check("store-forward object kept", m.store_fwd is not None)
check("config kept", m.cfg is CFG and m.sweep_alt == 12.0, m.sweep_alt)

# ---- reset is idempotent and safe when nothing was dirty ----------------
m.reset()
m.reset()
check("double reset is harmless", m.phase == "BOOT" and m.payload is None)

# a fresh mission and a reset mission must look identical to the FSM
fresh = Mission(DummyFC(), DummyRig(), None, hub, CFG)
same = all(getattr(m, a, "<missing>") == getattr(fresh, a, "<missing>") for a in
           ("phase", "payload", "home", "plan", "fence", "sprayer_seqs",
            "stats", "_cue", "_cue_hits", "_advancing", "_offmode_n",
            "_cov", "_cov_hot", "_blob_obs"))
check("reset state == freshly constructed state", same)
check("no attribute exists only after reset()",
      all(hasattr(fresh, a) for a in ("_offmode_n", "_target_last",
                                      "_blob_last_push", "_search_t0")),
      [a for a in ("_offmode_n", "_target_last", "_blob_last_push")
       if not hasattr(fresh, a)])

# ---- the mission loop must not resurrect the payload after reset --------
# (regression guard for a background worker still holding the old answer)
m.payload = "STALE"
m._stop.set()
m.reset()
time.sleep(0.3)
check("nothing repopulates the payload after reset", m.payload is None, m.payload)

print("FAILS: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
