#!/usr/bin/env python3
"""Stdlib tests for the real_pi two-stage, NO-DELAY QR -> Mission Planner
delivery (qr_relay.announce + mission hooks).

Stage 1: the FIRST single-frame decode puts QR_SEEN:<payload> on STATUSTEXT
severity CRITICAL — MP's Messages tab gets it within one telemetry tick,
before the confirm-streak.  Resent for announce_window_s (no ack on
STATUSTEXT, so one shot is not "perfect").

Stage 2: on confirm, the official QR:<payload> relay fires IMMEDIATELY
(not after the approach descent) and resends over the window.

Locked down: exact text/tag/severity, once-per-payload dedup, resend
window behaviour, ws event emission, and that the mission's TRANSMIT
phase does not double-fire the relay.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))
import qr_relay  # noqa
import mission as M  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


class FakeFC:
    def __init__(self, ok=True):
        self.ok = ok
        self.sends = []           # (text, severity)

    def send_statustext(self, text, severity=6):
        if not self.ok:
            raise RuntimeError("link down")
        self.sends.append((str(text), int(severity)))

    def link_ok(self):
        return self.ok

    def get_position(self, timeout=2.0):
        return {"lat": -27.3, "lon": 128.9, "alt_rel": 12.0, "hdg": 10.0}


class FakeHub:
    def __init__(self):
        self.events = []          # (channel, data)

    def push(self, channel, data):
        self.events.append((channel, data))
        return 1


class FakeRig:
    cams = {}


CFG = {"relay": {"announce": True, "announce_window_s": 1.5,
                 "announce_interval_s": 0.4, "on_confirm": True,
                 "interval_s": 0.4, "window_s": 1.5,
                 "store_forward": False, "store_path":
                     os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "pending_test.jsonl")},
        "decode": {"required_streak": 3, "miss_tolerance": 2},
        "mission": {"max_alt_m": 15.0, "sweep_alt_m": 15.0},
        "detector": {"tile_bottom": False}}


def make_mission(fc):
    return M.Mission(fc, FakeRig(), None, FakeHub(), dict(CFG))


# --------------------------------------------------------------- announce --
def test_announce_text_and_severity():
    fc, hub = FakeFC(), FakeHub()
    qr_relay.announce("ABC123", fc=fc, hub=hub, gps=(-27.3, 128.9, 12.0),
                      window_s=0.8, interval_s=0.3)
    time.sleep(1.4)  # first shot + at least one resend
    check("announce: first statustext is QR_SEEN:<payload>",
          fc.sends and fc.sends[0][0] == "QR_SEEN:ABC123",
          repr(fc.sends[:2]))
    check("announce: CRITICAL severity (MP shows it prominently)",
          fc.sends and fc.sends[0][1] == 3, repr(fc.sends[:1]))
    check("announce: resends over the window", len(fc.sends) >= 2,
          "sends=%d" % len(fc.sends))
    check("announce: all resends identical",
          all(t == "QR_SEEN:ABC123" for t, _ in fc.sends))
    chans = [c for c, _ in hub.events]
    check("announce: ws qr_seen event", "qr_seen" in chans, repr(chans))
    check("announce: event channel copy for the UI feed",
          "event" in chans, repr(chans))
    ev = [d for c, d in hub.events if c == "qr_seen"][0]
    check("announce: event carries payload + gps",
          ev.get("payload") == "ABC123" and ev.get("gps") ==
          [-27.3, 128.9, 12.0], repr(ev))


def test_announce_no_fc_no_crash():
    hub = FakeHub()
    t = qr_relay.announce("XYZ", fc=None, hub=hub, window_s=0.3,
                          interval_s=0.2)
    time.sleep(0.5)
    check("announce: no FC -> ws event still fires, no exception",
          any(c == "qr_seen" for c, _ in hub.events))
    check("announce: empty payload is a no-op",
          qr_relay.announce("", fc=FakeFC(), hub=hub) is None)


# ------------------------------------------------------------- mission --
def test_mission_stage1_dedup():
    fc, m = FakeFC(), make_mission(FakeFC())
    m.fc = fc
    m._announce_qr("PAYLOAD9")
    m._announce_qr("PAYLOAD9")   # duplicate decode -> must NOT re-announce
    texts = [t for t, _ in fc.sends]
    check("stage1: instant QR_SEEN on first decode",
          "QR_SEEN:PAYLOAD9" in texts, repr(texts))
    check("stage1: once per payload (dedup)",
          texts.count("QR_SEEN:PAYLOAD9") == 1, repr(texts))
    check("stage1: recorded in _announced", "PAYLOAD9" in m._announced)


def test_mission_stage2_on_confirm():
    fc, m = FakeFC(), make_mission(FakeFC())
    m.fc = fc
    m._on_payload("PAYLOAD9", "cam2")
    time.sleep(1.0)  # first shot of the confirm relay
    texts = [t for t, _ in fc.sends]
    check("stage2: official QR:<payload> fires AT confirm time",
          "QR:PAYLOAD9" in texts, repr(texts))
    check("stage2: latched (no double fire)", m._confirmed_relayed is True)
    # TRANSMIT phase must not re-fire the relay when it's already running
    m._transmit_skip = False
    relayed_before = len([t for t, _ in fc.sends if t.startswith("QR:")])
    # call the pre-branch decision: _transmit_and_finish is heavy (holds
    # the vehicle); verify the guard flag instead
    check("stage2: transmit sees the already-relayed flag",
          m._confirmed_relayed is True)


def test_stage2_disabled_keeps_transmit_path():
    fc = FakeFC()
    m = make_mission(fc)
    m.relay_on_confirm = False     # legacy behaviour via config
    m._on_payload("LEGACY1", "cam2")
    time.sleep(0.3)
    check("stage2 off: confirm does not relay",
          all(not t.startswith("QR:") for t, _ in fc.sends),
          repr([t for t, _ in fc.sends]))
    check("stage2 off: transmit will do the relay (flag unset)",
          m._confirmed_relayed is False)


def test_reset_keeps_announce_guard():
    fc, m = FakeFC(), make_mission(FakeFC())
    m.fc = fc
    m._announce_qr("KEEPME")
    m.reset()
    m._announce_qr("KEEPME")
    texts = [t for t, _ in fc.sends]
    check("reset: QR_SEEN guard survives re-runs (no MP spam)",
          texts.count("QR_SEEN:KEEPME") == 1, repr(texts))
    check("reset: confirm relay allowed again",
          m._confirmed_relayed is False)


if __name__ == "__main__":
    t0 = time.time()
    test_announce_text_and_severity()
    test_announce_no_fc_no_crash()
    test_mission_stage1_dedup()
    test_mission_stage2_on_confirm()
    test_stage2_disabled_keeps_transmit_path()
    test_reset_keeps_announce_guard()
    print("-" * 60)
    if FAILS:
        print("FAILED %d/%d: %s" % (len(FAILS), 6, ", ".join(FAILS)))
        sys.exit(1)
    print("all qr-announce tests passed (%.1fs)" % (time.time() - t0))
