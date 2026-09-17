#!/usr/bin/env python3
"""tests/test_qrlatch.py — the QR payload path on the Windows bridge.

Two bugs found on the bench (2026-09-14) are locked down here:

1. QR_RE was `QR[:\\s]+([0-9A-Za-z]{1,6})` — it silently dropped every real
   payload: "QR:MISSION-QR-001" has a dash and 13 chars, so the bridge saw the
   STATUSTEXT, matched nothing, and the UI never learned the payload. Widened
   to `[0-9A-Za-z][0-9A-Za-z._+-]{0,31}` (still anchored on a `QR:` prefix so
   ordinary chatter does not produce false payloads).

2. A QR caught on the MAVLink route existed only as a live SSE event: a UI
   opened AFTER the fly-past polled /api/mp/state and saw `qr: null`, so the
   operator concluded the hunt had found nothing. Hub now latches the first
   payload from any route and bridge_state() falls back to it.

Stdlib only. Exit 0 = all pass.
"""
import asyncio
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bridge"))
import mp_bridge as MB  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


# ---- 1. the regex: real payloads in, chatter out --------------------------
WANT = {
    "QR:MISSION-QR-001": "MISSION-QR-001",
    "QR: ABC123": "ABC123",                       # space after the colon
    "QR :PANEL-3": "PANEL-3",                      # space before the colon
    "qr:panel_07": None,                           # case-sensitive prefix
    "QR:PANEL.A-07_B+2": "PANEL.A-07_B+2",         # dots, dash, underscore, plus
    "STATUSTEXT QR:TARGET-9 ok": "TARGET-9",       # embedded in a longer line
    "QR:" + "Z" * 32: "Z" * 32,                    # 32-char ceiling
    "QR:X": "X",                                   # single char
    "the QR code was missed": None,                # prose: no colon -> no match
    "QR CONFIRMED via bottom: 'MISSION-QR-001'": None,   # the Pi's own log line
    "no QR payload decoded": None,                 # prose again
    "SQUADRON:QR:1": "1",                          # still needs the QR: prefix
    "QR:  ": None,                                 # empty payload
    "QR:-BAD": None,                               # must start alphanumeric
}
for line, want in WANT.items():
    m = MB.QR_RE.search(line)
    got = m.group(1) if m else None
    if want is None:
        check("no payload from %r" % line[:34], got is None, got)
    else:
        check("payload from %r" % line[:34], got == want,
              "got %r want %r" % (got, want))

m = MB.QR_RE.search("QR:" + "Z" * 40)
check("payload capped at 32 chars", m and len(m.group(1)) == 32,
      len(m.group(1)) if m else None)

# the MAVLink client uses the same pattern — they must not drift apart
try:
    import mav_client as MC
    check("mav_client.QR_RE agrees with mp_bridge.QR_RE",
          MC.QR_RE.pattern == MB.QR_RE.pattern,
          "%r vs %r" % (MC.QR_RE.pattern, MB.QR_RE.pattern))
except Exception as exc:                                    # noqa: BLE001
    print("INFO mav_client not importable here: %s" % exc)


# ---- 2. Hub latches the first QR from ANY route ---------------------------
def emit(hub, event, payload, source):
    asyncio.run(hub._emit_now(event, {"payload": payload, "source": source,
                                      "ts": time.time(),
                                      "line": "QR:%s" % payload}))


hub = MB.Hub()
check("hub starts with no QR", hub.qr is None)

emit(hub, "mp-qr", "MISSION-QR-001", "mavlink")
check("mavlink route latches the payload",
      hub.qr and hub.qr["payload"] == "MISSION-QR-001", hub.qr)
check("latch records the source", hub.qr["source"] == "mavlink", hub.qr)
check("latch records the line", hub.qr["line"] == "QR:MISSION-QR-001", hub.qr)

emit(hub, "mp-qr", "SECOND-PAYLOAD", "pi-ws")
check("first payload wins (later routes cannot overwrite it)",
      hub.qr["payload"] == "MISSION-QR-001", hub.qr)

# a Pi-relayed QR (channel "qr") must NOT latch: that is telemetry, not a
# bridge-side detection, and the Pi already owns its own confirmation logic
asyncio.run(hub._emit_now("qr", {"payload": "PI-SIDE", "confirmed": True}))
check("Pi 'qr' channel does not clobber the bridge latch",
      hub.qr["payload"] == "MISSION-QR-001", hub.qr)

# an empty/None payload must not latch (a missed decode is not a payload)
asyncio.run(hub._emit_now("mp-qr", {"payload": None, "source": "manual"}))
asyncio.run(hub._emit_now("mp-qr", {"source": "manual"}))
check("empty payloads do not latch", hub.qr["payload"] == "MISSION-QR-001",
      hub.qr)

# non-dict data must not blow up the hub (a stray emit should never kill SSE)
try:
    asyncio.run(hub._emit_now("mp-qr", "not-a-dict"))
    asyncio.run(hub._emit_now("mp-qr", None))
    check("malformed mp-qr data tolerated", True)
except Exception as exc:                                    # noqa: BLE001
    check("malformed mp-qr data tolerated", False, repr(exc))


# ---- 3. bridge_state() falls back to the latch ---------------------------
class _Tailer:
    watch = []


class _Tiles:
    @staticmethod
    def info():
        return {"available": False, "path": None}


class _Args:
    mock = False
    mav_port = 14551


class Stub:
    """Just enough of Server for bridge_state() to run (no sockets, no
    threads): the fallback `self.bridge["qr"] or self.hub.qr` is the code
    under test."""

    def __init__(self, bridge_qr, hub_qr):
        self.tailer = _Tailer()
        self.bridge = {"qr": bridge_qr}
        self.hub = type("H", (), {"qr": hub_qr})()
        self.args = _Args()
        self.mav = None
        self.tilestore = _Tiles()
        self.started = time.time()


LATCHED = {"payload": "MISSION-QR-001", "source": "mavlink",
           "ts": time.time(), "line": "QR:MISSION-QR-001"}
CONFIRMED = {"payload": "PI-CONFIRMED", "source": "pi", "streak": 3,
             "confirmed": True, "ts": time.time()}

st = MB.Server.bridge_state(Stub(None, LATCHED))
check("bridge_state exposes the latched QR when nothing was confirmed",
      st["qr"] and st["qr"]["payload"] == "MISSION-QR-001", st["qr"])

st2 = MB.Server.bridge_state(Stub(CONFIRMED, LATCHED))
check("a Pi-confirmed QR still wins over the latch",
      st2["qr"]["payload"] == "PI-CONFIRMED", st2["qr"])

st3 = MB.Server.bridge_state(Stub(None, None))
check("bridge_state reports qr=None when nothing was ever seen",
      st3["qr"] is None, st3["qr"])
check("bridge_state stays JSON-shaped",
      isinstance(st3, dict) and "mavlink" in st3 and "tiles" in st3)

print("FAILURES: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
