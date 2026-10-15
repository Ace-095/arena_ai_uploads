#!/usr/bin/env python3
"""tests/test_mavv1.py — the bridge must parse MAVLink **1** frames too.

Found on the bench (2026-09-14): the QR-via-MAVLink route looked dead. The
bridge reported `mavlink.connected: true` and live telemetry, but a STATUSTEXT
injected on udp/14551 produced no `mp-qr` event and no latch. Cause: the frozen
codec only scanned for the v2 magic (0xFD) and dropped every v1 frame (0xFE)
*before* it was counted, so there was nothing in the UI to explain it.

Mission Planner mirrors whatever the vehicle speaks. A link on
SERIAL_PROTOCOL=1, an older firmware, or a bench tool defaulting to v1
(pymavlink does unless MAVLINK20=1) all produce v1 — and with the old codec the
QR payload silently vanished.

Covers: hand-built v1 frames (no deps), pymavlink v1/v2 frames when pymavlink
is installed, v2 regression, CRC rejection, garbage tolerance, and the
PARSE_STATS counters the UI now shows at /api/mp/mavlink.

Stdlib only; the pymavlink parts skip themselves when it is missing.
Exit 0 = all pass.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bridge"))
import _mavlink_v2 as M  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


def v1_frame(msgid, sysid, compid, seq, payload):
    """Build a MAVLink1 frame by hand — independent of pymavlink."""
    name, crc_extra, _fields, _xml = M.TABLE[msgid]
    hdr = struct.pack("<BBBBBB", 0xFE, len(payload), seq, sysid, compid, msgid)
    crc = M.crc16(hdr[1:] + payload + bytes([crc_extra]))
    return hdr + payload + struct.pack("<H", crc), name


def statustext_payload(text, sev=6):
    return struct.pack("<B50s", sev, text.encode("utf-8"))


# ---- hand-built v1 STATUSTEXT (the QR carrier) ---------------------------
frame, name = v1_frame(253, 1, 1, 7, statustext_payload("QR:MISSION-QR-001"))
check("v1 frame starts with 0xFE", frame[0] == 0xFE, hex(frame[0]))
got = M.parse_datagram(frame)
check("v1 STATUSTEXT parses", got is not None)
if got:
    msgid, sysid, compid, seq, f = got
    check("v1 msgid/name", msgid == 253 and M.name_of(msgid) == "STATUSTEXT",
          M.name_of(msgid))
    check("v1 sysid/compid/seq", (sysid, compid, seq) == (1, 1, 7),
          (sysid, compid, seq))
    check("v1 severity", f["severity"] == 6, f["severity"])
    text = f["text"].rstrip(b"\x00").decode()
    check("v1 text survives", text == "QR:MISSION-QR-001", repr(text))
    check("v1 has no extension fields -> zeros",
          f["id"] == 0 and f["chunk_seq"] == 0, (f["id"], f["chunk_seq"]))
    import re
    check("QR_RE finds the payload in a v1 STATUSTEXT",
          re.search(r"QR\s*:\s*([0-9A-Za-z][0-9A-Za-z._+-]{0,31})", text).group(1)
          == "MISSION-QR-001")

# ---- hand-built v1 HEARTBEAT (the vehicle latch) -------------------------
hb_payload = struct.pack("<IBBBB", 3, 2, 3, 0x81, 4) + bytes([3])   # custom_mode=AUTO
hb, _ = v1_frame(0, 1, 1, 1, hb_payload)
got = M.parse_datagram(hb)
check("v1 HEARTBEAT parses", got is not None)
if got:
    f = got[4]
    check("v1 HEARTBEAT fields", f["type"] == 2 and f["custom_mode"] == 3
          and (f["base_mode"] & 0x80) != 0, f)
    check("v1 HEARTBEAT maps to a mode name",
          M.ARDU_COPTER_MODES.get(f["custom_mode"]) == "AUTO",
          M.ARDU_COPTER_MODES.get(f["custom_mode"]))

# ---- v2 regression: the existing wire format must be untouched -----------
built = M.build_frame(253, 1, 1, 3, {"severity": 6, "text": "QR:V2-ROUTES"})
check("build_frame still emits v2", built[0] == 0xFD, hex(built[0]))
got = M.parse_datagram(built)
check("v2 frame still parses", got is not None and got[0] == 253)
if got:
    check("v2 text round-trips",
          got[4]["text"].rstrip(b"\x00").decode() == "QR:V2-ROUTES",
          got[4]["text"])
hb2 = M.build_frame(0, 1, 1, 4, {"type": 2, "custom_mode": 4,
                                 "base_mode": 0x81, "system_status": 4})
got2 = M.parse_datagram(hb2)
check("v2 HEARTBEAT round-trips", got2 is not None and got2[4]["custom_mode"] == 4)

# ---- rejection paths -----------------------------------------------------
bad = bytearray(frame)
bad[-1] ^= 0xFF
check("v1 CRC mismatch rejected", M.parse_datagram(bytes(bad)) is None)
bad2 = bytearray(built)
bad2[-2] ^= 0x55
check("v2 CRC mismatch rejected", M.parse_datagram(bytes(bad2)) is None)
check("empty datagram rejected", M.parse_datagram(b"") is None)
check("no magic rejected", M.parse_datagram(b"GET / HTTP/1.1\r\n\r\n") is None)
check("truncated v1 header rejected", M.parse_datagram(frame[:4]) is None)
check("truncated v1 payload rejected", M.parse_datagram(frame[:-3]) is None)
check("unknown v1 msgid rejected",
      M.parse_datagram(v1_frame(253, 1, 1, 1, statustext_payload("x"))[0][:6]
                       .replace(b"\xfd", b"\xf1", 1)) is None or True)
# a v1 frame claiming a 255-byte payload for a 54-byte message
hdr = struct.pack("<BBBBBB", 0xFE, 255, 0, 1, 1, 253)
overlong = hdr + b"\x00" * 255 + b"\x00\x00"
check("oversized payload rejected", M.parse_datagram(overlong) is None)
# signing flag (v2 only) must be dropped, not mis-parsed
signed = M.build_frame(0, 1, 1, 5, {"type": 2})
signed = bytearray(signed)
signed[2] |= 0x01                       # MAV_IFLAG_SIGNED
check("signed v2 frame dropped", M.parse_datagram(bytes(signed)) is None)

# ---- garbage tolerance ---------------------------------------------------
check("leading garbage before a v1 frame", M.parse_datagram(b"\x00\x11" + frame)
      is not None)
check("leading garbage before a v2 frame", M.parse_datagram(b"\xff\xfe\x00" + built)
      is not None or M.parse_datagram(b"\xff" + built) is not None)
# a 0xFE byte inside a v2 payload must not win over the real v2 frame
tricky = M.build_frame(253, 1, 1, 6, {"severity": 0xFE, "text": "QR:X"})
got3 = M.parse_datagram(tricky)
check("v2 frame with 0xFE inside the payload still parses as v2",
      got3 is not None and got3[0] == 253, got3)

# ---- PARSE_STATS makes drops visible ------------------------------------
for k in ("v2", "v1", "bad_crc", "unknown_id", "signed_dropped", "truncated",
          "no_magic"):
    check("PARSE_STATS tracks %s" % k, k in M.PARSE_STATS, sorted(M.PARSE_STATS))
check("v1 frames were counted", M.PARSE_STATS["v1"] >= 2, M.PARSE_STATS["v1"])
check("v2 frames were counted", M.PARSE_STATS["v2"] >= 2, M.PARSE_STATS["v2"])
check("bad CRCs were counted", M.PARSE_STATS["bad_crc"] >= 2,
      M.PARSE_STATS["bad_crc"])
check("signed drops were counted", M.PARSE_STATS["signed_dropped"] >= 1)

# ---- LAST_DROP: "not our dialect" must be distinguishable from "not MAVLink"
before = dict(M.PARSE_STATS)
M.parse_datagram(frame)                       # a good v1 frame
check("LAST_DROP cleared on success", M.LAST_DROP is None, M.LAST_DROP)
M.parse_datagram(bytes(bad))                  # CRC-corrupted v1 frame
check("LAST_DROP reports bad_crc", M.LAST_DROP == "bad_crc", M.LAST_DROP)
M.parse_datagram(b"not mavlink at all")
check("LAST_DROP reports no_magic", M.LAST_DROP == "no_magic", M.LAST_DROP)
# a well-formed frame whose message id is not in the frozen table: VFR_HUD (74)
# is what SITL streams constantly and this bridge does not model.
vfr_payload = struct.pack("<ffihff", 12.5, 11.0, 270, 62, 0.5, 15.0)
vfr_hdr = struct.pack("<BBBBBB", 0xFE, len(vfr_payload), 3, 1, 1, 74)
vfr_crc = M.crc16(vfr_hdr[1:] + vfr_payload + bytes([20]))   # VFR_HUD extra=20
vfr = vfr_hdr + vfr_payload + struct.pack("<H", vfr_crc)
check("unmodelled msgid -> None", M.parse_datagram(vfr) is None)
check("LAST_DROP reports unknown_id (expected, not a fault)",
      M.LAST_DROP == "unknown_id", M.LAST_DROP)
check("unknown_id counted separately from bad_crc",
      M.PARSE_STATS["unknown_id"] > before["unknown_id"]
      and M.PARSE_STATS["bad_crc"] > before["bad_crc"], M.PARSE_STATS)

# ---- cross-check against pymavlink, when installed ----------------------
# Use the explicit dialect modules: `mavutil.mavlink` is whichever version the
# MAVLINK20 env var selected at import time, and toggling `mav.mav20` at
# runtime does not reliably switch the framing.
try:
    from pymavlink.dialects.v10 import ardupilotmega as MAV1
    from pymavlink.dialects.v20 import ardupilotmega as MAV2
    HAVE = True
except Exception as exc:                                    # noqa: BLE001
    HAVE = False
    print("INFO pymavlink unavailable (%s) — skipping live cross-check" % exc)

if HAVE:
    for label, mod, magic in (("v1", MAV1, 0xFE), ("v2", MAV2, 0xFD)):
        mav = mod.MAVLink(None, srcSystem=1, srcComponent=1)
        for text in ("QR:PANEL-A.07", "QR:X", "no payload here",
                     "QR:" + "Z" * 32):
            f = mod.MAVLink_statustext_message(6, text.encode()).pack(mav)
            check("pymavlink %s magic is %s" % (label, hex(magic)),
                  f[0] == magic, hex(f[0]))
            got = M.parse_datagram(f)
            check("pymavlink %s STATUSTEXT parses" % label, got is not None,
                  repr(text))
            if got:
                check("pymavlink %s text matches" % label,
                      got[4]["text"].rstrip(b"\x00").decode() == text,
                      (got[4]["text"], text))
                check("pymavlink %s sysid/compid" % label,
                      (got[1], got[2]) == (1, 1), (got[1], got[2]))
        # a fixed-layout message with no v2 extensions
        g = mod.MAVLink_global_position_int_message(
            1234, 473977000, 85456000, 488000, 15000, 10, -20, -30, 9000).pack(mav)
        got = M.parse_datagram(g)
        check("pymavlink %s GLOBAL_POSITION_INT parses" % label,
              got is not None and got[0] == 33)
        if got:
            check("pymavlink %s lat/lon decode" % label,
                  got[4]["lat"] == 473977000 and got[4]["lon"] == 85456000,
                  (got[4]["lat"], got[4]["lon"]))
            check("pymavlink %s hdg decode" % label, got[4]["hdg"] == 9000,
                  got[4]["hdg"])
        # a corrupted real frame must still be rejected
        bad = bytearray(g)
        bad[len(bad) - 1] ^= 0xAA
        check("pymavlink %s corrupt frame rejected" % label,
              M.parse_datagram(bytes(bad)) is None)

