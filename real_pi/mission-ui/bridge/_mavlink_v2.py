"""
_mavlink_v2.py — minimal MAVLink v2 codec for the Mission Companion bridge.

STDLIB ONLY (struct + the generated table in _mavtable_gen.py). The table was
generated from pymavlink's official MAVLink 2.0 (v20) ardupilotmega
definitions — including the per-message CRC_EXTRA signature that hehe's
vendored shim faked with a zero checksum (real ArduPilot firmware rejects
that). tests/test_mavcodec.py verifies every frame byte-for-byte against
pymavlink before this codec is trusted.

Design notes
------------
* UDP transport here is one frame per datagram (the MAVLink-over-UDP
  convention used by Mission Planner forwarding). parse_datagram() scans
  for a frame start and verifies the CRC; bad datagrams are dropped,
  never fatal.
* BOTH wire versions are accepted on input: MAVLink 2 (magic 0xFD) and
  MAVLink 1 (magic 0xFE). We only ever SEND v2, but MP mirrors whatever the
  vehicle speaks, and a link on SERIAL_PROTOCOL=1 (or older firmware) sends
  v1. A v2-only parser dropped every such frame silently — telemetry looked
  half-alive and the STATUSTEXT carrying the QR payload never arrived, which
  on the bench reads as "the bridge is connected but the hunt found nothing".
  PARSE_STATS below makes any drop visible instead of silent.
* Message values are plain dicts keyed by field name (wire order handled
  internally). char[N] fields are bytes; uint16_t[N] etc. are tuples.
* We never sign messages and never accept signing (incompat-flag check) —
  signing is out of scope for this project.
"""
import re
import struct

from _mavtable_gen import TABLE, ID_TO_NAME

_ARRAY_RE = re.compile(r"^(\d+)([a-zA-Z])$")

STX_V2 = 0xFD
STX_V1 = 0xFE
HEADER_FMT = "<BBBBBBBHB"   # magic, len, iflag, cflag, seq, sysid, compid, msgid(3)
HEADER_LEN = 10
HEADER_V1_FMT = "<BBBBBB"   # magic, len, seq, sysid, compid, msgid(1)
HEADER_V1_LEN = 6
MAX_PAYLOAD = 255

# Parse diagnostics — surfaced by mav_client.state_dict() at /api/mp/mavlink so
# a link that delivers bytes but no usable frames is explainable from the UI.
PARSE_STATS = {"v2": 0, "v1": 0, "bad_crc": 0, "unknown_id": 0,
               "signed_dropped": 0, "truncated": 0, "no_magic": 0,
               "short_header": 0, "bad_length": 0, "no_frame": 0}

# Why the most recent parse_datagram() returned None. Callers must be able to
# tell "a message id this bridge does not model" (expected, forward-compatible
# — SITL streams VFR_HUD etc. all day) from "the bytes are not MAVLink" (a real
# problem: wrong port, another service, corrupt mirror). Lumping them together
# made a healthy link look 25% broken.
LAST_DROP = None


def _drop(reason):
    PARSE_STATS[reason] = PARSE_STATS.get(reason, 0) + 1
    return reason


def crc16(data: bytes) -> int:
    """CRC-16/MCRF4XX (X.25) — the MAVLink wire CRC.
    Ported verbatim from pymavlink generator/mavcrc.py (LGPL, Andrew Tridgell)
    / the official mavlink checksum.h. Verified against pymavlink in
    tests/test_mavcodec.py. (NOTE: 0xA001 bit-wise-reflected style is the
    MODBUS polynomial — wrong here; the nibble-folding form below is MCRF4XX.)
    """
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc


def name_of(msgid: int) -> str:
    return ID_TO_NAME.get(msgid, "MSG_%d" % msgid)


# Name -> msgid (from the generated table; authoritative for this dialect)
MSG_ID = {name: mid for mid, (name, _c, _p, _x) in TABLE.items()}


def build_frame(msgid: int, sysid: int, compid: int, seq: int, values: dict) -> bytes:
    """Encode one MAVLink v2 frame. values: field name -> value (missing
    fields default to 0 / zero-bytes)."""
    entry = TABLE[msgid]
    crc_extra = entry[1]
    payload = b""
    for field, fchar in entry[2]:
        v = values.get(field, 0)
        m = _ARRAY_RE.match(fchar)
        if m and m.group(2) == "s":
            # byte-string field (char[N]): single bytes arg, pad/truncate
            if isinstance(v, str):
                v = v.encode("utf-8")
            n = int(m.group(1))
            v = (bytes(v) + b"\x00" * n)[:n]
            payload += struct.pack("<" + fchar, v)
        elif m:
            # numeric array (e.g. '10H'): splat sequence into positional args
            if isinstance(v, (int, float)):
                v = [v] * int(m.group(1))
            payload += struct.pack("<" + fchar, *v)
        else:
            payload += struct.pack("<" + fchar, v)
    # MAVLink v2 wire format: trailing zero payload bytes MAY be stripped
    # (pymavlink does this by default; receivers re-pad from mlen). We match
    # pymavlink byte-for-byte — tests/test_mavcodec.py proves it.
    while len(payload) > 1 and payload[-1] == 0:
        payload = payload[:-1]
    header = struct.pack(HEADER_FMT, STX_V2, len(payload), 0, 0,
                         seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                         msgid & 0xFFFF, (msgid >> 16) & 0xFF)
    crc = crc16(header[1:] + payload + bytes([crc_extra]))
    return header + payload + struct.pack("<H", crc)


def parse_datagram(data: bytes):
    """Parse (up to) one frame from a UDP datagram, MAVLink v2 or v1.
    Returns (msgid, sysid, compid, seq, fields) or None if no valid frame.

    Fields are returned in the v2 layout: v1 frames carry no extension
    fields, so those come back as zeros (e.g. STATUSTEXT.id/chunk_seq).
    """
    global LAST_DROP
    i2 = data.find(bytes([STX_V2]))
    i1 = data.find(bytes([STX_V1]))
    if i2 < 0 and i1 < 0:
        LAST_DROP = _drop("no_magic")
        return None
    # try each candidate start in order of appearance: a datagram may be
    # prefixed with garbage, and a 0xFD inside a payload must not win over a
    # real frame that starts earlier.
    reason = None
    for i in sorted(x for x in (i2, i1) if x >= 0):
        got = _parse_at(data, i)
        if isinstance(got, tuple):
            LAST_DROP = None
            return got
        if reason is None:
            # report the FIRST candidate's verdict: the earliest magic byte is
            # the intended frame start, so its reason is the honest one. (A
            # corrupted v1 frame also contains a 0xFD msgid byte, and reporting
            # that bogus candidate's verdict read as "signed frames dropped".)
            reason = got
    LAST_DROP = reason or "no_frame"
    return None


def _parse_at(data: bytes, i: int):
    """Parse one frame starting at data[i]; None if it is not a valid frame."""
    magic = data[i]
    if magic == STX_V2:
        hlen = HEADER_LEN
        if i + hlen > len(data):
            return _drop("short_header")
        (_mg, ln, iflag, _cflag, seq, sysid, compid,
         m16, mhi) = struct.unpack_from(HEADER_FMT, data, i)
        if iflag & 0x01:      # signing — not supported, drop
            return _drop("signed_dropped")
        msgid = m16 | (mhi << 16)
    elif magic == STX_V1:
        hlen = HEADER_V1_LEN
        if i + hlen > len(data):
            return _drop("short_header")
        _mg, ln, seq, sysid, compid, m8 = struct.unpack_from(HEADER_V1_FMT,
                                                             data, i)
        msgid = m8            # v1 message ids are one byte (< 256)
    else:
        return None
    if ln > MAX_PAYLOAD:
        return _drop("bad_length")
    entry = TABLE.get(msgid)
    if entry is None:
        return _drop("unknown_id")   # not modelled here — ignore (fwd compat)
    total = hlen + ln + 2
    if i + total > len(data):
        return _drop("truncated")
    frame = data[i:i + total]
    crc_in = struct.unpack_from("<H", frame, hlen + ln)[0]
    # the CRC covers everything after the magic byte, plus the message's
    # CRC_EXTRA signature — identical rule in v1 and v2.
    if crc16(frame[1:hlen + ln] + bytes([entry[1]])) != crc_in:
        return _drop("bad_crc")      # corrupt, or a dialect we do not have
    # A v2 payload may be trailing-zero-stripped, and a v1 payload is short by
    # the v2 extension fields; re-pad to the full field layout before
    # unpacking. Reject frames claiming more payload than the definition allows.
    full_len = sum(struct.calcsize("<" + fc) for _f, fc in entry[2])
    if ln > full_len:
        return _drop("bad_length")
    payload = frame[hlen:hlen + ln] + b"\x00" * (full_len - ln)
    fields = {}
    off = 0
    for field, fchar in entry[2]:
        m = _ARRAY_RE.match(fchar)
        if m and m.group(2) != "s":
            val = struct.unpack_from("<" + fchar, payload, off)  # whole array
        else:
            val = struct.unpack_from("<" + fchar, payload, off)[0]
        fields[field] = val
        off += struct.calcsize("<" + fchar)
    PARSE_STATS["v2" if magic == STX_V2 else "v1"] += 1
    return (msgid, sysid, compid, seq, fields)


# ---------------------------------------------------------------------------
# Constants (values from the MAVLink 2.0 definitions / ArduPilot docs)
# ---------------------------------------------------------------------------
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SPRAYER = 216
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001

MAV_FRAME_GLOBAL = 0

MAV_MISSION_TYPE_MISSION = 0
MAV_MISSION_TYPE_FENCE = 1
MAV_MISSION_ACCEPTED = 0

MAV_PARAM_TYPE_REAL32 = 9

MAV_SEVERITY_INFO = 6

MAV_TYPE_GCS = 6
MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_AUTOPILOT_INVALID = 8  # what a GCS puts in heartbeat.autopilot (there is no AUTOPILOT_GCS)
MAV_STATE_STANDBY = 3  # 0 is UNINIT; STANDBY is 3
MAV_STATE_ACTIVE = 4
MAV_MODE_FLAG_SAFETY_ARMED = 128

MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49
MAVLINK_MSG_ID_HOME_POSITION = 242

# ArduCopter custom modes — authoritative: pymavlink v20 ardupilotmega
# COPTER_MODE_* enum (+ POSITION/OF_LOITER: real in ArduPilot 4.x, absent
# from that dialect snapshot; 12 was removed upstream, 29 is ENUM_END).
ARDU_COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 8: "POSITION", 9: "LAND",
    10: "OF_LOITER", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
    16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
    28: "TURTLE",
}
MODE_STABILIZE, MODE_GUIDED, MODE_AUTO, MODE_RTL, MODE_LAND = 0, 4, 3, 6, 9

# FENCE_TYPE bits (AP_Fence.h): ALT_MAX=1 CIRCLE=2 POLYGON=4 ALT_MIN=8
FENCE_TYPE_POLYGON_BIT = 4

# COMMAND_ACK result codes (MAV_RESULT) — for readable UI logs.
COMMAND_ACK_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                      3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
# Mission-protocol result codes (MAV_MISSION_RESULT).
MAV_MISSION_RESULT = {0: "ACCEPTED", 1: "ERROR", 2: "UNSUPPORTED_FRAME", 3: "UNSUPPORTED",
                      4: "NO_SPACE", 5: "INVALID", 6: "INVALID_PARAM1", 7: "INVALID_PARAM2",
                      8: "INVALID_PARAM3", 9: "INVALID_PARAM4", 10: "INVALID_PARAM5_X",
                      11: "INVALID_PARAM6_Y", 12: "INVALID_PARAM7", 13: "INVALID_SEQUENCE",
                      14: "DENIED", 15: "OPERATION_CANCELLED"}
