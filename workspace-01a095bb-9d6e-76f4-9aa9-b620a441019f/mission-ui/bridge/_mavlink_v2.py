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
  for the first 0xFD and verifies the CRC; bad datagrams are dropped,
  never fatal.
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
HEADER_FMT = "<BBBBBBBHB"   # magic, len, iflag, cflag, seq, sysid, compid, msgid(3)
HEADER_LEN = 10
MAX_PAYLOAD = 255


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
    """Parse (up to) one frame from a UDP datagram.
    Returns (msgid, sysid, compid, seq, fields) or None if no valid frame."""
    i = data.find(bytes([STX_V2]))
    if i < 0 or i + HEADER_LEN > len(data):
        return None
    (magic, ln, iflag, _cflag, seq, sysid, compid,
     m16, mhi) = struct.unpack_from(HEADER_FMT, data, i)
    if iflag & 0x01:          # signing — not supported, drop
        return None
    if ln > MAX_PAYLOAD:
        return None
    msgid = m16 | (mhi << 16)
    entry = TABLE.get(msgid)
    if entry is None:
        return None           # unknown message id — ignore (forward compat)
    total = HEADER_LEN + ln + 2
    if i + total > len(data):
        return None
    frame = data[i:i + total]
    crc_in = struct.unpack_from("<H", frame, HEADER_LEN + ln)[0]
    if crc16(frame[1:HEADER_LEN + ln] + bytes([entry[1]])) != crc_in:
        return None           # CRC mismatch — corrupt / wrong dialect
    # The payload may be trailing-zero-stripped (valid MAVLink v2); re-pad to
    # the full field layout before unpacking. Reject frames claiming more
    # payload bytes than the message definition allows.
    full_len = sum(struct.calcsize("<" + fc) for _f, fc in entry[2])
    if ln > full_len:
        return None
    payload = frame[HEADER_LEN:HEADER_LEN + ln] + b"\x00" * (full_len - ln)
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
    return msgid, sysid, compid, seq, fields


# ---------------------------------------------------------------------------
# Constants (values from the MAVLink 2.0 definitions / ArduPilot docs)
# ---------------------------------------------------------------------------
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SPRAYER = 310
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001

MAV_FRAME_GLOBAL = 0

MAV_MISSION_TYPE_MISSION = 0
MAV_MISSION_TYPE_FENCE = 8
MAV_MISSION_ACCEPTED = 0

MAV_PARAM_TYPE_REAL32 = 9

MAV_SEVERITY_INFO = 6

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_GCS = 6
MAV_STATE_STANDBY = 0
MAV_STATE_ACTIVE = 4
MAV_MODE_FLAG_SAFETY_ARMED = 128

MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49

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
