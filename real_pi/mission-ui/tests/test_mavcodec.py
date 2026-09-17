#!/usr/bin/env python3
"""
tests/test_mavcodec.py — verify bridge/_mavlink_v2.py against pymavlink.

RUNS ONLY where pymavlink is installed (dev venv / CI). On the competition
laptop this test is not needed — the codec is frozen and already verified.

Checks, for every message in the table:
  1. build_frame() output is BYTE-IDENTICAL to pymavlink's pack (same seq,
     sysid, compid) — proves header layout, field order, struct packing AND
     the CRC_EXTRA signatures all match the real definitions.
  2. Our parser decodes frames produced by pymavlink with identical fields.
  3. A corrupted byte is rejected (CRC) by both implementations.

Exit code 0 = all pass.
"""
import io
import os
import random
import struct
import sys

os.environ["MAVLINK20"] = "1"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "bridge"))

try:
    from pymavlink.dialects.v20 import ardupilotmega as mod
except Exception as e:
    print("SKIP: pymavlink not importable (%r) — codec unverified on this machine." % (e,))
    sys.exit(0)

import _mavlink_v2 as C  # noqa: E402
from _mavtable_gen import TABLE  # noqa: E402

random.seed(20260913)

FAILS = []


def rand_value(fchar):
    if fchar.endswith("s"):
        n = int(fchar[:-1])
        return bytes(random.randint(32, 126) for _ in range(random.randint(1, n)))
    import re as _re
    m = _re.match(r"^(\d+)([a-zA-Z])$", fchar)
    if m:  # numeric array, e.g. '10H'
        n = int(m.group(1))
        return tuple(rand_value(m.group(2)) for _ in range(n))
    base = fchar
    if base == "f":
        return round(random.uniform(-1e3, 1e3), 4)
    if base == "b":
        return random.randint(-128, 127)
    if base == "B":
        return random.randint(0, 255)
    if base == "h":
        return random.randint(-32768, 32767)
    if base == "H":
        return random.randint(0, 65535)
    if base in "iI":
        return random.randint(0, 2000000000)
    if base in "qQ":
        return random.randint(0, 2**48)
    if base in "dD":
        return round(random.uniform(-1e3, 1e3), 4)
    raise ValueError(fchar)


def norm(v):
    if isinstance(v, (tuple, list)):
        return tuple(int(x) for x in v)
    if isinstance(v, (bytes, bytearray)):
        # char[N] field: compare content minus trailing NULs (pymavlink's
        # to_dict() already strips them)
        return bytes(v).rstrip(b"\x00").decode("latin-1")
    if isinstance(v, str):
        return v
    if isinstance(v, float):
        return round(v, 6)
    return v


def main():
    tested = 0
    for msgid, (name, crc_extra, pairs, xml_fields) in sorted(TABLE.items()):
        # --- 1. byte-identical build ---
        vals = {f: rand_value(fc) for f, fc in pairs}
        seq, sysid, compid = 7, 255, 1
        mine = C.build_frame(msgid, sysid, compid, seq, vals)

        M = mod.MAVLink(io.BytesIO(), srcSystem=sysid, srcComponent=compid)
        M.seq = seq
        cls = getattr(mod, "MAVLink_" + name.lower() + "_message")
        xml_vals = []
        for f in xml_fields:
            fc = dict(pairs)[f]
            v = vals[f]
            if fc.endswith("s"):
                n = int(fc[:-1])
                v = (v + b"\x00" * n)[:n]
            xml_vals.append(v)
        theirs = bytes(cls(*xml_vals).pack(M))
        if mine != theirs:
            FAILS.append("%s: build mismatch\n  mine  %s\n  theirs %s" % (name, mine.hex(), theirs.hex()))
            continue
        tested += 1

        # --- 2. mutual parse ---
        p_mine = C.parse_datagram(mine)
        assert p_mine is not None, "self-parse failed for %s" % name
        _, _, _, seq2, f_mine = p_mine
        assert seq2 == seq
        M2 = mod.MAVLink(io.BytesIO())
        out = M2.parse_buffer(mine)
        if not out:
            FAILS.append("%s: pymavlink could not parse our frame" % name)
            continue
        f_theirs = out[0].to_dict()
        for f, fc in pairs:
            a, b = norm(f_mine[f]), norm(f_theirs[f])
            if a != b:
                FAILS.append("%s: field %s differs: %r vs %r" % (name, f, a, b))
                break
        # and the reverse direction: their frame -> our parser
        p_rev = C.parse_datagram(theirs)
        if p_rev is None:
            FAILS.append("%s: we could not parse pymavlink frame" % name)
            continue

        # --- 3. corruption rejected ---
        bad = bytearray(mine)
        bad[-1] ^= 0x55  # flip a CRC bit
        if C.parse_datagram(bytes(bad)) is not None:
            FAILS.append("%s: corrupted CRC accepted!" % name)
        M3 = mod.MAVLink(io.BytesIO())
        try:
            if M3.parse_buffer(bytes(bad)):
                FAILS.append("%s: pymavlink accepted corrupted frame (unexpected)" % name)
        except Exception:
            pass  # pymavlink raises on bad CRC — that counts as rejection

    total = len(TABLE)
    print("codec vs pymavlink: %d/%d messages byte-identical + mutual-parse OK" % (tested, total))
    if FAILS:
        print("FAILURES:")
        for f in FAILS:
            print(" -", f)
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
