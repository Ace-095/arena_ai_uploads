#!/usr/bin/env python3
"""Stdlib tests for consensus.py (run anywhere: python tests/test_consensus.py)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from consensus import ConsensusBuffer  # noqa

FAILS = []


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        FAILS.append(name)


c = ConsensusBuffer(required_consecutive=3, miss_tolerance=1)
n, s = c.update("AB12")
check("streak1", not n and s["streak"] == 1 and not s["confirmed"])
n, s = c.update("AB12")
check("streak2", not n and s["streak"] == 2)
n, s = c.update(None)  # tolerated miss
check("miss tolerated", not n and s["streak"] == 2)
n, s = c.update("AB12")
check("confirmed", n and s["confirmed"] and s["value"] == "AB12")
n, s = c.update("AB12")
check("stays confirmed", not n and s["confirmed"])

c.reset()
for v in ["X", None, None]:
    n, s = c.update(v)
check("sustained blank resets", not s["confirmed"] and s["streak"] == 0)

c.reset()
for v in ["A", "A", "B", "B"]:
    n, s = c.update(v)
check("switch after tolerance", s["tracking"] == "B" and s["streak"] == 1)

print("FAILURES:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
