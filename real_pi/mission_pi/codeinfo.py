#!/usr/bin/env python3
"""Which code is running? — the first question a field log has to answer.

The 2026-10-12 session cost a full round trip: the pasted Pi log still showed
`idx = ord(k) - ord("1")` and HAILO_NETWORK_GROUP_NOT_ACTIVATED(69), which can
only happen on a checkout that is NOT the one being worked on — the Pi's
`git pull` had pulled its own (other) branch, and nothing in the output said
so. Every tool that talks to hardware now stamps its version, so one pasted
header answers "is the fix actually in the run?" before anyone debugs a
symptom that was already fixed.

`code_version` is injectable and never raises: on a copy without git (a
tarball, a scp'd folder) it says so instead of crashing the bring-up.
"""
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))


def _run_git(args, root):
    """stdout of `git -C <root> <args>`; raises if git is missing/fails."""
    return subprocess.run(["git", "-C", root] + list(args),
                          capture_output=True, text=True, timeout=5).stdout


def code_version(root=None, run=None):
    """-> {'hash', 'branch', 'dirty', 'error'} — never raises, never guesses.

    A short commit hash is the whole point: `git log --oneline -1` on the
    machine doing the work and this line in the log must agree.
    """
    root = root or HERE
    run = run or _run_git
    out = {"hash": "", "branch": "", "dirty": False, "error": ""}
    try:
        out["hash"] = (run(["rev-parse", "--short", "HEAD"], root) or "").strip()
    except Exception as e:
        out["error"] = "%s: %s" % (type(e).__name__, e)
        return out
    try:
        out["branch"] = (run(["rev-parse", "--abbrev-ref", "HEAD"],
                             root) or "").strip()
        out["dirty"] = bool((run(["status", "--porcelain"], root) or "").strip())
    except Exception as e:                    # partial info is still useful
        out["error"] = "%s: %s" % (type(e).__name__, e)
    return out


def describe(v, prefix=""):
    """One log line: 'ab12cd3 (arena/…, dirty)' / 'unknown (no git …)'."""
    v = v or {}
    h = (v.get("hash") or "").strip()
    if not h:
        why = v.get("error") or "no git repository here"
        return "%sunknown — %s (an installed copy does this)" % (prefix, why)
    dirty = ", uncommitted changes" if v.get("dirty") else ""
    return "%s%s (%s%s)" % (prefix, h, v.get("branch") or "?", dirty)
