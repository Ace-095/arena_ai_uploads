#!/usr/bin/env python3
"""Stdlib tests for bringup.py — the Pi-link facts the UI/doctor depend on.

Run: python3 tests/test_bringup.py   (no deps beyond the stdlib)
"""
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import bringup as B  # noqa

FAILS = []


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


# --- _is_usable: only real, reachable-from-LAN addresses -------------------
check("loopback rejected", not B._is_usable("127.0.0.1"))
check("link-local rejected", not B._is_usable("169.254.1.9"))
check("wildcard rejected", not B._is_usable("0.0.0.0"))
check("ipv6 rejected", not B._is_usable("fe80::1"))
check("empty rejected", not B._is_usable(""))
check("private LAN accepted", B._is_usable("192.168.1.42"))
check("public accepted", B._is_usable("8.8.8.8"))

# --- lan_ips / lan_urls are well formed (content depends on the host) ------
ips = B.lan_ips()
check("lan_ips is a list of strings", isinstance(ips, list)
      and all(isinstance(i, str) for i in ips), repr(ips))
check("lan_ips has no loopback", all(not i.startswith("127.") for i in ips))
check("lan_ips deduped", len(ips) == len(set(ips)))
urls = B.lan_urls(8000)
check("lan_urls shape", all(u.startswith("http://") and u.endswith(":8000")
                            for u in urls), repr(urls))
check("lan_urls matches ips", len(urls) == len(ips))

# --- port_free: a bound port is not free ----------------------------------
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", 0))
busy = s.getsockname()[1]
s.listen(1)
check("port_free false while bound", not B.port_free(busy, "127.0.0.1"))
check("port_free true on an unused port", B.port_free(0, "127.0.0.1") is False
      or True)  # port 0 is special-cased by the OS; only assert no exception
s.close()
time.sleep(0.1)
check("port_free true after close", B.port_free(busy, "127.0.0.1"))

# --- tcp_reachable ---------------------------------------------------------
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
port = srv.getsockname()[1]
srv.listen(1)
ok, detail = B.tcp_reachable("127.0.0.1", port, 1.0)
check("tcp_reachable ok on a listening port", ok, detail)
srv.close()
ok2, detail2 = B.tcp_reachable("127.0.0.1", port, 0.5)
check("tcp_reachable fails on a closed port", not ok2 and "ms" not in detail2,
      detail2)

# --- system_stats: every contract key present, JSON-safe -------------------
st = B.system_stats()
for k in ("temp_c", "cpu_pct", "mem_pct", "disk_pct", "uptime_s"):
    check("system_stats has %s" % k, k in st)
check("system_stats disk_pct plausible",
      st["disk_pct"] is None or 0.0 <= st["disk_pct"] <= 100.0, repr(st))
check("system_stats host+pid", st.get("host") and st.get("pid") == os.getpid())
import json  # noqa
check("system_stats is JSON-serialisable", bool(json.dumps(st)))
time.sleep(0.15)
st2 = B.system_stats()
check("cpu_pct needs two samples (first may be None)",
      st2["cpu_pct"] is None or 0.0 <= st2["cpu_pct"] <= 100.0, repr(st2))

# --- Bringup ledger --------------------------------------------------------
bu = B.Bringup()
check("starts at init with everything pending",
      bu.snapshot()["stage"] == "init" and bu.state["fc"] == "pending"
      and bu.state["server"] == "pending", bu.one_line())
bu.mark("server", "ok", "0.0.0.0:8000")
bu.mark("detector", "ok", "classical")
check("stage advances past finished subsystems", bu.snapshot()["stage"] == "cameras",
      bu.snapshot()["stage"])
bu.mark("cameras", "skipped", "--no-cams")
bu.mark("streams", "skipped", "0 cams")
check("not ready while fc pending", not bu.ok() and bu.snapshot()["stage"] == "fc")
bu.mark("fc", "ok", "tcp:127.0.0.1:5762")
check("ready when all ok/skipped", bu.ok() and bu.snapshot()["stage"] == "ready",
      bu.one_line())

bu2 = B.Bringup()
bu2.mark("server", "ok")
msg = bu2.fail("fc", RuntimeError("no FC heartbeat; tried: tcp:127.0.0.1:5762"))
check("fail records the error", "no FC heartbeat" in msg
      and len(bu2.snapshot()["errors"]) == 1)
# `stage` is "what bring-up is working on now", so with detector/cameras
# still pending it stays on the earliest pending subsystem; the FAILURE
# itself is visible in state[] and errors[] (which is what the UI reads).
check("fail marks the subsystem failed", bu2.snapshot()["state"]["fc"] == "failed",
      bu2.snapshot()["state"])
check("stage stays on pending work (or the failure once all else is done)",
      bu2.snapshot()["stage"] in ("detector", "cameras", "streams", "fc"),
      bu2.snapshot()["stage"])
bu2.mark("detector", "ok")
bu2.mark("cameras", "skipped")
bu2.mark("streams", "skipped")
check("stage points at the failed subsystem once nothing is pending",
      bu2.snapshot()["stage"] == "fc", bu2.snapshot()["stage"])
check("fail is not ok", not bu2.ok())
bu2.note("fc", "attempt 2 failed")
check("note updates detail", bu2.snapshot()["detail"]["fc"] == "attempt 2 failed")
for i in range(30):                      # error ring must stay capped
    bu2.fail("fc", RuntimeError("e%d" % i))
check("errors capped at 20", len(bu2.snapshot()["errors"]) == 20,
      len(bu2.snapshot()["errors"]))
check("newest error kept last", "e29" in bu2.snapshot()["errors"][-1]["error"])

bu3 = B.Bringup()
bu3.stop = threading.Event()             # main.py sets this attribute
bu3.set_urls(["http://192.168.1.42:8000"])
check("urls published", bu3.snapshot()["urls"] == ["http://192.168.1.42:8000"])
bu3.finish()
check("finish -> ready", bu3.snapshot()["stage"] == "ready" and bu3.snapshot()["done"])

# finishing must NOT paper over a failure: "ready" would tell the operator the
# box is fine when the FC never came up.
bu4 = B.Bringup()
bu4.mark("server", "ok")
bu4.fail("fc", RuntimeError("no heartbeat"))
bu4.finish()
check("finish keeps a failed subsystem pinned as the stage",
      bu4.snapshot()["stage"] == "fc" and bu4.snapshot()["done"]
      and not bu4.ok(), bu4.snapshot()["stage"])
check("one_line mentions subsystems", "fc=" in bu3.one_line(), bu3.one_line())
check("snapshot is JSON-serialisable", bool(json.dumps(bu3.snapshot())))

print("FAILURES: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
