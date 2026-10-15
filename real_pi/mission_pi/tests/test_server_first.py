#!/usr/bin/env python3
"""Integration test: the HTTP server must answer *before* the FC is found.

This is the exact bug that made the Windows UI show "Pi link: down" (found
2026-09-14): main.py called fc.connect() — which blocks for
hb_timeout*len(bauds) seconds and raised when no flight controller answered —
*before* uvicorn bound the port. With no FC running, nothing ever listened on
:8000, so the UI's connect probe failed and every diagnostic endpoint was
unreachable: the operator could not even ask the box why.

Boots main.py against a device where nothing listens and asserts:
  * GET /health answers quickly, with the process still alive;
  * bring-up says server=ok and fc=failed (the truth is published, not hidden
    behind a dead port), with the device string and watchdog state;
  * GET /api/bringup answers too;
  * ws://.../ws/telemetry completes a real handshake (101) when a websocket
    implementation is installed — urllib cannot do this, so the test speaks
    the handshake by hand like tools/link_doctor.py does;
  * the startup banner with the LAN URLs reaches the console.

Skips itself (exit 0) if fastapi/uvicorn are missing.
"""
import base64
import hashlib
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.join(HERE, "..")
sys.path.insert(0, PI)

FAILS = []
WS_KEY = "dGhlIHNhbXBsZSBub25jZQ=="


def check(name, cond, extra=""):
    print(("PASS " if cond else "FAIL ") + name, extra if not cond else "")
    if not cond:
        FAILS.append(name)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def ws_handshake(port, timeout=4.0):
    """Return (status_line, accept_header) from a real Upgrade handshake."""
    s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
    s.sendall(("GET /ws/telemetry HTTP/1.1\r\n"
               "Host: 127.0.0.1:%d\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n"
               "Origin: http://127.0.0.1:8100\r\n\r\n" % (port, WS_KEY)).encode())
    s.settimeout(timeout)
    head = b""
    while b"\r\n\r\n" not in head:
        chunk = s.recv(1024)
        if not chunk:
            break
        head += chunk
    s.close()
    line = head.split(b"\r\n", 1)[0].decode("utf-8", "replace")
    accept = ""
    for h in head.split(b"\r\n"):
        if h.lower().startswith(b"sec-websocket-accept:"):
            accept = h.split(b":", 1)[1].strip().decode()
    return line, accept


try:
    import fastapi  # noqa
    import uvicorn  # noqa
except Exception as exc:                                    # noqa: BLE001
    print("SKIP test_server_first: %s" % exc)
    sys.exit(0)

PORT = free_port()
DEAD_DEV = "tcp:127.0.0.1:%d" % free_port()     # nothing listens there
CFG = os.path.join(tempfile.gettempdir(), "mission_pi_server_first.yaml")
with open(CFG, "w") as f:
    # `ui:` on purpose — main.py must accept it as an alias for `server:`
    f.write("""\
mission_id: server-first-test
fc:
  device: "%s"          # `conn:` is canonical; `device:` is the accepted alias
  bauds: [115200]
  hb_timeout: 2
  retry_s: 1
  connect_timeout_s: 4
  sysid: 255
  compid: 190
  stream_hz: 4
  dead_s: 3
cameras: {}
ui:
  host: 127.0.0.1
  port: %d
store:
  path: "%s"
""" % (DEAD_DEV, PORT, tempfile.gettempdir()))

proc = subprocess.Popen(
    [sys.executable, "-u", os.path.join(PI, "main.py"), "--config", CFG,
     "--no-cams"],
    cwd=PI, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
t0 = time.time()
health = None
try:
    # the whole point: /health must answer while the FC is still unreachable
    while time.time() - t0 < 25:
        if proc.poll() is not None:
            break
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d/health" % PORT,
                                        timeout=1.0) as r:
                health = json.loads(r.read().decode())
                break
        except Exception:                                   # noqa: BLE001
            time.sleep(0.2)
    answered_after = time.time() - t0

    check("main.py did not exit when the FC was unreachable",
          proc.poll() is None,
          "exit=%s after %.1fs" % (proc.returncode, answered_after))
    check("/health answered quickly (was: never)",
          health is not None and answered_after < 15.0,
          "no answer in %.1fs" % answered_after)
    check("config `ui:` host/port honoured (bound %d, not 8000)" % PORT,
          health is not None, "nothing on port %d" % PORT)

    # let bring-up actually attempt the FC (hb_timeout=2 -> ~2-3 s), and let
    # the startup banner print, before we assert on either
    deadline = time.time() + 20.0
    state = {}
    while time.time() < deadline:
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d/health" % PORT,
                                        timeout=1.0) as r:
                health = json.loads(r.read().decode())
            state = (health.get("bringup") or {}).get("state") or {}
            if state.get("fc") not in (None, "pending"):
                break
        except Exception:                                   # noqa: BLE001
            pass
        time.sleep(0.3)
    time.sleep(1.0)                       # banner + system channel catch-up

    bu = health.get("bringup") or {}
    check("/health publishes the bring-up ledger", bool(bu), sorted(health))
    check("bring-up reports server=ok", state.get("server") == "ok", state)
    check("bring-up reports the FC honestly (failed, not ok)",
          state.get("fc") == "failed", state)
    check("bring-up stage points at the FC, not 'ready'",
          bu.get("stage") == "fc", bu.get("stage"))
    check("bring-up records WHY the FC failed",
          any("fc" == (e.get("stage") or e.get("sub")) or "heartbeat" in
              str(e.get("error", "")) for e in (bu.get("errors") or [])),
          bu.get("errors"))
    check("/health ready=False while the FC is down",
          health.get("ready") is False, health.get("ready"))
    fcst = health.get("fc") or {}
    check("/health carries the FC device string",
          fcst.get("device") == DEAD_DEV, fcst)
    check("/health carries the watchdog state", "watchdog" in fcst
          and "reconnects" in fcst, fcst)
    check("/health reports link=False", health.get("link") is False)

    with urllib.request.urlopen("http://127.0.0.1:%d/api/bringup" % PORT,
                                timeout=2.0) as r:
        body = json.loads(r.read().decode())
    check("/api/bringup answers", body.get("ok") in (True, False), body)
    check("/api/bringup includes the failure reason",
          bool(body.get("errors")), body.get("errors"))

    # WS route: urllib gets a 404 here because FastAPI only registers the
    # websocket scope — do the Upgrade by hand, like the UI's browser does.
    try:
        import websockets                               # noqa
        have_ws = True
    except Exception:                                   # noqa: BLE001
        have_ws = False
    line, accept = ws_handshake(PORT)
    if have_ws:
        want = base64.b64encode(hashlib.sha1(
            (WS_KEY + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()
        ).digest()).decode()
        check("ws route upgrades (101) with a ws impl installed",
              "101" in line, line)
        check("Sec-WebSocket-Accept is correct", accept == want,
              "%r != %r" % (accept, want))
    else:
        # without websockets/wsproto, uvicorn 404s WS routes and the UI must
        # fall back to polling /api/fsm/status + /api/qr/status (see js/links.js
        # and SIM_GUIDE.md §9)
        print("INFO no websocket impl installed: handshake -> %s "
              "(UI falls back to polling)" % line)
        check("ws route fails loudly (404) without a ws impl, not a hang",
              "404" in line or "400" in line, line)
finally:
    proc.terminate()
    try:
        out = proc.communicate(timeout=8)[0] or ""
    except Exception:                                       # noqa: BLE001
        proc.kill()
        out = ""
    check("startup banner printed the Pi-link URLs",
          "Pi link" in out and "http://" in out,
          "\n".join(out.splitlines()[-8:]))
    check("banner tells the operator how to verify from Windows",
          "/health" in out, "\n".join(out.splitlines()[-8:]))
    check("no traceback escaped to the console", "Traceback" not in out,
          "\n".join(out.splitlines()[-8:]))
    try:
        os.remove(CFG)
    except OSError:
        pass

print("FAILS: %s" % (", ".join(FAILS) if FAILS else "none"))
sys.exit(1 if FAILS else 0)
