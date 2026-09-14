#!/usr/bin/env python3
"""link_doctor.py — why can't the UI reach the Pi?

Two jobs, one script (stdlib only — it runs on the Pi/Linux laptop AND on
the Windows machine with nothing but Python installed):

  LOCAL  (run on the machine that runs mission_pi)
      python3 tools/link_doctor.py
    Checks: python deps importable, port 8000 free/owned by us, which LAN
    IPs exist (and which one the UI must be given), whether a firewall is
    blocking it, whether SITL's 5760/5762 are listening, whether the Gazebo
    camera bridge on 8099 is alive, and whether this server answers its own
    /health. Prints the exact URL to paste into the UI.

  REMOTE (run on the Windows laptop, or anywhere the UI runs)
      python tools/link_doctor.py --target http://192.168.1.42:8000
    Checks the same path the browser uses: TCP connect, HTTP GET /health,
    GET /api/cameras, a real WebSocket handshake on /ws/telemetry and how
    many envelopes arrive. This separates the three failure modes that look
    identical in the UI ("PI down · retry 1"):
        * nothing listening / wrong IP / firewall  -> TCP fails
        * mission_pi running but not this project   -> HTTP 404 on /health
        * HTTP fine but WS blocked (proxy/browser)  -> handshake fails

Exit code: 0 = every check passed, 1 = something failed (usable in scripts).
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"
_results = []


def say(level, msg, hint=None):
    _results.append((level, msg))
    mark = {PASS: "  ok  ", FAIL: " FAIL ", WARN: " warn ", INFO: "  ..  "}[level]
    print("[%s] %s" % (mark, msg))
    if hint:
        for line in str(hint).splitlines():
            print("         %s" % line)


def section(title):
    print("\n== %s ==" % title)


# ------------------------------------------------------------- local checks --
def check_deps():
    section("python deps (mission_pi)")
    core = {"pymavlink": "pip install -r requirements.txt",
            "yaml": "pip install pyyaml",
            "fastapi": "pip install fastapi  (the UI link needs it)",
            "uvicorn": "pip install uvicorn  (the UI link needs it)",
            "numpy": "pip install numpy"}
    vision = {"cv2": "sudo apt install python3-opencv  |  pip install opencv-python",
              "pyzbar": "sudo apt install libzbar0 && pip install pyzbar"}
    for mod, fix in list(core.items()):
        try:
            __import__(mod)
            say(PASS, "import %s" % mod)
        except Exception as e:
            say(FAIL, "import %s — %s" % (mod, e), fix)
    for mod, fix in list(vision.items()):
        try:
            __import__(mod)
            say(PASS, "import %s" % mod)
        except Exception as e:
            say(WARN, "import %s — %s" % (mod, e),
                fix + "\n(mission_pi still serves the UI; detection/decode "
                      "are dead without it)")


def check_host(port):
    section("network / host")
    try:
        import bringup as bu
    except Exception as e:
        say(FAIL, "cannot import bringup.py (%r) — run from the mission_pi dir" % e)
        return []
    ips = bu.lan_ips()
    if ips:
        say(PASS, "LAN IPv4: %s" % ", ".join(ips))
        say(INFO, "paste into the UI's Pi-link box:  http://%s:%d" % (ips[0], port))
    else:
        say(FAIL, "no LAN IPv4 address found",
            "hostname -I   # empty? the laptop has no WiFi/cable link\n"
            "The UI on Windows cannot reach 127.0.0.1 of THIS machine.")
    # is something already on the port?
    if bu.port_free(port):
        say(WARN, "nothing is listening on port %d yet" % port,
            "start mission_pi first:  python3 main.py --config config.laptop.yaml")
    else:
        s = socket.socket()
        s.settimeout(2.0)
        try:
            s.connect(("127.0.0.1", port))
            say(PASS, "port %d has a listener (localhost connect ok)" % port)
        except Exception as e:
            say(FAIL, "port %d busy but not answering: %s" % (port, e),
                "ss -lptn 'sport = :%d'   # who owns it?" % port)
        finally:
            s.close()
        # bound to loopback only?  (the classic "works in curl, dead in browser")
        try:
            out = subprocess.run(["ss", "-ltnp"], capture_output=True,
                                 text=True, timeout=5).stdout
            for line in out.splitlines():
                if ":%d " % port in line or line.split()[3].endswith(":%d" % port):
                    addr = line.split()[3]
                    if addr.startswith("127.") or addr.startswith("[::1]:"):
                        say(FAIL, "port %d is bound to LOOPBACK only: %s" % (port, addr),
                            "server.host must be 0.0.0.0 (config.*.yaml already "
                            "sets it) — 127.0.0.1 locks out every other machine.")
                    else:
                        say(PASS, "port %d bound on %s (reachable from the LAN)"
                            % (port, addr))
        except Exception:
            pass
    return ips


def check_firewall(port):
    section("firewall")
    blocked = None
    try:
        out = subprocess.run(["ufw", "status"], capture_output=True, text=True,
                             timeout=5).stdout
        if "Status: active" in out:
            if ("%d/tcp" % port) in out or ("%d " % port) in out:
                say(PASS, "ufw active and port %d allowed" % port)
            else:
                blocked = "sudo ufw allow %d/tcp" % port
                say(FAIL, "ufw is ACTIVE and does not list port %d" % port, blocked)
        else:
            say(INFO, "ufw inactive (nothing blocked by it)")
    except Exception:
        say(INFO, "ufw not available here")
    try:
        out = subprocess.run(["firewall-cmd", "--state"], capture_output=True,
                             text=True, timeout=5).stdout.strip()
        if out == "running":
            ports = subprocess.run(["firewall-cmd", "--list-ports"],
                                   capture_output=True, text=True,
                                   timeout=5).stdout
            if str(port) in ports:
                say(PASS, "firewalld running, port %d open" % port)
            else:
                say(FAIL, "firewalld running, port %d NOT open" % port,
                    "sudo firewall-cmd --add-port=%d/tcp --permanent && "
                    "sudo firewall-cmd --reload" % port)
    except Exception:
        pass
    if blocked is None:
        say(INFO, "Windows side: Defender Firewall does not block OUTBOUND "
                  "browser connections — nothing to do there")


def check_sim(gz=True):
    section("simulator ports (SITL / Gazebo)")
    try:
        import bringup as bu
    except Exception:
        return
    ok, detail = bu.tcp_reachable("127.0.0.1", 5760, 1.0)
    say(PASS if ok else WARN, "tcp 5760 (SITL serial0, Mission Planner): %s"
        % detail,
        None if ok else "start SITL:  Tools/autotest/sim_vehicle.py -v ArduCopter "
                        "-f quad --no-mavproxy")
    ok2, detail2 = bu.tcp_reachable("127.0.0.1", 5762, 1.0)
    say(PASS if ok2 else WARN, "tcp 5762 (SITL serial1, mission_pi): %s" % detail2,
        None if ok2 else
        "serial1 only appears AFTER a GCS connects to 5760 — connect Mission\n"
        "Planner first. mission_pi retries on its own once it does.")
    if gz:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8099/health",
                                        timeout=2.0) as r:
                data = json.loads(r.read().decode())
            say(PASS, "gz camera bridge :8099 %s" % json.dumps(data)[:120])
        except Exception as e:
            say(INFO, "gz camera bridge :8099 not answering (%s)" % type(e).__name__,
                "only needed for the Gazebo run:  python3 tools/gz_cam_bridge.py "
                "--port 8099  (SYSTEM python3, not the venv)")


def check_self_http(port):
    section("this server over HTTP (what the browser does)")
    for path in ("/health", "/api/cameras", "/api/fsm/status", "/api/qr/status"):
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path),
                                        timeout=3.0) as r:
                body = r.read().decode("utf-8", "replace")
            say(PASS, "GET %s -> %d %s" % (path, r.status, body[:100].replace("\n", " ")))
            if path == "/health":
                try:
                    d = json.loads(body)
                    bu = d.get("bringup") or {}
                    if bu:
                        for k, v in sorted((bu.get("state") or {}).items()):
                            lvl = PASS if v in ("ok", "skipped") else (
                                WARN if v == "pending" else FAIL)
                            say(lvl, "  bring-up %-9s %s%s" % (
                                k, v,
                                ("  (%s)" % (bu.get("detail") or {}).get(k))
                                if (bu.get("detail") or {}).get(k) else ""))
                        for e in (bu.get("errors") or [])[-3:]:
                            say(FAIL, "  bring-up error [%s]: %s"
                                % (e.get("stage"), e.get("error")))
                    if d.get("link") is False:
                        say(WARN, "  FC link is DOWN (UI works, mission waits)",
                            "see the simulator section above")
                except Exception:
                    pass
        except urllib.error.HTTPError as e:
            say(FAIL, "GET %s -> HTTP %s %s" % (path, e.code, e.reason),
                "a 404 here usually means something ELSE owns port %d" % port)
        except Exception as e:
            say(FAIL, "GET %s -> %s: %s" % (path, type(e).__name__, e),
                "is mission_pi running?  python3 main.py --config config.laptop.yaml")
            return


# ------------------------------------------------------------ remote checks --
WS_KEY = base64.b64encode(os.urandom(16)).decode()


def _http_get(url, timeout=5.0):
    req = urllib.request.Request(url, headers={"User-Agent": "link_doctor/1.0"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read()
        return r.status, body, (time.time() - t0) * 1000


def _ws_recv(sock, timeout=6.0):
    """Read one WS text frame (client-side, server frames are unmasked)."""
    sock.settimeout(timeout)
    hdr = _readn(sock, 2)
    if hdr is None:
        return None
    op = hdr[0] & 0x0F
    ln = hdr[1] & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", _readn(sock, 2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", _readn(sock, 8))[0]
    data = _readn(sock, ln)
    if op == 8:
        return None
    return data.decode("utf-8", "replace") if op in (1,) else ""


def _readn(sock, n):
    buf = b""
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except Exception:
            return None
        if not chunk:
            return None
        buf += chunk
    return buf


def check_remote(target, ws_wait=6.0):
    section("remote target: %s" % target)
    url = target.rstrip("/")
    if "://" not in url:
        url = "http://" + url
    hostport = url.split("//", 1)[1].split("/", 1)[0]
    host, _, port = hostport.partition(":")
    port = int(port or 80)

    t0 = time.time()
    try:
        s = socket.create_connection((host, port), timeout=4.0)
        s.close()
        say(PASS, "TCP connect %s:%d in %.0f ms" % (host, port,
                                                    (time.time() - t0) * 1000))
    except Exception as e:
        say(FAIL, "TCP connect %s:%d — %s: %s" % (host, port, type(e).__name__, e),
            "Nothing is listening there. In order of likelihood:\n"
            "  1. mission_pi is not running on that machine (start it: "
            "python3 main.py ...)\n"
            "  2. wrong IP — on the Pi/Linux laptop run: hostname -I  (or "
            "python3 tools/link_doctor.py)\n"
            "  3. firewall on the Linux box — sudo ufw allow %d/tcp\n"
            "  4. the two machines are on different networks/APs (phone "
            "hotspot vs WiFi)." % port)
        return

    for path in ("/health", "/api/cameras", "/api/fsm/status"):
        try:
            status, body, ms = _http_get(url + path)
            text = body.decode("utf-8", "replace")
            say(PASS, "GET %s -> %d in %.0f ms" % (path, status, ms))
            if path == "/health":
                try:
                    d = json.loads(text)
                    say(INFO, "  mission_pi: link=%s cams=%s ws_clients=%s "
                              "bring-up=%s" % (
                                  d.get("link"),
                                  list((d.get("cams") or {}).keys()),
                                  d.get("ws_clients"),
                                  ((d.get("bringup") or {}).get("stage"))))
                    for e in ((d.get("bringup") or {}).get("errors") or [])[-3:]:
                        say(WARN, "  bring-up error [%s]: %s"
                            % (e.get("stage"), e.get("error")))
                except Exception:
                    say(WARN, "  /health answered but is not mission_pi JSON: %s"
                        % text[:120],
                        "Something else owns port %d on that machine — the UI "
                        "would connect to the wrong server." % port)
        except urllib.error.HTTPError as e:
            say(FAIL, "GET %s -> HTTP %s" % (path, e.code),
                "HTTP answered, so a server IS there — but it is not serving "
                "mission_pi's routes (wrong port? old checkout? `git pull`).")
        except Exception as e:
            say(FAIL, "GET %s -> %s: %s" % (path, type(e).__name__, e))

    # the WebSocket the UI actually uses
    section("websocket %s/ws/telemetry (the UI's Pi link)" % url)
    try:
        s = socket.create_connection((host, port), timeout=4.0)
        req = ("GET /ws/telemetry HTTP/1.1\r\n"
               "Host: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n"
               "Origin: http://127.0.0.1:8100\r\n\r\n" % (host, port, WS_KEY))
        s.sendall(req.encode())
        s.settimeout(5.0)
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = s.recv(1024)
            if not chunk:
                break
            head += chunk
        line = head.split(b"\r\n", 1)[0].decode("utf-8", "replace")
        if "101" not in line:
            say(FAIL, "handshake rejected: %s" % line,
                "A proxy or the wrong service answered. The UI's Pi lamp stays "
                "red with this.")
            s.close()
            return
        accept = ""
        for h in head.split(b"\r\n"):
            if h.lower().startswith(b"sec-websocket-accept:"):
                accept = h.split(b":", 1)[1].strip().decode()
        want = base64.b64encode(hashlib.sha1(
            (WS_KEY + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        ).decode()
        say(PASS if accept == want else WARN,
            "handshake 101 (%s)" % ("accept verified" if accept == want
                                    else "accept mismatch"))
        # a server may have pipelined frames right after the handshake
        rest = head.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in head else b""
        got, t_end = [], time.time() + ws_wait
        buf = rest
        while time.time() < t_end:
            while True:
                frame = _frame_from(buf)
                if frame is None:
                    break
                text, buf = frame
                if text:
                    got.append(text)
            if len(got) >= 4:
                break
            try:
                s.settimeout(max(0.2, t_end - time.time()))
                chunk = s.recv(65536)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
        s.close()
        if got:
            chans = []
            for g in got[:8]:
                try:
                    chans.append(json.loads(g).get("channel"))
                except Exception:
                    chans.append("?")
            say(PASS, "%d envelope(s) in %.1fs: %s" % (len(got), ws_wait,
                                                       ", ".join(map(str, chans))))
            say(INFO, "  first: %s" % got[0][:160])
        else:
            say(WARN, "socket opened but no envelope in %.1fs" % ws_wait,
                "The Pi pushes `system` every 2 s and replays state on connect,\n"
                "so silence means an old mission_pi (git pull + restart) or a\n"
                "proxy that buffers websockets.")
    except Exception as e:
        say(FAIL, "websocket: %s: %s" % (type(e).__name__, e),
            "HTTP worked but WS did not — a proxy/VPN/filter in between, or "
            "the server is too old to have /ws/telemetry.")


def _frame_from(buf):
    """(text, remaining) for one complete unmasked server frame, else None."""
    if len(buf) < 2:
        return None
    op = buf[0] & 0x0F
    ln = buf[1] & 0x7F
    off = 2
    if ln == 126:
        if len(buf) < 4:
            return None
        ln = struct.unpack(">H", buf[2:4])[0]
        off = 4
    elif ln == 127:
        if len(buf) < 10:
            return None
        ln = struct.unpack(">Q", buf[2:10])[0]
        off = 10
    if len(buf) < off + ln:
        return None
    payload = buf[off:off + ln]
    rest = buf[off + ln:]
    if op == 1:
        return payload.decode("utf-8", "replace"), rest
    if op == 8:
        return None
    return "", rest


def main(argv=None):
    ap = argparse.ArgumentParser(description="diagnose the mission_ui <-> "
                                             "mission_pi link")
    ap.add_argument("--target", default=None,
                    help="check a remote Pi (run this on the Windows laptop): "
                         "http://<pi-ip>:8000")
    ap.add_argument("--port", type=int, default=8000,
                    help="local mission_pi port (default 8000)")
    ap.add_argument("--config", default="config.yaml",
                    help="config to read server.port / fc.conn from")
    ap.add_argument("--ws-wait", type=float, default=6.0,
                    help="seconds to wait for WS envelopes (default 6)")
    ap.add_argument("--no-sim", action="store_true",
                    help="skip the SITL/Gazebo port checks")
    ap.add_argument("--quick", action="store_true",
                    help="deps + host + firewall only")
    args = ap.parse_args(argv)

    print("link_doctor — mission_ui <-> mission_pi  (%s, python %s)"
          % (time.strftime("%Y-%m-%d %H:%M:%S"), sys.version.split()[0]))

    if args.target:
        check_remote(args.target, ws_wait=args.ws_wait)
    else:
        cfg = {}
        try:
            import yaml
            if os.path.isfile(args.config):
                with open(args.config) as f:
                    cfg = yaml.safe_load(f) or {}
                print("config: %s" % args.config)
        except Exception:
            pass
        port = int((cfg.get("server") or {}).get("port", args.port))
        check_deps()
        check_host(port)
        check_firewall(port)
        if not args.quick:
            check_self_http(port)
            if not args.no_sim:
                check_sim()
        fc_conn = (cfg.get("fc") or {}).get("conn")
        if fc_conn:
            section("fc.conn from config")
            say(INFO, "fc.conn = %s" % fc_conn)

    fails = [m for lvl, m in _results if lvl == FAIL]
    warns = [m for lvl, m in _results if lvl == WARN]
    print("\n== summary ==  %d checks, %d FAIL, %d WARN"
          % (len(_results), len(fails), len(warns)))
    if fails:
        print("Fix the FAIL lines above first — they are the reason the UI "
              "cannot connect.")
    elif warns:
        print("No hard failure: the link should work. WARN lines are worth a "
              "look (order of operations, missing sim).")
    else:
        print("Everything green.")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
