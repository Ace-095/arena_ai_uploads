"""Bring-up state + host facts for the Pi link (stdlib only).

Why this module exists (2026-09-14 bug hunt):
  `main.py` used to connect the FC *first* and start the HTTP/WS server
  *last*. Any bring-up failure — SITL not up yet, serial1 (5762) not
  listening until Mission Planner connects to 5760, missing cv2/pyzbar,
  a camera that refuses to open — killed the process with a traceback and
  the UI on the laptop got `connection refused` on http://<ip>:8000 with
  nothing to explain why. The Pi link must be the FIRST thing up and the
  LAST thing down: it is the only window the operator has into the box.

So:
  * `Bringup`  — thread-safe stage/error ledger. The server publishes it at
    GET /health and pushes it over the WS `log`/`system` channels, so a
    browser can tell "mission_pi is not running" from "mission_pi is up but
    the FC link is still retrying".
  * `lan_urls()` — the exact strings to paste into the UI. `<this-pi>` in a
    log line is not an address anybody can click; the real IPv4s are.
  * `system_stats()` — the `system` WS channel the UI contract expects
    (temp_c / cpu_pct / mem_pct / disk_pct / uptime_s), read from /proc on
    Linux and best-effort elsewhere. Doubles as the WS keepalive.

No third-party imports: this must work on a bare Pi before `pip install`.
"""
import logging
import os
import shutil
import socket
import subprocess
import threading
import time

log = logging.getLogger("bringup")

STAGES = ("init", "server", "detector", "cameras", "streams", "fc", "ready")


# ---------------------------------------------------------------- host IPs --
def _outbound_ip(probe=("8.8.8.8", 53)):
    """The IP this host would use to reach the LAN/Internet (no packets
    sent — a UDP connect only consults the routing table)."""
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.4)
        s.connect(probe)
        return s.getsockname()[0]
    except Exception:
        return None
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:
                pass


def _is_usable(ip):
    if not ip or ":" in ip:
        return False
    if ip.startswith("127.") or ip.startswith("169.254.") or ip == "0.0.0.0":
        return False
    return True


def lan_ips():
    """Every non-loopback IPv4 on this box, best guess first.

    Order: the routed outbound IP, then `hostname -I` / `ip addr`, then
    gethostbyname_ex. A laptop on WiFi + docker0 + a VPN gives several —
    the UI wants the one the Windows machine can reach, which is the
    routed one, so it goes first.
    """
    found = []

    def add(ip):
        if _is_usable(ip) and ip not in found:
            found.append(ip)

    add(_outbound_ip())
    for cmd in (["hostname", "-I"], ["ip", "-o", "-4", "addr", "show"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=3).stdout
        except Exception:
            continue
        for tok in out.replace("/", " ").split():
            parts = tok.split(".")
            if len(parts) == 4 and all(p.isdigit() and 0 <= int(p) <= 255
                                       for p in parts):
                add(tok)
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            add(ip)
    except Exception:
        pass
    return found


def lan_urls(port=8000, path=""):
    """Paste-ready URLs for the laptop UI, e.g. http://192.168.1.42:8000."""
    return ["http://%s:%d%s" % (ip, int(port), path) for ip in lan_ips()]


def port_free(port, host="0.0.0.0"):
    """True when nothing is listening on `port` yet (a stale mission_pi from
    the last run is the classic reason the UI reaches the WRONG process)."""
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, int(port)))
        return True
    except OSError:
        return False
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:
                pass


def tcp_reachable(host, port, timeout=1.5):
    """(ok, detail) for a plain TCP connect — used by tools/link_doctor.py."""
    t0 = time.time()
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True, "%.0f ms" % ((time.time() - t0) * 1000)
    except Exception as e:
        return False, "%s: %s" % (type(e).__name__, e)


# ------------------------------------------------------------ system stats --
_CPU_PREV = None
_CPU_LOCK = threading.Lock()


def _cpu_pct_linux():
    """% busy since the previous call (two /proc/stat samples)."""
    global _CPU_PREV
    try:
        with open("/proc/stat") as f:
            parts = f.readline().split()
        vals = [int(x) for x in parts[1:]]
    except Exception:
        return None
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    total = sum(vals)
    with _CPU_LOCK:
        prev = _CPU_PREV
        _CPU_PREV = (idle, total)
    if not prev:
        return None
    dt = total - prev[1]
    if dt <= 0:
        return None
    return round(100.0 * (1.0 - (idle - prev[0]) / float(dt)), 1)


def _temp_c_linux():
    """SoC temperature: the Pi's thermal zone, else any zone that reads sane."""
    try:
        zones = sorted(os.listdir("/sys/class/thermal"))
    except Exception:
        return None
    for z in zones:
        if z.startswith("thermal_zone") and "cpu" not in z and "soc" not in z:
            continue
        try:
            with open("/sys/class/thermal/%s/temp" % z) as f:
                v = float(f.read().strip()) / 1000.0
            if -20.0 < v < 150.0:
                return round(v, 1)
        except Exception:
            continue
    for z in zones:
        try:
            with open("/sys/class/thermal/%s/temp" % z) as f:
                v = float(f.read().strip()) / 1000.0
            if -20.0 < v < 150.0:
                return round(v, 1)
        except Exception:
            continue
    return None


def _mem_linux():
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, rest = line.partition(":")
                info[k.strip()] = float(rest.split()[0])  # kB
        total = info.get("MemTotal", 0.0)
        avail = info.get("MemAvailable",
                         info.get("MemFree", 0.0) + info.get("Cached", 0.0))
        if total > 0:
            return round(100.0 * (1.0 - avail / total), 1)
    except Exception:
        pass
    return None


def system_stats(root="/"):
    """UI `system` channel payload: {temp_c, cpu_pct, mem_pct, disk_pct,
    uptime_s}. Every field is None when this host cannot measure it — the
    UI already renders `—` for missing values."""
    cpu = _cpu_pct_linux()
    mem = _mem_linux()
    temp = _temp_c_linux()
    if cpu is None or mem is None:
        try:
            import psutil  # optional, never required
            if cpu is None:
                cpu = psutil.cpu_percent(interval=None)
            if mem is None:
                mem = psutil.virtual_memory().percent
            if temp is None:
                t = psutil.sensors_temperatures() or {}
                for _k, entries in t.items():
                    if entries:
                        temp = round(float(entries[0].current), 1)
                        break
        except Exception:
            pass
    disk = None
    try:
        u = shutil.disk_usage(root)
        disk = round(100.0 * u.used / float(u.total), 1)
    except Exception:
        pass
    if cpu is None:
        try:  # BSD/macOS: no /proc/stat — load average is the honest proxy
            la = os.getloadavg()
            cpu = round(100.0 * la[0] / max(1, os.cpu_count() or 1), 1)
        except Exception:
            pass
    uptime = None
    try:
        with open("/proc/uptime") as f:
            uptime = round(float(f.read().split()[0]), 1)
    except Exception:
        pass
    return {"temp_c": temp, "cpu_pct": cpu, "mem_pct": mem,
            "disk_pct": disk, "uptime_s": uptime,
            "host": socket.gethostname(), "pid": os.getpid()}


# --------------------------------------------------------------- ledger ----
class Bringup:
    """What is up, what is still coming, and what broke on the way.

    Written by the bring-up thread, read by the HTTP server (and therefore
    by the operator's browser) — hence the lock and the JSON-safe snapshot.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self.t0 = time.time()
        self.stage = "init"
        self.state = {           # per-subsystem: pending|ok|failed|skipped
            "server": "pending", "detector": "pending",
            "cameras": "pending", "streams": "pending", "fc": "pending",
        }
        self.detail = {}         # subsystem -> human string
        self.errors = []         # [{stage, error, ts}] — newest last, capped
        self.urls = []           # paste-ready Pi URLs
        self.done = False

    # -- writers ---------------------------------------------------------
    def mark(self, subsystem, state, detail=None):
        with self._lock:
            self.state[subsystem] = state
            if detail is not None:
                self.detail[subsystem] = str(detail)
            self.stage = self._stage_locked()

    def note(self, subsystem, detail):
        with self._lock:
            self.detail[subsystem] = str(detail)

    def fail(self, subsystem, exc, fatal=False):
        """Record a failure. Never raises — a broken subsystem must not take
        the Pi link down with it."""
        msg = "%s: %s" % (type(exc).__name__, exc)
        with self._lock:
            self.state[subsystem] = "failed"
            self.detail[subsystem] = msg
            self.errors.append({"stage": subsystem, "error": msg,
                                "fatal": bool(fatal), "ts": time.time()})
            del self.errors[:-20]
            self.stage = self._stage_locked()
        log.error("bring-up %s FAILED — %s", subsystem, msg)
        return msg

    def set_urls(self, urls):
        with self._lock:
            self.urls = list(urls)

    def finish(self):
        with self._lock:
            self.done = True
            self.stage = self._stage_locked()

    def _stage_locked(self):
        # a failed subsystem wins over `done`: bring-up can finish (we gave up
        # retrying) while the box is still not usable, and /health must not
        # claim "ready" when the FC or a camera never came up.
        for s in STAGES:
            if s in ("init", "ready"):
                continue
            if self.state.get(s) == "failed":
                return s
        if self.done:
            return "ready"
        for s in STAGES:
            if s in ("init", "ready"):
                continue
            if self.state.get(s, "pending") == "pending":
                return s
        if all(v in ("ok", "skipped") for v in self.state.values()):
            return "ready"
        # something failed: stay on the subsystem that broke so /health and
        # the UI point at it instead of pretending we are ready.
        for s in STAGES:
            if self.state.get(s) == "failed":
                return s
        return self.stage

    # -- readers ---------------------------------------------------------
    def snapshot(self):
        with self._lock:
            return {"stage": self.stage, "state": dict(self.state),
                    "detail": dict(self.detail), "errors": list(self.errors),
                    "urls": list(self.urls), "done": self.done,
                    "age_s": round(time.time() - self.t0, 1)}

    def ok(self):
        with self._lock:
            return all(v in ("ok", "skipped") for v in self.state.values())

    def one_line(self):
        with self._lock:
            bits = ["%s=%s" % (k, v) for k, v in self.state.items()
                    if k != "server"]
            return "bring-up[%s] %s" % (self.stage, " ".join(bits))
