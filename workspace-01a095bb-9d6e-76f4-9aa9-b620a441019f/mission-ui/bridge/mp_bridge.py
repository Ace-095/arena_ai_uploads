#!/usr/bin/env python3
"""
mp_bridge.py — Mission Companion: UI host + Mission Planner log bridge (+ mock Pi).

Runs on the ground laptop (Windows 11) AND on any dev machine.
STDLIB ONLY — no pip installs. (pymavlink is optional, only for .bin parsing.)

    python mp_bridge.py                 # UI + MP log bridge on :8100
    python mp_bridge.py --mock          # + simulated Pi (full api-contract.md demo)
    python mp_bridge.py --watch "C:\\path\\to\\mp\\log" [--port 8100] [--verbose]

Why this exists (ui-spec.md §4b):
  The only Pi->laptop path that needs NO router/LTE is Pi -> Pixhawk (USB) ->
  telemetry radio -> Mission Planner. So the bridge tails MP's logs, extracts the
  decoded QR message (`QR:<payload>` STATUSTEXT) and pushes it to the UI locally.
  Routes of delivery shown in the UI: PI-WS / MP-LOG / MANUAL.
"""

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import re
import struct
import threading
import time

# ---------------------------------------------------------------------------
# Optional: pymavlink DFReader for .bin (dataflash) log parsing
# ---------------------------------------------------------------------------
try:
    from pymavlink import DFReader as _DFReader
    HAVE_DFR = True
except Exception:
    HAVE_DFR = False

QR_RE = re.compile(r"QR[:\s]+([0-9A-Za-z]{1,6})\b")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # mission-ui/
STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}
STATUS_TEXT = {200: "OK", 204: "No Content", 400: "Bad Request", 404: "Not Found",
               405: "Method Not Allowed", 500: "Internal Server Error", 503: "Service Unavailable"}

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _sse_chunk(name: str, obj) -> bytes:
    body = ("event: %s\ndata: %s\n\n" % (name, json.dumps(obj, separators=(",", ":"), default=str))).encode()
    return ("%X\r\n" % len(body)).encode() + body + b"\r\n"


# ---------------------------------------------------------------------------
# Hub — fan-out to SSE + WS clients
# ---------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.loop = None
        self.clients = {}          # id -> {"kind": "sse"|"ws", "writer": w}
        self.lock = asyncio.Lock()

    async def add(self, kind, writer):
        cid = id(writer)
        async with self.lock:
            self.clients[cid] = {"kind": kind, "writer": writer}
        return cid

    async def remove(self, cid):
        async with self.lock:
            self.clients.pop(cid, None)

    async def emit(self, event, data):
        await self._emit_now(event, data)

    def emit_threadsafe(self, event, data):
        if self.loop is None or not self.loop.is_running():
            return
        try:
            asyncio.run_coroutine_threadsafe(self.emit(event, data), self.loop)
        except Exception:
            pass

    async def _emit_now(self, event, data):
        sse_frame = _sse_chunk(event, data)
        # WS clients expect the contract envelope {channel, t, data} (api-contract.md)
        env = {"channel": event, "t": time.time(), "data": data}
        ws_frame = ws_text_frame(json.dumps(env, separators=(",", ":"), default=str).encode())
        dead = []
        async with self.lock:
            clients = list(self.clients.items())
        for cid, c in clients:
            try:
                c["writer"].write(sse_frame if c["kind"] == "sse" else ws_frame)
                await c["writer"].drain()
            except Exception:
                dead.append(cid)
        for cid in dead:
            await self.remove(cid)


# ---------------------------------------------------------------------------
# Minimal WebSocket server frames (RFC6455: text + ping/close)
# ---------------------------------------------------------------------------
def ws_text_frame(payload: bytes) -> bytes:
    ln = len(payload)
    hdr = bytes([0x81])
    if ln < 126:
        hdr += bytes([ln])
    elif ln < 65536:
        hdr += bytes([126]) + struct.pack(">H", ln)
    else:
        hdr += bytes([127]) + struct.pack(">Q", ln)
    return hdr + payload


async def ws_read_frame(reader):
    b1b2 = await reader.readexactly(2)
    opcode = b1b2[0] & 0x0F
    masked = b1b2[1] & 0x80
    ln = b1b2[1] & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", await reader.readexactly(2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", await reader.readexactly(8))[0]
    mask = await reader.readexactly(4) if masked else b""
    data = await reader.readexactly(ln) if ln else b""
    if masked and data:
        data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
    return opcode, data


# ---------------------------------------------------------------------------
# MP log tailer (thread)
# ---------------------------------------------------------------------------
class LogTailer(threading.Thread):
    def __init__(self, hub: Hub):
        super().__init__(daemon=True, name="mp-tailer")
        self.hub = hub
        self.watch = []
        self.text_off = {}
        self.bin_state = {}
        self.last_qr = (None, 0.0)

    def add_path(self, p) -> bool:
        p = (p or "").strip().strip('"')
        if not p or p in self.watch:
            return False
        self.watch.append(p)
        return True

    def _emit_qr(self, payload, source, line):
        payload = payload.strip()
        now = time.time()
        if self.last_qr[0] == payload and now - self.last_qr[1] < 10:
            return
        self.last_qr = (payload, now)
        data = {"payload": payload, "source": source, "ts": now, "line": line[:300]}
        self.hub.emit_threadsafe("mp-qr", data)

    def _read_text(self, path):
        try:
            if not os.path.isfile(path):
                return
            st = os.stat(path)
            off = self.text_off.get(path)
            if off is None:
                self.text_off[path] = st.st_size      # new file: tail from EOF
                return
            if st.st_size < off:
                off = 0                               # truncated/rotated
            if st.st_size == off:
                return
            with open(path, "rb") as f:
                f.seek(off)
                data = f.read()
            self.text_off[path] = off + len(data)
            for line in data.decode("utf-8", "replace").splitlines():
                if not line.strip():
                    continue
                self.hub.emit_threadsafe("mp-line", {"line": line[:500], "ts": time.time(),
                                                     "source": os.path.basename(path)})
                m = QR_RE.search(line)
                if m:
                    self._emit_qr(m.group(1), os.path.basename(path), line)
        except OSError:
            pass

    def _scan_bin(self, path):
        if not HAVE_DFR:
            return
        try:
            st = os.stat(path)
            last_size, last_count = self.bin_state.get(path, (0, 0))
            if st.st_size < last_size:
                self.bin_state[path] = (st.st_size, 0)
                return
            if st.st_size == last_size:
                return
            r = _DFReader.Reader(str(path))
            matches = []
            for msg in r.itermessages():
                if msg.getType() == "MESSAGE":
                    text = getattr(msg, "Message", "") or ""
                    if "QR:" in text:
                        matches.append(text)
            for text in matches[last_count:]:
                m = QR_RE.search(text)
                if m:
                    self._emit_qr(m.group(1), os.path.basename(path), text)
            self.bin_state[path] = (st.st_size, len(matches))
        except Exception:
            pass

    def run(self):
        while True:
            for p in list(self.watch):
                if os.path.isdir(p):
                    try:
                        for name in os.listdir(p):
                            full = os.path.join(p, name)
                            ext = os.path.splitext(name)[1].lower()
                            if ext in (".txt", ".log"):
                                self._read_text(full)
                            elif ext == ".bin":
                                self._scan_bin(full)
                    except OSError:
                        pass
                else:
                    if os.path.splitext(p)[1].lower() == ".bin":
                        self._scan_bin(p)
                    else:
                        self._read_text(p)
            time.sleep(0.7)


# ---------------------------------------------------------------------------
# Geo helpers (flat-earth ENU — same math as ui map.js)
# ---------------------------------------------------------------------------
def latlon_to_enu(lat, lon, olat, olon):
    dlat = math.radians(lat - olat)
    dlon = math.radians(lon - olon)
    return (dlon * 6371000.0 * math.cos(math.radians(olat)), dlat * 6371000.0)


def enu_to_latlon(x, y, olat, olon):
    return (olat + math.degrees(y / 6371000.0),
            olon + math.degrees(x / (6371000.0 * math.cos(math.radians(olat)))))


def point_in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


# ---------------------------------------------------------------------------
# Mock Pi — full api-contract.md simulation (demo + UI development target)
# ---------------------------------------------------------------------------
MOCK_ORIGIN = (15.3697, 75.1235)   # Hubballi — demo only, NOT a venue
MOCK_PAYLOAD = "42"                 # 2-digit payload (confirmed spec)


def _qrbits(n=21):
    """Deterministic fake-QR bit grid (looks like a QR code, decodes nowhere)."""
    grid = []
    for r in range(n):
        row = []
        for c in range(n):
            if (r < 7 and c < 7) or (r < 7 and c >= n - 7) or (r >= n - 7 and c < 7):
                rr = r if r < 7 else r - (n - 7)
                cc = c if c < 7 else c - (n - 7)
                row.append(1 if max(abs(rr - 3), abs(cc - 3)) in (1, 2, 3) else 0)
            else:
                row.append(1 if ((r * 31 + c * 17 + 7) % 7) < 3 else 0)
        grid.append(row)
    return grid


def render_cam_svg(m) -> bytes:
    """Dynamic SVG stand-in camera frame (the real Pi serves JPEG)."""
    w, h = 320, 240
    t = time.time()
    p = ['<?xml version="1.0" encoding="UTF-8"?>',
         '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">' % (w, h, w, h),
         '<rect width="%d" height="%d" fill="#0b0f14"/>' % (w, h)]
    for gx in range(0, w, 40):
        p.append('<line x1="%d" y1="0" x2="%d" y2="%d" stroke="#161d26" stroke-width="1"/>' % (gx, gx, h))
    for gy in range(0, h, 40):
        p.append('<line x1="0" y1="%d" x2="%d" y2="%d" stroke="#161d26" stroke-width="1"/>' % (gy, w, gy))
    p.append('<line x1="%d" y1="0" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1" stroke-dasharray="4 4" opacity="0.6"/>' % (w // 2, w // 2, h))
    p.append('<line x1="0" y1="%d" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1" stroke-dasharray="4 4" opacity="0.6"/>' % (h // 2, w, h // 2))

    if m.phase in ("TARGET_FOUND", "DESCEND", "DECODED", "TRANSMIT"):
        prog = {"TARGET_FOUND": 0.25,
                "DESCEND": 0.25 + 0.55 * min(1.0, m.phase_t / 9.0)}.get(m.phase, 0.8)
        size = int(30 + 100 * prog)
        x0, y0 = w // 2 - size // 2, h // 2 - size // 2
        cell = max(2, size // 21)
        grid = _qrbits(21)
        for r in range(21):
            for c in range(21):
                if grid[r][c]:
                    p.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#000"/>'
                             % (x0 + c * cell, y0 + r * cell, cell, cell))
        p.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#dcdcdc" stroke-width="2"/>'
                 % (x0 - 2, y0 - 2, size + 4, size + 4))
        if m.qr["confirmed"]:
            p.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#34d17c" stroke-width="3"/>'
                     % (x0 - 6, y0 - 6, size + 12, size + 12))
            p.append('<text x="%d" y="%d" fill="#34d17c" font-family="monospace" font-size="16" '
                     'text-anchor="middle">DECODED: %s</text>' % (w // 2, y0 - 14, m.qr["payload"]))
        elif m.qr["streak"]:
            p.append('<text x="%d" y="%d" fill="#ffb020" font-family="monospace" font-size="12" '
                     'text-anchor="middle">streak %d/3</text>' % (w // 2, y0 - 10, m.qr["streak"]))

    p.append('<text x="8" y="16" fill="#52c7e0" font-family="monospace" font-size="11">MOCK cam1 · Pi Cam 3 (sim)</text>')
    p.append('<text x="8" y="30" fill="#7b8794" font-family="monospace" font-size="10">'
             'phase=%s alt=%.1fm exp=%dus</text>' % (m.phase, m.alt, m.controls["exposure_us"]))
    p.append('<text x="%d" y="%d" fill="#7b8794" font-family="monospace" font-size="10">%.2fs</text>'
             % (w - 52, h - 8, t % 1000))
    p.append('</svg>')
    return "".join(p).encode()


class MockPi:
    def __init__(self, hub: Hub):
        self.hub = hub
        self.t0 = time.time()
        self.phase, self.prev_phase = "IDLE", None
        self.phase_t = 0.0
        self.armed, self.mode, self.alt = False, "STANDBY", 0.0
        self.pos = list(MOCK_ORIGIN)
        self.fence = None
        self.board_enu = self._default_board()
        self.qr = {"streak": 0, "required": 3, "payload": None, "confirmed": False,
                   "bbox": None, "source": "cam1"}
        self.controls = {"exposure_us": 8000, "gain": 1.5, "af_mode": "continuous",
                         "brightness": 0.1, "contrast": 1.2, "saturation": 1.0,
                         "sharpness": 1.2, "adaptive": True}
        self.path, self.path_i = [], 0
        self._target_seen = False
        self._result_sent = False
        self._last_sys = 0.0
        self._qr_tick = -10.0

    def _default_board(self):
        return (12.0, 8.0)

    # -- emissions ---------------------------------------------------------
    async def log(self, level, msg):
        await self.hub.emit("log", {"level": level, "msg": msg, "ts": time.time()})
        print("[mock] %s: %s" % (level, msg))

    async def event(self, etype, value):
        await self.hub.emit("event", {"type": etype, "value": value, "ts": time.time()})

    async def emit_fsm(self):
        await self.hub.emit("fsm", {
            "state": self.phase, "previous": self.prev_phase,
            "reason": getattr(self, "_reason", ""), "armable": self.armed,
            "link_healthy": True, "plan": {"items": 3, "trigger_seq": 2, "synced": self.phase != "IDLE"},
            "payload": self.qr["payload"] if self.qr["confirmed"] else None, "ts": time.time()})

    async def set_phase(self, p, reason=""):
        self.prev_phase, self.phase, self.phase_t = self.phase, p, 0.0
        self._reason = reason
        await self.log("INFO", "FSM: %s -> %s%s" % (self.prev_phase, p, " | " + reason if reason else ""))
        await self.event("fsm_state", {"from": self.prev_phase, "to": p, "reason": reason})
        await self.emit_fsm()

    async def emit_telemetry(self):
        await self.hub.emit("telemetry", {
            "lat": round(self.pos[0], 7), "lon": round(self.pos[1], 7),
            "alt_rel": round(self.alt, 2), "alt_msl": round(self.alt + 340.0, 1),
            "vx": 0.0, "vy": 0.0, "vz": 0.0, "hdg": 90.0,
            "attitude": {"roll": 0.5 * math.sin(time.time()), "pitch": 0.3 * math.cos(time.time() * 0.7), "yaw": 90.0},
            "mode": self.mode, "armed": self.armed,
            "gps": {"fix": 3, "sats": 12},
            "battery": {"v": round(15.6 - 0.001 * (time.time() - self.t0), 2), "pct": 96},
            "ts": time.time()})

    async def emit_system(self):
        t = time.time() - self.t0
        await self.hub.emit("system", {
            "cpu": round(25 + 15 * math.sin(t / 7) + 3 * math.sin(t * 1.3), 1),
            "ram": 38.0, "ram_used_mb": 2050, "ram_total_mb": 4096, "disk": 41.0,
            "temp_c": round(51 + 2 * math.sin(t / 20), 1), "freq_mhz": 1800, "uptime_s": round(t, 1),
            "network": {"router": "up", "lte": "down"}, "ts": time.time()})

    async def emit_qr(self):
        await self.hub.emit("qr", {**self.qr, "ts": time.time()})

    # -- search path ---------------------------------------------------------
    def _build_search_path(self):
        ox, oy = MOCK_ORIGIN
        if self.fence and self.fence.get("vertices_latlon"):
            olat, olon = self.fence["origin_lat"], self.fence["origin_lon"]
            poly = [latlon_to_enu(v["lat"], v["lon"], olat, olon) for v in self.fence["vertices_latlon"]]
            cx, cy = self.fence["centroid_enu"]["x"], self.fence["centroid_enu"]["y"]
            radius = min(self.fence.get("max_radius_m") or 20.0, 30.0)
            self.board_enu = (cx + 0.25 * radius, cy + 0.2 * radius)
            base = (cx, cy)
        else:
            poly = []
            radius = 20.0
            base = (0.0, 0.0)
            self.board_enu = self._default_board()
        step = 1.5
        path = []
        x = y = 0.0
        for k in range(1, int(radius / step) + 1):
            for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                for _ in range(k):
                    x += dx * step
                    y += dy * step
                    px, py = base[0] + x, base[1] + y
                    if math.hypot(x, y) <= radius + step and (not poly or point_in_poly(px, py, poly)):
                        path.append((px, py))
        if not path:
            path = [(base[0] + i * step, base[1]) for i in range(1, 10)]
        self.path, self.path_i = path, 0
        return path

    # -- reset / abort ---------------------------------------------------------
    def reset(self):
        self.phase, self.prev_phase, self.phase_t = "IDLE", None, 0.0
        self.armed, self.mode, self.alt = False, "STANDBY", 0.0
        self.pos = list(MOCK_ORIGIN)
        self.qr = {"streak": 0, "required": 3, "payload": None, "confirmed": False,
                   "bbox": None, "source": "cam1"}
        self._target_seen = False
        self._result_sent = False
        self._qr_tick = -10.0

    # -- main sim loop ---------------------------------------------------------
    async def run(self):
        await self.log("INFO", "MOCK Pi started (demo origin: Hubballi — NOT a venue)")
        await self.emit_system()
        await asyncio.sleep(2.0)
        await self.set_phase("PLAN_SYNC", "boot: syncing plan from FC")
        while True:
            dt = 0.2
            self.phase_t += dt
            t = self.phase_t
            ox, oy = MOCK_ORIGIN

            if self.phase == "IDLE":
                pass
            elif self.phase == "PLAN_SYNC" and t >= 3.0:
                await self.event("plan_synced", {"items": 3, "trigger_seq": 2})
                await self.log("INFO", "Plan synced: 3 items, DO_SPRAYER trigger at seq 2")
                self.armed, self.mode, self.alt, self.pos = True, "AUTO", 14.0, [ox, oy]
                await self.set_phase("WAITING_TRIGGER", "in AUTO — waiting for DO_SPRAYER trigger")
            elif self.phase == "WAITING_TRIGGER" and t >= 6.0:
                await self.log("INFO", "TRIGGER: MISSION_CURRENT.seq == 2 (DO_SPRAYER) — PI handover")
                self.mode = "GUIDED"
                self._target_seen = False
                self._build_search_path()
                await self.set_phase("SEARCH", "GUIDED — sweeping search area (fence polygon)")
            elif self.phase == "SEARCH":
                if self.path and self.path_i < len(self.path):
                    tx, ty = self.path[self.path_i]
                    px, py = latlon_to_enu(self.pos[0], self.pos[1], ox, oy)
                    dx, dy = tx - px, ty - py
                    d = math.hypot(dx, dy)
                    if d < 0.5:
                        self.path_i += 1
                    elif d > 0:
                        self.pos = enu_to_latlon(px + dx / d * 0.3, py + dy / d * 0.3, ox, oy)
                if not self._target_seen and (t >= 12.0 or self.path_i >= len(self.path)):
                    self._target_seen = True
                    await self.event("target_detected",
                                     {"bbox": {"x": 150, "y": 110, "w": 40, "h": 55}, "confidence": 0.93})
                    await self.log("INFO", "TARGET: A3 QR board detected — flying over it")
                    await self.set_phase("TARGET_FOUND", "approach to over-target point")
            elif self.phase == "TARGET_FOUND":
                bx, by = self.board_enu
                px, py = latlon_to_enu(self.pos[0], self.pos[1], ox, oy)
                dx, dy = bx - px, by - py
                d = math.hypot(dx, dy)
                if d > 0.4:
                    self.pos = enu_to_latlon(px + dx / d * 0.4, py + dy / d * 0.4, ox, oy)
                elif t >= 1.5:
                    await self.set_phase("DESCEND", "over target — closed-loop descent from 14.0 m")
            elif self.phase == "DESCEND":
                self.alt = max(4.5, 14.0 - 0.95 * t)
                if t > 0.5 and t - self._qr_tick > 1.6 and self.alt < 9.0:
                    self._qr_tick = t
                    self.qr["streak"] += 1
                    await self.log("INFO", "QR consensus streak %d/3" % self.qr["streak"])
                    await self.emit_qr()
                    if self.qr["streak"] >= 3:
                        self.qr.update(payload=MOCK_PAYLOAD, confirmed=True)
                        await self.event("qr_decoded", {"payload": MOCK_PAYLOAD, "altitude_m": self.alt})
                        await self.log("INFO", "QR DECODED: %s at %.1f m" % (MOCK_PAYLOAD, self.alt))
                        await self.emit_qr()
                        await self.set_phase("DECODED", "consensus 3/3 — payload locked")
                elif t >= 12.0:
                    # safety net: force confirm if streaks were throttled
                    self.qr.update(streak=3, payload=MOCK_PAYLOAD, confirmed=True)
                    await self.event("qr_decoded", {"payload": MOCK_PAYLOAD, "altitude_m": self.alt})
                    await self.log("INFO", "QR DECODED (backstop): %s at %.1f m" % (MOCK_PAYLOAD, self.alt))
                    await self.emit_qr()
                    await self.set_phase("DECODED", "consensus locked")
            elif self.phase == "DECODED" and t >= 2.0:
                self._result_sent = False
                await self.set_phase("TRANSMIT", "sending result: STATUSTEXT->MP (1st), WS->UI")
            elif self.phase == "TRANSMIT":
                n = int(t / 2.0)
                if n and n < 6 and abs(t - n * 2.0) < 0.21:
                    await self.log("INFO", "STATUSTEXT sent: QR:%s (send %d/6, window 15 s)" % (MOCK_PAYLOAD, n))
                if not self._result_sent and t >= 4.0:
                    self._result_sent = True
                    await self.event("mission_result", {"payload": MOCK_PAYLOAD,
                                                        "routes": ["statustext->MP", "ws->UI"]})
                if t >= 6.0:
                    self.mode, self.alt = "RTL", 14.0
                    await self.set_phase("RTL", "result transmitted — returning to launch")
            elif self.phase == "RTL":
                px, py = latlon_to_enu(self.pos[0], self.pos[1], ox, oy)
                d = math.hypot(px, py)
                if d > 0.5:
                    self.pos = enu_to_latlon(px - px / d * 0.5, py - py / d * 0.5, ox, oy)
                elif t >= 2.0:
                    self.mode = "LAND"
                    await self.set_phase("LAND", "at launch point — descending to land")
            elif self.phase == "LAND":
                self.alt = max(0.0, 14.0 - 2.4 * t)
                if self.alt <= 0.05:
                    self.armed, self.mode = False, "STANDBY"
                    await self.log("INFO", "LANDED at launch point. Mission complete. "
                                           "(debug: POST /api/fsm/start to re-run)")
                    await self.set_phase("COMPLETE", "landed")
            elif self.phase == "FAILSAFE":
                if t >= 3.0:
                    self.mode = "RTL"
                    await self.set_phase("RTL", "failsafe action -> RTL")

            await self.emit_telemetry()
            now = time.time()
            if now - self._last_sys > 1.0:
                self._last_sys = now
                await self.emit_system()
            if self.qr["streak"] or self.qr["confirmed"]:
                await self.emit_qr()
            await asyncio.sleep(dt)


# ---------------------------------------------------------------------------
# HTTP server + routing
# ---------------------------------------------------------------------------
class Server:
    def __init__(self, args):
        self.args = args
        self.hub = Hub()
        self.mock = MockPi(self.hub) if args.mock else None
        self.tailer = LogTailer(self.hub)
        self.bridge = {"watching": [], "qr": None, "bin_parser_available": HAVE_DFR,
                       "mock": bool(args.mock)}
        self.started = time.time()

    async def start(self):
        self.hub.loop = asyncio.get_running_loop()
        for p in self.args.watch:
            self.tailer.add_path(p)
        cand = os.path.join(os.path.expanduser("~"), "Documents", "Mission Planner", "logs")
        if os.path.isdir(cand):
            self.tailer.add_path(cand)
            print("[bridge] auto-watching MP log dir: %s" % cand)
        self.tailer.start()
        srv = await asyncio.start_server(self.handle, self.args.host, self.args.port)
        host = "localhost" if self.args.host in ("0.0.0.0", "::") else self.args.host
        print("=" * 66)
        print(" Mission Companion bridge   http://%s:%d" % (host, self.args.port))
        print(" mode: %s" % ("MOCK — full demo incl. simulated Pi" if self.args.mock
                            else "real — Pi connects separately over router/LTE"))
        print(" DFReader(.bin): %s   ·   MP log watch: %s" % (
            "yes" if HAVE_DFR else "no (text logs + manual entry only)",
            self.tailer.watch or "[none — add via UI or --watch]"))
        print("=" * 66)
        async with srv:
            if self.mock:
                asyncio.create_task(self.mock.run())
            await srv.serve_forever()

    # -- connection handler ---------------------------------------------------
    async def handle(self, reader, writer):
        cid = None
        try:
            head = b""
            while b"\r\n\r\n" not in head:
                chunk = await asyncio.wait_for(reader.read(8192), timeout=20)
                if not chunk:
                    return
                head += chunk
            head, rest = head.split(b"\r\n\r\n", 1)
            lines = head.decode("latin-1").split("\r\n")
            method, path, _ = lines[0].split(" ", 2)
            headers = {}
            for ln in lines[1:]:
                if ":" in ln:
                    k, v = ln.split(":", 1)
                    headers[k.strip().lower()] = v.strip()

            if headers.get("upgrade", "").lower() == "websocket" and path.startswith("/ws/"):
                await self.handle_ws(reader, writer, path, headers)
                return

            body = rest
            clen = int(headers.get("content-length", 0) or 0)
            while len(body) < clen:
                body += await asyncio.wait_for(reader.read(clen - len(body)), timeout=20)

            status, ctype, payload, extra = await self.route(method, path, body)
            resp = "HTTP/1.1 %d %s\r\n" % (status, STATUS_TEXT.get(status, "OK"))
            if extra.get("sse"):
                resp += ("Content-Type: text/event-stream\r\nCache-Control: no-cache\r\n"
                         "Transfer-Encoding: chunked\r\nX-Accel-Buffering: no\r\n\r\n")
            else:
                resp += ("Content-Type: %s\r\nContent-Length: %d\r\n"
                         "Access-Control-Allow-Origin: *\r\nCache-Control: no-store\r\n\r\n" % (ctype, len(payload)))
            writer.write(resp.encode())
            if not extra.get("sse"):
                writer.write(payload)
            await writer.drain()

            if extra.get("sse"):
                cid = await self.hub.add("sse", writer)
                writer.write(_sse_chunk("bridge-state", self.bridge))
                await writer.drain()
                while True:  # keepalive until the client goes away
                    await asyncio.sleep(15)
                    writer.write(_sse_chunk("keepalive", {"ts": time.time()}))
                    await writer.drain()
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError, OSError):
            pass
        except Exception as e:
            if self.args.verbose:
                print("[bridge] handler error:", repr(e))
        finally:
            if cid is not None:
                await self.hub.remove(cid)
            try:
                writer.close()
            except Exception:
                pass

    async def handle_ws(self, reader, writer, path, headers):
        key = headers.get("sec-websocket-key", "")
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
        await writer.drain()
        cid = await self.hub.add("ws", writer)
        try:
            while True:
                opcode, data = await ws_read_frame(reader)
                if opcode == 0x8:
                    writer.write(bytes([0x88, 0x00]))
                    await writer.drain()
                    break
                elif opcode == 0x9:
                    writer.write(bytes([0x8A, len(data)]) + data)
                    await writer.drain()
                elif opcode == 0x1 and data:
                    try:
                        msg = json.loads(data)
                    except Exception:
                        continue
                    if path == "/ws/webrtc/cam1" and self.mock:
                        writer.write(ws_text_frame(json.dumps(
                            {"type": "error", "message": "WebRTC disabled in mock — use frame polling"}
                        ).encode()))
                        await writer.drain()
                    elif path == "/ws/telemetry" and self.mock and msg.get("type") == "ping":
                        writer.write(ws_text_frame(b'{"type":"pong"}'))
                        await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            pass
        finally:
            await self.hub.remove(cid)
            try:
                writer.close()
            except Exception:
                pass

    # -- routing ----------------------------------------------------------------
    def json(self, obj, status=200):
        return status, "application/json", json.dumps(obj).encode(), {}

    async def route(self, method, p, body):
        try:
            jbody = json.loads(body) if body else {}
        except Exception:
            jbody = {}

        # --- bridge endpoints (always on) ---
        if p == "/api/mp/state":
            return self.json({**self.bridge, "uptime_s": round(time.time() - self.started, 1)})
        if p == "/api/mp/qr" and method == "POST":
            payload = str(jbody.get("payload", "")).strip()
            if not re.fullmatch(r"[0-9A-Za-z]{1,6}", payload):
                return self.json({"detail": "payload must be 1-6 alnum chars"}, 400)
            data = {"payload": payload, "source": "manual", "ts": time.time(),
                    "line": "manual entry via UI"}
            self.bridge["qr"] = data
            await self.hub.emit("mp-qr", data)
            return self.json({"status": "ok", "qr": data})
        if p == "/api/mp/watch" and method == "POST":
            ok = self.tailer.add_path(str(jbody.get("path", "")))
            self.bridge["watching"] = list(self.tailer.watch)
            await self.hub.emit("bridge-state", self.bridge)
            return self.json({"status": "ok" if ok else "ignored",
                              "watching": list(self.tailer.watch)})
        if p == "/api/mp/stream":
            return 200, "text/event-stream", b"", {"sse": True}

        # --- mock Pi (implements the contract) ---
        if self.mock is not None:
            r = await self.mock_route(method, p, jbody)
            if r is not None:
                return r

        # --- static UI ---
        if method == "GET":
            rel = p[1:] if p.startswith("/") else p
            if rel in ("", "index.html"):
                rel = "index.html"
            fp = os.path.normpath(os.path.join(ROOT, rel))
            if fp.startswith(os.path.normpath(ROOT)) and os.path.isfile(fp):
                ctype = STATIC_TYPES.get(os.path.splitext(fp)[1].lower(), "application/octet-stream")
                with open(fp, "rb") as f:
                    return 200, ctype, f.read(), {}
        return self.json({"detail": "not found: %s %s" % (method, p)}, 404)

    async def mock_route(self, method, p, jbody):
        m = self.mock
        if p == "/health":
            return self.json({"status": "ok", "mock": True})

        if p == "/api/geofence":
            if method == "POST":
                verts = jbody.get("vertices", [])
                if not (3 <= len(verts) <= 255) or any(
                        not (isinstance(v, dict) and -90 <= float(v.get("lat", 1e9)) <= 90
                             and -180 <= float(v.get("lon", 1e9)) <= 180) for v in verts):
                    return self.json({"detail": "need 3-255 valid {lat,lon} vertices"}, 400)
                olat = sum(v["lat"] for v in verts) / len(verts)
                olon = sum(v["lon"] for v in verts) / len(verts)
                enu = [latlon_to_enu(v["lat"], v["lon"], olat, olon) for v in verts]
                a = 0.0
                for i in range(len(enu)):
                    x0, y0 = enu[i]
                    x1, y1 = enu[(i + 1) % len(enu)]
                    a += x0 * y1 - x1 * y0
                a = abs(a) / 2.0
                cx = sum(x[0] for x in enu) / len(enu)
                cy = sum(x[1] for x in enu) / len(enu)
                rmax = max(math.hypot(x - cx, y - cy) for x, y in enu)
                status = {"loaded": True, "confirmed": True,
                          "reason": "Fence stored + readback OK (mock FC)",
                          "vertex_count": len(verts), "area_m2": round(a, 1),
                          "max_radius_m": round(rmax, 1),
                          "centroid_enu": {"x": round(cx, 2), "y": round(cy, 2)},
                          "origin_lat": olat, "origin_lon": olon,
                          "vertices_latlon": [{"lat": v["lat"], "lon": v["lon"]} for v in verts]}
                m.fence = status
                await m.log("INFO", "Fence uploaded: %d verts, %.0f m² — readback OK — "
                                     "STATUSTEXT 'FENCE: %d verts' sent to MP" % (len(verts), a, len(verts)))
                await m.event("fence_updated", status)
                return self.json(status)
            if method == "GET":
                return self.json(m.fence or {"loaded": False, "confirmed": False,
                                             "reason": "No fence uploaded yet",
                                             "vertex_count": 0, "vertices_latlon": []})
            if method == "DELETE":
                m.fence = None
                await m.log("INFO", "Fence cleared on (mock) FC + locally")
                await m.event("fence_updated", {"loaded": False, "vertex_count": 0})
                return self.json({"loaded": False, "confirmed": False, "reason": "Fence cleared",
                                  "vertex_count": 0, "vertices_latlon": []})

        if p == "/api/fsm/status":
            return self.json({"state": m.phase, "previous": m.prev_phase, "armable": m.armed,
                              "link_healthy": True,
                              "plan": {"items": 3, "trigger_seq": 2, "synced": m.phase != "IDLE"},
                              "payload": m.qr["payload"] if m.qr["confirmed"] else None})
        if p == "/api/fsm/abort" and method == "POST":
            if m.phase not in ("FAILSAFE", "RTL", "LAND", "COMPLETE", "IDLE"):
                await m.log("WARN", "ABORT — FAILSAFE -> RTL")
                await m.event("abort", {"from": m.phase})
                m.prev_phase, m.phase, m.phase_t = m.phase, "FAILSAFE", 0.0
                await m.emit_fsm()
            return self.json({"state": m.phase})
        if p == "/api/fsm/start" and method == "POST":
            m.reset()
            await m.log("INFO", "Mission (re)started via debug /api/fsm/start")
            return self.json({"state": m.phase})
        if p == "/api/qr/status":
            return self.json({**m.qr, "ts": time.time()})
        if p == "/api/camera/status":
            return self.json({"mode": "cam1", "active": True,
                              "type": "Picamera2Source (mock)", "controls": m.controls})
        if p == "/api/camera/controls" and method == "POST":
            for k in ("exposure_us", "gain", "af_mode", "brightness", "contrast",
                      "saturation", "sharpness", "adaptive"):
                if k in jbody:
                    m.controls[k] = jbody[k]
            await m.log("INFO", "Camera controls updated: %s" % jbody)
            return self.json({"status": "ok", "controls": m.controls})
        if p == "/api/camera/frame/cam1" and method == "GET":
            return 200, "image/svg+xml", render_cam_svg(m), {}
        return None


def main():
    ap = argparse.ArgumentParser(description="Mission Companion bridge")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8100)
    ap.add_argument("--mock", action="store_true", help="run simulated Pi (full contract demo)")
    ap.add_argument("--watch", action="append", default=[], help="file/dir to tail (repeatable)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    try:
        asyncio.run(Server(args).start())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
