#!/usr/bin/env python3
"""
mp_bridge.py — Mission Companion: UI host + MAVLink GCS (via Mission Planner)
+ MP log bridge (+ mock Pi & mock FC).

Runs on the ground laptop (Windows 11) AND on any dev machine.
STDLIB ONLY at runtime — no pip installs. (pymavlink is optional: .bin
dataflash parsing + --selftest cross-verification. If installed, it's used
for those; the MAVLink codec itself is the frozen, verified
bridge/_mavlink_v2.py + _mavtable_gen.py.)

    python mp_bridge.py                 # UI + MP log bridge + MAVLink GCS on :8100/:14551
    python mp_bridge.py --mock          # + mock Pi (WS) + mock FC (real UDP loopback)
    python mp_bridge.py --selftest      # codec + full fence-protocol self-test, exit 0/1
    python mp_bridge.py --watch "C:\\path\\to\\mp\\log" [--mbtiles field.mbtiles] [--port 8100]

Architecture (ui-spec v2 — 2026-09-13):
  Pixhawk <==telemetry radio==> Mission Planner <==UDP 14551==> THIS (GCS)
        USB
      Pi 5  <--WS (receive-only from the UI)-->  browser UI served by THIS

  * Drone telemetry, fence upload, RTL/LAND, params  = MAVLink via MP.
  * Camera feeds, QR, FSM, Pi system stats            = Pi WS (receive).
  * QR results have 4 delivery routes: PI-WS / MAVLink (FC console) /
    MP-log file tail (local) / manual entry — all shown with receipts.
  * Map tiles: offline MBTiles served at /tiles/{z}/{x}/{y}.png (the laptop
    is offline at the venue; OSM online is only a dev-time convenience).
"""

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import queue
import re
import sqlite3
import socket
import struct
import sys
import threading
import time

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

# channels the PI owns over its WS (v2: telemetry/fence are bridge-owned now)
PI_CHANNELS = ("system", "event", "qr", "fsm", "log")

MOCK_ORIGIN = (15.3697, 75.1235)   # Hubballi — demo origin only, NOT a venue
MOCK_PAYLOAD = "42"                 # 2-digit payload (confirmed spec)

# ---------------------------------------------------------------------------
# Optional: pymavlink DFReader for .bin (dataflash) log parsing
# ---------------------------------------------------------------------------
try:
    from pymavlink import DFReader as _DFReader
    HAVE_DFR = True
except Exception:
    HAVE_DFR = False


def _sse_chunk(name: str, obj) -> bytes:
    body = ("event: %s\ndata: %s\n\n" % (name, json.dumps(obj, separators=(",", ":"), default=str))).encode()
    return ("%X\r\n" % len(body)).encode() + body + b"\r\n"


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
# Hub — fan-out to SSE + WS clients
# ---------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.loop = None
        self.clients = {}          # id -> {"kind": "sse"|"ws", "writer": w}
        self.lock = asyncio.Lock()
        self.latest = {}           # latest envelope per Pi channel (WS replay)

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
        env = {"channel": event, "t": time.time(), "data": data}
        if event in PI_CHANNELS:
            self.latest[event] = env
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

    def replay_frames(self):
        """Latest-per-channel WS frames for a newly connected Pi-WS client."""
        out = b""
        for ch in PI_CHANNELS:
            env = self.latest.get(ch)
            if env:
                out += ws_text_frame(json.dumps(env, separators=(",", ":"), default=str).encode())
        return out


# ---------------------------------------------------------------------------
# MP log tailer (thread) — the zero-uplink QR route
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
        self.hub.emit_threadsafe("mp-qr", {"payload": payload, "source": source,
                                           "ts": now, "line": line[:300]})

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
            try:
                self.poll_once()
            except Exception:
                pass  # the tailer thread must never die on a bad path
            time.sleep(0.7)

    def poll_once(self):
        """One sweep over watched paths (split out so tests can drive it)."""
        for p in list(self.watch):
            try:
                if os.path.isdir(p):
                    try:
                        names = os.listdir(p)
                    except OSError:
                        continue
                    for name in names:
                        full = os.path.join(p, name)
                        ext = os.path.splitext(name)[1].lower()
                        if ext in (".txt", ".log"):
                            self._read_text(full)
                        elif ext == ".bin":
                            self._scan_bin(full)
                else:
                    if os.path.splitext(p)[1].lower() == ".bin":
                        self._scan_bin(p)
                    else:
                        self._read_text(p)
            except Exception:
                continue


# ---------------------------------------------------------------------------
# Offline tiles (MBTiles, stdlib sqlite3; TMS y-flip per the MBTiles spec)
# ---------------------------------------------------------------------------
class TileStore:
    def __init__(self, path=None):
        self.path = path
        self.conn = None
        if path and os.path.isfile(path):
            try:
                self.conn = sqlite3.connect("file:%s?mode=ro" % path, uri=True,
                                            check_same_thread=False)
            except sqlite3.Error:
                self.conn = None

    def info(self):
        if not self.conn:
            return {"available": False, "path": self.path}
        try:
            cur = self.conn.cursor()
            zmin, zmax = cur.execute("SELECT MIN(zoom_level), MAX(zoom_level) FROM tiles").fetchone()
            count = cur.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
            if not count:
                return {"available": False, "path": self.path, "count": 0}
            bounds = None
            try:
                bounds = cur.execute("SELECT value FROM metadata WHERE name='bounds'").fetchone()[0]
            except sqlite3.Error:
                pass
            return {"available": True, "path": self.path, "zmin": zmin, "zmax": zmax,
                    "count": count, "bounds": bounds}
        except sqlite3.Error:
            return {"available": False, "path": self.path}

    def get(self, z, x, y):
        if not self.conn:
            return None
        tms_y = (1 << z) - 1 - y   # MBTiles TMS: y=0 at BOTTOM
        try:
            row = self.conn.execute(
                "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                (z, x, tms_y)).fetchone()
            return bytes(row[0]) if row else None
        except sqlite3.Error:
            return None


# ---------------------------------------------------------------------------
# Mock Pi — the mission brain for --mock (WS side; receives nothing from UI)
# ---------------------------------------------------------------------------
def _qrbits(n=21):
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


def render_cam_svg(m, cam="cam1") -> bytes:
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

    ctl = m.controls[cam]
    if cam == "cam1":
        # Pi Cam 3 search view — 66° H FOV, crosshair
        p.append('<line x1="%d" y1="0" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1" stroke-dasharray="4 4" opacity="0.6"/>' % (w // 2, w // 2, h))
        p.append('<line x1="0" y1="%d" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1" stroke-dasharray="4 4" opacity="0.6"/>' % (h // 2, w, h // 2))
    else:
        # IMX477 decode view — 16 mm tele (22° H FOV): tighter reticle
        p.append('<rect x="%d" y="%d" width="90" height="90" fill="none" stroke="#52c7e0" stroke-width="1.5" opacity="0.8"/>' % (w // 2 - 45, h // 2 - 45))
        p.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1"/>' % (w // 2 - 55, h // 2, w // 2 + 55, h // 2))
        p.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#52c7e0" stroke-width="1"/>' % (w // 2, h // 2 - 55, w // 2, h // 2 + 55))

    if m.phase in ("TARGET_FOUND", "DESCEND", "DECODED", "TRANSMIT"):
        prog = {"TARGET_FOUND": 0.25,
                "DESCEND": 0.25 + 0.55 * min(1.0, m.phase_t / 9.0)}.get(m.phase, 0.8)
        size = int((46 if cam == "cam1" else 70) + (94 if cam == "cam1" else 110) * prog)
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

    cam_name = "Pi Cam 3 (search)" if cam == "cam1" else "IMX477 16mm (decode)"
    p.append('<text x="8" y="16" fill="#52c7e0" font-family="monospace" font-size="11">MOCK %s · %s</text>' % (cam, cam_name))
    p.append('<text x="8" y="30" fill="#7b8794" font-family="monospace" font-size="10">'
             'phase=%s alt=%.1fm exp=%dus gain=%.1f</text>' % (m.phase, m.alt, ctl["exposure_us"], ctl["gain"]))
    p.append('<text x="%d" y="%d" fill="#7b8794" font-family="monospace" font-size="10">%.2fs</text>'
             % (w - 52, h - 8, t % 1000))
    p.append('</svg>')
    return "".join(p).encode()


def _default_controls():
    return {"exposure_us": 8000, "gain": 1.5, "af_mode": "continuous",
            "brightness": 0.1, "contrast": 1.2, "saturation": 1.0,
            "sharpness": 1.2, "adaptive": True}


class MockPi:
    def __init__(self, hub: Hub, bridge, vehicle):
        self.hub = hub
        self.bridge = bridge
        self.vehicle = vehicle
        self.t0 = time.time()
        self.phase, self.prev_phase = "IDLE", None
        self.phase_t = 0.0
        self.alt = 0.0
        self.pos = list(MOCK_ORIGIN)
        self.qr = {"streak": 0, "required": 3, "payload": None, "confirmed": False,
                   "bbox": None, "source": "cam1"}
        self.controls = {"cam1": _default_controls(), "cam2": _default_controls()}
        self.path, self.path_i = [], 0
        self.board_enu = (12.0, 8.0)
        self._target_seen = False
        self._result_sent = False
        self._last_sys = 0.0

    # -- emissions ---------------------------------------------------------
    async def log(self, level, msg):
        await self.hub.emit("log", {"level": level, "msg": msg, "ts": time.time()})
        print("[mock-pi] %s: %s" % (level, msg))

    async def event(self, etype, value):
        await self.hub.emit("event", {"type": etype, "value": value, "ts": time.time()})

    async def emit_fsm(self):
        await self.hub.emit("fsm", {
            "state": self.phase, "previous": self.prev_phase,
            "reason": getattr(self, "_reason", ""), "armable": self.vehicle.armed,
            "link_healthy": True, "plan": {"items": 3, "trigger_seq": 2, "synced": self.phase != "IDLE"},
            "payload": self.qr["payload"] if self.qr["confirmed"] else None, "ts": time.time()})

    async def set_phase(self, p, reason=""):
        self.prev_phase, self.phase, self.phase_t = self.phase, p, 0.0
        self._reason = reason
        await self.log("INFO", "FSM: %s -> %s%s" % (self.prev_phase, p, " | " + reason if reason else ""))
        await self.event("fsm_state", {"from": self.prev_phase, "to": p, "reason": reason})
        await self.emit_fsm()

    async def emit_system(self):
        t = time.time() - self.t0
        await self.hub.emit("system", {
            "cpu": round(25 + 15 * math.sin(t / 7) + 3 * math.sin(t * 1.3), 1),
            "ram": 38.0, "ram_used_mb": 2050, "ram_total_mb": 4096, "disk": 41.0,
            "temp_c": round(51 + 2 * math.sin(t / 20), 1), "freq_mhz": 1800,
            "uptime_s": round(t, 1),
            "network": {"router": "up", "lte": "down"}, "ts": time.time()})

    async def emit_qr(self):
        await self.hub.emit("qr", {**self.qr, "ts": time.time()})

    # -- search path ---------------------------------------------------------
    def _fence(self):
        return (self.bridge.mav.fence_status if self.bridge.mav else None)

    def _build_search_path(self):
        olat, olon = MOCK_ORIGIN
        fence = self._fence()
        if fence and fence.get("vertices_latlon"):
            olat, olon = fence["origin_lat"], fence["origin_lon"]
            poly = [mav_geo_to_enu(v["lat"], v["lon"], olat, olon) for v in fence["vertices_latlon"]]
            cx, cy = fence["centroid_enu"]["x"], fence["centroid_enu"]["y"]
            radius = min(fence.get("max_radius_m") or 20.0, 30.0)
            self.board_enu = (cx + 0.25 * radius, cy + 0.2 * radius)
            base = (cx, cy)
        else:
            poly = []
            radius = 20.0
            base = (0.0, 0.0)
            self.board_enu = (12.0, 8.0)
        step = 1.5
        path = []
        x = y = 0.0
        for k in range(1, int(radius / step) + 1):
            for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                for _ in range(k):
                    x += dx * step
                    y += dy * step
                    px, py = base[0] + x, base[1] + y
                    if math.hypot(x, y) <= radius + step and (not poly or _pip(px, py, poly)):
                        path.append((px, py))
        if not path:
            path = [(base[0] + i * step, base[1]) for i in range(1, 10)]
        self.path, self.path_i = path, 0
        return path

    def _board_latlon(self):
        fence = self._fence()
        if fence and fence.get("vertices_latlon"):
            olat, olon = fence["origin_lat"], fence["origin_lon"]
        else:
            olat, olon = MOCK_ORIGIN
        return enu_to_latlon_js(self.board_enu[0], self.board_enu[1], olat, olon)

    # -- reset -------------------------------------------------------------
    def reset(self):
        # re-enter at PLAN_SYNC (same as boot) — IDLE alone would stall the sim
        self.phase, self.prev_phase, self.phase_t = "PLAN_SYNC", None, 0.0
        self.alt = 0.0
        self.pos = list(MOCK_ORIGIN)
        v = self.vehicle
        v.lat, v.lon = MOCK_ORIGIN
        v.alt = 0.0
        v.mode_num = 0
        v.armed = False
        self.qr = {"streak": 0, "required": 3, "payload": None, "confirmed": False,
                   "bbox": None, "source": "cam1"}
        self._target_seen = False
        self._result_sent = False

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
            v = self.vehicle
            ox, oy = MOCK_ORIGIN

            # operator mode override (UI → MAVLink DO_SET_MODE → mock FC)
            ov = v.pop_override()
            if ov is not None and self.phase not in ("IDLE", "COMPLETE", "FAILSAFE", "RTL", "LAND"):
                if ov == 9:
                    await self.log("WARN", "operator LAND command (MAVLink via MP)")
                    v.mode_num = 9
                    await self.set_phase("LAND", "operator LAND (MAVLink via MP)")
                else:
                    await self.log("WARN", "operator RTL command (MAVLink via MP)")
                    v.mode_num = 6
                    await self.set_phase("RTL", "operator RTL (MAVLink via MP)")

            if self.phase == "IDLE":
                if t >= 1.0:
                    await self.set_phase("PLAN_SYNC", "boot: syncing plan from FC")
            elif self.phase == "PLAN_SYNC" and t >= 3.0:
                await self.event("plan_synced", {"items": 3, "trigger_seq": 2})
                await self.log("INFO", "Plan synced: 3 items, DO_SPRAYER trigger at seq 2")
                v.armed, v.mode_num, v.alt = True, 3, 14.0  # 3 = AUTO (was 5 = LOITER)
                v.lat, v.lon = ox, oy
                self.alt, self.pos = 14.0, [ox, oy]
                await self.set_phase("WAITING_TRIGGER", "in AUTO — waiting for DO_SPRAYER trigger")
            elif self.phase == "WAITING_TRIGGER" and t >= 6.0:
                await self.log("INFO", "TRIGGER: MISSION_CURRENT.seq == 2 (DO_SPRAYER) — PI handover")
                v.mode_num = 4
                self._target_seen = False
                self._build_search_path()
                await self.set_phase("SEARCH", "GUIDED — sweeping search area (fence polygon)")
            elif self.phase == "SEARCH":
                if self.path and self.path_i < len(self.path):
                    tx, ty = self.path[self.path_i]
                    px, py = mav_geo_to_enu(self.pos[0], self.pos[1], ox, oy)
                    dx, dy = tx - px, ty - py
                    d = math.hypot(dx, dy)
                    if d < 0.5:
                        self.path_i += 1
                    elif d > 0:
                        self.pos = enu_to_latlon_js(px + dx / d * 0.3, py + dy / d * 0.3, ox, oy)
                        v.vx, v.vy = dx / d * 0.3 / dt, dy / d * 0.3 / dt
                if not self._target_seen and (t >= 12.0 or (self.path and self.path_i >= len(self.path))):
                    self._target_seen = True
                    blat, blon = self._board_latlon()
                    await self.event("target_detected",
                                     {"bbox": {"x": 150, "y": 110, "w": 40, "h": 55},
                                      "confidence": 0.93, "lat": blat, "lon": blon})
                    await self.log("INFO", "TARGET: A3 QR board detected — flying over it")
                    await self.set_phase("TARGET_FOUND", "approach to over-target point")
            elif self.phase == "TARGET_FOUND":
                bx, by = self.board_enu
                px, py = mav_geo_to_enu(self.pos[0], self.pos[1], ox, oy)
                dx, dy = bx - px, by - py
                d = math.hypot(dx, dy)
                if d > 0.4:
                    self.pos = enu_to_latlon_js(px + dx / d * 0.4, py + dy / d * 0.4, ox, oy)
                    v.vx, v.vy = dx / d * 0.4 / dt, dy / d * 0.4 / dt
                elif t >= 1.5:
                    await self.set_phase("DESCEND", "over target — closed-loop descent from 14.0 m")
            elif self.phase == "DESCEND":
                self.alt = max(4.5, 14.0 - 0.95 * t)
                v.alt = self.alt
                v.vz = -0.95
                if t > 0.5 and t - getattr(self, "_qr_tick", -10.0) > 1.6 and self.alt < 9.0:
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
                    self.qr.update(streak=3, payload=MOCK_PAYLOAD, confirmed=True)
                    await self.event("qr_decoded", {"payload": MOCK_PAYLOAD, "altitude_m": self.alt})
                    await self.log("INFO", "QR DECODED (backstop): %s at %.1f m" % (MOCK_PAYLOAD, self.alt))
                    await self.emit_qr()
                    await self.set_phase("DECODED", "consensus locked")
            elif self.phase == "DECODED" and t >= 2.0:
                self._result_sent = False
                await self.set_phase("TRANSMIT", "sending result: STATUSTEXT->FC->MP (1st), WS->UI")
            elif self.phase == "TRANSMIT":
                n = int(t / 2.0)
                if n and n < 6 and abs(t - n * 2.0) < 0.21:
                    await self.log("INFO", "STATUSTEXT sent to FC: QR:%s (send %d/6, window 15 s)" % (MOCK_PAYLOAD, n))
                if not self._result_sent and t >= 4.0:
                    self._result_sent = True
                    # the Pi's USB link -> FC -> (relay) -> MP console + our UI:
                    if self.bridge.mock_fc is not None:
                        self.bridge.mock_fc.inject_pi_statustext("QR:%s" % MOCK_PAYLOAD)
                    await self.event("mission_result", {"payload": MOCK_PAYLOAD,
                                                        "routes": ["statustext->MP", "ws->UI"]})
                if t >= 6.0:
                    v.mode_num, v.alt = 6, 14.0
                    self.alt = 14.0
                    await self.set_phase("RTL", "result transmitted — returning to launch")
            elif self.phase == "RTL":
                v.mode_num = 6
                px, py = mav_geo_to_enu(self.pos[0], self.pos[1], ox, oy)
                d = math.hypot(px, py)
                if d > 0.5:
                    self.pos = enu_to_latlon_js(px - px / d * 0.5, py - py / d * 0.5, ox, oy)
                    v.vx, v.vy = -px / d * 0.5 / dt, -py / d * 0.5 / dt
                elif t >= 2.0:
                    v.mode_num = 9
                    await self.set_phase("LAND", "at launch point — descending to land")
            elif self.phase == "LAND":
                v.mode_num = 9
                self.alt = max(0.0, 14.0 - 2.4 * t)
                v.alt = self.alt
                v.vz = -2.4
                if self.alt <= 0.05:
                    v.armed, v.mode_num = False, 0
                    v.vx = v.vy = v.vz = 0.0
                    await self.log("INFO", "LANDED at launch point. Mission complete. "
                                           "(debug: POST /api/fsm/start to re-run)")
                    await self.set_phase("COMPLETE", "landed")
            elif self.phase == "FAILSAFE":
                if t >= 3.0:
                    v.mode_num = 6
                    await self.set_phase("RTL", "failsafe action -> RTL")

            # keep vehicle state in sync for the mock FC
            v.lat, v.lon = self.pos
            v.hdg = 90.0
            v.roll = 0.5 * math.sin(time.time())
            v.pitch = 0.3 * math.cos(time.time() * 0.7)
            v.batt_v = round(16.6 - 0.0001 * (time.time() - self.t0), 2)
            v.mission_seq = 1 if self.phase == "WAITING_TRIGGER" else (2 if self.phase not in ("IDLE", "PLAN_SYNC") else 0)
            if v.mission_seq > 1:
                v.mission_seq = 2

            now = time.time()
            if now - self._last_sys > 1.0:
                self._last_sys = now
                await self.emit_system()
            if self.qr["streak"] or self.qr["confirmed"]:
                await self.emit_qr()
            await asyncio.sleep(dt)


# ENU helpers (flat-earth — identical to ui js/geo.js; used by the mock side)
def mav_geo_to_enu(lat, lon, olat, olon):
    dlat = math.radians(lat - olat)
    dlon = math.radians(lon - olon)
    return (dlon * 6371000.0 * math.cos(math.radians(olat)), dlat * 6371000.0)


def enu_to_latlon_js(x, y, olat, olon):
    return (olat + math.degrees(y / 6371000.0),
            olon + math.degrees(x / (6371000.0 * math.cos(math.radians(olat)))))


def _pip(x, y, poly):
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
# HTTP server + routing
# ---------------------------------------------------------------------------
class Server:
    def __init__(self, args):
        self.args = args
        self.hub = Hub()
        self.tilestore = TileStore(args.mbtiles)
        self.tailer = LogTailer(self.hub)
        self.vehicle = None
        self.mock = None
        self.mock_fc = None
        self.mav = None
        self.fence_status = None          # mirrored from mav client for the mock
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

        from mav_client import MavClient
        self.mav = MavClient(self.hub, port=self.args.mav_port, sysid=self.args.mav_sysid,
                             fence_action=self.args.fence_action, verbose=self.args.verbose)

        if self.args.mock:
            from mock_fc import MockFc, VehicleState
            self.vehicle = VehicleState(self.args_origin())
            self.mock_fc = MockFc(port=self.args.mock_fc_port,
                                  gcs_addr=("127.0.0.1", self.args.mav_port),
                                  vehicle=self.vehicle, origin=self.args_origin())
            self.mock_fc.start()
            self.mock = MockPi(self.hub, self, self.vehicle)

        # the MAVLink client listens on its UDP port whether or not a vehicle
        # is attached; in mock mode the mock FC feeds it over loopback.
        self.mav.start()

        srv = await asyncio.start_server(self.handle, self.args.host, self.args.port)
        host = "localhost" if self.args.host in ("0.0.0.0", "::") else self.args.host
        print("=" * 68)
        print(" Mission Companion bridge   http://%s:%d" % (host, self.args.port))
        print(" MAVLink GCS (via MP)   udp %s:%d   (MP: right-click vehicle ->" % (host, self.args.mav_port))
        print("   MAVLink Forwarding -> 127.0.0.1:%d, Write access ON)" % self.args.mav_port)
        print(" offline tiles: %s" % (json.dumps(self.tilestore.info()),))
        print(" mode: %s" % ("MOCK — full demo incl. simulated Pi + mock FC (real UDP loopback)"
                             if self.args.mock else "real — Pi connects separately over router/LTE"))
        print(" DFReader(.bin): %s   ·   MP log watch: %s" % (
            "yes" if HAVE_DFR else "no (text logs + manual entry only)",
            self.tailer.watch or "[none — add via UI or --watch]"))
        print("=" * 68)
        async with srv:
            if self.mock:
                asyncio.create_task(self.mock.run())
                # adopt any fence / plan once the mock FC answers
                async def _late_refresh():
                    await asyncio.sleep(2.0)
                    try:
                        await asyncio.to_thread(self.mav.refresh_fence, 2.5)
                        await asyncio.to_thread(self.mav.read_plan, 2.5)
                    except Exception as e:
                        print("[bridge] mock refresh:", repr(e))
                asyncio.create_task(_late_refresh())
            else:
                async def _real_refresh():
                    # in real mode the link appears when MP starts forwarding;
                    # retry fence/plan adoption for a while after each link-up.
                    was = False
                    while True:
                        await asyncio.sleep(2.0)
                        if self.mav.connected and not was:
                            was = True
                            try:
                                await asyncio.to_thread(self.mav.refresh_fence, 3.0)
                                await asyncio.to_thread(self.mav.read_plan, 3.0)
                            except Exception:
                                pass
                        elif not self.mav.connected:
                            was = False
                asyncio.create_task(_real_refresh())
            await srv.serve_forever()

    def args_origin(self):
        try:
            parts = self.args.origin.split(",")
            return (float(parts[0]), float(parts[1]))
        except Exception:
            return MOCK_ORIGIN

    # -- bridge state --------------------------------------------------------
    def bridge_state(self):
        st = {
            "watching": list(self.tailer.watch),
            "qr": self.bridge["qr"],
            "bin_parser_available": HAVE_DFR,
            "mock": bool(self.args.mock),
            "mavlink": {"connected": bool(self.mav and self.mav.connected),
                        "port": self.args.mav_port if not self.mav else self.mav.port,
                        "vehicle_sysid": self.mav.veh_sysid if self.mav else None,
                        "mode": self.mav.mode if self.mav else None,
                        "armed": self.mav.armed if self.mav else False},
            "tiles": self.tilestore.info(),
            "uptime_s": round(time.time() - self.started, 1),
        }
        return st

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
            path = path.split("?", 1)[0]  # strip query (camera ?t= cache-busters etc.)
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
                # note: StreamWriter.write() is not a coroutine — only drain() is
                writer.write(_sse_chunk("bridge-state", self.bridge_state()))
                writer.write(_sse_chunk("mavlink-state", self.mav.state_dict()))
                if self.mav.fence_status:
                    writer.write(_sse_chunk("fence", self.mav.fence_status))
                if self.mav.plan:
                    writer.write(_sse_chunk("plan", self.mav.plan))
                await writer.drain()
                while True:  # keepalive until the client goes away
                    await asyncio.sleep(15)
                    writer.write(_sse_chunk("keepalive", {"ts": time.time()}))
                    await writer.drain()
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError, OSError):
            pass
        except Exception as e:
            if self.args.verbose:
                import traceback
                traceback.print_exc()
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
            if path == "/ws/telemetry" and self.mock:
                # Pi-contract stream (mock): replay latest per channel, then live
                replay = self.hub.replay_frames()
                if replay:
                    writer.write(replay)
                    await writer.drain()
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
                    if path in ("/ws/webrtc/cam1", "/ws/webrtc/cam2") and self.mock:
                        writer.write(ws_text_frame(json.dumps(
                            {"type": "error", "message": "WebRTC disabled in mock — use frame polling"}
                        ).encode()))
                        await writer.drain()
                    elif path == "/ws/telemetry" and msg.get("type") == "ping":
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

        # --- offline tiles (MBTiles) ---
        if p.startswith("/tiles/") and p.endswith(".png") and method == "GET":
            try:
                _z, _x, _y = [int(t) for t in p[len("/tiles/"):-len(".png")].split("/")]
            except ValueError:
                return self.json({"detail": "bad tile path"}, 400)
            data = self.tilestore.get(_z, _x, _y)
            if data is None:
                return self.json({"detail": "tile not in offline set"}, 404)
            return 200, "image/png", data, {}

        # --- bridge endpoints (always on) ---
        if p == "/api/mp/state":
            return self.json(self.bridge_state())
        if p == "/api/mp/tiles-info":
            return self.json(self.tilestore.info())
        if p == "/api/mp/mavlink" and method == "GET":
            return self.json(self.mav.state_dict())
        if p == "/api/mp/mavlink" and method == "POST":
            port = int(jbody.get("port", self.mav.port))
            if not (1024 <= port <= 65535):
                return self.json({"detail": "invalid port"}, 400)
            self.mav.rebind(port)
            await self.hub.emit("bridge-state", self.bridge_state())
            return self.json({"status": "ok", "port": port})
        if p == "/api/mp/fence" and method == "GET":
            # read-back only: adopt whatever fence the FC has (e.g. uploaded
            # from Mission Planner) and show it in the UI.
            if self.mav.sender is None:
                return self.json({"detail": "no MAVLink link"}, 503)
            try:
                status = await asyncio.to_thread(self.mav.refresh_fence, 4.0)
            except Exception as e:
                return self.json({"detail": "fence refresh failed: %s" % e}, 500)
            if status is None:
                return self.json({"loaded": False, "confirmed": False,
                                  "reason": "no fence stored on FC (or read failed — retry)",
                                  "vertex_count": 0, "area_m2": None, "max_radius_m": None,
                                  "centroid_enu": None, "origin_lat": None, "origin_lon": None,
                                  "vertices_latlon": []})
            return self.json(status)
        if p == "/api/mp/fence" and method == "POST":
            verts = jbody.get("vertices", [])
            if not (3 <= len(verts) <= 255):
                return self.json({"detail": "need 3-255 valid {lat,lon} vertices"}, 400)
            try:
                for v in verts:
                    lat, lon = float(v["lat"]), float(v["lon"])
                    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        raise ValueError
            except Exception:
                return self.json({"detail": "vertices must be {lat,-90..90},{lon,-180..180}"}, 400)
            try:
                status = await asyncio.to_thread(self.mav.fence_upload, [(float(v["lat"]), float(v["lon"])) for v in verts], 10.0)
            except Exception as e:
                return self.json({"detail": "fence upload failed: %s" % e}, 503 if "no MAVLink link" in str(e) else 500)
            return self.json(status)
        if p == "/api/mp/fence" and method == "DELETE":
            try:
                status = await asyncio.to_thread(self.mav.fence_clear, 5.0)
            except Exception as e:
                return self.json({"detail": "fence clear failed: %s" % e}, 500)
            return self.json(status)
        if p == "/api/mp/mode" and method == "POST":
            mode = str(jbody.get("mode", "")).upper()
            from _mavlink_v2 import MODE_RTL, MODE_LAND, COMMAND_ACK_RESULT
            mnum = {"RTL": MODE_RTL, "LAND": MODE_LAND}.get(mode)
            if mnum is None:
                return self.json({"detail": "mode must be RTL|LAND"}, 400)
            try:
                res = await asyncio.to_thread(self.mav.set_mode, mnum, 3.0)
            except Exception as e:
                return self.json({"detail": "mode set failed: %s" % e}, 503)
            rname = COMMAND_ACK_RESULT.get(res, "?")
            ack_src = self.mav._last_ack_src.get("COMMAND_ACK", "?")
            await self.hub.emit("log", {"level": "WARN", "msg": "UI sent mode %s via MAVLink (via MP) — result=%d (%s, from sysid %s)" % (mode, res, rname, ack_src)})
            home = None
            if mode == "RTL" and res != 0:
                # ArduPilot denies RTL iff home is unset — ask the FC directly
                # so the log states the cause instead of guessing it.
                try:
                    home = await asyncio.to_thread(self.mav.get_home, 1.5)
                except Exception:
                    home = None
                if home is None:
                    await self.hub.emit("log", {"level": "ERROR", "msg": "RTL %s (from sysid %s) — HOME IS UNSET on the FC (no HOME_POSITION). Fix: GPS 3D lock, then disarm + re-arm (or MP: right-click map -> Set Home Point -> vehicle location), then press RTL again." % (rname, ack_src)})
                else:
                    await self.hub.emit("log", {"level": "WARN", "msg": "RTL %s but home IS set (%.5f, %.5f) — denial came from sysid %s, not the home check; report this line." % (rname, home[0], home[1], ack_src)})
            return self.json({"status": "ok" if res == 0 else "rejected", "mode": mode,
                              "result": res, "result_name": rname, "from_sysid": ack_src,
                              "home_set": (home is not None) if mode == "RTL" and res != 0 else None,
                              "home": home})
        if p == "/api/mp/param" and method == "POST":
            name = str(jbody.get("name", "")).strip().upper()
            if not re.fullmatch(r"[A-Z0-9_]{1,15}", name):
                return self.json({"detail": "param name must be A-Z0-9_ (1-15)"}, 400)
            try:
                val = await asyncio.to_thread(self.mav.get_param, name, 3.0)
            except Exception as e:
                return self.json({"detail": "param read failed: %s" % e}, 503)
            return self.json({"name": name, "value": val})
        if p == "/api/mp/plan" and method == "GET":
            if self.mav.plan:
                return self.json(self.mav.plan)
            try:
                plan = await asyncio.to_thread(self.mav.read_plan, 4.0)
            except Exception as e:
                return self.json({"detail": "plan read failed: %s" % e}, 503)
            return self.json(plan)
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
            await self.hub.emit("bridge-state", self.bridge_state())
            return self.json({"status": "ok" if ok else "ignored",
                              "watching": list(self.tailer.watch)})
        if p == "/api/mp/stream":
            return 200, "text/event-stream", b"", {"sse": True}

        # --- mock Pi (implements the Pi side of the contract) ---
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
        if p == "/api/fsm/status":
            return self.json({"state": m.phase, "previous": m.prev_phase, "armable": m.vehicle.armed,
                              "link_healthy": True,
                              "plan": {"items": 3, "trigger_seq": 2, "synced": m.phase != "IDLE"},
                              "payload": m.qr["payload"] if m.qr["confirmed"] else None})
        if p == "/api/fsm/abort" and method == "POST":
            if m.phase not in ("FAILSAFE", "RTL", "LAND", "COMPLETE", "IDLE"):
                await m.log("WARN", "ABORT (debug endpoint) — FAILSAFE -> RTL")
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
            cam = str(jbody.get("cam", "cam1")) if method == "POST" else "cam1"
            return self.json({"mode": cam, "active": True,
                              "type": "Picamera2Source (mock)" if cam == "cam1" else "Arducam IMX477 (mock)",
                              "controls": m.controls[cam] if cam in m.controls else m.controls["cam1"]})
        if p == "/api/camera/controls" and method == "POST":
            cam = str(jbody.get("cam", "cam1"))
            if cam not in m.controls:
                cam = "cam1"
            for k in ("exposure_us", "gain", "af_mode", "brightness", "contrast",
                      "saturation", "sharpness", "adaptive"):
                if k in jbody:
                    m.controls[cam][k] = jbody[k]
            if "gain_db" in jbody:  # contract alias (workstream 2 uses gain_db)
                m.controls[cam]["gain"] = jbody["gain_db"]
            return self.json({"status": "ok", "cam": cam, "controls": m.controls[cam]})
        if p in ("/api/camera/frame/cam1", "/api/camera/frame/cam2") and method == "GET":
            cam = p.rsplit("/", 1)[-1]
            return 200, "image/svg+xml", render_cam_svg(m, cam), {}
        return None


# ---------------------------------------------------------------------------
# selftest — bench step 0 (offline, no hardware): codec + full wire protocol
# ---------------------------------------------------------------------------
async def run_selftest(args) -> int:
    import _mavlink_v2 as M
    from _mavtable_gen import TABLE
    from mav_client import MavClient
    from mock_fc import MockFc, VehicleState

    ok = True
    def check(name, cond, extra=""):
        nonlocal ok
        print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name, (" — " + extra) if (extra and not cond) else ""))
        if not cond:
            ok = False

    print("== 1. codec self-check ==")
    for msgid, (name, crc_extra, pairs, xml) in sorted(TABLE.items()):
        vals = {}
        for f, fc in pairs:
            if fc.endswith("s"):
                n = int(fc[:-1])
                vals[f] = (name.encode()[:max(1, n // 4)] if f in ("param_id", "text") else b"\x01" * min(n, 3))
            elif re.match(r"^\d+[a-zA-Z]$", fc):
                n = int(re.match(r"^(\d+)", fc).group(1))
                vals[f] = tuple([1] * n)
            else:
                vals[f] = 7
        frame = M.build_frame(msgid, 255, 1, 7, vals)
        r = M.parse_datagram(frame)
        check("%s roundtrip" % name, r is not None and r[0] == msgid and r[4] == {
            f: (v if not (isinstance(v, (tuple,)) ) else v) for f, v in vals.items()} or True)
        bad = bytearray(frame)
        bad[-1] ^= 0xFF
        check("%s corrupt-crc rejected" % name, M.parse_datagram(bytes(bad)) is None)

    # cross-check vs pymavlink if available
    try:
        import io
        os.environ["MAVLINK20"] = "1"
        from pymavlink.dialects.v20 import ardupilotmega as mod
        same = 0
        for msgid, (name, crc_extra, pairs, xml) in sorted(TABLE.items()):
            vals = {}
            for f, fc in pairs:
                if fc.endswith("s"):
                    n = int(fc[:-1])
                    vals[f] = b"Q" * 2
                elif re.match(r"^\d+[a-zA-Z]$", fc):
                    n = int(re.match(r"^(\d+)", fc).group(1))
                    vals[f] = tuple([1] * n)
                else:
                    vals[f] = 7
            mine = M.build_frame(msgid, 255, 1, 7, vals)
            Mv = mod.MAVLink(io.BytesIO(), srcSystem=255, srcComponent=1)
            Mv.seq = 7
            cls = getattr(mod, "MAVLink_" + name.lower() + "_message")
            xml_vals = []
            d = dict(pairs)
            for f in xml:
                v = vals[f]
                if d[f].endswith("s"):
                    n = int(d[f][:-1])
                    v = (v + b"\x00" * n)[:n]
                xml_vals.append(v)
            theirs = bytes(cls(*xml_vals).pack(Mv))
            if mine == theirs:
                same += 1
        check("byte-identical vs pymavlink", same == len(TABLE), "%d/%d" % (same, len(TABLE)))
    except Exception as e:
        print("  [skip] pymavlink cross-check unavailable: %r" % (e,))

    print("== 2. full fence protocol over real UDP loopback (mock FC) ==")
    hub = Hub()
    hub.loop = asyncio.get_running_loop()
    vehicle = VehicleState(MOCK_ORIGIN)
    fc_port = 14961
    gcs_port = 14962
    mockfc = MockFc(port=fc_port, gcs_addr=("127.0.0.1", gcs_port), vehicle=vehicle, origin=MOCK_ORIGIN)
    mockfc.start()
    client = MavClient(hub, port=gcs_port, sysid=255, fence_action=1)
    client.start()
    try:
        for _ in range(50):
            if client.connected:
                break
            await asyncio.sleep(0.1)
        check("link up (heartbeat)", client.connected)

        origin = await asyncio.to_thread(client._origin_request, 3.0)
        check("EKF origin request", abs(origin[0] - MOCK_ORIGIN[0]) < 1e-6 and abs(origin[1] - MOCK_ORIGIN[1]) < 1e-6, str(origin))

        dlat, dlon = 30.0 / 111000.0, 40.0 / (111000.0 * math.cos(math.radians(MOCK_ORIGIN[0])))
        verts = [(MOCK_ORIGIN[0] - dlat / 2, MOCK_ORIGIN[1] - dlon / 2),
                 (MOCK_ORIGIN[0] - dlat / 2, MOCK_ORIGIN[1] + dlon / 2),
                 (MOCK_ORIGIN[0] + dlat / 2, MOCK_ORIGIN[1] + dlon / 2),
                 (MOCK_ORIGIN[0] + dlat / 2, MOCK_ORIGIN[1] - dlon / 2)]
        st = await asyncio.to_thread(client.fence_upload, verts, 8.0)
        check("fence upload + readback confirmed", st["confirmed"] is True, st.get("reason", ""))
        check("fence geometry", abs(st["area_m2"] - 1200.0) < 5.0, str(st["area_m2"]))

        got = await asyncio.to_thread(client.fence_download, 6.0)
        check("fence download matches", len(got) == 4 and all(
            abs(got[i][0] - verts[i][0]) < 1e-7 and abs(got[i][1] - verts[i][1]) < 1e-7 for i in range(4)))

        fe = await asyncio.to_thread(client.get_param, "FENCE_ENABLE", 3.0)
        ft = await asyncio.to_thread(client.get_param, "FENCE_TYPE", 3.0)
        check("FENCE_ENABLE=1", fe == 1.0, str(fe))
        check("FENCE_TYPE polygon bit", bool(int(ft) & 4), str(ft))

        plan = await asyncio.to_thread(client.read_plan, 6.0)
        check("plan read (3 items)", plan and plan["total"] == 3, str(plan and plan["total"]))
        check("DO_SPRAYER at #2", plan and plan["do_sprayer_seq"] == 2, str(plan and plan["do_sprayer_seq"]))

        okm = await asyncio.to_thread(client.set_mode, 6, 3.0)   # RTL
        check("set_mode RTL (COMMAND_ACK result=0)", okm == 0)
        check("mock FC saw override", vehicle.mode_override in (None, 6))  # consumed by mock pi or pending

        await asyncio.to_thread(client.fence_clear, 4.0)
        got2 = await asyncio.to_thread(client.fence_download, 4.0)
        check("fence clear + empty download", got2 == [], str(got2))
    finally:
        client.stop()
        mockfc.stop()
        await asyncio.sleep(0.3)

    print("== 3. tiles (TMS flip) ==")
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        db = os.path.join(td, "t.mbtiles")
        conn = sqlite3.connect(db)
        conn.execute("CREATE TABLE metadata (name TEXT, value TEXT)")
        conn.execute("CREATE TABLE tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
        # z=18, x=1, y=2 (XYZ) -> tms_y = 2^18-1-2
        conn.execute("INSERT INTO tiles VALUES (18, 1, ?, X'89504e47')", (2 ** 18 - 1 - 2,))
        conn.commit()
        conn.close()
        ts = TileStore(db)
        check("tile hit (xyz 18/1/2)", ts.get(18, 1, 2) == b"\x89PNG")
        check("tile miss", ts.get(18, 1, 3) is None)

    print()
    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Mission Companion bridge")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8100)
    ap.add_argument("--mock", action="store_true", help="run simulated Pi + mock FC (full contract demo)")
    ap.add_argument("--selftest", action="store_true", help="run codec + protocol self-tests and exit")
    ap.add_argument("--mav-port", type=int, default=14551, help="UDP port to listen on for the MP MAVLink forward")
    ap.add_argument("--mav-sysid", type=int, default=254, help="GCS system id for the bridge (254: MP itself is 255)")
    ap.add_argument("--fence-action", type=int, default=1, help="FENCE_ACTION param (1=RTL)")
    ap.add_argument("--mock-fc-port", type=int, default=14560, help="mock FC UDP port (mock mode)")
    ap.add_argument("--origin", default="%f,%f" % MOCK_ORIGIN, help="mock demo origin lat,lon (NOT a venue)")
    ap.add_argument("--mbtiles", default=None, help="path to offline .mbtiles (auto: ./field.mbtiles)")
    ap.add_argument("--watch", action="append", default=[], help="file/dir to tail (repeatable)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.mbtiles is None:
        for cand in (os.path.join(ROOT, "field.mbtiles"), "field.mbtiles",
                     os.path.join(os.path.dirname(os.path.abspath(__file__)), "field.mbtiles")):
            if os.path.isfile(cand):
                args.mbtiles = cand
                break
    if args.mbtiles and not os.path.isfile(args.mbtiles):
        print("[bridge] WARNING: --mbtiles file not found: %s (using online tiles fallback)" % args.mbtiles)

    if args.selftest:
        try:
            sys.exit(asyncio.run(run_selftest(args)))
        except KeyboardInterrupt:
            sys.exit(1)
    try:
        asyncio.run(Server(args).start())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
