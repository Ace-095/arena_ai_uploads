"""
mav_client.py — MAVLink GCS client over UDP for the Mission Companion bridge.

Role (ui-spec v2): the laptop has no radio of its own. Mission Planner owns
the telemetry link to the Pixhawk; its "MAVLink Forwarding" (with Write
Access) pipes the same MAVLink stream to this client over UDP (default
127.0.0.1:14551). So:

    Pixhawk <==telemetry radio==> Mission Planner <==UDP 14551==> THIS

  * IN : telemetry (GPS/pos/att/battery/mode/armed), STATUSTEXT (the MP
         console stream — QR results land here), mission status, fence status.
  * OUT: fence polygon upload (mission_type=FENCE) + readback + FENCE_*
         params, plan read (DO_SPRAYER discovery), params, DO_SET_MODE
         (RTL/LAND — the only flight commands the UI may send; MP stays
         the head for arming/takeoff), STATUSTEXT notices.

Protocol discipline (ported from hehe's validated mavlink_fence.py):
  * subscribe BEFORE send on every request/response pair (the reply can
    beat a late subscriber on a fast link — a proven race).
  * vertex_count (param1) on EVERY MISSION_ITEM_INT, not just the first.
  * readback-verify after upload (1e-7 deg tolerance ~ 1 cm).

Transport: plain UDP, one frame per datagram (MAVLink-over-UDP convention).
Replies go back to the last-seen sender (the MP forwarder). No signing.
"""
import math
import queue
import re
import socket
import struct
import threading
import time

import _mavlink_v2 as M

QR_RE = re.compile(r"QR[:\s]+([0-9A-Za-z]{1,6})\b")

EARTH_RADIUS_M = 6371000.0


# ---------------------------------------------------------------------------
# ENU polygon math (flat-earth — identical to ui js/geo.js; keep in sync)
# ---------------------------------------------------------------------------
def latlon_to_enu(lat, lon, olat, olon):
    dlat = math.radians(lat - olat)
    dlon = math.radians(lon - olon)
    return (dlon * EARTH_RADIUS_M * math.cos(math.radians(olat)),
            dlat * EARTH_RADIUS_M)


def polygon_area_m2(enu):
    a = 0.0
    for i in range(len(enu)):
        x0, y0 = enu[i]
        x1, y1 = enu[(i + 1) % len(enu)]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def polygon_centroid(enu):
    """Area-weighted centroid (degenerate fallback: vertex mean)."""
    n = len(enu)
    a = cx = cy = 0.0
    for i in range(n):
        x0, y0 = enu[i]
        x1, y1 = enu[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        a += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    a *= 0.5
    if abs(a) < 1e-9:
        return (sum(p[0] for p in enu) / n, sum(p[1] for p in enu) / n)
    return (cx / (6 * a), cy / (6 * a))


def max_radius_m(enu, centroid):
    return max(math.hypot(p[0] - centroid[0], p[1] - centroid[1]) for p in enu)


def build_fence_status(vertices, origin_lat, origin_lon, confirmed, reason):
    enu = [latlon_to_enu(v[0], v[1], origin_lat, origin_lon) for v in vertices]
    cx, cy = polygon_centroid(enu)
    return {
        "loaded": True,
        "confirmed": bool(confirmed),
        "reason": reason,
        "vertex_count": len(vertices),
        "area_m2": round(polygon_area_m2(enu), 1),
        "max_radius_m": round(max_radius_m(enu, (cx, cy)), 1),
        "centroid_enu": {"x": round(cx, 2), "y": round(cy, 2)},
        "origin_lat": round(origin_lat, 7),
        "origin_lon": round(origin_lon, 7),
        "vertices_latlon": [{"lat": v[0], "lon": v[1]} for v in vertices],
    }


class MavError(Exception):
    pass


class MavClient:
    def __init__(self, hub, port=14551, sysid=255, compid=1,
                 fence_action=1, verbose=False):
        self.hub = hub
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.fence_action = fence_action
        self.verbose = verbose

        self.sock = None
        self.sender = None            # (ip, port) of the forwarder (MP / mock FC)
        self.veh_sysid = None
        self.veh_type = None
        self.connected = False
        self.last_hb = 0.0

        # latest telemetry values
        self.pos = None               # lat, lon, alt_msl, alt_rel, vx..vz, hdg
        self.att = None               # roll, pitch, yaw (deg)
        self.gps = None               # fix, sats
        self.batt = None              # v, pct, current_a
        self.mode_num = None
        self.mode = None
        self.armed = False
        self.veh_state = None
        self.sysstatus = None
        self.mission = None           # seq, total
        self.fence_breach = None      # last FENCE_STATUS msg

        self.params = {}
        self.fence_status = None      # GeofenceStatus dict (last known)
        self.plan = None              # {items, do_sprayer_seq, synced}

        self.rx_msgs = 0
        self.rx_rate = 0.0
        self._rx_window = 0
        self._rx_window_t = time.time()

        self._subs = {}               # queue -> set(msg names)
        self._subs_lock = threading.Lock()
        self._seq = 0
        self._stop = threading.Event()
        self._rebind_port = None
        self._lock = threading.Lock()
        self._last_telem = 0.0
        self._last_state = 0.0
        self._mode_key = None
        self._started = False

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #
    def start(self):
        if self._started:
            return
        self._started = True
        threading.Thread(target=self._reader_loop, name="mav-reader", daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, name="mav-hb", daemon=True).start()
        self._maybe_emit_state()

    def stop(self):
        self._stop.set()
        try:
            self.sock and self.sock.close()
        except OSError:
            pass

    def _open_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("", self.port))
        s.settimeout(0.25)
        return s

    # ------------------------------------------------------------------ #
    # raw send
    # ------------------------------------------------------------------ #
    def _send(self, msgid, values):
        with self._lock:
            seq = self._seq
            self._seq = (seq + 1) & 0xFF
        frame = M.build_frame(msgid, self.sysid, self.compid, seq, values)
        if self.sender is None:
            raise MavError("no MAVLink link (nothing is forwarding to port %d — "
                           "check MP: right-click vehicle → MAVLink Forwarding → "
                           "127.0.0.1:%d + Write access)" % (self.port, self.port))
        try:
            self.sock.sendto(frame, self.sender)
            return True
        except OSError:
            return False

    def _tsend(self, msgid, values):
        """Send, swallow errors (fire-and-forget, e.g. heartbeats)."""
        if self.sender is None:
            return
        try:
            self.sock.sendto(M.build_frame(msgid, self.sysid, self.compid,
                                           self._next_seq(), values), self.sender)
        except OSError:
            pass

    def _next_seq(self):
        with self._lock:
            s = self._seq
            self._seq = (s + 1) & 0xFF
            return s

    # ------------------------------------------------------------------ #
    # pub/sub (subscribe-before-send)
    # ------------------------------------------------------------------ #
    def _subscribe(self, names):
        q = queue.Queue(maxsize=200)
        with self._subs_lock:
            self._subs[q] = set(names)
        return q

    def _unsubscribe(self, q):
        with self._subs_lock:
            self._subs.pop(q, None)

    def _fanout(self, name, msg):
        with self._subs_lock:
            subs = list(self._subs.items())
        for q, names in subs:
            if name in names:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    try:
                        q.get_nowait()
                        q.put_nowait(msg)
                    except queue.Empty:
                        pass

    def _pull_until(self, q, predicate, deadline):
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            try:
                m = q.get(timeout=remaining)
            except queue.Empty:
                continue
            if predicate(m):
                return m

    def _transact(self, expect, msgid, values, timeout, pred=None):
        """Subscribe, send, wait for first matching reply. Subscribe-first is
        mandatory (see module docstring)."""
        q = self._subscribe(expect)
        try:
            self._send(msgid, values)
            return self._pull_until(q, pred or (lambda m: True), time.time() + timeout)
        finally:
            self._unsubscribe(q)

    # ------------------------------------------------------------------ #
    # inbound dispatch
    # ------------------------------------------------------------------ #
    def _reader_loop(self):
        while not self._stop.is_set():
            if self._rebind_port is not None:
                try:
                    self.sock and self.sock.close()
                except OSError:
                    pass
                self.port = self._rebind_port
                self._rebind_port = None
                self.connected = False
                try:
                    self.sock = self._open_socket()
                    self.hub.emit_threadsafe("log", {"level": "INFO", "msg": "MAVLink listener re-bound to port %d" % self.port})
                except OSError as e:
                    self.hub.emit_threadsafe("log", {"level": "ERROR", "msg": "mav rebind failed: %r" % (e,)})
                    time.sleep(1.0)
                    continue
            if self.sock is None:
                try:
                    self.sock = self._open_socket()
                except OSError as e:
                    time.sleep(0.5)
                    continue
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                self._check_link_liveness()
                continue
            except OSError:
                if self._stop.is_set():
                    break
                time.sleep(0.2)
                continue
            parsed = M.parse_datagram(data)
            if parsed is None:
                continue
            msgid, src_sys, _src_comp, _seq, fields = parsed
            self.rx_msgs += 1
            self._rx_window += 1
            self._update_rx_rate(addr)
            if self.sender is None:
                self.sender = addr
                self.hub.emit_threadsafe("log", {
                    "level": "INFO",
                    "msg": "MAVLink link up: receiving from %s:%d" % (addr[0], addr[1])})
            name = M.name_of(msgid)
            self._fanout(name, (name, fields))
            self._dispatch(name, src_sys, fields)

    def _update_rx_rate(self, addr):
        now = time.time()
        if now - self._rx_window_t >= 5.0:
            self.rx_rate = self._rx_window / (now - self._rx_window_t)
            self._rx_window = 0
            self._rx_window_t = now

    def _check_link_liveness(self):
        was = self.connected
        self.connected = self.sender is not None and (time.time() - self.last_hb) < 2.5
        if was and not self.connected:
            self.hub.emit_threadsafe("log", {
                "level": "WARN", "msg": "MAVLink link LOST (no heartbeat >2.5 s)"})
        self._maybe_emit_state()

    def _dispatch(self, name, src_sys, f):
        if name == "HEARTBEAT":
            if f["type"] != M.MAV_TYPE_GCS:
                self.veh_sysid = src_sys
                self.veh_type = f["type"]
                self.last_hb = time.time()
                self.mode_num = f["custom_mode"]
                self.mode = M.ARDU_COPTER_MODES.get(f["custom_mode"], "MODE(%d)" % f["custom_mode"])
                self.armed = bool(f["base_mode"] & M.MAV_MODE_FLAG_SAFETY_ARMED)
                self.veh_state = f["system_status"]
                if not self.connected:
                    self.connected = True
                    self.hub.emit_threadsafe("log", {
                        "level": "INFO",
                        "msg": "VEHICLE CONNECTED: sysid=%d type=%s mode=%s" % (
                            src_sys, f["type"], self.mode)})
                self._maybe_emit_state()
        elif name == "GLOBAL_POSITION_INT":
            hdg = None if f["hdg"] == 65535 else f["hdg"] / 100.0
            if self.pos and hdg is None:
                hdg = self.pos.get("hdg")
            self.pos = {"lat": f["lat"] / 1e7, "lon": f["lon"] / 1e7,
                        "alt_msl": f["alt"] / 1e3, "alt_rel": f["relative_alt"] / 1e3,
                        "vx": f["vx"] / 100.0, "vy": f["vy"] / 100.0, "vz": f["vz"] / 100.0,
                        "hdg": hdg}
            self._maybe_telemetry()
        elif name == "ATTITUDE":
            d = 180.0 / math.pi
            self.att = {"roll": f["roll"] * d, "pitch": f["pitch"] * d, "yaw": f["yaw"] * d}
            self._maybe_telemetry()
        elif name == "GPS_RAW_INT":
            self.gps = {"fix": f["fix_type"], "sats": f["satellites_visible"]}
            self._maybe_telemetry()
        elif name == "BATTERY_STATUS":
            volt_mV = f["voltages"][0] if f["voltages"] and f["voltages"][0] > 0 else 0
            pct = f["battery_remaining"] if f["battery_remaining"] >= 0 else None
            self.batt = {
                "v": (volt_mV / 1000.0) if volt_mV else (self.batt or {}).get("v"),
                "pct": pct,
                "current_a": f["current_battery"] / 100.0 if f["current_battery"] >= 0 else None,
            }
            self._maybe_telemetry()
        elif name == "SYS_STATUS":
            self.sysstatus = f
            if not self.batt or self.batt.get("v") is None:
                self.batt = {"v": f["voltage_battery"] / 1000.0, "pct": None, "current_a": None}
            self._maybe_telemetry()
        elif name == "MISSION_CURRENT":
            self.mission = {"seq": f["seq"],
                            "total": (f.get("total") or 0) or (self.plan or {}).get("total")}
            self._maybe_telemetry()
        elif name == "MISSION_ITEM_REACHED":
            self.hub.emit_threadsafe("log", {
                "level": "INFO", "msg": "mission item reached: seq %d" % f["seq"]})
        elif name == "FENCE_STATUS":
            if f["breach_status"] != 0:
                self.fence_breach = f
                self.hub.emit_threadsafe("log", {
                    "level": "ERROR",
                    "msg": "FENCE BREACH: status=%d type=%d count=%d" % (
                        f["breach_status"], f["breach_type"], f["breach_count"])})
                self._maybe_emit_state()
        elif name == "PARAM_VALUE":
            pid = self._decode_id(f["param_id"])
            self.params[pid] = f["param_value"]
        elif name == "STATUSTEXT":
            text = f["text"].decode("utf-8", "replace").rstrip("\x00 ").strip()
            if not text:
                return
            self.hub.emit_threadsafe("mp-console", {
                "line": text, "ts": time.time(), "severity": f["severity"]})
            m = QR_RE.search(text)
            if m:
                self.hub.emit_threadsafe("mp-qr", {
                    "payload": m.group(1), "source": "mavlink",
                    "ts": time.time(), "line": text[:300]})
                self.hub.emit_threadsafe("log", {
                    "level": "WARN",
                    "msg": "QR via MAVLink (FC console): %s" % m.group(1)})
        elif name == "MISSION_COUNT":
            if f["mission_type"] == M.MAV_MISSION_TYPE_MISSION:
                # plan announcement (e.g. after MP uploads) — refresh lazily
                self._maybe_emit_state()

    @staticmethod
    def _decode_id(raw):
        if isinstance(raw, (bytes, bytearray)):
            return bytes(raw).rstrip(b"\x00").decode("ascii", "replace")
        return str(raw).rstrip("\x00")

    # ------------------------------------------------------------------ #
    # outbound envelopes
    # ------------------------------------------------------------------ #
    def _maybe_telemetry(self):
        if not self.pos:
            return
        now = time.time()
        if now - self._last_telem < 0.2:   # ≤5 Hz
            return
        self._last_telem = now
        self.hub.emit_threadsafe("telemetry", {
            "lat": self.pos["lat"], "lon": self.pos["lon"],
            "alt_rel": self.pos["alt_rel"], "alt_msl": self.pos["alt_msl"],
            "vx": self.pos["vx"], "vy": self.pos["vy"], "vz": self.pos["vz"],
            "hdg": self.pos.get("hdg"),
            "attitude": self.att,
            "mode": self.mode, "mode_num": self.mode_num,
            "armed": self.armed,
            "gps": self.gps,
            "battery": self.batt,
            "mission": self.mission,
        })

    def state_dict(self):
        return {
            "connected": self.connected,
            "port": self.port,
            "gcs_sysid": self.sysid,
            "vehicle_sysid": self.veh_sysid,
            "vehicle_type": self.veh_type,
            "mode": self.mode, "mode_num": self.mode_num,
            "armed": self.armed, "vehicle_state": self.veh_state,
            "hb_age_s": round(time.time() - self.last_hb, 2) if self.last_hb else None,
            "rx_msgs": self.rx_msgs,
            "rx_rate": round(self.rx_rate, 1),
            "fence": self.fence_status,
            "plan": self.plan,
        }

    def _maybe_emit_state(self):
        key = (self.connected, self.mode_num, self.armed, bool(self.fence_status), bool(self.plan))
        now = time.time()
        if key != self._mode_key or now - self._last_state > 10.0:
            self._mode_key = key
            self._last_state = now
            self.hub.emit_threadsafe("mavlink-state", self.state_dict())

    # ------------------------------------------------------------------ #
    # high-level protocol ops (BLOCKING — call via asyncio.to_thread)
    # ------------------------------------------------------------------ #
    def fence_upload(self, vertices, timeout=8.0):
        n = len(vertices)
        if not (3 <= n <= 255):
            raise MavError("fence needs 3-255 vertices (got %d)" % n)
        if self.sender is None:
            raise MavError("no MAVLink link — is Mission Planner forwarding to 127.0.0.1:%d ?" % self.port)
        ts = self.veh_sysid or 1

        # 1) EKF origin (best effort — fall back to polygon mean)
        origin_lat = origin_lon = None
        origin_reason = "EKF origin from FC"
        try:
            origin_lat, origin_lon = self._origin_request(timeout=3.0)
        except MavError:
            pass
        if origin_lat is None:
            origin_lat = sum(v[0] for v in vertices) / n
            origin_lon = sum(v[1] for v in vertices) / n
            origin_reason = "EKF origin unavailable — using polygon mean"

        # 2) upload (subscribe-before-send; FC drives per-seq requests)
        deadline = time.time() + timeout
        q = self._subscribe(["MISSION_REQUEST_INT", "MISSION_ACK"])
        try:
            self._send(M.MSG_ID['MISSION_COUNT'], {  # MISSION_COUNT
                "count": n, "target_system": ts, "target_component": 1,
                "mission_type": M.MAV_MISSION_TYPE_FENCE,
            })
            sent = set()
            ack = None
            while (len(sent) < n or ack is None) and time.time() < deadline:
                msg = self._pull_until(q, lambda m: True, time.time() + 0.5)
                if msg is None:
                    continue
                name, f = msg
                if f.get("mission_type") != M.MAV_MISSION_TYPE_FENCE:
                    continue
                if name == "MISSION_REQUEST_INT" and f["seq"] not in sent:
                    seq = f["seq"]
                    if seq >= n:
                        raise MavError("FC requested out-of-range fence seq %d (n=%d)" % (seq, n))
                    lat, lon = vertices[seq]
                    self._send(M.MSG_ID['MISSION_ITEM_INT'], {  # MISSION_ITEM_INT
                        "param1": float(n), "param2": 0.0, "param3": 0.0, "param4": 0.0,
                        "x": int(lat * 1e7), "y": int(lon * 1e7), "z": 0.0,
                        "seq": seq, "command": M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                        "target_system": ts, "target_component": 1,
                        "frame": M.MAV_FRAME_GLOBAL, "current": 0, "autocontinue": 0,
                        "mission_type": M.MAV_MISSION_TYPE_FENCE,
                    })
                    sent.add(seq)
                elif name == "MISSION_ACK":
                    ack = f
        finally:
            self._unsubscribe(q)
        if ack is None:
            raise MavError("fence upload timed out (FC not responding — check MP forwarding)")
        if ack["type"] != M.MAV_MISSION_ACCEPTED:
            raise MavError("fence upload REJECTED by FC (MAV_MISSION type=%d)" % ack["type"])

        # 3) readback verify
        confirmed = True
        readback_note = "readback matched"
        try:
            got = self.fence_download(timeout=6.0)
            confirmed = (len(got) == n and all(
                abs(got[i][0] - vertices[i][0]) < 1e-7 and
                abs(got[i][1] - vertices[i][1]) < 1e-7 for i in range(n)))
            if not confirmed:
                readback_note = "READBACK MISMATCH (got %d verts)" % len(got)
        except MavError as e:
            confirmed = False
            readback_note = "readback failed: %s" % e

        # 4) enable fence params (warn-only on failure — some firmwares differ)
        param_notes = []
        try:
            self.set_param("FENCE_ENABLE", 1.0, timeout=3.0)
        except MavError as e:
            param_notes.append("FENCE_ENABLE: %s" % e)
        try:
            self.set_param("FENCE_ACTION", float(self.fence_action), timeout=3.0)
        except MavError as e:
            param_notes.append("FENCE_ACTION: %s" % e)
        try:
            ft = self.get_param("FENCE_TYPE", timeout=3.0)
            if int(ft) & M.FENCE_TYPE_POLYGON_BIT == 0:
                self.set_param("FENCE_TYPE", float(int(ft) | M.FENCE_TYPE_POLYGON_BIT), timeout=3.0)
        except MavError as e:
            param_notes.append("FENCE_TYPE: %s" % e)

        reason = "Fence on FC — %s — %s" % (
            origin_reason, "FENCE_* params set" if not param_notes else "; ".join(param_notes))
        status = build_fence_status(vertices, origin_lat, origin_lon, confirmed, reason)
        self.fence_status = status
        self.hub.emit_threadsafe("fence", status)
        self.hub.emit_threadsafe("log", {
            "level": "WARN",
            "msg": "FENCE APPLIED: %d verts, %.0f m², %s — MP console gets STATUSTEXT notice" % (
                n, status["area_m2"], readback_note)})
        try:
            self.send_statustext("FENCE: %d verts, %s" % (n, "readback OK" if confirmed else "MISMATCH"))
        except MavError:
            pass
        self._maybe_emit_state()
        return status

    def fence_download(self, timeout=6.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        deadline = time.time() + timeout
        q = self._subscribe(["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"])
        try:
            self._send(M.MSG_ID['MISSION_REQUEST_LIST'], {  # MISSION_REQUEST_LIST
                "target_system": ts, "target_component": 1,
                "mission_type": M.MAV_MISSION_TYPE_FENCE,
            })
            cnt = self._pull_until(
                q, lambda m: m[0] == "MISSION_COUNT" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_FENCE,
                deadline)
            if cnt is None:
                raise MavError("no MISSION_COUNT from FC (fence download)")
            count = cnt[1]["count"]
            if count == 0:
                # protocol: with zero items the FC sends no MISSION_ACK
                return []
            verts = []
            for seq in range(count):
                self._send(M.MSG_ID['MISSION_REQUEST_INT'], {  # MISSION_REQUEST_INT
                    "seq": seq, "target_system": ts, "target_component": 1,
                    "mission_type": M.MAV_MISSION_TYPE_FENCE,
                })
                item = self._pull_until(
                    q, lambda m: m[0] == "MISSION_ITEM_INT" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_FENCE
                    and m[1]["seq"] == seq, deadline)
                if item is None:
                    raise MavError("timeout on fence item seq %d" % seq)
                verts.append((item[1]["x"] / 1e7, item[1]["y"] / 1e7))
            ack = self._pull_until(
                q, lambda m: m[0] == "MISSION_ACK" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_FENCE,
                deadline)
            if ack is None:
                if verts:
                    return verts   # items all in hand — tolerate missing ACK
                raise MavError("no MISSION_ACK after fence download")
            if ack[1]["type"] != M.MAV_MISSION_ACCEPTED:
                raise MavError("FC rejected fence download (type=%d)" % ack[1]["type"])
            return verts
        finally:
            self._unsubscribe(q)

    def fence_clear(self, timeout=4.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        q = self._subscribe(["MISSION_ACK"])
        try:
            self._send(M.MSG_ID['MISSION_CLEAR_ALL'], {  # MISSION_CLEAR_ALL
                "target_system": ts, "target_component": 1,
                "mission_type": M.MAV_MISSION_TYPE_FENCE,
            })
            ack = self._pull_until(
                q, lambda m: m[0] == "MISSION_ACK" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_FENCE,
                time.time() + timeout)
        finally:
            self._unsubscribe(q)
        if ack is None:
            raise MavError("timeout clearing fence")
        if ack[1]["type"] != M.MAV_MISSION_ACCEPTED:
            raise MavError("FC rejected fence clear (type=%d)" % ack[1]["type"])
        self.fence_status = {"loaded": False, "confirmed": False, "reason": "Fence cleared on FC",
                             "vertex_count": 0, "area_m2": None, "max_radius_m": None,
                             "centroid_enu": None, "origin_lat": None, "origin_lon": None,
                             "vertices_latlon": []}
        self.hub.emit_threadsafe("fence", self.fence_status)
        self._maybe_emit_state()
        return self.fence_status

    def refresh_fence(self, timeout=3.0):
        """Non-fatal: adopt whatever fence the FC already has (e.g. uploaded
        by Mission Planner)."""
        try:
            verts = self.fence_download(timeout=timeout)
        except MavError:
            return None
        if not verts:
            return None
        olat = sum(v[0] for v in verts) / len(verts)
        olon = sum(v[1] for v in verts) / len(verts)
        try:
            o = self._origin_request(timeout=2.0)
            if o:
                olat, olon = o
        except MavError:
            pass
        self.fence_status = build_fence_status(
            verts, olat, olon, True, "Fence read from FC (may have been set in Mission Planner)")
        self.hub.emit_threadsafe("fence", self.fence_status)
        self.hub.emit_threadsafe("log", {
            "level": "INFO", "msg": "existing fence adopted from FC: %d verts" % len(verts)})
        self._maybe_emit_state()
        return self.fence_status

    def _origin_request(self, timeout=3.0):
        ts = self.veh_sysid or 1
        q = self._subscribe(["GPS_GLOBAL_ORIGIN"])
        try:
            self._send(M.MSG_ID['COMMAND_LONG'], {
                "param1": float(M.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN),
                "param2": 0, "param3": 0, "param4": 0,
                "param5": 0, "param6": 0, "param7": 0,
                "command": M.MAV_CMD_REQUEST_MESSAGE,
                "target_system": ts, "target_component": 1, "confirmation": 0,
            })
            msg = self._pull_until(q, lambda m: m[0] == "GPS_GLOBAL_ORIGIN", time.time() + timeout)
        finally:
            self._unsubscribe(q)
        if msg is None:
            raise MavError("timeout waiting for GPS_GLOBAL_ORIGIN")
        f = msg[1]
        if f["latitude"] == 0 and f["longitude"] == 0:
            raise MavError("FC returned zero EKF origin")
        return (f["latitude"] / 1e7, f["longitude"] / 1e7)

    def read_plan(self, timeout=6.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        deadline = time.time() + timeout
        q = self._subscribe(["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"])
        try:
            self._send(M.MSG_ID['MISSION_REQUEST_LIST'], {"target_system": ts, "target_component": 1,
                            "mission_type": M.MAV_MISSION_TYPE_MISSION})
            cnt = self._pull_until(
                q, lambda m: m[0] == "MISSION_COUNT" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_MISSION,
                deadline)
            if cnt is None:
                raise MavError("no MISSION_COUNT from FC (plan read)")
            count = cnt[1]["count"]
            items = []
            for seq in range(count):
                self._send(M.MSG_ID['MISSION_REQUEST_INT'], {"seq": seq, "target_system": ts, "target_component": 1,
                                "mission_type": M.MAV_MISSION_TYPE_MISSION})
                item = self._pull_until(
                    q, lambda m: m[0] == "MISSION_ITEM_INT" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_MISSION
                    and m[1]["seq"] == seq, deadline)
                if item is None:
                    raise MavError("timeout on plan item seq %d" % seq)
                f = item[1]
                items.append({"seq": f["seq"], "command": f["command"],
                              "lat": f["x"] / 1e7 if abs(f["x"]) > 1e6 else 0.0,
                              "lon": f["y"] / 1e7 if abs(f["y"]) > 1e6 else 0.0,
                              "alt_m": f["z"]})
            self._pull_until(
                q, lambda m: m[0] == "MISSION_ACK" and m[1]["mission_type"] == M.MAV_MISSION_TYPE_MISSION,
                deadline)  # tolerate None — items already in hand
        finally:
            self._unsubscribe(q)
        do_seq = next((i + 1 for i, it in enumerate(items)
                       if it["command"] == M.MAV_CMD_DO_SPRAYER), None)
        self.plan = {"items": items, "total": len(items),
                     "do_sprayer_seq": do_seq, "synced": len(items) > 0,
                     "ts": time.time()}
        self.hub.emit_threadsafe("plan", self.plan)
        self.hub.emit_threadsafe("log", {
            "level": "INFO",
            "msg": "plan read from FC: %d items%s" % (
                len(items), "" if do_seq is None else ", DO_SPRAYER trigger at #%d" % do_seq)})
        self._maybe_emit_state()
        return self.plan

    def get_param(self, name, timeout=3.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        q = self._subscribe(["PARAM_VALUE"])
        try:
            self._send(M.MSG_ID['PARAM_REQUEST_READ'], {  # PARAM_REQUEST_READ
                "param_index": -1, "target_system": ts, "target_component": 1,
                "param_id": name.encode("ascii"),
            })
            msg = self._pull_until(
                q, lambda m: m[0] == "PARAM_VALUE" and self._decode_id(m[1]["param_id"]) == name,
                time.time() + timeout)
        finally:
            self._unsubscribe(q)
        if msg is None:
            raise MavError("timeout reading param %s" % name)
        return float(msg[1]["param_value"])

    def set_param(self, name, value, timeout=3.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        q = self._subscribe(["PARAM_VALUE"])
        try:
            self._send(M.MSG_ID['PARAM_SET'], {  # PARAM_SET
                "param_value": float(value), "target_system": ts, "target_component": 1,
                "param_id": name.encode("ascii"), "param_type": M.MAV_PARAM_TYPE_REAL32,
            })
            msg = self._pull_until(
                q, lambda m: m[0] == "PARAM_VALUE" and self._decode_id(m[1]["param_id"]) == name,
                time.time() + timeout)
        finally:
            self._unsubscribe(q)
        if msg is None:
            raise MavError("timeout setting param %s" % name)
        if abs(msg[1]["param_value"] - value) > 0.01:
            raise MavError("param %s is %s, expected %s" % (name, msg[1]["param_value"], value))
        self.params[name] = value
        return value

    def set_mode(self, mode_num, timeout=3.0):
        if self.sender is None:
            raise MavError("no MAVLink link")
        ts = self.veh_sysid or 1
        q = self._subscribe(["COMMAND_ACK"])
        try:
            self._send(M.MSG_ID['COMMAND_LONG'], {  # COMMAND_LONG DO_SET_MODE
                "param1": float(mode_num),
                "param2": 0, "param3": 0, "param4": 0,
                "param5": 0, "param6": 0, "param7": 0,
                "command": M.MAV_CMD_DO_SET_MODE,
                "target_system": ts, "target_component": 1, "confirmation": 0,
            })
            ack = self._pull_until(
                q, lambda m: m[0] == "COMMAND_ACK" and m[1]["command"] == M.MAV_CMD_DO_SET_MODE,
                time.time() + timeout)
        finally:
            self._unsubscribe(q)
        if ack is None:
            raise MavError("timeout waiting for COMMAND_ACK (mode set)")
        return ack[1]["result"] == 0

    def send_statustext(self, text, severity=M.MAV_SEVERITY_INFO):
        self._send(M.MSG_ID['STATUSTEXT'], {"severity": severity, "text": text.encode("utf-8")[:50]})

    def _heartbeat_loop(self):
        while not self._stop.is_set():
            try:
                self._tsend(0, {  # HEARTBEAT (GCS)
                    "custom_mode": 0, "type": M.MAV_TYPE_GCS,
                    "autopilot": M.MAV_AUTOPILOT_GCS, "base_mode": 0,
                    "system_status": M.MAV_STATE_ACTIVE, "mavlink_version": 3,
                })
            except Exception:
                pass
            time.sleep(1.0)

    def rebind(self, port):
        self._rebind_port = int(port)
