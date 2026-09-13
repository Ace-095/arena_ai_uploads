"""Flight-controller link over USB (pymavlink).

- Auto-detects the Pixhawk: /dev/serial/by-id/*, /dev/ttyACM*, /dev/ttyUSB*,
  probing bauds until a heartbeat answers. Explicit --device also accepts
  pymavlink strings (udp:/tcp:) for SITL bench tests.
- Single reader thread + subscribe/wait_for fanout (same proven pattern as
  mission-ui's bridge): nothing else touches recv_match().
- Pi presents as system 51 / ONBOARD_CONTROLLER so it never collides with
  MP (255) or the laptop UI (254).
- High-level ops the mission needs: home, fence, plan+sprayer scan, mode,
  guided goto/yaw, STATUSTEXT relay, battery.

This link is DIRECT (USB) — the full mission protocol works here, unlike
the MP-forwarding path the laptop UI sits behind.
"""
import glob
import logging
import os
import queue
import threading
import time

log = logging.getLogger("fc_link")

try:
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil
    _HAVE_PYMAVLINK = True
except Exception as e:
    mavutil = None
    _HAVE_PYMAVLINK = False
    _IMPORT_ERR = e

COPTER_MODES = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
                "GUIDED": 4, "LOITER": 5, "RTL": 6, "CIRCLE": 7,
                "POSITION": 8, "LAND": 9, "OF_LOITER": 10, "DRIFT": 11,
                "SPORT": 13, "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16,
                "BRAKE": 17, "THROW": 18, "AVOID_ADSB": 19,
                "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22,
                "FOLLOW": 23, "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26,
                "AUTO_RTL": 27, "TURTLE": 28}

DO_SPRAYER = 216
MASK_POS_ONLY = 0b0000111111111000  # SET_POSITION_TARGET_*: position only


def _require_pymavlink():
    if not _HAVE_PYMAVLINK:
        raise RuntimeError("pymavlink not installed (%r)" % (_IMPORT_ERR,))


def candidate_devices():
    """Ordered serial candidates: stable by-id names first."""
    devs = []
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.exists(p):
            devs.append(p)
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for p in sorted(glob.glob(pat)):
            if p not in devs:
                devs.append(p)
    return devs


def find_fc(bauds=(115200, 57600, 921600), hb_timeout=3.0, device=None):
    """Return (conn, device) for the first FC that answers a heartbeat."""
    _require_pymavlink()
    if device:
        devs = [device]
    else:
        devs = candidate_devices()
        if not devs:
            raise RuntimeError("no serial candidates (no /dev/ttyACM* or /dev/ttyUSB*)")
    errors = []
    for dev in devs:
        for baud in bauds:
            try:
                conn = mavutil.mavlink_connection(
                    dev, baud=baud, source_system=51,
                    source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
            except Exception as e:
                errors.append("%s@%d: %s" % (dev, baud, e))
                continue
            try:
                hb = conn.wait_heartbeat(timeout=hb_timeout)
            except Exception:
                hb = None
            if hb is not None:
                log.info("FC found: %s @ %d (sysid=%d compid=%d)", dev, baud,
                         hb.get_srcSystem(), hb.get_srcComponent())
                return conn, dev
            try:
                conn.close()
            except Exception:
                pass
            errors.append("%s@%d: no heartbeat" % (dev, baud))
    raise RuntimeError("no FC heartbeat; tried: %s" % "; ".join(errors[:8]))


class FCError(Exception):
    pass


class FCLink:
    def __init__(self):
        self.conn = None
        self.device = None
        self.target_system = 1
        self.target_component = 1
        self.veh_sysid = None
        self.mode = None
        self.mode_num = None
        self.armed = False
        self.last_hb = 0.0
        self._subs = {}
        self._subs_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = None
        self._hb_thread = None

    # -- lifecycle ------------------------------------------------------
    def connect(self, device=None, bauds=(115200, 57600, 921600)):
        _require_pymavlink()
        conn, dev = find_fc(bauds=bauds, device=device)
        self.conn = conn
        self.device = dev
        self.target_system = conn.target_system or 1
        self.target_component = conn.target_component or 1
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="fc-reader", daemon=True)
        self._reader.start()
        self._hb_thread = threading.Thread(target=self._hb_loop, name="fc-hb", daemon=True)
        self._hb_thread.start()
        return dev

    def close(self):
        self._stop.set()
        for t in (self._reader, self._hb_thread):
            if t:
                t.join(timeout=2.0)
        try:
            self.conn and self.conn.close()
        except Exception:
            pass
        self.conn = None

    def link_ok(self, max_hb_age=3.0):
        return self.conn is not None and (time.time() - self.last_hb) < max_hb_age

    # -- pub/sub --------------------------------------------------------
    def subscribe(self, types):
        q = queue.Queue(maxsize=100)
        with self._subs_lock:
            for t in types:
                self._subs.setdefault(t, []).append(q)
        return q

    def unsubscribe(self, q):
        with self._subs_lock:
            for lst in self._subs.values():
                if q in lst:
                    lst.remove(q)

    def wait_for(self, types, pred, timeout):
        q = self.subscribe(types)
        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    m = q.get(timeout=max(0.05, deadline - time.time()))
                except queue.Empty:
                    continue
                try:
                    if pred(m):
                        return m
                except Exception:
                    continue
            return None
        finally:
            self.unsubscribe(q)

    def _send(self, fn, *a, **k):
        with self._send_lock:
            if self.conn is None:
                raise FCError("not connected")
            fn(*a, **k)

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                m = self.conn.recv_match(blocking=True, timeout=0.3)
            except Exception:
                time.sleep(0.1)
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == "HEARTBEAT" and m.get_srcSystem() != 51:
                if m.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self.veh_sysid = m.get_srcSystem()
                    self.last_hb = time.time()
                self._track_mode(m)
            with self._subs_lock:
                targets = list(self._subs.get(t, [])) + list(self._subs.get("*", []))
            for q in targets:
                try:
                    q.put_nowait(m)
                except queue.Full:
                    try:
                        q.get_nowait()
                        q.put_nowait(m)
                    except queue.Empty:
                        pass

    def _track_mode(self, hb):
        try:
            mmap = {v: k for k, v in self.conn.mode_mapping().items()}
            name = mmap.get(hb.custom_mode)
        except Exception:
            name = None
        if name is None:
            name = {v: k for k, v in COPTER_MODES.items()}.get(hb.custom_mode,
                                                              "MODE(%d)" % hb.custom_mode)
        if name != self.mode:
            log.info("FC mode: %s -> %s", self.mode, name)
        self.mode = name
        self.mode_num = hb.custom_mode
        self.armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _hb_loop(self):
        while not self._stop.is_set():
            try:
                self._send(self.conn.mav.heartbeat_send,
                           mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                           mavutil.mavlink.MAV_STATE_ACTIVE)
            except Exception:
                pass
            time.sleep(1.0)

    # -- ops ------------------------------------------------------------
    def _cmd_long(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, ack_timeout=4.0):
        self._send(self.conn.mav.command_long_send, self.target_system,
                   self.target_component, cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        if ack_timeout:
            ack = self.wait_for(["COMMAND_ACK"],
                                lambda m: m.command == cmd, ack_timeout)
            if ack is None:
                raise FCError("no COMMAND_ACK for cmd %d" % cmd)
            return int(ack.result)
        return None

    def request_message(self, msgid, ack_timeout=2.0):
        try:
            self._cmd_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, float(msgid),
                           ack_timeout=ack_timeout)
        except FCError:
            pass  # some builds don't ACK requests; the message may still come

    def get_home(self, timeout=4.0):
        self.request_message(242)  # HOME_POSITION
        m = self.wait_for(["HOME_POSITION"], lambda m: True, timeout)
        if m is None:
            raise FCError("no HOME_POSITION from FC")
        if m.latitude == 0 and m.longitude == 0:
            raise FCError("FC home is zero/unset")
        return (m.latitude / 1e7, m.longitude / 1e7, m.altitude / 1e3)

    def get_position(self, timeout=4.0):
        m = self.wait_for(["GLOBAL_POSITION_INT"], lambda m: True, timeout)
        if m is None:
            raise FCError("no GLOBAL_POSITION_INT from FC")
        return {"lat": m.lat / 1e7, "lon": m.lon / 1e7,
                "alt_msl": m.alt / 1e3, "alt_rel": m.relative_alt / 1e3,
                "hdg": None if m.hdg == 65535 else m.hdg / 100.0}

    def get_battery(self, timeout=2.0):
        m = self.wait_for(["BATTERY_STATUS"], lambda m: True, timeout)
        if m is None:
            return {"pct": None, "v": None}
        return {"pct": int(m.battery_remaining) if m.battery_remaining >= 0 else None,
                "v": (m.voltages[0] / 1000.0) if m.voltages and m.voltages[0] > 0 else None}

    def _download_mission_type(self, mtype, timeout=8.0):
        ts, tc = self.target_system, self.target_component
        deadline = time.time() + timeout
        q = self.subscribe(["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"])
        try:
            self._send(self.conn.mav.mission_request_list_send, ts, tc, mtype)
            cnt = self.wait_for(["MISSION_COUNT"],
                                lambda m, mt=mtype: m.mission_type == mt, 4.0)
            if cnt is None:
                raise FCError("no MISSION_COUNT (type %d)" % mtype)
            items = []
            for seq in range(cnt.count):
                self._send(self.conn.mav.mission_request_int_send, ts, tc, seq, mtype)
                it = self.wait_for(
                    ["MISSION_ITEM_INT"],
                    lambda m, s=seq, mt=mtype: m.mission_type == mt and m.seq == s,
                    max(0.5, deadline - time.time()))
                if it is None:
                    raise FCError("timeout on item %d (type %d)" % (seq, mtype))
                items.append(it)
            try:
                self._send(self.conn.mav.mission_ack_send, ts, tc,
                           mavutil.mavlink.MAV_MISSION_ACCEPTED, mtype)
            except Exception:
                pass
            return items
        finally:
            self.unsubscribe(q)

    def read_plan(self, timeout=8.0):
        items = self._download_mission_type(
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION, timeout)
        return [{"seq": m.seq, "command": m.command,
                 "lat": m.x / 1e7, "lon": m.y / 1e7, "alt": m.z,
                 "param1": m.param1} for m in items]

    def read_fence(self, timeout=8.0):
        items = self._download_mission_type(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE, timeout)
        return [(m.x / 1e7, m.y / 1e7) for m in items]

    def find_sprayer_seq(self, plan):
        for it in plan:
            if it["command"] == DO_SPRAYER:
                return it["seq"]
        return None

    def set_mode(self, mode, timeout=5.0):
        num = COPTER_MODES.get(str(mode).upper(), mode) if isinstance(mode, str) else mode
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout:
            try:
                last = self._cmd_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                      float(num), ack_timeout=3.0)
            except FCError as e:
                last = "timeout: %s" % e
            if last == 0:
                return 0
            time.sleep(0.5)
        raise FCError("set_mode(%s) failed: %r" % (mode, last))

    def goto_global(self, lat, lon, alt_rel_m):
        """GUIDED goto (position only, relative altitude). Fire-and-hold:
        ArduPilot holds the last target, so callers re-send at ~1 Hz."""
        self._send(self.conn.mav.set_position_target_global_int_send,
                   0, self.target_system, self.target_component,
                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                   MASK_POS_ONLY, int(lat * 1e7), int(lon * 1e7),
                   float(alt_rel_m), 0, 0, 0, 0, 0, 0, 0, 0)

    def condition_yaw(self, angle_deg, speed_deg_s=15.0, relative=True, direction=1):
        self._cmd_long(mavutil.mavlink.MAV_CMD_CONDITION_YAW, float(angle_deg),
                       float(speed_deg_s), float(direction),
                       1.0 if relative else 0.0, ack_timeout=0)

    def send_statustext(self, text, severity=6):
        raw = str(text).encode("utf-8")[:50]
        self._send(self.conn.mav.statustext_send, int(severity), raw)
