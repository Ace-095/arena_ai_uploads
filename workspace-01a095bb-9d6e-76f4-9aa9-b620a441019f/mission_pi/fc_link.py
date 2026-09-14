"""Flight-controller link over USB or TCP/UDP (SITL bench) via pymavlink.

  probing bauds until a heartbeat answers. Explicit --device (or fc.conn in
  config) also accepts pymavlink network strings — tcp:/udp:/udpin: — for
  SITL bench runs (single attempt, no baud probing on network links).
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


def _is_network_device(dev):
    return str(dev).lower().startswith(
        ("tcp:", "udp:", "udpin:", "udpout:", "udpbcast:"))


def _wait_vehicle_heartbeat(conn, timeout):
    """First non-GCS heartbeat on the wire (sysid < 200, type != GCS).

    Bench links also carry Mission Planner's forwarded heartbeats
    (sysid 255, custom_mode=0). Accepting one of those here would route
    every command at MP and poison mode tracking, so the vehicle
    heartbeat is filtered the same way the ADDC reference stack does.
    Falls back to ANY heartbeat when no vehicle one appears.
    """
    t_end = time.time() + timeout
    fallback = None
    while time.time() < t_end:
        try:
            hb = conn.wait_heartbeat(timeout=max(0.5, t_end - time.time()))
        except Exception:
            hb = None
        if hb is None:
            break
        if fallback is None:
            fallback = hb
        try:
            sysid = hb.get_srcSystem()
        except Exception:
            sysid = 0
        if sysid and sysid < 200 and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            return hb
    return fallback


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
        attempt_bauds = (None,) if _is_network_device(dev) else bauds
        for baud in attempt_bauds:
            try:
                kw = dict(source_system=51,
                          source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
                if baud is not None:
                    kw["baud"] = baud
                if _is_network_device(dev):
                    # SITL/Gazebo restarts and laptop sleep/wake drop the TCP
                    # socket; pymavlink's own autoreconnect puts it back
                    # without us rebuilding the whole connection (our watchdog
                    # below is the second line of defence, and the only one
                    # that works for a USB replug).
                    kw["autoreconnect"] = True
                conn = mavutil.mavlink_connection(dev, **kw)
            except Exception as e:
                errors.append("%s@%s: %s" % (dev, baud, e))
                continue
            hb = _wait_vehicle_heartbeat(conn, hb_timeout)
            if hb is not None:
                # wait_heartbeat() retargets the connection on EVERY heartbeat
                # it sees — pin routing back to the heartbeat we selected.
                conn.target_system = hb.get_srcSystem()
                conn.target_component = hb.get_srcComponent()
                log.info("FC found: %s @ %s (sysid=%d compid=%d)", dev,
                         baud if baud else "net",
                         hb.get_srcSystem(), hb.get_srcComponent())
                return conn, dev
            try:
                conn.close()
            except Exception:
                pass
            errors.append("%s@%s: no heartbeat" % (dev, baud if baud else "net"))
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
        self._mode_ts = 0.0      # last VEHICLE heartbeat (mode tracking input)
        self._hb_sources = {}    # (sysid, type) -> count; proves who is on the wire
        self.armed = False
        self.last_hb = 0.0
        self._subs = {}
        self._subs_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._conn_lock = threading.RLock()
        self._stop = threading.Event()
        self._reader = None
        self._hb_thread = None
        self._stream_thread = None
        # link supervision (a dead USB/TCP link must not leave us deaf forever)
        self.link_down = False
        self.reconnects = 0
        self._watchdog = None
        self._wd_stop = threading.Event()
        self._wd_dead_s = 10.0
        self._wd_retry_s = 5.0
        self._on_event = None
        self._bauds = (115200, 57600, 921600)

    # -- lifecycle ------------------------------------------------------
    def connect(self, device=None, bauds=(115200, 57600, 921600),
                supervise=True, on_event=None, dead_s=10.0, retry_s=5.0):
        """Acquire the FC and (by default) supervise it forever after.

        supervise: a watchdog thread notices a missing vehicle heartbeat and
        rebuilds the connection, so a USB replug, an SITL restart or a radio
        drop recovers on its own instead of leaving mission_pi permanently
        deaf (which used to look like "the mission just stopped working").
        on_event(kind, detail) is called for link_lost / link_retry /
        link_restored — main.py forwards those to the UI log.
        """
        _require_pymavlink()
        self._bauds = tuple(bauds or (115200, 57600, 921600))
        if device:
            # publish the intended device immediately: while find_fc is
            # failing, /health must say WHAT we are trying to reach instead
            # of showing device=None (which reads as "not configured").
            self.device = device
        conn, dev = find_fc(bauds=self._bauds, device=device)
        self._adopt(conn, dev)
        if supervise:
            self.start_watchdog(dead_s=dead_s, retry_s=retry_s,
                                on_event=on_event)
        return dev

    def _adopt(self, conn, dev):
        """Install a fresh connection and (re)start the three link threads."""
        with self._conn_lock:
            self.conn = conn
            self.device = dev
            self.target_system = conn.target_system or 1
            self.target_component = conn.target_component or 1
            self.last_hb = time.time()      # find_fc just saw a heartbeat
            self.link_down = False
            self._stop.clear()
            self._reader = threading.Thread(target=self._read_loop,
                                            name="fc-reader", daemon=True)
            self._reader.start()
            self._hb_thread = threading.Thread(target=self._hb_loop,
                                               name="fc-hb", daemon=True)
            self._hb_thread.start()
            self._stream_thread = threading.Thread(target=self._stream_loop,
                                                   name="fc-streams", daemon=True)
            self._stream_thread.start()

    def _teardown(self):
        """Stop the link threads and drop the socket (used by reconnect)."""
        self._stop.set()
        for t in (self._reader, self._hb_thread, self._stream_thread):
            if t and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.5)
        self._reader = self._hb_thread = self._stream_thread = None
        with self._conn_lock:
            conn, self.conn = self.conn, None
        try:
            conn and conn.close()
        except Exception:
            pass

    def reconnect(self):
        """Drop and re-acquire the FC on the same device. Raises on failure."""
        _require_pymavlink()
        self._teardown()
        conn, dev = find_fc(bauds=self._bauds, device=self.device)
        self._adopt(conn, dev)
        self.reconnects += 1
        log.warning("FC link re-established on %s (reconnect #%d)", dev,
                    self.reconnects)
        return dev

    # -- link supervision ------------------------------------------------
    def start_watchdog(self, dead_s=10.0, retry_s=5.0, on_event=None):
        if on_event is not None:
            self._on_event = on_event
        self._wd_dead_s = max(3.0, float(dead_s))
        self._wd_retry_s = max(1.0, float(retry_s))
        if self._watchdog is not None and self._watchdog.is_alive():
            return
        self._wd_stop.clear()
        self._watchdog = threading.Thread(target=self._watch_loop,
                                          name="fc-watchdog", daemon=True)
        self._watchdog.start()
        log.info("FC watchdog up (dead after %.0fs, retry every %.0fs)",
                 self._wd_dead_s, self._wd_retry_s)

    def stop_watchdog(self):
        self._wd_stop.set()
        t = self._watchdog
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=2.0)
        self._watchdog = None

    def _event(self, kind, detail):
        log.warning("fc link %s: %s", kind, detail)
        cb = self._on_event
        if cb is None:
            return
        try:
            cb(kind, detail)
        except Exception:
            pass

    def _watch_loop(self):
        down_since = None
        while not self._wd_stop.wait(self._wd_retry_s):
            hb_age = (time.time() - self.last_hb) if self.last_hb else float("inf")
            alive = (self.conn is not None) and hb_age < self._wd_dead_s
            if alive:
                if down_since is not None:
                    self.link_down = False
                    self._event("link_restored", "vehicle heartbeat is back "
                                                 "(%.1fs old)" % hb_age)
                    down_since = None
                continue
            if down_since is None:
                down_since = time.time()
                self.link_down = True
                self._event("link_lost", "no vehicle heartbeat for %.0fs on %s "
                                         "— reconnecting" % (hb_age, self.device))
                continue
            try:
                dev = self.reconnect()
                down_since = None
                self._event("link_restored", "reconnected to %s (#%d)"
                            % (dev, self.reconnects))
            except Exception as e:
                self._event("link_retry", "reconnect failed: %s" % e)

    def link_state(self):
        """JSON-safe link summary for /health and the UI."""
        return {"device": self.device,
                "connected": self.conn is not None,
                "down": bool(self.link_down),
                "reconnects": int(self.reconnects),
                "hb_age_s": (round(time.time() - self.last_hb, 1)
                             if self.last_hb else None),
                "mode": self.mode, "armed": bool(self.armed),
                "veh_sysid": self.veh_sysid,
                "watchdog": bool(self._watchdog is not None
                                 and self._watchdog.is_alive())}

    def close(self):
        self.stop_watchdog()
        self._stop.set()
        for t in (self._reader, self._hb_thread, self._stream_thread):
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
        quiet = 0
        while not self._stop.is_set():
            conn = self.conn
            if conn is None:                 # swapped out by a reconnect
                time.sleep(0.2)
                continue
            try:
                m = conn.recv_match(blocking=True, timeout=0.3)
            except Exception:
                quiet += 1
                # A dead TCP socket makes pymavlink raise/EOF in a tight loop
                # (it prints "EOF on TCP socket" per read). Back off so the
                # watchdog can work in peace and the log stays readable.
                time.sleep(min(2.0, 0.2 * quiet))
                continue
            if m is None:
                quiet += 1
                if quiet > 20:
                    time.sleep(min(2.0, 0.1 * (quiet // 20)))
                continue
            quiet = 0
            t = m.get_type()
            if t == "HEARTBEAT" and m.get_srcSystem() != 51:
                try:
                    hbsys = m.get_srcSystem()
                    hbtype = m.type
                except Exception:
                    hbsys, hbtype = -1, -1
                key = (hbsys, hbtype)
                self._hb_sources[key] = self._hb_sources.get(key, 0) + 1
                # VEHICLE heartbeats only. GCS heartbeats (MP, sysid 255,
                # custom_mode=0) ride this wire too and used to drag
                # fc.mode to STABILIZE, silently vetoing the AUTO-gated
                # trigger for the whole flight. Same fix as the ADDC
                # reference: a separate vehicle-only heartbeat cache.
                if hbsys == self.target_system and \
                        hbtype != mavutil.mavlink.MAV_TYPE_GCS:
                    self.veh_sysid = hbsys
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
            try:
                sysid = hb.get_srcSystem()
            except Exception:
                sysid = -1
            log.info("FC mode: %s -> %s (sysid=%d)", self.mode, name, sysid)
        self.mode = name
        self.mode_num = hb.custom_mode
        self._mode_ts = time.time()
        self.armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def mode_age(self):
        """Seconds since the last VEHICLE heartbeat (inf if never seen)."""
        if not self._mode_ts:
            return float("inf")
        return time.time() - self._mode_ts

    def hb_sources(self):
        """Heartbeat census {(sysid, type): count} — who is on this wire."""
        return dict(self._hb_sources)

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

    def _stream_loop(self):
        """Re-assert stream rates every 15 s (reference-stack keep-alive).

        SET_MESSAGE_INTERVAL persists, but a late FC reboot, a GCS rate
        reset, or a flaky first request can silently starve the mission
        of MISSION_CURRENT/HOME. Best-effort, DEBUG-quiet.
        """
        while not self._stop.wait(15.0):
            if self.conn is None:
                continue
            try:
                self.start_streams()
                log.debug("stream rates re-asserted")
            except Exception:
                pass

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
        """Returns the COMMAND_ACK result, or None if unacked/failed."""
        try:
            return self._cmd_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                  float(msgid), ack_timeout=ack_timeout)
        except FCError:
            return None  # some builds don't ACK requests; the message may still come

    def get_home(self, timeout=4.0):
        ack = self.request_message(242)  # HOME_POSITION
        log.info("HOME_POSITION request ack: %r", ack)
        m = self.wait_for(["HOME_POSITION"], lambda m: True, timeout)
        if m is not None and not (m.latitude == 0 and m.longitude == 0):
            return (m.latitude / 1e7, m.longitude / 1e7, m.altitude / 1e3)
        # fallback: disarmed on the ground => current position IS home
        # (covers FCs that don't answer the explicit home request).
        try:
            pos = self.get_position(timeout=3.0)
        except FCError:
            raise FCError("no HOME_POSITION from FC (req ack=%r)" % (ack,))
        log.warning("no HOME_POSITION (req ack=%r) — using current position as home",
                    ack)
        return (pos["lat"], pos["lon"], pos["alt_msl"])

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

    def find_sprayer_seqs(self, plan, cmds=(216, 222, 223, 42600)):
        """All mission seqs whose command is a sprayer/trigger command.

        216 = DO_SPRAYER; 222/42600 are the fallbacks the field-tested
        ftest references accept (MP sprayer widgets vary).
        """
        want = set(cmds)
        return [it["seq"] for it in plan if it["command"] in want]

    def find_sprayer_seq(self, plan):
        seqs = self.find_sprayer_seqs(plan)
        return min(seqs) if seqs else None

    def get_mission_item(self, seq, mtype=0, timeout=3.0):
        """Fetch one mission item (peek-ahead trigger checks). Safe
        mid-mission — does not disturb mission state."""
        _require_pymavlink()
        self._send(self.conn.mav.mission_request_int_send,
                   self.target_system, self.target_component, int(seq), mtype)
        return self.wait_for(
            ["MISSION_ITEM_INT"],
            lambda m: m.mission_type == mtype and m.seq == int(seq), timeout)

    def set_message_interval(self, msgid, hz):
        """Best-effort stream rate for msgid (0/None restores default)."""
        us = int(1e6 / hz) if hz and hz > 0 else -1
        try:
            self._cmd_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                           float(msgid), float(us), ack_timeout=1.5)
        except FCError:
            pass

    #: Streams the mission needs. Telem-class ports (SITL serial1/5762
    #: included) stream NOTHING by default — heartbeat-only until a GCS
    #: sets rates (MP does this on every connect). So we set our own.
    STREAM_DEFAULTS = ((242, 1.0),    # HOME_POSITION
                       (33, 5.0),     # GLOBAL_POSITION_INT
                       (147, 1.0),    # BATTERY_STATUS
                       (42, 4.0),     # MISSION_CURRENT (4 Hz: finer trigger timing)
                       (245, 2.0),    # EXTENDED_SYS_STATE (landed state)
                       (74, 2.0))     # VFR_HUD (throttle/alt cross-check)

    def start_streams(self, specs=None):
        for msgid, hz in (specs or self.STREAM_DEFAULTS):
            self.set_message_interval(msgid, hz)

    @staticmethod
    def _param_id(m):
        try:
            pid = m.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", "ignore")
            return str(pid).split("\x00")[0].strip().upper()
        except Exception:
            return ""

    def _request_param(self, name, timeout=4.0):
        """PARAM_REQUEST_READ -> (value, mav_param_type). Raises FCError."""
        _require_pymavlink()
        want = str(name).upper()[:16]
        self._send(self.conn.mav.param_request_read_send,
                   self.target_system, self.target_component,
                   want.encode("utf-8"), -1)
        m = self.wait_for(["PARAM_VALUE"],
                          lambda m, w=want: self._param_id(m) == w, timeout)
        if m is None:
            raise FCError("get_param(%s) timeout" % want)
        return float(m.param_value), int(m.param_type)

    def get_param(self, name, timeout=4.0):
        """PARAM_REQUEST_READ + wait for the PARAM_VALUE echo (float)."""
        return self._request_param(name, timeout)[0]

    def set_param(self, name, value, timeout=4.0):
        """PARAM_SET with the FC's own type + wait for the echo.

        Type matters: the FC silently ignores a set whose type doesn't
        match the parameter (names/units/types vary by firmware: WP_SPD is
        float m/s on 4.7+, WPNAV_SPEED was INT cm/s — learning beats assuming
        always-REAL32 form could never take). We learn the type first;
        if learning fails we still try REAL32 once (harmless if the FC
        ignores it) before reporting the echo failure honestly.
        """
        _require_pymavlink()
        want = str(name).upper()[:16]
        try:
            _cur, ptype = self._request_param(name, timeout=2.0)
        except FCError as e:
            log.warning("set_param(%s): type-learn failed (%s) — trying REAL32", want, e)
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        self._send(self.conn.mav.param_set_send,
                   self.target_system, self.target_component,
                   want.encode("utf-8"), float(value), ptype)
        m = self.wait_for(["PARAM_VALUE"],
                          lambda m, w=want: self._param_id(m) == w, timeout)
        if m is None:
            raise FCError("set_param(%s) not echoed (tried type %d)" % (want, ptype))
        return float(m.param_value)

    def set_mode(self, mode, timeout=5.0):
        """DO_SET_MODE with canonical encoding: param1=1 (custom enabled),
        param2=mode number — the MAVLink-spec form the ADDC reference
        stack uses. Retries until COMMAND_ACK 0 or timeout."""
        num = COPTER_MODES.get(str(mode).upper(), mode) if isinstance(mode, str) else mode
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout:
            try:
                last = self._cmd_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                      1.0, float(num), ack_timeout=3.0)
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
