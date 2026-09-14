#!/usr/bin/env python3
"""fake_sitl.py — a tiny MAVLink vehicle for proving the Pi link + UI.

WHY THIS EXISTS
  The real bench needs ArduPilot SITL (a ~1 h build) or Gazebo, and SITL's
  serial1 (tcp 5762) only starts listening AFTER a GCS joins serial0
  (tcp 5760) — an ordering trap that looks exactly like "mission_pi is
  broken / the UI cannot connect". This script is a 300-line stand-in that
  speaks the same wire protocol on BOTH ports at once, so you can verify

      mission_pi  ->  FC link, snapshot, DO_SPRAYER trigger, GUIDED takeover
      mission-ui  ->  Pi WS + camera tiles + fsm/system events

  in seconds, on any machine, with no simulator. It is NOT a flight sim:
  position moves toward the last guided target at a fixed speed, there is
  no physics, no battery model, no GPS noise. Use it for link/UI work, then
  re-run the same steps against SITL/Gazebo (SIM_GUIDE.md) before trusting
  anything that flies.

RUN
  python3 tools/fake_sitl.py                       # tcp 5760 + 5762
  python3 tools/fake_sitl.py --udp 127.0.0.1:14551  # + the bridge/MP path
  python3 tools/fake_sitl.py --ports 5762          # mission_pi only
  python3 tools/fake_sitl.py --auto                # arm+AUTO by itself in 5 s
  python3 tools/fake_sitl.py --origin 15.3647,75.1240 --fence 40

  then, in another terminal:
      python3 main.py --config config.laptop.yaml --no-cams
      # or:  python3 main.py --device tcp:127.0.0.1:5762 --no-cams

  Keys (this terminal):  a=arm  m=AUTO  g=GUIDED  n=next mission item
                         s=fire DO_SPRAYER now  r=RTL  l=LAND  q=quit

WHAT IT ANSWERS (everything mission_pi/fc_link.py asks for)
  HEARTBEAT, GLOBAL_POSITION_INT, HOME_POSITION, ATTITUDE, VFR_HUD,
  SYS_STATUS/BATTERY_STATUS/EXTENDED_SYS_STATE, MISSION_CURRENT,
  MISSION_COUNT + MISSION_ITEM_INT (mission AND fence), PARAM_VALUE
  (read + set echo), COMMAND_ACK for REQUEST_MESSAGE / SET_MESSAGE_INTERVAL
  / DO_SET_MODE / DO_SPRAYER / MISSION_START / CONDITION_YAW, STATUSTEXT.
  Guided motion honours SET_POSITION_TARGET_GLOBAL_INT + CONDITION_YAW.
"""
import argparse
import math
import os
import select
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
    from pymavlink.mavutil import mavlink as M
except Exception as e:                      # pragma: no cover
    print("fake_sitl needs pymavlink:  pip install pymavlink  (%r)" % (e,))
    sys.exit(2)

MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
         "RTL": 6, "LAND": 9}
MODE_BY_NUM = {v: k for k, v in MODES.items()}
ARMED_FLAG = M.MAV_MODE_FLAG_SAFETY_ARMED

DEFAULT_PLAN = [
    # (command, param1, x_deg, y_deg, z_m, frame)
    (M.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 15.0, M.MAV_FRAME_GLOBAL_RELATIVE_ALT),
    (M.MAV_CMD_NAV_WAYPOINT, 0.0, None, None, 15.0, M.MAV_FRAME_GLOBAL),
    (M.MAV_CMD_DO_SPRAYER, 1.0, 0.0, 0.0, 0.0, M.MAV_FRAME_GLOBAL_RELATIVE_ALT),
    (M.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0, 0.0, M.MAV_FRAME_GLOBAL),
]


# pymavlink message classes expose `fieldnames` but not the C types, so the
# few ARRAY fields we send are listed explicitly (everything else zero-fills
# as a scalar, which struct.pack coerces for both int and float fields).
ARRAY_FIELDS = {"voltages": 10, "voltages_ext": 4, "q": 4, "rotation": 4}
TEXT_FIELDS = ("param_id", "text", "name")


def _zero_for(name):
    """Sane empty value for an unset MAVLink field."""
    if name in ARRAY_FIELDS:
        return [0] * ARRAY_FIELDS[name]
    if name in TEXT_FIELDS:
        return b""
    return 0


def _mk(cls, **kw):
    """Build a MAVLink message across pymavlink versions.

    Fields this version does not have are dropped; fields it requires but the
    caller did not set are zero-filled. pymavlink message classes gained
    fields over time (BATTERY_STATUS.temperature, MISSION_CURRENT.total,
    SYS_STATUS.errors_countN, HOME_POSITION.time_usec), so a hard-coded
    positional call breaks on whichever version the laptop/Pi happens to
    have — the fake never makes one.
    """
    names = list(getattr(cls, "fieldnames", []) or [])
    if not names:
        return cls(**kw)
    args = {}
    for name in names:
        if name in kw:
            args[name] = kw[name]
        elif name not in args:
            args[name] = _zero_for(name)
    return cls(**args)


def m_offset(lat, lon, east_m, north_m):
    """Small-offset ENU -> lat/lon (same flat-earth approx as geo.py)."""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * max(0.01, math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


class FakeVehicle:
    """One vehicle, many TCP clients (MP on 5760, mission_pi on 5762, ...)."""

    def __init__(self, origin, fence_m=40.0, speed=3.0, plan=None):
        self.hlat, self.hlon = origin
        self.home_alt = 584.0                      # m MSL, cosmetic
        self.lat, self.lon = origin
        self.alt_rel = 0.0
        self.hdg = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = 0.0
        self.mode = "STABILIZE"
        self.armed = False
        self.loop = False                             # --loop: endless demo
        self._land_t = 0.0
        self.speed = float(speed)
        self.t0 = time.time()
        self.lock = threading.RLock()
        self.clients = []
        # mission
        self.plan = list(plan or DEFAULT_PLAN)
        self._fill_plan()
        self.mission_seq = 0
        self.mission_running = False
        self._next_item_t = 0.0
        # fence: square around home, inclusion vertices (cmd 5001)
        h = float(fence_m) / 2.0
        self.fence = []
        if fence_m and fence_m > 0:
            for e, n in ((-h, -h), (h, -h), (h, h), (-h, h)):
                la, lo = m_offset(self.hlat, self.hlon, e, n)
                self.fence.append((la, lo))
        # guided target
        self.target = None                          # (lat, lon, alt_rel)
        # params the mission touches
        self.params = {"WP_SPD": (float(speed), M.MAV_PARAM_TYPE_REAL32),
                       "WPNAV_SPEED": (speed * 100.0, M.MAV_PARAM_TYPE_REAL32),
                       "FENCE_ENABLE": (1.0, M.MAV_PARAM_TYPE_UINT8),
                       "FENCE_ACTION": (1.0, M.MAV_PARAM_TYPE_UINT8),
                       "FENCE_TYPE": (12.0, M.MAV_PARAM_TYPE_UINT8),
                       "RTL_ALT": (1500.0, M.MAV_PARAM_TYPE_INT32),
                       "BATT_MONITOR": (4.0, M.MAV_PARAM_TYPE_UINT8)}
        self.intervals = {}                         # msgid -> hz (client-set)

    # -- plan helpers ----------------------------------------------------
    def _fill_plan(self):
        """Put real coordinates on the waypoint (20 m east of home)."""
        out = []
        for cmd, p1, x, y, z, frame in self.plan:
            if x is None:
                la, lo = m_offset(self.hlat, self.hlon, 20.0, 0.0)
                x, y = la, lo
            out.append((cmd, p1, x, y, z, frame))
        self.plan = out

    def sprayer_seq(self):
        for i, it in enumerate(self.plan):
            if it[0] in (216, 222, 223, 42600):
                return i
        return None

    # -- client registry -------------------------------------------------
    def add_client(self, c):
        with self.lock:
            self.clients.append(c)

    def drop_client(self, c):
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)

    def broadcast(self, msg):
        with self.lock:
            cs = list(self.clients)
        for c in cs:
            c.send(msg)

    def say(self, text, severity=M.MAV_SEVERITY_INFO):
        self.broadcast(_mk(M.MAVLink_statustext_message, severity=int(severity),
                           text=str(text)[:50].encode()))

    # -- state changes ---------------------------------------------------
    def set_mode(self, name):
        with self.lock:
            if self.mode == name:
                return
            # real bench flow: the operator arms, THEN selects AUTO. A fake
            # that stayed disarmed would never move, so arm on the flight
            # modes (and say so loudly — it is a fake, not a safety case).
            if name in ("AUTO", "GUIDED") and not self.armed:
                self.armed = True
                print("[fake] auto-armed for %s" % name)
            self.mode = name
            if name in ("RTL", "LAND"):
                self.target = None
            if name == "AUTO":
                self.mission_running = True
                self._next_item_t = time.time() + 3.0
        print("[fake] mode -> %s" % name)
        self.say("mode %s" % name)

    def arm(self, on=True):
        with self.lock:
            self.armed = bool(on)
        print("[fake] armed=%s" % self.armed)
        self.say("ARMED" if on else "DISARMED")

    def next_item(self, quiet=False):
        with self.lock:
            if self.mission_seq + 1 < len(self.plan):
                self.mission_seq += 1
                self.mission_running = True
                seq = self.mission_seq
                cmd = self.plan[seq][0]
            else:
                seq, cmd = self.mission_seq, None
        if cmd is not None and not quiet:
            print("[fake] MISSION_CURRENT -> %d (cmd %s)" % (seq, cmd))
            if cmd in (216, 222, 223, 42600):
                self.say("Sprayer ON")
        self.broadcast(_mk(M.MAVLink_mission_current_message, seq=seq))

    def fire_sprayer(self):
        seq = self.sprayer_seq()
        if seq is None:
            self.say("no sprayer item in plan")
            return
        with self.lock:
            self.mission_seq = seq
            self.mission_running = True
        print("[fake] firing DO_SPRAYER at seq %d" % seq)
        self.broadcast(_mk(M.MAVLink_mission_current_message, seq=seq))
        self.say("Sprayer ON")

    # -- inbound ---------------------------------------------------------
    def handle(self, c, m):
        t = m.get_type()
        if t == "BAD_DATA":
            return                                    # pymavlink parse noise
        if t == "HEARTBEAT":
            return                                    # GCS liveness, ignore
        if t == "STATUSTEXT":
            # This is how the QR payload reaches Mission Planner in real life:
            # the Pi writes STATUSTEXT to the FC, the FC re-broadcasts it to
            # every GCS. Echo it to the OTHER clients so MP (on 5760) would
            # show `QR:<payload>` in its Messages tab.
            try:
                txt = m.text
                if isinstance(txt, bytes):
                    txt = txt.decode("utf-8", "replace")
            except Exception:
                txt = "?"
            print("[fake] STATUSTEXT from client %s: %r" % (c.addr[0], txt))
            with self.lock:
                others = [o for o in self.clients if o is not c]
            for o in others:
                o.send(m)
            return
        if t == "COMMAND_LONG":
            return self._command_long(c, m)
        if t == "MISSION_REQUEST_LIST":
            return self._mission_count(c, m)
        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            return self._mission_item(c, m)
        if t == "MISSION_ACK":
            return
        if t == "PARAM_REQUEST_READ":
            return self._param_reply(c, m)
        if t == "PARAM_SET":
            return self._param_set(c, m)
        if t == "SET_POSITION_TARGET_GLOBAL_INT":
            with self.lock:
                self.target = (m.lat_int / 1e7, m.lon_int / 1e7,
                               m.alt if m.alt else self.alt_rel)
            return
        if t == "SET_POSITION_TARGET_LOCAL_NED":
            return
        if t == "MISSION_SET_CURRENT":
            with self.lock:
                self.mission_seq = int(m.seq)
            self.broadcast(_mk(M.MAVLink_mission_current_message,
                               seq=self.mission_seq))
            return
        if t in ("REQUEST_DATA_STREAM", "MAV_CMD_SET_MESSAGE_INTERVAL"):
            return
        c.note("unhandled %s" % t)

    def _command_long(self, c, m):
        cmd = int(m.command)
        def ack(result=0):
            c.send(_mk(M.MAVLink_command_ack_message, command=cmd,
                       result=int(result),
                       target_system=m.get_srcSystem(),
                       target_component=m.get_srcComponent()))
        if cmd == M.MAV_CMD_REQUEST_MESSAGE:            # 512
            ack()
            self._send_msgid(c, int(m.param1))
        elif cmd == M.MAV_CMD_SET_MESSAGE_INTERVAL:     # 76
            self.intervals[int(m.param1)] = m.param2
            ack()
        elif cmd == M.MAV_CMD_DO_SET_MODE:              # 176
            num = int(m.param2)
            name = MODE_BY_NUM.get(num)
            if name is None:
                ack(M.MAV_RESULT_UNSUPPORTED)
            else:
                ack()
                if name == "AUTO" and not self.armed:
                    self.arm(True)
                self.set_mode(name)
        elif cmd == M.MAV_CMD_MISSION_START:            # 300
            ack()
            with self.lock:
                self.mission_seq = 0
                self.mission_running = True
                self._next_item_t = time.time() + 3.0
            if not self.armed:
                self.arm(True)
            self.set_mode("AUTO")
            self.broadcast(_mk(M.MAVLink_mission_current_message, seq=0))
        elif cmd == M.MAV_CMD_NAV_TAKEOFF:              # 22
            ack()
            self.arm(True)
            with self.lock:
                self.target = (self.lat, self.lon, float(m.param7 or 15.0))
        elif cmd in (216, 222, 223, 42600):             # DO_SPRAYER & friends
            ack()
            self.say("Sprayer %s" % ("ON" if m.param1 else "OFF"))
        elif cmd == M.MAV_CMD_CONDITION_YAW:            # 115
            ack()
            if not m.param4:                            # absolute
                self.hdg = float(m.param1) % 360.0
        elif cmd == M.MAV_CMD_COMPONENT_ARM_DISARM:     # 400
            ack()
            self.arm(bool(m.param1))
        elif cmd == M.MAV_CMD_NAV_RETURN_TO_LAUNCH:     # 20
            ack()
            self.set_mode("RTL")
        elif cmd == M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:  # 246
            ack(M.MAV_RESULT_FAILED)
            self.say("fake vehicle will not reboot")
        else:
            ack(M.MAV_RESULT_UNSUPPORTED)
            c.note("unsupported cmd %d" % cmd)

    def _send_msgid(self, c, msgid):
        if msgid == 242:                                # HOME_POSITION
            c.send(self._home_msg())
        elif msgid == 33:                               # GLOBAL_POSITION_INT
            c.send(self._gpi_msg())
        elif msgid == 42:                               # MISSION_CURRENT
            c.send(_mk(M.MAVLink_mission_current_message, seq=self.mission_seq))
        elif msgid == 147:                              # BATTERY_STATUS
            c.send(self._battery_msg())
        elif msgid == 74:                               # VFR_HUD
            c.send(self._vfr_msg())

    def _mission_count(self, c, m):
        mtype = getattr(m, "mission_type", 0)
        n = len(self.fence) if mtype == M.MAV_MISSION_TYPE_FENCE else len(self.plan)
        c.send(_mk(M.MAVLink_mission_count_message,
                   target_system=m.get_srcSystem(),
                   target_component=m.get_srcComponent(),
                   count=n, mission_type=mtype))

    def _mission_item(self, c, m):
        mtype = getattr(m, "mission_type", 0)
        seq = int(m.seq)
        if mtype == M.MAV_MISSION_TYPE_FENCE:
            if not (0 <= seq < len(self.fence)):
                c.send(_mk(M.MAVLink_mission_ack_message,
                           target_system=m.get_srcSystem(),
                           target_component=m.get_srcComponent(),
                           type=M.MAV_MISSION_INVALID_SEQUENCE,
                           mission_type=mtype))
                return
            la, lo = self.fence[seq]
            c.send(_mk(M.MAVLink_mission_item_int_message,
                       target_system=m.get_srcSystem(),
                       target_component=m.get_srcComponent(),
                       seq=seq, frame=M.MAV_FRAME_GLOBAL,
                       command=M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                       current=0, autocontinue=1, param1=len(self.fence),
                       param2=0, param3=0, param4=0,
                       x=int(la * 1e7), y=int(lo * 1e7), z=0.0,
                       mission_type=mtype))
            return
        if not (0 <= seq < len(self.plan)):
            c.send(_mk(M.MAVLink_mission_ack_message,
                       target_system=m.get_srcSystem(),
                       target_component=m.get_srcComponent(),
                       type=M.MAV_MISSION_INVALID_SEQUENCE,
                       mission_type=mtype))
            return
        cmd, p1, x, y, z, frame = self.plan[seq]
        c.send(_mk(M.MAVLink_mission_item_int_message,
                   target_system=m.get_srcSystem(),
                   target_component=m.get_srcComponent(),
                   seq=seq, frame=frame, command=cmd, current=0,
                   autocontinue=1, param1=p1, param2=0, param3=0, param4=0,
                   x=int(x * 1e7), y=int(y * 1e7), z=float(z),
                   mission_type=mtype))

    def _param_reply(self, c, m):
        name = m.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "ignore")
        name = str(name).split("\x00")[0].strip().upper()
        with self.lock:
            val = self.params.get(name)
        if val is None:
            val = (0.0, M.MAV_PARAM_TYPE_REAL32)
        c.send(self._param_value_msg(name, val[0], val[1],
                                     index=getattr(m, "param_index", -1)))

    def _param_set(self, c, m):
        name = m.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "ignore")
        name = str(name).split("\x00")[0].strip().upper()
        with self.lock:
            ptype = self.params.get(name, (0.0, M.MAV_PARAM_TYPE_REAL32))[1]
            self.params[name] = (float(m.param_value), ptype)
            val = self.params[name]
        print("[fake] param %s = %s" % (name, val[0]))
        c.send(self._param_value_msg(name, val[0], val[1]))

    def _param_value_msg(self, name, value, ptype, index=-1):
        return _mk(M.MAVLink_param_value_message,
                   param_id=name.encode("utf-8")[:16].ljust(16, b"\x00"),
                   param_value=float(value), param_type=int(ptype),
                   param_count=len(self.params),
                   param_index=int(index if index is not None and index >= 0
                                   else 0))

    # -- outbound telemetry ---------------------------------------------
    def _home_msg(self):
        return _mk(M.MAVLink_home_position_message,
                   latitude=int(self.hlat * 1e7), longitude=int(self.hlon * 1e7),
                   altitude=int(self.home_alt * 1e3), x=0.0, y=0.0, z=0.0,
                   q=[1.0, 0.0, 0.0, 0.0], approach_x=0.0, approach_y=0.0,
                   approach_z=0.0)

    def _gpi_msg(self):
        with self.lock:
            return _mk(M.MAVLink_global_position_int_message,
                       time_boot_ms=int((time.time() - self.t0) * 1000) & 0xFFFFFFFF,
                       lat=int(self.lat * 1e7), lon=int(self.lon * 1e7),
                       alt=int((self.home_alt + self.alt_rel) * 1e3),
                       relative_alt=int(self.alt_rel * 1e3),
                       vx=int(self.vx * 100), vy=int(self.vy * 100),
                       vz=int(self.vz * 100),
                       hdg=int(self.hdg * 100) % 36000)

    def _battery_msg(self):
        return _mk(M.MAVLink_battery_status_message,
                   id=0, battery_function=M.MAV_BATTERY_FUNCTION_UNKNOWN,
                   type=M.MAV_BATTERY_TYPE_LIPO, temperature=2500,
                   voltages=[16600] + [65535] * 9, current_battery=444,
                   current_consumed=120, energy_consumed=2000,
                   battery_remaining=96)

    def _vfr_msg(self):
        with self.lock:
            gs = math.hypot(self.vx, self.vy)
            return _mk(M.MAVLink_vfr_hud_message, airspeed=gs, groundspeed=gs,
                       heading=int(self.hdg) % 360, throttle=50,
                       alt=self.alt_rel, climb=self.vz)

    def heartbeat_msg(self):
        with self.lock:
            base = ARMED_FLAG if self.armed else 0
            base |= M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            num = MODES.get(self.mode, 0)
            state = (M.MAV_STATE_ACTIVE if self.armed else M.MAV_STATE_STANDBY)
        return _mk(M.MAVLink_heartbeat_message, type=M.MAV_TYPE_QUADROTOR,
                   autopilot=M.MAV_AUTOPILOT_ARDUPILOTMEGA, base_mode=base,
                   custom_mode=num, system_status=state, mavlink_version=3)

    # -- 10 Hz motion + telemetry ---------------------------------------
    def tick(self, dt):
        with self.lock:
            mode = self.mode
            armed = self.armed
            target = self.target
            seq = self.mission_seq
            running = self.mission_running
            nxt = self._next_item_t
        # AUTO: fly the plan item, then advance
        if armed and mode == "AUTO" and running:
            if 0 <= seq < len(self.plan):
                cmd, _p1, x, y, z, _f = self.plan[seq]
                if cmd in (M.MAV_CMD_NAV_TAKEOFF, M.MAV_CMD_NAV_WAYPOINT,
                           M.MAV_CMD_NAV_LOITER_UNLIM):
                    target = (x, y, z)
                elif cmd == M.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    target = (self.hlat, self.hlon, 15.0)
            if time.time() >= nxt and seq + 1 < len(self.plan):
                self.next_item()
                with self.lock:
                    self._next_item_t = time.time() + 6.0
        if mode == "RTL":
            # real RTL: come home at RTL alt, then descend on top of home.
            de, dn = _enu_of(self.lat - self.hlat, self.lon - self.hlon,
                             self.hlat)
            over_home = math.hypot(de, dn) < 2.0
            target = (self.hlat, self.hlon, 0.0 if over_home else 15.0)
            if over_home and self.alt_rel < 0.4:
                self.set_mode("LAND")
        if mode == "LAND":
            target = (self.lat, self.lon, 0.0)
        # move toward the target
        with self.lock:
            self.target = target
            if armed and target:
                tlat, tlon, talt = target
                de, dn = _enu_of(tlat - self.lat, tlon - self.lon, self.lat)
                dist = math.hypot(de, dn)
                step = min(dist, self.speed * dt)
                if dist > 0.05:
                    self.lat += (tlat - self.lat) * (step / dist)
                    self.lon += (tlon - self.lon) * (step / dist)
                    self.hdg = (math.degrees(math.atan2(de, dn))) % 360.0
                    self.vx, self.vy = dn and (dn / dist) * self.speed or 0.0, \
                        de and (de / dist) * self.speed or 0.0
                else:
                    self.vx = self.vy = 0.0
                dalt = talt - self.alt_rel
                va = min(abs(dalt), 2.0 * dt) * (1 if dalt > 0 else -1)
                self.alt_rel += va
                self.vz = va / max(dt, 1e-3)
                if mode == "LAND" and self.alt_rel <= 0.05:
                    self.alt_rel = 0.0
                    if self.armed:
                        self._land_t = time.time()
                    self.armed = False
            else:
                self.vx = self.vy = self.vz = 0.0
        # --loop: after touchdown, reset and fly the sortie again so a demo
        # (or a UI left open on a bench) keeps showing a live mission.
        if self.loop and not self.armed and self.mode in ("LAND", "STABILIZE") \
                and self._land_t and (time.time() - self._land_t) > 8.0:
            with self.lock:
                self._land_t = 0.0
                self.lat, self.lon = self.hlat, self.hlon
                self.alt_rel = 0.0
                self.mission_seq = 0
                self.mission_running = False
                self.target = None
                self.mode = "STABILIZE"
            print("[fake] --loop: reset at home, re-flying the sortie")
            self.say("fake vehicle reset")
            self.set_mode("AUTO")
        # telemetry out
        self.broadcast(self._gpi_msg())
        self.broadcast(_mk(M.MAVLink_attitude_message,
                           time_boot_ms=int((time.time() - self.t0) * 1000) & 0xFFFFFFFF,
                           roll=math.radians(self.roll),
                           pitch=math.radians(self.pitch),
                           yaw=math.radians(self.hdg),
                           rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))
        self.broadcast(self._vfr_msg())
        self.broadcast(self._battery_msg())


def _enu_of(dlat, dlon, lat):
    return (dlon * 111320.0 * math.cos(math.radians(lat)), dlat * 111320.0)


class Client:
    """One TCP connection: MAVLink stream in/out with its own codec."""

    def __init__(self, veh, sock, addr):
        self.veh = veh
        self.sock = sock
        self.addr = addr
        self.lock = threading.Lock()
        self.mav = M.MAVLink(None, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.dead = False
        self.notes = []
        self.rx = self.tx = 0

    def note(self, msg):
        if msg not in self.notes:
            self.notes.append(msg)
            if len(self.notes) <= 6:
                print("[fake] client %s: %s" % (self.addr[0], msg))

    def send(self, msg):
        if self.dead:
            return
        try:
            with self.lock:
                self.sock.sendall(msg.pack(self.mav))
            self.tx += 1
        except Exception as e:
            self.dead = True
            self.veh.drop_client(self)
            print("[fake] client %s gone (%s)" % (self.addr[0], e))

    def poll(self, timeout=0.05):
        if self.dead:
            return
        try:
            r, _w, _x = select.select([self.sock], [], [], timeout)
            if not r:
                return
            data = self.sock.recv(4096)
            if not data:
                raise OSError("closed")
            self.rx += len(data)
            for m in (self.mav.parse_buffer(data) or []):
                try:
                    self.veh.handle(self, m)
                except Exception as e:
                    self.note("handle %s failed: %r" % (m.get_type(), e))
        except Exception as e:
            self.dead = True
            self.veh.drop_client(self)
            print("[fake] client %s disconnected (%s)" % (self.addr[0], e))

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class UdpClient:
    """The same vehicle over UDP — emulates Mission Planner's MAVLink
    forwarding to the laptop bridge (mission-ui/bridge/mp_bridge.py listens on
    UDP :14551 and answers to whoever last sent to it).

    One socket, both directions: telemetry goes out to the bridge, and the
    bridge's requests (MISSION_REQUEST_LIST for the fence/plan, PARAM_READ,
    COMMAND_LONG) come back in and are handled by the SAME code the TCP
    clients use. That makes the UI's MAVLink side testable with no MP and no
    radio.
    """

    def __init__(self, veh, target, bind_port=0):
        self.veh = veh
        self.target = target
        self.addr = ("udp:%s:%d" % target, bind_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", int(bind_port)))
        self.sock.setblocking(False)
        self.mav = M.MAVLink(None, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.dead = False
        self.notes = []
        self.lock = threading.Lock()
        print("[fake] forwarding MAVLink over UDP to %s:%d (bridge/MP path)"
              % target)

    def note(self, msg):
        if msg not in self.notes:
            self.notes.append(msg)
            print("[fake] udp: %s" % msg)

    def send(self, msg):
        if self.dead:
            return
        try:
            with self.lock:
                self.sock.sendto(msg.pack(self.mav), self.target)
        except Exception as e:
            self.note("sendto failed: %r" % (e,))

    def poll(self, timeout=0.2):
        try:
            r, _w, _x = select.select([self.sock], [], [], timeout)
            if not r:
                return
            data, src = self.sock.recvfrom(65535)
            # answer wherever the GCS actually came from (MP/bridge may sit
            # behind NAT or use an ephemeral source port)
            self.target = src
            for m in (self.mav.parse_buffer(data) or []):
                try:
                    self.veh.handle(self, m)
                except Exception as e:
                    self.note("handle %s failed: %r" % (m.get_type(), e))
        except Exception:
            pass

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def serve_udp(veh, target, bind_port=0):
    c = UdpClient(veh, target, bind_port)
    veh.add_client(c)

    def loop():
        while not c.dead:
            c.poll(0.2)

    threading.Thread(target=loop, daemon=True, name="fake-udp").start()
    return c


def serve_port(veh, port, host="0.0.0.0"):
    """SITL-style TCP *server* port (clients connect to us, like 5760/5762)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port)))
    srv.listen(8)
    print("[fake] listening on tcp:%s:%s (SITL-style server port)" % (host, port))

    def loop():
        while True:
            try:
                sock, addr = srv.accept()
            except Exception as e:
                print("[fake] accept failed: %r" % (e,))
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c = Client(veh, sock, addr)
            veh.add_client(c)
            print("[fake] client connected from %s:%d (%d total)"
                  % (addr[0], addr[1], len(veh.clients)))
            veh.say("fake vehicle: STABILIZE, home %.5f,%.5f"
                    % (veh.hlat, veh.hlon))
            threading.Thread(target=_client_loop, args=(c,), daemon=True).start()

    threading.Thread(target=loop, daemon=True).start()
    return srv


def _client_loop(c):
    while not c.dead:
        c.poll(0.1)
    c.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="tiny fake MAVLink vehicle")
    ap.add_argument("--ports", default="5760,5762",
                    help="comma list of TCP server ports (SITL uses 5760 for "
                         "the GCS and 5762 for serial1)")
    ap.add_argument("--origin", default="15.3647,75.1240", help="lat,lon home")
    ap.add_argument("--fence", type=float, default=40.0,
                    help="square fence side in metres (0 = no fence)")
    ap.add_argument("--speed", type=float, default=3.0, help="m/s")
    ap.add_argument("--auto", action="store_true",
                    help="arm + AUTO by itself after 5 s (no MP needed)")
    ap.add_argument("--udp", default=None, metavar="HOST:PORT",
                    help="also forward MAVLink over UDP to HOST:PORT — that is "
                         "the bridge/MP path (mission-ui listens on 14551)")
    ap.add_argument("--loop", action="store_true",
                    help="after landing, reset at home and fly the sortie "
                         "again (endless demo for an open UI)")
    ap.add_argument("--no-keys", action="store_true", help="no keyboard control")
    args = ap.parse_args(argv)

    try:
        la, lo = [float(x) for x in str(args.origin).split(",")[:2]]
    except Exception:
        print("--origin must be lat,lon (e.g. 15.3647,75.1240)")
        return 2
    veh = FakeVehicle((la, lo), fence_m=args.fence, speed=args.speed)
    veh.loop = bool(args.loop)
    print("[fake] home %.6f,%.6f  fence %s  plan: %s (sprayer seq %s)"
          % (la, lo,
             ("%dm square, %d verts" % (args.fence, len(veh.fence)))
             if veh.fence else "none",
             [c for c, *_ in veh.plan], veh.sprayer_seq()))
    socks = [serve_port(veh, p.strip()) for p in args.ports.split(",") if p.strip()]
    udp_client = None
    if args.udp:
        try:
            h, _, prt = str(args.udp).rpartition(":")
            udp_client = serve_udp(veh, (h or "127.0.0.1", int(prt or 14551)))
        except Exception as e:
            print("[fake] --udp ignored (%r) — want HOST:PORT, e.g. 127.0.0.1:14551" % (e,))

    stop = threading.Event()

    def beat():
        last_hb = last_tick = 0.0
        while not stop.is_set():
            now = time.time()
            dt = now - last_tick if last_tick else 0.1
            last_tick = now
            try:
                veh.tick(min(0.5, max(0.001, dt)))
            except Exception as e:
                print("[fake] tick failed: %r" % (e,))
            if now - last_hb >= 1.0:
                last_hb = now
                hb = veh.heartbeat_msg()
                veh.broadcast(hb)
                veh.broadcast(_mk(M.MAVLink_sys_status_message,
                                  onboard_control_sensors_present=0x3F,
                                  onboard_control_sensors_enabled=0x3F,
                                  onboard_control_sensors_health=0x3F,
                                  load=120, voltage_battery=16600,
                                  current_battery=444, battery_remaining=96,
                                  drop_rate_comm=0, errors_comm=0))
                veh.broadcast(veh._home_msg())
                veh.broadcast(_mk(M.MAVLink_mission_current_message,
                                  seq=veh.mission_seq))
            stop.wait(0.1)

    threading.Thread(target=beat, daemon=True).start()

    if args.auto:
        def auto():
            stop.wait(5.0)
            print("[fake] --auto: arming + AUTO")
            veh.arm(True)
            veh.set_mode("AUTO")
        threading.Thread(target=auto, daemon=True).start()

    print("[fake] keys: a=arm m=AUTO g=GUIDED n=next item s=sprayer r=RTL "
          "l=LAND q=quit")
    try:
        if args.no_keys:
            while True:
                time.sleep(0.5)
        else:
            # stdin may be a tty OR a pipe/FIFO (scripts drive it that way);
            # readline() returns '' at EOF, which ends the loop cleanly.
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                k = line.strip().lower()[:1]
                if k == "q":
                    break
                elif k == "a":
                    veh.arm(not veh.armed)
                elif k == "m":
                    veh.set_mode("AUTO")
                elif k == "g":
                    veh.set_mode("GUIDED")
                elif k == "n":
                    veh.next_item()
                elif k == "s":
                    veh.fire_sprayer()
                elif k == "r":
                    veh.set_mode("RTL")
                elif k == "l":
                    veh.set_mode("LAND")
                elif k == "p":
                    print("[fake] %s armed=%s pos=%.6f,%.6f alt=%.1f seq=%d "
                          "clients=%d" % (veh.mode, veh.armed, veh.lat, veh.lon,
                                          veh.alt_rel, veh.mission_seq,
                                          len(veh.clients)))
                elif k:
                    print("[fake] ? keys: a m g n s r l p q")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if udp_client is not None:
            udp_client.close()
        for s in socks:
            try:
                s.close()
            except Exception:
                pass
        print("[fake] bye")
    return 0


if __name__ == "__main__":
    sys.exit(main())
