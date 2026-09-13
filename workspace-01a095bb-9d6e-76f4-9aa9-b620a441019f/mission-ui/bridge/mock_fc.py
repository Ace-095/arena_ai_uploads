"""
mock_fc.py — mock ArduCopter over REAL MAVLink/UDP loopback (--mock mode).

Why real wire: the whole point of --mock is that the UI (and the bridge's
mav_client code paths — fence upload, readback, params, mode set, plan
read, STATUSTEXT relay) exercise the SAME code that runs against the real
Pixhawk through Mission Planner forwarding. A mock that skipped the wire
would hide exactly the bugs that matter (see hehe's "mock FC is not a
substitute" discipline — this one at least speaks the real protocol).

Topology (all inside the bridge process, on loopback):

    mock FC  ◄──UDP 127.0.0.1:14560──►  mav_client GCS (bound :14551)
      ▲                                      ▲
      │ STATUSTEXT (sysid 42, "the Pi")      │ everything else (sysid 255)
      └── inject_pi_statustext() ──┘         └── mp_client via MP forwarding

The vehicle's dynamic state (position/alt/mode/battery) is read from a
shared VehicleState object written by MockPi (the mission brain), so the
drone motion on the map and the MAVLink telemetry are one source of truth.
"""
import math
import socket
import struct
import threading
import time

import _mavlink_v2 as M

PI_SYSID = 42          # the "Pi" talking to the FC over USB (simulated)
FC_SYSID = 1
FC_COMPID = 1

# mock plan as MP would have uploaded: [climb] -> [DO_SPRAYER] -> [land]
MOCK_PLAN = [
    (0, M.MAV_CMD_NAV_TAKEOFF, 14.0),
    (1, M.MAV_CMD_DO_SPRAYER, 0.0),
    (2, M.MAV_CMD_NAV_LAND, 0.0),
]

ALL_SENSORS = 0x0FFF


class VehicleState:
    """Shared mutable state: MockPi (mission brain) writes, MockFc reads."""
    def __init__(self, origin):
        self.origin = origin
        self.lat, self.lon = origin
        self.alt = 0.0
        self.hdg = 90.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = 0.0
        self.mode_num = M.MODE_STANDBY
        self.armed = False
        self.batt_v = 16.6
        self.batt_pct = 97
        self.batt_i = 0.3
        self.fix = 3
        self.sats = 12
        self.mission_seq = 0
        self.mission_total = 3
        # set by the mock FC when the UI sends DO_SET_MODE (operator override)
        self.mode_override = None
        self.mode_override_ts = 0.0

    def pop_override(self):
        ov = self.mode_override
        self.mode_override = None
        return ov


class MockFc(threading.Thread):
    def __init__(self, port=14560, gcs_addr=("127.0.0.1", 14551),
                 vehicle=None, origin=(15.3697, 75.1235)):
        super().__init__(daemon=True, name="mock-fc")
        self.port = port
        self.gcs_addr = gcs_addr
        self.vehicle = vehicle
        self.origin = origin
        self.sock = None
        self.fence_items = []          # [(lat_e7, lon_e7, n)]
        self.params = {"FENCE_ENABLE": 0.0, "FENCE_ACTION": 1.0, "FENCE_TYPE": 0.0}
        self.plan = list(MOCK_PLAN)
        self._seq = 0
        self._stop = threading.Event()
        self._last_hb = 0.0
        self._last_tel = 0.0
        self._last_sys = 0.0

    # ------------------------------------------------------------------ #
    def _send(self, msgid, values, to=None, sysid=FC_SYSID):
        if self.sock is None:
            return
        self._seq = (self._seq + 1) & 0xFF
        try:
            self.sock.sendto(M.build_frame(msgid, sysid, FC_COMPID, self._seq, values),
                             to or self.gcs_addr)
        except OSError:
            pass

    def stop(self):
        self._stop.set()
        try:
            self.sock and self.sock.close()
        except OSError:
            pass

    # ------------------------------------------------------------------ #
    def inject_pi_statustext(self, text):
        """Simulate the Pi's USB link sending STATUSTEXT to the FC."""
        if self.sock is None:
            return
        self._seq = (self._seq + 1) & 0xFF
        frame = M.build_frame(253, PI_SYSID, 1, self._seq,
                              {"severity": M.MAV_SEVERITY_INFO,
                               "text": text.encode("utf-8")[:50]})
        try:
            # from a client socket, like a real USB-MAVLink link would
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(frame, ("127.0.0.1", self.port))
            s.close()
        except OSError:
            pass

    # ------------------------------------------------------------------ #
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.port))
        self.sock.settimeout(0.1)
        print("[mock-fc] listening on udp:%d (gcs at %s)" % (self.port, self.gcs_addr))
        while not self._stop.is_set():
            now = time.time()
            self._emit_vehicle_state(now)
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            parsed = M.parse_datagram(data)
            if parsed is None:
                continue
            msgid, src_sys, _c, _s, f = parsed
            try:
                self._handle(msgid, src_sys, f)
            except Exception as e:
                print("[mock-fc] handler error:", repr(e))
        self.stop()

    # ------------------------------------------------------------------ #
    def _emit_vehicle_state(self, now):
        v = self.vehicle
        if v is None:
            return
        # heartbeat 1 Hz
        if now - self._last_hb > 1.0:
            self._last_hb = now
            self._send(M.MSG_ID['HEARTBEAT'], {
                "custom_mode": v.mode_num, "type": 2, "autopilot": 3,
                "base_mode": (128 if v.armed else 0) | 1,
                "system_status": 4 if v.armed else 0, "mavlink_version": 3,
            })
        if now - self._last_tel < 0.2:   # telemetry 5 Hz
            return
        self._last_tel = now
        msl_mm = int((v.alt + 340.0) * 1000)
        self._send(M.MSG_ID['GLOBAL_POSITION_INT'], {  # GLOBAL_POSITION_INT
            "time_boot_ms": int(now * 1000) & 0xFFFFFFFF,
            "lat": int(v.lat * 1e7), "lon": int(v.lon * 1e7),
            "alt": msl_mm, "relative_alt": int(v.alt * 1000),
            "vx": int(v.vx * 100), "vy": int(v.vy * 100), "vz": int(v.vz * 100),
            "hdg": int(v.hdg * 100) % 36000,
        })
        self._send(M.MSG_ID['GPS_RAW_INT'], {  # GPS_RAW_INT
            "time_usec": int(now * 1e6) & 0xFFFFFFFFFFFFFFFF,
            "lat": int(v.lat * 1e7), "lon": int(v.lon * 1e7),
            "alt": msl_mm, "eph": 80, "epv": 80,
            "vel": int(math.hypot(v.vx, v.vy) * 100), "cog": int(v.hdg * 100),
            "fix_type": v.fix, "satellites_visible": v.sats,
        })
        d = math.pi / 180.0
        self._send(M.MSG_ID['ATTITUDE'], {  # ATTITUDE
            "time_boot_ms": int(now * 1000) & 0xFFFFFFFF,
            "roll": v.roll * d, "pitch": v.pitch * d, "yaw": v.hdg * d,
            "rollspeed": 0.01 * math.sin(now), "pitchspeed": 0.01 * math.cos(now),
            "yawspeed": 0.0,
        })
        if now - self._last_sys > 1.0:
            self._last_sys = now
            self._send(M.MSG_ID['SYS_STATUS'], {  # SYS_STATUS
                "onboard_control_sensors_present": ALL_SENSORS,
                "onboard_control_sensors_enabled": ALL_SENSORS,
                "onboard_control_sensors_health": ALL_SENSORS,
                "load": 250, "voltage_battery": int(v.batt_v * 1000),
                "current_battery": int(v.batt_i * 100),
                "drop_rate_comm": 0, "errors_comm": 0,
                "errors_count1": 0, "errors_count2": 0,
                "errors_count3": 0, "errors_count4": 0,
                "battery_remaining": int(v.batt_pct) if v.batt_pct >= 0 else -1,
            })
            volt = int(v.batt_v * 1000)
            self._send(M.MSG_ID['BATTERY_STATUS'], {  # BATTERY_STATUS
                "current_consumed": 0, "energy_consumed": 0,
                "temperature": 2800, "voltages": (volt, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                "current_battery": int(v.batt_i * 100),
                "id": 0, "battery_function": 0, "type": 0,
                "battery_remaining": int(v.batt_pct) if v.batt_pct >= 0 else -1,
                "time_remaining": -1, "charge_state": 0,
                "voltages_ext": (0, 0, 0, 0), "mode": 0, "fault_bitmask": 0,
            })
        # MISSION_CURRENT — id from the generated table (42 in these definitions)
        self._send(M.MSG_ID["MISSION_CURRENT"],
                   {"seq": v.mission_seq, "total": v.mission_total,
                    "mission_state": 0, "mission_mode": v.mode_num})

    # ------------------------------------------------------------------ #
    def _handle(self, msgid, src_sys, f):
        name = M.name_of(msgid)
        if src_sys == PI_SYSID:
            # the Pi over USB: we relay its STATUSTEXT to the GCS exactly like
            # ArduPilot does (this is what lands in Mission Planner's console)
            if name == "STATUSTEXT":
                self._send(M.MSG_ID['STATUSTEXT'], {"severity": f["severity"], "text": f["text"]})
                print("[mock-fc] relay Pi STATUSTEXT -> GCS: %r"
                      % (bytes(f["text"]).rstrip(b"\x00").decode("utf-8", "replace"),))
            return

        if name == "HEARTBEAT":
            return  # GCS heartbeat — noted, nothing to do

        if name == "MISSION_COUNT":
            if f["mission_type"] == M.MAV_MISSION_TYPE_FENCE:
                self._upload_fence(f["count"], f["target_system"], f["target_component"])
            return

        if name == "MISSION_REQUEST_LIST":
            if f["mission_type"] == M.MAV_MISSION_TYPE_FENCE:
                self._send(M.MSG_ID['MISSION_COUNT'], {"count": len(self.fence_items),
                                "target_system": f["target_system"],
                                "target_component": f["target_component"],
                                "mission_type": M.MAV_MISSION_TYPE_FENCE})
            elif f["mission_type"] == M.MAV_MISSION_TYPE_MISSION:
                self._send(M.MSG_ID['MISSION_COUNT'], {"count": len(self.plan),
                                "target_system": f["target_system"],
                                "target_component": f["target_component"],
                                "mission_type": M.MAV_MISSION_TYPE_MISSION})
            return

        if name == "MISSION_REQUEST_INT":
            ts, tc = f["target_system"], f["target_component"]
            if f["mission_type"] == M.MAV_MISSION_TYPE_FENCE:
                seq = f["seq"]
                if seq < len(self.fence_items):
                    x, y, n = self.fence_items[seq]
                    self._send(M.MSG_ID['MISSION_ITEM_INT'], {
                        "param1": float(n), "param2": 0.0, "param3": 0.0, "param4": 0.0,
                        "x": x, "y": y, "z": 0.0, "seq": seq,
                        "command": M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                        "target_system": ts, "target_component": tc,
                        "frame": M.MAV_FRAME_GLOBAL, "current": 0, "autocontinue": 0,
                        "mission_type": M.MAV_MISSION_TYPE_FENCE,
                    })
                    if seq == len(self.fence_items) - 1:
                        self._send(M.MSG_ID['MISSION_ACK'], {"target_system": ts, "target_component": tc,
                                        "type": M.MAV_MISSION_ACCEPTED,
                                        "mission_type": M.MAV_MISSION_TYPE_FENCE})
            elif f["mission_type"] == M.MAV_MISSION_TYPE_MISSION:
                seq = f["seq"]
                if seq < len(self.plan):
                    s, cmd, alt = self.plan[seq]
                    self._send(M.MSG_ID['MISSION_ITEM_INT'], {
                        "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0,
                        "x": 0, "y": 0, "z": alt, "seq": s,
                        "command": cmd, "target_system": ts, "target_component": tc,
                        "frame": 1 if cmd != M.MAV_CMD_DO_SPRAYER else M.MAV_FRAME_GLOBAL,
                        "current": 0, "autocontinue": 0,
                        "mission_type": M.MAV_MISSION_TYPE_MISSION,
                    })
                    if s == len(self.plan) - 1:
                        self._send(M.MSG_ID['MISSION_ACK'], {"target_system": ts, "target_component": tc,
                                        "type": M.MAV_MISSION_ACCEPTED,
                                        "mission_type": M.MAV_MISSION_TYPE_MISSION})
            return

        if name == "MISSION_CLEAR_ALL":
            if f["mission_type"] == M.MAV_MISSION_TYPE_FENCE:
                self.fence_items = []
                self._send(M.MSG_ID['MISSION_ACK'], {"target_system": f["target_system"],
                                "target_component": f["target_component"],
                                "type": M.MAV_MISSION_ACCEPTED,
                                "mission_type": M.MAV_MISSION_TYPE_FENCE})
            return

        if name == "PARAM_SET":
            pid = bytes(f["param_id"]).rstrip(b"\x00").decode("ascii", "replace")
            self.params[pid] = f["param_value"]
            self._send(M.MSG_ID['PARAM_VALUE'], {"param_value": f["param_value"], "param_count": 3,
                            "param_index": 0, "param_id": f["param_id"],
                            "param_type": M.MAV_PARAM_TYPE_REAL32})
            return

        if name == "PARAM_REQUEST_READ":
            pid = bytes(f["param_id"]).rstrip(b"\x00").decode("ascii", "replace")
            val = self.params.get(pid, 0.0)
            self._send(M.MSG_ID['PARAM_VALUE'], {"param_value": val, "param_count": 3,
                            "param_index": 0,
                            "param_id": pid.encode("ascii")[:16],
                            "param_type": M.MAV_PARAM_TYPE_REAL32})
            return

        if name == "COMMAND_LONG":
            ts, tc = f["target_system"], f["target_component"]
            if f["command"] == M.MAV_CMD_DO_SET_MODE:
                mode = int(f["param1"])
                print("[mock-fc] DO_SET_MODE -> %d (%s)"
                      % (mode, M.ARDU_COPTER_MODES.get(mode, "?")))
                if self.vehicle is not None:
                    self.vehicle.mode_override = mode
                    self.vehicle.mode_override_ts = time.time()
                self._send(M.MSG_ID['COMMAND_ACK'], {"command": f["command"], "result": 0})
            elif f["command"] == M.MAV_CMD_REQUEST_MESSAGE:
                if int(f["param1"]) == M.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                    self._send(M.MSG_ID['GPS_GLOBAL_ORIGIN'], {"latitude": int(self.origin[0] * 1e7),
                                    "longitude": int(self.origin[1] * 1e7),
                                    "altitude": 340000, "time_usec": 0})
                self._send(M.MSG_ID['COMMAND_ACK'], {"command": f["command"], "result": 0})
            else:
                self._send(M.MSG_ID['COMMAND_ACK'], {"command": f["command"], "result": 3})  # unsupported
            return

        if name == "STATUSTEXT":
            print("[mock-fc] GCS STATUSTEXT: %r"
                  % (bytes(f["text"]).rstrip(b"\x00").decode("utf-8", "replace"),))
            return

    def _upload_fence(self, count, ts, tc):
        """FC drives the upload: request each item, wait for it, ACK.
        (Mirrors ArduPilot's GCS_MAVLink mission-protocol server behavior.)"""
        items = []
        for seq in range(count):
            self._send(M.MSG_ID['MISSION_REQUEST_INT'], {"seq": seq, "target_system": ts, "target_component": tc,
                            "mission_type": M.MAV_MISSION_TYPE_FENCE})
            deadline = time.time() + 2.0
            item = None
            while time.time() < deadline:
                try:
                    data, _addr = self.sock.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    return
                p = M.parse_datagram(data)
                if p is None:
                    continue
                mid, _ss, _cc, _s, f = p
                if mid == 73 and f["mission_type"] == M.MAV_MISSION_TYPE_FENCE and f["seq"] == seq:
                    item = f
                    break
                self._handle(mid, _ss, f)   # don't drop interleaved messages
            if item is None:
                print("[mock-fc] upload timeout at seq %d" % seq)
                return
            items.append((item["x"], item["y"], int(item["param1"])))
        self.fence_items = items
        self.params.setdefault("FENCE_TYPE", 0.0)
        self._send(M.MSG_ID['MISSION_ACK'], {"target_system": ts, "target_component": tc,
                        "type": M.MAV_MISSION_ACCEPTED,
                        "mission_type": M.MAV_MISSION_TYPE_FENCE})
        print("[mock-fc] fence stored: %d verts (readback will confirm)" % len(items))
