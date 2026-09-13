"""
Minimal mock flight controller that speaks just enough of the ArduPilot
fence + param protocol to validate mavlink_fence.py end-to-end over a real
MAVLink/UDP transport, without needing a Pixhawk or a full SITL build.

This is a correctness check for the client-side protocol implementation,
NOT a substitute for testing against real ArduPilot firmware (SITL or
hardware) before competition. It doesn't replicate ArduPilot's actual
fence storage/enforcement logic, EKF behavior, or edge cases — just enough
of the wire protocol to catch client-side bugs.

IMPORTANT: Uses udp:0.0.0.0:PORT with input=True to listen for incoming
connections. The test client connects to udp:127.0.0.1:PORT with input=False.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import threading
import time
from pymavlink import mavutil


def run_mock_fc(bind_addr: str, stop_event: threading.Event,
                origin_lat: float = 12.9716, origin_lon: float = 77.5946,
                simulated_relative_alt_m: float = 14.0):
    # bind_addr should be like "udp:0.0.0.0:14560" - use input=True to listen
    conn = mavutil.mavlink_connection(bind_addr, input=True)
    fence_items = []  # list of (x_int, y_int, command)
    last_hb = 0.0
    last_pos = 0.0
    boot_time = time.time()  # reference for time_boot_ms — must stay well within uint32 range
    # State for upload protocol
    upload_expected_count = 0
    upload_received = {}  # seq -> (x, y, cmd)
    upload_next_seq = 0
    fence_items = []  # This is missing in the original code

    while not stop_event.is_set():
        now = time.time()
        if now - last_hb > 1.0:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                0, 0, mavutil.mavlink.MAV_STATE_STANDBY,
            )
            last_hb = now

        if now - last_pos > 0.3:
            # A stationary fake position at origin_lat/lon, holding at
            # simulated_relative_alt_m — enough for tests that need SOME
            # position feed (e.g. search_algorithm's approach-descent
            # logic, which reads GLOBAL_POSITION_INT to know where to
            # hold/descend from). Doesn't simulate real flight dynamics —
            # this is a fixed value, not a moving vehicle.
            conn.mav.global_position_int_send(
                int((now - boot_time) * 1000),  # time_boot_ms — relative, not epoch
                int(origin_lat * 1e7),
                int(origin_lon * 1e7),
                int(simulated_relative_alt_m * 1000),  # alt (mm)
                int(simulated_relative_alt_m * 1000),  # relative_alt (mm)
                0, 0, 0,  # vx, vy, vz (cm/s)
                65535,  # hdg unknown
            )
            last_pos = now

        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            if int(msg.param1) == mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                conn.mav.gps_global_origin_send(
                    int(origin_lat * 1e7), int(origin_lon * 1e7), 0, int(time.time() * 1e6)
                )

        elif mtype == "MISSION_COUNT" and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            count = msg.count
            target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
            target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
            received_items = []
            for seq in range(count):
                conn.mav.mission_request_int_send(
                    target_system, target_component, seq,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )
                deadline = time.time() + 2.0
                item_msg = None
                while time.time() < deadline:
                    m = conn.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=0.1)
                    if m and m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE and m.seq == seq:
                        item_msg = m
                        break
                if item_msg is not None:
                    received_items.append((item_msg.x, item_msg.y, item_msg.command))
                else:
                    break

            if len(received_items) == count:
                fence_items = received_items
                conn.mav.mission_ack_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )

        elif mtype == "MISSION_REQUEST_LIST" and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            # Client is downloading the fence - send count then items
            count = len(fence_items)
            target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
            target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
            conn.mav.mission_count_send(target_system, target_component, count, mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        elif mtype == "MISSION_REQUEST_INT" and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            # Client is requesting an item during download
            seq = msg.seq
            if seq < len(fence_items):
                x, y, cmd = fence_items[seq]
                target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
                target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
                conn.mav.mission_item_int_send(
                    target_system, target_component, seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL, cmd,
                    0, 0,  # current, autocontinue
                    0, 0, 0, 0,  # params 1-4
                    x, y, 0.0,  # x, y, z
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )
            # If this was the last item, send ACK
            if seq == len(fence_items) - 1:
                target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
                target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
                conn.mav.mission_ack_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )

        elif mtype == "MISSION_CLEAR_ALL" and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            fence_items = []
            upload_expected_count = 0
            upload_received = {}
            upload_next_seq = 0
            target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
            target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
            conn.mav.mission_ack_send(
                target_system, target_component,
                mavutil.mavlink.MAV_MISSION_ACCEPTED,
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )

        elif mtype == "PARAM_SET":
            param_id = msg.param_id
            if isinstance(param_id, str):
                param_id = param_id.encode("ascii")
            target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
            target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
            conn.mav.param_value_send(
                param_id, msg.param_value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                target_system, target_component
            )

        elif mtype == "PARAM_REQUEST_READ":
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id_str = param_id.decode("ascii", errors="ignore").rstrip("\x00")
            else:
                param_id_str = str(param_id).rstrip("\x00")

            val = 4.0 if "FENCE_TYPE" in param_id_str else 1.0
            target_system = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else 1
            target_component = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else 1
            conn.mav.param_value_send(
                param_id_str.encode("ascii"), val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                target_system, target_component
            )

    try:
        conn.close()
    except Exception:
        pass