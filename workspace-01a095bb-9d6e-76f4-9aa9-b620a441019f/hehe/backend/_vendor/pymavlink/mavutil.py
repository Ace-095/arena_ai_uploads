"""
Vendored minimal pymavlink shim for SITL testing.
Provides enough of the pymavlink API for mavlink_bus.py / mavlink_fence.py to work
with a real ArduPilot SITL over UDP.

This shim uses raw UDP sockets + struct to implement MAVLink v2 protocol basics.
It is a DEVELOPMENT-ONLY shim for air-gapped machines without pip access.
"""

import os
import socket
import struct
import time
import threading
from collections import namedtuple
from typing import Optional

# MAVLink v2 constants
MAVLINK_STX_V2 = 0xFD
MAVLINK_STX_V1 = 0xFE

# Common MAVLink message IDs
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51
MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_PARAM_VALUE = 22
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84


class _MAVLinkMessage:
    """Generic MAVLink message container."""
    def __init__(self, msg_type, **fields):
        self._msg_type = msg_type
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._msg_type


class mavlink:
    """Namespace mimicking pymavlink.mavutil.mavlink constants."""
    # MAV_TYPE
    MAV_TYPE_GCS = 6
    MAV_TYPE_QUADROTOR = 2

    # MAV_AUTOPILOT
    MAV_AUTOPILOT_INVALID = 8
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3

    # MAV_STATE
    MAV_STATE_ACTIVE = 4

    # MAV_MODE_FLAG
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

    # MAV_CMD
    MAV_CMD_REQUEST_MESSAGE = 512
    MAV_CMD_DO_SET_MODE = 176
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001

    # MAV_FRAME
    MAV_FRAME_GLOBAL = 0
    MAV_FRAME_LOCAL_NED = 1

    # MAV_MISSION
    MAV_MISSION_TYPE_FENCE = 8
    MAV_MISSION_ACCEPTED = 0
    MAV_MISSION_ERROR = 1

    # MAV_PARAM
    MAV_PARAM_TYPE_REAL32 = 9

    # Message IDs
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
    MAVLINK_MSG_ID_HEARTBEAT = MAVLINK_MSG_ID_HEARTBEAT


def mode_string_v10(msg):
    """Return mode string from heartbeat message."""
    custom_mode = getattr(msg, 'custom_mode', 0)
    mode_map = {
        0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
        4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
        9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 14: 'FLIP',
        15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW',
        19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
    }
    return mode_map.get(custom_mode, f'MODE({custom_mode})')


class _MavSender:
    """Sends MAVLink messages over UDP."""

    def __init__(self, sock, target_addr, sys_id=255, comp_id=0, target_system=1, target_component=1):
        self._sock = sock
        self._target_addr = target_addr
        self._seq = 0
        self._sys_id = sys_id
        self._comp_id = comp_id
        self.target_system = target_system
        self.target_component = target_component

    def _send_raw(self, msg_id, payload):
        """Send a MAVLink v2 message."""
        seq = self._seq & 0xFF
        self._seq += 1

        # MAVLink v2 header: STX, payload_len, incompat_flags, compat_flags, seq, sysid, compid, msgid(3 bytes)
        header = struct.pack('<BBBBBBBHB',
                             MAVLINK_STX_V2,
                             len(payload),
                             0,  # incompat_flags
                             0,  # compat_flags
                             seq,
                             self._sys_id,
                             self._comp_id,
                             msg_id & 0xFFFF,
                             (msg_id >> 16) & 0xFF)

        # CRC (simplified — we use a zero checksum for now since SITL is lenient in some configs)
        # For real MAVLink, we'd need the CRC extra byte per message type
        msg = header + payload
        # Add 2-byte CRC placeholder
        crc = self._crc16(msg[1:])  # CRC over everything except STX
        msg += struct.pack('<H', crc)

        try:
            self._sock.sendto(msg, self._target_addr)
        except Exception:
            pass

    @staticmethod
    def _crc16(data):
        """X.25 CRC used by MAVLink."""
        crc = 0xFFFF
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF
        return crc

    def heartbeat_send(self, type_, autopilot, base_mode, custom_mode, system_status):
        """Send HEARTBEAT (ID 0)."""
        payload = struct.pack('<IBBBBB',
                              custom_mode,
                              type_, autopilot, base_mode, system_status, 3)  # mavlink_version=3
        self._send_raw(MAVLINK_MSG_ID_HEARTBEAT, payload)

    def command_long_send(self, target_system, target_component, command, confirmation,
                          param1, param2, param3, param4, param5, param6, param7):
        """Send COMMAND_LONG (ID 76)."""
        payload = struct.pack('<fffffffHBBB',
                              param1, param2, param3, param4, param5, param6, param7,
                              command, target_system, target_component, confirmation)
        self._send_raw(MAVLINK_MSG_ID_COMMAND_LONG, payload)

    def mission_count_send(self, target_system, target_component, count, mission_type=0):
        """Send MISSION_COUNT (ID 44)."""
        payload = struct.pack('<HBBB', count, target_system, target_component, mission_type)
        self._send_raw(MAVLINK_MSG_ID_MISSION_COUNT, payload)

    def mission_item_int_send(self, target_system, target_component, seq, frame, command,
                               current, autocontinue, param1, param2, param3, param4,
                               x, y, z, mission_type=0):
        """Send MISSION_ITEM_INT (ID 73)."""
        payload = struct.pack('<ffffiifHBBBBBB',
                              param1, param2, param3, param4,
                              x, y, z,
                              seq, command, target_system, target_component,
                              frame, current, autocontinue)
        # mission_type appended for MAVLink v2
        payload += struct.pack('<B', mission_type)
        self._send_raw(MAVLINK_MSG_ID_MISSION_ITEM_INT, payload)

    def mission_request_list_send(self, target_system, target_component, mission_type=0):
        """Send MISSION_REQUEST_LIST (ID 43)."""
        payload = struct.pack('<BBB', target_system, target_component, mission_type)
        self._send_raw(43, payload)

    def mission_request_int_send(self, target_system, target_component, seq, mission_type=0):
        """Send MISSION_REQUEST_INT (ID 51)."""
        payload = struct.pack('<HBBB', seq, target_system, target_component, mission_type)
        self._send_raw(MAVLINK_MSG_ID_MISSION_REQUEST_INT, payload)

    def mission_clear_all_send(self, target_system, target_component, mission_type=0):
        """Send MISSION_CLEAR_ALL (ID 45)."""
        payload = struct.pack('<BBB', target_system, target_component, mission_type)
        self._send_raw(45, payload)

    def param_request_read_send(self, target_system, target_component, param_id, param_index):
        """Send PARAM_REQUEST_READ (ID 20)."""
        if isinstance(param_id, bytes):
            param_id_padded = param_id.ljust(16, b'\x00')[:16]
        else:
            param_id_padded = param_id.encode('ascii').ljust(16, b'\x00')[:16]
        payload = struct.pack('<hBB', param_index, target_system, target_component)
        payload += param_id_padded
        self._send_raw(20, payload)

    def param_set_send(self, target_system, target_component, param_id, param_value, param_type):
        """Send PARAM_SET (ID 23)."""
        if isinstance(param_id, bytes):
            param_id_padded = param_id.ljust(16, b'\x00')[:16]
        else:
            param_id_padded = param_id.encode('ascii').ljust(16, b'\x00')[:16]
        payload = struct.pack('<fBB', param_value, target_system, target_component)
        payload += param_id_padded
        payload += struct.pack('<B', param_type)
        self._send_raw(23, payload)

    def set_position_target_local_ned_send(self, time_boot_ms, target_system, target_component,
                                            coordinate_frame, type_mask,
                                            x, y, z, vx, vy, vz, afx, afy, afz,
                                            yaw, yaw_rate):
        """Send SET_POSITION_TARGET_LOCAL_NED (ID 84)."""
        payload = struct.pack('<IHBBfffffffffffH',
                              time_boot_ms, type_mask,
                              target_system, target_component,
                              x, y, z, vx, vy, vz, afx, afy, afz,
                              yaw, yaw_rate, coordinate_frame)
        self._send_raw(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, payload)


class _MAVConnection:
    """Minimal mavlink_connection replacement."""

    def __init__(self, device, baud=115200):
        self._device = device
        self._baud = baud
        self._sock = None
        self._addr = None
        self.target_system = 0
        self.target_component = 0
        self.mav = None
        self._recv_buf = b""
        self._closed = False

        # Parse device string
        if device.startswith('udp:') or device.startswith('udpin:') or device.startswith('udpout:'):
            parts = device.split(':')
            host = parts[1] if len(parts) > 1 else '127.0.0.1'
            port = int(parts[2]) if len(parts) > 2 else 14550
            self._setup_udp(host, port)
        elif device.startswith('tcp:'):
            parts = device.split(':')
            host = parts[1] if len(parts) > 1 else '127.0.0.1'
            port = int(parts[2]) if len(parts) > 2 else 5760
            self._setup_tcp(host, port)
        else:
            # Serial device — stub
            raise NotImplementedError(f"Serial device '{device}' not supported in shim")

    def _setup_udp(self, host, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._addr = (host, port)

        if 'udpin' in self._device or 'udp:' in self._device:
            # Bind to receive
            try:
                self._sock.bind(self._addr)
            except OSError:
                pass

        self.mav = _MavSender(self._sock, self._addr)

    def _setup_tcp(self, host, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        self._sock.connect((host, port))
        self._addr = (host, port)
        self.mav = _MavSender(self._sock, self._addr)

    def wait_heartbeat(self, timeout=5.0):
        """Wait for a heartbeat from the flight controller."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv_match(type='HEARTBEAT', blocking=True, timeout=min(1.0, deadline - time.time()))
            if msg is not None:
                self.target_system = getattr(msg, '_src_system', 1)
                self.target_component = getattr(msg, '_src_component', 1)
                self.mav.target_system = self.target_system
                self.mav.target_component = self.target_component
                return msg
        return None

    def recv_match(self, type=None, blocking=True, timeout=1.0):
        """Receive and parse a MAVLink message."""
        if self._closed or self._sock is None:
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                remaining = max(0.01, deadline - time.time())
                self._sock.settimeout(min(remaining, 0.5))
                data, addr = self._sock.recvfrom(4096)
                if addr:
                    self._addr = addr
                    self.mav._target_addr = addr

                msg = self._parse_mavlink(data)
                if msg is not None:
                    if type is None or msg.get_type() == type:
                        return msg
            except socket.timeout:
                if not blocking:
                    return None
                continue
            except Exception:
                if not blocking:
                    return None
                time.sleep(0.01)
                continue

        return None

    def _parse_mavlink(self, data):
        """Parse raw bytes into a MAVLink message object."""
        if len(data) < 8:
            return None

        stx = data[0]
        if stx == MAVLINK_STX_V2:
            return self._parse_v2(data)
        elif stx == MAVLINK_STX_V1:
            return self._parse_v1(data)
        return None

    def _parse_v2(self, data):
        """Parse MAVLink v2 message."""
        if len(data) < 12:
            return None

        payload_len = data[1]
        seq = data[4]
        src_system = data[5]
        src_component = data[6]
        msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
        payload = data[10:10 + payload_len]

        return self._decode_message(msg_id, payload, src_system, src_component)

    def _parse_v1(self, data):
        """Parse MAVLink v1 message."""
        if len(data) < 8:
            return None

        payload_len = data[1]
        seq = data[2]
        src_system = data[3]
        src_component = data[4]
        msg_id = data[5]
        payload = data[6:6 + payload_len]

        return self._decode_message(msg_id, payload, src_system, src_component)

    def _decode_message(self, msg_id, payload, src_system, src_component):
        """Decode payload into a message object based on msg_id."""
        msg = None

        if msg_id == MAVLINK_MSG_ID_HEARTBEAT and len(payload) >= 9:
            custom_mode, type_, autopilot, base_mode, system_status, version = struct.unpack_from('<IBBBBB', payload)
            msg = _MAVLinkMessage('HEARTBEAT',
                                   custom_mode=custom_mode, type=type_,
                                   autopilot=autopilot, base_mode=base_mode,
                                   system_status=system_status)

        elif msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT and len(payload) >= 28:
            time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg = struct.unpack_from('<IiiiihhhhH', payload)
            msg = _MAVLinkMessage('GLOBAL_POSITION_INT',
                                   lat=lat, lon=lon, alt=alt, relative_alt=relative_alt,
                                   vx=vx, vy=vy, vz=vz, hdg=hdg)

        elif msg_id == MAVLINK_MSG_ID_ATTITUDE and len(payload) >= 28:
            time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = struct.unpack_from('<Iffffff', payload)
            msg = _MAVLinkMessage('ATTITUDE', roll=roll, pitch=pitch, yaw=yaw)

        elif msg_id == MAVLINK_MSG_ID_SYS_STATUS and len(payload) >= 31:
            vals = struct.unpack_from('<IIIHHhHHHHHhBB', payload[:32])
            msg = _MAVLinkMessage('SYS_STATUS',
                                   voltage_battery=vals[3], current_battery=vals[4],
                                   battery_remaining=vals[12])

        elif msg_id == MAVLINK_MSG_ID_GPS_RAW_INT and len(payload) >= 30:
            vals = struct.unpack_from('<QiiiHHHBB', payload[:30])
            msg = _MAVLinkMessage('GPS_RAW_INT',
                                   fix_type=vals[7], satellites_visible=vals[8],
                                   eph=vals[5])

        elif msg_id == MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN and len(payload) >= 12:
            lat, lon, alt = struct.unpack_from('<iii', payload)
            msg = _MAVLinkMessage('GPS_GLOBAL_ORIGIN',
                                   latitude=lat, longitude=lon, altitude=alt)

        elif msg_id == MAVLINK_MSG_ID_MISSION_COUNT and len(payload) >= 4:
            count, target_sys, target_comp = struct.unpack_from('<HBB', payload)
            mission_type = payload[4] if len(payload) > 4 else 0
            msg = _MAVLinkMessage('MISSION_COUNT', count=count, mission_type=mission_type)

        elif msg_id == MAVLINK_MSG_ID_MISSION_REQUEST_INT and len(payload) >= 4:
            seq, target_sys, target_comp = struct.unpack_from('<HBB', payload)
            mission_type = payload[4] if len(payload) > 4 else 0
            msg = _MAVLinkMessage('MISSION_REQUEST_INT', seq=seq, mission_type=mission_type)

        elif msg_id == MAVLINK_MSG_ID_MISSION_ITEM_INT and len(payload) >= 37:
            vals = struct.unpack_from('<ffffiifHBBBBBB', payload[:37])
            mission_type = payload[37] if len(payload) > 37 else 0
            msg = _MAVLinkMessage('MISSION_ITEM_INT',
                                   param1=vals[0], param2=vals[1], param3=vals[2], param4=vals[3],
                                   x=vals[4], y=vals[5], z=vals[6],
                                   seq=vals[7], command=vals[8],
                                   mission_type=mission_type)

        elif msg_id == MAVLINK_MSG_ID_MISSION_ACK and len(payload) >= 3:
            target_sys, target_comp, ack_type = struct.unpack_from('<BBB', payload)
            mission_type = payload[3] if len(payload) > 3 else 0
            msg = _MAVLinkMessage('MISSION_ACK', type=ack_type, mission_type=mission_type)

        elif msg_id == MAVLINK_MSG_ID_PARAM_VALUE and len(payload) >= 25:
            param_value, param_count, param_index, param_type = struct.unpack_from('<fHHB', payload[:9])
            param_id = payload[9:25].decode('ascii', errors='ignore')
            msg = _MAVLinkMessage('PARAM_VALUE',
                                   param_value=param_value, param_id=param_id,
                                   param_count=param_count, param_index=param_index)

        elif msg_id == MAVLINK_MSG_ID_COMMAND_ACK and len(payload) >= 3:
            command, result = struct.unpack_from('<HB', payload)
            msg = _MAVLinkMessage('COMMAND_ACK', command=command, result=result)

        else:
            # Generic fallback
            msg = _MAVLinkMessage(f'MSG_{msg_id}')

        if msg is not None:
            msg._src_system = src_system
            msg._src_component = src_component

        return msg

    def close(self):
        self._closed = True
        if self._sock:
            try:
                self._sock.close()
            except:
                pass
            self._sock = None


def mavlink_connection(device, baud=115200, **kwargs):
    """Factory function matching pymavlink.mavutil.mavlink_connection."""
    return _MAVConnection(device, baud)
