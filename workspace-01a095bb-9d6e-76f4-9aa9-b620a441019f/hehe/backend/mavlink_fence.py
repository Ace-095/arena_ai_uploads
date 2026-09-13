"""
MAVLink polygon-fence upload/download for ArduPilot Copter (Pixhawk 6C).

Protocol details below were verified against ArduPilot's own source
(GCS_MAVLink/MissionItemProtocol_Fence.cpp) and the MAVLink mission
protocol docs (mavlink.io/en/services/mission.html), not written from
memory. Things worth knowing before you touch this file:

1. MAVLINK20 must be set to "1" in the environment BEFORE pymavlink is
   imported anywhere in the process. `mavutil.mavlink` silently resolves
   to the MAVLink1 dialect otherwise, which has no `mission_type` field at
   all — fence upload will silently fail with a confusing "mission ack:
   denied" if you miss this. This module sets it at import time.
2. The `vertex_count` sent in every MISSION_ITEM_INT must equal the total
   number of vertices in the polygon (every item, not just the first —
   confirmed against ArduPilot's source, `ret.vertex_count =
   mission_item_int.param1` runs per-item).
3. Every function below subscribes to the expected response BEFORE
   sending the request that triggers it. This is not incidental style —
   the naive "send, then subscribe and wait" pattern has a real race:
   MavlinkBus does not buffer messages for subscribers that don't exist
   yet, so on a fast link (a loopback mock is fast enough to trigger this
   reliably) the flight controller's reply can arrive before anything is
   listening for it, and gets silently dropped. Confirmed by running
   test_fence_protocol.py twice and getting two different non-deterministic
   timeout failures on identical code. Do not reorder subscribe/send in
   any function added here later.

All functions here are stateless and take a `MavlinkBus` instance.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import queue
import time
from typing import Callable, Optional

from pymavlink import mavutil

from geo_utils import latlon_to_enu, polygon_centroid, max_radius_from_point, point_in_polygon
from models import GeofenceStatus, Vertex, ENUPoint
from mavlink_bus import MavlinkBus, FenceError, ConnectionError


# ---------------------------------------------------------------------------
# Subscribe-before-send helpers (the fix)
# ---------------------------------------------------------------------------

def _subscribe_then_send(bus: MavlinkBus, msg_types: list[str], send_fn: Callable, *args, **kwargs) -> queue.Queue:
    """
    Subscribe to msg_types, THEN send the request. Returns the queue —
    caller must call bus.unsubscribe(q) when done (use try/finally).
    Order matters: subscribing first guarantees no response, however
    fast, can be missed.
    """
    q = bus.subscribe(msg_types)
    bus.send(send_fn, *args, **kwargs)
    return q


def _pull_until(q: queue.Queue, predicate: Callable, deadline: float):
    """Pull from an already-subscribed queue until predicate(msg) is True
    or the deadline passes. Returns the matching msg or None."""
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        try:
            msg = q.get(timeout=remaining)
        except queue.Empty:
            continue
        if predicate(msg):
            return msg


# ---------------------------------------------------------------------------
# EKF origin
# ---------------------------------------------------------------------------

def request_ekf_origin(bus: MavlinkBus, timeout_s: float) -> tuple[float, float]:
    """
    Request the EKF origin (GPS_GLOBAL_ORIGIN) from the flight controller.
    Returns (lat, lon) in degrees (WGS84). Raises FenceError on timeout
    or missing origin.
    """
    deadline = time.time() + timeout_s
    q = _subscribe_then_send(
        bus,
        ["GPS_GLOBAL_ORIGIN"],
        bus.conn.mav.command_long_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
        0, 0, 0, 0, 0, 0,
    )
    try:
        msg = _pull_until(q, lambda m: True, deadline)
    finally:
        bus.unsubscribe(q)

    if msg is None:
        raise FenceError("Timeout waiting for GPS_GLOBAL_ORIGIN from flight controller")

    lat = msg.latitude / 1e7
    lon = msg.longitude / 1e7
    if lat == 0 and lon == 0:
        raise FenceError("FC returned zero origin (EKF not initialized?)")
    return lat, lon


# ---------------------------------------------------------------------------
# Fence upload
# ---------------------------------------------------------------------------

def upload_polygon_fence(bus: MavlinkBus, vertices: list[Vertex], timeout_s: float) -> None:
    """
    Upload a polygon fence via MAVLink mission protocol (mission_type=FENCE).
    """
    n = len(vertices)
    if n < 3:
        raise FenceError("Polygon needs at least 3 vertices")
    if n > 255:
        raise FenceError("Polygon exceeds 255 vertices (ArduPilot fence storage limit)")

    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    deadline = time.time() + timeout_s

    # Subscribe to BOTH response types we'll need across this whole
    # transaction before sending anything — MISSION_COUNT triggers a
    # stream of MISSION_REQUEST_INT, followed by one MISSION_ACK.
    q = _subscribe_then_send(
        bus,
        ["MISSION_REQUEST_INT", "MISSION_ACK"],
        bus.conn.mav.mission_count_send,
        bus.conn.target_system,
        bus.conn.target_component,
        n,
        mission_type=mission_type,
    )
    try:
        sent: set[int] = set()
        ack = None
        while len(sent) < n or ack is None:
            remaining = deadline - time.time()
            if remaining <= 0:
                if len(sent) < n:
                    raise FenceError(f"Timeout waiting for MISSION_REQUEST_INT seq={len(sent)}")
                raise FenceError("Timeout waiting for MISSION_ACK after fence upload")
            try:
                msg = q.get(timeout=remaining)
            except queue.Empty:
                continue
            if msg.mission_type != mission_type:
                continue

            mtype = msg.get_type()
            if mtype == "MISSION_REQUEST_INT" and msg.seq not in sent:
                seq = msg.seq
                if seq >= n:
                    raise FenceError(f"FC requested out-of-range seq {seq} (n={n})")
                v = vertices[seq]
                bus.send(
                    bus.conn.mav.mission_item_int_send,
                    bus.conn.target_system,
                    bus.conn.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    0,  # current
                    0,  # autocontinue
                    float(n),  # param1: vertex_count, every item (see module docstring)
                    0.0, 0.0, 0.0,  # params 2-4 unused (must be floats)
                    int(v.lat * 1e7),
                    int(v.lon * 1e7),
                    0.0,  # alt unused for fence vertices
                    mission_type,
                )
                sent.add(seq)
            elif mtype == "MISSION_ACK":
                ack = msg

        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise FenceError(f"Fence upload rejected by FC: MAV_MISSION_ERROR type={ack.type}")
    finally:
        bus.unsubscribe(q)


# ---------------------------------------------------------------------------
# Fence download
# ---------------------------------------------------------------------------

def download_fence(bus: MavlinkBus, timeout_s: float) -> list[Vertex]:
    """
    Download the currently stored fence from the flight controller.
    Returns list of Vertex (lat, lon). Preserves the one-item-at-a-time
    request cadence the FC drives (documented MAVLink mission download
    behavior) — only the subscribe timing changed, not the protocol shape.
    """
    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    deadline = time.time() + timeout_s

    q = _subscribe_then_send(
        bus,
        ["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"],
        bus.conn.mav.mission_request_list_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mission_type=mission_type,
    )
    try:
        count_msg = _pull_until(
            q, lambda m: m.mission_type == mission_type and m.get_type() == "MISSION_COUNT", deadline
        )
        if count_msg is None:
            raise FenceError("Timeout waiting for MISSION_COUNT on fence download")
        count = count_msg.count
        if count == 0:
            return []

        vertices: list[Vertex] = []
        for seq in range(count):
            bus.send(
                bus.conn.mav.mission_request_int_send,
                bus.conn.target_system,
                bus.conn.target_component,
                seq,
                mission_type=mission_type,
            )
            item = _pull_until(
                q,
                lambda m: m.mission_type == mission_type
                and m.get_type() == "MISSION_ITEM_INT"
                and m.seq == seq,
                deadline,
            )
            if item is None:
                raise FenceError(f"Timeout waiting for MISSION_ITEM_INT seq={seq} on download")
            vertices.append(Vertex(lat=item.x / 1e7, lon=item.y / 1e7))

        ack = _pull_until(
            q, lambda m: m.mission_type == mission_type and m.get_type() == "MISSION_ACK", deadline
        )
        if ack is None:
            raise FenceError("Timeout waiting for MISSION_ACK after fence download")
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise FenceError(f"Fence download rejected by FC: type={ack.type}")

        return vertices
    finally:
        bus.unsubscribe(q)


# ---------------------------------------------------------------------------
# Fence clear
# ---------------------------------------------------------------------------

def clear_fence(bus: MavlinkBus, timeout_s: float) -> None:
    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    deadline = time.time() + timeout_s
    q = _subscribe_then_send(
        bus,
        ["MISSION_ACK"],
        bus.conn.mav.mission_clear_all_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mission_type=mission_type,
    )
    try:
        ack = _pull_until(q, lambda m: m.mission_type == mission_type, deadline)
    finally:
        bus.unsubscribe(q)
    if ack is None:
        raise FenceError("Timeout waiting for MISSION_ACK on fence clear")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise FenceError(f"Fence clear rejected by FC: type={ack.type}")


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

def send_statustext(bus: MavlinkBus, text: str, severity: int = 6) -> None:
    """
    Send a STATUSTEXT message — this is what actually shows up in Mission
    Planner's message log, distinct from telemetry.py's push_event which
    only reaches our own browser UI over its own WebSocket. Confirmed gap:
    this function didn't previously exist anywhere in the codebase, and
    TRANSMIT_RESULT only ever called push_event — the decoded QR result
    never reached Mission Planner at all.

    severity: MAV_SEVERITY_* — default 6 (INFO). Fire-and-forget, no
    ACK exists for STATUSTEXT in the MAVLink spec, so this doesn't wait
    for confirmation the way fence operations do.

    text is capped at 50 bytes per the MAVLink STATUSTEXT field width —
    truncated (not chunked) here since our actual messages ("QR:27") are
    always far under that limit. If a future message could exceed 50
    bytes, this needs the chunked STATUSTEXT variant (id/chunk_seq
    fields) instead of silent truncation.

    Signature verified against pymavlink's actual statustext_send:
    (severity, text, id=0, chunk_seq=0) — checked directly rather than
    assumed, since getting field order wrong here would silently send
    garbage to Mission Planner.
    """
    encoded = text.encode("utf-8")[:50]
    padded = encoded.ljust(50, b"\x00")
    bus.send(
        bus.conn.mav.statustext_send,
        severity,
        padded,
    )


def get_param(bus: MavlinkBus, param_id: str, timeout_s: float) -> float:
    """
    Read a single parameter value from the flight controller. Used by the
    Phase 2 SITL tests to read back FENCE_ENABLE/FENCE_ACTION/FENCE_TYPE
    after enable_fence() to confirm ArduPilot actually accepted them.
    """
    deadline = time.time() + timeout_s
    q = _subscribe_then_send(
        bus,
        ["PARAM_VALUE"],
        bus.conn.mav.param_request_read_send,
        bus.conn.target_system,
        bus.conn.target_component,
        param_id.encode("ascii"),
        -1,  # param_index=-1 means "look up by name"
    )
    try:
        msg = _pull_until(q, lambda m: m.param_id.rstrip("\x00") == param_id, deadline)
    finally:
        bus.unsubscribe(q)
    if msg is None:
        raise FenceError(f"Timeout waiting for PARAM_VALUE for {param_id}")
    return float(msg.param_value)


def set_param(bus: MavlinkBus, param_id: str, value: float, timeout_s: float) -> None:
    """Set a parameter on the flight controller and wait for confirmation."""
    deadline = time.time() + timeout_s
    q = _subscribe_then_send(
        bus,
        ["PARAM_VALUE"],
        bus.conn.mav.param_set_send,
        bus.conn.target_system,
        bus.conn.target_component,
        param_id.encode("ascii"),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    try:
        msg = _pull_until(q, lambda m: m.param_id.rstrip("\x00") == param_id, deadline)
    finally:
        bus.unsubscribe(q)
    if msg is None:
        raise FenceError(f"Timeout waiting for PARAM_VALUE confirmation for {param_id}")
    if abs(msg.param_value - value) > 0.01:
        raise FenceError(f"Param {param_id} set to {msg.param_value}, expected {value}")


# FENCE_TYPE bitmask value for a polygon-inclusion fence.
# Verified against ArduPilot source: AP_Fence/AP_Fence.h
#   AC_FENCE_TYPE_ALT_MAX = 1
#   AC_FENCE_TYPE_CIRCLE  = 2
#   AC_FENCE_TYPE_POLYGON = 4  <- this one
#   AC_FENCE_TYPE_ALT_MIN = 8
# NEED TEST: confirm this value against the exact firmware version in use by
# running test_fence_sitl.py and checking the printed FENCE_TYPE readback.
_FENCE_TYPE_POLYGON_BIT = 4


def enable_fence(bus: MavlinkBus, fence_action: int, timeout_s: float) -> None:
    """
    Enable fence: FENCE_ENABLE=1, FENCE_ACTION=fence_action, and OR in the
    polygon bit on FENCE_TYPE (safe no-op if ArduPilot already inferred it —
    see test_fence_sitl.py's NEED TEST note on firmware-version behavior).
    """
    set_param(bus, "FENCE_ENABLE", 1, timeout_s=timeout_s)
    set_param(bus, "FENCE_ACTION", fence_action, timeout_s=timeout_s)

    current_fence_type = get_param(bus, "FENCE_TYPE", timeout_s=timeout_s)
    desired_fence_type = int(current_fence_type) | _FENCE_TYPE_POLYGON_BIT
    if int(current_fence_type) != desired_fence_type:
        set_param(bus, "FENCE_TYPE", float(desired_fence_type), timeout_s=timeout_s)


def build_geofence_status(
    bus: MavlinkBus,
    vertices: list[Vertex],
    origin_lat: float,
    origin_lon: float,
    fence_action: int,
    timeout_s: float,
    fc_readback_matched: bool = True,
) -> GeofenceStatus:
    """Convert uploaded vertices to ENU, compute centroid and max radius."""
    vertices_enu = [latlon_to_enu(v.lat, v.lon, origin_lat, origin_lon) for v in vertices]
    centroid = polygon_centroid(vertices_enu)
    max_radius = max_radius_from_point(vertices_enu, centroid)
    vertices_enu_models = [ENUPoint(x=p.x, y=p.y) for p in vertices_enu]

    return GeofenceStatus(
        loaded=True,
        armable=True,
        reason="Fence confirmed on flight controller",
        vertex_count=len(vertices),
        vertices_latlon=vertices,
        vertices_enu=vertices_enu_models,
        centroid_enu=ENUPoint(x=centroid.x, y=centroid.y),
        max_radius_m=max_radius,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        fc_readback_matched=fc_readback_matched,
    )
