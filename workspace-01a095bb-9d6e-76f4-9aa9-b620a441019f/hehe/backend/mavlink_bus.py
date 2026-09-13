"""
MAVLink Bus — single connection, single reader thread, multiple subscribers.

This is the sole owner of the MAVLink connection to the Pixhawk. All MAVLink
communication in the system (fence ops, telemetry, guidance commands) goes
through this bus. No other module opens a connection or calls recv_match()
directly.

Threading model:
- One reader thread (_read_loop) calls conn.recv_match() in a loop.
- Subscribers register queues for message types (or '*' for all).
- Reader thread pushes a copy of each message to matching queues (non-blocking,
  drops oldest if queue full).
- wait_for() is a convenience: subscribe, pull until predicate matches or timeout.
- send() is a thin lock around the socket write — does NOT hold lock across
  wait_for() (would deadlock against the reader thread).

Exception classes defined here to avoid circular imports with mavlink_fence.py.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import queue
import threading
import time
from typing import Callable, Optional

from pymavlink import mavutil


class FenceError(Exception):
    """Any failure during a fence transaction (upload, download, param set, readback mismatch)."""
    pass


class ConnectionError(FenceError):
    """Connection-level failure: no heartbeat, socket error, timeout."""
    pass


class MavlinkBus:
    """
    Single MAVLink connection with pub/sub dispatch.

    Usage:
        bus = MavlinkBus(device="/dev/ttyACM0", baud=115200, timeout_s=5.0)
        bus.start()
        try:
            # For request/response patterns:
            msg = bus.wait_for(["GLOBAL_POSITION_INT"], lambda m: True, timeout_s=2.0)
            # For fire-and-forget commands:
            bus.send(bus.conn.mav.command_long_send, ...)
            # For continuous streams:
            q = bus.subscribe(["GLOBAL_POSITION_INT", "ATTITUDE"])
            while True:
                msg = q.get()
                ...
        finally:
            bus.stop()
    """

    def __init__(self, device: str, baud: int, timeout_s: float):
        self._device = device
        self._baud = baud
        self._timeout_s = timeout_s
        self._conn: Optional[mavutil.mavlink_connection] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._send_lock = threading.Lock()
        self._subscribers: dict[str, list[queue.Queue]] = {"*": []}
        self._sub_lock = threading.Lock()

    @property
    def conn(self):
        """Access to the underlying pymavlink connection for mav.*_send calls."""
        if self._conn is None:
            raise ConnectionError("MAVLinkBus not started")
        return self._conn

    def start(self) -> None:
        """Connect, send GCS heartbeat, wait for FC heartbeat, start reader thread."""
        if self._conn is not None:
            return  # already started

        # Establish connection
        try:
            self._conn = mavutil.mavlink_connection(self._device, baud=self._baud)
        except Exception as e:
            raise ConnectionError(f"Failed to open MAVLink connection to {self._device}: {e}")

        # Send our GCS heartbeat so FC knows we're here (required for some param/fence ops)
        self._conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
        )

        # Wait for heartbeat from flight controller (sets target_system and target_component)
        try:
            hb = self._conn.wait_heartbeat(timeout=self._timeout_s)
        except Exception:
            hb = None

        if hb is None:
            self._conn.close()
            self._conn = None
            raise ConnectionError(f"No heartbeat from flight controller on {self._device} within {self._timeout_s}s")

        # Start the single reader thread
        self._stop_event.clear()
        self._read_thread = threading.Thread(target=self._read_loop, name="mavlink-reader", daemon=True)
        self._read_thread.start()

    def stop(self) -> None:
        """Stop reader thread and close connection."""
        self._stop_event.set()
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def send(self, send_fn: Callable, *args, **kwargs) -> None:
        """
        Thread-safe send. Acquires send_lock only around the socket write.
        Does NOT hold lock across wait_for() — caller must not do that either.
        """
        with self._send_lock:
            if self._conn is None:
                raise ConnectionError("MAVLinkBus not started")
            send_fn(*args, **kwargs)

    def subscribe(self, msg_types: list[str]) -> queue.Queue:
        """
        Register interest in one or more MAVLink message type names.
        Returns a Queue that the reader thread will push matching messages onto.
        Caller must call unsubscribe(q) when done.
        """
        q: queue.Queue = queue.Queue(maxsize=50)
        with self._sub_lock:
            for mt in msg_types:
                self._subscribers.setdefault(mt, []).append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        """Remove a queue from all subscriber lists."""
        with self._sub_lock:
            for lst in self._subscribers.values():
                if q in lst:
                    lst.remove(q)

    def wait_for(self, msg_types: list[str], predicate: Callable, timeout_s: float):
        """
        Convenience: subscribe, pull from queue until predicate(msg) is True or timeout,
        unsubscribe, return the matching msg or None.
        """
        q = self.subscribe(msg_types)
        try:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                try:
                    msg = q.get(timeout=max(0.01, deadline - time.time()))
                except queue.Empty:
                    continue
                if predicate(msg):
                    return msg
            return None
        finally:
            self.unsubscribe(q)

    def _read_loop(self) -> None:
        """
        The ONLY place recv_match is called anywhere in this codebase.
        Runs in its own thread. For each message received, look up subscribers
        for msg.get_type() AND subscribers registered for '*' (catch-all),
        push a copy to each matching queue (non-blocking put — if a queue is
        full, drop oldest, don't block the reader on a slow consumer).
        """
        while not self._stop_event.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception:
                # Connection error or timeout — if stop_event not set, keep trying
                if not self._stop_event.is_set():
                    time.sleep(0.1)
                continue

            if msg is None:
                continue

            msg_type = msg.get_type()

            # Find matching subscriber queues
            with self._sub_lock:
                targets = []
                if msg_type in self._subscribers:
                    targets.extend(self._subscribers[msg_type])
                if "*" in self._subscribers:
                    targets.extend(self._subscribers["*"])

            # Push to each queue (non-blocking, drop oldest if full)
            for q in targets:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    try:
                        q.get_nowait()  # drop oldest
                    except queue.Empty:
                        pass
                    try:
                        q.put_nowait(msg)
                    except queue.Full:
                        pass  # give up on this queue