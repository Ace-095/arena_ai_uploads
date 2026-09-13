"""
Telemetry WebSocket backbone (Phase 3).

Provides real-time position updates (2-5 Hz), stats (1 Hz), and event messages via WebSocket.
Single connection owner: MavlinkBus handles all MAVLink I/O.

Message Envelope Schema:
- {"channel": "position", "t": timestamp, "data": {...}}
- {"channel": "stats",    "t": timestamp, "data": {...}}
- {"channel": "event",    "t": timestamp, "data": {"type": "...", "value": "..."}}
"""
import asyncio
import concurrent.futures
import json
import logging
import queue
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, Dict, Optional, Set

from fastapi import WebSocket, WebSocketDisconnect
from pymavlink import mavutil

from mavlink_bus import MavlinkBus
from models import Position
import system_monitor
import network_monitor

log = logging.getLogger("telemetry")


@dataclass
class TelemetrySnapshot:
    """Legacy container structure retained for compatibility."""
    timestamp: float
    position: Optional[Position] = None
    attitude: Optional[dict] = None
    velocity: Optional[dict] = None
    gps: Optional[dict] = None
    battery: Optional[dict] = None
    mode: Optional[str] = None
    armed: Optional[bool] = None


class TelemetryHub:
    """
    Manages WebSocket clients and broadcasts channel envelope messages.
    Single producer (MAVLink reader task), multiple consumers (WS clients).
    Supports push_event for immediate event-driven messages.
    """

    def __init__(self, bus: MavlinkBus, position_hz: float = 5.0, stats_hz: float = 1.0, system_hz: float = 0.2):
        self._bus = bus
        self._position_interval = 1.0 / position_hz
        self._stats_interval = 1.0 / stats_hz
        # System/network stats are local reads, not MAVLink-driven, and
        # don't need to be fast — default 0.2Hz = every 5s.
        self._system_interval = 1.0 / system_hz
        # Dedicated executor for system_monitor/network_monitor's blocking
        # calls (file reads, a subprocess call, a socket connect with up
        # to a couple seconds' timeout). NOT the shared default executor —
        # asyncio.Task.cancel() cannot actually stop a thread that's
        # already blocked inside one of these calls (a real, documented
        # Python limitation, confirmed by reproduction: cancelling left a
        # background thread still blocked in socket.create_connection()
        # when the process tried to exit, causing a native
        # "terminate called without an active exception" crash during
        # interpreter shutdown). shutdown(cancel_futures=True) below is
        # the actual fix — it abandons pending work without blocking,
        # rather than hoping cancellation propagates into the thread.
        self._system_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="telemetry-sysmon"
        )
        self._clients: Set[WebSocket] = set()
        self._lock = asyncio.Lock()
        self._producer_task: Optional[asyncio.Task] = None
        self._system_task: Optional[asyncio.Task] = None
        self._latest_messages: Dict[str, dict] = {}
        self._subscriber_queues: list = []
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._clients.add(ws)
            for ch, msg in self._latest_messages.items():
                await self._safe_send_envelope(ws, msg)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)

    async def _safe_send_envelope(self, ws: WebSocket, msg: dict) -> bool:
        try:
            await ws.send_text(json.dumps(msg, separators=(",", ":")))
            return True
        except Exception as e:
            log.warning("Telemetry send_text error: %s", e)
            return False

    async def broadcast_envelope(self, channel: str, data: dict, timestamp: Optional[float] = None) -> None:
        if timestamp is None:
            timestamp = time.time()
        msg = {
            "channel": channel,
            "t": timestamp,
            "data": data,
        }
        self._latest_messages[channel] = msg
        async with self._lock:
            if not self._clients:
                return
            dead = []
            for ws in self._clients:
                if not await self._safe_send_envelope(ws, msg):
                    dead.append(ws)
            for ws in dead:
                self._clients.discard(ws)

    def push_event(self, event_type: str, value: Any) -> None:
        """Immediately broadcast an event envelope to all connected WebSocket clients."""
        now = time.time()
        data = {"type": event_type, "value": value}
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self.broadcast_envelope("event", data, now))
        except RuntimeError:
            if self._loop and self._loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_envelope("event", data, now),
                    self._loop
                )

    def start(self) -> None:
        if self._producer_task is not None:
            return

        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            self._loop = None

        pos_q = self._bus.subscribe([
            "GLOBAL_POSITION_INT",
            "ATTITUDE",
            "SYS_STATUS",
            "HEARTBEAT",
            "GPS_RAW_INT",
        ])
        self._subscriber_queues.append(pos_q)

        self._producer_task = asyncio.create_task(self._producer_loop(pos_q))
        self._system_task = asyncio.create_task(self._system_loop())
        log.info("TelemetryHub producer started (pos=%.1fHz, stats=%.1fHz, system=%.2fHz)",
                 1.0 / self._position_interval, 1.0 / self._stats_interval, 1.0 / self._system_interval)

    def stop(self) -> None:
        if self._producer_task is not None:
            self._producer_task.cancel()
            self._producer_task = None
        if self._system_task is not None:
            self._system_task.cancel()
            self._system_task = None
        # cancel_futures=True (Python 3.9+) drops any QUEUED work
        # immediately. wait=True blocks for any CURRENTLY-RUNNING call to
        # actually finish first — deliberately not wait=False here:
        # abandoning a thread mid-syscall (e.g. still holding an open
        # socket) while the interpreter starts finalizing modules is what
        # was causing an intermittent native "terminate called without an
        # active exception" crash, confirmed by reproduction (~1/3 runs
        # crashed with wait=False, 0/10 with wait=True — see commit
        # message for the actual repro numbers). The internal timeouts in
        # network_monitor/system_monitor are short (~1-2s) specifically so
        # this wait has a small, bounded worst case.
        self._system_executor.shutdown(wait=True, cancel_futures=True)
        for q in self._subscriber_queues:
            self._bus.unsubscribe(q)
        self._subscriber_queues.clear()
        log.info("TelemetryHub stopped")

    async def _system_loop(self) -> None:
        """
        Periodically broadcasts Pi system health (CPU/RAM/disk/temp) and
        internet connectivity status — confirmed gaps against the
        original project notes ("System Info" and "Internet status" were
        both specified UI sections with nothing implemented for either).

        Both system_monitor.get_system_stats() and
        network_monitor.check_internet_status() are blocking calls (file
        reads, a subprocess call, a socket connect with up to a few
        seconds' timeout) — run in the default executor so they don't
        stall the asyncio event loop that's also handling WebSocket
        traffic and MAVLink message dispatch.
        """
        loop = asyncio.get_running_loop()
        while True:
            try:
                stats = await loop.run_in_executor(self._system_executor, system_monitor.get_system_stats)
                internet_up = await loop.run_in_executor(self._system_executor, network_monitor.check_internet_status)
                await self.broadcast_envelope(
                    "system",
                    {**stats.model_dump(), "internet_status": "up" if internet_up else "down"},
                )
            except asyncio.CancelledError:
                break
            except Exception as exc:
                log.warning("System stats collection failed: %s", exc)
            await asyncio.sleep(self._system_interval)

    async def _producer_loop(self, q) -> None:
        self._loop = asyncio.get_running_loop()
        last_pos_broadcast = 0.0
        last_stats_broadcast = 0.0
        position_data = {}
        attitude_data = {}
        sys_status_data = {}
        heartbeat_data = {}
        gps_data = {}

        while True:
            try:
                msg = await asyncio.wait_for(
                    asyncio.get_running_loop().run_in_executor(None, q.get, True, 0.1),
                    timeout=0.2
                )
            except (asyncio.TimeoutError, queue.Empty):
                now = time.time()
                last_pos_broadcast, last_stats_broadcast = await self._maybe_broadcast(
                    now, last_pos_broadcast, last_stats_broadcast,
                    position_data, attitude_data, sys_status_data,
                    heartbeat_data, gps_data
                )
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                log.debug("Telemetry producer queue error: %s", e)
                continue

            msg_type = msg.get_type()
            now = time.time()

            if msg_type == "GLOBAL_POSITION_INT":
                position_data = {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt_msl": msg.alt / 1000.0,
                    "alt_rel": msg.relative_alt / 1000.0,
                    "vx": msg.vx / 100.0,
                    "vy": msg.vy / 100.0,
                    "vz": msg.vz / 100.0,
                    "hdg": msg.hdg / 100.0 if msg.hdg != 65535 else None,
                }
            elif msg_type == "ATTITUDE":
                attitude_data = {
                    "roll": msg.roll * 180 / 3.14159265359,
                    "pitch": msg.pitch * 180 / 3.14159265359,
                    "yaw": msg.yaw * 180 / 3.14159265359,
                }
            elif msg_type == "SYS_STATUS":
                sys_status_data = {
                    "voltage_battery": msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else None,
                    "current_battery": msg.current_battery / 100.0 if msg.current_battery > 0 else None,
                    "battery_remaining": msg.battery_remaining if msg.battery_remaining >= 0 else None,
                }
            elif msg_type == "HEARTBEAT":
                heartbeat_data = {
                    "mode": mavutil.mode_string_v10(msg),
                    "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                }
            elif msg_type == "GPS_RAW_INT":
                gps_data = {
                    "fix_type": msg.fix_type,
                    "satellites_visible": msg.satellites_visible,
                    "hdop": msg.eph / 100.0 if msg.eph != 65535 else None,
                }

            last_pos_broadcast, last_stats_broadcast = await self._maybe_broadcast(
                now, last_pos_broadcast, last_stats_broadcast,
                position_data, attitude_data, sys_status_data,
                heartbeat_data, gps_data
            )

    async def _maybe_broadcast(self, now: float, last_pos: float, last_stats: float,
                               pos: dict, att: dict, sys: dict, hb: dict, gps: dict) -> tuple[float, float]:
        if now - last_pos >= (self._position_interval - 0.01) and pos:
            pos_payload = {
                "position": {
                    "lat": pos.get("lat", 0),
                    "lon": pos.get("lon", 0),
                    "alt_msl": pos.get("alt_msl"),
                    "alt_rel": pos.get("alt_rel"),
                },
                "velocity": {"vx": pos.get("vx"), "vy": pos.get("vy"), "vz": pos.get("vz")} if pos.get("vx") is not None else None,
                "attitude": att if att else None,
                "gps": gps if gps else None,
                "mode": hb.get("mode"),
                "armed": hb.get("armed"),
            }
            await self.broadcast_envelope("position", pos_payload, now)
            last_pos = now

        if now - last_stats >= (self._stats_interval - 0.01) and (sys or hb or gps):
            stats_payload = {
                "battery": sys if sys else None,
                "mode": hb.get("mode"),
                "armed": hb.get("armed"),
                "gps": gps if gps else None,
            }
            await self.broadcast_envelope("stats", stats_payload, now)
            last_stats = now

        return last_pos, last_stats


_telemetry_hub: Optional[TelemetryHub] = None


def get_telemetry_hub() -> TelemetryHub:
    if _telemetry_hub is None:
        raise RuntimeError("TelemetryHub not initialized")
    return _telemetry_hub


def init_telemetry_hub(bus: MavlinkBus, position_hz: float = 5.0, stats_hz: float = 1.0) -> TelemetryHub:
    global _telemetry_hub
    if _telemetry_hub is not None:
        return _telemetry_hub
    _telemetry_hub = TelemetryHub(bus, position_hz, stats_hz)
    _telemetry_hub.start()
    return _telemetry_hub


def shutdown_telemetry_hub() -> None:
    global _telemetry_hub
    if _telemetry_hub is not None:
        _telemetry_hub.stop()
        _telemetry_hub = None


def push_event(event_type: str, value: Any) -> None:
    """Global helper to push an event through the TelemetryHub if active."""
    if _telemetry_hub is not None:
        _telemetry_hub.push_event(event_type, value)


@asynccontextmanager
async def websocket_endpoint(ws: WebSocket):
    hub = get_telemetry_hub()
    await hub.connect(ws)
    try:
        yield
    except WebSocketDisconnect:
        pass
    finally:
        await hub.disconnect(ws)