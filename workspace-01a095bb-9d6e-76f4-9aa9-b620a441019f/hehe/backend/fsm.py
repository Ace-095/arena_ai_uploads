import logging
import threading
import time
from typing import Optional
import os

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

from config import config
from models import FSMStatus
import state
from telemetry import push_event
from search_algorithm import search_controller
from qr_pipeline import qr_pipeline_runner
import mavlink_fence as mf

log = logging.getLogger("fsm")


class MissionFSM:
    """
    Finite State Machine for autonomous mission execution.
    Handles fail-closed gating, MAVLink flight commands, telemetry events,
    and automatic failsafes.
    """

    def __init__(self, bus, camera_manager=None):
        self._bus = bus
        self._camera_manager = camera_manager
        self._status = FSMStatus(state="INIT")
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._state_start_time = 0.0
        self._last_heartbeat = 0.0
        self._last_battery_pct = 100.0
        self._last_alt_rel = 0.0
        self._motors_armed = False
        self._statustext_last_send_time = 0.0

        self._msg_queue = None

    def start(self) -> FSMStatus:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("FSM is already running")

            self._stop_event.clear()
            self._transition_to("PRE_FLIGHT_CHECK", "Mission started")
            
            self._msg_queue = self._bus.subscribe(["HEARTBEAT", "SYS_STATUS", "GLOBAL_POSITION_INT"])
            self._thread = threading.Thread(target=self._fsm_loop, name="mission-fsm", daemon=True)
            self._thread.start()
            return self._status.model_copy()

    def abort(self) -> FSMStatus:
        """
        Emergency manual abort -> FAILSAFE.

        INIT was previously in the exclusion list below, which meant
        calling abort() on a freshly-created FSM (before start() had ever
        been called) silently did nothing — confirmed by test failure:
        test_fsm_abort expected FAILSAFE and got INIT. An emergency abort
        should work regardless of whether the mission has technically
        begun yet.
        """
        with self._lock:
            if self._status.state not in ["FAILSAFE", "RTL", "LAND", "MISSION_COMPLETE"]:
                self._transition_to("FAILSAFE", "Manual abort triggered")
            return self._status.model_copy()

    def get_status(self) -> FSMStatus:
        with self._lock:
            # Update health flags for status
            fence_state = state.get()
            self._status.armable = fence_state.armable if fence_state else False
            self._status.telemetry_link_healthy = (time.time() - self._last_heartbeat) < config.FSM_LINK_LOSS_TIMEOUT_S
            return self._status.model_copy()

    def _transition_to(self, new_state: str, reason: str, payload: Optional[str] = None):
        if self._status.state == new_state:
            return

        old_state = self._status.state
        self._status.previous_state = old_state
        self._status.state = new_state
        self._status.reason = reason
        self._status.timestamp = time.time()
        if payload is not None:
            self._status.payload = payload
            
        self._state_start_time = time.time()

        log.info(f"FSM Transition: {old_state} -> {new_state} | Reason: {reason}")
        push_event("fsm_state", {
            "from": old_state,
            "to": new_state,
            "timestamp": self._status.timestamp,
            "reason": reason,
            "payload": payload
        })

    def _update_telemetry(self):
        """Process pending MAVLink messages to update internal state."""
        if not self._msg_queue:
            return
            
        while not self._msg_queue.empty():
            msg = self._msg_queue.get_nowait()
            msg_type = msg.get_type()
            
            if msg_type == "HEARTBEAT":
                self._last_heartbeat = time.time()
                self._motors_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == "SYS_STATUS":
                if msg.battery_remaining >= 0:
                    self._last_battery_pct = msg.battery_remaining
            elif msg_type == "GLOBAL_POSITION_INT":
                self._last_alt_rel = msg.relative_alt / 1000.0

    def _check_failsafe_conditions(self) -> bool:
        """Check if any condition warrants an immediate transition to FAILSAFE."""
        curr_state = self._status.state
        if curr_state in ["INIT", "PRE_FLIGHT_CHECK", "FAILSAFE", "RTL", "LAND", "MISSION_COMPLETE"]:
            return False

        # 1. Telemetry Link Loss
        if (time.time() - self._last_heartbeat) > config.FSM_LINK_LOSS_TIMEOUT_S:
            self._transition_to("FAILSAFE", "Telemetry link loss timeout")
            return True

        # 2. Geofence Unarmable
        fence_state = state.get()
        if not fence_state or not fence_state.armable:
            self._transition_to("FAILSAFE", "Geofence invalidated or unarmable")
            return True

        # 3. Low Battery
        if self._last_battery_pct < config.FSM_MIN_BATTERY_PCT:
            self._transition_to("FAILSAFE", f"Low battery: {self._last_battery_pct}%")
            return True

        return False

    def _switch_camera(self, mode: str, context: str) -> bool:
        """
        Switch the active camera and repoint qr_pipeline_runner's frame
        source to match, resetting consensus (a streak built up against
        one camera's view is meaningless once the view has changed).

        This was previously missing entirely: qr_pipeline_runner's source
        was set once at startup to cam1 and never changed, so the DECODE
        state was reading Camera 1 (the wide search camera) instead of
        Camera 2 (the motorized-focus decode camera) for the whole
        mission — confirmed by grepping for camera_manager/set_source
        calls outside of app.py's one-time startup wiring and finding
        none. Fail-closed: if the switch can't be done, this transitions
        to FAILSAFE rather than silently continuing on the wrong camera,
        since DECODE cannot succeed that way regardless.
        """
        if self._camera_manager is None:
            self._transition_to(
                "FAILSAFE", f"No camera_manager wired into FSM, cannot switch to {mode} for {context}"
            )
            return False
        try:
            self._camera_manager.set_active(mode)
            source = self._camera_manager.get_source(mode)
            if source is None:
                self._transition_to(
                    "FAILSAFE", f"Camera source '{mode}' not found, cannot proceed with {context}"
                )
                return False
            qr_pipeline_runner.set_source(source)
            qr_pipeline_runner.consensus.reset()
            log.info("Camera switched to '%s' for %s", mode, context)
            return True
        except Exception as e:
            self._transition_to("FAILSAFE", f"Camera switch to {mode} failed: {e}")
            return False

    def _send_command(self, command, *args):
        if config.FSM_DRY_RUN:
            log.info(f"[DRY-RUN] Sending MAVLink command {command} with args: {args}")
        else:
            self._bus.send(
                self._bus.conn.mav.command_long_send,
                self._bus.conn.target_system,
                self._bus.conn.target_component,
                command,
                0,
                *args
            )

    def _set_mode(self, mode_enum):
        if config.FSM_DRY_RUN:
            log.info(f"[DRY-RUN] Setting mode {mode_enum}")
        else:
            self._bus.send(
                self._bus.conn.mav.command_long_send,
                self._bus.conn.target_system,
                self._bus.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_enum,
                0, 0, 0, 0, 0
            )

    def _fsm_loop(self):
        while not self._stop_event.is_set():
            time.sleep(0.1)
            with self._lock:
                self._update_telemetry()
                
                if self._check_failsafe_conditions():
                    continue

                curr_state = self._status.state
                elapsed = time.time() - self._state_start_time

                if curr_state == "INIT":
                    pass

                elif curr_state == "PRE_FLIGHT_CHECK":
                    fence_state = state.get()
                    armable = fence_state.armable if fence_state else False
                    link_healthy = (time.time() - self._last_heartbeat) < config.FSM_LINK_LOSS_TIMEOUT_S

                    if armable and link_healthy:
                        self._transition_to("ARM", "Pre-flight checks passed")
                    elif elapsed > config.FSM_PREFLIGHT_TIMEOUT_S:
                        self._transition_to("FAILSAFE", "Pre-flight check timeout (Fence or Link unhealthy)")

                elif curr_state == "ARM":
                    if self._motors_armed or config.FSM_DRY_RUN:
                        self._transition_to("TAKEOFF", "Motors armed")
                    else:
                        if elapsed > 10.0:
                            self._transition_to("FAILSAFE", "Arming timeout")
                        elif int(elapsed * 10) % 20 == 0:  # Every 2 seconds
                            self._set_mode(4)  # GUIDED
                            self._send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)

                elif curr_state == "TAKEOFF":
                    if self._last_alt_rel >= (config.FSM_TAKEOFF_ALTITUDE_M - 0.5) or config.FSM_DRY_RUN:
                        self._transition_to("SEARCH", "Takeoff altitude reached")
                    else:
                        if elapsed > 30.0:
                            self._transition_to("FAILSAFE", "Takeoff timeout")
                        elif int(elapsed * 10) % 30 == 0:  # Every 3 seconds
                            self._send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, config.FSM_TAKEOFF_ALTITUDE_M)

                elif curr_state == "SEARCH":
                    if elapsed < 0.2:
                        if not self._switch_camera("cam1", "SEARCH"):
                            continue  # _switch_camera already transitioned to FAILSAFE
                        try:
                            search_controller.start_search(
                                dry_run=config.FSM_DRY_RUN,
                                altitude_m=config.FSM_TAKEOFF_ALTITUDE_M
                            )
                        except Exception as e:
                            self._transition_to("FAILSAFE", f"Search start failed: {e}")
                            
                    search_status = search_controller.get_status()
                    if search_status.state == "target_found":
                        self._transition_to("CACHE_DETECTED", "Target found by search algorithm")
                    elif search_status.state == "completed":
                        self._transition_to("FAILSAFE", "Search completed without detecting target")
                    elif elapsed > config.FSM_QR_SWEEP_TIMEOUT_S:
                        self._transition_to("FAILSAFE", "QR sweep timeout exceeded")

                elif curr_state == "CACHE_DETECTED":
                    # Switch to Camera 2 (motorized-focus decode camera)
                    # before advancing — DECODE must not read Camera 1.
                    if self._switch_camera("cam2", "APPROACH/DECODE"):
                        self._transition_to("APPROACH", "Initiating approach, switched to decode camera")

                elif curr_state == "APPROACH":
                    if elapsed < 0.2:
                        try:
                            search_controller.begin_approach_descent()
                        except Exception as e:
                            self._transition_to("FAILSAFE", f"Approach descent failed to start: {e}")
                            continue

                    approach_status = search_controller.get_status()
                    if approach_status.state == "decoded":
                        # Decode already succeeded mid-descent — no need
                        # for a separate DECODE phase to re-check the
                        # same consensus buffer at the same altitude.
                        qr_status = qr_pipeline_runner.consensus.get_status()
                        self._transition_to(
                            "CONFIRMED",
                            f"Consensus reached during descent at {approach_status.altitude_m:.1f}m: {qr_status.payload}",
                            payload=qr_status.payload,
                        )
                    elif approach_status.state == "error":
                        # Reached the safety floor (or descent-level
                        # timeout) without a confirmed decode — hand off
                        # to DECODE for a final attempt at the current
                        # (lowest reached) altitude, rather than giving
                        # up immediately.
                        self._transition_to(
                            "DECODE", f"Descent ended without decode ({approach_status.reason}); trying at current altitude"
                        )
                    # else still "approaching" — keep polling, no transition yet

                elif curr_state == "DECODE":
                    qr_status = qr_pipeline_runner.consensus.get_status()
                    if qr_status.confirmed and qr_status.payload:
                        self._transition_to("CONFIRMED", f"Consensus reached: {qr_status.payload}", payload=qr_status.payload)
                    elif elapsed > 60.0:
                        self._transition_to("FAILSAFE", "Decode timeout exceeded")

                elif curr_state == "CONFIRMED":
                    self._statustext_last_send_time = 0.0  # fresh window for the upcoming TRANSMIT_RESULT entry
                    self._transition_to("TRANSMIT_RESULT", "Payload confirmed")

                elif curr_state == "TRANSMIT_RESULT":
                    resend_due = (time.time() - self._statustext_last_send_time) >= config.STATUSTEXT_RESEND_INTERVAL_S
                    if resend_due and elapsed < config.STATUSTEXT_RESEND_WINDOW_S:
                        if self._statustext_last_send_time == 0.0:
                            # First send since entering this state — UI
                            # gets this once; it doesn't need resending
                            # the way STATUSTEXT does, since a
                            # reconnecting browser is replayed the latest
                            # cached message per channel automatically.
                            push_event("mission_result", {"payload": self._status.payload, "timestamp": time.time()})
                        try:
                            mf.send_statustext(self._bus, f"QR:{self._status.payload}", severity=6)
                            self._statustext_last_send_time = time.time()
                            log.info(
                                "Sent result to Mission Planner via STATUSTEXT (elapsed=%.1fs into %.0fs window): QR:%s",
                                elapsed, config.STATUSTEXT_RESEND_WINDOW_S, self._status.payload,
                            )
                        except Exception as e:
                            # Don't block the mission on this — the UI
                            # already has the result via push_event
                            # above. But this must not fail silently.
                            log.error("Failed to send result to Mission Planner: %s", e)

                    if elapsed >= config.STATUSTEXT_RESEND_WINDOW_S:
                        self._transition_to("RTL", "Result transmitted, returning to launch")

                elif curr_state == "FAILSAFE":
                    search_controller.stop_search("Failsafe triggered")
                    self._transition_to("RTL", "Failsafe action -> RTL")

                elif curr_state == "RTL":
                    if (not self._motors_armed and elapsed > 5.0) or config.FSM_DRY_RUN:
                        self._transition_to("LAND", "RTL complete, preparing to land")
                    else:
                        if int(elapsed * 10) % 30 == 0:  # Every 3 seconds
                            self._set_mode(6)  # RTL
                            self._send_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0)
                            
                        # If altitude drops significantly, we might be landing
                        if self._last_alt_rel < 1.0 and elapsed > 10.0:
                             self._transition_to("LAND", "Altitude low, entering LAND mode")

                elif curr_state == "LAND":
                    if not self._motors_armed or config.FSM_DRY_RUN:
                        self._transition_to("MISSION_COMPLETE", "Landed successfully")
                    else:
                        if int(elapsed * 10) % 30 == 0:
                             self._set_mode(9) # LAND mode
                             self._send_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)

                elif curr_state == "MISSION_COMPLETE":
                    if self._msg_queue:
                        self._bus.unsubscribe(self._msg_queue)
                        self._msg_queue = None
                    self._stop_event.set()


_mission_fsm: Optional[MissionFSM] = None

def init_fsm(bus, camera_manager=None) -> MissionFSM:
    global _mission_fsm
    if _mission_fsm is None:
        _mission_fsm = MissionFSM(bus, camera_manager=camera_manager)
    return _mission_fsm

def get_fsm() -> MissionFSM:
    if _mission_fsm is None:
        raise RuntimeError("MissionFSM not initialized")
    return _mission_fsm
