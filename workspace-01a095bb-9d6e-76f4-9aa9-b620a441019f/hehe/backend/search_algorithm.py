"""
Search algorithm on geofence-derived parameters (Phase 6).

Reads fence centroid_enu / max_radius_m / origin from state.py (Phase 2).
Generates search sweeps (ExpandingSquare or Lawnmower strategy) scaled to max_radius_m.
Checks every generated waypoint against geo_utils.point_in_polygon before sending.
Sends position targets via MavlinkBus using SET_POSITION_TARGET_LOCAL_NED.
Exposes on_detection(bbox, confidence) interface for Phase 7 target detection integration.
Supports --dry-run CLI and programmatic mode for standalone testing.
"""

import argparse
import logging
import math
import os
import queue
import sys
import time
import threading
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

from config import config
from geo_utils import ENU, enu_to_latlon, latlon_to_enu, point_in_polygon
from models import ENUPoint, GeofenceStatus, SearchStartRequest, SearchStatus, SearchWaypoint
import state
from telemetry import push_event

log = logging.getLogger("search_algorithm")


class SearchStrategy(ABC):
    """Abstract interface for search sweep plan generation."""

    @abstractmethod
    def generate_waypoints(
        self,
        centroid: ENUPoint,
        max_radius_m: float,
        polygon_enu: List[ENUPoint],
        step_m: float,
        altitude_m: float,
        origin_lat: float,
        origin_lon: float,
    ) -> List[SearchWaypoint]:
        """Generate a sequence of waypoints bounded by polygon_enu and max_radius_m."""
        pass


class ExpandingSquareStrategy(SearchStrategy):
    """
    Generates an expanding square spiral pattern starting from the centroid.
    Each candidate point is checked against the geofence polygon and max radius.
    """

    def generate_waypoints(
        self,
        centroid: ENUPoint,
        max_radius_m: float,
        polygon_enu: List[ENUPoint],
        step_m: float,
        altitude_m: float,
        origin_lat: float,
        origin_lon: float,
    ) -> List[SearchWaypoint]:
        polygon_nodes = [ENU(p.x, p.y) for p in polygon_enu]
        waypoints: List[SearchWaypoint] = []
        idx = 0

        # Start at centroid if inside polygon
        c_enu = ENU(centroid.x, centroid.y)
        if point_in_polygon(c_enu, polygon_nodes):
            lat, lon = enu_to_latlon(c_enu.x, c_enu.y, origin_lat, origin_lon)
            waypoints.append(
                SearchWaypoint(
                    index=idx,
                    enu=ENUPoint(x=c_enu.x, y=c_enu.y),
                    lat=lat,
                    lon=lon,
                    altitude_m=altitude_m,
                )
            )
            idx += 1

        # Expanding spiral: move right, up, left, left, down, down, right, right, right, etc.
        dx = [step_m, 0.0, -step_m, 0.0]
        dy = [0.0, step_m, 0.0, -step_m]
        dir_idx = 0
        leg_length = 1
        curr_x, curr_y = centroid.x, centroid.y
        max_legs = int(math.ceil((max_radius_m * 2.5) / max(step_m, 0.5)))

        for leg in range(1, max_legs + 1):
            for _ in range(2):  # Two legs of same length per expansion step
                for _ in range(leg_length):
                    curr_x += dx[dir_idx]
                    curr_y += dy[dir_idx]

                    dist_from_centroid = math.hypot(curr_x - centroid.x, curr_y - centroid.y)
                    if dist_from_centroid > max_radius_m + step_m:
                        continue

                    pt = ENU(curr_x, curr_y)
                    # Software-side second check: point must be strictly inside polygon
                    if point_in_polygon(pt, polygon_nodes):
                        lat, lon = enu_to_latlon(curr_x, curr_y, origin_lat, origin_lon)
                        waypoints.append(
                            SearchWaypoint(
                                index=idx,
                                enu=ENUPoint(x=curr_x, y=curr_y),
                                lat=lat,
                                lon=lon,
                                altitude_m=altitude_m,
                            )
                        )
                        idx += 1

                dir_idx = (dir_idx + 1) % 4
            leg_length += 1

        return waypoints


class LawnmowerStrategy(SearchStrategy):
    """
    Generates a back-and-forth lawnmower (boustrophedon) sweep across polygon bounds.
    """

    def generate_waypoints(
        self,
        centroid: ENUPoint,
        max_radius_m: float,
        polygon_enu: List[ENUPoint],
        step_m: float,
        altitude_m: float,
        origin_lat: float,
        origin_lon: float,
    ) -> List[SearchWaypoint]:
        polygon_nodes = [ENU(p.x, p.y) for p in polygon_enu]
        min_x = min(p.x for p in polygon_enu)
        max_x = max(p.x for p in polygon_enu)
        min_y = min(p.y for p in polygon_enu)
        max_y = max(p.y for p in polygon_enu)

        waypoints: List[SearchWaypoint] = []
        idx = 0
        y = min_y + step_m / 2.0
        direction = 1  # 1: left to right, -1: right to left

        while y <= max_y:
            x_start = min_x if direction == 1 else max_x
            x_end = max_x if direction == 1 else min_x
            dx = step_m if direction == 1 else -step_m

            x = x_start
            while (direction == 1 and x <= x_end) or (direction == -1 and x >= x_end):
                dist = math.hypot(x - centroid.x, y - centroid.y)
                if dist <= max_radius_m:
                    pt = ENU(x, y)
                    if point_in_polygon(pt, polygon_nodes):
                        lat, lon = enu_to_latlon(x, y, origin_lat, origin_lon)
                        waypoints.append(
                            SearchWaypoint(
                                index=idx,
                                enu=ENUPoint(x=x, y=y),
                                lat=lat,
                                lon=lon,
                                altitude_m=altitude_m,
                            )
                        )
                        idx += 1
                x += dx

            y += step_m
            direction *= -1

        return waypoints


def get_strategy(strategy_name: str) -> SearchStrategy:
    name = strategy_name.lower().strip()
    if name in ("lawnmower", "grid"):
        return LawnmowerStrategy()
    return ExpandingSquareStrategy()


class SearchController:
    """
    Manages autonomous search execution over MavlinkBus.
    Reads geofence state from state.get() (never recomputed).
    Sends SET_POSITION_TARGET_LOCAL_NED guidance targets to Pixhawk.
    """

    def __init__(self, bus=None):
        self._bus = bus
        self._lock = threading.Lock()
        self._status = SearchStatus()
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def set_bus(self, bus) -> None:
        self._bus = bus

    def get_status(self) -> SearchStatus:
        with self._lock:
            return self._status.model_copy()

    def generate_plan(
        self,
        strategy_name: Optional[str] = None,
        altitude_m: Optional[float] = None,
        step_m: Optional[float] = None,
    ) -> Tuple[GeofenceStatus, List[SearchWaypoint]]:
        """
        Fetch fence state and generate waypoint sequence.
        Fails closed if fence is missing or unarmable.
        """
        fence = state.get()
        if not fence.loaded or not fence.armable:
            raise RuntimeError(f"Cannot generate search plan: {fence.reason}")

        if fence.centroid_enu is None or fence.max_radius_m is None:
            raise RuntimeError("Fence state missing centroid or max_radius_m")

        if fence.origin_lat is None or fence.origin_lon is None:
            raise RuntimeError("Fence state missing EKF origin")

        strat_name = strategy_name or config.SEARCH_STRATEGY
        alt_m = altitude_m if altitude_m is not None else config.SEARCH_ALTITUDE_M
        step = step_m if step_m is not None else config.SEARCH_STEP_M

        strat = get_strategy(strat_name)
        waypoints = strat.generate_waypoints(
            centroid=fence.centroid_enu,
            max_radius_m=fence.max_radius_m,
            polygon_enu=fence.vertices_enu,
            step_m=step,
            altitude_m=alt_m,
            origin_lat=fence.origin_lat,
            origin_lon=fence.origin_lon,
        )

        if not waypoints:
            raise RuntimeError("Generated search plan has 0 valid waypoints inside geofence")

        return fence, waypoints

    def run_dry_run(
        self,
        strategy_name: Optional[str] = None,
        altitude_m: Optional[float] = None,
        step_m: Optional[float] = None,
    ) -> List[SearchWaypoint]:
        """Compute and log full waypoint sequence without sending MAVLink commands."""
        fence, waypoints = self.generate_plan(strategy_name, altitude_m, step_m)

        log.info(
            "DRY-RUN Search Plan generated: %d waypoints (Strategy: %s, Altitude: %.1fm, Radius: %.1fm)",
            len(waypoints),
            strategy_name or config.SEARCH_STRATEGY,
            altitude_m if altitude_m is not None else config.SEARCH_ALTITUDE_M,
            fence.max_radius_m,
        )
        for wp in waypoints:
            log.debug(
                "  WP #%d: ENU=(%.2f, %.2f) LatLon=(%.6f, %.6f) Alt=%.1fm",
                wp.index,
                wp.enu.x,
                wp.enu.y,
                wp.lat,
                wp.lon,
                wp.altitude_m,
            )

        with self._lock:
            self._status = SearchStatus(
                state="idle",
                dry_run=True,
                strategy=strategy_name or config.SEARCH_STRATEGY,
                altitude_m=altitude_m if altitude_m is not None else config.SEARCH_ALTITUDE_M,
                current_waypoint_idx=0,
                total_waypoints=len(waypoints),
                reason="Dry-run completed successfully",
                waypoints=waypoints,
            )
        return waypoints

    def start_search(
        self,
        strategy_name: Optional[str] = None,
        dry_run: bool = False,
        altitude_m: Optional[float] = None,
        step_m: Optional[float] = None,
    ) -> SearchStatus:
        """Start search execution (or dry-run). Fails closed if not armable."""
        if dry_run:
            self.run_dry_run(strategy_name, altitude_m, step_m)
            with self._lock:
                return self._status.model_copy()

        with self._lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                raise RuntimeError("Search algorithm is already running")

            if self._bus is None:
                raise RuntimeError("MavlinkBus not attached to SearchController")

        fence, waypoints = self.generate_plan(strategy_name, altitude_m, step_m)

        with self._lock:
            self._stop_event.clear()
            self._status = SearchStatus(
                state="searching",
                dry_run=False,
                strategy=strategy_name or config.SEARCH_STRATEGY,
                altitude_m=altitude_m if altitude_m is not None else config.SEARCH_ALTITUDE_M,
                current_waypoint_idx=0,
                total_waypoints=len(waypoints),
                reason="Search sweep active",
                waypoints=waypoints,
            )
            self._worker_thread = threading.Thread(
                target=self._search_loop, args=(waypoints,), name="search-loop", daemon=True
            )
            self._worker_thread.start()
            return self._status.model_copy()

    def stop_search(self, reason: str = "Search stopped by user") -> SearchStatus:
        """Stop active search execution and attempt position hold."""
        self._stop_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
            self._worker_thread = None

        with self._lock:
            if self._status.state in ("searching", "holding"):
                self._status.state = "idle"
                self._status.reason = reason
            return self._status.model_copy()

    def on_detection(self, bbox: dict, confidence: float) -> None:
        """
        Decoupled interface for Phase 7 target detection.
        Pauses sweep, records target metadata, and triggers telemetry event.
        """
        log.info("TARGET DETECTED in search algorithm! Confidence=%.2f BBox=%s", confidence, bbox)
        now = time.time()
        target_info = {"bbox": bbox, "confidence": confidence, "timestamp": now}

        with self._lock:
            self._status.state = "target_found"
            self._status.target = target_info
            self._status.reason = f"Target detected with confidence {confidence:.2f}"

        # Broadcast WebSocket event envelope via TelemetryHub
        push_event("target_detected", target_info)

    def _send_position_target_ned(self, north: float, east: float, down: float) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED (Message #84) via MavlinkBus.
        Frame: MAV_FRAME_LOCAL_NED (1)
        type_mask: 0b0000111111111000 (0x0FF8) -> position active, ignore vel/accel/yaw
        """
        if self._bus is None:
            return

        type_mask = 0b0000111111111000  # Position x, y, z target active
        self._bus.send(
            self._bus.conn.mav.set_position_target_local_ned_send,
            0,  # time_boot_ms
            self._bus.conn.target_system,
            self._bus.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north,
            east,
            down,
            0.0,
            0.0,
            0.0,  # vx, vy, vz
            0.0,
            0.0,
            0.0,  # afx, afy, afz
            0.0,
            0.0,  # yaw, yaw_rate
        )

    def begin_approach_descent(self) -> None:
        """
        Automatic, closed-loop descent for the DECODE phase.

        Deliberately does NOT descend to any pre-decided fixed altitude.
        The required altitude depends on the real camera lens/QR-size
        combination, which varies enough (a 6mm lens needs ~9m for
        4px/module, a 16mm lens could manage from ~25m) that hardcoding
        a single number would just be a different guess wearing a config
        variable's clothes. Instead: hold current horizontal position,
        step down by APPROACH_DESCENT_STEP_M, hold and check live QR
        decode consensus (from qr_pipeline_runner, reading whichever
        camera the FSM has switched to on entering APPROACH — see
        fsm.py::_switch_camera) for APPROACH_STEP_HOLD_S, and stop the
        instant decode succeeds. Only descends further if it doesn't.
        Hard-stops at APPROACH_MIN_ALTITUDE_M regardless of decode
        status — that's a safety floor, not a target.

        Runs in its own thread so the FSM's polling loop (fsm.py) can
        watch get_status() without blocking on this.
        """
        if self._bus is None:
            raise RuntimeError("MavlinkBus not attached to SearchController")

        self._stop_event.clear()
        with self._lock:
            self._status.state = "approaching"
            self._status.reason = "Beginning automatic closed-loop descent"

        self._worker_thread = threading.Thread(
            target=self._approach_descent_loop, name="approach-descent", daemon=True
        )
        self._worker_thread.start()

    def _approach_descent_loop(self) -> None:
        # Imported here, not at module top, to avoid a circular import
        # (qr_pipeline.py takes a SearchController reference via
        # set_search_controller() rather than importing this module).
        from qr_pipeline import qr_pipeline_runner

        fence = state.get()
        if fence.origin_lat is None or fence.origin_lon is None:
            with self._lock:
                self._status.state = "error"
                self._status.reason = "Fence state missing EKF origin — cannot compute local NED frame for descent"
            return

        self._set_guided_mode()

        # Subscribe BEFORE we need it, same discipline as the fence
        # protocol fix — no response, however fast, gets missed.
        pos_queue = self._bus.subscribe(["GLOBAL_POSITION_INT"])
        try:
            pos_msg = None
            deadline = time.time() + 5.0
            while pos_msg is None and time.time() < deadline and not self._stop_event.is_set():
                try:
                    pos_msg = pos_queue.get(timeout=max(0.1, deadline - time.time()))
                except queue.Empty:
                    continue
            if pos_msg is None:
                with self._lock:
                    self._status.state = "error"
                    self._status.reason = "No position telemetry available to begin approach descent"
                return

            # Hold this horizontal position for the whole descent — we
            # don't navigate anywhere new here, SEARCH/on_detection
            # already got the vehicle over the target. We only change Z.
            current_lat = pos_msg.lat / 1e7
            current_lon = pos_msg.lon / 1e7
            current_alt_rel = pos_msg.relative_alt / 1000.0
            hold_point = latlon_to_enu(current_lat, current_lon, fence.origin_lat, fence.origin_lon)
            north, east = hold_point.y, hold_point.x

            target_alt = current_alt_rel
            step = config.APPROACH_DESCENT_STEP_M
            min_alt = config.APPROACH_MIN_ALTITUDE_M
            hold_s = config.APPROACH_STEP_HOLD_S
            overall_deadline = time.time() + config.APPROACH_MAX_DURATION_S

            log.info(
                "Approach descent starting: current_alt=%.1fm, step=%.1fm, "
                "min_alt=%.1fm, hold=%.1fs/step", current_alt_rel, step, min_alt, hold_s
            )

            while not self._stop_event.is_set() and time.time() < overall_deadline:
                down = -abs(target_alt)  # NED: down positive downward
                with self._lock:
                    self._status.state = "approaching"
                    self._status.altitude_m = target_alt
                    self._status.reason = f"Holding at {target_alt:.1f}m, checking decode"

                step_deadline = time.time() + hold_s
                while time.time() < step_deadline and not self._stop_event.is_set():
                    self._send_position_target_ned(north, east, down)

                    qr_status = qr_pipeline_runner.consensus.get_status()
                    if qr_status.confirmed:
                        with self._lock:
                            self._status.state = "decoded"
                            self._status.altitude_m = target_alt
                            self._status.reason = f"Decode succeeded at {target_alt:.1f}m"
                        log.info(
                            "QR decode succeeded at %.1fm — descent stopped here, "
                            "not at any pre-decided altitude.", target_alt
                        )
                        push_event("approach_decoded", {"altitude_m": target_alt, "payload": qr_status.payload})
                        return

                    time.sleep(0.5)  # 2Hz position-target stream rate

                if target_alt <= min_alt:
                    log.warning(
                        "Reached safety floor %.1fm without a confirmed decode.", min_alt
                    )
                    with self._lock:
                        self._status.state = "error"
                        self._status.reason = f"Reached safety floor {min_alt:.1f}m without decode"
                    return

                target_alt = max(min_alt, target_alt - step)

            with self._lock:
                self._status.state = "error"
                self._status.reason = "Approach descent timed out"
        finally:
            self._bus.unsubscribe(pos_queue)

    def _set_guided_mode(self) -> None:
        """Set Pixhawk mode to GUIDED via MAV_CMD_DO_SET_MODE."""
        if self._bus is None:
            return
        self._bus.send(
            self._bus.conn.mav.command_long_send,
            self._bus.conn.target_system,
            self._bus.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED mode for ArduCopter
            0,
            0,
            0,
            0,
            0,
        )

    def _search_loop(self, waypoints: List[SearchWaypoint]) -> None:
        """Background thread streaming position targets to MavlinkBus."""
        log.info("Starting MAVLink search sweep thread (%d waypoints)...", len(waypoints))
        self._set_guided_mode()

        for idx, wp in enumerate(waypoints):
            if self._stop_event.is_set():
                log.info("Search sweep interrupted.")
                break

            with self._lock:
                if self._status.state == "target_found":
                    log.info("Search paused due to target detection.")
                    break
                self._status.current_waypoint_idx = idx

            # Convert ENU to local NED: North = Y, East = X, Down = -Altitude
            north = wp.enu.y
            east = wp.enu.x
            down = -abs(wp.altitude_m)

            log.info("Streaming target WP #%d: North=%.2fm, East=%.2fm, Down=%.2fm", idx, north, east, down)

            # Stream position target at 2Hz for ~3 seconds per waypoint leg
            leg_deadline = time.time() + 3.0
            while time.time() < leg_deadline and not self._stop_event.is_set():
                with self._lock:
                    if self._status.state == "target_found":
                        break
                self._send_position_target_ned(north, east, down)
                time.sleep(0.5)

        with self._lock:
            if not self._stop_event.is_set() and self._status.state == "searching":
                self._status.state = "completed"
                self._status.reason = "Search sweep completed"
        log.info("Search sweep thread finished.")


# Global instance
search_controller = SearchController()


def main():
    parser = argparse.ArgumentParser(description="Search algorithm runner")
    parser.add_argument("--dry-run", action="store_true", help="Compute and log waypoints without sending commands")
    parser.add_argument("--strategy", type=str, default="expanding_square", help="Search strategy (expanding_square | lawnmower)")
    parser.add_argument("--altitude", type=float, default=5.0, help="Search altitude in meters")
    parser.add_argument("--step", type=float, default=2.0, help="Step spacing in meters")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    if args.dry_run or True:  # CLI dry-run mode
        try:
            wps = search_controller.run_dry_run(
                strategy_name=args.strategy, altitude_m=args.altitude, step_m=args.step
            )
            print(f"Successfully generated {len(wps)} waypoints.")
            for wp in wps:
                print(f"  WP #{wp.index:02d}: ENU=({wp.enu.x:6.2f}, {wp.enu.y:6.2f})  LatLon=({wp.lat:.6f}, {wp.lon:.6f})")
        except Exception as e:
            print(f"Dry-run failed: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
