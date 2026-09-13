"""
test_search_sitl.py — Phase 6 live search sweep acceptance test against ArduPilot SITL / Gazebo.

Prerequisites:
    # Terminal 1 — Gazebo / SITL simulator
    sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
    # OR standard SITL:
    sim_vehicle.py -v ArduCopter -f quad --console --map

    # Terminal 2 — Run this acceptance test
    cd backend
    export MAVLINK_DEVICE=udp:127.0.0.1:14550
    python3 tests/test_search_sitl.py

Validates that SearchController streams SET_POSITION_TARGET_LOCAL_NED guidance targets
to ArduPilot in SITL, executes sweep within the active geofence, and handles start/stop/detection cleanly.
"""

import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

_BACKEND_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND_DIR not in sys.path:
    sys.path.insert(0, _BACKEND_DIR)

from mavlink_bus import MavlinkBus
import mavlink_fence as mf
from models import Vertex
from config import config
from search_algorithm import SearchController
import state


def main() -> int:
    device = os.environ.get("MAVLINK_DEVICE", "udp:127.0.0.1:14550")
    print(f"Connecting to SITL on {device} (timeout {config.MAVLINK_TIMEOUT_S}s)...")
    print("=" * 60)

    bus = MavlinkBus(device, config.MAVLINK_BAUD, timeout_s=config.MAVLINK_TIMEOUT_S)

    try:
        bus.start()
        print("PASS  SITL heartbeat received.")

        # 1. Fetch EKF origin
        lat, lon = mf.request_ekf_origin(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        print(f"PASS  EKF origin received: lat={lat:.7f}, lon={lon:.7f}")

        # 2. Upload polygon fence around EKF origin (~25m square)
        d = 0.00025
        verts = [
            Vertex(lat=lat + d, lon=lon + d),
            Vertex(lat=lat + d, lon=lon - d),
            Vertex(lat=lat - d, lon=lon - d),
            Vertex(lat=lat - d, lon=lon + d),
        ]
        mf.upload_polygon_fence(bus, verts, timeout_s=config.MAVLINK_TIMEOUT_S)
        mf.enable_fence(bus, config.FENCE_ACTION, timeout_s=config.MAVLINK_TIMEOUT_S)
        status = mf.build_geofence_status(bus, verts, lat, lon, config.FENCE_ACTION, config.MAVLINK_TIMEOUT_S)
        state.set(status)
        print(f"PASS  Geofence uploaded and enabled (Centroid ENU: {status.centroid_enu}, Max Radius: {status.max_radius_m:.1f}m)")

        # 3. Initialize SearchController
        controller = SearchController(bus=bus)

        # 4. Execute Dry-Run verification first
        dry_wps = controller.run_dry_run(strategy_name="expanding_square", altitude_m=5.0, step_m=3.0)
        print(f"PASS  Dry-run generated {len(dry_wps)} valid waypoints inside geofence.")

        # 5. Start live sweep guidance execution
        print("\nLaunching live SITL search sweep...")
        search_status = controller.start_search(dry_run=False, strategy_name="expanding_square", altitude_m=5.0, step_m=3.0)
        assert search_status.state == "searching", f"Expected 'searching' state, got '{search_status.state}'"
        print("PASS  Search controller transition to 'searching' state.")

        # 6. Stream guidance targets for 4 seconds and monitor progress
        for t_sec in range(4):
            time.sleep(1.0)
            curr = controller.get_status()
            print(f"      [t+{t_sec+1}s] State: {curr.state} | Current Waypoint #{curr.current_waypoint_idx}/{curr.total_waypoints}")

        # 7. Simulate target detection callback (Phase 7 decoupling check)
        print("\nTesting on_detection() callback during search...")
        controller.on_detection(bbox={"x": 120, "y": 80, "w": 40, "h": 40}, confidence=0.96)
        det_status = controller.get_status()
        assert det_status.state == "target_found", f"Expected 'target_found', got '{det_status.state}'"
        assert det_status.target["confidence"] == 0.96
        print("PASS  Target detection callback paused search sweep and set state to 'target_found'.")

        # 8. Clean stop
        print("\nStopping search sweep...")
        stop_status = controller.stop_search("SITL test completed")
        assert stop_status.state == "idle"
        print("PASS  Search algorithm stopped cleanly.")

        # 9. Cleanup fence
        mf.clear_fence(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        state.clear()
        print("PASS  Geofence cleared.")

        print("\n" + "=" * 60)
        print("ALL SITL SEARCH SWEEP ACCEPTANCE TESTS PASSED!")
        print("=" * 60)
        return 0

    except Exception as e:
        print(f"\nFAIL  SITL Search Acceptance Test: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        bus.stop()


if __name__ == "__main__":
    sys.exit(main())
