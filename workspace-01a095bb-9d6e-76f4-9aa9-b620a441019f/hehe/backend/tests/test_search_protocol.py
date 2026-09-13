"""
Mock protocol test for search algorithm position guidance (Phase 6).

Validates search guidance streaming over MavlinkBus using SET_POSITION_TARGET_LOCAL_NED
against mock_fc responder.

Run from backend/ directory:
    python3 tests/test_search_protocol.py
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mock_fc import run_mock_fc
from mavlink_bus import MavlinkBus
import mavlink_fence as mf
from models import Vertex as FenceVertex
from search_algorithm import SearchController
import state

PORT = 14562


def main():
    stop = threading.Event()
    t = threading.Thread(
        target=run_mock_fc, args=(f"udp:0.0.0.0:{PORT}", stop), daemon=True
    )
    t.start()
    time.sleep(1.0)

    bus = MavlinkBus(f"udpout:127.0.0.1:{PORT}", 115200, timeout_s=10.0)
    bus.start()

    try:
        lat, lon = mf.request_ekf_origin(bus, timeout_s=5.0)
        assert -90 <= lat <= 90 and -180 <= lon <= 180

        # Upload dummy polygon fence
        verts = [
            FenceVertex(lat=lat + 0.0005, lon=lon + 0.0005),
            FenceVertex(lat=lat + 0.0005, lon=lon - 0.0005),
            FenceVertex(lat=lat - 0.0005, lon=lon - 0.0005),
            FenceVertex(lat=lat - 0.0005, lon=lon + 0.0005),
        ]
        mf.upload_polygon_fence(bus, verts, timeout_s=10.0)
        mf.enable_fence(bus, 1, timeout_s=5.0)
        status = mf.build_geofence_status(bus, verts, lat, lon, 1, 5.0)
        state.set(status)

        controller = SearchController(bus=bus)

        # 1. Start live search sweep
        print("Starting live MAVLink search sweep...")
        status = controller.start_search(dry_run=False, strategy_name="expanding_square", altitude_m=4.0, step_m=3.0)
        assert status.state == "searching", f"Expected state 'searching', got '{status.state}'"
        assert status.total_waypoints > 0

        # 2. Let worker thread stream targets for 2 seconds
        time.sleep(2.0)
        curr_status = controller.get_status()
        print(f"Active search progress: WP #{curr_status.current_waypoint_idx}/{curr_status.total_waypoints}")

        # 3. Stop search
        print("Stopping search sweep...")
        stop_status = controller.stop_search("Protocol test completion")
        assert stop_status.state == "idle"

        print("PASS: Search algorithm guidance protocol test passed against mock flight controller.")

    finally:
        bus.stop()
        stop.set()


if __name__ == "__main__":
    main()
