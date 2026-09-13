"""
Run: python3 tests/test_fence_protocol.py   (from the backend/ directory)

Validates mavlink_fence.py against the mock responder. This is a fast
sanity check for wire-protocol bugs — it is NOT a substitute for testing
against ArduPilot SITL or real hardware (see README's NEED TEST list).
"""

import os
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mock_fc import run_mock_fc
import mavlink_fence as mf
from mavlink_bus import MavlinkBus
from models import Vertex as FenceVertex

PORT = 14560


def main():
    stop = threading.Event()
    t = threading.Thread(
        target=run_mock_fc, args=(f"udp:0.0.0.0:{PORT}", stop), daemon=True
    )
    t.start()
    time.sleep(1.0)  # Give mock FC time to bind and be ready

    failures = []

    def check(name, fn):
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            failures.append(name)

    # Use MavlinkBus instead of old connect()
    bus = MavlinkBus(f"udpout:127.0.0.1:{PORT}", 115200, timeout_s=10.0)
    bus.start()

    origin = {}

    def do_connect():
        assert bus.conn.target_system is not None

    def do_origin():
        lat, lon = mf.request_ekf_origin(bus, timeout_s=5.0)
        assert -90 <= lat <= 90 and -180 <= lon <= 180
        origin["lat"], origin["lon"] = lat, lon

    def do_upload_readback():
        lat, lon = origin["lat"], origin["lon"]
        verts = [
            FenceVertex(lat=lat + 0.0005, lon=lon + 0.0005),
            FenceVertex(lat=lat + 0.0005, lon=lon - 0.0005),
            FenceVertex(lat=lat - 0.0005, lon=lon - 0.0005),
            FenceVertex(lat=lat - 0.0005, lon=lon + 0.0005),
        ]
        mf.upload_polygon_fence(bus, verts, timeout_s=10.0)
        readback = mf.download_fence(bus, timeout_s=10.0)
        assert len(readback) == len(verts)
        for rlatlon, v in zip(readback, verts):
            assert abs(rlatlon.lat - v.lat) < 1e-6 and abs(rlatlon.lon - v.lon) < 1e-6

    def do_params():
        mf.set_param(bus, "FENCE_ENABLE", 1, timeout_s=5.0)
        mf.set_param(bus, "FENCE_ACTION", 1, timeout_s=5.0)

    def do_clear():
        mf.clear_fence(bus, timeout_s=5.0)
        readback = mf.download_fence(bus, timeout_s=5.0)
        assert readback == []

    def do_reject_short_polygon():
        try:
            mf.upload_polygon_fence(bus, [FenceVertex(lat=0, lon=0)] * 2, timeout_s=5.0)
            raise AssertionError("expected FenceError for <3 vertices")
        except mf.FenceError:
            pass

    check("connect + heartbeat", do_connect)
    check("request EKF origin", do_origin)
    check("upload + readback match", do_upload_readback)
    check("param set/confirm", do_params)
    check("clear fence", do_clear)
    check("reject <3 vertices", do_reject_short_polygon)

    bus.stop()
    stop.set()

    print()
    if failures:
        print(f"{len(failures)} test(s) failed: {failures}")
        sys.exit(1)
    print("All protocol tests passed against the mock responder.")
    print("Reminder: still run against SITL / real hardware before competition.")


if __name__ == "__main__":
    main()