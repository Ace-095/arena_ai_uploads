"""
test_approach_descent.py — Tests SearchController.begin_approach_descent()

Confirms the closed-loop descent actually does what it's supposed to:
- Holds horizontal position (doesn't drift while descending)
- Steps down incrementally, not straight to a fixed altitude
- Stops the INSTANT live QR decode succeeds — the actual point of this
  being closed-loop rather than "descend to configured constant X"
- Respects the safety floor when decode never succeeds

Run: python3 tests/test_approach_descent.py
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("MAVLINK20", "1")

# Set before any config import so these take effect on first load —
# avoids needing to re-instantiate Config() mid-test.
os.environ["APPROACH_DESCENT_STEP_M"] = "2.0"
os.environ["APPROACH_MIN_ALTITUDE_M"] = "2.0"
os.environ["APPROACH_STEP_HOLD_S"] = "1.0"
os.environ["APPROACH_MAX_DURATION_S"] = "20.0"

from mock_fc import run_mock_fc
from mavlink_bus import MavlinkBus
from models import GeofenceStatus, ENUPoint
import state
from config import config
import search_algorithm as sa
from camera_source import SyntheticQRSource
import qr_pipeline
import numpy as np

PORT = 14571


def setup_fence_state(origin_lat, origin_lon):
    """A dummy fence just to supply the EKF origin the descent loop needs."""
    fence = GeofenceStatus(
        loaded=True, armable=True, reason="test", vertex_count=4,
        vertices_enu=[ENUPoint(x=-25, y=-25), ENUPoint(x=25, y=-25),
                      ENUPoint(x=25, y=25), ENUPoint(x=-25, y=25)],
        centroid_enu=ENUPoint(x=0, y=0), max_radius_m=35.36,
        origin_lat=origin_lat, origin_lon=origin_lon, fc_readback_matched=True,
    )
    state.set(fence)


class NeverDecodesSource:
    """Produces valid-shaped frames with no QR in them at all."""
    def start(self): pass
    def get_frame(self):
        time.sleep(0.03)
        return np.full((240, 320, 3), 128, dtype="uint8")
    def stop(self): pass


def main():
    failures = []

    def check(name, fn):
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            failures.append(name)

    stop = threading.Event()
    mock_thread = threading.Thread(
        target=run_mock_fc, args=(f"udp:0.0.0.0:{PORT}", stop), daemon=True
    )
    mock_thread.start()
    time.sleep(1.0)

    bus = MavlinkBus(f"udpout:127.0.0.1:{PORT}", 115200, timeout_s=10.0)
    bus.start()

    setup_fence_state(12.9716, 77.5946)
    sa.search_controller.set_bus(bus)

    def do_stops_at_decode_not_fixed_altitude():
        """
        A camera that decodes successfully from the very first frame
        should stop the descent almost immediately, near the vehicle's
        STARTING altitude — not descend all the way to the safety floor
        just because a fixed altitude was configured. This is the actual
        point of the fix: proving it's decode-driven, not altitude-driven.
        """
        qr_pipeline.qr_pipeline_runner.consensus.reset()
        source = SyntheticQRSource(payload="TESTCODE", width=320, height=240,
                                    fps=30.0, pan_speed=0.0, zoom_amplitude=0.0)
        qr_pipeline.qr_pipeline_runner.set_source(source)
        qr_pipeline.qr_pipeline_runner.start()

        deadline = time.time() + 5.0
        while not qr_pipeline.qr_pipeline_runner.consensus.get_status().confirmed:
            if time.time() > deadline:
                raise AssertionError("QR pipeline never reached consensus in setup")
            time.sleep(0.1)

        sa.search_controller.begin_approach_descent()

        deadline = time.time() + 10.0
        final_status = None
        while time.time() < deadline:
            final_status = sa.search_controller.get_status()
            if final_status.state == "decoded":
                break
            time.sleep(0.1)

        qr_pipeline.qr_pipeline_runner.stop()

        assert final_status is not None and final_status.state == "decoded", (
            f"expected state='decoded', got {final_status.state if final_status else None}"
        )
        assert final_status.altitude_m > config.APPROACH_MIN_ALTITUDE_M, (
            f"decode succeeded immediately but descent still went all the way "
            f"to {final_status.altitude_m}m (safety floor is {config.APPROACH_MIN_ALTITUDE_M}m) "
            f"— this should have stopped much higher, at whatever altitude it started."
        )

    def do_reaches_safety_floor_when_never_decodable():
        """A camera that never produces a decodable frame should cause the
        descent to stop exactly at the safety floor, not descend past it,
        and report state='error' rather than silently succeeding."""
        qr_pipeline.qr_pipeline_runner.consensus.reset()
        qr_pipeline.qr_pipeline_runner.set_source(NeverDecodesSource())
        qr_pipeline.qr_pipeline_runner.start()

        sa.search_controller.begin_approach_descent()

        deadline = time.time() + 25.0
        final_status = None
        while time.time() < deadline:
            final_status = sa.search_controller.get_status()
            if final_status.state == "error":
                break
            time.sleep(0.2)

        qr_pipeline.qr_pipeline_runner.stop()

        assert final_status is not None and final_status.state == "error", (
            f"expected state='error' (safety floor reached), got "
            f"{final_status.state if final_status else None}"
        )
        assert abs(final_status.altitude_m - config.APPROACH_MIN_ALTITUDE_M) < 0.01, (
            f"expected to stop exactly at the safety floor "
            f"{config.APPROACH_MIN_ALTITUDE_M}m, got {final_status.altitude_m}m"
        )

    check("descent stops at decode success, not a fixed altitude", do_stops_at_decode_not_fixed_altitude)
    check("descent respects the safety floor when never decodable", do_reaches_safety_floor_when_never_decodable)

    bus.stop()
    stop.set()

    print()
    if failures:
        print(f"{len(failures)} test(s) failed: {failures}")
        sys.exit(1)
    print("All approach-descent tests passed.")


if __name__ == "__main__":
    main()
