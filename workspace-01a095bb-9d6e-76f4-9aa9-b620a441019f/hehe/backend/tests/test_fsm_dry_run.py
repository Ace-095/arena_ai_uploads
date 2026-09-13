import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import logging

os.environ.setdefault("MAVLINK20", "1")
os.environ["FSM_DRY_RUN"] = "1"
os.environ["FSM_PREFLIGHT_TIMEOUT_S"] = "2.0"
# Default is 120s. search_controller.start_search(dry_run=True) only
# computes and logs a waypoint plan — it never runs a live detection loop
# or calls on_detection(), by design (see search_algorithm.py::run_dry_run).
# That means under FSM_DRY_RUN, SEARCH can NEVER reach "target_found" —
# it can only ever end via this sweep timeout. Shortened here so the test
# finishes in a few seconds instead of hanging for 2 minutes waiting on a
# timeout it was always going to hit.
os.environ["FSM_QR_SWEEP_TIMEOUT_S"] = "2.0"

from config import config
import state
from models import GeofenceStatus, Vertex, ENUPoint
import fsm
from mavlink_bus import MavlinkBus
from camera_manager import CameraManager
from camera_source import SyntheticQRSource

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

def main():
    """
    Scope of this test — read before "fixing" it again.

    FSM_DRY_RUN exercises PRE_FLIGHT_CHECK -> ARM -> TAKEOFF -> SEARCH
    (real waypoint-plan generation + real camera switch to cam1) and the
    fail-closed cascade FAILSAFE -> RTL -> LAND -> MISSION_COMPLETE. It
    does NOT and CANNOT exercise CACHE_DETECTED/APPROACH/DECODE, because
    dry-run search never runs a live detection loop against a camera feed
    — confirmed by reading search_algorithm.py::run_dry_run, which only
    computes and logs a plan. Testing the actual "finds and decodes the
    target" happy path needs a live MAVLink connection driving a
    non-dry-run search (SITL at minimum) — that's a separate test, not
    something this dry-run script can fake without lying about what it
    verified. See docs/IMPLEMENTATION_PLAN.md Phase 6's acceptance
    criteria: the live-sweep test was always meant to run against SITL,
    not pure dry-run.
    """
    print("--- Starting Tier 2: FSM DRY-RUN Acceptance Test ---")
    print("(scope: pre-flight/arm/takeoff/search-plan/camera-switch/failsafe-cascade")
    print(" — NOT the search-finds-target happy path, see docstring)")
    
    # Use a dummy UDP endpoint for bus just to initialize it
    bus = MavlinkBus(device="udp:127.0.0.1:14550", baud=115200, timeout_s=1.0)
    # We won't start the bus thread for dry-run since we just want to test logic without SITL
    # But wait, without bus thread, we won't get heartbeat.
    # We can mock state and let FSM run.

    # A real CameraManager with synthetic sources — without this, SEARCH
    # immediately fails closed before ever reaching a real search plan.
    cam_mgr = CameraManager({
        "cam1": SyntheticQRSource(payload=config.SYNTHETIC_QR_PAYLOAD, fps=30.0),
        "cam2": SyntheticQRSource(payload=config.SYNTHETIC_QR_PAYLOAD, fps=30.0),
    })

    # A minimal loaded=True/armable=True fence isn't enough —
    # search_controller.start_search() needs real geometry to generate a
    # plan at all (confirmed: it raised "Fence state missing centroid or
    # max_radius_m" without this). Build an actual small square fence.
    square_vertices_enu = [
        ENUPoint(x=-25.0, y=-25.0), ENUPoint(x=25.0, y=-25.0),
        ENUPoint(x=25.0, y=25.0), ENUPoint(x=-25.0, y=25.0),
    ]
    fence = GeofenceStatus(
        loaded=True,
        armable=True,
        reason="DRY RUN OK",
        vertex_count=4,
        vertices_latlon=[Vertex(lat=12.9716 + dy * 1e-4, lon=77.5946 + dx * 1e-4)
                         for dx, dy in [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)]],
        vertices_enu=square_vertices_enu,
        centroid_enu=ENUPoint(x=0.0, y=0.0),
        max_radius_m=35.36,  # sqrt(25^2 + 25^2), matches the square above
        origin_lat=12.9716,
        origin_lon=77.5946,
        fc_readback_matched=True,
    )
    state.set(fence)
    
    mission = fsm.init_fsm(bus, camera_manager=cam_mgr)
    mission._last_heartbeat = time.time()  # Fake healthy link
    
    mission.start()
    
    print("Waiting for FSM to transition through states...")
    visited_states = []
    try:
        while True:
            status = mission.get_status()
            if not visited_states or visited_states[-1] != status.state:
                visited_states.append(status.state)
            print(f"Current State: {status.state} | Reason: {status.reason}")
            
            # Keep link alive
            mission._last_heartbeat = time.time()
            
            if status.state == "MISSION_COMPLETE":
                # The ONLY legitimate way to reach MISSION_COMPLETE under
                # dry-run is via SEARCH's sweep timeout cascading through
                # FAILSAFE -> RTL -> LAND. If SEARCH was skipped or the
                # plan was never actually generated, something upstream
                # broke silently — check for that instead of accepting
                # any arrival at MISSION_COMPLETE as success.
                expected_min_path = ["PRE_FLIGHT_CHECK", "ARM", "TAKEOFF", "SEARCH",
                                      "FAILSAFE", "RTL", "LAND", "MISSION_COMPLETE"]
                if visited_states == expected_min_path:
                    print(f"Test passed: walked the expected dry-run path {visited_states} "
                          f"(sweep-timeout -> failsafe cascade, as designed for this mode).")
                else:
                    print(f"Test FAILED: expected exactly {expected_min_path}, "
                          f"got {visited_states}")
                    raise SystemExit(1)
                break
                
            time.sleep(0.02)
    except KeyboardInterrupt:
        mission.abort()
        
if __name__ == "__main__":
    main()
