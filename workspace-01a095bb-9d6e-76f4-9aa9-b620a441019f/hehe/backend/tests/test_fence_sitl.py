"""
test_fence_sitl.py — Phase 2 fence protocol validation against ArduPilot SITL.

Unlike test_fence_protocol.py (which validates against a minimal mock FC),
this test runs against real ArduPilot SITL and closes several gaps that only
real firmware can close:

  1. Confirms fence upload/readback/enable work end-to-end with ArduPilot.
  2. FENCE_TYPE observation: reads FENCE_TYPE *before* enable_fence() is called,
     recording whether ArduPilot auto-inferred the type from the uploaded polygon
     items. This resolves the NEED TEST item from the original README.
  3. Reads back FENCE_ENABLE, FENCE_ACTION, FENCE_TYPE after enable to confirm
     ArduPilot accepted all three.

Prerequisites:
    # Terminal 1 — SITL (with or without Gazebo physics)
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
    # OR without Gazebo:
    sim_vehicle.py -v ArduCopter -f quad --console --map

    # Terminal 2 — this test
    cd <repo>/backend
    export MAVLINK_DEVICE=udp:127.0.0.1:14550
    python3 tests/test_fence_sitl.py

Output will print the observed FENCE_TYPE value (before and after enable_fence).
Copy those values into the enable_fence() docstring in mavlink_fence.py to close
the NEED TEST item permanently.

Exit code: 0 = all tests passed. Non-zero = failure.
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


# ---------------------------------------------------------------------------
# Test coordinates — small square offset from wherever the SITL EKF origin is.
# ~0.00018 deg ≈ 20m at equatorial latitudes. Small enough to arm and fly safely
# inside, large enough to not trigger the fence immediately on takeoff.
# ---------------------------------------------------------------------------
_FENCE_OFFSET_DEG = 0.00018   # ≈ 20m per side


def _make_fence_square(origin_lat: float, origin_lon: float) -> list[Vertex]:
    d = _FENCE_OFFSET_DEG
    return [
        Vertex(lat=origin_lat + d, lon=origin_lon + d),
        Vertex(lat=origin_lat + d, lon=origin_lon - d),
        Vertex(lat=origin_lat - d, lon=origin_lon - d),
        Vertex(lat=origin_lat - d, lon=origin_lon + d),
    ]


# ---------------------------------------------------------------------------
# Simple test runner
# ---------------------------------------------------------------------------

class Runner:
    def __init__(self):
        self.failures: list[str] = []

    def check(self, name: str, fn):
        try:
            result = fn()
            status = "PASS"
        except Exception as e:
            result = None
            status = f"FAIL  {name}: {type(e).__name__}: {e}"
            self.failures.append(name)
        if status == "PASS":
            print(f"PASS  {name}")
        else:
            print(status)
        return result

    def done(self) -> int:
        print()
        print("─" * 60)
        if not self.failures:
            print("All SITL fence tests passed.")
            print()
            print("ACTION REQUIRED: copy the FENCE_TYPE values printed above into")
            print("the enable_fence() docstring in mavlink_fence.py to close the")
            print("NEED TEST item. Cite the ArduPilot version printed below.")
        else:
            print(f"{len(self.failures)} test(s) FAILED: {self.failures}")
        return 1 if self.failures else 0


def main() -> int:
    device = os.environ.get("MAVLINK_DEVICE", "udp:127.0.0.1:14550")
    print(f"Connecting to SITL on {device}  (timeout {config.MAVLINK_TIMEOUT_S}s)")
    print("─" * 60)

    bus = MavlinkBus(device, config.MAVLINK_BAUD, timeout_s=config.MAVLINK_TIMEOUT_S)
    runner = Runner()
    origin = {}
    ap_version = {}

    # ------------------------------------------------------------------ #
    # T1 — connection + heartbeat                                          #
    # ------------------------------------------------------------------ #

    def t1_connect():
        bus.start()
        assert bus.conn is not None, "bus.conn is None after start()"
        ts = bus.conn.target_system
        tc = bus.conn.target_component
        assert ts and ts != 0, f"target_system={ts!r} (heartbeat not received)"
        assert tc is not None, f"target_component={tc!r}"

    runner.check("T1: SITL heartbeat — target_system/component populated", t1_connect)

    # ------------------------------------------------------------------ #
    # T2 — read ArduPilot firmware version from heartbeat                 #
    # (informational only — used to document FENCE_TYPE observations)     #
    # ------------------------------------------------------------------ #

    def t2_autopilot_version():
        """
        Read the AUTOPILOT_VERSION message to get the firmware version string.
        This is how we cite the ArduPilot version for the NEED TEST resolution.
        If the message isn't available within timeout, print "unknown" but don't fail.
        """
        from pymavlink import mavutil as mu
        bus.send(
            bus.conn.mav.command_long_send,
            bus.conn.target_system,
            bus.conn.target_component,
            mu.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mu.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0,
        )
        msg = bus.wait_for(["AUTOPILOT_VERSION"], lambda m: True, timeout_s=5.0)
        if msg is not None:
            # The flight_sw_version field is a uint32 packed as major.minor.patch.type
            v = msg.flight_sw_version
            major = (v >> 24) & 0xFF
            minor = (v >> 16) & 0xFF
            patch = (v >> 8) & 0xFF
            # type: 0=dev, 64=alpha, 128=beta, 192=rc, 255=release
            vtype = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "release"}.get(v & 0xFF, "?")
            version_str = f"{major}.{minor}.{patch}-{vtype}"
        else:
            version_str = "unknown (AUTOPILOT_VERSION not received within 5s)"
        ap_version["str"] = version_str
        print(f"      ArduPilot version: {version_str}")

    runner.check("T2: ArduPilot firmware version (informational)", t2_autopilot_version)

    # ------------------------------------------------------------------ #
    # T3 — EKF origin                                                     #
    # ------------------------------------------------------------------ #

    def t3_ekf_origin():
        lat, lon = mf.request_ekf_origin(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        assert -90 <= lat <= 90 and -180 <= lon <= 180, f"origin out of range: {lat}, {lon}"
        origin["lat"], origin["lon"] = lat, lon
        print(f"      EKF origin: {lat:.7f}, {lon:.7f}")

    runner.check("T3: EKF origin received", t3_ekf_origin)

    if not origin:
        print("\nCannot proceed without EKF origin — aborting.")
        bus.stop()
        return 1

    # ------------------------------------------------------------------ #
    # T4 — read FENCE_TYPE *before* any upload (observe baseline)         #
    # ------------------------------------------------------------------ #

    def t4_fence_type_baseline():
        """
        Read FENCE_TYPE before we upload anything. This tells us what ArduPilot
        reports by default. After the polygon upload (T5) but before enable_fence()
        (T7), we read it again. The delta tells us whether ArduPilot auto-infers
        the fence type from uploaded items.
        """
        ft = mf.get_param(bus, "FENCE_TYPE", timeout_s=config.MAVLINK_TIMEOUT_S)
        origin["fence_type_baseline"] = ft
        print(f"      FENCE_TYPE (before upload): {int(ft)}  (bitmask: see enable_fence docstring)")

    runner.check("T4: FENCE_TYPE baseline read (before upload)", t4_fence_type_baseline)

    # ------------------------------------------------------------------ #
    # T5 — upload polygon fence                                           #
    # ------------------------------------------------------------------ #

    def t5_upload():
        verts = _make_fence_square(origin["lat"], origin["lon"])
        mf.upload_polygon_fence(bus, verts, timeout_s=config.MAVLINK_TIMEOUT_S)
        origin["verts"] = verts
        print(f"      Uploaded {len(verts)}-vertex square fence around origin")

    runner.check("T5: Fence upload accepted", t5_upload)

    # ------------------------------------------------------------------ #
    # T6 — read FENCE_TYPE after upload, before enable                    #
    # (key observation: did ArduPilot auto-set the polygon bit?)          #
    # ------------------------------------------------------------------ #

    def t6_fence_type_after_upload():
        ft = mf.get_param(bus, "FENCE_TYPE", timeout_s=config.MAVLINK_TIMEOUT_S)
        origin["fence_type_after_upload"] = ft
        baseline = origin.get("fence_type_baseline", -1)
        polygon_bit = 4  # _FENCE_TYPE_POLYGON_BIT from mavlink_fence.py
        auto_inferred = bool(int(ft) & polygon_bit)
        print(f"      FENCE_TYPE (after upload, before enable_fence): {int(ft)}")
        if auto_inferred:
            print(f"      → ArduPilot DID auto-infer the polygon bit (bit 2 = value 4)")
            print(f"        The explicit FENCE_TYPE set in enable_fence() is a safe no-op.")
        else:
            print(f"      → ArduPilot did NOT auto-infer the polygon bit.")
            print(f"        The explicit FENCE_TYPE set in enable_fence() is REQUIRED.")
        print(f"      → Update enable_fence() docstring with: ArduPilot {ap_version.get('str','?')}")
        print(f"        FENCE_TYPE before upload={int(baseline)}, after upload={int(ft)}")
        # This is the key assertion: after upload, FENCE_TYPE observation is non-negative.
        # We do NOT assert a specific value here — the point is to OBSERVE and document,
        # not to enforce a pre-assumed behavior.
        assert ft >= 0, f"FENCE_TYPE read returned invalid value: {ft}"

    runner.check("T6: FENCE_TYPE observed after upload (key NEED TEST resolution)", t6_fence_type_after_upload)

    # ------------------------------------------------------------------ #
    # T7 — readback: downloaded fence matches uploaded vertices           #
    # ------------------------------------------------------------------ #

    def t7_readback():
        verts = origin.get("verts", [])
        readback = mf.download_fence(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        assert len(readback) == len(verts), (
            f"Readback count {len(readback)} != uploaded {len(verts)}"
        )
        for i, (r, v) in enumerate(zip(readback, verts)):
            assert abs(r.lat - v.lat) < 1e-6, (
                f"Vertex {i} lat mismatch: got {r.lat}, expected {v.lat}"
            )
            assert abs(r.lon - v.lon) < 1e-6, (
                f"Vertex {i} lon mismatch: got {r.lon}, expected {v.lon}"
            )

    runner.check("T7: Fence readback matches uploaded vertices", t7_readback)

    # ------------------------------------------------------------------ #
    # T8 — enable fence and read back all three params                    #
    # ------------------------------------------------------------------ #

    def t8_enable_and_verify():
        mf.enable_fence(bus, config.FENCE_ACTION, timeout_s=config.MAVLINK_TIMEOUT_S)

        fe = mf.get_param(bus, "FENCE_ENABLE", timeout_s=config.MAVLINK_TIMEOUT_S)
        fa = mf.get_param(bus, "FENCE_ACTION", timeout_s=config.MAVLINK_TIMEOUT_S)
        ft = mf.get_param(bus, "FENCE_TYPE", timeout_s=config.MAVLINK_TIMEOUT_S)

        print(f"      FENCE_ENABLE={int(fe)}, FENCE_ACTION={int(fa)}, FENCE_TYPE={int(ft)}")

        assert int(fe) == 1, f"FENCE_ENABLE readback = {fe}, expected 1"
        assert int(fa) == config.FENCE_ACTION, (
            f"FENCE_ACTION readback = {fa}, expected {config.FENCE_ACTION}"
        )
        polygon_bit = 4  # _FENCE_TYPE_POLYGON_BIT
        assert int(ft) & polygon_bit, (
            f"FENCE_TYPE readback = {int(ft)}, polygon bit (4) not set after enable_fence(). "
            f"Check _FENCE_TYPE_POLYGON_BIT in mavlink_fence.py against your firmware version."
        )
        origin["fence_type_after_enable"] = ft

    runner.check("T8: FENCE_ENABLE/ACTION/TYPE readback correct after enable_fence()", t8_enable_and_verify)

    # ------------------------------------------------------------------ #
    # T9 — clear fence (cleanup)                                          #
    # ------------------------------------------------------------------ #

    def t9_clear():
        mf.clear_fence(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        readback = mf.download_fence(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        assert readback == [], f"Fence not empty after clear: {readback}"
        # Also disable fence after clear
        mf.set_param(bus, "FENCE_ENABLE", 0, timeout_s=config.MAVLINK_TIMEOUT_S)

    runner.check("T9: Fence clear + disable (cleanup)", t9_clear)

    # ------------------------------------------------------------------ #
    # Summary and FENCE_TYPE resolution report                            #
    # ------------------------------------------------------------------ #

    print()
    print("═" * 60)
    print("FENCE_TYPE Resolution (paste into enable_fence() docstring):")
    print("═" * 60)
    print(f"  ArduPilot version : {ap_version.get('str', 'unknown')}")
    print(f"  FENCE_TYPE before upload  : {int(origin.get('fence_type_baseline', -1))}")
    print(f"  FENCE_TYPE after upload   : {int(origin.get('fence_type_after_upload', -1))}")
    print(f"  FENCE_TYPE after enable   : {int(origin.get('fence_type_after_enable', -1))}")
    polygon_bit = 4
    after_upload = origin.get("fence_type_after_upload", -1)
    if after_upload >= 0:
        if int(after_upload) & polygon_bit:
            print("  Conclusion: ArduPilot AUTO-INFERS polygon bit after upload.")
            print("              explicit FENCE_TYPE set in enable_fence() is safe but redundant.")
        else:
            print("  Conclusion: ArduPilot does NOT auto-infer polygon bit.")
            print("              explicit FENCE_TYPE set in enable_fence() is REQUIRED.")

    bus.stop()
    return runner.done()


if __name__ == "__main__":
    sys.exit(main())
