"""
test_fence_breach_sitl.py — Phase 2 fence breach validation against ArduPilot SITL.

Proves that FENCE_ACTION=1 (RTL) actually fires when the simulated drone
crosses a fence boundary in GUIDED mode. This is the behavior gap that was
impossible to test against the mock FC — only real ArduPilot firmware enforces
fence actions.

Sequence:
  1. Upload a small (20m × 20m) fence centred on SITL home.
  2. Enable fence with FENCE_ACTION=1 (RTL).
  3. Arm + take off to 5m in GUIDED mode via MAVLink.
  4. Issue a GUIDED waypoint 30m outside the fence edge (guarantees breach).
  5. Wait for FENCE_STATUS.breach_status != 0 (breach detected).
  6. Wait for mode to transition to RTL (ArduCopter mode 6).
  7. Land and disarm (cleanup).

Prerequisites:
    # Terminal 1 — SITL + Gazebo (or plain SITL):
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
    # Optionally: gz sim sim/worlds/qr_search_field.sdf

    # Terminal 2 — this test
    cd <repo>/backend
    export MAVLINK_DEVICE=udp:127.0.0.1:14550
    python3 tests/test_fence_breach_sitl.py

IMPORTANT: Run test_fence_sitl.py first (Plan 2.1) to confirm basic fence
protocol works before running this test.

Mode enum notes (ArduCopter):
    GUIDED = 4, RTL = 6 — from ArduCopter/mode.h enum class Mode::Number.
    NEED TEST: these values are cited from ArduCopter source code. Confirm
    they match your firmware build by checking the printed heartbeat mode
    before takeoff (test will print the mode observed at each stage).
    If the assertion fails with an unexpected mode number, update
    _COPTER_MODE_RTL and _COPTER_MODE_GUIDED to the observed values.

Exit code: 0 = all tests passed. Non-zero = failure.
"""

import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

_BACKEND_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND_DIR not in sys.path:
    sys.path.insert(0, _BACKEND_DIR)

from pymavlink import mavutil
from mavlink_bus import MavlinkBus
import mavlink_fence as mf
from models import Vertex
from config import config


# ---------------------------------------------------------------------------
# ArduCopter custom mode numbers.
# Source: ArduCopter/mode.h enum class Mode::Number (uint8_t).
# NEED TEST: verify these match your exact firmware build. The test prints
# the observed mode at each stage — if an assertion fails, update these.
# ---------------------------------------------------------------------------
_COPTER_MODE_GUIDED = 4
_COPTER_MODE_RTL = 6
_COPTER_MODE_LAND = 9

# Fence size: ±10m from origin (~0.00009 deg per metre at equatorial lat)
# This gives a 20m×20m fence. The breach waypoint is placed 30m outside.
_FENCE_HALF_DEG = 0.0000898   # ≈ 10m in degrees latitude (1 deg ≈ 111km)
_BREACH_WAYPOINT_NORTH_M = 30  # metres North of fence edge = ~40m from origin

# Timeouts
_TAKEOFF_TIMEOUT_S = 30.0
_BREACH_TIMEOUT_S  = 45.0  # time allowed for vehicle to reach fence and breach
_RTL_TIMEOUT_S     = 15.0  # time after breach for RTL mode to engage


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_fence_square(lat: float, lon: float) -> list[Vertex]:
    d = _FENCE_HALF_DEG
    return [
        Vertex(lat=lat + d, lon=lon + d),
        Vertex(lat=lat + d, lon=lon - d),
        Vertex(lat=lat - d, lon=lon - d),
        Vertex(lat=lat - d, lon=lon + d),
    ]


def _get_current_mode(bus: MavlinkBus, timeout_s: float = 3.0) -> int:
    """Subscribe to heartbeat and return the current custom_mode value."""
    msg = bus.wait_for(["HEARTBEAT"], lambda m: True, timeout_s=timeout_s)
    if msg is None:
        raise RuntimeError("No heartbeat received")
    return msg.custom_mode


def _send_command_long(bus: MavlinkBus, command: int, *params) -> None:
    """Send MAV_CMD via COMMAND_LONG. params is up to 7 floats (padded with 0)."""
    p = list(params) + [0.0] * (7 - len(params))
    bus.send(
        bus.conn.mav.command_long_send,
        bus.conn.target_system,
        bus.conn.target_component,
        command, 0,  # command, confirmation
        p[0], p[1], p[2], p[3], p[4], p[5], p[6],
    )


def _wait_command_ack(bus: MavlinkBus, command: int, timeout_s: float = 5.0) -> bool:
    """Wait for COMMAND_ACK for a specific command. Returns True if accepted."""
    msg = bus.wait_for(
        ["COMMAND_ACK"],
        lambda m: m.command == command,
        timeout_s=timeout_s,
    )
    if msg is None:
        return False
    return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED


def _set_mode_guided(bus: MavlinkBus) -> None:
    """Switch to GUIDED mode via MAV_CMD_DO_SET_MODE."""
    _send_command_long(
        bus,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        _COPTER_MODE_GUIDED,
    )
    ok = _wait_command_ack(bus, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout_s=5.0)
    if not ok:
        raise RuntimeError("MAV_CMD_DO_SET_MODE GUIDED not accepted by FC")


def _arm(bus: MavlinkBus) -> None:
    """Arm the vehicle. SITL may require force-arm (param2=21196)."""
    _send_command_long(
        bus,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        1,      # param1: 1=arm
        21196,  # param2: force arm magic number (bypasses pre-arm checks in SITL)
    )
    ok = _wait_command_ack(bus, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout_s=10.0)
    if not ok:
        raise RuntimeError("ARM command not accepted. Check SITL pre-arm checks.")


def _disarm(bus: MavlinkBus) -> None:
    """Disarm the vehicle."""
    _send_command_long(bus, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)


def _takeoff(bus: MavlinkBus, altitude_m: float) -> None:
    """Command takeoff to altitude_m (relative)."""
    _send_command_long(
        bus,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0,   # pitch, empty, empty, yaw
        0, 0,         # lat, lon (0 = use current)
        altitude_m,   # param7: altitude MSL or relative — NEED TEST
    )
    # ArduPilot SITL sends COMMAND_ACK for NAV_TAKEOFF immediately but the
    # vehicle starts climbing. We wait for altitude via GLOBAL_POSITION_INT below.


def _wait_altitude(bus: MavlinkBus, min_alt_m: float, timeout_s: float) -> bool:
    """Wait until relative altitude ≥ min_alt_m. Returns True if achieved."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = bus.wait_for(["GLOBAL_POSITION_INT"], lambda m: True, timeout_s=1.0)
        if msg is not None:
            alt_m = msg.relative_alt / 1000.0  # mm → m
            if alt_m >= min_alt_m:
                return True
    return False


def _fly_to_local_ned(bus: MavlinkBus, north_m: float, east_m: float, down_m: float) -> None:
    """
    Send SET_POSITION_TARGET_LOCAL_NED to fly to a position relative to home.
    type_mask: ignore velocity/accel/yaw, use position only.

    type_mask bit layout (MAVLink spec):
      Bit  0 (0x001): ignore x (North)  — we WANT position, so bit=0
      Bit  1 (0x002): ignore y (East)   — bit=0
      Bit  2 (0x004): ignore z (Down)   — bit=0
      Bit  3 (0x008): ignore vx         — bit=1 (ignore)
      Bit  4 (0x010): ignore vy         — bit=1
      Bit  5 (0x020): ignore vz         — bit=1
      Bit  6 (0x040): ignore ax         — bit=1
      Bit  7 (0x080): ignore ay         — bit=1
      Bit  8 (0x100): ignore az         — bit=1
      Bit  9 (0x200): force set         — bit=0
      Bit 10 (0x400): ignore yaw        — bit=1
      Bit 11 (0x800): ignore yaw_rate   — bit=1
    Position-only: type_mask = 0b111111111000 = 0x7F8 = 2040
    NEED TEST: verify this type_mask value against your ArduPilot build. Some
    versions use a different convention. If the vehicle doesn't move as expected,
    try type_mask=0x0DF8 (ignores yaw but keeps force-set bit clear).
    """
    TYPE_MASK_POS_ONLY = 0x7F8

    bus.send(
        bus.conn.mav.set_position_target_local_ned_send,
        0,                                  # time_boot_ms (0 = use system time)
        bus.conn.target_system,
        bus.conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POS_ONLY,
        north_m, east_m, down_m,           # position (m)
        0, 0, 0,                            # velocity (ignored)
        0, 0, 0,                            # acceleration (ignored)
        0, 0,                               # yaw, yaw_rate (ignored)
    )


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

class Runner:
    def __init__(self):
        self.failures: list[str] = []

    def check(self, name: str, fn):
        try:
            fn()
            print(f"PASS  {name}")
            return True
        except Exception as e:
            print(f"FAIL  {name}: {type(e).__name__}: {e}")
            self.failures.append(name)
            return False

    def done(self) -> int:
        print()
        print("─" * 60)
        if not self.failures:
            print("All breach tests passed — FENCE_ACTION=RTL confirmed in simulation.")
        else:
            print(f"{len(self.failures)} test(s) FAILED: {self.failures}")
        return 1 if self.failures else 0


def main() -> int:
    device = os.environ.get("MAVLINK_DEVICE", "udp:127.0.0.1:14550")
    print(f"Connecting to SITL on {device}")
    print("Fence breach test (GUIDED → fly past fence → observe RTL)")
    print("─" * 60)

    bus = MavlinkBus(device, config.MAVLINK_BAUD, timeout_s=config.MAVLINK_TIMEOUT_S)
    runner = Runner()
    origin = {}

    def b0_connect():
        bus.start()
        ts = bus.conn.target_system
        assert ts and ts != 0, f"No heartbeat (target_system={ts!r})"
        mode = _get_current_mode(bus)
        print(f"      Initial mode: {mode}  (GUIDED={_COPTER_MODE_GUIDED}, RTL={_COPTER_MODE_RTL})")

    if not runner.check("B0: Connect to SITL", b0_connect):
        return 1

    def b1_ekf_origin():
        lat, lon = mf.request_ekf_origin(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        origin["lat"], origin["lon"] = lat, lon
        print(f"      Origin: {lat:.7f}, {lon:.7f}")

    if not runner.check("B1: EKF origin", b1_ekf_origin):
        bus.stop()
        return 1

    def b2_upload_fence():
        # Clear any previous fence first
        try:
            mf.clear_fence(bus, timeout_s=config.MAVLINK_TIMEOUT_S)
        except Exception:
            pass  # Ignore if nothing to clear
        verts = _make_fence_square(origin["lat"], origin["lon"])
        mf.upload_polygon_fence(bus, verts, timeout_s=config.MAVLINK_TIMEOUT_S)
        mf.enable_fence(bus, fence_action=1, timeout_s=config.MAVLINK_TIMEOUT_S)
        print(f"      Uploaded {len(verts)}-vertex fence (≈20m × 20m), FENCE_ACTION=RTL")

    if not runner.check("B2: Upload + enable fence", b2_upload_fence):
        bus.stop()
        return 1

    def b3_set_guided_arm_takeoff():
        _set_mode_guided(bus)
        time.sleep(0.5)
        mode = _get_current_mode(bus)
        print(f"      Mode after GUIDED cmd: {mode}")
        assert mode == _COPTER_MODE_GUIDED, (
            f"Expected GUIDED ({_COPTER_MODE_GUIDED}), got {mode}. "
            f"NEED TEST: verify _COPTER_MODE_GUIDED matches your firmware."
        )
        _arm(bus)
        time.sleep(1.0)
        _takeoff(bus, altitude_m=5.0)
        reached = _wait_altitude(bus, min_alt_m=4.0, timeout_s=_TAKEOFF_TIMEOUT_S)
        assert reached, (
            f"Vehicle did not reach 4m within {_TAKEOFF_TIMEOUT_S}s. "
            "Check SITL is running and the vehicle armed correctly."
        )
        alt_msg = bus.wait_for(["GLOBAL_POSITION_INT"], lambda m: True, timeout_s=2.0)
        actual_alt = (alt_msg.relative_alt / 1000.0) if alt_msg else "?"
        print(f"      Reached altitude ≈ {actual_alt}m")

    if not runner.check("B3: GUIDED mode + arm + takeoff to 5m", b3_set_guided_arm_takeoff):
        # Try to disarm and bail out cleanly
        try:
            _disarm(bus)
        except Exception:
            pass
        bus.stop()
        return 1

    def b4_fly_past_fence_and_wait_breach():
        """
        Fly ~40m North — past the 10m fence edge — and wait for FENCE_STATUS breach.

        The vehicle should hit the fence edge (10m North) during the flight.
        FENCE_STATUS.breach_status is a bitmask; non-zero means a breach type fired.
        NEED TEST: observe the actual breach_status value — ArduPilot may set it
        to 4 (polygon), 1 (circle), or a combination.
        """
        # Subscribe to FENCE_STATUS before commanding the flight
        fence_q = bus.subscribe(["FENCE_STATUS"])

        # Fly North well past the fence edge
        # down_m = -(altitude) in NED convention (negative = upward)
        _fly_to_local_ned(bus, north_m=40.0, east_m=0.0, down_m=-5.0)
        print(f"      Sent waypoint 40m North (past the 10m North fence edge)")

        # Wait for breach
        breach_msg = None
        deadline = time.time() + _BREACH_TIMEOUT_S
        while time.time() < deadline:
            try:
                msg = fence_q.get(timeout=0.5)
                if msg.breach_status != 0:
                    breach_msg = msg
                    break
            except Exception:
                pass

        bus.unsubscribe(fence_q)

        assert breach_msg is not None, (
            f"No fence breach detected within {_BREACH_TIMEOUT_S}s. "
            "Possible causes: fence not enabled, vehicle not reaching fence edge, "
            "FENCE_STATUS messages not flowing. Check SITL console for fence state."
        )
        print(f"      Breach detected: breach_status={breach_msg.breach_status} "
              f"breach_type={breach_msg.breach_type} "
              f"breach_count={breach_msg.breach_count}")

    if not runner.check("B4: Fly past fence edge — breach detected", b4_fly_past_fence_and_wait_breach):
        try:
            _disarm(bus)
        except Exception:
            pass
        bus.stop()
        return 1

    def b5_rtl_mode_engaged():
        """
        After breach, ArduPilot should switch to RTL automatically.
        Wait up to _RTL_TIMEOUT_S for the custom_mode to become _COPTER_MODE_RTL.
        """
        deadline = time.time() + _RTL_TIMEOUT_S
        observed_modes = []
        while time.time() < deadline:
            msg = bus.wait_for(["HEARTBEAT"], lambda m: True, timeout_s=1.0)
            if msg is not None:
                m = msg.custom_mode
                if m not in observed_modes:
                    observed_modes.append(m)
                if m == _COPTER_MODE_RTL:
                    print(f"      Mode transitioned to RTL ({_COPTER_MODE_RTL}) ✓")
                    return

        assert False, (
            f"RTL mode ({_COPTER_MODE_RTL}) not observed within {_RTL_TIMEOUT_S}s after breach. "
            f"Modes observed: {observed_modes}. "
            f"NEED TEST: if RTL fires but the mode number is different, update _COPTER_MODE_RTL. "
            f"If RTL never fires, check FENCE_ACTION=1 was accepted (run test_fence_sitl.py first)."
        )

    runner.check("B5: Mode transitioned to RTL after breach", b5_rtl_mode_engaged)

    # ------------------------------------------------------------------ #
    # Cleanup — wait for land (or force disarm after timeout)            #
    # ------------------------------------------------------------------ #

    print("      Waiting for vehicle to land (RTL descent)...")
    landed = False
    deadline = time.time() + 60.0
    while time.time() < deadline:
        msg = bus.wait_for(["EXTENDED_SYS_STATE"], lambda m: True, timeout_s=1.0)
        if msg is not None and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            landed = True
            break
        # Also check altitude
        alt_msg = bus.wait_for(["GLOBAL_POSITION_INT"], lambda m: True, timeout_s=0.5)
        if alt_msg and alt_msg.relative_alt < 300:  # < 0.3m
            landed = True
            break

    if landed:
        print("      Vehicle landed — disarming")
    else:
        print("      Land timeout — force disarming (60s elapsed)")

    try:
        _disarm(bus)
    except Exception as e:
        print(f"      Disarm failed (may already be disarmed): {e}")

    # Disable fence after test to leave SITL in a clean state
    try:
        mf.set_param(bus, "FENCE_ENABLE", 0, timeout_s=5.0)
        mf.clear_fence(bus, timeout_s=5.0)
    except Exception:
        pass

    bus.stop()
    return runner.done()


if __name__ == "__main__":
    sys.exit(main())
