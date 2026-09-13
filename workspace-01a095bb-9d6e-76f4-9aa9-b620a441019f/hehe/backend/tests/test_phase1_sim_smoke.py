"""
test_phase1_sim_smoke.py — Phase 1 acceptance smoke test.

This is the standard pre-flight sanity check for every phase from Phase 1
onward. Run it before starting any new development session to confirm the
simulation environment and vision pipeline basics are still intact.

Tests (all tiers run independently — each can pass without the others):

    Tier A — MavlinkBus (requires mock FC or SITL running):
        A1. MavlinkBus connects and receives a heartbeat.
        A2. target_system and target_component are both populated (non-zero/non-None).

    Tier B — SyntheticQRSource (pure Python, no simulator needed):
        B1. get_frame() returns a correctly shaped uint8 BGR numpy array.
        B2. pyzbar can decode the embedded QR and the payload matches config.

    Tier C — GazeboCameraSource (requires ROS 2 + Gazebo, dev machine only):
        C1. GazeboCameraSource.start() succeeds without ImportError.
        C2. get_frame() returns a non-empty, correctly shaped frame within
            timeout. (Skipped automatically if ROS 2 is not available.)

Run:
    cd <repo>/backend
    python3 tests/test_phase1_sim_smoke.py                  # Tier B only (always works)
    python3 tests/test_phase1_sim_smoke.py --with-mavlink    # Tier A (needs mock or SITL)
    python3 tests/test_phase1_sim_smoke.py --with-gazebo     # Tiers A + B + C

Environment variables:
    MAVLINK_DEVICE   — MAVLink device string (default: udp:127.0.0.1:14550)
    CAMERA_SOURCE    — Ignored by this test (we exercise sources directly)
    SYNTHETIC_QR_PAYLOAD  — QR payload to embed and verify (from config)
    ROS2_CAMERA_TOPIC     — ROS 2 topic for GazeboCameraSource test (from config)

Exit code: 0 = all run tests passed. Non-zero = at least one failure.
"""

import argparse
import os
import sys
import threading
import time

# Allow running from either repo root or backend/ directory
_BACKEND_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND_DIR not in sys.path:
    sys.path.insert(0, _BACKEND_DIR)
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)

os.environ.setdefault("MAVLINK20", "1")


# ---------------------------------------------------------------------------
# Simple test runner
# ---------------------------------------------------------------------------

class TestRunner:
    def __init__(self):
        self._results: list[tuple[str, str, str]] = []  # (name, status, detail)

    def run(self, name: str, fn, *, skip_if: bool = False, skip_reason: str = ""):
        if skip_if:
            self._results.append((name, "SKIP", skip_reason))
            print(f"SKIP  {name}  ({skip_reason})")
            return

        try:
            fn()
            self._results.append((name, "PASS", ""))
            print(f"PASS  {name}")
        except AssertionError as e:
            detail = str(e) or "assertion failed"
            self._results.append((name, "FAIL", detail))
            print(f"FAIL  {name}: {detail}")
        except Exception as e:
            detail = f"{type(e).__name__}: {e}"
            self._results.append((name, "FAIL", detail))
            print(f"FAIL  {name}: {detail}")

    def summary(self) -> int:
        """Print summary and return exit code (0 = all passed/skipped)."""
        passed = sum(1 for _, s, _ in self._results if s == "PASS")
        failed = sum(1 for _, s, _ in self._results if s == "FAIL")
        skipped = sum(1 for _, s, _ in self._results if s == "SKIP")
        print()
        print(f"{'─'*60}")
        print(f"Results: {passed} passed, {failed} failed, {skipped} skipped")
        if failed == 0:
            print("Phase 1 smoke tests: ALL PASSED ✓")
        else:
            failures = [n for n, s, _ in self._results if s == "FAIL"]
            print(f"Phase 1 smoke tests: FAILED — {failures}")
        return 1 if failed > 0 else 0


# ---------------------------------------------------------------------------
# Tier A — MavlinkBus tests
# ---------------------------------------------------------------------------

def tier_a_mavlink(runner: TestRunner, mavlink_device: str, use_mock: bool) -> None:
    """Run Tier A tests. If use_mock=True, spins up the mock FC internally."""
    from mavlink_bus import MavlinkBus

    mock_stop: threading.Event | None = None
    mock_thread: threading.Thread | None = None
    mock_port = 14561  # Use a different port from the existing protocol test

    if use_mock:
        from mock_fc import run_mock_fc
        mock_stop = threading.Event()
        mock_thread = threading.Thread(
            target=run_mock_fc,
            args=(f"udp:0.0.0.0:{mock_port}", mock_stop),
            daemon=True,
        )
        mock_thread.start()
        mavlink_device = f"udpout:127.0.0.1:{mock_port}"
        time.sleep(1.0)  # Give mock FC time to bind

    bus: MavlinkBus | None = None

    try:
        def a1_connect():
            nonlocal bus
            bus = MavlinkBus(mavlink_device, 115200, timeout_s=10.0)
            bus.start()
            assert bus.conn is not None, "bus.conn is None after start()"

        def a2_heartbeat_ids():
            assert bus is not None, "bus not created"
            ts = bus.conn.target_system
            tc = bus.conn.target_component
            assert ts is not None and ts != 0, (
                f"target_system={ts!r} — heartbeat not received or IDs not populated. "
                "Confirm SITL or mock FC is running and MAVLink link is active."
            )
            assert tc is not None, f"target_component={tc!r}"

        runner.run("A1: MavlinkBus connects and starts", a1_connect)
        runner.run("A2: target_system + target_component populated", a2_heartbeat_ids)

    finally:
        if bus is not None:
            try:
                bus.stop()
            except Exception:
                pass
        if mock_stop is not None:
            mock_stop.set()


# ---------------------------------------------------------------------------
# Tier B — SyntheticQRSource tests
# ---------------------------------------------------------------------------

def tier_b_synthetic(runner: TestRunner) -> None:
    """Run Tier B tests. Pure Python — no sim, no hardware needed."""
    from config import config
    from camera_source import SyntheticQRSource

    source: SyntheticQRSource | None = None

    def b1_frame_shape():
        nonlocal source
        source = SyntheticQRSource(
            payload=config.SYNTHETIC_QR_PAYLOAD,
            width=640,
            height=480,
            fps=30.0,
            pan_speed=0.0,       # static QR for decode test
            zoom_amplitude=0.0,  # no zoom — keep QR at consistent size
        )
        source.start()
        frame = source.get_frame()

        import numpy as np
        assert isinstance(frame, np.ndarray), f"frame is {type(frame)}, expected ndarray"
        assert frame.dtype == np.uint8, f"dtype={frame.dtype}, expected uint8"
        assert frame.ndim == 3, f"ndim={frame.ndim}, expected 3 (H, W, C)"
        assert frame.shape == (480, 640, 3), f"shape={frame.shape}, expected (480, 640, 3)"
        assert frame.size > 0, "frame is empty (zero bytes)"

    def b2_qr_decode():
        assert source is not None, "B1 must pass before B2"
        # Get a fresh static frame
        frame = source.get_frame()

        # Try pyzbar first (fastest)
        decoded_payload = None
        try:
            from pyzbar.pyzbar import decode as zbar_decode
            import cv2
            results = zbar_decode(frame)
            if results:
                decoded_payload = results[0].data.decode("utf-8", errors="replace")
        except ImportError:
            pass

        # Fall back to OpenCV WeChatQRCode if pyzbar not available
        if decoded_payload is None:
            try:
                import cv2
                detector = cv2.wechat_qrcode_WeChatQRCode()
                texts, _ = detector.detectAndDecode(frame)
                if texts:
                    decoded_payload = texts[0]
            except Exception:
                pass

        assert decoded_payload is not None, (
            "QR decode failed — pyzbar and WeChatQRCode both returned empty results. "
            "Check that pyzbar + libzbar0 are installed (sudo apt install libzbar0 && "
            "pip install pyzbar), or that opencv-contrib-python is installed."
        )
        assert decoded_payload == config.SYNTHETIC_QR_PAYLOAD, (
            f"Decoded QR payload '{decoded_payload}' != "
            f"config.SYNTHETIC_QR_PAYLOAD '{config.SYNTHETIC_QR_PAYLOAD}'. "
            "The synthetic source is generating a different payload than configured."
        )

    try:
        runner.run("B1: SyntheticQRSource frame shape correct", b1_frame_shape)
        runner.run("B2: SyntheticQRSource QR payload decodes correctly", b2_qr_decode)
    finally:
        if source is not None:
            try:
                source.stop()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Tier C — GazeboCameraSource tests
# ---------------------------------------------------------------------------

def tier_c_gazebo(runner: TestRunner) -> None:
    """Run Tier C tests. Requires ROS 2 + Gazebo + gz-ros-bridge."""
    # Check ROS 2 availability first — skip gracefully if absent
    ros2_available = False
    try:
        import rclpy  # noqa: F401
        ros2_available = True
    except ImportError:
        pass

    def c1_import_and_start():
        from camera_source import GazeboCameraSource
        src = GazeboCameraSource(timeout_s=2.0)
        src.start()
        src.stop()

    def c2_frame_received():
        from config import config
        from camera_source import GazeboCameraSource
        import numpy as np

        src = GazeboCameraSource(timeout_s=5.0)
        src.start()
        try:
            frame = src.get_frame()
            assert isinstance(frame, np.ndarray), f"frame is {type(frame)}"
            assert frame.ndim == 3 and frame.shape[2] == 3, (
                f"Expected (H, W, 3) array, got shape {frame.shape}"
            )
            assert frame.size > 0, "Received empty frame"
        finally:
            src.stop()

    runner.run(
        "C1: GazeboCameraSource starts without ImportError",
        c1_import_and_start,
        skip_if=not ros2_available,
        skip_reason="rclpy not available (ROS 2 not installed on this machine)",
    )
    runner.run(
        "C2: GazeboCameraSource receives a non-empty frame",
        c2_frame_received,
        skip_if=not ros2_available,
        skip_reason="rclpy not available (ROS 2 not installed on this machine)",
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 1 smoke test")
    parser.add_argument(
        "--with-mavlink",
        action="store_true",
        help="Run Tier A (MavlinkBus) tests using the internal mock FC",
    )
    parser.add_argument(
        "--mavlink-device",
        default=os.environ.get("MAVLINK_DEVICE", "udp:127.0.0.1:14550"),
        help="MAVLink device string (only used with --with-mavlink if not using mock)",
    )
    parser.add_argument(
        "--no-mock",
        action="store_true",
        help="With --with-mavlink: connect to real SITL instead of the internal mock FC",
    )
    parser.add_argument(
        "--with-gazebo",
        action="store_true",
        help="Run Tier C (GazeboCameraSource) tests (requires ROS 2 + Gazebo)",
    )
    args = parser.parse_args()

    runner = TestRunner()

    # Always run Tier B (pure Python, no dependencies beyond requirements.txt)
    print("=" * 60)
    print("Tier B — SyntheticQRSource (pure Python)")
    print("=" * 60)
    tier_b_synthetic(runner)

    if args.with_mavlink:
        print()
        print("=" * 60)
        use_mock = not args.no_mock
        tier_label = "mock FC" if use_mock else args.mavlink_device
        print(f"Tier A — MavlinkBus ({tier_label})")
        print("=" * 60)
        tier_a_mavlink(runner, args.mavlink_device, use_mock=use_mock)

    if args.with_gazebo:
        print()
        print("=" * 60)
        print("Tier C — GazeboCameraSource (requires ROS 2 + Gazebo)")
        print("=" * 60)
        tier_c_gazebo(runner)

    return runner.summary()


if __name__ == "__main__":
    sys.exit(main())
