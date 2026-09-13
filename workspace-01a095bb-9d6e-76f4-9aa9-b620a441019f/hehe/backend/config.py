"""
Runtime configuration for the geofence service.

INVARIANT: nothing in this file, or anywhere else in this service, may contain
a hardcoded venue coordinate, bounding box, or fence polygon. The venue is
unknown until 1-2 days before competition. Every location-derived value must
come from a runtime request (the browser POSTing a polygon) or an environment
variable set at deploy time — never a constant baked into source.

Phase 1 additions: camera abstraction mode + simulation-only topic name.
"""

import os


class Config:
    # --- MAVLink connection to the Pixhawk ---
    # For real hardware: prefer a stable /dev/serial/by-id/... path over
    # /dev/ttyACM0, which can renumber across reboots or if other USB
    # devices are plugged in. Confirm the exact by-id path on your actual
    # Pi + Pixhawk wiring — this is a NEED TEST item, not something to assume.
    MAVLINK_DEVICE: str = os.environ.get("MAVLINK_DEVICE", "/dev/ttyACM0")
    MAVLINK_BAUD: int = int(os.environ.get("MAVLINK_BAUD", "115200"))

    # For bench/SITL testing without hardware, set:
    #   MAVLINK_DEVICE=udp:127.0.0.1:14550
    # and run ArduPilot SITL (sim_vehicle.py) — see README.

    # Timeout (seconds) waiting for a MAVLink heartbeat / message response
    # before treating the flight controller as unreachable.
    MAVLINK_TIMEOUT_S: float = float(os.environ.get("MAVLINK_TIMEOUT_S", "5.0"))

    # Fence behavior on breach. 1 = RTL is the conservative default for this
    # mission (per your team's stated RTL-after-mission behavior). Confirm
    # this matches AC_Fence's current FENCE_ACTION enum on your firmware
    # version before competition — NEED TEST.
    FENCE_ACTION: int = int(os.environ.get("FENCE_ACTION", "1"))

    # Where the Pi persists its own copy of the last-uploaded fence (for UI
    # convenience/status only — the Pixhawk's own fence storage is the
    # safety-authoritative copy and survives Pi restarts independently).
    STATE_FILE: str = os.environ.get(
        "GEOFENCE_STATE_FILE",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), ".fence_state.json"),
    )

    # HTTP server bind
    HOST: str = os.environ.get("GEOFENCE_HOST", "0.0.0.0")
    PORT: int = int(os.environ.get("GEOFENCE_PORT", "8000"))

    # Telemetry broadcast rates (Hz)
    TELEMETRY_POSITION_HZ: float = float(os.environ.get("TELEMETRY_POSITION_HZ", "5.0"))
    TELEMETRY_STATS_HZ: float = float(os.environ.get("TELEMETRY_STATS_HZ", "1.0"))

    # Offline tile MBTiles database path (Phase 4)
    MBTILES_PATH: str = os.environ.get("MBTILES_PATH", "field.mbtiles")

    # --- Camera abstraction (Phase 1) ---
    # Selects which CameraSource implementation is constructed by get_camera_source().
    # Valid values: "synthetic" | "gazebo" | "picamera2"
    #   synthetic  — generates QR codes in pure Python; no simulator required.
    #                Use for rapid iteration on vision pipeline logic.
    #   gazebo     — subscribes to a ROS 2 topic bridged from Gazebo (dev machine only).
    #                Requires ROS 2 Humble + gz-ros-bridge running on the same machine.
    #                Never set this on the Pi — ROS 2 is not installed there.
    #   picamera2  — real hardware; built in Phase 9. Raises NotImplementedError until then.
    CAMERA_SOURCE: str = os.environ.get("CAMERA_SOURCE", "synthetic")

    # ROS 2 image topic name bridged from Gazebo for GazeboCameraSource.
    # NEED TEST: the exact topic name depends on the SDF world file (1b) and
    # the gz-ros-bridge configuration. Set this after launching the sim and
    # running `ros2 topic list` to find the actual topic name.
    # Example: "/iris/camera/image_raw"
    ROS2_CAMERA_TOPIC: str = os.environ.get("ROS2_CAMERA_TOPIC", "/iris/camera/image_raw")

    # Camera topics for dual-camera setup (Gazebo / ROS 2 tier)
    ROS2_CAM1_TOPIC: str = os.environ.get("ROS2_CAM1_TOPIC", os.environ.get("ROS2_CAMERA_TOPIC", "/iris/camera/image_raw"))
    ROS2_CAM2_TOPIC: str = os.environ.get("ROS2_CAM2_TOPIC", "/iris/camera2/image_raw")

    # Camera WebRTC streaming configuration (Phase 5)
    WEBRTC_TARGET_FPS: float = float(os.environ.get("WEBRTC_TARGET_FPS", "30.0"))
    DEFAULT_CAMERA_MODE: str = os.environ.get("DEFAULT_CAMERA_MODE", "dual")

    # QR payload format — CONFIRMED: the drone generates QR codes that encode
    # only numbers. "Just the numbers" means the payload is numeric data only.
    # The exact field format (e.g. "42", "123456", lat/lon as integer degrees,
    # or a competition-assigned target ID) is NEED TEAM INPUT — keep this as a
    # config value so it can be updated without a code change. The default below
    # is a plausible numeric stand-in for simulation testing; replace it with the
    # actual competition format once the exact spec is known.
    SYNTHETIC_QR_PAYLOAD: str = os.environ.get("SYNTHETIC_QR_PAYLOAD", "42")

    # --- Mission Planner result transmission ---
    # STATUSTEXT has no ACK in the MAVLink spec — a single-shot send has
    # no way to know if Mission Planner actually received it, and if the
    # telemetry radio link happens to be down at that exact moment, the
    # message is gone with no way to recover it (unlike the browser UI
    # path, which replays the last known result to any newly-connecting
    # WebSocket client). Mitigated by resending periodically for a
    # bounded window, so a brief radio dropout has a real chance of
    # landing one of the retries.
    STATUSTEXT_RESEND_INTERVAL_S: float = float(os.environ.get("STATUSTEXT_RESEND_INTERVAL_S", "2.0"))
    STATUSTEXT_RESEND_WINDOW_S: float = float(os.environ.get("STATUSTEXT_RESEND_WINDOW_S", "15.0"))

    # --- Search Algorithm (Phase 6) ---
    # SEARCH_ALTITUDE_M — the planned search-sweep altitude is ~14-15m per
    # the project's actual hardware discussion (T-Motor Air 2216/920KV
    # class, camera FOV math for Camera 1 Wide). Still fully overridable
    # via env var, not a hardcoded assumption baked into logic anywhere.
    SEARCH_ALTITUDE_M: float = float(os.environ.get("SEARCH_ALTITUDE_M", "14.0"))
    SEARCH_SPEED_MS: float = float(os.environ.get("SEARCH_SPEED_MS", "1.5"))
    SEARCH_STEP_M: float = float(os.environ.get("SEARCH_STEP_M", "2.0"))
    SEARCH_WAYPOINT_RADIUS_M: float = float(os.environ.get("SEARCH_WAYPOINT_RADIUS_M", "1.0"))
    SEARCH_STRATEGY: str = os.environ.get("SEARCH_STRATEGY", "expanding_square")

    # --- Approach/decode descent (Phase 6/8) ---
    # Deliberately NOT a single fixed "decode altitude" — the actual
    # altitude decode succeeds at depends on the real lens/QR-size
    # combination (confirmed by GSD math: a 6mm lens needs ~9m for
    # 4px/module, a 16mm lens could manage from ~25m — too far apart to
    # guess a single number). Instead the FSM descends step by step,
    # checking live QR decode consensus after each step, and stops as
    # soon as decode succeeds OR the safety floor below is reached.
    APPROACH_DESCENT_STEP_M: float = float(os.environ.get("APPROACH_DESCENT_STEP_M", "1.5"))
    # Hard safety floor — never descended below regardless of decode
    # status. NEED TEAM INPUT: set based on the actual cache's physical
    # height and your comfort margin above it, once known.
    APPROACH_MIN_ALTITUDE_M: float = float(os.environ.get("APPROACH_MIN_ALTITUDE_M", "1.5"))
    # How long to hold at each altitude step before descending further —
    # must be long enough for the QR consensus buffer (default requires 3
    # consecutive good frames) to actually reach a decision at that step.
    APPROACH_STEP_HOLD_S: float = float(os.environ.get("APPROACH_STEP_HOLD_S", "2.5"))
    # Overall descent timeout — a safety backstop distinct from the
    # per-step hold time, in case something upstream never resolves.
    APPROACH_MAX_DURATION_S: float = float(os.environ.get("APPROACH_MAX_DURATION_S", "45.0"))

    # --- QR / Cache Pipeline (Phase 7) ---
    QR_CONSENSUS_STREAK: int = int(os.environ.get("QR_CONSENSUS_STREAK", "3"))
    CACHE_DETECTOR_STRATEGY: str = os.environ.get("CACHE_DETECTOR_STRATEGY", "heuristic")

    # --- FSM / Mission Integration (Phase 8) ---
    FSM_PREFLIGHT_TIMEOUT_S: float = float(os.environ.get("FSM_PREFLIGHT_TIMEOUT_S", "10.0"))
    FSM_TAKEOFF_ALTITUDE_M: float = float(os.environ.get("FSM_TAKEOFF_ALTITUDE_M", "5.0"))
    FSM_QR_SWEEP_TIMEOUT_S: float = float(os.environ.get("FSM_QR_SWEEP_TIMEOUT_S", "120.0"))
    FSM_MIN_BATTERY_PCT: float = float(os.environ.get("FSM_MIN_BATTERY_PCT", "20.0"))
    FSM_LINK_LOSS_TIMEOUT_S: float = float(os.environ.get("FSM_LINK_LOSS_TIMEOUT_S", "5.0"))
    FSM_DRY_RUN: bool = os.environ.get("FSM_DRY_RUN", "False").lower() in ("true", "1", "yes")


config = Config()




