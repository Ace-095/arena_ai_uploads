#!/usr/bin/env python3
"""Cross-check our frozen MAVLink constants against pymavlink (authoritative).

Skips the live checks when pymavlink is unavailable (e.g. field laptop).
Exists because MAV_MISSION_TYPE_FENCE=8 (real value: 1) shipped and the mock
shared the bug — wire-format tests alone cannot catch a wrong constant.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bridge"))
import _mavlink_v2 as M

# Verified 2026-09-13 vs pymavlink v20 ardupilotmega COPTER_MODE_* (+ POSITION
# / OF_LOITER per ArduPilot ModeNumber, absent from that dialect snapshot).
EXPECTED_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                  5: "LOITER", 6: "RTL", 7: "CIRCLE", 8: "POSITION", 9: "LAND",
                  10: "OF_LOITER", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
                  16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
                  20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
                  24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
                  28: "TURTLE"}


def main():
    assert M.ARDU_COPTER_MODES == EXPECTED_MODES, "mode map drifted — re-verify vs pymavlink"
    assert (M.MODE_STABILIZE, M.MODE_GUIDED, M.MODE_AUTO, M.MODE_RTL, M.MODE_LAND) == (0, 4, 3, 6, 9)
    try:
        from pymavlink.dialects.v20 import ardupilotmega as P
    except Exception as e:
        print("MAVCONST-SKIP (no pymavlink: %r); static mode-map check passed" % (e,))
        return
    checks = [
        ("MISSION_TYPE_MISSION", M.MAV_MISSION_TYPE_MISSION, P.MAV_MISSION_TYPE_MISSION),
        ("MISSION_TYPE_FENCE", M.MAV_MISSION_TYPE_FENCE, P.MAV_MISSION_TYPE_FENCE),
        ("DO_SET_MODE", M.MAV_CMD_DO_SET_MODE, P.MAV_CMD_DO_SET_MODE),
        ("REQUEST_MESSAGE", M.MAV_CMD_REQUEST_MESSAGE, P.MAV_CMD_REQUEST_MESSAGE),
        ("NAV_TAKEOFF", M.MAV_CMD_NAV_TAKEOFF, P.MAV_CMD_NAV_TAKEOFF),
        ("NAV_RTL", M.MAV_CMD_NAV_RETURN_TO_LAUNCH, P.MAV_CMD_NAV_RETURN_TO_LAUNCH),
        ("NAV_LAND", M.MAV_CMD_NAV_LAND, P.MAV_CMD_NAV_LAND),
        ("DO_SPRAYER", M.MAV_CMD_DO_SPRAYER, P.MAV_CMD_DO_SPRAYER),
        ("FENCE_INCLUSION", M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
         P.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION),
        ("FRAME_GLOBAL", M.MAV_FRAME_GLOBAL, P.MAV_FRAME_GLOBAL),
        ("MISSION_ACCEPTED", M.MAV_MISSION_ACCEPTED, P.MAV_MISSION_ACCEPTED),
        ("PARAM_REAL32", M.MAV_PARAM_TYPE_REAL32, P.MAV_PARAM_TYPE_REAL32),
        ("SEVERITY_INFO", M.MAV_SEVERITY_INFO, P.MAV_SEVERITY_INFO),
        ("TYPE_GCS", M.MAV_TYPE_GCS, P.MAV_TYPE_GCS),
        ("AUTOPILOT_INVALID", M.MAV_AUTOPILOT_INVALID, P.MAV_AUTOPILOT_INVALID),
        ("STATE_STANDBY", M.MAV_STATE_STANDBY, P.MAV_STATE_STANDBY),
        ("STATE_ACTIVE", M.MAV_STATE_ACTIVE, P.MAV_STATE_ACTIVE),
        ("ARM_FLAG", M.MAV_MODE_FLAG_SAFETY_ARMED, P.MAV_MODE_FLAG_SAFETY_ARMED),
        ("GPS_ORIGIN_ID", M.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, P.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN),
    ]
    bad = [(n, a, b) for n, a, b in checks if a != b]
    for k, v in vars(P).items():  # every pymavlink COPTER_MODE_* must agree with us
        if k.startswith("COPTER_MODE_") and not k.endswith("ENUM_END"):
            if v in M.ARDU_COPTER_MODES and M.ARDU_COPTER_MODES[v] != k.replace("COPTER_MODE_", ""):
                bad.append((k, M.ARDU_COPTER_MODES[v], k))
    for k, v in vars(P).items():  # mission-result + command-ack names we display
        if k.startswith("MAV_MISSION_") and k not in ("MAV_MISSION_TYPE_MISSION", "MAV_MISSION_TYPE_FENCE",
                                                     "MAV_MISSION_TYPE_RALLY", "MAV_MISSION_TYPE_ALL") and isinstance(v, int):
            short = k.replace("MAV_MISSION_", "")
            if v in M.MAV_MISSION_RESULT and M.MAV_MISSION_RESULT[v] != short:
                bad.append((k, M.MAV_MISSION_RESULT[v], short))
        if k.startswith("MAV_RESULT_") and isinstance(v, int):
            short = k.replace("MAV_RESULT_", "")
            if v in M.COMMAND_ACK_RESULT and M.COMMAND_ACK_RESULT[v] != short:
                bad.append((k, M.COMMAND_ACK_RESULT[v], short))
    assert not bad, bad
    print("MAVCONST-PASS (%d value checks + mode/result-name cross-checks vs pymavlink)" % len(checks))


if __name__ == "__main__":
    main()
