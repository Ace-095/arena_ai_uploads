# Are movement commands actually transmitted?

**Verified:** real mission tracking code calls real `FCLink` methods, which
serialize and transmit MAVLink2 packets. A separate UDP receiver parses those
packets and checks their targets, frames, masks, coordinates, altitude and yaw.

**Not verified here:** your physical Pixhawk's USB reception, ArduPilot's actual
acceptance of each target, actuator response or aircraft displacement. The receiver
is a deterministic packet/telemetry fixture, **not** ArduPilot SITL or flight
physics. Receipt of bytes is not proof of movement.

## Exact path

```
qualified camera cue
    Mission._track_and_approach()
        FCLink.goto_global(latitude, longitude, relative_altitude)
            FCLink._send()
                pymavlink SET_POSITION_TARGET_GLOBAL_INT
                    actual loopback UDP datagram -> independent parser

front-camera turn:
    FCLink.condition_yaw(...)
        FCLink._cmd_long(MAV_CMD_CONDITION_YAW, ...)
            COMMAND_LONG -> receiver
```

These are **position targets**, not raw roll/pitch commands, RC overrides or motor
PWM. The FC's navigation/attitude controllers decide how to reach the commanded
location. A bottom-camera target left of the image is converted to a geographical
target left of the aircraft, accounting for heading. A front-camera cue commands
a yaw bearing plus a forward geographical target (not a guaranteed completed
turn before translation).

Packet fields verified independently:

- Sender system/component: `51 / 191` (companion computer).
- Destination: fixture FC `17 / 1`, deliberately not the default system ID 1.
- Movement message: `SET_POSITION_TARGET_GLOBAL_INT`.
- Frame: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` = 6. Altitude is metres **above home**,
  not negative NED-down, not absolute MSL and not measured terrain height.
- Position-only mask: `0x0DF8` = 3576. Position fields active; velocity,
  acceleration, yaw and yaw-rate ignored; acceleration-as-force flag clear.
- Latitude and longitude scaled by `1e7`, including negative latitude.
- Turn message: `COMMAND_LONG` / `MAV_CMD_CONDITION_YAW`; angle in degrees,
  turn-rate 25 deg/s for front tracking, direction -1 left / +1 right, absolute
  heading flag 0. Relative sweep commands retain flag 1 and clockwise steps.

The position/frame/mask encoding follows ArduPilot's
[2](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
Guided command documentation. ArduPilot handles the position-target message
separately from the `COMMAND_LONG` acknowledgement protocol: do not equate a
successful socket write with an accepted or reached position target.

## Received packet examples

Bottom camera, upright image, 15m relative altitude, heading north:

| QR offset | Receiver decoded geographical target |
|---|---|
| right | 3.96m east, current altitude 15m |
| left | 3.97m west, current altitude 15m |
| forward/image up | 3.95m north, current altitude 15m |
| back/image down | 3.96m south, current altitude 15m |
| centered | same horizontal location, first descent target 12m |

All four lateral directions are tested at headings **0°, 90°, 180°, 270°** (16
combinations). For example, facing east: forward targets east and right targets
south. Small asymmetries are integer coordinate quantization, not distinct speeds.

Front camera, heading north:

- Left detection: heading **345.22°**, direction **-1**, forward target ~3m at 15m.
- Right detection: heading **14.78°**, direction **+1**, forward target ~3m at 15m.
- Wrap-around cases near 0°/360° preserve the correct short turn direction.

These tests assume the declared camera geometry and `rotation_deg: 0`. They do
not calibrate your mounting, optical distortion, camera-to-body alignment, GPS,
heading or terrain height. Those must be verified independently.

## Issues found by the wire tests

1. The previous mask was 4088 (`0x0FF8`), which unnecessarily included FORCE_SET.
   It is now the documented position-only mask 3576. Because acceleration fields
   were ignored, the old flag does **not** establish that earlier firmware rejected
   all movement commands; this is a standards-alignment correction.
2. Front tracking always used the default clockwise yaw direction, including for
   left-hand detections. It now computes the signed shortest heading difference
   and explicitly requests counterclockwise for left turns.
3. `_cmd_long` subscribed to ACKs after sending. A sufficiently fast ACK could be
   lost. It now subscribes first, matches the command/FC/recipient, waits past
   IN_PROGRESS for a final result, and removes its subscription on exit.

The initial regression run failed on the mask and fast ACK. After fixing the mask,
left-turn cases still failed; those passed only after the yaw-direction fix.

## What the 10 test methods exercise

`mission_pi/tests/test_movement_wire.py` starts a loopback UDP telemetry peer and
uses the production FC receive loop for heartbeats, position and battery messages.
`goto_global`, `condition_yaw`, `_send`, encoding and UDP transport are not mocked
in the movement cases. Only the end-of-iteration sleep is replaced, to stop after
one tracking command instead of running an indefinite flight loop.

The fixture acknowledges mode commands for receive-loop testing. Separate tests
inject an immediate parsed ACK during `_send`, a foreign ACK, a wrong-recipient
ACK, IN_PROGRESS then DENIED, and missing ACKs. Fixture acknowledgement is **not
an ArduPilot acceptance result**. Tracking yaw remains deliberately fire-and-forget;
this work does not add an acknowledgement requirement to every yaw update.

Run safely (no hardware connection, no arm/takeoff commands):

```bash
cd real_pi/mission_pi
python tests/test_movement_wire.py
```

This adds wire-level proof to `test_detection_response.py`, which already tests
the real YOLO output through the detection-to-steering decision with a recording FC.
The new test begins with a qualified synthetic box rather than re-running YOLO.

## What remains before claiming the drone moves correctly

1. Verify physical link/system ID and telemetry on the intended Pixhawk, disarmed
   and with props removed. These bench checks do not validate airborne motion.
2. Verify Armed/GUIDED state, position estimate, heading, home, fence enforcement
   and camera orientation. A received target can be ignored when the FC cannot
   or must not execute it.
3. Run ArduPilot SITL with live position telemetry to confirm target acceptance
   and displacement; then perform the appropriate controlled hardware validation.
   Do not bypass pre-arm checks or fence protections merely to make a target work.

No physical flight-readiness, zero-latency or no-crash guarantee follows from these
software tests.
