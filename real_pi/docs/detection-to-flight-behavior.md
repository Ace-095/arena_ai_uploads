# What the drone commands after a YOLO detection

## Short answer

During the Pi's active GUIDED search, a qualified cue **interrupts the current
search leg/sweep and enters TRACK**. It does not intentionally finish all the
waypoints and inspect the detection later. A YOLO box is not necessarily a
qualified cue: filters, consecutive observations, preset strictness and mission
phase all matter.

| Situation | Decision |
|---|---|
| Before sprayer/manual mission trigger | Detection alone does not trigger GUIDED takeover; AUTO mission is not redirected by the box. |
| Active search, `day` / `far` / `aggressive` / `bench` | Filtered boxes can qualify before decoding; by default need two consecutive qualifying processed frames from the same camera. |
| Active search, `dark` / `kabaddi` / `low` | An undecoded box does not qualify. The box must decode on qualifying frames. |
| Bottom-camera cue, off center | Replace the search target with a projected ground target; lateral alignment before descent. |
| Bottom-camera cue, centered (<1.5m calculated offset) | Enter APPROACH, stepping through configured `mission.approach_stair_m`, without commanding a climb. |
| Front-camera cue | Bearing-only: command yaw toward the box and a 3m forward target at current altitude, limited by the fence. No reliable range is inferred from a front box. |
| Payload consensus reached (default three matching decodes, with configured miss tolerance) | TRANSMIT/hold then configured `post_action` (currently RTL); approaching the printed marker is not mandatory once the payload is known. |
| Accepted cue becomes stale | Exit tracking and return to grid search. Current code rebuilds the grid; it does not bookmark the exact interrupted waypoint. |
| Low battery or requested abort | Command RTL instead of chasing. |
| Sustained departure from GUIDED | Stop sending tracking targets via the same external-mode guard used by search. |

Preset values are loaded from `mission_pi/config.yaml`; changing strictness in
that table changes which column of behavior applies. `detector.cue_frames: 2`
means **processed frames**, not two seconds and not two raw camera frames.
The cue counter confirms observations per camera, not object identity: it is not
an object-association tracker and cannot eliminate every repeatable false positive.
Strict decoding is the stronger ground-use gate.

## Timing: not zero-delay flight

Detection/decoding happens in camera workers. Search reads their accepted cue at
its next checkpoint. Existing loop timing remains:

- Grid travel has a nominal 1s sleep between position/control iterations, plus
  position reads, vision work and other processing. It does not wait for arrival
  at the waypoint when a fresh cue is present at the checkpoint.
- Yaw sweep checks between steps. Default 30° / 20°/s + 1s settling is about
  **2.5s per step**, before additional API/telemetry overhead. A fleeting cue can
  expire while the loop is busy; detection is not a hard-real-time interrupt.
- TRACK updates nominally every 0.5s plus processing/FC waits. Cues age out at
  2s in search and 2.5s in tracking; a negative observation also clears an
  accepted cue once it is older than 2s. Brief occlusions can therefore still use
  the last accepted target. Pending streaks reset after a miss or >2s gap.
- A cue before the initial takeover altitude hold finishes does not bypass that
  hold. Detection alone never bypasses the mission trigger.
- Inference and difficult full-frame decode can dominate timing on CPU fallback.
  Detector throughput, Wi-Fi preview FPS and actual vehicle response are different
  measurements. None of the bench tests establishes real vehicle latency.

## Test and fixes

`tests/test_detection_response.py` runs the bundled ONNX QR model on a synthetic
1280×720 image. In this run its QR box was approximately `(810,298,156,144)` with
confidence **0.881**. The test replays that real model output through the actual
`Mission._worker` filter/decode/cue logic. It uses controlled decode failures for
bbox-only cases and real OpenCV decoding for successful payload cases.

A virtual clock and recording FC then exercise real search and tracking functions.
No MAVLink connection, SITL dynamics, physical camera or drone is involved. The
harness stops at the first tracking command, so “moves toward” here means a
**command was produced**, not that a vehicle was observed moving.

Observed trace with a right-of-center bottom-camera QR at 15m altitude, heading
north:

```
SWEEP_GRID: command first waypoint
  qualifying YOLO frames arrive while that leg is unfinished
TRACK: command target 3.79m east, -0.16m north, altitude 15m
  no second waypoint and no descent command before lateral alignment
```

The first 16-case run found **five failing cases** on the previous code:

1. `cue_frames` only controlled an event/log, not the steering gate.
2. A miss or stale gap did not correctly reset the pending streak.
3. Cameras shared a global hit count rather than independent streaks.
4. Switching to a strict preset retained an old permissive, undecoded cue.
5. TRACK lacked the external-mode guard that the search loops already used.

Fixes now gate accepted cues by per-camera consecutive frames, reset pending
streaks on misses, clear old cues on preset switches, reject results from frames
started before a preset/reset change, and check the mode guard in tracking.
Decoded boxes qualify in permissive presets too while awaiting payload consensus.
Removed-camera cues are ignored instead of dereferencing a missing camera.

A further confirmed-payload-path check found `rcfg` was uninitialized when the
payload had already been relayed. The exception was swallowed and skipped the
intended hold. It is now initialized on both paths; the test verifies hold then
RTL with no duplicate relay.

**Final result: 23/23 behavior tests passed.** Coverage includes all seven presets,
configurable cue count, current-leg interruption, yaw interruption at a checkpoint,
front yaw/forward command, bottom centering/descent, fence clamping, rejected
boxes, lost/removed-camera cues, strict decode gating, real payload consensus,
pre-trigger behavior, abort/battery, preset-change races and external-mode guard.

Run in the Pi project's vision-enabled venv:

```bash
cd real_pi/mission_pi
python tests/test_detection_response.py
```

Required packages include the production vision dependencies plus `qrcode` and
`pillow` for the synthetic test image. The test does not need an FC or issue any
network flight commands.

## Additional wire-level verification

The follow-up [movement wire tests](movement-wire-verification.md) exercise the real FCLink sender and receive actual MAVLink2 packets over UDP, rather than recording method calls. They also correct the front-camera yaw-direction flag and command-ACK race. Neither suite establishes physical vehicle displacement.

## Hardware verification still required

Before trusting autonomous pursuit, verify camera role/orientation/FOV, heading,
focus, fence enforcement parameters and projected target direction with motors
inhibited/props removed, then use an appropriate simulator and controlled flight
procedure. Fence clamping in these tests is geometry validation, not proof of
Pixhawk fence enforcement or a general no-crash guarantee.
