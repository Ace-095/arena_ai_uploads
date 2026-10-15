# MARK 9 — Project Verification and Bench-Test Report

**Project:** Pi 5 QR detection, Pixhawk telemetry and guided-search companion system  
**Report:** MARK 9  
**Date:** 23 September 2026  
**Repository branch:** `arena/01a0af20-arena-ai-uploads`  
**Latest repository commit:** `a83f462`  
**Status:** Electronics bench validation in progress; no motors, ESCs, RC receiver or flight performed

---

## 1. Executive summary

The project implements a Raspberry Pi 5 companion computer for camera-based QR detection and Pixhawk/MAVLink mission assistance. The system contains:

- CSI/USB camera auto-detection and role assignment.
- Pi web UI and MJPEG camera streaming.
- YOLO QR detection with ONNX/CPU fallback.
- Hailo AI HAT support through a compiled HEF model.
- QR decoding, confidence/size/aspect filtering and decode consensus.
- Mission-state safety guards and trigger/lost-cue handling.
- Pixhawk USB/MAVLink auto-detection.
- MAVLink telemetry, QR relay and store-and-forward support.
- Guided position-target and yaw command generation.
- Fence/parameter/UI command handling.
- Bench and wire-level verification tests.

The software movement path has been verified at the MAVLink packet level. The Pixhawk USB heartbeat path has also been verified on the physical table setup. The Hailo AI HAT now loads and executes the HEF at approximately 1 ms per inference, but the project adapter still requires correction for the installed HailoRT NMS/inference API before Hailo can be promoted to the production mission detector.

**No flight claim is made.** No motors, ESCs, propellers, RC receiver or airborne vehicle have been tested in this report.

---

## 2. Hardware currently available

The current physical bench contains:

- Raspberry Pi 5.
- Hailo AI HAT.
- Pixhawk/Pixhawk1 flight controller.
- GPS connected to Pixhawk.
- Vivo Y71 phone running DroidCam over USB as a temporary camera.
- USB connection between Pi and Pixhawk.

Not yet connected for the current report:

- Motors.
- ESCs.
- Propellers.
- Flight battery/power module.
- RC receiver.
- Complete aircraft frame.
- Physical telemetry radio/GCS receiver path.

---

## 3. Architecture and data paths

### 3.1 Detection path

```text
Camera
  -> frame capture
  -> optional 3x3 tiled preprocessing
  -> YOLO detector
  -> QR decoder
  -> confidence/geometry/decode filters
  -> consecutive confirmation/consensus
  -> mission response and relay
```

### 3.2 Pixhawk path

```text
Pixhawk USB/TELEM
  -> FCLink MAVLink reader
  -> heartbeat, mode, battery, GPS and home telemetry
  -> mission safety guards
  -> position/yaw target serializers
  -> MAVLink transmission
```

### 3.3 UI path

```text
Pi HTTP/WebSocket server
  -> health and camera APIs
  -> MJPEG camera streams
  -> telemetry/event log
  -> preset and camera controls
```

### 3.4 QR telemetry fallback

When the Pi remains powered and connected to Pixhawk, QR status can be sent through the Pixhawk telemetry path even when the Pi is no longer connected to the Wi-Fi router:

```text
Pi camera/detector -> Pixhawk MAVLink -> telemetry link -> Mission Planner/GCS
```

If the Pi is powered off, it cannot decode or transmit a new QR message. Pixhawk safety behavior must remain independent of the Pi.

---

## 4. Detection and QR behavior

### 4.1 Detection models

Available model files currently include:

```text
models/qr_yolov8n.onnx
models/qr_yolov8n.pt
models/qr_yolov8n.hef
```

- ONNX runs through the CPU detector path.
- PT is a heavier PyTorch/Ultralytics fallback.
- HEF is the Hailo-compiled artifact for the AI HAT.

### 4.2 Tiled detection

Tiling is a preprocessing strategy, not a different model. A frame is divided into a 3x3 grid and YOLO is run on each tile. Results are then combined before QR filtering and decoding.

The current tested concept is:

```yaml
tile_bottom: true
tile_grid: [3, 3]
tile_overlap: 0.25
```

Tiling increases the effective size of distant QR codes but multiplies inference work. With Hailo, nine tile inferences are practical from a throughput perspective; with CPU ONNX, they are substantially more expensive.

### 4.3 QR safety rule

A bounding box without a valid decoded payload is not sufficient for a ground action. The decode-required presets and confirmation streak protect against false boxes from textured backgrounds, window grills and unrelated shapes.

The project supports:

- Confidence threshold.
- Minimum box side.
- Aspect-ratio filtering.
- Optional decode-required cue filtering.
- Consecutive cue frames.
- Decode consensus.
- Miss/gap reset behavior.
- Preset changes with state reset.
- Store-and-forward relay.

### 4.4 Presets

The configuration contains named operating presets, including day, far, dark, kabaddi, low, aggressive and bench-style profiles. Presets control confidence, tiling, decode cadence and cue strictness. They are intended to remain easy to change in `config.yaml` and through the UI API.

---

## 5. Pixhawk and MAVLink verification

### 5.1 Physical Pixhawk USB result

The physical Pixhawk was detected by Linux as:

```text
/dev/ttyACM0
/dev/serial/by-id/usb-ArduPilot_Pixhawk1_...
```

The direct probe produced:

```text
mode: STABILIZE
armed: False
link: True
```

This proves the Pi receives a real Pixhawk heartbeat over USB.

At the time of the test, position/home/battery were unavailable:

```text
pos: no GLOBAL_POSITION_INT from FC
home: no HOME_POSITION from FC
batt: pct=None, v=None
```

This can occur when GPS has no fix or when the Pixhawk is powered over USB without a flight battery/power module.

### 5.2 USB auto-detection correction

The mission startup code previously skipped USB auto-detection when `fc.conn` was empty and immediately tried SITL addresses such as:

```text
tcp:127.0.0.1:5762
tcp:127.0.0.1:5760
udp:127.0.0.1:14550
```

The correction makes physical USB auto-detection the first candidate when:

```yaml
fc:
  conn: ""
```

The fix is recorded in commit `a83f462`.

### 5.3 Movement wire verification

The dedicated movement wire suite contains 10 passing tests. It exercises:

- Real mission steering.
- Real FCLink serializers.
- Real pymavlink packet transmission.
- UDP loopback receiver.
- Independent packet parsing.
- Production FC receive/ACK loop.
- Four bottom-camera movement directions.
- Four vehicle headings.
- Centered descent target.
- Front-camera yaw direction.
- Heading wraparound.
- Raw coordinate and field checks.
- Relative yaw sweep.
- ACK source/recipient/command filtering.
- Immediate ACK race handling.
- IN_PROGRESS followed by final result.
- Missing ACK timeout.

Verified position-target fields include:

```text
sender system/component: 51/191
receiver target system/component: 17/1
frame: GLOBAL_RELATIVE_ALT_INT (6)
position-only mask: 3576 / 0x0DF8
latitude/longitude: integer degrees x 1e7
altitude: metres above home
velocity/acceleration: ignored/zero
```

Example decoded targets from a north-facing vehicle at approximately 15 m:

| Image target | Geographic result |
|---|---|
| Right | approximately 3.96 m east |
| Left | approximately 3.97 m west |
| Up/forward | approximately 3.95 m north |
| Down/back | approximately 3.96 m south |

Front-camera yaw tests verified left targets use negative yaw direction and right targets positive yaw direction, including heading wraparound.

**Boundary:** these tests prove emitted and received MAVLink packets. They do not prove ArduPilot acceptance, vehicle displacement or flight physics.

---

## 6. Hailo AI HAT verification

### 6.1 What passed

The Pi reports the Hailo runtime and the HEF exists:

```text
hailo_platform import OK
models/qr_yolov8n.hef exists
```

`hailortcli parse-hef` reports:

```text
Architecture: HAILO8
Network group: qr_yolov8n
Input: UINT8 NHWC 640x640x3
Output: HAILO NMS BY CLASS
Classes: 1
Maximum boxes per class: 100
```

The HEF-only test executed successfully at approximately:

```text
0.8–1.1 ms per inference
```

The 3x3 bench loop also executed nine Hailo calls per frame and maintained continuous camera capture for hundreds of frames.

### 6.2 What is not yet passing

The project Hailo adapter initially used older HailoRT APIs. Compatibility fixes were required for:

- `HailoStreamInterface` versus `StreamInterface`.
- Hailo network configuration parameters.
- Input/output stream parameter creation.
- Explicit inference pipeline activation.

After activation, the error changed to:

```text
[h ail o-debug] output_shapes=[None]
```

and the current Hailo result parser still returns zero project boxes. The installed HailoRT output is an NMS result object/list rather than the NumPy tensor layouts currently handled by the parser. Manual context handling also produced bus/segmentation faults on exit, so safe lifecycle cleanup is still required.

The phone QR decoder independently decoded `hello`, but the Hailo detector returned no project boxes. Therefore Hailo is **not yet safe to make the primary mission detector**.

### 6.3 Correct next Hailo work

The adapter must:

1. Use the installed HailoRT context-manager pattern.
2. Read the actual Hailo NMS result structure.
3. Convert one-class NMS detections into project `BBox` objects.
4. Apply tile-coordinate offsets when using 3x3 tiling.
5. Close inference/network resources safely.
6. Add Hailo-specific tests with mocked NMS structures.
7. Re-run full detection and mission regression tests.

Until then, ONNX remains the validated fallback detector.

---

## 7. Camera and streaming verification

### 7.1 CSI/USB auto-detection

The Pi camera probe reported no CSI camera currently attached:

```text
no CSI cameras detected
```

The Vivo Y71 DroidCam USB device was assigned explicitly as:

```text
cam2 -> usb /dev/video0 (bottom)
```

### 7.2 DroidCam

Initial DroidCam tests showed intermittent frame timeouts. After restarting the service, continuous capture was achieved in the Hailo tiled bench:

```text
hundreds of good frames
```

The test also demonstrated that QR decoding can work from the phone camera.

The camera must be tested separately from detection because a timeout can otherwise be mistaken for a model failure.

### 7.3 UI streaming

The Pi web server is configured for LAN serving on port 8000. The valid laptop URL observed on the bench is:

```text
http://192.168.29.21:8000
```

`192.168.29.255` should not be used as the normal browser URL because it is the broadcast address.

The UI should be tested for:

- Health endpoint.
- Camera API.
- Continuous MJPEG frames.
- Preset changes.
- Stream watchdog/restart behavior.
- No delay escalation under tiled inference.

---

## 8. Fallback and failure behavior

### 8.1 Pi power loss

If the Pi shuts down:

- Pi camera/detection stops.
- Pi cannot send new QR messages.
- Pixhawk must continue independent flight-controller failsafes.
- No Pi-generated RTL can be expected after power loss.

### 8.2 Router/network loss

If the Pi remains powered and the Pixhawk link remains connected, QR messages can still be sent over the Pixhawk telemetry path to the GCS. The Pi UI/network path will be unavailable.

### 8.3 Persistent relay

Store-and-forward can preserve messages while the Pi remains powered and a relay endpoint is temporarily unavailable. It cannot recover a message that was never persisted before an abrupt power loss.

### 8.4 Pixhawk safety ownership

The Pixhawk, not the Pi, must own:

- Arming checks.
- RC failsafe.
- Battery failsafe.
- GPS/EKF failsafe.
- Fence action.
- RTL/LAND behavior.
- Stabilization and motor control.

The Pi may request actions, but it must not be the only safety system.

---

## 9. Test record

### Passing repository-level verification

Previous full regression on the project branch:

```text
29 Python test files passed
Movement wire suite: 10/10 passed
```

The movement wire suite was run with a synthetic FC receiver/ACK source, not real ArduPilot SITL or physical flight.

### Passing physical bench results

```text
Pixhawk USB enumeration: PASS
Pixhawk heartbeat: PASS
armed=False: PASS
STABILIZE mode readable: PASS
Hailo device/runtime: PASS
HEF loading: PASS
Hailo raw inference execution: PASS
Phone camera continuous capture: PASS after DroidCam restart
QR decoder reads test payload hello: PASS
3x3 Hailo invocation loop: PASS
```

### Not yet passing or not yet performed

```text
Hailo NMS result adapter: FAILING
Hailo boxes through production detector: NOT VERIFIED
Safe Hailo teardown: NEEDS FIX
Full Hailo + mission integration: NOT VERIFIED
Physical Pixhawk telemetry-to-GCS QR relay: NOT YET TESTED
Router-loss QR relay: NOT YET TESTED
RC receiver/failsafe: NOT AVAILABLE
Motor/ESC test: NOT AVAILABLE
Arming: NOT PERFORMED
Flight: NOT PERFORMED
```

---

## 10. Recommended next sequence

### Before connecting motors

1. Reboot the Pi after Hailo bus/segmentation faults.
2. Keep ONNX as the active mission detector.
3. Correct the Hailo NMS adapter and resource cleanup.
4. Add a production Hailo QR test that requires a valid `BBox`.
5. Run all 29 regression files again.
6. Test Pi-to-Pixhawk QR status delivery with Mission Planner.
7. Disconnect the router and repeat the telemetry relay test.
8. Shut down the Pi and confirm Pixhawk remains alive and disarmed.

### When motors and receiver are available

1. Inspect all wiring with power disconnected.
2. Connect receiver and verify channel values.
3. Verify receiver failsafe with propellers removed.
4. Verify Pixhawk arming checks.
5. Test ESC/motor outputs with propellers removed.
6. Confirm motor order and direction.
7. Test mode changes without takeoff.
8. Test RTL only in a controlled, safe environment.
9. Perform SITL or restrained validation before any autonomous flight.

---

## 11. Final assessment

The project has a substantial working software base and has passed real Pixhawk USB heartbeat testing, real MAVLink packet transmission tests, camera capture tests, QR decoding tests and raw Hailo HEF execution tests.

The highest-priority remaining software issue is the HailoRT NMS adapter and safe Hailo resource lifecycle. Hailo hardware is working, but production Hailo detection is not yet proven because the current adapter reports no project boxes and can crash during teardown.

The highest-priority remaining operational test is the physical QR telemetry relay from Pi through Pixhawk to Mission Planner with the router disconnected.

This report therefore classifies the project as:

> **MARK 9 — bench-capable, Pixhawk-link verified, MAVLink wire verified, QR decode verified, Hailo execution verified, production Hailo detection and physical flight not yet verified.**

No component of this report authorizes arming or flight without the remaining safety and hardware tests.
