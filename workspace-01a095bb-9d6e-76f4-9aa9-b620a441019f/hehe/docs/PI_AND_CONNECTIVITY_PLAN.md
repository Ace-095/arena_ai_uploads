# Raspberry Pi Software & Pi ↔ Laptop Connectivity — Phase Plan

**Scope:** this document covers only two things — what runs *on* the Pi
(object detection, QR decode, camera handling, mission FSM) and how the
Pi talks to the laptop/ground-station (network topology, telemetry,
video, fence upload). It does not repeat `IMPLEMENTATION_PLAN.md`'s
broader milestone structure — treat this as a status-accurate drill-down
into the subset of that plan that's specifically Pi/connectivity-related.

**Status legend used throughout:**
- ✅ VERIFIED — built and confirmed working by actually running it
- ⚠️ PARTIAL — built, but with a known gap or unverified piece
- 🚧 NOT STARTED — stubbed or not yet written
- 🔬 NEED TEST — can't be confirmed without real hardware/Gazebo

---

## Phase A — Pi-side core services (MAVLink, fence, telemetry)

**Status: mostly complete, verified against a mock flight controller.**

| Component | File | Status |
|---|---|---|
| Single MAVLink connection, pub/sub dispatch | `mavlink_bus.py` | ✅ VERIFIED — heartbeat handshake, target_system/component both populate correctly |
| Fence upload/download/param set | `mavlink_fence.py` | ✅ VERIFIED — subscribe-before-send race fixed, re-ran protocol test twice with 6/6 passing both times |
| Fence status cache | `state.py` | ✅ VERIFIED — module-level singleton API |
| Telemetry broadcast (position/stats/events) | `telemetry.py` | ✅ VERIFIED — rate-limiting bug fixed and confirmed |
| FENCE_TYPE bitmask handling | `mavlink_fence.py::enable_fence` | ⚠️ PARTIAL — reads current value and ORs in the polygon bit rather than guessing; 🔬 NEED TEST against your actual Pixhawk 6C firmware version |

**Remaining work:** none structurally — this layer is in good shape.
The only open item is firmware-version confirmation, which needs
SITL or real hardware, not more code.

---

## Phase B — Object detection & QR decode pipeline

**Status: partial. The individual pieces work; the "detection" half is
currently a placeholder, not real object detection.**

| Component | File | Status |
|---|---|---|
| QR generation for testing | `camera_source.py::SyntheticQRSource` | ✅ VERIFIED — real decodable QR via cv2.QRCodeEncoder, confirmed pixel values |
| QR decode + multi-frame consensus | `qr_pipeline.py::ConsensusBuffer` | ✅ VERIFIED — miss-tolerance fix confirmed: single bad frame no longer wipes an in-progress streak |
| Stage 1 "find the cache" detector | `qr_pipeline.py::PlaceholderColorShapeDetector` | ⚠️ PARTIAL — this is a color/contour heuristic, **not** the Hailo-accelerated detector the hardware spec calls for. Fixed the fake-fallback-bbox bug, but it's still not real object detection. |
| Hailo AI HAT integration | *(not written)* | 🚧 NOT STARTED — deliberately deferred, see `HARDWARE_SETUP.md`'s NEED TEST note: exact HailoRT API depends on confirming Hailo-8 vs 8L on the physical part |

**What "done" looks like for this phase:** `PlaceholderColorShapeDetector`
gets replaced with a real Hailo-accelerated model behind the same
`detect_cache(frame) -> Optional[BBox]` interface — this is why that
interface was worth getting right early, the swap should not require
touching `search_algorithm.py` or `fsm.py` at all.

**Immediate next step, doesn't require hardware:** decide what the
actual Stage 1 model is going to be (a fine-tuned lightweight detector
trained on your real Intelligence Cache appearance) and get *something*
trained, even on synthetic/Gazebo-rendered images, so there's a real
model to convert for Hailo once hardware arrives — training a model is
the long-lead-time item here, not the Hailo integration code itself.

---

## Phase C — Camera pipeline (capture, switch, stream to laptop)

**Status: wiring is now correct; hardware capture is still a stub.**

| Component | File | Status |
|---|---|---|
| Camera abstraction | `camera_source.py::CameraSource` | ✅ VERIFIED — `SyntheticQRSource` fully working, `GazeboCameraSource` correct (deferred rclpy import confirmed by reading, not yet run — 🔬 NEED TEST, requires ROS2+Gazebo) |
| Dual/single switching + WebRTC | `camera_manager.py` | ⚠️ PARTIAL — `get_source()` accessor added and verified; actual WebRTC publish path (aiortc) not yet confirmed under load |
| FSM-driven camera switching | `fsm.py::_switch_camera` | ✅ VERIFIED (just fixed) — SEARCH now switches to cam1, CACHE_DETECTED→APPROACH switches to cam2. This was the "Camera 2 never gets used" bug from last session — confirmed fixed by grep + code read, not yet exercised in a live FSM run |
| Real hardware capture | `camera_source.py::Picamera2Source` | 🚧 NOT STARTED (intentional stub) — correctly deferred to Phase F below, real hardware needed to write this correctly |

**Remaining work before hardware arrives:** run a live FSM cycle end to
end with `CAMERA_SOURCE=synthetic` and confirm in logs that the camera
actually switches at the right transitions — this is a cheap test that
doesn't need Gazebo or hardware, just hasn't been done yet.

---

## Phase D — Mission FSM (orchestration)

**Status: structurally complete, camera-switch gap just closed, not yet
run as a full end-to-end cycle.**

`fsm.py` states: `INIT → PRE_FLIGHT_CHECK → ARM → TAKEOFF → SEARCH →
CACHE_DETECTED → APPROACH → DECODE → CONFIRMED → TRANSMIT_RESULT → RTL →
LAND → MISSION_COMPLETE`, with `FAILSAFE` reachable from any state.

| Check | Status |
|---|---|
| Gates ARM on fence-armable | ✅ VERIFIED by code read |
| Camera switches at SEARCH and APPROACH | ✅ VERIFIED (this session's fix) |
| Consensus reset on camera switch | ✅ VERIFIED — added as part of the same fix, prevents a stale streak from one camera's view leaking into the other's |
| Fail-closed on camera-switch failure | ✅ VERIFIED by code read — transitions to FAILSAFE rather than continuing silently |
| Full state-machine run, start to finish | 🚧 NOT DONE — this is the actual next milestone: run it against the mock/SITL and watch it walk every state in order |

**This is the single most valuable next test to run** — everything
above has been verified piece-by-piece, but never all together in one
live sequence.

---

## Phase E — Pi ↔ Laptop connectivity (the full picture)

```
┌─────────────────────────┐        Wi-Fi Router        ┌──────────────────────────┐
│      Raspberry Pi 5       │◄──── (local LAN only) ────►│   Laptop / Ground Station │
│                          │                             │                          │
│  FastAPI app (app.py)    │                             │  Browser (index.html)    │
│   ├─ POST /api/geofence  │◄──── HTTP ─────────────────►│   ├─ draw fence           │
│   ├─ GET  /api/geofence  │                             │   ├─ view telemetry       │
│   ├─ WS   /ws/telemetry  │◄──── WebSocket ────────────►│   ├─ view camera feeds    │
│   └─ WS   /ws/webrtc/*   │◄──── WebRTC (LAN) ─────────►│   └─ Mission Planner       │
│         │                │                             │        (separate MAVLink  │
│         │ USB            │                             │         link — see below) │
│         ▼                │                             │                          │
│    Pixhawk 6C            │                             │                          │
└─────────────────────────┘                             └──────────────────────────┘
```

| Link | Protocol | Status |
|---|---|---|
| Browser → Pi: fence upload | HTTP POST/GET, JSON | ✅ VERIFIED |
| Pi → Browser: live telemetry | WebSocket, tiered rate (2-5Hz position, 1Hz stats, event-driven) | ✅ VERIFIED — rate-limiting bug fixed |
| Pi → Browser: camera feeds | WebRTC over LAN (aiortc), signaled over WS | ⚠️ PARTIAL — endpoints exist (`/ws/webrtc/{camera_id}`), not yet load-tested with real video |
| Pi ↔ Pixhawk | MAVLink over USB | ✅ VERIFIED against mock; 🔬 NEED TEST against real hardware |
| Mission Planner ↔ Pixhawk | **RESOLVED**: separate telemetry radio pair, plugged directly into the Pixhawk 6C's TELEM port — independent of the Pi's USB link entirely. No MAVLink forwarder needed. | ✅ Decision made |
| Pi → Browser: Pi system health (CPU/RAM/disk/temp) | WebSocket, `system` channel, ~0.2Hz | ✅ VERIFIED — real values confirmed (CPU temp reads `null` off-Pi where no thermal zone exists, expected) |
| Pi → Browser: internet status (LTE dongle) | WebSocket, same `system` channel | ✅ VERIFIED — purely informational, never gates any FSM state |
| Browser → Pi: offline map tiles | HTTP, MBTiles-backed | 🚧 NOT STARTED (Phase 4/M3 in the main plan) |

**Mission Planner decision, resolved:** the telemetry radio connects
directly to the Pixhawk 6C's own TELEM port, completely independent of
the Pi's USB connection. This is the simpler of the two options — no
`mavlink-router` or forwarding software needed on the Pi at all. Update
your BOM to include a telemetry radio pair if you haven't already
sourced one.

---

## LTE dongle / internet status

Per the original 3-way comms architecture (telemetry / router / dongle),
the dongle leg is now implemented — but stays strictly non-safety-critical,
matching the project's core invariant (MAVLink over USB, browser UI over
local LAN, neither depends on internet availability).

- `backend/network_monitor.py::check_internet_status()` — a lightweight
  TCP connection attempt to a well-known host, broadcast over the same
  `system` WebSocket channel as Pi health stats. Drives the UI's
  "Internet" indicator only — nothing reads this value for any flight or
  mission decision.
- **Dongle network setup itself is OS-level, not app code** — see
  `docs/HARDWARE_SETUP.md`'s new LTE Dongle section for the actual
  NetworkManager configuration steps.

---

## Phase F — Hardware bring-up (Pi 5 specific)

Already fully documented in `docs/HARDWARE_SETUP.md` — not repeating it
here. Summary of what plugs into this plan specifically:

1. AI HAT+ setup → unblocks Phase B's real Hailo detector.
2. Camera enablement (`rpicam-hello --list-cameras`) → unblocks
   `Picamera2Source` in Phase C.
3. Pixhawk USB (`/dev/serial/by-id/...`) → unblocks Phase A/D's
   hardware-tier acceptance tests.
4. `--system-site-packages` venv → required specifically because of
   `picamera2`, see the earlier requirements.txt discussion.

---

## What to actually do next, in order

1. **Run a full FSM cycle against the mock/SITL** (Phase D) — the
   highest-value remaining test, and it's free (no hardware, no Gazebo).
2. **Decide the Mission Planner link** (Phase E) — this gates a
   purchasing decision (telemetry radio or not), so resolve it before
   it blocks Phase F.
3. **Start sourcing/training a real Stage 1 detector model** (Phase B) —
   long lead time, doesn't need to wait for hardware.
4. **Load-test the WebRTC camera path** (Phase C) with `CAMERA_SOURCE=synthetic`
   feeding both virtual cameras at once, to catch encode-load problems
   before they show up as a surprise on real hardware.
5. Everything else in Phase F waits for hardware to physically arrive.
