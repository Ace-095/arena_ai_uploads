# Repository Analysis: `Ace-095/hehe` — "Geofence Pipeline" Drone System

> Full analysis of every file in https://github.com/Ace-095/hehe (121 files, ~15.3k lines of code/docs, single squashed commit).
> Purpose: this is the reference doc we extract building blocks from to build our own thing.
> Cloned to `/home/user/hehe`.

---

## 1. WHAT THIS REPO IS

An **autonomous quadcopter mission system** for a competition (referenced as "ADDC 2027" in sim files). The mission in one sentence:

> Browser draws a geofence polygon → uploaded to a Pixhawk (ArduCopter) over MAVLink → confirmed by readback → the drone autonomously arms, takes off, flies a search sweep looking for a QR-code "Intelligence Cache" with two cameras, descends until it can decode the QR, transmits the payload to Mission Planner + browser UI, then RTLs and lands.

**Hardware platform:** Raspberry Pi 4/5 (64-bit) + Pixhawk 6C + 2 camera modules (belly-mounted: cam1 = wide search cam, e.g. Pi Cam 3 / imx708; cam2 = motorized-focus decode cam, e.g. Arducam IMX477 12MP) + optional Hailo AI HAT + LTE dongle (informational only) + telemetry radio to laptop.

**Core architectural stance — fail-closed everywhere:**
- No fence uploaded & confirmed on the FC → system reports `armable: false`. **No fallback area, no default bbox, no hardcoded venue coordinate anywhere.** The venue is unknown until 1–2 days before competition.
- The Pixhawk's own fence enforcement is the *safety-authoritative* boundary (runs independently of the Pi). Software-side point-in-polygon checks are convenience only, never safety.
- Camera missing/failing → refuse to start or FAILSAFE, never run blind.
- Internet status is explicitly *informational only* — never gates arming/flight.

**Milestone structure (from docs):**
| Phase | Content | Status in repo |
|---|---|---|
| M1/1 | Geofence upload pipeline | ✅ done |
| 2 | Fence SITL validation, ENU frame | ✅ done |
| 3 | Telemetry WebSocket backbone | ✅ done |
| 4 | Offline MBTiles map caching + meter grid | ✅ done |
| 5 | Camera pipeline + WebRTC streaming | ✅ done (hardware stub) |
| 6 | Search algorithm (sweeps) | ✅ done |
| 7 | QR detection/decode pipeline | ✅ done (heuristic detector, Hailo stub) |
| 8 | Full mission FSM | ✅ implemented in code, not yet run end-to-end (STATE.md paused right before) |
| 9 | Real hardware bring-up | 🚧 not started |

---

## 2. COMPLETE FILE TREE (121 files)

```
hehe/
├── readme.md                        # M1 geofence pipeline docs + API + verified-vs-need-test
├── requirements.txt                 # consolidated: fastapi, uvicorn, pymavlink, pydantic, psutil,
│                                    #   requests, numpy, opencv-contrib, pyzbar, aiortc, qrcode[pil], Pillow
├── .gitignore                       # venvs, pycache, .fence_state.json, *.mbtiles
│
├── backend/                         # THE CORE — FastAPI service running on the Pi
│   ├── app.py                       # 529L — FastAPI app, lifespan wiring, ALL HTTP/WS endpoints
│   ├── config.py                    # all config from env vars; search/approach/FSM tunables
│   ├── models.py                    # pydantic models (fence, search, QR, FSM, camera)
│   ├── state.py                     # FenceState JSON-persisted singleton (UI cache only)
│   ├── geo_utils.py                 # ENU conversion, area-weighted centroid, point-in-polygon
│   ├── mavlink_bus.py               # single MAVLink conn, 1 reader thread, pub/sub queues
│   ├── mavlink_fence.py             # fence upload/download/clear, params, STATUSTEXT
│   ├── telemetry.py                 # TelemetryHub: WS broadcast, channels, replay-on-reconnect
│   ├── camera_source.py             # CameraSource ABC + Synthetic/Gazebo/Picamera2 impls
│   ├── camera_manager.py            # dual-cam mode mgmt + WebRTC (aiortc) publishing
│   ├── qr_pipeline.py               # detect → decode → consensus buffer → notify search
│   ├── search_algorithm.py          # strategies (expanding square, lawnmower) + sweep executor
│   │                                #   + closed-loop approach descent
│   ├── fsm.py                       # 14-state mission finite state machine
│   ├── system_monitor.py            # psutil CPU/RAM/disk/temp/uptime + Hailo check
│   ├── network_monitor.py           # informational internet up/down (TCP to 8.8.8.8:53)
│   ├── _bootstrap_vendor.py         # adds _vendor/ to sys.path (air-gap dev)
│   ├── requirements.txt             # fastapi, uvicorn, pymavlink, pydantic
│   ├── _vendor/                     # DEV-ONLY shims for air-gapped machines:
│   │   ├── fastapi/__init__.py      #   minimal FastAPI (111L)
│   │   ├── fastapi/middleware/cors.py, staticfiles.py
│   │   ├── pydantic/__init__.py     #   minimal pydantic v2 (153L, dataclass-based)
│   │   └── pymavlink/mavutil.py     #   RAW MAVLink v2 over UDP sockets + struct (470L)
│   └── tests/                       # 18 test files + mock FC (details in §12)
│       ├── mock_fc.py               # mock flight controller, real MAVLink over UDP
│       ├── test_fence_protocol.py, test_fence_sitl.py, test_fence_breach_sitl.py
│       ├── test_fsm_protocol.py, test_fsm_dry_run.py, test_fsm_sitl.py
│       ├── test_search_algorithm.py, test_search_protocol.py, test_search_sitl.py
│       ├── test_qr_pipeline.py, test_qr_gazebo_sim.py
│       ├── test_camera_api.py, test_camera_manager.py, test_camera_sim_acceptance.py
│       ├── test_approach_descent.py, test_telemetry_ws.py
│       ├── test_phase1_sim_smoke.py # standard pre-flight sanity, run every session
│       └── test_standalone_full.py  # 702L, pure-logic, stdlib+numpy+cv2 only
│
├── frontend/                        # browser ground station (vanilla JS, no build step)
│   ├── index.html                   # dark cockpit UI: Leaflet map + left control panel
│   └── js/
│       ├── telemetry.js             # TelemetryClient — WS, envelopes, auto-reconnect
│       ├── tiles.js                 # TileManager — OSM↔offline toggle, ENU meter grid
│       └── cameras.js               # CameraStreamManager — WebRTC + img-poll fallback
│
├── docs/
│   ├── HARDWARE_SETUP.md            # Pi+Pixhawk+cams BOM, serial by-id, camera enumeration
│   ├── SIMULATION_SETUP.md          # SITL + Gazebo Harmonic + ardupilot_gazebo + ROS 2
│   ├── PI_AND_CONNECTIVITY_PLAN.md  # phase status tables w/ ✅⚠️🚧🔬 legend
│   └── BENCH_TEST_QUICKREF.md       # no-prop bench test checklist
│
├── sim/                             # Gazebo Harmonic simulation assets
│   ├── worlds/qr_search_field.sdf   # 60×60m field, iris+ArduPilot plugin, QR box @ (10,5,0.05)
│   ├── models/qr_box/               # 1×1×0.1m slab, QR texture on top face
│   │   ├── model.sdf, model.config
│   │   └── materials/textures/qr_code.png   # committed, decodable QR
│   └── textures/generate_qr_texture.py
│
├── tools/fetch_tiles.py             # OSM → MBTiles (TMS y conversion) offline tile downloader
│
├── gsd-template/                    # (empty dir)
├── .gemini/GEMINI.md                # GSD methodology rules for Gemini
├── .gsd/                            # "Get Shit Done" project-management layer
│   ├── STATE.md                     # current position (paused at Phase 8, 2026-09-03)
│   ├── JOURNAL.md                   # session log
│   ├── examples/                    # quick-reference, workflow-example, multi-wave, cross-platform
│   └── templates/                   # 23 markdown templates (state, plan, spec, roadmap, ...)
└── .agent/workflows/                # 26 GSD slash-command workflows (~4.9k lines)
    ├── map.md plan.md execute.md verify.md debug.md progress.md
    ├── pause.md resume.md add-todo.md check-todos.md help.md install.md
    ├── new-project.md new-milestone.md complete-milestone.md audit-milestone.md
    ├── add-phase.md insert-phase.md remove-phase.md discuss-phase.md
    ├── research-phase.md list-phase-assumptions.md plan-milestone-gaps.md
    ├── sprint.md update.md web-search.md whats-new.md
```

---

## 3. SYSTEM ARCHITECTURE

```
┌────────────────────────── BROWSER (laptop, same LAN) ─────────────────────────┐
│ index.html: Leaflet map (draw polygon) │ telemetry panel │ system panel │     │
│ camera video grid (WebRTC) │ fence annunciator │ focus slider                     │
│  js/telemetry.js (WS /ws/telemetry)  js/cameras.js (WS /ws/webrtc/camN → RTCP)  │
└───────────────┬───────────────────────────────┬───────────────────────────────┘
                │ REST (fetch) + WebSocket      │ WebRTC (after WS SDP offer/answer)
┌───────────────▼───────────────────────────────▼───────────────────────────────┐
│                    RASPBERRY PI — FastAPI service (backend/app.py)             │
│                                                                                │
│  HTTP:  /api/geofence*  /api/search/*  /api/camera/*  /api/qr/*  /api/fsm/*   │
│         /tiles/{z}/{x}/{y}.png  /health        WS: /ws/telemetry /ws/webrtc/N │
│                                                                                │
│  ┌────────────┐  ┌────────────────┐  ┌──────────────────┐  ┌────────────────┐ │
│  │ MavlinkBus │  │ TelemetryHub   │  │ CameraManager    │  │ MissionFSM     │ │
│  │ (1 conn,   │  │ (WS broadcast, │  │ (cam1/cam2,      │  │ (14 states,    │ │
│  │ 1 reader,  │  │  pos 5Hz,      │  │  modes, WebRTC   │  │  thread,       │ │
│  │ pub/sub)   │  │  stats 1Hz,    │  │  aiortc tracks)  │  │  failsafes)    │ │
│  └─────┬──────┘  │  sys 0.2Hz,    │  └──────┬───────────┘  └───────┬────────┘ │
│        │         │  events)       │         │                      │          │
│  ┌─────▼─────────┴─────────────┐  │  ┌──────▼──────────────────────▼────────┐ │
│  │ mavlink_fence.py            │  │  │ qr_pipeline: detect→decode→consensus │ │
│  │  upload/download/clear/     │  │  │ search_algorithm: sweeps + descent   │ │
│  │  params/enable/STATUSTEXT   │  │  └──────────────────────────────────────┘ │
│  └─────┬───────────────────────┘  │                                            │
└────────┼──────────────────────────┼────────────────────────────────────────────┘
         │ MAVLink v2 (USB serial,  │ CameraSource abstraction:
         │ 115200 or udp:14550 SITL)│  synthetic | gazebo (ROS2) | picamera2
         ▼                          ▼
┌─────────────────┐        ┌──────────────────┐
│  Pixhawk 6C     │        │  Cam 1 (wide,    │  Cam 2 (tele, motorized focus)
│  ArduCopter     │        │  imx708)         │
│  • FENCE stored │        └──────────────────┘
│    & enforced   │
│  • EKF origin   │
└─────────────────┘
```

**Single-connection discipline:** `MavlinkBus` is the *only* module that opens the MAVLink link or calls `recv_match()`. Fence ops, telemetry, guidance all subscribe to it. Enforced in code + comments.

**Threading model:** FastAPI asyncio loop (WS/HTTP) + several daemon threads: mavlink reader, search sweep, approach descent, QR pipeline, FSM loop, ROS2 spin (sim only). Threads communicate via queue.Queue + threading.Event + locks; status objects are pydantic models copied under lock.

---

## 4. BACKEND MODULE DETAIL

### 4.1 `config.py` — env-var-only configuration
`Config` class, instantiated as singleton `config`. Key values (default → env var):

| Setting | Default | Env var | Notes |
|---|---|---|---|
| MAVLink device | `/dev/ttyACM0` | `MAVLINK_DEVICE` | prefer `/dev/serial/by-id/...`; SITL: `udpout:127.0.0.1:14550` |
| baud / timeout | 115200 / 5.0s | `MAVLINK_BAUD`, `MAVLINK_TIMEOUT_S` | |
| FENCE_ACTION | 1 (RTL) | `FENCE_ACTION` | confirm enum on firmware |
| state file | `backend/.fence_state.json` | `GEOFENCE_STATE_FILE` | |
| bind | 0.0.0.0:8000 | `GEOFENCE_HOST/PORT` | |
| telemetry rates | pos 5Hz, stats 1Hz | `TELEMETRY_POSITION_HZ`, `TELEMETRY_STATS_HZ` | |
| MBTiles | `field.mbtiles` | `MBTILES_PATH` | |
| camera source | `synthetic` | `CAMERA_SOURCE` | `synthetic\|gazebo\|picamera2` |
| ROS2 topics | `/iris/camera/image_raw`, `.../camera2/...` | `ROS2_CAMERA_TOPIC`, `ROS2_CAM1/2_TOPIC` | sim only |
| WebRTC fps / default mode | 30 / `dual` | `WEBRTC_TARGET_FPS`, `DEFAULT_CAMERA_MODE` | |
| synthetic QR payload | `"42"` | `SYNTHETIC_QR_PAYLOAD` | numeric-only confirmed; exact format NEED TEAM INPUT |
| STATUSTEXT resend | 2.0s interval / 15.0s window | `STATUSTEXT_RESEND_INTERVAL_S`, `STATUSTEXT_RESEND_WINDOW_S` | no ACK exists for STATUSTEXT → bounded retry |
| search | alt 14m, speed 1.5 m/s, step 2m, wp radius 1m, `expanding_square` | `SEARCH_*` | |
| approach descent | step 1.5m, min alt 1.5m (floor), hold 2.5s/step, max 45s | `APPROACH_*` | deliberately NOT a fixed decode altitude (GSD math: 6mm lens ≈9m, 16mm ≈25m) |
| QR | consensus streak 3, detector `heuristic` | `QR_CONSENSUS_STREAK`, `CACHE_DETECTOR_STRATEGY` | `heuristic\|hailo` |
| FSM | preflight 10s, takeoff 5m, sweep 120s, batt 20%, link loss 5s, dry_run False | `FSM_*` | |

**INVARIANT comment (worth stealing verbatim as a team rule):** *nothing may contain a hardcoded venue coordinate, bounding box, or fence polygon; every location value comes from a runtime request or env var.*

### 4.2 `models.py` — pydantic v2
- `Vertex{lat: -90..90, lon: -180..180}`, `GeofenceUploadRequest` with validator: **3 ≤ vertices ≤ 255** (255 = ArduPilot's uint8 `vertex_count` storage limit).
- `ENUPoint{x,y}`, `Position{lat,lon,alt_msl?,alt_rel?}`.
- `GeofenceStatus{loaded, armable, reason, vertex_count, vertices_latlon[], vertices_enu[], centroid_enu?, max_radius_m?, origin_lat?, origin_lon?, fc_readback_matched?}`.
- `CameraModeRequest{mode: cam1|cam2|dual|none}`, `CameraFocusRequest{position 0..1}`.
- `SearchStartRequest{strategy?, dry_run, altitude_m?, step_m?}`, `SearchWaypoint{index, enu, lat, lon, altitude_m}`.
- `SearchStatus{state: idle|searching|holding|target_found|approaching|decoded|completed|error, dry_run, strategy, altitude_m, current_waypoint_idx, total_waypoints, reason, target?, waypoints[]}`.
- `BBox{x,y,w,h,confidence}`, `QRDetectionResult{bbox?, payload?, confirmed, streak, required_streak, timestamp}`.
- `FSMStatus{state (14 values), previous_state?, reason, armable, telemetry_link_healthy, payload?, timestamp}`.

### 4.3 `state.py` — fence state cache
`FenceState` over a JSON file + threading lock; module singleton with `get()/set(status)/clear()`. Corrupt file → "no fence" (never guess). **Explicitly a UI convenience cache, not a safety record** — the FC's fence survives Pi restarts independently.

### 4.4 `geo_utils.py` — local ENU + polygon math
Flat-earth equirectangular around an origin (intentional: sub-mm error at field scale, no projection lib on the Pi).
- `latlon_to_enu / enu_to_latlon` (EARTH_RADIUS_M = 6371000).
- `polygon_centroid` — **area-weighted** (cross-product formula, 6·area denominator), vertex-average fallback for degenerate shapes.
- `max_radius_from_point` — worst-case search range (replaces any hardcoded "search radius").
- `point_in_polygon` — ray casting; docstring warns it's a software convenience, **not** the safety boundary (FC fence is).

### 4.5 `mavlink_bus.py` — the connection owner
- Sets `MAVLINK20=1` at import (critical: MAVLink1 has no `mission_type` field → silent fence denial).
- `start()`: open conn → send GCS heartbeat → `wait_heartbeat()` (sets target_system/component) → start reader thread. Fails with `ConnectionError` if no FC heartbeat in timeout.
- **Threading:** one `_read_loop` thread calls `recv_match(blocking=True, timeout=0.2)` in a loop (the ONLY recv in the codebase). Each message is fanned out to subscriber queues keyed by message type + a `"*"` catch-all. Queues are `maxsize=50`, **non-blocking put, drop-oldest-when-full** so a slow consumer never blocks the reader.
- `subscribe(msg_types) -> Queue`, `unsubscribe(q)`.
- `wait_for(msg_types, predicate, timeout_s)` = subscribe, pull until predicate/timeout, unsubscribe.
- `send(send_fn, *args)` = thin lock around the socket write only; **never holds lock across waits** (would deadlock vs reader).
- Exceptions defined here (`FenceError`, `ConnectionError(FenceError)`) to avoid circular imports.

### 4.6 `mavlink_fence.py` — ArduPilot fence protocol
Verified against ArduPilot source (`GCS_MAVLink/MissionItemProtocol_Fence.cpp`) + mavlink.io docs, explicitly "not written from memory".

**Three documented gotchas (steal these):**
1. `MAVLINK20=1` must be set **before** any pymavlink import (else silent failure).
2. `vertex_count` (param1) must equal total polygon vertex count **in every** `MISSION_ITEM_INT`, not just the first (FC runs `ret.vertex_count = mission_item_int.param1` per item).
3. **Subscribe-before-send**: every function subscribes to the expected response *before* sending the triggering request. Naive send-then-subscribe has a real race — the bus doesn't buffer for non-existent subscribers, so on a fast link the reply arrives and is silently dropped. (They hit it non-deterministically in tests: two different failures across two runs on identical code.)

Functions (all stateless, take a bus):
- `request_ekf_origin()` — `MAV_CMD_REQUEST_MESSAGE` for `GPS_GLOBAL_ORIGIN` (msg 49), decode `/1e7`, reject zero origin. (NEED TEST: firmware support for on-demand requests.)
- `upload_polygon_fence()` — `MISSION_COUNT` (type FENCE=8) → loop receiving `MISSION_REQUEST_INT` per seq → reply `mission_item_int_send` with `MAV_FRAME_GLOBAL` + `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` (5001) + `param1=float(n)` → wait `MISSION_ACK`; reject anything but `MAV_MISSION_ACCEPTED`.
- `download_fence()` — `MISSION_REQUEST_LIST` → `MISSION_COUNT` → per-seq `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → final ACK. Preserves FC-driven cadence.
- `clear_fence()` — `MISSION_CLEAR_ALL` + ACK.
- `get_param/set_param()` — `PARAM_REQUEST_READ` (index −1 = by name) / `PARAM_SET` (REAL32) with confirmation + 0.01 tolerance check.
- `send_statustext()` — pads/truncates to 50 bytes (STATUSTEXT field width), severity 6 (INFO), fire-and-forget (no ACK in spec). This is what shows in **Mission Planner's** log (distinct from `telemetry.push_event` which only hits the browser).
- `enable_fence(action)` — sets `FENCE_ENABLE=1`, `FENCE_ACTION`, then reads `FENCE_TYPE` and **ORs in polygon bit 4** (`AC_FENCE_TYPE_POLYGON=4`; bits: ALT_MAX=1, CIRCLE=2, POLYGON=4, ALT_MIN=8) — safe no-op if firmware already inferred it.
- `build_geofence_status()` — lat/lon→ENU, centroid, max radius → full `GeofenceStatus`.

**Key MAVLink constants used** (also in vendor shim): HEARTBEAT 0, SYS_STATUS 1, PARAM_VALUE 22, GPS_RAW_INT 24, ATTITUDE 30, GLOBAL_POSITION_INT 33, MISSION_COUNT 44, MISSION_ACK 47, MISSION_REQUEST_INT 51, MISSION_ITEM_INT 73, GPS_GLOBAL_ORIGIN 49, COMMAND_LONG 76, COMMAND_ACK 77, SET_POSITION_TARGET_LOCAL_NED 84. Commands: REQUEST_MESSAGE 512, DO_SET_MODE 176, NAV_TAKEOFF 22, NAV_RETURN_TO_LAUNCH 20, NAV_LAND 21, COMPONENT_ARM_DISARM 400, FENCE_POLYGON_VERTEX_INCLUSION 5001. ArduCopter custom modes: 4=GUIDED, 6=RTL, 9=LAND.

### 4.7 `telemetry.py` — WebSocket hub
- Envelope: `{"channel": "position"|"stats"|"event"|"system", "t": ts, "data": {...}}`.
- `TelemetryHub`: set of WS clients + asyncio lock; `broadcast_envelope` caches `latest_messages[channel]` and **replays the latest per channel to every newly-connecting client** (reconnect = instant resync).
- Producer task: subscribes to `GLOBAL_POSITION_INT, ATTITUDE, SYS_STATUS, HEARTBEAT, GPS_RAW_INT` via the bus; runs queue.get in executor (thread-safe bridge to async); rate-limits: position 5Hz (includes lat/lon/alt/hdg/vx-vy-vz/attitude/gps/mode/armed), stats 1Hz (battery/mode/armed/gps).
- System task (0.2Hz): runs `system_monitor.get_system_stats()` + `network_monitor.check_internet_status()` in a **dedicated single-thread executor** — with a documented real bug history: default-executor threads can't be cancelled mid-syscall; `shutdown(wait=False)` crashed ~1/3 of runs with "terminate called without an active exception", `wait=True` + `cancel_futures=True` → 0/10.
- `push_event(type, value)` — thread-safe (works from any thread via `run_coroutine_threadsafe`).

### 4.8 `camera_source.py` — 3-tier camera abstraction
Contract (ABC `CameraSource`): `start()` idempotent; `get_frame()` → uint8 BGR (H,W,3) numpy, blocks until available, callers must not mutate; `stop()` idempotent; all thread-safe. **Every downstream consumer gets frames only through this.**

- `SyntheticQRSource` — renders a **real decodable QR** once (`cv2.QRCodeEncoder`, fallback `qrcode[pil]`), composites onto 100-grey background with sinusoidal pan (x/y at different frequencies) + sinusoidal zoom (±15% of quarter-frame base size) to test motion robustness. FPS-throttled.
- `GazeboCameraSource` — ROS 2 Humble subscription to a gz-ros-bridged topic; **lazy rclpy import inside start()** so the module can be imported on the Pi without ROS 2; spins a SingleThreadedExecutor in a daemon thread; frame event with timeout → RuntimeError with the exact `ros2 topic list` instruction.
- `detect_cameras()` — `Picamera2.global_camera_info()`; uses **list position, not the 'Num' field** (documented upstream quirk: Num sometimes absent on single-cam boards, issue #919); returns `[]` when picamera2 missing (caller fails closed).
- Model hints: `_SEARCH_CAM_MODEL_HINTS=("imx708",)`, `_DECODE_CAM_MODEL_HINTS=("imx477",)` — role assignment by sensor model, not index order (index order isn't stable); NEED TEST on exact strings.
- `Picamera2Source` — real hardware: `Picamera2(camera_num=i)`, video config 1920×1080 RGB888, `capture_array("main")`. NEED TEST: RGB vs BGR byte order on some libcamera versions, real FPS. `set_focus()` = NotImplementedError (Arducam IMX477 motorized focus needs vendor driver confirmed).
- `get_camera_source(override=None, **kwargs)` — factory.

### 4.9 `camera_manager.py` — modes + WebRTC
- `CameraManager({cam1, cam2})`: `set_active(cam1|cam2|dual|none)` starts/stops per-camera sources; **precedence rule: manual API override wins until the FSM calls set_active again**.
- `CameraStreamTrack(aiortc.VideoStreamTrack)` — `recv()` pulls frames in executor, converts via `av.VideoFrame.from_ndarray(bgr24)`; on error → blank 640×480 frame (keeps stream alive).
- `create_webrtc_offer_answer(cam_id, sdp)` — per-peer `RTCPeerConnection`, attach track, setRemote(offer) → createAnswer; tracks connections per cam; `connectionstatechange` cleanup on failed/closed.
- `set_focus(pos)` — fans out to any source that implements it; logs no-op for non-hardware. `get_frame(cam)` — lazy-start if that cam is active in current mode.
- aiortc/av imports are optional (`AIORTC_AVAILABLE` flag) — degrades to non-WebRTC.

### 4.10 `qr_pipeline.py` — detect → decode → consensus
- Stage 1 `CacheDetector` ABC: `detect_cache(frame) -> Optional[BBox]`.
  - `PlaceholderColorShapeDetector` — gray → 5×5 blur → Otsu **inverted** → contours → filter area ratio 0.001–0.8 & aspect 0.6–1.6 (squarish) → score by squareness. **Deliberately returns None on no match** — a previous version fabricated a center-50% bbox with confidence 0.5 which "detected" blank frames (contract-breaking; removed after test proof).
  - `HailoCacheDetector` — NotImplementedError stub; swap-in point for real Hailo-8/8L model (Phase 9). Interface designed so the swap touches neither search_algorithm nor fsm.
- Stage 2 `decode_qr(frame, bbox)` — ROI (if bbox > 10px) → full frame via `cv2.QRCodeDetector` → `pyzbar` fallback.
- `ConsensusBuffer(required_consecutive=3, miss_tolerance=1)` — the key robustness piece: streak of identical decodes; a bad frame (no decode OR different decode) increments a miss counter; only after `miss_tolerance+1` consecutive misses does it abandon/switch. **Why:** original reset-on-first-bad-frame meant good,good,BLANK,good,good never reached consensus — a single motion-blurred frame shouldn't wipe progress.
- `QRPipelineRunner` — daemon loop: `source.get_frame()` → detect → decode → consensus.update; on newly confirmed → `search_controller.on_detection(bbox, 0.95)` + `push_event("qr_decoded", ...)`. `set_source()` **starts the source** (was previously missing → pipeline silently never decoded; confirmed by repro) but does NOT stop the old one (CameraManager owns lifecycle).

### 4.11 `search_algorithm.py` — sweep planning + execution
- `SearchStrategy` ABC. `get_strategy(name)`: `lawnmower|grid` → Lawnmower, else ExpandingSquare.
- `ExpandingSquareStrategy` — starts at centroid (if inside polygon), then expanding square spiral (right, up, left, left, down, down, right×3, ... two legs per expansion, leg length +1 each step). Each candidate: distance from centroid ≤ max_radius+step AND point-in-polygon → waypoint.
- `LawnmowerStrategy` — boustrophedon rows over polygon bbox at step spacing, each point: radius check + polygon check.
- `SearchController`:
  - `generate_plan()` — reads fence from `state.get()`; **fails closed** if not loaded/armable/missing centroid/radius/origin; 0 valid waypoints → error.
  - `start_search()` — spawns `_search_loop` thread: `_set_guided_mode()` (DO_SET_MODE → 4) then per waypoint streams `SET_POSITION_TARGET_LOCAL_NED` at **2Hz for ~3s** (type_mask `0b0000111111111000` = 0x0FF8: position x,y,z active, ignore vel/accel/yaw; frame LOCAL_NED; down = −altitude). Breaks on stop or `target_found`.
  - `on_detection(bbox, confidence)` — sets `target_found`, stores target, `push_event("target_detected", ...)` (decoupled Phase 7 hook).
  - `begin_approach_descent()` — **closed-loop descent**, the design highlight: NO pre-decided decode altitude (lens-dependent: 6mm lens ≈9m vs 16mm ≈25m for 4px/module). Hold current XY (from latest GLOBAL_POSITION_INT, converted to ENU vs fence origin) → step down `APPROACH_DESCENT_STEP_M` (1.5m) → stream position target at 2Hz while holding `APPROACH_STEP_HOLD_S` (2.5s) → check live consensus each 0.5s → **stop the instant decode confirms**; else step down; hard floor `APPROACH_MIN_ALTITUDE_M` (1.5m) → error; overall 45s backstop. Runs in own thread; FSM polls status.
  - CLI `main()` with `--dry-run --strategy --altitude --step`.

### 4.12 `fsm.py` — MissionFSM (14 states)
`INIT → PRE_FLIGHT_CHECK → ARM → TAKEOFF → SEARCH → CACHE_DETECTED → APPROACH → DECODE → CONFIRMED → TRANSMIT_RESULT → RTL → LAND → MISSION_COMPLETE`, plus `FAILSAFE` reachable from any in-flight state (and it immediately → RTL).

- Loop: 10Hz; each tick: drain `_msg_queue` (HEARTBEAT→armed flag/last_hb, SYS_STATUS→battery %, GLOBAL_POSITION_INT→alt_rel), check failsafes, then per-state logic.
- **Failsafes (checked every tick, skipped in INIT/PRE_FLIGHT/FAILSAFE/RTL/LAND/COMPLETE):** telemetry link loss > 5s; fence unarmable; battery < 20%.
- Per-state:
  - PRE_FLIGHT_CHECK: armable ∧ link healthy → ARM; else 10s timeout → FAILSAFE.
  - ARM: every 2s send DO_SET_MODE GUIDED + `COMPONENT_ARM_DISARM(1)`; wait for armed flag (or dry-run); 10s timeout → FAILSAFE.
  - TAKEOFF: every 3s `NAV_TAKEOFF` to 5m; 30s timeout → FAILSAFE.
  - SEARCH: on entry switch camera → cam1, start search (dry_run flag honored); poll: `target_found` → CACHE_DETECTED; `completed` (no target) → FAILSAFE; 120s sweep timeout → FAILSAFE.
  - CACHE_DETECTED: `_switch_camera("cam2")` (fail-closed → FAILSAFE on failure; **resets consensus** — a streak from one camera's view is meaningless on the other) → APPROACH.
  - APPROACH: on entry `begin_approach_descent()`; `decoded` mid-descent → CONFIRMED (skip DECODE — same altitude, same buffer); `error` (floor/timeout) → DECODE for a final attempt at the lowest reached altitude.
  - DECODE: poll consensus; confirmed → CONFIRMED (with payload); 60s timeout → FAILSAFE.
  - CONFIRMED → TRANSMIT_RESULT: `push_event("mission_result")` once (UI auto-replays for reconnects) + `send_statustext("QR:<payload>")` to Mission Planner **every 2s for a 15s window** (STATUSTEXT has no ACK — bounded retry survives a radio dropout).
  - FAILSAFE → stop search → RTL.
  - RTL: mode 6 + `NAV_RETURN_TO_LAUNCH` every 3s; LAND when disarmed (>5s) or alt < 1m (>10s).
  - LAND: mode 9 + `NAV_LAND` until disarmed → MISSION_COMPLETE (unsubscribes, stops loop).
- `_transition_to()` always pushes `fsm_state` event (from/to/reason/payload) — frontend gets every transition.
- `abort()` works from any state including INIT (bug fixed: INIT used to be in the exclusion list → abort before start silently no-oped).
- `FSM_DRY_RUN=True` → all MAVLink sends become log lines (safe offline runs).

### 4.13 `system_monitor.py` / `network_monitor.py`
- SystemStats (pydantic): cpu %, ram %/used/total, disk %, cpu temp (Linux sysfs thermal_zone0, millidegrees), cpu freq, uptime, `ai_accelerator_available/info` via `hailortcli fw-control identify` (NEED TEST on real Hailo). psutil first-call cpu_percent=0 caveat documented.
- `check_internet_status()` — raw TCP to 8.8.8.8:53, 1s timeout; any failure = down. **INvariant: informational only, never gates safety.**

### 4.14 `app.py` — the service
Lifespan wiring order: `init_mavlink_bus()` → `init_telemetry_hub(bus)` → `init_camera_manager()` → `search_controller.set_bus(bus)` → `qr_pipeline_runner.set_search_controller` + `set_source(cam1)` + `start()` → `fsm.init_fsm(bus, camera_manager)`. Shutdown reverses.

**Camera auto-detection at startup (picamera2 tier):** 0 cameras → **RuntimeError, refuse to start** (fail closed); 1 camera → both roles share it (logged as degraded operation — single lens determines both search & decode altitude); 2+ → assign search/decode roles by sensor-model hints, fall back to index order with a LOUD warning; >2 → warn and ignore extras.

CORS `*`; static mount of `frontend/` at `/static`. All endpoints in §5.

---

## 5. FULL API REFERENCE

| Method | Path | Body / notes | Returns |
|---|---|---|---|
| POST | `/api/geofence` | `{vertices:[{lat,lon}...]}` (3–255) | `GeofenceStatus` — full pipeline: EKF origin → upload → readback (1e-7° tol ≈1cm) → enable → ENU/centroid/radius → persist |
| GET | `/api/geofence` | | cached `GeofenceStatus` |
| GET | `/api/geofence/armable` | | `{armable, reason}` — light gate for arm seq/search |
| DELETE | `/api/geofence` | | clears on FC + locally |
| GET | `/health` | | `{status: ok}` |
| GET | `/tiles/{z}/{x}/{y}.png` | MBTiles w/ XYZ→TMS y flip: `tms_y = 2^z − 1 − y` | PNG or 404 |
| WS | `/ws/telemetry` | | envelope stream (position/stats/system/event), latest-per-channel replay on connect |
| POST | `/api/camera/mode` | `{mode: cam1\|cam2\|dual\|none}` | manager status |
| GET | `/api/camera/status` | | mode, aiortc flag, per-cam {type, active, peer_connections} |
| POST | `/api/camera/focus` | `{position: 0..1}` | ok (real hw only) |
| GET | `/api/camera/frame/{camera_id}` | | latest frame as JPEG |
| WS | `/ws/webrtc/{camera_id}` | `{type: offer, sdp}` in → `{type: answer, sdp}` out | signaling |
| POST | `/api/search/start` | `{strategy?, dry_run, altitude_m?, step_m?}` | `SearchStatus` |
| POST | `/api/search/stop` | | `SearchStatus` |
| POST | `/api/search/dry-run` | as start | computes + logs plan, no flight cmds |
| GET | `/api/search/status` | | `SearchStatus` |
| GET | `/api/qr/status` | | `QRDetectionResult` (consensus state) |
| POST | `/api/qr/process-frame?camera_id=cam1` | | single-frame detect/decode/consensus |
| POST | `/api/fsm/start` | | `FSMStatus` (starts mission at PRE_FLIGHT_CHECK) |
| POST | `/api/fsm/abort` | | emergency FAILSAFE→RTL, works even pre-start |
| GET | `/api/fsm/status` | | `FSMStatus` |

Error mapping: `ConnectionError` → 503 + `state.clear()`; `FenceError` → 500 + `state.clear()`.

---

## 6. FSM STATE MACHINE (visual)

```
                 ┌──────────┐
      ─────────▶ │    INIT  │
                 └────┬─────┘
                      ▼
              ┌───────────────────┐   timeout (10s)
              │ PRE_FLIGHT_CHECK  │ ────────────────┐
              │ (fence∧link gate) │                 │
              └────────┬──────────┘                 │
                       ▼ armed                      │
                 ┌─────┴─────┐  timeout(10s)        │
                 │    ARM    │ ────────────┐        │
                 └─────┬─────┘             │        │
                       ▼ alt≥4.5m          │        │
                 ┌─────┴──────┐  timeout   │        │
                 │  TAKEOFF   │ (30s) ────┐ │        │
                 └─────┬──────┘           │ │        │
                       ▼ cam1+sweep       │ │        │
                 ┌─────┴──────┐ target    │ │        │
                 │   SEARCH   │◀──────────┼─┼────────┼──── (link loss / fence / batt)
                 └─────┬──────┘           │ │        │       FAILSAFE from ANY
        no target (120s)/ │ completed     │ │        │
                 ┌────▼───────┐           │ │        │
                 │CACHE_DETECTED│          │ │        │
                 │ switch cam2 │          │ │        │
                 └─────┬──────┘           │ │        │
                       ▼                  │ │        │
                 ┌─────┴──────┐  decoded  │ │        │
                 │  APPROACH  │───────────┼─┼────────┼──▶ CONFIRMED
                 │ (closed-loop│  floor/timeout     │
                 │  descent)  │───────────▶ DECODE  │
                 └────────────┘     │ confirmed     │
                                    └───────────────┘
CONFIRMED → TRANSMIT_RESULT (STATUSTEXT QR:<payload> ×2s/15s + UI event) → RTL → LAND → MISSION_COMPLETE
FAILSAFE → (stop search) → RTL
```

---

## 7. FRONTEND DETAIL (vanilla JS, no build step)

**`index.html`** (~600 lines) — single-page dark "cockpit" UI, ES modules, Leaflet 1.9.4 from CDN.
- Layout: left 340px control panel | right map + camera grid overlay (bottom-right, 2× 280×210 video tiles).
- Panel cards:
  - **Backend** — URL input (drives all REST+WS; changing it reconnects telemetry).
  - **Map & Tile Source** — toggle Online OSM ↔ Offline MBTiles (`/tiles/...`), toggle meter grid.
  - **Telemetry** — badge + lat/lon/alt MSL/alt rel/hdg/vel/roll-pitch-yaw/mode/armed/GPS fix+battery V (from WS position/stats channels).
  - **System Info** — CPU/RAM/disk/temp/freq/uptime/AI accel/internet (from WS system channel).
  - **Camera Pipeline** — mode buttons (Cam1/Cam2/Dual/Off), per-cam status, WebRTC latency stats, focus slider 0–1 step 0.05.
  - **Fence status** — big ARMABLE/NOT ARMABLE annunciator (green/red lamp), vertices, max radius, origin, readback match, reason.
  - **Draw fence** — Start/Stop drawing (map click adds vertices + amber circle markers), Undo, Clear, point count; Upload (≥3 pts → POST /api/geofence), Delete.
- Polls `GET /api/geofence` every 3s as a connection check (documented as fine at this rate, "not the pattern for live position").
- After upload, the EKF origin from the response becomes the meter-grid origin (grid re-anchors to (0,0) = fence origin).

**`js/telemetry.js`** — `TelemetryClient(backendUrl)`: WS with 2s auto-reconnect; parses envelopes; emitter API `on(event, cb)` for `connected/disconnected/envelope/<channel>/drone_event/snapshot` plus fine-grained `_emitSpecific` (position/attitude/velocity/gps/battery/mode/armed).

**`js/tiles.js`** — `TileManager`: online layer (OSM) vs offline layer (backend `/tiles/{z}/{x}/{y}.png`, minZoom 15); ENU meter grid: 50m spacing, ±500m, center lines amber solid, others cyan dashed, origin marker (red "map center fallback" vs amber "EKF Geofence Origin"). **ENU math is a direct JS port of `backend/geo_utils.py`** — the two stay in lockstep by construction.

**`js/cameras.js`** — `CameraStreamManager`: per-camera WebRTC `RTCPeerConnection` (recvonly transceiver, `iceServers: []` — direct LAN), WS signaling: onopen → createOffer → send; onmessage answer → setRemoteDescription; ontrack → `video.srcObject`. **Any failure (offer error, WS error, init error) → fallback: poll `GET /api/camera/frame/{id}` every 200ms (≈5fps) into an img element.** Latency measurement callback on track + per-poll.

---

## 8. SIMULATION LAYER

- **`sim/worlds/qr_search_field.sdf`** — Gazebo Harmonic world: Bullet featherstone physics @ 500Hz (1ms step, RRF 1.0), sun+ambient, 60×60m green field, ArduPilot plugin systems (Physics, UserCommands, SceneBroadcaster, Contact, Sensors/ogre2, Imu, AirPressure, Altimeter, ...), iris vehicle with downward RGB camera at (0,0,0.3), `qr_box` at **(10.0, 5.0, 0.05)** — deliberately off-center so guidance bugs are visible. World ENU = ArduPilot local frame (+X East, +Y North, +Z Up). 60m scale = guess for competition field (NEED TEAM INPUT).
- **`sim/models/qr_box/`** — static 1×1×0.1m slab; two visuals (white box body + flat top-face plane with QR material — avoids UV-mapping the box). `materials/textures/qr_code.png` committed (regenerate with the script).
- **`sim/textures/generate_qr_texture.py`** — generates the decodable QR PNG (payload = `SYNTHETIC_QR_PAYLOAD` env or default; doesn't need to match real competition payload).
- **Runbook (`docs/SIMULATION_SETUP.md`):** ArduPilot SITL + Gazebo Harmonic (`libgz-sim8-dev`) + **ardupilot_gazebo** (the current official plugin; old `gazebo_sitl` tutorials are stale) + env `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH` (+repo sim/models). Camera path only: ROS 2 Humble + `ros-humble-ros-gz-bridge` (+ gz rosdep source list), never on the Pi. Launch: `sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map` + `gz sim sim/worlds/qr_search_field.sdf` + ros_gz_bridge for the camera topic; find real topic via `ros2 topic list` (expected pattern `/world/qr_search_field/model/iris_with_ardupilot/link/{link}/sensor/{sensor}/image`).
- **`docs/HARDWARE_SETUP.md`** — BOM, `/dev/serial/by-id` discovery, `libcamera-hello --list-cameras`, venv with `--system-site-packages` (critical for apt picamera2 bindings), piwheels index for picamera2.
- **`docs/BENCH_TEST_QUICKREF.md`** — no-prop bench procedure: power-up order (Pixhawk+GPS first!), wiring map, SITL rehearsal (SITL on 14552 → backend, MP on 14550), real-HW run, 8-item checklist incl. the two ⭐ unknowns (MP fence auto-refresh? FENCE_TYPE auto-inferred?), symptom→cause table.

---

## 9. TOOLS

- **`tools/fetch_tiles.py`** — OSM tile downloader → MBTiles SQLite. Handles the classic gotcha: MBTiles uses **TMS y (0 = bottom)** while OSM/XYZ uses y-from-top; `tms_y = (2^z − 1) − xyz_y` both directions. bbox + zoom-range args, progress, polite timing.

---

## 10. VENDORED SHIMS (`backend/_vendor/`)

Dev-only, for **air-gapped machines without pip access** (`_bootstrap_vendor.py` prepends to sys.path):
- `pymavlink/mavutil.py` (470L) — implements a **raw MAVLink v2 codec over UDP sockets + struct** (STX 0xFD framing), just enough for bus/fence code + SITL over UDP. Message-ID table for the ~14 messages actually used; `mavlink` constants namespace; `mode_string_v10` map.
- `pydantic/__init__.py` (153L) — dataclass-backed BaseModel/Field(field ge/le)/field_validator/model_dump_json.
- `fastapi/__init__.py` (111L) — FastAPI(lifespan), route decorators, HTTPException, WebSocket stub, Response, CORSMiddleware, StaticFiles.
> Pattern worth stealing: the codebase is written against the real APIs; shims let the same code run/test on a locked-down machine. `test_standalone_full.py` (702L) exercises all pure-logic modules using ONLY stdlib+numpy+cv2 + these shims.

---

## 11. TEST SUITE (18 files, 3 tiers)

**Tier 0 — mock FC (`tests/mock_fc.py`):** a minimal ArduPilot-protocol server over **real MAVLink/UDP** (`udp:0.0.0.0:14560`): heartbeats, `GPS_GLOBAL_ORIGIN` on REQUEST_MESSAGE, full fence upload (per-seq MISSION_REQUEST_INT), download, clear, param echo, FENCE_TYPE=4.0 stub, and a **fixed fake GLOBAL_POSITION_INT** (12.9716, 77.5946, 14m — a Bengaluru-ish origin) at ~3.3Hz so approach-descent logic has a position feed. Explicitly NOT a substitute for SITL.

**Tier 1 — protocol/unit (no hardware):** test_fence_protocol (upload/readback/clear/params over UDP loopback — the subscribe-before-send regression), test_fsm_protocol (MagicMock bus: initial state, abort works from INIT), test_search_protocol (position-target streaming vs mock), test_qr_pipeline (consensus streak edge cases: good/good/BLANK/good/good must reach consensus), test_search_algorithm (dry-run plans vs known geometry), test_approach_descent (holds XY, steps, stops at decode, respects floor), test_camera_api + test_camera_manager + test_camera_sim_acceptance (mode switching, dual, QR decode from frames, latency), test_telemetry_ws (real WS round trip), test_phase1_sim_smoke (standard pre-flight: Tier A bus, Tier B synthetic cam, ... run before any new session).

**Tier 2 — SITL/Gazebo (needs sim running):** test_fence_sitl (real ArduPilot fence storage + FENCE_TYPE observation before/after enable — resolves the README's NEED TEST), test_fence_breach_sitl (437L — prove FENCE_ACTION=1 actually fires: upload 20m fence, arm, takeoff, drive 30m outside, wait FENCE_STATUS.breach_status≠0), test_search_sitl (live sweep acceptance), test_fsm_sitl (full FSM vs SITL+Gazebo), test_qr_gazebo_sim (end-to-end camera→decode in sim), test_standalone_full (air-gap tier).

---

## 12. THE GSD LAYER (`.gsd/`, `.agent/workflows/`, `.gemini/`)

**"Get Shit Done"** — a spec-driven, context-engineered *methodology for AI agents*, embedded as prompt-engineered markdown. (Source referenced: github.com/glittercowboy/get-shit-done; "adapted for Google Antigravity".)

**Core principles:** 1) Plan before build (no code without FINALIZED spec) 2) State is sacred (every action updates persistent memory) 3) Context is limited (hygiene: after 3 failures → state dump + fresh session) 4) Verify empirically (no "trust me, it works" — proof required for "done").

**Lifecycle:** `/map` (analyze codebase → ARCHITECTURE.md/STACK.md) → `/plan` (phases, XML task structure) → `/execute` (wave-based, one **gsd-executor subagent per plan**, orchestrator keeps ~15% context) → `/verify` (must-haves + proof; gaps → gap-closure plans) → loop.

**26 workflow commands** (each a markdown file with `<objective>/<context>/<process>`, dual PowerShell+Bash snippets): map, plan, execute, verify, debug (3-strike rule), progress, pause (state dump), resume (load state), add-todo, check-todos, new-project (deep questioning → SPEC.md), new-milestone, complete-milestone (archive), audit-milestone, add-phase, insert-phase, remove-phase, discuss-phase, research-phase, list-phase-assumptions, plan-milestone-gaps, sprint, update, web-search, whats-new, help, install.

**Key files:** SPEC.md (vision, must be FINALIZED), ROADMAP.md (phases), STATE.md (session memory: current position, in-progress, blockers, context dump, next steps), JOURNAL.md (per-session log: objective/accomplished/verification/paused-because/handoff), TODO.md, ARCHITECTURE.md, STACK.md.

**23 templates** in `.gsd/templates/`: state, state_snapshot, plan, spec, roadmap, milestone, phase-summary, journal, todo, sprint, debug, research, summary, UAT, verification, architecture, context, decisions, discovery, project, requirements, stack, token_report, user-setup.

**Task format (XML inside markdown):**
```xml
<task type="auto">
  <name>Clear name</name>
  <files>exact/path.ts</files>
  <action>Specific instructions</action>
  <verify>Executable command</verify>
  <done>Measurable criteria</done>
</task>
```

**Live state in this repo:** STATE.md shows the project paused 2026-09-03 at the start of Phase 8 (FSM) — JOURNAL.md documents the handoff. (Note: GEMINI.md references `../PROJECT_RULES.md` and `.agents/` subagent files that are **not present** in the repo — broken references.)

---

## 13. DESIGN PATTERNS WORTH STEALING (ranked by transferability)

1. **Fail-closed invariants, stated loudly** — no fence → not armable; no camera → refuse start; wrong camera switch → FAILSAFE; corrupt state file → treat as empty. Comments say *why* and what the failure mode looked like.
2. **Subscribe-before-send** for any request/response over an async message bus (race where the reply beats the listener — proven in their tests).
3. **Single connection owner + pub/sub bus** with drop-oldest bounded queues; lock only around socket writes.
4. **Config via env vars only + a written INVARIANT against hardcoded venues/coordinates.**
5. **Consensus buffer with miss tolerance** (N consecutive identical decodes; K bad frames absorbed) — generalizable to any noisy sensor/decoder.
6. **Closed-loop approach instead of a preset target** (descend-in-small-steps + check-the-real-signal-each-step + hard safety floor) — beats any hardcoded altitude given sensor variance.
7. **3-tier device abstraction** (synthetic → sim → hardware) behind one contract, lazy hardware imports, factory + auto-detection with loud-warn fallbacks and model-hint role assignment.
8. **WebSocket latest-message replay per channel** — reconnecting clients resync instantly.
9. **WebRTC with graceful degradation** to HTTP frame polling (200ms), no ICE servers on LAN.
10. **Bounded at-least-once delivery** for ACK-less channels (STATUSTEXT resend every 2s in a 15s window; UI path self-heals via replay).
11. **Readback-verify everything** (fence upload → download → compare with 1e-7° tolerance; param set → confirm within 0.01).
12. **NEED TEST / NEED TEAM INPUT discipline** + status legend ✅ ⚠️ 🚧 🔬 — documented unknowns instead of silent guesses; every "NEED TEST" explains what would confirm it.
13. **Acceptance tiers**: mock responder (fast, wire-level) → real SITL (firmware-true) → hardware. Mocks are explicitly "not a substitute".
14. **Vendored dev shims + bootstrap** for air-gapped machines; code unchanged, only the import target swaps.
15. **Docs that record real bugs with repro numbers** (e.g. executor shutdown: ~1/3 crash with wait=False, 0/10 with wait=True; fabricated-bbox detector test; camera-never-switched grep proof).
16. **GSD project layer**: STATE.md/JOURNAL.md/templates/workflows as markdown prompts; subagent-per-plan execution with context budgets.
17. **Flat-earth ENU + area-weighted centroid** for field-scale geometry (no proj lib needed).
18. **Search plan constrained to (radius ∧ point-in-polygon) per waypoint** — plans adapt to any polygon shape; never fly outside the uploaded fence even at the software level.

---

## 14. OPEN ITEMS (the repo's own "NEED TEST / NEED TEAM INPUT" list)

**Firmware/hardware:** Pixhawk serial by-id path + baud; `MAV_CMD_REQUEST_MESSAGE` support (else passive-listen fallback for EKF origin); `FENCE_TYPE` auto-infer behavior per firmware; Mission Planner fence auto-refresh; physical fence-breach RTL confirmation (tethered flight); camera model strings; RGB888 vs BGR byte order; achievable FPS at 1080p; Arducam IMX477 focus driver; Hailo HAT variant (8 vs 8L) + SDK; ArduPilot firmware version.
**Spec unknowns:** competition QR payload exact format (numeric confirmed); actual field dimensions (60m assumed); decode altitude (lens-dependent — that's why it's closed-loop); search altitude tuning per real hardware.
**Not yet done (their plan):** full live FSM end-to-end run (Tier 1 SITL + Gazebo); real Hailo detector (Phase 9); Picamera2 bring-up (Phase 9).

---

## 15. BUILDING BLOCKS INDEX — what we can lift for our own thing

| Block | Where | Reuse notes |
|---|---|---|
| MAVLink pub/sub bus | `mavlink_bus.py` | drop-in for ANY multi-consumer MAVLink app |
| Fence upload/readback/enable | `mavlink_fence.py` | whole module; only venue-specific thing is the polygon input |
| ENU + polygon geometry | `geo_utils.py` | 100% generic |
| Telemetry hub + envelope schema | `telemetry.py` | channels/replay pattern reusable for any live-data UI |
| WS client w/ reconnect + emitter | `frontend/js/telemetry.js` | vanilla, no deps |
| WebRTC LAN streaming (aiortc + JS) | `camera_manager.py` + `js/cameras.js` | works for any frame source via get_frame() |
| QR detect→decode→consensus | `qr_pipeline.py` | swap detector/decoder freely; consensus is the gold |
| Sweep planner (2 strategies) | `search_algorithm.py` | any polygon + radius; strategies are pluggable ABC |
| Closed-loop descent | `search_algorithm.py::begin_approach_descent` | generalize: step + sense + floor |
| 14-state mission FSM | `fsm.py` | skeleton for any autonomous sequence w/ failsafes |
| Cockpit UI layout | `frontend/index.html` | copy the panel/annunciator/map/grid pattern |
| MBTiles offline tiles | `tools/fetch_tiles.py` + `/tiles` endpoint + `js/tiles.js` | offline map stack, 3 pieces |
| Mock FC test rig | `tests/mock_fc.py` | fast wire-protocol validation without hardware |
| 3-tier camera abstraction | `camera_source.py` | any multi-source sensor works the same way |
| Dev shims + bootstrap | `_vendor/` + `_bootstrap_vendor.py` | run/test without pip |
| GSD agent workflows | `.agent/workflows/` + `.gsd/` | the whole PM methodology for AI-driven builds |
| Bench-test checklist style | `docs/BENCH_TEST_QUICKREF.md` | template for ops docs |

---

## 16. HOW TO RUN (from their docs)

```bash
# Backend (real Pi)
cd backend
pip install -r requirements.txt --break-system-packages   # or venv
export MAVLINK_DEVICE=/dev/serial/by-id/<pixhawk-id>
uvicorn app:app --host 0.0.0.0 --port 8000
# open frontend/index.html, set backend URL http://<pi-ip>:8000

# SITL (no hardware)
sim_vehicle.py -v ArduCopter --console --map          # udp:127.0.0.1:14550
export MAVLINK_DEVICE=udpout:127.0.0.1:14550
uvicorn app:app --host 0.0.0.0 --port 8000

# Gazebo full sim
# (see docs/SIMULATION_SETUP.md for install; then:)
sim_vehicle.py -v ArduCopter -f gazebo-iris -model JSON --map --console
gz sim sim/worlds/qr_search_field.sdf
ros2 run ros_gz_bridge parameter_bridge /iris/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
export CAMERA_SOURCE=gazebo

# Offline tiles
python tools/fetch_tiles.py --bbox "lat0,lon0,lat1,lon1" --zooms 15-18 --out field.mbtiles
export MBTILES_PATH=field.mbtiles
```

*End of analysis. Repo clone lives at `/home/user/hehe` for code pulling.*
