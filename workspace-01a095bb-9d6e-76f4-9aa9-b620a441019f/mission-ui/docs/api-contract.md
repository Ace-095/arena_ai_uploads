# API CONTRACT — Mission Companion (v1)

> Single source of truth. The **UI**, the **Pi backend (workstream 2)** and the **mock**
> (`bridge/mp_bridge.py --mock`) all implement/consume THIS contract.
> Nothing here is copied from hehe as-is — hehe shapes are the starting point, re-specified
> deliberately. Every item is re-verified on bench/SITL before it is trusted.

## Roles

- **PI** (Raspberry Pi 5, competition): owns MAVLink (USB), camera, search/FSM, fence upload.
  Serves this contract on `:8000` (FastAPI or otherwise — transport is the contract, not the
  framework).
- **BRIDGE** (Windows 11 laptop): `mp_bridge.py`, stdlib-only, `:8100`. Serves the UI, tails
  MP logs, extracts QR messages, provides manual-entry fallback. In `--mock` mode it ALSO
  implements the full PI contract (demo/test target).
- **UI** (browser on laptop): consumes PI (WS + REST, over router/LTE) and BRIDGE (same origin,
  always available).

## Transport notes

- UI→PI REST: plain `fetch` JSON. Errors: `{ "detail": "..." }` with 4xx/5xx.
- UI→PI WS: `ws(s)://PI/ws/telemetry`. Envelope JSON objects, one per message.
  **On connect, PI replays the latest envelope per channel** (reconnect resync).
- UI→BRIDGE SSE: `GET /api/mp/stream` (same origin). `event: <name>\ndata: <json>\n\n`.
- All timestamps `t` = unix seconds (float).

## Channels / envelopes (WS `/ws/telemetry`)

| channel | rate | data |
|---|---|---|
| `telemetry` | ≤5 Hz | `{lat, lon, alt_rel, alt_msl, vx, vy, vz, hdg, attitude:{roll,pitch,yaw}, mode, armed, gps:{fix, sats}, battery:{v, pct}}` |
| `system` | 1 Hz | `{cpu, ram, ram_used_mb, ram_total_mb, disk, temp_c, freq_mhz, uptime_s, network:{router:"up|down", lte:"up|down"}}` |
| `event` | as happens | `{type, value}` — types: `fsm_state`, `qr_decoded`, `fence_updated`, `target_detected`, `mission_result`, `plan_synced`, `abort`, `link_up`, `link_down` |
| `qr` | 1 Hz | `{streak, required, payload, confirmed, bbox:{x,y,w,h}, source}` |
| `fsm` | on change | `{state, previous, reason, armable, link_healthy, plan:{items, trigger_seq, synced}, payload}` |
| `fence` | on change | `GeofenceStatus` (below) |
| `log` | as happens | `{level:"INFO|WARN|ERROR", msg}` |

### FSM states (PI-driven; MP is the mission head — PI never arms)

`IDLE → PLAN_SYNC → WAITING_TRIGGER → SEARCH → TARGET_FOUND → DESCEND → DECODED → TRANSMIT → RTL → LAND → COMPLETE`
+ `FAILSAFE` (from any in-mission state; abort/manual/safety).
Handover trigger = plan's DO_SPRAYER item detected via `MISSION_CURRENT` (see ui-spec §5).

### GeofenceStatus

`{loaded, confirmed, reason, vertex_count, area_m2, max_radius_m,
   centroid_enu:{x,y}, origin_lat, origin_lon, vertices_latlon:[{lat,lon}]}`
- `confirmed` = readback on FC matched (1e-7° tolerance). UI lamp: green only when `loaded && confirmed`.
- No fence ⇒ search must not run (fail-closed). Search area = this polygon.

## PI REST endpoints

| Method | Path | Notes |
|---|---|---|
| GET | `/health` | `{"status":"ok"}` |
| POST | `/api/geofence` | `{vertices:[{lat,lon}...]}` 3–255 → `GeofenceStatus`. PI uploads to FC (mission_type=FENCE), readback-verifies, enables fence, sends `STATUSTEXT "FENCE: n verts, readback OK"`. 503 = FC unreachable, 500 = fence op failed, 400 = validation. |
| GET | `/api/geofence` | cached `GeofenceStatus` |
| DELETE | `/api/geofence` | clear on FC + locally |
| GET | `/api/fsm/status` | `fsm` envelope data |
| POST | `/api/fsm/abort` | FAILSAFE → RTL. Works pre-mission too. |
| POST | `/api/fsm/start` | **debug only** (force handover now; labeled as such in UI) |
| GET | `/api/qr/status` | `qr` envelope data |
| GET | `/api/camera/status` | `{mode, active, controls:{exposure_us, gain, af_mode, brightness, contrast, saturation, sharpness, adaptive}}` |
| POST | `/api/camera/controls` | subset of controls fields → applied via `Picamera2.set_controls` (libcamera: `ExposureTime`, `AnalogueGain`, `AfMode`, `Brightness`, `Contrast`, `Saturation`, `Sharpness`); returns full controls |
| GET | `/api/camera/frame/cam1` | latest frame, `image/jpeg` (polling fallback when WebRTC unavailable) |
| WS | `/ws/webrtc/cam1` | optional WebRTC signaling (`{type:"offer",sdp}` → `{type:"answer",sdp}` / `{type:"error"}`); UI must fall back to frame polling on any failure |
| GET | `/tiles/{z}/{x}/{y}.png` | offline MBTiles (XYZ), 404 when absent |

**QR result delivery (3 routes, all shown in UI with receipts):**
1. PI → Pixhawk → **STATUSTEXT `QR:<payload>`** → MP message console (1st priority; retried 2 s / 15 s window).
2. PI → WS `event mission_result` + `qr` channel → UI (needs router/LTE).
3. BRIDGE tails MP logs / manual entry → UI (needs NO uplink).

## BRIDGE REST/SSE endpoints (laptop, `:8100`)

| Method | Path | Notes |
|---|---|---|
| GET | `/` + static | the UI (offline-safe; Leaflet vendored locally) |
| GET | `/api/mp/state` | `{qr:{payload, source, ts, line} | null, last_line, watching:[...], bin_parser_available:bool}` |
| GET | `/api/mp/stream` | SSE. events: `mp-line {line,ts,source}`, `mp-qr {payload,source,ts,line}`, `bridge-state {watching, ...}`. In `--mock` mode ALSO `pi <envelope>` (mirrors the mock PI's WS stream). |
| POST | `/api/mp/qr` | `{payload:"42"}` manual entry → emits `mp-qr {source:"manual"}`. Accepts 1–6 alnum. |
| POST | `/api/mp/watch` | `{path:"C:\\..."}` add watched file/dir (text tail; .bin if pymavlink present) |

MP log QR extraction: pattern `QR:<payload>` (case-insensitive) in any watched text stream
(matches the STATUSTEXT we send); in .bin via pymavlink DFReader `MESSAGE` records (optional dep).

## Invariants (carry into every implementation)

1. No hardcoded venue coordinates in any component; search area = runtime fence polygon.
2. Fail-closed: no fence ⇒ no search; no trigger ⇒ no handover; PI never arms.
3. Router/LTE status is informational — the radio (MP link) is the mission path.
4. PI fence readback must match before `confirmed=true`.
5. UI must be fully usable offline (vendored assets; OSM tiles need internet, grid/map still work).
