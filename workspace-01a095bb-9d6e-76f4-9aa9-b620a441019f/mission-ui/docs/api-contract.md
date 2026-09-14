# API CONTRACT — Mission Companion (v2)

> Single source of truth. The **UI**, the **Pi backend (workstream 2)** and the
> **mock** (`bridge/mp_bridge.py --mock`) all implement/consume THIS contract.
> v2 change: the laptop has no radio — Mission Planner owns the telemetry link
> and forwards MAVLink to the bridge over UDP. Nothing here is trusted until
> bench/SITL-verified (see `our-mission.md` §6).

## Roles

```
Pixhawk <==telemetry radio==> Mission Planner <==UDP 14551==> BRIDGE ==> UI (SSE)
   ^                               (the head: arm/home/plan)      |
   |                                                              | serves UI, tails MP log, serves tiles
Pi 5 (search FSM, cameras, QR) <==WS receive-only==> UI (mission brain panel)
```

- **FC** (Pixhawk + ArduCopter): flight truth. Inclusion fence, plan, DO_SPRAYER.
- **MP** (Mission Planner, laptop): the head — arming, home, plan upload,
  DO_SPRAYER trigger. Forwards MAVLink to the bridge (UDP, **Write access**).
- **BRIDGE** (this repo, laptop): GCS client (sysid 254) on UDP `:14551`, UI
  host, MP-log tail, MBTiles server. Owns telemetry / fence / plan / console
  data for the UI.
- **PI** (Pi 5): mission brain — search FSM, dual cameras, QR decode. The UI's
  Pi link is **receive-only** except camera tuning (no MAVLink path exists for
  libcamera controls).
- **UI** (browser): fence drawing → bridge → MP → FC; live everything.

## Transports

| path | endpoint | direction |
|---|---|---|
| Pi WS | `ws://<pi>:8000/ws/telemetry` | Pi → UI (envelopes `{channel, t, data}`) |
| Pi REST | `http://<pi>:8000/...` | UI → Pi (camera tuning only) + status reads |
| Bridge REST | same origin `/api/mp/*`, `/tiles/*` | UI ↔ bridge |
| Bridge SSE | same origin `/api/mp/stream` | bridge → UI (all channels) |
| MAVLink | UDP `:14551`, GCS sysid 254 | bridge ↔ MP ↔ FC |

v1's Pi `telemetry` / `fence` WS channels are **gone** — flight truth is
bridge-owned now. The UI ignores them if a stale Pi still sends them.

## Pi WS channels (`{channel, t, data}`)

- `system`: `{host, pid, temp_c, cpu_pct, mem_pct, disk_pct, uptime_s}` —
  the Pi's own health, pushed on the WS cadence so a failing Pi is
  diagnosable from the ground without SSH
- `fsm`: `{state, reason?, acknowledged?{by, reason}}`
  (mock may send `{from, to, reason}` — UI reads `state ?? to`)
- `qr`: `{payload, streak?, required?, confirmed?}`
- `coverage`: `{cells, new_cells, cell_m, pct?}` — searched-grid progress
- `event`: `{type, ...}` — `target_detected {bbox, confidence, lat, lon}`,
  `qr_decoded {payload, altitude_m}`, `mission_result {payload, routes}`,
  `plan_synced {items, trigger_seq}`, `fc_link {state, detail}`
  (`link_lost` | `link_retry` | `link_restored`), `abort {from}`
- `log`: `{level, msg}`

The Pi replays the **latest envelope per channel** on every WS connect, so a UI
opened mid-flight is correct immediately instead of waiting for the next push.
The mock replays a short history burst on every WS connect.

## Pi REST v2 (`http://<pi>:8000`)

| method | path | body → reply |
|---|---|---|
| GET | `/health` | `{ok, status, ts, uptime_s, link, ready, cams, ws_clients, ws, streams, mission{phase,detail}, fc{device, connected, down, reconnects, hb_age_s, mode, armed, veh_sysid, watchdog}, bringup{stage, state, detail, errors[], done}, urls[]}` — answers **even when the FC or cameras never came up** |
| GET | `/api/bringup` | the bring-up ledger alone: `{ok, stage, state{server,detector,cameras,streams,fc}, detail{}, errors[{stage,error,fatal,ts}], urls[], done}` — the "why isn't my Pi link working" endpoint |
| GET | `/api/fsm/status` | `{state, acknowledged, ...}` |
| GET | `/api/mission/status` | `{state, phase, detail, payload?, stats?}` |
| POST | `/api/fsm/start` | `{}` → started |
| POST | `/api/fsm/abort` | `{}` → aborted (debug only) |
| POST | `/api/mission/takeover` `/api/mission/abort` | bench-only local commands (the UI never sends these) |
| GET | `/api/qr/status` | `{payload, streak, required, confirmed}` |
| GET/POST | `/api/camera/status` | `{cam?}` → `{cam, model, controls}` |
| POST | `/api/camera/controls` | `{cam, exposure_us, gain_db, af_mode, adaptive, brightness, contrast, saturation, sharpness}` → `{ok, cam, controls}` |
| GET | `/api/cameras` | rig status `{cam1: {...}, cam2: {...}}` — UI hides tiles/tabs for missing cams (one camera ⇒ one tile); absent on mock/old Pi ⇒ UI keeps both tiles trying |
| GET | `/api/camera/frame/cam1` `/api/camera/frame/cam2` | JPEG snapshot bytes (mock: SVG) |
| GET | `/api/camera/stream/cam1` `/api/camera/stream/cam2` (`?fps=`) | MJPEG `multipart/x-mixed-replace` — UI tiles point `<img>` here first, snapshots are the fallback. Streams are adopted when cameras appear (`streamm1.sync`), because the server binds before camera detection |
| GET | `/api/camera/{cam}/controls` | current control values for one camera |
| WS | `/ws/telemetry` | the envelope stream above. **Requires a websocket implementation** (`websockets` or `wsproto`) on the Pi: uvicorn without one serves HTTP normally but silently 404s this route — `main.py` warns at startup and the UI falls back to polling `/api/fsm/status` + `/api/qr/status` |
| WS | `/ws/webrtc/cam1` `/ws/webrtc/cam2` | optional WebRTC; UI falls back to polling |

There is **no** `/api/geofence` on the Pi in v2 — the fence travels
UI → bridge → MP → FC over MAVLink.

## Bridge REST (same origin as the UI)

| method | path | body → reply |
|---|---|---|
| GET | `/api/mp/state` | bridge-state snapshot (below) |
| GET | `/api/mp/tiles-info` | `{available, path, zmin, zmax, count, bounds}` |
| GET | `/tiles/{z}/{x}/{y}.png` | tile bytes, else 404 `{error}` |
| GET/POST | `/api/mp/mavlink` | mavlink-state / `{port}` → rebind + state |
| GET | `/api/mp/fence` | read fence from FC (adopts MP-uploaded fence) → GeofenceStatus |
| POST/DELETE | `/api/mp/fence` | `{vertices:[{lat,lon}], mission_type:'fence', fence_action?}` → GeofenceStatus |
| GET | `/api/mp/plan` | plan (fresh read from FC) |
| POST | `/api/mp/mode` | `{mode:'RTL'|'LAND'}` → `{status, mode, result, result_name, from_sysid, home_set, home}` (`home_set`/`home` filled only when RTL is rejected — FC home query) |
| POST | `/api/mp/param` | `{name}` → `{name, value}` |
| POST | `/api/mp/qr` | `{payload}` → `{status, qr:{payload, source:'manual', ts, line}}` |
| POST | `/api/mp/watch` | `{path}` → `{watching}` |
| POST | `/api/mp/mock/restart` | `{}` → `{ok:true}` (mock only) |

`bridge-state`: `{watching, qr, bin_parser_available, mock, mavlink{connected,
port, vehicle_sysid, mode, armed}, tiles{...}, uptime_s}`

`qr` here is the **latched** payload: the first one seen on any of the four
routes (`{payload, source, ts, line}`), or a Pi-confirmed QR object when the Pi
route confirmed one, or `null` before anything was seen. A UI opened after the
fly-past therefore still reports the result — it does not have to have been
connected at the moment the payload flew past.

`mavlink-state`: `{connected, port, gcs_sysid, sender, vehicle_sysid, vehicle_type,
mode, mode_num, armed, vehicle_state, hb_age_s, rx_msgs, tx_msgs, rx_rate,
rx_v2, rx_v1, rx_bad_crc, rx_signed_dropped, bad_frames, unknown_ids,
sender_changes, link_flaps, fence, plan}`

Frame counters, because "connected but nothing arrives" must be explainable:
`bad_frames` = datagrams that are not usable MAVLink at all (wrong port,
another service, corrupt mirror); `unknown_ids` = well-formed frames whose
message id this bridge does not model (normal — SITL streams `VFR_HUD` and
friends constantly); `rx_v1` > 0 = the mirror is speaking MAVLink1 (parsed
fine; ArduPilot normally mirrors v2, see `SERIAL_PROTOCOL`).

`telemetry` (≤5 Hz): `{lat, lon, alt_rel, alt_msl, vx, vy, vz, hdg,
attitude{roll,pitch,yaw}|null, mode, mode_num, armed, gps{fix,sats}|null,
battery{v,pct,current_a}|null, mission{seq,total}|null}`

`GeofenceStatus`: `{loaded, confirmed, reason, vertex_count, area_m2,
max_radius_m, centroid_enu{x,y}, origin_lat, origin_lon,
vertices_latlon[{lat,lon}]}`

`plan`: `{items:[{seq, command, lat, lon, alt_m}], total, do_sprayer_seq|null,
synced, ts}`

## Bridge SSE (`GET /api/mp/stream`)

Events: `bridge-state`, `mavlink-state`, `fence`, `plan`, `telemetry`,
`mp-console {line, ts, severity}` (FC STATUSTEXT),
`mp-line {line, ts, source}` (tailed MP log file),
`mp-qr {payload, source, ts, line}` (`mavlink` | filename | `manual`),
`log {level, msg}`, `keepalive {ts}`.
In `--mock` the Pi channels (`system`, `event`, `qr`, `fsm`, `log`, `coverage`)
are relayed on the same stream; the UI only uses them when its Pi WS is down.
First burst on connect: `bridge-state` + `mavlink-state` (+ `fence`/`plan`
if known) + the latest envelope per Pi channel.

## MAVLink bridge (bridge ↔ MP ↔ FC)

- Transport: UDP `:14551` (rebindable), GCS sysid 254, one frame per datagram,
  replies to last-seen sender. No signing. Heartbeat 1 Hz.
- IN: `HEARTBEAT` (mode/armed, liveness 2.5 s), `GLOBAL_POSITION_INT`,
  `ATTITUDE`, `GPS_RAW_INT`, `BATTERY_STATUS`/`SYS_STATUS`, `MISSION_CURRENT`,
  `MISSION_ITEM_REACHED`, `FENCE_STATUS` (breach → ERROR log), `PARAM_VALUE`,
  `STATUSTEXT` → `mp-console` (+ `QR[:\s]+([A-Za-z0-9]{1,6})` → `mp-qr`).
- OUT: fence upload (`MISSION_COUNT`/`MISSION_ITEM_INT`, mission_type 1,
  `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` = 5001, frame GLOBAL,
  **param1 = vertex_count on every item**), readback verify (1e-7° ≈ 1 cm),
  `FENCE_ENABLE=1` + `FENCE_ACTION` + `FENCE_TYPE|=0x4`,
  `MISSION_REQUEST_LIST` (plan + DO_SPRAYER=216 discovery),
  `PARAM_REQUEST_READ`/`PARAM_SET`, `COMMAND_LONG DO_SET_MODE` (**RTL/LAND
  only** — arming/takeoff stay in MP), `STATUSTEXT` notices.
- Discipline (ported from validated hehe logic): **subscribe-before-send** on
  every request/response pair; the UI never sends arm/takeoff/mission-write.

## QR — 4 independent routes

| route | path | needs |
|---|---|---|
| pi-ws | Pi → UI socket | Pi network |
| mavlink | Pi → Pixhawk → radio → MP forward → bridge STATUSTEXT parse | MP link |
| mp-log | MP log file → bridge tail → UI | watch path set |
| manual | operator types payload into UI | always |

First payload wins the display; all four receipts are timestamped in the QR card.

**Wire format.** The Pi sends `QR:<payload>` as a `STATUSTEXT` (truncated to the
50-char MAVLink limit; `qr_relay.py` resends it a few times so a lossy mirror
still delivers). The bridge parses it with

```python
QR_RE = re.compile(r"QR\s*:\s*([0-9A-Za-z][0-9A-Za-z._+-]{0,31})")
```

in both `bridge/mp_bridge.py` and `bridge/mav_client.py` (`tests/test_qrlatch.py`
fails if they drift apart). Two hard rules:

* The **colon is required**. A looser `QR[:\s]+` matched ordinary prose — "the
  QR code was missed" latched payload `code`, and the Pi's own
  `QR CONFIRMED via bottom: 'X'` log line latched `CONFIRMED`. With
  first-payload-wins latching, one false positive poisons the whole hunt.
* The pattern is **identical** in the bridge and the MAVLink client, and the
  payload ceiling is 32 chars (the spec payload is 2 digits; panels on the bench
  use `MISSION-QR-001`-style ids, which the old `[0-9A-Za-z]{1,6}` silently
  dropped — dashes and length included).

**Latching.** `Hub` stores the first `mp-qr` payload from *any* route and
`bridge_state()` returns `bridge.qr or hub.qr`, so `/api/mp/state` and the
`bridge-state` SSE event carry the result for clients that connect later.

**MAVLink version.** The bridge accepts MAVLink 1 and 2 frames on input
(`_mavlink_v2.parse_datagram`) and always sends v2 — MP mirrors whatever the
vehicle speaks, and a v1-only link used to deliver nothing at all.
`tests/test_mavv1.py` covers both against pymavlink.

## Tiles (offline-first)

- The bridge serves MBTiles at `/tiles/{z}/{x}/{y}.png` with XYZ→TMS row flip
  (`y' = 2^z − 1 − y`); 404 JSON when the pack/zoom is missing.
- Build a venue pack (needs net ONCE): `tools/fetch_tiles.py --bbox minlon,minlat,maxlon,maxlat
  --zoom-min 15 --zoom-max 19 --out venue.mbtiles`, then run with
  `--mbtiles venue.mbtiles`.
- UI source order: `offline-mbtiles` (default when `tiles-info.available`) →
  `osm` → `carto-dark`, with auto-switch on tile-error streaks. Leaflet is
  vendored — no CDN at the venue.

## UI invariants

1. Flight truth comes ONLY from bridge SSE (never the Pi).
2. The Pi link is receive-only except camera tuning.
3. A fence counts as applied only with `loaded && confirmed` (FC readback).
4. The only flight commands are RTL/LAND via the MP link.
5. Map + grid work fully offline (vendored Leaflet + local pack).
6. The Pi link degrades, never dies silently: **probe** (`GET /health`, reported
   verbatim in the Pi-link box: `up 12 ms · link OK · cams cam2 · ready`, or
   `unreachable (<error>)` with the fix list) → **WS** (`open · <url>`) →
   **polling** (`down · retry N · polling HTTP`, polling `/api/fsm/status` +
   `/api/qr/status` every 2 s). A Pi that answers HTTP but refuses the socket is
   amber with the exact remedy (`pip install "uvicorn[standard]"`), not red.
7. First QR payload wins, and once latched it is reported to clients that
   connect later.

## Changelog

- **v2.1 (2026-09-14)**: Pi link reliability pass — the Pi's HTTP server binds
  **before** bring-up, so `/health` and the new `/api/bringup` answer (with the
  failing subsystem and reason) even with no flight controller; supervised FC
  link (`link_lost`/`link_retry`/`link_restored`, `fc.reconnects`/`watchdog` in
  `/health`); `system` channel carries host/bring-up health and the latest
  envelope per channel is replayed on connect; MJPEG streams adopt cameras
  discovered after startup; UI probe + polling fallback for the Pi link.
  Bridge: QR regex tightened to require the `QR:` colon (prose no longer
  latches a payload) and widened to 32 chars of `[A-Za-z0-9._+-]`; first
  payload latched into `bridge-state`; MAVLink 1 frames accepted as well as
  MAVLink 2; `mavlink-state` gains `rx_v1`/`rx_v2`/`rx_bad_crc`/`bad_frames`/
  `unknown_ids`.
- **v2 (2026-09-13)**: MP-forwarded MAVLink replaces the Pi radio path; Pi WS
  drops `telemetry`/`fence`; fence/plan/telemetry/console are bridge SSE;
  dual cameras with live controls; fixed-anchor culled meter grid; vendored
  Leaflet; QR gains the `mavlink` STATUSTEXT route (4 total).
- v1: initial contract — Pi owned MAVLink/fence over USB, 3 QR routes.
