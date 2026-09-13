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

- `system`: `{temp_c, cpu_pct, mem_pct, disk_pct, uptime_s}`
- `fsm`: `{state, reason?, acknowledged?{by, reason}}`
  (mock may send `{from, to, reason}` — UI reads `state ?? to`)
- `qr`: `{payload, streak?, required?, confirmed?}`
- `event`: `{type, ...}` — `target_detected {bbox, confidence, lat, lon}`,
  `qr_decoded {payload, altitude_m}`, `mission_result {payload, routes}`,
  `plan_synced {items, trigger_seq}`, `abort {from}`
- `log`: `{level, msg}`

The mock replays a short history burst on every WS connect.

## Pi REST v2 (`http://<pi>:8000`)

| method | path | body → reply |
|---|---|---|
| GET | `/health` | `{status, uptime_s, ...}` |
| GET | `/api/fsm/status` | `{state, acknowledged, ...}` |
| POST | `/api/fsm/start` | `{}` → started |
| POST | `/api/fsm/abort` | `{}` → aborted (debug only) |
| GET | `/api/qr/status` | `{payload, streak, required, confirmed}` |
| GET/POST | `/api/camera/status` | `{cam?}` → `{cam, model, controls}` |
| POST | `/api/camera/controls` | `{cam, exposure_us, gain_db, af_mode, adaptive, brightness, contrast, saturation, sharpness}` → `{ok, cam, controls}` |
| GET | `/api/camera/frame/cam1` `/api/camera/frame/cam2` | JPEG snapshot bytes (mock: SVG) |
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
| POST/DELETE | `/api/mp/fence` | `{vertices:[{lat,lon}], mission_type:'fence', fence_action?}` → GeofenceStatus |
| GET | `/api/mp/plan` | plan (fresh read from FC) |
| POST | `/api/mp/mode` | `{mode:'RTL'|'LAND'}` → `{status, mode, result, result_name, from_sysid}` |
| POST | `/api/mp/param` | `{name}` → `{name, value}` |
| POST | `/api/mp/qr` | `{payload}` → `{status, qr:{payload, source:'manual', ts, line}}` |
| POST | `/api/mp/watch` | `{path}` → `{watching}` |
| POST | `/api/mp/mock/restart` | `{}` → `{ok:true}` (mock only) |

`bridge-state`: `{watching, qr, bin_parser_available, mock, mavlink{connected,
port, vehicle_sysid, mode, armed}, tiles{...}, uptime_s}`

`mavlink-state`: `{connected, port, gcs_sysid, sender, vehicle_sysid, vehicle_type,
mode, mode_num, armed, vehicle_state, hb_age_s, rx_msgs, tx_msgs, rx_rate,
sender_changes, link_flaps, fence, plan}`

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
In `--mock` the Pi channels (`system`, `event`, `qr`, `fsm`, `log`) are relayed
on the same stream; the UI only uses them when its Pi WS is down.
First burst on connect: `bridge-state` + `mavlink-state` (+ `fence`/`plan`
if known).

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

## Changelog

- **v2 (2026-09-13)**: MP-forwarded MAVLink replaces the Pi radio path; Pi WS
  drops `telemetry`/`fence`; fence/plan/telemetry/console are bridge SSE;
  dual cameras with live controls; fixed-anchor culled meter grid; vendored
  Leaflet; QR gains the `mavlink` STATUSTEXT route (4 total).
- v1: initial contract — Pi owned MAVLink/fence over USB, 3 QR routes.
