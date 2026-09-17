# Mission Companion — mission-ui (v2)

Ground-station UI for the MP-first drone mission. **Mission Planner is the head**
(arming, home, plan, DO_SPRAYER trigger). The laptop has no radio: MP forwards
MAVLink over UDP to the bridge, and the UI gets all flight truth that way.

```
Pixhawk <==radio==> MP <==UDP:14551==> bridge/mp_bridge.py ==> browser UI (SSE)
Pi 5 (FSM/cams/QR) <==WS receive-only==> browser UI
```

> Status: UI v2 rebuilt 2026-09-13 (see `ui-spec.md` v2 changelog). Backend
> verified: `tests/test_mavcodec.py` 23/23 byte-identical, `mp_bridge.py
> --selftest` PASS. Nothing is "trusted" until bench → SITL (our-mission.md §6).
>
> Field laptop? Read the full guide first:
> [`docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md`](docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md)
> (MP + UI setup, both offline-map systems, field-day checklist).

## Quickstart (mock — everything on one machine)

```bash
python3 bridge/mp_bridge.py --mock --origin 15.3647,75.1240
# open http://127.0.0.1:8100 — mock Pi + mock FC + bridge, all loopback
```

## Real setup (competition laptop)

1. **MP forwarding** (the one MP-side step): connect MP to the vehicle →
   right-click vehicle → *MAVLink Forwarding* → add `127.0.0.1:14551` → tick
   **Write access**. The UI's MAV lamp goes green; fence/RTL flow out this way.
2. **Bridge**: `python3 bridge/mp_bridge.py [--port 8100] [--mav-port 14551]
   [--watch "C:\path\to\MP log"] [--mbtiles venue.mbtiles]`.
3. **UI**: open `http://127.0.0.1:8100`, paste the Pi URL (`http://<pi-ip>:8000`)
   → connect (or press **Probe** — see below). Draw fence → *APPLY FENCE → MP* →
   confirm the polygon in MP's Fence tab before flight.

### Pi link: probe, then fall back instead of failing

The Pi link has three layers, and the UI reports which one it is on rather than
showing a bare red lamp:

1. **Probe** (`btnPiProbe` in the Pi-link box) — does an HTTP `GET /health`
   against the URL you typed and prints what actually happened: DNS/TCP failure,
   HTTP status, `link`, `ready`, the bring-up `stage` and any `errors[]`, plus
   the LAN URLs the Pi itself suggests. This is the difference between "the Pi
   is down" and "the Pi is up but has no flight controller".
2. **WebSocket** `ws://<pi-ip>:8000/ws/telemetry` — the normal path (envelopes
   `{channel, t, data}`: `system`, `fsm`, `mission`, `qr`, `coverage`, `log`,
   `event`, `telemetry`). On connect the Pi replays the latest envelope per
   channel, so a UI opened mid-flight is immediately correct.
3. **Polling fallback** — if the socket never opens (uvicorn installed without
   `websockets`/`wsproto` *silently 404s WebSocket routes* while HTTP works
   fine; or a proxy strips `Upgrade`), `links.js` polls `/api/fsm/status` + `/api/qr/status` every 2 s and the
   Pi lamp reads `polling` instead of `down`. Slower, but the mission is still
   visible.

The `system` channel carries the Pi's own health (host, temps, cpu/mem/disk,
bring-up state) so a failing Pi is diagnosable from the Windows side without
SSH.

### QR payload: first one wins, and it is remembered

Four independent routes can deliver the payload (Pi WS `qr`, MAVLink
`STATUSTEXT`, MP log tail, manual entry). Two rules the UI relies on:

* The bridge matches **`QR:<payload>`** — the colon is required, up to 32 chars
  of `[A-Za-z0-9._+-]`. Prose such as "the QR code was missed" or the Pi's own
  "QR CONFIRMED via bottom: 'X'" log line must **not** produce a payload; with
  latching, one false positive would poison the whole hunt.
* The **first** payload is latched and stays in `GET /api/mp/state` → `qr`, so a
  UI opened *after* the fly-past still shows the result. Later payloads still
  stream as `mp-qr` events to whoever is watching.

### MAVLink 1 and 2

The bridge's frozen codec (`bridge/_mavlink_v2.py`) **parses both** wire
versions on input and sends v2. MP mirrors whatever the vehicle speaks, so a
link on `SERIAL_PROTOCOL=1` (or older firmware) still delivers telemetry and
the QR `STATUSTEXT`. `GET /api/mp/mavlink` reports `rx_v1` / `rx_v2` /
`rx_bad_crc` / `bad_frames` (unusable bytes) separately from `unknown_ids`
(well-formed frames for message ids this bridge does not model — normal: SITL
streams VFR_HUD and friends all day).


## Offline map pack (laptop is offline in the field)

```bash
python3 tools/fetch_tiles.py --bbox 75.10,15.35,75.14,15.38 \
    --zoom-min 15 --zoom-max 19 --out venue.mbtiles   # needs net ONCE, beforehand
python3 bridge/mp_bridge.py --mock --mbtiles venue.mbtiles
```

Leaflet itself is vendored (`vendor/leaflet.*`) — the map works with zero
network as long as the pack covers the venue. `/api/mp/tiles-info` reports
coverage; the UI defaults to the offline pack and falls back to OSM/Carto.

## Verify

```bash
python3 -m py_compile bridge/*.py tools/*.py
/path/to/venv/bin/python tests/test_mavcodec.py   # 23/23 byte-identical
python3 tests/test_mavv1.py       # MAVLink1 + MAVLink2 parsing, drop reasons
python3 tests/test_qrlatch.py     # QR regex (payloads in, prose out) + latch
python3 bridge/mp_bridge.py --selftest            # SELFTEST PASS
node --check js/app.js js/map.js js/links.js js/camera.js
for t in tests/*.py; do python3 "$t" || echo "FAILED $t"; done
```

All of them are stdlib-only and need no hardware; the pymavlink cross-checks
inside `test_mavcodec.py` / `test_mavv1.py` skip themselves where pymavlink is
not installed (the field laptop).

## Repo map

| path | what |
|---|---|
| `index.html`, `css/`, `js/` | UI v2 (map, links, dual-cam, app wiring) |
| `vendor/leaflet.*` | offline Leaflet (no CDN at venue) |
| `bridge/mp_bridge.py` | UI host + MAVLink/MP bridge + mock Pi/FC |
| `bridge/mav_client.py` | GCS-side MAVLink client (telemetry/fence/plan/params/mode) |
| `bridge/_mavlink_v2.py`, `_mavtable_gen.py` | stdlib MAVLink v2 codec + message table |
| `bridge/mock_fc.py` | mock flight controller (UDP loopback) |
| `tools/fetch_tiles.py` | MBTiles venue-pack fetcher |
| `tools/generate_mavtable.py` | regen `_mavtable_gen.py` from pymavlink |
| `docs/api-contract.md` | the contract (v2) — read this first |

## Camera note

CAM1 (Pi Cam 3) + CAM2 (IMX477) sliders apply **live** (debounced POST to the
Pi) — no Apply button. This is the UI's only write path to the Pi; everything
flight-related goes through MP. WebRTC first, polling fallback per camera.

## Open items (bench/test list — do NOT assume working)

- [ ] `.lap` export → import in MP, confirm Fence tab (frame GLOBAL / cmd 5001)
- [ ] MP MAVLink-forwarding throughput + Write-access behavior on our machine
- [ ] `FENCE_ENABLE/ACTION/TYPE` params on our FC firmware (mock accepts all)
- [ ] WebRTC on the real Pi — UI falls back to polling automatically
- [ ] venue MBTiles pack coverage (z15–19, bounds check at site)
- [ ] camera live-controls vs real libcamera ranges on both sensors
- [ ] 1 m grid legibility at real venue zoom (auto-coarsen helps)
