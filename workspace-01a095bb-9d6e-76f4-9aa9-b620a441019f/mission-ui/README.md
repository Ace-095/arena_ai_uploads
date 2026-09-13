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
   → connect. Draw fence → *APPLY FENCE → MP* → confirm the polygon in MP's
   Fence tab before flight.

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
python3 bridge/mp_bridge.py --selftest            # SELFTEST PASS
node --check js/app.js js/map.js js/links.js js/camera.js
```

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
