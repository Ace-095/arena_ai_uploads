# Mission Companion — UI (Workstream 1)

Ground-station UI for the MP-first drone mission. **Mission Planner is the head**
(arming, home point, plan, DO_SPRAYER trigger). The UI: draws the geofence (→ Pi → Pixhawk),
shows everything live (map, telemetry, Pi system info, camera feed with click-to-enlarge,
QR result with 3 delivery routes, full logs).

> Status: first working iteration. Nothing here is "trusted" — everything gets
> bench → SITL → Gazebo verification (see `our-mission.md` §6 working rules).
> hehe repo code is a candidate source, not a guarantee.

## Files

```
index.html            cockpit page (vanilla JS modules, no build step)
css/app.css           dark cockpit theme
js/geo.js             ENU math (must stay identical to bridge + Pi)
js/map.js             Leaflet map, meter grid (1 m default), fence drawing, markers
js/links.js           Pi WS client + local bridge SSE client
js/camera.js          WebRTC → polling fallback, enlarge modal
js/app.js             state, panels, wiring
vendor/leaflet/       Leaflet 1.9.4 vendored locally (UI works fully offline)
bridge/mp_bridge.py   THE laptop app: serves this UI + tails MP logs + extracts QR
                      (+ --mock: full simulated Pi implementing docs/api-contract.md)
docs/api-contract.md  the contract: UI + Pi + mock all follow it
```

## Run it

**Demo (no hardware, no Pi):**
```bash
python3 bridge/mp_bridge.py --mock          # UI + mock Pi on http://localhost:8100
```
Open http://localhost:8100 — the mock runs the whole mission automatically
(IDLE → … → DECODED "42" → TRANSMIT → RTL → LAND → COMPLETE). Draw a fence on
the map and hit "Apply fence" — the mock stores it and the next search uses it.
Restart the mock run from the Controls card.

**Real (competition laptop, Windows 11):**
```bash
python bridge/mp_bridge.py --watch "C:\path\to\MP session log" 
```
1. In the UI "Pi Link" panel enter the Pi address (`http://<pi-ip>:8000`) → Connect.
2. Point the bridge at MP's log files (MP session log text file, or the
   `Documents\Mission Planner\logs\…` folder for .bin — needs `pip install pymavlink`
   on the laptop for .bin parsing; text tail + manual entry work without it).
3. Draw the fence → Apply. MP's console gets the `FENCE: n verts` STATUSTEXT instantly;
   open MP's Fence tab to see it. `.lap` export = manual fallback (bench-test in MP).

## QR result — 3 independent delivery routes (all shown in the QR card)

| route | path | needs |
|---|---|---|
| PI (WS) | Pi → router/LTE → UI | network uplink |
| **MP log (local)** | Pi → Pixhawk → radio → **MP log file → bridge → UI** | **nothing — works with zero uplink** |
| manual | operator types the 2 digits into the UI | always (5-second fallback) |

The 1st-priority transmission (Pi side, workstream 2) is STATUSTEXT `QR:<payload>`
over the Pixhawk link so it lands in MP's message console regardless of network.

## Open items (bench/test list — do NOT assume working)

- [ ] .lap fence file format → load in Mission Planner, confirm Fence tab (frame 26 / cmd 5001)
- [ ] MP session log file path/format on our Windows 11 machine (set the bridge watch path)
- [ ] WebRTC on the real Pi (aiortc) — UI already falls back to polling automatically
- [ ] 1 m grid density at real zoom levels on the venue map
- [ ] camera controls → real libcamera behavior on Pi Cam 3 (exposure/gain/AF ranges)
- [ ] OSM vs offline MBTiles at the venue (offline db = workstream 2, `/tiles` endpoint)
