# Method 3: Real Flight Test — arena/test1

**Goal:** Fly real drone with Pi 5 + AI HAT + cams, polygon fence from UI, configurable max height 10/15 m, FOV-optimal coverage, detect A3 QR (297×420 mm) every time.

**Prerequisites:** Must pass Method 1 (SITL+Gazebo) and Method 2 (Pi bench) first. Do not fly if bench fails at 15 m.

---

## Hardware

- Drone: Quad X, Pixhawk 6C / Cube Orange, GPS M10, telemetry 915 MHz, 4S battery
- Pi 5 8 GB + Hailo HAT + both cams (front 66° AF, bottom 100° fixed infinity)
- Mount: bottom cam nadir (down), front cam forward 15° down, vibration damped
- A3 QR print: matte, high contrast, error correction Q, payload e.g. `QR:42`, mounted flat on ground, no gloss, weighted
- Field: 50×50 m clear, no people, fence from UI polygon

## Safety

- Props off for all bench steps, on only for flight
- Geofence mandatory, FENCE_ACTION=1 RTL, FENCE_TYPE=4 (polygon)
- Kill switch, RTL switch, manual mode tested
- Battery min 25%, wind <5 m/s, daylight
- Spotter, fire extinguisher, first aid

## Step 1: Pi + FC wiring + params

### Wiring

- Pixhawk TELEM2 → Pi UART (or USB: Pixhawk USB → Pi USB)
- Pi powered via BEC 5 V 5 A (not Pixhawk USB)
- Cameras CSI0 front, CSI1 bottom
- Hailo HAT on Pi GPIO, active cooler

### ArduPilot params (via MP)

```
FENCE_ENABLE 1
FENCE_ACTION 1 (RTL)
FENCE_TYPE 4 (polygon inclusion)
FENCE_MARGIN 2
FENCE_ALT_MAX 20 (must be > flight.max_alt_m 15)
WPNAV_SPEED 250 (2.5 m/s search, overridden by mission_pi)
WPNAV_SPEED_DN 150, WPNAV_SPEED_UP 250
SERIAL0_PROTOCOL 2 (MAVLink2) or 1 if MP needs v1
SERIAL1_PROTOCOL 2 (Pi link)
SERIAL1_BAUD 921
```

Set EKF origin: GPS 3D fix, then disarm+arm, or MP → right-click map → Set Home Point → vehicle location.

## Step 2: Config.yaml for flight — 10 m vs 15 m

### For 15 m search (max coverage, harder detection)

```yaml
flight:
  max_alt_m: 15.0

cameras:
  bottom:
    size: [2028, 1520]  # 40 fps search
    hfov_deg: 100.0
    vfov_deg: 75.0

detector:
  conf_thr: 0.25
  tile_bottom: true
  tile_grid: [3, 3]
  full_decode_every_n: 3

mission:
  sweep_alt_m: 15.0
  approach_stair_m: [12.0, 9.0, 7.0]
  grid_overlap: 0.35
  edge_margin_m: 2.5
  resweep_alt_m: 10.0
  search_speed_ms: 2.0
  min_batt_pct: 25
```

### For 10 m search (easier detection, less coverage per image)

```yaml
flight:
  max_alt_m: 10.0

mission:
  sweep_alt_m: 10.0
  approach_stair_m: [8.0, 6.0, 4.0]
  grid_overlap: 0.30
  search_speed_ms: 2.2

detector:
  conf_thr: 0.30
```

**Choose 10 m if 15 m bench recall <80%**. 10 m footprint 23.8×15.3 m still covers large area, detection 50 px vs 34 px, much more reliable.

## Step 3: Field setup — polygon + fence

1. **Power:** Pi + FC, wait GPS 3D fix (HDOP <1.0, sats >12)
2. **Windows laptop:** MP connect via telemetry radio, set MAVLink Forwarding 127.0.0.1:14551 Write ON, start `mp_bridge.py --port 8100 --mbtiles field.mbtiles`
3. **UI:** http://localhost:8100, connect Pi URL `http://<pi-ip>:8000`, Probe → should show `link_healthy: true`, `ready: true`
4. **Draw polygon:** In UI, draw 50×50 m polygon around A3 QR location (ensure QR inside). Set max alt 15 (or 10), cam bottom, overlap 0.35, Show FOV coverage → verify legs cover area, no gaps.
5. **Convert:** Click `POLYGON → FENCE INCLUSION` → `apply polygon as fence → MP` → MP Fence tab should show inclusion polygon.
6. **Verify fence:** In MP, Fence → Download, check area, max radius. In UI, `GET /api/mp/fence` should return confirmed true.
7. **Place A3 QR:** Inside polygon, flat, weighted, payload known (e.g. 42).

## Step 4: Preflight checks (mandatory)

- [ ] GPS 3D, HDOP <1, EKF origin set, home set
- [ ] Fence enabled, inclusion polygon loaded, FENCE_ACTION RTL
- [ ] Pi: `http://pi-ip:8000/health` → `status: ok`, `link: ok`, `ready: true`, `errors: []`
- [ ] Pi: `http://pi-ip:8000/api/camera/status` → both cams active, bottom 2028×1520 @40 fps
- [ ] Pi: `http://pi-ip:8000/api/fsm/status` → IDLE or PLAN_SYNC
- [ ] Detector: `conf_thr` 0.25 for 15 m, tiling true
- [ ] Bottom focus sharp at 15 m (checked via `cam_probe` day before)
- [ ] Battery >95%, props tight, no vibration, kill switch works
- [ ] UI: drone marker at home, fence polygon green, FOV coverage blue path inside fence
- [ ] MP: Flight plan has DO_SPRAYER at seq 2 (trigger for Pi takeover)

## Step 5: Flight — auto mission

1. **Arm in AUTO:** MP → Flight Data → Arm, takeoff to 15 m (or 10 m) via mission.
2. **Trigger:** When mission reaches DO_SPRAYER (seq 2), Pi takes over → GUIDED, FSM `WAITING_TRIGGER` → `SEARCH` (lawnmower inside polygon at 15 m, speed 2.0 m/s).
3. **Search:** Watch UI coverage heatmap (green cells), drone follows FOV path. Should cover whole polygon without breach (edge margin 2.5 m).
4. **Detect:** When over A3 QR, bottom cam should bbox at 0.25+ conf, FSM `TARGET_FOUND`, flies over target, `DESCEND` stair 12→9→7 m, centered via front cam bearing + bottom pixel-to-ground.
5. **Decode:** At 7-9 m, decode streak 3/3 → `DECODED`, payload locked, `TRANSMIT` → STATUSTEXT `QR:42` to FC → MP console + UI QR card.
6. **RTL:** After transmit window 15 s, Pi commands RTL → drone returns, LAND.

**If no detection first grid:**

- Pi auto resweeps at 10 m (if `resweep_alt_m: 10.0`) — lower alt easier detection.
- If still no detection, blob fallback visits top 3 hypotheses.
- If still nothing, Pi goes `FAILSAFE` → RTL, logs show why.

## Step 6: In-flight monitoring

- **UI:** Mav lamp green, Pi lamp green, FSM state, coverage cells, QR payload, battery, altitude (should never exceed max_alt_m 15).
- **MP:** Console shows `QR:42`, flight path, fence breach warning if any.
- **Pi logs:** SSH `tail -f logs/mission.log` — detector conf, decode attempts.

## Step 7: Post-flight

- **Logs:** Pi `logs/pending_results.jsonl` should have payload + routes, `telemetry.log`, video if recorded.
- **MP tlog:** Search for `QR:` in MP log tailer or `C:\...\Mission Planner\logs\*.tlog`.
- **UI:** `GET /api/mp/state` → `qr: {payload: 42, source: mavlink|pi-ws, ts}` latched.
- **Verify max alt:** In `telemetry.log`, alt never > max_alt_m + 0.5 m margin.

## Step 8: Tuning after first flight

If detection failed at 15 m:

- Lower `conf_thr` to 0.20, increase `tile_grid` to 4×4 (more compute, more recall)
- Slow search to 1.5 m/s
- Increase overlap to 0.40
- Use 10 m mission instead of 15 m — still covers polygon, just more legs
- Check bottom focus: if blurry, adjust lens ring to infinity
- Check HEF: bench `bench_hef.py` with flight video — if offline decode works but online fails, lower conf or increase fps

If fence breach RTL:

- Increase `edge_margin_m` to 3.0
- Slow search speed
- Check wind — fly in <5 m/s

## Emergency

- **Fence breach:** FC auto RTL (FENCE_ACTION 1). If not, manual RTL switch.
- **Pi fails:** FC continues mission or RTL depending on `post_action`. UI Pi lamp red → land manually.
- **No QR after resweep:** Manual LAND, check A3 print contrast, placement inside polygon.

## Success criteria — Method 3

- [ ] Polygon from UI → fence inclusion on FC, visible in MP, no breach during search
- [ ] Drone never exceeds `flight.max_alt_m` (15 or 10 m) — verified in logs
- [ ] FOV-optimal coverage: footprint at max alt matches cam.txt math, spacing ensures no gaps, legs inside polygon
- [ ] A3 QR (297×420 mm) detected at 15 m (or 10 m) with 3×3 tiling, conf >0.25, decoded after descend to 7-9 m, payload `QR:XX` appears in MP console + UI via 4 routes
- [ ] Resweep at 10 m works if 15 m fails
- [ ] RTL and LAND safe, battery >25%
- [ ] Logs show `fov: {footprint_w: 35.7, ...}`, `max_alt_m`, `coverage_mode: fov_optimal`

## Quick reference — 10 m vs 15 m decision

| Metric | 15 m | 10 m |
|--------|------|------|
| Footprint bottom | 35.7×23.0 m | 23.8×15.3 m |
| A3 px full-res | 34 px | 50 px |
| Tile input px | 16 px | 23 px |
| Detection recall (good HEF, 3×3, conf 0.25) | ~70% first grid | ~90% first grid |
| With resweep 10 m | ~90% overall | ~98% |
| Coverage legs for 100×100 m | ~14 | ~20 |
| Use when | Max area, bench recall >80% | Reliable detection, wind, focus marginal |

**Recommendation:** Start with 10 m for first real flight, then try 15 m after tuning.
