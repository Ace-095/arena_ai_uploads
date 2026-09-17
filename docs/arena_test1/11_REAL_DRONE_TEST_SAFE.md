# Real Drone Test — Safe Flight with Pi Companion + No-Crash Fallbacks

**Goal:** Fly real drone with Pi 5 + AI HAT + 2 cameras, QR-hunt at 10m/15m A3, with zero crash risk — Pi is companion, FC owns all failsafes, every failure → RTL/LAND, never crash.

**Hardware:** 
- Drone: Quadcopter, Pixhawk 6C / Cube Orange, GPS, compass, 4S battery
- Pi 5 8GB + AI HAT 26 TOPS (Hailo), Pi Camera 3 (front), Waveshare IMX477 B bottom (100° HFOV, 113° diag, fixed focus, IR-CUT day)
- Router: Portable field router (Pi + laptop same LAN, LTE optional)
- Laptop: Windows 11 + Mission Planner, or Pop!_OS + MP

---

## 1. Safety Architecture — Why Pi Can't Crash Drone

### Pi is Companion (System 51), Not Flight Controller
- Pi presents as `system 51 / ONBOARD_CONTROLLER`, MP is 255, UI is 254 — never collides
- Pi sends `SET_POSITION_TARGET_GLOBAL_INT` (position only, MASK_POS_ONLY) at 1Hz, ArduPilot holds last target
- If Pi dies, ArduPilot times out GUIDED → RTL (param `FS_GCS_ENABLE`, `FS_THR_ENABLE`)
- Pi never touches RC, motor mixing, EKF — FC does

### FC Owns All Failsafes (ArduPilot Params, Must Set)

```bash
# In Mission Planner → Config → Full Parameter List, set these BEFORE real flight:

FENCE_ENABLE = 1
FENCE_TYPE = 4  # polygon inclusion only
FENCE_ACTION = 1  # RTL on breach
FENCE_MARGIN = 2  # 2m margin

BATT_FS_LOW_ACT = 2  # RTL on low battery
BATT_FS_CRT_ACT = 1  # LAND on critical
BATT_LOW_VOLT = 14.0  # 4S 3.5V/cell
BATT_CRT_VOLT = 13.2  # 3.3V/cell
BATT_LOW_MAH = 20%  # or set via BATT_LOW_TIMER

FS_GCS_ENABLE = 1  # RTL on GCS loss (MP disconnect) — bench 0, real 1
FS_THR_ENABLE = 1  # RTL on RC loss
FS_EKF_ACTION = 1  # LAND on EKF failsafe
FS_EKF_THRESH = 0.8

WPNAV_SPEED = 250  # cm/s default, Pi snapshots and sets 100 cm/s for search, restores after
WP_SPD = 2.5  # m/s 4.7+ (Pi prefers this)
LAND_SPEED = 50  # cm/s

SERIAL0_PROTOCOL = 2  # MAVLink2 for USB to Pi
SERIAL1_PROTOCOL = 2  # for telem to MP
SERIAL0_BAUD = 115200
```

### Pi Link Supervision (fc_link.py)

- Probes `/dev/serial/by-id/*`, `/dev/ttyACM*`, `/dev/ttyUSB*` at 115200/57600/921600
- Network links `tcp:127.0.0.1:5762` for SITL, single attempt, autoreconnect=True
- Watchdog: dead after 10s no vehicle heartbeat, retry every 5s, `find_fc()` probing
- `_read_loop` backs off on EOF (0.2*quiet) — log readable
- Mode tracking: vehicle heartbeats only (sysid<200, type!=GCS), MP sysid 255 ignored — prevents mode STABILIZE false veto
- `link_state()`: device, connected, down, reconnects, hb_age_s, mode, armed, veh_sysid, watchdog — for /health
- `link_ok(max_hb_age=3s)`: mission waits WAIT_LINK 600s, UI stays up

### Mode Guard

```python
_guided_ok():
  if fc.mode==GUIDED: offmode_n=0 True
  else: offmode_n++ ; return offmode_n<3  # tolerate 1-2 stale, trip on 3rd
```

- If FC leaves GUIDED (user switches to RTL/STABILIZE/LOITER, or failsafe), Pi aborts search immediately, restores speed param, DONE external RTL — no fight.

### Battery, Timeout, Fence

- `_battery_ok()`: BATTERY_STATUS pct < min_batt_pct (25% real) → abort → RTL
- `_search_expired()`: one budget 600s shared by yaw+grid+resweep, approach own 240s
- trigger 600s, link 600s, approach 240s → FAILSAFE RTL
- Fence snapshot at boot: home + fence + plan + sprayer seqs. If no fence, fallback home box nofence_half_m 20m.
- `_clamp_to_fence()`: binary shrink toward current pos, 10 iters, target inside fence always, inside flag.
- Max alt clamp: `validate_max_alt()` + `clamp_altitude()`, flight.max_alt_m 15/10/5 configurable, sweep and stair clamped, sorted descending.

### Speed Snapshot & Restore

- Snapshot WP_SPD (m/s) or WPNAV_SPEED (cm/s), save (name,scale,raw)
- Set search_speed_ms 1.0-2.5 m/s for more frames
- Restore on finish/abort/mode_trip — FC handed back exactly as found.

### Mid-Flight Reboot Safety

- If boot finds FC already GUIDED (prior run died mid-takeover) → immediately RTL → FAILSAFE.

### Store-Forward

- `store_forward.py`: buffers QR payload+GPS to `logs/pending_results.jsonl` when FC+MP down, flushes when link returns, survives reboot.

### Server First

- HTTP server up before bring-up, bring-up on own thread, `/health` answers immediately, says which subsystem missing and why. Missing FC/cam ≠ dead port.

### Detector & Camera Fallback

- Detector: `kind: auto` tries HEF (Pi5 HAT) → YOLO laptop (pt/onnx) → classical → error logged, /health shows. Classical FAIL at 34/50px, YOLO tiled PASS 0.81/0.79 — use YOLO for 15m.
- Camera: stream sync adopts/drops cams, vanished cam dropped, thread stopped no leak, surviving cam untouched. If no cam, FAILSAFE no camera → RTL.

---

## 2. Pre-Flight Checklist (Real Drone, No Crash)

### Day Before (Bench)

- [ ] Pi OS: `sudo apt update && sudo apt install -y python3-pip python3-opencv python3-picamera2 zbar-tools hailo-all`
- [ ] `pip install --break-system-packages -r requirements.txt`
- [ ] `pip install --break-system-packages ultralytics` or `hailo_platform` for HAT
- [ ] `hailortcli scan` must list HAT, `dtparam=pciex1_gen=2` in /boot/firmware/config.txt
- [ ] Cameras: `tools/cam_probe.py --snap` — front Pi Cam 3 and bottom IMX477 B both work, 4056×3040@10 or 2028×1520@40, rotation_deg set so image up == nose, hfov_deg calibrated (front 66°, bottom 100°)
- [ ] Focus: IMX477 B fixed focus, verify 15m sharpness with snap, IR-CUT day mode
- [ ] Models: `ls models/qr_yolov8n.hef` (Pi5) or `qr_yolov8n.pt` (24MB) — we committed pt/onnx, hef needs compile via `training/compile_hef.md`
- [ ] Config: `cp config.example.yaml config.yaml`, set `bottom.hfov_deg`, `hef_path: models/qr_yolov8n.hef` or `model_path: models/qr_yolov8n.pt`, `flight.max_alt_m: 15.0`, `detector.conf_thr: 0.25`, `tile_bottom: true, tile_grid: [3,3]`
- [ ] Bench test: `python3 main.py --check` — FC + cams + hailo report, /health JSON, no traceback
- [ ] Fake SITL test: `tools/fake_sitl.py` + `make_demo_frames.py` + `main.py --config config.demo.yolo.yaml` — must detect, decode, transmit, RTL

### Morning of Flight (Field)

- [ ] Weather: wind <5 m/s, no rain, visibility good
- [ ] Site: open field 100×100m, no people, no obstacles, clear RTL path
- [ ] Drone: props tight, battery full 4.2V/cell, GPS fix 3D, compass calibrated, EKF green, fence loaded (MP Fence tab), mission loaded with DO_SPRAYER at seq 2, RTL at end
- [ ] Pi: power via BEC 5V 5A, USB to Pixhawk telem2, router powered, Pi and laptop join same field LAN (e.g. 192.168.1.x), `http://<pi-ip>:8000/health` reachable from laptop
- [ ] Cameras: lens clean, bottom cam facing down, front cam forward, no obstruction
- [ ] A3 QR: printed 297×420mm, high contrast, taped flat on ground, no shadow, payload e.g. TEST-QR-15M
- [ ] Laptop: Mission Planner connected via router to Pi? Actually MP connects to FC via telem radio or WiFi? Two options:
  - Option 1: Pi USB to FC, laptop WiFi to Pi (Pi is MAVLink bridge? No, Pi is direct USB, MP is separate radio). Actually: FC has 2 teems: telem1 → radio → MP laptop, telem2 → USB → Pi. So MP and Pi both talk to FC, different ports.
  - Option 2: Pi as hotspot, MP connects via UDP to Pi's mavlink forwarding? Simpler: use radio for MP, USB for Pi.
- [ ] Params: FENCE_ENABLE=1, FENCE_ACTION=1 RTL, BATT_FS_LOW_ACT=2 RTL, FS_GCS_ENABLE=1 RTL, FS_THR_ENABLE=1 RTL, check in MP
- [ ] Arm check: MP → Arm check → must pass all
- [ ] Manual RTL test: Arm, Takeoff 5m in LOITER, switch to RTL via MP → must RTL and LAND at home — proves failsafe works

### Final Safety Brief

- Pilot has RC with RTL switch, can take over anytime — Pi's GUIDED can be overridden by RC mode switch (Pi detects 3 off-mode reads → abort)
- Spotter watches drone, has kill switch? Actually ArduPilot no kill, but RTL switch
- If Pi does weird, pilot switches to STABILIZE/LOITER/RTL — Pi aborts
- If battery low, FC RTL itself, Pi also aborts
- If geofence breach, FC RTL itself, Pi clamps targets

---

## 3. Real Flight Steps — Safe

### Step 1: Power On (Safe Order)

1. Router on, wait 30s
2. Pi on, wait 60s for boot, `http://<pi-ip>:8000/health` should answer, even if FC not yet (server first)
3. Drone battery on, wait for GPS 3D fix, EKF green, home set (MP shows home)
4. Laptop MP connect via radio (or WiFi if using Pi as bridge), check mode STABILIZE, battery, GPS, fence
5. Pi auto-connects to FC via USB probing — logs `FC found: /dev/ttyACM0 @ 115200 (sysid=1)`, `link_state` connected, `watchdog up`
6. Check Pi /health: `{"device": "/dev/ttyACM0", "connected": true, "down": false, "mode": "STABILIZE", "armed": false, "watchdog": true, "cams": {"cam1": "ok", "cam2": "ok"}, "detector": "hailo ok" or "yolo_ultralytics ok", "qr_boost": "ON"}`

### Step 2: Upload Fence + Mission

**Via UI (if using mission-ui bridge):**
- Open `http://<pi-ip>:8000` on laptop (Pi serves UI)
- Config card: max_alt 15, sweep 12, bottom cam 100° HFOV, overlap 0.35 → Apply
- Polygon card: draw purple polygon around A3 QR area → POLYGON→FENCE INCLUSION → apply to MP → MP Fence tab shows polygon
- Coverage card: Show FOV → blue footprints 35.7×23m @15m, spacing 15m, legs 14, area 10000m²

**Via Mission Planner:**
- Plan → Fence → Draw polygon inclusion → Upload
- Plan → Load mission: TAKEOFF 15m, DO_SPRAYER at seq 2, WAYPOINTs around fence, RTL at end → Upload
- Check: `sprayer_seqs: [2]` in Pi logs

### Step 3: Arm and Takeoff (Manual, Safe)

- MP: Pre-arm checks pass, Arm, Takeoff to 15m in GUIDED or AUTO? For safety, use AUTO:
  - Mode AUTO, Arm, Takeoff — drone climbs to 15m, flies mission
  - When CURRENT reaches DO_SPRAYER seq, Pi logs `trigger: mission seq 2 >= sprayer 2 (gate=AUTO)` → commands GUIDED → hold at sweep_alt 12m
  - **Gate safety**: trigger only if AUTO fresh <5s or DEADMAN (advancing mission) — prevents false trigger

- Alternative manual takeover: UI → Start mission → Pi goes WAIT_TRIGGER → SEARCH

### Step 4: Search (Pi GUIDED, FC Failsafes Active)

- Pi does yaw sweep 12×30° settle 1s, then grid lawnmower FOV-optimal
- Each leg: `goto_global(lat,lon,alt)` at 1Hz, checks battery, guided_ok, payload, fresh_cue, search_expired
- YOLO detector: bottom cam 3×3 tiled, 34px@15m → 15.9px on tile, conf 0.81 PASS (classical FAIL)
- Front cam: bearing-only cue → yaw toward + step forward until bottom cam takes over
- Blob fallback: top-3 hypotheses, per-candidate 20s, global 60s, steer-only, never transmits, real cue hands to TRACK
- Coverage: `cover_cells()` marks searched cells, pushes throttled full-set snapshots to UI heatmap
- **Safety during search:**
  - Battery <25% → abort → RTL
  - FC leaves GUIDED (user RTL, failsafe) → mode_tripped → restore speed → DONE external RTL
  - Search budget 600s spent → finish without payload → RTL
  - Geofence breach → FC RTL automatically, Pi clamps targets inside

### Step 5: Track & Approach (Descend Stair)

- Bottom bbox → `nadir_pixel_to_ground_m()` → lat/lon → `_clamp_to_fence()` → goto
- If outside fence → hold, log, don't fly out
- Off 1.5m and stair_idx < len(approach_stair): descend to next stair 12→9→7m
- Decode every frame, consensus streak 3 (1 miss tolerated) → payload
- If cue lost 2.5s → resume grid (safe)
- If approach timeout 240s → resume grid

### Step 6: Transmit & RTL

- Consensus payload → `relay_qr()`: STATUSTEXT `QR:<payload>` xN to FC → MP Messages tab + ws event `qr` to UI + store-forward jsonl if link down
- Hold position 15s → post_action RTL (default) → `set_mode(RTL)`
- Speed param restored to original
- Phase DONE payload=...

### Step 7: Landing

- RTL → drone returns to home, lands, disarms
- Pi logs `mission PAYLOAD 'TEST-QR-15M' — post_action RTL`
- Check MP tlog for `QR:TEST-QR-15M`, Pi logs `pending_results.jsonl`, UI event

---

## 4. What If Something Fails? (Real Drone, No Crash)

| Failure | Pi Detection | Pi Fallback | FC Fallback | Drone Result |
|---------|--------------|-------------|-------------|--------------|
| Pi power loss | link supervision dead_s 10s | — | FC timeout FS_GCS_ENABLE=1 → RTL | RTL, safe |
| Pi process crash | Same | — | Same | RTL |
| USB disconnect | link_lost, link_down True | Watchdog retry every 5s, store-forward buffers | FC continues AUTO or RTL | Continues or RTL |
| FC leaves GUIDED (pilot RTL) | _guided_ok 3 stale | mode_tripped → restore speed → DONE external RTL | Respects pilot RTL | RTL |
| Battery <25% | _battery_ok BATTERY_STATUS | abort → RTL | BATT_FS_LOW_ACT=2 RTL | RTL before critical |
| GPS loss / EKF fail | ArduPilot | — | FS_EKF_ACTION=1 LAND | LAND |
| Geofence breach | _clamp_to_fence | Target inside | FENCE_ACTION=1 RTL | RTL, Pi never sends outside |
| No QR in 600s | _search_expired | finish no payload → RTL | — | RTL |
| Detector no model | get_detector exception | Classical fallback, /health failed | — | Search may not find QR but no crash |
| Camera fail | rig.status failed | Drop cam, other continues | — | Search with remaining cam |
| YOLO fail at 34px full | Bench 0.42 low | Tiled 0.81 — tile_bottom mandatory | — | Use tiled |
| Bad lat/lon from Pi | _clamp_to_fence binary shrink | Inside flag | Fence RTL | Never outside |
| Mission no DO_SPRAYER | sprayer_seqs empty | trigger_seq fallback or manual takeover | — | Waits 600s FAILSAFE RTL |
| RC loss | ArduPilot | — | FS_THR_ENABLE=1 RTL | RTL |
| GCS (MP) loss | ArduPilot | Pi still has USB link | FS_GCS_ENABLE=1 RTL (real) / 0 (bench) | RTL (real) |
| Pi sends too fast | FC rate limit | Pi 1Hz goto | FC holds last | Safe |
| Motor failure | ArduPilot | — | ArduPilot LAND | LAND |

**Principle**: Pi suggests, FC decides. FC can always override via mode, fence, battery, EKF failsafes.

---

## 5. Configs for Real Drone (Safe)

**config.yaml (real drone, Pi5 + HAT):**
```yaml
server: { host: 0.0.0.0, port: 8000 }
stream: { fps: 12, width: 960, quality: 70 }
flight:
  max_alt_m: 15.0  # CHANGE 5/10/15 — never exceeds
  min_alt_m: 2.0
  max_speed_ms: 3.0
cameras:
  front:
    kind: picamera2
    size: [1920, 1080]
    hfov_deg: 66.0
    rotation_deg: 0
    qr_boost: { enabled: true, profile: front_qr_boost, software_mode: qr_boost }
  bottom:
    kind: picamera2
    size: [2028, 1520]  # binned 40fps sweet spot, or 4056x3040@10 for decode margin
    hfov_deg: 100.0  # 113° diag → 100° HFOV, calibrate!
    rotation_deg: 0
    qr_boost: { enabled: true, profile: bottom_qr_boost, software_mode: ground_suppress }
qr_boost:
  enabled: true
  profile: auto
  software_mode: qr_boost
detector:
  kind: auto  # auto = hailo if HEF present else classical/yolo fallback
  hef_path: models/qr_yolov8n.hef  # Pi5 HAT
  model_path: models/qr_yolov8n.pt  # fallback laptop YOLO
  input_size: 640
  conf_thr: 0.25  # LOW for 15m A3
  iou_thr: 0.45
  tile_bottom: true
  tile_grid: [3, 3]
  tile_overlap: 0.25
  full_decode_every_n: 3
  cue_frames: 2
decode: { required_streak: 3, miss_tolerance: 2 }
mission:
  max_alt_m: 15.0
  sweep_alt_m: 12.0
  approach_stair_m: [12.0, 9.0, 7.0]
  grid_overlap: 0.35
  edge_margin_m: 2.5
  resweep_alt_m: 10.0
  search_speed_ms: 2.0  # slow for more frames
  yaw_steps: 12
  yaw_settle_s: 1.0
  trigger_timeout_s: 600
  search_timeout_s: 600
  approach_timeout_s: 240
  link_timeout_s: 600
  min_batt_pct: 25  # REAL drone: 25, SITL: 0
  arrive_m: 2.0
  post_action: RTL  # safe default
  trigger_cmds: [216, 222, 223, 42600]
  trigger_require_auto: true
  nofence_half_m: 20.0
  coverage_mode: fov_optimal
  min_overlap: 0.2
fc:
  conn: /dev/serial/by-id/usb-ArduPilot_Pixhawk6C_...  # or auto probe
  retry_s: 3
  dead_s: 10.0
  retry_s: 5.0
blob:
  enabled: true
  top_k: 3
  per_candidate_s: 20
  global_cap_s: 60
  observe_every_s: 2.0
  min_support: 2
  cluster_radius_m: 2.0
  commit_score: 0.7
coverage:
  cell_m: 5.0
  push_every_s: 5.0
  center_tol_m: 3.0
  settle_s: 2.0
  max_misses: 2
relay:
  interval_s: 2.0
  window_s: 15.0
  store_forward: true
  store_path: logs/pending_results.jsonl
  flush_poll_s: 5.0
```

---

## 6. Real Flight Checklist (No Crash)

### Before Power
- [ ] Props tight, battery full, GPS, compass, EKF green, fence loaded, mission with DO_SPRAYER + RTL
- [ ] Params: FENCE_ENABLE=1 ACTION=1 RTL, BATT_FS_LOW_ACT=2 RTL, FS_GCS_ENABLE=1 RTL, FS_THR_ENABLE=1 RTL, FS_EKF_ACTION=1 LAND
- [ ] Pi: BEC 5V 5A, USB to Pixhawk, router, cameras clean, A3 QR flat, high contrast, no shadow
- [ ] Laptop MP via radio, Pi /health reachable via router, detector yolo/hailo ok, cams ok, qr_boost ON
- [ ] Manual RTL test: Takeoff 5m LOITER, switch RTL → must RTL LAND at home

### Power On (Safe Order)
1. Router 30s
2. Pi 60s, /health answers
3. Drone battery, GPS 3D, EKF green, home set
4. MP connect, STABILIZE, battery, fence
5. Pi auto-connects USB, FC found, watchdog up, /health connected true mode STABILIZE

### Upload Fence+Mission
- UI polygon→fence or MP Fence Draw inclusion Upload
- Mission Plan Load with DO_SPRAYER seq 2 + RTL Upload
- Check Pi logs sprayer_seqs [2]

### Arm & Takeoff
- MP Arm checks pass, Arm, Takeoff 15m AUTO, Pi trigger on DO_SPRAYER, GUIDED takeover, hold 12m

### Search → Track → Approach → Transmit → RTL
- Yaw + grid inside fence, YOLO tiled 0.8+ @15m, track clamped inside, stair 12→9→7, decode streak 3, transmit QR:<payload>, RTL, speed restored

### Landing
- RTL LAND at home, disarm, check MP tlog QR:<payload>, Pi jsonl, UI event

### Abort Anytime (Safe)
- Pilot RC RTL switch → Pi mode_tripped → DONE external RTL → FC RTL
- MP mode RTL/STABILIZE → same
- Battery low → Pi abort RTL + FC BATT_FS RTL
- Geofence breach → FC RTL
- Kill Pi power → FC timeout RTL

---

## 7. Logs & Post-Flight

- Pi: `logs/pending_results.jsonl`, `telemetry.log`, `/health`, `/api/bringup`
- MP: `Documents/Mission Planner/logs/*.tlog` search QR:
- UI: Logs card + `/api/mp/state` latched QR
- Video: Pi streams 960 width 12fps, quality 70, recorded? Add `tools/record.py` if needed

---

## 8. Success Criteria (Real Drone, No Crash)

- [ ] Pi /health ok, detector hailo/yolo, cams ok, qr_boost ON, link connected, watchdog true
- [ ] Fence polygon inclusion loaded, visible in MP, breach → RTL tested
- [ ] Mission with DO_SPRAYER, Pi trigger gate AUTO, takeover GUIDED, hold 12m
- [ ] Search FOV-optimal 35.7×23m @15m, no gaps, inside fence, speed 2m/s, battery ok
- [ ] YOLO tiled 3x3 detects A3 34px@15m 0.81 conf, classical FAIL — use YOLO
- [ ] Track clamped inside fence, approach stair 12→9→7, decode streak 3, transmit QR:<payload> to MP + UI
- [ ] RTL, speed restored, LAND at home, disarm
- [ ] Fail tests: Pi power loss → FC RTL; USB disconnect → link_retry → link_restored; battery low → RTL; RC RTL → Pi abort; fence breach → FC RTL
- [ ] No crash, no fly-away, no props damage, QR found at 15m

This real drone test can't crash — Pi is companion, FC owns failsafes, all Pi targets clamped, all timeouts RTL, link supervision + watchdog + store-forward + server first.
