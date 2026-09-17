# Two-Laptop Setup — Pop!_OS SITL+Gazebo + Windows 11 Mission Planner + UI — Safe, No-Crash

**Exactly what you meant:** 
- **Laptop 1: Pop!_OS** — Gazebo Harmonic + ArduPilot SITL + gz_cam_bridge (QR boost) + mission_pi with YOLO (A3 297×420 @10m/15m)
- **Laptop 2: Windows 11** — Mission Planner + mission-ui bridge (mp_bridge.py) + browser UI (polygon→fence→FOV coverage→fly)

Both on same WiFi/router (e.g. 192.168.1.x), no internet needed after setup. Pi companion never crashes drone — FC owns failsafes, all Pi targets clamped, timeouts→RTL.

---

## Network Diagram (Safe)

```
Pop!_OS Laptop 192.168.1.10 (SITL+Gazebo+Pi)              Windows 11 Laptop 192.168.1.20 (MP+UI)
┌─────────────────────────────────────────────┐          ┌──────────────────────────────────────┐
│ Gazebo Harmonic                             │          │ Mission Planner 1.3.80+              │
│  sim/worlds/mission_world.sdf               │          │  TCP 192.168.1.10:5760 ← SITL        │
│  iris_dualcam + qr_target A3 0.297×0.42m    │          │  Fence + Mission upload              │
│                                             │          │  Arm, Takeoff, AUTO, RTL             │
│ gz_cam_bridge :8099 (QR boost)              │          │  Messages tab QR:<payload>           │
│  front.mjpg boosted + bottom.mjpg boosted   │          │                                      │
│  /health JSON qr_boost ON                   │          │ mission-ui bridge :8100              │
│                                             │          │  bridge/mp_bridge.py                 │
│ ArduPilot SITL                              │          │  udp 127.0.0.1:14551 ← MP forwarding│
│  --out tcp:0.0.0.0:5760 (MP)                 │◄────────►│  http://localhost:8100 UI            │
│  --out tcp:0.0.0.0:5762 (Pi)                 │   WiFi   │   polygon→fence→FOV→fly              │
│  --out udp:192.168.1.20:14551 (optional MP) │          │   offline tiles field.mbtiles        │
│                                             │          │   logs tail, QR result               │
│ mission_pi :8000 (YOLO)                     │          │                                      │
│  config.gazebo.yolo.yaml                    │          │ Browser http://localhost:8100        │
│  kind: yolo pt 24MB conf 0.25 tile 3x3      │          │  + http://192.168.1.10:8000 (Pi)     │
│  http://0.0.0.0:8000 + /health              │──────────►│  Pi link: http://192.168.1.10:8000   │
│  Safety: server first, watchdog, fence clamp│   HTTP   │  Shows cams, boxes, FSM, coverage    │
└─────────────────────────────────────────────┘          └──────────────────────────────────────┘

Safety: FC owns failsafes (FENCE RTL, BATT RTL, EKF LAND, GCS/THR RTL). Pi companion sys 51 suggests GUIDED positions 1Hz, FC can override via mode switch. All Pi targets clamped inside fence. Timeouts→RTL. Link supervision 10s dead 5s retry.
```

---

## Pop!_OS Laptop — Setup (Laptop 1)

### 1. Install ArduPilot SITL + Gazebo

```bash
# ArduPilot
sudo apt update && sudo apt install -y git python3-pip python3-venv python3-opencv python3-gz-transport13 python3-gz-msgs10
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl && ./waf copter

# Gazebo Harmonic
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install -y gz-harmonic

# ArduPilot Gazebo plugin
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc

# mission_pi + YOLO
cd ~
git clone https://github.com/Ace-095/arena_ai_uploads.git
cd arena_ai_uploads
git checkout arena/01a0a7de-arena-ai-uploads
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
pip install --break-system-packages -r requirements.txt
pip install --break-system-packages ultralytics onnxruntime qrcode pillow
ls models/qr_yolov8n.pt models/qr_yolov8n.onnx  # 24MB + 12MB — we committed, YOLO mandatory for 15m
```

### 2. Check IP (Critical for 2-Laptop)

```bash
ip addr | grep 192.168
# e.g. 192.168.1.10/24 on wlp0s20f3
# This IP is what Windows MP will connect to: tcp:192.168.1.10:5760

# Test from Pop!_OS: ping Windows
ping 192.168.1.20

# Firewall: allow 5760, 5762, 8000, 8099
sudo ufw allow 5760/tcp
sudo ufw allow 5762/tcp
sudo ufw allow 8000/tcp
sudo ufw allow 8099/tcp
```

### 3. Gazebo World with A3 QR

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
# Check A3 size 0.297×0.42m
grep -A2 "size" sim/models/qr_target/model.sdf
# If not A3, edit or generate:
python3 sim/textures/generate_qr_texture.py --payload TEST-QR-15M --size 0.297 --out sim/models/qr_target/materials/textures/qr.png
```

---

## Pop!_OS — Run (4 Terminals, Safe Order)

**Terminal 1: Gazebo (must be first, no crash risk)**
```bash
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Wait: iris_dualcam + qr_target spawned, ground with QR white/black crisp
```

**Terminal 2: Camera Bridge with QR Boost (SYSTEM python3, not venv)**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose
# Check Pop!_OS: http://127.0.0.1:8099/ → front.mjpg + bottom.mjpg boosted, /health JSON qr_boost ON
# Check from Windows: http://192.168.1.10:8099/ should also work (if firewall allows)
# Safety: bridge read-only MJPEG, no FC commands
```

**Terminal 3: SITL (ArduPilot) — Binds 0.0.0.0 for Windows**
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# --out 0.0.0.0:5760 for Windows MP, 5762 for local Pi
# Wait EKF3 ready, GPS 3D fix, home set
# In MAVProxy:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4
param set FENCE_ACTION 1
param set FS_GCS_ENABLE 0  # bench 0, real drone 1
# Safety: SITL internal failsafes — geofence RTL, battery, EKF
```

**Terminal 4: mission_pi with YOLO (NOT classical — classical FAILS at 15m)**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.yaml
# config.gazebo.yolo.yaml:
#   detector.kind: yolo, model_path: models/qr_yolov8n.pt, conf_thr: 0.25 LOW for 15m, tile_bottom true 3x3 MANDATORY (34px→15.9px)
#   flight.max_alt_m: 15.0, sweep_alt_m: 12.0, approach_stair [12,9,7], grid_overlap 0.35 edge 2.5
#   qr_boost enabled true profile auto per-cam front qr_boost bottom ground_suppress
#   fc.conn: tcp:127.0.0.1:5762 (local SITL), server 0.0.0.0:8000
# Logs: FSM BOOT→WAIT_LINK→SNAPSHOT home fence WxH speed snapshot WP_SPD, detector yolo_ultralytics ready, cams ready, QR boost ON, /health answers
# Safety: server first, bring-up thread, link supervision dead_s 10s retry_s 5s
```

**Check Pop!_OS health:**
```bash
curl http://127.0.0.1:8000/health | python3 -m json.tool
# Should show: device tcp:127.0.0.1:5762 connected true down false mode STABILIZE watchdog true cams ok detector yolo ok qr_boost ON max_alt 15
```

---

## Windows 11 Laptop — Setup (Laptop 2)

### 1. Install Mission Planner

- Download from https://ardupilot.org/planner/docs/mission-planner-installation.html
- Install, run

### 2. Check Network (Critical)

```powershell
# PowerShell
ipconfig
# e.g. 192.168.1.20

ping 192.168.1.10
# Must ping Pop!_OS

# Firewall: allow MP
# Windows Defender Firewall → Allow app → Mission Planner → Private+Public tick
```

### 3. Clone mission-ui (for polygon→fence→FOV UI)

```powershell
git clone https://github.com/Ace-095/arena_ai_uploads.git C:\arena
cd C:\arena
git checkout arena/01a0a7de-arena-ai-uploads
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui

python -m venv .venv
.venv\Scripts\activate
# No pip needed — mp_bridge.py stdlib only, but for offline tiles:
pip install pillow

# Offline tiles (once with internet)
python tools\fetch_tiles.py --bbox 75.10,15.35,75.14,15.38 --zoom-min 15 --zoom-max 19 --out field.mbtiles
# ~50-100MB for 4x4km
```

### 4. Start mission-ui Bridge

```powershell
cd C:\arena\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
.venv\Scripts\activate
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles --watch "C:\Users\YOURNAME\Documents\Mission Planner\logs"
# Output: Mission Companion bridge http://localhost:8100, MAVLink GCS udp 0.0.0.0:14551, offline tiles
```

Open browser: http://localhost:8100

### 5. Mission Planner Connect to Pop!_OS SITL

1. MP → Top right: Select **TCP**, `192.168.1.10` (Pop!_OS IP), Baud `57600`, Click **Connect**
2. If fails: try `192.168.1.10:5760` in TCP dialog, or check Pop!_OS `sim_vehicle.py` is running with `--out=tcp:0.0.0.0:5760`
3. MP should show HUD, mode STABILIZE, GPS 3D fix, EKF green, home set
4. **MAVLink Forwarding** (for UI bridge, critical):
   - MP → Press Ctrl+F → Button **MAVLink forwarding** → Add `127.0.0.1:14551` → Tick **Write access** → OK
   - Bridge MAV lamp should go green, `mavlink-state` shows connected, mode
   - Safety: Write access needed for UI to upload fence via MP, but Pi companion still owns GUIDED — MP forwarding is read-only for UI except fence upload

---

## Use UI — Polygon → Fence → FOV Coverage → Fly (Safe)

In http://localhost:8100 on Windows:

### 1. Config Card (Max Height 15/10/5)

- Set **max height 15 m** (or 10/5 for testing), sweep alt 12 m, cam bottom 100° HFOV, overlap 0.35, edge 2.5m → Apply
- `cfgMaxAlt` shows 15 m, `fov` shows footprint 35.7×23m @15m
- Safety: max_alt_m 15/10/5 configurable, sweep and stair clamped to it via `validate_max_alt()` + `clamp_altitude()`, drone never exceeds max

### 2. Polygon → Fence Inclusion

- Click **draw polygon** (purple) → click 4-6 points around Gazebo QR area on map → purple polygon + area ~10000m²
- Click **POLYGON → FENCE INCLUSION** → converts polygon to fence inclusion, shows `→ fence ✓`, logs `polygon → fence inclusion: N verts (shows on MP as fence)`
- Click **apply polygon as fence → MP** → POSTs to `/api/mp/fence`, bridge uploads to FC via MAVLink, MP Fence tab should show polygon inclusion
- **Safety:** `_clamp_to_fence()` binary shrink 10 iters pulls all Pi goto targets inside fence, outside flag. ArduPilot FENCE_ENABLE=1 ACTION=1 RTL on breach — FC RTL itself if breach, Pi also clamps.

### 3. FOV Coverage (Max Area per Cam FOV)

- Click **Show FOV coverage** → draws blue footprint rects (35×23m @15m) + dashed lawnmower path inside polygon
- Info: `cam bottom (100° HFOV) @15m footprint 35.7×23.0m spacing 15.0m legs 14 area 10000m² est. 18 footprints`
- Change max height to 10m → footprint shrinks to 23.8×15.3m, more legs, no gaps
- **Safety:** `lawnmower_rows_fov_optimal()` uses camera footprint at alt, overlap 0.35, edge_margin 2.5m, divides area according to cam FOV to cover max possible, no gaps. If no fence, fallback home box nofence_half_m 20m.

### 4. Verify on MP (Windows)

- MP → Plan → Fence → should see inclusion polygon matching UI
- If not: GET `http://localhost:8100/api/mp/fence` to read back, ensure FC has GPS 3D fix, EKF origin set

---

## Fly Mission — Safe Trigger + No-Crash Fallbacks

### In Mission Planner (Windows)

1. **Plan mission with DO_SPRAYER:**
   - Plan → Load `mission_pi/sim/missions/qr_search.txt` or create: TAKEOFF 15m, DO_SPRAYER (cmd 216) at seq 2, WAYPOINTs around fence, RTL at end → Upload
   - Or: UI → Start mission → Pi goes WAIT_TRIGGER → SEARCH (manual takeover)

2. **Pre-arm checks (Safety):**
   - MP → HUD: GPS 3D fix, EKF green, battery, fence, mode STABILIZE
   - Arm check must pass

3. **Arm and Takeoff:**
   - MP: Arm, Takeoff to 15m (or mode AUTO, Arm, Takeoff)
   - Wait 15m

4. **Pi Takeover (Safe Gate):**
   - When MISSION_CURRENT reaches DO_SPRAYER seq, Pi logs `trigger: mission seq 2 >= sprayer 2 (gate=AUTO)` → commands GUIDED → holds at sweep_alt 12m
   - **Gate safety:** trigger only if `fc.mode==AUTO` fresh <5s OR DEADMAN (stale heartbeat + advancing mission). Prevents false trigger. If no trigger in 900s → FAILSAFE RTL (safe)

5. **Search (Pi GUIDED, FC Failsafes Active):**
   - Yaw sweep 12×30° settle 1s (sharp frames)
   - Grid lawnmower FOV-optimal 14 legs @15m, spacing 15m, speed 1.0 m/s (slow for more frames)
   - Each leg: `goto_global` 1Hz, checks:
     - Battery <25% (real) / 0% (SITL) → abort → RTL
     - FC leaves GUIDED (user RTL, failsafe) → `_guided_ok()` 3 stale reads → `mode_tripped()` → restore speed → DONE external RTL (no fight)
     - Search budget 900s spent → finish without payload → RTL
     - Fresh cue? → TRACK
   - **Safety:** Pi 1Hz goto, FC holds last, can RTL itself anytime. Pi targets clamped inside fence.

6. **Track & Approach (Descend Stair):**
   - Bottom bbox → `nadir_pixel_to_ground_m()` → lat/lon → `_clamp_to_fence()` → goto
   - If outside fence → hold, log, don't fly out
   - Off 1.5m and stair_idx < len: descend 12→9→7m
   - Decode every frame, consensus streak 3 (1 miss tolerated)
   - If cue lost 2.5s → resume grid (safe)
   - If approach timeout 240s → resume grid

7. **Transmit & RTL:**
   - Consensus → `relay_qr()`: STATUSTEXT `QR:<payload>` xN to FC → MP Messages tab + ws event `qr` to UI + store-forward jsonl if link down
   - Hold 15s → post_action RTL → `set_mode(RTL)` → speed param restored
   - **Safety:** store-forward survives reboot, Pi logs pending_results.jsonl

8. **Landing:**
   - RTL → home → LAND → disarm
   - MP tlog has `QR:<payload>`, Pi logs, UI event

---

## What If Something Fails? (No Crash — 2-Laptop)

| Failure | Detection | Pi Fallback | FC Fallback | Drone Result (Sim) |
|---------|-----------|-------------|-------------|-------------------|
| Pop!_OS crash (Gazebo+SITL+Pi) | MP disconnect | — | SITL stops, sim ends | Sim ends, no real crash |
| Windows crash (MP+UI) | MP disconnect, bridge down | Pi still has tcp:127.0.0.1:5762 to SITL local | SITL continues AUTO or RTL per FS_GCS_ENABLE | Continues search, QR buffered, RTL at end, MP can reconnect |
| WiFi loss (192.168.1.x) | MP TCP 5760 breaks | Pi local 5762 still to SITL, store-forward jsonl | SITL FS_GCS_ENABLE 0 bench → continue, 1 real → RTL | Bench: continues, real: RTL safe |
| Pi process crash | FC link dead_s 10s | — | FC timeout → RTL | RTL |
| USB/TCP disconnect | link_lost, link_down True | Watchdog retry 5s, store-forward | FC AUTO or RTL | Continues or RTL |
| FC leaves GUIDED (user RTL) | _guided_ok 3 stale | mode_tripped → restore speed → DONE external RTL | Respects RTL | RTL |
| Battery low | _battery_ok | abort → RTL | BATT_FS RTL | RTL before critical |
| GPS/EKF fail | ArduPilot | — | FS_EKF LAND | LAND |
| Geofence breach | _clamp_to_fence | Inside flag | FENCE_ACTION RTL | RTL, Pi never outside |
| No QR 900s | _search_expired | finish no payload → RTL | — | RTL |
| Detector no model | get_detector exception | Classical fallback, /health failed | — | Search may not find QR but no crash |
| Camera fail | rig.status failed | Drop cam, other continues | — | Search with remaining cam |
| YOLO fail 34px full 0.42 low | Bench | Tiled 3x3 0.81 mandatory | — | Use tiled |
| Bad lat/lon | _clamp_to_fence binary shrink | Inside | Fence RTL | Never outside |
| No DO_SPRAYER | sprayer_seqs empty | trigger_seq fallback or manual takeover | — | Waits 900s FAILSAFE RTL |
| RC loss (real) | ArduPilot | — | FS_THR RTL | RTL |
| GCS loss (real) | ArduPilot | Pi still USB | FS_GCS RTL | RTL |

**Principle:** Pi suggests, FC decides. FC can always override via mode, fence, battery, EKF failsafes. Pilot RC RTL switch overrides Pi GUIDED immediately.

---

## Configs for 2-Laptop Safe

**Pop!_OS `config.gazebo.yolo.yaml` (SITL+Gazebo+Pi):**
```yaml
server: { host: 0.0.0.0, port: 8000 }  # 0.0.0.0 for Windows to reach http://192.168.1.10:8000
flight: { max_alt_m: 15.0, min_alt_m: 2.0 }
mission:
  max_alt_m: 15.0
  sweep_alt_m: 12.0
  approach_stair_m: [12.0, 9.0, 7.0]
  grid_overlap: 0.35
  edge_margin_m: 2.5
  resweep_alt_m: 10.0
  search_speed_ms: 1.0
  trigger_timeout_s: 900
  search_timeout_s: 900
  approach_timeout_s: 240
  min_batt_pct: 0  # SITL bench, real 25
  arrive_m: 2.0
  post_action: RTL
  nofence_half_m: 20.0
  coverage_mode: fov_optimal
detector:
  kind: yolo
  model_path: models/qr_yolov8n.pt
  conf_thr: 0.25
  tile_bottom: true
  tile_grid: [3, 3]
fc:
  conn: tcp:127.0.0.1:5762  # local SITL, SITL --out 0.0.0.0:5762
cameras:
  front: { kind: url, device: http://127.0.0.1:8099/front.mjpg, size: [2560,1440], hfov_deg: 68.0, qr_boost: { enabled: true, profile: front_qr_boost, software_mode: qr_boost } }
  bottom: { kind: url, device: http://127.0.0.1:8099/bottom.mjpg, size: [2560,1440], hfov_deg: 68.0, qr_boost: { enabled: true, profile: bottom_qr_boost, software_mode: ground_suppress } }
qr_boost: { enabled: true, profile: auto, software_mode: qr_boost }
```

**Windows MP:** Connect TCP 192.168.1.10:5760, Baud 57600, Forward 127.0.0.1:14551 Write access tick

**Windows UI `mp_bridge.py`:**
```powershell
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles
# Pi link in UI: http://192.168.1.10:8000 (Pop!_OS Pi)
```

---

## Verification Checklist (2-Laptop, No Crash)

- [ ] Pop!_OS IP 192.168.1.10, Windows IP 192.168.1.20, ping both ways, firewall allows 5760,8000,8099
- [ ] Pop!_OS Gazebo world loads iris_dualcam + qr_target A3
- [ ] Bridge http://127.0.0.1:8099/ front+bottom mjpg boosted, /health qr_boost ON, from Windows http://192.168.1.10:8099/ also works
- [ ] SITL EKF ready GPS 3D home set, --out 0.0.0.0:5760 for Windows, 5762 for Pi
- [ ] mission_pi http://127.0.0.1:8000/health ok detector yolo cams ok qr_boost ON max_alt 15, from Windows http://192.168.1.10:8000/health also ok
- [ ] Windows MP connects TCP 192.168.1.10:5760, HUD mode STABILIZE, GPS 3D, EKF green, fence tab empty or loaded
- [ ] Windows UI http://localhost:8100, bridge MAV green, Config max_alt 15 sweep 12 bottom 100° HFOV overlap 0.35 Apply, fov footprint 35.7×23m @15m
- [ ] Polygon draw purple → POLYGON→FENCE INCLUSION → apply to MP → MP Fence tab shows inclusion polygon
- [ ] FOV coverage Show → blue footprints + dashed path inside polygon, no gaps, spacing changes with max_alt 15/10/5
- [ ] Mission upload with DO_SPRAYER seq2 + RTL, MP Plan shows sprayer
- [ ] Arm Takeoff 15m AUTO, Pi trigger gate AUTO, GUIDED takeover hold 12m
- [ ] Search yaw+grid inside fence no breach, YOLO tiled 0.81 @15m PASS (classical FAIL), track clamped, stair 12→9→7, decode streak 3, transmit QR:<payload> to MP Messages + UI event
- [ ] RTL LAND at home, speed restored, logs pending_results.jsonl + /health + MP tlog QR:
- [ ] Fail tests: kill Pop!_OS Pi → SITL RTL itself; kill Windows MP → SITL+Pi continue; kill WiFi → MP disconnect but SITL+Pi continue local; battery low → RTL; mode RTL externally → Pi mode_tripped DONE; fence breach → FC RTL

---

## Logs

- Pop!_OS: `logs/pending_results.jsonl`, `http://127.0.0.1:8000/health`, `http://127.0.0.1:8000/api/bringup`, Gazebo console, SITL MAVProxy, mission_pi console
- Windows: MP `Documents/Mission Planner/logs/*.tlog` search QR:, bridge console, UI http://localhost:8100 Logs card + /api/mp/state latched QR

This 2-laptop setup can't crash real drone — it's SITL sim. For real drone, see Doc 11. Pi companion never crashes drone — FC owns failsafes, all Pi targets clamped, timeouts RTL, link supervision + watchdog + store-forward + server first.
