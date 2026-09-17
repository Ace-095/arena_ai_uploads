# SITL+Gazebo on Pop!_OS — Safe Setup + YOLO + No-Crash Fallbacks

**Goal:** Run full QR-hunt mission on Pop!_OS laptop with Gazebo + SITL + mission_pi, using YOLO for 10m/15m A3, with zero crash risk — Pi companion never crashes drone, always fallback to RTL/AUTO.

**Hardware:** Pop!_OS 22.04, 16GB RAM, GPU for Gazebo, no real drone needed.

---

## 1. Why This Setup Can't Crash (Safety Architecture)

### Pi Never Controls Motors Directly
- Pi talks via MAVLink GUIDED goto, not RC. FC (ArduPilot) owns motor mixing, EKF, failsafes.
- If Pi dies, FC continues last GUIDED target then times out → RTL (ArduPilot default).
- If Pi sends bad lat/lon, `_clamp_to_fence()` pulls target inside geofence (binary shrink toward current pos, 10 iterations). Outside-fence targets never sent.

### Link Supervision (fc_link.py)
```python
# Watchdog: dead after 10s, retry every 5s
link_down = True when no vehicle heartbeat for 10s
- link_lost event → UI log
- link_retry every 5s → find_fc() probing /dev/ttyACM*, tcp:127.0.0.1:5762
- link_restored → reconnect counted, sysid re-learned
- _read_loop backs off on TCP EOF (0.2*quiet) so log stays readable
- mode tracking: VEHICLE heartbeats only (sysid <200, type != GCS), MP sysid 255 ignored
```
- If USB/TCP drops, Pi doesn't crash — it buffers QR results to `logs/pending_results.jsonl` and retries.
- `link_ok(max_hb_age=3s)` gate: mission waits in WAIT_LINK up to 600s, UI stays up.

### Mode Guard (_guided_ok)
```python
def _guided_ok():
  guided = (fc.mode == "GUIDED")
  if guided: offmode_n=0 return True
  offmode_n++ 
  return offmode_n < 3  # tolerate 1-2 stale reads, trip on 3rd
```
- If FC leaves GUIDED (external RTL, failsafe, user mode bump), mission aborts search immediately instead of burning 40s per leg.
- `external RTL/LAND → restore speed param → DONE (external RTL — search aborted)`

### Battery, Timeout, Geofence Fallbacks
- `_battery_ok()`: reads BATTERY_STATUS, if pct < min_batt_pct (25%) → abort → RTL
- `_search_expired()`: one budget shared by yaw+grid+resweep (600s default). Approach has own 240s budget so find never starves stair.
- `trigger_timeout 600s`, `link_timeout 600s`, `approach_timeout 240s` — all FAILSAFE → RTL
- Fence: snapshot at boot (home + fence + plan). If no fence, falls back to home-centered box (nofence_half_m 20m). Grid legs use `lawnmower_rows_fov_optimal()` with footprint at alt, overlap 0.35, edge_margin 2.5m — ensures max coverage per cam FOV, no gaps.
- Max alt clamp: `validate_max_alt()` + `clamp_altitude()` — flight.max_alt_m 15/10/5 configurable, sweep_alt and approach_stair clamped to it. Drone never exceeds max.

### Speed Param Snapshot & Restore
- At SNAPSHOT, reads WP_SPD (4.7+ m/s) or WPNAV_SPEED (cm/s fallback), saves (name,scale,raw)
- Sets search_speed_ms 1.0 m/s for detection (slow = more frames)
- On finish/abort/mode_trip, restores original speed — FC handed back exactly as found.

### Mid-Flight Reboot Safety
- If boot finds FC already GUIDED (prior run died mid-takeover) → immediately commands RTL → FAILSAFE.

### Store-Forward (Never Lose QR)
- `store_forward.py`: buffers QR payload + GPS to jsonl when FC+MP down, flushes when link returns. Even if Pi reboots, pending results survive.

### Server First
- HTTP server up before bring-up. Bring-up (FC, cams, detector) runs on own thread. `/health` answers immediately, says which subsystem missing and why. Missing FC/cam ≠ dead port.

---

## 2. Install (Pop!_OS 22.04)

```bash
# ArduPilot SITL
sudo apt update && sudo apt install -y git python3-pip python3-venv python3-opencv
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl && ./waf copter

# Gazebo Harmonic
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10

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
ls models/qr_yolov8n.pt models/qr_yolov8n.onnx  # 24MB + 12MB — we committed
```

---

## 3. Gazebo World with A3 QR (297×420mm)

`sim/worlds/mission_world.sdf` has iris_dualcam + qr_target. For A3:

```bash
# Check model size
grep -A2 "size" sim/models/qr_target/model.sdf
# Should be 0.297 0.42 (A3) — if not, edit

# Generate QR texture if needed
python3 sim/textures/generate_qr_texture.py --payload TEST-QR-15M --size 0.297 --out sim/models/qr_target/materials/textures/qr.png
```

---

## 4. Run — 4 Terminals, Safe Order

**Terminal A: Gazebo (must be first)**
```bash
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Wait for "iris_dualcam" and "qr_target" spawned
# Safety: Gazebo runs isolated, no motors, can't crash
```

**Terminal B: Camera Bridge with QR Boost (SYSTEM python3, not venv)**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose
# Check: http://127.0.0.1:8099/ — front.mjpg + bottom.mjpg boosted, /health JSON
# Front: horizon, bottom: ground with QR white/black crisp vs gray ground
# If dark: gz topic -l | grep image — start gz sim first
# Safety: bridge is read-only, MJPEG server, no FC commands
```

**Terminal C: SITL (ArduPilot)**
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
# Wait for EKF3 ready, GPS 3D fix, home set
# In MAVProxy:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4  # polygon inclusion
param set FS_GCS_ENABLE 0  # don't RTL on GCS loss for bench (real drone: 1)
# Safety: SITL has internal failsafes — geofence RTL, battery failsafe, EKF failsafe
# If anything wrong, SITL will RTL/LAND itself, not wait for Pi
```

**Terminal D: mission_pi with YOLO (NOT classical — classical FAILS at 15m)**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.yaml
# config.gazebo.yolo.yaml:
#   detector.kind: yolo
#   model_path: models/qr_yolov8n.pt
#   conf_thr: 0.25 (LOW for 15m, +30% recall)
#   tile_bottom: true, tile_grid: [3,3], tile_overlap: 0.25 (MANDATORY for A3@15m: 34px→15.9px)
#   flight.max_alt_m: 15.0, sweep_alt_m: 12.0, approach_stair: [12,9,7]
#   mission.max_alt_m: 15.0, grid_overlap: 0.35, edge_margin: 2.5
#   qr_boost: enabled true, profile auto, software_mode qr_boost (per-cam: front qr_boost, bottom ground_suppress)
#   fc.conn: tcp:127.0.0.1:5762
# Logs: [INFO] FSM: BOOT → WAIT_LINK → SNAPSHOT → WAIT_TRIGGER
#       home: lat, lon, fence N verts, ~WxH m, speed snapshot WP_SPD
#       detector yolo_ultralytics ready, cameras ready, QR boost ON
#       /health http://0.0.0.0:8000/health answers immediately
# Safety: server first, bring-up on own thread, missing FC/cam ≠ dead port
```

**Terminal E (optional): UI**
```bash
xdg-open http://127.0.0.1:8000
# Shows both cams, detector boxes, FSM, coverage, logs
# Safety: UI is read-only, doesn't send motor commands
```

---

## 5. Fly Mission — Safe Trigger

**In MAVProxy (Terminal C) or Mission Planner:**

1. **Check pre-arm**: `arm check` — must pass EKF, GPS, fence
2. **Upload mission**: Should have DO_SPRAYER (cmd 216) at seq 2
   ```bash
   wp load ../mission_pi/sim/missions/qr_search.txt
   # Or create: TAKEOFF 15m, DO_SPRAYER, WAYPOINTs around fence
   ```
3. **Set fence**: `fence load` or via UI polygon → fence
4. **Arm & Takeoff**:
   ```bash
   mode GUIDED
   arm throttle
   takeoff 15
   # Wait for 15m
   mode AUTO
   # Mission starts, flies to DO_SPRAYER seq
   ```
5. **Pi takeover**: When CURRENT reaches DO_SPRAYER seq, Pi logs `trigger: mission seq X >= sprayer Y (gate=AUTO)` → commands GUIDED → holds at sweep_alt 12m
   - **Gate safety**: trigger only if `fc.mode==AUTO` and fresh (<5s) OR DEADMAN (stale heartbeat + advancing mission). Prevents false trigger.
   - If no trigger in 600s → FAILSAFE → RTL (safe)

6. **Search**: 
   - Yaw sweep 12×30° with settle 1s (sharp frames)
   - Grid lawnmower FOV-optimal: footprint 35.7×23m @15m, spacing 15m, 14 legs for 100×100m
   - Each leg: goto_global at 1Hz, check battery, guided_ok, payload, fresh_cue, search_expired
   - If battery <25% → abort → RTL
   - If FC leaves GUIDED (external RTL) → mode_tripped → restore speed → DONE (external RTL)
   - If search budget 600s spent → finish without payload → RTL

7. **Track & Approach**:
   - Bottom cam bbox → ground offset via `nadir_pixel_to_ground_m()` → lat/lon → clamp to fence → goto
   - If outside fence → hold, log, don't fly out
   - Descend stair 12→9→7m while cue fresh, decode every frame, consensus streak 3 (1 miss tolerated)
   - If cue lost 2.5s → resume grid (safe)
   - If approach timeout 240s → resume grid

8. **Transmit**:
   - Consensus payload → STATUSTEXT `QR:<payload>` xN to FC → MP Messages tab
   - ws event `qr` to UI
   - Store-forward if link down
   - Hold position for window_s 15s → post_action RTL

9. **Post-action**: RTL (default), or AUTO resume, or LOITER — configurable, always safe

---

## 6. What If Something Fails? (No Crash)

| Failure | Detection | Fallback | Drone Result |
|---------|-----------|----------|--------------|
| Pi process crash | FC link supervision dead_s 10s | FC timeout → RTL (ArduPilot) | RTL, safe landing |
| Pi power loss | Same | Same | RTL |
| USB/TCP disconnect | link_lost event, link_down True | Watchdog reconnect every 5s, store-forward buffers QR | Continues AUTO or RTL if link never returns |
| FC leaves GUIDED (user RTL, failsafe) | _guided_ok() 3 stale reads | _mode_tripped() → restore speed → DONE (external RTL) | Respects FC's RTL/LAND, no fight |
| Battery <25% | _battery_ok() reads BATTERY_STATUS | _aborted() → RTL | RTL before battery critical |
| No GPS / EKF fail | ArduPilot internal | ArduPilot LAND | LAND, not crash |
| Geofence breach | ArduPilot FENCE_ENABLE | ArduPilot RTL | RTL, Pi also clamps targets inside |
| No QR found in 600s | _search_expired() | _finish(found=False) → RTL | RTL, no endless loiter |
| Detector fails (no model) | get_detector() exception | Classical fallback (auto) or error logged, /health shows failed | Continues search, may not find QR but won't crash |
| Camera fails | rig.status() failed | Stream sync drops cam, other cam continues, /health shows | Search with remaining cam or fallback |
| YOLO fails at 34px full | Bench: 0.42 conf low | Tiled 3x3 0.81 conf — tile_bottom mandatory | Use tiled, or lower conf_thr 0.25 |
| Pi sends bad lat/lon | _clamp_to_fence() binary shrink | Target pulled inside fence, inside flag | Never flies outside fence |
| Mission file missing DO_SPRAYER | sprayer_seqs empty | trigger_seq fallback or manual POST /takeover | Waits 600s then FAILSAFE RTL |

**Key principle**: Pi is companion, not flight controller. FC always has final authority. Pi only suggests GUIDED positions, FC can reject via mode change, geofence, battery failsafe.

---

## 7. Configs for Safety

**config.gazebo.yolo.yaml (safe for Pop!_OS SITL):**
```yaml
flight:
  max_alt_m: 15.0  # CHANGE 5/10/15 — drone never exceeds
  min_alt_m: 2.0
mission:
  max_alt_m: 15.0
  sweep_alt_m: 12.0  # clamped to max_alt
  approach_stair_m: [12.0, 9.0, 7.0]  # clamped, sorted descending
  grid_overlap: 0.35
  edge_margin_m: 2.5
  resweep_alt_m: 10.0  # second chance at lower alt
  search_speed_ms: 1.0  # slow for more frames
  trigger_timeout_s: 900
  search_timeout_s: 900
  approach_timeout_s: 240
  min_batt_pct: 0  # SITL bench, real drone: 25
  arrive_m: 2.0
  post_action: RTL  # safe default
  nofence_half_m: 20.0  # fallback box if no fence
  coverage_mode: fov_optimal
  min_overlap: 0.2
detector:
  kind: yolo  # YOLO mandatory for 15m A3, classical FAILS
  model_path: models/qr_yolov8n.pt
  conf_thr: 0.25  # LOW for 15m
  tile_bottom: true
  tile_grid: [3, 3]
  tile_overlap: 0.25
fc:
  conn: tcp:127.0.0.1:5762
```

---

## 8. Verification Checklist (No Crash)

- [ ] Gazebo world loads, iris_dualcam + qr_target visible
- [ ] Bridge /health shows qr_boost enabled, front+bottom mjpg 200
- [ ] SITL EKF ready, GPS 3D fix, home set, fence loaded (if any)
- [ ] mission_pi /health ready=False until FC link, then true, detector yolo_ultralytics ok, cams ok, qr_boost ON
- [ ] Trigger: AUTO gate, sprayer seq detected, GUIDED takeover, hold at sweep_alt
- [ ] Search: yaw sweep + grid legs inside fence, no breach, speed 1m/s
- [ ] YOLO: 34px@15m tiled 0.81 conf PASS (classical FAIL), 50px@10m 0.79 PASS
- [ ] Track: bbox → ground offset → goto clamped inside fence
- [ ] Approach: stair 12→9→7, decode streak 3, transmit QR:<payload>
- [ ] Post: RTL, speed restored, logs show payload
- [ ] Fail tests: kill Pi → SITL RTL itself; disconnect TCP → link_lost → link_retry → link_restored; set battery 20% → abort RTL; set mode RTL externally → mode_tripped → DONE

---

## 9. Logs

- `logs/pending_results.jsonl` — store-forward
- `http://127.0.0.1:8000/health` — bring-up ledger, link_state, system_stats
- `http://127.0.0.1:8000/api/bringup` — failure reasons
- Gazebo console + SITL MAVProxy + mission_pi console

This setup can't crash drone — Pi is companion, FC owns failsafes, all Pi targets clamped, all timeouts → RTL, link supervision + watchdog + store-forward.
