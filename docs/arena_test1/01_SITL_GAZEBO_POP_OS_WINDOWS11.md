# Method 1: Two-Laptop SITL + Gazebo (Pop!_OS) + Mission Planner + UI (Windows 11) — arena/test1

**Goal:** Prove polygon → fence inclusion → FOV-optimal coverage → QR detection at 10/15 m for A3 without any hardware. This is the **first** method you must pass.

**Laptops:**
- **Pop!_OS Laptop (Linux):** ArduPilot SITL + Gazebo Harmonic + mission_pi (Pi companion) + fake SITL
- **Windows 11 Laptop:** Mission Planner + mp_bridge.py (UI host + MAVLink bridge) + browser UI

Both on same WiFi/router, no internet needed after setup (offline tiles).

---

## Hardware / OS

- Pop!_OS 22.04 (Ubuntu 22.04 base) — 16 GB RAM, GPU for Gazebo
- Windows 11 — Mission Planner 1.3.80+, Python 3.10+
- Router: same subnet, e.g. 192.168.1.x

## Network diagram

```
Pop!_OS (192.168.1.10)                          Windows 11 (192.168.1.20)
┌──────────────────────────────────┐           ┌─────────────────────────────────┐
│ Gazebo Harmonic                  │           │ Mission Planner                 │
│  └─ iris_dualcam model           │           │  TCP 5760 ←→ SITL               │
│ ArduPilot SITL                   │           │  MAVLink Forwarding             │
│  tcp 5760 (MP)  tcp 5762 (Pi)    │           │   udp 127.0.0.1:14551 ──┐       │
│  udp 14551 → Windows             │──────────►│                            │   │
│ mission_pi :8000                 │           │ mp_bridge.py :8100          │◄──┘
│  REST + WS + MJPEG               │──────────►│  bridge → browser UI         │
│  config.yaml max_alt_m 15        │  HTTP     │  polygon + FOV viz           │
└──────────────────────────────────┘           └─────────────────────────────────┘
```

---

## Step 1: Pop!_OS — Install ArduPilot SITL + Gazebo Harmonic

```bash
# ArduPilot
sudo apt update && sudo apt install -y git python3-pip python3-venv
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter

# Gazebo Harmonic (Ubuntu 22.04)
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install -y gz-harmonic

# ArduPilot Gazebo plugin
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
. ~/.bashrc
```

## Step 2: Pop!_OS — Clone arena_ai_uploads and setup mission_pi

```bash
cd ~
git clone https://github.com/Ace-095/arena_ai_uploads.git
cd arena_ai_uploads
git checkout arena/test1
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt  # includes websockets, opencv, pyzbar

# Check config.yaml — max height 15 m
cat config.yaml | grep -A2 flight:
# flight:
#   max_alt_m: 15.0   <-- change to 10 or 5 for testing

# Generate demo frames for A3 QR at 15 m (for laptop mode)
python3 tools/make_demo_frames.py --qr-size 0.297 --alt 15

# Check detector config — must be tiled for 15 m
cat config.yaml | grep -A10 detector:
# tile_bottom: true, tile_grid: [3,3], conf_thr: 0.25
```

## Step 3: Pop!_OS — Gazebo world with A3 QR

`mission_pi/sim/worlds/qr_search_field.sdf` already has QR box. For A3:

- Edit `sim/models/qr_box/model.sdf`: size 0.297×0.42 m (A3)
- Or generate texture: `python3 sim/textures/generate_qr_texture.py --payload TEST42 --size 0.297`

```bash
# Launch Gazebo with A3 QR field
gz sim -v 4 -r sim/worlds/qr_search_field.sdf
# Should see iris_dualcam + QR box 0.297×0.42 m on ground
```

## Step 4: Pop!_OS — Start SITL + mission_pi

Terminal 1 — SITL:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
# Wait for EKF3 ready, then in MAVProxy:
# param set SERIAL0_PROTOCOL 2
# param set SERIAL1_PROTOCOL 2
# param set SIM_GZ_EN 1
```

Terminal 2 — mission_pi (the Pi companion):

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python3 main.py --config config.gazebo.yaml
# config.gazebo.yaml uses Gazebo cameras via gz_cam_bridge.py
# Should see: [INFO] FSM: IDLE -> PLAN_SYNC, cameras ready, detector auto/hailo/classical
```

Or for laptop mode without Gazebo (fake_sitl):

```bash
python3 tools/fake_sitl.py --udp 192.168.1.20:14551 --auto --loop
python3 main.py --config config.laptop.yaml
```

## Step 5: Windows 11 — Mission Planner + Bridge + UI

### Install

```powershell
# Mission Planner: download from ardupilot.org, install

# Python + bridge
git clone https://github.com/Ace-095/arena_ai_uploads.git C:\arena
cd C:\arena
git checkout arena/test1
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
python -m venv .venv
.venv\Scripts\activate
# No pip needed — mp_bridge.py is stdlib only
```

### Offline tiles (do once with internet)

```powershell
python tools\fetch_tiles.py --bbox 75.10,15.35,75.14,15.38 --zoom-min 15 --zoom-max 19 --out field.mbtiles
# field.mbtiles ~50-100 MB for 4×4 km
```

### Start bridge + UI

```powershell
cd C:\arena\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
.venv\Scripts\activate
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles --watch "C:\Users\YOU\Documents\Mission Planner\logs"
# Output:
# Mission Companion bridge http://localhost:8100
# MAVLink GCS udp 0.0.0.0:14551
# offline tiles: {...}
```

Open browser: http://localhost:8100

### Mission Planner forwarding (critical)

1. MP → Connect to SITL: TCP, 192.168.1.10:5760 (Pop!_OS IP), Baud 57600
2. Right-click vehicle on map → MAVLink Forwarding → Add `127.0.0.1:14551` → Tick **Write access** → OK
3. Bridge MAV lamp should go green, `mavlink-state` shows connected, mode.

## Step 6: Polygon → Fence → FOV Coverage (arena/test1)

In UI http://localhost:8100:

1. **Config card:** Set max height 15 m (or 10 m), sweep alt 15 m, cam bottom (100° HFOV), overlap 0.35. Click Apply. `cfgMaxAlt` shows 15 m.

2. **Polygon card:**
   - Click `draw polygon` (purple) → click 4-6 points on map around Gazebo QR area → should see purple polygon + area ~10000 m².
   - Click `POLYGON → FENCE INCLUSION` — converts polygon to fence inclusion, shows `→ fence ✓`, logs `polygon → fence inclusion: N verts (shows on MP as fence)`.
   - Click `apply polygon as fence → MP` — POSTs to `/api/mp/fence`, bridge uploads to FC via MAVLink, MP Fence tab should show polygon.

3. **Coverage card:**
   - `Show FOV coverage` → draws blue footprint rects (35×23 m at 15 m) + dashed lawnmower path inside polygon.
   - Info: `cam bottom (100° HFOV) @ 15m footprint 35.7×23.0m spacing 15.0m legs 14 area 10000m² est. 18 footprints`
   - Change max height to 10 m → footprint shrinks to 23.8×15.3 m, more legs — verify no gaps.

4. **Verify on MP:** Fence tab → should see inclusion polygon matching UI. If not, `GET /api/mp/fence` to read back.

## Step 7: Fly mission and detect A3 QR at 10/15 m

In Mission Planner:

1. Plan tab → Draw polygon mission or use `mission_pi` auto trigger:
   - MP: `DO_SPRAYER` at seq 2 triggers Pi takeover (per `mission.trigger_cmds`).
   - Or manual: UI → `Start mission` → Pi goes `WAITING_TRIGGER` → `SEARCH`.

2. Arm and takeoff in GUIDED or AUTO:
   - MP: Arm, Takeoff to 15 m.

3. Watch UI:
   - Drone marker moves, coverage heatmap (green cells) marks searched area.
   - FOV coverage path should be followed.
   - When bottom cam sees A3 QR (34 px at 15 m, 50 px at 10 m), detector bbox appears, FSM `TARGET_FOUND` → `DESCEND` stair 12→9→7 m → `DECODED` → payload.
   - QR result appears via 4 routes: Pi WS, MAVLink STATUSTEXT `QR:<payload>`, MP log tail, manual.

4. If detection fails at 15 m:
   - Lower `conf_thr` to 0.25 in `config.yaml`, restart mission_pi.
   - Check `tile_bottom: true` and `tile_grid: [3,3]`.
   - Slow search speed to 2.0 m/s.
   - Trigger resweep at 10 m: `mission.resweep_alt_m: 10.0`.

## Step 8: Logs and verification

- Pop!_OS: `mission_pi/logs/` — `pending_results.jsonl`, `telemetry.log`
- Windows: `bridge` console + UI Logs card + `C:\Users\...\Documents\Mission Planner\logs\*.tlog` — search for `QR:`
- UI: `/api/mp/state` shows latched QR, `/api/mp/mavlink` shows mode.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Bridge MAV red | MP forwarding not set or Write access off | MP → MAVLink Forwarding → 127.0.0.1:14551 + Write |
| Polygon not on MP | Fence upload failed | Check `/api/mp/fence` GET, ensure FC has GPS 3D fix, EKF origin set |
| FOV coverage not shown | Poly <3 verts | Draw polygon first |
| No detection at 15 m | conf_thr 0.35 too high, no tiling | Set conf_thr 0.25, tile_bottom true, 3×3 |
| Gazebo QR not visible | Model size wrong | Check model.sdf size 0.297×0.42, texture |
| SITL no GPS | Gazebo not running or plugin path | Export GZ_SIM_SYSTEM_PLUGIN_PATH, restart SITL |

## Success criteria for Method 1

- [ ] Polygon drawn in UI, converted to fence, visible in MP Fence tab
- [ ] FOV coverage shows footprint rects at 15 m and 10 m, spacing changes
- [ ] SITL drone follows lawnmower inside polygon (no breach RTL)
- [ ] A3 QR at 15 m detected (bbox), descended, decoded, payload `QR:<payload>` in MP console + UI
- [ ] Resweep at 10 m works if first grid fails
- [ ] Logs show `fov: {cam: bottom, hfov: 100, footprint_w: 35.7, ...}` in Pi status
