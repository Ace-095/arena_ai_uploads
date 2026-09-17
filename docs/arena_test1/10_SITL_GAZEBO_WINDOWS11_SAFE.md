# SITL+Gazebo on Windows 11 (Different Laptop) — Safe Setup + No-Crash

**Goal:** Run SITL+Gazebo on Windows 11 laptop (second laptop), connect Mission Planner + mission_pi (or WSL2), with same safety fallbacks — Pi companion never crashes drone.

**Note:** Gazebo Harmonic native on Windows is experimental. Recommended: WSL2 Ubuntu 22.04 on Windows 11 for SITL+Gazebo, Mission Planner native Windows. Or two-laptop setup: Pop!_OS runs Gazebo+SITL, Windows 11 runs MP+UI.

---

## Option A: WSL2 on Windows 11 (Single Laptop, Recommended)

### Why WSL2?
- Gazebo Harmonic + ArduPilot SITL are Linux-native, run perfectly in WSL2
- Mission Planner runs native Windows, connects via TCP to WSL2 SITL
- Safety same as Pop!_OS — FC owns failsafes, Pi is companion

### Install WSL2

```powershell
# PowerShell Admin
wsl --install -d Ubuntu-22.04
wsl --set-default Ubuntu-22.04
# Reboot, create Ubuntu user

# In WSL2 Ubuntu
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3-pip python3-venv python3-opencv
```

### Install ArduPilot + Gazebo in WSL2 (same as Pop!_OS)

```bash
# Inside WSL2
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
pip3 install -r requirements.txt --break-system-packages
pip3 install ultralytics onnxruntime qrcode pillow --break-system-packages
```

### WSL2 Networking (Critical for Safety)

```bash
# In WSL2, check IP
ip addr | grep eth0
# e.g. 172.20.123.45

# Windows host can reach WSL2 via localhost or that IP
# SITL binds 0.0.0.0:5760, so Windows MP can connect to:
#   tcp:127.0.0.1:5760  or  tcp:172.20.123.45:5760
# For safety, use localhost — less firewall issues
```

### Run in WSL2 (4 terminals)

**WSL Terminal 1: Gazebo**
```bash
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Safety: Gazebo isolated, no real motors
```

**WSL Terminal 2: Bridge with QR boost**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress --verbose
# http://127.0.0.1:8099/ — check from Windows browser: http://localhost:8099/ should work via WSL2 port forwarding
```

**WSL Terminal 3: SITL**
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# --out 0.0.0.0:5760 for MP (Windows), 5762 for Pi
# Safety: SITL internal failsafes — geofence RTL, battery, EKF
```

**WSL Terminal 4: mission_pi with YOLO**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.yaml
# Logs: FSM, home, fence, detector yolo ready, QR boost ON
# /health http://0.0.0.0:8000 — from Windows: http://localhost:8000 or http://172.20.123.45:8000
# Safety: server first, bring-up thread, link supervision
```

### Windows Side: Mission Planner

1. **Install MP**: https://ardupilot.org/planner/docs/mission-planner-installation.html
2. **Connect to WSL2 SITL**:
   - MP → Select TCP, 127.0.0.1:5760, Baud 57600, Connect
   - If fails, try WSL2 IP: `ip addr` in WSL2 → 172.20.x.x:5760
   - Or: `netsh interface portproxy add v4tov4 listenport=5760 listenaddress=0.0.0.0 connectport=5760 connectaddress=172.20.123.45` (Admin PowerShell) to forward
3. **MAVLink Forwarding** (for UI bridge, optional):
   - MP → Ctrl+F → MAVLink forwarding → Add 127.0.0.1:14551, Write access tick
4. **Fence**: Plan → Fence → Load or draw inclusion polygon
5. **Mission**: Plan → Load qr_search.txt with DO_SPRAYER at seq 2
6. **Fly**: Arm, Takeoff 15m, AUTO — Pi takes over on DO_SPRAYER

**Safety in MP:**
- MP shows mode, battery, GPS, EKF, fence. If any red, don't arm.
- MP has its own failsafe: if GCS loss, FS_GCS_ENABLE param controls RTL. For bench set 0, real drone set 1.
- MP can always switch mode to RTL/STABILIZE — overrides Pi GUIDED immediately (Pi's _guided_ok detects 3 off-mode reads → abort)

---

## Option B: Two Laptops — Pop!_OS (Gazebo+SITL) + Windows 11 (MP+UI)

### Network Diagram (Safe)

```
Pop!_OS 192.168.1.10 (Gazebo+SITL+Pi)          Windows 11 192.168.1.20 (MP+UI)
┌─────────────────────────────────┐            ┌──────────────────────────────┐
│ Gazebo :mission_world.sdf       │            │ Mission Planner              │
│ Bridge :8099 MJPEG boosted      │            │  TCP 192.168.1.10:5760 SITL  │
│ SITL tcp 5760 MP, 5762 Pi       │◄──────────►│  Forward 127.0.0.1:14551     │
│ mission_pi :8000 YOLO           │   WiFi     │ mp_bridge.py :8100          │
│  http://192.168.1.10:8000       │───────────►│  http://localhost:8100 UI    │
│ Safety: FC owns failsafes       │   HTTP     │  polygon→fence→FOV→fly       │
└─────────────────────────────────┘            └──────────────────────────────┘
```

### Pop!_OS Side (Same as Doc 09)

- Run Gazebo, bridge, SITL, mission_pi as in Doc 09
- SITL: `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762`
- mission_pi: `config.gazebo.yolo.yaml` with `fc.conn: tcp:127.0.0.1:5762` (local) and server 0.0.0.0:8000
- Check: `ip addr` → 192.168.1.10, `curl http://192.168.1.10:8000/health` from Windows should work

### Windows 11 Side

```powershell
# Mission Planner
# Connect TCP 192.168.1.10:5760

# mission-ui bridge (optional, for polygon→fence UI)
git clone https://github.com/Ace-095/arena_ai_uploads.git C:\arena
cd C:\arena
git checkout arena/01a0a7de-arena-ai-uploads
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
python -m venv .venv
.venv\Scripts\activate
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles

# Open http://localhost:8100
# Config: max_alt 15, sweep 12, bottom cam 100° HFOV, overlap 0.35
# Polygon: draw purple → POLYGON→FENCE INCLUSION → apply to MP
# Coverage: Show FOV → blue footprints 35.7×23m @15m
# Fly: MP Arm Takeoff 15m AUTO, Pi takes over on DO_SPRAYER
```

### Safety: Two-Laptop Link

- **WiFi loss**: If WiFi drops, SITL→MP TCP breaks, MP shows disconnect, but SITL continues AUTO or RTL per FS_GCS_ENABLE. Pi on Pop!_OS still has local tcp:127.0.0.1:5762 to SITL, so search continues, QR buffered to jsonl, flushed when WiFi returns.
- **Pop!_OS crash**: SITL dies, MP disconnects, drone (sim) stops. No real crash.
- **Windows crash**: MP dies, SITL still runs, Pi still searches, RTL at end. MP can reconnect.
- **Pi crash**: FC supervision dead_s 10s → FC RTL itself. No crash.

---

## Common Safety Fallbacks (Both Options)

### 1. Geofence
- Fence snapshot at boot, `point_in_polygon()` check, `_clamp_to_fence()` pulls targets inside.
- ArduPilot FENCE_ENABLE=1, FENCE_TYPE=4 (inclusion), breach → RTL automatically.
- If no fence, fallback home box nofence_half_m 20m.

### 2. Battery
- `get_battery()` → pct, if < min_batt_pct (25% real, 0% SITL) → abort → RTL.
- SITL also has BATT_FS_LOW_ACT param.

### 3. Link
- Watchdog dead_s 10s, retry_s 5s, link_lost → link_retry → link_restored, reconnects counted.
- `link_ok(max_hb_age=3s)` gate, WAIT_LINK 600s, UI stays up.
- Store-forward jsonl survives reboot.

### 4. Mode
- _guided_ok() 3-stale-read tolerance, trip on 3rd → mode_tripped → restore speed → DONE external RTL.
- User can always switch MP to RTL/STABILIZE — Pi respects immediately.

### 5. Timeouts
- trigger 900s, search 900s, approach 240s, link 600s → all FAILSAFE RTL.
- Search budget shared, approach own budget.

### 6. Altitude
- max_alt_m 15/10/5 configurable, sweep and stair clamped, validate_max_alt().
- Drone never exceeds max, even if config says higher.

### 7. Detector
- YOLO kind: yolo auto tries pt→onnx→ultralytics→onnxruntime, fallback to classical if no model.
- Classical FAIL at 34/50px, YOLO tiled PASS 0.81/0.79 — use YOLO for 15m.
- If detector fails, /health shows failed, mission continues search (may not find QR but won't crash).

### 8. Camera
- Stream sync adopts/drops cams, vanished cam dropped, thread stopped no leak, surviving cam untouched.
- If no cam, FAILSAFE no camera for grid sweep → RTL.

---

## Windows 11 Specific Troubleshooting (Safe)

| Symptom | Cause | Safe Fix |
|---------|-------|----------|
| MP can't connect 5760 | WSL2 IP changed, firewall | Use 127.0.0.1:5760, or `wsl --shutdown` then restart, or portproxy, check Windows Firewall allow MP |
| Gazebo black screen | WSL2 no GPU | Install WSLg, or use Pop!_OS laptop for Gazebo, Windows only MP |
| Bridge :8099 not reachable from Windows | WSL2 port not forwarded | `http://localhost:8099/` should work via WSL2 localhost forwarding, or use Pop!_OS IP 192.168.1.10:8099 |
| SITL no GPS | Gazebo not running | Start Gazebo first, check GZ_SIM_SYSTEM_PLUGIN_PATH, SIM_GZ_EN=1 |
| Pi /health not reachable | WSL2 firewall | `curl http://172.20.123.45:8000/health` inside WSL2, from Windows use localhost:8000 |
| Drone flies away in sim | No fence, bad mission | Load fence, set FENCE_ENABLE=1, check mission has RTL at end, Pi clamps targets |
| YOLO 0 boxes at 15m | Classical config | Use config.gazebo.yolo.yaml kind: yolo conf_thr 0.25 tile_bottom 3x3 |

---

## Verification (No Crash)

- [ ] WSL2 Gazebo + SITL + Pi all up, /health ok, detector yolo, cams ok
- [ ] Windows MP connects TCP 127.0.0.1:5760 or 192.168.1.10:5760, mode AUTO, GPS 3D, EKF green
- [ ] Fence loaded, polygon→fence UI works, MP Fence tab shows inclusion
- [ ] FOV coverage 35.7×23m @15m, no gaps, spacing changes with max_alt 15/10/5
- [ ] Arm Takeoff 15m AUTO, Pi trigger on DO_SPRAYER, GUIDED takeover, hold
- [ ] Yaw + grid inside fence, no breach, YOLO tiled 0.8+ @15m
- [ ] Track → approach stair 12→9→7 → decode → transmit → RTL
- [ ] Kill Pi → SITL RTL itself; kill WiFi → MP disconnect but SITL+Pi continue; battery low → RTL; mode RTL externally → Pi aborts
- [ ] Logs: pending_results.jsonl, /health, MP tlog has QR:<payload>

This Windows setup can't crash real drone — it's SITL sim. For real drone, see Doc 11.
