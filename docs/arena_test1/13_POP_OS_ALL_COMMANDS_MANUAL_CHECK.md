# Pop!_OS — All Commands to Run Gazebo+SITL Manually (Copy-Paste)

**Laptop:** Pop!_OS 22.04, 16GB RAM, GPU
**Goal:** Manually check Gazebo+SITL+mission_pi YOLO — no crash, safe

---

## 0. Prerequisites Check (Once)

```bash
# Check OS
lsb_release -a
# Pop!_OS 22.04

# Check IP for 2-laptop (Windows MP will connect to this)
ip addr | grep 192.168
# Example: 192.168.1.10/24 on wlp0s20f3
# Save this IP: POP_IP=192.168.1.10

# Check GPU for Gazebo
glxinfo | grep "OpenGL renderer"
# Should show NVIDIA or Intel, not llvmpipe

# Check repo
cd ~
ls arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi/
# Should have models/qr_yolov8n.pt 24MB + .onnx 12MB
cd ~/arena_ai_uploads
git checkout arena/01a0a7de-arena-ai-uploads
git pull
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
ls -lh models/qr_yolov8n.pt models/qr_yolov8n.onnx
# -rw-r--r-- 24M qr_yolov8n.pt
# -rw-r--r-- 12M qr_yolov8n.onnx

# Check Python deps
pip3 list | grep -E "ultralytics|onnxruntime|opencv"
# If missing:
pip install --break-system-packages ultralytics onnxruntime opencv-python-headless qrcode pillow numpy
pip install --break-system-packages -r requirements.txt

# Firewall for 2-laptop (allow Windows MP to connect)
sudo ufw allow 5760/tcp
sudo ufw allow 5762/tcp
sudo ufw allow 8000/tcp
sudo ufw allow 8099/tcp
sudo ufw status

# Check Gazebo + ArduPilot
gz sim --version
# gz sim 8.x (Harmonic)
~/ardupilot/ArduCopter/waf --version
# or: ls ~/ardupilot/ArduCopter/arducopter
```

---

## 1. Terminal 1 — Gazebo World with A3 QR (Must Be First)

```bash
# Terminal 1
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Check A3 size 0.297×0.42m
grep -A2 "size" sim/models/qr_target/model.sdf
# Should be 0.297 0.42 — if not, edit

# Optional: regenerate QR texture
python3 sim/textures/generate_qr_texture.py --payload TEST-QR-15M --size 0.297 --out sim/models/qr_target/materials/textures/qr.png

# Launch Gazebo
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Wait for:
# [Msg] iris_dualcam spawned
# [Msg] qr_target spawned
# You should see ground plane + QR box white/black crisp + iris drone

# Manual check: list topics
# In another terminal:
gz topic -l | grep -E "image|iris"
# Should show /iris/front/image and /iris/bottom/image
```

**If Gazebo fails:**
```bash
# Check plugin path
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Should include $HOME/ardupilot_gazebo/build
# If not:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -v 4 -r sim/worlds/mission_world.sdf
```

---

## 2. Terminal 2 — Camera Bridge with QR Boost (SYSTEM python3, not venv)

```bash
# Terminal 2 — Use SYSTEM python3, not .venv (gz-transport is system package)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Check gz-transport installed
python3 -c "import gz.transport13; print('ok')"
# If fails: sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil python3-opencv

# Run bridge with Kabaddi QR boost (makes QR pop vs ground)
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose
# Expected output:
# [INFO] QR boost ON: contrast 1.8 sat 0.6 sharp 2.0 brightness 0.9 mode ground_suppress
# [INFO] Serving MJPEG on :8099
# [INFO] front.mjpg boosted, bottom.mjpg boosted, front_raw.mjpg raw, bottom_raw.mjpg raw
# [INFO] /health JSON

# Manual check (in another terminal or browser):
curl http://127.0.0.1:8099/ | head -20
# Should show HTML with links to front.mjpg, bottom.mjpg, /health
curl http://127.0.0.1:8099/health | python3 -m json.tool
# Should show: {"qr_boost": {"enabled": true, "contrast": 1.8, "saturation": 0.6, ...}, "front": "ok", "bottom": "ok"}

# Check from Windows laptop (if 2-laptop):
# On Windows PowerShell: curl http://192.168.1.10:8099/health
# Should work if firewall allows

# Check MJPEG stream:
# Browser: http://127.0.0.1:8099/bottom.mjpg — should show ground with QR white/black crisp vs gray ground
# If dark: gz sim must be running first, then bridge
```

---

## 3. Terminal 3 — SITL (ArduPilot) — Binds 0.0.0.0 for Windows MP

```bash
# Terminal 3
cd ~/ardupilot/ArduCopter

# Run SITL with Gazebo iris model, out to Windows MP + local Pi
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# --out 0.0.0.0:5760 for Windows Mission Planner (TCP 192.168.1.10:5760)
# --out 0.0.0.0:5762 for local mission_pi (tcp:127.0.0.1:5762)

# Wait for:
# EKF3 IMU0 is using GPS
# GPS 3D fix
# EKF3 ready
# Home set

# In MAVProxy (inside SITL console), set params for safety:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4
param set FENCE_ACTION 1
param set FS_GCS_ENABLE 0
# For bench FS_GCS_ENABLE 0 (don't RTL on GCS loss), real drone 1
param set BATT_FS_LOW_ACT 0
# Bench 0, real drone 2 RTL

# Manual check: check mode, arm check
mode
# Should be STABILIZE
arm check
# Should pass EKF, GPS, fence

# Check TCP listeners:
ss -tlnp | grep 5760
# Should show 0.0.0.0:5760 LISTEN
ss -tlnp | grep 5762
# Should show 0.0.0.0:5762 LISTEN

# For 2-laptop: Windows MP will connect to tcp:192.168.1.10:5760
# Test from Pop!_OS: 
# In another terminal: 
# python3 -c "import socket; s=socket.socket(); s.settimeout(2); print(s.connect_ex(('127.0.0.1',5760)))"
# Should be 0 (open)
```

**If SITL fails GPS:**
```bash
# Check Gazebo running, plugin path
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Should include ~/ardupilot_gazebo/build
# Restart Gazebo first, then SITL
```

---

## 4. Terminal 4 — mission_pi with YOLO (NOT classical — classical FAILS at 15m)

```bash
# Terminal 4
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Check config (YOLO mandatory for 15m A3)
cat config.gazebo.yolo.yaml | grep -A15 detector:
# Should show:
#   kind: yolo
#   model_path: models/qr_yolov8n.pt
#   conf_thr: 0.25
#   tile_bottom: true
#   tile_grid: [3, 3]

# Check model exists
ls -lh models/qr_yolov8n.pt
# 24M

# Optional: bench YOLO vs classical at 34px@15m (proves YOLO mandatory)
python3 tools/yolo_bench.py --model models/qr_yolov8n.pt --conf 0.25
# Expected:
# 34px classical 0 FAIL | yolo_full 1 0.427 PASS | yolo_tiled_3x3 1 0.813 PASS
# 50px classical 0 FAIL | yolo_tiled 0.793 PASS

# Run mission_pi with YOLO
python3 main.py --config config.gazebo.yolo.yaml
# Expected logs:
# [INFO] Server up http://0.0.0.0:8000, /health answers immediately (server first)
# [INFO] FC link: probing tcp:127.0.0.1:5762
# [INFO] FC found: tcp:127.0.0.1:5762 @ net (sysid=1)
# [INFO] FC watchdog up (dead after 10s, retry every 5s)
# [INFO] home: lat, lon, fence N verts, ~WxH m, speed snapshot WP_SPD
# [INFO] Loading YOLO ultralytics model: models/qr_yolov8n.pt conf 0.25 iou 0.45
# [INFO] YOLO ultralytics ready: models/qr_yolov8n.pt size=640
# [INFO] Cameras ready: front url http://127.0.0.1:8099/front.mjpg, bottom url http://127.0.0.1:8099/bottom.mjpg, QR boost ON
# [INFO] phase -> BOOT (init) -> WAIT_LINK (waiting for FC heartbeat) -> SNAPSHOT (reading home/fence/plan) -> WAIT_TRIGGER (waiting for sprayer seqs [2])
# [INFO] hb sources at WAIT: {(1, 2): 100, (255, 6): 10} — vehicle + MP
# [INFO] /health http://0.0.0.0:8000/health

# Manual check health (in another terminal):
curl http://127.0.0.1:8000/health | python3 -m json.tool
# Should show:
# {
#   "device": "tcp:127.0.0.1:5762",
#   "connected": true,
#   "down": false,
#   "reconnects": 0,
#   "hb_age_s": 0.5,
#   "mode": "STABILIZE",
#   "watchdog": true,
#   "cams": {"front": "ok", "bottom": "ok"},
#   "detector": "yolo_ultralytics ok",
#   "qr_boost": "ON",
#   "max_alt_m": 15.0,
#   "fov": {"cam": "cam2", "hfov_deg": 68.0, "footprint_w_m": 35.7, ...}
# }

# Check from Windows laptop:
# Browser: http://192.168.1.10:8000/health — should work if firewall allows
# Browser: http://192.168.1.10:8000/ — UI with both cams + boxes

# Check detector factory:
python3 -c "from detector import get_detector; d=get_detector({'kind':'yolo','model_path':'models/qr_yolov8n.pt','conf_thr':0.25}); print(d.name, d.model_path)"
# yolo_ultralytics models/qr_yolov8n.pt
```

---

## 5. Terminal 5 (Optional) — UI Browser

```bash
# On Pop!_OS
xdg-open http://127.0.0.1:8000
# Shows both cams front+bottom, detector boxes, FSM, coverage, logs

# On Windows (2-laptop):
# Browser: http://192.168.1.10:8000/ (Pop!_OS Pi)
# Should show same UI
```

---

## 6. Fly Mission — Manual Checks (Safe, No Crash)

### In MAVProxy (Terminal 3) — Safe Trigger

```bash
# Check pre-arm
arm check
# Must pass EKF, GPS, fence

# Load mission with DO_SPRAYER at seq 2
wp load ../mission_pi/sim/missions/qr_search.txt
# Or: wp list — should show TAKEOFF, DO_SPRAYER, WAYPOINTs, RTL

# Load fence (if any)
fence load ../mission_pi/sim/fence.txt
# Or via UI polygon→fence

# Arm and Takeoff to 15m
mode GUIDED
arm throttle
takeoff 15
# Wait for 15m: watch alt in MAVProxy or MP

# Switch to AUTO to start mission
mode AUTO
# Mission flies, CURRENT increases

# Watch mission_pi logs (Terminal 4):
# [INFO] mission current: 0, 1, 2
# [WARN] sprayer STATUSTEXT (gate=AUTO)
# [WARN] trigger: mission seq 2 >= sprayer 2 (gate=AUTO)
# [INFO] phase -> TAKEOVER (commanding GUIDED)
# [INFO] phase -> SWEEP_YAW (12 x 30 deg stepped sweep)
# [INFO] phase -> SWEEP_GRID (14 legs, 15.0m spacing @ 12m, 2.5m edge)

# Search: yaw sweep + grid inside fence, YOLO tiled 0.81 @15m PASS
# When QR cue:
# [WARN] QR cue on cam2 (bottom)
# [INFO] phase -> TRACK (steering to QR cue)
# [INFO] phase -> APPROACH (descending to 12m) -> 9m -> 7m
# [WARN] QR CONFIRMED via cam2: 'TEST-QR-15M'
# [INFO] phase -> TRANSMIT (relaying 'TEST-QR-15M')
# [INFO] phase -> DONE (payload='TEST-QR-15M') — post_action RTL

# Check MP Messages tab (if MP connected): QR:TEST-QR-15M

# Check Pi health after:
curl http://127.0.0.1:8000/health | grep payload
```

### Manual Takeover via UI (Alternative)

```bash
# In UI http://127.0.0.1:8000 or http://192.168.1.10:8000 from Windows
# Click Start mission — Pi goes WAIT_TRIGGER → SEARCH
```

### Safety Checks During Flight (Manual)

```bash
# In another terminal, monitor health every 5s:
watch -n 5 'curl -s http://127.0.0.1:8000/health | python3 -m json.tool | grep -E "mode|down|hb_age|detector|cams|max_alt"'

# Check link supervision:
curl -s http://127.0.0.1:8000/health | python3 -m json.tool | grep -A10 link_state

# Check YOLO detection on synthetic 34px@15m:
python3 tools/yolo_bench.py --model models/qr_yolov8n.pt --conf 0.25 --sizes 34 50

# Test fail-safes manually (safe in SITL):
# 1. Kill Pi (Ctrl+C Terminal 4) → SITL should RTL itself after timeout (check MAVProxy mode → RTL)
# 2. Disconnect TCP: in Terminal 3 MAVProxy, close? Actually kill bridge Terminal 2 → cams fail, /health shows cam failed, but mission continues with other cam or RTL if no cam
# 3. Battery low: param set BATT_LOW_VOLT 15 (high) → Pi _battery_ok() abort → RTL
# 4. Mode RTL externally: in MAVProxy, mode RTL → Pi logs "vehicle left GUIDED (now RTL) — external RTL/failsafe? aborting search" → DONE external RTL
# 5. Geofence breach: fly outside fence in GUIDED → FC FENCE_ACTION=1 RTL → Pi mode_tripped

# All should RTL, no crash, no fly-away
```

---

## 7. Windows 11 Laptop — Commands (2-Laptop)

```powershell
# PowerShell on Windows 11
ipconfig
# 192.168.1.20

ping 192.168.1.10
# Must ping Pop!_OS

# Mission Planner
# Open MP, Select TCP, 192.168.1.10:5760, Baud 57600, Connect
# HUD should show STABILIZE, GPS 3D, EKF green

# MAVLink Forwarding for UI bridge:
# MP → Ctrl+F → MAVLink forwarding → Add 127.0.0.1:14551 → Tick Write access → OK
# Bridge MAV lamp green

# mission-ui bridge
cd C:\arena\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
.venv\Scripts\activate
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles
# http://localhost:8100 UI

# Browser: http://localhost:8100
# Config: max_alt 15, sweep 12, bottom 100° HFOV, overlap 0.35 Apply
# Polygon: draw purple → POLYGON→FENCE INCLUSION → apply to MP → MP Fence tab shows polygon
# Coverage: Show FOV → blue footprints 35.7×23m @15m

# Check Pi from Windows:
# Browser: http://192.168.1.10:8000/ and http://192.168.1.10:8000/health and http://192.168.1.10:8099/health
# All should work

# Fly: MP Plan Load qr_search.txt with DO_SPRAYER seq2 + RTL Upload, Arm Takeoff 15m AUTO, Pi takeover
```

---

## 8. All Commands Summary (Copy-Paste Order)

```bash
# === Pop!_OS Laptop ===

# 0. Check IP
ip addr | grep 192.168
# POP_IP=192.168.1.10

# 1. Gazebo (Terminal 1)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
gz sim -v 4 -r sim/worlds/mission_world.sdf

# 2. Bridge (Terminal 2, SYSTEM python3)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose

# 3. SITL (Terminal 3)
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# In MAVProxy:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4
param set FENCE_ACTION 1
param set FS_GCS_ENABLE 0

# 4. mission_pi YOLO (Terminal 4)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/yolo_bench.py --model models/qr_yolov8n.pt --conf 0.25
python3 main.py --config config.gazebo.yolo.yaml

# 5. Health checks (Terminal 5)
curl http://127.0.0.1:8000/health | python3 -m json.tool
curl http://127.0.0.1:8099/health | python3 -m json.tool
xdg-open http://127.0.0.1:8000

# 6. Fly (MAVProxy in Terminal 3)
arm check
wp load ../mission_pi/sim/missions/qr_search.txt
mode GUIDED
arm throttle
takeoff 15
mode AUTO
# Watch Terminal 4 logs: TRIGGER → TAKEOVER → SWEEP_YAW → SWEEP_GRID → TRACK → APPROACH → TRANSMIT → DONE RTL

# === Windows 11 Laptop (2-laptop) ===
# PowerShell:
ipconfig
ping 192.168.1.10
# MP: TCP 192.168.1.10:5760 Connect, Ctrl+F MAVLink forwarding 127.0.0.1:14551 Write tick
cd C:\arena\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
.venv\Scripts\activate
python bridge\mp_bridge.py --port 8100 --mav-port 14551 --mbtiles field.mbtiles
# Browser http://localhost:8100 + http://192.168.1.10:8000
```

---

## 9. Safety — Why No Crash

- Pi companion sys 51, FC owns motors, EKF, failsafes
- All Pi goto clamped inside fence via _clamp_to_fence() binary shrink 10 iters
- Link supervision dead_s 10s retry_s 5s, link_lost→link_retry→link_restored, store-forward jsonl
- Mode guard _guided_ok() 3 stale tolerance, trip→mode_tripped→restore speed→DONE external RTL
- Battery <25%→abort RTL, search 900s→RTL, approach 240s→resume grid, trigger 900s→FAILSAFE RTL
- Max alt 15/10/5 clamped, sweep and stair clamped, sorted descending
- Fence snapshot at boot, no fence→home box 20m, FOV-optimal coverage no gaps
- Speed snapshot WP_SPD/WPNAV_SPEED, set 1m/s search, restore after
- Reboot GUIDED→RTL, server first /health immediate, bring-up thread
- Detector YOLO mandatory 34px@15m 0.81 tiled vs classical FAIL, fallback classical if no model
- Camera drop no leak, surviving cam untouched, no cam→FAILSAFE RTL
- Pilot RC RTL switch overrides Pi GUIDED immediately, Pi aborts

This Pop!_OS setup can't crash real drone — it's SITL sim. For real drone, see 11_REAL_DRONE_TEST_SAFE.md.

---

## 10. Logs

```bash
# Pop!_OS
cat logs/pending_results.jsonl
curl http://127.0.0.1:8000/health
curl http://127.0.0.1:8000/api/bringup
# Gazebo console, SITL MAVProxy, mission_pi console

# Windows
# MP Documents/Mission Planner/logs/*.tlog search QR:
# Bridge console, UI http://localhost:8100 Logs card + /api/mp/state
```

Copy-paste these commands in order, check each /health, and you can manually verify YOLO @15m works, no crash, safe.
