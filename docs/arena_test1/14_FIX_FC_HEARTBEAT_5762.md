# FIX: No FC Heartbeat on tcp:127.0.0.1:5762 + Mission Planner "Sequence contains no elements"

**Your log:**
```
bring-up fc FAILED — RuntimeError: no FC heartbeat; tried: tcp:127.0.0.1:5762@net: no heartbeat — port opened but no vehicle heartbeat — check SERIALn_PROTOCOL = MAVLink2 on that port, and that MP is not the only client.
...
still no FC heartbeat after 15s — the UI link stays up; SITL/Gazebo + MP must be connected first
```

**MP error:**
```
Sequence contains no elements
   at MissionPlanner.MAVLinkInterface.OpenBg
```

**Root cause:** SITL not running, or not outputting to TCP 5760/5762, or Gazebo not running, or SERIALn_PROTOCOL not MAVLink2. Your Pop!_OS IP is `192.168.29.221` (from Pi link log), Windows MP must connect to `192.168.29.221:5760`, not 127.0.0.1.

---

## 1. Kill Everything First (Clean Start)

```bash
# On Pop!_OS, kill old Gazebo, SITL, bridge, mission_pi
pkill -9 gz
pkill -9 -f sim_vehicle
pkill -9 -f gz_cam_bridge
pkill -9 -f main.py
sleep 2
ss -tlnp | grep -E "5760|5762|8000|8099"
# Should be empty, no LISTEN
```

---

## 2. Terminal 1 — Gazebo (Must Be First)

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Check plugin path — critical for Gazebo+ArduPilot
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Should contain $HOME/ardupilot_gazebo/build
# If empty:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Launch Gazebo with A3 QR world
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Wait for:
# [Msg] iris_dualcam spawned
# [Msg] qr_target spawned
# Leave this terminal running, don't close
```

**Check Gazebo topics:**
```bash
# In new terminal:
gz topic -l | grep -E "image|iris"
# Should show:
# /iris/front/image
# /iris/bottom/image
# If not, Gazebo world not loaded correctly
```

---

## 3. Terminal 2 — Camera Bridge (SYSTEM python3, not venv)

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Use SYSTEM python3, not venv-ardupilot (gz-transport is system package)
# If you are in venv, deactivate first:
deactivate
# Or: which python3 — should be /usr/bin/python3, not ~/.../venv

# Check gz-transport
python3 -c "import gz.transport13; print('gz transport ok')"

# Run bridge with QR boost
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose
# Expected:
# [INFO] QR boost ON
# [INFO] Serving MJPEG on :8099
# [INFO] front.mjpg boosted, bottom.mjpg boosted
# [INFO] /health JSON

# Check bridge (new terminal):
curl http://127.0.0.1:8099/health | python3 -m json.tool
# Should show qr_boost enabled true, front ok bottom ok
```

---

## 4. Terminal 3 — SITL (Critical Fix for 5760/5762)

**This is the fix for your error.** SITL must output to 0.0.0.0:5760 for Windows MP and 0.0.0.0:5762 for Pi.

```bash
cd ~/ardupilot/ArduCopter

# Activate ardupilot venv if you have it:
source ~/venv-ardupilot/bin/activate
# Or: source ~/.virtualenvs/ardupilot/bin/activate — check your venv-ardupilot prompt shows (venv-ardupilot)

# Run SITL with JSON model + Gazebo, out to both ports
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762

# Wait for:
# EKF3 IMU0 is using GPS
# GPS 3D fix
# EKF3 ready
# Home set

# In MAVProxy (inside SITL console), set these params — critical for heartbeat:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4
param set FENCE_ACTION 1
param set FS_GCS_ENABLE 0
param set BATT_FS_LOW_ACT 0
param show SERIAL*
# SERIAL0_PROTOCOL should be 2 (MAVLink2), SERIAL1_PROTOCOL 2

# Check TCP listeners (new terminal):
ss -tlnp | grep -E "5760|5762"
# Should show:
# 0.0.0.0:5760 LISTEN
# 0.0.0.0:5762 LISTEN
# If not, sim_vehicle.py didn't start correctly — check --out args

# Test TCP 5762 heartbeat manually:
python3 -c "
import socket, time
s=socket.socket()
s.settimeout(5)
try:
    s.connect(('127.0.0.1',5762))
    print('TCP 5762 connected, waiting heartbeat...')
    data=s.recv(1024)
    print(f'Got {len(data)} bytes heartbeat ok')
except Exception as e:
    print(f'Failed: {e}')
"
# Should print "Got X bytes heartbeat ok" — if "Failed: timed out", SITL not sending heartbeat

# Test TCP 5760 for Windows MP:
python3 -c "
import socket
s=socket.socket()
s.settimeout(2)
print('5760:', s.connect_ex(('127.0.0.1',5760)))
"
# Should be 0
```

**If still no heartbeat on 5762:**
```bash
# Try alternative: use udp instead of tcp for Pi
# In config.gazebo.yolo.yaml, change fc.conn to:
# fc:
#   conn: "tcp:127.0.0.1:5760"  # try 5760 instead of 5762, MP and Pi share same port (ArduPilot allows multiple clients on same TCP)
# Or: use --out=udp:127.0.0.1:14550 and set fc.conn: udp:127.0.0.1:14550

# Quick fix: edit config.gazebo.yolo.yaml
# Change 5762 to 5760 and restart mission_pi
```

---

## 5. Terminal 4 — mission_pi YOLO (After SITL is Ready)

**Only start after Terminal 3 shows EKF ready and ss -tlnp shows 5760+5762 LISTEN**

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Activate venv if needed, but use system python3 for YOLO? Actually YOLO needs ultralytics which is in venv-ardupilot or system
# Check:
python3 -c "from ultralytics import YOLO; print('ultralytics ok')"

# If not, pip install:
pip install --break-system-packages ultralytics onnxruntime opencv-python-headless

# Run
python3 main.py --config config.gazebo.yolo.yaml
# Should now show:
# [INFO] FC found: tcp:127.0.0.1:5762 @ net (sysid=1) — NOT FAILED
# [INFO] FC watchdog up
# [INFO] phase -> WAIT_LINK -> SNAPSHOT -> WAIT_TRIGGER
# NOT "no FC heartbeat"

# Check health:
curl http://127.0.0.1:8000/health | python3 -m json.tool
# Should show connected true, mode STABILIZE, not down true
```

---

## 6. Windows 11 Laptop — Mission Planner Connect (Fix "Sequence contains no elements")

**Your Pop!_OS IP is 192.168.29.221 (from log). Windows must connect to this, not 127.0.0.1**

```powershell
# On Windows PowerShell
ipconfig
# Your Windows IP: e.g. 192.168.29.2 (from log: UI websocket connected from 192.168.29.2)

ping 192.168.29.221
# Must ping Pop!_OS — if fails, check same WiFi, firewall

# In Mission Planner:
# 1. Top right: Connection type = TCP, IP = 192.168.29.221, Port = 5760, Baud = 57600, Connect
#    NOT 127.0.0.1:5760 (that's Windows itself), NOT 5762 (that's for Pi)
# 2. If "Sequence contains no elements" still:
#    - Check Pop!_OS Terminal 3 SITL is running and ss -tlnp shows 0.0.0.0:5760 LISTEN
#    - Check Pop!_OS firewall: sudo ufw allow 5760/tcp
#    - Try: In MP, click "Clear" then Connect again
#    - Try: In MP, use UDP instead? No, SITL TCP is correct
#    - Check MP log: C:\Users\...\Documents\Mission Planner\logs\ — look for MAVLink errors
#    - Restart MP, restart SITL

# 3. After MP connects:
#    HUD shows STABILIZE, GPS 3D fix, EKF green, battery
#    Ctrl+F → MAVLink forwarding → Add 127.0.0.1:14551 → Tick Write access → OK
#    For UI bridge

# 4. Check Pi from Windows browser:
#    http://192.168.29.221:8000/ — should show Pi UI with cams
#    http://192.168.29.221:8000/health — should show connected true
#    http://192.168.29.221:8099/health — bridge health
#    If hangs: On Pop!_OS: sudo ufw allow 8000/tcp && sudo ufw allow 8099/tcp
```

**MP "Sequence contains no elements" is known bug when MP can't get params:**
- Cause: SITL not sending heartbeat, or SERIALn_PROTOCOL not MAVLink2, or only one client allowed and Pi already connected to 5760
- Fix: Use separate ports: SITL --out 5760 for MP, --out 5762 for Pi (we did), and ensure both LISTEN
- If still fails: In config.gazebo.yolo.yaml, change Pi to 5760 as well (ArduPilot allows multiple TCP clients on same port) — edit and restart Pi:
```yaml
fc:
  conn: "tcp:127.0.0.1:5760"
```

---

## 7. All Commands Summary (Fixed Order, Copy-Paste)

```bash
# === Pop!_OS ===

# Kill old
pkill -9 gz; pkill -9 -f sim_vehicle; pkill -9 -f gz_cam_bridge; pkill -9 -f main.py; sleep 2

# Terminal 1: Gazebo
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -v 4 -r sim/worlds/mission_world.sdf

# Terminal 2: Bridge (SYSTEM python3, deactivate venv first)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
deactivate  # if in venv
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose

# Terminal 3: SITL (with venv-ardupilot, --out 0.0.0.0:5760 for MP + 5762 for Pi)
cd ~/ardupilot/ArduCopter
source ~/venv-ardupilot/bin/activate  # or your venv
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# Inside MAVProxy:
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set SIM_GZ_EN 1
param set FENCE_ENABLE 1
param set FENCE_TYPE 4
param set FENCE_ACTION 1
param set FS_GCS_ENABLE 0
# Check:
ss -tlnp | grep 5760
# 0.0.0.0:5760 LISTEN + 0.0.0.0:5762 LISTEN

# Terminal 4: mission_pi YOLO (after SITL ready)
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source ~/venv-ardupilot/bin/activate
python3 main.py --config config.gazebo.yolo.yaml
# Should show FC found, not FAILED

# Terminal 5: Health checks
curl http://127.0.0.1:8000/health | python3 -m json.tool
curl http://127.0.0.1:8099/health | python3 -m json.tool
xdg-open http://127.0.0.1:8000

# === Windows 11 ===
# PowerShell:
ping 192.168.29.221
# MP: TCP 192.168.29.221:5760 Baud 57600 Connect
# If Sequence error: check Pop!_OS ss -tlnp 5760 LISTEN, ufw allow 5760/tcp, restart SITL+MP
# MP Ctrl+F MAVLink forwarding 127.0.0.1:14551 Write tick
# Browser: http://192.168.29.221:8000/ + /health + :8099/health
```

---

## 8. Quick Fix If Still Fails

**If 5762 no heartbeat but 5760 works (MP connects but Pi fails):**

```bash
# Edit config.gazebo.yolo.yaml on Pop!_OS:
# Change fc.conn from tcp:127.0.0.1:5762 to tcp:127.0.0.1:5760
nano config.gazebo.yolo.yaml
# fc:
#   conn: "tcp:127.0.0.1:5760"

# Restart mission_pi Terminal 4
# ArduPilot allows multiple TCP clients on same port, so MP (Windows 192.168.29.221:5760) + Pi (127.0.0.1:5760) both work on 5760
```

**If MP still "Sequence contains no elements":**

1. On Pop!_OS, check SITL console for errors: `param show ARMING_CHECK` — set `param set ARMING_CHECK 0` for bench to allow arm without GPS? But need GPS 3D fix
2. On Windows MP, try UDP: In SITL Terminal 3, add `--out=udp:192.168.29.2:14550` (Windows IP from log), then MP connect UDP 14550
3. Restart everything in order: Gazebo → Bridge → SITL → wait EKF ready → Pi → MP

**Your IPs from log:**
- Pop!_OS: 192.168.29.221 (Pi link)
- Windows: 192.168.29.2 (UI websocket connected from)
- Use these for 2-laptop: MP TCP 192.168.29.221:5760, Pi local 127.0.0.1:5762, Windows browser http://192.168.29.221:8000

This fix ensures no crash — Pi waits 600s in WAIT_LINK, UI stays up, FC owns failsafes, timeouts→RTL.
