# FIX from Working Branch — RUN_GAZEBO.md Method (Was Working Before)

**Your issue:** `no FC heartbeat on 5762` + MP `no heartbeat packet received` / `Sequence contains no elements`

**Working branch `main` RUN_GAZEBO.md uses this exact SITL command — no --out args:**

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# This prints: "SERIAL0 on TCP port 5760" + "Waiting for connection"
# After MP connects to 5760, it prints: "SERIAL1 on TCP port 5762" — that's cue for Pi
```

**Your recent attempts used `--out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762` which can conflict with default SERIAL0/SERIAL1. Use working method.**

---

## Clean Start — Working Branch Method

### Kill All
```bash
pkill -9 gz; pkill -9 -f sim_vehicle; pkill -9 -f gz_cam_bridge; pkill -9 -f main.py; pkill -9 -f arducopter; sleep 3
ss -tlnp | grep -E "5760|5762|8000|8099"
# Must be empty
```

### Terminal 1 — Gazebo (Working Method)
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -v 4 -r sim/worlds/mission_world.sdf
# Wait: iris_dualcam + qr_target spawned
# Leave running
```

### Terminal 2 — SITL (Working Method — No --out, No MAVProxy)
```bash
# Exit conda
conda deactivate; deactivate; which python3
# /usr/bin/python3 for check, but SITL uses venv-ardupilot
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot
# NOT: --out=tcp:0.0.0.0:5760 --out=tcp:0.0.0.0:5762
# USE: --no-mavproxy — this makes SERIAL0 listen on 5760 by default
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# Expected:
# SERIAL0 on TCP port 5760
# Waiting for connection
# Leave running, don't close
# This command rebuilds if ardupilot/ changed, else starts in seconds
```

**Check SITL listening:**
```bash
# New terminal:
ss -tlnp | grep 5760
# Must show 0.0.0.0:5760 LISTEN (or 127.0.0.1:5760)
# At this point 5762 NOT yet LISTEN — it appears AFTER MP connects to 5760
```

### Terminal 3 — Mission Planner on Windows (Connect to 5760 FIRST)

**Your Pop!_OS IP is 192.168.29.221 (from Pi log). Windows is 192.168.29.2**

```powershell
# Windows PowerShell
ping 192.168.29.221
# Must ping

# Mission Planner:
# Top dropdown TCP → Host 192.168.29.221 Port 5760 → Connect Baud 57600
# Flight Data should come alive: HUD, GPS 3D fix, EKF green, mode STABILIZE

# If "Sequence contains no elements" or "no heartbeat packet received":
# 1. Check Pop!_OS Terminal 2 SITL is running with --no-mavproxy and shows SERIAL0 on TCP port 5760
# 2. Check Pop!_OS: ss -tlnp | grep 5760 — must LISTEN
# 3. Check Pop!_OS firewall: sudo ufw allow 5760/tcp
# 4. In MP, try Clear then Connect again
# 5. Check MP log: Documents/Mission Planner/logs
```

**After MP connects to 5760, Terminal 2 SITL will print:**
```
SERIAL1 on TCP port 5762
```
**This is cue for Pi — now 5762 is LISTEN**

**Check:**
```bash
ss -tlnp | grep 5762
# Now must show 0.0.0.0:5762 LISTEN or 127.0.0.1:5762 LISTEN
# If not, MP didn't connect properly — MP must be connected first
```

### Terminal 4 — Camera Bridge (SYSTEM python3, NOT venv)

```bash
# New terminal, NO venv, NO conda
conda deactivate; deactivate; which python3
# /usr/bin/python3

cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
/usr/bin/python3 tools/gz_cam_bridge.py --port 8099
# Or with boost:
# /usr/bin/python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress --verbose
# Check: curl http://127.0.0.1:8099/health
```

### Terminal 5 — mission_pi YOLO (After SITL shows SERIAL1 on 5762)

```bash
# New terminal, WITH venv-ardupilot
conda deactivate; deactivate
source ~/venv-ardupilot/bin/activate
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# Use working config (not yolo.5760) — now 5762 should work because MP connected first
python3 main.py --config config.gazebo.yolo.yaml
# Or: config.gazebo.yaml for classical
# Should show: FC found: tcp:127.0.0.1:5762 @ net (sysid=1) — NOT FAILED
# If still FAILED, use 5760 fix: config.gazebo.yolo.5760.yaml uses tcp:127.0.0.1:5760 (multiple clients allowed)

# Check health:
curl http://127.0.0.1:8000/health | python3 -m json.tool
```

### Terminal 6 — UI Bridge (Windows)

```powershell
# Windows PowerShell
cd C:\arena\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
.venv\Scripts\activate
python bridge\mp_bridge.py
# UI http://127.0.0.1:8100
# MP Ctrl+F Mavlink UDP Client Write access Connect 127.0.0.1 14551
# UI MAV green, drone marker
```

---

## Why This Order Matters (Working Branch)

From RUN_GAZEBO.md §4:

> **Mission Planner (Windows, same WiFi)**
> Top-right dropdown → **TCP** → Connect → host = Linux WiFi IP, port **5760** → OK. Flight Data comes alive; **Terminal B prints the `SERIAL1 on TCP port 5762` line — that is the cue for step 5.**

So:
1. Gazebo first
2. SITL with --no-mavproxy → SERIAL0 5760 LISTEN
3. MP connects to 5760 → Flight Data alive → SITL prints SERIAL1 on 5762 → now 5762 LISTEN
4. Bridge (any time after Gazebo)
5. mission_pi connects to 5762 (now LISTEN) → FC found

Your error was starting Pi before MP — 5762 not LISTEN yet, so Pi says no heartbeat. Working method: MP first, then Pi.

---

## All Commands — Working Branch Copy-Paste

```bash
# Pop!_OS
pkill -9 gz; pkill -9 -f sim_vehicle; pkill -9 -f gz_cam_bridge; pkill -9 -f main.py; sleep 3

# Terminal 1 Gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
gz sim -v 4 -r sim/worlds/mission_world.sdf

# Terminal 2 SITL (venv-ardupilot, --no-mavproxy, no --out)
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# Wait SERIAL0 on TCP port 5760

# Windows MP: TCP 192.168.29.221:5760 Connect — wait Flight Data alive
# Pop!_OS Terminal 2 will print SERIAL1 on TCP port 5762 — now 5762 LISTEN
# Check: ss -tlnp | grep 5762

# Terminal 3 Bridge (SYSTEM python3)
conda deactivate; deactivate
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
/usr/bin/python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress --verbose

# Terminal 4 Pi YOLO (venv-ardupilot, after MP connected)
source ~/venv-ardupilot/bin/activate
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.yaml
# FC found: tcp:127.0.0.1:5762

# If still FAILED, use 5760 config (multiple clients allowed):
# python3 main.py --config config.gazebo.yolo.5760.yaml
# fc.conn tcp:127.0.0.1:5760
```

This is the working method from main branch RUN_GAZEBO.md that was working before — MP first, then Pi, 5762 appears after MP connects to 5760.
