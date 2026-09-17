# Fix: MP Timeout on setWPTotal + Lag + Home Not Set + Connected to 5763

**Error you saw:**
```
Timeout on read - setWPTotal    at MissionPlanner.MAVLinkInterface.setWPTotalAsync (System.Byte sysid, System.Byte compid, System.UInt16 wp_total, MAVLink+MAV_MISSION_TYPE type)
...
mission planner is lagging couldnt connect to the port 5670 so got connected to the 5763, but mission planner is very unresponsive with home position isnt set
```

## Root Cause (Port Conflict)

**SITL's default 5760 is for MAVProxy internal, NOT for MP external.**

- ArduPilot SITL binary itself listens on `127.0.0.1:5760` for MAVProxy to connect as client.
- If you run `sim_vehicle.py --out tcp:0.0.0.0:5760`, MAVProxy tries to create a TCP server on same port 5760 that SITL already occupies → **bind fails**, MAVProxy falls back to next free port **5763** (your case).
- You then connect MP to 5763, which is actually Pi's port (mission_pi uses `tcp:127.0.0.1:5762` or `5763`), so **MP and Pi contend on same port** → lag + `setWPTotal` timeout (FC not responding to MISSION_COUNT because link saturated) + home not set (GPS/ EKF not ready due to Gazebo not fully up).

**Your typo `5670` vs `5760` also suggests port confusion.**

## Correct Ports (No Conflict)

```
SITL internal: 127.0.0.1:5760 (DO NOT use --out 5760, leave for SITL<->MAVProxy)
MP external:   0.0.0.0:5762  → Windows MP connects TCP 192.168.1.10:5762
Pi external:   0.0.0.0:5763  → Pi mission_pi uses tcp:127.0.0.1:5763
UDP fallback:  127.0.0.1:14550 (more stable than TCP, use if TCP laggy)
```

## Fixed Start Command

**Terminal 3: SITL (Pop!_OS) — USE THIS, NOT 5760:**
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
  --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550

# Inside MAVProxy:
param set SIM_GZ_EN 1
param set SERIAL0_PROTOCOL 2
param set SERIAL1_PROTOCOL 2
param set AHRS_EKF_TYPE 3
param set EK2_ENABLE 0
param set EK3_ENABLE 1
param set FENCE_ENABLE 0  # disable temp to test mission upload, enable later
param set FS_GCS_ENABLE 0  # bench 0, real 1
param set SIM_GPS_DELAY 0
param set GPS_TYPE 1
```

**Terminal 4: Pi — USE 5763 config (NOT 5762 which MP uses):**
```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 main.py --config config.gazebo.yolo.5763.yaml
# fc.conn: tcp:127.0.0.1:5763
```

**Windows MP:**
- Connection: **TCP**
- IP: `192.168.1.10` (Pop!_OS IP, not 127.0.0.1)
- Port: **5762** (NOT 5670 typo, NOT 5760 conflict, NOT 5763 Pi port)
- Baud: 57600 (ignored for TCP)
- Connect

If 5762 still fails, try UDP 14550 (more stable):
- MP Connection: UDP, Port 14550, Connect
- Needs SITL `--out udp:192.168.1.20:14550` (Windows IP)

## Home Position Not Set — Fix

Home not set = GPS no 3D fix or EKF origin not set.

**Check in MAVProxy Terminal 3:**
```bash
status
# Look for:
# GPS 3D fix
# EKF3 IMU0 is using GPS
# EKF origin set
# If not:
param set SIM_GPS_DELAY 0
param set SIM_GPS_TYPE 1
# Wait 10s
```

**Force home:**
```bash
# In MAVProxy:
arm throttle
disarm
# This forces EKF to set origin and home
```

**In Mission Planner:**
- Flight Data → Map → Right-click → Set Home Here → Set Home to Vehicle Location
- Or: Flight Data → Actions tab → Set Home to Vehicle Location
- Or: Ctrl+F → set_home

**Check Gazebo:**
- Terminal 1 must show `iris_dualcam + qr_target_a3 + qr_target_big spawned`
- `gz topic -l | grep iris` should show `/iris/front/image`, `/iris/bottom/image`, `/world/mission_world/clock`, etc.
- If not, Gazebo world didn't load — GPS will be no fix.

## setWPTotal Timeout — Fix

Timeout on `setWPTotal` = FC not responding to `MISSION_COUNT` (first step of mission upload).

**Causes:**
- Link saturated (MP + Pi on same port 5763)
- FC busy (fence upload from Pi + mission upload from MP at same time)
- Mission too large or corrupt
- EKF not ready

**Fix steps:**

1. **Stop Pi temporarily to free link:**
   ```bash
   pkill -f main.py
   ```

2. **In MP: test empty mission:**
   - Plan → Clear all waypoints (trash icon) → Write WPs
   - If empty write succeeds, link is OK.

3. **Load small mission:**
   - Plan → Load `mission_pi/sim/missions/qr_search.txt` (3-4 waypoints) → Write WPs
   - Should succeed quickly.

4. **If still timeout:**
   - MP: Config → Full Parameter List → `WP_TOTAL_MAX` 50
   - MP: Disconnect, wait 5s, Connect again to 5762
   - Try again

5. **After mission upload succeeds, restart Pi:**
   ```bash
   cd mission_pi && python3 main.py --config config.gazebo.yolo.5763.yaml
   ```

6. **Avoid contention:** Never have MP and Pi on same port. MP=5762, Pi=5763, separate.

## MP Laggy — Fix

MP very unresponsive = too much traffic or wrong port.

- Don't connect MP to 5763 if Pi is on 5763 — contention causes lag.
- Use 5762 for MP, 5763 for Pi.
- In MP: Config → Planner → uncheck "Show simple mode", reduce rate.
- In MP: Ctrl+F → MAVLink forwarding → remove extra forwards, keep only `127.0.0.1:14551` if needed for UI bridge.
- Close extra MP tabs (tlog replay, etc).

## Quick Diagnostic Script

Run on Pop!_OS after SITL started:

```bash
python3 /home/user/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi/tools/fix_mp_timeout_home.sh
# Or manual:
python3 - << 'PY'
import socket
for port in [5760,5762,5763]:
    s=socket.socket()
    s.settimeout(2)
    try:
        s.connect(('127.0.0.1',port))
        print(f"Port {port}: CONNECTED")
        data=s.recv(1024)
        print(f"  {port}: {len(data)} bytes OK")
    except Exception as e:
        print(f"Port {port}: FAIL {e}")
PY
```

## Verification Checklist After Fix

- [ ] `ss -tlnp | grep 576` shows 5762 and 5763 LISTEN, 5760 is SITL internal (127.0.0.1:5760, not 0.0.0.0:5760)
- [ ] Gazebo shows `iris_dualcam + qr_target_a3 + qr_target_big spawned`
- [ ] `gz topic -l | grep iris` shows front/bottom image topics
- [ ] MAVProxy `status` shows GPS 3D fix, EKF origin set, home set
- [ ] `curl http://127.0.0.1:8000/health` shows `connected true mode STABILIZE`
- [ ] Windows MP connects TCP 192.168.1.10:5762 (not 5760, not 5763), HUD shows mode, GPS 3D, EKF green, home set
- [ ] MP Plan → Write WPs empty succeeds, then small mission succeeds (no setWPTotal timeout)
- [ ] Pi on 5763, MP on 5762, no lag, no contention
- [ ] After mission upload, Pi takeover works, QR detection at 15m (triple QR 1.782x2.52 and big 3.564x5.04)

## Updated Configs

- `config.gazebo.yolo.5763.yaml`: Pi uses 5763, MP uses 5762 — no conflict, fixes timeout+lag
- `config.gazebo.yolo.5760.yaml`: fallback if 5762 heartbeat fails (multiple clients on same port allowed)
- `config.gazebo.yolo.yaml`: now should use 5763 (update to 5763)

## If Still Fails

- Try UDP: SITL `--out udp:192.168.1.20:14550`, MP UDP 14550 — more stable than TCP, no port conflict
- Check firewall: `sudo ufw allow 5762/tcp && sudo ufw allow 5763/tcp && sudo ufw allow 14550/udp`
- Check MP version: 1.3.80+ has better MAVLink2
- Try MP Beta: Help → Check for Beta Updates
- In MP: Config → Full Parameter List → `SERIAL0_BAUD 115`, `SERIAL1_BAUD 57` (should be 115200 and 57600)
