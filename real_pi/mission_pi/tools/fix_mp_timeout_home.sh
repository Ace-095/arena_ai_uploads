#!/bin/bash
# Fix MP Timeout on setWPTotal + lag + home not set + 5763 fallback
# This script diagnoses and fixes the common SITL+MP issues

set -e

echo "=== MP Timeout + Home Not Set Fix ==="
echo "Error: Timeout on read - setWPTotal at MissionPlanner.MAVLinkInterface.setWPTotalAsync"
echo "Cause: Link contention, port conflict 5760 vs 5762/5763, GPS no fix, EKF no origin"
echo ""

POP_IP=$(ip route get 8.8.8.8 | grep -oP 'src \K[0-9.]+' | head -1)
echo "Pop!_OS IP: $POP_IP"

echo ""
echo "=== 1. Check ports — 5760 is SITL internal, should NOT be used for MP external ==="
echo "SITL binary itself listens on 127.0.0.1:5760 for MAVProxy to connect."
echo "If you do --out tcp:0.0.0.0:5760, you conflict — MAVProxy fails to bind 5760, falls back to 5763 (your case)."
echo "Correct: --out tcp:0.0.0.0:5762 for MP, --out tcp:0.0.0.0:5763 for Pi"
echo ""
ss -tlnp | grep -E "5760|5762|5763|1455" || echo "No LISTEN yet"
echo ""
echo "Killing old SITL that may be hogging 5760..."
pkill -9 -f sim_vehicle || true
pkill -9 -f arducopter || true
sleep 2

echo ""
echo "=== 2. Check Gazebo + GPS ==="
if pgrep -f "gz sim" > /dev/null; then
  echo "Gazebo running: $(pgrep -f 'gz sim')"
  gz topic -l | grep -E "iris|navsat|gps" || echo "No iris/gps topics — Gazebo may not have spawned iris_dualcam"
else
  echo "Gazebo NOT running — GPS will be NO FIX, home NOT SET, MP laggy"
  echo "Start Gazebo first: gz sim -v 4 -r sim/worlds/mission_world.sdf"
fi

echo ""
echo "=== 3. Correct SITL start (no 5760 conflict) ==="
echo "Use this EXACT command in ~/ardupilot/ArduCopter:"
echo ""
echo "  ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \\"
echo "    --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550"
echo ""
echo "  - 5762 = Windows MP (TCP 192.168.1.10:5762, NOT 5760, NOT 5763)"
echo "  - 5763 = Pi mission_pi (tcp:127.0.0.1:5763)"
echo "  - 14550 = local MP UDP fallback (more stable than TCP)"
echo ""
echo "  Inside MAVProxy after start:"
echo "    param set SIM_GZ_EN 1"
echo "    param set SERIAL0_PROTOCOL 2"
echo "    param set SERIAL1_PROTOCOL 2"
echo "    param set AHRS_EKF_TYPE 3"
echo "    param set EK2_ENABLE 0"
echo "    param set EK3_ENABLE 1"
echo "    param set FENCE_ENABLE 0  # disable fence temp to test mission upload"
echo "    param set FS_GCS_ENABLE 0  # bench, real=1"
echo ""

echo "=== 4. Firewall ==="
sudo ufw allow 5762/tcp || true
sudo ufw allow 5763/tcp || true
sudo ufw allow 14550/udp || true
sudo ufw allow 8000/tcp || true
sudo ufw allow 8099/tcp || true
echo "Firewall allowed 5762,5763,14550,8000,8099"

echo ""
echo "=== 5. Test TCP connectivity ==="
echo "After SITL started, test from Pop!_OS:"
echo "  nc -zv 127.0.0.1 5762"
echo "  nc -zv 127.0.0.1 5763"
echo "From Windows PowerShell:"
echo "  Test-NetConnection $POP_IP -Port 5762"
echo "  Test-NetConnection $POP_IP -Port 5763"

echo ""
echo "=== 6. MP Connection — use 5762 NOT 5760 NOT 5763 ==="
echo "MP Top right:"
echo "  Connection: TCP"
echo "  IP: $POP_IP"
echo "  Port: 5762 (NOT 5670 typo, NOT 5760 conflict, NOT 5763 Pi port)"
echo "  Baud: 57600 (ignored for TCP but set)"
echo "  Connect"
echo ""
echo "If 5762 still fails, try UDP:"
echo "  MP Connection: UDP, Port 14550, Connect (needs SITL --out udp:192.168.1.20:14550)"
echo "  Or: MP Connection: TCP, $POP_IP:5763 (Pi port, but will contend with Pi)"
echo ""

echo "=== 7. Home Position Not Set — Fix ==="
echo "Home not set = GPS no 3D fix or EKF origin not set"
echo "In MAVProxy (Terminal 3):"
echo "  status"
echo "  # Look for: GPS 3D fix, EKF3 IMU0 is using GPS, EKF origin set"
echo "  # If not:"
echo "  param set SIM_GPS_DELAY 0"
echo "  param set SIM_GPS_TYPE 1"
echo "  param set GPS_TYPE 1"
echo "  # Wait 10s for GPS"
echo "  # Then:"
echo "  arm throttle"
echo "  disarm"
echo "  # This forces EKF to set origin and home"
echo ""
echo "In Mission Planner (Windows):"
echo "  Flight Data -> Map -> Right-click -> Set Home Here -> Set Home to Vehicle Location"
echo "  Or: Flight Data -> Actions tab -> Set Home to Vehicle Location"
echo "  Or: Ctrl+F -> set_home"
echo ""

echo "=== 8. setWPTotal Timeout — Fix ==="
echo "Timeout on setWPTotal = FC not responding to MISSION_COUNT"
echo "Causes:"
echo "  - Link saturated (too many --out, or Gazebo cam streams over same link? No)"
echo "  - Multiple GCS uploading at same time (Pi fence upload + MP mission upload)"
echo "  - FC busy, EKF not ready, or wrong sysid"
echo ""
echo "Fix:"
echo "  1. Stop mission_pi temporarily: pkill -f main.py"
echo "  2. In MP: Plan -> Clear all waypoints (trash icon) -> Write WPs (should succeed empty)"
echo "  3. If empty write succeeds, link is OK, previous mission was corrupt or too large"
echo "  4. Load qr_search.txt (small, 3-4 waypoints) -> Write WPs"
echo "  5. If still timeout:"
echo "     - MP: Config -> Full Parameter List -> WP_TOTAL_MAX 50"
echo "     - MP: Disconnect, wait 5s, Connect again to 5762"
echo "     - Try again"
echo "  6. After mission upload succeeds, restart mission_pi:"
echo "     cd mission_pi && python3 main.py --config config.gazebo.yolo.5763.yaml"
echo "     (config should use tcp:127.0.0.1:5763, NOT 5762 which MP uses)"
echo ""

echo "=== 9. MP Laggy — Fix ==="
echo "MP very unresponsive = too much MAVLink traffic or wrong port"
echo "  - Don't connect MP to 5763 if Pi is on 5763 — contention causes lag"
echo "  - Use 5762 for MP, 5763 for Pi, separate"
echo "  - In MP: Config -> Planner -> uncheck 'Show simple mode', reduce rate"
echo "  - In MP: Ctrl+F -> MAVLink forwarding -> remove extra forwards, keep only 127.0.0.1:14551 if needed"
echo "  - Close extra MP tabs (tlog replay, etc)"
echo ""

echo "=== 10. Quick Test Script ==="
cat > /tmp/test_sitl_link.py << 'PY'
import socket, time
for port in [5760,5762,5763]:
    s=socket.socket()
    s.settimeout(2)
    try:
        s.connect(('127.0.0.1',port))
        print(f"Port {port}: CONNECTED, waiting heartbeat...")
        data=s.recv(1024)
        print(f"  Port {port}: got {len(data)} bytes — OK")
        s.close()
    except Exception as e:
        print(f"Port {port}: FAIL {e}")
PY
python3 /tmp/test_sitl_link.py

echo ""
echo "=== Done ==="
echo "Now start in order:"
echo "  Terminal 1: gz sim -v 4 -r sim/worlds/mission_world.sdf"
echo "  Terminal 2: /usr/bin/python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --verbose"
echo "  Terminal 3: sim_vehicle.py -f gazebo-iris --model JSON --map --console --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550"
echo "  Terminal 4: python3 main.py --config config.gazebo.yolo.5763.yaml (uses 5763)"
echo "  Windows MP: TCP $POP_IP:5762"
echo ""
echo "If home still not set after GPS 3D fix:"
echo "  MAVProxy: arm throttle; disarm;  # forces home"
echo "  MP: Set Home to Vehicle Location"
