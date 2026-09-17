#!/bin/bash
# Fix 2-laptop network: Pop!_OS SITL (5762/5763) -> Windows MP no heartbeat
# Your SITL log shows local OK (1363 params) but Windows MP TCP 5762 no heartbeat = firewall/network

set -e

POP_IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)
echo "Pop!_OS IP: $POP_IP (Windows MP will connect to this)"

echo ""
echo "=== 1. Check SITL is listening on 0.0.0.0:5762 and 5763 (not just 127.0.0.1) ==="
ss -tlnp | grep -E "5760|5762|5763|14550" || echo "No LISTEN — SITL not running?"
echo ""
echo "If you see 127.0.0.1:5762 instead of 0.0.0.0:5762, Windows can't reach it!"
echo "Fix: sim_vehicle.py must use --out tcp:0.0.0.0:5762 (0.0.0.0, not 127.0.0.1)"
echo ""

echo "=== 2. Firewall — allow 5762,5763,14550,8000,8099 ==="
sudo ufw allow 5762/tcp || true
sudo ufw allow 5763/tcp || true
sudo ufw allow 14550/udp || true
sudo ufw allow 14551/udp || true
sudo ufw allow 8000/tcp || true
sudo ufw allow 8099/tcp || true
sudo ufw status | grep -E "5762|5763|14550|8000|8099" || true
echo "Firewall allowed"

echo ""
echo "=== 3. Test local TCP (Pop!_OS itself) ==="
for port in 5762 5763; do
  echo -n "Testing 127.0.0.1:$port ... "
  if timeout 2 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$port" 2>/dev/null; then
    echo "OK"
  else
    echo "FAIL — SITL not listening on $port"
  fi
done

echo ""
echo "=== 4. Find Windows IP (for UDP method) ==="
echo "On Windows PowerShell, run: ipconfig"
echo "Look for IPv4 192.168.x.x on same WiFi as Pop!_OS"
echo "Example: Windows IP 192.168.1.20, Pop!_OS IP $POP_IP"
echo ""
read -p "Enter Windows IP (e.g. 192.168.1.20) or press Enter to skip UDP setup: " WIN_IP
if [ -n "$WIN_IP" ]; then
  echo "Testing ping to Windows $WIN_IP..."
  ping -c 2 $WIN_IP || echo "Ping FAIL — different subnet or AP isolation (router blocks client-to-client)"
  echo ""
  echo "If ping FAILS, your router has AP isolation / client isolation ON"
  echo "Fix: Router admin -> Wireless -> Advanced -> AP Isolation OFF, or connect both laptops to same phone hotspot"
  echo ""
  echo "Setting up UDP output to Windows (more reliable than TCP for 2-laptop)..."
  echo "In your SITL Terminal 3 (MAVProxy), run:"
  echo "  output add udp:$WIN_IP:14550"
  echo "  output add udp:$WIN_IP:14551"
  echo "Then in Windows MP:"
  echo "  Connection: UDP, Port 14550, Connect"
  echo "  MP should get heartbeat via UDP even if TCP 5762 fails"
fi

echo ""
echo "=== 5. Windows MP Connection — use 5762 NOT 5760 NOT 5670 typo ==="
echo "MP Top right:"
echo "  Connection: TCP"
echo "  IP: $POP_IP (NOT 127.0.0.1, NOT localhost)"
echo "  Port: 5762 (NOT 5670 typo, NOT 5760 internal, NOT 5763 Pi port)"
echo "  Baud: 57600"
echo "  Connect"
echo ""
echo "If TCP 5762 still no heartbeat, try these in order:"
echo "  1. TCP $POP_IP:5763 (Pi port, but will contend with Pi — stop Pi first: pkill -f main.py)"
echo "  2. UDP 14550 (needs SITL --out udp:$WIN_IP:14550, see above)"
echo "  3. Check Windows Firewall: Allow Mission Planner through firewall"
echo ""

echo "=== 6. Check Windows can reach Pop!_OS ==="
echo "On Windows PowerShell:"
echo "  ping $POP_IP"
echo "  Test-NetConnection $POP_IP -Port 5762"
echo "  Test-NetConnection $POP_IP -Port 5763"
echo "Both should succeed. If not, firewall or AP isolation."
echo ""

echo "=== 7. Correct SITL start for 2-laptop (no 5760 conflict) ==="
echo "Your current: --out 127.0.0.1:14550 --master tcp:127.0.0.1:5760 --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550"
echo "This is ALMOST correct, but you have duplicate 127.0.0.1:14550 twice, and you need Windows UDP"
echo "Use:"
echo "  ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \\"
echo "    --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550 --out udp:$WIN_IP:14550"
echo ""
echo "  - 5762 = Windows MP TCP"
echo "  - 5763 = Pi mission_pi tcp:127.0.0.1:5763"
echo "  - 14550 = local UDP"
echo "  - udp:$WIN_IP:14550 = Windows MP UDP (more reliable)"
echo ""

echo "=== 8. Pi config — use 5763 to avoid contention with MP on 5762 ==="
echo "In mission_pi/config.gazebo.yolo.yaml or config.gazebo.yolo.5763.yaml:"
echo "  fc:"
echo "    conn: tcp:127.0.0.1:5763"
echo "NOT 5762 which MP uses"
echo ""

echo "=== 9. Home not set + setWPTotal timeout (after network fixed) ==="
echo "After MP connects and gets heartbeat, if home not set:"
echo "  MAVProxy: status (check GPS 3D fix, EKF origin set)"
echo "  MAVProxy: arm throttle; disarm (forces home)"
echo "  MP: Flight Data -> right-click Map -> Set Home Here -> Set Home to Vehicle Location"
echo ""
echo "If setWPTotal timeout:"
echo "  Stop Pi: pkill -f main.py"
echo "  MP Plan -> Clear WPs -> Write WPs (empty should succeed)"
echo "  Load small mission qr_search.txt -> Write WPs"
echo "  Restart Pi on 5763"
echo ""

echo "=== Done ==="
echo "Quick test from Pop!_OS:"
echo "  ss -tlnp | grep 5762"
echo "  nc -zv 127.0.0.1 5762"
echo "  nc -zv $POP_IP 5762"
echo ""
echo "From Windows:"
echo "  ping $POP_IP"
echo "  Test-NetConnection $POP_IP -Port 5762"
echo ""
echo "If both succeed but MP still no heartbeat, try UDP method (more reliable for 2-laptop)"
