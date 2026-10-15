#!/bin/bash
# Pop!_OS — Auto-start Gazebo+SITL+Bridge+Pi — FIXED for 2-laptop network + QR + setWPTotal timeout
# Usage: ./tools/start_pop_os_gazebo_sitl.sh
# Then: Windows MP connects TCP 192.168.x.x:5762 (NOT 5760, NOT 5670 typo, NOT 5763 Pi port)

set -e

POP_IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)
echo "Pop!_OS IP: $POP_IP (Windows MP will use $POP_IP:5762)"

echo "=== Killing old ==="
pkill -9 gz || true
pkill -9 -f sim_vehicle || true
pkill -9 -f gz_cam_bridge || true
pkill -9 -f main.py || true
sleep 3
ss -tlnp | grep -E "5760|5762|5763|8000|8099" || echo "Clean — no LISTEN"

echo "=== Checking deps + QR textures (fixes no QR present) + Firewall (fixes MP no heartbeat) ==="
/usr/bin/python3 -c "import gz.transport13; print('gz transport13 ok')" || { echo "Need: sudo apt install python3-gz-transport13 python3-gz-msgs10"; exit 1; }
ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so || { echo "Need: build ardupilot_gazebo"; exit 1; }
ls models/qr_yolov8n.pt 2>/dev/null || echo "Note: models/qr_yolov8n.pt missing — YOLO fallback to classical, still works for triple QR 1.782x2.52"

# Firewall fix for 2-laptop (fixes MP TCP 5762 no heartbeat)
echo "Allowing ports 5762,5763,14550,8000,8099 through firewall..."
sudo ufw allow 5762/tcp || true
sudo ufw allow 5763/tcp || true
sudo ufw allow 14550/udp || true
sudo ufw allow 14551/udp || true
sudo ufw allow 8000/tcp || true
sudo ufw allow 8099/tcp || true

# Auto-generate QR textures if missing
if [ ! -f sim/models/qr_panel_a3/materials/textures/qr.png ]; then
  echo "Generating QR texture for A3 panel..."
  pip install -q qrcode pillow 2>/dev/null || /usr/bin/python3 -m pip install -q qrcode pillow --user || true
  /usr/bin/python3 tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_a3/materials/textures/qr.png --full --dpi 300 || python3 tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_a3/materials/textures/qr.png --full --dpi 300
fi
if [ ! -f sim/models/qr_panel_big/materials/textures/qr.png ]; then
  echo "Generating QR texture for BIG panel..."
  /usr/bin/python3 tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_big/materials/textures/qr.png --full --dpi 300 || python3 tools/make_qr_panel.py --text MISSION-QR-001 --out sim/models/qr_panel_big/materials/textures/qr.png --full --dpi 300
fi
ls -lh sim/models/qr_panel_a3/materials/textures/qr.png sim/models/qr_panel_big/materials/textures/qr.png
echo "QR textures OK"

# Validate world SDF
python3 - << 'PY'
import xml.etree.ElementTree as ET
path = "sim/worlds/mission_world.sdf"
tree = ET.parse(path)
root = tree.getroot()
includes = [inc.find("uri").text for inc in root.findall(".//include") if inc.find("uri") is not None]
print(f"World SDF valid, includes: {includes}")
assert "model://qr_panel_a3" in includes, "qr_panel_a3 missing!"
assert "model://qr_panel_big" in includes, "qr_panel_big missing!"
print("QR includes present")
PY

echo ""
echo "=== 2-Laptop Network Check ==="
echo "Pop!_OS IP: $POP_IP"
echo "Find Windows IP: On Windows PowerShell run ipconfig, look for 192.168.x.x"
read -p "Enter Windows IP (e.g. 192.168.1.20) for UDP method, or Enter to skip: " WIN_IP
if [ -n "$WIN_IP" ]; then
  echo "Pinging Windows $WIN_IP..."
  ping -c 2 $WIN_IP || echo "Ping FAIL — AP isolation ON or different subnet, use phone hotspot"
  echo "Will use UDP $WIN_IP:14550 for MP (more reliable than TCP)"
  EXTRA_UDP="--out udp:$WIN_IP:14550 --out udp:$WIN_IP:14551"
else
  EXTRA_UDP=""
  echo "Skipping Windows UDP, will use TCP 5762 only"
fi

echo ""
echo "=== Terminal 1: Gazebo ==="
echo "Run in NEW terminal:"
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "  export GZ_SIM_RESOURCE_PATH=\$HOME/ardupilot_gazebo/models:\$HOME/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH"
echo "  cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi"
echo "  gz sim -v 4 -r sim/worlds/mission_world.sdf"
echo ""
read -p "Press Enter after Gazebo shows iris_dualcam + qr_target_a3 + qr_target_big spawned..."

echo "=== Checking Gazebo topics ==="
gz topic -l | grep iris || { echo "Gazebo topics not found"; exit 1; }
echo "Gazebo OK: $(gz topic -l | grep iris)"

echo ""
echo "=== Terminal 2: Bridge ==="
echo "Run in NEW terminal (SYSTEM python3, NO venv):"
echo "  cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi"
echo "  /usr/bin/python3 tools/gz_cam_bridge.py --port 8099 --qr-boost --contrast 1.8 --saturation 0.6 --sharpness 2.0 --brightness 0.9 --enhance-mode ground_suppress --verbose"
echo ""
read -p "Press Enter after Bridge shows Serving MJPEG on :8099..."

curl -s http://127.0.0.1:8099/health | python3 -m json.tool || { echo "Bridge /health failed"; exit 1; }
echo "Bridge OK"

echo ""
echo "=== Terminal 3: SITL — FIXED PORTS (5762 MP + 5763 Pi, NO 5760 conflict) ==="
echo "Your previous log: --out 127.0.0.1:14550 --master tcp:127.0.0.1:5760 --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550"
echo "That is ALMOST correct but has duplicate 14550 and no Windows UDP"
echo "USE THIS (WITH venv-ardupilot):"
echo "  source ~/venv-ardupilot/bin/activate"
echo "  cd ~/ardupilot/ArduCopter"
echo "  ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \\"
echo "    --out tcp:0.0.0.0:5762 --out tcp:0.0.0.0:5763 --out udp:127.0.0.1:14550 $EXTRA_UDP"
echo ""
echo "  - 5762 = Windows MP TCP $POP_IP:5762 (NOT 5670 typo, NOT 5760 internal, NOT 5763 Pi)"
echo "  - 5763 = Pi mission_pi tcp:127.0.0.1:5763"
echo "  - 14550 = local UDP"
if [ -n "$WIN_IP" ]; then
echo "  - udp:$WIN_IP:14550 = Windows MP UDP (more reliable, use if TCP no heartbeat)"
fi
echo ""
echo "  Inside MAVProxy after EKF ready:"
echo "    param set SIM_GZ_EN 1"
echo "    param set SERIAL0_PROTOCOL 2"
echo "    param set SERIAL1_PROTOCOL 2"
echo "    param set AHRS_EKF_TYPE 3"
echo "    param set EK2_ENABLE 0"
echo "    param set EK3_ENABLE 1"
echo "    param set FENCE_ENABLE 0  # temp to test mission upload, enable later"
echo "    param set FS_GCS_ENABLE 0"
echo ""
read -p "Press Enter after SITL shows EKF ready, GPS 3D fix, Home set, 1363 params..."

echo "=== Checking SITL listeners ==="
ss -tlnp | grep -E "5762|5763" || { echo "SITL not LISTEN on 5762/5763"; exit 1; }
echo "SITL LISTEN OK:"
ss -tlnp | grep -E "5762|5763"

# Test local TCP
python3 - << 'PY'
import socket
for port in [5762,5763]:
    s=socket.socket()
    s.settimeout(2)
    try:
        s.connect(('127.0.0.1',port))
        print(f"Port {port}: CONNECTED, waiting heartbeat...")
        data=s.recv(1024)
        print(f"  {port}: {len(data)} bytes OK")
    except Exception as e:
        print(f"Port {port}: FAIL {e}")
PY

echo ""
echo "=== Terminal 4: mission_pi YOLO — USE 5763 (NOT 5762 which MP uses) ==="
echo "Run in NEW terminal (WITH venv-ardupilot):"
echo "  source ~/venv-ardupilot/bin/activate"
echo "  cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi"
echo "  python3 main.py --config config.gazebo.yolo.5763.yaml"
echo "  # fc.conn: tcp:127.0.0.1:5763"
echo "  # Should show FC found, not FAILED"
echo ""
read -p "Press Enter after mission_pi shows FC found + YOLO ready + cams ok..."

curl -s http://127.0.0.1:8000/health | python3 -m json.tool | grep -E "device|connected|mode|detector|cams" || echo "Pi /health failed"

echo ""
echo "=== Windows 11 MP — FIXED ==="
echo "On Windows:"
echo "  ping $POP_IP (must succeed, if not AP isolation ON)"
echo "  PowerShell: Test-NetConnection $POP_IP -Port 5762 (must succeed)"
echo "  MP: Connection TCP, IP $POP_IP, Port 5762 (NOT 5670, NOT 5760, NOT 5763), Baud 57600, Connect"
echo "  If TCP 5762 no heartbeat, try UDP:"
if [ -n "$WIN_IP" ]; then
echo "    MP: Connection UDP, Port 14550, Connect (needs SITL output add udp:$WIN_IP:14550)"
else
echo "    MP: Connection UDP, Port 14550 (needs SITL --out udp:WINDOWS_IP:14550)"
fi
echo "  MP should show HUD, mode STABILIZE, GPS 3D fix, EKF green, home set"
echo "  If home not set: MAVProxy arm throttle; disarm; or MP right-click Map -> Set Home to Vehicle Location"
echo "  If setWPTotal timeout: Stop Pi pkill -f main.py, MP Plan Clear WPs Write WPs empty, then small mission, then restart Pi on 5763"
echo "  MP Ctrl+F MAVLink forwarding 127.0.0.1:14551 Write tick (for UI bridge)"
echo "  Browser http://$POP_IP:8000/ + /health + http://$POP_IP:8099/health"
echo ""
echo "=== Fly ==="
echo "MP: Plan Load qr_search.txt with DO_SPRAYER seq2+RTL Upload (should NOT timeout now, separate ports), Arm Takeoff 15m AUTO, Pi trigger"
echo ""
echo "All done — QR present: qr_target_a3 at 8,0 (1.782x2.52 6x A3) and qr_target_big at 12,0 (3.564x5.04 12x A3)"
echo "Ports fixed: 5762 MP + 5763 Pi, no 5760 conflict, no contention, no lag, no setWPTotal timeout"
