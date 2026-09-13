# Gazebo closed loop — paste-and-go run sheet (this laptop, verified 2026-09-14)

Four terminals + MP + UI. Guts + troubleshooting live in `SIM_GUIDE.md`
(§9/§10); this file is just the commands, in order, with the exact
paths on this machine. Already persisted in `~/.bashrc`, so NOT repeated
below: `GZ_CONFIG_PATH` (line ~153), the `gz` → `QT_QPA_PLATFORM=xcb gz`
alias, `GZ_SIM_*` paths, QR textures.

## 0. Fresh day? Pull + preflight (any terminal, 30 s)

```bash
cd ~/arena_ai_uploads && git pull
ss -ltn | grep -E '5760|5762|8000|8099'   # want: EMPTY (nothing stale)
hostname -I                               # note the WiFi IP for MP + UI
```

Port still held from yesterday? `pkill -f gz_cam_bridge; pkill -f arducopter; pkill gz; pkill -f sim_vehicle` then re-check.

## 1. Terminal A — the world (~30 s to load)

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
gz sim -v4 -r sim/worlds/mission_world.sdf
# want: window with iris on ground + white QR panel ~8 m out on +X,
#        both /iris/.../image topics advertised. LEAVE RUNNING.
```

## 2. Terminal B — SITL on the iris

```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# want: "SERIAL0 on TCP port 5760" + "Waiting for connection". LEAVE RUNNING.
# (First run of the day rebuilds if ardupilot/ changed; else starts in seconds.)
```

## 3. Terminal C — camera bridge (NO mission_pi venv here)

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099
# want: both subscribes -> OK, frame counters climbing. LEAVE RUNNING.
```

Verify pixels (any spare terminal / browser):

```bash
curl -s http://127.0.0.1:8099/health   # want: both ages < 1 s, 1280x720
# browser: http://127.0.0.1:8099/ — front = horizon, bottom = ground.
```

## 4. Mission Planner (Windows, same WiFi)

Top-right dropdown → **TCP** → Connect → host = Linux WiFi IP, port
**5760** → OK. Flight Data comes alive; Terminal B prints the
`SERIAL1 on TCP port 5762` line — that is the cue for step 5.

Plan tab (before EVERY flight): TAKEOFF 15 m → WAYPOINT ~20 m out @
15 m → DO_SPRAYER → RTL → **Write WPs**.

## 5. Terminal D — check, then fly

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python main.py --check --config config.gazebo.yaml
# want: [fc] OK ... [cam] cam1: kind=url ... [cam] cam2: kind=url ...
python main.py --config config.gazebo.yaml
# want: BOOTSTRAP -> SNAPSHOT -> WAIT_TRIGGER. LEAVE RUNNING.
```

## 6. UI + flight

mission-ui → Pi link `http://<linux-ip>:8000` → connect → both CAM1 +
CAM2 tiles live. Then MP Flight Data → **Arm** → **Auto**. Watch
Terminal D (`trigger → TAKEOVER → GUIDED → TRACK → TRANSMIT`) and MP
Messages for `QR:MISSION-QR-001`.

Re-fly: MP re-arm + Auto again, restart `main.py` for a clean slate
(SITL + Gazebo + bridge stay up).

## 7. Shutdown

Ctrl-C in D, C, B, A (in that order). Stuck ports: see the `pkill` line
in §0.
