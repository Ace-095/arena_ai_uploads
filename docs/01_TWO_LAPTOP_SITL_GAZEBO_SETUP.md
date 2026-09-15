# 01 — Two-laptop SITL + Gazebo: from `git clone` to a working hunt

**Audience:** whoever brings up the bench. Two laptops, no drone, no radio, no Pi
needed for stages 0–2.

**What you end up with:** the Linux laptop running ArduPilot SITL (+ optionally
Gazebo with two simulated cameras) and `mission_pi`; the Windows laptop running
Mission Planner, the `mp_bridge.py` bridge and the browser UI; the payload
appearing in **MP's Messages tab** *and* in the **UI** at the same time.

**Scope, honestly:**

| Doc | Vehicle | Camera | Flight controller |
|---|---|---|---|
| **this one (01)** | SITL / Gazebo | webcam or 2 sim cameras | simulated (SITL) |
| [`02_PI5_AI_HAT_CAMERAS_BENCH.md`](02_PI5_AI_HAT_CAMERAS_BENCH.md) | still SITL | **real** Pi 5 + AI HAT + CSI cams | simulated (SITL, over the LAN) |
| [`03_REAL_FLIGHT_INTEGRATION.md`](03_REAL_FLIGHT_INTEGRATION.md) | real airframe | real Pi 5 cameras | real Pixhawk + telemetry radio |

Deeper references this doc does not repeat: `mission_pi/SIM_GUIDE.md` (why the
ladder looks like this + the full troubleshooting table),
`mission_pi/RUN_GAZEBO.md` (paste-and-go Gazebo run sheet),
`mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` (the Windows side in
detail, **both** offline-map systems, field-day checklist),
`mission-ui/docs/api-contract.md` (every endpoint/channel).

---

## 0. Topology, ports, and the stage map

```text
        LINUX LAPTOP (Pop!_OS / Ubuntu 22.04)              WINDOWS 11 LAPTOP
 ┌────────────────────────────────────────────┐   ┌──────────────────────────────┐
 │ [A] gz sim  sim/worlds/mission_world.sdf   │   │  Mission Planner  (the head: │
 │       │ /iris/front/image  /iris/bottom/…  │   │   arm, plan, DO_SPRAYER,     │
 │ [C] gz_cam_bridge.py :8099  ◄──────────────┘   │   takeoff, RTL)              │
 │       │ MJPEG                                  │      ▲            │          │
 │ [D] mission_pi :8000  ◄── REST/MJPEG/WS ────────┼──────┼────────────┤ MAVLink  │
 │       ▲                                        │      │  bridge    │ mirror   │
 │ [B] SITL  tcp 5760 ◄──────────────────────────────────┘ mp_bridge  ▼ udp      │
 │           tcp 5762 ──► mission_pi (FC link)    │      │   :8100  127.0.0.1:   │
 └────────────────────────────────────────────┘   │  browser UI  ◄── 14551       │
                                                   └──────────────────────────────┘
                              one Wi‑Fi LAN — both laptops on the same subnet
```

| What | Where | Value |
|---|---|---|
| MP → SITL (GCS port, serial0) | TCP | `<linux-ip>:5760` |
| mission_pi → SITL (serial1 spare) | TCP | `<linux-ip>:5762` (loopback when same box) |
| mission_pi HTTP/WS (the "Pi link") | TCP | `http://<linux-ip>:8000` |
| Gazebo camera bridge (MJPEG) | TCP | `127.0.0.1:8099` (Linux-local) |
| Bridge UI (Windows) | TCP | `http://127.0.0.1:8100` |
| MP MAVLink mirror → bridge | UDP | `127.0.0.1:14551`, **Write access ON** |
| Bridge GCS identity | — | sysid 254 (MP itself is 255); mission_pi is 51 |

Why two SITL ports: `--no-mavproxy` SITL listens on TCP **5760** for one GCS, so
Mission Planner takes 5760, and SITL's serial1 already listens on **5762** as a
spare MAVLink port — that is mission_pi's. No MAVProxy anywhere, and do **not**
move serial1 to 5763 (serial2 already lives there and SITL exits on the
collision).

**Stage map** — do them in order; each one isolates a different failure.

| Stage | What it proves | Time |
|---|---|---|
| 1 | Windows side installs, bridge selftest | 20 min |
| 2 | Linux side installs, SITL builds | 30–60 min |
| 3 | zero-hardware smoke (fake vehicle + file camera) | 5 min |
| 4 | SITL + MP + webcam bench (`config.laptop.yaml`) | 20 min |
| 5 | Gazebo closed loop, 2 sim cameras (`config.gazebo.yaml`) | 30 min |

---

## 1. Stage 1 — Windows laptop, one-time

1. **Mission Planner**: download `MissionPlanner-latest.msi` from
   <https://ardupilot.org/planner/>, install with defaults, accept the USB/SiK
   driver prompts.
2. **Python 3.8+** and **Git for Windows** (<https://git-scm.com/download/win>).
   The bridge is pure stdlib — no `pip` packages required (`pymavlink` is
   optional, only for `.bin` log parsing).
3. **Clone the repo** (the code is on `main`; see §1.3):

   ```powershell
   git clone https://github.com/Ace-095/arena_ai_uploads.git C:\mission\arena_ai_uploads
   cd C:\mission\arena_ai_uploads
   git log --oneline -1
   ```

   Only `mission-ui/` is needed on Windows, but cloning the whole repo is
   simpler and gives you the docs.
4. **Sanity-check the copy** (this must print `SELFTEST PASS`):

   ```powershell
   cd C:\mission\arena_ai_uploads\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
   py -3 -m py_compile bridge\mp_bridge.py bridge\mav_client.py bridge\_mavlink_v2.py
   py -3 bridge\mp_bridge.py --selftest
   ```

5. **Firewall / network**: Windows will prompt to allow Python on the private
   network — allow it. Chrome/Edge will ask for **Local Network Access** when
   the UI reaches `http://<linux-ip>:8000` — allow that too, or the Pi link
   looks dead while everything else works.
6. **Offline maps (do this 🌐 days before the venue)** — MP's own cache
   (Prefetch) *and* the UI's `.mbtiles` pack:
   `mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` §6 and §7. Not needed
   for the bench; needed on competition day.

### 1.3 Which branch has the code?

`main`. The historical branch `arena/01a0a01d-arena-ai-uploads` was merged into
`main` (PR #2, 2026-09-14) — a fresh clone of `main` contains
`mission_pi/main.py`, all four configs, the tools and the UI. Verify instead of
trusting:

```bash
git clone https://github.com/Ace-095/arena_ai_uploads.git && cd arena_ai_uploads
ls workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi/main.py   # must exist
git rev-parse --abbrev-ref HEAD                                        # -> main
```

(The root `README.md` still carries a stale "main has no `mission_pi/`" warning
from before the merge — ignore it, or re-read it after the docs PR.)

---

## 2. Stage 2 — Linux laptop, one-time

Target: Pop!_OS 22.04 or Ubuntu 22.04 (both Ubuntu-22.04-based, so the ArduPilot
prereqs script applies as-is). Needs ~10 GB free (5 GB ArduPilot, ~5 GB Gazebo)
and, for stage 5, a real GPU or a strong iGPU.

```bash
# 2.1 system libs
sudo apt update
sudo apt install -y git python3-venv python3-pip libgl1 libzbar0 v4l-utils curl

# 2.2 the repo
git clone https://github.com/Ace-095/arena_ai_uploads.git ~/arena_ai_uploads
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi

# 2.3 python env (a few minutes)
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt            # pymavlink numpy fastapi uvicorn[standard] websockets
pip install opencv-python pyzbar pillow qrcode   # laptop vision stack

# 2.4 ArduPilot SITL (~30-60 min, mostly unattended)
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile                               # reload PATH, or open a fresh terminal
git submodule update --init --recursive
./waf configure --board sitl
./waf copter                               # first build is the long one
```

Two details that bite people:

* `libzbar0` **before** `pip install pyzbar` — otherwise pyzbar installs and
  then fails at *import*. If it is missing the decoder still works through
  OpenCV's `QRCodeDetector`, slower and less tolerant (a real
  `[vision] pyzbar MISSING` line is survivable, not fatal).
* `libgl1` before `cv2` — otherwise `import cv2` dies on `libGL.so.1`.
* Webcam permission: `ls /dev/video*` must list a node; if permission is denied,
  `sudo usermod -aG video $USER` then log out/in.

Note the Linux laptop's LAN IP once and write it on the lid — everything on the
Windows side needs it:

```bash
hostname -I          # e.g. 192.168.1.42
```

### 2.5 Gazebo Harmonic + the ArduPilot plugin (stage 5 only)

Follow the binary install at <https://gazebosim.org/docs/harmonic/install>
(Harmonic is the recommended release for Ubuntu 22.04), verify `gz sim` opens,
then build the ArduPilot plugin
([plugin README](https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md)):

```bash
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

Then the camera-bridge bindings, our models on the resource path, and the two QR
panel textures (generated, gitignored — regenerate after every fresh clone):

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python tools/make_qr_panel.py --text MISSION-QR-001 \
    --out sim/models/qr_panel_a3/materials/textures/qr.png
python tools/make_qr_panel.py --text MISSION-QR-001 \
    --out sim/models/qr_panel_big/materials/textures/qr.png
echo "export GZ_SIM_RESOURCE_PATH=$PWD/sim/models:\${GZ_SIM_RESOURCE_PATH}" >> ~/.bashrc
source ~/.bashrc
```

> **A missing texture renders the panel blank white/black — detection can then
> never fire.** `ls -la sim/models/qr_panel_a3/materials/textures/qr.png`
> should show ~100 KB+.
>
> ROS 2 installed too? Its `setup.bash` overwrites `GZ_CONFIG_PATH` with
> vendor-only paths, which hides `gz sim` from the `gz` CLI (symptom: `gz help`
> lists no `sim`). Append **after** the ROS source line:
> `export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH`, fresh terminal,
> `gz sim --versions` → 8.x.

### 2.6 Firewall (only if `ufw` is active)

```bash
sudo ufw status
sudo ufw allow 5760/tcp    # MP -> SITL
sudo ufw allow 5762/tcp    # mission_pi -> SITL (needed when the Pi is a separate box, doc 02)
sudo ufw allow 8000/tcp    # browser UI -> mission_pi
```

SITL binds all interfaces, so the Windows box can reach 5760 across the LAN —
the usual cause of "connection failed" is the IP, not the bind.

---

## 3. Stage 3 — zero-hardware smoke test (5 min, do this first every session)

No SITL, no camera, no Mission Planner: a fake MAVLink vehicle stands in for
SITL (TCP 5760/5762 + a UDP mirror on 14551, exactly like SITL + MP's mirror),
a folder of PNG/JPEG frames stands in for the camera. If this is green, every
piece of *software* in the chain is fine and any later failure is environment
or hardware.

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python tools/make_demo_frames.py --out demo_frames --jpg

# terminal 1 — fake vehicle
python tools/fake_sitl.py --udp 127.0.0.1:14551 --auto --loop --no-keys

# terminal 2 — the Pi link
python main.py --config config.demo.yaml

# terminal 3 — UI + bridge
cd ../mission-ui && python bridge/mp_bridge.py --port 8100
```

Open `http://127.0.0.1:8100`. With `--auto --loop` the fake vehicle flies the
mission by itself, "finds" the panel and restarts, so within about a minute the
UI should show `MISSION-QR-001` with nobody touching anything.

**Verified output** (this exact run, 2026-09-15, Python 3.11 — copy the shape,
not the values):

```text
main INFO: bring-up detector: ok (classical)
cameras INFO: assigned cam2 -> file 'demo_frames' (bottom)
main INFO: bring-up streams: ok (10fps 960w q72 x1)
fc_link INFO: FC found: tcp:127.0.0.1:5762 @ net (sysid=1 compid=1)
fc_link INFO: FC watchdog up (dead after 10s, retry every 3s)
mission INFO: phase -> SNAPSHOT (reading home / fence / plan)
mission INFO: search area: 4 fence verts, ~40 x 40 m
mission INFO: sprayer seqs: [2]
mission INFO: speed snapshot: WP_SPD = 3.0
mission INFO: phase -> WAIT_TRIGGER (waiting for sprayer seqs [2])
mission INFO: trigger: sprayer STATUSTEXT @seq 1
mission INFO: phase -> TAKEOVER (commanding GUIDED)
fc_link INFO: FC mode: AUTO -> GUIDED (sysid=1)
mission INFO: phase -> SWEEP_YAW (4 x 90 deg stepped sweep)
mission INFO: phase -> TRANSMIT (relaying 'MISSION-QR-001')
qr_relay INFO: QR relay done: 'QR:MISSION-QR-001' x4 over MAVLink + ws event
mission INFO: WP_SPD restored to 3.0
mission INFO: mission PAYLOAD 'MISSION-QR-001' — post_action RTL
mission INFO: phase -> DONE (payload=MISSION-QR-001)
```

Four cross-checks worth running while it is up:

```bash
curl -s localhost:8000/health   | python3 -m json.tool   # "link": true, cams, fc detail
curl -s localhost:8000/api/bringup                        # stage: "ready", all subsystems "ok"
curl -s localhost:8100/api/mp/state                       # qr.payload latched, source "mavlink"
python tools/link_doctor.py --target http://127.0.0.1:8000   # 8 checks, 0 FAIL
```

Expected: `/api/bringup` → `"stage": "ready"` with
`server/detector/cameras/streams/fc` all `ok`; `/api/mp/state` →
`"qr": {"payload": "MISSION-QR-001", "source": "mavlink"}` — i.e. the payload
travelled vehicle → `STATUSTEXT` → bridge, which is the same route the real
radio uses; `link_doctor` → `8 checks, 0 FAIL, 0 WARN`.

The fake vehicle is also how the FC watchdog is exercised: kill terminal 1,
watch `/health` go `"link": false` with `link_lost` in the UI log, restart it
and watch `link_restored` + `"reconnects": 1` — no `main.py` restart needed.

> Difference from real SITL worth knowing: `fake_sitl.py` binds **both** 5760
> and 5762 immediately. Real SITL prints `SERIAL1 on TCP port 5762` only
> **after** the first client connects to 5760 — so with real SITL, connect MP
> first, mission_pi second.

---

## 4. Stage 4 — SITL + Mission Planner + laptop webcam (the bench)

The webcam plays the drone camera. Point it at a printed A3 QR; the sim flies at
15 m while the webcam sits on a desk — **positions and pixels are decoupled on
purpose**. This stage proves the software loop; the air proves the optics.

### 4.1 Print the target

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python tools/make_qr_panel.py --text MISSION-QR-001 --out qr_panel_a3.png
# print at 100% on A3 (297 x 420 mm portrait). No A3 printer? Tile it over
# 2x2 A4, or show it big on a monitor (mind the glare).
```

The relay you must later see in MP is literally `QR:MISSION-QR-001`.

### 4.2 Terminal 1 (Linux) — SITL

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-mavproxy
# wait for: "SERIAL0 on TCP port 5760" ... "Waiting for connection"
```

### 4.3 Windows — MP connects

MP top-right dropdown → **TCP** → Connect → host `<linux-ip>` (from
`hostname -I`), port **5760**. Flight Data comes alive; home is somewhere in
Australia by default — normal. The moment MP connects, terminal 1 prints
`SERIAL1 on TCP port 5762` — that is the cue for terminal 2.

### 4.4 Terminal 2 (Linux) — check, then run

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python main.py --check --config config.laptop.yaml
# want: [fc] OK: tcp:127.0.0.1:5762 (sysid=1)
#       [cam] cam2: kind=usb model=usb facing=bottom size=(1280, 720)
#       [net] ws impl: websockets
python main.py --config config.laptop.yaml
```

`config.laptop.yaml` is the single-webcam bench config (`kind: usb`,
`device: 0`, detector `classical`, `fc.conn: tcp:127.0.0.1:5762`). One camera ⇒
one UI tile (CAM2); CAM1 reading "frame fetch failed" is expected here.

### 4.5 Windows — bridge + UI

```powershell
cd C:\mission\arena_ai_uploads\workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission-ui
py -3 bridge\mp_bridge.py --port 8100        # leave running
```

Open `http://127.0.0.1:8100`, then in MP: **Ctrl+F → Mavlink → UDP Client →
tick *Write access* → Connect → `127.0.0.1` → `14551`**. The UI's MAV lamp goes
green within ~3 s and the drone marker appears on the map. (Write access is
what lets the UI's *APPLY FENCE* and *RTL/LAND* buttons reach the vehicle; the
mirror dialog must stay open, and changing mirror settings usually needs an MP
restart — see `mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` §3.)

In the UI's **Pi link** box paste `http://<linux-ip>:8000` → Connect (or
**Probe**, which prints the actual reason on failure).

### 4.6 Fly it

MP **Plan** tab: `TAKEOFF` 15 m → `WAYPOINT` ~20 m out @ 15 m → **`DO_SPRAYER`**
→ `RTL`, then **Write WPs**. Optionally draw a ~50×50 m polygon fence and
*Write Fence* (exercises the fence snapshot + grid clip; skip it to exercise the
home-box fallback instead — both are valid tests).

Then MP **Flight Data → Arm → Auto**, with the webcam looking at the print.
Expected in terminal 2: `trigger: mission seq N >= sprayer N` → `TAKEOVER …
GUIDED` → `SWEEP_YAW` → `cue … TRACK` → `PAYLOAD … MISSION-QR-001`, and
**`QR:MISSION-QR-001` in MP's Messages tab** — that line is the end-to-end
proof. Approach stairs then run against the sim; after LAND the vehicle stays
where it descended (that is the test working; RTL-after-land may refuse while
disarmed — just re-arm for the next run).

Re-run: MP re-arm + Auto; restart `main.py` for a clean slate (SITL stays up).

---

## 5. Stage 5 — Gazebo closed loop (sim physics + two sim cameras)

Same as stage 4 except the iris flies in Gazebo, the laptop webcam is replaced
by two simulated cameras (front + bottom, fixed mounts on `iris_dualcam`), and
the QR target lives in the world. Four terminals + MP + UI.

### 5.0 Preflight (30 s, every fresh day)

```bash
cd ~/arena_ai_uploads && git pull
cd workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
ls -la sim/models/qr_panel_a3/materials/textures/qr.png   # must EXIST (else: blank panel)
ls -la sim/models/qr_panel_big/materials/textures/qr.png  # must EXIST
ss -ltn | grep -E '5760|5762|8000|8099'                   # want: EMPTY (nothing stale)
hostname -I                                               # note the LAN IP
# stale ports from yesterday:
# pkill -f gz_cam_bridge; pkill -f arducopter; pkill gz; pkill -f sim_vehicle
```

### 5.1 Terminal A — the world

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
gz sim -v4 -r sim/worlds/mission_world.sdf
# want: iris on the ground at the origin, white QR panel ~8 m away on +X.
# No window / GUI segfault = almost always Wayland: prefix with
#   QT_QPA_PLATFORM=xcb   (full ladder in SIM_GUIDE.md §9). Headless -s is the
#   last resort — the mission needs no window, and -s also halves render load.
```

### 5.2 Terminal B — SITL on the iris

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
# note --model JSON + frame gazebo-iris; same two-port trick as §4.2
```

### 5.3 Terminal C — the camera bridge (**system** python3, *not* the venv)

The gz-transport python bindings are installed system-wide, so activating the
venv here breaks the import:

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
python3 tools/gz_cam_bridge.py --port 8099
# want: subscribe /iris/front/image -> OK, /iris/bottom/image -> OK, then
#       "front: N frames age=0.0x" lines
curl -s http://127.0.0.1:8099/health     # both ages < 1 s, sizes [2560, 1440]
```

Check the pictures **before** flying: browser → `http://127.0.0.1:8099/` — front
pane = horizon, bottom pane = ground (and the panel after takeoff).

### 5.4 Terminal D — mission_pi on the sim cameras

```bash
cd ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi
source .venv/bin/activate
python main.py --check --config config.gazebo.yaml
# want: [fc] OK ... [cam] cam1: kind=url ... [cam] cam2: kind=url ...
python main.py --config config.gazebo.yaml
```

Then MP (connect 5760, write the plan, Arm + Auto) and the UI (bridge up, paste
the Pi link) exactly as in §4.3/§4.5. The UI probe now finds **two** cameras, so
both tiles light up. From the trigger on the loop is fully closed: sim pixels
steer sim motion.

### 5.5 The sim target, and why the resolution matters

`mission_world.sdf` already includes the target at (8, 0) — inside the 40×40 m
home search box the mission flies without a fence. Default is true A3
(`qr_panel_a3`, 297×420 mm) with sim cameras at **2560×1440**: from 15 m the
code is ~38 ideal px (GSD ~7.9 mm/px @ 68°), the same detectability class as the
real Pi Cam 3's ~70 px. Pixels grow down the stair: ~47 @ 12 m, ~63 @ 9 m,
~81 @ 7 m.

* `config.gazebo.yaml` sets `size: [2560, 1440]` — it **must** equal the sim
  render, because mission_pi resizes every grabbed frame to `size`; 720p here
  would silently throw away the pixels the A3 panel needs.
* Easy mode if the laptop can't hold 1440p: swap the commented `<include>` to
  `qr_panel_big` (2× A3), or drop the sensor `<update_rate>` to 10 Hz first
  (halves localhost transport for free).
* Sim lens HFOV is 68° in **both** `sim/models/iris_dualcam/model.sdf`
  (`<horizontal_fov>` 1.1868 rad) and `config.gazebo.yaml` — change them
  together or the approach geometry lies.

---

## 6. Acceptance checklist (all green ⇒ the bench is done)

| # | Check | Command / where | Expected |
|---|---|---|---|
| 1 | Repo present on both laptops | `ls workspace-*/mission_pi/main.py` | exists |
| 2 | Bridge healthy | `py -3 bridge\mp_bridge.py --selftest` | `SELFTEST PASS` |
| 3 | Python deps | `python main.py --check …` | `[vision] cv2 OK … pymavlink OK` |
| 4 | **WebSocket impl present** | same | `[net] ws impl: websockets` (NOT `NONE`) |
| 5 | SITL up | `ss -ltn \| grep 5760` | LISTEN on `0.0.0.0:5760` |
| 6 | serial1 appeared | SITL console | `SERIAL1 on TCP port 5762` **after** MP joins |
| 7 | MP connected | MP HUD | horizon alive, mode STABILIZE/AUTO |
| 8 | Pi link green | UI Pi box / **Probe** | `link: true`, `ready: true` |
| 9 | Bridge MAV lamp | UI | green; `/api/mp/mavlink` → `connected: true`, `bad_frames: 0` |
| 10 | Cameras | `curl -s localhost:8000/api/cameras` | both cams `running: true`, `age_s` < 1 |
| 11 | Plan + trigger | terminal D log | `sprayer seqs: [N]` non-empty |
| 12 | Takeover | MP mode window | AUTO → **GUIDED** when the sprayer item runs |
| 13 | Payload in MP | MP Messages tab | `QR:MISSION-QR-001` |
| 14 | Payload in UI | UI QR panel | same payload, first-one-wins latch |
| 15 | End-to-end doctor | `python tools/link_doctor.py --target http://<linux-ip>:8000` (run **from Windows**) | `8 checks, 0 FAIL` |
| 16 | Recovery | kill SITL, restart it | `link_lost` → `link_restored`, `reconnects: 1`, no `main.py` restart |

Rows 4, 8 and 15 are the ones that used to fail silently — keep them.

---

## 7. Troubleshooting (top of the ladder)

| Symptom | Cause → fix |
|---|---|
| Fresh clone has no `mission_pi/` | You cloned something old. `git pull`; `main` has it (§1.3) |
| `[net] ws impl: NONE` | `pip install "uvicorn[standard]"` (or `websockets`). Without one, uvicorn serves HTTP but **silently 404s every WS route** — `/health` looks fine while the UI's Pi lamp stays red. The UI then degrades to polling `/api/fsm/status` + `/api/qr/status` every 2 s (`polling`) |
| UI says "Pi link: down" but `/health` opens in a browser | Same cause as above, or Chrome blocked Local Network Access |
| Nothing listens on `:8000` | Old code raised before binding. Current code binds the server **first** and runs bring-up on its own thread — `GET /health` + `GET /api/bringup` answer while the FC is still retrying and say which subsystem failed |
| UI on Windows can't reach `http://<linux-ip>:8000` | Same-subnet check: `hostname -I` on Linux (not 127.0.0.1), `Test-NetConnection <ip> -Port 8000` from Windows, `sudo ufw allow 8000/tcp`; then run `link_doctor.py --target …` **from Windows** (stdlib only) |
| MP "connection failed" on 5760 | SITL console must say `Waiting for connection`; `ss -ltn \| grep 5760`; IP and port in **separate** MP fields |
| No `SERIAL1 on TCP port 5762` | It appears only after the first 5760 client — connect MP first. Fallback: `-A "--serial1=udpclient:127.0.0.1:14555"` + `fc.conn: "udpin:0.0.0.0:14555"` |
| Bridge `connected` but no telemetry / no QR | `GET /api/mp/mavlink`: `bad_frames > 0` = not MAVLink on that port; `unknown_ids > 0` is **normal** (SITL streams VFR_HUD); `rx_v1 > 0` = MAVLink 1 mirror, parsed fine |
| QR in MP Messages but not the UI | The bridge latches only `QR:<payload>` (colon required, ≤32 chars of `[A-Za-z0-9._+-]`) — prose must not latch. The Pi WS route carries the full text regardless |
| `[cam] none assigned` | `ls /dev/video*`; close Cheese/Zoom; `device: 1`; video group (§2); `--verbose` |
| Gazebo GUI segfault / no window | Wayland: `QT_QPA_PLATFORM=xcb gz sim …`; full ladder in `SIM_GUIDE.md` §9 |
| bridge `subscribe FAIL` / dark panes | Start order A → C; `gz topic -l \| grep -i image` must list both topics; dark + fresh ages = camera inside the body mesh or headless without a GL context |
| Sim slower than realtime | Normal with 2 cameras on an iGPU: `-s`, `<update_rate>10</update_rate>`. Mission timeouts are wall-clock, so slowness only stretches the run |
| QR never found in Gazebo | Panel texture missing (§5.0)? Panel inside the flown box? True A3 + classical at 15 m is marginal — fly one pass at `sweep_alt_m: 10` to prove detection, then diagnose altitude |

Everything else (including the full Wayland/GLX ladder, PreArm chatter and the
MP mirror quirks) is tabulated in `mission_pi/SIM_GUIDE.md` §9 and
`mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` §9.

---

## 8. Shutdown, cleanup, and what's next

* Stop in order **D → C → B → A** (Ctrl-C). Stuck ports: the `pkill` line in §5.0.
* `demo_frames/`, `snaps/`, `logs/`, `config.yaml` and the generated panel
  textures are gitignored — safe to delete, regenerate with the commands above.
* Tests, when you touch code (all stdlib-only except the vision ones):

  ```bash
  cd workspace-*/mission_pi && for t in tests/*.py; do python3 "$t"; done
  cd workspace-*/mission-ui && for t in tests/*.py; do python3 "$t"; done
  cd workspace-*/mission-ui && python3 bridge/mp_bridge.py --selftest
  ```

  Reference result on a clean tree (2026-09-15, Python 3.11, deps installed):
  **10/10 `mission_pi` suites and 7/7 `mission-ui` suites pass**, plus
  `SELFTEST PASS` and `node --check` clean on all five UI scripts.

**Next:** stage 5 green ⇒ move the *real* cameras and the AI HAT onto the Pi
while the vehicle is still simulated →
[`02_PI5_AI_HAT_CAMERAS_BENCH.md`](02_PI5_AI_HAT_CAMERAS_BENCH.md).
