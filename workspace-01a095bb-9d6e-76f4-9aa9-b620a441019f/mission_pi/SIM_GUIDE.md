# mission_pi laptop bench guide — Pop!_OS + SITL + webcam (+ Gazebo later)

Run the whole stack on your Linux laptop: ArduPilot SITL plays the
Pixhawk, Mission Planner on the Windows laptop plays the GCS, your
laptop webcam plays the drone camera (pointed at a printed A3 QR),
and `mission_pi` runs exactly as it will on the Pi 5. Gazebo swaps in
later as the physics/visual backend with zero `mission_pi` changes.

## 0. What this proves (read first — honest scope)

Proved on the bench:

* FC link, boot snapshot (home / fence / plan / sprayer scan).
* DO_SPRAYER trigger out of AUTO, GUIDED takeover + verify.
* YOLO/classical detect workers, QR decode, 3/6 s consensus, QR relay
  over STATUSTEXT (you will SEE it in Mission Planner) + websocket.
* Approach stairs, grid coverage, failsafes, UI snapshot polling.

NOT proved on the bench:

* True 15 m optics (your webcam sits on a desk; range geometry is only
  exercised for real in the air). The bench validates the *software
  loop*; the air validates the *pixels*.
* Hailo timing (laptop runs the classical detector — which is the
  right tool here anyway: stock YOLO knows no QR class).

## 1. Topology

```
Windows laptop (Mission Planner)  <--WiFi/LAN-->  Linux laptop (Pop!_OS)
  TCP <linux-ip>:5760  ──────────────►  SITL serial0 (GCS port)
                                         SITL serial1 ──► tcp:127.0.0.1:5762
                                                            ▲
                                         mission_pi ────────┘ (FC link)
                                         mission_pi ◄── /dev/video0 (webcam
                                                          pointed at A3 print)
                                         UI/health: http://<linux-ip>:8000
```

Why two SITL ports: `--no-mavproxy` SITL listens on TCP **5760** for
one GCS ([docs](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)),
so Mission Planner takes 5760 — and SITL's serial1 already listens on
TCP **5762** as a spare MAVLink port, which is where `mission_pi`
connects. No port overrides needed (do NOT move serial1 to 5763:
serial2 already lives there and SITL exits on the collision).
No MAVProxy anywhere in this guide.

Find your Linux LAN IP once: `hostname -I` (e.g. `192.168.1.42` —
both laptops must be on the same WiFi).

## 2. One-time laptop setup

```bash
# system libs: video/file dialogs, OpenGL for cv2, zbar for pyzbar
sudo apt update
sudo apt install -y git python3-venv libgl1 libzbar0 v4l-utils

# repo (same repo, your branch)
git clone <your-repo-url> ~/arena_ai_uploads   # or: git pull, if cloned
cd ~/arena_ai_uploads/workspace-*/mission_pi    # Tab-complete the middle dir

# venv + python deps (takes a few minutes)
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt   # pymavlink numpy fastapi uvicorn ...
pip install opencv-python pyzbar pillow qrcode   # laptop vision stack
```

Notes:

* `requirements.txt` is the core set; the extra line adds the laptop
  vision stack (Pi gets its opencv via apt instead — see README).
* If `pip install pyzbar` later fails at *import*, you missed
  `libzbar0` above. If `cv2` fails at import with `libGL.so`, you
  missed `libgl1`. See §9.
* Webcam permission: `ls /dev/video*` must exist; if permission is
  denied, `sudo usermod -aG video $USER`, then log out/in.

## 3. Webcam check (2 min)

```bash
cd mission_pi && source .venv/bin/activate
ls /dev/video*                       # expect /dev/video0 (maybe +video1)
python tools/cam_probe.py --snap-dir snaps   # legacy CSI probe: expect "none"
```

`cam_probe` with no config probes Pi cameras only, so on the laptop
use the config-driven check instead (SITL not needed for cameras, but
`--check` also probes the FC — run it after §6 the first time, or
ignore the `[fc] FAIL` line for now):

```bash
cp config.laptop.yaml config.yaml    # or pass --config every time
python main.py --check --config config.laptop.yaml
# want: [cam] cam2: kind=usb model=usb facing=bottom size=(1280, 720)
```

If `model=usb` didn't appear: wrong `device:` index (try `1`), or the
cam is held by another app (close Cheese/Zoom/browser), or perms §2.

Live-view the bench cam any time while the stack runs (the UI shows a
single CAM2 tile — one camera, one tile):

```
http://127.0.0.1:8000/api/camera/frame/cam2
```

Tune the webcam (contrast etc.) with the UI's CAMERAS sliders — they POST
the contract tuning and the 0..2 scalers work relative to your driver's
own defaults (1.0 == leave it alone). Uncheck `adaptive` to flip toward
manual exposure so the exposure/gain sliders bite. Raw driver props for
bench/SSH debugging:

```bash
curl http://127.0.0.1:8000/api/camera/cam2/controls
curl -X POST http://127.0.0.1:8000/api/camera/cam2/controls \
  -H "Content-Type: application/json" -d '{"contrast":40,"saturation":64}'
```

Bench trick: during SEARCH, hold the printed A3 panel (or a phone showing
a big QR) up to the webcam — the detector decodes it like a real sighting
and you get to watch TRACK → APPROACH → TRANSMIT end to end.

## 4. Print the A3 QR panel (5 min)

```bash
python tools/make_qr_panel.py --text MISSION-QR-001 --out qr_panel_a3.png
# print qr_panel_a3.png at 100% on A3 (297 x 420 mm portrait), lay it FLAT
# on the floor. No A3 printer? Tile it on 2x2 A4 sheets, or show it big
# on a monitor/tablet in a pinch (mind the glare).
```

The relay you must see later in Mission Planner is literally:

```
QR:MISSION-QR-001
```

## 5. ArduPilot SITL one-time install (~30-60 min, unattended-ish)

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile                       # reload PATH (or open a fresh terminal)
git submodule update --init --recursive
./waf configure --board sitl       # first build takes a while; one time only
./waf copter
```

Pop!_OS 22.04 is Ubuntu-22.04-based, so the Ubuntu prereqs script
applies as-is. If `waf` fails on missing bits, re-run the prereqs
script and retry (see §9).

## 6. Run day, part 1 — SITL + Mission Planner link

Terminal 1 (Linux):

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-mavproxy
# wait for: "SERIAL0 on TCP port 5760" ... "Waiting for connection"
```

Normal at this point: **no SERIAL1 line yet**. Serial0 blocks the boot
at "Waiting for connection" until the first client connects to 5760 —
only then does `SERIAL1 on TCP port 5762` (mission_pi's port) appear.
So connect MP first, mission_pi second.`

On Windows Mission Planner (same WiFi):

1. Top-right connection dropdown → select **TCP** → Connect.
2. Host = your Linux IP (`hostname -I`), port **5760** → OK.
3. Flight Data appears, artificial horizon moves, mode STABILIZE,
   home set (somewhere in Australia by default — normal).

If MP won't connect: SITL console must say "Waiting for connection";
check the IP. If `sudo ufw status` says active, open our three ports:

```bash
sudo ufw allow 5760/tcp   # MP -> SITL
sudo ufw allow 5762/tcp   # mission_pi -> SITL
sudo ufw allow 8000/tcp   # UI browser -> mission_pi
```

## 7. Run day, part 2 — the test mission in MP

Plan tab (this mirrors the real competition flow):

1. Right-click map → Planner → draw nothing yet; use "Wp" rows or
   right-click → Add WP Below: build:
   * `TAKEOFF` — alt 15 m
   * `WAYPOINT` — ~20 m from home, alt 15 m
   * `DO_SPRAYER` — (sprayer on; the trigger!)
   * `RTL` — safety tail
2. **Write WPs** to the vehicle (SITL keeps them in RAM).
3. (Optional but recommended) Fence → Polygon → draw ~50×50 m →
   Write Fence — exercises the fence snapshot + grid clip path. Skip
   it to exercise the home-box fallback instead (both are valid tests).

Terminal 2 (Linux) — prove `mission_pi` sees the same vehicle:

```bash
cd mission_pi && source .venv/bin/activate
python main.py --check --config config.laptop.yaml
# want: [fc] OK: tcp:127.0.0.1:5762 ... [cam] cam2: kind=usb ...
```

## 8. Run day, part 3 — fly the loop

Terminal 2 (Linux), SITL still up, MP still connected:

```bash
python main.py --config config.laptop.yaml
# FC link: tcp:127.0.0.1:5762 ... detector: classical
# BOOTSTRAP -> SNAPSHOT (home/fence/plan, sprayer seqs: [3]) -> WAIT_TRIGGER
```

Now:

1. Point the webcam at the printed A3 (prop the laptop so the panel
   fills a good chunk of frame; check
   `http://127.0.0.1:8000/api/camera/frame/cam2`).
2. In MP Flight Data → Actions → **Arm**, then mode **Auto**
   (or Arm + Auto from the mode list). The sim copter takes off,
   climbs to 15 m, flies the waypoint.
3. Watch Terminal 2. Expected sequence:
   * `mission current: 1 ... 2 ...` then
     `trigger: mission seq 3 >= sprayer 3`
   * `TAKEOVER ... GUIDED` (MP mode flips to GUIDED),
     climb/hold at 15 m
   * `SWEEP_YAW` — the sim yaws; the webcam already sees the print,
     so expect quickly: `cue ... -> TRACK`, then
     `PAYLOAD ... MISSION-QR-001`
   * **In MP Messages you must see `QR:MISSION-QR-001`** — that is
     the relay working end to end. The UI websocket gets it too.
   * Approach stairs + descend run against the sim (watch MP map +
     altitude), then LAND and post-action.
4. Re-run: MP Actions → re-arm, re-select Auto (SITL restarts the
   mission from the top). Restart `main.py` for a clean slate.

   In mission-ui (Pi link `http://<linux-ip>:8000`), the bench webcam
   shows in the **CAM2** tile; CAM1 reads "frame fetch failed" — normal,
   the laptop has one camera. If CAM2 is blank too, T2's process exited
   or the Pi link is wrong (the process now stays alive after the
   mission ends, so a blank CAM2 with T2 running means link trouble).

Two honest bench artefacts to expect:

* The sim flies at 15 m while your webcam sits on a desk — positions
  and pixels are decoupled on purpose. The bench proves the loop, not
  the optics.
* After the sim LAND, the vehicle is down wherever it descended.
  That's the test working. RTL-after-land may refuse (disarmed) —
  just re-arm for the next run.

Kill everything with Ctrl-C (Terminal 1 SITL, Terminal 2 mission).

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `waf`/prereqs fail | Re-run `install-prereqs-ubuntu.sh -y`, fresh terminal, retry; needs ~5 GB disk |
| MP "connection failed" on 5760 | SITL listens on all interfaces (verified in source) — it's network/typing: Linux `ss -ltn \| grep 5760` must show LISTEN; `hostname -I` for the WiFi IP (not 127.0.0.1); Windows `ping <ip>` then PowerShell `Test-NetConnection <ip> -Port 5760`; `sudo ufw allow 5760/tcp` if ufw is active; in MP put IP and port in SEPARATE fields |
| SITL console lacks "SERIAL1 on TCP port 5762" | It appears only AFTER the first 5760 client connects — connect MP first. Fallback: `-A "--serial1=udpclient:127.0.0.1:14555"` + `fc.conn: "udpin:0.0.0.0:14555"` |
| `[fc] FAIL: no heartbeat` on 5762 | SITL up? (Terminal 1). Port shared/IPC clash: nothing else may hold 5762 |
| `[cam] none assigned` | `ls /dev/video*`; close apps holding the cam; `device: 1`; video-group perms (§2); `--verbose` |
| `cv2` import error (libGL) | `sudo apt install libgl1` |
| `pyzbar` import error | `sudo apt install libzbar0` (decode still works via OpenCV alone, slower) |
| Trigger never fires | Plan actually **written**? `DO_SPRAYER` present? Read the SNAPSHOT lines (`plan N items, sprayer seqs: [...]`); force it: `curl -X POST http://127.0.0.1:8000/api/mission/takeover` |
| GUIDED refused | Check MP Messages for the refusal reason; SITL GPS is fine — usually a pre-arm state; re-arm |
| QR never decodes | Bigger in frame (move closer), more light, hold still; snap + `python tools/qr_bench.py snaps/cam2.jpg` to iterate offline |
| PreArm failures in SITL | Read MP Messages; common: wait 10 s for EKF; SITL compass/GPS are pre-baked, no cal needed |
| gz sim can't find `iris_dualcam` | `GZ_SIM_RESOURCE_PATH` missing `sim/models` (§10.1 export + `source ~/.bashrc` + NEW terminal); `model://iris_with_standoffs` missing = ardupilot_gazebo models path absent |
| bridge `subscribe FAIL` / no frames | Gazebo not up yet (start order: A → C); else `gz topic -l \| grep -i image` must list both `/iris/*` topics — if absent the Sensors plugin didn't load (ogre2 ok?) |
| bridge runs but panes dark | `/health` ages climbing = Gazebo rendering stalled (GPU/driver — try `gz sim -s` headless, still renders offscreen); ages fresh but black frames = camera inside the body mesh (mount pose) |
| mission_pi cam1/cam2 503 on gazebo config | Browser-test the MJPEG URLs first (dark = bridge/Gazebo side; live = cv2 ffmpeg http reader — reinstall `opencv-python`) |
| QR never found in Gazebo | Confirm the panel is IN the flown box (world pose vs home box/fence); confirm size story §10.3 (A3 + classical = expected fail); fly lower once (`sweep_alt_m: 10`) to prove detection, then diagnose altitude |
| Sim runs slower than realtime | Normal on iGPU with 2 cameras: headless `-s`, `<update_rate>10</update_rate>`, close the Gazebo GUI render loop; mission timeouts are wall-clock so slowness only stretches the run |

## 10. Gazebo phase (closed loop — sim physics + sim cameras + UI)

Needs a real GPU or a strong iGPU (ogre2) and ~5 GB more disk.
Everything in §1-§9 stays identical in spirit; the differences:

- the iris flies in Gazebo (`sim/worlds/mission_world.sdf`) instead of
  bare SITL, driven by the same MP mission + sprayer trigger;
- TWO sim cameras (front + bottom, fixed mounts on `iris_dualcam`)
  replace the laptop webcam, served to `mission_pi` as MJPEG by
  `tools/gz_cam_bridge.py` — the UI shows both tiles, like two webcams;
- the QR target lives in the world (`qr_panel_big`, 2x-A3 for first
  runs so the classical detector can actually decode it).

Data path: Gazebo sensors → gz-transport topics → bridge :8099 →
`mission_pi` (`config.gazebo.yaml`) → UI tiles + search FSM. No ROS,
no GStreamer, no model surgery by hand — the model/world files ship in
`sim/`.

### 10.1 Install Gazebo Harmonic + the ArduPilot plugin (one time)

Follow the binary install at
[ gazebosim.org/docs/harmonic/install ](https://gazebosim.org/docs/harmonic/install)
(Ubuntu 22.04 / Pop!_OS 22.04 → Harmonic is the recommended release),
verify `gz sim` opens, then ([plugin README](https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md)):

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

Camera-bridge deps (one time) + our models on the resource path +
both QR textures (same PNG content, two model homes):

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil
cd mission_pi
source .venv/bin/activate && pip install qrcode pillow   # texture gen only
python tools/make_qr_panel.py --text MISSION-QR-001 \
    --out sim/models/qr_panel_big/materials/textures/qr.png
python tools/make_qr_panel.py --text MISSION-QR-001 \
    --out sim/models/qr_panel_a3/materials/textures/qr.png
echo "export GZ_SIM_RESOURCE_PATH=$PWD/sim/models:\${GZ_SIM_RESOURCE_PATH}" >> ~/.bashrc
source ~/.bashrc
```

ROS 2 on this machine? Its `setup.bash` overwrites `GZ_CONFIG_PATH`
with vendor-only paths, which hides `gz sim` from the `gz` CLI even
though `gz-sim8` is installed (symptom: `gz help` lists no `sim`
command). Append AFTER the ROS source line in `~/.bashrc`:

```bash
export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH
```

then open a fresh terminal and check `gz sim --versions` → 8.x.

### 10.2 Run day — four terminals + MP + UI

Terminal A — the world (Linux, ~30 s to load):

```bash
cd mission_pi
gz sim -v4 -r sim/worlds/mission_world.sdf
# want: iris on the ground at the origin, white QR panel ~8 m away on +X.
# NO WINDOW / GUI SEGFAULT (ogre2 "Unable to create the rendering window",
# libEGL "driver (null)" on NVIDIA)? The mission doesn't need the window —
# cameras are server-side sensors. Run headless instead and verify pixels
# via the bridge (Terminal C): the GUI is optional chrome.
#   gz sim -s -r sim/worlds/mission_world.sdf
# weak GPU? -s also halves render load (sensors still publish).
```

Terminal B — SITL on the iris (Linux). Note `--model JSON` + frame
`gazebo-iris`; same no-MAVProxy two-port trick as §6:

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --no-mavproxy
# want: "Waiting for connection" + (after MP joins) the serial1 line.
```

Terminal C — the camera bridge (Linux, SYSTEM python3 — the gz
bindings live outside the venv, so do NOT activate it here):

```bash
cd mission_pi
python3 tools/gz_cam_bridge.py --port 8099
# want: subscribe /iris/front/image -> OK, /iris/bottom/image -> OK,
#        then "front: N frames age=0.0x" log lines.
```

Check the pictures BEFORE starting the mission (saves a wasted flight):

```bash
curl -s http://127.0.0.1:8099/health   # both ages < 1 s, sizes [1280, 720]
# and in a browser:  http://127.0.0.1:8099/
#   front pane = horizon/runway, bottom pane = ground + (after takeoff) panel.
```

Terminal D — mission_pi on the sim cameras:

```bash
cd mission_pi && source .venv/bin/activate
python main.py --check --config config.gazebo.yaml
# want: [fc] OK ... [cam] cam1: kind=url ... [cam] cam2: kind=url ...
python main.py --config config.gazebo.yaml
```

Then MP (connect 5760, Write WPs with the sprayer plan, Arm + Auto)
and the UI (bridge up, page open, paste `http://<laptop-ip>:8000`,
connect) exactly like §6-§8. The UI probe finds TWO cameras, so both
tiles light up. From the trigger on, the loop is fully closed: sim
pixels steer sim motion, and `QR:MISSION-QR-001` lands in MP Messages.

### 10.3 The QR target (shipped in the world — nothing to drag)

`mission_world.sdf` already includes the target at (8, 0) — inside the
40x40 m home search box the mission flies without a fence. Two sizes
ship; be honest about which one you are running:

- `qr_panel_big` (594x840 mm, 2x-A3 linear) — DEFAULT. The classical
  bench detector needs the code ~80+ px to decode; from 15 m this is
  ~53 px (cue) growing past 100 px down the approach stair. First
  closed-loop success runs on this panel.
- `qr_panel_a3` (true 297x420 mm) — ~26 px from 15 m. The classical
  detector will NOT reliably close the loop on it; it is the YOLO
  model's graduation exam (see §11). Swap it in by uncommenting the
  second `<include>` in `sim/worlds/mission_world.sdf`.

Both textures were generated in §10.1. If the panel renders BLACK in
Gazebo, the PNG is missing — re-run the two `make_qr_panel.py` lines.
To move the target, edit the `<pose>` in the world's `<include>` block
(x y z roll pitch yaw, metres) — keep it inside the search box and on
flat ground (z = 0).

### 10.4 How the video path works (and its knobs)

- The iris carries two FIXED mounts (`front_cam_link` pitched 20° down,
  `bottom_cam_link` straight down), each with a 1280x720 R8G8B8 camera
  at 15 Hz publishing `/iris/front/image` + `/iris/bottom/image`
  (explicit `<topic>` tags in `sim/models/iris_dualcam/model.sdf`, so
  topic names never depend on world/model renames).
- `tools/gz_cam_bridge.py` subscribes via the gz python bindings and
  re-serves MJPEG at `:8099/front.mjpg` + `:8099/bottom.mjpg` (+ `/`
  for humans, `/health` for scripts). No ROS, no GStreamer, no numpy.
- `mission_pi` opens both as `url`/`mjpeg` cameras — plain OpenCV http
  readers, the same code path the UI tiles already use.

Knobs: sensor rate lives in the model's `<update_rate>` (15 Hz keeps a
laptop happy next to SITL + detector; raise to 30 with headroom);
bridge JPEG quality is `--jpeg-quality`; sim lens HFOV is 68° in both
`model.sdf` (`<horizontal_fov>` 1.1868 rad) and `config.gazebo.yaml` —
change both together or the approach geometry lies.

Alternative (not needed): ardupilot_gazebo also ships a GStreamer RTP
path ("Streaming camera video" in its README) that `mission_pi` can
drink with `backend: gstreamer` — but that needs an OpenCV WITH
GStreamer (apt `python3-opencv`, not pip) plus H.264 CPU. The MJPEG
bridge above is the supported path; only revisit RTP if localhost
bandwidth ever matters (it won't — the pixels never leave the laptop).

## 11. Moving to the Pi 5 (when the bench is green)

1. Same repo/branch on the Pi; system deps per `README.md`
   (picamera2 stack instead of §2).
2. `cp config.example.yaml config.yaml`, set your lens HFOVs.
3. Drop the fine-tuned `qr_yolov8n.hef` into `models/`
   (`training/compile_hef.md`), detector `kind: auto` picks it up.
4. Run without `--device` (USB auto-detect) — or keep `--device` for
   an explicit `/dev/serial/by-id/...` path.

Bench green + air test = done. Good hunting.
i; system deps per `README.md`
   (picamera2 stack instead of §2).
2. `cp config.example.yaml config.yaml`, set your lens HFOVs.
3. Drop the fine-tuned `qr_yolov8n.hef` into `models/`
   (`training/compile_hef.md`), detector `kind: auto` picks it up.
4. Run without `--device` (USB auto-detect) — or keep `--device` for
   an explicit `/dev/serial/by-id/...` path.

Bench green + air test = done. Good hunting.
