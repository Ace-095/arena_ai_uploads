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

Live-view the bench cam any time while the stack runs:

```
http://127.0.0.1:8000/api/camera/frame/cam2
```

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

## 10. Gazebo phase (later — same setup, physics + visuals)

Needs a real GPU or a strong iGPU (ogre2) and ~5 GB more disk.
Everything in §1-§9 stays identical; only Terminal 1 changes, plus an
optional QR panel in the world.

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

### 10.2 Run the world + SITL

Terminal A (Linux):

```bash
gz sim -v4 -r iris_runway.sdf
```

Terminal B (Linux) — note `--model JSON` + frame `gazebo-iris`, same
no-MAVProxy two-port trick as §6:

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --no-mavproxy
```

Then repeat §6(MP on 5760)-§8 unchanged: same mission, same
`config.laptop.yaml`, webcam on the printed panel. The iris now flies
in Gazebo with real physics while the full QR loop runs. (SITL talks
to Gazebo over JSON on localhost — no extra config.)

### 10.3 Put the QR panel in the world (for the human eye)

```bash
cd mission_pi
python tools/make_qr_panel.py --text MISSION-QR-001 \
    --out sim/gazebo_qr_panel/materials/textures/qr.png
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/arena_ai_uploads/workspace-*/mission_pi/sim:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
# fix the * to your real middle dir name, then:
source ~/.bashrc
```

Then either drag `qr_panel_a3` from the Insert tab onto the runway
(~8-10 m from the iris), or bake it into a world copy — exact block
in `sim/WORLD_SNIPPET.sdf.txt`:

```bash
cp ~/ardupilot_gazebo/worlds/iris_runway.sdf ~/mission_world.sdf
# paste the <include> block inside <world>, then:
gz sim -v4 -r ~/mission_world.sdf
```

### 10.4 (Optional, advanced) Closed-loop Gazebo camera

Instead of the webcam, `mission_pi` can drink the simulated down-cam
as an RTP/H.264 stream: add a camera + `GstCameraPlugin` (UDP 5600)
to the iris model per the plugin README's "Streaming camera video"
section, enable streaming on its topic, and point the bench cam at it:

```yaml
cameras:
  bottom: { kind: url, backend: gstreamer, size: [1280, 720], hfov_deg: 90.0,
    device: "udpsrc port=5600 caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink" }
```

This is genuinely the full closed loop (sim pixels → sim motion), but
model surgery + gstreamer plumbing is fiddly — do §10.2 first, treat
this as a stretch goal. If the stream stutters, suspect CPU (software
H.264 decode) before suspecting `mission_pi`.

## 11. Moving to the Pi 5 (when the bench is green)

1. Same repo/branch on the Pi; system deps per `README.md`
   (picamera2 stack instead of §2).
2. `cp config.example.yaml config.yaml`, set your lens HFOVs.
3. Drop the fine-tuned `qr_yolov8n.hef` into `models/`
   (`training/compile_hef.md`), detector `kind: auto` picks it up.
4. Run without `--device` (USB auto-detect) — or keep `--device` for
   an explicit `/dev/serial/by-id/...` path.

Bench green + air test = done. Good hunting.
