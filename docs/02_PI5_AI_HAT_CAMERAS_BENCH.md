# 02 — Pi 5 + AI HAT + cameras on the bench (vehicle still simulated)

**Audience:** whoever owns the onboard computer. This doc brings up the *real*
vision hardware — Raspberry Pi 5, Hailo AI HAT, two CSI cameras — while the
vehicle is **still SITL** on the Linux laptop, with Mission Planner + UI on the
Windows laptop.

**Why this stage exists:** it isolates the two things that are hardest to debug
at the field — the Hailo toolchain and the CSI camera stack — from everything
flight-related. If this doc is green, [`03_REAL_FLIGHT_INTEGRATION.md`](03_REAL_FLIGHT_INTEGRATION.md)
only adds wiring, parameters and props.

```text
            PI 5 (onboard computer, on the desk)                 LINUX LAPTOP          WINDOWS
 ┌─────────────────────────────────────────────┐        ┌───────────────────┐  ┌────────────────┐
 │ AI HAT+ (Hailo-8, PCIe)  models/qr_yolov8n.hef│        │ SITL              │  │ Mission Planner│
 │ Cam 3 (imx708) ──CSI──┐                      │  Wi‑Fi │  tcp 5760 ◄───────┼──┤  (arm/plan/    │
 │ IMX477 (bottom) ─CSI──┤                      │  / LAN │  tcp 5762 ◄───────┼─┐│   DO_SPRAYER)  │
 │                       ▼                      │        └───────────────────┘ │       │ mirror │
 │ mission_pi :8000  (HTTP + MJPEG + WS) ◄──────┼──────────────────────────────┼───────┼────────┤
 │   REST/MJPEG/WS ─────────────────────────────┼──────────────────────────────┼───────▼────────┤
 └─────────────────────────────────────────────┘        one field router / LAN │ bridge + UI    │
                                                                              │ http://<pi-ip>:8000
                                                                              └────────────────┘
```

Only difference from doc 01 stage 4: the cameras and the detector are real, and
`mission_pi` runs on the Pi instead of the Linux laptop. Everything on the
Windows side is unchanged.

---

## 1. Bill of materials

| Item | Notes / gotchas |
|---|---|
| Raspberry Pi 5, **8 GB** | 4 GB works; 8 GB gives headroom for two capture threads + MJPEG encode + the detector |
| Raspberry Pi **Active Cooler** (or equivalent) | Sustained inference throttles without it — `vcgencmd get_throttled` must read `throttled=0x0` |
| Official **27 W (5 V/5 A)** USB-C PSU | The AI HAT+ draws real power; under-powered PSUs cause exactly the "NPU disappears" and camera I/O errors below |
| **AI HAT+ (26 TOPS, Hailo-8)** — the project baseline | See the variant table below; the HEF must be compiled for the same architecture |
| microSD (32 GB+, A2) or NVMe HAT+ SSD | Note: an NVMe HAT+ and the AI HAT+ both want the PCIe connector — pick one |
| **Pi Camera Module 3** (IMX708, standard lens) — front | 66° HFOV, PDAF autofocus. The **standard** CM3 box now includes both the 15-pin and the 22-pin→15-pin cables; the **wide-angle** version does not include the Pi 5 cable |
| **Waveshare IMX477 IR-CUT B** (bottom) | 113° diagonal (~100° HFOV), 2.7 mm f/2.8, **fixed focus** — no PDAF, so 15 m sharpness must be verified, not assumed |
| 2× CSI ribbon, **22-pin (0.5 mm) ↔ 15-pin** adapter | Pi 5's CSI connectors are 22-pin 4-lane; the IMX477 board is 15-pin, so it needs the adapter cable |
| Field router (or the venue LAN) | Gives the Pi and the Windows laptop one subnet. LTE uplink on the router is optional and never safety-critical |
| USB-A ↔ USB cable **or** USB-to-serial (FTDI) | Not needed for this bench (the "FC" is SITL over the LAN) — it is needed in doc 03 |

### Hailo variants — get the package and the `--hw-arch` right

| Product | Chip | TOPS | Pi OS package | DFC `--hw-arch` |
|---|---|---|---|---|
| AI HAT+ (26 TOPS) | Hailo-8 | 26 | `hailo-all` | `hailo8` |
| AI HAT+ (13 TOPS) | Hailo-8L | 13 | `hailo-all` | `hailo8l` |
| AI Kit (M.2 HAT+ + module) | Hailo-8L | 13 | `hailo-all` (+ `dtparam=pciex1`) | `hailo8l` |
| AI HAT+ 2 | Hailo-10H | 40 | `hailo-h10-all` | Hailo-10H HEFs only |

The two package families **cannot co-exist**, and a HEF compiled for one chip
will not run on another. `hailortcli fw-control identify` tells you which chip
you actually have — read it before you compile anything
([Raspberry Pi AI software docs](https://www.raspberrypi.com/documentation/computers/ai.html)).

---

## 2. Flash and first boot

1. Raspberry Pi Imager → **Raspberry Pi OS (64-bit)** → set hostname
   (`mission-pi`), enable SSH, set the user (`pi`) and password, join the
   **field Wi-Fi** (the same SSID the Windows laptop will use), set locale/time.
2. First boot, then upgrade everything **before** touching the Hailo stack
   (a stale driver/library pair is the #1 cause of `hailortcli` silence):

   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo rpi-eeprom-update -a
   sudo reboot
   ```

3. Confirm the basics:

   ```bash
   hostname -I                 # note the Pi's LAN IP — the UI needs it
   vcgencmd measure_temp       # sanity
   vcgencmd get_throttled      # want: throttled=0x0
   ```

---

## 3. AI HAT bring-up

Order matters: `dkms` **before** the Hailo packages, and a reboot after each
install step. (Raspberry Pi's own AI documentation lists this exact sequence;
the driver is built by DKMS at install time, and skipping `dkms` installs the
runtime without a working kernel module.)

```bash
sudo apt install -y dkms
sudo apt install -y hailo-all          # AI Kit / AI HAT+  (Hailo-8 / 8L)
#  sudo apt install -y hailo-h10-all   # ONLY for AI HAT+ 2 (Hailo-10H)
sudo reboot
```

Verify:

```bash
hailortcli fw-control identify     # chip name + firmware version
hailortcli scan                    # lists the device on PCIe
lspci | grep -i hailo              # "Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor"
dmesg | grep -i hailo              # driver probe lines
ls -l /dev/hailo0
```

If `/dev/hailo0` exists but `hailortcli` says permission denied:

```bash
sudo usermod -aG hailo $USER       # then log out / back in
```

PCIe speed: the AI HAT+ handles PCIe itself; the **AI Kit (M.2)** needs
`dtparam=pciex1` in `/boot/firmware/config.txt`. The optional speed override
(`dtparam=pciex1_gen=2` / `=3`, or `sudo raspi-config` → Advanced → PCIe Speed)
is worth having in your back pocket — `mission_pi/training/compile_hef.md` lists
it under "`hailortcli scan` empty on Pi", together with `sudo rpi-eeprom-update -a`.

### 3.1 Version pitfalls worth knowing before you waste an afternoon

* **`Driver version (X) is different from library version (Y)`** — the kernel
  module and the runtime disagree. Fix: `sudo apt update && sudo apt full-upgrade`,
  `sudo apt install --reinstall hailo-dkms hailort`, reboot. If you installed a
  Hailo `.deb` by hand, remove it first so apt owns the stack again.
* **Bookworm vs Trixie.** The Hailo stack on Raspberry Pi OS follows the
  distribution: newer `hailo-all` releases are built for **Trixie** and are not
  offered for Bookworm. On Bookworm, pin the version your image ships (e.g.
  `sudo apt install hailort=4.19.0-3 hailo-tappas-core=3.30.0-1 hailo-dkms=4.19.0-1 python3-hailort=4.19.0-2`
  then `sudo apt-mark hold hailo-tappas-core hailort hailo-dkms python3-hailort`,
  per the Raspberry Pi AI docs) rather than chasing the newest number.
* **Don't mix install methods.** System `apt` (recommended here) *or* Hailo's
  `.deb`/wheel from the developer zone — not both.

---

## 4. Cameras

### 4.1 Physical

| Camera | Port | Facing | Role in this project |
|---|---|---|---|
| Pi Camera Module 3 (imx708) | either CSI | **forward**, pitched slightly down | context stream + bearing-only cues |
| Waveshare IMX477 IR-CUT B | the other CSI | **straight down (nadir)** | primary search sensor + goto geometry |

* Pi 5 has two 4-lane mini CSI connectors (22-pin, 0.5 mm). The IMX477 board is
  15-pin — use the 22↔15 adapter cable; the CM3 standard box includes one.
* Ribbon contacts face the connector's contact side, and the cable must be
  fully seated and clamped: an almost-seated ribbon produces `[Errno 5]
  Input/output error`, not a clear "not found".
* Leave the IMX477's **IR-CUT in day mode** — there is no IR illuminator onboard.
* `rotation_deg` in config is the *physical mount twist* (0/90/180/270) so that
  "image up" == nose direction.

### 4.2 Detect and snap (before any Python env)

```bash
rpicam-hello --list-cameras        # Bookworm/Trixie name; libcamera-hello on older images
sudo apt install -y rpicam-apps python3-picamera2 python3-opencv zbar-tools
rpicam-still -o /tmp/shot.jpg      # autofocus + capture proves sensor + ribbon + driver
```

Then the project's own probe (no config needed — it probes both CSI slots):

```bash
cd ~/mission_pi            # or wherever you cloned it (§5)
python3 tools/cam_probe.py --snap-dir snaps
# want: cam1: ... frames=N age=0.x s   -> wrote snaps/cam1.jpg
#       cam2: ... frames=N age=0.x s   -> wrote snaps/cam2.jpg
```

### 4.3 How roles are assigned (read this if you swap sensors)

`cameras.py` assigns CSI roles **by sensor name**, not by port:

* `imx708` → **cam1 / front**
* `imx477` → **cam2 / bottom**
* any other sensor → remaining slots filled **in enumeration order**
* exactly one CSI camera → its role comes from `cameras.single_cam_role`
  (default `bottom`)
* `kind: mirror` + `source:` gives a display-only alias (bench trick: light up
  the UI's CAM1 tile from the single webcam without opening the device twice)

So a swapped or non-standard sensor shows up as the *wrong tile* rather than as
a failure — `main.py --check` prints `model=` per camera; check it.

### 4.4 Calibrate `hfov_deg` (it drives the grid spacing and the goto math)

`hfov_deg` is not cosmetic: strip spacing = footprint × (1 − overlap), and the
bottom-box → ground projection uses it directly.

1. Put the drone (or the camera alone) at a measured height `h` over flat ground.
2. Mark the left and right edges of the frame on the ground, measure the width `W`.
3. `HFOV = 2 · atan(W / (2h))` in degrees → that is `hfov_deg`.

The IMX477 IR-CUT B is quoted at 113° **diagonal** (≈100° horizontal on 4:3,
<1.5% distortion) — `config.example.yaml` starts at `100.0`; measure yours.
The Pi Cam 3 standard lens is 66° H (75° D).

---

## 5. Install `mission_pi` on the Pi

Two supported layouts. Pick **A** (simplest, what `mission_pi/README.md` uses) or
**B** (isolated venv).

```bash
sudo apt update
sudo apt install -y git python3-pip python3-opencv python3-picamera2 zbar-tools
git clone https://github.com/Ace-095/arena_ai_uploads.git ~/arena_ai_uploads
ln -s ~/arena_ai_uploads/workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/mission_pi ~/mission_pi
cd ~/mission_pi
```

**A — system packages (recommended on Pi OS):**

```bash
pip install --break-system-packages -r requirements.txt
pip install --break-system-packages pyzbar
```

**B — venv that can see the Hailo + picamera2 system packages:**

```bash
python3 -m venv --system-site-packages .venv     # --system-site-packages is REQUIRED
source .venv/bin/activate
pip install -r requirements.txt
pip install pyzbar
```

> `hailo_platform` (from `python3-hailort`) and `picamera2` are installed
> **system-wide** by apt, so a plain `python3 -m venv .venv` hides them and the
> Hailo detector silently falls back to classical. `--system-site-packages` is
> the fix.

`requirements.txt` pins `websockets` / `uvicorn[standard]` on purpose: without a
websocket implementation uvicorn serves HTTP but **silently 404s every WS
route**, which reads as "the UI can't connect to the Pi" while `/health` looks
perfect.

### 5.1 The HEF model slot

* No HEF? The detector falls back to `classical` (OpenCV/pyzbar-based
  hypotheses). That is fine for bench bring-up and short range, and it is what
  the laptop runs — but the 15 m regime wants the fine-tuned model.
* Build it on an **x86** machine, never on the Pi:
  `mission_pi/training/README.md` (dataset → train → export ONNX) then
  `mission_pi/training/compile_hef.md` (DFC parse/optimize/compile, `--hw-arch hailo8`).
* Ship it:

  ```bash
  scp qr_yolov8n.hef pi@<pi-ip>:/home/pi/mission_pi/models/
  # config: detector: { kind: auto, hef_path: models/qr_yolov8n.hef, input_size: 640 }
  ```

  `kind: auto` = "Hailo if a `.hef` exists and `hailo_platform` imports, else
  classical", so the same config works before and after the model lands.

---

## 6. Bench config (paste into `config.yaml`)

`config.yaml` is gitignored by design — create it from the example and edit. For
this bench the only real changes from `config.example.yaml` are the FC device
(SITL on the *other* laptop) and the tuned lens values:

```yaml
server: { host: 0.0.0.0, port: 8000 }

stream: { fps: 12, width: 960, quality: 70 }

fc:
  conn: "tcp:<LINUX-LAPTOP-IP>:5762"   # SITL serial1 — the simulated vehicle
  retry_s: 3
  connect_timeout_s: 0                 # 0 = keep retrying; the UI stays served
  dead_s: 10                           # watchdog: stale heartbeat -> reconnect

cameras:
  front:  { size: [1920, 1080], hfov_deg: 66.0,  rotation_deg: 0, jpeg_quality: 80 }
  bottom: { size: [2028, 1520], hfov_deg: 100.0, rotation_deg: 0, jpeg_quality: 80 }
  single_cam_role: bottom              # if only one CSI cam is found

detector:
  kind: auto                           # Hailo if HEF + runtime, else classical
  hef_path: models/qr_yolov8n.hef
  input_size: 640
  conf_thr: 0.35
  iou_thr: 0.5
  tile_bottom: true                    # the 15 m answer on a 100° lens
  tile_grid: [3, 3]
  full_decode_every_n: 5
  cue_frames: 2

decode: { required_streak: 3, miss_tolerance: 1 }

mission:
  sweep_alt_m: 15.0
  approach_stair_m: [12.0, 9.0, 7.0]
  grid_overlap: 0.3
  edge_margin_m: 2.0
  resweep_alt_m: 10.0
  search_speed_ms: 2.5                 # 0 = leave the FC's speed alone
  min_batt_pct: 0                      # SITL battery is silly; re-enable for real flight
  post_action: RTL
  trigger_cmds: [216, 222, 223, 42600]
  trigger_require_auto: true

relay: { interval_s: 2.0, window_s: 15.0, store_forward: true }
```

Bottom-cam resolution note: `2028×1520@40` binned is the fps/range sweet spot
with 3×3 tiles; `4056×3040@10` full-res buys decode margin at 15 m. `size`
resizes every grabbed frame, so asking for more than the sensor delivers gains
nothing.

---

## 7. Run and verify

Start order matters, because **real SITL only opens 5762 after the first client
joins 5760** and each SITL TCP port takes one client:

1. Linux laptop: SITL (doc 01 §4.2).
2. Windows: MP → TCP `<linux-ip>:5760`. Watch the SITL console print
   `SERIAL1 on TCP port 5762`.
3. Linux laptop: `sudo ufw allow 5762/tcp` (and 8000 on the Pi).
4. Pi: the checks, then the run.

```bash
cd ~/mission_pi
python3 main.py --check --config config.yaml
```

Expected (shape verified 2026-09-15 on the bench config; values are yours):

```text
[fc] OK: tcp:<LINUX-IP>:5762 (sysid=1)
[cam] cam1: kind=rpi model=imx708 facing=front  size=(1920, 1080)
[cam] cam2: kind=rpi model=imx477 facing=bottom size=(2028, 1520)
[vision] cv2 OK / pyzbar OK / numpy OK / yaml OK / fastapi OK / uvicorn OK / pymavlink OK
[hailo] hailo_platform import OK
[hef] models/qr_yolov8n.hef EXISTS
[net] ws impl: websockets
[net] UI would serve on port 8000:
[net]     http://<pi-ip>:8000
== check done ==
```

`[vision] pyzbar MISSING` is survivable (decoding falls back to OpenCV's
`QRCodeDetector`, slower and less tolerant) — fix it with `libzbar0` +
`pyzbar`. `[hailo] unavailable … classical fallback will be used` is expected
until §3 is done. `[net] ws impl: NONE` is **not** survivable — the UI's Pi link
would 404.

Then run it:

```bash
python3 main.py --config config.yaml
```

The startup banner prints the exact URL to paste into the UI. From the Pi:

```bash
curl -s localhost:8000/health    | python3 -m json.tool   # link, ready, cams, fc
curl -s localhost:8000/api/bringup                        # stage: ready
python3 tools/link_doctor.py                              # 8 checks, 0 FAIL (self)
python3 tools/fc_probe.py --device tcp:<LINUX-IP>:5762 --fence --plan   # FC-side proof
python3 tools/param_probe.py --device tcp:<LINUX-IP>:5762 --params WP_SPD,WPNAV_SPEED,FENCE_ENABLE
```

Vision checks with the printed A3 panel in front of the bottom camera:

```bash
python3 tools/cam_probe.py --snap-dir snaps               # snaps/cam1.jpg, snaps/cam2.jpg
python3 tools/qr_bench.py snaps/cam2.jpg                  # detector + decode on a saved frame
python3 tools/qr_bench.py --detector hailo --hef models/qr_yolov8n.hef snaps/cam2.jpg
```

---

## 8. Windows side (unchanged from doc 01)

1. MP connects to SITL: TCP `<linux-ip>:5760`. Plan: `TAKEOFF` 15 →
   `WAYPOINT` ~20 m → **`DO_SPRAYER`** → `RTL`, **Write WPs**.
2. Bridge: `py -3 bridge\mp_bridge.py --port 8100` → `http://127.0.0.1:8100`.
3. MP mirror: **Ctrl+F → Mavlink → UDP Client → Write access → `127.0.0.1:14551`**.
4. UI **Pi link**: `http://<pi-ip>:8000` → **Probe**, then **Connect**. Both
   CAM1 and CAM2 tiles should now be live MJPEG from the real cameras — this is
   the first time the UI shows real pixels.
5. Arm + Auto in MP. The Pi takes GUIDED at the sprayer item, sweeps, and the
   payload lands in MP Messages as `QR:<payload>` **and** in the UI.

Diagnose the Pi link from the Windows box rather than guessing:

```powershell
py -3 workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f\mission_pi\tools\link_doctor.py --target http://<pi-ip>:8000
```

It checks TCP connect, `/health`, `/api/cameras`, a real WebSocket handshake
(101 + `Sec-WebSocket-Accept`) and how many telemetry envelopes actually arrive.

---

## 9. Auto-start on boot (do this before doc 03)

On the airframe there is no keyboard. `/etc/systemd/system/mission_pi.service`:

```ini
[Unit]
Description=mission_pi — QR hunt companion
After=network-online.target
Wants=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/mission_pi
ExecStart=/usr/bin/python3 /home/pi/mission_pi/main.py --config /home/pi/mission_pi/config.yaml
Restart=on-failure
RestartSec=3
StandardOutput=append:/home/pi/mission_pi/logs/mission_pi.log
StandardError=append:/home/pi/mission_pi/logs/mission_pi.log

[Install]
WantedBy=multi-user.target
```

```bash
mkdir -p /home/pi/mission_pi/logs
sudo systemctl daemon-reload && sudo systemctl enable --now mission_pi
systemctl status mission_pi --no-pager
curl -s localhost:8000/health | python3 -m json.tool
```

(If you used venv layout B, point `ExecStart` at
`/home/pi/mission_pi/.venv/bin/python` instead.) The server binds **before**
bring-up, so a slow FC link never leaves you with a dead port: `/health` and
`/api/bringup` answer immediately and say which subsystem is still coming up.

---

## 10. Power, thermals, EMI (bench habits that survive the flight)

* **Throttle check under load:** while the detector is running,
  `watch -n 2 vcgencmd get_throttled` must stay `0x0`, and
  `vcgencmd measure_temp` should sit well under the throttle threshold. If it
  throttles, the cooler is missing/under-performing or the PSU is weak.
* **Never power the Pi from the flight controller's 5 V servo rail** — budget a
  dedicated 5 V/5 A BEC. This matters again in doc 03.
* **EMI:** keep CSI ribbons short and away from ESC/motor wires; the telemetry
  radio antenna away from the Pi and the carbon frame.
* **Time sync:** the Pi logs timestamps the UI shows. Either let the router
  provide NTP, or accept that Pi and laptop clocks differ — `t` in every WS
  envelope comes from the sender.

---

## 11. Acceptance checklist

| # | Check | Command | Expected |
|---|---|---|---|
| 1 | NPU visible | `hailortcli fw-control identify` | Hailo-8 + firmware version |
| 2 | NPU device | `ls -l /dev/hailo0` | exists, group `hailo` |
| 3 | Runtime importable | `python3 -c "import hailo_platform; print('ok')"` | `ok` (in the env you'll run) |
| 4 | Both sensors seen | `rpicam-hello --list-cameras` | 2 cameras, names imx708 + imx477 |
| 5 | Snap works | `python3 tools/cam_probe.py --snap-dir snaps` | two JPGs, `age` < 1 s |
| 6 | Roles correct | `python3 main.py --check` | cam1=imx708/front, cam2=imx477/bottom |
| 7 | Deps | same | `[vision] … OK`, `[net] ws impl: websockets` |
| 8 | HEF present | same | `[hef] models/qr_yolov8n.hef EXISTS` |
| 9 | FC link over LAN | same | `[fc] OK: tcp:<linux-ip>:5762 (sysid=1)` |
| 10 | Plan + fence readable | `tools/fc_probe.py --device tcp:<linux-ip>:5762 --fence --plan` | `fence: N verts`, `DO_SPRAYER` item listed |
| 11 | Serve + WS | `tools/link_doctor.py` (from Windows) | `8 checks, 0 FAIL` |
| 12 | UI tiles live | UI CAM1/CAM2 | real MJPEG, mode `mjpeg` (not `polling`) |
| 13 | Decode works | `tools/qr_bench.py snaps/cam2.jpg` | payload decoded from the printed panel |
| 14 | Full loop | MP Arm + Auto | trigger → GUIDED → `QR:<payload>` in MP Messages **and** UI |
| 15 | No throttling | `vcgencmd get_throttled` | `throttled=0x0` |
| 16 | Auto-start | reboot, then `curl localhost:8000/health` | `link: true` without logging in |

---

## 12. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `hailortcli scan` prints nothing | Driver not built (`dkms` missing at install time) → reinstall `hailo-dkms`, reboot; `lspci \| grep -i hailo` tells you whether the device is even on the bus; reseat the HAT+; check `dtparam=pciex1` (AI Kit) and the EEPROM (`sudo rpi-eeprom-update -a`) |
| `Driver version (X) is different from library version (Y)` | Mismatched kernel module / runtime → full-upgrade + reinstall `hailo-dkms hailort`, reboot (§3.1) |
| `hailo-all : Depends: hailort (>= …) but it is not installable` | Stale package index / half-upgraded OS → `sudo apt update && sudo apt full-upgrade -y`, reboot, retry; on Bookworm pin the versions your image ships (§3.1) |
| `[hailo] unavailable` inside the venv | Plain venv hides system packages → recreate with `--system-site-packages` (§5) |
| Detector runs but never fires the NPU | No `.hef` at `detector.hef_path`, or `kind: none/classical` in config; `[hef] … EXISTS` in `--check` is the proof |
| `picamera2 not installed` / `[Errno 5] Input/output error` | `sudo apt install python3-picamera2`; ribbon not fully seated; camera cable wrong pitch for Pi 5 (§4.1) |
| Cameras swapped in the UI | Roles are assigned **by sensor name** (§4.3) — check `model=` in `--check` |
| Frames black but `age_s` fresh | Lens cap, IR-CUT in night mode with no illuminator, or `rotation_deg` making you look at the frame edge |
| QR won't decode from a saved snap | `tools/qr_bench.py` isolates it: fixed focus means 15 m may be soft — refocus the mount distance or accept the descend stair; add light; keep the panel flat and square |
| Pi link red in the UI | From Windows: `link_doctor.py --target http://<pi-ip>:8000`. Then in order: `[net] ws impl`, `sudo ufw allow 8000/tcp`, same subnet (`hostname -I` on both), Chrome Local Network Access |
| `[fc] FAIL: no FC heartbeat` | SITL's 5762 only opens after MP joins 5760; the laptop's firewall; SITL's port takes one client (kill any other `main.py`/MAVProxy) |
| Link dies when SITL restarts | Supervised: `link_lost` → `link_retry` → `link_restored`, `reconnects` in `/health`. Tunables `fc.dead_s`, `fc.retry_s`, `fc.connect_timeout_s` |

---

**Next:** bench green with real pixels and a real NPU ⇒ wire the airframe, the
telemetry radio and the FC → [`03_REAL_FLIGHT_INTEGRATION.md`](03_REAL_FLIGHT_INTEGRATION.md).
