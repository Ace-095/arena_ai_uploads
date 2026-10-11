# Camera + Pixhawk setup (Pi 5)

What to change when the hardware changes. Two files matter and they are
different things:

| File | What lives there |
|---|---|
| `/boot/firmware/config.txt` | **Linux/firmware**: which CSI sensor drivers load, VCM (focus motor) driver, camera detection |
| `real_pi/mission_pi/config.yaml` | **This stack**: which camera is `front`/`bottom`, its lens numbers, focus policy, QR settings, and how the Pixhawk is found |

`config.yaml` cannot enable a camera the kernel does not know about, and
`config.txt` cannot tell the mission which camera is the downward one. Both
matter; they are not interchangeable.

---

## 1. Arducam UC-517 (IMX477 + motorised/VCM focus) on a Pi 5

**What the module is:** an IMX477 HQ-class camera board (Arducam's own PCB;
`UC-517` is the board reference, sold as B0270/B0272/B0273/B0274 depending on
the package and the target board) with a **motorised focus (VCM) lens**. Some
revisions have an IR-CUT filter. It is a **MIPI CSI-2** camera, not USB — so it
is *not* a `/dev/video*` device and `kind: usb` is wrong for it.

### 1.1 Boot configuration (Pi 5, Bookworm)

Arducam's own documented procedure for this module on a Pi 5:

```bash
sudo nano /boot/firmware/config.txt
```

```ini
# find:
camera_auto_detect=1
# change it to:
camera_auto_detect=0

# under the [all] section:
dtoverlay=imx477,vcm
# ...and if the camera is on the CAM0 port:
# dtoverlay=imx477,vcm,cam0
```

`vcm` is what makes the focus motor exist. Reboot.

### 1.2 The two-camera trap (this is the one that bites)

`camera_auto_detect=0` **disables automatic detection of every camera**, not
just the Arducam. With a Pi Camera Module 3 (`imx708`) as the front camera, you
must now declare it too, and each camera should name its port:

```ini
camera_auto_detect=0
[all]
dtoverlay=imx708,cam0
dtoverlay=imx477,vcm,cam1
```

Swap `cam0`/`cam1` to match your physical wiring (they are the two 22-pin
`CAM/DISP` connectors; the labels are on the PCB). Also note the Pi 5's CSI
connectors are **22-pin** while the camera boards are 15-pin — use the 15↔22
cable that came with the module.

Verify before touching `config.yaml`:

```bash
rpicam-hello --list-cameras          # must list BOTH sensors, with indices
vcgencmd get_throttled               # 0x0 = no undervoltage (see §1.5)
```

### 1.3 Autofocus: needs the right tuning file

On the Pi 5 the ISP is `pisp` (Pi 4 and older use `vc4`), so the autofocus
tuning file lives at:

```bash
ls /usr/share/libcamera/ipa/rpi/pisp/imx477_af.json
```

If it is missing, the camera still streams but exposes **no focus controls** —
Arducam's AF only exists with that tuning file. `config.yaml` now loads it for
the role:

```yaml
  bottom:
    autofocus: true
    tuning_file: imx477_af.json
```

Behaviour (implemented in `cameras.py`, tested in
`tests/test_cam_autodetect.py`):

* `autofocus: true` → the role gets `AfMode = Continuous` **and** the tuning
  file is loaded through `Picamera2.load_tuning_file()`.
* `autofocus: false` → never touched (correct for the fixed-focus Waveshare
  module, where AF controls do not exist).
* key omitted → old behaviour: only `imx708` autofocuses by default.
* a tuning file that cannot be loaded logs a warning and falls back to the
  default tuning. **A missing file never crashes the mission.**

A focused-at-15 m fixed setting is usually better than continuous AF for a
downward camera on a moving drone (contrast-detect AF can hunt over low-texture
ground). To pin focus after arming-up checks:

```bash
rpicam-still -t 0 --autofocus-mode manual --lens-position 5 \
  --tuning-file /usr/share/libcamera/ipa/rpi/pisp/imx477_af.json
```

### 1.4 The lens numbers are the config that actually matters

`hfov_deg` / `vfov_deg` feed the 15 m footprint, the mm-per-pixel figure, the
lawnmower coverage and the cue geometry. The motorised-focus module's stock M12
lens is a **3.9 mm f/2.8, quoted 75° horizontal** — which is *not* the
Waveshare's 100° H / 113° diagonal, and also not the 6 mm CS-mount (65° H) or
the 100° H autofocus variants Arducam sells on the same sensor. The config
ships the 75° UC-517 values active with the Waveshare block right below,
commented, for a one-line revert. **Measure yours** rather than trusting the
datasheet (these boards get lens swaps):

1. Put a metre ruler/tape flat on the ground, camera at the mission altitude
   (15 m) pointing down, aimed at the tape.
2. Read the tape across the full width of the frame → that is the horizontal
   footprint `W`. Also read the height `H`.
3. `hfov_deg = 2 · atan(W / (2 · 15))`, `vfov_deg = 2 · atan(H / (2 · 15))`.
   On a Pi the same thing can be read from the frame: `python3 tools/cam_probe.py`
   saves a snapshot you can measure in any viewer.
4. Sanity check with the numbers: at 15 m a 75° lens sees ≈ 23–24 m of ground
   across 2028 px → ≈ 11 mm/px (binned) or ≈ 6 mm/px at full 4056 px. A
   narrower lens therefore *helps* QR decode and *shrinks* the area per pass —
   the code adapts the grid from `hfov_deg`, which is exactly why a wrong
   number is worse than a pessimistic one.

### 1.5 Power and thermals

Pi 5 + AI HAT + two cameras + a focus motor needs a real supply: use the
official 27 W (5 A) PSU. Symptoms of a weak supply are cameras dropping off the
bus and random I/O failures, which look like software bugs:

```bash
vcgencmd get_throttled      # 0x0 = clean; 0x50000/0x50005 = undervoltage now
```

### 1.6 Support matrix (UC-517 + Pi 5)

| Feature | Pi 5 status |
|---|---|
| Streaming / stills | works, native `imx477` libcamera driver |
| Autofocus (VCM) | works **with** `dtoverlay=imx477,vcm` + `imx477_af.json` |
| IR-CUT | controlled by the module itself (light sensor on the board); not exposed to this stack, so no `config.yaml` key |
| `camera_auto_detect=1` | must be `0` for the VCM driver to load |
| Arducam's legacy `install_pivariety_pkgs.sh` / MIPI_Camera stack | **not** for Pi 5; use the native driver + tuning file above |

---

## 2. Pixhawk 2.4.8 over USB — what to change in `config.yaml`

**Nothing.** The `fc:` block already auto-detects:

```yaml
fc:
  conn: ""                 # "" = AUTO-DETECT (the real-flight default)
  bauds: [115200, 57600, 921600]
```

A 2.4.8 running ArduPilot appears as a USB CDC device (`/dev/ttyACM0`), and
`candidate_devices()` tries `/dev/serial/by-id/*` first (ArduPilot/Pixhawk
names, then any other by-id name) and then `/dev/ttyACM*` / `/dev/ttyUSB*`.
Baud is irrelevant over USB, so the list is only there for serial dongles. The
by-id naming of these clones varies (`usb-ArduPilot…`, `usb-3D_Robotics_PX4_FMU_v2.x…`,
or vendor `0x1209`), which is precisely why the fallback covers all three.
Only pin the device if you have several ttyACM devices:

```yaml
  conn: /dev/serial/by-id/usb-ArduPilot_...
```

### 2.1 Check the link (on the Pi)

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
python3 tools/fc_probe.py --device /dev/ttyACM0          # heartbeat/mode/pos/home
python3 tools/fc_probe.py --device /dev/ttyACM0 --fence-check
```

`--fence-check` (new) answers the question that decides whether the UI's
geofence button can work on *this* board:

```
firmware    : {'version': '4.5.7', 'fw_type': 3, 'capabilities': 24577, ...}
capability  : capabilities=0x6001 (mavlink2=True mission_fence=True)
mavlink     : MAVLink 2 (needed for the fence mission type)
params      : FENCE_ENABLE 1.0 / FENCE_TYPE 12.0 / FENCE_ACTION 1.0 / ...
fence       : 4 vertices stored on the FC
verdict     : FENCE-READY (write test: UI 'UPLOAD FENCE → FC')
```

The same checks appear live in the UI's `fc firmware` row (`fc_capability` in
every `/api/fence` response), so a board that cannot take a fence is visible
before anyone presses UPLOAD. The probe is the stricter of the two: it exits
**1** when the capability question is unanswered (`did not answer
AUTOPILOT_VERSION`) as well as when the firmware lacks the fence mission type.

What it is really checking, and what to do if it fails:

| Line | Meaning | Fix |
|---|---|---|
| `mavlink : MAVLink 1` | the fence extension mission type does not exist in MAVLink 1 | set `SERIAL0_PROTOCOL=2` (MAVLink 2) and reboot |
| capability `mission_fence` false | firmware too old for mission-type fences | update to ArduCopter ≥ 4.0 (4.3+ recommended) |
| `FENCE_ENABLE 0` | fence stored but **not armed** to act on a breach | set `FENCE_ENABLE=1` (this stack deliberately never writes it) |
| `FENCE_TYPE` without the polygon bit (4) | polygon not part of the fence scope | set `FENCE_TYPE` to include 4, e.g. `4` or `7` |
| `fence : read FAILED` | no MAVLink 2 / no mission-protocol support | as above |
| `firmware : not reported` / `capability : FC did not report AUTOPILOT_VERSION` | the board never answered the capability question (real ArduPilot always does) | check the firmware version in Mission Planner; a 2.4.8 on unrelated/old firmware is the usual cause |

### 2.2 2.4.8-specific things worth knowing

* **Flash size decides the firmware.** The 2.4.8 is an STM32F427 board. Units
  with **2 MB** flash run current ArduCopter; **1 MB** variants (and "fake"
  F4 chips) only run the older `Pixhawk1-1M` builds, which are below the
  fence-friendly firmware line. If Mission Planner refuses to install recent
  ArduCopter, this is why — check the board in MP's firmware screen.
* **Permissions.** Joining `dialout` (or installing `tools/pixhawk-udev.rules`)
  is what stops "no FC link" from being a permissions problem:
  ```bash
  sudo cp tools/pixhawk-udev.rules /etc/udev/rules.d/99-drone.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
  The rules also cover the generic `ttyACM*` case, so a clone whose USB vendor
  is not `0x1209` still gets group access.
* **Both ports at once.** The Pi holds the USB link. Mission Planner must not
  claim the same physical port; use MP's telemetry radio for its own link (the
  QR `STATUSTEXT` reaches MP through the FC's rebroadcast, which is the design).
* **`FENCE_ALT_MAX`** is what the UI's altitude control writes; the polygon
  itself carries no altitude (see `docs/GEOFENCE_UI_TO_PIXHAWK.md`).

---

## 3. Change checklist

```text
[ ] /boot/firmware/config.txt : camera_auto_detect=0 (+ dtoverlay per camera)
[ ] reboot, then: rpicam-hello --list-cameras   → both sensors, note indices
[ ] ls /usr/share/libcamera/ipa/rpi/pisp/imx477_af.json    (UC-517 AF)
[ ] vcgencmd get_throttled → 0x0 (power is adequate)
[ ] python3 tools/list_cams.py  → sensor strings + indices + what config assigns
[ ] config.yaml: bottom match_name/hfov/vfov/size/autofocus/tuning_file (+ csi_index
    if two cameras report the same sensor)
[ ] python3 tools/cam_probe.py → one snapshot per camera, confirm the DOWN one
    is the ground view (rotate the drone; the image that moves is the wrong one)
[ ] measure the ground footprint at 15 m → set hfov_deg/vfov_deg
[ ] python3 tools/fc_probe.py --fence-check → FENCE-READY
[ ] python3 main.py --config config.yaml
[ ] UI: paste the Pi URL, GEOFENCE writer shows "DIRECT Pi → Pixhawk"
[ ] UI: the "fc firmware" row shows the real version + "MAVLink 2" + "fence
    mission type OK" (red / "unknown" here means do NOT fly the fence plan yet)
[ ] UI: upload the drawn fence, expect "FC readback verified"
```

`tests/` covers every software step in that list except the ones that need the
real optics: `./tools/run_tests.sh` must stay green.
