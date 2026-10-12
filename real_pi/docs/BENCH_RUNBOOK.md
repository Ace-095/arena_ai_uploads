# Bench runbook — camera → angles → SITL/TCP

Three stages, in the order that fails fastest and cheapest. **Do not skip stage 1
to get to SITL**: if the camera or the focus is wrong, every SITL "detection"
result is meaningless.

| Stage | What it proves | Hardware needed |
|---|---|---|
| 0 | the code is on your box and your config still works | none |
| 1 | camera alive, sharp, and the FOV number is real | Pi + camera |
| 2 | detection/decoding vs **angle** at a known distance | Pi + camera + printed A3 panel |
| 3 | the mission links to a vehicle over **TCP** and perception is live | SITL (or fake_sitl) |

---

## Stage 0 — get the code

### Option A — run the PR branch directly (recommended for this bench)

```bash
cd ~/arena_ai_uploads            # your existing checkout
git status                       # if dirty: git stash
git fetch origin
git checkout arena/1c4bb365-arena-ai-uploads
git pull
```

### Option B — merge it into the branch you already work on

```bash
cd ~/arena_ai_uploads
git status                       # if dirty: git stash
git fetch origin arena/1c4bb365-arena-ai-uploads
git merge origin/arena/1c4bb365-arena-ai-uploads
```

The change is **purely additive (+4587 lines, 0 deletions)**, so a merge is
conflict-free unless you have edited the same three files (`config.yaml`,
`main.py`, `server.py`) in the same regions.

### Verify you got it — and that nothing flight-critical moved

```bash
cd real_pi/mission_pi
ls tracker.py perception.py perception_tap.py tools/qr_angle_bench.py   # all must exist
git log --oneline -1
# PROOF the flight logic is untouched - this must print NOTHING:
git diff HEAD~1 --stat -- mission.py
```

On the Pi, the new tools need two packages the old ones did not. `qrcode` is what
generates the A3 panel (and the synthetic corpus); `psutil` is what fills in the
CPU/RAM columns. Everything else the tools use is already in `requirements.txt`
(`onnxruntime`) or comes from apt (`python3-opencv`, `pyzbar`, `picamera2`):

```bash
sudo apt install -y python3-opencv python3-picamera2 zbar-tools
pip install --break-system-packages qrcode psutil
# Hailo, if you have the AI HAT (never pip - it is an apt stack):
sudo apt install -y hailo-all && hailortcli scan
```

Run the test suite — you should see **34 passed, 0 failed, 2 skipped**:

```bash
bash tools/run_tests.sh
```

---

## Stage 1 — camera alive, and focused at 10 m

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi

# what cameras does the Pi actually see?
python3 tools/list_cams.py

# snapshot with the MISSION's real size/tuning/focus, not library defaults
python3 tools/cam_probe.py --config config.yaml

# check the config agrees with reality (2028x1520, the right role)
python3 main.py --check --config config.yaml
```

### Focus at 10 m — this is the step that decides everything

`focus_sweep.py` sweeps the VCM in **dioptres**: `0` = infinity, `1` = 1 m,
`0.067` = 15 m, **`0.1` = 10 m**.

```bash
# wide pass: infinity -> 50 cm
python3 tools/focus_sweep.py --source csi:bottom --positions 0:2:0.125

# fine pass around 10 m (0.1 dioptres) - pick the winner from the table
python3 tools/focus_sweep.py --source csi:bottom --positions 0.04,0.06,0.08,0.10,0.12,0.15
```

**Do not skip this.** A downward camera over low-texture ground must NOT hunt for
autofocus; paste the winning lens position into `config.yaml` exactly as the tool
prints it. This is also the cheapest fix for "the picture is soft at 10 m".

> A flat white A3 sheet has almost no texture, so a focus sweep over it can be
> uninformative. Sweep over the **printed panel** (stage 2) or the actual ground.

### Measure the real FOV (kills the biggest unknown in every range number)

Every distance figure in the docs rests on a **VENDOR-QUOTED 75°/60° FOV that has
never been measured**. Put the panel at exactly 10.0 m, snapshot, measure its
width in pixels, then:

```
HFOV = 2 * atan( (panel_width_m / 2) / measured_box_px * image_width_px / 10.0 ) ...
```

easier in practice: two distances (say 7 m and 10 m) and the px width at each;
the ratio fixes the focal length in pixels, and `HFOV = 2*atan(W/(2*f))`. Put the
result into `config.yaml` (`cameras.bottom.hfov_deg` / `vfov_deg`) and re-run:

```bash
python3 tools/fov_report.py --alts 7,10,15
```

then update the label from *vendor-quoted* to *measured in this runbook's sense*.

---

### Which camera? (bottom Arducam vs the Pi Camera Module 3)

Both are computed by the same tool — `--cam` selects the role:

```bash
python3 tools/fov_report.py --alts 10,12,15              # bottom (Arducam UC-517)
python3 tools/fov_report.py --cam front --alts 10,12,15  # front (IMX708)
```

| Camera | 4608/2028 px | FOV | px/module @15 m | net px @15 m (tiled) |
|---|---|---:|---:|---:|
| bottom (Arducam UC-517) | 2028×1520 | 75°/60° | 0.79 | 24.8 |
| front (Pi Cam Module 3) | 4608×2592 | 66°/41° | **2.13** | 29.3 |

**The front camera has 2.7× the resolution, and it still does not "do 15 m" by
itself** — for three separate reasons, all of which the tool now prints:

1. **It is the wrong way round.** `front` is forward-facing. A panel flat on the
   ground seen from a forward camera is nearly edge-on, so the modules are
   foreshortened. The `px/module` column assumes the panel faces the lens
   square-on, which is only true for a vertical panel or a mount that tilts the
   camera down.
2. **The network input, not the sensor, is the bottleneck.** 4608 px is
   letterboxed to 640 — a **7.2×** shrink — so 15 m gives ~9.8 px at the network
   against ~29.3 px when 3×3 tiled. `detector.tile_front` is **false** by
   default, so as shipped the front camera cannot see a 15 m panel at all.
3. **The bottom camera is the one that finds the QR.** `single_cam_role` and the
   whole mission design put the downward camera at the centre of the search.

So: the Pi camera is a *resolution upgrade*, not a solution. If you ever want it
for the downward job it must be **remounted as the bottom camera** (`match_name`
changed, `tile_bottom: true` kept) — and if you want tiling on the front camera,
set `detector.tile_front: true` and expect **9 inferences per frame** instead of
one, which is a real cost on the CPU rung.

## Stage 2 — QR detection vs ANGLE at a known distance

### Print and lay out the panel

```bash
python3 tools/make_qr_panel.py --text MISSION-QR-001 --out qr_panel_a3.png --full
```

Print at **100 % scale on A3** (no "fit to page"), lay it flat, tape it down so it
cannot curl. Measure the distance from the **lens to the panel** with a tape —
write it down, the tool takes it as `--dist`.

### Run the angle sweep

```bash
# roll: rotate the PANEL on the ground (or yaw the drone about its own axis)
python3 tools/qr_angle_bench.py \
    --source csi:bottom --dist 10 \
    --angles 0,15,30,45,60,90 \
    --per-angle 15 \
    --payload MISSION-QR-001 \
    --hailo \
    --save-frames /tmp/angles \
    --json /tmp/angles.json --md /tmp/angles.md
```

The tool waits for you to set each angle and press Enter (add `--no-prompt` for a
scripted run). It uses the mission's own detector/filter/decoder, so it is
measuring what flies.

**Then do tilt separately** — it is a different failure mode. Prop the panel up
against a wall at 15°/30°/45° while the camera stays level (that is what an
off-nadir camera sees), and pass the same angles. Roll and tilt do not fail the
same way.

### What to expect at 10 m — and how to read it

| Quantity | Predicted | Where it comes from |
|---|---|---|
| QR size on the sensor | **~39 px** (25 modules + 4 quiet) | `fov_report.py`, **vendor FOV** |
| px per module | **~1.19** | same |
| Decode | **marginal** | the empirical floor is ~1.3 px/module |
| Detection | expected fine | detection needs far fewer pixels than decoding |

Read the verdict column:

* `DETECT + DECODE` — good, this angle works.
* `DETECT only` — the box is there, the decoder cannot resolve it. Expected near
  10 m and out at 15 m.
* `DECODE != --payload` — a payload came out but it is not the text you asked
  for. The line `payloads seen at ... deg:` right under the table tells you what
  it actually read (wrong text, or a stray QR in the background).
* `MISS` / `weak detect` — a real problem; check focus first, then lighting, then
  the preset (`--conf`, or `detector.preset`).

### If 10 m will not decode: the 7 m check (this is the mission design)

The mission does **not** try to decode at search altitude. `config.yaml` has
`approach_stair_m: [12, 9, 7]`. So the question that actually matters is:

```bash
python3 tools/qr_angle_bench.py --source csi:bottom --dist 7 \
    --angles 0,45 --per-angle 15 --payload MISSION-QR-001 --hailo
```

If **7 m decodes and 10 m does not, the system is working as designed** and the
stair is doing its job. If 7 m does not decode either, stop and fix focus/lighting
before touching anything else.

### Score saved frames properly (optional)

`--save-frames` gives you JPEGs per angle as `a<angle>/f000.jpg`. Re-score any set
of them offline, with no camera, using the same tool and the same scoring maths
(this is the mode that was exercised on this machine):

```bash
python3 tools/qr_angle_bench.py \
    --frames "0:/tmp/angles/a0/*.jpg" \
    --frames "45:/tmp/angles/a45/*.jpg" \
    --detector onnx --dist 10 --payload MISSION-QR-001
```

`tools/perception_bench.py run` is the **IoU precision/recall** harness, and it
needs `labels.jsonl` with hand-written boxes — the JPEGs above are not in that
format, so do not point `--corpus` at them. If you want real precision/recall
(rather than "did a box appear"), annotate the frames and write
`labels.jsonl` one line per frame:

```json
{"frame": "frames/a0_00.jpg", "condition": "a0", "seq": "a0", "alt_m": 10.0,
 "boxes": [[x, y, w, h]], "payload": "MISSION-QR-001", "synthetic": false}
```

`frame` is a path **relative to the corpus directory** and must match a real
filename — if it does not, the tool silently scores zero frames and prints an empty
per-condition table. `boxes` is `[x, y, w, h]` per QR, and `payload` is the text you
know is printed there (or `null` if the frame has no panel). Then:

```bash
python3 tools/perception_bench.py run --corpus /tmp/angles --md /tmp/scored.md
```

---

## Stage 3 — SITL over TCP

### Fast path first: `fake_sitl` (no ArduPilot build, seconds)

This is the right first move — it proves the TCP link and the UI without a
30-minute ArduPilot install. It binds **both** 5760 and 5762 immediately.

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi

# terminal 1
python3 tools/fake_sitl.py --ports 5760,5762

# terminal 2 - note: no YAML editing needed, --device overrides fc.conn
python3 main.py --device "tcp:127.0.0.1:5762" --no-cams --port 18099
```

**Verified on this exact command**, with the perception tap installed:

```jsonc
// curl -s http://127.0.0.1:18099/health
{ "link": true,
  "mission": { "phase": "WAIT_TRIGGER", "detail": "waiting for sprayer seqs [2]" },
  "fc": { "device": "tcp:127.0.0.1:5762", "connected": true, "hb_age_s": 0.0,
          "mode": "STABILIZE", "armed": false, "veh_sysid": 1 } }
```

```jsonc
// curl -s http://127.0.0.1:18099/api/perception
{ "state": "SEARCHING", "source": "tap",
  "quality": { "measured": false, "reason": "not-measured" },   // no camera in that run
  "stats": { "frames": 0 },
  "perception_only": true, "drives_flight": false }
```

`--no-cams` is deliberate for this first pass: link and mission state without
camera noise. `quality.measured: false` is the layer being honest — no frames
flowed, so it refuses to invent a number.

### Real ArduCopter SITL (when you want actual flight behaviour)

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-mavproxy
# wait for: "SERIAL0 on TCP port 5760"
```

Two things that waste hours if you do not know them:

1. **SITL opens 5762 only AFTER the first client connects to 5760.** So connect
   Mission Planner to 5760 **first**; until then `tcp:127.0.0.1:5762` is refused
   and that is normal, not a bug.
2. SITL's serial1 is what mission_pi joins. Watch for `SERIAL1 on TCP port 5762`
   in the SITL console before starting terminal 2.

### VERIFIED against a real ArduCopter build (not just documented)

Both behaviours above were reproduced on a freshly built `arducopter` SITL
(ArduPilot master, `sim_vehicle.py -v ArduCopter -f quad --no-mavproxy -w`):

| Check | Result |
|---|---|
| connect straight to `tcp:5762` | **no heartbeat, ever** — not a slow start |
| connect to `tcp:5760` | heartbeat immediately |
| connect to `5760` **and hold it**, then `5762` | heartbeat in **<0.1 s**, sysid 1 |
| a second concurrent client on `5762` | **no heartbeat** — one client per SERIAL port |

So `mission_pi` needs something holding 5760 — Mission Planner, or a script:

```bash
# keep this running in its own terminal while you bench against real SITL
python3 - <<'EOF'
from pymavlink import mavutil
import time
c = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
c.wait_heartbeat(timeout=10)
print("holding SERIAL0 open - mission_pi can now use 5762")
while True: time.sleep(1)
EOF
```

`tests/test_sitl_flight.py` does this for you (it primes SERIAL0 and reuses one
link, because of the one-client-per-port rule).

```bash
# terminal 2 (after MP is connected to 5760)
cd ~/arena_ai_uploads/real_pi/mission_pi
python3 main.py --device "tcp:127.0.0.1:5762"
python3 main.py --check --device "tcp:127.0.0.1:5762"
# want: [fc] OK: tcp:127.0.0.1:5762 (sysid=1)
```

Firewall, if the Pi is a separate box:

```bash
sudo ufw allow 5760/tcp && sudo ufw allow 5762/tcp && sudo ufw allow 8000/tcp
```

### Add the camera to SITL

Give the mission a real video source while SITL flies. **Careful:** `--device` is
the **FC link**, not the camera — there is no camera CLI flag. Cameras come from
the `cameras:` section of the config, so either edit it:

```yaml
# config.yaml
cameras:
  front:
    kind: usb
    device: 0          # a USB webcam index
```

…or point `--config` at a bench copy of the file. Gazebo's camera is re-served as
MJPEG (system python3, **not** the mission venv):

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil python3-opencv
python3 tools/gz_cam_bridge.py --port 8099 --qr-boost \
    --contrast 1.8 --saturation 0.6 --sharpness 2.0 --enhance-mode ground_suppress
# browser check: http://127.0.0.1:8099/   raw: /bottom_raw.mjpg
```

then set the camera to read that stream — in `config.yaml`:

```yaml
cameras:
  bottom:
    kind: url
    backend: mjpeg
    device: "http://127.0.0.1:8099/bottom.mjpg"
```

### Watch the perception layer while SITL runs

```bash
curl -s http://<pi-ip>:18099/api/perception | python3 -m json.tool
watch -n1 'curl -s http://127.0.0.1:18099/api/perception | python3 -c \
  "import json,sys; d=json.load(sys.stdin); print(d[\"state\"], d[\"detail\"], \
   d[\"tracks\"], d[\"payload\"], d[\"stats\"][\"frames\"])"'
```

The state should walk `SEARCHING → CANDIDATE_DETECTED → TARGET_TRACKING →
QR_DECODING → PAYLOAD_VALIDATED → RESULT_REPORTED` as the panel comes into view.
If it stays in `SEARCHING` with `stats.frames` climbing, the camera is delivering
frames and nothing is being detected — that is a stage-2 problem, not a SITL one.

---

## Honest status of what these stages produce

| Stage | What a pass proves | What it does NOT prove |
|---|---|---|
| 1 | camera + focus work at that distance | anything about detection |
| 2 | detection/decoding rates vs angle, on a bench | flight: no vibration, no attitude change, no prop wash |
| 3 | the TCP link, the mission FSM and perception are wired and live | that the vehicle flies correctly |

**Detection columns in the angle bench rest on your assertion** that the panel was
in view — that is not a measurement. **Decode columns are self-verifying**: the
payload either came out or it did not, and `match` confirms it was the right text.

No Hailo and no real-camera numbers existed where this code was written. Everything
Hailo-, camera- and flight-dependent is **NOT YET MEASURED** until you run these
stages.

---

## Stage 0b — the one-run field survey (`tools/pi_field_report.py`)

**Why this exists:** the machine that wrote this repo cannot reach a private LAN
address, so the rig is only measurable from itself. Rather than have you run ten
commands and read numbers off a screen, this collects everything into one JSON
plus one short summary you can paste back.

**It is read-only.** It writes no config, sets no camera control (no VCM/focus,
no exposure), and sends no MAVLink. `--with-fc` is the ONLY path that touches the
autopilot and it only reads (`fc_probe.py --fence` / `--fence-check`). A test
fails if the source ever grows a call that actuates anything. It DOES open the
camera, so **stop `mission_pi` first** — two processes cannot share a CSI camera
and this would otherwise look like a camera fault.

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi

# 1. the plain survey (no camera? add --no-capture): ~10 s
python3 tools/pi_field_report.py

# 2. + read the Pixhawk (mission_pi STOPPED, vehicle DISARMED, props off)
python3 tools/pi_field_report.py --with-fc

# 3. MEASURE the real field of view (see the setup note below)
python3 tools/pi_field_report.py --dist-m 2.00 --panel-mm 420 --panel-mm-h 297

# 4. just one answer, quickly
python3 tools/pi_field_report.py --sec fov --dist-m 2.00 --panel-mm 420

# 5. + the whole test suite on the Pi itself
python3 tools/pi_field_report.py --with-tests
```

### Measuring the FOV (this is the number every range calculation rests on)

`config.yaml` still carries a **VENDOR-QUOTED 75°/60°** FOV. Until it is measured,
every range figure in this repo is conditional. To turn it into a measurement:

1. Put a **white sheet flat on a DARKER floor**, directly under the lens, **centred
   in the picture**. A3 is ideal; `--panel-mm` is the sheet's **LONG side in mm**
   (A3 = 420, A4 = 297).
2. Measure **lens to sheet** with a tape — that is `--dist-m`.
3. Run it. It refuses rather than guesses: the sheet must be centred, not touching
   the frame edges, larger than 0.5 % of the picture and clearly brighter than the
   ground. Every refusal prints why.
4. Read `FOV: MEASURED hfov=… vfov=…`, then put those numbers into
   `config.yaml` (`cameras.bottom.hfov_deg` / `vfov_deg`) — otherwise the UI's
   coverage planner and the mission's footprint maths keep using the vendor value
   (see `tests/test_ui_camera_specs.py`, which exists because they once drifted
   33 % apart).

**Accuracy, and its limits.** Recovering a *known* FOV from synthetic pictures is
within **0.35°** across 75°, 66° and 50° lenses — that is asserted in
`tests/test_pi_field_report.py`. Real-world caveats, all printed with the result:
a single centred sheet measures the **CENTRE** field of view, corner distortion is
not covered; the sheet must be flat and perpendicular to the lens; and the
distance is only as good as your tape. A very wide lens far from a small sheet may
find nothing measurable (at 100° and 3 m an A3 sheet is only ~0.3 % of the frame) —
move it closer or use a bigger sheet.

### What a field report does and does not prove

| Row | Proves | Does NOT prove |
|---|---|---|
| `board` / `thermal` | the Pi's own model, temperature and **throttle flags** | that it can hold a framerate under load |
| `deps` / `hailo` | whether `hailo_platform` and a device are present | that a HEF loads or runs |
| `detector` | **which rung the ladder chose** (`IS_HAILO=true/false`) | Hailo accuracy or speed |
| `cameras` | real capture size and `latest()` latency | image quality, motion blur, exposure |
| `FOV` | the actual field of view, ±0.35° on synthetic input | how well the QR decodes in flight |
| `decode` | the payload came out of a REAL frame (`decoded=true`) | that it decodes in flight, at speed, at 10–15 m |
| `inference` | detector cost on real frames, on the chosen rung | end-to-end fps with capture + decode in the loop |
| `fc` | firmware, capabilities, `FENCE_ENABLE`, fence readback | that the fence is *enforced* (storing ≠ arming) |
| `tests` | the suite passes **on the Pi**, not just on a laptop | flight behaviour |

A capture latency is only meaningful if `thermal` shows no throttle flags: a
throttled Pi cannot hold a framerate and the numbers will lie.

**Nothing here is a flight result.** It is bench evidence, and it is labelled as
such in the JSON (`read_only: true`, per-section provenance).

---

## Stage 0c — when the detector section crashes (SIGBUS on the Pi)

**What happened, measured.** On the real Pi, `scout` reported:

```
  system    done (534 ms)
  deps      done (3637 ms)
  hailo     done (228 ms)
```

and then the process died with **exit −7 — SIGBUS**. The next section was
`detector`, which builds the detector ladder and constructs
`HailoYolo8Detector` (`detector.py:538`).

Two things were wrong, and only one of them was the Pi's:

1. **`pi_field_report` could not survive it.** `try/except` catches exceptions, not
   signals. A native crash in an optional section killed the whole survey and
   wrote **no report at all** — the tool whose entire job is to bring information
   back brought none. Every accelerator step now runs in a child process, so a
   crash is a *result*: the section reports the signal and keeps going.
2. **Hailo initialisation on this Pi is failing at a native level.** That is a
   genuine finding, not a tool bug, and it is the thing to fix next.

### Read this first: the new `hailosteps` line

`scout` now walks Hailo initialisation one step at a time, each in its own child
process, so the report names the step that died **and captures HailoRT's own error
text** — which names the cause:

```
hailosteps : CRASHED at: open the device (hp.VDevice) (3/5 steps ok)
             died by SIGBUS
             [HailoRT] [error] CHECK failed - max_desc_page_size given 16384 ...
```

The five steps are exactly what `HailoYolo8Detector.__init__` does, in order:
`import hailo_platform` → `Device.scan()` → `HEF(path)` → `VDevice()` → build the
detector. **The first step that dies is the culprit**; the ones after it still get
probed.

### The two documented causes on a Pi 5, and their fixes

Both are known Pi 5 + Hailo integration problems. Which one you have is decided by
the `[HailoRT] [error]` line above, not by guessing.

**(a) PCIe descriptor page size — 16 KB pages vs a 4096-byte device.**
Pi 5 kernels run 16 KB pages; the Hailo PCIe device advertises a 4096-byte maximum
descriptor page. The symptom string is explicit:

```
[HailoRT] [error] CHECK failed - max_desc_page_size given 16384 is bigger than hw max desc page size 4096
```

Fix ([Hailo community](https://community.hailo.ai/t/hailort-error-check-failed-max-desc-page-size-given-16384-is-bigger-than-hw-max-desc-page-size-4096/3690),
[official Pi 5 guide](https://github.com/hailo-ai/hailo-rpi5-examples/blob/main/doc/install-raspberry-pi5.md)):

```bash
echo 'options hailo_pci force_desc_page_size=4096' | sudo tee /etc/modprobe.d/hailo_pci.conf
cat /etc/modprobe.d/hailo_pci.conf      # must show exactly that one line
sudo reboot
```

**(b) Driver and HailoRT version mismatch.** The community reports this constantly
on Pi 5, and it looks like this:

```
[HailoRT] [error] CHECK failed - Driver version (4.20.0) is different from library version (4.21.0)
[HailoRT] [error] Driver version mismatch, status HAILO_INVALID_DRIVER_VERSION(76)
```

Diagnose with:

```bash
hailortcli fw-control identify
apt list --installed 2>/dev/null | grep -Ei 'hailo|hailort'
dmesg | grep -i hailo | tail -20
```

Then make the driver and the library **the same version** — either install the
matching `hailort-pcie-driver` .deb, or move the runtime to match the driver. Do
not mix.

**(c) A third, community-reported lead:** Hailo on Pi 5 reportedly broke across
kernel `6.12.20 → 6.12.25`
([Hailo community](https://community.hailo.ai/t/problems-running-the-hailo-raspi5-examples/14474)).
`uname -r` and the kernel the working examples were built against are worth
comparing if (a) and (b) are both clean.

### Quick evidence bundle

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
uname -r
hailortcli fw-control identify 2>&1 | head -20
cat /etc/modprobe.d/hailo_pci.conf 2>/dev/null || echo "(no hailo_pci.conf)"
apt list --installed 2>/dev/null | grep -Ei 'hailo|hailort'
python3 tools/pi_field_report.py --sec hailo_steps     # names the failing step
```

`hailo_steps` is read-only and takes a couple of seconds. Or just ask the agent:

```bash
python3 tools/pi_agent.py --queue "/pi scout"
```

### Honest status

The crash is **measured**; the cause is **not yet determined**. The two failures
above are documented Pi 5 problems and are the leading candidates, which is why
the probe now captures the HailoRT error text that distinguishes them. Until the
detector builds, **every Hailo number in this repository is unmeasured** and the
ladder is falling back to CPU — which, per the brief, must be visible rather than
silent. `IS_HAILO=false` in the report is that visibility.
