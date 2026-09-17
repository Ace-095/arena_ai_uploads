# real_pi — the REAL hardware stack (Pi 5 + Pixhawk + UI)

This folder is the self-contained, real-flight stack, cut from the
`arena/01a0a7de` commit (the exact UI/Pi-link code you reference) and
hardened for the field:

```
real_pi/
├── config.yaml is at  mission_pi/config.yaml   ← ONE file, every knob
├── mission_pi/        the Pi 5 side (server, cameras, YOLO, FC link, mission)
└── mission-ui/        the Windows/laptop side (browser UI + mp_bridge)
```

Everything the mission does — video stream, camera auto-detect, Pixhawk
USB auto-connect, YOLO (Hailo first, CPU fallback), the two-stage
QR→Mission Planner relay, the altitude/sweep settings — lives in
**`mission_pi/config.yaml`**. No code edits for tuning.

---

## What was fixed / added for real flight

### 1. Video via the Pi no longer "always stuck"  (`mission_pi/streamm1.py`, `server.py`)
Root cause: the MJPEG endpoint only emitted bytes when a NEW frame seq
appeared. Before the first frame (CSI start-up, AE/AWB settle, YOLO load —
seconds on a Pi 5) it emitted *nothing*, so the browser `<img>` held an open
multipart connection with zero bytes: a permanently frozen tile.

Fixes (all config-tunable in `stream:`):
* **bounded first-frame wait** — `first_frame_wait_s: 8`; still nothing →
  503 → the UI's snapshot polling takes over instead of a dead socket;
* **liveness re-send** — if no new frame arrives within ~1 s the LATEST
  frame is re-sent, so the browser always has bytes + the freshest image;
* **camera watchdog** — `watchdog_s: 6`: a camera that goes dark is
  restarted automatically (bounded, cooled down, counted in
  `/api/cameras`);
* **adaptive quality** — the encoder watches its own load (a Pi 5 busy with
  3×3 YOLO tiles) and degrades fps/width/quality one ladder step at a time
  instead of freezing, restoring when headroom returns;
* the capture thread itself self-heals (`cameras.py`): no frames for 10 s
  → the camera re-opens in-thread, with cooldown, reported in `status()`.

### 2. Auto-detect the cams on the Pi 5  (`cameras.py`, `config.yaml cameras:`)
Every role (`front`/`bottom`) fills automatically, in this order:
1. explicit `kind:` (usb/url/file/mirror) if you set one;
2. **CSI** camera whose sensor matches `match_name` (imx708→front,
   imx477→bottom by default);
3. **USB** V4L2 camera matching `match_vidpid` (`vvvv:pppp`) or
   `match_name` (product-string substring);
4. the first remaining free USB capture node.

* `cameras.auto_rescan_s: 30` — re-scans on a timer: unplug/replug a USB
  cam **mid-mission** and it is adopted live (unchanged cameras are NOT
  touched — no feed interruption). Manual: `POST /api/cameras/rescan`.
* `tools/list_cams.py` shows exactly what THIS Pi sees and would assign.
* One camera only → it becomes `single_cam_role` (bottom by default).

### 3. Auto-connect the Pixhawk over USB  (`fc_link.py`, `config.yaml fc:`)
`fc.conn: ""` = full auto-detect:
1. `/dev/serial/by-id/*` with **ArduPilot/Pixhawk** names first (stable
   across re-plugs), then other by-id;
2. `/dev/ttyACM*` (a direct-USB Pixhawk IS a ttyACM) before `/dev/ttyUSB*`.

Every candidate is tried at every `bauds:` and the first one answering a
VEHICLE heartbeat wins. If permissions are missing the log tells you the
fix (`sudo usermod -aG dialout $USER` or `tools/install-udev.sh`).
**Re-plug / reboot / port-change recovery is automatic**: the FC watchdog
(`dead_s: 10`) notices the dead heartbeat and re-detects the device
(`link_lost → link_restored` in the UI log) — the mission waits, never
crashes.

### 4. QR → Mission Planner, NO DELAY, two-stage  (`qr_relay.py`, `mission.py`)
* **Stage 1 — instant**: the FIRST single-frame decode puts
  `QR_SEEN:<payload>` on MAVLink STATUSTEXT (severity CRITICAL). The
  Pixhawk forwards it to every GCS, so it lands in **MP's Messages tab
  within one telemetry tick (~50 ms over USB)** — before the confirm
  streak even starts. Resent for `relay.announce_window_s` (6 s) because
  STATUSTEXT has no ack.
* **Stage 2 — confirm**: when the confirm-streak completes, the official
  `QR:<payload>` fires **immediately** (not after the approach descent)
  and resends every `interval_s` for `window_s`.
* If BOTH routes (MAVLink + UI websocket) are down, the result is buffered
  to disk and flushed on recovery — `relay.store_forward: true`. Nothing
  is ever lost.

Verified on the MAVLink wire (fake-FC mirror, i.e. what MP displays):
```
T+12.4s STATUSTEXT sev=3 | 'QR_SEEN:MISSION-QR-001'   ← stage 1, ~1 tick
T+12.5s STATUSTEXT sev=6 | 'QR:MISSION-QR-001'        ← stage 2
```

### 5. YOLO on the Pi 5, Hailo first, CPU fallback  (`detector.py`, `config.yaml detector:`)
`kind: auto` ladder — every rung falls through, so the Pi never ends up
with NO detector:
1. **Hailo** `models/qr_yolov8n.hef` (AI HAT, needs `hailo-all` apt stack);
2. **ONNX** `models/qr_yolov8n.onnx` + onnxruntime (light on ARM — the
   default CPU rung, `prefer: [onnx, pt]`);
3. **.pt** `models/qr_yolov8n.pt` + ultralytics (torch, heavy on ARM);
4. **classical** cv2+pyzbar (no model, weak past ~8 m, day-one fallback).

Both YOLO models are bundled in `mission_pi/models/`. The 15 m A3 answer:
3×3 tiled inference on the bottom frame (`tile_bottom: true`) so a 34 px
QR becomes ~150 px on the network input. Measured (bench, 1920×1080 frame):
| QR size | ≈ altitude (113° lens) | detected | decoded |
|---|---|---|---|
| 34 px | 15 m | ✅ | at the limit |
| 38 px | ~13 m | ✅ | ✅ |
| 80 px | ~6 m | ✅ | ✅ |
| 150 px | ~3 m | ✅ | ✅ |

So: detection + instant `QR_SEEN` happen AT 15 m; the mission descends the
approach stair (12→9→7 m) where decode is comfortable, then the official
`QR:` relay. The resweep at 10 m is the second safety net.

### 6. "the config adjusts the drone's max and sweep"
```yaml
flight:
  max_alt_m: 15.0        # HARD ceiling — every altitude command is clamped
mission:
  sweep_alt_m: 15.0      # the altitude the search grid flies (<= max)
  resweep_alt_m: 10.0    # 2nd grid if the 1st finds nothing
  approach_stair_m: [12.0, 9.0, 7.0]
```
Drop both to 10 for a 10 m panel; the grid spacing re-computes from your
camera FOV automatically.

### 7. No-drone-crash fallbacks (already in the mission FSM, now default-on)
* battery < `min_batt_pct` (25) → abort + `post_action` (RTL);
* grid legs stay `edge_margin_m` INSIDE the fence (breach = RTL Rsn 10);
* external mode change (MP bump, RTL) trips the search within 3 ticks;
* FC link loss → mission failsafes to `post_action`, the UI stays up, and
  everything re-acquires on recovery;
* every subsystem (detector/cameras/streams/FC) is fail-soft: a failure is
  reported at `/health` + the UI log instead of killing the process;
* MAVLink sends are deadline-bounded — a wedged link can never block the
  control loop.

---

## Bring-up (ladder — test each rung before the next)

### Rung 0 — on the Pi 5, once
```bash
cd real_pi/mission_pi
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt          # core + onnxruntime
# vision: sudo apt install -y python3-opencv python3-picamera2 zbar-tools
# pyzbar: pip install pyzbar
# Hailo (only with the AI HAT): sudo apt install -y hailo-all && hailortcli scan
bash tools/install-udev.sh               # Pixhawk + cam USB permissions
```

### Rung 1 — hardware probe (no flight)
```bash
python3 main.py --check                  # FC + cameras + YOLO + ws impl
python3 tools/list_cams.py               # which cam becomes cam1/cam2
dmesg | tail -5                          # after plugging the Pixhawk: ttyACMx?
```
Expected: `[fc] OK: /dev/serial/by-id/...ArduPilot... (sysid=1)`,
both cams assigned, detector = `yolo_onnx` (or `hailo` with the HAT).

### Rung 2 — bench with fake vehicle + QR frames (no drone)
```bash
python3 tools/make_demo_frames.py --out demo_frames
python3 tools/fake_sitl.py --udp 127.0.0.1:14551 --auto --loop --no-keys &
python3 main.py                          # config.yaml (cameras: kind: file demo_frames)
# open http://<pi-ip>:8000  — tiles must MOVE (not photos, not black)
```
You should see `QR_SEEN sent to MP (instant, pre-confirm)` in the log within
seconds, then `QR relay fired on confirm`, then `DONE (payload=...)`.

### Rung 3 — real Pixhawk on the bench (powered, not armed)
Plug the USB cable. `python3 main.py` must log `FC found: /dev/serial/...`
and `/health` must show `link: true`. Mission Planner on the laptop connects
to the SAME Pixhawk (`tcp:<pi-ip>:5760` style or direct serial) — both GCSs
coexist; the Pi link is receive-only from the UI's point of view.

### Rung 4 — flight
1. MP: connect, set home, fence + mission (sprayer item = trigger), arm.
2. Pi: `python3 main.py` — phase WAIT_LINK → SNAPSHOT → WAIT_TRIGGER.
3. UI (Windows/laptop): `mission-ui/bridge/mp_bridge.py --port 8100`,
   paste `http://<pi-ip>:8000` into the Pi-link box, **Probe**.
4. Sprayer fires → TAKEOVER → SWEEP_YAW → SWEEP_GRID (at `sweep_alt_m`) →
   QR detected: **`QR_SEEN:<payload>` in MP Messages within ~50 ms**,
   `QR:<payload>` at confirm, drone descends, `DONE`, RTL.

---

## Testing (no hardware, seconds each)
```bash
cd real_pi/mission_pi  && for t in tests/*.py; do python3 "$t"; done
cd real_pi/mission-ui  && for t in tests/*.py; do python3 "$t"; done
cd real_pi/mission-ui  && python3 bridge/mp_bridge.py --selftest
```
New in real_pi: `test_stream_stuck_fix` (SendGate/watchdog/adaptive),
`test_qr_announce` (two-stage no-delay delivery), `test_cam_autodetect`
(CSI/USB auto-assign + hot-plug rescan), `test_fc_auto` (Pixhawk USB
candidate order + replug re-detect).

## Config reference (the ONE file)
Every knob, commented in place: [`mission_pi/config.yaml`](mission_pi/config.yaml).
The load-bearing ones:

| section · key | what it does |
|---|---|
| `server.host/port` | 0.0.0.0:8000 — paste into the UI |
| `stream.fps/width/quality` | UI MJPEG base rate |
| `stream.watchdog_s` | dark-camera auto-restart window |
| `stream.adaptive` | auto fps/width/quality under Pi 5 load |
| `stream.first_frame_wait_s` | stream gives up → UI polling fallback |
| `cameras.auto_detect / auto_rescan_s` | CSI+USB auto-assign, hot-plug timer |
| `cameras.front/bottom.match_name` | sensor/USB product match (imx708/imx477) |
| `cameras.*.match_vidpid` | exact USB VID:PID pin |
| `fc.conn` | `""` = USB auto-detect; pin a device if you must |
| `fc.bauds / dead_s / retry_s` | connect order + link-loss supervision |
| `detector.kind / prefer / conf_thr` | hailo→onnx→pt→classical ladder |
| `detector.tile_bottom / tile_grid` | the 15 m answer (3×3 tiles) |
| `decode.required_streak` | confirm streak (mission completion) |
| `flight.max_alt_m` | **the** ceiling — everything clamps to it |
| `mission.sweep_alt_m / resweep_alt_m` | search + re-sweep altitudes |
| `mission.min_batt_pct / post_action` | battery failsafe + end-of-mission |
| `relay.announce / announce_window_s` | **Stage 1 instant `QR_SEEN`** |
| `relay.on_confirm / interval_s / window_s` | **Stage 2 `QR:` relay** |
| `relay.store_forward` | disk buffer when both routes are down |
