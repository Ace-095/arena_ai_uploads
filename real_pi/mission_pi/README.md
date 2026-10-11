# mission_pi — QR-hunt companion for the Pi 5

Pi 5 (8 GB) + AI HAT (26 TOPS) + Pi Camera 3 (front) + IMX477 (bottom),
USB to the Pixhawk. Watches the AUTO mission, takes over in GUIDED when
the `DO_SPRAYER` item executes, searches the geofence for an A3 QR at
~15 m, flies to it, decodes it, and reports the payload to Mission
Planner (Messages tab) and to the laptop UI (over the field router).

```
AUTO mission ──► 15 m over home ──► DO_SPRAYER fires ──► Pi takes GUIDED
                                                              │
            snapshot home + geofence (= search area) at boot ──┘
                                                              ▼
            360° stepped yaw sweep ──► lawnmower over fence ──► cue?
                                                              ▼
                     bottom-cam box → ground offset → goto → descend
                                                              ▼
                       decode ×N consensus ──► QR:<payload>
                                                              ▼
                    STATUSTEXT → FC → MP messages  +  ws event → UI
```

## Hardware + wiring

| part | connection | notes |
|---|---|---|
| Pi 5 ↔ Pixhawk | USB-A ↔ USB (telem2 coverts to `/dev/ttyACM0`) | FC auto-detect probes `by-id`, `ttyACM*`, `ttyUSB*` @ 115200/57600/921600 |
| AI HAT+ 26 TOPS | PCIe ribbon, `hailo-all` stack | `hailortcli scan` must see it; needs `dtparam=pciex1_gen=2` |
| Pi Cam 3 (imx708) | CSI 0 or 1, facing FORWARD | role assigned by sensor, not port |
| Arducam UC-517 (IMX477 + VCM focus) — *or* Waveshare IMX477 IR-CUT B (bottom) | other CSI, facing DOWN | **both report `imx477`** — pin the role with `cameras.bottom.csi_index` when both are fitted; UC-517: `autofocus: true` + `tuning_file: imx477_af.json` + `dtoverlay=imx477,vcm` at boot; Waveshare: fixed focus, 113° diag, day IR-CUT mode. Calibrate `hfov_deg` — [setup guide](../docs/CAMERA_AND_PIXHAWK_SETUP.md) |
| Router/LTE | Pi + laptop join the same field LAN | Pi serves `http://<pi-ip>:8000`; laptop pastes that as the Pi link |

Camera roles come from `config.yaml`, and every key has a fallback: `kind`,
`match_name`/`match_vidpid`, per-role `csi_index` (an explicit libcamera index
wins; a stale or duplicate index only warns and falls back to the sensor
match), `autofocus` (`true`/`false`/unset = sensor default) and `tuning_file`
(a missing file warns and uses the default tuning — the camera still streams).
`tools/list_cams.py` prints the real sensor strings + indices on your Pi and
what the config picks, so role assignment is verified, not assumed. The UI's
`scan cams` button does the same thing live (`POST /api/cameras/rescan`) and
prints `detected: cam1 (front) … · cam2 (bottom) …`, so "which camera became
the bottom one?" is answerable without a terminal. Auto-detect is meant to work
with whatever is plugged in: a single camera of ANY kind (CSI or USB) takes the
configured `single_cam_role`, and a rescan keeps a camera that is still
streaming even if the probe missed it (dropped only once its frames really
stop — `cameras.rescan_keep_grace_s`).

Why router/LTE: the Pi streams + pushes events to whatever can reach
its HTTP port. A portable field router gives Pi + laptop one LAN (LTE
uplink on the router is only needed if someone remote must watch too).

## The 15 m detection math (why the pipeline looks like this)

Numbers below are computed from the FOV in `config.yaml`, so **they move when
the lens changes** — recalibrate `hfov_deg` first. With the fitted Arducam
UC-517 (75° HFOV): at 15 m the footprint is **~23 m wide → 11.4 mm/px binned /
5.7 mm/px full-res**. An A3 QR (297 mm side) is then **~26 px** binned
(**~52 px** at full res) but only **~8 px** after a naive 640 resize. (The
previous Waveshare lens, ~100° HFOV, gave ~36 m and ~17 px binned.) 8 px is
hopeless — so:

1. **Bottom cam runs YOLO on 3×3 tiles of the frame**: each tile is
   ~676 px wide, so the A3 QR lands at ~25 px on the 640 network input.
   Detectable for a fine-tuned 1-class YOLOv8n — and the 23 m footprint
   means a 50×50 m fence needs only a few grid rows. (2028×1520 binned
   @40 fps + 3×3 is the fps/range sweet spot; 4056×3040@10 full-res buys
   decode margin at 15 m.)
2. **Detect → crop full-res → upscale → decode**: YOLO proposes the box,
   pyzbar/cv2 decode the zoomed crop (plus full-frame + tiled attempts
   every Nth frame as backup).
3. **Consensus**: same payload on 3 consecutive frames (1 miss tolerated)
   before anything is reported or flown-to.

Camera decision (per the brief — hardware calls it): the **bottom
IMX477 is the primary search sensor** (ground QR + stable nadir
geometry for goto math); the **front Pi Cam 3 streams context and
contributes bearing-only cues** (yaw toward + step forward until the
bottom cam takes over). The 26 TOPS HAT runs both streams without
breaking a sweat (YOLOv8n-640 ≈ milliseconds); without the HAT the
classical fallback still works at short range.

Bottom lens truth: the **currently fitted** bottom module is the **Arducam
UC-517** — IMX477 + motorised (VCM) focus, ~75° HFOV on the stock 3.9 mm M12
lens
(both `hfov_deg` and the Waveshare's 100°/113° values are kept side by side in
`config.yaml`: the UC-517 block is active, the Waveshare block is commented out).
Focus is only real with `dtoverlay=imx477,vcm` at boot **and** the Arducam AF
tuning file (`imx477_af.json`) loaded — `config.yaml` does the latter via
`autofocus: true` + `tuning_file`. The previous Waveshare B was fixed-focus,
f/2.8, 113° diagonal, no PDAF; for either module, measure the ground footprint
at 15 m rather than trusting the datasheet (steps in
[`../docs/CAMERA_AND_PIXHAWK_SETUP.md`](../docs/CAMERA_AND_PIXHAWK_SETUP.md)),
and set `rotation_deg` so "image up" == nose direction.

> **No Pi yet?** Run the whole stack on a Linux laptop against SITL:
> `SIM_GUIDE.md` (webcam + printed QR + Mission Planner, Gazebo later).

## Install (Pi 5, Pi OS)

```
sudo apt update && sudo apt install -y python3-pip python3-opencv \
  python3-picamera2 zbar-tools hailo-all
pip install --break-system-packages -r requirements.txt
hailortcli scan            # must list the HAT
cp config.example.yaml config.yaml   # then set bottom hfov + hef path
python main.py --check      # FC + cams + hailo report
```

Auto-start on boot (recommended): a systemd unit running
`/home/pi/mission_pi/main.py --config /home/pi/mission_pi/config.yaml`
after `network-online.target`.

## Run

```
python main.py --config config.yaml        # flight
python main.py --check                     # bring-up report, no flight
python main.py --config config.demo.yaml   # zero hardware: fake FC + file cam
```

**The HTTP server is up before anything else.** Bring-up (FC link, cameras,
detector, streams) runs on its own thread, so a missing flight controller or
camera no longer means a dead port — `http://<pi-ip>:8000/health` answers
immediately and says exactly which subsystem is missing and why. On startup
`main.py` prints the LAN URLs to paste into the UI:

```
Pi link (paste ONE of these into the UI's Pi-link box on Windows):
    http://192.168.1.42:8000
  from Windows:  curl http://<that-ip>:8000/health
  if that hangs:  sudo ufw allow 8000/tcp   (and 5760/5762 for SITL)
```

Then paste `http://<pi-ip>:8000` into the laptop UI as the Pi link
(receive-only: snapshots + ws events flow Pi → UI; the UI never
commands the Pi — `takeover`/`abort` are local bench endpoints only).

`requirements.txt` pins **`websockets`**: without a websocket implementation
uvicorn serves HTTP but silently 404s `/ws/telemetry`, which reads as "the UI
can't connect" while `/health` looks fine. `main.py` warns at startup if it is
missing, and the UI falls back to polling `/api/fsm/status` + `/api/qr/status` every 2 s.

### Endpoints

| Endpoint | What it gives you |
|---|---|
| `GET /health` | `link`, `ready`, `bringup` ledger, `fc` link detail (`device`, `connected`, `down`, `reconnects`, `hb_age_s`, `watchdog`), cams, streams, ws clients, `urls` |
| `GET /api/bringup` | just the bring-up ledger — the "why isn't it working" endpoint (`stage`, per-subsystem `state`, `detail`, capped `errors[]`) |
| `GET /api/cameras` | per-camera status (kind, size, facing, age) |
| `GET /api/camera/frame/{cam}` | one JPEG snapshot |
| `GET /api/camera/stream/{cam}` | MJPEG stream (encode once, serve many) |
| `GET /api/camera/{cam}/controls` | detector/slider tuning |
| `GET /api/mission/status`, `/api/fsm/status`, `/api/qr/status` | mission + FSM + QR state |
| `WS /ws/telemetry` | envelopes `{channel, t, data}`: `system`, `fsm`, `mission`, `qr`, `coverage`, `log`, `event`, `telemetry` |
| `POST /api/mission/takeover`, `/api/mission/abort`, `/api/mission/reset` | bench only |

### FC link supervision

The link is watched: if the vehicle heartbeat goes stale (`fc.dead_s`, default
10 s — SITL restarted, USB replugged, laptop slept) the socket is torn down,
the device is re-acquired (`find_fc()`, with pymavlink `autoreconnect` on
network devices) and the reader/pusher threads are restarted. Each step is
published to the UI log and to `/health`:

```json
{"link": true, "ready": true,
 "fc": {"device": "tcp:127.0.0.1:5762", "connected": true, "down": false,
        "reconnects": 1, "hb_age_s": 0.4, "watchdog": true}}
```

Events: `link_lost` → `link_retry` (per failed attempt) → `link_restored`.
`fc.connect_timeout_s: 0` (the default) retries forever while the UI stays
served; a non-zero value gives up after that many seconds and pins the
bring-up stage on `fc`.

SITL bench test (no hardware): point `--device` at SITL
(`udp:127.0.0.1:14550`) and run with no cameras to exercise
link → snapshot → trigger → GUIDED against the simulator. For a full
chain test with no simulator either, see `SIM_GUIDE.md` §0.5
(`tools/fake_sitl.py` + `config.demo.yaml`).

### Diagnosing a link

```
python tools/link_doctor.py                                # this machine
python tools/link_doctor.py --target http://<pi-ip>:8000   # from the UI box
```

Checks the TCP connect, `/health`, `/api/cameras`, a real WebSocket handshake
(101 + `Sec-WebSocket-Accept`) and how many telemetry envelopes actually
arrive, printing a fix hint for each failure.

## Mission details

* **Snapshot at boot** (while still AUTO): `HOME_POSITION` + fence
  polygon (`mission_type=FENCE` download) + plan scan for `DO_SPRAYER`
  (216). No fence (< 3 pts) → FAILSAFE, never takes over.
* **Trigger** (aligned with the ftest refs on `main:uploads/`): plan
  scan for sprayer cmds {216, 222, 42600}; fire when `MISSION_CURRENT`
  reaches/passes the first one, when the current item's command matches
  (DO items emit no ITEM_REACHED), optionally when the NEXT item matches
  (`trigger_peek_ahead`, claude3 style — may pre-empt 15 m), or on
  sprayer STATUSTEXT — all gated on AUTO (or manual takeover).
* **Takeover**: GUIDED via `DO_SET_MODE` (ACK + heartbeat mode verify),
  then climb/hold to sweep altitude before the yaw sweep starts.
* **Yaw sweep**: 12 × 30° relative steps with settle pauses — sharp
  frames detect; motion-blurred ones don't.
* **Grid**: serpentine over the fence bbox clipped to the polygon, row
  spacing = footprint × (1 − overlap).
* **Approach**: bottom-box center → nadir ground projection → goto
  (target always clamped INSIDE the fence) → descend stair
  12 → 9 → 7 m while centered → decode consensus.
* **Relay**: `qr` ws event instantly + `STATUSTEXT "QR:<payload>"`
  every 2 s for 15 s → MP Messages tab.
* **Post**: `post_action` (`RTL` default, or `AUTO` to resume the
  mission, `LOITER`, `LAND`). Battery under `min_batt_pct`, link loss,
  or any phase timeout → best-effort RTL + loud logging.

Payload note: mission-ui's MAVLink QR catcher latches
`QR\s*:\s*([0-9A-Za-z][0-9A-Za-z._+-]{0,31})` — colon required, first char
alphanumeric, up to 32 chars total, so `MISSION-QR-001` is captured. (The old
`[0-9A-Za-z]{1,6}` limit is gone; prose such as "the QR code was missed" still
must not latch.) The ws event always carries the full text.

## Fine-tune loop (x86)

```
cd training
python make_qr_dataset.py --out qr_dataset --n-train 3000 --n-val 400
python train_qr.py --data qr_dataset/qr.yaml --epochs 120
# compile_hef.md: best.onnx -> qr_yolov8n.hef (Dataflow Compiler, x86)
scp qr_yolov8n.hef pi@<pi>:~/mission_pi/models/
python ../tools/qr_bench.py --detector hailo --tiles <frames>   # on Pi
```

Re-train from field miss-frames whenever the bench/field shows drops
(details in `training/README.md`). A later custom model drops into the
`detector.kind: custom` slot — contract in `models/README.md`.

### Box order (only if boxes ever look mirrored)

A Hailo NMS box is five values — `y_min, x_min, y_max, x_max, score` — in
HailoRT's own buffer layout (tappas `structures.hpp`, `hailo_bbox_t`), and the
values are **normalised 0..1**, not pixels. The x↔y swap cannot be detected
from the data (both readings are valid rectangles for any well-formed box), so
it is an explicit knob instead of a guess:

```
config.yaml     detector.nms_order: yxyx      # or xyxy
bench tool      --nms-order yxyx|xyxy
```

The tell that a unit needs `xyxy` is a box drawn mirrored across the frame
diagonal (QR low in view, box shown to the left). The runtime logs the order it
used (`hailo ready: … nms_order=yxyx`), and unit tests pin both readings.

### Test QR detection with the REAL camera + Hailo

`test_yolo_webcam_qr.py` runs the mission's own detector + fake-QR filter +
preset table and can drive **the Pi's CSI camera** (OpenCV cannot open a
libcamera device, so `--source 0` only ever finds USB webcams) and the
**Hailo HAT** — same code paths the mission uses, no server needed:

```
hailortcli scan                                        # HAT must be listed
python3 tools/test_yolo_webcam_qr.py --source csi:bottom --hailo \
    --hef models/qr_yolov8n.hef --preset day --require-decode --tiled
python3 tools/test_yolo_webcam_qr.py --source csi:bottom --hailo --no-tile \
    --save-frames /tmp/bench        # 1 inference/frame + annotated JPEGs
python3 tools/test_yolo_webcam_qr.py --source csi --hailo --headless   # SSH/CI
python3 tools/test_yolo_webcam_qr.py --source csi:bottom --onnx \
    --model models/qr_yolov8n.onnx   # CPU comparison, same frames
```

`--source csi[:role|:slot|:index]` = `csi:bottom` (default guess: the
downward camera), `csi:front`, `csi:cam2`, `csi:1`. `--hailo` without the
HAT/HEF prints why and falls back to the CPU ladder instead of crashing.

**Measured throughput (Pi 5, 2026-10-11, no HEF yet)** — the number that
decides the field behaviour: tiled (3×3) ONNX inference over the 2028×1520
frame runs at **~1 fps** (~900 ms/frame); one full-frame inference
(`--no-tile`) is ~9× cheaper but cannot see an A3 QR at 15 m (≈8 px). So on
the CPU ladder the mission can either see the QR or keep up — not both. The
`.hef` is what makes 15 m real-time, and the bench prints `maxconf=` plus a
windowed `ms/frame` so this is visible rather than inferred. `--save-frames
DIR` writes annotated JPEGs (one per second) — the way to *see* a headless
run over SSH.

Running over SSH with no display is detected automatically (Qt would abort the
whole process, not raise). A set `DISPLAY` is not enough to trust: the tool
tries to open a throwaway window **in a child process**, so the classic
`ssh -X`/`.bashrc` `DISPLAY` with no X cookie — which kills Qt with SIGABRT —
degrades to a headless run instead of taking the bench with it. `--show` forces
a window, `--headless` silences the note, `--save-frames DIR` writes annotated
JPEGs, and Ctrl-C prints a summary instead of a traceback.

For a pure model check on stills from that same camera:

```
python3 tools/cam_probe.py --snap-dir snaps
python3 tools/qr_bench.py --detector hailo --hef models/qr_yolov8n.hef --tiles snaps/*.jpg
```

## Geofence: the UI can set it, over the Pi's own FC link

`GET/POST/DELETE /api/fence` uploads a single inclusion polygon with the
ArduPilot **fence mission protocol** (`MISSION_COUNT` →
`MISSION_REQUEST[_INT]` → `MISSION_ITEM[_INT]`, cmd **5001**, `param1` = vertex
count), then **downloads the fence back from the Pixhawk and compares it**
before reporting success. Disarmed only; `409` while armed (re-checked inside
the mission lock), `503` with no FC link, `400` for non-polygon geometry.
`FENCE_ENABLE` / `FENCE_TYPE` / `FENCE_ACTION` are **read and displayed, never
written** — the UI must not quietly arm or disarm a safety boundary.

Mission Planner is not needed to write the fence. The browser refuses to guess
which server may write it: it asks the origin's `/health` for
`service: "mission_pi"` and only then offers `DIRECT Pi → Pixhawk`; otherwise
the buttons are blocked (or the MP relay is used, deliberately, via the
*allow MP bridge fallback* tick).

Every fence state also carries **`fc_capability`** — what the controller says
about itself (`AUTOPILOT_VERSION`): firmware version, MAVLink 2 on/off, and
whether the capability bits implement the fence mission type. The UI shows this
in the `fc firmware` row so a Pixhawk whose firmware predates mission fences is
visible *before* an upload is attempted. `known: false` means the FC did not
answer — that is reported as *unproven*, never as ready, and it never disables
the buttons: the readback comparison stays the thing that decides success.

Details + the executed test matrix + the bench procedure:
[`../docs/GEOFENCE_UI_TO_PIXHAWK.md`](../docs/GEOFENCE_UI_TO_PIXHAWK.md).

```bash
./tools/run_tests.sh fence    # protocol + end-to-end + browser-click tests
```

## Bench tools

* `tools/fc_probe.py` — FC heartbeat/mode/pos/home/batt, `--fence`, `--plan`,
  and **`--fence-check`**: firmware + MAVLink 2 + `FENCE_*` params + stored
  fence, with a `FENCE-READY` / not-ready verdict and a non-zero exit code, so
  a new (or clone) Pixhawk can be judged before the UI is asked to upload
* `tools/cam_probe.py` — list CSI cams, save one snap each (`--config FILE`
  probes with the mission's real size/tuning/focus, `--defaults` ignores it)
* `tools/focus_sweep.py` — sweep the VCM lens through positions (dioptres,
  printed as distances) and score sharpness, then print the exact
  `autofocus: false` + `lens_position:` lines for `config.yaml`. REFUSES to
  sweep when the tuning has no `rpi.af` algorithm (LensPosition is a no-op
  then — proven in the field), and flags a flat curve as "the lens did not
  move / there is nothing to focus on" instead of reporting a winner
* `hailo_decode.py` — the ONE place that turns a HEF's output into boxes:
  styles A / A'' / B / **A'** (`[1, 5*C, N]` NMS rows, features-major,
  normalised) and the `(N,5)` flat form, plus the HailoRT configure calls that
  moved in 4.2x (`hef.create_configure_params(StreamInterface.PCIe)`,
  `InputVStreamParams.make(network_group, …)`), with fallbacks for older
  versions. Pure numpy, no Hailo runtime — so every branch is unit-tested
  without the HAT
* `tools/hef_info.py` — is this `.hef` usable on THIS Pi? Prints the HEF's
  architecture, its input/output tensors and which output style our parser
  will use — A (class id in the last column), A'' (class axis), B (raw head),
  or **A' — the Hailo NMS rows `[1, 5*C, N]`, which is what our own HEF
  returns as `[1, 5, 100]`**: 5 values `y_min, x_min, y_max, x_max, score`
  (normalised) × 100 boxes × 1 class. Anything else means "no boxes ever". It also validates a
  Hailo `nms_config.json` (`--nms-config`) against the model, and answers from
  the SAME code the runtime uses (`hailo_decode.py`), so the verdict cannot
  drift from the Pi's behaviour
* `tools/make_calib_set.py` — build the Hailo DFC calibration set from our
  own frames (camera/video/dir), dropping dark, flat and near-duplicate
  frames and letterboxing to 640×640 exactly like the runtime; prints the
  `hailo optimize` command. See `training/compile_hef.md`
* `tools/qr_bench.py` — detector+decoder over images, boxes/payload/ms
  (`--detector onnx|pt|yolo|hailo|classical|auto`; `onnx` runs the mission's
  own ONNX rung, so the ms you see is the ms the mission gets). The names an
  operator types are aliases of the mission's own kinds — `onnx`, `pt`,
  `yolov8`, `yolo_onnx`… — and an unknown one raises with the full list
* `tools/link_doctor.py` — end-to-end Pi-link diagnosis (local or `--remote`)
* `tools/fake_sitl.py` — SITL stand-in: tcp 5760/5762 + optional udp mirror
  (`--udp`), `--auto --loop` flies a demo mission by itself. Also a **writable
  fence machine**: it accepts/validates/stores mission-type FENCE transfers, so
  the whole UI → Pi → FC fence path can be exercised with no hardware, and
  `--fence-fault reject|discard|corrupt|stall` makes it lie to prove the UI
  reports failure honestly
* `tools/run_tests.sh` — runs every test in `tests/` (unittest modules *and*
  the self-checking scripts, which `pytest tests/` cannot run)
* `tools/make_demo_frames.py` — synthesises `demo_frames/` for `config.demo.yaml`
* `tools/make_qr_panel.py`, `tools/gz_cam_bridge.py` — A3 panel, Gazebo → MJPEG

## Files

```
mission_pi/
  main.py            entry: server FIRST, then bring-up on its own thread
  bringup.py         bring-up ledger + LAN URL/port/system checks for /health
  fc_link.py         USB/TCP auto-detect + pymavlink bus + guided ops
                     + supervised reconnect (link_lost/link_retry/link_restored)
  mission.py         QR-hunt state machine + detection workers (+ reset())
  cameras.py         picamera2/USB/file auto-detect (by sensor) + capture threads
  detector.py        Hailo YOLOv8 / classical / custom-slot + tiling
  decoder.py         crop→upscale→decode (pyzbar + cv2, tiles, CLAHE)
  consensus.py       N-frame payload consensus (stdlib)
  qr_relay.py        STATUSTEXT→MP + ws event→UI
  server.py          FastAPI: /health, /api/bringup, frames, MJPEG, /ws/telemetry
  streamm1.py        MJPEG fan-out; adopts cameras discovered after startup
  store_forward.py   results survive a UI/link outage (flushed when it returns)
  geo.py             ENU / lawnmower / nadir projection (stdlib)
  config.example.yaml  config.laptop.yaml  config.gazebo.yaml  config.demo.yaml
  SIM_GUIDE.md       bench guide (§0.5 zero-hardware demo, §9 troubleshooting)
  training/          dataset gen + fine-tune + HEF compile docs
  models/            weight slots + custom-model contract
  sim/               Gazebo world + iris_dualcam + QR panels
  fence.py           single-inclusion-polygon validation + readback compare
  tools/             fc/cam/qr probes, link_doctor, fake_sitl (writable
                     fence FC + fault injection), run_tests.sh, gz_cam_bridge
  tests/             unit + wire-protocol + end-to-end tests (bring-up,
                     server-first, link supervision, streams, mission reset,
                     geo, consensus, fence protocol, fence e2e on fake_sitl,
                     UI command API + browser DOM clicks, ...)
```

Hehe was used as reference for ideas only (consensus shape, STATUSTEXT
resend cadence, envelope contract) — every module here is written fresh
for this hardware and mission.
