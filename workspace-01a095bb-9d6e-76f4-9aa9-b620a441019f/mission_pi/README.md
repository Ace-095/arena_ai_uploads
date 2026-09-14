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
| Waveshare IMX477 IR-CUT B (bottom) | other CSI, facing DOWN | 113° diag (~100° HFOV), 2.7 mm f/2.8, FIXED focus — set `bottom.hfov_deg` (calibrate!); day IR-CUT mode |
| Router/LTE | Pi + laptop join the same field LAN | Pi serves `http://<pi-ip>:8000`; laptop pastes that as the Pi link |

Why router/LTE: the Pi streams + pushes events to whatever can reach
its HTTP port. A portable field router gives Pi + laptop one LAN (LTE
uplink on the router is only needed if someone remote must watch too).

## The 15 m detection math (why the pipeline looks like this)

Bottom lens (Waveshare IMX477 B, ~100° HFOV from the 113° diagonal,
4056 px wide): at 15 m the footprint is **~36 m wide → 8.9 mm/px**. An
A3 QR (297 mm side) is **~34 px** in the frame, **~5 px** after a naive
640 resize. 5 px is hopeless — so:

1. **Bottom cam runs YOLO on 3×3 tiles of the frame**: the QR is ~15 px
   on the 640 network input there. Detectable for a fine-tuned 1-class
   YOLOv8n — and the 36 m footprint means a 50×50 m fence needs only a
   few grid rows. (2028×1520 binned @40 fps + 3×3 is the fps/range sweet
   spot; 4056×3040@10 full-res buys decode margin at 15 m.)
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

Bottom lens truth (per `cam.txt` on `main:uploads/`): the Waveshare B is
fixed-focus, f/2.8, 113° diagonal with <1.5% distortion — no PDAF, so
verify 15 m sharpness with `tools/cam_probe.py` snaps before trusting
far decodes, and leave the IR-CUT in day mode (no illuminator onboard).
Set `rotation_deg` so "image up" == nose direction.

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
`[0-9A-Za-z]{1,6}` — keep competition payloads short if you want the
MP-message path to auto-capture there. The ws event always carries the
full text.

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

## Bench tools

* `tools/fc_probe.py` — FC heartbeat/mode/pos/home/batt, `--fence`, `--plan`
* `tools/cam_probe.py` — list CSI cams, save one snap each
* `tools/qr_bench.py` — detector+decoder over images, boxes/payload/ms
* `tools/link_doctor.py` — end-to-end Pi-link diagnosis (local or `--remote`)
* `tools/fake_sitl.py` — SITL stand-in: tcp 5760/5762 + optional udp mirror
  (`--udp`), `--auto --loop` flies a demo mission by itself
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
  tools/             fc/cam/qr probes, link_doctor, fake_sitl, gz_cam_bridge
  tests/             stdlib unit tests (bring-up, server-first, link
                     supervision, streams, mission reset, geo, consensus, ...)
```

Hehe was used as reference for ideas only (consensus shape, STATUSTEXT
resend cadence, envelope contract) — every module here is written fresh
for this hardware and mission.
