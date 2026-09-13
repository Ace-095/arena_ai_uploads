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
```

Then paste `http://<pi-ip>:8000` into the laptop UI as the Pi link
(receive-only: snapshots + ws events flow Pi → UI; the UI never
commands the Pi — `takeover`/`abort` are local bench endpoints only).

SITL bench test (no hardware): point `--device` at SITL
(`udp:127.0.0.1:14550`) and run with no cameras to exercise
link → snapshot → trigger → GUIDED against the simulator.

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

## Files

```
mission_pi/
  main.py            entry (--check bring-up / flight run)
  fc_link.py         USB auto-detect + pymavlink bus + guided ops
  mission.py         QR-hunt state machine + detection workers
  cameras.py         picamera2 auto-detect (by sensor) + capture threads
  detector.py        Hailo YOLOv8 / classical / custom-slot + tiling
  decoder.py         crop→upscale→decode (pyzbar + cv2, tiles, CLAHE)
  consensus.py       N-frame payload consensus (stdlib)
  qr_relay.py        STATUSTEXT→MP + ws event→UI
  server.py          FastAPI: frame snapshots + /ws/telemetry + status
  geo.py             ENU / lawnmower / nadir projection (stdlib)
  config.example.yaml
  training/          dataset gen + fine-tune + HEF compile docs
  models/            weight slots + custom-model contract
  tools/             fc/cam/qr bench probes
  tests/             stdlib unit tests (geo, consensus)
```

Hehe was used as reference for ideas only (consensus shape, STATUSTEXT
resend cadence, envelope contract) — every module here is written fresh
for this hardware and mission.
