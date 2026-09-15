# 03 — Real flight: airframe, flight controller, telemetry, Pi onboard

**Audience:** the whole team, on the day everything is physically connected.
Prereqs: [`01_TWO_LAPTOP_SITL_GAZEBO_SETUP.md`](01_TWO_LAPTOP_SITL_GAZEBO_SETUP.md)
green (the software loop is proven) and
[`02_PI5_AI_HAT_CAMERAS_BENCH.md`](02_PI5_AI_HAT_CAMERAS_BENCH.md) green (the Pi,
the AI HAT and both cameras are proven).

This doc adds only what the bench cannot teach you: **wiring, parameters, radio,
mounting, failsafes and the flight procedure.**

---

## 0. What actually changes from the bench

| | Bench (docs 01–02) | Real flight (this doc) |
|---|---|---|
| Vehicle | SITL / Gazebo | ArduCopter on a Pixhawk-class FC |
| FC link for `mission_pi` | `tcp:<laptop>:5762` | `/dev/serial/by-id/…` (USB) or TELEM2 UART |
| GCS link | MP → SITL `tcp:5760` | MP → **telemetry radio** (COM port @ 57600) |
| Cameras | sim / webcam | Pi Cam 3 (front) + IMX477 (bottom), mounted |
| Detector | classical | Hailo HEF (doc 02 §5.1) |
| Fence | optional | **mandatory** — the mission snapshots it at boot |
| Battery | `min_batt_pct: 0` | real failsafe voltages + `min_batt_pct` |
| Risk | none | props, LiPo, people. Read §11 before arming |

Everything else — the mission FSM, the trigger, the QR relay, the UI, the bridge
— is byte-for-byte the same code you already validated.

---

## 1. Bill of materials

| Group | Items |
|---|---|
| Airframe | Quad frame, 4× motors + ESCs (or 4-in-1), props (+ spare set), landing skids tall enough to protect a downward camera |
| Flight control | Pixhawk-class FC (project reference: **Pixhawk 6C**), GPS + compass module, safety switch, buzzer |
| Power | LiPo (4S/6S per frame), power module / smart battery, **5 V 5 A BEC for the Pi**, XT60 leads, fire-safe charging kit |
| RC | Transmitter + receiver (protocol per `RC_PROTOCOL`), bound and range-checked |
| Onboard computer | Pi 5 + AI HAT+ + both CSI cameras (doc 02), microSD/NVMe, 22↔15-pin CSI adapters |
| Pi ↔ FC link | USB-A ↔ USB-C/micro cable (FC USB port), **or** USB-to-serial (FTDI) for TELEM2 |
| Telemetry | **SiK telemetry radio pair** (915 MHz or 433 MHz per region), ground-side USB |
| Ground | Windows laptop (MP + bridge + UI), field router, printed **A3 QR panel** (target), spare LiPos |
| Tools | Hex drivers, soldering iron, zip ties / hook-and-loop, double-sided foam (vibration), multimeter, laptop for MP |

Firmware: flash the current ArduPilot **stable** for your board — Copter-4.7.1
was released stable on 04-Sep-2026; confirm in MP's *Install Firmware* screen
and write the exact version in your flight log, because parameter names move
between releases (this project already handles the `WP_SPD` ↔ `WPNAV_SPEED`
rename at 4.7 — see §6.3).

---

## 2. Wiring map

```text
                                 ┌──────────────────────────────┐
        GPS + compass ───────────┤ GPS1                         │
        RC receiver ─────────────┤ RCIN                         │
        power module ────────────┤ POWER                        │
                                 │                              │
        ESC 1..4 (signal) ───────┤ MAIN OUT 1..4                │
                                 │                              │
        safety switch + buzzer ──┤ SAFETY / on-board            │
                                 │                              │
        AIR telemetry radio ─────┤ TELEM1  (SERIAL1, 57600)     │
        Pi 5  ──USB or FTDI──────┤ USB / TELEM2 (SERIAL2)       │
                                 └──────────────────────────────┘
                    Pi 5: 5 V 5 A BEC from the main battery (NOT the FC rail)
                          CSI0 ← Pi Cam 3 (front, pitched down)
                          CSI1 ← IMX477 (bottom, nadir)
                          PCIe ← AI HAT+
```

| Connection | Port | Notes |
|---|---|---|
| GPS/compass | `GPS1` | Mount per the arrow; keep it above and away from the power distribution |
| RC receiver | `RCIN` | Bind, then verify every stick + the flight-mode switch in MP's *Radio Calibration* |
| Power module | `POWER` | Supplies the FC and gives voltage/current telemetry; **never** powers the Pi |
| ESCs | `MAIN OUT 1–4` | Motor order per your frame; verify rotation direction and ESC calibration |
| Air radio | `TELEM1` | Default GCS port: MAVLink2 @ 57600 — plug it in and MP usually just works |
| Pi 5 | **USB** (→ `/dev/ttyACM0`) **or** TELEM2 via FTDI | See §6 — pick one and keep the other free for laptop debugging |
| Pi power | dedicated **5 V 5 A BEC** | Pi 5 + AI HAT+ + 2 cameras will brown out on a servo rail |

Wiring discipline that prevents a whole class of field failures:

* Signal and telemetry wires **away from** ESC/motor phase wires; twist or route
  them separately. CSI ribbons short and clamped.
* Radio antennas clear of carbon and of the Pi; air-side antenna pointing down
  and away from the battery.
* Strain-relieve every cable that crosses a moving part; nothing hanging below
  the bottom camera's field of view.
* Common ground everywhere (BEC, FC, radio, Pi share the battery ground).

---

## 3. Flight controller parameters

Set these in MP → *CONFIG/TUNING → Full Parameter List* (or the tree), then
**Write Params** and reboot where indicated. Values are the project's starting
point; anything marked *(airframe)* depends on your hardware.

### 3.1 Serial ports — the two links that matter

| Param | Value | Why |
|---|---|---|
| `SERIAL1_PROTOCOL` | `2` (MAVLink2) | telemetry radio → MP on TELEM1 |
| `SERIAL1_BAUD` | `57` (= 57600) | SiK radios' working rate; MP's baud dropdown must match |
| `SERIAL2_PROTOCOL` | `2` (MAVLink2) | Pi on TELEM2 (skip if the Pi is on USB) |
| `SERIAL2_BAUD` | `92` (= 921600) or `57` | `SERIALn_BAUD` is in units of 100 baud. Start at 57, raise only if you need the bandwidth |
| `SERIAL_PROTOCOL` note | — | The bridge parses MAVLink **1 and 2**, so a legacy v1 mirror still works — but v2 is the default and what the QR `STATUSTEXT` expects |

`SERIALn_BAUD` units trip everyone up once: `57` = 57 600, `115` = 115 200,
`92` = 921 600.

### 3.2 Geofence (mandatory — the mission reads it at boot)

| Param | Value | Why |
|---|---|---|
| `FENCE_ENABLE` | `1` | the Pi refuses to take over without a fence (see §8) |
| `FENCE_TYPE` | `7` (bit0 max-alt + bit1 home circle + bit2 inclusion/exclusion polygons) | bit 2 is the polygon the UI uploads; bits 0/1 are the Copter defaults |
| `FENCE_ACTION` | `1` (RTL) | what a breach does; the UI's *APPLY FENCE* sets this too |
| `FENCE_MARGIN` | `2` m | margin kept inside the boundary; `mission.edge_margin_m: 2.0` matches it |
| `FENCE_ALT_MAX` | *(airframe)* | ceiling above your 15 m search altitude |

The polygon itself is uploaded from the UI (**draw → APPLY FENCE → MP**) or from
MP's Plan tab; the bridge reads it **back** from the FC and compares every
vertex (~1 cm) before reporting `loaded && confirmed`. Eyeball it in MP's
Fence tab anyway, before every flight.

### 3.3 Failsafes — decide these deliberately (§11 explains the GUIDED case)

| Param | Suggested | Meaning |
|---|---|---|
| `FS_THR_ENABLE` | `1` (always RTL) | RC loss. Alternatives: `3` Land, `4` SmartRTL/RTL, `5` SmartRTL/Land |
| `FS_GCS_ENABLE` | `1` (always RTL) | telemetry/heartbeat loss (`FS_GCS_TIMEOUT`, default 5 s) |
| `FS_OPTIONS` | bit 0 (+1) and/or bit 1 (+2) | "continue if in Auto on RC/GCS failsafe". **Note the hunt runs in GUIDED**, so bit 2 ("continue if in Guided on RC failsafe") is the one that would let the hunt survive an RC dropout |
| `BATT_LOW_VOLT` / `BATT_CRIT_VOLT` | *(airframe)* | low / critical cell voltages |
| `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` | `2` (RTL) / `1` (Land) | battery failsafe actions |
| `RTL_ALT` | ≥ 20 m | above anything in the venue |
| `WPNAV_SPEED` / `WP_SPD` | *(see §6.3)* | cruise speed; the Pi **temporarily** overrides it during the search and restores it after |

### 3.4 Everything else

Frame (`FRAME_CLASS`/`FRAME_TYPE`), motor order, ESC calibration, accel +
compass calibration, `RC_PROTOCOL`, `AHRS_EKF_TYPE`, `GPS_TYPE`, `ARMING_CHECKS`
— standard airframe bring-up, outside this project's scope. Do it before the
mission work, outdoors, away from metal and cars, and re-do the compass cal
after the Pi/HAT/radio are mounted in their final positions (they are all
magnetically or electrically interesting).

---

## 4. Telemetry radio and Mission Planner

1. **Pair the radios** (same NETID, matching air/ground). Most SiK pairs ship
   bound; if not, use the AT-command/MP *Setup* route from the radio's docs.
2. Air side → `TELEM1`. Ground side → the Windows laptop's USB.
3. Power the vehicle (or the FC over USB). MP → top-right dropdown → the new
   **COM port** → baud **57600** → **CONNECT**.
4. The HUD comes alive. No movement ⇒ wrong COM port 90 % of the time (Device
   Manager → Ports). *MAVLink Inspector* (Ctrl+F → Mavlink) is the fastest
   "is data flowing?" check.
5. **Range/RSSI sanity:** with the radios ~1 m apart, local and remote RSSI
   should read >190 on a standard SiK radio — much lower means a faulty antenna
   or a radio that was ever powered without one.
6. **Mirror to the bridge** (the one MP-side step that feeds the UI):
   Ctrl+F → **Mavlink** → **UDP Client** → tick **Write access** → Connect →
   `127.0.0.1` → `14551`. Leave the dialog open; changing mirror settings
   usually needs an MP restart.

Windows side from here is exactly doc 02 §8: `py -3 bridge\mp_bridge.py
--port 8100 --mbtiles venue.mbtiles` → `http://127.0.0.1:8100` → MAV lamp green
→ Pi link `http://<pi-ip>:8000` → Probe/Connect. Offline maps for both MP and
the UI: `mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` §6–§7.

---

## 5. Ground-station link budget

```text
Pixhawk ──TELEM1──► air radio ~~~ RF ~~~ ground radio ──USB──► Mission Planner
                                                                   │ MAVLink mirror
                                                                   ▼ udp 127.0.0.1:14551
                                                            mp_bridge.py :8100
                                                                   ▼ SSE / HTTP
                                                              browser UI
Pi 5 ──Wi‑Fi (field router)──► laptop browser UI   (receive-only: telemetry,
                                                  frames, QR events)
```

Two independent paths, by design: **flight truth and the QR `STATUSTEXT` ride
the radio** (MP → bridge), while **camera frames and the Pi's own health ride
Wi‑Fi**. If the Wi‑Fi dies you still get the payload in MP's Messages tab; if
the radio dies you still see the video. Neither path needs the internet — the
LTE leg on the router is informational only and nothing safety-critical reads
it.

---

## 6. The Pi ↔ FC link, for real

### 6.1 Pick a physical link

| Option | Device node | Pros | Cons |
|---|---|---|---|
| **USB cable, Pi → FC USB port** | `/dev/ttyACM0` | no extra hardware, no baud juggling | occupies the port you'd use for laptop reflashing; cable through the frame |
| **FTDI, Pi → TELEM2** | `/dev/ttyUSB0` | keeps USB free, dedicated companion port | one more part; `SERIAL2_*` must be set |

Whichever you choose, use a **stable device path** so a replug can't renumber it:

```bash
ls /dev/serial/by-id/          # e.g. usb-ArduPilot_Pixhawk6C_...-if00
```

`mission_pi`'s auto-detect already prefers `/dev/serial/by-id/*`, then
`/dev/ttyACM*`, then `/dev/ttyUSB*`, and tries 115200 / 57600 / 921600 on each
until a **vehicle** heartbeat answers (GCS heartbeats, e.g. MP's sysid 255, are
filtered out so commands can never be routed at Mission Planner).

### 6.2 Config

```yaml
fc:
  conn: "/dev/serial/by-id/usb-ArduPilot_Pixhawk6C_XXXX-if00"
  retry_s: 3
  connect_timeout_s: 0     # keep retrying; the UI is served either way
  dead_s: 10               # watchdog: stale vehicle heartbeat -> tear down + reconnect
```

CLI overrides: `python3 main.py --device /dev/serial/by-id/... --baud 115200`.

Two behaviours worth knowing:

* **The Pi sets its own stream rates.** Telem-class ports stream nothing until a
  GCS asks, so `mission_pi` requests `HOME_POSITION` 1 Hz,
  `GLOBAL_POSITION_INT` 5 Hz, `BATTERY_STATUS` 1 Hz, `MISSION_CURRENT` 4 Hz,
  `EXTENDED_SYS_STATE` 2 Hz, `VFR_HUD` 2 Hz. It does not depend on MP's rates.
* **It identifies as an onboard computer**, `sysid 51`
  (`MAV_COMP_ID_ONBOARD_COMPUTER`), while MP stays 255. Both links see the
  vehicle's broadcast traffic; the Pi's commands (mode change, goto, param set)
  carry its own identity, so MP's Messages tab shows who did what.

### 6.3 What the Pi writes to the FC (the complete list)

| Action | MAVLink | When |
|---|---|---|
| Take over the hunt | `DO_SET_MODE` → GUIDED (ACK + heartbeat verify) | at the sprayer trigger |
| Slow the search | `PARAM_SET` on `WP_SPD` (m/s, 4.7+) or `WPNAV_SPEED` (cm/s, older) — **learned type, restored after the hunt** | during SWEEP |
| Move to a candidate | guided goto / descend stair | TRACK → APPROACH |
| Report the payload | `STATUSTEXT "QR:<payload>"` every `relay.interval_s` for `relay.window_s`, plus an instant WS event to the UI | TRANSMIT |
| Finish | `post_action` → `RTL` (default) / `AUTO` / `LOITER` / `LAND` | after payload or timeout |

It **reads** home, the fence (`MAV_MISSION_TYPE_FENCE`), the plan (scanning for
`DO_SPRAYER` 216 / 222 / 223 / 42600) and the cruise speed — and it never arms,
never takes off, never uploads a mission. Arming, home and the plan stay in MP.

### 6.4 QR payload format (read this once, save a flight)

The bridge latches `QR_RE = QR\s*:\s*([0-9A-Za-z][0-9A-Za-z._+-]{0,31})` —
**colon required**, first character alphanumeric, then up to 31 more of
`A–Z a–z 0–9 . _ + -`. `MISSION-QR-001` works; "the QR code was missed" must
not latch. The first payload wins and stays in `GET /api/mp/state` → `qr` even
for a UI opened after the fly-past. (Older docs in this repo quote a
`[0-9A-Za-z]{1,6}` limit — that was the pre-v2.1 regex and is no longer true.)

---

## 7. Mounting the Pi and the cameras

| Item | Requirement |
|---|---|
| Bottom IMX477 | True nadir. Landing skids or a belly cut-out must not enter the frame. Set `rotation_deg` so image-up == nose |
| Front Pi Cam 3 | Forward, pitched a little down. Context + bearing cues only |
| Pi 5 + AI HAT+ | Rigid mount, **airflow over the cooler**, away from the ESC stack; the HAT's heatsink needs air |
| Vibration | Double-sided foam / soft mount for the Pi; the FC gets its own damping per its manual. Blurry frames cost you the decode, not just the photo |
| Cables | Strain-relieved, dressed clear of props and of the bottom lens' view |
| Balance | Check CG after everything is mounted — the Pi + HAT is not a light payload |

After mounting, **re-verify the two numbers the geometry depends on**:

```bash
python3 tools/cam_probe.py --snap-dir snaps     # orientation + focus + nothing in frame
python3 tools/qr_bench.py snaps/cam2.jpg        # decode from a real mounted-camera frame
```

and re-measure `bottom.hfov_deg` (§4.4 of doc 02) if the lens or mount changed.
The Waveshare IMX477 B is **fixed focus** — confirm 15 m sharpness with saved
snaps before trusting a far decode, and leave the IR-CUT in day mode.

---

## 8. Pre-arm bench sequence (props OFF, everything connected)

Do this the day before, and again on the field before the first flight. Every
line has an expected result — a surprise here is a no-fly.

```bash
# --- on the Pi (or over SSH) ---
cd ~/mission_pi
python3 main.py --check --config config.yaml
#   [fc] OK: /dev/serial/by-id/... (sysid=1)        <- the REAL FC, not SITL
#   [cam] cam1: kind=rpi model=imx708 facing=front
#   [cam] cam2: kind=rpi model=imx477 facing=bottom
#   [hailo] hailo_platform import OK   [hef] models/qr_yolov8n.hef EXISTS
#   [net] ws impl: websockets

python3 tools/fc_probe.py --device /dev/serial/by-id/... --fence --plan
#   mode/armed/link, position, home, battery, "fence: N verts", plan lines
#   -> the DO_SPRAYER item must appear in the plan list
python3 tools/param_probe.py --device /dev/serial/by-id/... \
    --params FENCE_ENABLE,FENCE_ACTION,FENCE_TYPE,FS_THR_ENABLE,FS_GCS_ENABLE,WP_SPD,WPNAV_SPEED
python3 tools/link_doctor.py                                  # 8 checks, 0 FAIL

# --- on the Windows laptop ---
py -3 ...\mission_pi\tools\link_doctor.py --target http://<pi-ip>:8000   # 8 checks, 0 FAIL
```

Then, still props-off:

1. MP connected over the radio; MP mirror → bridge; UI open; **MAV lamp green**;
   Pi link green; **both camera tiles live**.
2. Draw the venue fence in the UI → **APPLY FENCE → MP** → status
   `loaded && confirmed` → **eyeball the polygon in MP's Fence tab**.
3. Plan: `TAKEOFF` 15 m → `WAYPOINT` over the search area → **`DO_SPRAYER`** →
   `RTL`; **Write WPs**. Read the plan back.
4. Arm (props off), switch to Auto, watch the Pi take GUIDED at the sprayer
   item, then **disarm**. That single run exercises the trigger, the mode
   change and the FSM without leaving the bench.
5. Failsafe dry runs (props off, on the bench): switch the TX off → confirm the
   FC does what `FS_THR_ENABLE` says; kill the Pi process → confirm the UI
   reports `link_lost` and that MP is unaffected; pull the Pi's USB → confirm
   `link_retry` → `link_restored` and `reconnects: 1` in `/health`.
6. Reboot the Pi and confirm `mission_pi` comes back by itself
   (`curl localhost:8000/health`, doc 02 §9).

---

## 9. Flight-day procedure

**Roles:** *pilot* (MP + TX, owns the vehicle), *UI operator* (fence, RTL/LAND
buttons, QR readout), *observer* (eyes on the aircraft and the crowd). The pilot
has final authority; the UI never commands the vehicle except the RTL/LAND
buttons and the fence upload.

| # | Step | Owner | Pass criterion |
|---|---|---|---|
| 1 | Airframe check: props tight, battery charged + secured, wires dressed, cameras clean | observer | nothing loose |
| 2 | Power up: FC boots, Pi boots, `systemctl status mission_pi` | UI op | `/health` → `link: true`, `ready: true` |
| 3 | MP connect over radio; RSSI sane | pilot | HUD alive, GPS 3D, home set **where the aircraft is** |
| 4 | UI: MAV lamp green, Pi link green, both tiles live, tile source = offline pack | UI op | 4/4 green |
| 5 | Fence: uploaded, `loaded && confirmed`, visible in MP | UI op + pilot | agreed polygon |
| 6 | Plan uploaded, `DO_SPRAYER` present, RTL tail present | pilot | read back in MP |
| 7 | Failsafe settings read aloud (`FS_THR_ENABLE`, `FS_GCS_ENABLE`, battery actions) | pilot | matches §3.3 |
| 8 | Arm + Auto | pilot | clean takeoff to 15 m |
| 9 | Watch for `trigger … TAKEOVER … GUIDED` in the Pi log and the mode flip in MP | UI op | mode shows GUIDED |
| 10 | Payload: `QR:<payload>` in MP Messages **and** in the UI | UI op | both routes |
| 11 | Post-action RTL / land, disarm, props off | pilot | aircraft secured |
| 12 | Save records: MP `.tlog`, Pi `logs/`, `curl /health > health.json`, UI log download | UI op | archived before packing |

**Abort ladder** (in order of escalation):

1. UI **RTL** button (or MP RTL) — works while the hunt is in GUIDED.
2. TX flight-mode switch to Loiter/Stabilize — pilot takes manual control.
3. TX throttle cut / kill switch — last resort.

Practice 1 and 2 once, props off, before the first real flight.

---

## 10. What to watch while it flies

| Where | What | Meaning |
|---|---|---|
| MP Messages | `TRIGGER: …`, `QR:<payload>` | the Pi talking through the FC |
| MP mode window | AUTO → **GUIDED** → RTL | the takeover and the post-action |
| UI FSM card | `WAIT_TRIGGER → TAKEOVER → SWEEP_YAW → SWEEP_GRID → TRACK → APPROACH → TRANSMIT → DONE` | where the hunt is |
| UI coverage overlay | 5 m cells shading green/amber | grid progress (needs the Pi link) |
| UI CAM2 tile | the A3 panel appearing + centring | the vision loop closing |
| `/health` → `fc` | `hb_age_s`, `reconnects`, `watchdog: true` | link quality mid-flight |
| UI log | `link_lost` / `link_restored` | Wi‑Fi or USB hiccups (non-fatal) |

If the hunt finds nothing: it runs the yaw sweep, the grid, a **resweep at
`resweep_alt_m`** (10 m), then optional blob hypotheses, all inside one
`search_timeout_s` budget — and then does `post_action`. A timeout is a normal,
logged outcome, not a hang.

---

## 11. Failsafe behaviour during the hunt (read before arming)

The subtle part: **after the trigger the aircraft is in GUIDED, commanded by the
Pi** — not in AUTO. That changes what the failsafes do:

* `FS_OPTIONS` bit 0/1 ("continue if in Auto …") do **not** cover GUIDED. Bit 2
  ("continue if in Guided on RC failsafe") does. Decide explicitly whether an RC
  dropout should abort the hunt (default: yes, RTL) or let it finish.
* `FS_GCS_ENABLE` fires on the *MP* heartbeat (`FS_GCS_TIMEOUT`, default 5 s).
  The Pi's own link dying does **not** trigger a GCS failsafe — the vehicle
  keeps flying the last guided command until the Pi's watchdog reconnects. If
  the Pi link is the one you are worried about, `post_action: RTL` plus a short
  `search_timeout_s` is your safety net, not a GCS failsafe.
* Battery failsafes (`BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT`) fire in any mode;
  `mission.min_batt_pct` is an *additional* software gate that aborts to RTL
  early. On the bench it is 0 — set it for real flight.
* A fence breach does `FENCE_ACTION` (RTL) in any mode. The grid legs already
  stop `edge_margin_m` inside the boundary, so a breach means something else
  went wrong — treat it as such.

---

## 12. Post-flight

```bash
# Pi
curl -s localhost:8000/health | python3 -m json.tool > ~/logs/health_$(date +%s).json
ls -la ~/mission_pi/logs/                 # store-forward buffer, if any route failed
# Windows
#   Documents\Mission Planner\logs\*.tlog  (MP's own log)
#   UI log download button                 (the bridge's console tail)
```

Things worth checking after every flight, not just after a bad one:

* `fc.reconnects` > 0 → the USB/UART link dropped mid-flight (cable, EMI, or a
  watchdog trip). Investigate before the next flight.
* `logs/pending_results.jsonl` non-empty → **both** QR routes failed at some
  point; the store-and-forward buffer flushed on recovery. Check why.
* Decode distance: at what altitude did `TRANSMIT` happen? If it is always the
  bottom of the stair, the 15 m detection margin is thinner than the optics
  math says — re-check focus and `hfov_deg`, and consider
  `mission.resweep_alt_m`.
* Battery used vs planned; RTL altitude vs the venue's obstacles.

---

## 13. Troubleshooting (real-hardware specific)

| Symptom | Cause → fix |
|---|---|
| `[fc] FAIL: no FC heartbeat` on the real FC | Wrong device path (`ls /dev/serial/by-id/`); `SERIAL2_PROTOCOL`/`_BAUD` wrong for a TELEM2 link; Pi and MP fighting over the same port; cable/connector |
| FC found, but `mode: None` forever | You are seeing only MP's heartbeats — the vehicle link is not up. Check the physical port and the FC's own USB/serial config |
| MP connects, UI shows no telemetry | Mirror not flowing (Write access, port 14551, dialog still open) or bridge started after the mirror connected — re-Connect the mirror. `GET /api/mp/mavlink` → `bad_frames` vs `unknown_ids` tells you which |
| `QR:` visible in MP but not the UI | Payload regex (§6.4) or the Pi link is down; the WS route carries the full text regardless — check `link_doctor` from Windows |
| Trigger never fires | The plan must actually be **written** and contain `DO_SPRAYER` (216/222/223/42600); `SNAPSHOT` logs `sprayer seqs: [...]` — empty means the plan scan found nothing. Manual bench trigger: `curl -X POST http://<pi-ip>:8000/api/mission/takeover` |
| Pi refuses to take over | No fence (< 3 vertices) → it stays in FAILSAFE by design. Upload the polygon and re-check with `fc_probe.py --fence` |
| GUIDED refused | Read MP Messages for the reason (pre-arm/EKF/battery); `trigger_require_auto: true` also means it only fires out of AUTO |
| Hunt ends in timeout with no payload | Panel outside the flown area, focus/blur at 15 m, `hfov_deg` wrong (grid spacing scales with it), or the detector needs the fine-tuned HEF |
| Frames fine, decode never confirms | `decode.required_streak: 3` with `miss_tolerance: 1` — one bad frame is fine, three is a real miss. Save the stream frames and run `tools/qr_bench.py` on them |
| Pi brown-outs / reboots in flight | Under-powered BEC. 5 V/5 A minimum, measured under load |
| NPU disappears after a few minutes | Thermal or power — `vcgencmd get_throttled`, cooler airflow, PSU rating |
| Video stalls but telemetry fine | Wi‑Fi congestion at the venue: the Pi serves MJPEG to every viewer; drop `stream.fps`/`width`/`quality`, and remember the mission does not depend on the video |

---

## 14. Sign-off

| Gate | Evidence | Initials |
|---|---|---|
| Docs 01 + 02 checklists complete | both acceptance tables green | |
| Airframe bench sequence (§8) passed | props off, all expected lines | |
| Fence uploaded **and** confirmed **and** eyeballed in MP | UI `loaded && confirmed` + MP screenshot | |
| Failsafe settings reviewed aloud | §3.3 values recorded in the flight log | |
| Abort ladder practised | RTL + mode switch, props off | |
| First flight: payload delivered on **both** routes | MP Messages + UI, archived | |

Nothing in this stack is "trusted" until it has been seen working on the bench,
then in SITL, then in the air — in that order. If any gate above is unchecked,
that is the next task.
