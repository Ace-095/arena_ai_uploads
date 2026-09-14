# Drone QR-hunt — mission_pi + mission-ui

Companion stack for a QR-code hunt mission: an ArduPilot vehicle flies a search
pattern, a downward camera finds and decodes a QR panel, and the payload is
delivered to the ground on **two** independent routes (MAVLink `STATUSTEXT` via
Mission Planner, and WebSocket/HTTP to a browser UI).

```
        LINUX LAPTOP (or Pi 5)                        WINDOWS 11
 ┌──────────────────────────────────┐      ┌────────────────────────────────┐
 │ ArduPilot SITL (+ Gazebo)        │      │ Mission Planner  ← the flight  │
 │   tcp 5760 ─────────────────────────────►  head: arm, takeoff, mission   │
 │   tcp 5762 ──► mission_pi :8000  │      │   MAVLink forwarding           │
 │                 │  REST + MJPEG  │      │   udp 127.0.0.1:14551          │
 │                 │  + /ws/telemetry      │        │                       │
 │                 └──────────────────────────► browser UI ◄── mp_bridge   │
 └──────────────────────────────────┘      │        http://localhost:8100   │
                                           └────────────────────────────────┘
```

Mission Planner stays the head for flight control. The Pi link is
**receive-only** from the UI's point of view — the UI never commands the
vehicle.

---

## ⚠️ Which branch has the code? (read this first)

`main` on this repository is **stale**: it contains only `hehe/` (an older
reference implementation) and an early `mission-ui/` snapshot. **`main` has no
`mission_pi/` at all**, so a fresh clone of `main` gives you nothing that
listens on `http://<ip>:8000` — which is exactly why the UI on Windows showed
"Pi link: down".

All current work lives on:

```
arena/01a0a01d-arena-ai-uploads
```

```bash
git clone https://github.com/Ace-095/arena_ai_uploads.git
cd arena_ai_uploads
git checkout arena/01a0a01d-arena-ai-uploads        # ← the branch with the code
ls workspace-*/mission_pi/main.py                   # must exist
```

A pull request merging this branch into `main` is the intended fix; until it is
merged, **check the branch out on both machines** (the Linux laptop and the
Windows box). To confirm you are on the right one:

```bash
git rev-parse --abbrev-ref HEAD     # -> arena/01a0a01d-arena-ai-uploads
git log --oneline -1
```

---

## Layout

Everything lives under `workspace-01a095bb-9d6e-76f4-9aa9-b620a441019f/`:

| Path | What it is | Runs on |
|---|---|---|
| `mission_pi/` | The Pi/laptop companion: MAVLink link, cameras, QR detector, mission FSM, FastAPI server on `:8000` | Linux laptop / Pi 5 |
| `mission-ui/` | Browser ground UI + `bridge/mp_bridge.py` (serves the UI on `:8100`, bridges Mission Planner's MAVLink mirror) | Windows 11 |
| `hehe/` | Older reference implementation — ideas only, not part of the run | — |
| `our-mission.md`, `ui-spec.md`, `hehe-analysis.md` | Requirements and design notes | — |

Key docs:

* `mission_pi/SIM_GUIDE.md` — the full bench guide (SITL, webcam, Gazebo,
  troubleshooting ladder, demo mode).
* `mission_pi/RUN_GAZEBO.md` — Gazebo Harmonic + ArduPilot plugin specifics.
* `mission-ui/README.md` — UI/bridge quickstart, offline map pack.
* `mission-ui/docs/api-contract.md` — every channel, endpoint and invariant
  between the Pi, the bridge and the UI.

---

## Fastest possible check: zero hardware, one machine

No SITL, no camera, no Mission Planner. A fake vehicle + a file-backed camera +
the bridge, all on loopback — proves the whole chain (FC link → detection →
`STATUSTEXT` → bridge → UI) end to end:

```bash
cd workspace-*/mission_pi
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt          # NOTE: includes `websockets` — see below
python3 tools/make_demo_frames.py        # synthesises demo_frames/*.png (gitignored)

# terminal 1 — fake vehicle (SITL stand-in: tcp 5760/5762 + udp 14551 mirror)
python3 tools/fake_sitl.py --udp 127.0.0.1:14551 --auto --loop --no-keys

# terminal 2 — the Pi link
python3 main.py --config config.demo.yaml

# terminal 3 — the UI + bridge (Windows: py -3 bridge\mp_bridge.py --port 8100)
cd ../mission-ui && python3 bridge/mp_bridge.py --port 8100
```

Open `http://localhost:8100`. The demo mission cycles automatically; when the
payload is found you should see it appear in the UI (QR panel) and in
`http://localhost:8000/health`.

---

## Real two-machine setup (your topology)

**Linux laptop** — SITL + Gazebo + `mission_pi`:

```bash
cd workspace-*/mission_pi
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map   # or see SIM_GUIDE §6
python3 main.py --config config.gazebo.yaml                   # webcam: config.laptop.yaml
```

`main.py` prints a banner with the exact URLs to paste into the UI:

```
Pi link (paste ONE of these into the UI's Pi-link box on Windows):
    http://192.168.1.42:8000
  from Windows:  curl http://<that-ip>:8000/health
  if that hangs:  sudo ufw allow 8000/tcp   (and 5760/5762 for SITL)
```

**Windows 11** — Mission Planner + the UI:

1. Connect MP to SITL (`tcp:<linux-ip>:5760`), arm and fly from MP.
2. Right-click the vehicle → **MAVLink Forwarding** → `127.0.0.1:14551`,
   *Write access ON*.
3. `py -3 bridge\mp_bridge.py --port 8100` → open `http://localhost:8100`.
4. In the UI's **Pi link** box enter `http://<linux-ip>:8000` and press
   **Probe** — it reports exactly which check failed instead of a red lamp.

---

## If the UI cannot reach the Pi link

In order of how often it bites:

1. **`websockets` is not installed.** uvicorn then serves HTTP fine but
   **silently 404s every WebSocket route** — the UI's telemetry socket dies
   while `/health` looks healthy. `pip install websockets` (or
   `uvicorn[standard]`). `mission_pi/requirements.txt` pins it; `main.py`
   warns at startup if it is missing, and the UI falls back to polling
   `/api/fsm/status` + `/api/qr/status`.
2. **Nothing is listening on `:8000`.** Older code raised out of `main.py`
   during FC connect *before* the HTTP server bound, so a missing flight
   controller meant a dead port. The server now comes up **first** and reports
   the FC failure at `GET /health` and `GET /api/bringup`.
3. **Firewall / different subnet.** `sudo ufw allow 8000/tcp`; then from
   Windows `curl http://<linux-ip>:8000/health`. Chrome's Local Network Access
   prompt must be allowed.
4. **Wrong branch** — see above.

Diagnose it properly instead of guessing:

```bash
python3 tools/link_doctor.py                 # on the Pi/laptop: self-check
python3 tools/link_doctor.py --target http://<linux-ip>:8000   # from Windows
```

`link_doctor` checks the TCP connect, `/health`, `/api/cameras`, a real
WebSocket handshake (101 + `Sec-WebSocket-Accept`) and how many telemetry
envelopes actually arrive.

The FC link is supervised: if the vehicle heartbeat stops (SITL restarted, USB
replugged, laptop slept) `mission_pi` reports `link_lost`, re-acquires the
device and reports `link_restored`, with the count in `/health`:

```json
{"link": true, "ready": true,
 "fc": {"device": "tcp:127.0.0.1:5762", "connected": true, "down": false,
        "reconnects": 1, "hb_age_s": 0.4, "watchdog": true}}
```

---

## Tests

Stdlib-only, no hardware, a few seconds each:

```bash
cd workspace-*/mission_pi && for t in tests/*.py; do python3 "$t"; done
cd workspace-*/mission-ui && for t in tests/*.py; do python3 "$t"; done
cd workspace-*/mission-ui && python3 bridge/mp_bridge.py --selftest
```

| Test | Locks down |
|---|---|
| `test_server_first.py` | HTTP answers *before* the FC is found; `/api/bringup`; real WS handshake |
| `test_bringup.py` | The bring-up ledger, LAN URL discovery, port/system checks |
| `test_link_supervision.py` | FC watchdog: loss → retries → recovery, no EOF spin |
| `test_stream_sync.py` | MJPEG streams adopt cameras discovered after startup |
| `test_mission_reset.py` | Re-running a hunt cannot reuse the previous payload |
| `test_qrlatch.py` | QR regex (real payloads in, prose out) + first-payload latch |
| `test_mavv1.py` | The bridge parses MAVLink **1** as well as MAVLink 2 |
| `test_mavcodec.py` | The frozen MAVLink codec is byte-identical to pymavlink |
