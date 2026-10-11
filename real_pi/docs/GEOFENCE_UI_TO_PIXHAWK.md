# Setting the geofence from the UI: browser → Pi → Pixhawk

**Short answer: yes, it is implemented, and it does not need Mission Planner.**

```text
browser UI  ──HTTP──▶  mission_pi /api/fence  ──MAVLink 2──▶  Pixhawk
    ▲                              │                              │
    └──── confirmed state ─────────┴── download + compare ◀────────┘
```

The polygon is uploaded with the ArduPilot *fence* mission protocol, then the
fence is **downloaded back from the flight controller and compared** before the
UI is allowed to say "done". Mission Planner may be used afterwards to look at
or download the fence — it is not in the write path.

This document is the verification record: what the implementation is, the exact
test matrix that was executed, how to re-run it, and what is still *not* proven.

---

## 1. What the UI does now

| Step | Where | Code |
|---|---|---|
| Draw / paste polygon | browser | `mission-ui/js/map.js` |
| Convert to fence vertices | browser | `map.polygonToFence()` |
| Decide **who may write** | browser | `app.js: fencePiBase() / resolveFenceTarget() / renderFenceWriter()` |
| Upload | `POST <pi>/api/fence` | `mission_pi/server.py` → `FCLink.upload_fence()` |
| Persist | mission type FENCE | `fc_link.py` |
| Verify | download + compare | `FCLink.read_fence()` + `mission_pi/fence.py: matches()` |
| Clear | `DELETE <pi>/api/fence` | `FCLink.clear_fence()` |
| Refresh | `GET <pi>/api/fence` | `FCLink.read_fence()` |

### The fence writer is verified, not assumed

This was the actual defect. The UI is normally served by the **laptop bridge**
(`mission-ui/bridge/mp_bridge.py`), not by the Pi. The old boot code did:

```js
S.piUrl = window.location.origin;      // ← the LAPTOP address
```

so `UPLOAD FENCE → FC` posted to `http://<laptop>:8080/api/fence`, which the
bridge does not implement — it answers `404 not found: POST /api/fence`. From
the operator's seat that is indistinguishable from "the UI cannot set a
geofence".

Now the UI asks the origin what it is, using explicit markers:

| Origin answers `/health` with | UI verdict | Fence writes go to |
|---|---|---|
| `service: "mission_pi"` | **DIRECT Pi → Pixhawk** | `<origin>/api/fence` |
| `service: "mp_bridge"` | **BLOCKED — no Pi link** | nothing (refused) |
| a verified Pi URL typed/`?pi=` | **DIRECT Pi → Pixhawk** | `<that pi>/api/fence` |
| `allow MP bridge fallback` ticked | **BRIDGE → Mission Planner** | `/api/mp/fence` |

The Mission Planner relay is now a **deliberate, labelled opt-in**. There is no
silent fallback: if the Pi link is not verified, the apply button refuses and
says why, so a fence write can never be quietly routed somewhere that cannot
store it.

### What the Pi refuses

* **Armed** → `409`, and the check is repeated *inside* the mission lock, so
  arming mid-transaction cannot slip a fence edit through.
* **No FC link** → `503`, fence untouched.
* **Geometry that is not a polygon** (fewer than 3 vertices, duplicates,
  zero area, self-intersecting, non-finite/out-of-range, wrong types) → `400`
  before anything is sent to the aircraft.
* **A second click while one transaction is in flight** → `409 busy`.
* **ACK without the vertices** → `502`: the readback is compared, so an FC that
  accepts a mission and stores nothing can never be reported as success.

### What the Pi never does

It never writes `FENCE_ENABLE`, `FENCE_TYPE` or `FENCE_ACTION`. Storing a
polygon is not the same as arming enforcement; enabling a safety boundary is an
operator decision. The UI *reads* those parameters and shows them (row
`enforcement`) precisely so "vertices stored" cannot be mistaken for "the
vehicle will stop at the boundary".

---

## 2. Wire format (why Mission Planner is not needed)

Modern ArduPilot carries polygons in the mission protocol, using the **FENCE
extension mission type**:

| Message | Value used |
|---|---|
| `MISSION_COUNT` | `mission_type = MAV_MISSION_TYPE_FENCE (1)`, `count = vertices` |
| FC asks | `MISSION_REQUEST_INT` (or legacy `MISSION_REQUEST`) |
| `MISSION_ITEM_INT` | `command = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001)`, `frame = MAV_FRAME_GLOBAL_INT (5)`, `x/y = degrees × 1e7`, `param1 = TOTAL vertex count` |
| FC answers | `MISSION_ACK` — accepted is *necessary, not sufficient* |
| Verify | `MISSION_REQUEST_LIST` → `MISSION_COUNT` → `MISSION_ITEM_INT` per seq |

`param1` must repeat the **total** vertex count on every item; repeating the
first point to "close" the ring is wrong (`fence.normalize()` strips it).
MAVLink 2 is required, and `fc_link.py` sets `MAVLINK20=1`.

---

## 3. Test matrix (executed)

Every row below was run and passed. "fake FC" is
`mission_pi/tools/fake_sitl.py` — a real MAVLink 2 vehicle that implements the
fence write/read/clear protocol and is deliberately **strict**: it rejects
`command != 5001`, `param1 != count`, wrong frames and bad sequences exactly
like ArduPilot, so a green run means the upload really is a legal fence. It can
also lie on demand (`--fence-fault`) to prove the UI never invents success.

### 3.1 Wire protocol — `tests/test_fence_protocol.py`

| # | Case | Expect |
|---|---|---|
| P1 | Upload 4 vertices, closing point included | 4 items sent, `seq` from FC (out of order + re-request honoured) |
| P2 | Every item's fields | `mission_type=1`, `frame=GLOBAL_INT`, `command=5001`, `param1=4`, `x/y` = degrees×1e7 |
| P3 | Legacy `MISSION_REQUEST` | answered with float `MISSION_ITEM`, `frame=GLOBAL` |
| P4 | Download + compare + clear | matches, then empty |
| P5 | FC rejects (`ACK DENIED`) | `FCError`, no state change |
| P6 | FC silent | `FCError` timeout |
| P7 | `seq` out of range | `FCError` |
| P8 | Reply from the wrong sysid/compid | ignored, timeout |
| P9 | Subscriptions released | no leaked queues after success/failure |
| P10 | Validation (`fence.normalize`) | closed ring trimmed, rotation/reversal accepted by `matches`, 10 malformed shapes rejected |

### 3.2 End-to-end — `tests/test_fence_e2e_fake_fc.py`

Real FastAPI app (`server.create_app`) + real `FCLink` + real MAVLink 2 over a
real TCP socket + the fake Pixhawk. Nothing on the path is stubbed except the
aircraft.

| # | Case | Expect |
|---|---|---|
| E0 | `/health` | `service=mission_pi`, `link=true`, `fc.device` shown (what the UI keys off) |
| E1 | **`POST /api/fence` (happy path)** | `200`, `confirmed`, `source=pi`, `fc_count=4`, **and the vehicle itself holds the vertices**, `fence_writes=1` |
| E2 | Legality of the sent items | strict fake set no error; independent download re-matches geometry |
| E3 | Closed ring + reversed winding | FC receives exactly 4 vertices |
| E4 | Something else changed the fence (e.g. MP) | a refresh returns the FC's fence, not the UI's |
| E5 | `DELETE /api/fence` | `200`, confirmed, **FC is empty**, `clear` counted |
| E6 | Upload twice | replaces (4 vertices), never appends |
| E10 | **FC ACKs then discards** | `502`, `confirmed=false`, FC keeps the old fence, Pi keeps the last FC-verified fence (never the request) |
| E11 | FC stores shifted geometry | `502`, not confirmed |
| E12/E13 | FC denies upload / denies clear | `502` with the ACK reason; fence survives |
| E14 | FC never answers (stall) | `502` quickly — an honest failure, not a spinner |
| E20 | **Armed** | `409` on POST **and** DELETE, FC untouched, zero writes; the same FC still allows `GET` (observation is not an edit) |
| E21 | Pixhawk unplugged | `503`, `confirmed=false`, nothing written |
| E30 | 8 malformed geometries | `400`, and **zero** FC writes |
| E31 | Broken JSON / null coordinates | `400`/`422`, no FC writes |
| E32 | Two clicks at once | never two transfers in flight (fake counts overlapping `MISSION_COUNT`), every `200` is readback-confirmed, final FC fence is exactly the drawn polygon |
| E35 | **Negative control:** raw MAVLink client sends `param1 != count`, `command=16`, wrong frame | FC refuses (`INVALID_PARAM1` / `UNSUPPORTED`); the same client with legal items is accepted — proves the suite is not vacuous |
| E40 | The exact JSON `map.js` builds (`{vertices:[{lat,lon},…]}`) | accepted; reply carries every field `renderFence()` reads |
| E45 | **Real browser buttons → Pi → FC.** `index.html` + `app.js` in jsdom, clicking `UPLOAD FENCE`, `refresh from FC`, `clear on FC` against this Pi | UI shows `MATCH`/`LOADED on FC`; the fake FC afterwards is empty with exactly 1 clear, 0 concurrent transfers |

### 3.3 The UI's own routing — `mission-ui/tests/test_writer_detection_dom.cjs`

| # | Case | Expect |
|---|---|---|
| W1 | Bridge-served page | badge `BLOCKED`, the bridge URL is **not** pasted into the Pi box, clicking apply fires **no** request, and the reason mentions the Pi URL; ticking the opt-in writes once to `/api/mp/fence` and is labelled "not direct" |
| W2 | Pi-served page | badge `DIRECT Pi → Pixhawk`, Pi URL auto-filled, one write to `<origin>/api/fence` |
| W3 | Bridge page + `?pi=http://<pi-ip>:8000` | the verified Pi owns the fence; clear goes to the Pi |

### 3.4 UI click-through against a live Pi — `tests/test_ui_command_api.py`

| # | Case | Expect |
|---|---|---|
| U1 | `test_ui_dom_clicks_to_live_pi` | upload / polygon-apply / refresh / clear all hit `<pi>/api/fence`; a failing FC shows `NOT confirmed` and never `LOADED on FC`; assert proves the writer was proven before use and that MP is only used after the explicit opt-in |
| U2 | `test_ui_picks_the_fence_writer_from_health_not_the_url` | runs 3.3 |
| U3 | `test_fence_roundtrip`, `test_failures_never_claim_confirmed`, `test_invalid_input_is_atomic` | HTTP contract: 200/400/409/502/503 and `mission.fence` only ever holds FC-verified geometry |
| U4 | `test_slow_fence_does_not_block_continuous_stream` | a 1.2 s fence transaction does not stall MJPEG (31 frames, max gap 84 ms) |

### 3.5 Run it

```bash
cd real_pi/mission_pi
./tools/run_tests.sh fence     # the geofence matrix (protocol + e2e + UI)
./tools/run_tests.sh           # every mission_pi test, both test styles
./tools/run_tests.sh --list    # show what would run
```

`run_tests.sh` exists because this repo has two test styles (unittest modules
and self-checking scripts that `sys.exit()` at import), so `pytest tests/` alone
is **not** a valid "everything passed" signal.

Last full run: **23 files passed, 0 failed, 0 skipped** (twice in a row, so the
timing-sensitive cases are not flaky). The fence subset is 4 files / 32 cases,
including 20 end-to-end ones.

Useful pieces on their own:

```bash
python tests/test_fence_e2e_fake_fc.py                      # 20 e2e cases
node mission-ui/tests/test_writer_detection_dom.cjs         # 3 routing cases
python tools/fake_sitl.py --fence 40 --fence-fault reject   # make the fake lie
```

---

## 4. Bench test with the real Pixhawk

The automated tests prove the software. They cannot prove your Pixhawk, your
cable or your parameters — that is what this section is for. **Propellers off.
Vehicle disarmed.**

```bash
# 0. on the Pi
cd ~/arena_ai_uploads && git pull --ff-only
cd real_pi/mission_pi
python3 tools/fc_probe.py --device /dev/ttyACM0        # or /dev/serial/by-id/...
python3 main.py --config config.yaml
```

```bash
# 1. the Pi must identify itself and find the FC
curl -s http://127.0.0.1:8000/health | python3 -m json.tool | head -20
#   expect: "service": "mission_pi", "link": true, "fc": {"device": "/dev/ttyACM0", ...}

# 2. read the CURRENT fence (changes nothing)
curl -s http://127.0.0.1:8000/api/fence | python3 -m json.tool
```

```bash
# 3. upload a small square near where you are (Pixhawk DISARMED)
curl -s -X POST http://127.0.0.1:8000/api/fence \
  -H 'Content-Type: application/json' \
  -d '{"vertices":[[15.3647000,75.1240000],[15.3647000,75.1241000],
                   [15.3648000,75.1241000],[15.3648000,75.1240000]]}' \
  | python3 -m json.tool
#   expect: "confirmed": true, "source": "pi", "fc_count": 4, "reason": "FC readback verified ..."
```

```bash
# 4. verify independently, then clear
curl -s http://127.0.0.1:8000/api/fence | python3 -m json.tool
curl -s -X DELETE http://127.0.0.1:8000/api/fence | python3 -m json.tool
```

Then in the browser open `http://<pi-ip>:8000/?pi=http://<pi-ip>:8000` (or the
bridge UI and paste the Pi URL into **PI LINK**). The GEOFENCE card must show
`writer: DIRECT Pi → Pixhawk` before you press **UPLOAD FENCE → FC**, and
`readback: MATCH ✓` afterwards.

Arm the vehicle once and press **UPLOAD FENCE → FC** again: it must refuse with
`Disarm before editing fence` and change nothing.

Optional — prove the *failure* paths with real hardware by making the
controller-of-record a lying fake instead of the aircraft:

```bash
python3 tools/fake_sitl.py --no-keys --fence 40 --fence-fault discard
python3 main.py --device tcp:127.0.0.1:5762 --no-cams
#   the UI must now report NOT confirmed even though the FC ACKs
```

---

## 4b. The §4 procedure, executed (with the fake as the vehicle)

The commands in §4 were run for real — `main.py`, a real TCP MAVLink link, a
real HTTP server — with `tools/fake_sitl.py` standing in for the Pixhawk. What
came back:

| Step | Result |
|---|---|
| `GET /health` | `"service": "mission_pi"`, `"role": "fence-writer"`, `"link": true`, `"fc": {"device": "tcp:127.0.0.1:5781", "armed": false}` |
| `GET /api/fence` (empty FC) | `confirmed: true`, `vertex_count: 0`, plus read-only `fc_params` (`FENCE_ENABLE: 1`, `FENCE_TYPE: 12`, `FENCE_ACTION: 1`) |
| `POST /api/fence` (4-vertex square) | `200`, `confirmed: true`, `fc_count: 4`, `fc_uploaded: true`, **and the vehicle's own fence now held those 4 vertices** |
| arm the vehicle, then `POST` | `409 "Disarm before editing fence"`, `confirmed: false`, nothing written |
| arm the vehicle, then `DELETE` | `409`, same reason |
| arm the vehicle, then `GET` | `200`, still shows the fence (observation is not an edit) |
| disarm, then `DELETE` | `200`, `cleared: true`, `fc_count: 0` |
| `GET /api/fence` | `confirmed: true`, `vertex_count: 0` — the controller really is empty |
| restart the FC as a **lying** one (`--fence-fault discard`) and `POST` | `502`, `confirmed: false`, `reason: "FC did not store the fence we sent (readback differs) — nothing was confirmed — the FC may now hold a fence that could NOT be verified; re-read the fence before relying on anything"`, and the FC is genuinely still empty |

The Pi's link watchdog also reconnected by itself when the vehicle was killed and
restarted on the same port, without restarting `mission_pi`.

---

## 5. Not proven here (read this before flying)

1. **Enforcement.** These tests prove the polygon is stored on the controller
   and read back identically. They do **not** prove `FENCE_ENABLE=1`,
   `FENCE_TYPE` includes the polygon type, or `FENCE_ACTION` does what you
   expect. Check those in Mission Planner / with `tools/param_probe.py` before
   trusting the boundary. The UI shows them read-only in the `enforcement` row.
2. **The real Pixhawk.** Everything above ran against the fake vehicle. The
   bench procedure in §4 is the remaining step, and only you can run it.
3. **In-flight behaviour.** Fence edits while armed are refused by design, so
   nothing here tests changing a boundary in flight (ArduPilot does not support
   that safely either).
4. **`FENCE_ALT_MAX`.** The UI's altitude control writes `FENCE_ALT_MAX` and
   reads the echo back; the polygon itself carries no altitude.
5. **One polygon.** The Pi/UI support a single inclusion polygon. If the FC
   holds exclusion zones, circles or several polygons, `read_fence()` raises
   instead of silently flattening them into one boundary.
