# Geofence Pipeline — Milestone 1

Browser draws a polygon → backend on the Raspberry Pi uploads it to the
Pixhawk 6C as an ArduPilot FENCE → backend reads it back to confirm →
system reports itself ARMABLE only once that's confirmed.

```
Browser (Leaflet map)
   │  POST /api/geofence  {vertices: [{lat,lon}, ...]}
   ▼
Pi: FastAPI service (backend/app.py)
   │  1. request EKF origin from Pixhawk
   │  2. upload polygon via MAVLink FENCE mission protocol
   │  3. download it back, compare against what was sent
   │  4. set FENCE_ENABLE=1 / FENCE_ACTION
   │  5. convert to local ENU meters (for Milestone 5's search algorithm)
   ▼
Pixhawk: stores fence, enforces it as a hard boundary (independent of the Pi)
```

## Invariant

**No component in this system may hardcode a venue coordinate, bounding
box, or default fence.** The venue is unknown until 1-2 days before
competition. If no fence has been uploaded and confirmed, the service
reports `armable: false` — there is no fallback area. This is checked by
`GET /api/geofence/armable`, which the search algorithm (Milestone 5) and
your arm sequence should both gate on before doing anything.

## Setup (on the Raspberry Pi)

```bash
cd backend
pip install -r requirements.txt --break-system-packages
export MAVLINK_DEVICE=/dev/serial/by-id/<your-pixhawk-id>   # see NEED TEST below
uvicorn app:app --host 0.0.0.0 --port 8000
```

Open `frontend/index.html` in a browser on the same LAN (or serve it with
any static file server), and set the backend URL in the panel to
`http://<pi-ip>:8000`.

## Testing without hardware

Two options, in order of how close they get you to reality:

**1. Mock responder (included, already run in CI-style form during
development)** — `backend/tests/mock_fc.py` speaks just enough of the
fence + param protocol to validate the client code end-to-end over a real
MAVLink/UDP transport. Useful for catching wire-protocol bugs fast, but it
does **not** replicate ArduPilot's actual fence storage, enforcement, or
firmware-version quirks.

**2. ArduPilot SITL (do this before trusting this on a real flight)** —
run a real ArduPilot Copter simulator and point this service at it:
```bash
# in an ardupilot checkout
sim_vehicle.py -v ArduCopter --console --map
# SITL exposes MAVLink on udp:127.0.0.1:14550 by default
export MAVLINK_DEVICE="udpout:127.0.0.1:14550"
```
This exercises real ArduPilot fence storage/parameters and is the closest
you can get to hardware without a Pixhawk on the bench.

## What's been verified vs. what still needs real hardware

**Verified in this session** (mock responder + full HTTP stack, all passing):
- MAVLink mission-protocol handshake for fence upload (`MISSION_COUNT` →
  `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`, all tagged
  `mission_type=FENCE`)
- Fence readback/download and byte-for-byte-equivalent comparison against
  what was uploaded
- Param set/confirm round trip (`FENCE_ENABLE`, `FENCE_ACTION`)
- Fence clear
- Lat/lon ↔ local ENU conversion, polygon centroid, max-radius, and
  point-in-polygon (unit-tested against known geometry)
- Fail-closed behavior: no fence → `armable: false`; upload failure at any
  step → `armable: false`, never a partial/ambiguous state
- Input validation (rejects <3 or >255 vertices before touching the wire)

**NEED TEST — cannot be verified without your actual Pixhawk 6C + firmware:**
- `MAV_CMD_REQUEST_MESSAGE` support for on-demand `GPS_GLOBAL_ORIGIN`
  requests. If your firmware version doesn't support it, `request_ekf_origin()`
  needs a passive-listen fallback instead (the retry loop is already there;
  only the active request may need to be dropped).
- `FENCE_TYPE` parameter behavior. Some ArduPilot versions auto-infer fence
  type from uploaded item commands; others need `FENCE_TYPE`'s polygon bit
  set explicitly. This code deliberately does not touch `FENCE_TYPE` —
  confirm on your firmware and set it via Mission Planner or extend
  `enable_fence()` once confirmed, rather than guessing the bitmask.
  Fully cite-checked reference: https://ardupilot.org/copter/docs/common-polygon_fence.html
- The actual serial device path for your Pi↔Pixhawk USB connection —
  confirm with `ls /dev/serial/by-id/` on the real Pi.
- Whether Mission Planner auto-refreshes its fence display when the
  Pixhawk's fence changes underneath it, or needs a manual re-read
  (flagged earlier in our design discussion — still open).
- Physical breach behavior: confirm `FENCE_ACTION` actually does what you
  expect (RTL) with a real tethered/careful flight test before trusting it
  operationally. This is Phase 6 territory from your testing roadmap, not
  something a mock can substitute for.

## API

| Method | Path | Purpose |
|---|---|---|
| POST | `/api/geofence` | Upload a polygon `{vertices:[{lat,lon},...]}`, returns full status |
| GET | `/api/geofence` | Current cached status |
| GET | `/api/geofence/armable` | Lightweight `{armable, reason}` — poll this from the arm sequence / search algorithm |
| DELETE | `/api/geofence` | Clear fence on FC and locally |

## Acceptance criteria (from our milestone plan)

- [x] Draw a polygon in the browser
- [x] Upload reaches the Pixhawk and is confirmed by readback
- [ ] Confirm the fence shows as active in Mission Planner (**NEED TEST** — real MP + real FC)
- [ ] Physically fly toward an edge and confirm breach action fires (**NEED TEST** — real flight)

## Explicitly out of scope for Milestone 1

- Telemetry/position WebSocket backbone → Milestone 2 (the frontend polls
  `GET /api/geofence` every 3s for now, which is fine at this data rate but
  should not be the pattern used for live position/attitude later)
- Offline map tile caching, meter-grid overlay → Milestone 3
- The search algorithm actually consuming `centroid_enu` / `max_radius_m` → Milestone 5
