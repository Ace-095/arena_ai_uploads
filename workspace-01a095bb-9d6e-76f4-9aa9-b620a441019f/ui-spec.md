# UI SPEC — Workstream 1: "Mission Companion" (built FIRST)

> Source: user brief 2026-09-12 + research. Mission background: `our-mission.md`. Reuse source: `hehe-analysis.md`.
> **Nothing built yet** — this is the agreed spec, pending the 2 open questions at the bottom.

## 0. THE CORE PRINCIPLE (user's words, stated as law)

**Mission Planner (Windows 11, laptop) is the mission head.** Arming, home point, plan upload,
takeoff — all done in MP. The system "runs on Mission Planner input, not the other way around".
The UI **never arms and never flies the drone by itself**. The UI's job:
1. Draw the geofence on the map (pointer) → make it land in Mission Planner.
2. Show everything live: map + drone position, telemetry stats, Pi system info, camera feed,
   QR decoded message, FSM/mission state, full logs.
3. When QR is decoded: the message goes **1st priority via Pixhawk telemetry → MP message
   console**, and the UI shows it too (over router/LTE).

## 1. Deployment & connectivity (as given by user)

```
   FLIGHT SIDE                              GROUND SIDE
 ┌──────────────┐  USB cable  ┌────────────────────────────┐
 │ Pixhawk 6C   │◄────────────►│ Pi 5 (companion computer)  │
 │ (ArduCopter) │              │ • FastAPI service (UI backend)
 │              │  TELEM ─ radio┤ • MAVLink link owner (USB)
 │              │        (air)  │ • camera (Pi Cam 3)        │
 └──────────────┘              │ • router AND/OR LTE uplink │
                               └─────────────┬──────────────┘
                                             │ router / LTE (any combo, or neither)
                               ┌─────────────▼──────────────┐
                               │ Laptop (Windows 11)        │
                               │ • Mission Planner (MP) ── ground radio (USB COM)
                               │ • Browser → our UI (web page)
                               └────────────────────────────┘
```

- **UI = browser page on the laptop** (same machine as MP). Backend = Pi 5 (reuse hehe's FastAPI stack).
- **Link uplinks are arbitrary**: router only, LTE only, both, or neither. UI must degrade
  gracefully: connection indicator, auto-reconnect, last-known-value display, clear "OFFLINE"
  state on the map/panels. (Pi sits on the field; whichever network it's on, its IP may change →
  UI has a "Pi address" input, remembers last-good, manual re-point.)
- **No second MAVLink link between laptop and Pixhawk exists** (only the radio pair MP uses).
  Every MAVLink action by us goes through the Pi's USB link. (This constrains fence→MP; see §4.)

## 2. End-to-end mission sequence (who does what)

| # | Step | Actor | Detail |
|---|------|-------|--------|
| 1 | Draw fence polygon on UI map (pointer, meter grid, snap) | User → UI | Search area for the Pi = this polygon (user's decision: unknown search area ⇒ geofence IS the search area) |
| 2 | "Apply fence" | UI → Pi → Pixhawk | Pi uploads via MAVLink mission protocol (mission_type=FENCE, reuse hehe code), readback-verify, set FENCE_ENABLE/ACTION. Pi sends **STATUSTEXT "FENCE: n verts, readback OK"** → appears in **MP's message console instantly** (MP is on the same radio). Fence then visible in MP's Fence tab (MP reads fence from FC when the tab is opened/refreshed — MP won't magically refresh mid-session without a second link). Fallback: UI exports a `.lap`/`.wp` file loadable in MP. |
| 3 | Set home point, build & upload plan: **[launch station → climb to 14 m] → [DO_SPRAYER]**, arm, fly (AUTO) | User → MP | MP is head. 14 m climb ≈ search altitude (matches our 15 m detection math). |
| 4 | Trigger detection | Pi (passive, on USB link) | Pi keeps the plan synced (downloads plan items from FC, finds DO_SPRAYER's seq; auto on connect + "Sync plan" button). Monitors **`MISSION_CURRENT`** (streamed ~1 Hz in AUTO; seq = current item) with `MISSION_ITEM_REACHED` as one-shot backup. Fire when seq == DO_SPRAYER seq (or passed it while in AUTO). |
| 5 | Handover | Pi | On trigger: Pi switches Pixhawk to **GUIDED**, begins autonomous search over the fence polygon (search algorithm from hehe: expanding square or lawnmower, every waypoint radius+point-in-polygon checked). If drone isn't at fence center yet, fly there first. |
| 6 | Detect → approach → decode | Pi | Camera (Pi Cam 3, 1080p) → detect A3 QR board → hover over it → closed-loop descent (1.5 m steps, consensus check, ~2 m floor) → QR consensus (3-frame streak) → payload. (hehe qr_pipeline + search_algorithm, single-cam v1 — 2nd cam is "very later".) |
| 7 | Transmit result | Pi, 2 channels | **1st priority: STATUSTEXT `QR:<payload>`** over the Pixhawk link → shows in **MP's message console** (retried 2 s interval / 15 s window, hehe pattern — STATUSTEXT has no ACK). **In parallel: WS event** over router/LTE → UI shows big payload + status. No router/LTE ⇒ MP channel is the guaranteed one; UI sees it next time it reconnects (replay-on-connect, hehe pattern). |
| 8 | After transmit | Pi (auto, **D2**) | Pi does **RTL → LAND → MISSION_COMPLETE** itself. Hands-free. |

## 3. UI layout (simple, dark, scrollable panels — hehe cockpit style, all buttons must work)

```
┌────────────────────────────────────────────────────────────────────────────────┐
│  ┌─ LEFT PANEL (340 px, scrollable) ────────────┐  ┌────────────── MAP ──────┐ │
│  │ ● LINK: Pi ✓/✗  ·  MP-radio: (info)         │  │  Leaflet: OSM online /  │ │
│  │ ─ MISSION STATE ─                            │  │  offline MBTiles toggle │ │
│  │   [STANDBY] big lamp + state + reason        │  │  1 m meter grid overlay │ │
│  │   log tail (last ~30 events, live)           │  │  (1/5/10/20/50 m select,│ │
│  │ ─ FENCE ─                                    │  │   snap-to-grid toggle)  │ │
│  │   verts / area / max-radius / readback OK?   │  │  fence polygon (editable│ │
│  │   [Apply fence] [Clear] [Export .lap]        │  │   pointer: click vert)  │ │
│  │   plan-sync: 3 items · trigger #2 ✓/✗        │  │  drone marker + heading │ │
│  │ ─ TELEMETRY ─                                │  │  target marker (when    │ │
│  │   lat lon alt rel/MSL · mode · armed         │  │   detected)             │ │
│  │   vel · att · GPS fix/sats · battery         │  │                         │ │
│  │ ─ SYSTEM (Pi) ─                              │  │  ┌────────┐ ┌────────┐  │ │
│  │   CPU RAM disk temp freq uptime              │  │  │ camera │ │ camera │  │ │
│  │   net: router ●  LTE ●  (informational)      │  │  │ feed   │ │ feed   │  │ │
│  │ ─ QR / RESULT ─                              │  │  └────────┘ └────────┘  │ │
│  │   streak 2/3 · payload box (BIG on confirm)  │  │  click feed → FULLSIZE  │ │
│  │   sent: STATUSTEXT ✓  WS ✓  (timestamps)     │  │  modal (v1: single cam) │ │
│  │ ─ CONTROLS ─                                 │  └─────────────────────────┘ │
│  │   [ ABORT MISSION ]  (big red; Pi→safe)      │                              │
│  └───────────────────────────────────────────────┘                              │
└────────────────────────────────────────────────────────────────────────────────┘
```

Key UI decisions:
- **No arm/fly buttons.** The only flight-affecting control is the big red **ABORT** (tells the
  Pi to fail-safe: stop search, RTL). Everything else is read-only monitoring + fence authoring.
- **Map grid: 1 m default** (user's requirement for precise fence placement) with 5/10/20/50 m
  options; grid drawn over a ±60 m region around map origin/EKF origin; **snap-to-grid** toggle
  so fence vertices land on meter marks. (Note: 1 m grid is only resolvable at zoom ~19+; UI
  hints "zoom in for 1 m grid" when spacing < 2 px.)
- **Camera feed**: v1 = single Pi Cam 3 (dual-cam layout stays available for later). WebRTC with
  hehe's fallback polling. **Click → enlarge fullscreen modal** (user requirement).
- **QR message display**: streak counter (n/3), on confirm → payload in a big highlighted box +
  transmission receipts (STATUSTEXT sent ✓, WS delivered ✓, timestamps).
- **Logs**: everything goes in — fence ops, plan sync, FSM transitions, QR events, link events,
  system warnings. Live tail in panel + full scrollable log (in-browser, capped ring buffer).
- **Offline/online map**: OSM tiles when internet up; MBTiles offline stack when down (hehe
  `/tiles` + `fetch_tiles.py` pre-cached for the venue once known). Toggle manual + auto-fallback
  on tile errors.
- **Camera settings panel (new, per user + reference attachment `qrfinal_gpt4.txt`)**: expose
  over UI — exposure time (µs), gain/ISO, AF mode (off/continuous/manual), brightness/contrast/
  saturation/sharpness, plus a "motion-adaptive exposure" strategy toggle. Pi side maps these to
  `Picamera2.set_controls()` libcamera controls (`ExposureTime`, `AnalogueGain`, `AfMode`,
  `Brightness`, `Contrast`, `Saturation`, `Sharpness`) — the attachment's approach, re-implemented
  and bench-verified (attachment targets Pi Cam 3 WIDE; we use the standard 4.74 mm lens — do not
  copy blindly).
- **MP-log panel feed**: QR result can arrive via 3 independent routes (Pi WS / MP log bridge /
  manual entry); the QR panel shows which route delivered it.
- Reused as-is from hehe: `telemetry.js` (WS client/reconnect), `tiles.js` (tile manager + ENU
  grid, parameterized spacing), `cameras.js` (WebRTC + fallback), panel/annunciator CSS, dark
  cockpit layout.

## 4. Fence → Mission Planner mechanics (researched constraint)

- MP talks to the Pixhawk **only** via the ground telemetry radio. A web page on the laptop has
  no direct MAVLink path to the FC.
- Therefore: **the Pi (which owns the USB MAVLink link) does the fence upload** — the exact
  protocol code hehe already proved (upload → readback → FENCE_ENABLE → ENU/centroid/radius).
- "Instantly load on Mission Planner" achieved as:
  1. STATUSTEXT `FENCE: n verts, readback OK` lands in **MP's message console immediately**
     (MP is connected to the same radio — guaranteed, no manual step).
  2. Fence is in the FC ⇒ MP's **Fence tab** shows it on next read (user opens/refreshes tab;
     this is MP's own behavior — flagged as the one manual touch; hehe had the same open item).
  3. `.lap` export from the UI as a belt-and-braces file path.
- **True zero-click MP injection is not possible with this wiring** (would need a second
  radio/UDP link laptop↔Pixhawk). Flagged; user to confirm acceptance (Q1).
- After any fence change, the Pi also recomputes the search geometry (centroid, max radius, ENU
  polygon) and the UI fence panel shows them — these drive the search algorithm.

## 4b. MP LOG BRIDGE (new feature, user-requested 2026-09-12)

**Why:** the guaranteed Pi→laptop path with NO router/LTE is Pi → Pixhawk (USB) → radio → MP
console (MP is always connected for the mission). So the UI must also be able to **read MP's logs
locally** and extract the QR decode message from them — result displays even when both router and
LTE are down ("rather than wait for router and LTE which can come when they come").

**Component:** `bridge/mp_bridge.py` — ONE stdlib-only Python asyncio app that runs on the
Windows 11 laptop (zero pip installs). It:
1. **Serves the UI** (static files) at `http://localhost:8100` — so the UI has a local home base
   that works fully offline.
2. **Tails MP log sources** (watched files, new lines streamed as SSE + broadcast to WS clients):
   - MP **session log** text file (user enables MP session logging; path configurable, default
     autodetect under `Documents\Mission Planner\`),
   - any user-specified `.txt/.log` file/dir (the UI has a "watch path" input — operator can point
     it at whatever MP writes on that machine),
   - MP-saved **dataflash `.bin`** files under `Documents\Mission Planner\logs\*\` (parsed with
     pymavlink DFReader if installed; optional — text tail + manual entry work without it),
   - **manual entry** (UI box — the 2-digit payload makes this a 5-second guaranteed fallback).
3. **QR extraction**: pattern `QR:<payload>` (case-insensitive; matches the STATUSTEXT we send,
   `QR:<2 digits>`) in any watched text stream → emits `mp-qr {payload, source, ts, line}`.
4. **Demo mode** (`--mock`): also implements the full **Pi API contract** (WS telemetry, camera
   frames, fence, FSM, QR) with a simulated mission — so the UI is fully demonstrable before the
   Pi exists and before any hardware.

**UI impact:** QR panel shows 3 delivery receipts: `PI-WS ✓/✗`, `MP-LOG ✓/✗ (local, instant)`,
`MANUAL ✓/✗`. MP-log route needs no network uplink at all (same laptop).

## 5. DO_SPRAYER trigger — verified mechanism

- `MAV_CMD_DO_SPRAYER` (310) is a real ArduPilot command (sprayer pump on/off,
  `SERVOx_FUNCTION=22` pump / 23 spinner per ArduPilot docs) — works fine as a zero-side-effect
  "marker" command in the plan even with no pump wired.
- **Detection (pure MAVLink, no extra hardware):**
  - Pi downloads the plan (mission_type=MISSION) → finds DO_SPRAYER's `seq`.
  - Monitors **`MISSION_CURRENT`** (#42, streamed ~1 Hz while mission running; spec says "should
    be streamed all the time") → fire at `seq == sprayer_seq` (or `seq > sprayer_seq` in AUTO,
    in case it was already passed).
  - Backup: **`MISSION_ITEM_REACHED`** (emitted on each waypoint arrival).
  - UI shows "Plan: 3 items · DO_SPRAYER at #2 · synced ✓" so the user sees the trigger is armed.
  - Safety: if no plan is synced and the vehicle is in AUTO past the climb, Pi waits for the
    trigger (fail-closed: no plan known ⇒ no handover ⇒ no blind search).
- Post-trigger: Pi → GUIDED (hehe `command_long MAV_CMD_DO_SET_MODE` → custom mode 4) → search.

## 6. Backend changes vs. hehe (Pi side, workstream-2 flavor but defined now)

| Module | hehe has | we change |
|---|---|---|
| `mavlink_bus.py` | ✅ | reuse unchanged |
| `mavlink_fence.py` | ✅ | reuse + STATUSTEXT on success (hehe has `send_statustext`) |
| `state.py`, `geo_utils.py`, `models.py` | ✅ | reuse (search area = fence polygon — already how hehe works) |
| `telemetry.py` hub | ✅ | add `log` channel; replay-on-connect stays |
| `system_monitor` / `network_monitor` | ✅ | reuse; net = router/LTE status (informational) |
| `camera_source/manager` | ✅ (3-tier) | v1: single `picamera2` cam, no role split; WebRTC stays |
| `qr_pipeline.py` | ✅ | reuse (consensus 3-streak; heuristic detector v1; Hailo later) |
| `search_algorithm.py` | ✅ | reuse strategies + descent; handover entry instead of takeoff |
| `fsm.py` | 14-state | **rewrite entry path**: no ARM/TAKEOFF (MP did it) — new states: `IDLE → PLAN_SYNC → WAITING_TRIGGER → SEARCH → TARGET → DESCEND → DECODED → TRANSMIT → RTL → LAND → COMPLETE` (+FAILSAFE). Trigger = MISSION_CURRENT module. |
| NEW `plan_trigger.py` | — | plan download, DO_SPRAYER seq discovery, MISSION_CURRENT/MISSION_ITEM_REACHED watch |
| NEW fence-notify / .lap export | — | small: STATUSTEXT on fence load; LAP file writer |

## 7. Reuse list from hehe (exact files)

`backend/mavlink_bus.py`, `backend/mavlink_fence.py` (upload/download/enable/readback/statustext),
`backend/geo_utils.py`, `backend/state.py` pattern, `backend/telemetry.py`,
`backend/system_monitor.py`, `backend/network_monitor.py`, `backend/camera_source.py`
(Picamera2Source + factory), `backend/camera_manager.py` (WebRTC, single-cam),
`backend/qr_pipeline.py` (detector ABC + consensus + runner), `backend/search_algorithm.py`
(strategies + controller + closed-loop descent), `frontend/js/telemetry.js`, `frontend/js/tiles.js`
(parameterize grid), `frontend/js/cameras.js`, `tools/fetch_tiles.py`, `backend/tests/mock_fc.py`
(test rig), cockpit CSS from `frontend/index.html`.

## 8. Constraints & invariants (carry over from hehe + user)

1. No hardcoded venue coordinates anywhere; search area always = runtime fence polygon.
2. Fail-closed: no fence ⇒ no search; no plan trigger ⇒ no handover; camera dead ⇒ no decode,
   abort to safe state; Pi never arms (MP owns arming).
3. Pixhawk's own fence enforcement is the safety boundary (Pi-side checks are convenience).
4. Internet/router/LTE status is **informational only** — never gates the mission (radio link is
   the mission path; network is the UI path).
5. QR result: STATUSTEXT first (radio → MP), WS second (router/LTE → UI); both logged with receipts.
6. Simple UI: every button functional, scrollable panels, no dead features, camera click-to-enlarge.

## 9. DECISIONS (user-confirmed 2026-09-12)

- **D1 — Fence→MP path = Pi upload (confirmed).** UI draws → Pi uploads to Pixhawk (hehe code) →
  MP message console shows `FENCE: n verts, readback OK` instantly (same radio) → fence visible in
  MP's Fence tab on next read → `.lap` export as fallback. No second link / no extra hardware.
- **D2 — After decode + transmit = full auto (confirmed).** Pi does **RTL → LAND** itself
  (hehe behavior). Hands-free end-to-end; user only manages up to the DO_SPRAYER trigger.

## 10. v2 rebuild (2026-09-13) — MP-forwarded MAVLink

The laptop has no radio of its own, so v1's "Pi owns MAVLink over USB" path is
replaced: **Mission Planner owns the telemetry link and forwards MAVLink over
UDP (Write access) to the bridge**, which acts as a GCS (sysid 254) and serves
the UI over SSE.

- **D1 SUPERSEDED.** Fence path is now UI → bridge → MP → FC over MAVLink
  (mission_type 1, cmd 5001, readback verify, `FENCE_ENABLE/ACTION/TYPE`), not
  UI → Pi → Pixhawk. The `.lap` export stays as manual fallback.
- **D2 unchanged** (Pi-side auto RTL→LAND after decode+transmit is workstream 2
  scope); the UI keeps its own ABORT→RTL button, now sent as DO_SET_MODE via
  the MP link. Arming/takeoff/mission control stay in MP, always.
- Backend recovered from upstream patch onto `arena/01a09bbf-arena-ai-uploads`:
  stdlib MAVLink v2 codec (`bridge/_mavlink_v2.py` + `_mavtable_gen.py`),
  GCS client (`bridge/mav_client.py`), mock FC (`bridge/mock_fc.py`),
  rewritten bridge (`bridge/mp_bridge.py`), codec test + table generator.
  Verified: 23/23 byte-identical + SELFTEST PASS.
- Pi link is WS **receive-only** (fsm/system/qr/events); the UI's sole write
  path to the Pi is camera tuning (no MAVLink path exists for libcamera).
- Map: offline MBTiles default (bridge `/tiles`, XYZ→TMS flip) → OSM →
  Carto-dark; **Leaflet vendored** (`vendor/leaflet/`) — zero network needed.
- Meter grid fixed: frozen anchor (boot centre; fence origin wins; re-anchor
  button), canvas renderer, viewport culling (cap ~280 lines), auto-coarsen
  1→2→5→10→20→50→100 m, click-hint zoom-to-grid, live draw-area readout.
- Dual cameras CAM1 (Pi Cam 3) + CAM2 (IMX477): WebRTC→polling fallback each,
  shared enlarge modal, per-cam **live** controls (debounced, no Apply).
- QR: 4 routes (pi-ws, mavlink STATUSTEXT, mp-log file, manual) with receipt
  timestamps; new MP-console panel shows FC STATUSTEXT live.
- Fence counts as applied only with `loaded && confirmed` (FC readback);
  armable = MAVLink link + confirmed fence.
- Docs: `mission-ui/docs/api-contract.md` v2, `mission-ui/README.md` v2
  runbook. Bench list lives in the README open items.
