# Laptop setup: Mission Planner + mission-ui + offline maps (field guide)

Everything the competition laptop needs: Mission Planner talking to the
vehicle, the bridge feeding mission-ui, and **both** map systems working
with zero internet at the venue. Do the online steps (marked 🌐) days
before the event — at the field there is no second chance.

```text
Pixhawk <==telemetry radio==> Mission Planner <==UDP 127.0.0.1:14551==> bridge/mp_bridge.py ==> browser UI (SSE)
  (the head: arm/home/plan)            (Write access ON)              (GCS sysid 254, hosts UI on :8100)
Pi 5 (search FSM, cameras, QR) <==WS receive-only==> browser UI   (paste http://<pi-ip>:8000)
```

Roles in one line: **MP flies the drone, the bridge translates, the UI
displays.** Flight truth (position, battery, mode, fence, plan) always
comes from MP — never from the Pi.

---

## 0. Ports, URLs and files (the whole system on one page)

| What | Where | Default |
|---|---|---|
| UI in browser | `http://127.0.0.1:8100` | `--port 8100` |
| MP → bridge MAVLink mirror | UDP `127.0.0.1:14551` | `--mav-port 14551` |
| Bridge GCS identity | sysid 254 (MP itself is 255) | `--mav-sysid 254` |
| Pi link (paste into UI) | `http://<pi-ip>:8000` | set on the Pi |
| Mock FC (practice only) | UDP `127.0.0.1:14560` | `--mock-fc-port 14560` |
| UI offline tiles | `--mbtiles venue.mbtiles` (auto: `./field.mbtiles`) | served at `/tiles/{z}/{x}/{y}.png` |
| MP log auto-watch | `~/Documents/Mission Planner/logs` | + `--watch` / UI watch box |
| MP map cache (Win, modern) | `C:\ProgramData\Mission Planner\gmapcache` | created by Prefetch |
| MP map cache (Win, older) | `C:\Program Files (x86)\Mission Planner\gmapcache\TileDBv3` | same |

---

## 1. Install (🌐 once, at home)

1. **Mission Planner (Windows laptop).** Download `MissionPlanner-latest.msi`
   from <https://ardupilot.org/planner/> and install with defaults. The
   installer also drops the SiK-radio / Pixhawk USB drivers — accept the
   driver prompts. Tikona-style captive networks sometimes corrupt the
   download: if MP crashes on start, re-download.
2. **Python 3.8+** on the same laptop (`python3 --version`). The bridge is
   **pure stdlib** — `argparse, asyncio, sqlite3, socket…`, zero `pip`
   packages. (`pymavlink` is optional: only for `.bin` log parsing; the
   bridge runs fine without it.)
3. **Copy this repo** (or at least the `mission-ui/` folder) to the laptop,
   e.g. `C:\mission\mission-ui\`. No build step — the UI is static files
   served by the bridge, and Leaflet is vendored (`vendor/leaflet.*`, no CDN).
4. **Sanity-check the copy:**
   ```bash
   cd mission-ui
   python3 -m py_compile bridge/*.py tools/*.py
   python3 bridge/mp_bridge.py --selftest        # expect: SELFTEST PASS
   ```

---

## 2. Mission Planner talks to the vehicle (field or bench)

1. Plug in the telemetry radio (USB SiK radio, ground side) **or** USB cable
   to the Pixhawk for bench tests.
2. Open MP → top-right dropdown: pick the COM port + baud —
   **57600** for SiK radios, **115200** for direct USB — → `CONNECT`.
3. The HUD should come alive (artificial horizon, GPS lock, battery). No
   movement? Wrong COM port 90% of the time (check Windows Device Manager →
   Ports). `MAVLink Inspector` (Ctrl+F → *Mavlink Inspector*) shows the raw
   message stream and is the fastest "is data flowing?" check.
4. Bench/SITL practice instead of the real drone: connect MP to the
   simulator over TCP (`127.0.0.1:5760`) exactly as in `RUN_GAZEBO.md` /
   `SIM_GUIDE.md`, then continue below unchanged — the mirror doesn't care
   whether the vehicle is real or simulated.

> The fence the mission flies is the **MP-side polygon fence**
> (inclusion, `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` = 5001,
> `FENCE_ENABLE=1`, `FENCE_ACTION=1` RTL). The bridge uploads exactly this
> when you APPLY FENCE from the UI (§5) — but before flight day, open MP's
> **SETUP → Geofence** tab once and confirm your firmware shows polygon
> fence support (ArduCopter 4.x does).

---

## 3. Mirror MAVLink from MP to the bridge (the one MP-side step)

The bridge *listens* on UDP `:14551`; MP must be told to *send* a copy of
the telemetry there **with Write access**, otherwise the bridge can only
watch — fence upload, RTL/LAND and param reads flow back through this same
mirror.

**Primary path (works on all recent MP versions):**

1. MP connected to the vehicle (§2). Press **Ctrl+F** → click **Mavlink**.
2. Top dropdown: **UDP Client**. Tick **Write access** ✅.
3. `Connect` → host `127.0.0.1` → port **`14551`** → OK.
4. Leave the mirror window open (minimise MP if you like — closing the
   mirror dialog's connection kills the feed).

**If that dialog looks different on your MP version, two equivalent paths:**

- **SETUP → Advanced → Mavlink Mirror**: type *UDP Client*, baud *115200*,
  tick *Write access*, `Connect`. ([ArduPilot SITL docs describe this
  screen.](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html))
- **Flight Data → right-click map → MAVLink Forwarding/Mirroring**: add
  `127.0.0.1:14551`, tick *Write access*.

⚠️ **Quirks worth knowing:**
- Changing mirror settings usually needs an **MP restart** to take effect —
  set it once, then leave it.
- "No connection / target actively refused" = the bridge isn't listening
  (start it first, §4) or the port is wrong. The bridge must be up **before**
  MP connects the mirror, or MP's first packets go nowhere (harmless, but
  the MAV lamp stays red until you re-Connect the mirror).
- Windows Firewall sometimes prompts for MP on first UDP send — Allow it
  (loopback `127.0.0.1` normally needs no rule, but click Allow anyway).

---

## 4. Start the bridge, open the UI

```bash
cd mission-ui
# real thing (MP mirror already connected):
python3 bridge/mp_bridge.py --mbtiles venue.mbtiles
# practice with no vehicle at all (mock Pi + mock FC, all loopback):
python3 bridge/mp_bridge.py --mock --origin 15.3647,75.1240 --mbtiles venue.mbtiles
```

Useful flags: `--port 8100` (UI host port), `--mav-port 14551` (must match
the mirror), `--watch "C:\path\to\extra.log"` (repeatable; MP's own log dir
is auto-watched), `--verbose`.

**Checks (in order):**

1. Bridge console prints the MAVLink listener + tile status + watched logs.
2. Open `http://127.0.0.1:8100` in Chrome/Edge. The **MAV lamp goes green**
   within ~3 s (bridge liveness timeout is 2.5 s of no heartbeat).
3. `http://127.0.0.1:8100/api/mp/state` → `"mavlink": {"connected": true,
   "vehicle_sysid": 1, ...}`. `.../api/mp/mavlink` rebinds the listen port
   live if you ever need it.
4. Telemetry card fills (lat/lon/alt/battery/mode) — this is MP-forwarded
   truth, ≤5 Hz.

---

## 5. Connect the Pi, draw the fence, verify

1. **Pi link:** in the UI's Pi box paste `http://<pi-ip>:8000` → Connect.
   The Pi field router/LTE LAN must be up; the UI only *receives* on this
   link (telemetry envelopes + camera frames) — the sole write path is
   camera tuning sliders. No Pi on the bench? `--mock` (§4) simulates it.
2. **Fence:** pick the offline tile source (§8) → *Draw* the polygon →
   **APPLY FENCE → MP** → confirm the dialog. Success = status
   `loaded && confirmed` (the bridge reads the fence *back* from the FC and
   compares every vertex to ~1 cm). Then **open MP's Fence/Geofence tab and
   eyeball the polygon** — belt and suspenders before every flight.
   MP-uploaded fences are adopted automatically: *refresh fence from FC*
   pulls whatever MP/the FC currently holds into the UI.
3. **RTL/LAND buttons** send `DO_SET_MODE` through the MP link — the UI's
   only flight commands. Arming, takeoff and mission control stay in MP.
4. **QR arrives on 4 independent routes** (first payload wins the display,
   all receipts timestamped): Pi websocket, MAVLink STATUSTEXT
   (`QR:<payload>` in the MP Messages tab — keep competition payloads ≤6
   alphanumerics or the MP-message regex won't auto-latch it), tailed MP
   log file, and manual typing. If the venue kills every link at once, the
   Pi's store-and-forward buffer holds the result and flushes on recovery.
5. **MP log tail:** the bridge auto-watches
   `Documents\Mission Planner\logs`; point `--watch` / the UI watch box at
   any extra file to tail it into the console panel.

---

## 6. Offline maps for MISSION PLANNER (🌐 prep + field use)

MP must show the venue map with no internet. Its cache is **per map
provider**: tiles prefetched under *Google Satellite* do NOT appear when
viewing *OSM* — prefetch every provider you intend to use.

**Prefetch (laptop online, before the event):**

1. MP → **Flight Plan** tab → bottom map-provider dropdown → choose your
   provider (e.g. *Google Satellite* for a photo view of the field).
2. Zoom to the venue (~z17), then **hold ALT and drag a box** over the area
   (venue + generous margin for RTL/drift — 1–2 km² is plenty).
3. **Right-click → Map Tool → Prefetch** → accept the prompts. MP downloads
   from the current zoom through the most detailed — this can take a long
   while; **ESC skips the deeper zooms** if you're in a hurry.
4. Repeat for each provider you'll use (Satellite + Map is the common pair).
5. Cache lands in `C:\ProgramData\Mission Planner\gmapcache` (modern MP) or
   `...\Mission Planner\gmapcache\TileDBv3` (older). **Back it up** to a USB
   stick — same MP version on another laptop can reuse it by copying the
   folder to the same path.

**At the field (offline):** open MP and connect as usual — **MP detects the
missing internet and loads the cache automatically, no setting to flip.**
Verify *before* the first flight: pan/zoom across the whole venue at every
zoom you'll use; grey grid = that provider/zoom wasn't prefetched → switch
provider or accept the gap (do NOT re-prefetch — there's no net).

---

## 7. Offline maps for MISSION-UI (🌐 prep + field use) 🗺️

The UI's map is independent of MP's: the bridge serves a single
`.mbtiles` pack (SQLite) at `/tiles/{z}/{x}/{y}.png`, and the UI picks it
automatically when `/api/mp/tiles-info` reports `available: true`
(order: `offline-mbtiles` → `osm` → `carto-dark`, auto-switching on tile
errors). Leaflet itself is vendored, so with the pack present the map is
100% offline.

### 7.1 Build the pack (any machine with internet)

```bash
cd mission-ui
python3 tools/fetch_tiles.py --bbox "75.118,15.365,75.129,15.374" \
    --zoom-min 15 --zoom-max 19 --out field.mbtiles
```

- `--bbox` is `min_lon,min_lat,max_lon,max_lat`. Get corners from MP's map
  readout, Google Maps (right-click → coordinates), or openstreetmap.org
  (right-click → *Show address*). Cover the fence + margin — the fence is
  ~50 m, so even a 1 km box is 20× margin.
- **Zooms:** `15–19` is the tested default. z15–16 = context when zoomed
  out; z17–19 = flight zooms (the UI boots at z18). OSM serves max z19 —
  asking for z20+ just 404s.
- The script prints per-zoom tile counts **before** downloading, so you see
  the size upfront. Rough guide near Hubballi (z19 tile ≈ 75 m):
  | area | z19 tiles ≈ | whole 15–19 pack ≈ |
  |---|---|---|
  | 500 × 500 m (fence + margin) | ~50 | ~70 tiles, ~2 MB, ~1 min |
  | 1 × 1 km | ~200 | ~270 tiles, ~5 MB, ~1 min |
  | 4 × 3 km (the README example) | ~2,800 | ~3,700 tiles, ~60 MB, ~10 min |
  (OSM PNG ≈ 15–30 KB/tile; `--delay 0.15` = polite ~7 tiles/s.)
- **Resume is free:** re-running the same command skips tiles already in
  the DB (`skipped=N`) and fills gaps — safe to Ctrl+C and continue.
- **Tile-source courtesy:** the default is OSM's public tile server with a
  proper User-Agent + delay, fine for a small venue pack. For huge areas use
  `--tile-url` with a source whose terms you comply with (e.g. a provider
  you have a key for, or your own render) — same `{z}/{x}/{y}` template.
- Verify the file: `python3 -c` not needed — the bridge reports it (§7.3).
  Keep the `.mbtiles` with the field kit (USB stick next to the MP cache
  backup).

### 7.2 Install the pack on the field laptop

Copy `field.mbtiles` next to the bridge and either name it exactly
`./field.mbtiles` (auto-loaded) or pass `--mbtiles <path>`. That's the
whole install — one file, no import step, no service.

### 7.3 Verify (do this ONLINE first, then re-verify OFFLINE)

1. Start the bridge (§4) and open `/api/mp/tiles-info`:
   `{"available": true, "path": ..., "zmin": 15, "zmax": 19,
   "count": 3700, "bounds": "minlon,minlat,maxlon,maxlat"}`.
   `available: false` = wrong path (bridge looks for `./field.mbtiles`
   unless `--mbtiles` says otherwise) or an empty DB.
2. In the UI the log says `map ready (tiles: offline pack)` and the tile
   dropdown shows `offline-mbtiles` selected.
3. **Pull the network cable / enable airplane mode**, restart the browser
   tab, pan + zoom through z15–z19 across the whole venue. Every view must
   render from the pack — `tile errors` in the log + auto-switch to OSM =
   a zoom/bbox gap (fix: rebuild a bigger pack while online, §7.1).
4. Cross-check `bounds` covers the fence polygon with margin; the heatmap /
   grid / fence overlays are vector layers and always work offline.

### 7.4 How it works (for debugging)

- MBTiles stores rows as **TMS** (y=0 at the *bottom*); the bridge flips
  every request (`tms_y = 2^z − 1 − y`). If tiles ever look vertically
  mirrored, that flip is the suspect — but `--selftest` covers it.
- Missing tile = HTTP 404 JSON (`{error}`), which the UI counts toward its
  tile-error streak (8 errors in 10 s → auto-switch source, logged).
- The DB is opened **read-only** (`mode=ro`) — the bridge never modifies
  your pack; safe to share one file across laptops.

---

## 8. Field-day checklist (tape this to the laptop lid)

- [ ] 🌐 done days earlier: venue pack built (§7.1) + MP prefetch done (§6),
      both USB-backed-up.
- [ ] Laptop boots → MP → CONNECT (COM + baud) → HUD alive.
- [ ] MP mirror connected: Ctrl+F → Mavlink → UDP Client → Write access →
      `127.0.0.1:14551`.
- [ ] Bridge up (`--mbtiles venue.mbtiles`), UI open, **MAV lamp green**,
      telemetry live.
- [ ] Airplane-mode test passed earlier; today just confirm the tile source
      reads `offline-mbtiles`.
- [ ] Pi link pasted + connected; camera tiles show frames.
- [ ] Fence drawn → APPLIED (`loaded && confirmed`) → **eyeballed in MP**.
- [ ] Fly. Post-flight: UI log download button + MP `.tlog` in
      `Documents\Mission Planner\logs` are your replay/debug records.

---

## 9. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| MAV lamp red, `connected: false` | Mirror not flowing → MP still connected? mirror dialog still open? bridge started **before** mirror Connect? port 14551 both sides? (`/api/mp/mavlink` POST `{port}` rebinds live.) |
| Lamp flaps green/red | Heartbeats older than 2.5 s → radio link marginal; check `hb_age_s`, `link_flaps` in mavlink-state; move the SiK antenna. |
| Fence apply → `loaded:false` / rejected | Write access off (mirror is read-only) — re-Connect mirror with the box ticked (MP restart may be needed); or wrong mission_type/frame — the UI sends type 1 / GLOBAL / cmd 5001, don't hand-edit. |
| `map ready (tiles: online fallback)` | Pack not found → `--mbtiles` path wrong, or DB empty (`count: 0`); check `/api/mp/tiles-info`. |
| Grey grid at some zooms (UI) | Pack lacks that zoom/bbox → rebuild bigger (§7.1); bounds in tiles-info tell you what you have. |
| Grey grid in MP | That provider/zoom wasn't prefetched (§6) → switch provider; prefetch can't run offline. |
| UI can't reach Pi | Different LAN (Pi hotspot vs venue router)? `http://<pi-ip>:8000/health` in a tab is the raw test; mock mode covers UI-only practice. |
| QR in MP Messages but not UI | Payload >6 chars or lowercase/symbols — the MP-message regex latches `[A-Za-z0-9]{1,6}` only; Pi-WS route carries the full text regardless. |
| Bridge dies on start | Port 8100 busy (old bridge still running) or Python <3.8; `--port` overrides. |
| Mirror settings won't stick | Known MP quirk — set mirror, **restart MP**, re-Connect vehicle + mirror. |

---

*Companion docs: `api-contract.md` (the contract — read first),
`mission-ui/README.md` (quickstart), repo `SIM_GUIDE.md` / `RUN_GAZEBO.md`
(SITL practice), `our-mission.md` §6 (bench→SITL verification rule).*
